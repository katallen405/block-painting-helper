#!/usr/bin/env python3
"""
SMACH state machine for block-painting-helper arm demo
kat.allen@tufts.edu

States:
    Wait                    -> RetrievingObject        (on /button String: payload is color to fetch)
    RetrievingObject        -> WaitingForObject         (GoToLocation service accepted)
    WaitingForObject        -> NavigatingHome           (on /button message)
    NavigatingHome          -> LocatingObjectAndPeople  (GoToLocation service accepted)
    LocatingObjectAndPeople -> PickAndPlace             (always; pose may be None if perception failed)
    PickAndPlace            -> Grasping                 (arm at grasp position)
    Grasping                -> MoveToWorkspace          (on /button: manual stand-in for gripper close)
    MoveToWorkspace         -> SpringController         (arm in workspace)
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

import smach
import smach_ros
from controller_manager_msgs.srv import SwitchController
from bph_interfaces.srv import GoToLocation, GetTargetPose, MoveToPose
from bph_pickmeup.bph_pickmeup_client import BphPickmeupClient


# ---------------------------------------------------------------------------
# _Latch — thread-safe single-message capture
# ---------------------------------------------------------------------------
class _Latch:
    def __init__(self):
        self._event = threading.Event()
        self.value = None

    def reset(self):
        self._event.clear()
        self.value = None

    def callback(self, msg):
        self.value = msg.data if hasattr(msg, "data") else msg
        self._event.set()

    def wait(self, timeout=None):
        self._event.wait(timeout=timeout)
        return self.value


# ---------------------------------------------------------------------------
# RobotFetchNode — owns all ROS2 infrastructure
# ---------------------------------------------------------------------------
class RobotFetchNode(Node):
    def __init__(self):
        super().__init__("robot_fetch_smach")

        self.declare_parameter("home_x", 0.365)
        self.declare_parameter("home_y", -0.195)
        self.declare_parameter("home_yaw", 0.0)
        self.declare_parameter("supply_closet_x", 2.6)
        self.declare_parameter("supply_closet_y", -0.195)
        self.declare_parameter("supply_closet_yaw", 0.0)

        self.requested_object_color = "red"

        self.switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.get_logger().info("RobotFetchNode initialised")

    @property
    def home_location(self):
        return (
            self.get_parameter("home_x").value,
            self.get_parameter("home_y").value,
            self.get_parameter("home_yaw").value,
        )

    @property
    def supply_closet_location(self):
        return (
            self.get_parameter("supply_closet_x").value,
            self.get_parameter("supply_closet_y").value,
            self.get_parameter("supply_closet_yaw").value,
        )

    def switch_controllers(self, activate: list, deactivate: list):
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = 2
        event = threading.Event()
        result = [None]

        def cb(f):
            result[0] = f.result()
            event.set()

        self.switch_client.call_async(req).add_done_callback(cb)
        event.wait()
        return result[0]

    def build_and_run_sm(self):
        pickmeup = BphPickmeupClient(self)

        sm = smach.StateMachine(outcomes=["task_complete"])
        sm.userdata.target_pose = None

        with sm:
            smach.StateMachine.add(
                "WAIT", Wait(self),
                transitions={"button_pressed": "RETRIEVING_OBJECT"},
            )
            smach.StateMachine.add(
                "RETRIEVING_OBJECT", RetrievingObject(self),
                transitions={"at_supply_closet": "WAITING_FOR_OBJECT", "nav_error": "WAIT"},
            )
            smach.StateMachine.add(
                "WAITING_FOR_OBJECT", WaitingForObject(self),
                transitions={"object_loaded": "NAVIGATING_HOME"},
            )
            smach.StateMachine.add(
                "NAVIGATING_HOME", NavigatingHome(self),
                transitions={"at_home": "LOCATING_OBJECT_AND_PEOPLE", "nav_error": "WAIT"},
            )
            smach.StateMachine.add(
                "LOCATING_OBJECT_AND_PEOPLE", LocatingObjectAndPeople(self),
                transitions={"proceed": "PICK_AND_PLACE"},
            )
            smach.StateMachine.add(
                "PICK_AND_PLACE", PickAndPlace(self, pickmeup),
                transitions={"arm_at_grasp_position": "GRASPING", "failed": "WAIT"},
            )
            smach.StateMachine.add(
                "GRASPING", Grasping(self),
                transitions={"grasp_confirmed": "MOVE_TO_WORKSPACE"},
            )
            smach.StateMachine.add(
                "MOVE_TO_WORKSPACE", MoveToWorkspace(self, pickmeup),
                transitions={"in_workspace": "SPRING_CONTROLLER", "failed": "WAIT"},
            )
            smach.StateMachine.add(
                "SPRING_CONTROLLER", SpringController(self),
                transitions={"done": "WAIT"},
            )

        sis = smach_ros.IntrospectionServer("robot_fetch_smach", sm, "/SM_ROOT")
        sis.start()
        outcome = sm.execute()
        self.get_logger().info("State machine finished: %s" % outcome)
        sis.stop()


# ---------------------------------------------------------------------------
# _FetchState — base class with shared helpers
# ---------------------------------------------------------------------------
class _FetchState(smach.State):
    EFFORT_CONTROLLER   = "forward_effort_controller"
    POSITION_CONTROLLER = "scaled_joint_trajectory_controller"

    def __init__(self, node: RobotFetchNode, outcomes, input_keys=None, output_keys=None):
        smach.State.__init__(
            self,
            outcomes=outcomes,
            input_keys=input_keys or [],
            output_keys=output_keys or [],
        )
        self._node = node

    def _call_service(self, client, request):
        """Call a service safely while MultithreadedExecutor is running."""
        event = threading.Event()
        result = [None]

        def cb(f):
            result[0] = f.result()
            event.set()

        client.call_async(request).add_done_callback(cb)
        event.wait()
        return result[0]

    def _wait_for_button(self):
        """Block until a String message arrives on /button; return its payload."""
        latch = _Latch()
        sub = self._node.create_subscription(String, "/button", latch.callback, 10)
        value = latch.wait()
        self._node.destroy_subscription(sub)
        return value


# ---------------------------------------------------------------------------
# State: Wait
# ---------------------------------------------------------------------------
class Wait(_FetchState):
    """Idle. The /button message payload is the color to fetch (e.g. 'red')."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["button_pressed"])

    def execute(self, userdata):
        self._node.get_logger().info("[Wait] Waiting for /button press (payload = color)...")
        color = self._wait_for_button()
        if color:
            self._node.requested_object_color = color
            self._node.get_logger().info("[Wait] Requested color: %s" % color)
        return "button_pressed"


# ---------------------------------------------------------------------------
# State: RetrievingObject
# ---------------------------------------------------------------------------
class RetrievingObject(_FetchState):
    """Navigate to the supply closet."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["at_supply_closet", "nav_error"])
        self._nav_client = node.create_client(GoToLocation, "navigate_to")

    def execute(self, userdata):
        self._node.get_logger().info("[RetrievingObject] Navigating to supply closet...")
        req = GoToLocation.Request()
        req.x, req.y, req.yaw = self._node.supply_closet_location
        resp = self._call_service(self._nav_client, req)
        if resp and resp.accepted:
            self._node.get_logger().info("[RetrievingObject] Arrived at supply closet")
            return "at_supply_closet"
        self._node.get_logger().warn(
            "[RetrievingObject] Navigation failed: %s"
            % (resp.message if resp else "no response")
        )
        return "nav_error"


# ---------------------------------------------------------------------------
# State: WaitingForObject
# ---------------------------------------------------------------------------
class WaitingForObject(_FetchState):
    """Wait at the supply closet for a human to load the object (button confirm)."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["object_loaded"])

    def execute(self, userdata):
        self._node.get_logger().info(
            "[WaitingForObject] Waiting for /button to confirm object loaded..."
        )
        self._wait_for_button()
        return "object_loaded"


# ---------------------------------------------------------------------------
# State: NavigatingHome
# ---------------------------------------------------------------------------
class NavigatingHome(_FetchState):
    """Return to home base."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["at_home", "nav_error"])
        self._nav_client = node.create_client(GoToLocation, "navigate_to")

    def execute(self, userdata):
        self._node.get_logger().info("[NavigatingHome] Navigating home...")
        req = GoToLocation.Request()
        req.x, req.y, req.yaw = self._node.home_location
        resp = self._call_service(self._nav_client, req)
        if resp and resp.accepted:
            self._node.get_logger().info("[NavigatingHome] Arrived at home")
            return "at_home"
        self._node.get_logger().warn(
            "[NavigatingHome] Navigation failed: %s"
            % (resp.message if resp else "no response")
        )
        return "nav_error"


# ---------------------------------------------------------------------------
# State: LocatingObjectAndPeople
# ---------------------------------------------------------------------------
class LocatingObjectAndPeople(_FetchState):
    """
    Call color_picker for the target pose.
    Passes the pose (or None on failure) to PickAndPlace via SMACH userdata.
    Always proceeds — PickAndPlace handles the None case with a named-position fallback.
    """

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["proceed"], output_keys=["target_pose"])
        self._perception_client = node.create_client(
            GetTargetPose, "/color_picker/get_target_pose"
        )

    def execute(self, userdata):
        self._node.get_logger().info(
            "[LocatingObjectAndPeople] Requesting pose for color '%s'..."
            % self._node.requested_object_color
        )
        req = GetTargetPose.Request()
        req.color = self._node.requested_object_color
        resp = self._call_service(self._perception_client, req)
        if resp and resp.success:
            self._node.get_logger().info(
                "[LocatingObjectAndPeople] Pose found: %s" % resp.message
            )
            userdata.target_pose = resp.pose
        else:
            self._node.get_logger().warn(
                "[LocatingObjectAndPeople] Perception failed (%s) — will use pre_grasp fallback"
                % (resp.message if resp else "no response")
            )
            userdata.target_pose = None
        return "proceed"


# ---------------------------------------------------------------------------
# State: PickAndPlace
# ---------------------------------------------------------------------------
class PickAndPlace(_FetchState):
    """
    Move the arm to the grasp position.
    If color_picker returned a pose, use the MoveToPose service (Cartesian move)
    If perception failed, fall back to the named 'pre_grasp' position.
    """

    def __init__(self, node: RobotFetchNode, pickmeup_client: BphPickmeupClient):
        super().__init__(node, outcomes=["arm_at_grasp_position", "failed"],
                         input_keys=["target_pose"])
        self._pickmeup = pickmeup_client
        self._move_to_pose_client = node.create_client(MoveToPose, "/bph_pickmeup/move_to_pose")

    def execute(self, userdata):
        pose = userdata.target_pose
        if pose is not None:
            self._node.get_logger().info(
                "[PickAndPlace] Moving arm to detected object pose..."
            )
            req = MoveToPose.Request()
            req.target_pose = pose
            resp = self._call_service(self._move_to_pose_client, req)
            if resp and resp.success:
                return "arm_at_grasp_position"
            self._node.get_logger().warn(
                "[PickAndPlace] Cartesian move failed (%s) — falling back to pre_grasp"
                % (resp.message if resp else "no response")
            )

        self._node.get_logger().info("[PickAndPlace] Moving arm to pre_grasp position...")
        success, code = self._pickmeup.send_goal(position_name="pre_grasp")
        if success:
            return "arm_at_grasp_position"
        self._node.get_logger().warn("[PickAndPlace] Arm move failed, code=%d" % code)
        return "failed"


# ---------------------------------------------------------------------------
# State: Grasping
# ---------------------------------------------------------------------------
class Grasping(_FetchState):
    """No gripper — /button press is a manual stand-in for gripper close confirmation."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["grasp_confirmed"])

    def execute(self, userdata):
        self._node.get_logger().info("[Grasping] Press /button to confirm grasp...")
        self._wait_for_button()
        return "grasp_confirmed"


# ---------------------------------------------------------------------------
# State: MoveToWorkspace
# ---------------------------------------------------------------------------
class MoveToWorkspace(_FetchState):
    """Carry the grasped object to the workspace position."""

    def __init__(self, node: RobotFetchNode, pickmeup_client: BphPickmeupClient):
        super().__init__(node, outcomes=["in_workspace", "failed"])
        self._pickmeup = pickmeup_client

    def execute(self, userdata):
        self._node.get_logger().info("[MoveToWorkspace] Moving arm to workspace...")
        success, code = self._pickmeup.send_goal(position_name="workspace")
        if success:
            return "in_workspace"
        self._node.get_logger().warn("[MoveToWorkspace] Arm move failed, code=%d" % code)
        return "failed"


# ---------------------------------------------------------------------------
# State: SpringController
# ---------------------------------------------------------------------------
class SpringController(_FetchState):
    """Switch to effort controller and run spring/impedance control."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["done"])

    def execute(self, userdata):
        self._node.get_logger().info("[SpringController] Switching to effort controller...")
        self._node.switch_controllers(
            activate=[self.EFFORT_CONTROLLER],
            deactivate=[self.POSITION_CONTROLLER],
        )
        self._node.get_logger().info(
            "[SpringController] spring controller running"
        )

        return "done"


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    node = RobotFetchNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.build_and_run_sm()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
