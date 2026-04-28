#!/usr/bin/env python3
"""
Scaffold from Claude.ai to get the SMACH syntax right.


States:
    Wait                    -> RetrievingObject        (on /button message)
    RetrievingObject        -> WaitingForObject         (when /turtlebot/location == "supply closet")
    WaitingForObject        -> NavigatingHome           (on /button message)
    NavigatingHome          -> LocatingObjectAndPeople  (when /turtlebot/location == "home")
    LocatingObjectAndPeople -> PickAndPlace             (on /object/location message)
    PickAndPlace            -> Grasping                 (when /arm/location == "grasp position")
    Grasping                -> MoveToWorkspace          (on /button message, simulating successful grasp)
    MoveToWorkspace         -> SpringController         (when /arm/location == "in workspace")
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, String

import smach
import smach_ros
from bph_interfaces.srv import GoToLocation
from bph_interfaces.srv import MoveToPose

# ---------------------------------------------------------------------------
# Helper: a simple latch that a subscriber callback can set
# ---------------------------------------------------------------------------
class _TopicLatch:
    """Signals execute() that a matching message has arrived on a topic."""

    def __init__(self):
        self.triggered = False
        self.value = None

    def reset(self):
        self.triggered = False
        self.value = None

    def callback(self, msg):
        self.value = msg.data if hasattr(msg, "data") else msg
        self.triggered = True


# ---------------------------------------------------------------------------
# RobotFetchNode — owns all ROS2 infrastructure
# ---------------------------------------------------------------------------
class RobotFetchNode(Node):
    """
    Central ROS2 node for the fetch-and-deliver task.

    Subclassing Node means:
      - Parameters, publishers, timers, and services all live here.
      - States receive `self` (the node) at construction, giving them full
        access to everything declared below without any globals.
      - Adding new ROS2 features later (e.g. action clients, TF listeners)
        is just a matter of adding attributes here.
    """

    def __init__(self):
        super().__init__("robot_fetch_smach")

        # ------------------------------------------------------------------
        # Parameters — declare them here so they're visible to `ros2 param`
        # ------------------------------------------------------------------
        self.declare_parameter("home_location", "home")
        self.declare_parameter("supply_closet_location", "supply closet")
        self.declare_parameter("grasp_position_name", "grasp position")
        self.declare_parameter("workspace_position_name", "in workspace")

        # ------------------------------------------------------------------
        # Publishers — add any outgoing topics here
        # ------------------------------------------------------------------
        # TODO: self.nav_goal_pub = self.create_publisher(...)
        # TODO: self.arm_goal_pub = self.create_publisher(...)
        # TODO: self.gripper_pub  = self.create_publisher(...)


        self.get_logger().info("RobotFetchNode initialised")

    # Convenience: read parameters in one place so states don't hard-code strings
    @property
    def home_location(self):
        x = self.get_parameter("home_location.x").get_parameter_value().double_value
        y = self.get_parameter("home_location.y").get_parameter_value().double_value
        yaw = self.get_parameter("home_location.yaw").get_parameter_value().double_value
        return x, y, yaw

    @property
    def supply_closet_location(self):
        x = self.get_parameter("supply_closet_location.x").get_parameter_value().double_value
        y = self.get_parameter("supply_closet_location.y").get_parameter_value().double_value
        yaw = self.get_parameter("supply_closet_location.yaw").get_parameter_value().double_value
        return x, y, yaw

    @property
    def grasp_position_name(self):
        return self.get_parameter("grasp_position_name").get_parameter_value().string_value

    @property
    def workspace_position_name(self):
        return self.get_parameter("workspace_position_name").get_parameter_value().string_value

    def build_and_run_sm(self):
        """Construct the SMACH state machine and execute it."""
        sm = smach.StateMachine(outcomes=["task_complete"])

        with sm:
            smach.StateMachine.add(
                "WAIT",
                Wait(self),
                transitions={"button_pressed": "RETRIEVING_OBJECT"},
            )
            smach.StateMachine.add(
                "RETRIEVING_OBJECT",
                RetrievingObject(self),
                transitions={"at_supply_closet": "WAITING_FOR_OBJECT"},
            )
            smach.StateMachine.add(
                "WAITING_FOR_OBJECT",
                WaitingForObject(self),
                transitions={"object_loaded": "NAVIGATING_HOME"},
            )
            smach.StateMachine.add(
                "NAVIGATING_HOME",
                NavigatingHome(self),
                transitions={"at_home": "LOCATING_OBJECT_AND_PEOPLE"},
            )
            smach.StateMachine.add(
                "LOCATING_OBJECT_AND_PEOPLE",
                LocatingObjectAndPeople(self),
                transitions={"object_located": "PICK_AND_PLACE"},
            )
            smach.StateMachine.add(
                "PICK_AND_PLACE",
                PickAndPlace(self),
                transitions={"arm_at_grasp_position": "GRASPING"},
            )
            smach.StateMachine.add(
                "GRASPING",
                Grasping(self),
                transitions={"grasp_confirmed": "MOVE_TO_WORKSPACE"},
            )
            smach.StateMachine.add(
                "MOVE_TO_WORKSPACE",
                MoveToWorkspace(self),
                transitions={"in_workspace": "SPRING_CONTROLLER"},
            )
            smach.StateMachine.add(
                "SPRING_CONTROLLER",
                SpringController(self),
                transitions={"done": "task_complete"},
            )

        sis = smach_ros.IntrospectionServer("robot_fetch_smach", sm, "/SM_ROOT")
        sis.start()

        outcome = sm.execute()
        self.get_logger().info("State machine finished with outcome: %s" % outcome)
        sis.stop()


# ---------------------------------------------------------------------------
# State base class — removes boilerplate from every state
# ---------------------------------------------------------------------------
class _FetchState(smach.State):
    """
    Shared base for all states in this machine.
    Stores the node reference and provides _wait_for_latch().
    """

    def __init__(self, node: RobotFetchNode, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        self._node = node
        self._latch = _TopicLatch()

    def _wait_for_latch(self):
        rate = self._node.create_rate(10)
        while rclpy.ok() and not self._latch.triggered:
            rate.sleep()
        return self._latch.last_message


# ---------------------------------------------------------------------------
# State: Wait
# ---------------------------------------------------------------------------
class Wait(_FetchState):
    """Idle state. Waits for a press on /button before starting the fetch."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["button_pressed"])

    def execute(self, userdata):
        self._node.get_logger().info("[Wait] Waiting for /button press …")
        self._latch.reset()
        sub = self._node.create_subscription(String, "/button", self._latch.callback, 10)
        msg = self._wait_for_latch()
        if msg:
            self._node.get_logger().info("[Wait] Button pressed")
            self._node.destroy_subscription(sub)
            return "button_pressed"


# ---------------------------------------------------------------------------
# State: RetrievingObject
# ---------------------------------------------------------------------------
class RetrievingObject(_FetchState):
    """Navigate toward the supply closet. Completes when location is confirmed."""
    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["at_supply_closet"])
        self._nav_client = self.create_client(GoToLocation, 'navigate_to')
        self._nav_client.wait_for_service(timeout_sec=5.0)
        self.create_subscription(String, 'navigation_status', self._status_callback, 10)


    def _location_cb(self, msg):
        if msg.data == self._node.supply_closet_location:
            self._latch.callback(msg)

    def execute(self, userdata):
        self._node.get_logger().info("[RetrievingObject] Navigating to supply closet …")
        request = GoToLocation.Request()
        x, y, yaw = self.supply_closet_location()
        request.x = x
        request.y = y
        request.yaw = yaw
        future = self._nav_client.call_async(request)
        future.add_done_callback(self._nav_response_callback)


        self._latch.reset()
        sub = self._node.create_subscription(String, "/turtlebot/location", self._location_cb, 10)
        msg = self._wait_for_latch()
        if msg == "supply_closet":
            self._node.get_logger().info("[RetrievingObject] Arrived at supply closet")
            self._node.destroy_subscription(sub)
            return "at_supply_closet"
        else:
            self._node.get_logger().warn("[RetrievingObject] Received unexpected location: %s" % msg)
            self._node.destroy_subscription(sub)
            return "nav_error" 


# ---------------------------------------------------------------------------
# State: WaitingForObject
# ---------------------------------------------------------------------------
class WaitingForObject(_FetchState):
    """Wait at the supply closet for a human to load the object (button confirm)."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["object_loaded"])

    def execute(self, userdata):
        self._node.get_logger().info("[WaitingForObject] Waiting for object to be loaded (/button) …")
        self._latch.reset()
        sub = self._node.create_subscription(String, "/button", self._latch.callback, 10)
        self._wait_for_latch()
        self._node.destroy_subscription(sub)
        self._node.get_logger().info("[WaitingForObject] Object loaded")
        return "object_loaded"


# ---------------------------------------------------------------------------
# State: NavigatingHome
# ---------------------------------------------------------------------------
class NavigatingHome(_FetchState):
    """Return to the home base."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["at_home"])

    def _location_cb(self, msg):
        if msg.data == self._node.home_location:
            self._latch.callback(msg)

    def execute(self, userdata):
        self._node.get_logger().info("[NavigatingHome] Navigating home …")
        request = GoToLocation.Request()
        x, y, yaw = self.home_location()
        request.x = x
        request.y = y
        request.yaw = yaw
        self._latch.reset()
        sub = self._node.create_subscription(String, "/turtlebot/location", self._location_cb, 10)
        msg = self._wait_for_latch()
        if msg == "home":
             self._node.get_logger().info("[NavigatingHome] Arrived at home")
             self._node.destroy_subscription(sub)
             return "at_home"
        else:
             self._node.get_logger().warn("[NavigatingHome] Received unexpected location: %s" % msg)
             self._node.destroy_subscription(sub)
             return "nav_error"



# ---------------------------------------------------------------------------
# State: LocatingObjectAndPeople
# ---------------------------------------------------------------------------
class LocatingObjectAndPeople(_FetchState):
    """Use perception to locate the object and nearby people."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["object_located"])

    def execute(self, userdata):
        self._node.get_logger().info("[LocatingObjectAndPeople] Waiting for /object/location …")
        # TODO: request locations of people to avoid
        # TODO: request location of object to grasp
        
        self._latch.reset()
        sub = self._node.create_subscription(String, "/object/location", self._latch.callback, 10)
        self._wait_for_latch()
        self._node.destroy_subscription(sub)
        self._node.get_logger().info(
            "[LocatingObjectAndPeople] Object located at: %s" % self._latch.value
        )
        return "object_located"


# ---------------------------------------------------------------------------
# State: PickAndPlace
# ---------------------------------------------------------------------------
class PickAndPlace(_FetchState):
    """Move the arm to the grasp position above the object."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["arm_at_grasp_position"])

    def _arm_cb(self, msg):
        if msg.data == self._node.grasp_position_name:
            self._latch.callback(msg)

    def execute(self, userdata):
        self._node.get_logger().info("[PickAndPlace] Moving arm to grasp position …")
        request = MoveToPose.Request()
        x, y, yaw = self.home_location()
        request.target_pose = geometry_msgs/PoseStamped()
        request.target_pose.pose.position.x = x
        request.target_pose.pose.position.y = y
        request.target_pose.pose.position.z = 0.0
        request.target_pose.pose.orientation.z = yaw
        request.arm_name = ""
        request.end_effector_link = ""
        self._latch.reset()
        sub = self._node.create_subscription(String, "/arm/location", self._arm_cb, 10)
        msg = self._wait_for_latch()
        if msg == "grasp position":
            self._node.get_logger().info("[PickAndPlace] Arm at grasp position")
            self._node.destroy_subscription(sub)
            return "arm_at_grasp_position"
        else:
            self._node.get_logger().warn("[PickAndPlace] Received unexpected arm location: %s" % msg)   
            self._node.destroy_subscription(sub)
            return "arm_not_at_grasp_position"


# ---------------------------------------------------------------------------
# State: Grasping
# ---------------------------------------------------------------------------
class Grasping(_FetchState):
    """Close the gripper and confirm grasp via /button (simulated success signal)."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["grasp_confirmed"])

    def execute(self, userdata):
        self._node.get_logger().info("[Grasping] Closing gripper – waiting for confirmation (/button) …")
        # TODO: send gripper close command here
        self._latch.reset()
        sub = self._node.create_subscription(Bool, "/button", self._latch.callback, 10)
        msg = self._wait_for_latch()
        if msg:
            self._node.get_logger().info("[Grasping] Grasp confirmed")
            self._node.destroy_subscription(sub)
            return "grasp_confirmed"


# ---------------------------------------------------------------------------
# State: MoveToWorkspace
# ---------------------------------------------------------------------------
class MoveToWorkspace(_FetchState):
    """Carry the grasped object to the workspace position."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["in_workspace"])

    def _arm_cb(self, msg):
        if msg.data == self._node.workspace_position_name:
            self._latch.callback(msg)

    def execute(self, userdata):
        self._node.get_logger().info("[MoveToWorkspace] Moving arm to workspace …")
        # TODO: send arm goal here
        self._latch.reset()
        sub = self._node.create_subscription(String, "/arm/location", self._arm_cb, 10)
        msg = self._wait_for_latch()
        if msg == "workspace position":
            self._node.destroy_subscription(sub)
            self._node.get_logger().info("[MoveToWorkspace] Arm in workspace – handing off to SpringController")
            return "in_workspace"


# ---------------------------------------------------------------------------
# State: SpringController  (terminal / placeholder)
# ---------------------------------------------------------------------------
class SpringController(_FetchState):
    """Placeholder for spring/impedance controller once the arm is in workspace."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["done"])

    def execute(self, userdata):
        self._node.get_logger().info(
            "[SpringController] Spring controller active – TODO: implement control loop"
        )
        # TODO: implement spring/impedance controller
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
