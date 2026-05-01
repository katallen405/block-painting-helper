#!/usr/bin/env python3
"""
bph_pickmeup — Action Server
Wraps RobotMover (MoveIt MoveGroup client) in a ROS2 action server so that
callers (e.g. the SMACH state machine) can:
  - Send a goal (joint angles or named position)
  - Receive streaming feedback (planning / executing)
  - Get a clean success/failure result with a MoveIt error code

Action:   bph_interfaces/action/BphPickmeup
Server:   /bph_pickmeup

Usage
-----
  ros2 run bph_pickmeup bph_pickmeup_action_server
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Vector3
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
)
from shape_msgs.msg import SolidPrimitive

from bph_interfaces.action import BphPickmeup
from bph_interfaces.srv import MoveToPose


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Named positions registry
NAMED_POSITIONS: dict[str, list[float]] = {
    "home":      [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0],
    "pre_grasp": [0.0, -1.2000, 0.5, -1.8000, 0.0, 0.0],
    # add more named positions here
}


class BphPickmeupServer(Node):

    def __init__(self):
        super().__init__("bph_pickmeup_action_server")
        
        self._execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'  
)
        # ReentrantCallbackGroup lets the action server handle goals
        # concurrently with the MoveIt client callbacks.
        self._cb_group = ReentrantCallbackGroup()

        # MoveIt MoveGroup action client
        self._moveit_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
            callback_group=self._cb_group,
        )
        self.get_logger().info("Waiting for /move_action server...")
        self._moveit_client.wait_for_server()
        self.get_logger().info("Connected to /move_action.")

        # Our own action server
        self._action_server = ActionServer(
            self,
            BphPickmeup,
            "/bph_pickmeup",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info("bph_pickmeup action server ready.")

        self._move_to_pose_srv = self.create_service(
            MoveToPose,
            "/bph_pickmeup/move_to_pose",
            self._move_to_pose_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info("bph_pickmeup/move_to_pose service ready.")

    # ------------------------------------------------------------------
    # Goal / cancel policy
    # ------------------------------------------------------------------

    def _goal_cb(self, goal_request):
        """Accept all incoming goals."""
        self.get_logger().info("Received goal request.")
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        """Accept cancellation requests."""
        self.get_logger().info("Cancel requested.")
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # Main execution callback
    # ------------------------------------------------------------------

    def _execute_cb(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback = BphPickmeup.Feedback()
        result   = BphPickmeup.Result()

        # ---- Resolve joint angles ----
        joint_angles = list(goal_handle.request.joint_angles)
        if not joint_angles:
            name = goal_handle.request.position_name
            if name not in NAMED_POSITIONS:
                msg = f"Unknown position '{name}' and no joint_angles provided."
                self.get_logger().error(msg)
                result.success    = False
                result.error_code = MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS
                result.message    = msg
                goal_handle.abort()
                return result
            joint_angles = NAMED_POSITIONS[name]

        if len(joint_angles) != len(JOINT_NAMES):
            msg = f'Expected {len(JOINT_NAMES)} joint angles, got {len(joint_angles)}.'
            self.get_logger().error(msg)
            result.success    = False
            result.error_code = MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS
            result.message    = msg
            goal_handle.abort()
            return result

        # ---- Feedback: planning ----
        feedback.current_state = 'planning'
        feedback.progress      = 0.1
        goal_handle.publish_feedback(feedback)

        # ---- Send to MoveIt (plan_only=True first to get plan confirmation) ----
        self.get_logger().info('Requesting plan from MoveIt...')
        plan_goal          = self._build_moveit_goal(joint_angles)
        plan_goal.planning_options.plan_only = True   # plan without executing

        send_future = self._moveit_client.send_goal_async(plan_goal)
        rclpy.spin_until_future_complete(self, send_future)
        plan_handle = send_future.result()

        if not plan_handle.accepted:
            feedback.current_state = 'planning_failed'
            feedback.progress      = 0.0
            goal_handle.publish_feedback(feedback)
            result.success    = False
            result.error_code = MoveItErrorCodes.PLANNING_FAILED
            result.message    = 'MoveIt rejected the plan request.'
            goal_handle.abort()
            return result

        plan_result_future = plan_handle.get_result_async()
        rclpy.spin_until_future_complete(self, plan_result_future)
        plan_result = plan_result_future.result().result

        if plan_result.error_code.val != MoveItErrorCodes.SUCCESS:
            # ---- Feedback: planning failed — state machine sees this and can act ----
            feedback.current_state = 'planning_failed'
            feedback.progress      = 0.0
            goal_handle.publish_feedback(feedback)

            result.success    = False
            result.error_code = plan_result.error_code.val
            result.message    = f'MoveIt planning failed: {plan_result.error_code.val}'
            self.get_logger().warn(result.message)
            goal_handle.abort()
            return result

        # ---- Feedback: planning_complete — state machine transitions here ----
        #
        # This is the key message.  The SMACH state is polling for exactly this
        # string and will return 'planned' as soon as it arrives, transitioning
        # to MovingToPickUpObject while the arm has not yet started moving.
        #
        feedback.current_state = 'planning_complete'
        feedback.progress      = 0.4
        goal_handle.publish_feedback(feedback)
        self.get_logger().info('Plan confirmed, publishing planning_complete feedback.')

        # Small yield so the feedback message is flushed before we block again
        import time
        time.sleep(0.05)

        # Check for cancellation (the SMACH state may have moved on and cancelled)
        if goal_handle.is_cancel_requested:
            self.get_logger().info('Goal cancelled after planning.')
            goal_handle.canceled()
            result.success = False
            result.message = 'Cancelled after planning.'
            return result

        # ---- Now execute the plan ----
        self.get_logger().info('Executing plan...')
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = plan_result.planned_trajectory # reuse the plan from before

        feedback.current_state = 'executing'
        feedback.progress      = 0.6
        goal_handle.publish_feedback(feedback)

        exec_future = self._moveit_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, exec_future)
        exec_handle = exec_future.result()

        if not exec_handle.accepted:
            result.success    = False
            result.error_code = MoveItErrorCodes.PLANNING_FAILED
            result.message    = 'MoveIt rejected the execution goal.'
            goal_handle.abort()
            return result

        exec_result_future = exec_handle.get_result_async()
        rclpy.spin_until_future_complete(self, exec_result_future)

        if goal_handle.is_cancel_requested:
            exec_handle.cancel_goal_async()
            goal_handle.canceled()
            result.success = False
            result.message = 'Cancelled during execution.'
            return result

        exec_result  = exec_result_future.result().result
        error_val    = exec_result.error_code.val

        if error_val == MoveItErrorCodes.SUCCESS:
            feedback.current_state = 'done'
            feedback.progress      = 1.0
            goal_handle.publish_feedback(feedback)
            result.success    = True
            result.error_code = error_val
            result.message    = 'Motion completed successfully.'
            self.get_logger().info('Goal succeeded.')
            goal_handle.succeed()
        else:
            result.success    = False
            result.error_code = error_val
            result.message    = f'Execution failed: {error_val}'
            self.get_logger().warn(result.message)
            goal_handle.abort()

        return result


    # ------------------------------------------------------------------
    # MoveToPose service — Cartesian end-effector move
    # ------------------------------------------------------------------

    def _move_to_pose_cb(self, request: MoveToPose.Request, response: MoveToPose.Response):
        self.get_logger().info("MoveToPose request received.")
        end_effector = request.end_effector_link or "tool0"
        goal = self._build_cartesian_goal(request.target_pose, end_effector)

        accept_event = threading.Event()
        accept_result = [None]

        def accept_cb(f):
            accept_result[0] = f.result()
            accept_event.set()

        self._moveit_client.send_goal_async(goal).add_done_callback(accept_cb)
        accept_event.wait()

        goal_handle = accept_result[0]
        if not goal_handle.accepted:
            response.success = False
            response.message = "MoveIt rejected the Cartesian goal."
            return response

        result_event = threading.Event()
        result_val = [None]

        def result_cb(f):
            result_val[0] = f.result().result
            result_event.set()

        goal_handle.get_result_async().add_done_callback(result_cb)
        result_event.wait()

        moveit_result = result_val[0]
        if moveit_result.error_code.val == MoveItErrorCodes.SUCCESS:
            response.success = True
            response.message = "Cartesian motion completed successfully."
            self.get_logger().info("MoveToPose succeeded.")
        else:
            response.success = False
            response.message = f"MoveIt failed: {moveit_result.error_code.val}"
            self.get_logger().warn(response.message)
        return response

    def _build_cartesian_goal(self, target_pose, end_effector_link: str) -> MoveGroup.Goal:
        pos_c = PositionConstraint()
        pos_c.header = target_pose.header
        pos_c.link_name = end_effector_link
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        pos_c.constraint_region = BoundingVolume(
            primitives=[sphere], primitive_poses=[target_pose.pose]
        )
        pos_c.weight = 1.0

        ori_c = OrientationConstraint()
        ori_c.header = target_pose.header
        ori_c.link_name = end_effector_link
        ori_c.orientation = target_pose.pose.orientation
        ori_c.absolute_x_axis_tolerance = 0.01
        ori_c.absolute_y_axis_tolerance = 0.01
        ori_c.absolute_z_axis_tolerance = 0.01
        ori_c.weight = 1.0

        request = MotionPlanRequest()
        request.group_name = "ur_manipulator"
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3
        frame = target_pose.header.frame_id or "base_link"
        request.workspace_parameters.header.frame_id = frame
        request.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        request.workspace_parameters.max_corner = Vector3(x=1.0, y=1.0, z=1.0)
        constraints = Constraints()
        constraints.position_constraints = [pos_c]
        constraints.orientation_constraints = [ori_c]
        request.goal_constraints = [constraints]

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        goal.planning_options.planning_scene_diff.is_diff = True
        return goal

    # ------------------------------------------------------------------
    # MoveIt goal construction (ported from RobotMover.move_to)
    # ------------------------------------------------------------------

    def _build_moveit_goal(self, joint_angles: list[float]) -> MoveGroup.Goal:
        request = MotionPlanRequest()
        request.group_name = "ur_manipulator"
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3
        request.workspace_parameters.header.frame_id = "base_link"
        request.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        request.workspace_parameters.max_corner = Vector3(x=1.0, y=1.0, z=1.0)

        constraints = Constraints()
        for joint_name, angle in zip(JOINT_NAMES, joint_angles):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = angle
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        request.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        goal.planning_options.planning_scene_diff.is_diff = True
        return goal


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    server = BphPickmeupServer()

    # MultiThreadedExecutor is required so the MoveIt client callbacks
    # can fire while the action server execute callback is blocked on
    # spin_until_future_complete.
    executor = MultiThreadedExecutor()
    executor.add_node(server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
