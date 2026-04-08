#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, JointConstraint, Constraints, MoveItErrorCodes


# Double check joint names within RVIZ
JOINT_NAMES = [
   "shoulder_pan_joint",
   "shoulder_lift_joint",
   "elbow_joint",
   "wrist_1_joint",
   "wrist_2_joint",
   "wrist_3_joint",
]
 
class RobotMover(Node):
 
   def __init__(self):
       super().__init__("robot_mover")
       self.action_client = ActionClient(self, MoveGroup, "/move_action")
       print("Connecting to robot...")
       self.action_client.wait_for_server()
       print("Connected.\n")
 
   def move_to(self, name, joint_angles):
       print(f"Moving to {name}...")
 
       request = MotionPlanRequest()
       request.group_name = "ur_manipulator"
       request.num_planning_attempts = 10
       request.allowed_planning_time = 5.0
       request.max_velocity_scaling_factor = 0.3
       request.max_acceleration_scaling_factor = 0.3
       request.workspace_parameters.header.frame_id = "base_link"
       request.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
       request.workspace_parameters.max_corner = Vector3(x= 1.0, y= 1.0, z= 1.0)
 
       constraints = Constraints()
       for name_j, angle in zip(JOINT_NAMES, joint_angles):
           jc = JointConstraint()
           jc.joint_name      = name_j
           jc.position        = angle
           jc.tolerance_above = 0.01
           jc.tolerance_below = 0.01
           jc.weight          = 1.0
           constraints.joint_constraints.append(jc)
       request.goal_constraints.append(constraints)
 
       goal = MoveGroup.Goal()
       goal.request = request
       goal.planning_options.plan_only = False
       goal.planning_options.replan = True
       goal.planning_options.replan_attempts = 5
       goal.planning_options.planning_scene_diff.is_diff = True
 
       future = self.action_client.send_goal_async(goal)
       rclpy.spin_until_future_complete(self, future)
       result_future = future.result().get_result_async()
       rclpy.spin_until_future_complete(self, result_future)
 
       if result_future.result().result.error_code.val == MoveItErrorCodes.SUCCESS:
           print(f"Reached {name}.\n")
       else:
           print(f"Failed to reach {name}. Error code: {result_future.result().result.error_code.val}\n")
 
 
def main(object_name, position):
   rclpy.init()
   robot = RobotMover()
   robot.move_to(object_name,position)
   robot.destroy_node()
   rclpy.shutdown()
 
if __name__ == "__main__":
    POSITION_1 = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]
    POSITION_2 = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]
    POSITION_3 = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]
    main("Position 1", POSITION_1)
