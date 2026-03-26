from moveit_py import MoveItPy
from moveit_msgs.msg import CollisionObject
import rclpy
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

rclpy.init()
logger = rclpy.logging.get_logger("moveit_py.pose_goal")

# instantiate MoveItPy instance and get planning component
arm = MoveItPy(node_name="moveit_py")
arm_planning_component = arm.get_planning_component("my_arm")
logger.info("MoveItPy instance created")
planning_scene_monitor = arm.get_planning_scene_monitor()

# functions from the moveit examples in ROS2 
def plan_and_execute(
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        ):
        """A helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
                plan_result = planning_component.plan(
                        multi_plan_parameters=multi_plan_parameters
                )
        elif single_plan_parameters is not None:
                plan_result = planning_component.plan(
                        single_plan_parameters=single_plan_parameters
                )
        else:
                plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
                logger.info("Executing plan")
                robot_trajectory = plan_result.trajectory
                robot.execute(robot_trajectory, controllers=[])
        else:
                logger.error("Planning failed")

def add_planning_object()
        with planning_scene_monitor.read_write() as scene:
            collision_object = CollisionObject()
            collision_object.header.frame_id = "arm_link0"
            collision_object.id = "boxes"

            box_pose = Pose()
            box_pose.position.x = 0.15
            box_pose.position.y = 0.1
            box_pose.position.z = 0.6

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dimensions

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD

            scene.apply_collision_object(collision_object)
            scene.current_state.update()  # Important to ensure the scene is updated
