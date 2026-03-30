import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_py import MoveItPy
from shape_msgs.msg import SolidPrimitive

from bph_pickmeup_msgs.srv import MoveToPose


class MoveItPickupNode(Node):
    def __init__(self):
        super().__init__('bph_pickmeup')

        self.declare_parameter('arm_names', ['my_arm'])
        self.declare_parameter('default_arm', 'my_arm')

        arm_names = self.get_parameter('arm_names').value
        self._default_arm = self.get_parameter('default_arm').value

        self._robot = MoveItPy(node_name='bph_pickmeup_moveit')
        self._planning_scene_monitor = self._robot.get_planning_scene_monitor()

        self._planning_components = {}
        for name in arm_names:
            self._planning_components[name] = self._robot.get_planning_component(name)
            self.get_logger().info(f'Registered planning component: {name}')

        self.create_service(MoveToPose, 'move_to_pose', self._move_to_pose_callback)
        self.get_logger().info('bph_pickmeup ready')

    def _move_to_pose_callback(self, request, response):
        arm_name = request.arm_name if request.arm_name else self._default_arm

        if arm_name not in self._planning_components:
            response.success = False
            response.message = (
                f'Unknown arm "{arm_name}". '
                f'Available: {list(self._planning_components.keys())}'
            )
            return response

        planning_component = self._planning_components[arm_name]
        planning_component.set_start_state_to_current_state()
        planning_component.set_goal_state(
            pose_stamped_msg=request.target_pose,
            pose_link=request.end_effector_link,
        )

        plan_result = planning_component.plan()
        if not plan_result:
            response.success = False
            response.message = f'Motion planning failed for arm "{arm_name}"'
            return response

        self._robot.execute(plan_result.trajectory, controllers=[])
        response.success = True
        response.message = f'Moved "{arm_name}" to target pose'
        return response

    def add_collision_box(self, frame_id, object_id, position, dimensions):
        """Add a box collision object to the planning scene.

        position: (x, y, z) tuple
        dimensions: (size_x, size_y, size_z) tuple
        """
        with self._planning_scene_monitor.read_write() as scene:
            collision_object = CollisionObject()
            collision_object.header.frame_id = frame_id
            collision_object.id = object_id

            box_pose = Pose()
            box_pose.position.x = float(position[0])
            box_pose.position.y = float(position[1])
            box_pose.position.z = float(position[2])

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = list(dimensions)

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD

            scene.apply_collision_object(collision_object)
            scene.current_state.update()


def main(args=None):
    rclpy.init(args=args)
    node = MoveItPickupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
