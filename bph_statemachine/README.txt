Smoke-testing the state machine
================================

State diagram: state_machine.png / state_machine.pdf

Start the node:
  ros2 run bph_statemachine simple_sm_node

Optionally watch live state transitions:
  ros2 run smach_viewer smach_viewer

For each transition below: the left column is what drives it, and the
right column is the manual command to use when the real driver isn't running.

─────────────────────────────────────────────────────────────────────────────
WAIT → RETRIEVING_OBJECT

  Driver: /button String — payload is the color to fetch.
  The SM stores this as requested_object_color for the perception step.

  ros2 topic pub --once /button std_msgs/msg/String "{data: 'red'}"

  Valid colors: red, green, blue, yellow  (must match color_picker profiles)

─────────────────────────────────────────────────────────────────────────────
RETRIEVING_OBJECT → WAITING_FOR_OBJECT  (or → WAIT on nav_error)

  Driver: navigate_to service (GoToLocation) returning accepted=true.
  Happens automatically when the nav stack accepts and completes the goal.

  To mock without nav stack — run this in a separate terminal:
    ros2 run bph_statemachine mock_nav_server
    (TODO: add mock_nav_server node, or use the snippet below)

  Quick Python mock (paste into a terminal):
    python3 -c "
import rclpy
from rclpy.node import Node
from bph_interfaces.srv import GoToLocation
rclpy.init()
n = Node('mock_nav')
n.create_service(GoToLocation, 'navigate_to',
    lambda req, resp: setattr(resp, 'accepted', True) or setattr(resp, 'message', 'ok') or resp)
rclpy.spin(n)"

─────────────────────────────────────────────────────────────────────────────
WAITING_FOR_OBJECT → NAVIGATING_HOME

  Driver: /button String (payload ignored — just confirms the block is loaded).

  ros2 topic pub --once /button std_msgs/msg/String "{data: 'go'}"

─────────────────────────────────────────────────────────────────────────────
NAVIGATING_HOME → LOCATING_OBJECT_AND_PEOPLE  (or → WAIT on nav_error)

  Driver: navigate_to service — same as RETRIEVING_OBJECT above.
  The mock server above handles both nav calls.

─────────────────────────────────────────────────────────────────────────────
LOCATING_OBJECT_AND_PEOPLE → PICK_AND_PLACE

  Driver: /color_picker/get_target_pose service (GetTargetPose).
  Always proceeds to PICK_AND_PLACE — if perception fails, target_pose is
  set to None and PickAndPlace falls back to the named 'pre_grasp' position.

  To mock a successful detection:
    python3 -c "
import rclpy
from rclpy.node import Node
from bph_interfaces.srv import GetTargetPose
from geometry_msgs.msg import PoseStamped
rclpy.init()
n = Node('mock_perception')
def cb(req, resp):
    resp.success = True
    resp.message = 'mock pose'
    p = PoseStamped()
    p.header.frame_id = 'base_link'
    p.pose.position.x = 0.4
    p.pose.position.y = 0.0
    p.pose.position.z = 0.1
    p.pose.orientation.w = 1.0
    resp.pose = p
    return resp
n.create_service(GetTargetPose, '/color_picker/get_target_pose', cb)
rclpy.spin(n)"

  To mock a failed detection (SM will use pre_grasp fallback):
    python3 -c "
import rclpy
from rclpy.node import Node
from bph_interfaces.srv import GetTargetPose
rclpy.init()
n = Node('mock_perception')
n.create_service(GetTargetPose, '/color_picker/get_target_pose',
    lambda req, resp: setattr(resp, 'success', False) or setattr(resp, 'message', 'not found') or resp)
rclpy.spin(n)"

─────────────────────────────────────────────────────────────────────────────
PICK_AND_PLACE → GRASPING  (or → WAIT on failed)

  Driver: /bph_pickmeup/move_to_pose service (if perception succeeded)
          or /bph_pickmeup action server (pre_grasp fallback).
  Requires bph_pickmeup_node and MoveIt to be running.

  ros2 run bph_pickmeup bph_pickmeup_node

─────────────────────────────────────────────────────────────────────────────
GRASPING → MOVE_TO_WORKSPACE

  Driver: /button String (no gripper — button confirms grasp manually).

  ros2 topic pub --once /button std_msgs/msg/String "{data: 'go'}"

─────────────────────────────────────────────────────────────────────────────
MOVE_TO_WORKSPACE → SPRING_CONTROLLER  (or → WAIT on failed)

  Driver: /bph_pickmeup action server ('workspace' named position).
  Requires bph_pickmeup_node and MoveIt — same as PICK_AND_PLACE above.

─────────────────────────────────────────────────────────────────────────────
SPRING_CONTROLLER → WAIT  (loops)

  Driver: automatic after switching to effort controller.
  Requires /controller_manager/switch_controller service to be available.
  The SM loops back to WAIT ready for the next block.
