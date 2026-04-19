#!/usr/bin/env python3
"""
ROS2 SMACH State Machine: Pick and Place
States:
  - FindObjectLocation
  - ReadyToPickUpObject
  - MovingToPickUpObject
  - PickUpObject
  - ObjectInGripperMoveToWorkspace
  - SpringController
  - OpenGripperForUser
  - MoveObjectOnMobileBase (recovery)
  - AskUserForHelp (recovery)
  - Waiting
"""

import rclpy
from rclpy.node import Node
import smach
import smach_ros
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose
from bph_pickmeup.bph_pickmeup_client import BphPickmeupClient


# ---------------------------------------------------------------------------
# Shared userdata keys (passed between states via SMACH userdata)
# ---------------------------------------------------------------------------
# object_pose      : geometry_msgs/Pose  – current detected object pose
# ee_plan          : object              – motion plan to object location
# gripper_holding  : bool                – True if gripper has object
# fail_count       : int                 – retry counter for workspace move
# FAIL_LIMIT       : int                 – max retries before asking user


FAIL_LIMIT = 3


   

# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------

class FindObjectLocation(smach.State):
    """
    Recovery / entry state.
    Runs perception pipeline to localise the object.
    Outputs: object_pose in userdata
    """

    def __init__(self, node: Node):
        super().__init__(
            outcomes=['found', 'not_found'],
            input_keys=[],
            output_keys=['object_pose']
        )
        self._node = node
        

    def execute(self, userdata):
        self._node.get_logger().info('[FindObjectLocation] Searching for object...')
        # TODO: call your perception service here
        # e.g. result = self._node.perception_client.call(...)
        # Stub: assume found
        userdata.object_pose = Pose()   # replace with real pose
        self._node.get_logger().info('[FindObjectLocation] Object found.')
        return 'found'


import threading
import rclpy
from rclpy.action import ActionClient
import smach
from moveit_msgs.msg import MoveItErrorCodes

from bph_interfaces.action import BphPickmeup   # your generated action


class ReadyToPickUpObject(smach.State):
    """
    Sends the object location to bph_pickmeup and transitions as soon as
    a motion plan is confirmed — without waiting for execution to finish.

    Outcomes
    --------
    planned      : a valid plan exists  → MovingToPickUpObject
    cannot_plan  : MoveIt cannot plan   → MoveObjectOnMobileBase
    no_precond   : missing precondition → FindObjectLocation
    """

    def __init__(self, node):
        super().__init__(
            outcomes=['planned', 'cannot_plan', 'no_precond'],
            input_keys=['object_pose'],
            output_keys=['ee_plan']
        )
        self._node = node

        # Action client — created once, reused across executions
        self._client = ActionClient(node, BphPickmeup, '/bph_pickmeup')
        self._node.get_logger().info('Waiting for /bph_pickmeup...')
        self._client.wait_for_server()
        self._node.get_logger().info('/bph_pickmeup ready.')

        # Shared state written by the feedback callback, read by execute()
        self._feedback_state: str = ''
        self._feedback_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Feedback callback — called by the ROS executor on every feedback msg
    # ------------------------------------------------------------------

    def _on_feedback(self, feedback_msg):
        """Stores the latest current_state string from the server."""
        state = feedback_msg.feedback.current_state
        with self._feedback_lock:
            self._feedback_state = state
        self._node.get_logger().debug(
            f'[ReadyToPickUpObject] feedback: {state} '
            f'({feedback_msg.feedback.progress * 100:.0f}%)'
        )

    # ------------------------------------------------------------------
    # SMACH execute
    # ------------------------------------------------------------------

    def execute(self, userdata):
        self._node.get_logger().info('[ReadyToPickUpObject] Checking preconditions...')

        # ---- Precondition checks ----
        if not getattr(userdata, 'object_pose', None):
            self._node.get_logger().warn('[ReadyToPickUpObject] No object pose.')
            return 'no_precond'

        object_in_workspace = True   # TODO: real workspace check
        if not object_in_workspace:
            return 'no_precond'

        # ---- Send goal with feedback callback ----
        self._node.get_logger().info('[ReadyToPickUpObject] Sending goal to bph_pickmeup...')

        with self._feedback_lock:
            self._feedback_state = ''

        goal = BphPickmeup.Goal()
        # Use joint_angles or position_name depending on your convention:
        # goal.joint_angles = userdata.object_pose   # if already joint angles
        # goal.position_name = "pre_grasp"           # or a named position
        goal.position_name = 'pre_grasp'             # example

        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback,    # <-- registered here
        )

        # Wait only until the server *accepts* the goal (very fast)
        rclpy.spin_until_future_complete(self._node, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self._node.get_logger().warn('[ReadyToPickUpObject] Goal rejected.')
            return 'cannot_plan'

        # ---- Poll until we see planning feedback or a terminal state ----
        #
        # spin_once lets the executor deliver incoming feedback callbacks
        # without blocking indefinitely.  We exit as soon as:
        #   - feedback says planning is done  → 'planned'
        #   - the action result arrives early (e.g. immediate failure) → handled below
        #
        rate = self._node.create_rate(20)   # 20 Hz poll

        while rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.05)

            with self._feedback_lock:
                current = self._feedback_state

            self._node.get_logger().debug(
                f'[ReadyToPickUpObject] polling, state={current!r}'
            )

            if current == 'planning_complete':
                # Plan exists — transition NOW, before execution finishes.
                # Store the goal_handle so MovingToPickUpObject can monitor it.
                userdata.ee_plan = goal_handle
                self._node.get_logger().info(
                    '[ReadyToPickUpObject] Plan confirmed → transitioning.'
                )
                return 'planned'

            if current == 'planning_failed':
                self._node.get_logger().warn('[ReadyToPickUpObject] Planning failed.')
                # Cancel the action so the server cleans up
                goal_handle.cancel_goal_async()
                return 'cannot_plan'

            # If the action completed before we got a planning_complete feedback
            # (shouldn't happen with a well-behaved server, but be safe)
            if goal_handle.status in (
                # rclpy action status codes
                4,   # SUCCEEDED
                5,   # CANCELED
                6,   # ABORTED
            ):
                self._node.get_logger().warn(
                    f'[ReadyToPickUpObject] Action ended prematurely, '
                    f'status={goal_handle.status}'
                )
                return 'cannot_plan'

        return 'cannot_plan'   


class MovingToPickUpObject(smach.State):
    """
    Preconditions:
      - object in arm workspace
      - ee_plan is set
    Action:
      - execute motion plan, move EE to object location
    Transitions:
      at_object    -> PickUpObject
      object_moved -> FindObjectLocation
      ee_failed    -> ReadyToPickUpObject
    """

    def __init__(self, node: Node):
        super().__init__(
            outcomes=['at_object', 'object_moved', 'ee_failed'],
            input_keys=['ee_plan', 'object_pose'],
            output_keys=[]
        )
        self._node = node

    def execute(self, userdata):
        self._node.get_logger().info('[MovingToPickUpObject] Executing motion plan...')

        # TODO: execute userdata.ee_plan via MoveIt or similar
        # Stub outcomes – replace with real execution feedback
        execution_result = 'success'   # 'success' | 'object_moved' | 'failed'

        if execution_result == 'success':
            self._node.get_logger().info('[MovingToPickUpObject] EE reached object location.')
            return 'at_object'
        elif execution_result == 'object_moved':
            self._node.get_logger().warn('[MovingToPickUpObject] Object has moved.')
            return 'object_moved'
        else:
            self._node.get_logger().warn('[MovingToPickUpObject] EE did not reach location.')
            return 'ee_failed'


class PickUpObject(smach.State):
    """
    Preconditions:
      - object in arm workspace
      - EE at gripping location
    Action:
      - close gripper
    Transitions:
      gripped      -> ObjectInGripperMoveToWorkspace
      grip_failed  -> FindObjectLocation
    """

    def __init__(self, node: Node):
        super().__init__(
            outcomes=['gripped', 'grip_failed'],
            input_keys=[],
            output_keys=['gripper_holding']
        )
        self._node = node

    def execute(self, userdata):
        self._node.get_logger().info('[PickUpObject] Closing gripper...')

        # TODO: call gripper close service / action
        grip_success = True   # replace with real feedback

        if grip_success:
            userdata.gripper_holding = True
            self._node.get_logger().info('[PickUpObject] Object gripped successfully.')
            return 'gripped'
        else:
            userdata.gripper_holding = False
            self._node.get_logger().warn('[PickUpObject] Grip failed.')
            return 'grip_failed'


class ObjectInGripperMoveToWorkspace(smach.State):
    """
    Precondition:
      - gripper_holding == True
    Action:
      - plan and move to workspace location
    Transitions:
      at_workspace -> SpringController
      retry        -> ObjectInGripperMoveToWorkspace   (increments fail_count)
      ask_for_help -> AskUserForHelp                  (fail_count > FAIL_LIMIT)
    """

    def __init__(self, node: Node):
        super().__init__(
            outcomes=['at_workspace', 'retry', 'ask_for_help'],
            input_keys=['gripper_holding', 'fail_count'],
            output_keys=['fail_count']
        )
        self._node = node

    def execute(self, userdata):
        self._node.get_logger().info('[ObjectInGripperMoveToWorkspace] Planning to workspace...')

        if not getattr(userdata, 'gripper_holding', False):
            self._node.get_logger().error('[ObjectInGripperMoveToWorkspace] Gripper is not holding – precondition failure.')
            userdata.fail_count = getattr(userdata, 'fail_count', 0) + 1
            if userdata.fail_count > FAIL_LIMIT:
                return 'ask_for_help'
            return 'retry'

        # TODO: plan to workspace pose via MoveIt or similar
        plan_success = True   # replace with real result

        if plan_success:
            userdata.fail_count = 0
            self._node.get_logger().info('[ObjectInGripperMoveToWorkspace] Reached workspace.')
            return 'at_workspace'
        else:
            fail_count = getattr(userdata, 'fail_count', 0) + 1
            userdata.fail_count = fail_count
            self._node.get_logger().warn(
                f'[ObjectInGripperMoveToWorkspace] Failed to reach workspace (attempt {fail_count}).'
            )
            if fail_count > FAIL_LIMIT:
                return 'ask_for_help'
            return 'retry'


class SpringController(smach.State):
    """
    Preconditions:
      - gripper_holding == True
      - EE in workspace
    Action:
      - activate spring controller
    Transitions:
      hold_mode_requested  -> PositionHolding  (button pressed)
      open_requested       -> OpenGripperForUser
    """

    def __init__(self, node: Node):
        super().__init__(
            outcomes=['hold_mode_requested', 'open_requested'],
            input_keys=['gripper_holding'],
            output_keys=[]
        )
        self._node = node
        # Subscribe to operator button topics
        self._hold_button = False
        self._open_button = False
        self._hold_sub = node.create_subscription(
            Bool, '/operator/hold_button', self._hold_cb, 1
        )
        self._open_sub = node.create_subscription(
            Bool, '/operator/open_gripper_button', self._open_cb, 1
        )

    def _hold_cb(self, msg: Bool):
        if msg.data:
            self._hold_button = True

    def _open_cb(self, msg: Bool):
        if msg.data:
            self._open_button = True

    def execute(self, userdata):
        self._node.get_logger().info('[SpringController] Activating spring controller...')
        # TODO: activate your spring controller here
        self._hold_button = False
        self._open_button = False

        rate = self._node.create_rate(10)
        while rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._hold_button:
                self._node.get_logger().info('[SpringController] Position-hold mode requested.')
                return 'hold_mode_requested'
            if self._open_button:
                self._node.get_logger().info('[SpringController] Open gripper requested.')
                return 'open_requested'


class OpenGripperForUser(smach.State):
    """
    Preconditions:
      - gripper_holding == True
      - open_gripper button pressed
    Action:
      - open gripper when button pressed
    Transitions:
      opened -> Waiting
    """

    def __init__(self, node: Node):
        super().__init__(
            outcomes=['opened'],
            input_keys=['gripper_holding'],
            output_keys=['gripper_holding']
        )
        self._node = node
        self._button_pressed = False
        self._sub = node.create_subscription(
            Bool, '/operator/open_gripper_button', self._button_cb, 1
        )

    def _button_cb(self, msg: Bool):
        if msg.data:
            self._button_pressed = True

    def execute(self, userdata):
        self._node.get_logger().info('[OpenGripperForUser] Waiting for open-gripper button press...')
        self._button_pressed = False

        while rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._button_pressed:
                # TODO: send open-gripper command to hardware
                self._node.get_logger().info('[OpenGripperForUser] Gripper opened.')
                userdata.gripper_holding = False
                return 'opened'


class MoveObjectOnMobileBase(smach.State):
    """
    Recovery: reposition the object using the mobile base so it falls
    within arm workspace for planning.
    Transitions:
      repositioned -> ReadyToPickUpObject
      failed       -> FindObjectLocation
    """

    def __init__(self, node: Node):
        super().__init__(
            outcomes=['repositioned', 'failed'],
            input_keys=['object_pose'],
            output_keys=['object_pose']
        )
        self._node = node

    def execute(self, userdata):
        self._node.get_logger().info('[MoveObjectOnMobileBase] Repositioning mobile base...')
        # TODO: navigate mobile base so object enters arm workspace
        success = True   # replace with real navigation result
        if success:
            self._node.get_logger().info('[MoveObjectOnMobileBase] Object now in workspace.')
            return 'repositioned'
        else:
            self._node.get_logger().warn('[MoveObjectOnMobileBase] Repositioning failed.')
            return 'failed'


class AskUserForHelp(smach.State):
    """
    Recovery: notify the user and wait for manual intervention.
    Transitions:
      helped -> ObjectInGripperMoveToWorkspace
    """

    def __init__(self, node: Node):
        super().__init__(
            outcomes=['helped'],
            input_keys=[],
            output_keys=['fail_count']
        )
        self._node = node
        self._help_received = False
        self._sub = node.create_subscription(
            Bool, '/operator/help_provided', self._help_cb, 1
        )

    def _help_cb(self, msg: Bool):
        if msg.data:
            self._help_received = True

    def execute(self, userdata):
        self._node.get_logger().warn(
            '[AskUserForHelp] Manual intervention required. '
            'Publish True to /operator/help_provided when ready.'
        )
        self._help_received = False
        while rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._help_received:
                userdata.fail_count = 0
                self._node.get_logger().info('[AskUserForHelp] User provided help, resuming.')
                return 'helped'


class Waiting(smach.State):
    """
    Idle/waiting state reached after object is released.
    Waits for operator signal to start a new pick cycle.
    Transitions:
      restart -> FindObjectLocation
    """

    def __init__(self, node: Node):
        super().__init__(
            outcomes=['restart'],
            input_keys=[],
            output_keys=[]
        )
        self._node = node
        self._restart = False
        self._sub = node.create_subscription(
            Bool, '/operator/start_pick', self._restart_cb, 1
        )

    def _restart_cb(self, msg: Bool):
        if msg.data:
            self._restart = True

    def execute(self, userdata):
        self._node.get_logger().info('[Waiting] Idle. Publish True to /operator/start_pick to restart.')
        self._restart = False
        while rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._restart:
                return 'restart'


class PositionHolding(smach.State):
    """
    Holds EE position until further instruction.
    Transitions:
      done -> Waiting
    """

    def __init__(self, node: Node):
        super().__init__(
            outcomes=['done'],
            input_keys=[],
            output_keys=[]
        )
        self._node = node

    def execute(self, userdata):
        self._node.get_logger().info('[PositionHolding] Holding position...')
        # TODO: activate position hold controller
        # Stub: just return done immediately
        return 'done'


# ---------------------------------------------------------------------------
# Build state machine
# ---------------------------------------------------------------------------

def build_sm(node: Node) -> smach.StateMachine:
    sm = smach.StateMachine(outcomes=['shutdown'])
    sm.userdata.object_pose = None
    sm.userdata.ee_plan = None
    sm.userdata.gripper_holding = False
    sm.userdata.fail_count = 0

    with sm:
        smach.StateMachine.add(
            'FIND_OBJECT_LOCATION',
            FindObjectLocation(node),
            transitions={
                'found':     'READY_TO_PICK_UP_OBJECT',
                'not_found': 'FIND_OBJECT_LOCATION',   # keep searching
            }
        )

        smach.StateMachine.add(
            'READY_TO_PICK_UP_OBJECT',
            ReadyToPickUpObject(node),
            transitions={
                'planned':      'MOVING_TO_PICK_UP_OBJECT',
                'cannot_plan':  'MOVE_OBJECT_ON_MOBILE_BASE',
                'no_precond':   'FIND_OBJECT_LOCATION',
            }
        )

        smach.StateMachine.add(
            'MOVING_TO_PICK_UP_OBJECT',
            MovingToPickUpObject(node),
            transitions={
                'at_object':    'PICK_UP_OBJECT',
                'object_moved': 'FIND_OBJECT_LOCATION',
                'ee_failed':    'READY_TO_PICK_UP_OBJECT',
            }
        )

        smach.StateMachine.add(
            'PICK_UP_OBJECT',
            PickUpObject(node),
            transitions={
                'gripped':    'OBJECT_IN_GRIPPER_MOVE_TO_WORKSPACE',
                'grip_failed': 'FIND_OBJECT_LOCATION',
            }
        )

        smach.StateMachine.add(
            'OBJECT_IN_GRIPPER_MOVE_TO_WORKSPACE',
            ObjectInGripperMoveToWorkspace(node),
            transitions={
                'at_workspace': 'SPRING_CONTROLLER',
                'retry':        'OBJECT_IN_GRIPPER_MOVE_TO_WORKSPACE',
                'ask_for_help': 'ASK_USER_FOR_HELP',
            }
        )

        smach.StateMachine.add(
            'SPRING_CONTROLLER',
            SpringController(node),
            transitions={
                'hold_mode_requested': 'POSITION_HOLDING',
                'open_requested':      'OPEN_GRIPPER_FOR_USER',
            }
        )

        smach.StateMachine.add(
            'OPEN_GRIPPER_FOR_USER',
            OpenGripperForUser(node),
            transitions={
                'opened': 'WAITING',
            }
        )

        smach.StateMachine.add(
            'POSITION_HOLDING',
            PositionHolding(node),
            transitions={
                'done': 'WAITING',
            }
        )

        smach.StateMachine.add(
            'WAITING',
            Waiting(node),
            transitions={
                'restart': 'FIND_OBJECT_LOCATION',
            }
        )

        smach.StateMachine.add(
            'MOVE_OBJECT_ON_MOBILE_BASE',
            MoveObjectOnMobileBase(node),
            transitions={
                'repositioned': 'READY_TO_PICK_UP_OBJECT',
                'failed':       'FIND_OBJECT_LOCATION',
            }
        )

        smach.StateMachine.add(
            'ASK_USER_FOR_HELP',
            AskUserForHelp(node),
            transitions={
                'helped': 'OBJECT_IN_GRIPPER_MOVE_TO_WORKSPACE',
            }
        )

    return sm


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = Node('pick_place_sm_node')
    node.get_logger().info('Starting pick-and-place state machine...')

    sm = build_sm(node)

    # Optional: launch introspection server for smach_viewer
    sis = smach_ros.IntrospectionServer('pick_place_sm', sm, '/PICK_PLACE_SM')
    sis.start()

    try:
        outcome = sm.execute()
        node.get_logger().info(f'State machine terminated with outcome: {outcome}')
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        sis.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()