#!/usr/bin/env python3
"""
Arm State Machine
=================
Manages the robot arm independently of the mobile base SM.

Top-level arm modes (each is a sub-SM registered as a state):
  MoveToLocation    — uses bph_pickmeup action server
  SpringController  — activates spring controller, handles add/remove spring
  Teleop            — passes control to teleop node
  HoldPosition      — activates position hold controller
  Idle              — waiting for a mode command

Gripper SM runs as a concurrent child of the arm SM, maintaining open/closed
state independently and providing gripper_holding as shared userdata.

UI topics (inbound)
-------------------
  /ui/arm_mode            std_msgs/String   — "movetolocation|teleop|holdposition|springcontroller"
  /ui/target_location     geometry_msgs/PoseStamped  — target for MoveToLocation
  /ui/spring_command      std_msgs/String   — "add|remove|show_model"
  /ui/gripper_command     std_msgs/String   — "open|close"
  /ui/mode_done           std_msgs/Bool     — user signals done (teleop / hold)

Status topics (outbound)
------------------------
  /arm/mode_status        std_msgs/String   — current mode label
  /arm/state_status       std_msgs/String   — current sub-state label
  /arm/gripper_status     std_msgs/String   — "open|closed"
  /arm/error              std_msgs/String   — error messages to UI

Hardware topics
---------------
  /arm/torque_command     std_msgs/Float64MultiArray  — zero-torque flush on mode switch
  /gripper/command        std_msgs/String             — "open|close" to gripper hardware

Configuration
-------------
  ZERO_TORQUE_FLUSH_SEC   float   0.1   — how long to publish zero torques before switching
  NUM_JOINTS              int     6     — joints to include in torque flush
  POLL_HZ                 int     20
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import smach
import smach_ros

from std_msgs.msg import String, Bool, Float64MultiArray
from geometry_msgs.msg import PoseStamped

from bph_interfaces.action import BphPickmeup  # your generated action
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

ZERO_TORQUE_FLUSH_SEC = 0.1
NUM_JOINTS            = 6
POLL_HZ               = 20

# ---------------------------------------------------------------------------
# Arm interface — owns all ROS I/O for the arm SM
# ---------------------------------------------------------------------------

class ArmInterface:
    """
    Single object owning all ROS publishers, subscribers, and action clients
    for the arm SM.  Injected into every state.
    """

    def __init__(self, node: Node):
        self.node = node

        # ---- inbound ----
        self._pending_mode: str | None         = None
        self._pending_location: PoseStamped | None = None
        self._pending_spring_cmd: str | None   = None
        self._pending_gripper_cmd: str | None  = None
        self._mode_done: bool                  = False

        self._lock = threading.Lock()

        node.create_subscription(String,      '/ui/arm_mode',        self._mode_cb,       10)
        node.create_subscription(PoseStamped, '/ui/target_location', self._location_cb,   10)
        node.create_subscription(String,      '/ui/spring_command',  self._spring_cb,     10)
        node.create_subscription(String,      '/ui/gripper_command', self._gripper_cmd_cb,10)
        node.create_subscription(Bool,        '/ui/mode_done',       self._mode_done_cb,  10)

        # ---- outbound ----
        self._mode_pub    = node.create_publisher(String,              '/arm/mode_status',  1)
        self._state_pub   = node.create_publisher(String,              '/arm/state_status', 1)
        self._gripper_pub = node.create_publisher(String,              '/arm/gripper_status',1)
        self._error_pub   = node.create_publisher(String,              '/arm/error',        1)
        self._torque_pub  = node.create_publisher(Float64MultiArray,   '/arm/torque_command',1)
        self._hw_grip_pub = node.create_publisher(String,              '/gripper/command',  1)

        # ---- bph_pickmeup action client ----
        self.pickmeup_client = ActionClient(node, BphPickmeup, '/bph_pickmeup')
        node.get_logger().info('Waiting for /bph_pickmeup...')
        if not self.pickmeup_client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error('Failed to connect to /bph_pickmeup action server.')
            node.get_logger().error('bph_pickmeup action server not available')
        node.get_logger().info('/bph_pickmeup ready.')

        # Active pickmeup goal handle — written by MoveToLocation states,
        # read by the preemption logic to cancel mid-motion
        self.active_pickmeup_handle = None
        self._pickmeup_lock = threading.Lock()

    # ---- callbacks ----

    def _mode_cb(self, msg: String):
        with self._lock:
            self._pending_mode = msg.data.lower().strip()
        self.node.get_logger().info(f'[ArmInterface] mode request: {msg.data}')

    def _location_cb(self, msg: PoseStamped):
        with self._lock:
            self._pending_location = msg
        self.node.get_logger().info('[ArmInterface] target_location received.')

    def _spring_cb(self, msg: String):
        with self._lock:
            self._pending_spring_cmd = msg.data.lower().strip()
        self.node.get_logger().info(f'[ArmInterface] spring_command: {msg.data}')

    def _gripper_cmd_cb(self, msg: String):
        with self._lock:
            self._pending_gripper_cmd = msg.data.lower().strip()

    def _mode_done_cb(self, msg: Bool):
        if msg.data:
            with self._lock:
                self._mode_done = True
            self.node.get_logger().info('[ArmInterface] mode_done received.')

    # ---- consume helpers ----

    def pop_mode_request(self) -> str | None:
        with self._lock:
            v = self._pending_mode
            self._pending_mode = None
            return v

    def pop_location(self) -> PoseStamped | None:
        with self._lock:
            v = self._pending_location
            self._pending_location = None
            return v

    def pop_spring_command(self) -> str | None:
        with self._lock:
            v = self._pending_spring_cmd
            self._pending_spring_cmd = None
            return v

    def pop_gripper_command(self) -> str | None:
        with self._lock:
            v = self._pending_gripper_cmd
            self._pending_gripper_cmd = None
            return v

    def pop_mode_done(self) -> bool:
        with self._lock:
            v = self._mode_done
            self._mode_done = False
            return v

    def clear_all_pending(self):
        with self._lock:
            self._pending_mode       = None
            self._pending_location   = None
            self._pending_spring_cmd = None
            self._pending_gripper_cmd= None
            self._mode_done          = False

    # ---- publish helpers ----

    def publish_mode(self, mode: str):
        self._mode_pub.publish(String(data=mode))

    def publish_state(self, state: str):
        self._state_pub.publish(String(data=state))

    def publish_error(self, msg: str):
        self._error_pub.publish(String(data=msg))
        self.node.get_logger().warn(f'[ArmInterface] error: {msg}')

    def publish_gripper_status(self, status: str):
        self._gripper_pub.publish(String(data=status))

    def send_gripper_command(self, cmd: str):
        """Send open/close to gripper hardware."""
        self._hw_grip_pub.publish(String(data=cmd))

    # ---- zero-torque flush ----

    def flush_zero_torques(self):
        """
        Publish a zero-torque command for ZERO_TORQUE_FLUSH_SEC before
        any mode switch.  Ensures the spring controller (or any torque-mode
        controller) leaves the joints in a safe state.

        This is a blocking call — it returns only after the flush duration.
        """
        self.node.get_logger().info('[ArmInterface] Flushing zero torques...')
        msg = Float64MultiArray()
        msg.data = [0.0] * NUM_JOINTS
        deadline = time.monotonic() + ZERO_TORQUE_FLUSH_SEC
        period = 1.0 / POLL_HZ
        while time.monotonic() < deadline:
            self._torque_pub.publish(msg)
            time.sleep(period)
        self.node.get_logger().info('[ArmInterface] Zero-torque flush complete.')


# ---------------------------------------------------------------------------
# Shared spin helper
# ---------------------------------------------------------------------------

def _spin_tick(node: Node):
    period = 1.0 / POLL_HZ
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=period)
        yield True


# ---------------------------------------------------------------------------
# Gripper SM
# Runs as a concurrent child — maintains open/closed state, writes
# userdata.gripper_holding for arm states to read as a precondition.
# ---------------------------------------------------------------------------

class GripperOpen(smach.State):
    def __init__(self, iface: ArmInterface):
        super().__init__(
            outcomes=['close_requested', 'preempted'],
            input_keys=['gripper_holding'],
            output_keys=['gripper_holding']
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_gripper_status('open')
        self._iface.send_gripper_command('open')
        userdata.gripper_holding = False

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            cmd = self._iface.pop_gripper_command()
            if cmd == 'close':
                return 'close_requested'


class GripperClosed(smach.State):
    def __init__(self, iface: ArmInterface):
        super().__init__(
            outcomes=['open_requested', 'preempted'],
            input_keys=['gripper_holding'],
            output_keys=['gripper_holding']
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_gripper_status('closed')
        self._iface.send_gripper_command('close')
        userdata.gripper_holding = True

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            cmd = self._iface.pop_gripper_command()
            if cmd == 'open':
                return 'open_requested'


def build_gripper_sm(iface: ArmInterface) -> smach.StateMachine:
    sm = smach.StateMachine(
        outcomes=['shutdown'],
        input_keys=['gripper_holding'],
        output_keys=['gripper_holding']
    )
    with sm:
        smach.StateMachine.add(
            'GRIPPER_OPEN',
            GripperOpen(iface),
            transitions={
                'close_requested': 'GRIPPER_CLOSED',
                'preempted':       'shutdown',
            }
        )
        smach.StateMachine.add(
            'GRIPPER_CLOSED',
            GripperClosed(iface),
            transitions={
                'open_requested': 'GRIPPER_OPEN',
                'preempted':      'shutdown',
            }
        )
    return sm


# ---------------------------------------------------------------------------
# MoveToLocation sub-states
# ---------------------------------------------------------------------------

class MTL_WaitingForLocation(smach.State):
    """
    Waits for a target PoseStamped from the UI.
    Mode switch preempts immediately.
    """
    def __init__(self, iface: ArmInterface):
        super().__init__(
            outcomes=['location_received', 'mode_switch', 'preempted'],
            input_keys=[],
            output_keys=['target_location']
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_state('movetolocation:waiting_for_location')
        self._iface.node.get_logger().info('[MTL_WaitingForLocation] Waiting for target...')

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            mode = self._iface.pop_mode_request()
            if mode is not None:
                return 'mode_switch'

            loc = self._iface.pop_location()
            if loc is not None:
                userdata.target_location = loc
                return 'location_received'


class MTL_Planning(smach.State):
    """
    Sends goal to bph_pickmeup with plan_only semantics (planning_complete
    feedback triggers transition, as per the feedback-driven pattern we
    established earlier).
    Mode switch cancels the goal and transitions out immediately.
    """
    def __init__(self, iface: ArmInterface):
        super().__init__(
            outcomes=['planned', 'cannot_plan', 'mode_switch', 'preempted'],
            input_keys=['target_location'],
            output_keys=['active_goal_handle']
        )
        self._iface       = iface
        self._feedback_state = ''
        self._feedback_lock  = threading.Lock()

    def _on_feedback(self, feedback_msg):
        with self._feedback_lock:
            self._feedback_state = feedback_msg.feedback.current_state

    def execute(self, userdata):
        self._iface.publish_state('movetolocation:planning')
        self._iface.node.get_logger().info('[MTL_Planning] Sending goal to bph_pickmeup...')

        with self._feedback_lock:
            self._feedback_state = ''

        goal = BphPickmeup.Goal()
        # TODO: convert PoseStamped to joint angles via IK, or pass pose
        # directly if your bph_pickmeup action supports Cartesian goals.
        # For now we use position_name as a placeholder.
        goal.position_name = 'target'

        send_future = self._iface.pickmeup_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        rclpy.spin_until_future_complete(self._iface.node, send_future)
        handle = send_future.result()

        if not handle.accepted:
            self._iface.publish_error('bph_pickmeup rejected goal')
            return 'cannot_plan'

        with self._iface._pickmeup_lock:
            self._iface.active_pickmeup_handle = handle
        userdata.active_goal_handle = handle

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                handle.cancel_goal_async()
                with self._iface._pickmeup_lock:
                    self._iface.active_pickmeup_handle = None
                return 'preempted'

            mode = self._iface.pop_mode_request()
            if mode is not None:
                self._iface.node.get_logger().info(
                    '[MTL_Planning] Mode switch — cancelling plan.'
                )
                handle.cancel_goal_async()
                with self._iface._pickmeup_lock:
                    self._iface.active_pickmeup_handle = None
                return 'mode_switch'

            with self._feedback_lock:
                fb = self._feedback_state

            if fb == 'planning_complete':
                return 'planned'
            if fb == 'planning_failed':
                self._iface.publish_error('Planning failed')
                return 'cannot_plan'


class MTL_Moving(smach.State):
    """
    Arm is executing the motion plan.
    Mode switch: cancel goal immediately, arm halts in place via
    zero-torque flush, then transitions out.
    """
    def __init__(self, iface: ArmInterface):
        super().__init__(
            outcomes=['arrived', 'object_moved', 'motion_failed',
                      'mode_switch', 'preempted'],
            input_keys=['active_goal_handle'],
            output_keys=['active_goal_handle']
        )
        self._iface      = iface
        self._nav_result = None
        self._result_lock = threading.Lock()

    def _result_cb(self, future):
        result = future.result().result
        with self._result_lock:
            self._nav_result = result

    def execute(self, userdata):
        self._iface.publish_state('movetolocation:moving')
        self._iface.node.get_logger().info('[MTL_Moving] Arm moving to target...')

        with self._result_lock:
            self._nav_result = None

        handle = userdata.active_goal_handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                self._iface.node.get_logger().info(
                    '[MTL_Moving] Preempted — cancelling motion and flushing torques.'
                )
                handle.cancel_goal_async()
                self._iface.flush_zero_torques()
                with self._iface._pickmeup_lock:
                    self._iface.active_pickmeup_handle = None
                return 'preempted'

            mode = self._iface.pop_mode_request()
            if mode is not None:
                self._iface.node.get_logger().info(
                    f'[MTL_Moving] Mode switch to "{mode}" — '
                    f'halting immediately and flushing torques.'
                )
                handle.cancel_goal_async()
                self._iface.flush_zero_torques()
                with self._iface._pickmeup_lock:
                    self._iface.active_pickmeup_handle = None
                userdata.active_goal_handle = None
                # Store requested mode so the arm SM top level can act on it
                self._iface._pending_mode = mode
                return 'mode_switch'

            with self._result_lock:
                result = self._nav_result

            if result is not None:
                with self._iface._pickmeup_lock:
                    self._iface.active_pickmeup_handle = None
                userdata.active_goal_handle = None
                if result.success:
                    return 'arrived'
                # Distinguish object-moved from generic failure via error code
                # if your bph_pickmeup action populates a specific code for it.
                # For now treat all failures as motion_failed.
                self._iface.publish_error(
                    f'Motion failed: {result.message}'
                )
                return 'motion_failed'


class MTL_Grasping(smach.State):
    """
    EE is at the gripping location.  Commands the gripper to close.

    NON-PREEMPTIBLE — mode switches are queued, not acted on, until
    grasping completes.  This prevents gripper state ambiguity.
    A queued mode switch is stored back into pending_mode so the arm
    SM top level sees it on the next cycle.
    """
    def __init__(self, iface: ArmInterface):
        super().__init__(
            outcomes=['gripped', 'failed_to_grasp'],
            input_keys=['gripper_holding'],
            output_keys=['gripper_holding', 'queued_mode']
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_state('movetolocation:grasping')
        self._iface.node.get_logger().info(
            '[MTL_Grasping] Closing gripper — mode switches blocked until complete.'
        )
        userdata.queued_mode = None

        # Command gripper via the gripper SM's topic
        self._iface.pop_gripper_command()          # discard any stale command
        self._iface.send_gripper_command('close')

        # Give gripper time to close and confirm
        # TODO: replace with a real gripper feedback check
        deadline = time.monotonic() + 2.0
        for _ in _spin_tick(self._iface.node):
            # Queue (but do not act on) any mode switch that arrives
            mode = self._iface.pop_mode_request()
            if mode is not None:
                self._iface.node.get_logger().info(
                    f'[MTL_Grasping] Mode switch "{mode}" queued — '
                    f'will be applied after grasp completes.'
                )
                userdata.queued_mode = mode

            if time.monotonic() > deadline:
                break

        # TODO: read gripper force / position feedback to determine success
        grip_success = True   # replace with real check

        if grip_success:
            userdata.gripper_holding = True
            self._iface.node.get_logger().info('[MTL_Grasping] Grip successful.')
            return 'gripped'
        else:
            userdata.gripper_holding = False
            self._iface.publish_error('Failed to grasp object')
            return 'failed_to_grasp'


class MTL_HoldingObject(smach.State):
    """
    Object is in the gripper.  Waits for an explicit command to proceed
    (e.g. move to workspace, release) or a mode switch.

    NON-PREEMPTIBLE until the user explicitly requests a mode change
    through the confirm path — dropping a held object unexpectedly is
    as dangerous as interrupting a grasp.

    For now: waits for mode_done (user signals ready) or a mode switch,
    at which point it transitions out with the gripper still closed.
    The receiving state is responsible for deciding what to do with
    the held object.
    """
    def __init__(self, iface: ArmInterface):
        super().__init__(
            outcomes=['release_requested', 'mode_switch', 'preempted'],
            input_keys=['gripper_holding', 'queued_mode'],
            output_keys=['gripper_holding']
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_state('movetolocation:holding_object')
        self._iface.node.get_logger().info('[MTL_HoldingObject] Holding object.')

        # Apply any mode switch that was queued during grasping
        if getattr(userdata, 'queued_mode', None):
            self._iface.node.get_logger().info(
                f'[MTL_HoldingObject] Applying queued mode switch: '
                f'"{userdata.queued_mode}"'
            )
            # Re-inject so the arm SM top level picks it up
            with self._iface._lock:
                self._iface._pending_mode = userdata.queued_mode
            return 'mode_switch'

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            if self._iface.pop_mode_done():
                return 'release_requested'

            mode = self._iface.pop_mode_request()
            if mode is not None:
                # Re-inject for the arm SM top level
                with self._iface._lock:
                    self._iface._pending_mode = mode
                return 'mode_switch'


class MTL_FailedToGrasp(smach.State):
    """
    Grasp failed.  Notifies UI and waits for user instruction:
      - retry   → back to MTL_WaitingForLocation
      - mode_switch → exits MoveToLocation entirely
    """
    def __init__(self, iface: ArmInterface):
        super().__init__(
            outcomes=['retry', 'mode_switch', 'preempted'],
            input_keys=[],
            output_keys=[]
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_state('movetolocation:failed_to_grasp')
        self._iface.publish_error('Grasp failed — waiting for user instruction.')
        self._iface.node.get_logger().warn('[MTL_FailedToGrasp] Awaiting retry or mode switch.')

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            mode = self._iface.pop_mode_request()
            if mode is not None:
                return 'mode_switch'

            if self._iface.pop_mode_done():
                return 'retry'


def build_movetolocation_sm(iface: ArmInterface) -> smach.StateMachine:
    sm = smach.StateMachine(
        outcomes=['mode_switch', 'shutdown'],
        input_keys=['gripper_holding', 'target_location'],
        output_keys=['gripper_holding']
    )
    sm.userdata.active_goal_handle = None
    sm.userdata.queued_mode        = None

    with sm:
        smach.StateMachine.add(
            'WAITING_FOR_LOCATION',
            MTL_WaitingForLocation(iface),
            transitions={
                'location_received': 'PLANNING',
                'mode_switch':       'mode_switch',
                'preempted':         'shutdown',
            }
        )
        smach.StateMachine.add(
            'PLANNING',
            MTL_Planning(iface),
            transitions={
                'planned':      'MOVING',
                'cannot_plan':  'WAITING_FOR_LOCATION',
                'mode_switch':  'mode_switch',
                'preempted':    'shutdown',
            }
        )
        smach.StateMachine.add(
            'MOVING',
            MTL_Moving(iface),
            transitions={
                'arrived':      'GRASPING',
                'object_moved': 'WAITING_FOR_LOCATION',
                'motion_failed':'WAITING_FOR_LOCATION',
                'mode_switch':  'mode_switch',
                'preempted':    'shutdown',
            }
        )
        smach.StateMachine.add(
            'GRASPING',
            MTL_Grasping(iface),
            transitions={
                'gripped':        'HOLDING_OBJECT',
                'failed_to_grasp':'FAILED_TO_GRASP',
            }
        )
        smach.StateMachine.add(
            'HOLDING_OBJECT',
            MTL_HoldingObject(iface),
            transitions={
                'release_requested': 'WAITING_FOR_LOCATION',
                'mode_switch':       'mode_switch',
                'preempted':         'shutdown',
            }
        )
        smach.StateMachine.add(
            'FAILED_TO_GRASP',
            MTL_FailedToGrasp(iface),
            transitions={
                'retry':       'WAITING_FOR_LOCATION',
                'mode_switch': 'mode_switch',
                'preempted':   'shutdown',
            }
        )
    return sm


# ---------------------------------------------------------------------------
# SpringController sub-states
# ---------------------------------------------------------------------------

class SC_Running(smach.State):
    """
    Spring controller is active.  Listens for:
      - add_spring / remove_spring commands from UI
      - show_model command from UI
      - mode switch → flush zero torques then exit
    """
    def __init__(self, iface: ArmInterface):
        super().__init__(
            outcomes=['add_spring', 'remove_spring', 'mode_switch', 'preempted'],
            input_keys=[],
            output_keys=[]
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_state('springcontroller:running')
        self._iface.node.get_logger().info('[SC_Running] Spring controller active.')
        # TODO: activate spring controller via ROS2 control / translation node

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                self._iface.flush_zero_torques()
                return 'preempted'

            mode = self._iface.pop_mode_request()
            if mode is not None:
                self._iface.node.get_logger().info(
                    f'[SC_Running] Mode switch to "{mode}" — '
                    f'flushing zero torques before exit.'
                )
                self._iface.flush_zero_torques()
                # Re-inject so arm SM top level sees it
                with self._iface._lock:
                    self._iface._pending_mode = mode
                return 'mode_switch'

            cmd = self._iface.pop_spring_command()
            if cmd == 'add':
                return 'add_spring'
            if cmd == 'remove':
                return 'remove_spring'
            if cmd == 'show_model':
                # Show model is handled inline — publish a status and continue
                self._iface.publish_state('springcontroller:showing_model')
                self._iface.node.get_logger().info(
                    '[SC_Running] Show spring model requested — TODO: publish model.'
                )
                # TODO: trigger your spring model visualisation here
                self._iface.publish_state('springcontroller:running')


class SC_AddSpring(smach.State):
    """
    Handles an add-spring request.  Waits for the user to confirm the
    spring parameters, then returns to Running.
    """
    def __init__(self, iface: ArmInterface):
        super().__init__(
            outcomes=['done', 'mode_switch', 'preempted'],
            input_keys=[],
            output_keys=[]
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_state('springcontroller:add_spring')
        self._iface.node.get_logger().info('[SC_AddSpring] Adding spring — awaiting parameters.')
        # TODO: call your spring parameter service here

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                self._iface.flush_zero_torques()
                return 'preempted'

            mode = self._iface.pop_mode_request()
            if mode is not None:
                self._iface.flush_zero_torques()
                with self._iface._lock:
                    self._iface._pending_mode = mode
                return 'mode_switch'

            if self._iface.pop_mode_done():
                self._iface.node.get_logger().info('[SC_AddSpring] Spring added.')
                return 'done'


class SC_RemoveSpring(smach.State):
    """
    Handles a remove-spring request.
    """
    def __init__(self, iface: ArmInterface):
        super().__init__(
            outcomes=['done', 'mode_switch', 'preempted'],
            input_keys=[],
            output_keys=[]
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_state('springcontroller:remove_spring')
        self._iface.node.get_logger().info('[SC_RemoveSpring] Removing spring — awaiting confirmation.')
        # TODO: call your spring removal service here

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                self._iface.flush_zero_torques()
                return 'preempted'

            mode = self._iface.pop_mode_request()
            if mode is not None:
                self._iface.flush_zero_torques()
                with self._iface._lock:
                    self._iface._pending_mode = mode
                return 'mode_switch'

            if self._iface.pop_mode_done():
                self._iface.node.get_logger().info('[SC_RemoveSpring] Spring removed.')
                return 'done'


def build_springcontroller_sm(iface: ArmInterface) -> smach.StateMachine:
    sm = smach.StateMachine(
        outcomes=['mode_switch', 'shutdown'],
        input_keys=[],
        output_keys=[]
    )
    with sm:
        smach.StateMachine.add(
            'RUNNING',
            SC_Running(iface),
            transitions={
                'add_spring':    'ADD_SPRING',
                'remove_spring': 'REMOVE_SPRING',
                'mode_switch':   'mode_switch',
                'preempted':     'shutdown',
            }
        )
        smach.StateMachine.add(
            'ADD_SPRING',
            SC_AddSpring(iface),
            transitions={
                'done':        'RUNNING',
                'mode_switch': 'mode_switch',
                'preempted':   'shutdown',
            }
        )
        smach.StateMachine.add(
            'REMOVE_SPRING',
            SC_RemoveSpring(iface),
            transitions={
                'done':        'RUNNING',
                'mode_switch': 'mode_switch',
                'preempted':   'shutdown',
            }
        )
    return sm


# ---------------------------------------------------------------------------
# Teleop
# ---------------------------------------------------------------------------

class Teleop(smach.State):
    """
    Passes arm control to teleop node.
    Exits on mode_done button or a mode switch from UI.
    No zero-torque flush needed — teleop is already velocity/position mode.
    """
    def __init__(self, iface: ArmInterface):
        super().__init__(
            outcomes=['done', 'mode_switch', 'preempted'],
            input_keys=[],
            output_keys=[]
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_mode('teleop')
        self._iface.publish_state('teleop:running')
        self._iface.node.get_logger().info('[Teleop] Teleop active.')
        # TODO: activate teleop controller via ROS2 control

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            if self._iface.pop_mode_done():
                self._iface.node.get_logger().info('[Teleop] Done.')
                return 'done'

            mode = self._iface.pop_mode_request()
            if mode is not None:
                with self._iface._lock:
                    self._iface._pending_mode = mode
                return 'mode_switch'


# ---------------------------------------------------------------------------
# HoldPosition
# ---------------------------------------------------------------------------

class HoldPosition(smach.State):
    """
    Activates position-hold controller.
    Exits on mode_done or mode switch.
    Zero-torque flush not required — position mode is safe to switch from.
    """
    def __init__(self, iface: ArmInterface):
        super().__init__(
            outcomes=['done', 'mode_switch', 'preempted'],
            input_keys=[],
            output_keys=[]
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_mode('holdposition')
        self._iface.publish_state('holdposition:running')
        self._iface.node.get_logger().info('[HoldPosition] Holding position.')
        # TODO: activate position-hold controller via ROS2 control

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            if self._iface.pop_mode_done():
                self._iface.node.get_logger().info('[HoldPosition] Done.')
                return 'done'

            mode = self._iface.pop_mode_request()
            if mode is not None:
                with self._iface._lock:
                    self._iface._pending_mode = mode
                return 'mode_switch'


# ---------------------------------------------------------------------------
# Idle
# ---------------------------------------------------------------------------

class Idle(smach.State):
    """
    Default state — arm is not under active control.
    Waits for a mode switch from the UI.
    """
    def __init__(self, iface: ArmInterface):
        super().__init__(
            outcomes=['mode_switch', 'preempted'],
            input_keys=[],
            output_keys=[]
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_mode('idle')
        self._iface.publish_state('idle')
        self._iface.node.get_logger().info('[Idle] Waiting for mode command.')

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            mode = self._iface.pop_mode_request()
            if mode is not None:
                with self._iface._lock:
                    self._iface._pending_mode = mode
                return 'mode_switch'


# ---------------------------------------------------------------------------
# ModeRouter
# Reads the pending_mode from iface and transitions to the correct sub-SM.
# This is the hub that all mode_switch outcomes flow back through.
# ---------------------------------------------------------------------------

class ModeRouter(smach.State):
    """
    Consumes the pending mode request and returns an outcome that maps
    to the correct sub-SM.  Sitting between every mode transition means
    there is exactly one place in the SM where mode strings are interpreted.
    """
    VALID_MODES = {'movetolocation', 'springcontroller', 'teleop', 'holdposition', 'idle'}

    def __init__(self, iface: ArmInterface):
        super().__init__(
            outcomes=['to_movetolocation', 'to_springcontroller',
                      'to_teleop', 'to_holdposition', 'to_idle', 'preempted'],
            input_keys=[],
            output_keys=[]
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_state('routing')

        # Spin briefly to let any in-flight messages settle
        rclpy.spin_once(self._iface.node, timeout_sec=0.05)
        mode = self._iface.pop_mode_request()

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if mode is None:
            self._iface.node.get_logger().warn(
                '[ModeRouter] No pending mode — defaulting to idle.'
            )
            return 'to_idle'

        if mode not in self.VALID_MODES:
            self._iface.publish_error(f'Unknown mode: "{mode}" — staying idle.')
            self._iface.node.get_logger().warn(f'[ModeRouter] Unknown mode: {mode}')
            return 'to_idle'

        self._iface.node.get_logger().info(f'[ModeRouter] Routing to: {mode}')
        self._iface.publish_mode(mode)
        return f'to_{mode}'


# ---------------------------------------------------------------------------
# Top-level arm SM builder
# ---------------------------------------------------------------------------

def build_arm_sm(iface: ArmInterface) -> smach.StateMachine:
    sm = smach.StateMachine(outcomes=['shutdown'])
    sm.userdata.gripper_holding  = False
    sm.userdata.target_location  = None

    with sm:

        smach.StateMachine.add(
            'IDLE',
            Idle(iface),
            transitions={
                'mode_switch': 'MODE_ROUTER',
                'preempted':   'shutdown',
            }
        )

        smach.StateMachine.add(
            'MODE_ROUTER',
            ModeRouter(iface),
            transitions={
                'to_movetolocation':   'MOVE_TO_LOCATION',
                'to_springcontroller': 'SPRING_CONTROLLER',
                'to_teleop':           'TELEOP',
                'to_holdposition':     'HOLD_POSITION',
                'to_idle':             'IDLE',
                'preempted':           'shutdown',
            }
        )

        smach.StateMachine.add(
            'MOVE_TO_LOCATION',
            build_movetolocation_sm(iface),
            transitions={
                'mode_switch': 'MODE_ROUTER',
                'shutdown':    'shutdown',
            },
            remapping={
                'gripper_holding': 'gripper_holding',
                'target_location': 'target_location',
            }
        )

        smach.StateMachine.add(
            'SPRING_CONTROLLER',
            build_springcontroller_sm(iface),
            transitions={
                'mode_switch': 'MODE_ROUTER',
                'shutdown':    'shutdown',
            }
        )

        smach.StateMachine.add(
            'TELEOP',
            Teleop(iface),
            transitions={
                'done':        'IDLE',
                'mode_switch': 'MODE_ROUTER',
                'preempted':   'shutdown',
            }
        )

        smach.StateMachine.add(
            'HOLD_POSITION',
            HoldPosition(iface),
            transitions={
                'done':        'IDLE',
                'mode_switch': 'MODE_ROUTER',
                'preempted':   'shutdown',
            }
        )

    return sm


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = Node('arm_sm_node')
    node.get_logger().info('Starting arm state machine...')

    iface = ArmInterface(node)

    # Build arm SM and gripper SM separately — gripper runs on its own thread
    arm_sm     = build_arm_sm(iface)
    gripper_sm = build_gripper_sm(iface)

    # Introspection servers for smach_viewer
    arm_sis = smach_ros.IntrospectionServer('arm_sm',     arm_sm,     '/ARM_SM')
    grp_sis = smach_ros.IntrospectionServer('gripper_sm', gripper_sm, '/GRIPPER_SM')
    arm_sis.start()
    grp_sis.start()

    # MultiThreadedExecutor so action callbacks fire while states are polling
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    arm_thread = threading.Thread(target=arm_sm.execute,     daemon=True)
    grp_thread = threading.Thread(target=gripper_sm.execute, daemon=True)
    arm_thread.start()
    grp_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        arm_sm.request_preempt()
        gripper_sm.request_preempt()
        arm_thread.join(timeout=5.0)
        grp_thread.join(timeout=5.0)
        arm_sis.stop()
        grp_sis.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
