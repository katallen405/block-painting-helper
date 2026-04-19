
#!/usr/bin/env python3
"""
Mobile Base State Machine
=========================
Manages the mobile base (Turtlebot) independently of the arm SM.

States
------
  Waiting                 — idle, listening for retrieve_object requests
  NavigatingToObject      — Nav2 goal active, monitoring progress
  AwaitingConfirmation    — second retrieve arrived mid-nav; waiting for user confirm
  CancellingPreviousGoal  — confirmed; cancelling the active Nav2 goal
  WaitingForCancelAck     — waiting for Nav2 to acknowledge cancellation
  ObjectInRange           — base has arrived; publishes /object_available

UI topics (inbound)
-------------------
  /ui/retrieve_object            std_msgs/String   — object name to retrieve
  /ui/confirmation_response   std_msgs/Bool     — True=confirm, False=deny

Status topics (outbound)
------------------------
  /base/status                std_msgs/String   — current state label
  /base/confirm_required      std_msgs/String   — "current_object|requested_object"
  /base/object_available      std_msgs/String   — object name when in range

Nav2 action
-----------
  /navigate_to_pose           nav2_msgs/action/NavigateToPose

Object lookup
-------------
  /object_locator             (stub) maps object name → geometry_msgs/PoseStamped
  Replace with your actual perception / map service.

Configuration (set as ROS parameters or edit defaults below)
-------------------------------------------------------------
  confirmation_timeout_sec    float   10.0
  cancel_ack_timeout_sec      float    2.0
  arrival_distance_m          float    0.5
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

import smach
import smach_ros

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ---------------------------------------------------------------------------
# Configuration defaults
# ---------------------------------------------------------------------------

CONFIRMATION_TIMEOUT_SEC = 10.0
CANCEL_ACK_TIMEOUT_SEC   = 2.0
POLL_HZ                  = 20       # spin_once rate for blocking states


# ---------------------------------------------------------------------------
# Shared ROS interface
# Constructed once, injected into every state so they share subscriptions.
# ---------------------------------------------------------------------------

class BaseInterface:
    """
    Single object that owns all ROS pubs/subs for the mobile base SM.
    Passed to every state via __init__ rather than re-created per state.
    """

    def __init__(self, node: Node):
        self.node = node

        # ---- inbound ----
        self._pending_retrieve: str | None = None          # object name
        self._confirmation_response: bool | None = None
        self._retrieve_lock = threading.Lock()
        self._confirm_lock = threading.Lock()

        self.node.create_subscription(
            String, '/ui/retrieve_object',
            self._retrieve_cb, 10
        )
        self.node.create_subscription(
            Bool, '/ui/confirmation_response',
            self._confirm_cb, 10
        )

        # ---- outbound ----
        self._status_pub = node.create_publisher(String, '/base/status', 1)
        self._confirm_pub = node.create_publisher(String, '/base/confirm_required', 1)
        self._available_pub = node.create_publisher(String, '/base/object_available', 1)

        # ---- Nav2 ----
        self.nav_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        node.get_logger().info('Waiting for Nav2 /navigate_to_pose...')
        self.nav_client.wait_for_server()
        node.get_logger().info('Nav2 connected.')

        # Active Nav2 goal handle — written by NavigatingToObject, read by cancel states
        self.active_nav_handle = None
        self._nav_handle_lock = threading.Lock()

    # ---- callbacks ----

    def _retrieve_cb(self, msg: String):
        with self._retrieve_lock:
            self._pending_retrieve = msg.data
        self.node.get_logger().info(f'[BaseInterface] retrieve_object received: {msg.data}')

    def _confirm_cb(self, msg: Bool):
        with self._confirm_lock:
            self._confirmation_response = msg.data
        self.node.get_logger().info(
            f'[BaseInterface] confirmation_response: {"confirm" if msg.data else "deny"}'
        )

    # ---- helpers called by states ----

    def pop_pending_retrieve(self) -> str | None:
        """Consume and return the pending retrieve request, or None."""
        with self._retrieve_lock:
            val = self._pending_retrieve
            self._pending_retrieve = None
            return val

    def pop_confirmation(self) -> bool | None:
        """Consume and return the confirmation response, or None if not yet received."""
        with self._confirm_lock:
            val = self._confirmation_response
            self._confirmation_response = None
            return val

    def clear_pending(self):
        """Discard any queued retrieve or confirmation (used on state entry)."""
        with self._retrieve_lock:
            self._pending_retrieve = None
        with self._confirm_lock:
            self._confirmation_response = None

    def publish_status(self, status: str):
        self._status_pub.publish(String(data=status))

    def publish_confirm_required(self, current: str, requested: str):
        self._confirm_pub.publish(String(data=f'{current}|{requested}'))

    def publish_object_available(self, object_name: str):
        self._available_pub.publish(String(data=object_name))

    def resolve_object_pose(self, object_name: str) -> PoseStamped | None:
        """
        Look up the pose of a named object.
        TODO: replace with a real service call to your perception / map layer.
        Returning None signals that the object could not be located.
        """
        # Stub: return a fixed pose so the SM can be tested without perception
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 0.0
        pose.pose.orientation.w = 1.0
        self.node.get_logger().warn(
            f'[BaseInterface] resolve_object_pose: stub returning fixed pose for "{object_name}"'
        )
        return pose


# ---------------------------------------------------------------------------
# Helper: spin at POLL_HZ, yield True each tick
# ---------------------------------------------------------------------------

def _spin_tick(node: Node):
    """Yield True at POLL_HZ, processing ROS callbacks each time."""
    period = 1.0 / POLL_HZ
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=period)
        yield True


# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------

class Waiting(smach.State):
    """
    Idle state.  Blocks until a retrieve_object message arrives, then resolves
    the object pose and transitions to NavigatingToObject.

    Outcomes
    --------
    retrieve_received   → NavigatingToObject
    object_not_found → Waiting  (re-enter; UI notified via status topic)
    """

    def __init__(self, iface: BaseInterface):
        super().__init__(
            outcomes=['retrieve_received', 'object_not_found'],
            input_keys=[],
            output_keys=['current_object', 'current_pose', 'pending_object', 'pending_pose']
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_status('waiting')
        self._iface.clear_pending()
        self._iface.node.get_logger().info('[Waiting] Idle — waiting for retrieve_object.')

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                return 'object_not_found'

            name = self._iface.pop_pending_retrieve()
            if name is None:
                continue

            self._iface.node.get_logger().info(f'[Waiting] retrieve request: {name}')
            pose = self._iface.resolve_object_pose(name)
            if pose is None:
                self._iface.publish_status(f'object_not_found:{name}')
                self._iface.node.get_logger().warn(f'[Waiting] Could not locate {name}.')
                return 'object_not_found'

            userdata.current_object = name
            userdata.current_pose   = pose
            userdata.pending_object = None
            userdata.pending_pose   = None
            return 'retrieve_received'

        return 'object_not_found'


class NavigatingToObject(smach.State):
    """
    Sends a NavigateToPose goal and monitors it.

    While navigating, listens for a second retrieve_object request.
    If one arrives, transitions to AwaitingConfirmation rather than
    ignoring it or silently rerouting.

    Outcomes
    --------
    arrived          → ObjectInRange
    preempt_requested → AwaitingConfirmation   (second retrieve received)
    nav_failed       → Waiting
    """

    def __init__(self, iface: BaseInterface):
        super().__init__(
            outcomes=['arrived', 'preempt_requested', 'nav_failed'],
            input_keys=['current_object', 'current_pose',
                        'pending_object', 'pending_pose'],
            output_keys=['current_object', 'current_pose',
                         'pending_object', 'pending_pose']
        )
        self._iface = iface
        self._nav_result: int | None = None
        self._result_lock = threading.Lock()

    def _result_cb(self, future):
        status = future.result().status
        with self._result_lock:
            self._nav_result = status
        self._iface.node.get_logger().info(f'[NavigatingToObject] Nav2 result status: {status}')

    def execute(self, userdata):
        self._iface.publish_status(f'navigating_to:{userdata.current_object}')
        self._iface.node.get_logger().info(
            f'[NavigatingToObject] Navigating to {userdata.current_object}...'
        )

        with self._result_lock:
            self._nav_result = None

        # ---- Send Nav2 goal ----
        goal = NavigateToPose.Goal()
        goal.pose = userdata.current_pose

        send_future = self._iface.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._iface.node, send_future)
        handle = send_future.result()

        if not handle.accepted:
            self._iface.node.get_logger().warn('[NavigatingToObject] Nav2 rejected goal.')
            return 'nav_failed'

        # Store handle so cancel states can reach it
        with self._iface._nav_handle_lock:
            self._iface.active_nav_handle = handle

        # Register result callback so we don't have to block on spin_until_future
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

        # ---- Poll loop ----
        for _ in _spin_tick(self._iface.node):

            # Check Nav2 result
            with self._result_lock:
                result = self._nav_result

            if result is not None:
                with self._iface._nav_handle_lock:
                    self._iface.active_nav_handle = None
                if result == GoalStatus.STATUS_SUCCEEDED:
                    self._iface.node.get_logger().info('[NavigatingToObject] Arrived.')
                    return 'arrived'
                else:
                    self._iface.node.get_logger().warn(
                        f'[NavigatingToObject] Nav2 failed with status {result}.'
                    )
                    return 'nav_failed'

            # Check for a second retrieve request
            new_name = self._iface.pop_pending_retrieve()
            if new_name is not None:
                new_pose = self._iface.resolve_object_pose(new_name)
                if new_pose is None:
                    self._iface.publish_status(f'object_not_found:{new_name}')
                    self._iface.node.get_logger().warn(
                        f'[NavigatingToObject] Second retrieve for unknown object {new_name} — ignored.'
                    )
                    continue

                self._iface.node.get_logger().info(
                    f'[NavigatingToObject] Second retrieve received: {new_name} — '
                    f'requesting confirmation.'
                )
                userdata.pending_object = new_name
                userdata.pending_pose   = new_pose
                return 'preempt_requested'

        return 'nav_failed'


class AwaitingConfirmation(smach.State):
    """
    A second retrieve arrived while navigating.  Asks the user whether to
    cancel the current goal and reroute.

    Publishes to /base/confirm_required and waits up to
    CONFIRMATION_TIMEOUT_SEC for a response on /ui/confirmation_response.

    While waiting:
      - A *third* retrieve request is rejected immediately with a status
        message ("confirmation_already_pending").
      - The current Nav2 goal continues uninterrupted.

    Outcomes
    --------
    confirmed  → CancellingPreviousGoal
    denied     → NavigatingToObject      (resume original goal)
    timed_out  → NavigatingToObject      (treat as deny)
    """

    def __init__(self, iface: BaseInterface):
        super().__init__(
            outcomes=['confirmed', 'denied', 'timed_out'],
            input_keys=['current_object', 'current_pose',
                        'pending_object', 'pending_pose'],
            output_keys=['current_object', 'current_pose',
                         'pending_object', 'pending_pose']
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_status('awaiting_confirmation')
        self._iface.publish_confirm_required(
            userdata.current_object,
            userdata.pending_object
        )
        self._iface.node.get_logger().info(
            f'[AwaitingConfirmation] Asking user: cancel '
            f'"{userdata.current_object}" → "{userdata.pending_object}"?'
        )

        deadline = time.monotonic() + CONFIRMATION_TIMEOUT_SEC

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                return 'timed_out'

            # Reject any further retrieve requests that arrive while dialog is open
            extra_retrieve = self._iface.pop_pending_retrieve()
            if extra_retrieve is not None:
                self._iface.publish_status('confirmation_already_pending')
                self._iface.node.get_logger().warn(
                    f'[AwaitingConfirmation] Third retrieve for "{extra_retrieve}" rejected '
                    f'— confirmation already pending.'
                )

            response = self._iface.pop_confirmation()
            if response is True:
                self._iface.node.get_logger().info('[AwaitingConfirmation] User confirmed reroute.')
                return 'confirmed'
            if response is False:
                self._iface.node.get_logger().info('[AwaitingConfirmation] User denied reroute.')
                userdata.pending_object = None
                userdata.pending_pose   = None
                return 'denied'

            if time.monotonic() > deadline:
                self._iface.node.get_logger().warn(
                    '[AwaitingConfirmation] Confirmation timed out — resuming original goal.'
                )
                self._iface.publish_status(
                    f'confirmation_timeout:resuming_{userdata.current_object}'
                )
                userdata.pending_object = None
                userdata.pending_pose   = None
                return 'timed_out'

        return 'timed_out'


class CancellingPreviousGoal(smach.State):
    """
    User confirmed the reroute.  Sends a cancel request to Nav2 and
    immediately transitions to WaitingForCancelAck.

    Outcomes
    --------
    cancel_sent → WaitingForCancelAck
    no_goal     → NavigatingToObject   (handle was already gone; go straight to new goal)
    """

    def __init__(self, iface: BaseInterface):
        super().__init__(
            outcomes=['cancel_sent', 'no_goal'],
            input_keys=['current_object', 'current_pose',
                        'pending_object', 'pending_pose'],
            output_keys=['current_object', 'current_pose',
                         'pending_object', 'pending_pose']
        )
        self._iface = iface

    def execute(self, userdata):
        self._iface.publish_status('cancelling_previous_goal')
        self._iface.node.get_logger().info('[CancellingPreviousGoal] Sending Nav2 cancel...')

        with self._iface._nav_handle_lock:
            handle = self._iface.active_nav_handle

        if handle is None:
            # Nav2 finished on its own just before we got here
            self._iface.node.get_logger().info(
                '[CancellingPreviousGoal] No active handle — skipping cancel.'
            )
            # Promote pending → current
            userdata.current_object = userdata.pending_object
            userdata.current_pose   = userdata.pending_pose
            userdata.pending_object = None
            userdata.pending_pose   = None
            return 'no_goal'

        # Send cancel (non-blocking — ack comes back asynchronously)
        cancel_future = handle.cancel_goal_async()
        # Don't spin_until_future here — we let WaitingForCancelAck handle the ack
        self._iface.node.get_logger().info('[CancellingPreviousGoal] Cancel request sent.')
        return 'cancel_sent'


class WaitingForCancelAck(smach.State):
    """
    Waits for Nav2 to acknowledge the cancellation before sending the new goal.

    If Nav2 doesn't respond within CANCEL_ACK_TIMEOUT_SEC, we proceed anyway —
    Nav2 will resolve the conflict when the new goal arrives.

    On exit (either ack or timeout), promotes pending → current so
    NavigatingToObject sees the new goal.

    Outcomes
    --------
    cancelled   → NavigatingToObject   (clean ack received)
    timeout     → NavigatingToObject   (force proceed)
    """

    def __init__(self, iface: BaseInterface):
        super().__init__(
            outcomes=['cancelled', 'timeout'],
            input_keys=['current_object', 'current_pose',
                        'pending_object', 'pending_pose'],
            output_keys=['current_object', 'current_pose',
                         'pending_object', 'pending_pose']
        )
        self._iface = iface
        self._ack_received = False
        self._ack_lock = threading.Lock()

    def _cancel_ack_cb(self, future):
        # CancelGoal response — any response counts as acknowledgement
        with self._ack_lock:
            self._ack_received = True
        self._iface.node.get_logger().info('[WaitingForCancelAck] Cancel ack received.')

    def execute(self, userdata):
        self._iface.publish_status('waiting_for_cancel_ack')

        with self._ack_lock:
            self._ack_received = False

        # Re-register cancel callback on the handle
        with self._iface._nav_handle_lock:
            handle = self._iface.active_nav_handle

        if handle is not None:
            cancel_future = handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_ack_cb)
        else:
            # Handle disappeared — treat as immediate ack
            with self._ack_lock:
                self._ack_received = True

        deadline = time.monotonic() + CANCEL_ACK_TIMEOUT_SEC
        outcome = 'timeout'

        for _ in _spin_tick(self._iface.node):
            with self._ack_lock:
                acked = self._ack_received

            if acked:
                self._iface.node.get_logger().info(
                    '[WaitingForCancelAck] Nav2 cancelled cleanly.'
                )
                with self._iface._nav_handle_lock:
                    self._iface.active_nav_handle = None
                outcome = 'cancelled'
                break

            if time.monotonic() > deadline:
                self._iface.node.get_logger().warn(
                    '[WaitingForCancelAck] Cancel ack timed out — proceeding anyway.'
                )
                with self._iface._nav_handle_lock:
                    self._iface.active_nav_handle = None
                outcome = 'timeout'
                break

        # Promote pending → current regardless of how we got here
        self._iface.node.get_logger().info(
            f'[WaitingForCancelAck] Promoting pending goal: '
            f'"{userdata.pending_object}" → current.'
        )
        userdata.current_object = userdata.pending_object
        userdata.current_pose   = userdata.pending_pose
        userdata.pending_object = None
        userdata.pending_pose   = None

        return outcome


class ObjectInRange(smach.State):
    """
    Mobile base has arrived at the object location.
    Publishes /base/object_available and waits for the arm SM to
    acknowledge via /base/pickup_complete before returning to Waiting.

    The arm SM should publish True to /base/pickup_complete when it has
    finished grasping (success or failure) so the base knows it can move.

    Outcomes
    --------
    pickup_complete → Waiting
    new_retrieve       → NavigatingToObject   (arm took too long; user sent new retrieve)
    """

    def __init__(self, iface: BaseInterface):
        super().__init__(
            outcomes=['pickup_complete', 'new_retrieve'],
            input_keys=['current_object'],
            output_keys=['current_object', 'current_pose', 'pending_object', 'pending_pose']
        )
        self._iface = iface
        self._pickup_done = False
        self._pickup_lock = threading.Lock()
        self._iface.node.create_subscription(
            Bool, '/base/pickup_complete', self._pickup_cb, 1
        )

    def _pickup_cb(self, msg: Bool):
        if msg.data:
            with self._pickup_lock:
                self._pickup_done = True

    def execute(self, userdata):
        self._iface.publish_status(f'object_in_range:{userdata.current_object}')
        self._iface.publish_object_available(userdata.current_object)
        self._iface.node.get_logger().info(
            f'[ObjectInRange] Arrived at {userdata.current_object}. '
            f'Waiting for arm to complete pickup.'
        )

        with self._pickup_lock:
            self._pickup_done = False

        for _ in _spin_tick(self._iface.node):
            if self.preempt_requested():
                self.service_preempt()
                return 'pickup_complete'

            with self._pickup_lock:
                done = self._pickup_done

            if done:
                self._iface.node.get_logger().info('[ObjectInRange] Pickup complete.')
                userdata.current_object = None
                userdata.current_pose   = None
                userdata.pending_object = None
                userdata.pending_pose   = None
                return 'pickup_complete'

            # Allow a new retrieve to pull the base away if the arm is taking too long
            new_name = self._iface.pop_pending_retrieve()
            if new_name is not None:
                new_pose = self._iface.resolve_object_pose(new_name)
                if new_pose is not None:
                    self._iface.node.get_logger().info(
                        f'[ObjectInRange] New retrieve "{new_name}" received — departing.'
                    )
                    userdata.current_object = new_name
                    userdata.current_pose   = new_pose
                    userdata.pending_object = None
                    userdata.pending_pose   = None
                    return 'new_retrieve'

        return 'pickup_complete'


# ---------------------------------------------------------------------------
# State machine builder
# ---------------------------------------------------------------------------

def build_mobile_base_sm(iface: BaseInterface) -> smach.StateMachine:
    sm = smach.StateMachine(outcomes=['shutdown'])

    # Initialise all userdata keys to safe defaults
    sm.userdata.current_object = None
    sm.userdata.current_pose   = None
    sm.userdata.pending_object = None
    sm.userdata.pending_pose   = None

    with sm:

        smach.StateMachine.add(
            'WAITING',
            Waiting(iface),
            transitions={
                'retrieve_received':   'NAVIGATING_TO_OBJECT',
                'object_not_found': 'WAITING',
            }
        )

        smach.StateMachine.add(
            'NAVIGATING_TO_OBJECT',
            NavigatingToObject(iface),
            transitions={
                'arrived':           'OBJECT_IN_RANGE',
                'preempt_requested': 'AWAITING_CONFIRMATION',
                'nav_failed':        'WAITING',
            }
        )

        smach.StateMachine.add(
            'AWAITING_CONFIRMATION',
            AwaitingConfirmation(iface),
            transitions={
                'confirmed': 'CANCELLING_PREVIOUS_GOAL',
                'denied':    'NAVIGATING_TO_OBJECT',
                'timed_out': 'NAVIGATING_TO_OBJECT',
            }
        )

        smach.StateMachine.add(
            'CANCELLING_PREVIOUS_GOAL',
            CancellingPreviousGoal(iface),
            transitions={
                'cancel_sent': 'WAITING_FOR_CANCEL_ACK',
                'no_goal':     'NAVIGATING_TO_OBJECT',
            }
        )

        smach.StateMachine.add(
            'WAITING_FOR_CANCEL_ACK',
            WaitingForCancelAck(iface),
            transitions={
                'cancelled': 'NAVIGATING_TO_OBJECT',
                'timeout':   'NAVIGATING_TO_OBJECT',
            }
        )

        smach.StateMachine.add(
            'OBJECT_IN_RANGE',
            ObjectInRange(iface),
            transitions={
                'pickup_complete': 'WAITING',
                'new_retrieve':       'NAVIGATING_TO_OBJECT',
            }
        )

    return sm


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = Node('mobile_base_sm_node')
    node.get_logger().info('Starting mobile base state machine...')

    iface = BaseInterface(node)
    sm    = build_mobile_base_sm(iface)

    # Introspection server for smach_viewer
    sis = smach_ros.IntrospectionServer('mobile_base_sm', sm, '/MOBILE_BASE_SM')
    sis.start()

    # MultiThreadedExecutor so Nav2 action callbacks fire while states are
    # blocked in their spin_once loops
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    import threading
    sm_thread = threading.Thread(target=sm.execute, daemon=True)
    sm_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sm.request_preempt()
        sm_thread.join(timeout=5.0)
        sis.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
