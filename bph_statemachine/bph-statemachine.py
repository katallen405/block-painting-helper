#!/usr/bin/env python3
# Copyright 2026 Kat Allen

import rclpy
from rclpy.node import Node

import smach
import smach_ros

from std_msgs.msg import String


# ---------------------------------------------------------------------------
# State definitions
# ---------------------------------------------------------------------------

class StateIdle(smach.State):
    """Initial idle state. Waits for a trigger before transitioning."""

    def __init__(self, node: Node):
        super().__init__(
            outcomes=['start', 'abort'],
            input_keys=['user_data_in'],
            output_keys=['user_data_out'],
        )
        self._node = node
        self._logger = node.get_logger()

    def execute(self, userdata):
        self._logger.info('Executing state: IDLE')

        # FIXME: replace with real wait for message, actions, services
        userdata.user_data_out = userdata.user_data_in

        self._logger.info('IDLE -> transitioning to RUNNING')
        return 'start'


class StateRunning(smach.State):
    """Active working state."""

    def __init__(self, node: Node):
        super().__init__(
            outcomes=['success', 'failure', 'abort'],
            input_keys=['user_data_in'],
            output_keys=['user_data_out'],
        )
        self._node = node
        self._logger = node.get_logger()

    def execute(self, userdata):
        self._logger.info('Executing state: RUNNING')

        # FIXME: replace with real work (what do we do when the state triggers?)
        success = True  # placeholder

        if success:
            userdata.user_data_out = userdata.user_data_in
            self._logger.info('RUNNING -> transitioning to SUCCESS')
            return 'success'
        else:
            self._logger.warn('RUNNING -> transitioning to FAILURE')
            return 'failure'


class StateSuccess(smach.State):
    """Terminal success state."""

    def __init__(self, node: Node):
        super().__init__(
            outcomes=['done'],
            input_keys=['user_data_in'],
        )
        self._node = node
        self._logger = node.get_logger()

    def execute(self, userdata):
        self._logger.info('Executing state: SUCCESS')
        # TODO: publish results, notify action server, etc.
        return 'done'


class StateFailure(smach.State):
    """Terminal failure / error-handling state."""

    def __init__(self, node: Node):
        super().__init__(
            outcomes=['retry', 'abort'],
            input_keys=['user_data_in'],
        )
        self._node = node
        self._logger = node.get_logger()
        self._retry_count = 0
        self._max_retries = 3

    def execute(self, userdata):
        self._logger.error('Executing state: FAILURE')
        self._retry_count += 1

        if self._retry_count <= self._max_retries:
            self._logger.info(
                f'Retrying ({self._retry_count}/{self._max_retries})'
            )
            return 'retry'

        self._logger.error('Max retries reached, aborting.')
        return 'abort'


# ---------------------------------------------------------------------------
# State machine builder
# ---------------------------------------------------------------------------

def build_state_machine(node: Node) -> smach.StateMachine:
    """Construct and return the top-level state machine."""

    sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted'],
        input_keys=[],
        output_keys=[],
    )

    # Initialise userdata defaults
    sm.userdata.user_data_in = None
    sm.userdata.user_data_out = None

#### temporary states, to be filled in later
    with sm:
        smach.StateMachine.add(
            'IDLE',
            StateIdle(node),
            transitions={
                'start': 'RUNNING',
                'abort': 'aborted',
            },
            remapping={
                'user_data_in': 'user_data_in',
                'user_data_out': 'user_data_out',
            },
        )

        smach.StateMachine.add(
            'RUNNING',
            StateRunning(node),
            transitions={
                'success': 'SUCCESS',
                'failure': 'FAILURE',
                'abort': 'aborted',
            },
            remapping={
                'user_data_in': 'user_data_out',
                'user_data_out': 'user_data_out',
            },
        )

        smach.StateMachine.add(
            'SUCCESS',
            StateSuccess(node),
            transitions={
                'done': 'succeeded',
            },
            remapping={
                'user_data_in': 'user_data_out',
            },
        )

        smach.StateMachine.add(
            'FAILURE',
            StateFailure(node),
            transitions={
                'retry': 'RUNNING',
                'abort': 'aborted',
            },
            remapping={
                'user_data_in': 'user_data_out',
            },
        )

    return sm


# ---------------------------------------------------------------------------
# ROS 2 node wrapper
# ---------------------------------------------------------------------------

class SmachNode(Node):
    """ROS 2 node that hosts a SMACH state machine."""

    def __init__(self):
        super().__init__('smach_node')

        # Optional: declare / read parameters
        self.declare_parameter('loop', False)
        self._loop = self.get_parameter('loop').get_parameter_value().bool_value

        # Optional: publisher for state-machine status
        self._status_pub = self.create_publisher(String, '~/sm_status', 10)

        # Build the state machine
        self._sm = build_state_machine(self)

        # Optional: SMACH introspection server (visualise in smach_viewer)
        self._sis = smach_ros.IntrospectionServer(
            'smach_introspection', self._sm, '/SM_ROOT'
        )
        self._sis.start()

        self.get_logger().info('SmachNode initialised. Starting state machine.')
        self._run_sm()

    def _run_sm(self):
        """Execute the state machine (blocking call on the node thread)."""
        outcome = self._sm.execute()
        self.get_logger().info(f'State machine finished with outcome: {outcome}')

        # Publish final status
        msg = String()
        msg.data = outcome
        self._status_pub.publish(msg)

        if self._loop and outcome == 'succeeded':
            self.get_logger().info('loop=true — restarting state machine.')
            self._run_sm()

    def destroy_node(self):
        self._sis.stop()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = SmachNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()