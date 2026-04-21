#!/usr/bin/env python3
"""
Smoke test for mobile base SM preemption flow.
Run with: python3 test_base_sm.py
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class BaseSmTester(Node):
    def __init__(self):
        super().__init__('base_sm_tester')
        self._fetch_pub  = self.create_publisher(String, '/ui/fetch_object', 1)
        self._confirm_pub = self.create_publisher(String, '/ui/confirmation_response', 1)
        self._status_sub  = self.create_subscription(
            String, '/base/status', self._status_cb, 10
        )
        self._confirm_sub = self.create_subscription(
            String, '/base/confirm_required', self._confirm_required_cb, 10
        )

    def _status_cb(self, msg):
        self.get_logger().info(f'[STATUS] {msg.data}')

    def _confirm_required_cb(self, msg):
        self.get_logger().info(f'[CONFIRM REQUIRED] {msg.data}')

    def pub_fetch(self, name):
        self.get_logger().info(f'>>> fetch_object: {name}')
        self._fetch_pub.publish(String(data=name))

    def pub_confirm(self, confirm: bool):
        self.get_logger().info(f'>>> confirmation_response: {confirm}')
        self._confirm_pub.publish(String(data='true' if confirm else 'false'))


def main():
    rclpy.init()
    tester = BaseSmTester()

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(tester, timeout_sec=0.05)

    # Test 1: normal fetch
    print('\n=== Test 1: normal fetch ===')
    tester.pub_fetch('block_A')
    spin_for(2.0)

    # Test 2: second fetch while navigating → confirm reroute
    print('\n=== Test 2: preemption with confirm ===')
    tester.pub_fetch('block_B')
    spin_for(1.0)
    tester.pub_confirm(True)
    spin_for(2.0)

    # Test 3: second fetch while navigating → deny reroute
    print('\n=== Test 3: preemption with deny ===')
    tester.pub_fetch('block_C')
    spin_for(1.0)
    tester.pub_confirm(False)
    spin_for(2.0)

    # Test 4: confirmation timeout
    print('\n=== Test 4: confirmation timeout (wait 12s) ===')
    tester.pub_fetch('block_D')
    spin_for(1.0)
    tester.pub_fetch('block_E')
    spin_for(12.0)   # longer than CONFIRMATION_TIMEOUT_SEC

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
