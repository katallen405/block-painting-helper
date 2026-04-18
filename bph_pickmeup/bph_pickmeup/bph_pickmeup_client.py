#!/usr/bin/env python3
"""
bph_pickmeup client helper — drop this into your SMACH state machine.

Replaces the stub send_to_bph_pickmeup() in pick_place_sm.py.

Usage in ReadyToPickUpObject.execute():
    from bph_pickmeup_client import BphPickmeupClient

    # create once in __init__, pass the same node
    self._bph_client = BphPickmeupClient(node)

    # call in execute()
    success, error_code = self._bph_client.send_goal(joint_angles)
    if success:
        return 'planned'
    ...
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.msg import MoveItErrorCodes

# Replace with your generated package import:
from bph_interfaces.action import BphPickmeup


class BphPickmeupClient:
    """
    Thin synchronous wrapper around the /bph_pickmeup action server.
    Designed to be constructed once and reused across SMACH state executions.
    """

    def __init__(self, node: Node, timeout_sec: float = 10.0):
        self._node = node
        self._client = ActionClient(node, BphPickmeup, "/bph_pickmeup")
        self._node.get_logger().info("Waiting for /bph_pickmeup action server...")
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            self._node.get_logger().error(
                "Timed out waiting for /bph_pickmeup action server."
            )
        else:
            self._node.get_logger().info("Connected to /bph_pickmeup.")

    def send_goal(
        self,
        joint_angles: list[float] | None = None,
        position_name: str = "",
        feedback_cb=None,
    ) -> tuple[bool, int]:
        """
        Send a goal and block until the action completes.

        Parameters
        ----------
        joint_angles  : list of 6 floats (radians), takes precedence over name.
        position_name : named position string (used if joint_angles is None/empty).
        feedback_cb   : optional callable(feedback_msg) for progress updates.

        Returns
        -------
        (success: bool, error_code: int)
            error_code mirrors MoveItErrorCodes.val; 1 == SUCCESS.
        """
        goal = BphPickmeup.Goal()
        goal.joint_angles = list(joint_angles) if joint_angles else []
        goal.position_name = position_name

        self._node.get_logger().info(
            f"Sending bph_pickmeup goal: "
            f"{'joint_angles' if joint_angles else repr(position_name)}"
        )

        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=feedback_cb or self._default_feedback_cb,
        )
        rclpy.spin_until_future_complete(self._node, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self._node.get_logger().warn("bph_pickmeup goal was rejected.")
            return False, MoveItErrorCodes.PLANNING_FAILED

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)

        result = result_future.result().result
        self._node.get_logger().info(
            f"bph_pickmeup result: success={result.success}, "
            f"code={result.error_code}, msg='{result.message}'"
        )
        return result.success, result.error_code

    def _default_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self._node.get_logger().info(
            f"[bph_pickmeup] {fb.current_state} ({fb.progress * 100:.0f}%)"
        )


# ---------------------------------------------------------------------------
# Minimal standalone test
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = rclpy.create_node("bph_pickmeup_client_test")
    client = BphPickmeupClient(node)

    success, code = client.send_goal(
        joint_angles=[0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0],
        position_name="home",
    )
    print(f"Result: success={success}, error_code={code}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
