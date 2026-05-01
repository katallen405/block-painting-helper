#!/usr/bin/env python3
"""
bph_pickmeup client helper 
"""

import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.msg import MoveItErrorCodes

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

        accept_event = threading.Event()
        accept_result = [None]

        def accept_cb(f):
            accept_result[0] = f.result()
            accept_event.set()

        self._client.send_goal_async(
            goal,
            feedback_callback=feedback_cb or self._default_feedback_cb,
        ).add_done_callback(accept_cb)
        accept_event.wait()

        goal_handle = accept_result[0]
        if not goal_handle.accepted:
            self._node.get_logger().warn("bph_pickmeup goal was rejected.")
            return False, MoveItErrorCodes.PLANNING_FAILED

        result_event = threading.Event()
        result_val = [None]

        def result_cb(f):
            result_val[0] = f.result()
            result_event.set()

        goal_handle.get_result_async().add_done_callback(result_cb)
        result_event.wait()

        result = result_val[0].result
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
