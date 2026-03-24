#!/usr/bin/env python3
"""
navigator_node.py
-----------------
Robot-agnostic Nav2 navigation node.

- Waits for SLAM (slam_toolbox) to produce a valid map
- Waits for Nav2 (lifecycle nodes) to become active
- Sends a single NavigateToPose goal
- Reports progress via feedback and result callbacks

Configuration (ROS parameters, all overridable at launch):
  goal_x          (float, default  2.0)  : Goal X in map frame
  goal_y          (float, default  1.0)  : Goal Y in map frame
  goal_yaw        (float, default  0.0)  : Goal heading (radians)
  map_frame       (str,   default "map") : Fixed/map frame id
  robot_base_frame(str,   default "base_link") : Robot base frame id
  navigate_on_start (bool, default True) : Navigate immediately on startup

Minimum external requirements
  - slam_toolbox running in lifelong / online-async mode
  - nav2_bringup stack running (bt_navigator, controller_server, etc.)
  - A depth camera publishing sensor_msgs/Image (or PointCloud2) consumed
    by a depth_image_proc / pointcloud_to_laserscan node whose output feeds
    the costmap — no topic wiring is done here; that lives in the Nav2
    parameter YAML (see config/nav2_params.yaml).
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from lifecycle_msgs.srv import GetState


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a geometry_msgs/Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class NavigatorNode(Node):
    """Sends a NavigateToPose goal once SLAM and Nav2 are ready."""

    # Nav2 lifecycle nodes that must be 'active' before we send a goal
    NAV2_NODES = [
        "bt_navigator",
        "controller_server",
        "planner_server",
        "recoveries_server",  # nav2_recoveries (Humble+)
    ]

    def __init__(self):
        super().__init__("navigator_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("goal_x", 2.0)
        self.declare_parameter("goal_y", 1.0)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("navigate_on_start", True)

        self._goal_x = self.get_parameter("goal_x").value
        self._goal_y = self.get_parameter("goal_y").value
        self._goal_yaw = self.get_parameter("goal_yaw").value
        self._map_frame = self.get_parameter("map_frame").value
        self._robot_base_frame = self.get_parameter("robot_base_frame").value
        self._navigate_on_start = self.get_parameter("navigate_on_start").value

        # ── State flags ───────────────────────────────────────────────────────
        self._map_received = False
        self._nav2_ready = False
        self._goal_sent = False

        # ── Action client ─────────────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── Map subscriber (latched-style QoS to catch maps published before
        #    this node starts) ─────────────────────────────────────────────────
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, "map", self._map_callback, map_qos
        )

        # ── Periodic readiness check ──────────────────────────────────────────
        self._check_timer = self.create_timer(2.0, self._check_ready)

        self.get_logger().info(
            f"NavigatorNode started. Goal: ({self._goal_x}, {self._goal_y}, "
            f"yaw={self._goal_yaw:.2f} rad) in frame '{self._map_frame}'"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _map_callback(self, msg: OccupancyGrid):
        if not self._map_received:
            w, h = msg.info.width, msg.info.height
            self.get_logger().info(f"Map received ({w}×{h} cells).")
            self._map_received = True
            self._map_sub  # keep alive — unsubscribe if memory matters

    def _check_ready(self):
        """Poll Nav2 lifecycle nodes; send goal once everything is ready."""
        if self._goal_sent:
            self._check_timer.cancel()
            return

        if not self._map_received:
            self.get_logger().info("Waiting for SLAM map …", throttle_duration_sec=5.0)
            return

        if not self._nav2_ready:
            self._nav2_ready = self._all_nav2_nodes_active()
            if not self._nav2_ready:
                return

        if self._navigate_on_start:
            self._send_goal()

    def _all_nav2_nodes_active(self) -> bool:
        """Return True only if every monitored Nav2 node reports 'active'."""
        for node_name in self.NAV2_NODES:
            srv = self.create_client(GetState, f"{node_name}/get_state")
            if not srv.wait_for_service(timeout_sec=0.2):
                self.get_logger().info(
                    f"Nav2 node '{node_name}' not yet available …",
                    throttle_duration_sec=5.0,
                )
                return False
            future = srv.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if future.result() is None:
                return False
            state = future.result().current_state.label
            if state != "active":
                self.get_logger().info(
                    f"Nav2 node '{node_name}' is '{state}' (need 'active') …",
                    throttle_duration_sec=5.0,
                )
                return False
        self.get_logger().info("All Nav2 nodes are active ✓")
        return True

    # ── Goal sending ──────────────────────────────────────────────────────────

    def _send_goal(self):
        self._goal_sent = True
        self._check_timer.cancel()

        self.get_logger().info(
            f"Sending NavigateToPose goal → "
            f"({self._goal_x:.2f}, {self._goal_y:.2f}) "
            f"yaw={self._goal_yaw:.2f} rad"
        )

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self._goal_x
        pose.pose.position.y = self._goal_y
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(self._goal_yaw)
        goal_msg.pose = pose

        send_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was REJECTED by Nav2.")
            return
        self.get_logger().info("Goal ACCEPTED — robot is navigating …")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        self.get_logger().info(
            f"[Nav2 feedback] Distance remaining: {dist:.2f} m",
            throttle_duration_sec=2.0,
        )

    def _result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🎉 Goal REACHED successfully!")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Goal was CANCELED.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Goal ABORTED by Nav2.")
        else:
            self.get_logger().warn(f"Goal ended with status code: {status}")


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
