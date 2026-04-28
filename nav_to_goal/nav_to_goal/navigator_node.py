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
from bph_interfaces.srv import GoToLocation
from std_msgs.msg import String



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
        self._srv = self.create_service(GoToLocation, 'navigate_to', self._navigate_callback)

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        
        self._map_frame = self.get_parameter("map_frame").value
        self._robot_base_frame = self.get_parameter("robot_base_frame").value

        # ── State flags ───────────────────────────────────────────────────────
        self._map_received = False
        self._nav2_ready = False
        self._goal_sent = False
        self._status_pub = self.create_publisher(String, '/navigation_status', 10)


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

    def _navigate_callback(self, request, response):
        if not self._map_received:
            response.accepted = False
            response.message = "No map yet"
            return response
        if not self._nav2_ready:
            self._nav2_ready = self._all_nav2_nodes_active()
            if not self._nav2_ready:
                response.accepted = False
                response.message = "Nav2 not ready"
                return response

        self._goal_x = request.x
        self._goal_y = request.y
        self._goal_yaw = request.yaw
        self._send_goal()
        response.accepted = True
        response.message = f"Goal sent to ({request.x}, {request.y})"
        return response

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
        msg = String()

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🎉 Goal REACHED successfully!")
            msg.data = f"SUCCEEDED:{self._goal_x},{self._goal_y}"
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Goal was CANCELED.")
            msg.data = "CANCELED"
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Goal ABORTED by Nav2.")
            msg.data = "ABORTED"
        else:
            self.get_logger().warn(f"Goal ended with status code: {status}")
            msg.data = f"STATUS_{status}"
        self._status_pub.publish(msg)


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
