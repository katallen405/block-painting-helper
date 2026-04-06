"""
topdown_viz_node.py
===================
ROS2 node that subscribes to /person_tracker/detections and renders a
true top-down map-frame view of the room, projecting detected persons
from camera pixel space onto the floor plane via TF2 and camera intrinsics.

How it works:
  1. For each detected person, take the pixel centroid (cx, cy)
  2. Unproject to a ray in camera_link frame using the pinhole model
  3. Look up map -> camera_link transform via TF2
  4. Intersect the ray with the Z=0 plane in map frame
  5. Plot the resulting (X, Y) map-frame position on the canvas

Subscriptions:
  /person_tracker/detections  (vision_msgs/Detection2DArray)

Publications:
  /person_tracker/topdown     (sensor_msgs/Image)  BGR8 top-down map canvas

Parameters:
  --- Canvas ---
  map_width_m   (float) Width  of the map view in metres. Default: 6.0
  map_height_m  (float) Height of the map view in metres. Default: 6.0
  map_origin_x  (float) Map-frame X at the left   edge of canvas. Default: -1.0
  map_origin_y  (float) Map-frame Y at the bottom edge of canvas. Default: -3.0
  image_width   (int)   Canvas width  in pixels. Default: 640
  image_height  (int)   Canvas height in pixels. Default: 640
  publish_hz    (float) Publish rate in Hz. Default: 10.0
  trail_length  (int)   History positions per person. Default: 40

  --- Camera intrinsics (C920 @ 640x480 defaults) ---
  fx  (float)  Focal length x. Default: 617.0
  fy  (float)  Focal length y. Default: 617.0
  cx  (float)  Principal point x. Default: 320.0
  cy  (float)  Principal point y. Default: 240.0

  --- Zones (map frame, metres) ---
  zones (str)  JSON dict: zone_name -> list of [x,y] pairs in map frame.
               Example:
                 '{"arm": [[0.0,-0.5],[0.8,-0.5],[0.8,0.5],[0.0,0.5]],
                   "turtlebot": [[1.0,-0.8],[2.5,-0.8],[2.5,0.8],[1.0,0.8]]}'
"""

import json
import colorsys
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from tf2_ros import TransformException

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


def _id_color(track_id: int) -> tuple:
    hue = (track_id * 0.618033988749895) % 1.0
    r, g, b = colorsys.hsv_to_rgb(hue, 0.85, 0.95)
    return (int(b * 255), int(g * 255), int(r * 255))


ZONE_OUTLINE_COLORS = [
    (0, 210, 140),
    (220, 130,  30),
    (180,  60, 200),
    (50,  200, 240),
]


class TopdownVizNode(Node):

    def __init__(self):
        super().__init__("topdown_viz")

        # ---- Parameters --------------------------------------------------
        self.declare_parameter("map_width_m",   6.0)
        self.declare_parameter("map_height_m",  6.0)
        self.declare_parameter("map_origin_x", -1.0)  # map-frame X at left edge
        self.declare_parameter("map_origin_y", -3.0)  # map-frame Y at bottom edge
        self.declare_parameter("image_width",   640)
        self.declare_parameter("image_height",  640)
        self.declare_parameter("publish_hz",    10.0)
        self.declare_parameter("trail_length",  40)
        self.declare_parameter("fx",  617.0)
        self.declare_parameter("fy",  617.0)
        self.declare_parameter("cx",  320.0)
        self.declare_parameter("cy",  240.0)
        self.declare_parameter("zones", "{}")

        self.map_w   = self.get_parameter("map_width_m").value
        self.map_h   = self.get_parameter("map_height_m").value
        self.orig_x  = self.get_parameter("map_origin_x").value
        self.orig_y  = self.get_parameter("map_origin_y").value
        self.W       = self.get_parameter("image_width").value
        self.H       = self.get_parameter("image_height").value
        hz           = self.get_parameter("publish_hz").value
        self.trail_len = self.get_parameter("trail_length").value
        self.fx      = self.get_parameter("fx").value
        self.fy      = self.get_parameter("fy").value
        self.cx_     = self.get_parameter("cx").value
        self.cy_     = self.get_parameter("cy").value
        zones_str    = self.get_parameter("zones").value

        # pixels per metre
        self.px_per_m_x = self.W / self.map_w
        self.px_per_m_y = self.H / self.map_h

        self.zones = self._parse_zones(zones_str)

        # ---- TF2 ---------------------------------------------------------
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- State -------------------------------------------------------
        self.trails:  dict[int, deque] = {}
        self.persons: dict[int, tuple] = {}  # tid -> (mx, my, conf)
        self.bridge = CvBridge()

        # ---- QoS ---------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- Subscriptions / publishers ----------------------------------
        self.det_sub = self.create_subscription(
            Detection2DArray,
            "/person_tracker/detections",
            self.detections_callback,
            sensor_qos,
        )
        self.img_pub = self.create_publisher(Image, "/person_tracker/topdown", 10)
        self.timer   = self.create_timer(1.0 / hz, self.publish_frame)

        self.get_logger().info(
            f"TopdownVizNode ready. Map: {self.map_w}x{self.map_h}m "
            f"origin=({self.orig_x},{self.orig_y}), "
            f"canvas={self.W}x{self.H}px, {len(self.zones)} zone(s)."
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _parse_zones(self, zones_str: str) -> list[dict]:
        if not zones_str or zones_str == "{}":
            return []
        try:
            raw = json.loads(zones_str)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Could not parse zones: {e}")
            return []
        zones = []
        for i, (name, pts) in enumerate(raw.items()):
            # pts are in map-frame metres — convert to canvas pixels for drawing
            canvas_pts = np.array(
                [self._map_to_canvas(x, y) for x, y in pts],
                dtype=np.int32,
            ).reshape(-1, 1, 2)
            color = ZONE_OUTLINE_COLORS[i % len(ZONE_OUTLINE_COLORS)]
            # Keep original metre pts for point-in-zone test
            metre_pts = np.array(pts, dtype=np.float32).reshape(-1, 1, 2)
            zones.append({
                "name":       name,
                "canvas_pts": canvas_pts,
                "metre_pts":  metre_pts,
                "color":      color,
            })
            self.get_logger().info(f"Zone '{name}' loaded with {len(pts)} vertices.")
        return zones

    def _map_to_canvas(self, mx: float, my: float) -> tuple[int, int]:
        """Convert map-frame (X, Y) in metres to canvas pixel (u, v).
        Map +X  → canvas right
        Map +Y  → canvas up  (so we flip v)
        """
        u = int((mx - self.orig_x) * self.px_per_m_x)
        v = int(self.H - (my - self.orig_y) * self.px_per_m_y)
        return (u, v)

    def _pixel_to_map(self, cx: float, cy: float) -> tuple[float, float] | None:
        """
        Unproject a pixel centroid to a map-frame floor position.
        Steps:
          1. Form a unit ray in camera_link frame from the pinhole model
          2. Look up camera_link pose in map frame via TF2
          3. Express the ray in map frame
          4. Intersect with the Z=0 plane (floor)
        Returns (mx, my) in metres, or None if TF is unavailable.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "camera_link",
                rclpy.time.Time(),           # latest available
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=2.0)
            return None

        # Camera origin in map frame
        t = tf.transform.translation
        cam_origin = np.array([t.x, t.y, t.z])

        # Rotation: camera_link → map
        q = tf.transform.rotation
        R = self._quat_to_rot(q.x, q.y, q.z, q.w)

        # Ray direction in camera_link frame (pinhole model, z-forward)
        ray_cam = np.array([
            (cx - self.cx_) / self.fx,
            (cy - self.cy_) / self.fy,
            1.0,
        ])
        ray_cam /= np.linalg.norm(ray_cam)

        # Ray direction in map frame
        ray_map = R @ ray_cam

        # Intersect with floor plane Z=0 in map frame:
        #   cam_origin + t * ray_map  →  z component = 0
        #   t = -cam_origin[2] / ray_map[2]
        if abs(ray_map[2]) < 1e-6:
            # Ray is nearly parallel to floor — no intersection
            return None

        t_intersect = -cam_origin[2] / ray_map[2]
        if t_intersect < 0:
            # Intersection is behind the camera
            return None

        floor_pt = cam_origin + t_intersect * ray_map
        return float(floor_pt[0]), float(floor_pt[1])

    @staticmethod
    def _quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
        """Quaternion → 3x3 rotation matrix."""
        return np.array([
            [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
            [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
            [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
        ])

    def _inside_zone(self, mx: float, my: float, metre_pts: np.ndarray) -> bool:
        return cv2.pointPolygonTest(
            metre_pts, (float(mx), float(my)), False
        ) >= 0

    # ------------------------------------------------------------------
    # Detection callback
    # ------------------------------------------------------------------
    def detections_callback(self, msg: Detection2DArray):
        self.get_logger().info(f"Got {len(msg.detections)} detections")
        seen_ids = set()

        for det in msg.detections:
            self.get_logger().info(f"  det.id='{det.id}' class_id='{det.results[0].hypothesis.class_id if det.results else None}'")
            if not det.results:
                continue
            # Track ID is stored in det.id (set by person_tracker_node)
            # Fall back to hashing class_id if somehow absent
            raw_id = det.id if det.id else det.results[0].hypothesis.class_id
            try:
                tid = int(raw_id)
            except (ValueError, TypeError):
                tid = hash(str(raw_id)) % 10000
            self.get_logger().info(f"  tid={tid}")

            conf = det.results[0].hypothesis.score
            cx   = det.bbox.center.position.x
            cy   = det.bbox.center.position.y
            self.get_logger().info(f"  pixel=({cx:.1f},{cy:.1f})")
            
            floor_pos = self._pixel_to_map(cx, cy)
            self.get_logger().info(f"  floor_pos={floor_pos}")
            if floor_pos is None:
                continue

            mx, my = floor_pos
            seen_ids.add(tid)
            self.persons[tid] = (mx, my, conf)

            if tid not in self.trails:
                self.trails[tid] = deque(maxlen=self.trail_len)
            self.trails[tid].append((mx, my))

        gone = set(self.persons.keys()) - seen_ids
        for tid in gone:
            del self.persons[tid]

    # ------------------------------------------------------------------
    # Render & publish
    # ------------------------------------------------------------------
    def publish_frame(self):
        canvas = np.zeros((self.H, self.W, 3), dtype=np.uint8)
        canvas[:] = (30, 30, 30)

        # Grid — every 0.5m
        grid_step_px_x = int(0.5 * self.px_per_m_x)
        grid_step_px_y = int(0.5 * self.px_per_m_y)
        grid_color = (55, 55, 55)
        for u in range(0, self.W, max(grid_step_px_x, 1)):
            cv2.line(canvas, (u, 0), (u, self.H), grid_color, 1)
        for v in range(0, self.H, max(grid_step_px_y, 1)):
            cv2.line(canvas, (0, v), (self.W, v), grid_color, 1)

        # Map origin axes
        ox, oy = self._map_to_canvas(0.0, 0.0)
        cv2.line(canvas, (ox, oy), (ox + int(0.3*self.px_per_m_x), oy),
                 (0, 80, 255), 2, cv2.LINE_AA)  # +X red
        cv2.line(canvas, (ox, oy), (ox, oy - int(0.3*self.px_per_m_y)),
                 (0, 200, 80), 2, cv2.LINE_AA)  # +Y green
        cv2.putText(canvas, "X", (ox + int(0.32*self.px_per_m_x), oy + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 80, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, "Y", (ox - 4, oy - int(0.32*self.px_per_m_y)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 80), 1, cv2.LINE_AA)

        # Scale bar — 1 metre
        bar_y   = self.H - 20
        bar_x0  = 20
        bar_x1  = bar_x0 + int(self.px_per_m_x)
        cv2.line(canvas, (bar_x0, bar_y), (bar_x1, bar_y), (180,180,180), 2)
        cv2.line(canvas, (bar_x0, bar_y-4), (bar_x0, bar_y+4), (180,180,180), 1)
        cv2.line(canvas, (bar_x1, bar_y-4), (bar_x1, bar_y+4), (180,180,180), 1)
        cv2.putText(canvas, "1 m", (bar_x0, bar_y - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180,180,180), 1, cv2.LINE_AA)

        # Zone polygons
        for zone in self.zones:
            poly  = zone["canvas_pts"]
            color = zone["color"]
            overlay = canvas.copy()
            cv2.fillPoly(overlay, [poly], color)
            cv2.addWeighted(overlay, 0.12, canvas, 0.88, 0, canvas)
            cv2.polylines(canvas, [poly], True, color, 2, cv2.LINE_AA)
            x, y, _, _ = cv2.boundingRect(poly)
            cv2.putText(canvas, zone["name"], (x + 6, y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

        # Trails
        for tid, trail in self.trails.items():
            pts = list(trail)
            if len(pts) < 2:
                continue
            color = _id_color(tid)
            for i in range(1, len(pts)):
                alpha = i / len(pts)
                faded = tuple(int(c * alpha * 0.6) for c in color)
                p1 = self._map_to_canvas(*pts[i-1])
                p2 = self._map_to_canvas(*pts[i])
                cv2.line(canvas, p1, p2, faded, 2, cv2.LINE_AA)

        # Persons
        total = 0
        in_zone: dict[str, int] = {z["name"]: 0 for z in self.zones}

        for tid, (mx, my, conf) in self.persons.items():
            color = _id_color(tid)
            u, v  = self._map_to_canvas(mx, my)
            total += 1

            badges = []
            for zone in self.zones:
                if self._inside_zone(mx, my, zone["metre_pts"]):
                    badges.append(zone["name"])
                    in_zone[zone["name"]] += 1

            # Person dot
            cv2.circle(canvas, (u, v), 10, color, -1, cv2.LINE_AA)
            cv2.circle(canvas, (u, v), 10, (255,255,255), 1, cv2.LINE_AA)

            # Direction arrow placeholder (future: use velocity estimate)
            # Label
            badge_str = f" [{', '.join(badges)}]" if badges else ""
            label = f"ID {tid}  {conf:.2f}{badge_str}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
            lx, ly = u + 13, v + 5
            cv2.rectangle(canvas, (lx-2, ly-th-2), (lx+tw+2, ly+2), (20,20,20), -1)
            cv2.putText(canvas, label, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

            # Map coords
            coord_str = f"({mx:.2f}, {my:.2f}) m"
            cv2.putText(canvas, coord_str, (lx, ly + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, (160,160,160), 1, cv2.LINE_AA)

        # Count overlay (bottom left)
        lines = [f"persons: {total}"] + [f"  {n}: {c}" for n, c in in_zone.items()]
        for i, line in enumerate(reversed(lines)):
            cv2.putText(canvas, line, (8, self.H - 10 - i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200,200,200), 1, cv2.LINE_AA)

        msg = self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        self.img_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TopdownVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
