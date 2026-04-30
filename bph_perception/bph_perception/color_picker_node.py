#!/usr/bin/env python3
"""
color_picker_node.py
--------------------
Overhead-camera colour-based object localiser for the block-painting helper.

Architecture
~~~~~~~~~~~~
This node is service-driven.  It keeps the latest camera frame buffered but
does no detection work until the state machine calls:

  ~/get_target_pose  (bph_interfaces/srv/GetTargetPose)
      Request:  string color  — e.g. "red", "green", "blue", "yellow", "natural"
      Response: bool success, string message, geometry_msgs/PoseStamped pose

On each call the node runs the full detect → localise pipeline against the
most recently received frame and returns the result synchronously.  There is
no background detection timer and no stored detection state between calls.

Additionally the node publishes:

  ~/debug_image  (sensor_msgs/Image)
      Annotated BGR image from the most recent service call — useful for
      RViz / rqt_image_view during tuning.  Only published when a subscriber
      is connected.

Parameters (set via launch / ros2 param)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  color_image_topic  (str)   default: /bph_overhead_camera/image_raw
  cam_info_topic     (str)   default: /bph_overhead_camera/camera_info
  cam_z              (float) default: 2.5   — camera height above the floor (m)
  object_height_m    (float) default: 0.0   — height of the object above the floor (m)
                                              set to turtlebot deck height if blocks
                                              are sitting on the robot, not the floor
  min_contour_area   (int)   default: 500   — px² noise filter

Coordinate frame
~~~~~~~~~~~~~~~~
Pixel centroid (u, v) is back-projected into the camera optical frame using
the pinhole model and a fixed Z = cam_z - object_height_m, then transformed
into the map frame via tf2.

TODO items
~~~~~~~~~~
  [ ] Add orientation estimation if the arm needs it (e.g. via contour moments)

Dependencies
~~~~~~~~~~~~
  pip install opencv-python numpy
  ros-kilted-cv-bridge ros-kilted-image-transport
  ros-kilted-tf2-ros ros-kilted-tf2-geometry-msgs
  bph_interfaces
"""

from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np
import rclpy
import rclpy.duration
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  — registers PoseStamped transform support

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image

from bph_interfaces.srv import GetTargetPose


# ─────────────────────────────────────────────────────────────────────────────
# Colour profile registry
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class HsvRange:
    """A single (lower, upper) HSV band.  OpenCV scale: H 0-179, S/V 0-255."""
    lower: np.ndarray
    upper: np.ndarray

    @staticmethod
    def make(h_lo, s_lo, v_lo, h_hi, s_hi, v_hi) -> "HsvRange":
        return HsvRange(
            lower=np.array([h_lo, s_lo, v_lo], dtype=np.uint8),
            upper=np.array([h_hi, s_hi, v_hi], dtype=np.uint8),
        )


@dataclass
class ColorProfile:
    """
    One named colour, potentially spanning multiple HSV bands.
    Red wraps around the 0/179 hue boundary so it uses two bands.
    min_contour_area overrides the node-level parameter for this colour —
    useful for 'natural' which needs a higher threshold to suppress noise.
    """
    name: str
    bands: list[HsvRange] = field(default_factory=list)
    debug_bgr: tuple[int, int, int] = (0, 255, 0)
    min_contour_area: Optional[int] = None  # None → use node-level default


COLOR_PROFILES: dict[str, ColorProfile] = {
    "red": ColorProfile(
        name="red",
        bands=[
            HsvRange.make(  0, 241,  50,  21, 255, 255),  # tuned lower band
            HsvRange.make(170, 120,  70, 179, 255, 255),  # wrap-around band
        ],
        debug_bgr=(0, 0, 255),
    ),
    "yellow": ColorProfile(
        name="yellow",
        bands=[
            HsvRange.make( 22,   0,  88,  30, 255, 202),
        ],
        debug_bgr=(0, 220, 220),
    ),
    "green": ColorProfile(
        name="green",
        bands=[
            HsvRange.make( 48,   0,   0,  83, 191, 126),
        ],
        debug_bgr=(0, 200, 0),
    ),
    "blue": ColorProfile(
        name="blue",
        bands=[
            HsvRange.make( 85, 144,   8, 112, 255, 144),
        ],
        debug_bgr=(255, 80, 0),
    ),
    "natural": ColorProfile(
        name="natural",
        bands=[
            HsvRange.make( 15,  36,  57,  30, 141, 173),
        ],
        debug_bgr=(0, 165, 255),  # orange so it stands out on the debug image
        min_contour_area=1500,    # low saturation → noisy; require larger blobs
    ),
}


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class ColorPickerNode(Node):

    def __init__(self):
        super().__init__("color_picker")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("color_image_topic", "/bph_overhead_camera/image_raw")
        self.declare_parameter("cam_info_topic",    "/bph_overhead_camera/camera_info")
        self.declare_parameter("cam_z",             2.5)
        self.declare_parameter("object_height_m",   0.5) #FIXME -turtlebot deck height
        self.declare_parameter("min_contour_area",  500)

        self._color_topic: str   = self.get_parameter("color_image_topic").value
        self._info_topic:  str   = self.get_parameter("cam_info_topic").value
        self._cam_z:       float = self.get_parameter("cam_z").value
        self._object_z:    float = self.get_parameter("object_height_m").value
        self._min_area:    int   = self.get_parameter("min_contour_area").value

        # Effective Z: distance from camera lens down to the object surface
        self._effective_z = self._cam_z - self._object_z

        # ── Internal state ───────────────────────────────────────────────────
        self._bridge                                 = CvBridge()
        self._camera_info: Optional[CameraInfo]      = None
        self._latest_color_img: Optional[np.ndarray] = None

        # ── tf2 ──────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Subscribers ──────────────────────────────────────────────────────
        qos = QoSPresetProfiles.SENSOR_DATA.value

        self._color_sub = self.create_subscription(
            Image, self._color_topic, self._color_cb, qos,
        )
        self._info_sub = self.create_subscription(
            CameraInfo, self._info_topic, self._info_cb, 10,
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self._debug_pub = self.create_publisher(Image, "~/debug_image", 10)

        # ── Service ──────────────────────────────────────────────────────────
        self._get_pose_srv = self.create_service(
            GetTargetPose, "~/get_target_pose", self._get_pose_cb,
        )

        self.get_logger().info(
            f"ColorPickerNode ready  |  "
            f"cam_z={self._cam_z}m  object_z={self._object_z}m  "
            f"effective_z={self._effective_z:.3f}m"
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Subscriber callbacks — just buffer the latest data
    # ─────────────────────────────────────────────────────────────────────────

    def _color_cb(self, msg: Image) -> None:
        try:
            self._latest_color_img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"color image conversion failed: {e}")

    def _info_cb(self, msg: CameraInfo) -> None:
        # Latch after the first valid message — intrinsics don't change at runtime
        if self._camera_info is None:
            self._camera_info = msg
            K = msg.k
            self.get_logger().info(
                f"CameraInfo latched  |  "
                f"fx={K[0]:.1f}  fy={K[4]:.1f}  "
                f"cx={K[2]:.1f}  cy={K[5]:.1f}"
            )

    # ─────────────────────────────────────────────────────────────────────────
    # Service callback — all detection work happens here
    # ─────────────────────────────────────────────────────────────────────────

    def _get_pose_cb(
        self,
        request: GetTargetPose.Request,
        response: GetTargetPose.Response,
    ) -> GetTargetPose.Response:

        # ── Validate the requested colour ────────────────────────────────────
        if request.color not in COLOR_PROFILES:
            response.success = False
            response.message = (
                f"Unknown color '{request.color}'. "
                f"Known: {list(COLOR_PROFILES.keys())}"
            )
            return response

        # ── Check we have a frame and intrinsics ─────────────────────────────
        if self._latest_color_img is None:
            response.success = False
            response.message = f"No image received yet on {self._color_topic}"
            return response

        if self._camera_info is None:
            response.success = False
            response.message = f"No CameraInfo received yet on {self._info_topic}"
            return response

        # Snapshot the current frame so the subscriber can't overwrite it
        # mid-detection
        color_img = self._latest_color_img.copy()

        # ── Detection pipeline ───────────────────────────────────────────────
        mask = self._build_color_mask(color_img, request.color)
        if mask is None:
            response.success = False
            response.message = "Mask build failed (unknown profile)"
            return response

        centroid_px = self._find_centroid(mask, request.color)
        if centroid_px is None:
            response.success = False
            response.message = f"No '{request.color}' object detected in current frame"
            self._publish_debug_image(color_img, mask, request.color, centroid_px=None)
            return response

        pose = self._pixel_to_map(centroid_px)
        if pose is None:
            response.success = False
            response.message = "TF2 transform to map frame failed"
            return response

        # ── Publish debug image ──────────────────────────────────────────────
        self._publish_debug_image(color_img, mask, request.color, centroid_px)

        response.success = True
        response.message = f"Found '{request.color}'"
        response.pose    = pose
        return response

    # ─────────────────────────────────────────────────────────────────────────
    # Colour segmentation
    # ─────────────────────────────────────────────────────────────────────────

    def _build_color_mask(
        self, bgr: np.ndarray, color_name: str
    ) -> Optional[np.ndarray]:
        """Return a binary mask where pixels match the requested colour."""
        profile = COLOR_PROFILES.get(color_name)
        if profile is None:
            return None

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for band in profile.bands:
            combined_mask = cv2.bitwise_or(
                combined_mask, cv2.inRange(hsv, band.lower, band.upper)
            )

        # Morphological clean-up: remove speckle noise, fill small holes
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN,  kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

        return combined_mask

    # ─────────────────────────────────────────────────────────────────────────
    # Contour → centroid
    # ─────────────────────────────────────────────────────────────────────────

    def _find_centroid(
        self, mask: np.ndarray, color_name: str
    ) -> Optional[tuple[int, int]]:
        """
        Return the (u, v) centroid of the largest contour above the area
        threshold, or None if nothing passes.
        """
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return None

        best = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best)

        profile  = COLOR_PROFILES.get(color_name)
        min_area = (
            profile.min_contour_area
            if (profile and profile.min_contour_area is not None)
            else self._min_area
        )
        if area < min_area:
            return None

        M = cv2.moments(best)
        if M["m00"] == 0:
            return None

        return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    # ─────────────────────────────────────────────────────────────────────────
    # 2-D pixel → 3-D map pose
    # ─────────────────────────────────────────────────────────────────────────

    def _pixel_to_map(
        self, centroid_px: tuple[int, int]
    ) -> Optional[PoseStamped]:
        """
        Convert a pixel centroid → PoseStamped in the map frame.

        Because the camera is fixed overhead, Z is constant:
            Z = cam_z - object_height_m

        The pinhole model gives the camera-frame X/Y:
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy

        TF2 then rotates the result into the map frame.
        """
        K    = self._camera_info.k
        fx   = K[0];  fy   = K[4]
        cx_i = K[2];  cy_i = K[5]

        u, v = centroid_px
        Z    = self._effective_z

        pt = PointStamped()
        pt.header.frame_id = self._camera_info.header.frame_id
        pt.header.stamp    = self.get_clock().now().to_msg()
        pt.point.x = (u - cx_i) * Z / fx
        pt.point.y = (v - cy_i) * Z / fy
        pt.point.z = Z

        try:
            pt_map = self._tf_buffer.transform(
                pt, "map", timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except:
            self.get_logger().warn("TF2 to map failed, returning camera-frame pose")
            # return the raw camera-frame point instead
            fallback = PoseStamped()
            fallback.header = pt.header   # frame_id = 'camera'
            fallback.pose.position.x = pt.point.x
            fallback.pose.position.y = pt.point.y
            fallback.pose.position.z = pt.point.z
            fallback.pose.orientation.w = 1.0
            return fallback, pt.header.frame_id

        pose = PoseStamped()
        pose.header = pt_map.header
        pose.pose.position.x    = pt_map.point.x
        pose.pose.position.y    = pt_map.point.y
        pose.pose.position.z    = pt_map.point.z
        pose.pose.orientation.w = 1.0  # identity — arm planner handles approach angle
        return pose

    # ─────────────────────────────────────────────────────────────────────────
    # Debug image
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_debug_image(
        self,
        bgr: np.ndarray,
        mask: np.ndarray,
        color_name: str,
        centroid_px: Optional[tuple[int, int]],
    ) -> None:
        if self._debug_pub.get_subscription_count() == 0:
            return

        vis = bgr.copy()
        profile   = COLOR_PROFILES.get(color_name)
        color_bgr = profile.debug_bgr if profile else (0, 255, 0)

        # Semi-transparent tint over masked pixels
        tint = np.zeros_like(vis)
        tint[mask > 0] = color_bgr
        cv2.addWeighted(tint, 0.3, vis, 0.7, 0, vis)

        if centroid_px is not None:
            cx, cy = centroid_px
            cv2.drawMarker(
                vis, (cx, cy), (255, 255, 255),
                cv2.MARKER_CROSS, 20, 2,
            )
            cv2.putText(
                vis, f"target: {color_name}", (cx + 8, cy - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2,
            )
        else:
            cv2.putText(
                vis, f"no detection: {color_name}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
            )

        try:
            self._debug_pub.publish(self._bridge.cv2_to_imgmsg(vis, "bgr8"))
        except Exception as e:
            self.get_logger().warn(f"debug image publish failed: {e}")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ColorPickerNode()
    print("Input from ", node._color_topic, "debug output on debug_image")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
