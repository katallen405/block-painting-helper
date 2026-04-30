"""
person_tracker_node.py
======================
ROS2 node that detects and tracks people across one or more named zones using
YOLOv8-pose for skeleton keypoint detection and ByteTrack for ID persistence.

YOLO inference runs once per frame. Results are then filtered per zone and
published to separate topic sets — so two robots sharing one camera each get
their own detection stream without paying double inference cost.

Subscriptions:
  /camera/color/image_raw          (sensor_msgs/Image)
  /camera/depth/image_rect_raw     (sensor_msgs/Image)      [optional]
  /camera/depth/camera_info        (sensor_msgs/CameraInfo) [optional]

Publications (one set per named zone, plus an unfiltered global set):
  /person_tracker/image_raw                  (sensor_msgs/Image)        viz
  /person_tracker/detections                 (vision_msgs/Detection2DArray) all detections
  /person_tracker/poses                      (geometry_msgs/PoseArray)   all 3D poses
  /person_tracker/<zone_name>/detections     (vision_msgs/Detection2DArray)
  /person_tracker/<zone_name>/poses          (geometry_msgs/PoseArray)

Parameters:
  model_path    (str)    YOLOv8 .pt model. Default: 'yolov8n-pose.pt'
  confidence    (float)  Detection threshold [0,1]. Default: 0.5
  device        (str)    'cpu' | 'cuda:0' | 'mps'. Default: 'cpu'
  tracker       (str)    'bytetrack.yaml' | 'botsort.yaml'. Default: 'bytetrack.yaml'
  publish_viz   (bool)   Publish annotated image. Default: True
  depth_enabled (bool)   Enable 3D pose estimation via depth image. Default: False
  zones         (str)    JSON dict mapping zone name → polygon point list.
                         Each polygon is a list of [x,y] pixel coordinate pairs.
                         Example:
                           '{"arm": [[100,50],[400,50],[400,350],[100,350]],
                             "turtlebot": [[300,200],[600,200],[600,450],[300,450]]}'
                         Default: '{}' (only global topics published)
"""

import json
import threading
from dataclasses import dataclass, field

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from std_msgs.msg import Header

try:
    from ultralytics import YOLO
except ImportError:
    raise ImportError(
        "ultralytics is not found, export PYTHONPATH=$VIRTUAL_ENV/lib/python3.12/site-packages:$PYTHONPATH"
    )


@dataclass
class Zone:
    """A named workspace zone with its polygon and dedicated publishers."""
    name: str
    polygon: np.ndarray                    # shape (N,1,2) int32
    color: tuple                           # BGR colour for viz outline
    det_pub: object = field(default=None)  # Detection2DArray publisher
    pose_pub: object = field(default=None) # PoseArray publisher


# ---------------------------------------------------------------------------
# Skeleton drawing config (COCO 17-keypoint order)
# ---------------------------------------------------------------------------
COCO_SKELETON = [
    (0, 1), (0, 2), (1, 3), (2, 4),        # head
    (5, 6),                                  # shoulders
    (5, 7), (7, 9), (6, 8), (8, 10),        # arms
    (5, 11), (6, 12), (11, 12),             # torso
    (11, 13), (13, 15), (12, 14), (14, 16), # legs
]

KEYPOINT_NAMES = [
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
]

# Distinct outline colours for zone polygons (BGR)
ZONE_COLORS = [
    (0, 255, 255),   # yellow  — arm workspace
    (255, 128,   0), # blue    — turtlebot workspace
    (0, 255, 128),   # green
    (255,   0, 200), # pink
    (128, 255,   0), # lime
]

# Colour per tracked ID (cycles through these)
ID_COLORS = [
    (255, 80,  80),  (80, 200, 120), (80, 160, 255), (255, 200,  50),
    (200,  80, 255), (80, 230, 230), (255, 130,  50), (180, 255,  80),
]


def _id_color(track_id: int):
    return ID_COLORS[int(track_id) % len(ID_COLORS)]


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class PersonTrackerNode(Node):

    def __init__(self):
        super().__init__("person_tracker")

        # ---- Declare & read parameters -----------------------------------
        self.declare_parameter("model_path",    "yolov8n-pose.pt")
        self.declare_parameter("confidence",    0.5)
        self.declare_parameter("device",        "cpu")
        self.declare_parameter("tracker",       "bytetrack.yaml")
        self.declare_parameter("publish_viz",   True)
        self.declare_parameter("depth_enabled", False)
        self.declare_parameter("zones",         "{}")
        self.declare_parameter("image_topic", "/bph_overhead_camera/image_raw")
        
        self.model_path    = self.get_parameter("model_path").value
        self.confidence    = self.get_parameter("confidence").value
        self.device        = self.get_parameter("device").value
        self.tracker_cfg   = self.get_parameter("tracker").value
        self.publish_viz   = self.get_parameter("publish_viz").value
        self.depth_enabled = self.get_parameter("depth_enabled").value

        # ---- Parse zones -------------------------------------------------
        self.zones: list[Zone] = self._parse_zones(
            self.get_parameter("zones").value
        )

        # ---- Load model --------------------------------------------------
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.get_logger().info("Model loaded.")

        # ---- cv_bridge ---------------------------------------------------
        self.bridge = CvBridge()

        # ---- QoS: sensor best-effort to match most camera drivers --------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- Subscriptions -----------------------------------------------
 #       self.image_sub = self.create_subscription(
 #           Image, "/camera/color/image_raw",
 #           self.image_callback, sensor_qos,
 #       )

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            sensor_qos,
        )
        
        self.depth_image: np.ndarray | None = None
        self.camera_info: CameraInfo | None = None
        self._depth_lock = threading.Lock()

        if self.depth_enabled:
            self.depth_sub = self.create_subscription(
                Image, "/camera/depth/image_rect_raw",
                self.depth_callback, sensor_qos,
            )
            self.info_sub = self.create_subscription(
                CameraInfo, "/camera/depth/camera_info",
                self.info_callback, 10,
            )

        # ---- Global publishers (all detections, unfiltered) --------------
        self.det_pub  = self.create_publisher(
            Detection2DArray, "/person_tracker/detections", 10)
        self.pose_pub = self.create_publisher(
            PoseArray, "/person_tracker/poses", 10)
        if self.publish_viz:
            self.viz_pub = self.create_publisher(
                Image, "/person_tracker/image_raw", 10)

        # ---- Per-zone publishers -----------------------------------------
        for zone in self.zones:
            base = f"/person_tracker/{zone.name}"
            zone.det_pub  = self.create_publisher(Detection2DArray, f"{base}/detections", 10)
            zone.pose_pub = self.create_publisher(PoseArray,        f"{base}/poses",      10)
            self.get_logger().info(
                f"Zone '{zone.name}': publishing on {base}/detections and {base}/poses"
            )

        self.get_logger().info(
            f"PersonTrackerNode ready. {len(self.zones)} zone(s) configured."
        )

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------
    def _parse_zones(self, zones_str: str) -> list[Zone]:
        """Parse the 'zones' JSON parameter into a list of Zone objects."""
        if not zones_str or zones_str == "{}":
            return []
        try:
            raw: dict = json.loads(zones_str)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Could not parse 'zones' parameter: {e}")
            return []

        zones = []
        for i, (name, pts) in enumerate(raw.items()):
            try:
                polygon = np.array(pts, dtype=np.int32).reshape(-1, 1, 2)
                color = ZONE_COLORS[i % len(ZONE_COLORS)]
                zones.append(Zone(name=name, polygon=polygon, color=color))
                self.get_logger().info(
                    f"Parsed zone '{name}' with {len(pts)} vertices."
                )
            except Exception as e:
                self.get_logger().warn(f"Skipping zone '{name}': {e}")
        return zones

    # ------------------------------------------------------------------
    # Depth / CameraInfo callbacks
    # ------------------------------------------------------------------
    def depth_callback(self, msg: Image):
        with self._depth_lock:
            # depth images are typically 16UC1 (millimetres) or 32FC1 (metres)
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    # ------------------------------------------------------------------
    # Main image callback
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        # Convert ROS image → OpenCV BGR
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        header = msg.header

        # Run YOLO tracking — single inference pass for all zones
        results = self.model.track(
            source=frame,
            conf=self.confidence,
            classes=[0],          # class 0 = person in COCO
            device=self.device,
            tracker=self.tracker_cfg,
            persist=True,
            verbose=False,
        )

        viz_frame = frame.copy() if self.publish_viz else None

        # Draw all zone polygons on the viz image
        if viz_frame is not None:
            for zone in self.zones:
                cv2.polylines(viz_frame, [zone.polygon], True, zone.color, 2)
                # Label the zone in the top-left corner of its bounding rect
                x, y, w, h = cv2.boundingRect(zone.polygon)
                cv2.putText(viz_frame, zone.name, (x + 4, y + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, zone.color, 2)

        with self._depth_lock:
            depth_snapshot = self.depth_image.copy() if self.depth_image is not None else None

        # Accumulators: global (all detections) + one per zone
        global_dets  = Detection2DArray(header=header)
        global_poses = PoseArray(header=header)
        zone_dets    = {z.name: Detection2DArray(header=header) for z in self.zones}
        zone_poses   = {z.name: PoseArray(header=header)        for z in self.zones}

        for result in results:
            boxes     = result.boxes
            keypoints = result.keypoints

            if boxes is None:
                continue

            for i, box in enumerate(boxes):
                # ---- Bounding box ----------------------------------------
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                conf  = float(box.conf[0])
                tid   = int(box.id[0]) if box.id is not None else -1
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                # ---- Build Detection2D -----------------------------------
                det = Detection2D(header=header)
                det.bbox = BoundingBox2D()
                det.bbox.center.position.x = float(cx)
                det.bbox.center.position.y = float(cy)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)
                hyp = ObjectHypothesisWithPose()
                det.id = str(id)
                hyp.hypothesis.class_id = "person"
                hyp.hypothesis.score    = conf
                det.results.append(hyp)

                # ---- 3D pose from depth ----------------------------------
                pose = Pose()
                if depth_snapshot is not None and self.camera_info is not None:
                    pose = self._estimate_3d_pose(cx, cy, depth_snapshot, self.camera_info)

                # ---- Add to global (unfiltered) output -------------------
                global_dets.detections.append(det)
                global_poses.poses.append(pose)

                # ---- Fan out to each zone whose polygon contains centroid -
                for zone in self.zones:
                    inside = cv2.pointPolygonTest(
                        zone.polygon, (float(cx), float(cy)), False
                    )
                    if inside >= 0:
                        zone_dets[zone.name].detections.append(det)
                        zone_poses[zone.name].poses.append(pose)

                # ---- Visualisation --------------------------------------
                if viz_frame is not None:
                    color = _id_color(tid)
                    cv2.rectangle(viz_frame, (x1, y1), (x2, y2), color, 2)
                    label = f"ID {tid}  {conf:.2f}" if tid >= 0 else f"{conf:.2f}"
                    cv2.putText(viz_frame, label, (x1, y1 - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
                    if keypoints is not None and i < len(keypoints.data):
                        kp = keypoints.data[i].cpu().numpy()
                        self._draw_skeleton(viz_frame, kp, color)

        # ---- Publish global ----------------------------------------------
        self.det_pub.publish(global_dets)
        self.pose_pub.publish(global_poses)

        # ---- Publish per-zone -------------------------------------------
        for zone in self.zones:
            zone.det_pub.publish(zone_dets[zone.name])
            zone.pose_pub.publish(zone_poses[zone.name])

        # ---- Publish viz -------------------------------------------------
        if viz_frame is not None and self.publish_viz:
            viz_msg = self.bridge.cv2_to_imgmsg(viz_frame, encoding="bgr8")
            viz_msg.header = header
            self.viz_pub.publish(viz_msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _estimate_3d_pose(
        self, cx: int, cy: int,
        depth_img: np.ndarray,
        info: CameraInfo,
    ) -> Pose:
        """Back-project a pixel centroid to 3D using the pinhole model."""
        h, w = depth_img.shape[:2]
        if not (0 <= cy < h and 0 <= cx < w):
            return Pose()

        raw = float(depth_img[cy, cx])
        # Handle both millimetre (16UC1) and metre (32FC1) encodings
        z = raw / 1000.0 if raw > 100 else raw  # heuristic: >100 → mm
        if z <= 0.0 or np.isnan(z):
            return Pose()

        fx, fy = info.k[0], info.k[4]
        cx_int, cy_int = info.k[2], info.k[5]

        x = (cx - cx_int) * z / fx
        y = (cy - cy_int) * z / fy

        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(w=1.0)
        return pose

    def _draw_skeleton(self, frame: np.ndarray, kp: np.ndarray, color: tuple):
        """Draw COCO skeleton lines and keypoint circles onto frame."""
        kp_conf_thresh = 0.3

        # Limb lines
        for a, b in COCO_SKELETON:
            if kp[a, 2] > kp_conf_thresh and kp[b, 2] > kp_conf_thresh:
                pt1 = (int(kp[a, 0]), int(kp[a, 1]))
                pt2 = (int(kp[b, 0]), int(kp[b, 1]))
                cv2.line(frame, pt1, pt2, color, 2)

        # Keypoint dots
        for k in range(len(kp)):
            if kp[k, 2] > kp_conf_thresh:
                pt = (int(kp[k, 0]), int(kp[k, 1]))
                cv2.circle(frame, pt, 4, (255, 255, 255), -1)
                cv2.circle(frame, pt, 4, color, 1)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
