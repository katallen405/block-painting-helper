# person_tracker

ROS2 node for real-time person detection and tracking using **YOLOv8-pose**,
designed for robot arm collision avoidance.

## What it does

- Subscribes to a camera image topic
- Runs YOLOv8-pose to detect people and extract 17-point COCO skeletons
- Tracks individuals across frames with ByteTrack (persistent IDs)
- Optionally filters detections to a configurable workspace polygon
- Optionally back-projects detections to 3D using a depth image
- Publishes bounding boxes (`vision_msgs/Detection2DArray`), 3D poses
  (`geometry_msgs/PoseArray`), and an annotated visualization image

---

## Prerequisites

**ROS2** (Humble or later) with:
```
vision_msgs  cv_bridge  sensor_msgs  geometry_msgs

v4l2-camera (for the USB camera, started by this node)
```

**Python packages:**
```bash
pip install ultralytics   # pulls in torch, torchvision, opencv, numpy
```

---

## Build

```bash
# Clone into your workspace src/
cd ~/ros2_ws/src
git clone <this-repo> person_tracker   # or copy the folder

# Install Python deps declared in package.xml
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select person_tracker
source install/setup.bash
```

---

## Quick start

```bash
# Launch with defaults (nano model, CPU, full image, no depth)
ros2 launch person_tracker person_tracker.launch.py

# Override single parameters on the command line
ros2 run person_tracker person_tracker_node \
  --ros-args \
  -p model_path:=yolov8s-pose.pt \
  -p confidence:=0.4 \
  -p device:=cuda:0 \
  -p publish_viz:=true

# Remap camera topic
ros2 run person_tracker person_tracker_node \
  --ros-args \
  -r /camera/color/image_raw:=/realsense/color/image_raw
```

---

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | **in** | RGB camera |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | in | Only if `depth_enabled: true` |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | in | Only if `depth_enabled: true` |
| `/person_tracker/detections` | `vision_msgs/Detection2DArray` | **out** | Bounding boxes + track IDs |
| `/person_tracker/poses` | `geometry_msgs/PoseArray` | out | 3D positions (depth required) |
| `/person_tracker/image_raw` | `sensor_msgs/Image` | out | Annotated viz (if enabled) |

---

## Parameters

See `config/params.yaml` for full documentation. Key parameters:

| Parameter | Default | Description |
|---|---|---|
| `model_path` | `yolov8n-pose.pt` | Model weights (auto-downloaded) |
| `confidence` | `0.5` | Detection threshold |
| `device` | `cpu` | `cpu` / `cuda:0` / `mps` |
| `tracker` | `bytetrack.yaml` | ByteTrack or BoTSORT |
| `workspace_polygon` | `""` | JSON pixel polygon to filter workspace |
| `depth_enabled` | `false` | Enable 3D pose estimation |
| `publish_viz` | `true` | Publish annotated image |

---

## Consuming detections in your collision avoidance node

```python
from vision_msgs.msg import Detection2DArray

def detections_callback(self, msg: Detection2DArray):
    for det in msg.detections:
        cx = det.bbox.center.position.x
        cy = det.bbox.center.position.y
        w  = det.bbox.size_x
        h  = det.bbox.size_y
        track_id = det.results[0].hypothesis.class_id  # "person"
        confidence = det.results[0].hypothesis.score
        # Feed into your safety zone / collision avoidance logic
```

If `depth_enabled: true`, consume `/person_tracker/poses` for 3D positions
in the camera frame — transform to the robot base frame with `tf2_ros`.

---

## Model size vs. performance

| Model | Size | CPU (i7) | GPU (RTX 3060) | Use when |
|---|---|---|---|---|
| `yolov8n-pose.pt` | 6 MB | ~8 FPS | ~60 FPS | Prototyping / low-power |
| `yolov8s-pose.pt` | 23 MB | ~4 FPS | ~50 FPS | Good default |
| `yolov8m-pose.pt` | 52 MB | ~2 FPS | ~35 FPS | High accuracy needed |
| `yolov8x-pose.pt` | 137 MB | <1 FPS | ~20 FPS | Offline / best accuracy |

For production collision avoidance on CPU, start with `yolov8n-pose.pt` and
only upgrade if skeleton keypoint accuracy is insufficient.
