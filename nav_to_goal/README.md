# nav_to_goal

Robot-agnostic ROS 2 package that navigates to a map coordinate using
**slam_toolbox** (online SLAM) and **Nav2**, driven by a **single depth camera**.

---

## Architecture

```
Depth Camera
    │ sensor_msgs/Image
    ▼
depthimage_to_laserscan          ← converts depth frame to LaserScan
    │ sensor_msgs/LaserScan  (/scan)
    ├──▶ slam_toolbox             ← builds + maintains the map
    │        │ nav_msgs/OccupancyGrid (/map)
    │        │ tf: map → odom → base_link
    │        ▼
    └──▶ Nav2 costmaps            ← obstacle avoidance
             │
             ▼
         Nav2 stack  (planner → controller → robot cmd_vel)
             ▲
             │ NavigateToPose action
    navigator_node  ◀── you configure goal_x / goal_y / goal_yaw
```

---

## Requirements

| Package | Notes |
|---|---|
| `ros-humble-nav2-bringup` | Full Nav2 stack |
| `ros-humble-slam-toolbox` | Online async SLAM |
| `ros-humble-depthimage-to-laserscan` | Depth → LaserScan |
| `ros-humble-nav2-regulated-pure-pursuit-controller` | Path tracking |

Install missing packages:
```bash
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-depthimage-to-laserscan \
  ros-humble-nav2-regulated-pure-pursuit-controller
```

---

## Build

```bash
cd ~/ros2_ws/src
# copy or symlink the nav_to_goal directory here
cd ~/ros2_ws
colcon build --packages-select nav_to_goal
source install/setup.bash
```

---

## Run

### Minimal (all defaults)
```bash
ros2 launch nav_to_goal bringup.launch.py
```

### Custom goal and camera topic
```bash
ros2 launch nav_to_goal bringup.launch.py \
    goal_x:=3.0 goal_y:=2.5 goal_yaw:=1.57 \
    depth_image_topic:=/camera/depth/image_raw
```

### TurtleBot4 (OAK-D camera)
```bash
ros2 launch nav_to_goal bringup.launch.py \
    depth_image_topic:=/oakd/stereo/image_raw \
    depth_info_topic:=/oakd/stereo/camera_info \
    robot_base_frame:=base_link \
    goal_x:=2.0 goal_y:=1.0
```

### Gazebo simulation
```bash
ros2 launch nav_to_goal bringup.launch.py use_sim_time:=true goal_x:=4.0 goal_y:=0.0
```

---

## Launch arguments

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use `/clock` topic |
| `params_file` | `config/nav2_params.yaml` | Nav2 parameter overrides |
| `slam_params_file` | `config/slam_params.yaml` | SLAM overrides |
| `goal_x` | `2.0` | Goal X (metres, map frame) |
| `goal_y` | `1.0` | Goal Y (metres, map frame) |
| `goal_yaw` | `0.0` | Goal heading (radians) |
| `depth_image_topic` | `/camera/depth/image_raw` | Depth image input |
| `depth_info_topic` | `/camera/depth/camera_info` | Camera info input |
| `robot_base_frame` | `base_link` | Robot base TF frame |

---

## Send a goal from the command line (runtime)

You can also bypass the node and send goals manually:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 0.0},
   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

---

## Robot-specific adaptations

The only values that change per robot are:

1. **`robot_base_frame`** — e.g. `base_footprint` for TurtleBot3
2. **`robot_radius`** in `nav2_params.yaml` — footprint inflation
3. **Depth topic names** — set via launch arguments
4. **`output_frame`** in the `depthimage_to_laserscan` node — should match
   the optical frame of your depth camera (e.g. `camera_depth_optical_frame`)
   or `base_link` if you want scans already in robot frame.

Everything else (SLAM, Nav2 stack, goal logic) is fully generic.

---

## Saving a map

Once the robot has explored, save the map for later re-use:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_room_map
```

To load a saved map instead of running SLAM:

```bash
# Replace slam_toolbox with map_server + AMCL in the launch file,
# or use nav2_bringup's localization_launch.py:
ros2 launch nav2_bringup localization_launch.py \
    map:=~/my_room_map.yaml
```
