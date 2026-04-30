Run with:
ros2 run bph_perception color_picker \
  --ros-args \
  -p color_image_topic:=/bph_overhead_camera/image_raw \
  -p cam_info_topic:=/bph_overhead_camera/camera_info \
  -p cam_z:=2.5 \
  -p object_height_m:=0.5
  
Test with
ros2 service call /color_picker/get_target_pose bph_interfaces/srv/GetTargetPose "{color: 'red'}"

All launch options:
ros2 launch bph_statemachine demo.launch.py \
  use_sim_time:=false \
  params_file:=<path_to_nav_to_goal>/config/nav2_params.yaml \
  slam_params_file:=<path_to_nav_to_goal>/config/slam_params.yaml \
  depth_image_topic:=/camera/depth/image_raw \
  depth_info_topic:=/camera/depth/camera_info \
  robot_base_frame:=base_link \
  cam_x:=0.0 \
  cam_y:=0.0 \
  cam_z:=2.5 \
  cam_roll:=0.0 \
  cam_pitch:=1.5707963 \
  cam_yaw:=0.0 \
  robot_ip:=10.3.4.10 \
  launch_rviz:=false \
  urdf_path:=/home/katallen/workspace/src/springcontroller/springcontroller/flat_urdf_files/ceeorobot_flat.urdf \
  joint_order:=[elbow_joint,shoulder_lift_joint,shoulder_pan_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint] \
  torque_topic:=/virtual_spring_node/joint_torques \
  command_topic:=/forward_effort_controller/commands \
  color_image_topic:=/bph_overhead_camera/image_raw \
  cam_info_topic:=/bph_overhead_camera/camera_info \
  target_color:=red \
  object_height_m:=0.0 \
  min_contour_area:=500 \
  detection_rate_hz:=10.0

