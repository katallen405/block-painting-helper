# block-painting-helper
A ros2 multimodal system for helping the user paint on wooden blocks or do other close-quarters manipulation tasks where an extra hand would be useful

Includes ros packages:
- bph_pickmeup:  a MoveIt package for picking up an object and moving it into the user's workspace
- person_finder:  a node using OpenCV and YOLO to find people in the room who might need to be avoided by either the Turtlebot or the manipulator arm
- nav_to_goal: a Nav2 node for moving around the room avoiding obstacles to retrieve the desired object 
- bph_statemachine: a SMACH package for high-level control of the system and triggering the other nodes
- 

Not included in this repository but used here:
- springcontroller: a platform-independent ROS node for torque control of a robot arm using virtual springs for constraints on the arm's position in the user's workspace
https://github.com/katallen405/springcontroller

In this repository but not run here:
- turtlebot_bringup.launch.py
	This brings up a rosbridge server, the kobuki node, openi2_camera,
	static transforms for the openni_depth_optical_frame and
	openni_rgb_optical_frame, and depthimage_to_laserscan.
	It gets copied to the turtlebot and run with
	ros2 launch turtlebot_bringup.launch.py (not in a package, to disrupt
	the turtlebot ecosystem as little as possible)


# Installation details:
  to run the person_tracker node and rosbridge, you need a virtual environment (venv) created with --system-site-packages
  python3 -m venv --system-site-packages ~/.ros_venv
  source ~/.ros_venv/bin/activate

  Inside the venv, you need to 
  pip install ultralytics 
  pip uninstall opencv-python
  pip install "numpy<2"
  pip install roslibpy

  In each terminal or in your .bashrc:
  export PYTHONPATH=$VIRTUAL_ENV/lib/python3.12/site-packages:$PYTHONPATH
  (this allows ROS to use the venv)
  (This also goes in the launch file, but is needed for debug)

# Running details
  ssh into the Turtlebot laptop FROM THE MAIN ROS MACHINE:
  cd into wherever you copied the bringup script and run:
  ros2 launch turtlebot_bringup.launch.py

On your machine local to the camera:
ros2 run v4l2_camera v4l2_camera_node  --ros-args -r image_raw:=/bph_overhead_camera/image_raw -r video_device:='/dev/video0'

On your static computer:
    source ~/.ros_venv/bin/activate
To enable rosbridge safely across the local network:
     ssh -L 9090:localhost:9090 baymax@10.5.10.74 (check this IP)
  To start everything else:
     ros2 launch bph_statemachine demo.launch.py




STATUS:
- 23 March 2026:  initial implementation 2f38d12
- 30 March 2026:  MoveIt and NavStack assignment b7e4767
- 6 March 2026:  Perception Assignment 19c79af (including venv instructions)
- 22 April 2026: SMACH assignment 9d1622c


Full launch:
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

