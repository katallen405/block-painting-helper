# block-painting-helper
A ros2 multimodal system for helping the user paint on wooden blocks or do other close-quarters manipulation tasks where an extra hand would be useful

Includes ros packages:
- bph_pickmeup:  a MoveIt package for picking up an object and moving it into the user's workspace
- person_finder:  a node using OpenCV and YOLO to find people in the room who might need to be avoided by either the Turtlebot or the manipulator arm
- nav_to_goal: a Nav2 node for moving around the room avoiding obstacles to retrieve the desired object 
- bph_statemachine: a SMACH package for high-level control of the system and triggering the other nodes

Not included in this repository but used here:
- springcontroller: a platform-independent ROS node for torque control of a robot arm using virtual springs for constraints on the arm's position in the user's workspace
https://github.com/katallen405/springcontroller

# Installation details:
to run the person_tracker node, you need a virtual environment (venv) created with --system-site-packages
Inside the venv, you need to 
pip install ultralytics 
pip uninstall opencv-python
pip install "numpy<2"

In each terminal or in your .bashrc:
export PYTHONPATH=$VIRTUAL_ENV/lib/python3.12/site-packages:$PYTHONPATH
(this allows ROS to use the venv)

STATUS:
- 23 March 2026:  initial implementation 2f38d12
- 30 March 2026:  MoveIt and NavStack assignment b7e4767
- 6 March 2026:  Perception Assignment 19c79af (including venv instructions)
- 22 April 2026: SMACH assignment 9d1622c

