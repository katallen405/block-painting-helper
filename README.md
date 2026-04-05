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



STATUS:
- 23 March 2026:  initial implementation 2f38d12
- 30 March 2026:  MoveIt and NavStack assignment b7e4767 

