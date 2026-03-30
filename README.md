# block-painting-helper
A ros2 multimodal system for helping the user paint on wooden blocks

Includes ros packages:
- bph_pickmeup:  a MoveIt package for picking up an object and moving it into the user's workspace
- bph_perception: a perception tool for identifying the desired object so that it can be manipulated
- nav_to_goal: a Nav2 node for moving around the room avoiding obstacles to retrieve the desired object (with help from Claude Sonnet 4.6)
- bph_statemachine: a SMACH package for high-level control of the system and triggering the other nodes

Not included in this repository but used here:
- springcontroller: a platform-independent ROS node for torque control of a robot arm using virtual springs for constraints on the arm's position in the user's workspace
https://github.com/katallen405/springcontroller



STATUS:
- 23 March 2026:  initial implementation 2f38d12
- 30 March 2026:  MoveIt and NavStack assignment b7e4767 

