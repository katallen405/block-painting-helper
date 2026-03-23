# block-painting-helper
A ros2 multimodal system for helping the user paint on wooden blocks

Includes ros packages:
bph-pickmeup:  a MoveIt package for picking up an object and moving it into the user's workspace

block-painting-helper: a high-level node to implement state machine control for the other nodes

bph-perception: a perception tool for identifying the desired object so that it can be manipulated

bph-navigation: a Nav2 node for moving around the room avoiding obstacles to retrieve the desired object


Not included in this repository but used here:
springcontroller: a platform-independent ROS node for torque control of a robot arm using virtual springs for constraints on the arm's position in the user's workspace
https://github.com/katallen405/springcontroller



STATUS:
23 March 2026:  initial implementation 
