Step through the state machine:

Go get object:
ros2 topic pub --once /button std_msgs/msg/String "{data: go}"

Got to location:
ros2 topic pub --once /turtlebot/location std_msgs/String "data: 'supply closet'"
Got object at supply closet:
ros2 topic pub --once /button std_msgs/msg/String "{data: go}"

Got back to home:
ros2 topic pub --once /turtlebot/location std_msgs/String "data: 'home'"

Located the object: (this should change to a location in the world frame)
ros2 topic pub --once /turtlebot/location std_msgs/String "data: 'anything'"

ros2 topic pub --once /object_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {
    position: {x: 1.0, y: 2.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"

LocatingObjectAndPeople -> PickAndPlace             (on /object/location message)
    PickAndPlace            -> Grasping                 (when /arm/location == "grasp position")
    Grasping                -> MoveToWorkspace          (on /button message, simulating successful grasp)
    MoveToWorkspace         -> SpringController         (when /arm/location == "in workspace")

ros2 topic pub --once /
