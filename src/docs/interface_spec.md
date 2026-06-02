Multi-Robot Mapping Challenge Interface Spec

Robot Namespaces
- Robot 1: /robot_1
- Robot 2: /robot_2

Allowed Topics (Per Robot)
- Odometry: /robot_[id]/odom [nav_msgs/msg/Odometry] (Read only) 
- LiDAR Scan: /robot_[id]/scan [sensor_msgs/msg/LaserScan] (Read only)
- Velocity Commands: /robot_[id]/cmd_vel [geometry_msgs/msg/Twist] (Write only)

Target Output Topic
- Global Map: /global_map [nav_msgs/msg/OccupancyGrid]

Forbidden Ground-Truth Topics
- /world/*
- /gazebo/*
- /model_states
- Any simulator internal state APIs
