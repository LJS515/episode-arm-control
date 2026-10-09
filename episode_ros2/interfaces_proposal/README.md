# Proposed custom interfaces (not built)

The official ROS2 package ships its own `AngleMode`, `MoveXyzRotation` and
`LinearMove` actions and `ReadMotorAngles`, `GripperControl`, `ServoGripper`,
`SetFreeMode` services (evidence B). Their `.action/.srv` field definitions
were not available to this project, so the interface node in this repository
uses only standard interfaces (`control_msgs/FollowJointTrajectory`,
`std_srvs/Trigger`, `std_srvs/SetBool`).

`MovePose.action` sketches what a Cartesian action would look like in an
`episode_interfaces` package. Before writing it for real, compare with the
official definitions so that existing official clients keep working
(an adapter, not a replacement).
