cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SectorScan.msg"
  "msg/AvoidanceCommand.msg"
  "msg/Waypoint.msg"
  "msg/WaypointPath.msg"
  "msg/NavigationStatus.msg"
  "msg/MissionStatus.msg"
  "srv/MarkStopPoint.srv"
  "srv/LoadPath.srv"
  "action/NavigateRoute.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_package()