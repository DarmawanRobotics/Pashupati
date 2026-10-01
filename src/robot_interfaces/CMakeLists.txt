cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SectorScan.msg"
  "msg/AvoidanceCommand.msg"
  "msg/Waypoint.msg"
  "msg/WaypointPath.msg"
  "msg/NavigationStatus.msg"
  "msg/MissionStatus.msg"
  "msg/StopPointEvent.msg"
  "srv/MarkStopPoint.srv"
  "srv/LoadPath.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()