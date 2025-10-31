cmake_minimum_required(VERSION 3.5)
project(mission_fsm)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(mission_fsm_node src/mission_fsm_node.cpp)
ament_target_dependencies(mission_fsm_node
  rclcpp
  std_msgs
  nav_msgs
  std_srvs
)

install(TARGETS
  mission_fsm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
