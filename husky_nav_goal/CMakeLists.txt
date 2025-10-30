cmake_minimum_required(VERSION 3.8)
project(husky_nav_goal)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Add executable
add_executable(goals_markers src/goals_markers.cpp)
ament_target_dependencies(goals_markers rclcpp nav_msgs visualization_msgs)

add_executable(tsp_clearance_goals src/tsp_clearance_goals.cpp)
ament_target_dependencies(tsp_clearance_goals rclcpp rclcpp_action nav_msgs geometry_msgs nav2_msgs)

add_executable(goal_nav src/goal_nav.cpp)
ament_target_dependencies(goal_nav ament_index_cpp rclcpp rclcpp_action nav_msgs nav2_msgs std_srvs)

add_executable(goal_tracker src/goal_tracker.cpp)
ament_target_dependencies(goal_tracker rclcpp geometry_msgs std_msgs nav_msgs)


# Install executable
install(TARGETS
  goals_markers
  tsp_clearance_goals
  goal_nav
  goal_tracker
  DESTINATION lib/${PROJECT_NAME}
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()