cmake_minimum_required(VERSION 3.8)
project(planting_conditions)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(soil_prep_fsm src/soil_prep_fsm.cpp)
ament_target_dependencies(soil_prep_fsm rclcpp std_msgs std_srvs geometry_msgs)

add_executable(water_level_sim src/water_level_sim.cpp)
ament_target_dependencies(water_level_sim rclcpp std_msgs std_srvs)

add_executable(planting_sim src/planting_sim.cpp)
ament_target_dependencies(planting_sim rclcpp std_msgs std_srvs geometry_msgs nav_msgs)

add_executable(cover_sim src/cover_sim.cpp)
ament_target_dependencies(cover_sim rclcpp std_msgs std_srvs geometry_msgs nav_msgs)

add_executable(irrigation_sim src/irrigation_sim.cpp)
ament_target_dependencies(irrigation_sim rclcpp std_msgs std_srvs geometry_msgs nav_msgs)

add_executable(planting_site_markers src/planting_site_markers.cpp)
ament_target_dependencies(planting_site_markers rclcpp std_msgs geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(seed_weight src/seed_weight.cpp)
ament_target_dependencies(seed_weight rclcpp std_msgs std_srvs geometry_msgs)

#chmod +x src/tcp_to_ros2.py
#chmod +x src/task_order.py
#chmod +x src/tcp_to_ros2_sunlight.py
#chmod +x src/planting_pipeline.py
#chmod +x src/tcp_to_ros2_both.py

install(TARGETS
  water_level_sim
  seed_weight
  soil_prep_fsm
  planting_sim           
  cover_sim              
  irrigation_sim 
  planting_site_markers  
  DESTINATION lib/${PROJECT_NAME})

  install(PROGRAMS
  src/planting_pipeline.py
  src/tcp_to_ros2_both.py
  DESTINATION lib/${PROJECT_NAME}
)
  
foreach(dir config launch models urdf worlds)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()


