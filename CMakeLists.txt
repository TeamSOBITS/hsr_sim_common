cmake_minimum_required(VERSION 3.8)
project(hsr_sim_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sobits_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Find libraries
include_directories(include)

# Library
add_library(${PROJECT_NAME} SHARED
  src/hsr_sim_joint_action_server.cpp
  src/hsr_sim_wheel_action_server.cpp
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "hsr_sim::JointActionServer"
  EXECUTABLE joint_action_server)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "hsr_sim::WheelActionServer"
  EXECUTABLE wheel_action_server)


ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "sobits_interfaces"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "trajectory_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
  "tf2")
  
add_executable(hsr_sim_teleop_key src/hsr_sim_teleop_key.cpp)
target_link_libraries(hsr_sim_teleop_key ${PROJECT_NAME})
ament_target_dependencies(hsr_sim_teleop_key
  rclcpp
  geometry_msgs
  trajectory_msgs
)

install(TARGETS hsr_sim_teleop_key
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


#############
## Install ##
#############

install(DIRECTORY
  config
  launch 
  rviz DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)



ament_package()
