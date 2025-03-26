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
# find_package(tf2_msgs REQUIRED)
# find_package(tf2_eigen REQUIRED)
# find_package(pcl_conversions REQUIRED)
# find_package(pcl_ros REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(nav2_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Find libraries
include_directories(include)

# Library
# add_library(${PROJECT_NAME} SHARED
#   src/hsr_sim_joint_action_server.cpp
#   src/hsr_sim_wheel_action_server.cpp
# )


# rclcpp_components_register_node(${PROJECT_NAME}
#   PLUGIN "hsr_sim::JointActionServer"
#   EXECUTABLE joint_action_server)

# rclcpp_components_register_node(${PROJECT_NAME}
#   PLUGIN "hsr_sim::WheelActionServer"
#   EXECUTABLE wheel_action_server)


# ament_target_dependencies(${PROJECT_NAME}
#   "rclcpp"
#   "rclcpp_action"
#   "rclcpp_components"
#   "sobits_interfaces"
#   "std_msgs"
#   "sensor_msgs"
#   "geometry_msgs"
#   "nav_msgs"
#   "trajectory_msgs"
#   "tf2_geometry_msgs"
#   "tf2_ros"
#   "tf2")


# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)


#############
## Install ##
#############

install(DIRECTORY
  config
  launch DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

# add_executable(handyman_pose_saver src/handyman_pose_saver.cpp)
# ament_target_dependencies(handyman_pose_saver
#   rclcpp
#   tf2
#   tf2_ros
#   tf2_msgs
#   geometry_msgs
# )
# install(TARGETS
#   handyman_pose_saver
#   DESTINATION lib/${PROJECT_NAME}
# )

# add_executable(pcl_scan_creater src/pcl_scan_creater.cpp)
# ament_target_dependencies(pcl_scan_creater
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   tf2
#   tf2_ros
#   tf2_eigen
#   pcl_conversions
#   pcl_ros
# )

# target_include_directories(pcl_scan_creater PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${PCL_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIR}
# )

# target_link_libraries(pcl_scan_creater
#   ${PCL_LIBRARIES}
# )

# install(TARGETS pcl_scan_creater
#   DESTINATION lib/${PROJECT_NAME}
# )

# add_executable(pose_saver src/pose_saver.cpp)
# ament_target_dependencies(pose_saver
#   rclcpp
#   tf2
#   tf2_ros
#   geometry_msgs
# )

# install(TARGETS
#   pose_saver
#   DESTINATION lib/${PROJECT_NAME}
# )

# add_executable(sub_obj_depth src/sub_obj_depth.cpp)
# ament_target_dependencies(sub_obj_depth
#   rclcpp
#   tf2
#   tf2_ros
#   tf2_eigen
#   tf2_geometry_msgs
#   sensor_msgs
#   pcl_conversions
#   PCL
#   hsr_sim_common_interfaces
# )

# target_include_directories(sub_obj_depth PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${PCL_INCLUDE_DIRS}
# )

# target_link_libraries(sub_obj_depth
#   ${PCL_LIBRARIES}
# )

# install(TARGETS
#   sub_obj_depth
#   DESTINATION lib/${PROJECT_NAME}
# )

# add_executable(waypoint_nav src/waypoint_nav.cpp)

# ament_target_dependencies(waypoint_nav
#   rclcpp
#   rclcpp_action
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   geometry_msgs
#   std_msgs
#   visualization_msgs
#   hsr_sim_common_interfaces     # provides waypoint_nav.srv, location_stock.msg
#   nav2_msgs     # provides nav2_msgs::action::MoveBase
# )

# install(TARGETS
#   waypoint_nav
#   DESTINATION lib/${PROJECT_NAME}
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
