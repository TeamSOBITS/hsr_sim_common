cmake_minimum_required(VERSION 2.8.3)
project(hsr_sim_common)

add_compile_options(-std=c++14 -O3 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  pcl_conversions
  pcl_ros
  tf
  tf2
  tf2_ros
  message_generation
  sensor_msgs
)

add_message_files(
 FILES
 location_stock.msg
)

add_service_files(
 FILES
 odom_base.srv
 grasp_ctrl.srv
 waypoint_nav.srv
 robot_motion.srv
 arm_height.srv
 put_ctrl.srv
 detect_ctrl.srv
 watch_motion.srv
 gripper_move.srv
 gripper_ctrl.srv
 is_grasped.srv
 obj_depth.srv
)

generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_saver_hsr src/pose_saver.cpp)
target_link_libraries(pose_saver_hsr ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(waypoint_nav_hsr src/waypoint_nav.cpp)
target_link_libraries(waypoint_nav_hsr ${catkin_LIBRARIES})
add_dependencies(waypoint_nav_hsr ${PROJECT_NAME}_generate_messages_cpp)

add_executable(sub_obj_depth src/sub_obj_depth.cpp)
target_link_libraries(sub_obj_depth ${catkin_LIBRARIES})
