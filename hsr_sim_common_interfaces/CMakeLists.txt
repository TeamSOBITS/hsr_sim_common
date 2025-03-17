cmake_minimum_required(VERSION 3.8)
project(hsr_sim_common_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(msg_files
  "msg/LocationStock.msg"
)

set(srv_files
  "srv/ArmHeight.srv"
  "srv/DetectCtrl.srv"
  "srv/GraspCtrl.srv"
  "srv/GripperCtrl.srv"
  "srv/GripperMove.srv"
  "srv/IsGrasped.srv"
  "srv/ObjDepth.srv"
  "srv/OdomBase.srv"
  "srv/PutCtrl.srv"
  "srv/RobotMotion.srv"
  "srv/WatchMotion.srv"
  "srv/WaypointNav.srv"
)

set(action_files
  # "action/AddTwoInts.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES geometry_msgs 
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
