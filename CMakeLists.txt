cmake_minimum_required(VERSION 3.8)
project(robot_arm_control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)
#find_package(Eigen3 REQUIRED)


set(dependencies
  rclcpp
  std_msgs
  orocos_kdl_vendor
  kdl_parser
  urdf
  std_srvs
  sensor_msgs
  ros_gz_interfaces
  controller_manager_msgs
)

add_library(kdl_ros_control_lib
  src/kdl_control.cpp
  src/kdl_planner.cpp
  src/kdl_robot.cpp
)
ament_target_dependencies(kdl_ros_control_lib ${dependencies})

add_executable(kdl_trajectory_test src/kdl_trajectory_test.cpp)
target_link_libraries(kdl_trajectory_test kdl_ros_control_lib)
ament_target_dependencies(kdl_trajectory_test ${dependencies}) 

add_executable(kdl_package_testing src/kdl_package_testing.cpp)
target_link_libraries(kdl_package_testing kdl_ros_control_lib)
ament_target_dependencies(kdl_package_testing ${dependencies}) 

target_include_directories(kdl_ros_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(kin_inv_test src/kin_inv_test.cpp)
target_link_libraries(kin_inv_test kdl_ros_control_lib)
ament_target_dependencies(kin_inv_test ${dependencies}) 

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

install(TARGETS
  kdl_ros_control_lib
  kdl_package_testing
  kdl_trajectory_test
  kin_inv_test
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/kdl_ros_control/
  DESTINATION include/kdl_ros_control
)

install(
  DIRECTORY
    launch
    urdf
    meshes
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
