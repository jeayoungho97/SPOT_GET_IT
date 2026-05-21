cmake_minimum_required(VERSION 3.8)
project(motion_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(stand_motion_node
  src/stand_motion_node.cpp
)

target_compile_features(stand_motion_node PUBLIC cxx_std_17)

ament_target_dependencies(stand_motion_node
  rclcpp
  std_msgs
  robot_interfaces
)

add_executable(joint_target_mux_node
  src/joint_target_mux_node.cpp
)

target_compile_features(joint_target_mux_node PUBLIC cxx_std_17)

ament_target_dependencies(joint_target_mux_node
  rclcpp
  std_msgs
  robot_interfaces
)

install(TARGETS
  stand_motion_node
  joint_target_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()