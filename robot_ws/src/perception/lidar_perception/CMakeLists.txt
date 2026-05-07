cmake_minimum_required(VERSION 3.8)
project(lidar_perception)              # package 이름 확인

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# find dependencies : 필요한 패키지 추가
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search segmentation)

find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# 실행 파일 생성
# 전처리 노드
add_executable(pointcloud_preprocess_node
  src/pointcloud_preprocess_node.cpp
)

# 클러스터화 노드
add_executable(obstacle_cluster_node
  src/obstacle_cluster_node.cpp
)

# Obstacle Model 노드
add_executable(obstacle_model_node
  src/obstacle_model_node.cpp
)

# local occupancy grid 노드
add_executable(local_occupancy_grid_node 
src/local_occupancy_grid_node.cpp
)

# free space model 노드
add_executable(free_space_model_node src/free_space_model_node.cpp)

# free space 시각화 노드
add_executable(free_space_marker_node src/free_space_marker_node.cpp)

# include/ 폴더 안의 헤더를 찾을 수 있도록 설정
# target_include_directories(obstacle_model_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# 전처리 노드
target_include_directories(pointcloud_preprocess_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 클러스터화 노드
target_include_directories(obstacle_cluster_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Obstacle Model 노드
target_include_directories(obstacle_model_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# local occupancy grid 노드
target_include_directories(local_occupancy_grid_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# free space model 노드
target_include_directories(free_space_model_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# free space 시각화 노드
target_include_directories(free_space_marker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 라이브러리 링크
# 전처리 노드
ament_target_dependencies(pointcloud_preprocess_node
  rclcpp
  sensor_msgs
  std_msgs
  robot_interfaces
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)

# 클러스터화 노드
ament_target_dependencies(obstacle_cluster_node
  rclcpp
  sensor_msgs
  std_msgs
  robot_interfaces
  pcl_conversions
)

# Obstacle Model 노드
ament_target_dependencies(obstacle_model_node
  rclcpp
  robot_interfaces
)

# local occupancy grid 노드
ament_target_dependencies(local_occupancy_grid_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
)

# free space model 노드
ament_target_dependencies(free_space_model_node
  rclcpp
  nav_msgs
  robot_interfaces
)

# # free space 시각화 노드
ament_target_dependencies(free_space_marker_node
  rclcpp
  visualization_msgs
  geometry_msgs
  robot_interfaces
)

# 전처리 노드
target_link_libraries(pointcloud_preprocess_node
  ${PCL_LIBRARIES}
)

# 클러스터화 노드
target_link_libraries(obstacle_cluster_node
  ${PCL_LIBRARIES}
)

# Obstacle Model 노드
target_link_libraries(obstacle_model_node
  ${PCL_LIBRARIES}
)

# local occupancy grid 노드
target_link_libraries(local_occupancy_grid_node
  ${PCL_LIBRARIES}
)

# 실행 파일 설치
# install(TARGETS
#   obstacle_model_node
#   DESTINATION lib/${PROJECT_NAME}
# )
install(TARGETS
  pointcloud_preprocess_node
  obstacle_cluster_node
  obstacle_model_node
  local_occupancy_grid_node
  free_space_model_node
  free_space_marker_node
  DESTINATION lib/${PROJECT_NAME}
)

# include/ 폴더 설치
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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
