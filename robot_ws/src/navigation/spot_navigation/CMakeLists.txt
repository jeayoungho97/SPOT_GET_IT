cmake_minimum_required(VERSION 3.8)
project(spot_navigation)

# ============================================================
# Compile options
# ============================================================
# -Wall      : 일반적인 경고 출력
# -Wextra    : 추가 경고 출력
# -Wpedantic : 표준 C++ 관점에서 엄격한 경고 출력
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Package dependencies
# - ament_cmake: ROS 2 CMake 기반 패키지 빌드 시스템
# - rclcpp: ROS 2 C++ 노드 작성용 클라이언트 라이브러리
# - robot_interfaces: 커스텀 메시지 인터페이스 패키지
# - nav_msgs: local_path를 nav_msgs/msg/Path publish를 위해 필요
# - geometry_msgs: geometry_msgs/msg/TransformStamped, Quaternion 사용
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

# ============================================================
# [1] mock_global_path_publisher_node
# ------------------------------------------------------------
# - 역할:
#   실제 planning/global_path_manager가 publish하는
#   /planning/global_path/spot_01 토픽을 mock 데이터로 재현한다.
# - Input : 없음
# - Output : /planning/global_path/spot_01
#     Type: robot_interfaces/msg/GlobalPathWaypoints
# ============================================================
add_executable(mock_global_path_publisher_node
  src/mock_global_path_publisher_node.cpp
)

target_include_directories(mock_global_path_publisher_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(mock_global_path_publisher_node
  rclcpp
  std_msgs
  geometry_msgs
  robot_interfaces
)

# 추가 노드 구현 후 add_executable(), ament_target_dependencies() 추가


# ============================================================
# Install targets
# ------------------------------------------------------------
# install/lib/spot_navigation/ 아래에 실행 파일 설치
# 추후 노드 구현 후에 개발한 노드 추가하기
# ============================================================
install(TARGETS
  mock_global_path_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# config / launch 디렉토리 설치
# ------------------------------------------------------------
# launch 파일에서 FindPackageShare("spot_navigation")로
# config/*.yaml 파일을 찾기 위해 반드시 설치해야 한다.
# ============================================================
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# include 폴더 설치
# 나중에 다른 패키지나 테스트 코드에서 헤더를 참조할 수 있게 하기 위함
install(DIRECTORY
  include/
  DESTINATION include/
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
