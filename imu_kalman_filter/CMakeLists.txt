cmake_minimum_required(VERSION 3.8)
project(imu_kalman_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)  # <<== ADD here!!

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(imu_kalman_filter_node
  src/imu_kalman_filter_node.cpp
  src/kalman_filter.cpp
)

ament_target_dependencies(imu_kalman_filter_node
  rclcpp
  sensor_msgs
  Eigen3
)

install(TARGETS
  imu_kalman_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(imu_visualizer_node
  src/imu_visualizer_node.cpp
)

ament_target_dependencies(imu_visualizer_node
  rclcpp
  sensor_msgs
  visualization_msgs
)

install(TARGETS
  imu_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

