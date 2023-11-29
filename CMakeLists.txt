cmake_minimum_required(VERSION 3.5)
project(imu_estimator)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(imu_estimator
  src/main.cpp
)

ament_target_dependencies(imu_estimator
  rclcpp
  std_msgs
  tf2_eigen
  tf2_ros
  sensor_msgs
  tf2_sensor_msgs
  tf2
  tf2_geometry_msgs
)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS
  imu_estimator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
