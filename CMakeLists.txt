cmake_minimum_required(VERSION 3.8)
project(estimation)

# Find required ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add include directories
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Add executable
add_executable(estimator src/estimator.cpp)

# Link dependencies
ament_target_dependencies(estimator rclcpp sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

# Ensure include directories are correctly set for this target
target_include_directories(estimator PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

# Install the executable
install(TARGETS estimator
  DESTINATION lib/${PROJECT_NAME}
)

# Enable ament package
ament_package()
