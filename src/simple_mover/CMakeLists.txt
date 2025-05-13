cmake_minimum_required(VERSION 3.8)
project(simple_mover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

# -------------------------
# Build the original example
# -------------------------
add_executable(hello_moveit src/main.cpp)
ament_target_dependencies(
  hello_moveit
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# -------------------------
# Build the new parameterized node
# -------------------------
add_executable(mover_auto src/main2.cpp)
ament_target_dependencies(
  mover_auto
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# Install executables
install(
  TARGETS hello_moveit mover_auto
  DESTINATION lib/${PROJECT_NAME}
)

# ament package macro
ament_package()
