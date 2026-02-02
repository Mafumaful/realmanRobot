cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Find orocos-kdl via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(orocos_kdl REQUIRED orocos-kdl)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(arm_controller_node
  src/arm_controller_node.cpp
  src/kinematics_solver.cpp
  src/cubic_spline.cpp
  src/kdl_parser.cpp
)

ament_target_dependencies(arm_controller_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  urdf
  tf2
  tf2_ros
)

target_link_libraries(arm_controller_node
  ${orocos_kdl_LIBRARIES}
)

install(TARGETS
  arm_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
