cmake_minimum_required(VERSION 3.5)
project(px4_offboard)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geodesy REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

add_executable(
  ${PROJECT_NAME}_node
  src/offboard_control_node.cpp
  src/offboardControl.cpp
  include/offboardControl.hpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_node
    rclcpp
    mavros_msgs
    sensor_msgs
    geometry_msgs
    geodesy
    geographic_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    GeographicLib
    nav_msgs
  )

target_include_directories(
    ${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY param
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
