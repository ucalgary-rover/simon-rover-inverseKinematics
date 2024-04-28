cmake_minimum_required(VERSION 3.5)
project(robot_lidar_control)

# Find rclcpp and other ROS 2 packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)  # Adjust this if necessary

# Include ROS 2 headers
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})  # Adjust this if necessary

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Find TBB
find_package(TBB REQUIRED)
include_directories(${TBB_INCLUDE_DIRS})

# Build our plugin
add_library(velodyne_plugin SHARED velodyne_plugin.cc)
target_link_libraries(velodyne_plugin ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES} ${TBB_LIBRARIES})  # Use rclcpp libraries

# Build the stand-alone test program
add_executable(vel vel.cc)

if (${gazebo_VERSION_MAJOR} LESS 6)
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(vel ${GAZEBO_LIBRARIES} -ltbb)  # Directly link TBB library here
endif()

