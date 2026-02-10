cmake_minimum_required(VERSION 3.0.2)
project(voxel_slam)

# Build configuration
set(CMAKE_BUILD_TYPE "RelWithDebInfo")
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# C++ standard and compile flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-std=c++17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")

# Dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nodelet
  std_msgs
  pcl_conversions
  pcl_ros
  livox_ros_driver2
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED QUIET)
# find_package(Ceres REQUIRED)

# Messages
file(GLOB MSG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg)
foreach(MSG IN ITEMS ${MSG_FILES})
  get_filename_component(MSG_FILENAME ${MSG} NAME)
  set(MSG_FILENAMES ${MSG_FILENAMES} ${MSG_FILENAME})
endforeach()

add_message_files(
  DIRECTORY msg
  FILES ${MSG_FILENAMES}
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

# Catkin package
catkin_package(
  CATKIN_DEPENDS geometry_msgs nodelet nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL GTSAM
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/voxelslam.cpp
  src/BTC.cpp
  src/voxel_slam_nodelet.cpp
)

# Ensure the produced library name matches nodelet_plugins.xml (libvoxel_slam.so)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "voxel_slam")

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_nodelet
  src/voxel_slam_nodelet.cpp
)

add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME})

