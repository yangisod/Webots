cmake_minimum_required(VERSION 3.0.2)
project(finallasermapping)
# add_compile_options(-std=c++11)
find_package(OpenMP QUIET)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

set(octomap_DIR "/opt/ros/noetic/share/octomap")
find_package(catkin REQUIRED COMPONENTS
  message_filters
  message_generation  
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  std_msgs
  pcl_ros
  tf
  pcl_conversions
  cv_bridge
  laser_geometry
  octomap
  octomap_msgs
  octomap_ros
  )

find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(OpenCV 3.4 REQUIRED)
find_package(Ceres REQUIRED)


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs message_runtime octomap_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS 

)
include_directories(
  include	
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

message(STATUS "******OpenCV_VERSION: ${OpenCV_VERSION}*******") 


#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "/home/yangod/livox_mapping/build/")

add_executable(finalmapping src/finalmapping.cpp)
target_link_libraries(finalmapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBRARIES} -fopenmp)

add_executable(mapping src/mapping.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES} ${CERES_LIBRARIES})
