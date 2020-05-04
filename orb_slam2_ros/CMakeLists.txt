cmake_minimum_required(VERSION 2.8)
project(orb_slam2_ros)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

#test
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O0 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O0 -march=native")


ADD_DEFINITIONS("-std=c++11")

find_package(OpenCV 3.1.0 REQUIRED)
#find_package(Pangolin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  orb_slam2_lib
  roscpp
  tf
  std_msgs
  sensor_msgs
  image_transport
  image_geometry
  cv_bridge
  pcl_ros
  pcl_conversions
  message_generation
  octomap_ros
)

# Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  ORBState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(
  ${roscpp_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/../orb_slam2_lib
  ${PROJECT_SOURCE_DIR}/../orb_slam2_lib/include

)
set(LIBS
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

# Node for monocular camera
add_executable(Mono
  src/ros_mono.cc
  src/ROSPublisher.cpp
)

add_dependencies(Mono ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(Mono
  ${LIBS}
)

# Node for monocular camera (Augmented Reality Demo)
#add_executable(MonoAR
#src/AR/ros_mono_ar.cc
#src/AR/ViewerAR.h
#src/AR/ViewerAR.cc
#)
#
#target_link_libraries(MonoAR
#${LIBS}
#)

# Node for stereo camera
add_executable(Stereo
  src/ros_stereo.cc
  src/ROSPublisher.cpp
)

add_dependencies(Stereo ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(Stereo
  ${LIBS}
)

# Node for RGB-D camera
add_executable(RGBD
  src/ros_rgbd.cc
  src/ROSPublisher.cpp
)

add_dependencies(Stereo ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(RGBD
  ${LIBS}
)


