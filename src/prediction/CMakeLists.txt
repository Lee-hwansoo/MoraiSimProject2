cmake_minimum_required(VERSION 3.5)
project(prediction)

SET(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(
	FILES
  PredictedObjectPath.msg
  PredictedObjectPathList.msg
  TrackedObjectPose.msg
  TrackedObjectPoseList.msg
  TrackedPoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES modules
  CATKIN_DEPENDS message_runtime rospy roscpp std_msgs
  DEPENDS OpenCV Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Declare a C++ library
add_library(modules
  src/state_models.cpp
  src/state_filters.cpp
  src/object_tracker.cpp
)

add_executable(test_node src/test.cpp)

target_link_libraries(test_node
  modules
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(object_tracker_node src/object_tracker.cpp)

target_link_libraries(object_tracker_node
  modules
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
