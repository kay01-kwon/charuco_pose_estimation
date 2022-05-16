cmake_minimum_required(VERSION 2.8.3)
project(opencv_pose_estimation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES opencv_pose_estimation
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs image_transport cv_bridge
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(chessboard_tracker src/main_chessboard_tracker.cpp include/opencv_pose_estimation/chessboard_tracker.cpp)
target_link_libraries(chessboard_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(charuco_tracker src/main_charuco_tracker.cpp include/opencv_pose_estimation/charuco_tracker.cpp)
target_link_libraries(charuco_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})