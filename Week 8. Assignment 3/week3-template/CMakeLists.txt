cmake_minimum_required(VERSION 3.10)
project(template VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## packages ##
find_package(OpenCV 4 REQUIRED)

find_package(catkin REQUIRED 
  rospy
  roscpp
  roslib      # evaluation of the ROS-package path 
  cv_bridge   # conversion between OpenCV and ROS-messages 
  sensor_msgs # Image ROS-message
  message_generation
)

add_message_files(
  FILES
  MotionDetection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## catkin specific configuration ##
catkin_package(CATKIN_DEPENDS message_runtime)

## build ##
add_executable(subscriber nodes/subscriber.cpp)

target_include_directories(subscriber PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(subscriber PRIVATE ${catkin_LIBRARIES} ${OpenCV_LIBS})

catkin_install_python(PROGRAMS
  nodes/image_converter.py
  nodes/motion_detector.py
  nodes/color_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)