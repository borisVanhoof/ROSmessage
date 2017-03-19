cmake_minimum_required(VERSION 2.8.3)
project(talker)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  message_generation
  cv_bridge
  image_transport
  geometry_msgs
)

FIND_PACKAGE(OpenCV REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -march=core-avx-i -O2") 

## Declare ROS messages and services
#add_message_files(FILES Num.msg)

## Generate added messages and services
generate_messages(
  DEPENDENCIES 
  std_msgs
)

LIST(APPEND CMAKE_MODULE_PATH /usr/local/lib/cmake)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)
