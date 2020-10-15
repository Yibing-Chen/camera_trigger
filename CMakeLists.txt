cmake_minimum_required(VERSION 2.8.3)
project(camera_trigger)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy cv_bridge OpenCV image_transport)
## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/image_publisher.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/image_viewer.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)

add_executable(trigger src/trigger.cpp)
target_link_libraries(trigger ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)