cmake_minimum_required(VERSION 2.8.3)
project(usb_sensor_message)

## Find catkin macros and libraries

find_package(catkin  REQUIRED COMPONENTS roscpp sensor_msgs std_msgs)
find_package(cmake_modules REQUIRED)
find_package(HIDAPI REQUIRED)


## Declare a catkin package
catkin_package()

## Include directories,
## Add target and link libraries

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${HIDAPI_INCLUDE_DIRS})
  

add_executable(usb_sensor_message_node src/usb_sensor_message.cpp)
target_link_libraries(usb_sensor_message_node ${catkin_LIBRARIES}   ${HIDAPI_LIBRARIES})
