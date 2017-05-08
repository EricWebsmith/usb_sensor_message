# Read Me

This project read IMU data from the usb port and generates ROS IMU messages. HIDAPI is used to fetch data.

To use HIDAPI, put FindHIDAPI.cmake to /your_workspace/src/cmake_modules/cmake/Modules

## commands:

source ~/ros_tutorials_ws/devel/setup.bash
rosrun usb_sensor_message usb_sensor_message_node
