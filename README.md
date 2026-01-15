# OBSBOT Tiny 2 Lite Controller

## What it has
- **main.cpp**[ros2 run joy_ros_client joy_ros_client] : this will subscribe to "/camera_angle" and access the indices __tilt_index__ and __pan_index__, and then formats that data through a TCP client to the port "3490" of "0.0.0.0". All these values can be changed easily in the main.hpp file

- **gimblectl/camctl2.c** [ ros2 run joy_ros_client gimblectl ] : this is a TCP server that directly interfaces with the Obsbot tiny 2 lite camera and controls its pan and tilt through the commands you feed to it through TCP. The format of the command is {x : NNNN,y : NNNN}. x and y range from 0 to 4095 (12 bit resolution, matches ESP32)

- **gimblectl/serial_client.c** [ros2 run joy_ros_client serial_client] : this file accesses serial directly and does what main.cpp would do. 


> NOTE : all parameters that might need changing are mentioned in #defined macros accross the files , making it easy for you to change.


## how do you set it up

### Video directory 
- first ensure that video for linux 2 is installed on your device, use ```sudo apt install v4l-utils```
- copy and paste this command in the terminal  ```v4l2-ctl --list-devices```
- In that, you will see your camera name and at least 2 files that look like /dev/video*, pick the first one.
- Put this value in the camctl2.c file in place of **videofile** macro 

### IP address
- Make sure that this package is in the workspace of both the devices.
- Now, note that the TCP client should reach out for a client in another device, so you should make sure that the IPv4 address is set correctly in the main.hpp file. The name of the macro is **sendaddr**

### what to run where
- the main.cpp file[refer to "what it has"] should be run on your device
- The camctl2.c file should be run on the device where the camera is attached.




