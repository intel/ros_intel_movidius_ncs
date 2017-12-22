## Detection for Video
This project supports multiple CNN models for detection. Please make sure you have already [set up environment](https://github.com/intel/ros_intel_movidius_ncs/tree/master#3-environment-setup) and [installed this project](https://github.com/intel/ros_intel_movidius_ncs/tree/master#4-building-and-installation) correctly. You can refer to the following links for your interested models then.  
#### [1 CNN Models](#1-cnn-models-1)
* [MobileNet_SSD](#mobilenet_ssd)
* [TinyYolo_V1](#tinyyolo_v1)
#### [2 Run with Other ROS Supported Cameras](#2-run-with-other-ros-supported-cameras-1)
#### [3 Other Arguments](#3-other-arguments-1)
----------------------------------

### 1 CNN Models
* #### MobileNet_SSD
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/SSD_MobileNet
make
```
Launch video streaming nodelet.
```Shell
# Option 1: RealSense
roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=mobilenetssd camera:=realsense
# Option 2: USB camera
roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=mobilenetssd camera:=usb
```
Launch image viewer to show the classification result on another console.
```Shell
# Option 1: RealSense
roslaunch movidius_ncs_launch ncs_stream_detection_example.launch camera_topic:="/camera/color/image_raw"
# Option 2: USB camera
roslaunch movidius_ncs_launch ncs_stream_detection_example.launch camera_topic:="/usb_cam/image_raw"
```
* #### TinyYolo_V1
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/TinyYolo
make
```
Launch video streaming nodelet.
```Shell
# Option 1: RealSense
roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=tinyyolo_v1 camera:=realsense
# Option 2: USB camera
roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=tinyyolo_v1 camera:=usb
```
Launch image viewer to show the classification result on another console.
```Shell
# Option 1: RealSense
roslaunch movidius_ncs_launch ncs_stream_detection_example.launch camera_topic:="/camera/color/image_raw"
# Option 2: USB camera
roslaunch movidius_ncs_launch ncs_stream_detection_example.launch camera_topic:="/usb_cam/image_raw"
```
### 2 Run with Other ROS Supported Cameras
Though this ROS package is only tested on RealSense D400 series camera and Microsoft HD-300 USB camera, other ROS supported cameras probably work as well. You can try in this way:
Launch your preferred camera node.
```Shell
#launch ROS master
roscore
#launch camera node on another console
rosrun <your-camera-pkg> <your-camera-node>
```
Launch video streaming nodelet and assign ```input_topic``` with the topic url of your RGB camera.
```Shell
# Launch video streaming nodelet
roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=mobilenetssd camera:=others input_topic:=<your_rgb_camera_topic>
# Launch image viewer to show the classification result on another console
roslaunch movidius_ncs_launch ncs_stream_detection_example.launch camera_topic:=<your_rgb_camera_topic>
```
### 3 Other Arguments
|Arguments|Description|Default Value|Valid Values|
|:-|:-|:-|:-|
|device_index|ncs device index|0|0~N-1(N is the maximum number of inserted NCS devices)|
|log_level|ncs log level|0|0:Nothing / 1:Errors / 2:Verbose|
|cnn_type|indicate different cnn models|googlenet|mobilenetssd / tinyyolo_v1|
|input_topic|RGB camera topic to be subscribed|/camera/color/image_raw|only valid when "camera:=others"|
|output_topic|published topic of inference results|/movidius_ncs_nodelet/detected_objects|any valid topic name|
|camera|camera type|others|usb / realsense / others|
|color_width|frame width of RealSense D400|640|any resolution that RealSense D400 supports|
|color_height|frame height of RealSense D400|480|any resolution that RealSense D400 supports|
|image_width|frame width of USB camera|640|any resolution that USB camera supports|
|image_height|frame height of USB camera|480|any resolution that USB camera supports|
|video_device|USB camera device node|/dev/video0|any available video device node|
