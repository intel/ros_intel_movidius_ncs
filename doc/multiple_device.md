## Multiple Devices Support
This project supports multiple devices in parallel for classification/detection. Please make sure you have already [set up environment](https://github.com/intel/ros_intel_movidius_ncs/tree/master#3-environment-setup) and [installed this project](https://github.com/intel/ros_intel_movidius_ncs/tree/master#4-building-and-installation) correctly.
### 1 Introduction
Movidius NCS SDK supported multiple NCS in parallel for acceleration. We integrate this feature into this project in v0.6.0 release. You can now accelerate your classification/detection for both image and stream input following below guides.
### 2 Arguments
|Arguments|Description|Default Value|Valid Values|
|:-|:-|:-|:-|
|max_device_number|max device number in one node|255|0~N-1(N is the maximum number of inserted availabe NCS devices)|
|start_device_index|start index of all inserted NCS devices in one node|0|0~N-1(N is the maximum number of inserted availabe NCS devices)|
|contrast_mode|operation mode|0|0: launch one node for image or stream inference / 1:launch two nodes for performance comparison between single and multiple device(s)|
### 3 Guide
In order to control the inserted devices, we add two arguments as described above. One example of how to use these arguments is in the launch files [ncs_camera.launch](https://github.com/intel/ros_intel_movidius_ncs/blob/devel/movidius_ncs_launch/launch/ncs_camera.launch) in line 42 and line 43. By default, one process will use at most 255 devices, and use the start index as 0.<br>
You can specify other choices of these arguments in your own launch files. Make sure they are in the valid scope.
### 4 Demo
#### 4.1 Image inference
Launch two object detection services on two consoles as comparison.
```Shell
roslaunch movidius_ncs_launch ncs_image_single.launch cnn_type:=mobilenetssd
roslaunch movidius_ncs_launch ncs_image_multiple.launch cnn_type:=mobilenetssd
```
Run the example application on another console. For example,
```Shell
roslaunch movidius_ncs_launch ncs_image_parallel_detection_example.launch contrast_mode:=1 image_base_path:=<absolute-image-directory>
``` 
[![IMAGE ALT TEXT](http://img.youtube.com/vi/E21KY5osOqE/0.jpg)](http://www.youtube.com/watch?v=E21KY5osOqE)

#### 4.2 Stream inference
##### 4.2.1 Stream Classification
Launch video streaming nodelet.
```Shell
# Option 1: RealSense
roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=<classification-cnn-type> camera:=realsense contrast_mode:=1
# Option 2: USB camera
roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=<classification-cnn-type> camera:=usb contrast_mode:=1
```
Launch image viewer on another console to show the comparison result.
```Shell
# Option 1: RealSense
roslaunch movidius_ncs_launch ncs_stream_classification_example.launch camera_topic:="/camera/color/image_raw" contrast_mode:=1
# Option 2: USB camera
roslaunch movidius_ncs_launch ncs_stream_classification_example.launch camera_topic:="/usb_cam/image_raw" contrast_mode:=1
```

##### 4.2.2 Stream Detection
Launch video streaming nodelet.
```Shell
# Option 1: RealSense
roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=<detection-cnn-type> camera:=realsense contrast_mode:=1
# Option 2: USB camera
roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=<detection-cnn-type> camera:=usb contrast_mode:=1
```
Launch image viewer on another console to show the comparison result.
```Shell
# Option 1: RealSense
roslaunch movidius_ncs_launch ncs_stream_detection_example.launch camera_topic:="/camera/color/image_raw" contrast_mode:=1
# Option 2: USB camera
roslaunch movidius_ncs_launch ncs_stream_detection_example.launch camera_topic:="/usb_cam/image_raw" contrast_mode:=1
```