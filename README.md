# ros_intel_movidius_ncs

## 1 Introduction
The Movidius™ Neural Compute Stick ([NCS](https://developer.movidius.com/)) is a tiny fanless deep learning device that you can use to learn AI programming at the edge. NCS is powered by the same low power high performance Movidius™ Vision Processing Unit ([VPU](https://www.movidius.com/solutions/vision-processing-unit)) that can be found in millions of smart security cameras, gesture controlled drones, industrial machine vision equipment, and more.  
This project is a ROS wrapper for NC API of [NCS SDK](https://ncsforum.movidius.com/discussion/98/latest-version-movidius-neural-compute-sdk), providing the following features in the latest release(v0.3.0):
* A ROS service for classifying objects in a static image file
* A ROS publisher for classifying objects in a video stream from a USB camera
* Demo applications to show the capbilities of ROS service and publisher
* Support multiple [CNN models](#table2)

## 2 Prerequisite
* An x86_64 computer running Ubuntu 16.04
* ROS Kinetic
* Movidius Neural Compute Stick (NCS)
* Movidius Neural Compute (MvNC) SDK
* USB Camera, e.g. Realsense ZR300 camera or standard USB camera

## 3 Environment Setup
* Install ROS Kinetic Desktop-Full ([guide](http://wiki.ros.org/kinetic/Installation/Ubuntu)) 
* Create a catkin workspace ([guide](http://wiki.ros.org/catkin/Tutorials/create_a_workspace))
* Install NCS SDK ([guide](https://developer.movidius.com/start/software-setup))  
* Create a symbol link in ```/opt``` to the installation path of NCS SDK ```<ncs-sdk-installation-path>```
```Shell
sudo ln -s <ncs-sdk-installation-path> /opt/NCS
```  
After that, make sure you can find graph data in ```/opt/NCS/ncapi/networks``` and image data in ```/opt/NCS/ncapi/images```
* Install ROS package for different cameras as needed. e.g.
  1. Standard USB camera
  ```Shell
  sudo apt-get install ros-kinetic-usb-cam
  ```
  2. Realsense ZR300 camera  
  Install Realsense camera package ([guide](https://github.com/IntelRealSense/realsense_samples_ros)). Refer to [Realsense ROS WiKi](http://wiki.ros.org/RealSense) for more information.
  
## 4 Building and Installation
```Shell
cd ~/catkin_ws/src
git clone https://github.com/intel/ros_intel_movidius_ncs.git
cd ~/catkin_ws
catkin_make
catkin_make install
source devel/setup.bash
```

## 5 Running the Demo
### 5.1 Video Streaming Inference
#### 5.1.1 NCS and Standard USB Camera
Launch video streaming nodelet. Refer [here](#table1) for more parameter configurations.
```Shell
roslaunch movidius_ncs_launch ncs_usbcam.launch
```
Make sure you can get result from the topic of object classification.
```Shell
rostopic echo /movidius_ncs_nodelet/classified_object
```
Launch image viewer to show the inference result.
```Shell
roslaunch movidius_ncs_launch ncs_stream_example.launch camera_topic:="/usb_cam/image_raw"
```
#### 5.1.2 NCS and Realsense ZR300 Camera
Launch video streaming nodelet. Refer [here](#table1) for more parameter configurations.
```Shell
roslaunch movidius_ncs_launch ncs_realsense.launch
```
Make sure you can get result from the topic of object classification.
```Shell
rostopic echo /movidius_ncs_nodelet/classified_object
```
Launch image viewer to show the inference result. You can launch it directly without setting ```camera_topic```, as its default value is ```/camera/color/image_raw```
```Shell
roslaunch movidius_ncs_launch ncs_stream_example.launch camera_topic:="/camera/color/image_raw"
#or
roslaunch movidius_ncs_launch ncs_stream_example.launch
```
#### 5.1.3 NCS and Other ROS Supported Camera
Launch your preferred camera node.
```Shell
#launch ROS master
roscore
#launch camera node in another console
rosrun <your-camera-pkg> <your-camera-node>
```
e.g.
```Shell
roscore
rosrun usb_cam usb_cam_node
```
Launch NCS nodelet and assign ```input_topic``` with the topic url of your rgb camera.
```Shell
roslaunch movidius_ncs_launch ncs_nocam.launch input_topic:=<your_rgb_camera_topic>
```
e.g.
```Shell
roslaunch movidius_ncs_launch ncs_nocam.launch input_topic:="/usb_cam/image_raw"
```
Launch image viewer to show the inference result.
```Shell
roslaunch movidius_ncs_launch ncs_stream_example.launch camera_topic:=<your_rgb_camera_topic>
```
e.g.
```Shell
roslaunch movidius_ncs_launch ncs_stream_example.launch camera_topic:="/usb_cam/image_raw"
```
### 5.2 Static Image Inference
Launch object classification service. Refer [here](#table1) for more parameter configurations.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch
```
Run the example application with an absolute path of an image 
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image <image_path_to_be_inferred>
```
e.g.
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image /opt/NCS/ncapi/images/cat.jpg
```
### 5.3 Choose Different CNN Models
You can choose different CNN Models for inference through the argument of ```network_conf_path```. e.g.  
Using Realsense camera
```Shell
#one console
roslaunch movidius_ncs_launch ncs_realsense.launch network_conf_path:="/opt/NCS/ncapi/networks/SqueezeNet/"
#another console
roslaunch movidius_ncs_launch ncs_stream_example.launch
```
Using standard USB camera
```Shell
#one console
roslaunch movidius_ncs_launch ncs_usbcam.launch network_conf_path:="/opt/NCS/ncapi/networks/AlexNet/"
#another console
roslaunch movidius_ncs_launch ncs_stream_example.launch camera_topic:="/usb_cam/image_raw"
```

## 6 Interfaces and Arguments
### 6.1 Topic
/movidius_ncs_nodelet/classified_object
### 6.2 Service
/movidius_ncs_image/classify_object
### 6.3 Arguments
###### *Table1*
|Node|Arguments|Default Value|Description|
|:-|:-|:-|:-|
|ncs|input_topic|/camera/color/image_raw|subscribed rgb camera topic|
|ncs|output_topic|/movidius_ncs_nodelet/classified_object|published topic of inference results|
|ncs|device_index|0|ncs device index|
|ncs|log_level|1|ncs log level|
|ncs|network_conf_path|/opt/NCS/ncapi/networks/GoogLeNet|CNN model for inference|
|ncs|top_n|3|the number of results to be shown|
|realsense|color_width|1920|frame width|
|realsense|color_height|1080|frame height|
|usb cam|image_width|640|frame width|
|usb cam|image_height|480|frame height|
|usb cam|video_device|/dev/video0|use camera device node|
## 7 CNN Support Status
###### *Table2*
|CNN Model|Weights|Status|
|:-|:-|:-|
|GoogleNet|[weights](http://dl.caffe.berkeleyvision.org/bvlc_googlenet.caffemodel)|Supported|
|AlexNet|[weights](http://dl.caffe.berkeleyvision.org/bvlc_alexnet.caffemodel)|Supported|
|SqueezeNet|[weights](https://github.com/DeepScale/SqueezeNet/raw/master/SqueezeNet_v1.0/squeezenet_v1.0.caffemodel)|Supported|
|Gender|[weights](https://dl.dropboxusercontent.com/u/38822310/gender_net.caffemodel)|Supported|
|Age|[weights](https://dl.dropboxusercontent.com/u/38822310/age_net.caffemodel)|Not supported|

## 8 Known Issues
* Only absolute path of image file supported in image inference demo
* Only test on Realsense ZR300 camera and Microsoft HD-300 USB camera

## 9 TODO
*  Support multiple NCS devices
*  Support more CNN models


###### *For security issues, please send mail to xiaojun.huang@intel.com*
