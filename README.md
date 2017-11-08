# ros_intel_movidius_ncs

## 1 Introduction
The Movidius™ Neural Compute Stick ([NCS](https://developer.movidius.com/)) is a tiny fanless deep learning device that you can use to learn AI programming at the edge. NCS is powered by the same low power high performance Movidius™ Vision Processing Unit ([VPU](https://www.movidius.com/solutions/vision-processing-unit)) that can be found in millions of smart security cameras, gesture controlled drones, industrial machine vision equipment, and more.  
This project is a ROS wrapper for NC API of [NCSDK](https://ncsforum.movidius.com/discussion/98/latest-version-movidius-neural-compute-sdk), providing the following features:
* A ROS service for classifying objects in a static image file
* A ROS publisher for classifying objects in a video stream from a USB camera
* Demo applications to show the capabilities of ROS service and publisher
* Support multiple [CNN models](#table2)

There are 2 active branches in this project:
* master - *stable branch*  
  The latest version on it is v0.4.0 which supports NCSDK v1.09.00. master branch is only updated when every milestone release ready.
* devel - *default branch*  
  This branch is updated from time to time and maintain the latest code on it. Each pull request should be submitted based on devel branch. We will merge patches to master branch on every milestone release.  
  
## 2 Prerequisite
* An x86_64 computer running Ubuntu 16.04
* ROS Kinetic
* Movidius Neural Compute Stick (NCS)
* Movidius Neural Compute (MvNC) SDK
* RGB Camera, e.g. RealSense ZR300, RealSense D400 Series or standard USB camera

## 3 Environment Setup
* Install ROS Kinetic Desktop-Full ([guide](http://wiki.ros.org/kinetic/Installation/Ubuntu)) 
* Create a catkin workspace ([guide](http://wiki.ros.org/catkin/Tutorials/create_a_workspace))
* Install NCSDK [v1.09.00](https://github.com/movidius/ncsdk/releases) ([github](https://github.com/movidius/ncsdk))
* NCSDK should be installed in ```/opt/movidius``` by default. Create a symbol link in ```/opt/movidius``` to the supported CNN network from NCSDK release package.
```Shell
sudo ln -s ncsdk/examples /opt/movidius/examples
```  
After that, make sure you can find graph data in ```/opt/movidius/examples/caffe``` or ```/opt/movidius/examples/tensorflow``` and image data in ```/opt/movidius/examples/data/images```
* Install ROS package for different cameras as needed. e.g.
  1. Standard USB camera
  ```Shell
  sudo apt-get install ros-kinetic-usb-cam
  ```
  2. RealSense ZR300 camera  
  - Install RealSense camera package ([guide](https://github.com/IntelRealSense/realsense_samples_ros)). Refer to [Realsense ROS WiKi](http://wiki.ros.org/RealSense) for more information.
      
  3. RealSense D400 series camera  
  - Install Intel® RealSense™ SDK 2.0 ([guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)). Refer [here](https://github.com/IntelRealSense/librealsense) for more details about Intel® RealSense™ SDK 2.0.  
  - Install Intel® RealSense™ ROS ([guide](https://github.com/intel-ros/realsense))  
  ```Shell
  cd ~/catkin_ws/src
  git clone https://github.com/intel-ros/realsense.git
  cd  realsense
  git checkout 2.0.0
  cd ~/catkin_ws
  catkin_make
  ```  
## 4 Building and Installation
```Shell
cd ~/catkin_ws/src
git clone https://github.com/intel/ros_intel_movidius_ncs.git
cd ros_intel_movidius_ncs
git checkout master
cd ~/catkin_ws
catkin_make
catkin_make install
source install/setup.bash
```
Copy ```categories.txt``` from this project to NCSDK installation location.
```Shell
cp ~/catkin_ws/src/ros_intel_movidius_ncs/data/categories.txt /opt/movidius/examples/data/ilsvrc12/
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
#### 5.1.2 NCS and RealSense ZR300 Camera
Launch video streaming nodelet. Refer [here](#table1) for more parameter configurations.
```Shell
roslaunch movidius_ncs_launch ncs_realsense_zr300.launch
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
#### 5.1.3 NCS and RealSense D400 Series Camera
Launch video streaming nodelet. Refer [here](#table1) for more parameter configurations.
```Shell
roslaunch movidius_ncs_launch ncs_realsense_d400.launch
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
#### 5.1.4 NCS and Other ROS Supported Camera
Launch your preferred camera node.
```Shell
#launch ROS master
roscore
#launch camera node in another console
rosrun <your-camera-pkg> <your-camera-node>
```
e.g.
```Shell
#launch ROS master
roscore
#launch camera node in another console
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
rosrun movidius_ncs_example movidius_ncs_example_image /opt/movidius/examples/data/images/cat.jpg
```
### 5.3 Choose Different CNN Models
You can choose different CNN Models for inference through the argument of ```graph_file_path```. e.g.  
Using RealSense camera
```Shell
#one console
roslaunch movidius_ncs_launch ncs_realsense_d400.launch graph_file_path:="/opt/movidius/examples/caffe/SqueezeNet/graph"
#another console
roslaunch movidius_ncs_launch ncs_stream_example.launch
```
Using standard USB camera
```Shell
#one console
roslaunch movidius_ncs_launch ncs_usbcam.launch graph_file_path:="/opt/movidius/examples/caffe/AlexNet/graph"
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
|ncs|graph_file_path|/opt/movidius/examples/caffe/GoogLeNet/graph|the path of CNN model graph file|
|ncs|category_file_path|/opt/movidius/examples/data/ilsvrc12/categories.txt|object category list|
|ncs|network_dimension|224|network dimension for input data|
|ncs|channel1_mean|0.40787054|mean value of the first channel of image, defualt value is for ImageNet dataset|
|ncs|channel2_mean|0.45752458|mean value of the second channel of image, defualt value is for ImageNet dataset|
|ncs|channel3_mean|0.48109378|mean value of the third channel of image, defualt value is for ImageNet dataset|
|ncs|top_n|3|the number of results to be shown|
|realsense|color_width|640|frame width|
|realsense|color_height|480|frame height|
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

## 8 ChangeLog
#### 0.3.0 (2017-09-30)
* Support NCSDK v1.07.06
* Support USB camera and RealSense ZR300
#### 0.4.0 (2017-11-03)
* Support NCSDK [v1.09.00](https://github.com/movidius/ncsdk/releases) and above ([#12](https://github.com/intel/ros_intel_movidius_ncs/pull/12))
* Add the support for RealSense D400 series camera ([#13](https://github.com/intel/ros_intel_movidius_ncs/pull/13))

## 9 Known Issues
* Only absolute path of image file supported in image inference demo
* Only test on RealSense ZR300 camera, RealSense D400 series camera and Microsoft HD-300 USB camera
* Current v0.4.0 supporting NCSDK v1.09.00 is on master branch. devel branch is the development branch for the next release.

## 10 TODO
*  Support multiple NCS devices
*  Support more CNN models
*  Support latest NCSDK


###### *For security issues, please send mail to xiaojun.huang@intel.com*
