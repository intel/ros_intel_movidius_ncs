# ros_intel_movidius_ncs

## 1 Introduction
The Movidius™ Neural Compute Stick ([NCS](https://developer.movidius.com/)) is a tiny fanless deep learning device that you can use to learn AI programming at the edge. NCS is powered by the same low power high performance Movidius™ Vision Processing Unit ([VPU](https://www.movidius.com/solutions/vision-processing-unit)) that can be found in millions of smart security cameras, gesture controlled drones, industrial machine vision equipment, and more.  

This project is a ROS wrapper for NC API of [NCSDK](https://movidius.github.io/ncsdk/), providing the following features:
* A ROS service for object classification and detection of a static image file
* A ROS publisher for object classification and detection of a video stream from a RGB camera
* Demo applications to show the capabilities of ROS service and publisher
* Support multiple CNN models of Caffe and Tensorflow

There are 2 active branches in this project:
* master - *stable branch*  
  The latest version on it is v0.4.0 which supports NCSDK v1.10.00. master branch is only updated when every milestone release ready.
* devel - *default branch*  
  This branch is updated from time to time and maintain the latest code on it. Each pull request should be submitted based on devel branch. We will merge patches to master branch on every milestone release.  
  
## 2 Prerequisite
* An x86_64 computer running Ubuntu 16.04
* ROS Kinetic
* Movidius Neural Compute Stick (NCS)
* Movidius Neural Compute (MvNC) SDK
* Movidius Neural Compute Application Zoo
* RGB Camera, e.g. RealSense D400 Series or standard USB camera

## 3 Environment Setup
* Install ROS Kinetic Desktop-Full ([guide](http://wiki.ros.org/kinetic/Installation/Ubuntu)) 
* Create a catkin workspace ([guide](http://wiki.ros.org/catkin/Tutorials/create_a_workspace))
* Install NCSDK [v1.10.00](https://github.com/movidius/ncsdk/releases) ([github](https://github.com/movidius/ncsdk))
* Install NC APP Zoo([github](https://github.com/movidius/ncappzoo))
* NCSDK should be installed in ```/opt/movidius``` by default. Create a symbol link in ```/opt/movidius``` to NC APP Zoo.
```Shell
sudo ln -s <your-workspace>/ncappzoo /opt/movidius/ncappzoo
```  
After that, make sure you can find graph data in ```/opt/movidius/ncappzoo/caffe``` or ```/opt/movidius/ncappzoo/tensorflow``` and image data in ```/opt/movidius/ncappzoo/data/images```
* Install ROS package for different cameras as needed. e.g.
  1. Standard USB camera
  ```Shell
  sudo apt-get install ros-kinetic-usb-cam
  ```    
  2. RealSense D400 series camera  
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
# Building
cd ~/catkin_ws/src
git clone https://github.com/intel/ros_intel_movidius_ncs.git
cd ros_intel_movidius_ncs
git checkout devel
cd ~/catkin_ws
catkin_make
# Installation
catkin_make install
source install/setup.bash
# Copy label files from this project to the installation location of NCSDK
cp ~/catkin_ws/src/ros_intel_movidius_ncs/data/labels/* /opt/movidius/ncappzoo/data/ilsvrc12/
```
## 5 Running the Demo
### 5.1 Classification
#### 5.1.1 Supported CNN Models
###### *Table1*
|CNN Model|Framework|FPS|Usage|
|:-|:-|:-|:-|
|AlexNet|Caffe||Link|
|GoogleNet|Caffe||Link|
|SqueezeNet|Caffe||Link|
|Inception_v1|Tensorflow||Link|
|Inception_v2|Tensorflow||Link|
|Inception_v3|Tensorflow||Link|
|Inception_v4|Tensorflow||Link|
|MobileNet|Tensorflow||Link|
#### 5.1.2 Classification Result with GoogleNet
![classification with googlenet](https://github.com/intel/ros_intel_movidius_ncs/blob/reorg_readme/data/results/googlenet_dog.png "classification with googlenet")
#### 5.1.3 Running the Demo
* [image](https://github.com/intel/ros_intel_movidius_ncs/blob/reorg_readme/doc/image_classification.md)
* [Video](https://github.com/intel/ros_intel_movidius_ncs/blob/reorg_readme/doc/video_classification.md)
### 5.2 Detection
#### 5.1.1 Supported CNN Models
|CNN Model|Framework|FPS|Usage|
|:-|:-|:-|:-|
|MobileNetSSD|Caffe||Link|
|TinyYolo_v1|Caffe||Link|
#### 5.1.2 Detection Result with MobileNetSSD
![detection with mobilenetssd](https://github.com/intel/ros_intel_movidius_ncs/blob/reorg_readme/data/results/mobilenetssd_car_bicycle.png "detection with mobilenetssd")
#### 5.1.3 Running the Demo
* [Image](https://github.com/intel/ros_intel_movidius_ncs/blob/reorg_readme/doc/image_detection.md)
* [Video](https://github.com/intel/ros_intel_movidius_ncs/blob/reorg_readme/doc/video_detection.md)

## 6 Interfaces and Arguments
### 6.1 Topic
Classification
```
/movidius_ncs_nodelet/classified_objects
```
Detection
```
/movidius_ncs_nodelet/detected_objects
```
### 6.2 Service
Classification
```
/movidius_ncs_image/classify_object
```
Detection
```
/movidius_ncs_image/detect_object
```
### 6.3 Arguments
###### *Table3*
|Node|Arguments|Default Value|Description|
|:-|:-|:-|:-|
|ncs|input_topic|/camera/color/image_raw|subscribed rgb camera topic|
|ncs|output_topic|/movidius_ncs_nodelet/classified_object|published topic of inference results|
|ncs|device_index|0|ncs device index|
|ncs|log_level|1|ncs log level|
|ncs|cnn_type|googlenet|indicate different cnn types for classification or detection|
|ncs|param_file|googlenet.yaml|configuration file of CNN models|
|ncs|top_n|3|the number of results to be shown, only valid for classification|
|camera|camera|others|value can be usb, realsense and others|
|realsense|color_width|640|frame width|
|realsense|color_height|480|frame height|
|usb cam|image_width|640|frame width|
|usb cam|image_height|480|frame height|
|usb cam|video_device|/dev/video0|use camera device node|

## 7 Known Issues
* Only absolute path of image file supported in image inference demo
* Only test on RealSense D400 series camera and Microsoft HD-300 USB camera
* Current v0.4.0 supporting NCSDK v1.10.00 is on master branch. devel branch is the development branch for the next release.

## 8 TODO
*  Support multiple NCS devices
*  Support more CNN models
*  Support latest NCSDK


###### *Any security issue should be reported using process at https://01.org/security*
