## Detection for Image
This project supports multiple CNN models for detection. Please make sure you have already [set up environment](https://github.com/intel/ros_intel_movidius_ncs/tree/master#3-environment-setup) and [installed this project](https://github.com/intel/ros_intel_movidius_ncs/tree/master#4-building-and-installation) correctly. You can refer to the following links for your interested models then.  
#### [1 CNN Models](#1-cnn-models-1)
* [MobileNet_SSD](#mobilenet_ssd)
* [TinyYolo_V1](#tinyyolo_v1)
#### [2 Other Arguments](#2-other-arguments-1)
----------------------------------

### 1 CNN Models
* #### MobileNet_SSD
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/SSD_MobileNet
make
```
Launch object detection service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=mobilenetssd
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image_detection /opt/movidius/ncappzoo/data/images/cat.jpg
```
* #### TinyYolo_V1
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/TinyYolo
make
```
Launch object detection service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=tinyyolo_v1
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image_detection /opt/movidius/ncappzoo/data/images/cat.jpg
```
### 2 Other Arguments
|Arguments|Description|Default Value|Valid Values|
|:-|:-|:-|:-|
|device_index|ncs device index|0|0~N-1(N is the maximum number of inserted NCS devices)|
|log_level|ncs log level|0|0:Nothing / 1:Errors / 2:Verbose|
|cnn_type|indicate different cnn models|tinyyolo_v1|tinyyolo_v1 / mobilenetssd|
