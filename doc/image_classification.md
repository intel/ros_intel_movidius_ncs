## Content
This project supports multiple CNN models for classification. Please make sure you have already [set up environment](https://github.com/intel/ros_intel_movidius_ncs/tree/reorg_readme#3-environment-setup) and [installed this project](https://github.com/intel/ros_intel_movidius_ncs/tree/reorg_readme#4-building-and-installation) correctly. You can refer to the following links for your interested models then.  
### 1 CNN Models
* [AlexNet](#AlexNet)
* [GoogleNet](#GoogleNet)
* [SqueezeNet](#SqueezeNet)
* [Inception_V1](#Inception_V1)
* [Inception_V2](#Inception_V2)
* [Inception_V3](#Inception_V3)
* [Inception_V4](#Inception_V4)
* [MobileNet](#MobileNet)
### 2 Other Arguments

## 1 CNN Models
### AlexNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/AlexNet
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=alexnet
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
### GoogleNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/GoogleNet
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=googlenet
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
### SqueezeNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/SqueezeNet
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=squeezenet
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
### Inception_V1
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v1
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=inception_v1
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
### Inception_V2
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v2
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=inception_v2
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
### Inception_V3
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v3
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=inception_v3
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
### Inception_V4
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v4
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=inception_v4
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
### MobileNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/mobilenets
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=mobilenet
```
Run the example application with an absolute path of an image on another console. For example,
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
## 2 Other Arguments
|Arguments|Description|Default Value|Valid Values|
|:-|:-|:-|:-|
|device_index|ncs device index|0|0~N-1(N is the maximum number of inserted NCS devices)|
|log_level|ncs log level|0|0:Noting / 1:Errors / 2:Verbose|
|cnn_type|indicate different cnn models|googlenet|alexnet / googlenet / squeezenet / inception_v1 / inception_v2 / inception_v3 / inception_v4 / mobilenet|
|top_n|the number of results to be shown, only valid for classification|3|0~5|
