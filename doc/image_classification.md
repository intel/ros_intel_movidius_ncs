## Classification for Image
This project supports multiple devices and multiple CNN models for classification. By default, maximum number of devices will be used in inference. <br>
Please make sure you have already [set up environment](https://github.com/intel/ros_intel_movidius_ncs/tree/master#3-environment-setup) and [installed this project](https://github.com/intel/ros_intel_movidius_ncs/tree/master#4-building-and-installation) correctly. You can refer to the following links for your interested models then.   
#### [1 CNN Models](#1-cnn-models-1)
* [AlexNet](#alexnet)
* [GoogleNet](#googlenet)
* [SqueezeNet](#squeezenet)
* [Inception_V1](#inception_v1)
* [Inception_V2](#inception_v2)
* [Inception_V3](#inception_v3)
* [Inception_V4](#inception_v4)
* [MobileNet](#mobilenet)
#### [2 Other Arguments](#2-other-arguments-1)
----------------------------------

### 1 CNN Models
* #### AlexNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/AlexNet
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=alexnet
```
Launch the example application on another console. For example,
```Shell
roslaunch movidius_ncs_launch ncs_image_classification_example.launch demo_mode:=0
```
* #### GoogleNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/GoogleNet
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=googlenet
```
Launch the example application on another console. For example,
```Shell
roslaunch movidius_ncs_launch ncs_image_classification_example.launch demo_mode:=0
```
* #### SqueezeNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/caffe/SqueezeNet
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=squeezenet
```
Launch the example application on another console. For example,
```Shell
roslaunch movidius_ncs_launch ncs_image_classification_example.launch demo_mode:=0
```
* #### Inception_V1
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v1
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=inception_v1
```
Launch the example application on another console. For example,
```Shell
roslaunch movidius_ncs_launch ncs_image_classification_example.launch demo_mode:=0
```
* #### Inception_V2
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v2
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=inception_v2
```
Launch the example application on another console. For example,
```Shell
roslaunch movidius_ncs_launch ncs_image_classification_example.launch demo_mode:=0
```
* #### Inception_V3
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v3
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=inception_v3
```
Launch the example application on another console. For example,
```Shell
roslaunch movidius_ncs_launch ncs_image_classification_example.launch demo_mode:=0
```
* #### Inception_V4
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/inception_v4
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=inception_v4
```
Launch the example application on another console. For example,
```Shell
roslaunch movidius_ncs_launch ncs_image_classification_example.launch demo_mode:=0
```
* #### MobileNet
Compile NCS graph.
```Shell
cd /opt/movidius/ncappzoo/tensorflow/mobilenets
make
```
Launch object classification service.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=mobilenet
```
Launch the example application on another console. For example,
```Shell
roslaunch movidius_ncs_launch ncs_image_classification_example.launch demo_mode:=0
```
### 2 Other Arguments
|Arguments|Description|Default Value|Valid Values|
|:-|:-|:-|:-|
|device_index|ncs device index|0|0~N-1(N is the maximum number of inserted NCS devices)|
|log_level|ncs log level|0|0:Nothing / 1:Errors / 2:Verbose|
|cnn_type|indicate different cnn models|googlenet|alexnet / googlenet / squeezenet / inception_v1 / inception_v2 / inception_v3 / inception_v4 / mobilenet|
|top_n|the number of results to be shown, only valid for classification|3|0~5|
|demo_mode|the display mode of processed results|0:sequentially output all results 1: display random results in infinite loop|0, 1|
|image_base_path|directory of images to be inferred|"/opt/movidius/ncappzoo/data/images/"||
