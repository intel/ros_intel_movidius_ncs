## Detection for Image
This project supports multiple devices and multiple CNN models for classification. By default, maximum number of devices will be used in inference. <br>
Please make sure you have already [set up environment](https://github.com/intel/ros_intel_movidius_ncs/tree/master#3-environment-setup) and [installed this project](https://github.com/intel/ros_intel_movidius_ncs/tree/master#4-building-and-installation) correctly. You can refer to the following links for your interested models then.
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
Launch the example application on another console. For example,
```Shell
roslaunch movidius_ncs_launch ncs_image_detection_example.launch demo_mode:=0 image_base_path:=<absolute-image-directory>
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
Launch the example application on another console. For example,
```Shell
roslaunch movidius_ncs_launch ncs_image_detection_example.launch demo_mode:=0 image_base_path:=<absolute-image-directory>
```
### 2 Other Arguments
|Arguments|Description|Default Value|Valid Values|
|:-|:-|:-|:-|
|device_index|ncs device index|0|0~N-1(N is the maximum number of inserted NCS devices)|
|log_level|ncs log level|0|0:Nothing / 1:Errors / 2:Verbose|
|cnn_type|indicate different cnn models|tinyyolo_v1|tinyyolo_v1 / mobilenetssd|
|demo_mode|the display mode of processed results|0:sequentially output all results 1: display random results in infinite loop|0, 1|
|image_base_path|directory of images to be inferred|"/opt/movidius/ncappzoo/data/images/"||
