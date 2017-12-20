You can choose different CNN models and camera types by launch arguments. Refer [here](#table1) for all the supported CNN models, cameras and other parameter configurations.
##### 5.1.1.1 NCS and Standard USB Camera
```Shell
# launch video streaming nodelet
roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=googlenet camera:=usb
# launch image viewer to show the classification result in another console
roslaunch movidius_ncs_launch ncs_stream_classification_example.launch camera_topic:="/usb_cam/image_raw"
```
##### 5.1.1.2 NCS and RealSense D400 Series Camera
```Shell
# launch video streaming nodelet
roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=googlenet camera:=realsense
# launch image viewer to show the classification result in another console
roslaunch movidius_ncs_launch ncs_stream_classification_example.launch camera_topic:="/camera/color/image_raw"
```
##### 5.1.1.3 NCS and Other ROS Supported Camera
Launch your preferred camera node.
```Shell
#launch ROS master
roscore
#launch camera node in another console
rosrun <your-camera-pkg> <your-camera-node>
```
Launch video streaming nodelet and assign ```input_topic``` with the topic url of your rgb camera.
```Shell
# launch video streaming nodelet
roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=googlenet camera:=others input_topic:=<your_rgb_camera_topic>
# launch image viewer to show the classification result in another console
roslaunch movidius_ncs_launch ncs_stream_classification_example.launch camera_topic:=<your_rgb_camera_topic>
```
#### 5.1.3 Choose Different CNN Models
You can choose different CNN Models for classification through the argument of ```cnn_type```.     
Take usb camera as an example
```Shell
#one console
roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=squeezenet camera:=usb
#another console
roslaunch movidius_ncs_launch ncs_stream_classification_example.launch camera_topic:="/usb_cam/image_raw"
```
