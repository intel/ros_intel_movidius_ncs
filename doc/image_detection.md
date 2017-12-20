### 5.2.2 Static Image
Launch object detection service. You can choose different CNN models by launch arguments. Refer [here](#table1) for more parameter configurations.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=tinyyolo_v1
```
Run the example application with an absolute path of an image 
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image_detection <image_path_to_be_inferred>
```
e.g.
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image_detection /opt/movidius/ncappzoo/data/images/cat.jpg
```
