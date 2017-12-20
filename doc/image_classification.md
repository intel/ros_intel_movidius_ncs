#### 5.1.2 Static Image
Launch object classification service. You can choose different CNN models by launch arguments. Refer [here](#table1) for more parameter configurations.
```Shell
roslaunch movidius_ncs_launch ncs_image.launch cnn_type:=googlenet
```
Run the example application with an absolute path of an image 
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image_classification <image_path_to_be_inferred>
```
e.g.
```Shell
rosrun movidius_ncs_example movidius_ncs_example_image_classification /opt/movidius/ncappzoo/data/images/cat.jpg
```
