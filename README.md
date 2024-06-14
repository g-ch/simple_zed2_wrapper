# Simple ZED2 WRAPPER

## Description
This is a ROS 1 wrapper used to realize object detection and human body tracking with the ZED2 camera. (Official ROS 1 wrapper doesn't support human body tracking.) But of course, we can also output depth image, localization data, point cloud etc.

We use the ros_msg defined in ZED ROS2 wrapper. The msg is included in this wrapper.

- Tested environment. Ubuntu 20.04 + ROS Noetic.

## TODO
~~Write a node that transform everything to the global coordinate system in the Vicon field.~~

## Installation
### Install ZED SDK
Follow the instructions in [https://www.stereolabs.com/docs/get-started-with-zed](https://www.stereolabs.com/docs/get-started-with-zed) to install the SDK. Select the right system, e.g., Linux, Jetson. You can try to compile and run one of the examples to verify the installation.

### Build the Wrapper in a ROS Workspace
In the source folder of your workspace
```
git clone git@github.com:INTERACT-tud-amr/simple_zed2_wrapper.git
catkin build
```

## Run

### Get images, point cloud, etc. and 3d human pose in ZED2's VIO coordinate system

```
source devel/setup.bash
roslaunch simple_zed2_wrapper zed2.launch 
```

In the launch file, you can change the configurations of the camera. For more details of the parameters and APIs of ZED camera, please refer to [APIs](https://www.stereolabs.com/docs/api).

- After running the launch file, you should be able to see the following topics
```
/zed2/left/depth/image
/zed2/left/rgb/image
/zed2/left/rgb/point_cloud
/zed2/left/rgb/point_cloud_global
/zed2/objects
/zed2/pose_stamped
```

The topic ```/zed2/objects``` has the type ```simple_zed2_wrapper/ObjectsStamped``` defined in the msg folder. Use this message if you want to subscribe the topic. The definition of joints in the message can be found in the [Additional Info](#additional-info).


### Get images, point cloud, etc. and 3d human pose in Vicon's coordinate system
First of all, make sure you have the right ```result_robot_to_camera_matrix.csv```, which is the calibration result from [robot-camera-calibration](https://github.com/INTERACT-tud-amr/camera_robot_calibration) tool. You should do the calibration once you have changed the position of the camera on the robot or the robot frame in the vicon (e.g., markers layout changed or the center position of the robot frame on the real robot changed). 

Then do

```
source devel/setup.bash
roslaunch simple_zed2_wrapper zed2_global_human_pose.launch
```
In the launch file, you can change the configurations of the camera. For more details of the parameters and APIs of ZED camera, please refer to [APIs](https://www.stereolabs.com/docs/api). Also, you can find the robot_pose_topic (default="/vision_pose/dingo1). Change it to the pose topic of the robot in ```geometry_msgs/PoseStamped``` form.

A new topic named ```/zed2/global_objects``` and another named ```/zed2/derived_objects``` will be published. The human pose in these two topics are corrected to the vicon global coordinate.


### Visualization
- Visualize the result with [human visualization utils](https://github.com/INTERACT-tud-amr/visualization_utils). 

- You can also visualize the human pose in ```/zed2/derived_objects``` topic with [object visualization utils](https://github.com/INTERACT-tud-amr/visualization_utils/tree/main/objects_visualization).



## Additional Info
### Human Joints Definition ([18 Joints](https://www.stereolabs.com/docs/body-tracking))
| keypoint index |  keypoint name | keypoint index | keypoint name |
|:--------------:|:--------------:|:--------------:|:-------------:|
| 0              | NOSE           | 9              | RIGHT_KNEE    |
| 1              | NECK           | 10             | RIGHT_ANKLE   |
| 2              | RIGHT_SHOULDER | 11             | LEFT_HIP      |
| 3              | RIGHT_ELBOW    | 12             | LEFT_KNEE     |
| 4              | RIGHT_WRIST    | 13             | LEFT_ANKLE    |
| 5              | LEFT_SHOULDER  | 14             | RIGHT_EYE     |
| 6              | LEFT_ELBOW     | 15             | LEFT_EYE      |
| 7              | LEFT_WRIST     | 16             | RIGHT_EAR     |
| 8              | RIGHT_HIP      | 17             | LEFT_EAR      |
