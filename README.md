# Simple ZED2 WRAPPER

## Description
This is a ROS 1 wrapper used to realize object detection, instance segmentation, localization for Semantic DSP map.

We use the ros_msg defined in ZED ROS2 wrapper. The msg is included in this wrapper.

<em>Tested environment. Ubuntu 20.04 + ROS Noetic.</em>


## Installation
### Install ZED SDK
Follow the instructions in [https://www.stereolabs.com/docs/get-started-with-zed](https://www.stereolabs.com/docs/get-started-with-zed) to install the SDK. Select the right system, e.g., Linux, Jetson. You can try to compile and run one of the examples to verify the installation.

### Build the Wrapper in a ROS Workspace
In the source folder of your workspace
```
git clone -b semantic_dsp git@github.com:g-ch/simple_zed2_wrapper.git
catkin build
```

## Run

```
source devel/setup.bash
roslaunch simple_zed2_wrapper zed2_semantic_dsp.launch 
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
/mask_group_super_glued
```

The topic ```/zed2/objects``` has the type ```simple_zed2_wrapper/ObjectsStamped``` defined in the msg folder. Use this message if you want to subscribe the topic. The definition of joints in the message can be found in the [Additional Info](#additional-info). 

The message of topic ```mask_group_super_glued``` is defined in this [repo](https://github.com/g-ch/mask_kpts_msgs).