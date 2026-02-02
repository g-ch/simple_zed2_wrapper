# Simple ZED2 WRAPPER

## Description
This is a ROS 1 wrapper used to realize object detection, instance segmentation, localization, depth image query, etc.


<em>Tested environment. Ubuntu 20.04 + ROS Noetic.</em>


## Installation
### Install ZED SDK
Follow the instructions in [https://www.stereolabs.com/docs/get-started-with-zed](https://www.stereolabs.com/docs/get-started-with-zed) to install the SDK. Select the right system, e.g., Linux, Jetson. You can try to compile and run one of the examples to verify the installation.

### Build the Wrapper in a ROS Workspace
In the source folder of your workspace
```
git clone -b scannet_recorder git@github.com:g-ch/simple_zed2_wrapper.git
catkin build
```

## Run

```bash
source devel/setup.bash
roslaunch simple_zed2_wrapper zed2.launch 
```

After running the launch file, you should be able to see the following topics
```
/zed2/left/depth/image
/zed2/left/rgb/image
/zed2/pose_stamped
/zed2/left/rgb/camera_info
```

Then create a folder to store the images, for example xxx/Socialroom_00 (or 01 if it is the second scan). 
Run the following to record the data:
```
roslaunch simple_zed2_wrapper scannet_converter.launch output_dir:=xxx/Socialroom_00
```
The sychronized RGB, Depth, and pose will be saved at the output_dir.
To terminate recording, press Ctrl+C to stop the scannet_converter.launch from running. An _info.txt containing camera info will be created after the termination automatically.
