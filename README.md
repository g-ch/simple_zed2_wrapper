# Simple ZED2 WRAPPER

## Description
This is a ROS 1 wrapper used to realize object detection and human body tracking with the ZED2 camera. Official ROS 1 wrapper doesn't support human body tracking. But of course, we can also output depth image, localization data, point cloud etc.

We use the ros_msg defined in ZED ROS2 wrapper.

## TODO
- Add launch file.
- Update readme.


## Interface
Original [APIs](https://www.stereolabs.com/docs/api)


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
