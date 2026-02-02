#!/usr/bin/env python3
"""
Simulates ZED2 camera output for testing the ScanNet converter.
Publishes the same topics that scannet_converter subscribes to:
  - zed2/left/rgb/image (gray image)
  - zed2/left/depth/image (32FC1, meters: 0.5m at center, 20m at corners)
  - zed2/pose_stamped (identity / eye matrix)
  - zed2/left/rgb/camera_info
"""

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


def make_gray_image(height, width):
    """Purely gray BGR image (same value in B, G, R)."""
    gray = np.full((height, width, 3), 128, dtype=np.uint8)
    return gray


def make_depth_image(height, width, center_m=0.5, corner_m=20.0):
    """
    Depth in meters (32FC1): center has center_m, corners have corner_m, growing gradually.
    Distance from center is normalized; depth interpolates linearly from center to corners.
    """
    cy, cx = height / 2.0, width / 2.0
    max_dist = np.sqrt(cx**2 + cy**2)  # distance from center to corner
    u = np.arange(width, dtype=np.float32)
    v = np.arange(height, dtype=np.float32)
    u, v = np.meshgrid(u, v)
    dist_from_center = np.sqrt((u - cx) ** 2 + (v - cy) ** 2)
    # Normalize to [0, 1] from center to corner
    t = np.clip(dist_from_center / max_dist, 0.0, 1.0)
    depth = center_m + (corner_m - center_m) * t
    return depth.astype(np.float32)


def make_identity_pose():
    """Camera pose as identity (eye) matrix: position (0,0,0), orientation identity quaternion (0,0,0,1)."""
    pose = PoseStamped()
    pose.header.frame_id = "zed2"
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose


def make_camera_info(width, height, frame_id="zed2"):
    """CameraInfo with default-style intrinsics (K, R, P)."""
    fx = 530.4337158203125
    fy = 530.4337158203125
    cx = width / 2.0
    cy = height / 2.0
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.height = height
    info.width = width
    info.distortion_model = "plumb_bob"
    info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    info.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


def main():
    rospy.init_node("zed2_data_simulator", anonymous=False)

    width = rospy.get_param("~width", 1280)
    height = rospy.get_param("~height", 720)
    rate_hz = rospy.get_param("~rate", 10.0)

    bridge = CvBridge()
    pub_rgb = rospy.Publisher("zed2/left/rgb/image", Image, queue_size=1)
    pub_depth = rospy.Publisher("zed2/left/depth/image", Image, queue_size=1)
    pub_pose = rospy.Publisher("zed2/pose_stamped", PoseStamped, queue_size=1)
    pub_info = rospy.Publisher("zed2/left/rgb/camera_info", CameraInfo, queue_size=1)

    rgb = make_gray_image(height, width)
    depth = make_depth_image(height, width)
    camera_info = make_camera_info(width, height)

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        t = rospy.Time.now()
        frame_id = "zed2"

        # RGB
        rgb_msg = bridge.cv2_to_imgmsg(rgb, encoding="bgr8")
        rgb_msg.header.stamp = t
        rgb_msg.header.frame_id = frame_id
        pub_rgb.publish(rgb_msg)

        # Depth (32FC1, meters)
        depth_msg = bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header.stamp = t
        depth_msg.header.frame_id = frame_id
        pub_depth.publish(depth_msg)

        # Pose (identity)
        pose_msg = make_identity_pose()
        pose_msg.header.stamp = t
        pub_pose.publish(pose_msg)

        # Camera info
        camera_info.header.stamp = t
        pub_info.publish(camera_info)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
