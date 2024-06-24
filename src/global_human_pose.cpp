/**
 * @file global_human_pose.cpp
 * @author Clarence (g.chen-5@tudelft.nl)
 * @brief This is a ROS 1 node that subscribes the human pose from the ZED camera and publishes the global human pose with the same topic and the dervied_object_msgs
 * @version 0.1
 * @date 2024-06-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include <iostream>
#include <fstream>
#include <ros/ros.h>  
#include <geometry_msgs/PoseStamped.h>
#include <simple_zed2_wrapper/ObjectsStamped.h>
#include "Eigen/Dense"
#include <math.h>
#include <ros/package.h>
#include <derived_object_msgs/ObjectArray.h>

using namespace Eigen;

ros::Publisher global_objects_pub;
ros::Publisher derived_objects_pub;

Eigen::Matrix4d camera_to_robot_transform;
Eigen::Matrix4d robot_to_global_transform;

bool camera_to_robot_transform_available = false;
bool robot_to_global_transform_available = false;


/// @brief Read the robot to camera transform from the file
/// @param file_path The file path of the robot to camera transform
void readTransform(std::string file_path)
{
    Eigen::Matrix4d robot_to_camera_transform;

    std::ifstream file(file_path);

    if(file.is_open())
    {
        std::string line;
        int row = 0;
        while(getline(file, line))
        {
            std::stringstream ss(line);
            std::string value;
            int col = 0;
            while(getline(ss, value, ','))
            {
                robot_to_camera_transform(row, col) = std::stod(value);
                col++;
            }
            row++;
        }
    }
    else
    {
        ROS_ERROR("Failed to open the file");
        file.close();
        return;
    }

    file.close();

    ROS_INFO("The robot to camera transform is: ");
    ROS_INFO_STREAM(robot_to_camera_transform);

    camera_to_robot_transform = robot_to_camera_transform.inverse().eval();  
    camera_to_robot_transform_available = true;
}


/// @brief Read the robot pose from the Vicon system
/// @param msg 
void robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{    
    Eigen::Matrix4d robot_pose = Eigen::Matrix4d::Identity();
    robot_pose.block<3,1>(0,3) = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    robot_pose.block<3,3>(0,0) = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z).toRotationMatrix();


    robot_to_global_transform = robot_pose;
    robot_to_global_transform_available = true;

    ROS_INFO_THROTTLE(1, "Robot pose Available");
}


/// @brief Do the camera to global transform
/// @param position 
/// @return Transformed position
Eigen::Vector3d doCamera2GlobalTransform(Eigen::Vector3d position)
{
    //Check if the camera to robot transform is available and the robot to global transform is available
    if(!camera_to_robot_transform_available || !robot_to_global_transform_available)
    {
        ROS_ERROR("The camera to robot transform or the robot to global transform is not available");
        return position;
    }

    Eigen::Vector4d position_4d;
    position_4d << position[0], position[1], position[2], 1;

    // Transform the position from the camera frame to the robot frame
    Eigen::Vector4d transformed_position_4d = camera_to_robot_transform * position_4d;

    // Transform the position from the robot frame to the global frame
    transformed_position_4d = robot_to_global_transform * transformed_position_4d;

    Eigen::Vector3d transformed_position;
    transformed_position << transformed_position_4d[0], transformed_position_4d[1], transformed_position_4d[2];

    return transformed_position;
}


/// @brief Callback function for the human pose from the ZED camera
/// @param msg 
void objectsCallback(const simple_zed2_wrapper::ObjectsStamped::ConstPtr& msg)
{
    simple_zed2_wrapper::ObjectsStamped objects_msg_copy = *msg;

    derived_object_msgs::ObjectArray derived_objects_msg;

    // Get the human pose from the ZED camera
    for(int i = 0; i < msg->objects.size(); i++)
    {
        if(msg->objects[i].label == "person") // We only consider the human pose for now
        {
            // Get the human pose
            Eigen::Vector3d human_position;
            human_position << msg->objects[i].position[0], msg->objects[i].position[1], msg->objects[i].position[2];

            // Transform the human position to the global frame
            Eigen::Vector3d global_human_position = doCamera2GlobalTransform(human_position);

            // Update the human position in objects_msg_copy
            objects_msg_copy.objects[i].position[0] = global_human_position[0];
            objects_msg_copy.objects[i].position[1] = global_human_position[1];
            objects_msg_copy.objects[i].position[2] = global_human_position[2];


            // Get the 3D bbox of the human
            simple_zed2_wrapper::BoundingBox3D bbox_3d = msg->objects[i].bounding_box_3d;
            for(int j = 0; j < 8; j++)
            {
                Eigen::Vector3d bbox_corner;
                bbox_corner << bbox_3d.corners[j].kp[0], bbox_3d.corners[j].kp[1], bbox_3d.corners[j].kp[2];

                // Transform the bbox corner to the world frame
                Eigen::Vector3d global_bbox_corner = doCamera2GlobalTransform(bbox_corner);

                // Update the bbox corner in objects_msg_copy
                objects_msg_copy.objects[i].bounding_box_3d.corners[j].kp[0] = global_bbox_corner[0];
                objects_msg_copy.objects[i].bounding_box_3d.corners[j].kp[1] = global_bbox_corner[1];
                objects_msg_copy.objects[i].bounding_box_3d.corners[j].kp[2] = global_bbox_corner[2];
            }

            // Get the 3D Skeleton of the human
            if(msg->objects[i].skeleton_available)
            {
                simple_zed2_wrapper::Skeleton3D skeleton_3d = msg->objects[i].skeleton_3d;

                int num_joints = 18;
                switch (msg->objects[i].body_format)
                {
                    case 0:
                        num_joints = 18;
                        break;
                    case 1:
                        num_joints = 34;
                        break;
                    case 2:
                        num_joints = 38;
                        break;
                    case 3:
                        num_joints = 70;
                        break;
                    default:
                        num_joints = 18;
                        break;
                }


                int id = 100; // Start from 100 for the derived object id for humans.

                // Get the human pose in the world frame
                for(int j = 0; j < num_joints; j++)
                {
                    Eigen::Vector3d joint_position;
                    joint_position << skeleton_3d.keypoints[j].kp[0], skeleton_3d.keypoints[j].kp[1], skeleton_3d.keypoints[j].kp[2];

                    // Transform the joint position to the world frame
                    Eigen::Vector3d global_joint_position = doCamera2GlobalTransform(joint_position);

                    // Check if the joint position in Nan or Inf. If it is, skip this joint
                    if(std::isnan(global_joint_position[0]) || std::isnan(global_joint_position[1]) || std::isnan(global_joint_position[2]) || std::isinf(global_joint_position[0]) || std::isinf(global_joint_position[1]) || std::isinf(global_joint_position[2]))
                    {
                        continue;
                    }

                    // Update the joint position in objects_msg_copy
                    objects_msg_copy.objects[i].skeleton_3d.keypoints[j].kp[0] = global_joint_position[0];
                    objects_msg_copy.objects[i].skeleton_3d.keypoints[j].kp[1] = global_joint_position[1];
                    objects_msg_copy.objects[i].skeleton_3d.keypoints[j].kp[2] = global_joint_position[2];

                    // Set a sphere for the joint in derived_object_msgs::Object msg
                    derived_object_msgs::Object derived_object_msg;
                    derived_object_msg.header = msg->header;
                    derived_object_msg.id = id;
                    id++;

                    derived_object_msg.pose.position.x = global_joint_position[0];
                    derived_object_msg.pose.position.y = global_joint_position[1];
                    derived_object_msg.pose.position.z = global_joint_position[2];

                    derived_object_msg.shape.type = shape_msgs::SolidPrimitive::SPHERE;
                    derived_object_msg.shape.dimensions.resize(3);
                    derived_object_msg.shape.dimensions[0] = 0.1;
                    derived_object_msg.shape.dimensions[1] = 0.1;
                    derived_object_msg.shape.dimensions[2] = 0.1;

                    derived_objects_msg.objects.push_back(derived_object_msg);
                }
            }
        }
    }

    // Publish the global human pose
    global_objects_pub.publish(objects_msg_copy);

    // Publish the derived object
    derived_objects_pub.publish(derived_objects_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_human_pose");
    ros::NodeHandle nh;

    // Get package path
    std::string package_path = ros::package::getPath("simple_zed2_wrapper");

    // Read the robot to camera transform
    std::string file_path = package_path + "/result_robot_to_camera_matrix.csv";
    ROS_INFO("Reading the robot to camera transform from %s", file_path.c_str());

    readTransform(file_path);


    // Subscribe the human pose from the ZED camera     ros::Publisher objects_pub = nh.advertise<simple_zed2_wrapper::ObjectsStamped>("zed2/objects", 1);
    ros::Subscriber human_pose_sub = nh.subscribe("/zed2/objects", 1,objectsCallback);

    ros::Subscriber robot_pose_sub = nh.subscribe("/vision_pose/dingo1", 1, robotPoseCallback);
    

    // Publish the global human pose
    global_objects_pub = nh.advertise<simple_zed2_wrapper::ObjectsStamped>("zed2/global_objects", 1);

    // Publish the derived object
    derived_objects_pub = nh.advertise<derived_object_msgs::ObjectArray>("zed2/derived_objects", 1);

    ros::spin();

    return 0;
}


