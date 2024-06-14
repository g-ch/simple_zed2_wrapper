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

using namespace Eigen;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_human_pose");
    ros::NodeHandle nh;

    

    return 0;
}


