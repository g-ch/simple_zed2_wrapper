/**
 * @file zed2_node.cpp
 * @author Clarence (g.chen-5@tudelft.nl)
 * @brief This is a ROS 1 node that interfaces with the ZED2 camera and publishes the camera pose, RGB image, depth image, point cloud, and object detection results.
 * @version 0.1
 * @date 2024-05-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */


// Standard includes
#include <iostream>
#include <fstream>
#include <ros/ros.h>  
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <simple_zed2_wrapper/ObjectsStamped.h>

#include <sl/Camera.hpp>
#include "utils.hpp"
#include <opencv2/opencv.hpp>

// Using std and sl namespaces
using namespace std;
using namespace sl;
bool is_playback = false;
void print(string msg_prefix, ERROR_CODE err_code = ERROR_CODE::SUCCESS, string msg_suffix = "");
void parseArgs(int argc, char **argv, InitParameters& param);
void swapRedBlueChannels(sensor_msgs::PointCloud2& cloud);

bool publish_rgb = true, publish_depth = false, publish_point_cloud = true;

bool use_object_detection = true, use_body_tracking = true;

int main(int argc, char **argv) {

    ros::init(argc, argv, "zed2_node");

    // Define ros topics to publish
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("zed2/pose_stamped", 1);
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("zed2/left/rgb/image", 1);
    ros::Publisher depth_mat_pub = nh.advertise<sensor_msgs::Image>("zed2/left/depth/image", 1);
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("zed2/rgb/point_cloud", 1);
    ros::Publisher objects_pub = nh.advertise<simple_zed2_wrapper::ObjectsStamped>("zed2/objects", 1);

#ifdef _SL_JETSON_
    const bool isJetson = true;
#else
    const bool isJetson = false;
#endif

    // Create ZED objects
    Camera zed;
    InitParameters init_parameters;
    init_parameters.depth_mode = DEPTH_MODE::ULTRA;
    init_parameters.depth_maximum_distance = 10.0f * 1000.0f;
    init_parameters.coordinate_units = UNIT::METER;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
    init_parameters.sdk_verbose = 1;

    parseArgs(argc, argv, init_parameters);

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Camera Open", returned_state, "Exit program.");
        return EXIT_FAILURE;
    }

    auto camera_config = zed.getCameraInformation().camera_configuration;
    PositionalTrackingParameters positional_tracking_parameters;
    // If the camera is static in space, enabling this settings below provides better depth quality and faster computation
    // positional_tracking_parameters.set_as_static = true;
    zed.enablePositionalTracking(positional_tracking_parameters);

    // Define the Objects detection module parameters
    BodyTrackingParameters body_tracking_parameters;
    body_tracking_parameters.enable_tracking = true;
    body_tracking_parameters.enable_segmentation = false; // designed to give person pixel mask
    body_tracking_parameters.detection_model = BODY_TRACKING_MODEL::HUMAN_BODY_MEDIUM; // HUMAN_BODY_MEDIUM, HUMAN_BODY_FAST, HUMAN_BODY_ACCURATE
    body_tracking_parameters.body_format = BODY_FORMAT::BODY_18;  // BODY_18, BODY_34, BODY_38
    body_tracking_parameters.instance_module_id = 0; // select instance ID

    if(use_body_tracking){
        print("Body Tracking: Loading Module...");
        returned_state = zed.enableBodyTracking(body_tracking_parameters);
        if (returned_state != ERROR_CODE::SUCCESS) {
            print("enableBodyTracking", returned_state, "\nExit program.");
            zed.close();
            return EXIT_FAILURE;
        }
    }

    //// Object model
    ObjectDetectionParameters object_detection_parameters;
    object_detection_parameters.enable_tracking = true;
    object_detection_parameters.enable_segmentation = false; // designed to give person pixel mask
    object_detection_parameters.detection_model = OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_MEDIUM;
    object_detection_parameters.instance_module_id = 1; // select instance ID

    if(use_object_detection){
        print("Object Detection: Loading Module...");
        returned_state = zed.enableObjectDetection(object_detection_parameters);
        if (returned_state != ERROR_CODE::SUCCESS) {
            print("enableObjectDetection", returned_state, "\nExit program.");
            zed.close();
            return EXIT_FAILURE;
        }
    }

    // Detection runtime parameters
    int detection_confidence_od = 20;
    ObjectDetectionRuntimeParameters detection_parameters_rt(detection_confidence_od);
    // To select a set of specific object classes:
    detection_parameters_rt.object_class_filter = { OBJECT_CLASS::ELECTRONICS, OBJECT_CLASS::SPORT,
        OBJECT_CLASS::ANIMAL, OBJECT_CLASS::BAG, OBJECT_CLASS::VEHICLE, OBJECT_CLASS::FRUIT_VEGETABLE };


    // Detection runtime parameters
    // default detection threshold, apply to all object class
    int body_detection_confidence = 60;
    BodyTrackingRuntimeParameters body_tracking_parameters_rt(body_detection_confidence);
    // Detection output
    bool quit = false;

    RuntimeParameters runtime_parameters;
    runtime_parameters.confidence_threshold = 50;

    /// TODO: Make body_detection_confidence and confidence_threshold ROS parameters

    Pose cam_w_pose;
    cam_w_pose.pose_data.setIdentity();
    Objects objects;
    Bodies skeletons;

    Mat image_left, depth_image, point_cloud;

    std::cout << "ZED2 node started" << std::endl;    

    bool gl_viewer_available = true;
    while ( ros::ok()) {

        auto grab_state = zed.grab(runtime_parameters);
        if (grab_state == ERROR_CODE::SUCCESS) {
            // Record the current time
            ros::Time time = ros::Time::now();
            string frame_id = "zed2";

            // Publish Camera Pose
            zed.getPosition(cam_w_pose, REFERENCE_FRAME::WORLD);

            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp = time;
            pose_msg.header.frame_id = frame_id;
            pose_msg.pose.position.x = cam_w_pose.pose_data.getTranslation().x;
            pose_msg.pose.position.y = cam_w_pose.pose_data.getTranslation().y;
            pose_msg.pose.position.z = cam_w_pose.pose_data.getTranslation().z;
            pose_msg.pose.orientation.x = cam_w_pose.pose_data.getOrientation().x;
            pose_msg.pose.orientation.y = cam_w_pose.pose_data.getOrientation().y;
            pose_msg.pose.orientation.z = cam_w_pose.pose_data.getOrientation().z;
            pose_msg.pose.orientation.w = cam_w_pose.pose_data.getOrientation().w;
            pose_pub.publish(pose_msg);

            /// TODO: Publish TF for camera pose

            // Publish Objects if detection is enabled
            if(use_object_detection){
                simple_zed2_wrapper::ObjectsStamped objects_msg;

                detection_parameters_rt.detection_confidence_threshold = detection_confidence_od;
                zed.retrieveObjects(objects, detection_parameters_rt, object_detection_parameters.instance_module_id);

                if(use_body_tracking)
                {
                    body_tracking_parameters_rt.detection_confidence_threshold = body_detection_confidence;
                    zed.retrieveBodies(skeletons, body_tracking_parameters_rt, body_tracking_parameters.instance_module_id);
                }

                for (int i = 0; i < objects.object_list.size(); i++) {
                    simple_zed2_wrapper::Object obj;
                    obj.label = toString(objects.object_list[i].label);
                    obj.label_id = objects.object_list[i].id;
                    obj.confidence = objects.object_list[i].confidence;

                    obj.position[0] = objects.object_list[i].position.x;
                    obj.position[1] = objects.object_list[i].position.y;
                    obj.position[2] = objects.object_list[i].position.z;
                    obj.position_covariance[0] = objects.object_list[i].position_covariance[0];
                    obj.position_covariance[1] = objects.object_list[i].position_covariance[1];
                    obj.position_covariance[2] = objects.object_list[i].position_covariance[2];
                    obj.position_covariance[3] = objects.object_list[i].position_covariance[3];
                    obj.position_covariance[4] = objects.object_list[i].position_covariance[4];
                    obj.position_covariance[5] = objects.object_list[i].position_covariance[5];
                    obj.position_covariance[6] = objects.object_list[i].position_covariance[6];

                    obj.velocity[0] = objects.object_list[i].velocity.x;
                    obj.velocity[1] = objects.object_list[i].velocity.y;
                    obj.velocity[2] = objects.object_list[i].velocity.z;

                    switch (objects.object_list[i].tracking_state)
                    {
                    case OBJECT_TRACKING_STATE::OK:
                        obj.tracking_state = 1;
                        break;
                    case OBJECT_TRACKING_STATE::OFF:
                        obj.tracking_state = 0;
                        break;
                    case OBJECT_TRACKING_STATE::SEARCHING:
                        obj.tracking_state = 2;
                        break;
                    default:
                        obj.tracking_state = -1;
                        break;
                    }
                    
                    switch (objects.object_list[i].action_state)
                    {
                    case OBJECT_ACTION_STATE::IDLE:
                        obj.action_state = 0;
                        break;
                    case OBJECT_ACTION_STATE::MOVING:
                        obj.action_state = 2;
                        break;
                    default:
                        break;
                    }

                    obj.dimensions_3d[0] = objects.object_list[i].dimensions.x;
                    obj.dimensions_3d[1] = objects.object_list[i].dimensions.y;
                    obj.dimensions_3d[2] = objects.object_list[i].dimensions.z;

                    // If use body tracking, add skeleton data
                    if(use_body_tracking)
                    {
                        obj.skeleton_available = true;
                        switch (body_tracking_parameters.body_format)
                        {
                        case BODY_FORMAT::BODY_18:
                            obj.body_format = 0;
                            break;
                        case BODY_FORMAT::BODY_34:
                            obj.body_format = 1;
                            break;
                        case BODY_FORMAT::BODY_38:
                            obj.body_format = 2;
                            break;
                        default:
                            break;
                        }

                        /// TODO: Publish skeleton data and added visualization in rviz
                    }

                    objects_msg.objects.push_back(obj);
                }

                objects_msg.header.stamp = time;
                objects_msg.header.frame_id = frame_id;
                objects_pub.publish(objects_msg);

            }


            if(publish_rgb){
                // Retrieve rgb image and publish
                zed.retrieveImage(image_left, VIEW::LEFT);
                cv::Mat cv_image_left = slMat2cvMat(image_left);

                // Check image channels
                if (cv_image_left.channels() == 4) {
                    cv::cvtColor(cv_image_left, cv_image_left, cv::COLOR_BGRA2BGR);
                }

                sensor_msgs::Image img_msg;
                img_msg.header.stamp = time;
                img_msg.header.frame_id = frame_id;
                img_msg.height = cv_image_left.rows;
                img_msg.width = cv_image_left.cols;
                img_msg.encoding = "bgr8";
                img_msg.is_bigendian = 0;
                img_msg.step = cv_image_left.cols * cv_image_left.elemSize();
                size_t size = cv_image_left.cols * cv_image_left.rows * cv_image_left.elemSize();
                img_msg.data.resize(size);
                memcpy(&img_msg.data[0], cv_image_left.data, size);
                img_pub.publish(img_msg); 
            }
        
            if(publish_depth){
                // Retrieve depth image and publish
                zed.retrieveMeasure(depth_image, MEASURE::DEPTH);
                cv::Mat cv_depth_image = slMat2cvMat(depth_image);

                sensor_msgs::Image depth_img_msg;
                depth_img_msg.header.stamp = time;
                depth_img_msg.header.frame_id = frame_id;
                depth_img_msg.height = cv_depth_image.rows;
                depth_img_msg.width = cv_depth_image.cols;
                depth_img_msg.encoding = "32FC1";
                depth_img_msg.is_bigendian = 0;
                depth_img_msg.step = cv_depth_image.cols * cv_depth_image.elemSize();
                size_t size = cv_depth_image.cols * cv_depth_image.rows * cv_depth_image.elemSize();
                depth_img_msg.data.resize(size);
                memcpy(&depth_img_msg.data[0], cv_depth_image.data, size);
                depth_mat_pub.publish(depth_img_msg);
            }

            if(publish_point_cloud){
                // Retrieve point cloud and publish
                zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA);
                sensor_msgs::PointCloud2 point_cloud_msg;
                point_cloud_msg.header.stamp = time;
                point_cloud_msg.header.frame_id = frame_id;
                point_cloud_msg.height = 1;
                point_cloud_msg.width = point_cloud.getWidth() * point_cloud.getHeight();
                point_cloud_msg.is_dense = false;
                point_cloud_msg.is_bigendian = false;
                point_cloud_msg.fields.resize(4);

                // Define the fields X, Y, Z, and RGBA
                point_cloud_msg.fields[0].name = "x"; point_cloud_msg.fields[0].offset = 0; point_cloud_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32; point_cloud_msg.fields[0].count = 1;
                point_cloud_msg.fields[1].name = "y"; point_cloud_msg.fields[1].offset = 4; point_cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32; point_cloud_msg.fields[1].count = 1;
                point_cloud_msg.fields[2].name = "z"; point_cloud_msg.fields[2].offset = 8; point_cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32; point_cloud_msg.fields[2].count = 1;
                point_cloud_msg.fields[3].name = "rgba"; point_cloud_msg.fields[3].offset = 12; point_cloud_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32; point_cloud_msg.fields[3].count = 1;
                
                // Configure the point step and row step
                point_cloud_msg.point_step = 16;  // 4 floats (X, Y, Z) + 1 uint32_t (RGBA)
                point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width;
                
                // Allocate memory for the data field
                point_cloud_msg.data.resize(point_cloud_msg.row_step * point_cloud_msg.height);

                // Copy data from sl::Mat to the data field of the point_cloud_msg message
                memcpy(&point_cloud_msg.data[0], point_cloud.getPtr<sl::float4>(), point_cloud_msg.data.size());

                point_cloud_pub.publish(point_cloud_msg);
            }

            if (is_playback && zed.getSVOPosition() == zed.getSVONumberOfFrames()){quit = true;}
        }        
    }

    point_cloud.free();
    image_left.free();

    zed.disableObjectDetection();
    zed.close();
    return EXIT_SUCCESS;
}

void print(string msg_prefix, ERROR_CODE err_code, string msg_suffix) {
    cout << "[Sample] ";
    if (err_code != ERROR_CODE::SUCCESS)
        cout << "[Error] ";
    cout << msg_prefix << " ";
    if (err_code != ERROR_CODE::SUCCESS) {
        cout << " | " << toString(err_code) << " : ";
        cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        cout << " " << msg_suffix;
    cout << endl;
}

void parseArgs(int argc, char **argv, InitParameters& param) {
    if (argc > 1 && string(argv[1]).find(".svo") != string::npos) {
        // SVO input mode
        param.input.setFromSVOFile(argv[1]);
        is_playback = true;
        cout << "[Sample] Using SVO File input: " << argv[1] << endl;
    } else if (argc > 1 && string(argv[1]).find(".svo") == string::npos) {
        string arg = string(argv[1]);
        unsigned int a, b, c, d, port;
        if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5) {
            // Stream input mode - IP + port
            string ip_adress = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
            param.input.setFromStream(sl::String(ip_adress.c_str()), port);
            cout << "[Sample] Using Stream input, IP : " << ip_adress << ", port : " << port << endl;
        } else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            // Stream input mode - IP only
            param.input.setFromStream(sl::String(argv[1]));
            cout << "[Sample] Using Stream input, IP : " << argv[1] << endl;
        } else if (arg.find("HD2K") != string::npos) {
            param.camera_resolution = RESOLUTION::HD2K;
            cout << "[Sample] Using Camera in resolution HD2K" << endl;
        } else if (arg.find("HD1200") != string::npos) {
            param.camera_resolution = RESOLUTION::HD1200;
            cout << "[Sample] Using Camera in resolution HD1200" << endl;
        } else if (arg.find("HD1080") != string::npos) {
            param.camera_resolution = RESOLUTION::HD1080;
            cout << "[Sample] Using Camera in resolution HD1080" << endl;
        } else if (arg.find("HD720") != string::npos) {
            param.camera_resolution = RESOLUTION::HD720;
            cout << "[Sample] Using Camera in resolution HD720" << endl;
        } else if (arg.find("SVGA") != string::npos) {
            param.camera_resolution = RESOLUTION::SVGA;
            cout << "[Sample] Using Camera in resolution SVGA" << endl;
        } else if (arg.find("VGA") != string::npos) {
            param.camera_resolution = RESOLUTION::VGA;
            cout << "[Sample] Using Camera in resolution VGA" << endl;
        }
    }

}


