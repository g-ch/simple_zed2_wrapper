/**
 * @file zed2_node.cpp
 * @author Clarence (g.chen-5@tudelft.nl)
 * @brief This is a ROS 1 node that interfaces with the ZED2 camera and publishes the camera pose, RGB image, depth image, point cloud, and object detection results
 * @version 0.1
 * @date 2024-05-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include <iostream>
#include <fstream>
#include <ros/ros.h>  
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/CameraInfo.h>
#include <simple_zed2_wrapper/ObjectsStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sl/Camera.hpp>
#include "utils.hpp"
#include <opencv2/opencv.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mask_kpts_msgs/MaskGroup.h>
#include <mask_kpts_msgs/MaskKpts.h>
#include <mask_kpts_msgs/Keypoint.h>
#include <mask_kpts_msgs/ImageWithID.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward{
    backward::SignalHandling sh;
}

// Using std and sl namespaces
using namespace std;
using namespace sl;
bool is_playback = false;

void print(string msg_prefix, ERROR_CODE err_code = ERROR_CODE::SUCCESS, string msg_suffix = "");
void parseArgs(int argc, char **argv, InitParameters& param);
void swapRedBlueChannels(sensor_msgs::PointCloud2& cloud);
void setRegularObjectsMsg(simple_zed2_wrapper::ObjectsStamped &objects_msg, Objects &objects);
void setHumanBodyMsg(simple_zed2_wrapper::ObjectsStamped &objects_msg, Bodies &skeletons, geometry_msgs::PoseStamped &camera_pose, bool detection_result_in_camera_frame);
void setMaskKptsMsg(mask_kpts_msgs::MaskGroup &mask_group_msg, visualization_msgs::MarkerArray &marker_msg,  Objects &objects, Bodies &skeletons, int image_width, int image_height);


sensor_msgs::Image semantic_seg_image;
int semantic_seg_image_id = -1;

/// @brief Callback function for the semantic segmentation image. The Segmentation Image should be a mono8 image filled with label ids.
/// @param msg 
void semanticSegCallback(const mask_kpts_msgs::ImageWithID& msg)
{
    semantic_seg_image = msg.image;
    semantic_seg_image_id = msg.id;
}


/// @brief  Main function. We have a loop that retrieves the camera pose, RGB image, depth image, point cloud, and object detection results and publishes them to ROS topics.
/// @param argc See parseArgs function
/// @param argv See parseArgs function
/// @return 
int main(int argc, char **argv) {

    ros::init(argc, argv, "zed2_node");

    // Define ros topics to publish
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("zed2/pose_stamped", 1);
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("zed2/left/rgb/image", 1);
    ros::Publisher rgb_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("zed2/left/rgb/camera_info", 1);
    ros::Publisher depth_mat_pub = nh.advertise<sensor_msgs::Image>("zed2/left/depth/image", 1);
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("zed2/left/rgb/point_cloud", 1);
    ros::Publisher point_cloud_global_pub = nh.advertise<sensor_msgs::PointCloud2>("zed2/left/rgb/point_cloud_global", 1);
    ros::Publisher objects_pub = nh.advertise<simple_zed2_wrapper::ObjectsStamped>("zed2/objects", 1);

    ros::Publisher img_with_id_pub = nh.advertise<mask_kpts_msgs::ImageWithID>("zed2/left/rgb/image_with_id", 1);
    ros::Publisher bboxes_pub = nh.advertise<visualization_msgs::MarkerArray>("zed2/bounding_boxes", 1);

    ros::Publisher mask_group_pub = nh.advertise<mask_kpts_msgs::MaskGroup>("/mask_group_super_glued", 1);
    ros::Publisher depth_repub = nh.advertise<sensor_msgs::Image>("/camera/depth_repub", 1);
    ros::Publisher pose_repub = nh.advertise<geometry_msgs::PoseStamped>("/camera/pose_repub", 1);

    ros::Subscriber semantic_seg_sub = nh.subscribe("zed2/left/rgb/image_mask_with_id", 1, semanticSegCallback);

    // Get ros parameters from the launch file
    bool enable_object_detection, enable_object_tracking, enable_object_segmentation;
    bool enable_body_tracking, enable_body_segmentation;

    int detection_confidence, body_detection_confidence, depth_confidence;

    bool detection_result_in_camera_frame;

    bool publish_rgb, publish_depth, publish_point_cloud, publish_point_cloud_global, publish_objects;

    bool synchoronize_pose_depth_mask, external_semantic_seg_on;

    ros::NodeHandle nh2("~");
    nh2.param<bool>("enable_object_detection", enable_object_detection, true);
    nh2.param<bool>("enable_body_tracking", enable_body_tracking, true);
    nh2.param<bool>("enable_body_segmentation", enable_body_segmentation, false);
    nh2.param<bool>("enable_object_tracking", enable_object_tracking, true);
    nh2.param<bool>("enable_object_segmentation", enable_object_segmentation, false);

    nh2.param<bool>("detection_result_in_camera_frame", detection_result_in_camera_frame, true);

    nh2.param<int>("detection_confidence", detection_confidence, 20);
    nh2.param<int>("body_detection_confidence", body_detection_confidence, 60);
    nh2.param<int>("depth_confidence", depth_confidence, 50);

    nh2.param<bool>("publish_rgb", publish_rgb, true);
    nh2.param<bool>("publish_depth", publish_depth, false);
    nh2.param<bool>("publish_objects", publish_objects, true);
    nh2.param<bool>("publish_point_cloud", publish_point_cloud, true);
    nh2.param<bool>("publish_point_cloud_global", publish_point_cloud_global, true);

    nh2.param<bool>("synchoronize_pose_depth_mask", synchoronize_pose_depth_mask, true);
    nh2.param<bool>("external_semantic_seg_on", external_semantic_seg_on, true);
    
    std::cout << "detection_result_in_camera_frame: " << detection_result_in_camera_frame << std::endl;


#ifdef _SL_JETSON_
    const bool isJetson = true;
#else
    const bool isJetson = false;
#endif

    // Create ZED objects
    Camera zed;
    InitParameters init_parameters;
    init_parameters.depth_maximum_distance = 10.0f * 1000.0f;
    init_parameters.coordinate_units = UNIT::METER;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::IMAGE; // Check https://www.stereolabs.com/docs/positional-tracking/coordinate-frames for more information
    init_parameters.sdk_verbose = 1;

    parseArgs(argc, argv, init_parameters);

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Camera Open", returned_state, "Exit program.");
        return EXIT_FAILURE;
    }

    auto camera_config = zed.getCameraInformation().camera_configuration;

    CalibrationParameters calibration_params = camera_config.calibration_parameters;

    // Publish camera info
    sensor_msgs::CameraInfo rgb_camera_info;
    rgb_camera_info.header.frame_id = "zed2";
    rgb_camera_info.width = camera_config.resolution.width;
    rgb_camera_info.height = camera_config.resolution.height;
    rgb_camera_info.distortion_model = "plumb_bob";
    rgb_camera_info.D = {calibration_params.left_cam.disto[0], calibration_params.left_cam.disto[1], calibration_params.left_cam.disto[2], calibration_params.left_cam.disto[3], calibration_params.left_cam.disto[4]};
    rgb_camera_info.K = {calibration_params.left_cam.fx, 0, calibration_params.left_cam.cx, 0, calibration_params.left_cam.fy, calibration_params.left_cam.cy, 0, 0, 1};
    rgb_camera_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    rgb_camera_info.P = {calibration_params.left_cam.fx, 0, calibration_params.left_cam.cx, 0, 0, calibration_params.left_cam.fy, calibration_params.left_cam.cy, 0, 0, 0, 1, 0};
    rgb_camera_info_pub.publish(rgb_camera_info);

    // Enable positional tracking
    PositionalTrackingParameters positional_tracking_parameters;
    // If the camera is static in space, enabling this settings below provides better depth quality and faster computation
    // positional_tracking_parameters.set_as_static = true;
    zed.enablePositionalTracking(positional_tracking_parameters);

    // Define the Objects detection module parameters
    BodyTrackingParameters body_tracking_parameters;
    body_tracking_parameters.enable_tracking = enable_body_tracking;
    body_tracking_parameters.enable_segmentation = enable_body_segmentation; // designed to give person pixel mask
    body_tracking_parameters.detection_model = BODY_TRACKING_MODEL::HUMAN_BODY_MEDIUM; // HUMAN_BODY_MEDIUM, HUMAN_BODY_FAST, HUMAN_BODY_ACCURATE
    body_tracking_parameters.body_format = BODY_FORMAT::BODY_18;  // BODY_18, BODY_34, BODY_38
    body_tracking_parameters.instance_module_id = 0; // select instance ID

    if(enable_body_tracking){
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
    object_detection_parameters.enable_tracking = enable_object_tracking;
    object_detection_parameters.enable_segmentation = enable_object_segmentation;
    object_detection_parameters.detection_model = OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_MEDIUM;
    object_detection_parameters.instance_module_id = 1; // select instance ID

    if(enable_object_detection){
        print("Object Detection: Loading Module...");
        returned_state = zed.enableObjectDetection(object_detection_parameters);
        if (returned_state != ERROR_CODE::SUCCESS) {
            print("enableObjectDetection", returned_state, "\nExit program.");
            zed.close();
            return EXIT_FAILURE;
        }
    }

    // Detection runtime parameters
    ObjectDetectionRuntimeParameters detection_parameters_rt(detection_confidence);

    // To select a set of specific object classes. Exclude person, which will be tracked by the body tracking module
    detection_parameters_rt.object_class_filter = { OBJECT_CLASS::ELECTRONICS, OBJECT_CLASS::SPORT,
        OBJECT_CLASS::ANIMAL, OBJECT_CLASS::BAG, OBJECT_CLASS::VEHICLE, OBJECT_CLASS::FRUIT_VEGETABLE };


    // Detection runtime parameters
    // default detection threshold, apply to all object class
    BodyTrackingRuntimeParameters body_tracking_parameters_rt(body_detection_confidence);
    // Detection output
    bool quit = false;

    RuntimeParameters runtime_parameters;
    runtime_parameters.confidence_threshold = depth_confidence;
    runtime_parameters.measure3D_reference_frame = REFERENCE_FRAME::WORLD; // Camera or World frame


    Pose cam_w_pose;
    cam_w_pose.pose_data.setIdentity();
    Objects objects;
    Bodies skeletons;

    Mat image_left, depth_image, point_cloud;

    std::cout << "ZED2 node started" << std::endl;    

    int image_id = 0;

    ros::Rate loop_rate(500);

    // Main loop
    while ( ros::ok()) {

        auto grab_state = zed.grab(runtime_parameters);
        if (grab_state == ERROR_CODE::SUCCESS) {
            // Record the current time
            ros::Time time = ros::Time::now();
            string frame_id = "zed2";

            image_id++;
            if(image_id > 1000000){image_id = 0;}

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
            

            // Publish TF2 for camera pose. Base frame is the world frame
            static tf2_ros::TransformBroadcaster br;
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = time;
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = "zed2";
            transformStamped.transform.translation.x = cam_w_pose.pose_data.getTranslation().x;
            transformStamped.transform.translation.y = cam_w_pose.pose_data.getTranslation().y;
            transformStamped.transform.translation.z = cam_w_pose.pose_data.getTranslation().z;
            transformStamped.transform.rotation.x = cam_w_pose.pose_data.getOrientation().x;
            transformStamped.transform.rotation.y = cam_w_pose.pose_data.getOrientation().y;
            transformStamped.transform.rotation.z = cam_w_pose.pose_data.getOrientation().z;
            transformStamped.transform.rotation.w = cam_w_pose.pose_data.getOrientation().w;
            br.sendTransform(transformStamped);    

            /** Publish rgb image***/
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

                rgb_camera_info_pub.publish(rgb_camera_info);


                // Publish image with ID
                if(external_semantic_seg_on){
                    mask_kpts_msgs::ImageWithID img_with_id_msg;
                    img_with_id_msg.image = img_msg;
                    img_with_id_msg.id = image_id;
                    img_with_id_pub.publish(img_with_id_msg);
                }

            }

            /********  Publish Objects if detection is enabled  ******/ 
            simple_zed2_wrapper::ObjectsStamped objects_msg;

            if(enable_object_detection){

                detection_parameters_rt.detection_confidence_threshold = detection_confidence;
                // detection_parameters_rt.measure3D_reference_frame = REFERENCE_FRAME::CAMERA;
                returned_state = zed.retrieveObjects(objects, detection_parameters_rt, object_detection_parameters.instance_module_id);

                if(returned_state == ERROR_CODE::SUCCESS && publish_objects){
                    // std::cout << "Objects detected: " << objects.object_list.size() << std::endl;
                    try{
                        setRegularObjectsMsg(objects_msg, objects);
                    }catch(const std::exception& e){
                        std::cerr << e.what() << '\n';
                    }
                }
            }

            /********  Publish Bodies if body tracking is enabled  ******/
            if(enable_body_tracking)
            {
                body_tracking_parameters_rt.detection_confidence_threshold = body_detection_confidence;
                // body_tracking_parameters_rt.measure3D_reference_frame = REFERENCE_FRAME::CAMERA;
                returned_state = zed.retrieveBodies(skeletons, body_tracking_parameters_rt, body_tracking_parameters.instance_module_id);

                if(returned_state == ERROR_CODE::SUCCESS && publish_objects){
                    try{
                        setHumanBodyMsg(objects_msg, skeletons, pose_msg, detection_result_in_camera_frame);
                    }catch(const std::exception& e) {
                        std::cerr << e.what() << '\n';
                    }
                }  

            }
            
            // Publish Objects
            if((enable_object_detection || enable_body_tracking) && publish_objects)    
            {
                objects_msg.header.stamp = time;
                objects_msg.header.frame_id = frame_id;
                objects_pub.publish(objects_msg);
            }        

            /********  Publish depth images  ********/
            if(publish_depth){
                // Retrieve depth image and publish
                zed.retrieveMeasure(depth_image, MEASURE::DEPTH);
                cv::Mat cv_depth_image = slMat2cvMat(depth_image);

                // cv::imshow("Depth Image", cv_depth_image);
                // cv::waitKey(1);

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

                if(synchoronize_pose_depth_mask){
                    // Get mask group message
                    mask_kpts_msgs::MaskGroup mask_group_msg;
                    visualization_msgs::MarkerArray bboxes_msg;

                    setMaskKptsMsg(mask_group_msg, bboxes_msg, objects, skeletons, cv_depth_image.cols, cv_depth_image.rows);

                    if(external_semantic_seg_on){
                        // Wait for the semantic segmentation image.
                        int timeout_counter = 0;
                        bool timeout = false;
                        while(semantic_seg_image_id != image_id){
                            timeout_counter++;
                            if(timeout_counter > 250){ //Wait for 0.5s at most
                                timeout = true;
                                break;
                            }

                            loop_rate.sleep();
                            ros::spinOnce();
                        }

                        if(!timeout){
                            // Add the semantic segmentation mask to the mask group
                            mask_kpts_msgs::MaskKpts mask_kpts_msg;
                            mask_kpts_msg.label = "static";
                            mask_kpts_msg.track_id = 65535;
                            mask_kpts_msg.mask = semantic_seg_image;
                            mask_group_msg.objects.push_back(mask_kpts_msg);
                        }else{
                            ROS_WARN_THROTTLE(1, "Timeout waiting for semantic segmentation image");
                        }
                    }

                    mask_group_msg.header.stamp = time;
                    mask_group_msg.header.frame_id = frame_id;

                    mask_group_pub.publish(mask_group_msg);
                    depth_repub.publish(depth_img_msg);

                    pose_repub.publish(pose_msg);

                    bboxes_pub.publish(bboxes_msg);
                }else{
                    depth_mat_pub.publish(depth_img_msg);
                }
            }

            if(publish_point_cloud){
                // Retrieve point cloud and publish
                zed.retrieveMeasure(point_cloud, MEASURE::XYZBGRA);

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

                // Publish global point cloud
                if(publish_point_cloud_global)
                {
                    try {
                        // Look up for the transformation from camera frame ("zed2") to world frame. Orientation is identity as it has been set in the camera pose
                        sensor_msgs::PointCloud2 transformed_cloud;
                        transformStamped.transform.rotation.x = 0;
                        transformStamped.transform.rotation.y = 0;
                        transformStamped.transform.rotation.z = 0;
                        transformStamped.transform.rotation.w = 1;
                        tf2::doTransform(point_cloud_msg, transformed_cloud, transformStamped);
                        point_cloud_global_pub.publish(transformed_cloud);
                    } catch (tf2::TransformException &ex) {
                        ROS_WARN("%s", ex.what());
                        ros::Duration(0.001).sleep();
                    }
                }
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



/// @brief This function prints the message to the console with the error code and message
/// @param msg_prefix 
/// @param err_code 
/// @param msg_suffix 
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

/// @brief This function parses the command line arguments and sets the camera resolution and input mode
/// @param argc 
/// @param argv 
/// @param param 
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
        } else {
            param.camera_resolution = RESOLUTION::HD720;
            cout << "[Sample] Using Camera in resolution HD720" << endl;
        }

        if(argc > 2) {
            arg = string(argv[2]);
            if(arg.find("FPS30") != string::npos) {
                param.camera_fps = 30;
                cout << "[Sample] Using Camera in FPS 30" << endl;
            } else if(arg.find("FPS60") != string::npos) {
                param.camera_fps = 60;
                cout << "[Sample] Using Camera in FPS 60" << endl;
            } else {
                param.camera_fps = 30;
                cout << "[Sample] Using Camera in FPS 30" << endl;
            }
        }
        
        if(argc >3)
        {
            arg = string(argv[3]);
            if(arg.find("ULTRA") != string::npos) {
                param.depth_mode = DEPTH_MODE::ULTRA;
                cout << "[Sample] Using Camera in ULTRA depth mode" << endl;
            } else if(arg.find("PERFORMANCE") != string::npos) {
                param.depth_mode = DEPTH_MODE::PERFORMANCE;
                cout << "[Sample] Using Camera in PERFORMANCE depth mode" << endl;
            } else if(arg.find("QUALITY") != string::npos) {
                param.depth_mode = DEPTH_MODE::QUALITY;
                cout << "[Sample] Using Camera in QUALITY depth mode" << endl;
            } else if(arg.find("NEURAL") != string::npos) {
                param.depth_mode = DEPTH_MODE::NEURAL;
                cout << "[Sample] Using Camera in NEURAL depth mode" << endl;
            } else {
                param.depth_mode = DEPTH_MODE::ULTRA;
                cout << "[Sample] Using Camera in ULTRA depth mode" << endl;
            }
        }
        
    }
}


/// @brief Set the regular objects message using the results from object detection
/// @param objects_msg 
/// @param objects 
void setRegularObjectsMsg(simple_zed2_wrapper::ObjectsStamped &objects_msg, Objects &objects)
{
    if (&objects == nullptr) {
        ROS_ERROR("Received a null reference to sl::Objects.");
        return;
    }

    // Iterate over the list of detected objects
    for (int i = 0; i < objects.object_list.size(); i++) {
        simple_zed2_wrapper::Object obj;
        obj.label = toString(objects.object_list[i].label);
        obj.label_id = objects.object_list[i].id; // We use the object ID as the label ID. This is useful for tracking the same object across frames
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

        // 2D and 3D bounding box
        if(!objects.object_list[i].bounding_box_2d.empty() && !objects.object_list[i].bounding_box.empty())
        {
            vector<sl::uint2> object_2Dbbox = objects.object_list[i].bounding_box_2d;
            for(int k=0; k<4; ++k)
            {
                obj.bounding_box_2d.corners[k].kp[0] = object_2Dbbox[k].x;
                obj.bounding_box_2d.corners[k].kp[1] = object_2Dbbox[k].y;
            }

            vector<sl::float3> object_3Dbbox = objects.object_list[i].bounding_box;
            for(int k=0; k<8; ++k)
            {
                obj.bounding_box_3d.corners[k].kp[0] = object_3Dbbox[k].x;
                obj.bounding_box_3d.corners[k].kp[1] = object_3Dbbox[k].y;
                obj.bounding_box_3d.corners[k].kp[2] = object_3Dbbox[k].z;
            }

            // 3D dimensions
            obj.dimensions_3d[0] = objects.object_list[i].dimensions.x;
            obj.dimensions_3d[1] = objects.object_list[i].dimensions.y;
            obj.dimensions_3d[2] = objects.object_list[i].dimensions.z;
        }
        

        obj.skeleton_available = false;

        objects_msg.objects.push_back(obj);
    }
}


/// @brief Set the visual representation of the 3D bounding box using the results from object detection
/// @param msg 
/// @param object_3Dbbox 
/// @param id 
void setMarkerMsgfor3DBBox(visualization_msgs::Marker &msg, vector<sl::float3> &object_3Dbbox, int id)
{
    msg.header.frame_id = "zed2";
    msg.header.stamp = ros::Time::now();
    msg.ns = "bounding_boxes";
    msg.id = id;
    msg.type = visualization_msgs::Marker::LINE_LIST;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = 0.05;
    msg.color.r = 0.0;
    msg.color.g = 1.0;
    msg.color.b = 0.0;
    msg.color.a = 1.0;

    geometry_msgs::Point p0, p1, p2, p3, p4, p5, p6, p7;
    p0.x = object_3Dbbox[0].x; p0.y = object_3Dbbox[0].y; p0.z = object_3Dbbox[0].z;
    p1.x = object_3Dbbox[1].x; p1.y = object_3Dbbox[1].y; p1.z = object_3Dbbox[1].z;
    p2.x = object_3Dbbox[2].x; p2.y = object_3Dbbox[2].y; p2.z = object_3Dbbox[2].z;
    p3.x = object_3Dbbox[3].x; p3.y = object_3Dbbox[3].y; p3.z = object_3Dbbox[3].z;
    p4.x = object_3Dbbox[4].x; p4.y = object_3Dbbox[4].y; p4.z = object_3Dbbox[4].z;
    p5.x = object_3Dbbox[5].x; p5.y = object_3Dbbox[5].y; p5.z = object_3Dbbox[5].z;
    p6.x = object_3Dbbox[6].x; p6.y = object_3Dbbox[6].y; p6.z = object_3Dbbox[6].z;
    p7.x = object_3Dbbox[7].x; p7.y = object_3Dbbox[7].y; p7.z = object_3Dbbox[7].z;

    msg.points.push_back(p0); msg.points.push_back(p1);
    msg.points.push_back(p1); msg.points.push_back(p2);
    msg.points.push_back(p2); msg.points.push_back(p3);
    msg.points.push_back(p3); msg.points.push_back(p0);

    msg.points.push_back(p4); msg.points.push_back(p5);
    msg.points.push_back(p5); msg.points.push_back(p6);
    msg.points.push_back(p6); msg.points.push_back(p7);
    msg.points.push_back(p7); msg.points.push_back(p4);

    msg.points.push_back(p0); msg.points.push_back(p4);
    msg.points.push_back(p1); msg.points.push_back(p5);
    msg.points.push_back(p2); msg.points.push_back(p6);
    msg.points.push_back(p3); msg.points.push_back(p7);

}


/// @brief Set the mask and keypoints message using the results from object detection. Keypoints are in global frame and are generated with bbox_3d.
/// @param mask_group_msg 
/// @param marker_msg
/// @param objects 
/// @param skeletons 
void setMaskKptsMsg(mask_kpts_msgs::MaskGroup &mask_group_msg, visualization_msgs::MarkerArray &marker_msg,  Objects &objects, Bodies &skeletons, int image_width, int image_height)
{
    int num_objects = 0, num_persons = 0;

    if (&objects != nullptr) {
        for (int i = 0; i < objects.object_list.size(); i++) {

            if(objects.object_list[i].mask.isInit())
            {
                num_objects++;

                mask_kpts_msgs::MaskKpts mask_kpts;
                mask_kpts.label = toString(objects.object_list[i].label);
                int track_id = objects.object_list[i].id;

                if(track_id >= 20000){track_id = track_id % 20000;} // MAX SUPPORTED TRACK ID
                mask_kpts.track_id = track_id;
             
                // Init a black mask with the same size as the image and then fill the mask with the object mask using the bounding box
                cv::Mat mask = cv::Mat::zeros(image_height, image_width, CV_8UC1);
                cv::Mat ori_mask = slMat2cvMat(objects.object_list[i].mask);

                // Get the bounding box
                vector<sl::uint2> object_2Dbbox = objects.object_list[i].bounding_box_2d;

                // Copy the object mask to the mask
                for(int y=0; y<ori_mask.rows; ++y)
                {
                    for(int x=0; x<ori_mask.cols; ++x)
                    {
                        if(y+object_2Dbbox[0].y >= image_height || x+object_2Dbbox[0].x >= image_width){continue;}
                        mask.at<uchar>(y+object_2Dbbox[0].y, x+object_2Dbbox[0].x) = ori_mask.at<uchar>(y, x);
                    }
                }

                // cv::imshow("mask", mask);
                // cv::waitKey(1);

                // Convert the mask to a ROS message
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mask).toImageMsg();
                mask_kpts.mask = *msg;

                // Get the keypoints. 0, 1, 3, 4 corners of the 3d bounding box. Check https://www.stereolabs.com/docs/object-detection/using-object-detection for details
                
                if(!objects.object_list[i].bounding_box.empty() && !objects.object_list[i].bounding_box_2d.empty())
                {
                    vector<sl::float3> object_3Dbbox = objects.object_list[i].bounding_box;

                    mask_kpts_msgs::Keypoint kp0, kp1, kp2, kp3;
                    kp0.x = object_3Dbbox[0].x;
                    kp0.y = object_3Dbbox[0].y;
                    kp0.z = object_3Dbbox[0].z;

                    kp1.x = object_3Dbbox[1].x;
                    kp1.y = object_3Dbbox[1].y;
                    kp1.z = object_3Dbbox[1].z;

                    kp2.x = object_3Dbbox[3].x;
                    kp2.y = object_3Dbbox[3].y;
                    kp2.z = object_3Dbbox[3].z;

                    kp3.x = object_3Dbbox[4].x;
                    kp3.y = object_3Dbbox[4].y;
                    kp3.z = object_3Dbbox[4].z;

                    mask_kpts.kpts_curr.push_back(kp0);
                    mask_kpts.kpts_curr.push_back(kp1);
                    mask_kpts.kpts_curr.push_back(kp2);
                    mask_kpts.kpts_curr.push_back(kp3);

                    mask_kpts.kpts_last = mask_kpts.kpts_curr;

                    // 2D bounding box
                    mask_kpts.bbox_tl.x = object_2Dbbox[0].x;
                    mask_kpts.bbox_tl.y = object_2Dbbox[0].y;
                    mask_kpts.bbox_br.x = object_2Dbbox[2].x;
                    mask_kpts.bbox_br.y = object_2Dbbox[2].y;

                    mask_group_msg.objects.push_back(mask_kpts);

                    // Set the marker message for the 3D bounding box
                    visualization_msgs::Marker marker;
                    setMarkerMsgfor3DBBox(marker, object_3Dbbox, track_id);

                    marker_msg.markers.push_back(marker);
                }
            }
        }
    }

    if(&skeletons != nullptr)
    {
        for(auto &body : skeletons.body_list)
        {
            if(body.mask.isInit())
            {
                num_persons++;

                mask_kpts_msgs::MaskKpts mask_kpts;
                mask_kpts.label = "person";
                
                int track_id = body.id + 20000;
                if(track_id > 65535) {track_id = track_id % 40000 + 20000;} // MAX SUPPORTED TRACK ID
                mask_kpts.track_id = track_id;
                            
                // Init a black mask with the same size as the image and then fill the mask with the object mask using the bounding box
                cv::Mat mask = cv::Mat::zeros(image_height, image_width, CV_8UC1);
                cv::Mat ori_mask = slMat2cvMat(body.mask);

                // Get the bounding box
                vector<sl::uint2> object_2Dbbox = body.bounding_box_2d;

                // Copy the object mask to the mask
                for(int y=0; y<ori_mask.rows; ++y)
                {
                    for(int x=0; x<ori_mask.cols; ++x)
                    {
                        if(y+object_2Dbbox[0].y >= image_height || x+object_2Dbbox[0].x >= image_width){continue;}
                        mask.at<uchar>(y+object_2Dbbox[0].y, x+object_2Dbbox[0].x) = ori_mask.at<uchar>(y, x);
                    }
                }

                cv::imshow("mask_human", mask);
                cv::waitKey(1);

                // Convert the mask to a ROS message
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mask).toImageMsg();
                mask_kpts.mask = *msg;

                if(!body.bounding_box.empty() && !body.bounding_box_2d.empty())
                {
                    // Get the keypoints. 0, 1, 3, 4 corners of the 3d bounding box. Check https://www.stereolabs.com/docs/object-detection/using-object-detection for details
                    vector<sl::float3> object_3Dbbox = body.bounding_box;

                    mask_kpts_msgs::Keypoint kp0, kp1, kp2, kp3;

                    kp0.x = object_3Dbbox[0].x;
                    kp0.y = object_3Dbbox[0].y;
                    kp0.z = object_3Dbbox[0].z;

                    kp1.x = object_3Dbbox[1].x;
                    kp1.y = object_3Dbbox[1].y;
                    kp1.z = object_3Dbbox[1].z;

                    kp2.x = object_3Dbbox[3].x;
                    kp2.y = object_3Dbbox[3].y;
                    kp2.z = object_3Dbbox[3].z;

                    kp3.x = object_3Dbbox[4].x;
                    kp3.y = object_3Dbbox[4].y;
                    kp3.z = object_3Dbbox[4].z;

                    mask_kpts.kpts_curr.push_back(kp0);
                    mask_kpts.kpts_curr.push_back(kp1);
                    mask_kpts.kpts_curr.push_back(kp2);
                    mask_kpts.kpts_curr.push_back(kp3);

                    mask_kpts.kpts_last = mask_kpts.kpts_curr;

                    // 2D bounding box
                    mask_kpts.bbox_tl.x = object_2Dbbox[0].x;
                    mask_kpts.bbox_tl.y = object_2Dbbox[0].y;
                    mask_kpts.bbox_br.x = object_2Dbbox[2].x;
                    mask_kpts.bbox_br.y = object_2Dbbox[2].y;

                    mask_group_msg.objects.push_back(mask_kpts);

                    // Set the marker message for the 3D bounding box
                    visualization_msgs::Marker marker;
                    setMarkerMsgfor3DBBox(marker, object_3Dbbox, track_id);

                    marker_msg.markers.push_back(marker);
                }
            }
        }
    }

    std::cout << "Number of objects: " << num_objects << ", Number of persons: " << num_persons << std::endl;
}


/// @brief Set the human body message using the results from body tracking
/// @param objects_msg 
/// @param skeletons 
void setHumanBodyMsg(simple_zed2_wrapper::ObjectsStamped &objects_msg, Bodies &skeletons, geometry_msgs::PoseStamped &camera_pose, bool detection_result_in_camera_frame = true)
{
    if (&skeletons == nullptr) {
        ROS_ERROR("Received a null reference to sl::Bodies.");
        return;
    }

    tf2::Transform cam_w_transform;
    tf2::fromMsg(camera_pose.pose, cam_w_transform);
    tf2::Transform cam_w_transform_inv = cam_w_transform.inverse();

    // Iterate over the list of detected persons
    for(auto &body : skeletons.body_list)
    {
        simple_zed2_wrapper::Object obj;
        obj.label = "person";
        obj.body_format = 0; // BODY_FORMAT::BODY_18
        obj.label_id = body.id;
        obj.confidence = body.confidence;

        if(!detection_result_in_camera_frame)
        {
            obj.position[0] = body.position.x;
            obj.position[1] = body.position.y;
            obj.position[2] = body.position.z;
        }else{
            // Transform the position from wolrd frame to camera frame by multiplying the inverse of the camera pose
            tf2::Vector3 pos(body.position.x, body.position.y, body.position.z);
            tf2::Vector3 pos_cam = cam_w_transform_inv * pos;
            obj.position[0] = pos_cam.x();
            obj.position[1] = -pos_cam.y(); // Invert the y-axis to match the camera frame
            obj.position[2] = -pos_cam.z(); // Invert the z-axis to match the camera frame
        }

        obj.position_covariance[0] = body.position_covariance[0];
        obj.position_covariance[1] = body.position_covariance[1];
        obj.position_covariance[2] = body.position_covariance[2];
        obj.position_covariance[3] = body.position_covariance[3];
        obj.position_covariance[4] = body.position_covariance[4];
        obj.position_covariance[5] = body.position_covariance[5];

        obj.velocity[0] = body.velocity.x;
        obj.velocity[1] = body.velocity.y;
        obj.velocity[2] = body.velocity.z;

        obj.dimensions_3d[0] = body.dimensions.x;
        obj.dimensions_3d[1] = body.dimensions.y;
        obj.dimensions_3d[2] = body.dimensions.z;

        // Tracking state
        switch (body.tracking_state)
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

        // 2D and 3D bounding box
        if(!body.bounding_box_2d.empty() && !body.bounding_box.empty())
        {
            vector<sl::uint2> object_2Dbbox = body.bounding_box_2d;
            for(int k=0; k<4; ++k)
            {
                obj.bounding_box_2d.corners[k].kp[0] = object_2Dbbox[k].x;
                obj.bounding_box_2d.corners[k].kp[1] = object_2Dbbox[k].y;
            }

            vector<sl::float3> object_3Dbbox = body.bounding_box;
            for(int k=0; k<8; ++k)
            {
                if(!detection_result_in_camera_frame)
                {
                    obj.bounding_box_3d.corners[k].kp[0] = object_3Dbbox[k].x;
                    obj.bounding_box_3d.corners[k].kp[1] = object_3Dbbox[k].y;
                    obj.bounding_box_3d.corners[k].kp[2] = object_3Dbbox[k].z;
                }else{
                    // Transform the position from wolrd frame to camera frame by multiplying the inverse of the camera pose
                    tf2::Vector3 pos(object_3Dbbox[k].x, object_3Dbbox[k].y, object_3Dbbox[k].z);
                    tf2::Vector3 pos_cam = cam_w_transform_inv * pos;
                    obj.bounding_box_3d.corners[k].kp[0] = pos_cam.x();
                    obj.bounding_box_3d.corners[k].kp[1] = -pos_cam.y(); // Invert the y-axis to match the camera frame
                    obj.bounding_box_3d.corners[k].kp[2] = -pos_cam.z(); // Invert the z-axis to match the camera frame
                
                }
            }
        }

        obj.skeleton_available = true;

        // head_bounding_box_2d and head_bounding_box_3d
        if(!body.head_bounding_box_2d.empty() && !body.head_bounding_box.empty())
        {
            vector<sl::uint2> head_2Dbbox = body.head_bounding_box_2d;
            for(int k=0; k<4; ++k)
            {
                obj.head_bounding_box_2d.corners[k].kp[0] = head_2Dbbox[k].x;
                obj.head_bounding_box_2d.corners[k].kp[1] = head_2Dbbox[k].y;
            }

            vector<sl::float3> head_3Dbbox = body.head_bounding_box;
            for(int k=0; k<8; ++k)
            {
                if(!detection_result_in_camera_frame)
                {
                    obj.head_bounding_box_3d.corners[k].kp[0] = head_3Dbbox[k].x;
                    obj.head_bounding_box_3d.corners[k].kp[1] = head_3Dbbox[k].y;
                    obj.head_bounding_box_3d.corners[k].kp[2] = head_3Dbbox[k].z;
                }else{
                    // Transform the position from wolrd frame to camera frame by multiplying the inverse of the camera pose
                    tf2::Vector3 pos(head_3Dbbox[k].x, head_3Dbbox[k].y, head_3Dbbox[k].z);
                    tf2::Vector3 pos_cam = cam_w_transform_inv * pos;
                    obj.head_bounding_box_3d.corners[k].kp[0] = pos_cam.x();
                    obj.head_bounding_box_3d.corners[k].kp[1] = -pos_cam.y(); // Invert the y-axis to match the camera frame
                    obj.head_bounding_box_3d.corners[k].kp[2] = -pos_cam.z(); // Invert the z-axis to match the camera frame
                }
            }
        }

        // Head position
        obj.head_position[0] = body.head_position.x;
        obj.head_position[1] = body.head_position.y;
        obj.head_position[2] = body.head_position.z;

        // skeleton_2d and skeleton_3d
        if(!body.keypoint_2d.empty() && !body.keypoint.empty())
        {
            for(int j=0; j<body.keypoint.size(); ++j)
            {
                obj.skeleton_2d.keypoints[j].kp[0] = body.keypoint_2d[j].x;
                obj.skeleton_2d.keypoints[j].kp[1] = body.keypoint_2d[j].y;

                if(!detection_result_in_camera_frame)
                {
                    obj.skeleton_3d.keypoints[j].kp[0] = body.keypoint[j].x;
                    obj.skeleton_3d.keypoints[j].kp[1] = body.keypoint[j].y;
                    obj.skeleton_3d.keypoints[j].kp[2] = body.keypoint[j].z;
                }else{
                    // Transform the position from wolrd frame to camera frame by multiplying the inverse of the camera pose
                    tf2::Vector3 pos(body.keypoint[j].x, body.keypoint[j].y, body.keypoint[j].z);
                    tf2::Vector3 pos_cam = cam_w_transform_inv * pos;
                    obj.skeleton_3d.keypoints[j].kp[0] = pos_cam.x();
                    obj.skeleton_3d.keypoints[j].kp[1] = -pos_cam.y(); // Invert the y-axis to match the camera frame
                    obj.skeleton_3d.keypoints[j].kp[2] = -pos_cam.z(); // Invert the z-axis to match the camera frame
                }
            }
        }
        
        objects_msg.objects.push_back(obj);

    }
}


