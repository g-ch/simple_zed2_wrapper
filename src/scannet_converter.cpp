/**
 * @file scannet_converter.cpp
 * @author Clarence (g.chen-5@tudelft.nl)
 * @brief ROS node that subscribes to ZED2 topics and saves data in ScanNet format
 * @version 0.1
 * @date 2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>


class ScanNetConverter {
private:
    ros::NodeHandle nh_;
    ros::Subscriber rgb_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber camera_info_sub_;
    
    std::string output_dir_;
    int frame_counter_;
    
    // Synchronized data storage
    sensor_msgs::ImageConstPtr rgb_msg_;
    sensor_msgs::ImageConstPtr depth_msg_;
    geometry_msgs::PoseStampedConstPtr pose_msg_;
    
    // Camera info storage
    sensor_msgs::CameraInfo camera_info_;
    bool camera_info_received_;
    int color_width_;
    int color_height_;
    int depth_width_;
    int depth_height_;
    
    ros::Time last_rgb_time_;
    ros::Time last_depth_time_;
    ros::Time last_pose_time_;
    ros::Time last_save_time_;
    
    double time_tolerance_; // Maximum time difference for synchronization (in seconds)
    double save_frequency_; // Saving frequency in Hz
    double min_save_interval_; // Minimum time between saves (1/frequency)
    
    bool saveData();
    void convertPoseToMatrix(const geometry_msgs::PoseStamped& pose, std::ofstream& file);
    
public:
    ScanNetConverter();
    ~ScanNetConverter();
    void rgbCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthCallback(const sensor_msgs::ImageConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void writeInfoFile(); // Public so signal handler can access it
};

ScanNetConverter::ScanNetConverter() : frame_counter_(0), time_tolerance_(0.1), save_frequency_(10.0),
    camera_info_received_(false), color_width_(0), color_height_(0), depth_width_(0), depth_height_(0) {
    ros::NodeHandle nh_private("~");
    
    // Get output directory parameter
    nh_private.param<std::string>("output_dir", output_dir_, "./scannet_data");
    nh_private.param<double>("time_tolerance", time_tolerance_, 0.1);
    nh_private.param<double>("save_frequency", save_frequency_, 10.0);
    
    // Calculate minimum time interval between saves
    if (save_frequency_ > 0.0) {
        min_save_interval_ = 1.0 / save_frequency_;
    } else {
        min_save_interval_ = 0.0; // Save all frames if frequency is 0 or negative
        ROS_WARN("Invalid save_frequency (%.2f Hz). Saving all synchronized frames.", save_frequency_);
    }
    
    last_save_time_ = ros::Time(0);
    
    // Create output directory
    mkdir(output_dir_.c_str(), 0755);
    
    ROS_INFO("ScanNet Converter initialized");
    ROS_INFO("Output directory: %s", output_dir_.c_str());
    ROS_INFO("Time tolerance: %.3f seconds", time_tolerance_);
    ROS_INFO("Save frequency: %.2f Hz (min interval: %.3f seconds)", save_frequency_, min_save_interval_);
    
    // Subscribe to topics
    rgb_sub_ = nh_.subscribe("zed2/left/rgb/image", 1, &ScanNetConverter::rgbCallback, this);
    depth_sub_ = nh_.subscribe("zed2/left/depth/image", 1, &ScanNetConverter::depthCallback, this);
    pose_sub_ = nh_.subscribe("zed2/pose_stamped", 1, &ScanNetConverter::poseCallback, this);
    camera_info_sub_ = nh_.subscribe("zed2/left/rgb/camera_info", 1, &ScanNetConverter::cameraInfoCallback, this);
}

ScanNetConverter::~ScanNetConverter() {
    writeInfoFile();
}

void ScanNetConverter::rgbCallback(const sensor_msgs::ImageConstPtr& msg) {
    rgb_msg_ = msg;
    last_rgb_time_ = msg->header.stamp;
    
    // Store image dimensions
    if (color_width_ == 0 || color_height_ == 0) {
        color_width_ = msg->width;
        color_height_ = msg->height;
    }
    
    // Try to save if we have all synchronized data
    if (rgb_msg_ && depth_msg_ && pose_msg_) {
        ros::Time rgb_time = rgb_msg_->header.stamp;
        ros::Time depth_time = depth_msg_->header.stamp;
        ros::Time pose_time = pose_msg_->header.stamp;
        
        // Check if timestamps are close enough (within tolerance)
        double time_diff1 = std::abs((rgb_time - depth_time).toSec());
        double time_diff2 = std::abs((rgb_time - pose_time).toSec());
        double time_diff3 = std::abs((depth_time - pose_time).toSec());
        
        if (time_diff1 < time_tolerance_ && time_diff2 < time_tolerance_ && time_diff3 < time_tolerance_) {
            // Check if enough time has passed since last save (rate limiting)
            ros::Time current_time = rgb_time;
            double time_since_last_save = (current_time - last_save_time_).toSec();
            
            if (min_save_interval_ <= 0.0 || time_since_last_save >= min_save_interval_) {
                if (saveData()) {
                    last_save_time_ = current_time;
                    // Clear messages after successful save
                    rgb_msg_.reset();
                    depth_msg_.reset();
                    pose_msg_.reset();
                }
            }
        }
    }
}

void ScanNetConverter::depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    depth_msg_ = msg;
    last_depth_time_ = msg->header.stamp;
    
    // Store image dimensions
    if (depth_width_ == 0 || depth_height_ == 0) {
        depth_width_ = msg->width;
        depth_height_ = msg->height;
    }
    
    // Try to save if we have all synchronized data
    if (rgb_msg_ && depth_msg_ && pose_msg_) {
        ros::Time rgb_time = rgb_msg_->header.stamp;
        ros::Time depth_time = depth_msg_->header.stamp;
        ros::Time pose_time = pose_msg_->header.stamp;
        
        double time_diff1 = std::abs((rgb_time - depth_time).toSec());
        double time_diff2 = std::abs((rgb_time - pose_time).toSec());
        double time_diff3 = std::abs((depth_time - pose_time).toSec());
        
        if (time_diff1 < time_tolerance_ && time_diff2 < time_tolerance_ && time_diff3 < time_tolerance_) {
            if (saveData()) {
                rgb_msg_.reset();
                depth_msg_.reset();
                pose_msg_.reset();
            }
        }
    }
}

void ScanNetConverter::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    pose_msg_ = msg;
    last_pose_time_ = msg->header.stamp;
    
    // Try to save if we have all synchronized data
    if (rgb_msg_ && depth_msg_ && pose_msg_) {
        ros::Time rgb_time = rgb_msg_->header.stamp;
        ros::Time depth_time = depth_msg_->header.stamp;
        ros::Time pose_time = pose_msg_->header.stamp;
        
        double time_diff1 = std::abs((rgb_time - depth_time).toSec());
        double time_diff2 = std::abs((rgb_time - pose_time).toSec());
        double time_diff3 = std::abs((depth_time - pose_time).toSec());
        
        if (time_diff1 < time_tolerance_ && time_diff2 < time_tolerance_ && time_diff3 < time_tolerance_) {
            if (saveData()) {
                rgb_msg_.reset();
                depth_msg_.reset();
                pose_msg_.reset();
            }
        }
    }
}

void ScanNetConverter::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    if (!camera_info_received_) {
        camera_info_ = *msg;
        camera_info_received_ = true;
        ROS_INFO("Received camera info: %dx%d", msg->width, msg->height);
    }
}

void ScanNetConverter::writeInfoFile() {
    std::string info_path = output_dir_ + "/_info.txt";
    std::ofstream info_file(info_path);
    
    if (!info_file.is_open()) {
        ROS_ERROR("Failed to open info file for writing: %s", info_path.c_str());
        return;
    }
    
    info_file << std::fixed << std::setprecision(6);
    
    // Write fixed values
    info_file << "m_versionNumber = 4\n";
    info_file << "m_sensorName = StructureSensor (calibrated)\n";
    
    // Write image dimensions (use actual values if available, otherwise use camera_info)
    int color_w = (color_width_ > 0) ? color_width_ : (camera_info_received_ ? camera_info_.width : 1296);
    int color_h = (color_height_ > 0) ? color_height_ : (camera_info_received_ ? camera_info_.height : 968);
    int depth_w = (depth_width_ > 0) ? depth_width_ : 640;
    int depth_h = (depth_height_ > 0) ? depth_height_ : 480;
    
    info_file << "m_colorWidth = " << color_w << "\n";
    info_file << "m_colorHeight = " << color_h << "\n";
    info_file << "m_depthWidth = " << depth_w << "\n";
    info_file << "m_depthHeight = " << depth_h << "\n";
    info_file << "m_depthShift = 1000\n";
    
    // Write camera intrinsics (from camera_info, or use defaults if not received)
    if (camera_info_received_ && camera_info_.K.size() >= 9) {
        // Convert 3x3 K matrix to 4x4 (row-major format)
        // K matrix: [fx  0  cx]
        //           [ 0 fy  cy]
        //           [ 0  0   1]
        // 4x4 format: [fx  0  cx  0]
        //             [ 0 fy  cy  0]
        //             [ 0  0   1  0]
        //             [ 0  0   0  1]
        
        // Color intrinsic (4x4 matrix format, row-major)
        info_file << "m_calibrationColorIntrinsic = ";
        info_file << camera_info_.K[0] << " " << camera_info_.K[1] << " " << camera_info_.K[2] << " 0 ";
        info_file << camera_info_.K[3] << " " << camera_info_.K[4] << " " << camera_info_.K[5] << " 0 ";
        info_file << camera_info_.K[6] << " " << camera_info_.K[7] << " " << camera_info_.K[8] << " 0 ";
        info_file << "0 0 0 1\n";
        
        // Depth intrinsic (same as color for ZED2)
        info_file << "m_calibrationDepthIntrinsic = ";
        info_file << camera_info_.K[0] << " " << camera_info_.K[1] << " " << camera_info_.K[2] << " 0 ";
        info_file << camera_info_.K[3] << " " << camera_info_.K[4] << " " << camera_info_.K[5] << " 0 ";
        info_file << camera_info_.K[6] << " " << camera_info_.K[7] << " " << camera_info_.K[8] << " 0 ";
        info_file << "0 0 0 1\n";
    } else {
        // Default values if camera_info not received
        // K: [530.4337158203125, 0.0, 627.7291870117188, 0.0, 530.4337158203125, 350.12493896484375, 0.0, 0.0, 1.0]
        ROS_WARN("Camera info not received, using default intrinsics");
        info_file << "m_calibrationColorIntrinsic = 530.433716 0 627.729187 0 0 530.433716 350.124939 0 0 0 1 0 0 0 0 1\n";
        info_file << "m_calibrationDepthIntrinsic = 530.433716 0 627.729187 0 0 530.433716 350.124939 0 0 0 1 0 0 0 0 1\n";
    }
    
    // Write identity extrinsics (color and depth are aligned in ZED2)
    info_file << "m_calibrationColorExtrinsic = 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n";
    info_file << "m_calibrationDepthExtrinsic = 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n";
    
    // Write frame count
    info_file << "m_frames.size = " << frame_counter_ << "\n";
    info_file << "selected_frames_count = " << frame_counter_ << "\n";
    
    info_file.close();
    ROS_INFO("Info file written: %s (saved %d frames)", info_path.c_str(), frame_counter_);
}

void ScanNetConverter::convertPoseToMatrix(const geometry_msgs::PoseStamped& pose, std::ofstream& file) {
    // Convert quaternion to rotation matrix
    tf2::Quaternion q(
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w
    );
    
    tf2::Matrix3x3 rot(q);
    
    // Write 4x4 transformation matrix (row-major)
    file << std::fixed << std::setprecision(6);
    file << rot[0][0] << " " << rot[0][1] << " " << rot[0][2] << " " << pose.pose.position.x << "\n";
    file << rot[1][0] << " " << rot[1][1] << " " << rot[1][2] << " " << pose.pose.position.y << "\n";
    file << rot[2][0] << " " << rot[2][1] << " " << rot[2][2] << " " << pose.pose.position.z << "\n";
    file << "0 0 0 1\n";
}

bool ScanNetConverter::saveData() {
    try {
        // Format frame number as 6-digit string
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << frame_counter_;
        std::string frame_str = ss.str();
        
        // 1. Save RGB image as JPG
        cv_bridge::CvImagePtr cv_rgb;
        try {
            cv_rgb = cv_bridge::toCvCopy(rgb_msg_, "bgr8");
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception (RGB): %s", e.what());
            return false;
        }
        
        std::string rgb_path = output_dir_ + "/frame-" + frame_str + ".color.jpg";
        cv::imwrite(rgb_path, cv_rgb->image);
        
        // 2. Save depth image as PGM (16-bit, in millimeters)
        cv_bridge::CvImagePtr cv_depth;
        try {
            // Depth is 32FC1 (float, meters), convert to 16-bit unsigned (millimeters)
            cv_depth = cv_bridge::toCvCopy(depth_msg_, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception (Depth): %s", e.what());
            return false;
        }
        
        // Convert from meters (float) to millimeters (16-bit unsigned)
        // Clamp to 65535 so values > 65.535 m don't wrap; invalid/NaN become 0
        cv::Mat depth_mm;
        cv_depth->image.convertTo(depth_mm, CV_16UC1, 1000.0); // Multiply by 1000 to convert m to mm
        cv::Mat cap(depth_mm.size(), CV_16UC1);
        cap.setTo(cv::Scalar(65535));
        cv::min(depth_mm, cap, depth_mm);
        
        std::string depth_path = output_dir_ + "/frame-" + frame_str + ".depth.pgm";
        
        // PGM P5 16-bit format requires BIG-ENDIAN byte order (Netpbm spec).
        // cv::Mat stores native (little-endian on x86/ARM), so we must byte-swap when writing.
        std::ofstream pgm_file(depth_path, std::ios::binary);
        if (!pgm_file.is_open()) {
            ROS_ERROR("Failed to open depth file for writing: %s", depth_path.c_str());
            return false;
        }
        
        pgm_file << "P5\n";
        pgm_file << depth_mm.cols << " " << depth_mm.rows << "\n";
        pgm_file << "65535\n";
        
        const int total = depth_mm.rows * depth_mm.cols;
        const uint16_t* src = depth_mm.ptr<uint16_t>(0);
        std::vector<char> row(2);
        for (int i = 0; i < total; ++i) {
            uint16_t v = src[i];
            row[0] = static_cast<char>((v >> 8) & 0xFF);
            row[1] = static_cast<char>(v & 0xFF);
            pgm_file.write(row.data(), 2);
        }
        pgm_file.close();
        
        // 3. Save pose as text file with 4x4 matrix
        std::string pose_path = output_dir_ + "/frame-" + frame_str + ".pose.txt";
        std::ofstream pose_file(pose_path);
        if (!pose_file.is_open()) {
            ROS_ERROR("Failed to open pose file for writing: %s", pose_path.c_str());
            return false;
        }
        
        convertPoseToMatrix(*pose_msg_, pose_file);
        pose_file.close();
        
        frame_counter_++;
        
        if (frame_counter_ % 100 == 0) {
            ROS_INFO("Saved %d frames", frame_counter_);
        }
        
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in saveData: %s", e.what());
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "scannet_converter");
    
    ScanNetConverter converter;
    
    ROS_INFO("ScanNet Converter node started. Waiting for synchronized data...");
    
    ros::spin();
    
    return 0;
}

