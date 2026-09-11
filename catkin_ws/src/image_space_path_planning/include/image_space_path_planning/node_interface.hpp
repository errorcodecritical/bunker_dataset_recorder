#pragma once

// === Current Package ===
#include "image_space_path_planning/path_deformer.hpp"
#include "image_space_path_planning/path_optimizer.hpp"
#include "image_space_path_planning/path_projector.hpp"
// === Third-Party ===
// ROS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_listener.h>
// NON-ROS
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
// === Standard Library ===
#include <deque>
#include <mutex>

class NodeInterface {
public:
    NodeInterface(ros::NodeHandle& nh);
    void run();
    Eigen::MatrixXf transformPointsToCameraFrame(
        const Eigen::MatrixXf& points,
        const Eigen::Matrix4f& map_T_robot,
        const Eigen::Matrix4f& robot_T_camera
    );
    Eigen::MatrixXf transformPointsToMapFrame(
        const Eigen::MatrixXf& points,
        const Eigen::Matrix4f& map_T_robot,
        const Eigen::Matrix4f& robot_T_camera
    );

private:
    std::mutex image_queue_mutex_;
    std::mutex path_queue_mutex_;
    std::mutex depth_queue_mutex_;
    std::mutex relative_depth_queue_mutex_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;


    bool debug_project_only_ = false;

    void loadParams(ros::NodeHandle& nh);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
    void relativeDepthCallback(const sensor_msgs::Image::ConstPtr& msg);
    cv::Mat getImageFromDeque();
    cv::Mat getDepthFromDeque();
    cv::Mat getRelativeDepthFromDeque();
    Eigen::MatrixXf getPathFromDeque();
    Eigen::Matrix4f getRobotPose();
    void publishOptimizedPath(
        const Eigen::MatrixXf& path,
        const std::string& frame_id,
        ros::Publisher& pub
    );

    ros::Subscriber path_sub_, image_sub_, depth_sub_, rel_depth_sub_;
    ros::Publisher path_pub_;
    PathDeformer path_deformer_;
    PathProjector path_projector_;
    PathOptimizer path_optimizer_;

    // Inputs and config parameters
    std::deque<cv::Mat> image_queue_;
    std::deque<cv::Mat> depth_queue_;
    std::deque<Eigen::MatrixXf> path_queue_;
    std::deque<cv::Mat> relative_depth_queue_;
    std::string path_topic_;
    std::string image_topic_;
    std::string depth_topic_;
    std::string relative_depth_topic_;
    std::string path_publisher_topic_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    int image_width_;
    int image_height_;
    Eigen::Matrix4f map_T_robot_;
    Eigen::Matrix4f robot_T_camera_;
    bool robot_pose_okay = true;
    float local_horizon_distance_;
    bool convert_depth_from_mm_to_m_;
    std::string robot_frame_;
    std::string world_frame_;
    std::string camera_optical_frame_;
};