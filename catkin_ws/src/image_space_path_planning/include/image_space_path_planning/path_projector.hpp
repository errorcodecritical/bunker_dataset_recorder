// path_projector.hpp
#pragma once

#include <Eigen/Core>
#include <Eigen/QR>
#include <opencv2/opencv.hpp>

#include <random>
#include <algorithm>

#include <vector>

class PathProjector {
public:
    PathProjector() = default;
    Eigen::MatrixXf projectTo2D(
        const Eigen::MatrixXf& path_3d,         // Nx3 matrix
        const cv::Mat& camera_matrix,           // 3x3
        const int image_width = 640,            // default value
        const int image_height = 480,           // default value
        const cv::Mat& dist_coeffs = cv::Mat()  // Optional distortion coefficients
    );
    Eigen::MatrixXf projectTo3D(
        const Eigen::MatrixXf& path_2d,         // Nx2 pixel coordinates
        const cv::Mat& depth_map,               // CV_32FC1 depth image
        const cv::Mat& camera_matrix,           // 3x3 intrinsic matrix
        const cv::Mat& dist_coeffs = cv::Mat()  // Optional distortion coefficients
    );
    cv::Mat drawProjectedPath(
        const Eigen::MatrixXf& path_2d,
        const cv::Mat& base_image,
        const cv::Scalar& color,
        const int radius,
        const int thickness = -1,
        const std::string& window_name = "Projected Path"  // default value
    );
    void visualizeProjections(
        const cv::Mat& image,
        const int delay = 1,
        const std::string& window_name = "Projected Path"  // default value
    );
    Eigen::MatrixXf cropPathToLocalHorizon(
        const Eigen::MatrixXf& path_3d, // Complete path projected to 3D
        float local_horizon_distance = 10.0f,  // default value
        Eigen::MatrixXf* path_3d_ignore = nullptr // Output parameter for far path (to be ignored in optimization)
    );
    Eigen::MatrixXf removeInvalidPoints(const Eigen::MatrixXf& path);

    cv::Mat normalizeDepthMapWithRealData(
        const cv::Mat& relative_depth,
        const cv::Mat& real_depth,
        float max_depth_m = 6.0f,
        int grid_rows = 10,
        int grid_cols = 10
    );
    Eigen::MatrixXf removeClosePoints(
        const Eigen::MatrixXf& path,
        float min_distance = 0.2f
    );
};