// path_deformer.hpp
#pragma once

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <vector>

class PathDeformer {
public:
    PathDeformer() = default;
    Eigen::MatrixXf deformPath(
        const Eigen::MatrixXf& input_path,
        const cv::Mat& image
    );
    Eigen::MatrixXf deformPathStripSearch(
        const Eigen::MatrixXf& input_path,
        const cv::Mat& image,
        int strip_height = 10,
        int x_search_radius = 30
    );
    Eigen::MatrixXf computeMeanPatchTraversability(
        const cv::Mat& integral_img, 
        const Eigen::MatrixXf& input_path, 
        int patch_size
    );
    Eigen::MatrixXf computePointAcceptance(
        const Eigen::MatrixXf& traversability_values,
        float threshold
    );
    Eigen::MatrixXf findBetterPatch(
        const Eigen::MatrixXf& input_path,
        const cv::Mat& integral_img,
        const Eigen::MatrixXf& acceptable_points,
        int search_radius,
        int patch_size,
        float alpha,
        float beta
    );
};