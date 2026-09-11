// path_optimizer.hpp
#pragma once

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <vector>
#include <ceres/ceres.h>
#include <ceres/jet.h>

class PathOptimizer {
public:
    PathOptimizer() = default;
    Eigen::MatrixXf smoothPathOutliers(
        const Eigen::MatrixXf& path,
        const int threshold = 5
    );
    Eigen::MatrixXf optimizePathWithCeres(
        const Eigen::MatrixXf& path_2d,
        const cv::Mat& traversability_image
    );

    struct SmoothnessCost {
        SmoothnessCost(double weight) : weight_(weight) {}  // Modified constructor
    
        template <typename T>
        bool operator()(const T* const prev,
                        const T* const curr,
                        const T* const next,
                        T* residual) const {
            T midpoint[2];
            midpoint[0] = (prev[0] + next[0]) * T(0.5);
            midpoint[1] = (prev[1] + next[1]) * T(0.5);
            
            // Encourage current point to stay between optimized neighbors
            residual[0] = T(weight_) * (curr[0] - midpoint[0]);
            residual[1] = T(weight_) * (curr[1] - midpoint[1]);
            return true;
        }
    
        double weight_;
    };
    
    struct TraversabilityCost {
        TraversabilityCost(const cv::Mat& image, double weight)
            : image_(image), weight_(weight) {}
    
        template <typename U>
        double GetScalar(const U& val) const {
            if constexpr (std::is_class_v<U>) {
                return val.a;  // For Jet types
            } else {
                return val;    // For plain doubles
            }
        }
    
        template <typename T>
        bool operator()(const T* point, T* residual) const {
            T x = point[0];
            T y = point[1];
    
            T x_floor = ceres::floor(x);
            T y_floor = ceres::floor(y);
            T dx = x - x_floor;
            T dy = y - y_floor;
    
            // Use GetScalar to handle both Jet and double
            int xi = static_cast<int>(GetScalar(x_floor));
            int yi = static_cast<int>(GetScalar(y_floor));
    
            // Check boundaries (adjust for interpolation)
            if (xi < 0 || xi >= image_.cols - 1 || yi < 0 || yi >= image_.rows - 1) {
                residual[0] = T(weight_) * T(1.0);
                return true;
            }
    
            // Bilinear interpolation with proper parentheses
            double p00 = image_.at<uchar>(yi, xi) / 255.0;
            double p01 = image_.at<uchar>(yi, xi + 1) / 255.0;
            double p10 = image_.at<uchar>(yi + 1, xi) / 255.0;
            double p11 = image_.at<uchar>(yi + 1, xi + 1) / 255.0;
    
            T interpolated = 
                (T(1.0) - dx) * (T(1.0) - dy) * T(p00) +  // Term 1
                dx * (T(1.0) - dy) * T(p01) +              // Term 2
                (T(1.0) - dx) * dy * T(p10) +              // Term 3
                dx * dy * T(p11);                          // Term 4
    
            residual[0] = T(weight_) * (interpolated);
            return true;
        }
    
        const cv::Mat& image_;
        double weight_;
    };
};