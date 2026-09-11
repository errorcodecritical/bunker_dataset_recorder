#include "image_space_path_planning/path_optimizer.hpp"

Eigen::MatrixXf PathOptimizer::smoothPathOutliers(const Eigen::MatrixXf& path, const int threshold)
{
    Eigen::MatrixXf optimized = path;

    int N = path.rows();
    std::vector<float> deviations;

    for (int i = 1; i < N - 1; ++i) {
        Eigen::Vector2f prev = path.row(i - 1);
        Eigen::Vector2f next = path.row(i + 1);
        Eigen::Vector2f center = (prev + next) * 0.5f;
        float deviation = path(i, 0) - center(0);  // horizontal deviation
        deviations.push_back(deviation);
    }

    // Compute dominant sign
    int count_left = std::count_if(deviations.begin(), deviations.end(), [](float d){ return d < 0; });
    int count_right = deviations.size() - count_left;
    int dominant_sign = (count_right >= count_left) ? 1 : -1;

    for (int i = 1; i < N - 1; ++i) {
        float deviation = deviations[i - 1];
        if (std::signbit(deviation) != std::signbit(dominant_sign) && std::abs(deviation) > threshold) {
            // Mark as outlier and adjust
            Eigen::Vector2f prev = path.row(i - 1);
            Eigen::Vector2f next = path.row(i + 1);
            Eigen::Vector2f center = (prev + next) * 0.5f;
            optimized.row(i) = center;  // simple smoothing, or rerun search here
        }
    }

    return optimized;
}

Eigen::MatrixXf PathOptimizer::optimizePathWithCeres(const Eigen::MatrixXf& path_2d, const cv::Mat& traversability_image) {
    assert(path_2d.cols() == 2);
    int N = path_2d.rows();
    if (N < 3) return path_2d;

    Eigen::MatrixXf optimized_path = path_2d;

    std::vector<std::array<double, 2>> points(N);
    for (int i = 0; i < N; ++i) {
        points[i][0] = path_2d(i, 0);
        points[i][1] = path_2d(i, 1);
    }

    ceres::Problem problem;
    double smooth_weight = 0.8;
    double traversability_weight = 0.2;

    for (int i = 0; i < N; ++i) {
        problem.AddParameterBlock(points[i].data(), 2);
    }

    problem.SetParameterBlockConstant(points.front().data());
    problem.SetParameterBlockConstant(points.back().data());

    for (int i = 1; i < N; ++i) {
        // Add traversability cost
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<TraversabilityCost, 1, 2>(
                new TraversabilityCost(traversability_image, traversability_weight)
            ),
            nullptr,
            points[i].data()
        );

        if (i > 0 && i < N - 1) {
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<SmoothnessCost, 2, 2, 2, 2>(
                    new SmoothnessCost(smooth_weight)
                ),
                nullptr,
                points[i-1].data(),  // Previous point
                points[i].data(),     // Current point
                points[i+1].data()    // Next point
            );
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 250;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    for (int i = 0; i < N; ++i) {
        optimized_path(i, 0) = static_cast<float>(points[i][0]);
        optimized_path(i, 1) = static_cast<float>(points[i][1]);
    }

    return optimized_path;
}