#include "image_space_path_planning/path_projector.hpp"

Eigen::MatrixXf PathProjector::projectTo2D(
    const Eigen::MatrixXf& path_3d,            // Nx3 matrix
    const cv::Mat& camera_matrix,              // 3x3
    const int image_width,                     // default value
    const int image_height,                    // default value
    const cv::Mat& dist_coeffs     			   // Optional distortion coefficients
) {
    CV_Assert(path_3d.cols() == 3);

    // Convert Eigen points to std::vector<cv::Point3f>
    std::vector<cv::Point3f> valid_points;

    for (int i = 0; i < path_3d.rows(); ++i) {
        float z = path_3d(i, 2);
        if (z > 0.0f) {
            valid_points.emplace_back(path_3d(i, 0), path_3d(i, 1), z);
        }
    }

    if (valid_points.empty()) {
        return Eigen::MatrixXf(0, 2); // No valid points
    }

    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

    std::vector<cv::Point2f> image_points;
    cv::projectPoints(valid_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);

    std::vector<cv::Point2f> filtered_points;
    for (const auto& pt : image_points) 
    {
        if (pt.x >= 0 && pt.x < image_width && pt.y >= 0 && pt.y < image_height) 
            filtered_points.push_back(pt);
    }

    // Fill the Eigen matrix
    Eigen::MatrixXf path_2d(filtered_points.size(), 2);
    for (size_t i = 0; i < filtered_points.size(); ++i) {
        path_2d(i, 0) = filtered_points[i].x;
        path_2d(i, 1) = filtered_points[i].y;
    }

    return path_2d;
}

Eigen::MatrixXf PathProjector::projectTo3D(
    const Eigen::MatrixXf& path_2d,              // Nx2 pixel coordinates
    const cv::Mat& depth_map,                    // CV_32FC1 depth image
    const cv::Mat& camera_matrix,                // 3x3 intrinsic matrix
    const cv::Mat& dist_coeffs			         // Optional distortion coefficients (ignored)
) {
    CV_Assert(path_2d.cols() == 2);
    CV_Assert(camera_matrix.rows == 3 && camera_matrix.cols == 3);
    CV_Assert(depth_map.type() == CV_32FC1);
    CV_Assert(!depth_map.empty());

    if (path_2d.rows() == 0) {
        return Eigen::MatrixXf(0, 3);
    }

    int N = path_2d.rows();
    Eigen::MatrixXf path_3d(N, 3);

    // Get camera intrinsics
    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);

    for (int i = 0; i < N; ++i) {
        float u = path_2d(i, 0);
        float v = path_2d(i, 1);

        if (u < 0 || v < 0 || u >= depth_map.cols || v >= depth_map.rows) {
            path_3d.row(i) = Eigen::RowVector3f::Constant(NAN);
            continue;
        }

        float z = depth_map.at<float>(static_cast<int>(v), static_cast<int>(u));
        if (z <= 0.0f) {
            path_3d.row(i) = Eigen::RowVector3f::Constant(NAN);
            continue;
        }

        // Normalize pixel coordinates
        double x_norm = (u - cx) / fx;
        double y_norm = (v - cy) / fy;

        double x = x_norm * z;
        double y = y_norm * z;

        path_3d(i, 0) = x;
        path_3d(i, 1) = y;
        path_3d(i, 2) = z;

        // std::cout << "Projected 3D point: " << path_3d.row(i) << std::endl;
    }

    return path_3d;
}

cv::Mat PathProjector::drawProjectedPath(
    const Eigen::MatrixXf& path_2d,
    const cv::Mat& base_image,
    const cv::Scalar& color,
    const int radius,
    const int thickness,
    const std::string& window_name
) {

    // Convert grayscale to color if needed
    cv::Mat image_color;
    if (base_image.channels() == 1) {
        cv::cvtColor(base_image, image_color, cv::COLOR_GRAY2BGR);
    } else {
        image_color = base_image.clone();
    }

    // Draw circles
    for (int i = 0; i < path_2d.rows(); ++i) {
        int x = static_cast<int>(path_2d(i, 0));
        int y = static_cast<int>(path_2d(i, 1));

        if (x >= 0 && x < image_color.cols && y >= 0 && y < image_color.rows) {
            cv::circle(image_color, cv::Point(x, y), radius, color, thickness);
        }
    }

    return image_color; // Return image with path drawn
}

void PathProjector::visualizeProjections(
    const cv::Mat& image,
    const int delay,
    const std::string& window_name
)
{
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    cv::imshow(window_name, image);
    cv::waitKey(delay);
}

Eigen::MatrixXf PathProjector::cropPathToLocalHorizon(
    const Eigen::MatrixXf& path_3d, // Complete path projected to 3D
    float local_horizon_distance,    // default value
    Eigen::MatrixXf* path_3d_ignore  // Output parameter for far path (to be ignored in optimization)
) {
    CV_Assert(path_3d.cols() == 3);

    Eigen::MatrixXf cropped_path;

    int split_idx = path_3d.rows(); // Default: keep all
    for (int i = 0; i < path_3d.rows(); ++i) {
        float dist = path_3d.row(i).norm() - path_3d.row(0).head<3>().norm(); // Distance from the first point
        if (dist > local_horizon_distance) {
            split_idx = i;
            break;
        }
    }

    // Points to keep (within horizon)
    if (split_idx > 0) {
        cropped_path = path_3d.topRows(split_idx);
    } else {
        cropped_path = Eigen::MatrixXf(0, 3);
    }

    // Points to ignore (beyond horizon)
    if (path_3d_ignore) {
        if (split_idx < path_3d.rows()) {
            *path_3d_ignore = path_3d.bottomRows(path_3d.rows() - split_idx);
        } else {
            *path_3d_ignore = Eigen::MatrixXf(0, 3);
        }
    }

    return cropped_path;
}

Eigen::MatrixXf PathProjector::removeInvalidPoints(const Eigen::MatrixXf& path) 
{
    std::vector<int> valid_indices;
    for (int i = 0; i < path.rows(); ++i) {
        if ((path.row(i).array() == path.row(i).array()).all() and std::isfinite(path.row(i).sum())) { // check for NaN and infs
            valid_indices.push_back(i);
        }
    }
    Eigen::MatrixXf cleaned_path(valid_indices.size(), path.cols());
    for (size_t i = 0; i < valid_indices.size(); ++i) {
        cleaned_path.row(i) = path.row(valid_indices[i]);
    }

    return cleaned_path;
}

std::pair<float, float> ransacLinearFit(
    const std::vector<float>& x_vals,
    const std::vector<float>& y_vals,
    int max_iterations = 100,
    float inlier_threshold = 0.1f,  // meters
    int min_inliers_required = 10)
{
    CV_Assert(x_vals.size() == y_vals.size());
    int N = x_vals.size();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, N - 1);

    float best_scale = 1.0f;
    float best_bias = 0.0f;
    int best_inlier_count = 0;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Randomly sample 2 points (minimum for a line)
        int i1 = dis(gen);
        int i2 = dis(gen);
        if (i1 == i2) continue;

        float x1 = x_vals[i1], y1 = y_vals[i1];
        float x2 = x_vals[i2], y2 = y_vals[i2];
        if (x2 == x1) continue;

        float scale = (y2 - y1) / (x2 - x1);
        float bias = y1 - scale * x1;

        // Count inliers
        int inlier_count = 0;
        for (int i = 0; i < N; ++i) {
            float y_pred = scale * x_vals[i] + bias;
            if (std::fabs(y_vals[i] - y_pred) < inlier_threshold) {
                inlier_count++;
            }
        }

        if (inlier_count > best_inlier_count) {
            best_inlier_count = inlier_count;
            best_scale = scale;
            best_bias = bias;
        }
    }

    // std::cout << "[RANSAC] Best inlier count: " << best_inlier_count << "/" << N << std::endl;

    if (best_inlier_count < min_inliers_required) {
        // std::cerr << "[RANSAC] Not enough inliers. Using default scale and bias.\n";
        return {1.0f, 0.0f};
    }

    return {best_scale, best_bias};
}

Eigen::Vector3f polyfitRANSAC(
    const std::vector<float>& x_vals,
    const std::vector<float>& y_vals,
    int max_iterations = 200,
    float inlier_threshold = 0.1f,
    int min_inliers_required = 10)
{
    CV_Assert(x_vals.size() == y_vals.size());
    int N = x_vals.size();
    if (N < 3) return Eigen::Vector3f(0, 1, 0);  // fallback to linear

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, N - 1);

    Eigen::Vector3f best_coeffs(0, 1, 0);
    int best_inlier_count = 0;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Randomly pick 3 points
        int i1 = dis(gen), i2 = dis(gen), i3 = dis(gen);
        if (i1 == i2 || i2 == i3 || i1 == i3) continue;

        Eigen::Matrix3f A;
        Eigen::Vector3f y;
        A << x_vals[i1] * x_vals[i1], x_vals[i1], 1.0f,
             x_vals[i2] * x_vals[i2], x_vals[i2], 1.0f,
             x_vals[i3] * x_vals[i3], x_vals[i3], 1.0f;
        y << y_vals[i1], y_vals[i2], y_vals[i3];

        Eigen::Vector3f coeffs = A.colPivHouseholderQr().solve(y);

        // Count inliers
        int inliers = 0;
        for (int i = 0; i < N; ++i) {
            float x = x_vals[i];
            float y_pred = coeffs[0]*x*x + coeffs[1]*x + coeffs[2];
            if (std::fabs(y_vals[i] - y_pred) < inlier_threshold)
                ++inliers;
        }

        if (inliers > best_inlier_count) {
            best_inlier_count = inliers;
            best_coeffs = coeffs;
        }
    }

    // std::cout << "[RANSAC] Best inliers: " << best_inlier_count << "/" << N << std::endl;
    if (best_inlier_count < min_inliers_required) {
        // std::cerr << "[RANSAC] Not enough inliers for polynomial. Falling back to linear.\n";
        return Eigen::Vector3f(0, 1, 0); // y = x
    }

    return best_coeffs;
}

cv::Mat PathProjector::normalizeDepthMapWithRealData(
    const cv::Mat& relative_depth,
    const cv::Mat& real_depth,
    float max_depth_m,
    int grid_rows,
    int grid_cols
) {
    CV_Assert(relative_depth.size() == real_depth.size());
    CV_Assert(relative_depth.type() == CV_32FC1);
    CV_Assert(real_depth.type() == CV_32FC1);

    std::vector<float> rel_values;
    std::vector<float> real_values;

    int step_y = relative_depth.rows / grid_rows;
    int step_x = relative_depth.cols / grid_cols;

    for (int y = step_y / 2; y < relative_depth.rows; y += step_y) {
        for (int x = step_x / 2; x < relative_depth.cols; x += step_x) {
            float z_rel = relative_depth.at<float>(y, x);
            float z_real = real_depth.at<float>(y, x);

            if (std::isfinite(z_rel) && std::isfinite(z_real) &&
                z_rel > 0.0f && z_real > 0.0f && z_real < max_depth_m)
            {
                rel_values.push_back(z_rel);
                real_values.push_back(z_real);
            }
        }
    }

    // std::cout << "[INFO] Matched samples: " << rel_values.size() << std::endl;

    if (rel_values.size() < 10) {
        // std::cerr << "[WARN] Not enough valid points for depth normalization. Skipping scaling.\n";
        return relative_depth.clone();
    }

    // auto [scale, bias] = ransacLinearFit(rel_values, real_values);
    // std::cout << "[INFO] RANSAC scale: " << scale << ", bias: " << bias << std::endl;

    // cv::Mat scaled_depth;
    // scaled_depth = relative_depth * scale + bias;

    Eigen::Vector3f coeffs = polyfitRANSAC(rel_values, real_values);
    // std::cout << "[INFO] Polyfit coeffs: a=" << coeffs[0]
    //           << ", b=" << coeffs[1] << ", c=" << coeffs[2] << std::endl;

    // Apply the polynomial scaling
    cv::Mat scaled_depth = cv::Mat::zeros(relative_depth.size(), CV_32FC1);
    for (int y = 0; y < relative_depth.rows; ++y) {
        for (int x = 0; x < relative_depth.cols; ++x) {
            float z = relative_depth.at<float>(y, x);
            if (std::isfinite(z) && z > 0.0f) {
                scaled_depth.at<float>(y, x) = coeffs[0]*z*z + coeffs[1]*z + coeffs[2];
            } else {
                scaled_depth.at<float>(y, x) = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }

    return scaled_depth;
}

Eigen::MatrixXf PathProjector::removeClosePoints(
    const Eigen::MatrixXf& path,
    float min_distance
) {
    if (path.rows() < 2) return path; // No points or only one point

    Eigen::MatrixXf filtered_path;

    std::vector<int> keep_indices;
    for (int i = 0; i < path.rows(); ++i) {
        float dist = path.row(i).norm();
        if (dist >= min_distance) {
            keep_indices.push_back(i);
        }
    }
    filtered_path.resize(keep_indices.size(), 3);
    for (size_t i = 0; i < keep_indices.size(); ++i) {
        filtered_path.row(i) = path.row(keep_indices[i]);
    }

    return filtered_path;
}