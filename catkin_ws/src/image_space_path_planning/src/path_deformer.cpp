#include "image_space_path_planning/path_deformer.hpp"

Eigen::MatrixXf PathDeformer::deformPath(
    const Eigen::MatrixXf& input_path,
    const cv::Mat& image)
{
    // 1. Receive a 2D path in the image space (pixels) and an image with the mean info of the previous N images (for temporal stability) -- done

    // 2. Sample a NxN patch around the point and compute the average or minimum traversability value
    // Compute integral image (CV_32F ensures precision)
    cv::Mat integral_img;
    cv::integral(image, integral_img, CV_32F);  // includes 1 extra row & col!
    int patch_size = 5;
    
    Eigen::MatrixXf traversability_values(input_path.rows(), 1);
    traversability_values = computeMeanPatchTraversability(integral_img, input_path, patch_size);

    // std::cout << "Traversability values: " << traversability_values.transpose() << std::endl;

    // 3. Mark each point location (NxN patch) as acceptable or not (lower or higher than a threshold) -- done
    float threshold = 200.0; // Example threshold
    Eigen::MatrixXf acceptable_points(input_path.rows(), 1);
    
    acceptable_points = computePointAcceptance(traversability_values, threshold);

    // std::cout << "Acceptable points: " << acceptable_points.transpose() << std::endl;

    // 4. For all unacceptable points: -- done
    //    a. Look for higher-traversability neighboring patches within a defined radius (e.g., 100 px)
    //    b. Use a cost function: low traversability + low deviation from original point
    //    c. Choose best candidate
    // Parameters for tuning
    int search_radius = 30; // in pixels
    float alpha = 1.0f;     // weight for traversability
    float beta = 0.05f;     // weight for distance (in pixels)

    Eigen::MatrixXf deformed_path = input_path; // copy to modify

    deformed_path = findBetterPatch(input_path, integral_img, acceptable_points, search_radius, patch_size, alpha, beta);

    // 6. Return the deformed 2D path -- done

    return deformed_path;
}

Eigen::MatrixXf PathDeformer::deformPathStripSearch(
    const Eigen::MatrixXf &input_path,
    const cv::Mat &image,
    int strip_height,
    int x_search_radius)
{
    CV_Assert(image.type() == CV_8UC1 || image.type() == CV_32FC1);

    cv::Mat traversability;
    if (image.type() == CV_8UC1)
    {
        image.convertTo(traversability, CV_32F);
    }
    else
    {
        traversability = image;
    }

    Eigen::MatrixXf deformed_path = input_path;

    for (int i = 0; i < input_path.rows(); ++i)
    {
        int orig_x = static_cast<int>(input_path(i, 0));
        int orig_y = static_cast<int>(input_path(i, 1));

        int strip_y_start = std::max(0, orig_y - strip_height / 2);
        int strip_y_end = std::min(image.rows - 1, orig_y + strip_height / 2);
        int x_start = std::max(0, orig_x - x_search_radius);
        int x_end = std::min(image.cols - 1, orig_x + x_search_radius);

        float best_score = std::numeric_limits<float>::max();
        cv::Point best_point(orig_x, orig_y); // fallback: keep original

        for (int y = strip_y_start; y <= strip_y_end; ++y)
        {
            for (int x = x_start; x <= x_end; ++x)
            {
                float score = traversability.at<float>(y, x);
                if (score < best_score)
                {
                    best_score = score;
                    best_point = cv::Point(x, y);
                }
            }
        }

        deformed_path(i, 0) = best_point.x;
        deformed_path(i, 1) = best_point.y;
    }

    return deformed_path;
}

Eigen::MatrixXf PathDeformer::computeMeanPatchTraversability(const cv::Mat& integral_img, 
                                               const Eigen::MatrixXf& input_path, 
                                               int patch_size) 
{
    Eigen::MatrixXf traversability_values(input_path.rows(), 1);

    for (int i = 0; i < input_path.rows(); ++i) {
        int x = static_cast<int>(input_path(i, 0));
        int y = static_cast<int>(input_path(i, 1));
        int half_patch = patch_size / 2;
    
        // Check patch boundaries
        if (x < half_patch || y < half_patch ||
            x + half_patch >= integral_img.cols - 1 || y + half_patch >= integral_img.rows - 1) {
            traversability_values(i, 0) = 255.0f;
            continue;
        }
    
        int x0 = x - half_patch;
        int y0 = y - half_patch;
        int x1 = x + half_patch + 1; // integral image has an extra row/col
        int y1 = y + half_patch + 1;
    
        // Access integral image to compute sum
        float A = integral_img.at<float>(y0, x0);
        float B = integral_img.at<float>(y0, x1);
        float C = integral_img.at<float>(y1, x0);
        float D = integral_img.at<float>(y1, x1);
    
        float sum = D - B - C + A;
        traversability_values(i, 0) = sum / (patch_size * patch_size);
    }

    return traversability_values;
}

Eigen::MatrixXf PathDeformer::computePointAcceptance(
    const Eigen::MatrixXf& traversability_values,
    float threshold)
{

    Eigen::MatrixXf acceptable_points(traversability_values.rows(), 1);

    for (int i = 0; i < traversability_values.rows(); ++i) {
        if (traversability_values(i, 0) > threshold) {
            acceptable_points(i, 0) = 0.0f; // unacceptable
        } else {
            acceptable_points(i, 0) = 1.0f; // acceptable
        }
    }

    return acceptable_points;
}

Eigen::MatrixXf PathDeformer::findBetterPatch(
    const Eigen::MatrixXf& input_path,
    const cv::Mat& integral_img,
    const Eigen::MatrixXf& acceptable_points,
    int search_radius,
    int patch_size,
    float alpha,
    float beta)
{
    // 4. For all unacceptable points: -- done
    //    a. Look for higher-traversability neighboring patches within a defined radius (e.g., 100 px)
    //    b. Use a cost function: low traversability + low deviation from original point
    //    c. Choose best candidate
    Eigen::MatrixXf deformed_path = input_path; // copy to modify

    for (int i = 0; i < input_path.rows(); ++i) {
        if (acceptable_points(i, 0) == 1.0f)
            continue;

        int x_orig = static_cast<int>(input_path(i, 0));
        int y_orig = static_cast<int>(input_path(i, 1));

        float min_cost = std::numeric_limits<float>::max();
        cv::Point best_candidate = cv::Point(x_orig, y_orig); // fallback

        for (int dx = -search_radius; dx <= search_radius; ++dx) {
            for (int dy = -search_radius; dy <= search_radius; ++dy) {
                int x_new = x_orig + dx;
                int y_new = y_orig + dy;

                // Patch boundaries
                int half_patch = patch_size / 2;
                int x1 = x_new - half_patch;
                int y1 = y_new - half_patch;
                int x2 = x_new + half_patch;
                int y2 = y_new + half_patch;

                // Bounds check (accounting for integral image's 1-pixel offset)
                if (x1 < 0 || y1 < 0 || x2 + 1 >= integral_img.cols || y2 + 1 >= integral_img.rows)
                    continue;

                // Use integral image to compute patch sum
                float patch_sum = integral_img.at<float>(y2 + 1, x2 + 1)
                    - integral_img.at<float>(y1, x2 + 1)
                    - integral_img.at<float>(y2 + 1, x1)
                    + integral_img.at<float>(y1, x1);

                float mean_traversability = patch_sum / (patch_size * patch_size);

                float distance = std::sqrt(dx * dx
                    + dy * dy);
                float cost = alpha * mean_traversability + beta * distance;
                if (cost < min_cost) {
                    min_cost = cost;
                    best_candidate = cv::Point(x_new, y_new);
                }
            }
        }
        // 5. Replace the original point with the best candidate -- done
        deformed_path(i, 0) = static_cast<float>(best_candidate.x);
        deformed_path(i, 1) = static_cast<float>(best_candidate.y);
    }
    // 6. Return the deformed 2D path -- done
    return deformed_path;
}