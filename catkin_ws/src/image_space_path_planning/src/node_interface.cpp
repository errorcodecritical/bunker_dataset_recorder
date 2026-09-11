#include "image_space_path_planning/node_interface.hpp"
#include <chrono>

NodeInterface::NodeInterface(ros::NodeHandle& nh) : tf_listener_(tf_buffer_) {
    loadParams(nh);

    path_sub_ = nh.subscribe(path_topic_, 1, &NodeInterface::pathCallback, this);
    image_sub_ = nh.subscribe(image_topic_, 1, &NodeInterface::imageCallback, this);
    depth_sub_ = nh.subscribe(depth_topic_, 1, &NodeInterface::depthCallback, this);
    rel_depth_sub_ = nh.subscribe(relative_depth_topic_, 1, &NodeInterface::relativeDepthCallback, this);

    path_pub_ = nh.advertise<nav_msgs::Path>(path_publisher_topic_, 1);


    if (!this->robot_T_camera_.isApprox(Eigen::Matrix4f::Identity())) {
        try {
            geometry_msgs::TransformStamped transform_stamped =
                tf_buffer_.lookupTransform(robot_frame_, camera_optical_frame_, ros::Time(0), ros::Duration(100.0));

            const auto& t = transform_stamped.transform.translation;
            const auto& q = transform_stamped.transform.rotation;

            Eigen::Quaternionf q_eigen(q.w, q.x, q.y, q.z);
            Eigen::Matrix3f rotation = q_eigen.toRotationMatrix();

            this->robot_T_camera_.block<3,3>(0,0) = rotation;
            this->robot_T_camera_(0,3) = t.x;
            this->robot_T_camera_(1,3) = t.y;
            this->robot_T_camera_(2,3) = t.z;

            ROS_INFO_STREAM("Transform from 'camera_color_optical_frame' to 'base':\n" << this->robot_T_camera_);

        } catch (tf2::TransformException &ex) {
            ROS_ERROR_STREAM("Failed to get transform: " << ex.what() << "Using default matrix.");
            this->robot_T_camera_ << 0.0000000,  0.0000000, 1.0000000, 0.16 ,
                                    -1.0000000,  0.0000000, 0.0000000, 0.015,
                                    0.0000000, -1.0000000, 0.0000000, 0.06 ,
                                    0, 		   0, 		0, 		   1;
        }
    }
}

void NodeInterface::run() {
    ROS_INFO_ONCE("Started NodeInterface");

    //-----------------------------------------------------new-------------------------------------
    // Grab the latest image + depth (these are the only things we truly need for the 2D->3D test)
    cv::Mat image = getImageFromDeque();
    cv::Mat depth = getDepthFromDeque();

    // ===================== DEBUG: 2D curve -> 3D projection only =====================
    // Turn this on by setting: debug_project_only:=true (recommended param in loadParams)
    if (debug_project_only_) {
        int width  = depth.cols;
        int height = depth.rows;

        // 1) Generate a fixed 2D curve in pixel space (u,v)
        int num_points = 120;
        Eigen::MatrixXf curve_2d(num_points, 2);

        float v_start = height - 5.0f;     // near bottom of the image
        float v_end   = height * 0.65f;    // towards the middle

        for (int i = 0; i < num_points; ++i) {
            float t = float(i) / float(num_points - 1);
            float v = v_start + t * (v_end - v_start);

            // Gentle horizontal oscillation around the image center
            float u = (width * 0.5f) + 15.0f * std::sin(v * 0.03f);

            curve_2d.row(i) << u, v;
        }

        // 2) Project pixels -> 3D using depth (output is in CAMERA frame)
        Eigen::MatrixXf curve_3d = this->path_projector_.projectTo3D(
            curve_2d, depth, camera_matrix_, dist_coeffs_
        );

        // Remove invalid points (NaNs etc.)
        curve_3d = this->path_projector_.removeInvalidPoints(curve_3d);

        // 3) Publish in CAMERA frame (no TF required for this test)
        // In RViz set Fixed Frame = camera_color_optical_frame
        publishOptimizedPath(curve_3d, camera_optical_frame_, path_pub_);

        // 4) Optional: draw the 2D curve on the image so you can confirm it’s constant
        cv::Mat canvas = image.clone();
        canvas = this->path_projector_.drawProjectedPath(curve_2d, canvas, cv::Scalar(0, 255, 0), 4);
        this->path_projector_.visualizeProjections(canvas, 1);

        return; // IMPORTANT: skip the normal pipeline below
    }
    // ================================================================================

    // ------------------------ NORMAL PIPELINE (unchanged) ------------------------
    
    Eigen::Matrix4f map_T_robot = Eigen::Matrix4f::Identity();
    if (this->robot_pose_okay)
    {
        map_T_robot = this->getRobotPose();
    }

    Eigen::Matrix4f robot_T_camera = this->robot_T_camera_;

    // cv::Mat relative_depth = getRelativeDepthFromDeque();

    Eigen::MatrixXf path_3d = getPathFromDeque();
    //

    // === TEMPORARY TESTING CODE ===

    // int width = depth.cols;
    // int height = depth.rows;

    // int num_points = 100;
    // Eigen::MatrixXf temp_path_2d(num_points, 2);
    // float step = static_cast<float>(height/2) / num_points;

    // for (int i = 0; i < num_points; ++i) {
    //     float v = height - i * step;  // goes from bottom (height) to middle (height/2)
    //     float u = (width / 2.0f) + 30.0f * std::sin(v * 0.05f);
    //     temp_path_2d.row(i) << u, v;
    // }

    // Eigen::MatrixXf path_3d = this->path_projector_.projectTo3D(temp_path_2d, depth, camera_matrix_, dist_coeffs_);

    // ==============================

    Eigen::MatrixXf path_3d_local, path_3d_far;
    path_3d_local = this->path_projector_.cropPathToLocalHorizon(path_3d, this->local_horizon_distance_, &path_3d_far);

    Eigen::MatrixXf path_3d_local_camera = transformPointsToCameraFrame(path_3d_local, map_T_robot, robot_T_camera);

    path_3d_local_camera = this->path_projector_.removeClosePoints(path_3d_local_camera, 0.2f);

    Eigen::MatrixXf path_2d = this->path_projector_.projectTo2D(path_3d_local_camera, this->camera_matrix_, this->image_width_, this->image_height_, this->dist_coeffs_);

    // visualize the projected points
    cv::Mat canvas = image.clone();
    // Visualize the original path (green)
    canvas = this->path_projector_.drawProjectedPath(path_2d, canvas, cv::Scalar(0, 255, 0), 5);

    // deform the path
    // Eigen::MatrixXf deformed_path = this->path_deformer_.deformPath(pixels, image);
    Eigen::MatrixXf deformed_path_strip_search = this->path_deformer_.deformPathStripSearch(path_2d, image, 30, 50);

    // Visualize the deformed path (red) on top of the same canvas
    canvas = this->path_projector_.drawProjectedPath(deformed_path_strip_search, canvas, cv::Scalar(0, 0, 255), 5);

    // Run the path optimizer
    Eigen::MatrixXf optimized_path = this->path_optimizer_.smoothPathOutliers(deformed_path_strip_search);

    // Eigen::MatrixXf optimized_path = this->path_optimizer_.optimizePathWithCeres(path_2d, image);

    // Visualize the optimized path (blue) on top of the same canvas
    canvas = this->path_projector_.drawProjectedPath(optimized_path, canvas, cv::Scalar(255, 0, 0), 8, 1);

    // relative_depth = this->path_projector_.normalizeDepthMapWithRealData(relative_depth, depth);

    // re-project the optimized path to 3D
    Eigen::MatrixXf reprojected_path = this->path_projector_.projectTo3D(optimized_path, depth, camera_matrix_, dist_coeffs_);

    // Remove NaNs from the reprojected path
    reprojected_path = this->path_projector_.removeInvalidPoints(reprojected_path);

    // Transform the reprojected path back to the map frame
    Eigen::MatrixXf optimized_path_map = transformPointsToMapFrame(reprojected_path, map_T_robot, robot_T_camera);

    if (optimized_path_map.rows() > 0) {
        // Get robot position in map frame
        Eigen::Vector3f robot_position = map_T_robot.block<3,1>(0,3);
        // Create a new matrix with robot position as the first row
        Eigen::MatrixXf path_with_robot(optimized_path_map.rows() + 1, 3);
        path_with_robot.row(0) = robot_position.transpose();
        path_with_robot.bottomRows(optimized_path_map.rows()) = optimized_path_map;
        optimized_path_map = path_with_robot;
    }

    if (optimized_path_map.rows() == 0) {
        ROS_WARN("Reprojected path is empty. Publishing the original global path.");
    }

    if (path_3d_far.rows() > 0) {
        Eigen::MatrixXf temp(optimized_path_map.rows() + path_3d_far.rows(), optimized_path_map.cols());
        temp << optimized_path_map, path_3d_far;
        optimized_path_map = temp;
    }

    // Publish the optimized path in the map frame
    publishOptimizedPath(optimized_path_map, world_frame_, path_pub_);

    this->path_projector_.visualizeProjections(canvas, 1);
}

void NodeInterface::loadParams(ros::NodeHandle& nh) {
    nh.param<std::string>("path_topic", path_topic_, "input_path");
    nh.param<std::string>("image_topic", image_topic_, "input_image");
    nh.param<std::string>("depth_topic", depth_topic_, "input_depth");
    nh.param<std::string>("relative_depth_topic", relative_depth_topic_, "input_relative_depth");
    nh.param<std::string>("path_publisher_topic", path_publisher_topic_, "optimized_path");

    nh.param<std::string>("robot_frame", robot_frame_, "base");
    nh.param<std::string>("world_frame", world_frame_, "map");
    nh.param<std::string>("camera_optical_frame", camera_optical_frame_, "camera_color_optical_frame");

    nh.param<int>("image_width", image_width_, 640);
    nh.param<int>("image_height", image_height_, 480);

    nh.param<float>("local_horizon_distance", local_horizon_distance_, 10.0f);

    nh.param<bool>("convert_depth_from_mm_to_m", convert_depth_from_mm_to_m_, false);


    nh.param<bool>("debug_project_only", debug_project_only_, false);

    std::vector<double> camera_matrix_vec;
    if (nh.getParam("camera_matrix/data", camera_matrix_vec) && camera_matrix_vec.size() == 9) {
        camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_vec.data()).clone();
    } else {
        ROS_WARN("Failed to load camera_matrix from parameter server. Using default values.");
        camera_matrix_ = (cv::Mat_<double>(3, 3) << 500.0, 0.0, this->image_width_ / 2.0,
                                             0.0, 500.0, this->image_height_ / 2.0,
                                             0.0, 0.0, 1.0);
    }

    std::vector<double> dist_coeffs_vec;
    if (nh.getParam("distortion_coefficients/data", dist_coeffs_vec) && dist_coeffs_vec.size() == 5) {
        dist_coeffs_ = cv::Mat(1, 5, CV_64F, dist_coeffs_vec.data()).clone();
    } else {
        ROS_WARN("Failed to load dist_coeffs from parameter server. Using default values.");
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F); // Assuming no distortion
    }

    // Load robot_T_camera from parameter server if available
    std::vector<double> robot_T_camera_vec;
    if (nh.getParam("robot_T_camera/data", robot_T_camera_vec) && robot_T_camera_vec.size() == 16) {
        Eigen::Matrix4f robot_T_camera_mat;
        for (int i = 0; i < 16; ++i) {
            robot_T_camera_mat(i / 4, i % 4) = static_cast<float>(robot_T_camera_vec[i]);
        }
        this->robot_T_camera_ = robot_T_camera_mat;
        ROS_INFO_STREAM("Loaded robot_T_camera from parameter server:\n" << this->robot_T_camera_);
    } else {
        this->robot_T_camera_ = Eigen::Matrix4f::Identity();
        ROS_WARN("Failed to load robot_T_camera from parameter server. Using identity matrix.");
    }
}

void NodeInterface::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    ROS_INFO_ONCE("Started receiving paths.");

    // Check if path is empty
    if (msg->poses.empty()) {
        ROS_WARN("Received empty path");
        return;
    }

    // Convert path to Eigen matrix (each row is a 3D point: x, y, z)
    Eigen::MatrixXf path(msg->poses.size(), 3);
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        const geometry_msgs::Point& p = msg->poses[i].pose.position;
        path(i, 0) = p.x;
        path(i, 1) = p.y;
        path(i, 2) = p.z;
    }

    // Add the eigen matrix to the path deque
    {
        std::lock_guard<std::mutex> lock(path_queue_mutex_);
        path_queue_.push_back(path);

        // Limit queue size to prevent memory issues (as we will be using a rolling window approach)
        const size_t MAX_QUEUE_SIZE = 30;
        if (path_queue_.size() > MAX_QUEUE_SIZE) {
            path_queue_.pop_front();
        }
    }
}

void NodeInterface::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO_ONCE("Started receiving images.");

    try {
        // Convert the ROS image to OpenCV format using the original encoding
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        if (msg->encoding == "32FC1" || msg->encoding == "32FC3" || msg->encoding == "32FC4") {
            cv::Mat float_image = cv_ptr->image;
            double minVal, maxVal;
            cv::minMaxLoc(float_image, &minVal, &maxVal);
            // Avoid division by zero
            if (maxVal - minVal < 1e-6) maxVal = minVal + 1.0;
            cv::Mat normalized;
            float_image.convertTo(normalized, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
            cv_ptr = boost::make_shared<cv_bridge::CvImage>(cv_ptr->header, "mono8", normalized);
        }
        // Always resize to 640x480 for consistency
        if (cv_ptr->image.cols != 640 || cv_ptr->image.rows != 480) {
            cv::Mat resized;
            cv::resize(cv_ptr->image, resized, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
            cv_ptr = boost::make_shared<cv_bridge::CvImage>(cv_ptr->header, cv_ptr->encoding, resized);
        }
        cv::Mat image = cv_ptr->image.clone();  // Clone to decouple from ROS message lifetime

        // Thread-safe insertion into image queue
        {
            std::lock_guard<std::mutex> lock(image_queue_mutex_);
            image_queue_.push_back(image);

            // Limit queue size to prevent memory issues (as we will be using a rolling window approach)
            const size_t MAX_QUEUE_SIZE = 30;
            if (image_queue_.size() > MAX_QUEUE_SIZE) {
                image_queue_.pop_front();
            }
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_STREAM(__FILE__ << " -> cv_bridge exception: " << e.what());
    }
}

void NodeInterface::depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO_ONCE("Started receiving depth images.");

    try {
        // Convert the ROS depth image to OpenCV format (assuming 32FC1 encoding)
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "32FC1");
        cv::Mat depth_image = cv_ptr->image.clone();  // Clone to decouple from ROS message lifetime

        if (this->convert_depth_from_mm_to_m_)
            depth_image.convertTo(depth_image, CV_32F, 1.0 / 1000.0); // convert mm to meters

        // Thread-safe insertion into depth queue
        {
            std::lock_guard<std::mutex> lock(depth_queue_mutex_);
            depth_queue_.push_back(depth_image);

            // Limit queue size to prevent memory issues (as we will be using a rolling window approach)
            const size_t MAX_QUEUE_SIZE = 30;
            if (depth_queue_.size() > MAX_QUEUE_SIZE) {
                depth_queue_.pop_front();
            }
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_STREAM(__FILE__ << " -> cv_bridge exception: " << e.what());
    }
}

Eigen::MatrixXf NodeInterface::transformPointsToCameraFrame(
    const Eigen::MatrixXf &points,                 // Nx3
    const Eigen::Matrix4f &map_T_robot,            // 4x4
    const Eigen::Matrix4f &robot_T_camera          // 4x4
) {
    // Compose full map-to-camera transform
    Eigen::Matrix4f map_T_camera = map_T_robot * robot_T_camera;
    Eigen::Matrix4f camera_T_map = map_T_camera.inverse();

    // Convert Nx3 to Nx4 homogeneous points
    Eigen::MatrixXf points_hom(points.rows(), 4);
    points_hom.leftCols(3) = points;
    points_hom.col(3).setOnes();

    // Apply transform
    Eigen::MatrixXf transformed_points_hom = (camera_T_map * points_hom.transpose()).transpose();

    // Return just the 3D part (drop homogeneous coordinate)
    return transformed_points_hom.leftCols(3);
}

Eigen::MatrixXf NodeInterface::transformPointsToMapFrame(
    const Eigen::MatrixXf &points,                 // Nx3
    const Eigen::Matrix4f &map_T_robot,            // 4x4
    const Eigen::Matrix4f &robot_T_camera          // 4x4
) {
    // Compose full map-to-camera transform
    Eigen::Matrix4f map_T_camera = map_T_robot * robot_T_camera;

    // Convert Nx3 to Nx4 homogeneous points
    Eigen::MatrixXf points_hom(points.rows(), 4);
    points_hom.leftCols(3) = points;
    points_hom.col(3).setOnes();

    // Apply transform
    Eigen::MatrixXf transformed_points_hom = (map_T_camera * points_hom.transpose()).transpose();

    // Return just the 3D part (drop homogeneous coordinate)
    return transformed_points_hom.leftCols(3);
}


cv::Mat NodeInterface::getImageFromDeque() {

    cv::Mat image;
    while (ros::ok()) {
        {
            std::lock_guard<std::mutex> lock(this->image_queue_mutex_);
            if (!this->image_queue_.empty()) {
                image = this->image_queue_.back();
                break; // Exit the loop once an image is retrieved
            }
        }
        ROS_WARN_THROTTLE(1.0, "Waiting for an image in the queue...");
    }

    if (image.channels() == 3) {
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    }
    if (image.type() != CV_8UC1) {
        image.convertTo(image, CV_8UC1);
    }

    return image;
}

cv::Mat NodeInterface::getDepthFromDeque() {

    cv::Mat depth;
    while (ros::ok()) {
        {
            std::lock_guard<std::mutex> lock(this->depth_queue_mutex_);
            if (!this->depth_queue_.empty()) {
                depth = this->depth_queue_.back();
                break; // Exit the loop once a depth image is retrieved
            }
        }
        ROS_WARN_THROTTLE(1.0, "Waiting for a depth image in the queue...");
    }

    return depth;
}

Eigen::MatrixXf NodeInterface::getPathFromDeque()
{
    Eigen::MatrixXf path;
    while (ros::ok()) {
        {
            std::lock_guard<std::mutex> lock(this->path_queue_mutex_);
            if (!this->path_queue_.empty()) {
                path = this->path_queue_.back();
                break; // Exit the loop once a path is retrieved
            }
        }
        ROS_WARN_THROTTLE(1.0, "Waiting for a path in the queue...");
    }

    return path;
}

Eigen::Matrix4f NodeInterface::getRobotPose() 
{
    Eigen::Matrix4f robot_pose = Eigen::Matrix4f::Identity();

    try {
        // Try to get the transform from 'map' to 'base'
        geometry_msgs::TransformStamped transform_stamped =
            tf_buffer_.lookupTransform(world_frame_, robot_frame_, ros::Time(0), ros::Duration(1.0));


        // Extract translation and rotation
        const auto& t = transform_stamped.transform.translation;
        const auto& q = transform_stamped.transform.rotation;

        // Convert quaternion to Eigen rotation matrix
        Eigen::Quaternionf q_eigen(q.w, q.x, q.y, q.z);
        Eigen::Matrix3f rotation = q_eigen.toRotationMatrix();

        // Fill in the pose matrix
        robot_pose.block<3,3>(0,0) = rotation;
        robot_pose(0,3) = t.x;
        robot_pose(1,3) = t.y;
        robot_pose(2,3) = t.z;

    } catch (tf2::TransformException& ex) {
        this->robot_pose_okay = false;
        ROS_ERROR_STREAM("Failed to get transform: " << ex.what() << ". Using identity.");
    }
    return robot_pose;
}

void NodeInterface::publishOptimizedPath(const Eigen::MatrixXf& path, const std::string& frame_id, ros::Publisher& pub) {
    nav_msgs::Path ros_path;
    ros_path.header.stamp = ros::Time::now();
    ros_path.header.frame_id = frame_id;

    for (int i = 0; i < path.rows(); ++i) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = ros_path.header;
        pose_stamped.pose.position.x = path(i, 0);
        pose_stamped.pose.position.y = path(i, 1);
        pose_stamped.pose.position.z = path(i, 2);
        pose_stamped.pose.orientation.w = 1.0; // No orientation
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        ros_path.poses.push_back(pose_stamped);
    }

    pub.publish(ros_path);
}

void NodeInterface::relativeDepthCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO_ONCE("Started receiving relative depth images.");

    try {
        // Convert the ROS depth image to OpenCV format (assuming 32FC1 encoding)
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "32FC1");
        cv::Mat depth_image = cv_ptr->image.clone();  // Clone to decouple from ROS message lifetime

        // Thread-safe insertion into relative depth queue
        {
            std::lock_guard<std::mutex> lock(relative_depth_queue_mutex_);
            relative_depth_queue_.push_back(depth_image);

            // Limit queue size to prevent memory issues (as we will be using a rolling window approach)
            const size_t MAX_QUEUE_SIZE = 30;
            if (relative_depth_queue_.size() > MAX_QUEUE_SIZE) {
                relative_depth_queue_.pop_front();
            }
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR_STREAM(__FILE__ << " -> cv_bridge exception: " << e.what());
    }
}

cv::Mat NodeInterface::getRelativeDepthFromDeque() {

    cv::Mat relative_depth;
    while (ros::ok()) {
        {
            std::lock_guard<std::mutex> lock(this->relative_depth_queue_mutex_);
            if (!this->relative_depth_queue_.empty()) {
                relative_depth = this->relative_depth_queue_.back();
                break; // Exit the loop once a relative depth image is retrieved
            }
        }
        ROS_WARN_THROTTLE(1.0, "Waiting for a relative depth image in the queue...");
    }

    return relative_depth;
}