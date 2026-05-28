// Copyright (c) Sensrad 2025

#include "Ymir.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ymir::Ymir)

namespace ymir {

Ymir::Ymir(const rclcpp::NodeOptions &options)
    : rclcpp::Node("ymir", options), path_publisher_(nullptr),
      marker_publisher_(nullptr), marker_counter_(0), tf_buffer_{get_clock()},
      tf_listener_{tf_buffer_}, odom_initialized_(false),
      last_published_time_(0, 0, RCL_ROS_TIME) {

  // Declare ROS2 parameters
  odom_frame_id_ = declare_parameter("odom_frame_id", "odom");
  baselink_frame_id_ = declare_parameter("baselink_frame_id", "base_link");
  alpha_ = declare_parameter("alpha", 0.3); // Low-pass filter coefficient
  ego_motion_topic_pattern_ = declare_parameter("ego_motion_topic_pattern",
                                                ".*/radar_.*/oden/ego_motion");
  max_path_length_ = declare_parameter("max_path_length", 300);
  path_publish_rate_ = declare_parameter("path_publish_rate", 5.0);
  marker_distance_threshold_ =
      declare_parameter("marker_distance_threshold", 1.0);
  marker_arrow_width_ = declare_parameter("marker_arrow_width", 0.05);
  discovery_period_ = declare_parameter(
      "discovery_period", 5.0); // Topic discovery period in seconds

  RCLCPP_INFO(get_logger(), "Ymir node starting");
  RCLCPP_INFO(get_logger(), "Low-pass filter alpha: %.3f", alpha_);
  RCLCPP_INFO(get_logger(), "Topic pattern for ego_motion: %s",
              ego_motion_topic_pattern_.c_str());
  RCLCPP_INFO(get_logger(), "Max path length: %zu poses", max_path_length_);
  RCLCPP_INFO(get_logger(), "Path publish rate: %.1f Hz", path_publish_rate_);
  RCLCPP_INFO(get_logger(), "Marker distance threshold: %.2f m",
              marker_distance_threshold_);
  RCLCPP_INFO(get_logger(), "Topic discovery period: %.1f s",
              discovery_period_);

  // Publishers
  path_publisher_ = create_publisher<nav_msgs::msg::Path>(EGO_TRAJECTORY_TOPIC,
                                                          QOS_BACKLOG_ODOM);
  marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      EGO_TRAJECTORY_MARKERS_TOPIC, QOS_BACKLOG);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  path_.header.frame_id = odom_frame_id_;

  // Initialize current odom transform to identity
  current_odom_baselink_tf_.setIdentity();

  // Create timer for path publishing at fixed rate
  auto timer_period = std::chrono::duration<double>(1.0 / path_publish_rate_);
  path_publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&Ymir::pathPublishTimerCallback, this));

  // Auto-discover and subscribe to radar ego_motion topics (initial discovery)
  discoverAndSubscribeToRadars();

  // Create timer for periodic topic re-discovery
  auto discovery_period = std::chrono::duration<double>(discovery_period_);
  discovery_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(discovery_period),
      std::bind(&Ymir::discoverAndSubscribeToRadars, this));

  RCLCPP_INFO(get_logger(), "Ymir node initialized successfully");
}

tf2::Transform
Ymir::egoMotiontoTf(const oden_interfaces::msg::EgoMotion &ego_motion) {
  // Create a transform from the ego motion data
  tf2::Quaternion q(ego_motion.rotation_quat_x, ego_motion.rotation_quat_y,
                    ego_motion.rotation_quat_z, ego_motion.rotation_quat_w);
  tf2::Vector3 t(ego_motion.translation_x, ego_motion.translation_y,
                 ego_motion.translation_z);

  return tf2::Transform(q, t);
}

std::string Ymir::getSensorFrameId(const std::string &topic_name) {
  // Extract sensor frame ID from topic name
  // Expected format: /sensrad/radar_N/oden/ego_motion
  // Returns: radar_N

  // Find "radar_" in the topic name
  size_t radar_pos = topic_name.find("radar_");
  if (radar_pos != std::string::npos) {
    // Find the next "/" after "radar_"
    size_t slash_pos = topic_name.find('/', radar_pos);
    if (slash_pos != std::string::npos) {
      return topic_name.substr(radar_pos, slash_pos - radar_pos);
    }
  }

  // Fallback: extract anything that looks like radar_N
  std::regex radar_pattern("radar_\\d+");
  std::smatch match;
  std::string topic_str = topic_name;
  if (std::regex_search(topic_str, match, radar_pattern)) {
    return match.str();
  }

  // Last resort
  RCLCPP_WARN(
      get_logger(),
      "Could not extract radar frame ID from topic '%s', using 'radar_1'",
      topic_name.c_str());
  return "radar_1";
}

void Ymir::lowPassFilter(const tf2::Transform &new_measurement, double alpha) {
  // Apply exponential low-pass filter: output = alpha * new + (1-alpha) * old

  // Filter translation
  tf2::Vector3 filtered_translation =
      alpha * new_measurement.getOrigin() +
      (1.0 - alpha) * current_odom_baselink_tf_.getOrigin();

  // Filter rotation using SLERP (Spherical Linear Interpolation)
  tf2::Quaternion current_rot = current_odom_baselink_tf_.getRotation();
  tf2::Quaternion new_rot = new_measurement.getRotation();

  current_rot.normalize();
  new_rot.normalize();

  // Ensure quaternions are in the same hemisphere
  if (current_rot.dot(new_rot) < 0.0) {
    new_rot =
        tf2::Quaternion(-new_rot.x(), -new_rot.y(), -new_rot.z(), -new_rot.w());
  }

  // SLERP between current and new rotation
  tf2::Quaternion filtered_rotation = current_rot.slerp(new_rot, alpha);
  filtered_rotation.normalize();

  current_odom_baselink_tf_.setRotation(filtered_rotation);
  current_odom_baselink_tf_.setOrigin(filtered_translation);
}

void Ymir::odometryCallback(
    const oden_interfaces::msg::EgoMotion::SharedPtr ego_motion,
    const std::string &radar_namespace) {

  // Check validity flags
  bool is_valid = ego_motion->velocity_is_valid &&
                  ego_motion->rotational_rate_is_valid &&
                  ego_motion->odometry_is_valid;

  if (!is_valid) {
    RCLCPP_DEBUG(get_logger(), "Radar %s has invalid ego motion data, skipping",
                 radar_namespace.c_str());
    return;
  }

  // Get radar info
  const auto &radar_info = radar_info_[radar_namespace];
  const auto &sensor_frame_id = radar_info.sensor_frame_id;

  // Get the ego motion transform from this radar
  tf2::Transform ego_motion_tf = egoMotiontoTf(*ego_motion);

  // Get the sensor to base_link transform
  tf2::Transform sensor_to_baselink_tf;
  if (!getSensorToBaselinkTransform(radar_namespace, sensor_frame_id,
                                    ego_motion->header.stamp,
                                    sensor_to_baselink_tf)) {
    return;
  }
  // Compute the odom -> base_link transform from this radar's measurement
  tf2::Transform measured_odom_baselink_tf =
      ego_motion_tf * sensor_to_baselink_tf;

  // Update state based on number of active radars
  size_t num_radars = radar_info_.size();

  if (!odom_initialized_) {
    // First measurement - initialize state
    current_odom_baselink_tf_ = measured_odom_baselink_tf;
    odom_initialized_ = true;
    RCLCPP_INFO(get_logger(), "Odometry initialized from radar: %s",
                radar_namespace.c_str());
  } else if (num_radars == 1) {
    // Single radar - use measurement directly (no filtering)
    current_odom_baselink_tf_ = measured_odom_baselink_tf;
    RCLCPP_DEBUG(get_logger(),
                 "Single radar mode - using direct measurement from: %s",
                 radar_namespace.c_str());
  } else {
    // Multiple radars - apply low-pass filter
    lowPassFilter(measured_odom_baselink_tf, alpha_);
    RCLCPP_DEBUG(
        get_logger(),
        "Multi-radar mode - filtered measurement from: %s (%zu radars)",
        radar_namespace.c_str(), num_radars);
  }

  // Publish immediately
  publishOdometry(ego_motion->header.stamp);
}

void Ymir::publishOdometry(const rclcpp::Time &timestamp) {
  if (!odom_initialized_) {
    RCLCPP_DEBUG(get_logger(),
                 "Odometry not yet initialized, skipping publish");
    return;
  }

  // HACK: Use max(current_timestamp, last_published_time) to prevent going
  // backwards
  rclcpp::Time publish_time =
      timestamp > last_published_time_ ? timestamp : last_published_time_;

  if (timestamp < last_published_time_) {
    RCLCPP_DEBUG(get_logger(),
                 "Out-of-order timestamp detected! Incoming: %.0f.%09ld, Last: "
                 "%.0f.%09ld - using last",
                 timestamp.seconds(), timestamp.nanoseconds(),
                 last_published_time_.seconds(),
                 last_published_time_.nanoseconds());
  }

  last_published_time_ = publish_time;

  // [ISR CHANGE] Apply 180-degree rotation to the odom frame
  tf2::Quaternion rotation_offset;
  rotation_offset.setRPY(0.0, M_PI, -M_PI/2);

  // Broadcast TF transform
  geometry_msgs::msg::TransformStamped tf_out;
  tf_out.header.stamp = publish_time;
  tf_out.header.frame_id = odom_frame_id_;
  tf_out.child_frame_id = baselink_frame_id_;
  tf_out.transform = tf2::toMsg(current_odom_baselink_tf_);

  // [ISR CHANGE] Convert to tf2::Quaternion, multiply, and convert back
  tf2::Quaternion q_original;
  tf2::fromMsg(tf_out.transform.rotation, q_original);
  tf2::Quaternion q_rotated = rotation_offset * q_original;
  tf_out.transform.rotation = tf2::toMsg(q_rotated);

  tf_broadcaster_->sendTransform(tf_out);

  // Add to trajectory path
  geometry_msgs::msg::PoseStamped pose;
  pose.header = tf_out.header;
  pose.pose.position.x = tf_out.transform.translation.x;
  pose.pose.position.y = tf_out.transform.translation.y;
  pose.pose.position.z = tf_out.transform.translation.z;
  pose.pose.orientation = tf_out.transform.rotation;

  path_.header.stamp = pose.header.stamp;
  path_.poses.insert(path_.poses.begin(), pose);

  // Enforce maximum path length (rolling window)
  // Since we insert at the beginning, oldest poses are at the end
  if (max_path_length_ > 0 && path_.poses.size() > max_path_length_) {
    // Remove oldest poses from the end to maintain max length
    path_.poses.erase(path_.poses.begin() + max_path_length_,
                      path_.poses.end());
  }

  // Note: Path is now published by timer callback, not here

  // Publish trajectory marker (decimated for history visualization)
  publishTrajectoryMarker();

  RCLCPP_DEBUG(
      get_logger(),
      "Updated odometry - position: (%.3f, %.3f, %.3f), path size: %zu",
      current_odom_baselink_tf_.getOrigin().x(),
      current_odom_baselink_tf_.getOrigin().y(),
      current_odom_baselink_tf_.getOrigin().z(), path_.poses.size());
}

void Ymir::pathPublishTimerCallback() {
  // Publish the current path at fixed rate
  if (!path_.poses.empty()) {
    path_publisher_->publish(path_);
    RCLCPP_DEBUG(get_logger(), "Published path with %zu poses",
                 path_.poses.size());
  }
}

void Ymir::publishTrajectoryMarker() {
  // Only publish if distance threshold is enabled (> 0)
  if (marker_distance_threshold_ <= 0.0) {
    return;
  }

  // Get current position
  tf2::Vector3 current_position = current_odom_baselink_tf_.getOrigin();

  // Check if this is the first marker or if we've moved far enough
  if (marker_counter_ == 0) {
    // First marker - always publish
    last_marker_position_ = current_position;
  } else {
    // Calculate distance from last marker
    double distance = (current_position - last_marker_position_).length();

    if (distance < marker_distance_threshold_) {
      // Haven't moved far enough yet
      return;
    }

    // Update last marker position
    last_marker_position_ = current_position;
  }

  marker_counter_++;

  // Create arrow marker showing pose
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = odom_frame_id_;
  marker.header.stamp =
      last_published_time_; // Use odometry timestamp, not now()
  marker.ns = "ego_trajectory";
  marker.id = static_cast<int32_t>(marker_counter_);
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Set pose from current odometry
  marker.pose.position.x = current_position.x();
  marker.pose.position.y = current_position.y();
  marker.pose.position.z = current_position.z();

  tf2::Quaternion q = current_odom_baselink_tf_.getRotation();
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  // Set scale (arrow dimensions)
  // Arrow length scales with distance threshold for visual consistency
  marker.scale.x = MARKER_ARROW_SCALE_FACTOR * marker_distance_threshold_;
  marker.scale.y = marker_arrow_width_; // Arrow width
  marker.scale.z = marker_arrow_width_; // Arrow height

  // Set color (cyan - highly visible)
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  // Long lifetime (1 hour) - effectively permanent but allows RViz to process
  // it
  marker.lifetime = rclcpp::Duration::from_seconds(3600.0);

  marker_publisher_->publish(marker);

  RCLCPP_DEBUG(
      get_logger(),
      "Published trajectory marker %d at (%.3f, %.3f), distance: %.2f m",
      marker.id, marker.pose.position.x, marker.pose.position.y,
      (current_position - last_marker_position_).length());
}

// Auto-discover radar topics and subscribe to them
void Ymir::discoverAndSubscribeToRadars() {
  // Get all topics in the system
  auto topic_names_and_types = get_topic_names_and_types();

  // Compile regex pattern for matching ego_motion topics
  std::regex pattern(ego_motion_topic_pattern_);

  RCLCPP_DEBUG(get_logger(), "Scanning for ego_motion topics...");

  int new_radar_count = 0;
  for (const auto &[topic_name, topic_types] : topic_names_and_types) {
    // Check if topic matches our pattern
    if (std::regex_match(topic_name, pattern)) {
      // Skip if already subscribed to this topic
      if (radar_info_.find(topic_name) != radar_info_.end()) {
        RCLCPP_DEBUG(get_logger(), "Already subscribed to: %s",
                     topic_name.c_str());
        continue;
      }

      // Verify it's the correct message type
      bool correct_type = false;
      for (const auto &type : topic_types) {
        if (type == "oden_interfaces/msg/EgoMotion") {
          correct_type = true;
          break;
        }
      }

      if (!correct_type) {
        RCLCPP_WARN(get_logger(),
                    "Topic %s matches pattern but has wrong type, skipping",
                    topic_name.c_str());
        continue;
      }

      // Create subscription
      auto subscription = create_subscription<oden_interfaces::msg::EgoMotion>(
          topic_name, QOS_BACKLOG,
          [this,
           topic_name](const oden_interfaces::msg::EgoMotion::SharedPtr msg) {
            odometryCallback(msg, topic_name);
          });

      ego_motion_subscriptions_.push_back(subscription);

      // Initialize radar info entry
      RadarInfo radar_info;
      radar_info.namespace_name = topic_name;
      radar_info.sensor_frame_id = getSensorFrameId(topic_name);
      radar_info_[topic_name] = radar_info;

      RCLCPP_INFO(get_logger(), "Auto-subscribed to: %s (frame: %s)",
                  topic_name.c_str(), radar_info.sensor_frame_id.c_str());
      new_radar_count++;
    }
  }

  // Log status
  size_t total_radars = radar_info_.size();
  if (total_radars == 0) {
    RCLCPP_INFO(get_logger(),
                "No ego_motion topics found matching pattern '%s'. "
                "Will continue scanning...",
                ego_motion_topic_pattern_.c_str());
  } else if (new_radar_count > 0) {
    RCLCPP_INFO(get_logger(), "Now tracking %zu radar(s) (added %d new)",
                total_radars, new_radar_count);
  }
}

// Get sensor-to-baselink transform for a radar (with caching)
bool Ymir::getSensorToBaselinkTransform(
    const std::string &radar_ns, const std::string &sensor_frame_id,
    const builtin_interfaces::msg::Time &stamp, tf2::Transform &transform_out) {
  try {
    geometry_msgs::msg::TransformStamped sensor_to_baselink_msg =
        tf_buffer_.lookupTransform(sensor_frame_id, baselink_frame_id_, stamp,
                                   tf2::durationFromSec(0.1));

    tf2::fromMsg(sensor_to_baselink_msg.transform, transform_out);

    // Cache the transform
    sensor_to_baselink_transforms_[radar_ns] = transform_out;
    return true;

  } catch (tf2::TransformException &ex) {
    // Try to use cached transform
    auto it = sensor_to_baselink_transforms_.find(radar_ns);
    if (it != sensor_to_baselink_transforms_.end()) {
      transform_out = it->second;
      RCLCPP_DEBUG(get_logger(), "Using cached transform from %s to %s",
                   sensor_frame_id.c_str(), baselink_frame_id_.c_str());
      return true;
    } else {
      RCLCPP_WARN(get_logger(), "Could not get transform from %s to %s: %s",
                  sensor_frame_id.c_str(), baselink_frame_id_.c_str(),
                  ex.what());
      return false;
    }
  }
}

} // namespace ymir
