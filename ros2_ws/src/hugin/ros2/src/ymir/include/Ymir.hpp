// Copyright (c) Sensrad 2025

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <oden_interfaces/msg/ego_motion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <regex>
#include <string>
#include <vector>

namespace ymir {

// Ymir node aggregates odometry data from multiple radars
// This node subscribes to ego_motion messages from multiple radar namespaces
// and publishes an aggregated odom TF and ego_trajectory path.
class Ymir : public rclcpp::Node {
private:
  static constexpr int QOS_BACKLOG = 1;
  static constexpr int QOS_BACKLOG_ODOM = 10;
  static constexpr const char *EGO_TRAJECTORY_TOPIC = "ymir/ego_trajectory";
  static constexpr const char *EGO_TRAJECTORY_MARKERS_TOPIC =
      "ymir/ego_trajectory_markers";
  static constexpr double MARKER_ARROW_SCALE_FACTOR =
      0.8; // Arrow length = scale_factor * distance_threshold

  // Structure to hold radar information
  struct RadarInfo {
    std::string namespace_name;
    std::string sensor_frame_id;
  };

  // Map of radar namespace to radar information
  std::map<std::string, RadarInfo> radar_info_;

  // Subscribers (one per radar)
  std::vector<rclcpp::Subscription<oden_interfaces::msg::EgoMotion>::SharedPtr>
      ego_motion_subscriptions_;

  // Publishers
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      marker_publisher_;
  nav_msgs::msg::Path path_;

  // Timer for path publishing
  rclcpp::TimerBase::SharedPtr path_publish_timer_;

  // Timer for periodic topic discovery
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  double discovery_period_; // Period (seconds) for topic re-discovery

  // Marker tracking
  size_t marker_counter_;
  double marker_distance_threshold_; // Minimum distance (m) between markers
  double marker_arrow_width_;
  tf2::Vector3 last_marker_position_; // Position of last published marker

  // Frame IDs
  std::string odom_frame_id_;     // Odometry frame id
  std::string baselink_frame_id_; // Base link frame id

  // Configuration
  std::string ego_motion_topic_pattern_; // Regex pattern for discovering
                                         // ego_motion topics
  double alpha_; // Low-pass filter coefficient for multi-radar fusion (0 <
                 // alpha <= 1, higher = more responsive)
  size_t max_path_length_;   // Maximum number of poses in path (0 = unlimited)
  double path_publish_rate_; // Rate (Hz) at which to publish path messages

  // TF2 buffer and listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Transform between sensor frame and base link frame (cached per radar)
  std::map<std::string, tf2::Transform> sensor_to_baselink_transforms_;

  // Current aggregated odom -> base_link transform (filtered state)
  tf2::Transform current_odom_baselink_tf_;
  bool odom_initialized_;

  // Track last published timestamp to prevent going backwards in time
  rclcpp::Time last_published_time_;

  // Convert EgoMotion message to tf2::Transform
  tf2::Transform
  egoMotiontoTf(const oden_interfaces::msg::EgoMotion &ego_motion);

  // Callback function for ego motion from a specific radar
  void
  odometryCallback(const oden_interfaces::msg::EgoMotion::SharedPtr ego_motion,
                   const std::string &radar_namespace);

  // Get sensor-to-baselink transform for a radar (with caching)
  bool getSensorToBaselinkTransform(const std::string &radar_ns,
                                    const std::string &sensor_frame_id,
                                    const builtin_interfaces::msg::Time &stamp,
                                    tf2::Transform &transform_out);

  // Auto-discover and subscribe to radar topics
  void discoverAndSubscribeToRadars();

  // Apply low-pass filter to blend new transform with current state
  void lowPassFilter(const tf2::Transform &new_measurement, double alpha);

  // Publish the current aggregated odometry
  void publishOdometry(const rclcpp::Time &timestamp);

  // Timer callback to publish path at fixed rate
  void pathPublishTimerCallback();

  // Publish trajectory marker (decimated for full history)
  void publishTrajectoryMarker();

  // Get sensor frame ID from topic name
  std::string getSensorFrameId(const std::string &topic_name);

  // [ISR CHANGE] Helper function to apply rotation offset
  tf2::Transform applyRotationOffset(const tf2::Transform& transform) const {
    tf2::Transform result = transform;
    tf2::Quaternion rotation_offset;
    rotation_offset.setRPY(0.0, M_PI, -M_PI/2);  // 180° around Y, -90° around Z
    
    // Rotate position
    result.setOrigin(tf2::quatRotate(rotation_offset, transform.getOrigin()));
    
    // Rotate orientation
    result.setRotation(rotation_offset * transform.getRotation());
    
    return result;
  }

public:
  explicit Ymir(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  virtual ~Ymir() = default;
};
} // namespace ymir
