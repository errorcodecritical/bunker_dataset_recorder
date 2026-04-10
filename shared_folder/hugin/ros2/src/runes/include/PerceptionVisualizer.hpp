// Copyright (c) Sensrad 2023-2025
#pragma once

#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <oden_interfaces/msg/free_space.hpp>
#include <oden_interfaces/msg/ground_plane.hpp>
#include <oden_interfaces/msg/multi_object_tracking.hpp>
#include <oden_interfaces/msg/occupancy_grid.hpp>
#include <oden_interfaces/msg/region_of_interest.hpp>
#include <oden_interfaces/msg/static_map.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include "RegionOfInterestVisualization.hpp"
#include <liboden/IO_Types.hpp>

namespace perception_visualizer {

// NOLINTBEGIN(misc-non-private-member-variables-in-classes)

// Class for re-mapping of track id to increment only for mature tracks
struct TrackIdManager {
  common::TrackState previous_track_state;
  bool visited;
  uint32_t output_track_id;
  TrackIdManager()
      : previous_track_state(common::TrackState::TENTATIVE), visited(true),
        output_track_id(0) {}
};
} // namespace perception_visualizer
#include "PolygonGenerator.hpp"

namespace perception_visualizer {
// NOLINTEND(misc-non-private-member-variables-in-classes)

class PerceptionVisualizer : public rclcpp::Node {
public:
  explicit PerceptionVisualizer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~PerceptionVisualizer() override = default;
  PerceptionVisualizer(const PerceptionVisualizer &) = delete;
  PerceptionVisualizer &operator=(const PerceptionVisualizer &) = delete;
  PerceptionVisualizer(PerceptionVisualizer &&) noexcept = default;
  PerceptionVisualizer &operator=(PerceptionVisualizer &&) noexcept = default;

private:
  // Quality of service queue length
  constexpr static int QOS_BACKLOG_ = 1;

  constexpr static std::size_t MAP_RESERVE_SIZE = 2000;
  constexpr static int64_t MAP_PUBLISH_RATE_MS = 250;

  // For re-mapping of track id to increment only for mature tracks
  uint32_t mature_track_id_counter_ = 0;

  std::unordered_map<uint32_t, TrackIdManager> trackid_manager_map_;

  float radar_elevation_angle_;

  // Subscriptions
  rclcpp::Subscription<oden_interfaces::msg::MultiObjectTracking>::SharedPtr
      object_tracks_subscription_;

  rclcpp::Subscription<oden_interfaces::msg::RegionOfInterest>::SharedPtr
      roi_subscription_;
  rclcpp::Subscription<oden_interfaces::msg::StaticMap>::SharedPtr
      static_map_subscription_;

  rclcpp::Subscription<oden_interfaces::msg::OccupancyGrid>::SharedPtr
      occupancy_grid_subscription_;

  rclcpp::Subscription<oden_interfaces::msg::FreeSpace>::SharedPtr
      free_space_subscription_;

  // Map related variables
  std::vector<geometry_msgs::msg::Point> latest_static_voxels_;
  std::vector<geometry_msgs::msg::Point> latest_anomaly_voxels_;

  std::vector<geometry_msgs::msg::Point> local_static_voxels_;
  std::vector<geometry_msgs::msg::Point> local_anomaly_voxels_;

  visualization_msgs::msg::Marker marker_static_;
  visualization_msgs::msg::Marker marker_anomaly_;

  rclcpp::TimerBase::SharedPtr marker_timer_;
  std::mutex voxel_mutex_;

  // Sync subscriptions with message_filters
  // Note that it's necessary to subscribe ground plane data msg twice because
  // these two subscriptions are in different types, we do so to ensure that
  // ground plane data callback can be called independent from free space data
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
      parameter_subscription_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      aabb_object_tracks_publisher_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      roi_marker_publisher_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      static_map_publisher_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      occupancy_grid_publisher_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      anomaly_publisher_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      free_space_ground_polygon_publisher_;

  void updateMatureTrackId(
      const oden_interfaces::msg::MultiObjectTracking::ConstSharedPtr
          &tracked_objects);

  // Callback functions
  void objectTracksCallback(
      const oden_interfaces::msg::MultiObjectTracking::ConstSharedPtr
          &object_tracks_data);

  void roiCallback(
      const oden_interfaces::msg::RegionOfInterest::ConstSharedPtr &roi_data);

  void mapDataCallback(
      const oden_interfaces::msg::StaticMap::ConstSharedPtr &map_data);

  void occupancyGridCallback(
      const oden_interfaces::msg::OccupancyGrid::ConstSharedPtr &og_data);

  void freeSpaceCallback(
      const oden_interfaces::msg::FreeSpace::ConstSharedPtr &free_space_data);

  void mapMarkerPublishTimerCallback();

  void parameterEventCallback(
      const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
};
} // namespace perception_visualizer
