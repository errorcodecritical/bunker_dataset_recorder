// Copyright (c) Sensrad 2023-2025

#include "PerceptionVisualizer.hpp"
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(perception_visualizer::PerceptionVisualizer)

namespace perception_visualizer {

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
PerceptionVisualizer::PerceptionVisualizer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("runes", options),
      radar_elevation_angle_(
          declare_parameter<float>("elevation_angle", 1.0f)) {
  RCLCPP_INFO(get_logger(), "Runes node started");

  // Create subscriptions
  object_tracks_subscription_ =
      create_subscription<oden_interfaces::msg::MultiObjectTracking>(
          "object_tracks", QOS_BACKLOG_,
          std::bind(&PerceptionVisualizer::objectTracksCallback, this,
                    std::placeholders::_1));

  roi_subscription_ =
      create_subscription<oden_interfaces::msg::RegionOfInterest>(
          "region_of_interest", QOS_BACKLOG_,
          std::bind(&PerceptionVisualizer::roiCallback, this,
                    std::placeholders::_1));

  static_map_subscription_ =
      create_subscription<oden_interfaces::msg::StaticMap>(
          "static_map", QOS_BACKLOG_,
          std::bind(&PerceptionVisualizer::mapDataCallback, this,
                    std::placeholders::_1));

  occupancy_grid_subscription_ =
      create_subscription<oden_interfaces::msg::OccupancyGrid>(
          "occupancy_grid", QOS_BACKLOG_,
          std::bind(&PerceptionVisualizer::occupancyGridCallback, this,
                    std::placeholders::_1));

  free_space_subscription_ =
      create_subscription<oden_interfaces::msg::FreeSpace>(
          "free_space_polygon", QOS_BACKLOG_,
          std::bind(&PerceptionVisualizer::freeSpaceCallback, this,
                    std::placeholders::_1));

  parameter_subscription_ =
      create_subscription<rcl_interfaces::msg::ParameterEvent>(
          "/parameter_events", QOS_BACKLOG_,
          std::bind(&PerceptionVisualizer::parameterEventCallback, this,
                    std::placeholders::_1));

  // Create publishers - publish under runes/ sub-namespace
  aabb_object_tracks_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
          "runes/aabb_object_tracks", QOS_BACKLOG_);

  roi_marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("runes/roi_marker",
                                                             QOS_BACKLOG_);
  static_map_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "runes/static_map_voxels", QOS_BACKLOG_);

  anomaly_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "runes/anomaly_voxels", QOS_BACKLOG_);

  occupancy_grid_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
          "runes/occupancy_grid_markers", QOS_BACKLOG_);

  free_space_ground_polygon_publisher_ =
      create_publisher<visualization_msgs::msg::Marker>(
          "runes/free_space_ground_polygon", QOS_BACKLOG_);

  // Init map related variables
  latest_static_voxels_.reserve(MAP_RESERVE_SIZE);
  latest_anomaly_voxels_.reserve(MAP_RESERVE_SIZE);
  local_static_voxels_.reserve(MAP_RESERVE_SIZE);
  local_anomaly_voxels_.reserve(MAP_RESERVE_SIZE);

  marker_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(MAP_PUBLISH_RATE_MS),
      std::bind(&PerceptionVisualizer::mapMarkerPublishTimerCallback, this));

  // Populate all known map marker attributes
  marker_static_.ns = "static_map";
  marker_static_.id = 0;
  marker_static_.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker_static_.action = visualization_msgs::msg::Marker::ADD;
  marker_static_.color.r = 0.0f;
  marker_static_.color.g = 1.0f;
  marker_static_.color.b = 0.0f;
  marker_static_.color.a = 1.0f;

  marker_anomaly_ = marker_static_;
  marker_anomaly_.ns = "anomalies";
  marker_anomaly_.id = 1;
  marker_anomaly_.color.r = 1.0f;
  marker_anomaly_.color.g = 0.0f;
  marker_anomaly_.color.b = 0.0f;
  marker_anomaly_.color.a = 1.0f;
}

// Callback functions below

// Logic for updating mature track id
void PerceptionVisualizer::updateMatureTrackId(
    const oden_interfaces::msg::MultiObjectTracking::ConstSharedPtr
        &tracked_objects) {
  // Set visited to false on all tracks in the map to detect that tracks have
  // been removed
  for (auto &trackid_manager : trackid_manager_map_) {
    trackid_manager.second.visited = false;
  }

  // Add new tracks to the trackid_manager_map_ and record which tracks are
  // still present. Also look for transitions from TENTATIVE to MATURE
  for (const auto &tracked_object : tracked_objects->object_track_list) {
    // Check for new tracks and emplace if not present
    trackid_manager_map_.try_emplace(tracked_object.track_id, TrackIdManager());

    // Look for transitions. Since ROS2 interface involves uint8 rather than
    // enums, we cast for clarity
    const auto current_track_state =
        static_cast<common::TrackState>(tracked_object.track_state);
    const auto previous_track_state =
        trackid_manager_map_[tracked_object.track_id].previous_track_state;

    if ((current_track_state == common::TrackState::MATURE) and
        (previous_track_state == common::TrackState::TENTATIVE)) {
      trackid_manager_map_[tracked_object.track_id].output_track_id =
          mature_track_id_counter_;
      mature_track_id_counter_++;
    }
    trackid_manager_map_[tracked_object.track_id].previous_track_state =
        current_track_state;
    trackid_manager_map_[tracked_object.track_id].visited = true;
  }

  // Remove tracks that are no longer present
  for (auto it = trackid_manager_map_.begin();
       it != trackid_manager_map_.end();) {
    if (!it->second.visited) {
      it = trackid_manager_map_.erase(it);
    } else {
      ++it;
    }
  }
}

// Publish topic to visualize the occupancy grid
void PerceptionVisualizer::occupancyGridCallback(
    const oden_interfaces::msg::OccupancyGrid::ConstSharedPtr &og_data) {
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker voxels_marker;
  voxels_marker.header = og_data->header;
  voxels_marker.ns = "occupancy_grid";
  voxels_marker.id = 0;
  voxels_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  voxels_marker.action = visualization_msgs::msg::Marker::ADD;
  voxels_marker.scale.x = og_data->voxel_size;
  voxels_marker.scale.y = og_data->voxel_size;
  voxels_marker.scale.z = og_data->voxel_size;
  voxels_marker.pose.orientation.w = 1.0;

  for (const auto &voxel : og_data->list_of_voxels) {
    geometry_msgs::msg::Point pt;
    pt.x = voxel.x_center_position;
    pt.y = voxel.y_center_position;
    pt.z = voxel.z_center_position;
    voxels_marker.points.push_back(pt);

    std_msgs::msg::ColorRGBA color;
    color.a = 0.5; // Opacity

    // Color: light red -> red with increasing occupancy probability
    const float min_log_odds =
        0.4055f; // Corresponding to occupancy probability 0.6
    float p =
        std::max(0.0f, (voxel.log_odds - min_log_odds)) / (1.0f - min_log_odds);
    color.r = 1.0;
    color.g = 1.0 - p;
    color.b = 1.0 - p;
    voxels_marker.colors.push_back(color);
  }

  marker_array.markers.push_back(voxels_marker);

  occupancy_grid_publisher_->publish(marker_array);
}

// Record latest static map data; but do not publish to ROS2 yet to save on
// bandwidth
void PerceptionVisualizer::mapDataCallback(
    const oden_interfaces::msg::StaticMap::ConstSharedPtr &map_data) {
  local_static_voxels_.clear();
  local_anomaly_voxels_.clear();

  for (const auto &voxel : map_data->grid_centers) {
    geometry_msgs::msg::Point p;
    p.x = voxel.x_center_position;
    p.y = voxel.y_center_position;
    p.z = voxel.z_center_position;

    if (voxel.anomaly)
      local_anomaly_voxels_.push_back(p);
    else
      local_static_voxels_.push_back(p);
  }

  // Lock the mutex to safely update the latest static and anomaly voxels
  {
    std::lock_guard<std::mutex> lock(voxel_mutex_);
    latest_static_voxels_ = std::move(local_static_voxels_);
    latest_anomaly_voxels_ = std::move(local_anomaly_voxels_);
  }

  marker_static_.header.frame_id = map_data->header.frame_id;
  marker_static_.header.stamp = map_data->header.stamp;
  marker_static_.scale.x = marker_static_.scale.y = marker_static_.scale.z =
      map_data->voxel_size;

  marker_anomaly_.header.frame_id = map_data->header.frame_id;
  marker_anomaly_.header.stamp = map_data->header.stamp;
  marker_anomaly_.scale.x = marker_anomaly_.scale.y = marker_anomaly_.scale.z =
      map_data->voxel_size;
}

// Publish the static map and anomaly voxels
void PerceptionVisualizer::mapMarkerPublishTimerCallback() {
  std::vector<geometry_msgs::msg::Point> static_voxels;
  std::vector<geometry_msgs::msg::Point> anomaly_voxels;

  // Lock the mutex to safely access the latest static and anomaly voxels
  {
    std::lock_guard<std::mutex> lock(voxel_mutex_);
    static_voxels = latest_static_voxels_;
    anomaly_voxels = latest_anomaly_voxels_;
  }

  // Only publish if there are voxels to visualize
  if (static_voxels.empty() && anomaly_voxels.empty()) {
    return;
  }

  marker_static_.points = std::move(static_voxels);
  marker_anomaly_.points = std::move(anomaly_voxels);

  static_map_publisher_->publish(marker_static_);
  anomaly_publisher_->publish(marker_anomaly_);
}

// Visualize the object tracks
void PerceptionVisualizer::objectTracksCallback(
    const oden_interfaces::msg::MultiObjectTracking::ConstSharedPtr
        &object_tracks_data) {
  // NOLINTNEXTLINE(clang-analyzer-cplusplus.Move)
  updateMatureTrackId(object_tracks_data);
  // NOLINTNEXTLINE(clang-analyzer-cplusplus.Move)
  aabb_object_tracks_publisher_->publish(PolygonGenerator::getAABBObjectTracks(
      object_tracks_data, trackid_manager_map_, radar_elevation_angle_));
}

void PerceptionVisualizer::roiCallback(
    const oden_interfaces::msg::RegionOfInterest::ConstSharedPtr &roi_data) {
  roi_marker_publisher_->publish(
      RegionOfInterestVisualization::getROIMarker(roi_data));
}

void PerceptionVisualizer::freeSpaceCallback(
    const oden_interfaces::msg::FreeSpace::ConstSharedPtr &free_space_data) {

  if (!free_space_data->is_valid ||
      free_space_data->free_space_polygon.empty()) {
    return;
  }

  // Convert ground polygon from message to Eigen format
  std::vector<Eigen::Vector3f> polygon;
  polygon.reserve(free_space_data->free_space_polygon.size());
  for (const auto &point : free_space_data->free_space_polygon) {
    polygon.emplace_back(point.x, point.y, point.z);
  }

  // Generate and publish the marker
  auto marker = PolygonGenerator::getFreeSpaceGroundPolygonMarker(
      polygon, free_space_data->header, free_space_data->ground_plane_is_valid);
  free_space_ground_polygon_publisher_->publish(marker);
}

void PerceptionVisualizer::parameterEventCallback(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
  for (const auto &changed_parameters : event->changed_parameters) {
    if (changed_parameters.name == "radar_elevation_angle") {
      radar_elevation_angle_ =
          static_cast<float>(changed_parameters.value.double_value);
    }
  }
}

} // namespace perception_visualizer
