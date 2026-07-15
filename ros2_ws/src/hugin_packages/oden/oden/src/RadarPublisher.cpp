// Copyright (c) Sensrad 2025
#include "RadarPublisher.hpp"

using PointField = sensor_msgs::msg::PointField;
using PointCloud2ConstIterator = sensor_msgs::PointCloud2ConstIterator<float>;
using PointCloud2Iterator = sensor_msgs::PointCloud2Iterator<float>;
using PointCloud2IteratorINT16 = sensor_msgs::PointCloud2Iterator<int16_t>;
using PointCloud2IteratorUINT8 = sensor_msgs::PointCloud2Iterator<uint8_t>;
using PointCloud2IteratorUINT32 = sensor_msgs::PointCloud2Iterator<uint32_t>;

// Constructor for RadarPublisher
RadarPublisher::RadarPublisher(rclcpp::Node *node)
    : ext_pc_modifier_{ext_pc_to_publish_} {
  // Create publishers for Oden processed data
  // All topics published under oden/ sub-namespace
  detections_publisher_ =
      node->create_publisher<oden_interfaces::msg::Detections>(
          "oden/detections", QOS_BACKLOG);

  odometry_publisher_ = node->create_publisher<oden_interfaces::msg::EgoMotion>(
      "oden/ego_motion", QOS_BACKLOG);

  local_map_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "oden/local_map_cloud", QOS_BACKLOG);
  local_map_modifier_.setPointCloud2Fields(3, "x", 1, PointField::FLOAT32, "y",
                                           1, PointField::FLOAT32, "z", 1,
                                           PointField::FLOAT32);
  object_tracks_publisher_ =
      node->create_publisher<oden_interfaces::msg::MultiObjectTracking>(
          "oden/object_tracks", QOS_BACKLOG);

  groundplane_publisher_ =
      node->create_publisher<oden_interfaces::msg::GroundPlane>(
          "oden/ground_plane_data", QOS_BACKLOG);
  extended_pointcloud_publisher_ =
      node->create_publisher<sensor_msgs::msg::PointCloud2>(
          "oden/extended_point_cloud", QOS_BACKLOG);

  statistics_publisher_ =
      node->create_publisher<oden_interfaces::msg::OdenStatistics>(
          "oden/statistics", QOS_BACKLOG);
  roi_publisher_ =
      node->create_publisher<oden_interfaces::msg::RegionOfInterest>(
          "oden/region_of_interest", QOS_BACKLOG);

  static_map_publisher_ =
      node->create_publisher<oden_interfaces::msg::StaticMap>("oden/static_map",
                                                              QOS_BACKLOG);

  occupancy_grid_publisher_ =
      node->create_publisher<oden_interfaces::msg::OccupancyGrid>(
          "oden/occupancy_grid", QOS_BACKLOG);

  free_space_volume_publisher_ =
      node->create_publisher<sensor_msgs::msg::Image>(
          "oden/free_space_volume/image", QOS_BACKLOG);

  free_space_volume_camera_info_publisher_ =
      node->create_publisher<sensor_msgs::msg::CameraInfo>(
          "oden/free_space_volume/camera_info", QOS_BACKLOG);

  free_space_volume_raw_publisher_ =
      node->create_publisher<sensor_msgs::msg::PointCloud2>(
          "oden/free_space_volume/raw", QOS_BACKLOG);

  free_space_publisher_ =
      node->create_publisher<oden_interfaces::msg::FreeSpace>(
          "oden/free_space_polygon", QOS_BACKLOG);

  ext_pc_modifier_.setPointCloud2Fields(
      20, "x", 1, PointField::FLOAT32, "y", 1, PointField::FLOAT32, "z", 1,
      PointField::FLOAT32, "power", 1, PointField::FLOAT32, "doppler", 1,
      PointField::FLOAT32, "delta_velocity", 1, PointField::FLOAT32,
      "motion_status", 1, PointField::UINT8, "available_for_tracker", 1,
      PointField::UINT8, "cluster_idx", 1, PointField::INT16,
      "annotation_cluster_idx", 1, PointField::INT16, "annotation_class", 1,
      PointField::UINT8, "distance_to_ground_plane", 1, PointField::FLOAT32,
      "range", 1, PointField::FLOAT32, "multi_path", 1, PointField::UINT8,
      "radar_cross_section", 1, PointField::FLOAT32, "rgba", 1,
      PointField::UINT32, "ghost_track", 1, PointField::UINT8,
      "track_log_likelihood", 1, PointField::FLOAT32, "is_local_max", 1,
      PointField::UINT8, "occluded", 1, PointField::UINT8);
}

// Publish clustered detection
void RadarPublisher::publishDetections(
    const common::Detections &detections,
    const std_msgs::msg::Header &header) const {

  oden_interfaces::msg::Detections pub_det;
  pub_det.header = header;

  // Loop over incoming detections
  for (const auto &det_i : detections) {
    oden_interfaces::msg::Detection this_det;

    this_det.range = det_i.range;
    this_det.azimuth = det_i.azimuth;
    this_det.elevation = det_i.elevation;
    this_det.radial_speed = det_i.radial_speed;
    this_det.power = det_i.power;
    this_det.power_std = det_i.power_std;
    this_det.doppler_std = det_i.doppler_std;
    this_det.position[0] = det_i.position(0);
    this_det.position[1] = det_i.position(1);
    this_det.position[2] = det_i.position(2);
    // Using row major order
    this_det.covariance[0] = det_i.covariance(0, 0);
    this_det.covariance[1] = det_i.covariance(0, 1);
    this_det.covariance[2] = det_i.covariance(0, 2);
    this_det.covariance[3] = det_i.covariance(1, 0);
    this_det.covariance[4] = det_i.covariance(1, 1);
    this_det.covariance[5] = det_i.covariance(1, 2);
    this_det.covariance[6] = det_i.covariance(2, 0);
    this_det.covariance[7] = det_i.covariance(2, 1);
    this_det.covariance[8] = det_i.covariance(2, 2);
    this_det.type = det_i.type;
    this_det.type_probability = det_i.type_probability;
    this_det.label_id = det_i.label_id;
    this_det.num_valid_detections = det_i.num_valid_detections;

    pub_det.detection_list.push_back(this_det);
  }

  detections_publisher_->publish(pub_det);
}

// Publish information about the ego motion
void RadarPublisher::publishEgoMotion(
    const std::optional<Eigen::Vector3f> &velocity_xyz,
    const bool velocity_is_valid,
    const std::optional<Eigen::Vector3f> &rotational_rate_xyz,
    const bool odometry_is_active, const Eigen::Vector3f &pose_translation,
    const Eigen::Quaternionf &pose_quaternion,
    const Eigen::Vector3f &delta_translation,
    const Eigen::Quaternionf &delta_rotation_quaternion,
    const std_msgs::msg::Header &header) const {

  oden_interfaces::msg::EgoMotion ego_motion;

  ego_motion.header = header;

  ego_motion.velocity_is_valid = false;
  ego_motion.velocity_x = 0.0f;
  ego_motion.velocity_y = 0.0f;
  ego_motion.velocity_z = 0.0f;
  if (velocity_xyz) {
    ego_motion.velocity_is_valid = velocity_is_valid;
    ego_motion.velocity_x = velocity_xyz->x();
    ego_motion.velocity_y = velocity_xyz->y();
    ego_motion.velocity_z = velocity_xyz->z();
  }

  // Get the rotational rate
  ego_motion.rotational_rate_x = 0.0f;
  ego_motion.rotational_rate_y = 0.0f;
  ego_motion.rotational_rate_z = 0.0f;
  ego_motion.rotational_rate_is_valid = false;
  if (rotational_rate_xyz) {
    ego_motion.rotational_rate_is_valid = true;
    ego_motion.rotational_rate_x = rotational_rate_xyz->x();
    ego_motion.rotational_rate_y = rotational_rate_xyz->y();
    ego_motion.rotational_rate_z = rotational_rate_xyz->z();
  }

  // Get the absolute pose
  ego_motion.odometry_is_valid = false;
  ego_motion.translation_x = 0.0f;
  ego_motion.translation_y = 0.0f;
  ego_motion.translation_z = 0.0f;
  ego_motion.rotation_quat_x = 0.0f;
  ego_motion.rotation_quat_y = 0.0f;
  ego_motion.rotation_quat_z = 0.0f;
  ego_motion.rotation_quat_w = 1.0f;
  if (odometry_is_active) {
    ego_motion.odometry_is_valid = true;
    ego_motion.translation_x = pose_translation.x();
    ego_motion.translation_y = pose_translation.y();
    ego_motion.translation_z = pose_translation.z();
    ego_motion.rotation_quat_x = pose_quaternion.x();
    ego_motion.rotation_quat_y = pose_quaternion.y();
    ego_motion.rotation_quat_z = pose_quaternion.z();
    ego_motion.rotation_quat_w = pose_quaternion.w();

    ego_motion.delta_translation_x = delta_translation.x();
    ego_motion.delta_translation_y = delta_translation.y();
    ego_motion.delta_translation_z = delta_translation.z();
    ego_motion.delta_rotation_quat_x = delta_rotation_quaternion.x();
    ego_motion.delta_rotation_quat_y = delta_rotation_quaternion.y();
    ego_motion.delta_rotation_quat_z = delta_rotation_quaternion.z();
    ego_motion.delta_rotation_quat_w = delta_rotation_quaternion.w();
  }

  odometry_publisher_->publish(ego_motion);
}

void RadarPublisher::publishOdometryLocalMap(
    const std::vector<Eigen::Vector3f> &local_map,
    const std_msgs::msg::Header &header) {
  // Clear message from previous iteration
  local_map_modifier_.clear();
  local_map_modifier_.resize(local_map.size());

  local_map_pc_.header = header;
  local_map_pc_.header.frame_id = "odom";

  local_map_pc_.width = static_cast<uint32_t>(local_map.size());
  local_map_pc_.height = 1;

  // Point cloud iterators
  PointCloud2Iterator x_it(local_map_pc_, "x");
  PointCloud2Iterator y_it(local_map_pc_, "y");
  PointCloud2Iterator z_it(local_map_pc_, "z");

  // Copy data
  for (const auto &p : local_map) {
    *x_it = p.x();
    *y_it = p.y();
    *z_it = p.z();
    ++x_it;
    ++y_it;
    ++z_it;
  }

  // Publish the local map point cloud
  local_map_publisher_->publish(local_map_pc_);
}

// Publish tracked objects
void RadarPublisher::publishTrackedObjects(
    const common::TrackedObjects &tracked_objects,
    const std_msgs::msg::Header &header) const {

  // Prepare the message
  oden_interfaces::msg::MultiObjectTracking object_tracks;
  object_tracks.header = header;

  // Loop over the tracked objects and fill the message
  for (const auto &track : tracked_objects) {
    oden_interfaces::msg::ObjectTrack single_track;
    single_track.track_id = track.track_id;
    single_track.track_state = track.track_state;
    single_track.yaw = track.yaw;
    single_track.type = track.type;

    // Copy position and velocity arrays
    for (size_t i = 0; i < 3; ++i) {
      single_track.position[i] = track.position[i];
      single_track.velocity[i] = track.velocity[i];
    }

    // Copy extent array (already stored row-major)
    for (size_t i = 0; i < 9; ++i) {
      single_track.extent[i] = track.extent[i];
    }

    object_tracks.object_track_list.push_back(single_track);
  }

  object_tracks_publisher_->publish(object_tracks);
}

// Publish ground plane model
void RadarPublisher::publishGroundplane(
    const bool is_valid, const Eigen::Vector4f &plane_coef,
    const std_msgs::msg::Header &header) const {
  oden_interfaces::msg::GroundPlane ground_plane_data;

  ground_plane_data.header = header;
  ground_plane_data.is_valid = is_valid;

  // Always publish plane coefficients (nominal or active)
  for (int i = 0; i < NOF_PLANE_COEF; ++i) {
    ground_plane_data.plane.coef[i] = plane_coef[i];
  }

  // Publish the message
  groundplane_publisher_->publish(ground_plane_data);
}

// Publish extended point cloud
void RadarPublisher::publishExtendedPointcloud(
    const common::OutgoingPointCloud &outgoing_pointcloud,
    const AnnotationPointCloud &annotated_pointcloud,
    const std_msgs::msg::Header &header) {
  ext_pc_modifier_.clear();

  std::size_t nof_points{outgoing_pointcloud.size()};

  ext_pc_modifier_.resize(nof_points);
  ext_pc_to_publish_.width = static_cast<uint32_t>(nof_points);
  ext_pc_to_publish_.height = 1;

  ext_pc_to_publish_.header = header;

  PointCloud2Iterator out_range_it(ext_pc_to_publish_, "range");
  PointCloud2Iterator out_power_it(ext_pc_to_publish_, "power");
  PointCloud2Iterator out_doppler_it(ext_pc_to_publish_, "doppler");
  PointCloud2Iterator out_x_it(ext_pc_to_publish_, "x");
  PointCloud2Iterator out_y_it(ext_pc_to_publish_, "y");
  PointCloud2Iterator out_z_it(ext_pc_to_publish_, "z");
  PointCloud2Iterator out_delta_velocity_it(ext_pc_to_publish_,
                                            "delta_velocity");
  PointCloud2IteratorUINT8 out_motion_status_it(ext_pc_to_publish_,
                                                "motion_status");
  PointCloud2IteratorUINT8 out_available_for_tracker_it(
      ext_pc_to_publish_, "available_for_tracker");
  PointCloud2IteratorINT16 out_cluster_idx_it(ext_pc_to_publish_,
                                              "cluster_idx");
  PointCloud2IteratorINT16 out_annotation_cluster_idx_it(
      ext_pc_to_publish_, "annotation_cluster_idx");
  PointCloud2IteratorUINT8 out_annotation_class_it(ext_pc_to_publish_,
                                                   "annotation_class");
  PointCloud2Iterator out_distance_to_ground_plane(ext_pc_to_publish_,
                                                   "distance_to_ground_plane");
  PointCloud2IteratorUINT8 out_multi_path_it(ext_pc_to_publish_, "multi_path");

  PointCloud2Iterator out_radar_cross_section_it(ext_pc_to_publish_,
                                                 "radar_cross_section");

  PointCloud2IteratorUINT32 out_rgba_it(ext_pc_to_publish_, "rgba");
  PointCloud2IteratorUINT8 out_ghost_track_it(ext_pc_to_publish_,
                                              "ghost_track");
  PointCloud2Iterator out_track_log_likelihood_it(ext_pc_to_publish_,
                                                  "track_log_likelihood");
  PointCloud2IteratorUINT8 out_is_local_max_it(ext_pc_to_publish_,
                                               "is_local_max");
  PointCloud2IteratorUINT8 out_occluded_it(ext_pc_to_publish_, "occluded");

  // Create outgoing extended point cloud
  size_t nof_out_points = 0;
  for (size_t i = 0; i < nof_points; ++i) {

    *out_range_it = outgoing_pointcloud[i].range;
    *out_power_it = outgoing_pointcloud[i].power;
    *out_doppler_it = outgoing_pointcloud[i].doppler;
    *out_x_it = outgoing_pointcloud[i].x;
    *out_y_it = outgoing_pointcloud[i].y;
    *out_z_it = outgoing_pointcloud[i].z;
    *out_delta_velocity_it = outgoing_pointcloud[i].delta_velocity;
    *out_motion_status_it = outgoing_pointcloud[i].motion_status;
    *out_available_for_tracker_it =
        static_cast<uint8_t>(outgoing_pointcloud[i].available_for_tracker);
    *out_cluster_idx_it =
        static_cast<int16_t>(outgoing_pointcloud[i].cluster_idx);

    const auto sample_idx = outgoing_pointcloud[i].sample_index;
    *out_annotation_cluster_idx_it =
        annotated_pointcloud[sample_idx].getAnnotationClusterIdx();
    *out_annotation_class_it =
        annotated_pointcloud[sample_idx].getAnnotationClass();
    *out_distance_to_ground_plane =
        outgoing_pointcloud[i].distance_to_ground_plane;
    *out_multi_path_it = outgoing_pointcloud[i].multi_path;
    *out_radar_cross_section_it = outgoing_pointcloud[i].radar_cross_section;
    *out_rgba_it = setColorOfPoint(outgoing_pointcloud[i]);
    *out_ghost_track_it =
        static_cast<uint8_t>(outgoing_pointcloud[i].ghost_track);
    *out_track_log_likelihood_it = outgoing_pointcloud[i].track_log_likelihood;
    *out_is_local_max_it = static_cast<uint8_t>(
        outgoing_pointcloud[i].local_max_margin > LOCAL_MAX_MARGIN);
    *out_occluded_it = static_cast<uint8_t>(outgoing_pointcloud[i].occluded);

    if (shallPointBePublished(outgoing_pointcloud[i])) {
      ++out_range_it;
      ++out_power_it;
      ++out_doppler_it;
      ++out_x_it;
      ++out_y_it;
      ++out_z_it;
      ++out_delta_velocity_it;
      ++out_motion_status_it;
      ++out_available_for_tracker_it;
      ++out_cluster_idx_it;
      ++out_annotation_cluster_idx_it;
      ++out_annotation_class_it;
      ++out_distance_to_ground_plane;
      ++out_multi_path_it;
      ++out_radar_cross_section_it;
      ++out_rgba_it;
      nof_out_points++;
      ++out_ghost_track_it;
      ++out_track_log_likelihood_it;
      ++out_is_local_max_it;
      ++out_occluded_it;
    }
  }
  ext_pc_modifier_.resize(nof_out_points);
  ext_pc_to_publish_.width = static_cast<uint32_t>(nof_out_points);
  ext_pc_to_publish_.height = 1;
  extended_pointcloud_publisher_->publish(ext_pc_to_publish_);
}

// Publish statistics about the radar perception
void RadarPublisher::publishStatistics(
    const common::TrackedObjects &tracked_objects,
    std::optional<Eigen::Vector3f> velocity_xyz,
    const std_msgs::msg::Header &header) const {
  oden_interfaces::msg::OdenStatistics statistics_msg;

  statistics_msg.header = header;

  statistics_msg.num_tracks = std::count_if(
      tracked_objects.begin(), tracked_objects.end(), [](const auto &track) {
        return track.track_state == common::TrackState::MATURE;
      });

  statistics_msg.ego_speed = 0.0f;
  if (auto vel_xyz = velocity_xyz) {
    statistics_msg.ego_speed = vel_xyz->norm();
  }
  statistics_publisher_->publish(statistics_msg);
}

void RadarPublisher::publishRegionOfInterest(
    const std::optional<common::RegionOfInterest> &roi,
    const std_msgs::msg::Header &header) const {
  // Define a message for region of interest
  oden_interfaces::msg::RegionOfInterest roi_msg;

  roi_msg.header = header;

  if (roi) {
    roi_msg.is_enabled = true;
    std::array<float, 8> data = {roi->x_center, roi->y_center, roi->z_center,
                                 roi->width,    roi->length,   roi->height,
                                 roi->yaw,      roi->pitch};
    roi_msg.data = data;
  } else {
    roi_msg.is_enabled = false;
  }

  roi_publisher_->publish(roi_msg);
}

// Publish the static map
void RadarPublisher::publishStaticMap(
    const bool is_dynamic, const float map_resolution,
    const std::vector<Eigen::Vector3f> &static_map_coordinates,
    const std::vector<Eigen::Vector3f> &static_map_anomalies,
    const std_msgs::msg::Header &header) const {

  oden_interfaces::msg::StaticMap pub_static_map;

  pub_static_map.header = header;

  // Create topic containing static map
  if (!is_dynamic) {
    pub_static_map.voxel_size = map_resolution;

    // Get the static map coordinates
    for (const Eigen::Vector3f &coord : static_map_coordinates) {
      oden_interfaces::msg::Voxel voxel;
      voxel.x_center_position = coord.x();
      voxel.y_center_position = coord.y();
      voxel.z_center_position = coord.z();
      voxel.anomaly = false;
      pub_static_map.grid_centers.push_back(voxel);
    }

    // Add also the anomaly points
    for (const Eigen::Vector3f &coord : static_map_anomalies) {
      oden_interfaces::msg::Voxel voxel;
      voxel.x_center_position = coord.x();
      voxel.y_center_position = coord.y();
      voxel.z_center_position = coord.z();
      voxel.anomaly = true;
      pub_static_map.grid_centers.push_back(voxel);
    }
  }
  static_map_publisher_->publish(pub_static_map);
}

// Publish the occupancy grid
void RadarPublisher::publishOccupancyGrid(
    const bool is_active, const std::vector<Eigen::Vector4f> &occupancy_grid,
    float voxel_size, const std_msgs::msg::Header &header) {

  oden_interfaces::msg::OccupancyGrid pub_og;
  pub_og.header = header;

  // Create topic containing static map

  // Get the static map coordinates
  pub_og.voxel_size = voxel_size;
  if (is_active) {
    for (const Eigen::Vector4f &coord : occupancy_grid) {
      oden_interfaces::msg::OccupancyGridVoxel og_voxel;
      og_voxel.x_center_position = coord.x();
      og_voxel.y_center_position = coord.y();
      og_voxel.z_center_position = coord.z();
      og_voxel.log_odds = coord.w();

      pub_og.list_of_voxels.push_back(og_voxel);
    }
  }
  occupancy_grid_publisher_->publish(pub_og);
}

// Create an image message for the free space volume visualizing the depth map
// to the closest static object
void RadarPublisher::publishFreeSpaceVolume(
    const std::vector<std::vector<float>> &free_space_matrix,
    const std::vector<float> &azimuth_angles,
    const std::vector<float> &elevation_angles,
    const std_msgs::msg::Header &header) const {

  if (free_space_matrix.empty() || azimuth_angles.empty() ||
      elevation_angles.empty()) {
    return;
  }

  // Create image message
  sensor_msgs::msg::Image image_msg;
  image_msg.header = header;
  image_msg.height = elevation_angles.size();
  image_msg.width = azimuth_angles.size();
  image_msg.encoding = "rgb8";
  image_msg.is_bigendian = false;
  image_msg.step = image_msg.width * 3; // 3 bytes per pixel (RGB)

  // Resize data array
  image_msg.data.resize(image_msg.height * image_msg.step);

  // Find min/max distances for normalization
  float min_distance = std::numeric_limits<float>::max();
  float max_distance = 0.0f;

  for (const auto &row : free_space_matrix) {
    for (float distance : row) {
      if (distance > 0.0f) { // Ignore invalid/zero distances
        min_distance = std::min(min_distance, distance);
        max_distance = std::max(max_distance, distance);
      }
    }
  }

  // Avoid division by zero
  if (max_distance <= min_distance) {
    max_distance = min_distance + 1.0f;
  }

  // Convert distances to RGB values
  for (size_t elevation_idx = 0; elevation_idx < elevation_angles.size();
       ++elevation_idx) {
    for (size_t azimuth_idx = 0; azimuth_idx < azimuth_angles.size();
         ++azimuth_idx) {

      // Get distance value
      float distance = 0.0f;
      if (elevation_idx < free_space_matrix.size() &&
          azimuth_idx < free_space_matrix[elevation_idx].size()) {
        distance = free_space_matrix[elevation_idx][azimuth_idx];
      }

      // Calculate pixel position (flip vertically so smallest elevation is at
      // bottom)
      size_t flipped_elevation_idx =
          elevation_angles.size() - 1 - elevation_idx;
      size_t pixel_idx =
          flipped_elevation_idx * image_msg.step + azimuth_idx * 3;

      // Normalize distance to [0, 1]
      float normalized_distance =
          (distance - min_distance) / (max_distance - min_distance);

      // Create depth colormap: close = red/yellow, far = blue/cyan
      uint8_t r, g, b;
      if (normalized_distance < 0.5f) {
        // Close range: red to yellow
        float t = normalized_distance * 2.0f;
        r = 255;
        g = static_cast<uint8_t>(255 * t);
        b = 0;
      } else {
        // Far range: yellow to cyan to blue
        float t = (normalized_distance - 0.5f) * 2.0f;
        r = static_cast<uint8_t>(255 * (1.0f - t));
        g = 255;
        b = static_cast<uint8_t>(255 * t);
      }

      image_msg.data[pixel_idx] = r;
      image_msg.data[pixel_idx + 1] = g;
      image_msg.data[pixel_idx + 2] = b;
    }
  }

  // Create and publish camera info to match the image
  sensor_msgs::msg::CameraInfo camera_info_msg;
  camera_info_msg.header = header;
  camera_info_msg.width = image_msg.width;
  camera_info_msg.height = image_msg.height;

  // Set distortion model to none
  camera_info_msg.distortion_model = "plumb_bob";
  camera_info_msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};

  // Create identity camera matrix - this represents a simple projection
  // for visualization purposes in RViz2
  double focal_length = std::max(camera_info_msg.width, camera_info_msg.height);
  camera_info_msg.k = {focal_length,
                       0.0,
                       static_cast<double>(camera_info_msg.width) / 2.0,
                       0.0,
                       focal_length,
                       static_cast<double>(camera_info_msg.height) / 2.0,
                       0.0,
                       0.0,
                       1.0};

  // Set rectification matrix to identity
  camera_info_msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  // Set projection matrix (same as camera matrix with zero translation)
  camera_info_msg.p = {focal_length,
                       0.0,
                       static_cast<double>(camera_info_msg.width) / 2.0,
                       0.0,
                       0.0,
                       focal_length,
                       static_cast<double>(camera_info_msg.height) / 2.0,
                       0.0,
                       0.0,
                       0.0,
                       1.0,
                       0.0};

  // Publish the image and camera info
  free_space_volume_publisher_->publish(image_msg);
  free_space_volume_camera_info_publisher_->publish(camera_info_msg);
}

// Create and publish raw free-space volume as PointCloud2
void RadarPublisher::publishFreeSpaceVolumeRaw(
    const std::vector<std::vector<float>>
        &free_space_matrix, // [elevation][azimuth] distances [m]
    const std::vector<float> &azimuth_angles,   // rad (auto-deg detection)
    const std::vector<float> &elevation_angles, // rad (auto-deg detection)
    const std_msgs::msg::Header &header) const {

  using sensor_msgs::msg::PointCloud2;
  using sensor_msgs::msg::PointField;

  if (free_space_matrix.empty() || azimuth_angles.empty() ||
      elevation_angles.empty()) {
    return;
  }

  const uint32_t nof_elevation_angles =
      static_cast<uint32_t>(elevation_angles.size());
  const uint32_t nof_azimuth_angles =
      static_cast<uint32_t>(azimuth_angles.size());

  PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1; // unorganized
  cloud.width =
      static_cast<uint32_t>(nof_azimuth_angles * nof_elevation_angles);
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  // Fields: x, y, z, distance
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2Fields(4, "x", 1, PointField::FLOAT32, "y", 1,
                           PointField::FLOAT32, "z", 1, PointField::FLOAT32,
                           "distance", 1, PointField::FLOAT32);
  mod.resize(static_cast<uint32_t>(nof_azimuth_angles * nof_elevation_angles));

  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> it_r(cloud, "distance");

  for (uint32_t ei = 0; ei < nof_elevation_angles; ++ei) {
    const auto elevation_angle = elevation_angles[ei];

    // Precompute sin and cos of elevation angle
    const auto sin_elevation = std::sin(elevation_angle);
    const auto cos_elevation = std::cos(elevation_angle);

    for (uint32_t ai = 0; ai < nof_azimuth_angles; ++ai) {
      const auto azimuth_angle = azimuth_angles[ai];

      const auto distance = free_space_matrix[ei][ai];
      if (!(distance > 0.f && std::isfinite(distance))) {
        continue;
      }

      // Calculate the cartesian coordinates
      *it_x = distance * std::sin(azimuth_angle) * cos_elevation;
      *it_y = distance * std::cos(azimuth_angle) * cos_elevation;
      *it_z = distance * sin_elevation;
      *it_r = distance;

      ++it_x;
      ++it_y;
      ++it_z;
      ++it_r;
    }
  }

  free_space_volume_raw_publisher_->publish(cloud);
}

// Function that checks if a point should be published. Typically we want to
// publish all points, but in some cases we might want to filter out points
// for example at a demo. Then the user can put custom logic here. In the long
// run we could include these variants in the GUI.
bool RadarPublisher::shallPointBePublished(const common::OutputPoint &point) {
  // Example 1: output only dynamic points
  // const bool output = (point.motion_status ==
  // common::MotionStatus::DYNAMIC); Example 2: output only static points
  // const bool output = (point.motion_status ==
  // common::MotionStatus::STATIONARY);

  (void)(point); // Avoid unused parameter warning
  return true;
}

uint32_t RadarPublisher::setColorOfPoint(const common::OutputPoint &point) {
  // Function to set custom color palette
  if (point.cluster_idx >= 0) {
    // Set tracked points to white
    return 0xFF000000 | (Color::WHITE[0] << 16) | (Color::WHITE[1] << 8) |
           Color::WHITE[2];
  }
  const float dist = std::abs(point.distance_to_ground_plane);

  if (dist < MIN_COLOR_INTENSITY) {
    // Cyan color for points close to the ground plane
    return 0xFF000000 | (Color::CYAN[0] << 16) | (Color::CYAN[1] << 8) |
           Color::CYAN[2];
  }
  // Check if above or below ground plance
  const bool is_above_plane = (point.distance_to_ground_plane > 0.0f);

  float ratio = std::clamp(dist / MAX_COLOR_INTENSITY, 0.0f, 1.0f);

  // Choose endpoints based on sign (above/below plane)
  const Eigen::Vector3i &c0 =
      (is_above_plane) ? Color::LIGHT_BLUE : Color::CYAN;
  const Eigen::Vector3i &c1 =
      (is_above_plane) ? Color::CONE_BLUE : Color::DARK_GREEN;

  Eigen::Vector3i rgb = colorInterpolate(c0, c1, ratio);

  return 0xFF000000 | (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
}

Eigen::Vector3i RadarPublisher::colorInterpolate(const Eigen::Vector3i color1,
                                                 const Eigen::Vector3i color2,
                                                 const float ratio) {
  // Function to interpolate between two colors

  // Convert color1 and color2 to floating-point vectors for the interpolation
  Eigen::Vector3f color1f = color1.cast<float>();
  Eigen::Vector3f color2f = color2.cast<float>();

  // Perform the interpolation in floating-point arithmetic
  Eigen::Vector3f colorInterpolated = color1f + (color2f - color1f) * ratio;

  // Threshold the values to be within 0 and 255 (Otherwise infeasible
  // RGB-values)
  colorInterpolated = colorInterpolated.cwiseMax(0.0f);
  colorInterpolated = colorInterpolated.cwiseMin(255.0f);

  // Convert the result back to an integer vector for the return value
  return colorInterpolated.cast<int>();
}

void RadarPublisher::publishFreeSpace(
    const std::vector<Eigen::Vector3f> &free_space_polygon,
    const std_msgs::msg::Header &header,
    const bool ground_plane_is_valid) const {

  oden_interfaces::msg::FreeSpace msg;
  msg.header = header;
  msg.is_valid = !free_space_polygon.empty();
  msg.ground_plane_is_valid = ground_plane_is_valid;

  // Populate ground polygon
  msg.free_space_polygon.reserve(free_space_polygon.size());
  for (const auto &vertex : free_space_polygon) {
    geometry_msgs::msg::Point point;
    point.x = vertex.x();
    point.y = vertex.y();
    point.z = vertex.z();
    msg.free_space_polygon.push_back(point);
  }

  free_space_publisher_->publish(msg);
}
