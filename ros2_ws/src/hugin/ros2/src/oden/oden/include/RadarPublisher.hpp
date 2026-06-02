// Copyright (c) Sensrad 2025
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "AnnotationPoint.hpp"
#include <liboden/IO_Types.hpp>

// Custom messages
#include <oden_interfaces/msg/detections.hpp>
#include <oden_interfaces/msg/ego_motion.hpp>
#include <oden_interfaces/msg/free_space.hpp>
#include <oden_interfaces/msg/ground_plane.hpp>
#include <oden_interfaces/msg/multi_object_tracking.hpp>
#include <oden_interfaces/msg/occupancy_grid.hpp>
#include <oden_interfaces/msg/occupancy_grid_voxel.hpp>
#include <oden_interfaces/msg/oden_control_state.hpp>
#include <oden_interfaces/msg/oden_statistics.hpp>
#include <oden_interfaces/msg/region_of_interest.hpp>
#include <oden_interfaces/msg/static_map.hpp>

// ROS2 message types
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace Color {
// RGB-values for different colors
static const Eigen::Vector3i WHITE(255, 255, 255);
static const Eigen::Vector3i CYAN(0, 255, 255);
static const Eigen::Vector3i LIGHT_BLUE(0, 230, 255);
static const Eigen::Vector3i CONE_BLUE(0, 100, 250);
static const Eigen::Vector3i DARK_GREEN(0, 150, 50);
} // namespace Color

// Class for publishing processed radar data as ROS2 messages.
class RadarPublisher {
public:
  explicit RadarPublisher(rclcpp::Node *node);
  void publishDetections(const common::Detections &detections,
                         const std_msgs::msg::Header &header) const;

  void
  publishEgoMotion(const std::optional<Eigen::Vector3f> &velocity_xyz,
                   const bool velocity_is_valid,
                   const std::optional<Eigen::Vector3f> &rotational_rate_xyz,
                   const bool odometry_is_active,
                   const Eigen::Vector3f &pose_translation,
                   const Eigen::Quaternionf &pose_quaternion,
                   const Eigen::Vector3f &delta_translation,
                   const Eigen::Quaternionf &delta_rotation_quaternion,
                   const std_msgs::msg::Header &header) const;

  void publishTrackedObjects(const common::TrackedObjects &tracked_objects,
                             const std_msgs::msg::Header &header) const;

  void publishGroundplane(const bool is_valid,
                          const Eigen::Vector4f &plane_coef,
                          const std_msgs::msg::Header &header) const;

  void publishExtendedPointcloud(
      const common::OutgoingPointCloud &outgoing_pointcloud,
      const AnnotationPointCloud &annotated_pointcloud,
      const std_msgs::msg::Header &header);

  void publishStatistics(const common::TrackedObjects &tracked_objects,
                         std::optional<Eigen::Vector3f> velocity_xyz,
                         const std_msgs::msg::Header &header) const;

  void
  publishRegionOfInterest(const std::optional<common::RegionOfInterest> &roi,
                          const std_msgs::msg::Header &header) const;

  void
  publishStaticMap(const bool is_dynamic, const float map_resolution,
                   const std::vector<Eigen::Vector3f> &static_map_coordinates,
                   const std::vector<Eigen::Vector3f> &static_map_anomalies,
                   const std_msgs::msg::Header &header) const;

  void publishOccupancyGrid(const bool is_active,
                            const std::vector<Eigen::Vector4f> &occupancy_grid,
                            float voxel_size,
                            const std_msgs::msg::Header &header);
  void publishFreeSpaceVolume(
      const std::vector<std::vector<float>> &free_space_matrix,
      const std::vector<float> &azimuth_angles,
      const std::vector<float> &elevation_angles,
      const std_msgs::msg::Header &header) const;

  void publishFreeSpaceVolumeRaw(
      const std::vector<std::vector<float>> &free_space_matrix,
      const std::vector<float> &azimuth_angles,
      const std::vector<float> &elevation_angles,
      const std_msgs::msg::Header &header) const;
  void publishFreeSpace(const std::vector<Eigen::Vector3f> &ground_polygon,
                        const std_msgs::msg::Header &header,
                        const bool ground_plane_is_valid) const;

  void publishOdometryLocalMap(const std::vector<Eigen::Vector3f> &local_map,
                               const std_msgs::msg::Header &header);

private:
  static constexpr int QOS_BACKLOG = 10;

  // Number of ground plane coefficients
  static constexpr int NOF_PLANE_COEF = 4;

  // Indices for azimuth and range in free space matrix
  static constexpr int AZIMUTH_IDX = 0;
  static constexpr int RANGE_IDX = 1;

  // Margin [dB] for local-max labelling
  static constexpr float LOCAL_MAX_MARGIN = 0.1f;

  // Declare objects for publishing an extended point-cloud
  sensor_msgs::msg::PointCloud2 ext_pc_to_publish_;
  sensor_msgs::PointCloud2Modifier ext_pc_modifier_;

  // Declare object for publishing local map point cloud
  sensor_msgs::msg::PointCloud2 local_map_pc_;
  sensor_msgs::PointCloud2Modifier local_map_modifier_{local_map_pc_};

  // Publishers
  rclcpp::Publisher<oden_interfaces::msg::Detections>::SharedPtr
      detections_publisher_;
  rclcpp::Publisher<oden_interfaces::msg::EgoMotion>::SharedPtr
      odometry_publisher_;
  rclcpp::Publisher<oden_interfaces::msg::MultiObjectTracking>::SharedPtr
      object_tracks_publisher_;
  rclcpp::Publisher<oden_interfaces::msg::GroundPlane>::SharedPtr
      groundplane_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      extended_pointcloud_publisher_;
  rclcpp::Publisher<oden_interfaces::msg::OdenStatistics>::SharedPtr
      statistics_publisher_;
  rclcpp::Publisher<oden_interfaces::msg::RegionOfInterest>::SharedPtr
      roi_publisher_;
  rclcpp::Publisher<oden_interfaces::msg::StaticMap>::SharedPtr
      static_map_publisher_;
  rclcpp::Publisher<oden_interfaces::msg::OccupancyGrid>::SharedPtr
      occupancy_grid_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      free_space_volume_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      free_space_volume_camera_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      free_space_volume_raw_publisher_;
  rclcpp::Publisher<oden_interfaces::msg::FreeSpace>::SharedPtr
      free_space_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      local_map_publisher_;

  // Range of color intensity
  static constexpr float MAX_COLOR_INTENSITY = 6.0f;
  static constexpr float MIN_COLOR_INTENSITY = 0.5f;
  // Threshold for color palette
  static constexpr float COLOR_PALETTE_THRESHOLD = 0.5f;

  // Functions for color coding
  uint32_t setColorOfPoint(const common::OutputPoint &point);
  Eigen::Vector3i colorInterpolate(const Eigen::Vector3i color1,
                                   const Eigen::Vector3i color2,
                                   const float ratio);

  // Custom logic to determine if a point should be published
  bool shallPointBePublished(const common::OutputPoint &point);
};
