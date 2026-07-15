// Copyright (c) 2024-2025 Sensrad
#pragma once

#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <vector>

#include <oden_interfaces/msg/multi_object_tracking.hpp>

#include "PerceptionVisualizer.hpp"
#include <liboden/IO_Types.hpp>
namespace perception_visualizer {
class PolygonGenerator {
public:
  PolygonGenerator() = default;
  ~PolygonGenerator() = default;
  PolygonGenerator(const PolygonGenerator &) = delete;
  PolygonGenerator &operator=(const PolygonGenerator &) = delete;
  PolygonGenerator(PolygonGenerator &&) noexcept = default;
  PolygonGenerator &operator=(PolygonGenerator &&) noexcept = default;

  // Populate vertices in a polygon
  static void populateVertices(geometry_msgs::msg::Polygon &polygon,
                               const Eigen::Vector3f &point);

  // Generate AABB object tracks given data message
  static visualization_msgs::msg::MarkerArray getAABBObjectTracks(
      const oden_interfaces::msg::MultiObjectTracking::ConstSharedPtr
          &tracked_objects_data,
      const std::unordered_map<uint32_t, TrackIdManager> &trackid_manager_map,
      const float radar_elevation_angle);

  // Generate free space ground polygon marker
  static visualization_msgs::msg::Marker
  getFreeSpaceGroundPolygonMarker(const std::vector<Eigen::Vector3f> &polygon,
                                  const std_msgs::msg::Header &header,
                                  const bool ground_plane_is_valid);

private:
  constexpr static float DEG_2_RAD =
      M_PIf / 180.0f; // Convert degrees to radians
  // Tolerance for judging if a value is zero
  static constexpr double ZERO_TOLERANCE = 1e-7;

  // If true, assign new track ids only to mature tracks (when visualizing
  // tracks)
  constexpr static bool OUTPUT_COUNT_MATURE_ONLY = true;

  // Scale constant for ground plane polygon
  static constexpr float GROUND_PLANE_SCALE = 5.0f;

  // Coordinate value of point0 for ground plane
  static constexpr float POINT_0_X = 1.0f;
  static constexpr float POINT_0_Y = 4.0f;

  // Coordinate dimension index for a point
  static constexpr int POINT_DIM_X = 0;
  static constexpr int POINT_DIM_Y = 1;
  static constexpr int POINT_DIM_Z = 2;

  // Coefficient index for a plane
  static constexpr int PLANE_COEF_IDX_A = 0;
  static constexpr int PLANE_COEF_IDX_B = 1;
  static constexpr int PLANE_COEF_IDX_C = 2;
  static constexpr int PLANE_COEF_IDX_D = 3;

  // Parameters for AABB object tracks
  static constexpr float FADE_DURATION =
      0.5; // Duration in time for visibility of AABB
  static constexpr float NOF_STD_AABB = 1.5; // Number of standard deviations
                                             // for AABB size
  static constexpr float MIN_SIZE =
      0.1; // Protect against zero size when computing square root

  static constexpr double CLASS_OPACITY = 0.7;

  struct Color {
    float a; // Alpha (Opacity)
    float r; // Red
    float g; // Green
    float b; // Blue
  };

  // Conversion factor from m/s to km/h
  static constexpr float MS_2_KMH = 3.6f;

  // Free space marker ID
  static constexpr int AREA_MARKER_ID = 238;
  static constexpr int BORDER_LINE_MARKER_ID = 239;

  // Free space marker scale
  static constexpr double AREA_MARKER_SCALE = 1.0;
  static constexpr double BORDER_LINE_MARKER_SCALE = 0.1;

  // Free space marker namespace
  static const std::string AREA_MARKER_NS;
  static const std::string BORDER_LINE_MARKER_NS;

  // Free space marker area color definition
  static const std::unordered_map<std::string, Color> MARKER_COLOR_MAP;

  // Parameters for text marker over Bounding Box
  static constexpr float TEXT_OFFSET = 1.0;
  static constexpr float TEXT_SIZE = 0.8;

  static const std::unordered_map<common::Type, Color> CLASS_COLOR_MAP;

  // Size of extent matrix
  static constexpr int EXTENT_SIZE = 3;

  static std::string typeToString(common::Type type);

  static float
  createAABBMarker(const oden_interfaces::msg::ObjectTrack &tracked_object,
                   const std_msgs::msg::Header &header,
                   visualization_msgs::msg::Marker &aabb_marker_object,
                   const float radar_elevation_angle);

  static void createTextMarker(
      const oden_interfaces::msg::ObjectTrack &tracked_object,
      const std_msgs::msg::Header &header, float aabb_half_hight,
      visualization_msgs::msg::Marker &text_marker_object,
      const std::unordered_map<uint32_t, TrackIdManager> &trackid_manager_map);
};
} // namespace perception_visualizer
