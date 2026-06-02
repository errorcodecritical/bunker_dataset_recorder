// Copyright (c) Sensrad 2025

#pragma once

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include <oden_interfaces/msg/region_of_interest.hpp>

namespace perception_visualizer {

class RegionOfInterestVisualization {
public:
  RegionOfInterestVisualization() = default;
  ~RegionOfInterestVisualization() = default;
  RegionOfInterestVisualization(const RegionOfInterestVisualization &) = delete;
  RegionOfInterestVisualization &
  operator=(const RegionOfInterestVisualization &) = delete;
  RegionOfInterestVisualization(RegionOfInterestVisualization &&) noexcept =
      default;
  RegionOfInterestVisualization &
  operator=(RegionOfInterestVisualization &&) = default;

  static visualization_msgs::msg::MarkerArray getROIMarker(
      const oden_interfaces::msg::RegionOfInterest::ConstSharedPtr &roi_data);

private:
  static constexpr float ROI_COLOR_R = 1.0f;
  static constexpr float ROI_COLOR_G = 1.0f;
  static constexpr float ROI_COLOR_B = 0.0f;
  static constexpr float ROI_OPACITY = 0.5f;

  static constexpr float FADE_DURATION = 0.5f;

  static constexpr int32_t ROI_ID = 0;
};

} // namespace perception_visualizer
