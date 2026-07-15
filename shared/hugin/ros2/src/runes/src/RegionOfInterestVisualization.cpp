// Copyright (c) Sensrad 2025

#include "RegionOfInterestVisualization.hpp"

namespace perception_visualizer {

visualization_msgs::msg::MarkerArray
RegionOfInterestVisualization::getROIMarker(
    const oden_interfaces::msg::RegionOfInterest::ConstSharedPtr &roi_data) {

  // Create a MarkerArray to hold the ROI markers
  visualization_msgs::msg::MarkerArray roi_marker_array;

  // If the ROI is not enabled, return an empty marker array
  if (!roi_data->is_enabled) {
    return roi_marker_array;
  }

  // We expect at least 8 floats in roi_data->data
  // (x_center, y_center, z_center, width, length, height, yaw, pitch).
  if (roi_data->data.size() < 8) {
    RCLCPP_WARN(
        rclcpp::get_logger("RegionOfInterestVisualization"),
        "ROI message has insufficient data. Expected at least 8 floats.");
    return roi_marker_array;
  }

  // Unpack the ROI data
  float x_center = roi_data->data[0];
  float y_center = roi_data->data[1];
  float z_center = roi_data->data[2];
  float width = roi_data->data[3];
  float length = roi_data->data[4];
  float height = roi_data->data[5];
  float yaw = roi_data->data[6];
  float pitch = roi_data->data[7];

  // Construct a single Marker for the bounding box
  visualization_msgs::msg::Marker box_marker;

  // Extract the header from the ROI data
  box_marker.header = roi_data->header;

  // Set the frame ID and timestamp
  box_marker.ns = "roi_visualization";
  box_marker.id = ROI_ID;
  box_marker.type = visualization_msgs::msg::Marker::CUBE;
  box_marker.action = visualization_msgs::msg::Marker::ADD;

  // Position at the center of the ROI
  box_marker.pose.position.x = x_center;
  box_marker.pose.position.y = y_center;
  box_marker.pose.position.z = z_center;

  // Orientation: set roll=0, use pitch & yaw
  tf2::Quaternion quat;
  quat.setRPY(pitch, 0.0, yaw); // roll=0
  // Convert to geometry_msgs::msg::Quaternion
  geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(quat);
  box_marker.pose.orientation = q_msg;

  // Scale to full dimensions
  box_marker.scale.x = width;
  box_marker.scale.y = length;
  box_marker.scale.z = height;

  // Set color and opacity
  box_marker.color.r = ROI_COLOR_R;
  box_marker.color.g = ROI_COLOR_G;
  box_marker.color.b = ROI_COLOR_B;
  box_marker.color.a = ROI_OPACITY;

  // Set lifetime of the marker
  box_marker.lifetime = rclcpp::Duration::from_seconds(FADE_DURATION);

  // Add it to the MarkerArray
  roi_marker_array.markers.push_back(box_marker);

  return roi_marker_array;
}

} // namespace perception_visualizer
