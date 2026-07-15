// Copyright (c) Sensrad 2023-2025
#pragma once

#include <cstring>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

#include <boost/format.hpp>

#include <liboden/IO_Types.hpp>
#include <liboden/PerceptionAPI.hpp>

#include "AnnotationPoint.hpp"
#include "RadarPublisher.hpp"

// Custom Messages
#include <oden_interfaces/msg/oden_control_state.hpp>
#include <oden_interfaces/msg/region_of_interest.hpp>
#include <oden_interfaces/msg/static_map.hpp>
#include <raf2_interfaces/msg/header.hpp>

// Services
#include <oden_interfaces/srv/oden_ctrl_reset_perception.hpp>
#include <oden_interfaces/srv/oden_ctrl_set_perception_parameters.hpp>
#include <oden_interfaces/srv/oden_ctrl_set_roi_parameters.hpp>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

// ROS message types
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

constexpr double NANOSEC_SCALE{1.0e9};
constexpr double MILLISEC_SCALE{1.0e3};

class RadarPerception : public rclcpp::Node {
public:
  RadarPerception();

  virtual ~RadarPerception() {
    RCLCPP_INFO(get_logger(), "Radar perception node stopped");
  }
  RadarPerception(const RadarPerception &) = delete;
  RadarPerception &operator=(const RadarPerception &) = delete;
  RadarPerception(RadarPerception &&) noexcept = default;
  RadarPerception &operator=(RadarPerception &&) noexcept = default;

private:
  std::unique_ptr<RadarPublisher> publisher_;
  // Note: QOS_BACKLOG no longer used - using SensorDataQoS instead
  // Kept for backward compatibility with IMU subscription if needed
  static constexpr int QOS_BACKLOG = 10;

  // If pointcloud as larger than this, a reallocation will be done.
  static constexpr std::size_t POINTCLOUD_RESERVE = 32768;
  static constexpr std::size_t OBJECTS_RESERVE = 256;
  static constexpr std::size_t MAX_POINTS_TO_PROCESS = 7500;
  static constexpr std::size_t TRACKED_OBJECTS_RESERVE = 256;
  static constexpr std::size_t STATIC_MAP_RESERVE = 2000;

  // Extended PointCloud, containing original data as well as derived
  // quantities
  common::OutgoingPointCloud outgoing_pointcloud_;
  common::RadarData radar_data_;
  AnnotationPointCloud annotated_pointcloud_;

  // Perception object
  api::PerceptionAPI perception_;

  // Perception outputs
  common::TrackedObjects tracked_objects_;

  // Objects for recording time of incoming radar point-cloud
  int radar_time_sec_;
  uint32_t radar_time_nanosec_;

  // Frame id for incoming data, to be propagated to outgoing data
  std::string frame_id_;

  // Subscribers
  void createSubscriptions();
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_subscription_;

  rclcpp::Subscription<raf2_interfaces::msg::Header>::SharedPtr
      resolution_subscription_;

  // Parameters management
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rcl_interfaces::msg::SetParametersResult
  onParametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  // Publishers
  rclcpp::Publisher<oden_interfaces::msg::OdenControlState>::SharedPtr
      control_state_publisher_;
  oden_interfaces::msg::OdenControlState control_state_msg_;

  // Services for GUI control and statistics.
  rclcpp::Service<oden_interfaces::srv::OdenCtrlResetPerception>::SharedPtr
      reset_perception_service_;
  rclcpp::Service<oden_interfaces::srv::OdenCtrlSetPerceptionParameters>::
      SharedPtr update_parameters_service_;
  rclcpp::Service<oden_interfaces::srv::OdenCtrlSetRoiParameters>::SharedPtr
      update_roi_service_;

  // Call-back functions for incoming data
  void
  radarDataCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud2);

  void resolutionDataCallback(
      const raf2_interfaces::msg::Header::SharedPtr header_msg);

  void createRadarDataFromPointcloud(
      const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud2ct_list);

  void publishControlState();
};
