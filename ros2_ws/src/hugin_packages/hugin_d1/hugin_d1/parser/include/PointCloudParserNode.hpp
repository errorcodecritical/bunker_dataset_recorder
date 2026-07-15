// Copyright (c) Sensrad 2024-2025

#pragma once

#include <optional>
#include <string>

// ROS2
#include <rclcpp/rclcpp.hpp>

// Custom interfaces
#include <raf2_interfaces/msg/binary_data.hpp>
#include <raf2_interfaces/msg/header.hpp>

// Pointcloud2 headers
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <PointCloud.hpp>

class PointCloudParserNode : public rclcpp::Node {
private:
  // Types
  using PointCloud2Iterator = sensor_msgs::PointCloud2Iterator<float>;

  // Constants
  constexpr static int NUM_POINCLOUD_FIELDS = 9;
  constexpr static int QOS_BACKLOG = 10;
  constexpr static std::size_t INITIAL_POINTCLOUD_SIZE = 65536;
  // constexpr static std::size_t MAX_POINTCLOUD_SIZE = 65536;

  constexpr static std::uint16_t RDR_COMMON_COM_HEADER_TYPE_POINT_CLOUD =
      0x0002;

  std::optional<std::uint16_t> current_frame_counter_;

  // Pointcloud messages and modifiers
  sensor_msgs::msg::PointCloud2 pointcloud_;
  sensor_msgs::PointCloud2Modifier pointcloud_modifier_;

  std::vector<Point> points_; // Accumulated points

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_publisher_;

  rclcpp::Publisher<raf2_interfaces::msg::Header>::SharedPtr header_publisher_;

  // Subscriber of binary data
  rclcpp::Subscription<raf2_interfaces::msg::BinaryData>::SharedPtr
      binary_data_subscription_;

  raf2_interfaces::msg::Header header_msg_;

  std::string frame_id_;

  void publishPointCloud();

  void handlePointcloud(const PointCloud &pc);

  void logHeader(const PointCloud &pc);
  void updateHeader(const PointCloud &pc);

  // Common initialization for all constructors
  void initialize();

  // void accumulatePoints(const PointCloud &pc);

  // Callback for incoming binary data
  void onBinaryData(const raf2_interfaces::msg::BinaryData::SharedPtr msg);

public:
  explicit PointCloudParserNode(const std::string &frame_id = "radar_1");
  explicit PointCloudParserNode(const rclcpp::NodeOptions &options);

  // Delete unused constructors
  PointCloudParserNode(const PointCloudParserNode &) = delete;
  PointCloudParserNode &operator=(const PointCloudParserNode &) = delete;
  PointCloudParserNode(PointCloudParserNode &&) = delete;
  PointCloudParserNode &operator=(PointCloudParserNode &&) = delete;

  // Destructor
  ~PointCloudParserNode() override;
};
