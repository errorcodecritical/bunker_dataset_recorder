// Copyright (c) Sensrad 2024-2025

#include <PointCloudParserNode.hpp>

int main(int argc, char **argv) {
  const auto logger = rclcpp::get_logger("hugin_d1_parser_node");
  RCLCPP_INFO(logger, "Starting Hugin D1 point cloud parser node...");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudParserNode>());
  rclcpp::shutdown();

  return 0;
}
