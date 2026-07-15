// Copyright (c) Sensrad 2024-2025

#include <rclcpp/rclcpp.hpp>

#include "ControlNode.hpp"

int main(int argc, char **argv) {
  const auto logger = rclcpp::get_logger("hugin_d1_control");

  RCLCPP_INFO(logger, "Starting Hugin D1 control node...");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();

  return 0;
}
