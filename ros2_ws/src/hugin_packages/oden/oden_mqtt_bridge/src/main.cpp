// Copyright (c) Sensrad 2025

#include <rclcpp/rclcpp.hpp>

#include "OdenMqttBridge.hpp"

int main(int argc, char **argv) {

  // Get a named logger since there's no node logger yet.
  const auto logger = rclcpp::get_logger("OdenMqttBridge");
  RCLCPP_INFO(logger, "Starting OdenMqttBridge...");

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<OdenMqttBridge>());
  rclcpp::shutdown();

  return 0;
}
