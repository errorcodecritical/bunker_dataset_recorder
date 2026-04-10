// Copyright (c) Sensrad 2023-2024
#include <rclcpp/rclcpp.hpp>

#include "RadarPerception.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarPerception>());
  rclcpp::shutdown();

  return 0;
}
