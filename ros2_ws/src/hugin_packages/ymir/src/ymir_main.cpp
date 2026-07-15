// Copyright (c) Sensrad 2025

#include "Ymir.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ymir::Ymir>());
  rclcpp::shutdown();
  return 0;
}
