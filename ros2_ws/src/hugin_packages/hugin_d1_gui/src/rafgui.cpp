// Copyright (c) Sensrad 2024-2025

#include <rclcpp/rclcpp.hpp>

#include "application.hpp"

int main(int argc, char **argv) {
  // Init ROS2 and parse arguments
  // args will contain all none-ros2 args.
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  // Log startup message, this required that ROS2 is initialized.
  RCLCPP_INFO(rclcpp::get_logger("rafgui_node"), "Starting RAF GUI node...");

  // Create a char** from std::vector<std::string>, this is required by
  // Gtk::Application::create().
  std::vector<char *> argv2_v;
  argv2_v.reserve(args.size());
  int argc2 = static_cast<int>(args.size());

  // NOLINTBEGIN(cppcoreguidelines-pro-type-const-cast)
  std::transform(
      args.begin(), args.end(), std::back_inserter(argv2_v),
      [](const std::string &arg) { return const_cast<char *>(arg.c_str()); });
  // NOLINTEND(cppcoreguidelines-pro-type-const-cast)
  char **argv2 = argv2_v.data();

  // Create Gtk::Application
  // Allow to create multiple instances of the application.
  auto application = Application::create(argc2, argv2);
  // And run it, this function does not return until the application is
  // closed.
  application->run();

  return 0;
}
