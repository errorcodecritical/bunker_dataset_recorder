// Copyright (c) Sensrad 2025

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <string>
#include <vector>

class ApplicationFinder {
public:
  ApplicationFinder()
      : node_(std::make_shared<rclcpp::Node>("application_finder")) {};

  std::vector<std::string> present_applications() const {
    // Get the node graph interface
    auto graph = node_->get_node_graph_interface();
    auto pairs = graph->get_node_names_and_namespaces(); // NEW

    // Create a list for storing active application nodes
    std::vector<std::string> active_applications;

    // Search after Sensrad application nodes
    for (const auto &p : pairs) {
      const auto &name = p.first; // node name
      if (name == "rockfall") {
        active_applications.push_back(name);
      }
    }

    return active_applications;
  }

private:
  rclcpp::Node::SharedPtr node_;
};
