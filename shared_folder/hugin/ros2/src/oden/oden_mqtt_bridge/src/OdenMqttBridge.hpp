// Copyright (c) Sensrad 2025

#pragma once

// STL
#include <memory>

// RCLCPP
#include <rclcpp/rclcpp.hpp>

// MQTT
#include <mqtt/async_client.h>

// Oden messages
#include "raf2_interfaces/msg/meta_data.hpp"
#include <oden_interfaces/msg/multi_object_tracking.hpp>
#include <raf2_interfaces/msg/meta_data.hpp>

class OdenMqttBridge final : public rclcpp::Node {

  static constexpr int QOS_BACKLOG_ = 10;
  static constexpr int MQTT_QOS = 0;

  std::string broker_uri_;
  std::string client_id_;

  // MQTT topics
  std::string timer_topic_ = "oden_mqtt_bridge/timer";
  std::string meta_data_topic_ = "oden_mqtt_bridge/meta_data";
  std::string object_tracks_topic_ = "oden_mqtt_bridge/object_tracks";

  // MQTT client unique ptr
  std::unique_ptr<mqtt::async_client> mqtt_client_;

  // Add a ROS2 timer to the node for keep alive messages
  rclcpp::TimerBase::SharedPtr timer_;

  // Subscription to multi object tracking messages
  rclcpp::Subscription<oden_interfaces::msg::MultiObjectTracking>::SharedPtr
      object_tracks_subscription_;

  rclcpp::Subscription<raf2_interfaces::msg::MetaData>::SharedPtr
      meta_data_subscription_;

  void metaDataCallback(
      const raf2_interfaces::msg::MetaData ::ConstSharedPtr meta_data_msg);

  // On object tracks callback
  void objectTracksCallback(
      const oden_interfaces::msg::MultiObjectTracking::ConstSharedPtr
          object_tracks_msg);

  // Timer callback
  void timerCallback();

  // Delete copy constructor
  OdenMqttBridge(const OdenMqttBridge &) = delete;
  // Delete copy assignment
  OdenMqttBridge &operator=(const OdenMqttBridge &) = delete;
  // Delete move constructor
  OdenMqttBridge(OdenMqttBridge &&) = delete;
  // Delete move assignment
  OdenMqttBridge &operator=(OdenMqttBridge &&) = delete;

  void onShutdown();

public:
  explicit OdenMqttBridge(
      const std::string &broker_uri = "tcp://127.0.0.1:1883",
      const std::string &client_id = "oden_mqtt_bridge");

  ~OdenMqttBridge() override = default;
};
