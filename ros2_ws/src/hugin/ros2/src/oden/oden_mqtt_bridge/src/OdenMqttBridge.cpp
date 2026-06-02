// Copyright (c) Sensrad 2025

#include "OdenMqttBridge.hpp"

#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>

OdenMqttBridge::OdenMqttBridge(const std::string &broker_uri,
                               const std::string &client_id)
    : Node("oden_mqtt_bridge") {

  RCLCPP_INFO(get_logger(), "Hello");

  on_shutdown(std::bind(&OdenMqttBridge::onShutdown, this));

  // Parameters
  // Broker URI
  broker_uri_ = declare_parameter("broker_uri", broker_uri);
  // Client ID
  client_id_ = declare_parameter("client_id", client_id);

  // MQTT topics
  meta_data_topic_ = declare_parameter("meta_data_topic", meta_data_topic_);
  object_tracks_topic_ =
      declare_parameter("object_tracks_topic", object_tracks_topic_);
  timer_topic_ = declare_parameter("timer_topic", timer_topic_);

  // Create subscription
  object_tracks_subscription_ =
      create_subscription<oden_interfaces::msg::MultiObjectTracking>(
          "object_tracks", QOS_BACKLOG_,
          std::bind(&OdenMqttBridge::objectTracksCallback, this,
                    std::placeholders::_1));

  // MetaData subscription
  meta_data_subscription_ = create_subscription<raf2_interfaces::msg::MetaData>(
      "meta_data", QOS_BACKLOG_,
      std::bind(&OdenMqttBridge::metaDataCallback, this,
                std::placeholders::_1));

  // Async connect to broker
  mqtt_client_ = std::make_unique<mqtt::async_client>(broker_uri_, client_id_);

  // MQTT connection options
  auto conn_opts = mqtt::connect_options_builder();
  conn_opts.clean_session(true);
  conn_opts.automatic_reconnect(true);

  mqtt_client_->connect(conn_opts.finalize());

  // Create "alive" timer
  using namespace std::chrono_literals;
  timer_ =
      create_wall_timer(1s, std::bind(&OdenMqttBridge::timerCallback, this));

  // Log startup configuration
  RCLCPP_INFO(get_logger(), "broker_uri: %s", broker_uri_.c_str());
}

void OdenMqttBridge::onShutdown() {
  RCLCPP_INFO(get_logger(), "Goodbye");

  mqtt_client_->disconnect();
}

void OdenMqttBridge::timerCallback() {
  using json = nlohmann::json;

  // I'm alive test message with ROS2 time timestamp
  json j = {
      {"message", "I'm alive"},
      {"timestamp", now().nanoseconds()},
  };

  // Dump to string
  const auto json_msg = j.dump();
  // Log
  RCLCPP_INFO(get_logger(), "message: %s", json_msg.c_str());
  // Publish via MQTT
  mqtt::message_ptr pubmsg = mqtt::make_message(timer_topic_, json_msg);
  pubmsg->set_qos(MQTT_QOS);
  mqtt_client_->publish(pubmsg);
}

void OdenMqttBridge::metaDataCallback(
    const raf2_interfaces::msg::MetaData ::ConstSharedPtr meta_data_msg) {

  RCLCPP_INFO(get_logger(), "meta_data_msg");

  using json = nlohmann::json;

  const auto nanosec = rclcpp::Time(meta_data_msg->header.stamp).nanoseconds();

  json j = {
      {"header",
       {{"frame_id", meta_data_msg->header.frame_id},
        {"stamp_nanosec", nanosec}}},
      {"raf_version", meta_data_msg->raf_version},
      {"radar_device_module_part_name_model_number",
       meta_data_msg->radar_device_module_part_name_model_number},
  };

  // Dump to string
  const auto json_msg = j.dump();
  // Log
  RCLCPP_INFO(get_logger(), "message: %s", json_msg.c_str());
  // Publish via MQTT
  mqtt::message_ptr pubmsg = mqtt::make_message(meta_data_topic_, json_msg);
  pubmsg->set_qos(MQTT_QOS);
  mqtt_client_->publish(pubmsg);
};

void OdenMqttBridge::objectTracksCallback(
    const oden_interfaces::msg::MultiObjectTracking::ConstSharedPtr
        object_tracks_msg) {
  RCLCPP_INFO(get_logger(), "object_tracks_msg");

  using json = nlohmann::json;

  // Create an empty json list
  json list = json::array();

  // Iterate the object tracks
  for (auto object_track : object_tracks_msg->object_track_list) {
    json j = {
        {"track_id", object_track.track_id},
        {"track_state", object_track.track_state},
        {"yaw", object_track.yaw},
        {"type", object_track.type},
        {"position",
         {object_track.position[0], object_track.position[1],
          object_track.position[2]}},
        {"velocity",
         {object_track.velocity[0], object_track.velocity[1],
          object_track.velocity[2]}},
        {"extent",
         {object_track.extent[0], object_track.extent[1],
          object_track.extent[2], object_track.extent[3],
          object_track.extent[4], object_track.extent[5],
          object_track.extent[6], object_track.extent[7],
          object_track.extent[8]}},
    };

    // Add to the list
    list.push_back(j);
  }

  const auto nanosec =
      rclcpp::Time(object_tracks_msg->header.stamp).nanoseconds();

  json j = {
      {"header",
       {{"frame_id", object_tracks_msg->header.frame_id},
        {"stamp_nanosec", nanosec}}},
      {"tracks", list},
  };

  // Dump to string
  const auto json_msg = j.dump();
  // Log
  RCLCPP_INFO(get_logger(), "message: %s", json_msg.c_str());
  // Publish via MQTT
  mqtt::message_ptr pubmsg = mqtt::make_message(object_tracks_topic_, json_msg);
  pubmsg->set_qos(MQTT_QOS);
  mqtt_client_->publish(pubmsg);
}
