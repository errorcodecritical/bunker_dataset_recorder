// Copyright 2016-2025 Aeva, Inc. All rights reserved.
// 
// Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
// code for terms and conditions that govern your use of this software. Any use, reproduction,
// disclosure, or distribution of this software and related documentation outside the terms of the
// EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

// This program demonstrates how to use ROS build of the Aeva API to publish ROS messages from the
// sensor. Unlike the callbacks with the standard Aeva API, this version of the API directly
// provides the converted ROS types.

#include <unistd.h>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

#include <ros/ros.h>

#include "aeva/api/AevaAPI.h"

#include "AevaAPICallbacks.h"
#include "AevaPublisher.h"

using namespace aeva;
using namespace aeva::api::ros_examples;

namespace {

// Time interval to poll the events (set to 2x the highest frequency stream from the API).
// Point clouds are published at 10 Hz and IMU is published 100 Hz (highest frequency stream).
constexpr int64_t kPollIntervalMilliseconds{5};
constexpr int64_t kReconfigPollIntervalMilliseconds{10000};

std::string CodeToString(api::ConfigReturn code) {
  if (code == api::ConfigReturn::OK) {
    return "OK";
  } else if (code == api::ConfigReturn::FAIL) {
    return "FAIL";
  } else if (code == api::ConfigReturn::TIMEOUT) {
    return "TIMEOUT";
  } else if (code == api::ConfigReturn::INVALID_SENSOR) {
    return "INVALID_SENSOR";
  } else {
    return "UNKNOWN";
  }
}

template <typename ConfigValueT>
void SetConfig(api::AevaAPI& api, const std::string& sensor_id, const std::string& node,
               const std::string& key, const ConfigValueT& value) {
  auto code = api.ConfigureValue(sensor_id, node, key, value);
  if (code != api::ConfigReturn::OK) {
    ROS_FATAL_STREAM("Failed to set config: " << node << "." << key << " = " << value
                                              << " [status: " << CodeToString(code) << "]");
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    ROS_FATAL_STREAM(
        "Incorrect arguments. Please provide the sensor information in the given "
        "sequence: SENSOR_IP_0 SENSOR_NAME_0 [... SENSOR_IP_N SENSOR_NAME_N]");
    return EXIT_FAILURE;
  }

  // Using a unique node name so multiple instances can be run.
  std::srand(std::time(0));
  std::ostringstream node_name;
  node_name << "aeva_publisher_" << std::to_string(std::rand() % 256);
  ROS_INFO_STREAM("Node name: " << node_name.str());

  // Initialize the node.
  ros::init(argc, argv, node_name.str());
  ros::NodeHandle nh("~");

  // Parse node parameters.
  // Set _KEY:=VALUE on command line.

  // Aeva raw data messages are required to open the bag file using Aeviz.
  bool publish_raw_topics{true};
  nh.param("publish_raw_topics", publish_raw_topics, true);
  ROS_INFO_STREAM("Parsed publish_raw_topics: " << std::boolalpha << publish_raw_topics);

  // Enable publishing output in native ROS formats.
  bool publish_ros_topics{true};
  nh.param("publish_ros_topics", publish_ros_topics, true);
  ROS_INFO_STREAM("Parsed publish_ros_topics: " << std::boolalpha << publish_ros_topics);

  // Passthrough the raw sensor data from the sensor to the callbacks, bypass all API algorithms.
  // This may be used to reduce CPU usage and process the raw data later.
  bool bypass_api{false};
  nh.param("bypass_api", bypass_api, false);
  ROS_INFO_STREAM("Parsed bypass_api: " << std::boolalpha << bypass_api);

  // This is used to offset the sensor indices; it's needed for running and recording messages from
  // multiple instances of this app.
  int sensor_index_offset{0};
  nh.param("sensor_index_offset", sensor_index_offset, 0);
  ROS_INFO_STREAM("Parsed sensor_index_offset: " << sensor_index_offset);

  // The sensor ID is used to update the topic and frame ID namespaces, which is useful when
  // multiple instances of the ROS publisher node are instantiated.
  std::vector<std::string> sensor_ids;
  std::vector<std::string> sensor_urls;
  for (int i = 1; i < argc; i += 2) {
    sensor_urls.push_back(std::string(argv[i]));

    // Sensor IDs must conform to ROS naming scheme where underscore is the sole
    // special character allowed
    std::string sensor_id{std::string(argv[i + 1])};
    for (char& ch : sensor_id) {
      if (!std::isalnum(ch)) {
        ch = '_';
      }
    }
    sensor_ids.push_back(sensor_id);
  }
  ROS_INFO_STREAM("Parsed " << sensor_ids.size() << " sensor(s) information");

  // Instantiate the ROS publishers.
  std::vector<std::shared_ptr<AevaPublisher>> publishers;
  std::size_t sensor_index = sensor_index_offset;
  for (const auto& sensor_id : sensor_ids) {
    publishers.emplace_back(
        std::make_shared<AevaPublisher>(sensor_id, sensor_index++, sensor_index_offset, nh,
                                        publish_raw_topics, publish_ros_topics));
  }

  api::AevaAPI api;

  // Connect to the Atlas sensors.
  std::vector<api::Sensor> sensors;
  for (std::size_t i = 0; i < sensor_ids.size(); ++i) {
    api::Sensor sensor;
    sensor.id_ = sensor_ids[i];
    sensor.url_ = sensor_urls[i];
    sensor.platform_.type_ = api::Sensor::Platform::Type::ATLAS;
    sensor.platform_.configuration_ = api::Sensor::Platform::Configuration::STANDARD;
    sensor.protocol_ = api::Sensor::Protocol::TCP;
    auto processing_mode =
        bypass_api ? api::ProcessingMode::PASSTHROUGH : api::ProcessingMode::DEFAULT;
    ROS_INFO_STREAM("Connecting to " << sensor_ids[i] << " at " << sensor_urls[i]);
    if (!api.Connect(sensor, processing_mode)) {
      ROS_FATAL_STREAM("Failed to connect to sensor: " << sensor_ids[i] << " at "
                                                       << sensor_urls[i]);
      return EXIT_FAILURE;
    }
    sensors.push_back(sensor);
  }

  // Bind callbacks with the publishers.
  auto calibration_callback =
      std::bind(CalibrationCallback, std::placeholders::_1, std::ref(publishers));
  auto config_callback = std::bind(ConfigCallback, std::placeholders::_1, std::ref(publishers));
  auto cpc_pointgroup_callback =
      std::bind(CPCPointGroupCallback, std::placeholders::_1, std::ref(publishers));
  auto health_callback = std::bind(HealthCallback, std::placeholders::_1, std::ref(publishers));
  auto imu_callback = std::bind(IMUCallback, std::placeholders::_1, std::ref(publishers));
  auto log_callback = std::bind(LogCallback, std::placeholders::_1, std::ref(publishers));
  auto metadata_callback = std::bind(MetadataCallback, std::placeholders::_1, std::ref(publishers));
  auto object_list_callback =
      std::bind(ObjectListCallback, std::placeholders::_1, std::ref(publishers), false);
  auto odometry_callback = std::bind(OdometryCallback, std::placeholders::_1, std::ref(publishers));
  auto point_cloud_callback =
      std::bind(PointCloudCallback, std::placeholders::_1, std::ref(publishers));
  auto reconfig_callback = std::bind(ReconfigCallback, std::placeholders::_1, std::ref(publishers));
  auto raw_packet_callback =
      std::bind(RawPacketCallback, std::placeholders::_1, std::ref(publishers));
  auto telemetry_callback =
      std::bind(TelemetryCallback, std::placeholders::_1, std::ref(publishers));

  // Register callbacks.
  api.RegisterCalibrationMessageCallback(calibration_callback);
  api.RegisterHealthMessageCallback(health_callback);
  api.RegisterIMUMessageCallback(imu_callback);
  api.RegisterObjectListMessageCallback(object_list_callback);
  api.RegisterOdometryMessageCallback(api::kOdometrySensorDataStreamId, odometry_callback);
  api.RegisterPointCloudMessageCallback(api::kPointCloudDataStreamId, point_cloud_callback);

  if (publish_raw_topics) {
    api.RegisterConfigMessageCallback(config_callback);
    api.RegisterLogMessageCallback(log_callback);
    api.RegisterMetadataMessageCallback(metadata_callback);
    api.RegisterNodeConfigurationMessageCallback(reconfig_callback);
    api.RegisterRawCartesianPointGroupMessageCallback(cpc_pointgroup_callback);
    api.RegisterRawPacketMessageCallback(raw_packet_callback);
    api.RegisterTelemetryMessageCallback(telemetry_callback);
  }

  // Subscribe and poll sensors.
  for (const auto& sensor : sensors) {
    api.Subscribe(sensor.id_, api::kCalibrationDataStreamId);
    api.Subscribe(sensor.id_, api::kHealthDataStreamId);
    api.Subscribe(sensor.id_, api::kIMUDataStreamId);
    api.Subscribe(sensor.id_, api::kObjectListDataStreamId);
    api.Subscribe(sensor.id_, api::kOdometrySensorDataStreamId);
    api.Subscribe(sensor.id_, api::kPointCloudDataStreamId);

    if (publish_raw_topics) {
      api.Subscribe(sensor.id_, api::kConfigDataStreamId);
      api.Subscribe(sensor.id_, api::kLogDataStreamId);
      api.Subscribe(sensor.id_, api::kRawCpcPointGroupDataStreamId);
      api.Subscribe(sensor.id_, api::kRawPacketDataStreamId);
      api.Subscribe(sensor.id_, api::kReconfigDataStreamId);
      api.Subscribe(sensor.id_, api::kTelemetryDataStreamId);
    }
  }

  std::size_t reconfig_poll_counter{0};
  while (ros::ok()) {
    api.PollEvents();
    usleep(kPollIntervalMilliseconds * 1000);

    // Periodically poll the reconfig manifest. Requesting a reconfig manifest will cause all nodes
    // of the connected sensors to send a message containing the state of their reconfigs. These
    // messages can be recorded and used during playback.
    if (reconfig_poll_counter++ > kReconfigPollIntervalMilliseconds / kPollIntervalMilliseconds) {
      api.RequestReconfigManifest();
      reconfig_poll_counter = 0;
    }
  }

  // Unsubscribe and disconnect.
  api.UnsubscribeAll(api::kCalibrationDataStreamId);
  api.UnsubscribeAll(api::kConfigDataStreamId);
  api.UnsubscribeAll(api::kHealthDataStreamId);
  api.UnsubscribeAll(api::kIMUDataStreamId);
  api.UnsubscribeAll(api::kLogDataStreamId);
  api.UnsubscribeAll(api::kObjectListDataStreamId);
  api.UnsubscribeAll(api::kOdometrySensorDataStreamId);
  api.UnsubscribeAll(api::kPointCloudDataStreamId);
  api.UnsubscribeAll(api::kRawCpcPointGroupDataStreamId);
  api.UnsubscribeAll(api::kRawPacketDataStreamId);
  api.UnsubscribeAll(api::kReconfigDataStreamId);
  api.UnsubscribeAll(api::kSensorFusionObjectListDataStreamId);
  api.UnsubscribeAll(api::kTelemetryDataStreamId);
  api.DisconnectAll();

  return EXIT_SUCCESS;
}
