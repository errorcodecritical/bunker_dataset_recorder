// Copyright 2016-2025 Aeva, Inc. All rights reserved.
// 
// Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
// code for terms and conditions that govern your use of this software. Any use, reproduction,
// disclosure, or distribution of this software and related documentation outside the terms of the
// EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

// Program to set and get the network settings of a sensor by changing the SOME/IP configurations

#include <unistd.h>
#include <iostream>
#include <string>

#include "aeva/api/AevaAPI.h"

using namespace aeva;

int main(int argc, char** argv) {
  // Requires the user to input the sensor url to connect to and the new SOME/IP configurations.
  if (argc == 4) {
    std::cerr << "Not enough arguments." << std::endl;
    std::cerr << "  Example: ./change_someip_settings [sensor ip] [service id] [instance id] "
              << std::endl;
    return EXIT_FAILURE;
  }

  // Assign sensor info.
  api::AevaAPI api;
  api::Sensor sensor;
  sensor.id_ = "sensor";
  sensor.url_ = argv[1];
  sensor.protocol_ = api::Sensor::Protocol::TCP;
  sensor.platform_.type_ = api::Sensor::Platform::Type::ATLAS;

  // Get the current network settings.
  api::NetworkSettings initial_network_settings = api.GetSensorNetworkSettings(sensor);
  if (!initial_network_settings.valid_) {
    std::cerr << "Failed to get the current network settings of the sensor" << std::endl;
    return EXIT_FAILURE;
  }

  // Create the new network settings and update the sensor url.
  api::NetworkSettings new_network_settings = initial_network_settings;
  try {
    new_network_settings.someip_service_id_ = std::stoi(argv[2]);
  } catch (...) {
    new_network_settings.someip_service_id_ = 0;
  }
  try {
    new_network_settings.someip_instance_id_ = std::stoi(argv[3]);
  } catch (...) {
    new_network_settings.someip_instance_id_ = 0;
  }

  // Validate the new network settings.
  std::string valid_network_settings_result = api.ValidateNetworkSettings(new_network_settings);
  if (!valid_network_settings_result.empty()) {
    std::cerr << "New network settings were invalid: " << valid_network_settings_result
              << std::endl;
    return EXIT_FAILURE;
  }

  // Update the sensor network settings.
  bool set_network_settings = api.SetNetworkSettings(sensor, new_network_settings);
  if (!set_network_settings) {
    std::cerr << "Failed to set the new network settings. Sensor did not receive the instructions "
                 "to update network settings."
              << std::endl;
    return EXIT_FAILURE;
  }

  // wait for the sensor reboot
  sleep(45);

  // Confirm the new ip is as expected.
  new_network_settings = api.GetSensorNetworkSettings(sensor);
  if (!new_network_settings.valid_) {
    std::cerr << "Failed to get the new network settings" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Successfully changed the someip setting." << std::endl;
  std::cout << "someip_service_id = " << new_network_settings.someip_service_id_ << std::endl;
  std::cout << "someip_instance_id = " << new_network_settings.someip_instance_id_ << std::endl;
  return EXIT_SUCCESS;
}
