// Copyright 2016-2025 Aeva, Inc. All rights reserved.
// 
// Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
// code for terms and conditions that govern your use of this software. Any use, reproduction,
// disclosure, or distribution of this software and related documentation outside the terms of the
// EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

// Program to set the sensor system time

#include <unistd.h>
#include <iostream>
#include <string>

#include "aeva/api/AevaAPI.h"

int main(int argc, char** argv) {
  // Requires the user to input the sensor url to connect to
  if (argc < 2) {
    std::cerr << "Not enough arguments. Please provide the current url of the sensor" << std::endl;
    return EXIT_FAILURE;
  }

  // Assign sensor info.
  aeva::api::AevaAPI api;
  aeva::api::Sensor sensor;
  sensor.id_ = "sensor";
  sensor.url_ = argv[1];
  sensor.protocol_ = aeva::api::Sensor::Protocol::TCP;
  sensor.platform_.type_ = aeva::api::Sensor::Platform::Type::ATLAS;

  // Get the current network settings.
  aeva::api::NetworkSettings initial_network_settings = api.GetSensorNetworkSettings(sensor);
  if (!initial_network_settings.valid_) {
    std::cerr << "Failed to get the current network settings of the sensor" << std::endl;
    return EXIT_FAILURE;
  }

  if (initial_network_settings.time_protocol_ !=
      aeva::api::NetworkSettings::TimeProtocol::HOST_SYSTEM_TIME) {
    // Change the sensor time protocol to HOST_SYSTEM_TIME
    aeva::api::NetworkSettings new_network_settings = initial_network_settings;
    new_network_settings.time_protocol_ =
        aeva::api::NetworkSettings::TimeProtocol::HOST_SYSTEM_TIME;
    api.SetNetworkSettings(sensor, new_network_settings);

    sleep(60);

    new_network_settings = api.GetSensorNetworkSettings(sensor);
    if (!new_network_settings.valid_) {
      std::cerr << "Failed to get the current network settings of the sensor" << std::endl;
      return EXIT_FAILURE;
    }
    if (new_network_settings.time_protocol_ !=
        aeva::api::NetworkSettings::TimeProtocol::HOST_SYSTEM_TIME) {
      std::cerr << "Failed to set the time protocol to HOST_SYSTEM_TIME" << std::endl;
      return EXIT_FAILURE;
    } else {
      std::cout << "Successfully changed the sensor time protocol to HOST_SYSTEM_TIME" << std::endl;
    }
  }

  // Set the sensor system time
  struct timespec cur_time;
  clock_gettime(CLOCK_REALTIME, &cur_time);
  if (!api.SetSensorSystemTime(sensor, cur_time)) {
    std::cerr << "Failed to update sensor system time" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Successfully updated the sensor time " << std::endl;

  return EXIT_SUCCESS;
}
