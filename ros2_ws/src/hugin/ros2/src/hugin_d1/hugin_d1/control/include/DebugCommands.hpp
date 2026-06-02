// Copyright (c) Sensrad 2025

#pragma once

#include <cstdint>
#include <string>

struct ProductionData {
  std::string production_date;
  std::string radar_device_system_id;
  std::string part_number;
  std::string radar_device_module_part_name_model_number;
  std::string eco_rev;
  std::string analog_board_part_number;
  std::string analog_board_serial_number;
  std::string digital_board_part_number;
  std::string digital_board_serial_number;
  std::string rfic_rx_version;
  std::string rfic_tx_version;
  std::string spare1;
  std::string spare2;
  std::string spare3;
};

class DebugCommands {
public:
  static ProductionData readProductionData(const std::string &ip,
                                           const uint16_t port);
};
