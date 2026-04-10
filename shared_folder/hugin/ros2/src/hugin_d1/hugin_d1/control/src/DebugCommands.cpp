// Copyright (c) Sensrad 2025

#include <DebugCommands.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_prefix.hpp>

#include <boost/process.hpp>
#include <boost/process/system.hpp>

#include <nlohmann/json.hpp>

static void from_json(const nlohmann::json &j, ProductionData &d) {
  j.at("production_date").get_to(d.production_date);
  j.at("radar_device_system_id").get_to(d.radar_device_system_id);
  j.at("part_number").get_to(d.part_number);
  j.at("radar_device_module_part_name_model_number")
      .get_to(d.radar_device_module_part_name_model_number);
  j.at("eco_rev").get_to(d.eco_rev);
  j.at("analog_board_part_number").get_to(d.analog_board_part_number);
  j.at("analog_board_serial_number").get_to(d.analog_board_serial_number);
  j.at("digital_board_part_number").get_to(d.digital_board_part_number);
  j.at("digital_board_serial_number").get_to(d.digital_board_serial_number);
  j.at("rfic_rx_version").get_to(d.rfic_rx_version);
  j.at("rfic_tx_version").get_to(d.rfic_tx_version);
  j.at("spare1").get_to(d.spare1);
  j.at("spare2").get_to(d.spare2);
  j.at("spare3").get_to(d.spare3);
}

static std::string read_all(boost::process::ipstream &stdout_pipe) {
  // Read all from stdout_pipe
  std::ostringstream oss;
  std::string line;
  while (std::getline(stdout_pipe, line)) {
    oss << line << '\n';
  }

  return oss.str();
}

ProductionData DebugCommands::readProductionData(const std::string &ip,
                                                 const uint16_t port) {

  const auto logger = rclcpp::get_logger("control_node.DebugCommands");

  ProductionData production_data{};

  boost::process::ipstream stdout_pipe;
  boost::process::ipstream stderr_pipe;

  const std::string bin = ament_index_cpp::get_package_prefix("hugin_d1") +
                          "/lib/hugin_d1/sensrad-flash";

  RCLCPP_INFO(logger, "Running command: %s --ip %s --port %d production",
              bin.c_str(), ip.c_str(), port);

  // Launch the process and capture stdout/stderr
  boost::process::child child(bin, "--ip", ip, "--port", std::to_string(port),
                              "production",
                              boost::process::std_out > stdout_pipe,
                              boost::process::std_err > stderr_pipe);

  // Read all from stdout_pipe
  std::string json_blob = read_all(stdout_pipe);

  // Wait for the child process to finish
  child.wait();

  // Check if there was any error output
  const int result = child.exit_code();

  if (result != 0) {
    RCLCPP_WARN(logger, "Failed to read production data from radar at %s:%d",
                ip.c_str(), port);

    // Return empty production data on error
    return production_data;
  }

  nlohmann::json j = nlohmann::json::parse(json_blob);
  from_json(j, production_data);

  // Log success
  RCLCPP_INFO(logger, "Successfully read production data from radar at %s:%d",
              ip.c_str(), port);

  return production_data;
}
