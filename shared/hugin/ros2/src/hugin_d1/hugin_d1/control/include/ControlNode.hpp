// Copyright (c) Sensrad 2024-2025

#pragma once

// ROS2 includes
#include <rcl/logging.h>

// Sensrad Classes
#include <ArbeReceiver.hpp>
#include <DynamicThresholds.hpp>
#include <Hugin.hpp>
#include <LogClient.hpp>
#include <RTPReceiver.hpp>
#include <StaticThresholds.hpp>
#include <TCPReceiver.hpp>

// STL
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Sensrad messages and services
#include <raf2_interfaces/msg/binary_data.hpp>
#include <raf2_interfaces/msg/control_state.hpp>
#include <raf2_interfaces/msg/meta_data.hpp>

#include <raf2_interfaces/srv/rdr_ctrl_set_active_seq.hpp>
#include <raf2_interfaces/srv/rdr_ctrl_set_thresholds.hpp>
#include <raf2_interfaces/srv/rdr_ctrl_start_tx.hpp>
#include <raf2_interfaces/srv/rdr_ctrl_stop_tx.hpp>
#include <raf2_interfaces/srv/sys_cfg_set_time.hpp>

// ControlNode class; this class is the main class for controlling the radar
// and publishing raw data as well as logging messages
class ControlNode : public rclcpp::Node {
private:
  // Define protocols to use
  // If true use TCP for data streaming
  bool use_tcp_;

  constexpr static int DATA_MSG_RESERVE =
      2 * 12 * 65536; // 12 bytes per point, 65536 points

  constexpr static int QOS_BACKLOG = 10; // Num messages to keep in queue

  // This offset is required to align with Arbe API definitions.
  constexpr static uint64_t TIME_OFFSET_MS = 1000; // milliseconds

  // Startup timer, this is used to move network initialization to a
  // separate callback.
  rclcpp::TimerBase::SharedPtr startup_timer_;

  // Radar mode configurations
  std::unordered_map<uint32_t, std::string> radar_mode_name_map_;

  // The ROS2 frame id
  std::string frame_id_;

  // IP configuration
  std::string ip_;
  std::string bind_addr_;
  std::uint16_t ctrl_port_;
  std::uint16_t log_port_;
  std::uint16_t data_port_;
  std::uint16_t someip_port_;

  bool ptp_is_active_;
  bool auto_start_tx_;

  // Pointcloud receivers
  RTPReceiver rtp_receiver_;
  TCPReceiver tcp_receiver_;
  // TCP log client
  LogClient log_client_;
  // SOME/IP control client
  Hugin hugin_client_;

  // Last ArbePacket received time
  std::chrono::steady_clock::time_point last_arbe_packet_time_;

  // This message contains firmware version and production data.
  raf2_interfaces::msg::MetaData meta_data_msg_;

  // This message contains the latest binary data received from the radar.
  raf2_interfaces::msg::BinaryData data_msg_;

  // Mutex for the network thread
  std::mutex io_mutex_;

  // Periodically log statistics and control state
  void logDataStatistics();

  // Called when an Arbe packet is received
  void onArbePacketReceived(const ArbeGenericHeader &arbe_generic_header,
                            const std::vector<uint8_t> &payload);

  // Log received callback
  void onLogReceived(const std::vector<std::string> &log_lines);

  // Common initialization for all constructors
  void initialize();

  // Called after constructor
  void onStartup();
  // Called on node shutdown
  void onShutdown();
  // Creates ROS2 services
  void createServices();
  // All start tx calls goes through this function
  bool startTx();
  // All stop tx calls goes through this function
  bool stopTx();
  // Set active mode
  bool setMode(const uint32_t mode_id);

  // Read production data
  void readDebugProductionData();

  // Show version
  void showVersion();
  // Show firmware version
  void showFirmwareVersion();
  // Update the mode list
  void updateModeList();

  // Set radar time to ROS2 time
  bool setRadarTimeToROS2Time();

  // Configure the radar signal processing
  bool setThresholds(const DynamicThresholds &dt, const StaticThresholds &st);

  // Publish best known control state message
  void publishControlState();

  // Compute system time in milli-seconds
  static uint64_t currentROS2TimeMs();

  // ROS2 service callbacks
  void startTxService(
      const std::shared_ptr<raf2_interfaces::srv::RdrCtrlStartTx::Request>,
      std::shared_ptr<raf2_interfaces::srv::RdrCtrlStartTx::Response> response);

  void stopTxService(
      const std::shared_ptr<raf2_interfaces::srv::RdrCtrlStopTx::Request>,
      std::shared_ptr<raf2_interfaces::srv::RdrCtrlStopTx::Response> response);

  void setThresholdsService(
      const std::shared_ptr<raf2_interfaces::srv::RdrCtrlSetThresholds::Request>
          request,
      std::shared_ptr<raf2_interfaces::srv::RdrCtrlSetThresholds::Response>
          response);

  void setModeService(
      const std::shared_ptr<raf2_interfaces::srv::RdrCtrlSetActiveSeq::Request>,
      std::shared_ptr<raf2_interfaces::srv::RdrCtrlSetActiveSeq::Response>
          response);

  void setTimeService(
      const std::shared_ptr<raf2_interfaces::srv::SysCfgSetTime::Request>,
      std::shared_ptr<raf2_interfaces::srv::SysCfgSetTime::Response> response);

  // Binary data publisher (these are "raw" pointcloud frames).
  rclcpp::Publisher<raf2_interfaces::msg::BinaryData>::SharedPtr
      binary_data_publisher_;

  // Log data from the radar
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ethernet_logger_;

  // Best known control state of the radar publisher
  rclcpp::Publisher<raf2_interfaces::msg::ControlState>::SharedPtr
      control_state_pub_;

  // Best known control state of the radar message
  raf2_interfaces::msg::ControlState control_state_msg_;

  // Production data publisher
  rclcpp::Publisher<raf2_interfaces::msg::MetaData>::SharedPtr meta_data_pub_;

  // Radar control services
  rclcpp::Service<raf2_interfaces::srv::RdrCtrlStartTx>::SharedPtr
      start_tx_service_;

  rclcpp::Service<raf2_interfaces::srv::RdrCtrlStopTx>::SharedPtr
      stop_tx_service_;

  rclcpp::Service<raf2_interfaces::srv::RdrCtrlSetThresholds>::SharedPtr
      set_thresholds_service_;

  rclcpp::Service<raf2_interfaces::srv::RdrCtrlSetActiveSeq>::SharedPtr
      set_active_seq_service_;

  rclcpp::Service<raf2_interfaces::srv::SysCfgSetTime>::SharedPtr
      set_time_service_;

  // Delete the copy constructor and copy assignment operator
  ControlNode(const ControlNode &) = delete;
  ControlNode &operator=(const ControlNode &) = delete;
  // Delete move constructor and move assignment operator
  ControlNode(ControlNode &&) = delete;
  ControlNode &operator=(ControlNode &&) = delete;

public:
  ControlNode(const std::string &ip = "10.20.30.40",
              const std::uint16_t ctrl_port = 6001,
              const std::uint16_t data_port = 6003,
              const std::uint16_t log_port = 6010,
              const std::uint16_t someip_port = 6007,
              const bool ptp_is_active = false,
              const bool auto_start_tx = false);

  explicit ControlNode(const rclcpp::NodeOptions &options);

  ~ControlNode() override;
};
