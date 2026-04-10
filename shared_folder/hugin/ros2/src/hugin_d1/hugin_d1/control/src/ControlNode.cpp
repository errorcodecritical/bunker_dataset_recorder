// Copyright (c) Sensrad 2024-2025

#include "Hugin.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include <ArbeReceiver.hpp>
#include <HumanReadableBytes.hpp>
#include <LogClient.hpp>
#include <RTPReceiver.hpp>
#include <TCPReceiver.hpp>

#include <DynamicThresholds.hpp>
#include <StaticThresholds.hpp>

#include <DebugCommands.hpp>

#include <ControlNode.hpp>

// Constructor for standalone node
ControlNode::ControlNode(const std::string &ip, const std::uint16_t ctrl_port,
                         const std::uint16_t data_port,
                         const std::uint16_t log_port,
                         const std::uint16_t someip_port,
                         const bool ptp_is_active, const bool auto_start_tx)
    : Node("control_node"), ip_(ip), ctrl_port_(ctrl_port), log_port_(log_port),
      data_port_(data_port), someip_port_(someip_port),
      ptp_is_active_(ptp_is_active), rtp_receiver_{get_logger()},
      tcp_receiver_{get_logger()}, log_client_(get_logger()),
      hugin_client_(get_logger()) {

  auto_start_tx_ = auto_start_tx; // Store the parameter
  initialize();
}

// Constructor for composable node (accepts NodeOptions)
ControlNode::ControlNode(const rclcpp::NodeOptions &options)
    : Node("control_node", options), ip_("10.20.30.40"), ctrl_port_(6001),
      log_port_(6010), data_port_(6003), someip_port_(6007),
      ptp_is_active_(false), rtp_receiver_{get_logger()},
      tcp_receiver_{get_logger()}, log_client_(get_logger()),
      hugin_client_(get_logger()) {

  initialize();
}

// Common initialization for all constructors
void ControlNode::initialize() {
  // Publisher qos
  rclcpp::QoS binary_data_qos((rclcpp::KeepLast(QOS_BACKLOG)));

  // Publishers
  binary_data_publisher_ = create_publisher<raf2_interfaces::msg::BinaryData>(
      "binary_data", binary_data_qos);

  ethernet_logger_ =
      create_publisher<std_msgs::msg::String>("ethernet_logger", QOS_BACKLOG);

  rclcpp::QoS control_state_qos(rclcpp::KeepLast(1));
  control_state_qos.transient_local();

  control_state_pub_ = create_publisher<raf2_interfaces::msg::ControlState>(
      "control_state", control_state_qos);

  meta_data_pub_ = create_publisher<raf2_interfaces::msg::MetaData>(
      "meta_data", control_state_qos);

  // Default frame id
  frame_id_ = "radar";

  // Declare and get launch parameters
  ip_ = declare_parameter("ip", ip_);
  bind_addr_ = declare_parameter("bind_addr", "0.0.0.0");
  ctrl_port_ = declare_parameter("ctrl_port", ctrl_port_);
  log_port_ = declare_parameter("log_port", log_port_);
  data_port_ = declare_parameter("data_port", data_port_);
  someip_port_ = declare_parameter("someip_port", someip_port_);
  auto_start_tx_ = declare_parameter("auto_start_tx", auto_start_tx_);
  use_tcp_ = declare_parameter("use_tcp", false);
  ptp_is_active_ = declare_parameter("ptp_is_active", ptp_is_active_);

  control_state_msg_.header.frame_id = frame_id_;

  data_msg_.header.frame_id = frame_id_;
  // Reserve space for the data message
  data_msg_.data.reserve(DATA_MSG_RESERVE);

  control_state_msg_.ptp_is_active = ptp_is_active_;

  RCLCPP_INFO(get_logger(), "Hugin D1 control node started with parameters:");
  RCLCPP_INFO(get_logger(), "  - Radar IP address: %s", ip_.c_str());
  RCLCPP_INFO(get_logger(), "  - UDP bind address: %s", bind_addr_.c_str());
  RCLCPP_INFO(get_logger(), "  - Control port: %u", ctrl_port_);
  RCLCPP_INFO(get_logger(), "  - Data port: %u", data_port_);
  RCLCPP_INFO(get_logger(), "  - Log port: %u", log_port_);
  RCLCPP_INFO(get_logger(), "  - SOME/IP port: %u", someip_port_);
  RCLCPP_INFO(get_logger(), "  - PTP is active: %s",
              ptp_is_active_ ? "true" : "false");

  // Register on_shutdown callback
  rclcpp::on_shutdown(std::bind(&ControlNode::onShutdown, this));

  // Create ROS2 services
  createServices();

  // One shot timer so we don't do any work in the constructor.
  startup_timer_ = create_wall_timer(std::chrono::milliseconds(1),
                                     std::bind(&ControlNode::onStartup, this));

  ArbeReceiver arbe_receiver{get_logger()};

  arbe_receiver.add_arbe_packet_received_hook(
      std::bind(&ControlNode::onArbePacketReceived, this, std::placeholders::_1,
                std::placeholders::_2));

  rtp_receiver_.add_rtp_packet_received_hook(
      arbe_receiver); // If we reach this point, the radar is connected

  tcp_receiver_.add_arbe_packet_received_hook(
      std::bind(&ControlNode::onArbePacketReceived, this, std::placeholders::_1,
                std::placeholders::_2));

  log_client_.add_log_received_hook(
      std::bind(&ControlNode::onLogReceived, this, std::placeholders::_1));
}

ControlNode::~ControlNode() {}

void ControlNode::logDataStatistics() {
  // Network state

  if (use_tcp_) {
    RCLCPP_INFO(get_logger(), "TCPReceiver:");
    RCLCPP_INFO(get_logger(), "  - Arbe Packet count: %zu",
                tcp_receiver_.arbe_packet_count());
    RCLCPP_INFO(
        get_logger(), "  - Total received bytes: %s",
        humanReadableBytes(tcp_receiver_.total_bytes_received()).c_str());
  } else {

    RCLCPP_INFO(get_logger(), "RTPReceiver statistics:");
    RCLCPP_INFO(get_logger(), "  - RTP packet count: %zu",
                rtp_receiver_.packet_count());
    RCLCPP_INFO(
        get_logger(), "  - Total received bytes: %s",
        humanReadableBytes(rtp_receiver_.total_bytes_received()).c_str());
  }
}

void ControlNode::createServices() {
  // Create services
  start_tx_service_ = create_service<raf2_interfaces::srv::RdrCtrlStartTx>(
      "start_tx", std::bind(&ControlNode::startTxService, this,
                            std::placeholders::_1, std::placeholders::_2));

  stop_tx_service_ = create_service<raf2_interfaces::srv::RdrCtrlStopTx>(
      "stop_tx", std::bind(&ControlNode::stopTxService, this,
                           std::placeholders::_1, std::placeholders::_2));

  set_thresholds_service_ =
      create_service<raf2_interfaces::srv::RdrCtrlSetThresholds>(
          "set_thresholds",
          std::bind(&ControlNode::setThresholdsService, this,
                    std::placeholders::_1, std::placeholders::_2));

  set_active_seq_service_ =
      create_service<raf2_interfaces::srv::RdrCtrlSetActiveSeq>(
          "set_active_seq",
          std::bind(&ControlNode::setModeService, this, std::placeholders::_1,
                    std::placeholders::_2));

  set_time_service_ = create_service<raf2_interfaces::srv::SysCfgSetTime>(
      "set_time", std::bind(&ControlNode::setTimeService, this,
                            std::placeholders::_1, std::placeholders::_2));
}

void ControlNode::showVersion() {
  const auto version = hugin_client_.get_version();

  RCLCPP_INFO(get_logger(), "Hugin D1 Version:");
  RCLCPP_INFO(get_logger(), "  - Version: %s", version.version.c_str());
  RCLCPP_INFO(get_logger(), "  - Name: %s", version.name.c_str());
  RCLCPP_INFO(get_logger(), "  - Date: %s", version.date.c_str());

  // Add this to the meta_data message

  meta_data_msg_.raf_version = version.version;
}

void ControlNode::showFirmwareVersion() {
  const auto firmware_version = hugin_client_.get_firmware_version();

  RCLCPP_INFO(get_logger(), "Firmware Version:");

  for (const auto &image : firmware_version) {
    RCLCPP_INFO(get_logger(), "Firmware image:");
    RCLCPP_INFO(get_logger(), "  - obj_type: %u", image.obj_type);
    RCLCPP_INFO(get_logger(), "  - sha256: %s", image.sha256_hex.c_str());
    for (const auto &entry : image.entries) {
      RCLCPP_INFO(get_logger(), "    - Version ID: %u", entry.id);
      RCLCPP_INFO(get_logger(), "      Major: %u", entry.major);
      RCLCPP_INFO(get_logger(), "      Minor: %u", entry.minor);
      RCLCPP_INFO(get_logger(), "      Patch: %u", entry.patch);
    }
  }
}

void ControlNode::updateModeList() {

  const auto modes = hugin_client_.get_modes();

  RCLCPP_INFO(get_logger(), "Available modes:");

  for (const auto &mode : modes) {
    RCLCPP_INFO(get_logger(), "  - ID: %d, Name: %s", mode.first,
                mode.second.c_str());

    // Should the sorting be different?

    control_state_msg_.mode_ids.push_back(mode.first);
    control_state_msg_.mode_names.push_back(mode.second);
  }

  publishControlState();
}

void ControlNode::readDebugProductionData() {

  const ProductionData pd = DebugCommands::readProductionData(ip_, ctrl_port_);

  meta_data_msg_.production_date = pd.production_date;
  meta_data_msg_.radar_device_system_id = pd.radar_device_system_id;
  meta_data_msg_.part_number = pd.part_number;
  meta_data_msg_.radar_device_module_part_name_model_number =
      pd.radar_device_module_part_name_model_number;
  meta_data_msg_.eco_rev = pd.eco_rev;
  meta_data_msg_.analog_board_part_number = pd.analog_board_part_number;
  meta_data_msg_.analog_board_serial_number = pd.analog_board_serial_number;
  meta_data_msg_.digital_board_part_number = pd.digital_board_part_number;
  meta_data_msg_.digital_board_serial_number = pd.digital_board_serial_number;
  meta_data_msg_.rfic_rx_version = pd.rfic_rx_version;
  meta_data_msg_.rfic_tx_version = pd.rfic_tx_version;
  meta_data_msg_.spare1 = pd.spare1;
  meta_data_msg_.spare2 = pd.spare2;
  meta_data_msg_.spare3 = pd.spare3;
}

void ControlNode::onStartup() {

  // Make sure the device is initialized once.
  static std::once_flag startup_flag;
  std::call_once(startup_flag, [this]() {
    // Cancel the startup timer, we only need one-shot (but there is not
    // one-shot timer in ROS2).
    startup_timer_->cancel();

    // Read production data
    readDebugProductionData();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(get_logger(), "Starting up ControlNode...");

    RCLCPP_INFO(get_logger(), "Connecting to Hugin on %s:%d...", ip_.c_str(),
                someip_port_);

    // Connect to Hugin, this function blocks until we are connected.
    hugin_client_.connect(ip_, someip_port_);
    RCLCPP_INFO(get_logger(), "Connected to Hugin D1");
    control_state_msg_.is_connected = true;

    // Start log client
    RCLCPP_INFO(get_logger(), "Starting log client on %s:%d...", ip_.c_str(),
                log_port_);

    log_client_.start(ip_, log_port_);

    // Logs the version information and also populates the meta_data_msg_
    showVersion();

    // Meta data is now populated, publish it.
    meta_data_msg_.header.frame_id = frame_id_;
    meta_data_msg_.header.stamp = rclcpp::Clock().now();
    meta_data_pub_->publish(meta_data_msg_);

    showFirmwareVersion();

    updateModeList();

    // The SOME/IP API currently does not support getting the current mode.
    const auto first_mode = control_state_msg_.mode_ids.front();
    setMode(first_mode);

    // Set's the default thresholds
    setThresholds(DynamicThresholds{}, StaticThresholds{});

    if (not ptp_is_active_) {
      setRadarTimeToROS2Time();
    }

    // If auto_start_tx
    if (auto_start_tx_) {
      startTx();
    }

    // Prevent io thread to continue until this function returns
    std::unique_lock<std::mutex> lock(io_mutex_);

    // Start data streaming workers
    if (use_tcp_) {
      tcp_receiver_.start(ip_, data_port_);
    } else {
      // TODO: Bind's to all interfaces
      rtp_receiver_.start(bind_addr_, data_port_);
    }
  });
}

void ControlNode::onShutdown() {
  RCLCPP_INFO(get_logger(), "Shutting down ControlNode...");

  stopTx();

  // TODO: clean up
  if (use_tcp_) {
    tcp_receiver_.stop();
  } else {
    rtp_receiver_.stop();
  }

  // Stop log client if it is running
  log_client_.stop();

  control_state_msg_.is_connected = false;

  publishControlState();
}

bool ControlNode::setRadarTimeToROS2Time() {
  const auto time_ms = currentROS2TimeMs() + TIME_OFFSET_MS;
  const auto tod_config = 1;
  const auto status = hugin_client_.set_tod_time(tod_config, time_ms);

  if (status) {
    control_state_msg_.timestamp_ms = time_ms;
    publishControlState();
  } else {
    RCLCPP_WARN(get_logger(), "Failed to set time");
  }

  return status;
}

bool ControlNode::startTx() {
  const bool status = hugin_client_.start_tx();
  if (status) {
    control_state_msg_.tx_enabled = true;
    publishControlState();
  } else {
    RCLCPP_WARN(get_logger(), "Failed to start radar");
  }

  return status;
}

bool ControlNode::stopTx() {
  const bool status = hugin_client_.stop_tx();

  if (status) {
    control_state_msg_.tx_enabled = false;
    publishControlState();
  } else {
    RCLCPP_WARN(get_logger(), "Failed to stop radar");
  }

  return status;
}

bool ControlNode::setMode(const uint32_t mode_id) {

  const auto status = hugin_client_.get_change_mode(mode_id);

  if (status) {

    control_state_msg_.selected_mode_id = mode_id;

    const auto it = std::find(control_state_msg_.mode_ids.begin(),
                              control_state_msg_.mode_ids.end(), mode_id);
    const auto mode_index =
        std::distance(control_state_msg_.mode_ids.begin(), it);

    control_state_msg_.selected_mode_name =
        control_state_msg_.mode_names.at(mode_index);

    publishControlState();
  } else {
    RCLCPP_WARN(get_logger(), "Failed to set mode");
  }

  return status;
}

// NB: This is called on a network IO thread (not the ROS2 thread).
void ControlNode::onArbePacketReceived(const ArbeGenericHeader &header,
                                       const std::vector<uint8_t> &data) {

  // This callback is called when an Arbe packet is received.
  // It's called on a background thread.
  std::unique_lock<std::mutex> lock(io_mutex_);

  // Only publishes if the message type is 2 (BinaryData) for now
  if (header.msg_type() == 2) {

    RCLCPP_DEBUG(get_logger(),
                 "Received Arbe packet with type: %u, size: %zu bytes",
                 header.msg_type(), data.size());

    // Frame counter
    RCLCPP_DEBUG(get_logger(), "  - frame_counter: %u", header.seq_num());

    data_msg_.type = header.msg_type();
    data_msg_.header.stamp = rclcpp::Clock().now();

    data_msg_.data.clear();
    // Create a new BinaryData message
    std::copy(data.begin(), data.end(), std::back_inserter(data_msg_.data));

    // Publish
    binary_data_publisher_->publish(data_msg_);
  }

  // Now
  const auto now = std::chrono::steady_clock::now();
  // Get duration
  const auto elapsed_time = now - last_arbe_packet_time_;
  // Get millis
  const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time)
          .count();

  // TODO: Make this interval an option
  if (elapsed_ms > 2000) {
    logDataStatistics();

    last_arbe_packet_time_ = now;
  }
}

// NB: This is called on a network IO thread (not the ROS2 thread).
void ControlNode::onLogReceived(const std::vector<std::string> &log_lines) {
  for (const auto &line : log_lines) {
    // Print to console
    RCLCPP_INFO(get_logger(), "%s", line.c_str());

    // Create a String message
    std_msgs::msg::String msg;
    msg.data = line;

    // Publish the log line
    ethernet_logger_->publish(msg);
  }
}

bool ControlNode::setThresholds(const DynamicThresholds &dynamic,
                                const StaticThresholds &stat) {

  bool status = true;
  // Dynamic thresholds
  for (uint32_t array_type : {0, 1}) {
    for (uint32_t sub_array : {0, 1}) {

      // Non beam forming 1,2 beam forming 4,5
      const uint32_t sub_array_type = (array_type * 3) + sub_array + 1;

      // Dynamic coarse thresholds
      if (!hugin_client_.set_dynamic_coarse_thresholds(
              sub_array_type, dynamic.coarse_nearfield_db(),
              dynamic.coarse_farfield_db())) {
        status = false;
        RCLCPP_WARN(get_logger(), "Failed to set dynamic coarse thresholds");
      }

      // These are normally set to zero by Arbe
      assert(dynamic.coarse_nearfield_db() == 0);
      assert(dynamic.coarse_farfield_db() == 0);
    }

    // Dynamic fine thresholds
    if (!hugin_client_.set_dynamic_fine_thresholds(
            array_type, dynamic.azimuth_nearfield_db(),
            dynamic.azimuth_farfield_db(), dynamic.elevation_farfield_db(),
            dynamic.elevation_nearfield_db(), dynamic.basic_nearfield_db(),
            dynamic.basic_farfield_db())) {
      status = false;
      RCLCPP_WARN(get_logger(), "Failed to set dynamic fine thresholds");
    }
  }

  // Static thresholds
  if (!hugin_client_.set_static_coarse_thresholds(stat.threshold_3d_db(),
                                                  stat.threshold_4d_db())) {
    status = false;
    RCLCPP_WARN(get_logger(), "Failed to set static coarse thresholds");
  }

  if (!hugin_client_.set_static_fine_thresholds(stat.threshold_3d_db(),
                                                stat.threshold_4d_db())) {
    status = false;
    RCLCPP_WARN(get_logger(), "Failed to set static fine thresholds");
  }

  if (status) {
    // Update control state with dynamic thresholds
    control_state_msg_.thresholds.dynamic_basic = dynamic.basic_farfield();
    control_state_msg_.thresholds.dynamic_azimuth = dynamic.azimuth_farfield();
    control_state_msg_.thresholds.dynamic_elevation =
        dynamic.elevation_farfield();

    // Update control state with static threshold
    control_state_msg_.thresholds.static_threshold = stat.static_threshold();

    // Publish updated state
    publishControlState();
  }

  return status;
}

// Publish best known control state message
void ControlNode::publishControlState() {
  control_state_msg_.header.stamp = rclcpp::Clock().now();
  control_state_pub_->publish(control_state_msg_);
}

// Radar start Tx service interface
void ControlNode::startTxService(
    const std::shared_ptr<raf2_interfaces::srv::RdrCtrlStartTx::Request>
        request,
    std::shared_ptr<raf2_interfaces::srv::RdrCtrlStartTx::Response> response) {
  (void)request;

  const auto status = startTx();
  if (status) {
    response->success = true;
    response->message = "Started";
  } else {
    response->success = false;
    response->message = "Failed to start radar";
  }
}

// Radar stop Tx service interface
void ControlNode::stopTxService(
    const std::shared_ptr<raf2_interfaces::srv::RdrCtrlStopTx::Request> request,
    std::shared_ptr<raf2_interfaces::srv::RdrCtrlStopTx::Response> response) {

  (void)request;

  const auto status = stopTx();
  if (status) {
    response->success = true;
    response->message = "Stopped";
  } else {
    response->success = false;
    response->message = "Failed to stop radar";
  }
}

// Radar set thresholds service interface
void ControlNode::setThresholdsService(
    const std::shared_ptr<raf2_interfaces::srv::RdrCtrlSetThresholds::Request>
        request,
    std::shared_ptr<raf2_interfaces::srv::RdrCtrlSetThresholds::Response>
        response) {

  RCLCPP_INFO(get_logger(), "Received request to change thresholds");

  // Dynamic thresholds
  DynamicThresholds dynamic_thr(request->dynamic_basic,
                                request->dynamic_azimuth,
                                request->dynamic_elevation);

  // Static thresholds
  StaticThresholds static_thr(request->static_threshold);

  const auto status = setThresholds(dynamic_thr, static_thr);
  if (status) {
    response->success = true;
    response->message = "Thresholds modified successfully";
  } else {
    response->success = false;
    response->message = "Failed to set one or several thresholds";
  }
}

// Set mode
void ControlNode::setModeService(
    const std::shared_ptr<raf2_interfaces::srv::RdrCtrlSetActiveSeq::Request>
        request,
    std::shared_ptr<raf2_interfaces::srv::RdrCtrlSetActiveSeq::Response>
        response) {

  const auto status = setMode(request->requested_sequence_id);
  if (status) {
    response->success = true;
    response->message = "Radar mode changed successfully";
  } else {
    response->success = false;
    response->message = "Failed to set mode";
  }
}

// Compute current time in milliseconds
uint64_t ControlNode::currentROS2TimeMs() {
  using namespace std::chrono;

  rclcpp::Time now = rclcpp::Clock().now();
  return duration_cast<milliseconds>(nanoseconds(now.nanoseconds())).count();
}

void ControlNode::setTimeService(
    const std::shared_ptr<raf2_interfaces::srv::SysCfgSetTime::Request> request,
    std::shared_ptr<raf2_interfaces::srv::SysCfgSetTime::Response> response) {

  (void)request;

  if (not ptp_is_active_) {
    if (setRadarTimeToROS2Time()) {
      response->success = true;
      response->message = "Time set";
    } else {
      response->success = false;
      response->message = "Failed to set time";
    }
  } else {
    response->success = false;
    response->message = "PTP is active, not allowing manual setting of time ";

    RCLCPP_WARN(get_logger(), "PTP is active, not setting time");
  }
}
