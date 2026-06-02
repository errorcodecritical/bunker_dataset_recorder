// Copyright (c) Sensrad 2023-2025

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <glibmm.h>

// Standard message
#include <std_msgs/msg/header.hpp>
// Sensor messages
#include <sensor_msgs/msg/point_cloud2.hpp>
// Hugin messages
#include <raf2_interfaces/msg/control_state.hpp>
#include <raf2_interfaces/msg/header.hpp>

// Oden messages
#include <oden_interfaces/msg/oden_control_state.hpp>
#include <oden_interfaces/msg/oden_statistics.hpp>

// Hugin services
#include <raf2_interfaces/srv/rdr_ctrl_set_active_seq.hpp>
#include <raf2_interfaces/srv/rdr_ctrl_set_thresholds.hpp>
#include <raf2_interfaces/srv/rdr_ctrl_start_tx.hpp>
#include <raf2_interfaces/srv/rdr_ctrl_stop_tx.hpp>
#include <raf2_interfaces/srv/sys_cfg_set_time.hpp>

// Oden services
#include <oden_interfaces/srv/oden_ctrl_reset_perception.hpp>
#include <oden_interfaces/srv/oden_ctrl_set_perception_parameters.hpp>
#include <oden_interfaces/srv/oden_ctrl_set_roi_parameters.hpp>

// Recorder
#include "recorder.hpp"

#include <chrono>
#include <future>
#include <mutex>
#include <unordered_map>

#include <boost/format.hpp>

using namespace std::literals::chrono_literals;

class RAFGuiNode : public rclcpp::Node {
public:
  // Shorthand for signals
  using PointCloud2SharedPtr = sensor_msgs::msg::PointCloud2::SharedPtr;
  using ControlStateSharedPtr = raf2_interfaces::msg::ControlState::SharedPtr;
  using OdenControlStateSharedPtr =
      oden_interfaces::msg::OdenControlState::SharedPtr;
  using OdenStatisticsSharedPtr =
      oden_interfaces::msg::OdenStatistics::SharedPtr;

  // Signal types
  using signal_on_pointcloud_t =
      sigc::signal<void(PointCloud2SharedPtr pointcloud2)>;
  using signal_on_control_state_t =
      sigc::signal<void(ControlStateSharedPtr control_state)>;
  using signal_on_oden_control_state_t =
      sigc::signal<void(OdenControlStateSharedPtr oden_control_state)>;
  using signal_on_oden_statistics_t =
      sigc::signal<void(OdenStatisticsSharedPtr oden_statistics)>;
  using signal_on_t11_pointcloud_t = sigc::signal<void>;
  using signal_on_message_t = sigc::signal<void, std::string>;

  sigc::signal<void, const std::string &> signal_record_update;

private:
  constexpr static int QOS_BACKLOG = 10; // QoS backlog size

  Recorder recorder_; // ROS2 recorder instance

  // Signals (accessors further down)
  signal_on_pointcloud_t signal_on_pointcloud_;
  signal_on_control_state_t signal_on_control_state_;
  signal_on_t11_pointcloud_t signal_on_t11_pointcloud_;
  signal_on_message_t signal_on_message_;
  signal_on_oden_control_state_t signal_on_oden_control_state_;
  signal_on_oden_statistics_t signal_on_oden_statistics_;
  // Node lock
  std::mutex node_mutex_;

  rclcpp::TimerBase::SharedPtr alive_timer_;

  // ROS2 clients
  rclcpp::Client<raf2_interfaces::srv::RdrCtrlStartTx>::SharedPtr
      start_tx_client_;
  rclcpp::Client<raf2_interfaces::srv::RdrCtrlStopTx>::SharedPtr
      stop_tx_client_;
  rclcpp::Client<raf2_interfaces::srv::RdrCtrlSetActiveSeq>::SharedPtr
      set_active_seq_client_;
  rclcpp::Client<raf2_interfaces::srv::RdrCtrlSetThresholds>::SharedPtr
      set_thresholds_client_;
  rclcpp::Client<raf2_interfaces::srv::SysCfgSetTime>::SharedPtr
      set_time_client_;

  // Perception clients
  rclcpp::Client<oden_interfaces::srv::OdenCtrlResetPerception>::SharedPtr
      reset_perception_client_;
  rclcpp::Client<oden_interfaces::srv::OdenCtrlSetPerceptionParameters>::
      SharedPtr set_perception_parameters_client_;

  rclcpp::Client<oden_interfaces::srv::OdenCtrlSetRoiParameters>::SharedPtr
      set_roi_parameters_client_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_subscription_;
  rclcpp::Subscription<raf2_interfaces::msg::Header>::SharedPtr
      header_subscription_;
  rclcpp::Subscription<raf2_interfaces::msg::ControlState>::SharedPtr
      control_state_subscription_;
  rclcpp::Subscription<oden_interfaces::msg::OdenControlState>::SharedPtr
      oden_control_state_subscription_;
  rclcpp::Subscription<oden_interfaces::msg::OdenStatistics>::SharedPtr
      oden_statistics_subscription_;

  // Timer for rate limiting requests.
  rclcpp::TimerBase::SharedPtr rate_timer_;

  bool ptp_is_active_;

  // Emit a log message to be shown in the GUI.
  void emit_message(std::string &message) {
    // Emit message signal on main thread
    // Create a message that will outlive the callback
    std::shared_ptr<std::string> shared_msg =
        std::make_shared<std::string>(message);
    Glib::MainContext::get_default()->invoke([this, shared_msg]() {
      // Signal emitted on glib main thread
      signal_on_message_.emit(*shared_msg);
      return false; // false = single shot
    });
  }

  // Prune expired requests
  void on_alive_timer() {
    std::lock_guard<std::mutex> lock(node_mutex_);

    // Clear expired requests
    const auto expiration_time = std::chrono::system_clock::now() - 2s;
    std::size_t num_expired = 0;

    RCLCPP_DEBUG(get_logger(), "Pruning expired requests...");

    std::vector<int64_t> pruned_requests;
    // There's a bug in the prune_requests_older_than function.
    // https://github.com/ros2/rclcpp/issues/2007
    num_expired += start_tx_client_->prune_requests_older_than(
        expiration_time, &pruned_requests);
    num_expired += stop_tx_client_->prune_requests_older_than(expiration_time,
                                                              &pruned_requests);
    num_expired += set_active_seq_client_->prune_requests_older_than(
        expiration_time, &pruned_requests);
    num_expired += set_thresholds_client_->prune_requests_older_than(
        expiration_time, &pruned_requests);
    num_expired += set_time_client_->prune_requests_older_than(
        expiration_time, &pruned_requests);

    // Perception clients
    num_expired += reset_perception_client_->prune_requests_older_than(
        expiration_time, &pruned_requests);

    num_expired += set_perception_parameters_client_->prune_requests_older_than(
        expiration_time, &pruned_requests);

    num_expired += set_roi_parameters_client_->prune_requests_older_than(
        expiration_time, &pruned_requests);

    RCLCPP_DEBUG(get_logger(), "Expired %ld requests", num_expired);

    // Emit message for gui
    if (num_expired > 0) {
      std::string message =
          (boost::format("Connection to control node timed out (%d requests)") %
           num_expired)
              .str();
      emit_message(message);
    }
  }

  // Subscription callbacks
  void on_header(const raf2_interfaces::msg::Header::SharedPtr msg) {
    Glib::MainContext::get_default()->invoke([this, msg]() {
      signal_on_t11_pointcloud().emit();
      return false;
    });
  }

  void on_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    Glib::MainContext::get_default()->invoke([this, msg]() {
      // The ref count on msg is increased and will be valid until emit return.
      signal_on_pointcloud().emit(msg);
      return false;
    });
  }

  void
  on_control_state(const raf2_interfaces::msg::ControlState::SharedPtr msg) {
    // The ref count on msg is increased and will be valid until emit return.
    Glib::MainContext::get_default()->invoke([this, msg]() {
      signal_on_control_state().emit(msg);

      return false;
    });
  }

  void on_oden_control_state(
      const oden_interfaces::msg::OdenControlState::SharedPtr msg) {
    // The ref count on msg is increased and will be valid until emit return.
    Glib::MainContext::get_default()->invoke([this, msg]() {
      signal_on_oden_control_state().emit(msg);

      return false;
    });
  }

  void on_oden_statistics(
      const oden_interfaces::msg::OdenStatistics::SharedPtr msg) {
    // The ref count on msg is increased and will be valid until emit return.
    Glib::MainContext::get_default()->invoke([this, msg]() {
      signal_on_oden_statistics().emit(msg);

      return false;
    });
  }

  // Client slots
  void invoke_rate_limit(std::function<void()> &&callback) {
    rate_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        // capture shared_from_this() to keep the node alive
        // capture this for ease of access to the timer
        // move callback into lambda to avoid copying
        [lifetime = shared_from_this(), this, cb = std::move(callback)]() {
          cb();
          rate_timer_->cancel();
        });
  }

  // Update the status of the recording
  void update_status(const std::string &status) {
    signal_record_update.emit(status);
  }

public:
  // Signal accessors
  signal_on_pointcloud_t &signal_on_pointcloud() {
    return signal_on_pointcloud_;
  }

  signal_on_control_state_t &signal_on_control_state() {
    return signal_on_control_state_;
  }

  signal_on_t11_pointcloud_t &signal_on_t11_pointcloud() {
    return signal_on_t11_pointcloud_;
  }

  signal_on_message_t &signal_on_message() { return signal_on_message_; }

  signal_on_oden_control_state_t &signal_on_oden_control_state() {
    return signal_on_oden_control_state_;
  }

  signal_on_oden_statistics_t &signal_on_oden_statistics() {
    return signal_on_oden_statistics_;
  }

  explicit RAFGuiNode(const std::string &node_name)
      : Node(node_name), recorder_(), ptp_is_active_(false) {

    RCLCPP_INFO(get_logger(), "RAF GUI node started");

    std::lock_guard<std::mutex> lock(node_mutex_);

    // Create subscriptions
    header_subscription_ = create_subscription<raf2_interfaces::msg::Header>(
        "radar_header", QOS_BACKLOG,
        std::bind(&RAFGuiNode::on_header, this, std::placeholders::_1));

    pointcloud_subscription_ =
        create_subscription<sensor_msgs::msg::PointCloud2>(
            "radar_data", QOS_BACKLOG,
            std::bind(&RAFGuiNode::on_pointcloud, this, std::placeholders::_1));

    // Control state has transient local QoS.
    auto control_state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    control_state_qos.transient_local();

    control_state_subscription_ =
        create_subscription<raf2_interfaces::msg::ControlState>(
            "control_state", control_state_qos,
            std::bind(&RAFGuiNode::on_control_state, this,
                      std::placeholders::_1));
    // Oden control state has transient local QoS.
    auto oden_state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    oden_state_qos.transient_local();
    oden_control_state_subscription_ =
        create_subscription<oden_interfaces::msg::OdenControlState>(
            "oden/control_state", oden_state_qos,
            std::bind(&RAFGuiNode::on_oden_control_state, this,
                      std::placeholders::_1));

    oden_statistics_subscription_ =
        create_subscription<oden_interfaces::msg::OdenStatistics>(
            "oden/statistics", QOS_BACKLOG,
            std::bind(&RAFGuiNode::on_oden_statistics, this,
                      std::placeholders::_1));
    // Create a timer that will fire every 100ms.
    alive_timer_ =
        create_wall_timer(1000ms, std::bind(&RAFGuiNode::on_alive_timer, this));
    // Clients
    // Start tx client
    start_tx_client_ =
        create_client<raf2_interfaces::srv::RdrCtrlStartTx>("start_tx");
    // Stop tx client
    stop_tx_client_ =
        create_client<raf2_interfaces::srv::RdrCtrlStopTx>("stop_tx");
    // Set active seq client
    set_active_seq_client_ =
        create_client<raf2_interfaces::srv::RdrCtrlSetActiveSeq>(
            "set_active_seq");
    // Set thresholds client
    set_thresholds_client_ =
        create_client<raf2_interfaces::srv::RdrCtrlSetThresholds>(
            "set_thresholds");
    // Set time client
    set_time_client_ =
        create_client<raf2_interfaces::srv::SysCfgSetTime>("set_time");

    // Perception clients

    // Reset perception client
    reset_perception_client_ =
        create_client<oden_interfaces::srv::OdenCtrlResetPerception>(
            "oden/reset_perception");

    // Setperception parameter client
    set_perception_parameters_client_ =
        create_client<oden_interfaces::srv::OdenCtrlSetPerceptionParameters>(
            "oden/set_perception_parameters");

    // Set ROI parameters client
    set_roi_parameters_client_ =
        create_client<oden_interfaces::srv::OdenCtrlSetRoiParameters>(
            "oden/set_roi_parameters");

    // Recorder signal
    recorder_.signal_record_update.connect(
        sigc::mem_fun(*this, &RAFGuiNode::update_status));

    RCLCPP_INFO(get_logger(), "RAF GUI node initialized");
  }

  virtual ~RAFGuiNode() {}

  // Slots
  void start_tx() {
    std::lock_guard<std::mutex> lock(node_mutex_);

    // This will be executed on the ros2 run loop
    invoke_rate_limit([this]() {
      const auto request =
          std::make_shared<raf2_interfaces::srv::RdrCtrlStartTx::Request>();
      start_tx_client_->async_send_request(request);
    });
  }

  void stop_tx() {
    std::lock_guard<std::mutex> lock(node_mutex_);

    invoke_rate_limit([this]() {
      const auto request =
          std::make_shared<raf2_interfaces::srv::RdrCtrlStopTx::Request>();
      stop_tx_client_->async_send_request(request);
    });
  }

  void set_time() {
    std::lock_guard<std::mutex> lock(node_mutex_);

    invoke_rate_limit([this]() {
      const auto request =
          std::make_shared<raf2_interfaces::srv::SysCfgSetTime::Request>();
      request->use_ros2_time = true;
      set_time_client_->async_send_request(request);
    });
  }

  void set_sequence(uint32_t sequence_type) {
    std::lock_guard<std::mutex> lock(node_mutex_);

    invoke_rate_limit([this, sequence_type]() {
      const auto request = std::make_shared<
          raf2_interfaces::srv::RdrCtrlSetActiveSeq::Request>();
      request->requested_sequence_id = sequence_type;
      set_active_seq_client_->async_send_request(request);
    });
  }

  void set_threshold(const float static_threshold, const float dynamic_azimuth,
                     const float dynamic_elevation, const float dynamic_basic) {
    std::lock_guard<std::mutex> lock(node_mutex_);

    // This will be executed on the ros2 run loop
    invoke_rate_limit([this, static_threshold, dynamic_azimuth,
                       dynamic_elevation, dynamic_basic]() {
      const auto request = std::make_shared<
          raf2_interfaces::srv::RdrCtrlSetThresholds::Request>();

      request->static_threshold = static_threshold;
      request->dynamic_azimuth = dynamic_azimuth;
      request->dynamic_elevation = dynamic_elevation;
      request->dynamic_basic = dynamic_basic;

      set_thresholds_client_->async_send_request(request);
    });
  }

  void reset_perception() {
    std::lock_guard<std::mutex> lock(node_mutex_);

    // This will be executed on the ros2 run loop
    invoke_rate_limit([this]() {
      const auto request = std::make_shared<
          oden_interfaces::srv::OdenCtrlResetPerception::Request>();
      reset_perception_client_->async_send_request(request);
    });
  }

  void set_perception_parameters(
      const uint8_t perception_config, const float vertical_height,
      const float elevation_angle, const float cluster_scale,
      const float free_space_max_range, const float free_space_lower_bound,
      const float free_space_upper_bound, const int stationary_updates) {
    std::lock_guard<std::mutex> lock(node_mutex_);

    // This will be executed on the ros2 run loop
    invoke_rate_limit([this, perception_config, vertical_height,
                       elevation_angle, cluster_scale, free_space_max_range,
                       free_space_lower_bound, free_space_upper_bound,
                       stationary_updates]() {
      const auto request = std::make_shared<
          oden_interfaces::srv::OdenCtrlSetPerceptionParameters::Request>();
      request->perception_config = perception_config;
      request->vertical_height = vertical_height;
      request->elevation_angle = elevation_angle;
      request->cluster_scale = cluster_scale;
      request->free_space_max_range = free_space_max_range;
      request->free_space_lower_bound = free_space_lower_bound;
      request->free_space_upper_bound = free_space_upper_bound;
      request->stationary_updates = stationary_updates;

      set_perception_parameters_client_->async_send_request(request);
    });
  }

  void set_roi_parameters(const bool roi_onoff, const float roi_x_center,
                          const float roi_y_center, const float roi_z_center,
                          const float roi_width, const float roi_length,
                          const float roi_height, const float roi_yaw,
                          const float roi_pitch) {

    std::lock_guard<std::mutex> lock(node_mutex_);

    // This will be executed on the ros2 run loop
    invoke_rate_limit([this, roi_onoff, roi_x_center, roi_y_center,
                       roi_z_center, roi_width, roi_length, roi_height, roi_yaw,
                       roi_pitch]() {
      const auto request = std::make_shared<
          oden_interfaces::srv::OdenCtrlSetRoiParameters::Request>();
      request->enabled = roi_onoff;
      request->roi_x_center = roi_x_center;
      request->roi_y_center = roi_y_center;
      request->roi_z_center = roi_z_center;
      request->roi_width = roi_width;
      request->roi_length = roi_length;
      request->roi_height = roi_height;
      request->roi_yaw = roi_yaw;
      request->roi_pitch = roi_pitch;

      set_roi_parameters_client_->async_send_request(request);
    });
  }

  // Recorder functions
  void start_record(const std::string &recording_name,
                    const Recorder::ERecordingState &record_select,
                    const bool do_compression) {
    recorder_.startRecord(recording_name, record_select, do_compression);
  }

  void stop_record() { recorder_.stopRecord(); }

  bool is_recording() { return recorder_.isRecording(); }
};
