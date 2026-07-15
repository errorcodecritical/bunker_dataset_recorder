// Copyright (c) Sensrad 2025

#pragma once

// STL
#include <atomic>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

// ROS2
#include <rclcpp/logger.hpp>

// Boost
#include <boost/asio.hpp>

#include <someip_header.hpp>

struct get_version_t {
  std::string version;
  std::string name;
  std::string date;
};

struct firmware_entry_t {
  std::uint32_t id;
  std::uint32_t major;
  std::uint32_t minor;
  std::uint32_t patch;

  std::string time;
  std::string ci_commit_sha;
  std::string comment;
};

struct firmware_image_t {
  std::uint32_t obj_type;
  std::string sha256_hex;
  std::vector<firmware_entry_t> entries;
};

class Hugin {

  boost::asio::io_context io_context_;
  boost::asio::ip::tcp::socket socket_;

  std::mutex mutex_;

  std::chrono::milliseconds timeout_connect_;
  std::chrono::milliseconds timeout_read_;
  std::chrono::milliseconds timeout_write_;

  static constexpr std::chrono::milliseconds DEFAULT_CONNECT_TIMEOUT{5000};
  static constexpr std::chrono::milliseconds DEFAULT_READ_TIMEOUT{1000};
  static constexpr std::chrono::milliseconds DEFAULT_WRITE_TIMEOUT{1000};

  std::atomic<std::uint16_t> session_id_;

  std::uint16_t next_session_id() noexcept;

  // ROS2 logger
  rclcpp::Logger logger_;

  rclcpp::Logger &get_logger() noexcept { return logger_; }

  std::optional<std::vector<uint8_t>>
  send_someip_request(const std::vector<uint8_t> &request,
                      const uint16_t expected_session_id);

  void write_someip_request(const std::vector<uint8_t> &request);
  std::optional<some_ip_header> read_someip_header();
  std::optional<std::vector<uint8_t>>
  read_someip_payload(const some_ip_header &header);

  // Helper function to handle different versions
  template <typename ResponseType>
  std::map<uint32_t, std::string>
  try_parse_modes(std::vector<uint8_t> &resp_buffer);

public:
  explicit Hugin(rclcpp::Logger logger);
  ~Hugin();

  // Delete copy and move operations
  Hugin(const Hugin &) = delete;
  Hugin &operator=(const Hugin &) = delete;
  Hugin(Hugin &&) = delete;
  Hugin &operator=(Hugin &&) = delete;

  // Connect
  void connect(const std::string &host = "10.20.30.40",
               const uint16_t port = 6007);

  void disconnect();

  // Control the device
  get_version_t get_version();

  std::vector<firmware_image_t> get_firmware_version();

  bool start_tx();
  bool stop_tx();
  bool set_tod_time(const uint32_t tod_config, const uint64_t milliseconds);

  bool set_cw(const uint32_t un_tx_id, const uint32_t un_num_of_bf_channels,
              const uint32_t un_ant_id, const uint64_t ul_cw_base_freq_hz);

  bool release_cw(const uint32_t un_tx_id);

  // This naming follows Arbe's method naming
  std::map<uint32_t, std::string> get_modes();

  // Note: This call is named get_change_mode in the API, but it's not
  // possible to get the current mode. Only change.
  bool get_change_mode(const uint32_t mode_id);

  // Dynamic thresholds
  bool set_dynamic_coarse_thresholds(const uint32_t sub_array_type,
                                     const uint32_t backoff_azimuth_near_field,
                                     const uint32_t backoff_azimuth_far_field);
  bool set_dynamic_fine_thresholds(
      const uint32_t array_type, const uint32_t backoff_azimuth_near_field,
      const uint32_t backoff_azimuth_far_field,
      const uint32_t backoff_elevation_far_field,
      const uint32_t backoff_elevation_near_field,
      const uint32_t backoff_azimuth_elevation_near_field,
      const uint32_t backoff_azimuth_elevation_far_field);

  // Static thresholds
  bool set_static_coarse_thresholds(const int32_t coarse_threshold,
                                    const int32_t coarse_dop0_threshold);

  bool set_static_fine_thresholds(const int32_t fine_threshold,
                                  const int32_t fine_dop0_threshold);

  bool set_frame_trace_req(const bool enable, const uint32_t request_type,
                           const uint32_t num_of_time, const uint32_t p);

  bool set_sequence(const uint32_t sequence_id,
                    const std::vector<uint32_t> &frame_ids);

  bool file_get_info(const uint32_t file_type_id);

  // File read
  bool file_read(const uint32_t file_type_id);
};
