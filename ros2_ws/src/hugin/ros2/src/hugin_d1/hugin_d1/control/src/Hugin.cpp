// Copyright (c) Sensrad 2025

#include "Hugin.hpp"

// C++ Standard Library
#include <array>
#include <cstdint>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <vector>

// boost
#include <boost/asio/ip/address.hpp>
#include <boost/endian/arithmetic.hpp>

// SOME/IP
#include <SomeIpBuilder.hpp>
#include <SomeIpReader.hpp>
#include <someip_header.hpp>
#include <someip_typedef.hpp>

// For boost endian conversion types
using namespace boost::endian;
using namespace SomeIpReader;

Hugin::Hugin(rclcpp::Logger logger)
    : socket_(io_context_), timeout_connect_(DEFAULT_CONNECT_TIMEOUT),
      timeout_read_(DEFAULT_READ_TIMEOUT),
      timeout_write_(DEFAULT_WRITE_TIMEOUT), session_id_(0), logger_(logger) {}

Hugin::~Hugin() { disconnect(); }

void Hugin::connect(const std::string &host, const uint16_t port) {
  const std::lock_guard<std::mutex> lock(mutex_);
  boost::system::error_code error;
  bool done = false;

  auto addr = boost::asio::ip::make_address(host, error);
  if (error) {
    throw std::runtime_error("Invalid IP address '" + host +
                             "': " + error.message());
  }
  const boost::asio::ip::tcp::endpoint endpoint{addr, port};

  boost::system::error_code ignore;
  socket_.close(ignore); // NOLINT(bugprone-unused-return-value)

  error.clear();
  socket_.async_connect(endpoint,
                        [&](const boost::system::error_code &connect_ec) {
                          error = connect_ec;
                          done = true;
                        });

  io_context_.restart();
  io_context_.run_for(timeout_connect_);

  if (!done) {
    // Timed out
    socket_.close(ignore); // NOLINT(bugprone-unused-return-value)
    io_context_.run();
    throw std::runtime_error("Timed out connecting to " + host + ":" +
                             std::to_string(port));
  }

  if (error) {
    throw std::runtime_error("Failed to connect to " + host + ":" +
                             std::to_string(port) + " (" + error.message() +
                             ")");
  }
}

void Hugin::disconnect() {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (socket_.is_open()) {
    boost::system::error_code ec;
    // Close and shutdown socket
    // NOLINTBEGIN(bugprone-unused-return-value)
    socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    socket_.close(ec);
    // NOLINTEND(bugprone-unused-return-value)
  }
}

std::optional<some_ip_header> Hugin::read_someip_header() {
  // Read the response header
  some_ip_header header;

  boost::system::error_code ec;
  bool done = false;
  bool timed_out = false;

  boost::asio::steady_timer timer(io_context_);
  timer.expires_after(timeout_read_);
  timer.async_wait([&](const boost::system::error_code &tec) {
    if (!tec && !done) {
      // timeout occurred, cancel read
      timed_out = true;
      boost::system::error_code ignore;
      socket_.cancel(ignore); // NOLINT(bugprone-unused-return-value)
    }
  });

  boost::asio::async_read(
      socket_, boost::asio::buffer(&header, sizeof(header)),
      [&](const boost::system::error_code &read_ec, std::size_t /*n*/) {
        ec = read_ec;
        done = true;
        timer.cancel(); // stop timer if read finished first
      });

  io_context_.restart();
  io_context_.run(); // runs until both handlers have run/cancelled

  if (timed_out) {
    RCLCPP_WARN(get_logger(), "Timed out reading SOME/IP header");
    return std::nullopt;
  }

  if (ec) {
    throw std::runtime_error(std::string("Failed to read SOME/IP header: ") +
                             ec.message());
  }

  return header;
}

std::optional<std::vector<uint8_t>>
Hugin::read_someip_payload(const some_ip_header &header) {
  // Read the response payload
  const auto len = header.payload_length();
  std::vector<uint8_t> payload(len, 0);
  if (len == 0) {
    return payload;
  }

  boost::system::error_code ec;
  bool done = false;
  bool timed_out = false;

  boost::asio::steady_timer timer(io_context_);
  timer.expires_after(timeout_read_);
  timer.async_wait([&](const boost::system::error_code &tec) {
    if (!tec && !done) {
      // timeout occurred, cancel read
      timed_out = true;
      boost::system::error_code ignore;
      // Keep connection open for failed responses
      socket_.cancel(ignore); // NOLINT(bugprone-unused-return-value)
    }
  });

  boost::asio::async_read(
      socket_, boost::asio::buffer(payload),
      [&](const boost::system::error_code &read_ec, std::size_t /*n*/) {
        ec = read_ec;
        done = true;
        timer.cancel();
      });

  io_context_.restart();
  io_context_.run();

  if (timed_out) {
    RCLCPP_WARN(get_logger(), "Timed out reading SOME/IP payload");
    return std::nullopt;
  }

  if (ec) {
    throw std::runtime_error(std::string("Failed to read SOME/IP payload: ") +
                             ec.message());
  }

  return payload;
}

uint16_t Hugin::next_session_id() noexcept { return ++session_id_; }

void Hugin::write_someip_request(const std::vector<uint8_t> &request) {
  boost::system::error_code ec;
  bool done = false;
  bool timed_out = false;

  boost::asio::steady_timer timer(io_context_);
  timer.expires_after(timeout_write_);
  timer.async_wait([&](const boost::system::error_code &tec) {
    if (!tec && !done) {
      timed_out = true;
      boost::system::error_code ignore;
      socket_.close(ignore); // NOLINT(bugprone-unused-return-value)
    }
  });

  boost::asio::async_write(
      socket_, boost::asio::buffer(request),
      [&](const boost::system::error_code &write_ec, std::size_t /*n*/) {
        ec = write_ec;
        done = true;
        timer.cancel();
      });

  io_context_.restart();
  io_context_.run();

  if (timed_out) {
    throw std::runtime_error("Timed out sending SOME/IP request");
  }

  if (ec) {
    throw std::runtime_error(std::string("Failed to write SOME/IP request: ") +
                             ec.message());
  }
}

// Send request and read response
std::optional<std::vector<uint8_t>>
Hugin::send_someip_request(const std::vector<uint8_t> &request,
                           const uint16_t expected_session_id) {

  const std::lock_guard<std::mutex> lock(mutex_);

  write_someip_request(request);

  // Read headers until we find matching session_id, discard stale responses
  while (true) {
    auto header = read_someip_header();
    if (!header) {
      return std::nullopt;
    }

    // Found correct response
    if (header->session_id() == expected_session_id) {
      return read_someip_payload(*header);
    }

    // Discard stale response
    RCLCPP_WARN(get_logger(),
                "Discarding stale response (session %u, expected %u)",
                header->session_id(), expected_session_id);
    if (!read_someip_payload(*header)) {
      return std::nullopt;
    }
  }
}

get_version_t Hugin::get_version() {

  const auto session_id = next_session_id();
  const auto request = SomeIpBuilder()
                           .request(MSG_TYPE_GET_VERSION)
                           .session_id(session_id)
                           .payload(some_ip_get_version_t())
                           .build();

  auto payload = send_someip_request(request, session_id);
  if (!payload) {
    throw std::runtime_error("Failed to get version from radar");
  }
  const auto response = read<some_ip_get_version_result_t>(payload.value());
  return get_version_t{response.version(), response.name(), response.date()};
}

std::vector<firmware_image_t> Hugin::get_firmware_version() {

  std::vector<firmware_image_t> images;
  const auto session_id = next_session_id();

  const auto request = SomeIpBuilder()
                           .request(MSG_TYPE_GET_FIRMWARE_VERSION)
                           .session_id(session_id)
                           .payload(some_ip_get_version_t())
                           .build();

  auto payload_opt = send_someip_request(request, session_id);

  if (!payload_opt) {
    RCLCPP_ERROR(get_logger(), "Get firmware version timed out");
    return images;
  }

  auto payload = payload_opt.value();

  const auto status = read<big_uint32_t>(payload);

  if (some_ip_status_result_t(status).failure()) {
    // Empty result
    return images;
  }

  const auto num_images = read<big_uint32_t>(payload);

  for (uint32_t i = 0; i < num_images; i++) {

    firmware_image_t image;

    image.obj_type = read<big_uint32_t>(payload);
    const auto num_entries = read<big_uint32_t>(payload);
    image.sha256_hex = read_bytes_as_hex_string(payload, 32);

    for (uint32_t j = 0; j < num_entries; j++) {

      firmware_entry_t entry;

      entry.id = read<big_uint32_t>(payload);
      entry.major = read<big_uint32_t>(payload);
      entry.minor = read<big_uint32_t>(payload);
      entry.patch = read<big_uint32_t>(payload);

      entry.time = read_string(payload, 20);
      entry.ci_commit_sha = read_string(payload, 44);
      entry.comment = read_string(payload, 64);

      image.entries.push_back(entry);
    }

    images.push_back(image);
  }

  // We should have consumed the whole buffer at this point.
  assert(payload.empty());

  return images;
}

bool Hugin::start_tx() {

  const auto session_id = next_session_id();
  const auto request = SomeIpBuilder()
                           .request(MSG_TYPE_START_TX)
                           .session_id(session_id)
                           .payload(some_ip_start_tx_t())
                           .build();

  auto payload = send_someip_request(request, session_id);

  if (!payload) {
    RCLCPP_ERROR(get_logger(), "Start TX request timed out");
    return false;
  }

  const auto status = read<some_ip_status_result_t>(payload.value());

  return status.success();
};

bool Hugin::stop_tx() {

  const auto session_id = next_session_id();
  const auto request = SomeIpBuilder()
                           .request(MSG_TYPE_STOP_TX)
                           .session_id(session_id)
                           .payload(some_ip_stop_tx_t())
                           .build();

  auto payload = send_someip_request(request, session_id);
  if (!payload) {
    RCLCPP_ERROR(get_logger(), "Stop TX request timed out");
    return false;
  }

  const auto status = read<some_ip_status_result_t>(payload.value());

  return status.success();
};

bool Hugin::set_tod_time(const uint32_t tod_config,
                         const uint64_t milliseconds) {

  const auto session_id = next_session_id();
  const auto request =
      SomeIpBuilder()
          .request(MSG_TYPE_SET_TIME)
          .session_id(session_id)
          .payload(some_ip_set_tod_time_t(tod_config, milliseconds))
          .build();

  auto payload = send_someip_request(request, session_id);

  if (!payload) {
    RCLCPP_ERROR(get_logger(), "Set TOD request timed out");
    return false;
  }

  const auto status = read<some_ip_status_result_t>(payload.value());

  return status.success();
};

bool Hugin::set_cw(const uint32_t un_tx_id,
                   const uint32_t un_num_of_bf_channels,
                   const uint32_t un_ant_id,
                   const uint64_t ul_cw_base_freq_hz) {

  const auto session_id = next_session_id();
  const auto request =
      SomeIpBuilder()
          .request(MSG_TYPE_START_CW)
          .session_id(session_id)
          .payload(some_ip_set_cw_t(un_tx_id, un_num_of_bf_channels, un_ant_id,
                                    ul_cw_base_freq_hz))
          .build();

  auto payload = send_someip_request(request, session_id);

  if (!payload) {
    RCLCPP_ERROR(get_logger(), "Set CW request timed out");
    return false;
  }

  const auto status = read<some_ip_status_result_t>(payload.value());

  return status.success();
}

bool Hugin::release_cw(const uint32_t un_tx_id) {

  const auto session_id = next_session_id();
  const auto request = SomeIpBuilder()
                           .request(MSG_TYPE_RELEASE_CW)
                           .session_id(session_id)
                           .payload(some_ip_release_cw_t(un_tx_id))
                           .build();

  auto payload = send_someip_request(request, session_id);
  if (!payload) {
    RCLCPP_ERROR(get_logger(), "Release CW request timed out");
    return false;
  }

  const auto status = read<some_ip_status_result_t>(payload.value());

  return status.success();
}

template <typename ResponseType>
std::map<uint32_t, std::string>
Hugin::try_parse_modes(std::vector<uint8_t> &resp_buffer) {
  std::map<uint32_t, std::string> modes;

  const auto resp = read<ResponseType>(resp_buffer);
  const uint32_t num_of_modes = resp.num_of_modes();

  for (uint32_t i = 0; i < num_of_modes; i++) {
    const auto mode_name = read_array<char, 32>(resp_buffer);
    const auto mode_id = read<big_uint32_t>(resp_buffer);
    const auto num_of_frames = read<big_uint32_t>(resp_buffer);

    // Consume frame data from buffer (required for correct protocol parsing)
    for (uint32_t f = 0; f < num_of_frames; f++) {
      const auto not_used = read<big_uint32_t>(resp_buffer);
      (void)not_used;
    }

    const auto visible = static_cast<bool>(read<big_uint32_t>(resp_buffer));

    if (visible) {
      modes.emplace(mode_id, std::string(std::begin(mode_name),
                                         std::find(std::begin(mode_name),
                                                   std::end(mode_name), '\0')));
    }
  }

  return modes;
}

std::map<uint32_t, std::string> Hugin::get_modes() {

  std::map<uint32_t, std::string> modes;

  const auto session_id = next_session_id();
  const auto request = SomeIpBuilder()
                           .request(MSG_TYPE_GET_MODE)
                           .session_id(session_id)
                           .payload(some_ip_change_mode_t())
                           .build();

  auto resp_buffer = send_someip_request(request, session_id);
  if (!resp_buffer) {
    RCLCPP_ERROR(get_logger(), "Get modes request timed out");
    return modes;
  }

  const auto resp_buffer_copy = resp_buffer.value();

  try {
    modes = try_parse_modes<some_ip_get_modes_result_t>(resp_buffer.value());

    if (resp_buffer.value().empty()) {
      return modes;
    }
  } catch (const std::runtime_error &) {
    // Failed to parse modes, fallback on old format
  }

  // Try to parse using 2.10.5 format
  resp_buffer = resp_buffer_copy;
  modes =
      try_parse_modes<some_ip_get_modes_result_2_10_5_t>(resp_buffer.value());

  // We should have consumed the whole buffer at this point.
  assert(resp_buffer.value().empty());

  return modes;
}

bool Hugin::get_change_mode(const uint32_t mode_id) {

  const auto session_id = next_session_id();
  const auto request = SomeIpBuilder()
                           .request(MSG_TYPE_GET_CHANGE_MODE)
                           .session_id(session_id)
                           .payload(some_ip_change_mode_t(mode_id))
                           .build();

  auto payload = send_someip_request(request, session_id);
  if (!payload) {
    RCLCPP_ERROR(get_logger(), "Change mode request timed out");
    return false;
  }

  const auto status = read<some_ip_status_result_t>(payload.value());

  return status.success();
}

// Dynamic thresholds
bool Hugin::set_dynamic_coarse_thresholds(
    const uint32_t sub_array_type, const uint32_t backoff_azimuth_near_field,
    const uint32_t backoff_azimuth_far_field) {

  const some_ip_set_dynamic_coarse_threshold_t req_payload(
      sub_array_type, backoff_azimuth_near_field, backoff_azimuth_far_field);

  const auto session_id = next_session_id();
  const auto request = SomeIpBuilder()
                           .request(MSG_TYPE_SET_DYNAMIC_COARSE_THRESHOLD)
                           .session_id(session_id)
                           .payload(req_payload)
                           .build();

  auto payload = send_someip_request(request, session_id);
  if (!payload) {
    RCLCPP_ERROR(get_logger(),
                 "Set dynamic coarse threshold request timed out");
    return false;
  }

  const auto status = read<some_ip_status_result_t>(payload.value());

  return status.success();
}

bool Hugin::set_dynamic_fine_thresholds(
    const uint32_t array_type, const uint32_t backoff_azimuth_near_field,
    const uint32_t backoff_azimuth_far_field,
    const uint32_t backoff_elevation_far_field,
    const uint32_t backoff_elevation_near_field,
    const uint32_t backoff_azimuth_elevation_near_field,
    const uint32_t backoff_azimuth_elevation_far_field) {

  const some_ip_set_dynamic_fine_threshold_t req_payload(
      array_type, backoff_azimuth_near_field, backoff_azimuth_far_field,
      backoff_elevation_far_field, backoff_elevation_near_field,
      backoff_azimuth_elevation_near_field,
      backoff_azimuth_elevation_far_field);

  const auto session_id = next_session_id();
  const auto request = SomeIpBuilder()
                           .request(MSG_TYPE_SET_DYNAMIC_FINE_THRESHOLD)
                           .session_id(session_id)
                           .payload(req_payload)
                           .build();

  auto payload = send_someip_request(request, session_id);
  if (!payload) {
    RCLCPP_ERROR(get_logger(), "Set dynamic fine threshold request timed out");
    return false;
  }

  const auto status = read<some_ip_status_result_t>(payload.value());

  return status.success();
}

// Static thresholds
bool Hugin::set_static_coarse_thresholds(const int32_t coarse_threshold,
                                         const int32_t coarse_dop0_threshold) {

  const auto session_id = next_session_id();
  const auto request = SomeIpBuilder()
                           .request(MSG_TYPE_SET_STATIC_COARSE_THRESHOLD)
                           .session_id(session_id)
                           .payload(some_ip_set_static_coarse_thresholds_t(
                               coarse_threshold, coarse_dop0_threshold))
                           .build();

  auto payload = send_someip_request(request, session_id);

  if (!payload) {
    RCLCPP_ERROR(get_logger(), "Set static coarse threshold request timed out");
    return false;
  }

  const auto status = read<some_ip_status_result_t>(payload.value());

  return status.success();
}

bool Hugin::set_static_fine_thresholds(const int32_t fine_threshold,
                                       const int32_t fine_dop0_threshold) {

  const auto session_id = next_session_id();
  const auto request = SomeIpBuilder()
                           .request(MSG_TYPE_SET_STATIC_FINE_THRESHOLD)
                           .session_id(session_id)
                           .payload(some_ip_set_static_fine_thresholds_t(
                               fine_threshold, fine_dop0_threshold))
                           .build();

  auto payload = send_someip_request(request, session_id);

  if (!payload) {
    RCLCPP_ERROR(get_logger(), "Set static fine threshold request timed out");
    return false;
  }

  const auto status = read<some_ip_status_result_t>(payload.value());

  return status.success();
}

bool Hugin::set_frame_trace_req(const bool enable, const uint32_t request_type,
                                const uint32_t num_of_time, const uint32_t p) {

  const auto session_id = next_session_id();
  const auto request =
      SomeIpBuilder()
          .request(MSG_TYPE_SET_FRAME_TRACE_REQ)
          .session_id(session_id)
          .payload(some_ip_set_frame_trace_data_req_t(
              static_cast<uint32_t>(enable), request_type, num_of_time, p))
          .build();

  auto payload = send_someip_request(request, session_id);

  if (!payload) {
    RCLCPP_ERROR(get_logger(), "Set frame trace request timed out");
    return false;
  }

  const auto status = read<some_ip_status_result_t>(payload.value());

  return status.success();
}

bool Hugin::set_sequence(const uint32_t sequence_id,
                         const std::vector<uint32_t> &frame_ids) {

  const auto session_id = next_session_id();
  const auto request = SomeIpBuilder()
                           .request(MSG_TYPE_SET_SEQUENCE)
                           .session_id(session_id)
                           .payload(some_ip_set_seq_t(sequence_id, frame_ids))
                           .build();

  auto payload = send_someip_request(request, session_id);
  if (!payload) {
    RCLCPP_ERROR(get_logger(), "Set sequence request timed out");
    return false;
  }

  const auto status = read<some_ip_status_result_t>(payload.value());

  return status.success();
}

bool Hugin::file_get_info(const uint32_t file_type_id) {

  const auto session_id = next_session_id();
  const auto request = SomeIpBuilder()
                           .request(MSG_TYPE_FILE_GET_INFO)
                           .session_id(session_id)
                           .payload(some_ip_file_info_req_t(file_type_id))
                           .build();

  auto payload = send_someip_request(request, session_id);
  if (!payload) {
    RCLCPP_ERROR(get_logger(), "Get file info request timed out");
    return false;
  }

  const auto file_info = read<some_ip_file_info_rsp_t>(payload.value());

  const some_ip_status_result_t status(file_info.status_);

  return status.success();
}

bool Hugin::file_read(const uint32_t file_type_id) {

  const uint32_t buffer_size = 1024;

  const auto session_id = next_session_id();
  const auto request =
      SomeIpBuilder()
          .request(MSG_TYPE_FILE_READ)
          .session_id(session_id)
          .payload(some_ip_file_read_req_t(file_type_id, buffer_size))
          .build();

  auto payload = send_someip_request(request, session_id);
  if (!payload) {
    RCLCPP_ERROR(get_logger(), "File read request timed out");
    return false;
  }

  const auto file_read = read<some_ip_file_read_rsp_t>(payload.value());

  const some_ip_status_result_t status(file_read.status_);

  return status.success();
}
