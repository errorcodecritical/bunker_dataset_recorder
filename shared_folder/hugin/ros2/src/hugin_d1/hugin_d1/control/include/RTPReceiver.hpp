// Copyright (c) Sensrad 2025

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <boost/asio.hpp>

#include "RTPPacket.hpp"

using namespace boost::asio;

class RTPReceiver {

  io_context io_context_;
  ip::udp::socket socket_;

  ip::udp::endpoint sender_endpoint_;

  // Holds the current RTPPacket
  RTPPacket rtp_packet_;

  size_t packet_count_;
  size_t total_bytes_received_;
  size_t estimated_packet_loss_;

  uint16_t last_sequence_number_;

  // ROS2 logger
  rclcpp::Logger logger_;

  // Worker thread
  std::thread worker_thread_;

  // Type for callback hook
  using RTPPacketReceivedCallback =
      std::function<void(const RTPPacket &, const size_t)>;

  // Vector of callback hooks
  std::vector<RTPPacketReceivedCallback> rtp_packet_received_hooks_;

  void handle_packet(const size_t packet_size);

  // Boost asio callback function
  void handle_receive(const boost::system::error_code &error,
                      std::size_t bytes_transferred);

  // Schedules a new receive operation.
  void receive();

  void log_rtp_packet(const RTPPacket &rtp_packet) const noexcept;

  rclcpp::Logger &get_logger() noexcept { return logger_; }

  // Delete the copy constructor and copy assignment operator
  RTPReceiver(const RTPReceiver &) = delete;
  RTPReceiver &operator=(const RTPReceiver &) = delete;
  // Delete move constructor and move assignment operator
  RTPReceiver(RTPReceiver &&) = delete;
  RTPReceiver &operator=(RTPReceiver &&) = delete;

public:
  RTPReceiver(rclcpp::Logger logger);
  ~RTPReceiver();

  // Add callback hook
  void add_rtp_packet_received_hook(RTPPacketReceivedCallback hook);

  // Open socket and start receiving packets, this will block
  void start(const std::string &ip, const int port);

  // Total packet count.
  size_t packet_count() const noexcept;

  // Total bytes received
  size_t total_bytes_received() const noexcept { return total_bytes_received_; }

  // Estimated packet loss, number of packets.
  size_t estimated_packet_loss() const noexcept {
    return estimated_packet_loss_;
  }

  // Close socket and stop I/O context
  void stop();
};
