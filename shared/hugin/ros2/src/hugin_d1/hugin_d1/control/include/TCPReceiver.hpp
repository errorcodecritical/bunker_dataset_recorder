// Copyright (c) Sensrad 2025

#pragma once

#include <ArbeGenericHeader.hpp>

#include <string>
#include <thread>
#include <vector>

#include <rclcpp/logger.hpp>

#include <boost/asio.hpp>

using namespace boost::asio;

class TCPReceiver {

  // This is a constant in Arbe's implementation.
  constexpr static size_t TCP_PACKET_SIZE = 1460;

  io_context io_context_;
  ip::tcp::socket socket_;

  std::size_t arbe_packet_count_;
  std::size_t total_bytes_received_;

  // The current "active" header
  ArbeGenericHeader current_arbe_generic_header_;
  // Receive buffer
  std::vector<uint8_t> tcp_packet_buffer_;

  // Worker thread
  std::thread worker_thread_;

  // ROS2 logger
  rclcpp::Logger logger_;

  boost::asio::steady_timer reconnect_timer_;

  std::string host_;
  std::uint16_t port_;

  // Type for callback hook
  using ArbePacketReceivedCallback = std::function<void(
      const ArbeGenericHeader &, const std::vector<uint8_t> &)>;

  // Vector of callback hooks
  std::vector<ArbePacketReceivedCallback> arbe_packet_received_hooks_;

  void handle_payload(const boost::system::error_code &ec, std::size_t length);
  void read_payload(const std::size_t payload_length);

  void handle_connect(const boost::system::error_code &ec);

  void handle_header(const boost::system::error_code &ec, std::size_t length);
  void read_header();

  void handle_reconnect_timer(const boost::system::error_code &ec);
  void try_reconnect();
  void connect();

  rclcpp::Logger &get_logger() noexcept { return logger_; }

  // Delete the copy constructor and copy assignment operator
  TCPReceiver(const TCPReceiver &) = delete;
  TCPReceiver &operator=(const TCPReceiver &) = delete;
  // Delete move constructor and move assignment operator
  TCPReceiver(TCPReceiver &&) = delete;
  TCPReceiver &operator=(TCPReceiver &&) = delete;

public:
  TCPReceiver(rclcpp::Logger logger);
  ~TCPReceiver();

  std::size_t arbe_packet_count() const noexcept { return arbe_packet_count_; }
  std::size_t total_bytes_received() const noexcept {
    return total_bytes_received_;
  }

  // Add callback hook
  void add_arbe_packet_received_hook(ArbePacketReceivedCallback hook);

  // Open socket and start receiving packets, this will block
  void start(const std::string &host, const int port);

  // Close socket and stop I/O context
  void stop();
};
