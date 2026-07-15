// Copyright (c) Sensrad 2025

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

#include <ArbeCommonComHeader.hpp>

using namespace boost::asio;

class LogClient {

  // Attempt to reconnect every 2 seconds
  constexpr static int RECONNECT_TIMEOUT = 2000;

  io_context io_context_;
  ip::tcp::socket socket_;

  std::thread io_thread_;

  // ROS2 logger
  rclcpp::Logger logger_;

  boost::asio::steady_timer reconnect_timer_;

  std::string host_;
  std::uint16_t port_;

  // Type for callback hook
  using LogReceivedCallback =
      std::function<void(const std::vector<std::string> &)>;

  // Vector of callback hooks
  std::vector<LogReceivedCallback> log_received_hooks_;

  // Buffer for current header and payload
  ArbeCommonComHeader arbe_common_header_;
  std::vector<char> arbe_payload_;

  void handle_reconnect_timer(const boost::system::error_code &ec);
  void handle_connect(const boost::system::error_code &ec);
  void handle_header(const boost::system::error_code &ec, std::size_t length);
  void handle_payload(const boost::system::error_code &ec, std::size_t length);

  void read_header();
  void read_payload();
  void try_reconnect();
  void connect();

  rclcpp::Logger &get_logger() noexcept { return logger_; }

public:
  explicit LogClient(rclcpp::Logger logger);
  ~LogClient();

  void start(const std::string &host = "10.20.30.40",
             const uint16_t port = 6010);

  void stop();

  // Add callback hook
  void add_log_received_hook(LogReceivedCallback hook);
};
