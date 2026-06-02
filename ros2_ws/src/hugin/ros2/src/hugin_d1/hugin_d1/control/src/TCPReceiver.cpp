// Copyright (c) Sensrad 2025

// ROS2
#include <rclcpp/logging.hpp>

// C++ Standard Library
#include <string>
#include <vector>

#include <TCPReceiver.hpp>

// Boost
#include <boost/asio.hpp>
#include <boost/asio/error.hpp>
#include <boost/bind/bind.hpp>
#include <boost/endian/conversion.hpp>

// Constructor
TCPReceiver::TCPReceiver(rclcpp::Logger logger)
    : socket_{io_context_}, arbe_packet_count_{0}, total_bytes_received_{0},
      logger_{logger}, reconnect_timer_(io_context_) {

  tcp_packet_buffer_.reserve(TCP_PACKET_SIZE);
}

// Destructor
TCPReceiver::~TCPReceiver() { stop(); }

// Add a callback
void TCPReceiver::add_arbe_packet_received_hook(
    ArbePacketReceivedCallback hook) {
  arbe_packet_received_hooks_.push_back(hook);
}

void TCPReceiver::handle_payload(const boost::system::error_code &ec,
                                 std::size_t length) {

  // Check if disconnected
  if (ec == boost::asio::error::operation_aborted) {
    RCLCPP_INFO(get_logger(), "Payload read operation aborted");
    return;
  } else if (ec) {
    RCLCPP_WARN(get_logger(), "%s", ec.message().c_str());
    try_reconnect();
    return;
  }

  // Store total bytes received
  total_bytes_received_ += length;

  if (length != current_arbe_generic_header_.data_length()) {
    RCLCPP_WARN(get_logger(),
                "Payload length %zu does not match header length %u", length,
                current_arbe_generic_header_.data_length());

    read_header();
  } else {

    arbe_packet_count_ += 1;

    // Now tcp_packet_buffer_ contains the payload data.
    // Call all hooks
    for (const auto &hook : arbe_packet_received_hooks_) {
      hook(current_arbe_generic_header_, tcp_packet_buffer_);
    }

    // Schedule to read the next ArbeGenericPacket
    read_header();
  }
}

void TCPReceiver::read_payload(const std::size_t payload_length) {

  tcp_packet_buffer_.clear();
  tcp_packet_buffer_.reserve(payload_length);

  // Schedule a header read, it has to be 1460 bytes.
  boost::asio::async_read(
      socket_, boost::asio::dynamic_buffer(tcp_packet_buffer_),
      boost::asio::transfer_exactly(payload_length),
      boost::bind(&TCPReceiver::handle_payload, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void TCPReceiver::handle_header(const boost::system::error_code &ec,
                                std::size_t length) {

  // Check if disconnected
  if (ec == boost::asio::error::operation_aborted) {
    RCLCPP_INFO(get_logger(), "Payload read operation aborted");
    return;
  } else if (ec) {
    RCLCPP_WARN(get_logger(), "%s", ec.message().c_str());
    try_reconnect();
    return;
  }

  // Store total bytes received
  total_bytes_received_ += length;

  const auto logger = rclcpp::get_logger("hugin_d1_control");

  // Copy header bytes
  std::memcpy(&current_arbe_generic_header_, tcp_packet_buffer_.data(),
              sizeof(ArbeGenericHeader));

  if (current_arbe_generic_header_.signature() !=
      ARBE_GENERIC_HEADER_SIGNATURE) {
    RCLCPP_ERROR(get_logger(), "Wrong signature, try next header...");

    tcp_packet_buffer_.clear();

    read_header();
  } else if (current_arbe_generic_header_.data_length() ==
             current_arbe_generic_header_.actual_data_len()) {
    // Single packet

    RCLCPP_INFO(get_logger(), "Received single packet of length: %u",
                current_arbe_generic_header_.data_length());

    // Erase header from buffer
    tcp_packet_buffer_.erase(tcp_packet_buffer_.begin(),
                             tcp_packet_buffer_.begin() +
                                 sizeof(ArbeGenericHeader));

    // Trim end
    tcp_packet_buffer_.resize(current_arbe_generic_header_.data_length());

    arbe_packet_count_ += 1;

    // Call all hooks
    for (const auto &hook : arbe_packet_received_hooks_) {
      hook(current_arbe_generic_header_, tcp_packet_buffer_);
    }

    read_header();
  } else {
    // Multi "packet", read the payload
    read_payload(current_arbe_generic_header_.data_length());
  }
}

void TCPReceiver::read_header() {
  // Clear receive buffer
  tcp_packet_buffer_.clear();

  // Schedule a header read, it has to be 1460 bytes.
  boost::asio::async_read(
      socket_, boost::asio::dynamic_buffer(tcp_packet_buffer_),
      boost::asio::transfer_exactly(1460),
      boost::bind(&TCPReceiver::handle_header, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void TCPReceiver::handle_connect(const boost::system::error_code &ec) {
  if (ec == boost::asio::error::operation_aborted) {
    RCLCPP_INFO(get_logger(), "Connection aborted");
  } else if (ec) {
    RCLCPP_WARN(get_logger(), "Failed to connect: %s", ec.message().c_str());
    try_reconnect();
  } else {
    RCLCPP_INFO(get_logger(), "Connected");
    // Read header

    // Increase socket_ receive buffer size
    boost::asio::socket_base::receive_buffer_size option(1024 * 1024);
    socket_.set_option(option);

    read_header();
  }
}

void TCPReceiver::handle_reconnect_timer(const boost::system::error_code &ec) {
  if (ec == boost::asio::error::operation_aborted) {
    RCLCPP_WARN(get_logger(), "Reconnect timer aborted");
  } else if (ec) {
    RCLCPP_WARN(get_logger(), "%s", ec.message().c_str());
  } else {
    connect();
  }
}

void TCPReceiver::try_reconnect() {
  // Start a timer to reconnect
  // TODO: fix constant
  reconnect_timer_.expires_after(std::chrono::milliseconds(1000));
  // Wait for the timer to expire
  reconnect_timer_.async_wait(boost::bind(&TCPReceiver::handle_reconnect_timer,
                                          this,
                                          boost::asio::placeholders::error));
}

void TCPReceiver::connect() {

  ip::tcp::endpoint endpoint(ip::make_address(host_), port_);

  socket_.async_connect(endpoint,
                        boost::bind(&TCPReceiver ::handle_connect, this,
                                    boost::asio::placeholders::error));
}

// Start the TCPReceiver
void TCPReceiver::start(const std::string &host, const int port) {

  host_ = host;
  port_ = port;

  connect();

  // Run io_context in a separate thread
  // TODO: statistics has to be protected by a mutex
  worker_thread_ = std::thread([this]() { io_context_.run(); });
}

// Close socket and stop I/O context
void TCPReceiver::stop() {
  reconnect_timer_.cancel(); // Cancel any pending reconnect timer

  // Close socket if open
  if (socket_.is_open()) {
    socket_.shutdown(ip::tcp::socket::shutdown_both);
    socket_.close();
  }

  if (not io_context_.stopped()) {
    io_context_.stop();
  }

  // If joinable
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}
