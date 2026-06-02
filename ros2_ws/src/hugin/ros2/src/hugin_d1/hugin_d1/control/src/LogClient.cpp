// Copyright (c) Sensrad 2025

#include <LogClient.hpp>

#include <ArbeCommonComHeader.hpp>

#include <boost/endian/arithmetic.hpp>
#include <rclcpp/logging.hpp>
#include <string>

#include <boost/algorithm/string.hpp> // trim_right
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

// Constructor
LogClient::LogClient(rclcpp::Logger logger)
    : socket_(io_context_), logger_{logger}, reconnect_timer_(io_context_),
      arbe_common_header_{} {}

// Destructor
LogClient::~LogClient() { stop(); }

void LogClient::handle_connect(const boost::system::error_code &ec) {

  if (ec == boost::asio::error::operation_aborted) {
    RCLCPP_INFO(get_logger(), "Connection aborted");
  } else if (ec) {
    RCLCPP_WARN(get_logger(), "Failed to connect to log port: %s",
                ec.message().c_str());
    try_reconnect();
  } else {
    RCLCPP_INFO(get_logger(), "Connected to log port");
    // Read header
    read_header();
  }
}

void LogClient::handle_header(const boost::system::error_code &ec,
                              std::size_t length) {

  if (ec) {
    RCLCPP_WARN(get_logger(), "LogClient::handle_header: %s",
                ec.message().c_str());

    try_reconnect();

    return;
  }

  if (length != sizeof(ArbeCommonComHeader)) {
    RCLCPP_ERROR(get_logger(), "Invalid header length");
  }

  // Read payload length as defined in the current header
  read_payload();
}

void LogClient::handle_payload(const boost::system::error_code &ec,
                               std::size_t length) {

  if (ec) {
    RCLCPP_WARN(get_logger(), "LogClient::handle_payload: %s",
                ec.message().c_str());

    try_reconnect();

    return;
  }

  // Check if the payload length is as expected
  if (length != arbe_common_header_.length()) {
    RCLCPP_ERROR(get_logger(), "Invalid payload length %zu", length);
    read_header();
    return;
  }

  std::vector<std::string> log_lines;

  // The lines are coded with a uint32_t (big endian) length followed
  // by the line
  std::size_t offset = 0;
  while (offset < arbe_payload_.size()) {

    boost::endian::big_uint32_t line_length;

    std::memcpy(&line_length, arbe_payload_.data() + offset,
                sizeof(line_length));

    offset += sizeof(line_length);

    // Create a string_view to the line from offset to offset + line_length
    std::string line(arbe_payload_.data() + offset, line_length);
    offset += line_length;

    // Trim away any trailing newlines
    boost::algorithm::trim_right(line);

    log_lines.push_back(line);
  }

  // Call hooks
  for (const auto &hook : log_received_hooks_) {
    hook(log_lines);
  }

  // Read next header
  read_header();
}

void LogClient::read_header() {
  // Read header
  boost::asio::async_read(
      socket_,
      boost::asio::buffer(&arbe_common_header_, sizeof(arbe_common_header_)),
      boost::bind(&LogClient::handle_header, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void LogClient::read_payload() {
  // Clear payload buffer, otherwise we will append to the previous payload
  arbe_payload_.clear();

  // Read payload
  boost::asio::async_read(
      socket_, boost::asio::dynamic_buffer(arbe_payload_),
      boost::asio::transfer_exactly(arbe_common_header_.length()),
      boost::bind(&LogClient::handle_payload, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void LogClient::handle_reconnect_timer(
    [[maybe_unused]] const boost::system::error_code &ec) {

  if (ec == boost::asio::error::operation_aborted) {
    RCLCPP_WARN(get_logger(), "Reconnect timer aborted");
  } else {
    connect();
  }
}

void LogClient::try_reconnect() {
  // Start a timer to reconnect
  reconnect_timer_.expires_after(std::chrono::milliseconds(RECONNECT_TIMEOUT));
  // Wait for the timer to expire
  reconnect_timer_.async_wait(boost::bind(&LogClient::handle_reconnect_timer,
                                          this,
                                          boost::asio::placeholders::error));
}

void LogClient::connect() {

  ip::tcp::endpoint endpoint(boost::asio::ip::make_address(host_), port_);

  socket_.async_connect(endpoint,
                        boost::bind(&LogClient::handle_connect, this,
                                    boost::asio::placeholders::error));
}

void LogClient::start(const std::string &host, const uint16_t port) {

  host_ = host;
  port_ = port;

  connect();

  io_thread_ = std::thread([this]() { io_context_.run(); });
}

void LogClient::stop() {
  boost::system::error_code ec;

  reconnect_timer_.cancel(ec); // Cancel any pending reconnect timer

  if (socket_.is_open()) {
    socket_.shutdown(ip::tcp::socket::shutdown_both, ec);
    // Close socket
    socket_.close(ec);
  }

  if (not io_context_.stopped()) {
    io_context_.stop();
  }

  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

void LogClient::add_log_received_hook(LogReceivedCallback hook) {
  log_received_hooks_.push_back(hook);
}
