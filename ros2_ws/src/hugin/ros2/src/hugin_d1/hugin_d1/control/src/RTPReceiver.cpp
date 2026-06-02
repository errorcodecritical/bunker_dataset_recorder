// Copyright (c) Sensrad 2025

#include <RTPPacket.hpp>
#include <RTPReceiver.hpp>

// C++ Standard Library
#include <vector>

// ROS2
#include <rclcpp/logging.hpp>

// Boost
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/endian/conversion.hpp>

// Constructor for RTPReceiver
RTPReceiver::RTPReceiver(rclcpp::Logger logger)
    : socket_{io_context_}, rtp_packet_{}, packet_count_{0},
      total_bytes_received_{0}, estimated_packet_loss_{0},
      last_sequence_number_{0}, logger_(logger) {}

// Destructor for RTPReceiver
RTPReceiver::~RTPReceiver() { stop(); }

void RTPReceiver::log_rtp_packet(const RTPPacket &rtp_packet) const noexcept {
  const auto header = rtp_packet.header();
  (void)header;

  // Logging of RTP packet details can be added here.
}

// Add a callback
void RTPReceiver::add_rtp_packet_received_hook(RTPPacketReceivedCallback hook) {
  rtp_packet_received_hooks_.push_back(hook);
}

void RTPReceiver::handle_packet(const size_t packet_size) {
  // Here we validate the received packet and call all hooks

  packet_count_++;
  total_bytes_received_ += packet_size;

  // Check for packet using wrapping arithmetic
  const auto sequence_number = rtp_packet_.header().sequence_number();
  // Uses wrapping arithmetic to handle the case where the sequence number wraps
  // around

  // Expected
  const auto expected_sequence_nubmer =
      static_cast<uint16_t>(last_sequence_number_ + 1);

  if (last_sequence_number_ && sequence_number != expected_sequence_nubmer) {
    estimated_packet_loss_ += sequence_number - last_sequence_number_;
    // TODO: Maybe make this debug?
    RCLCPP_WARN(get_logger(), "RTP packet loss detected, expected: %d, got %d",
                last_sequence_number_ + 1, sequence_number);
  }
  // Update current sequence number
  last_sequence_number_ = sequence_number;

  // This is a very basic implementation. There are no more sanity checks.

  // Call all hooks
  for (const auto &hook : rtp_packet_received_hooks_) {
    hook(rtp_packet_, packet_size);
  }
}

void RTPReceiver::handle_receive(const boost::system::error_code &error,
                                 size_t bytes_transferred) {

  // Ignore "operation canceled" error
  if (error == boost::asio::error::operation_aborted) {
    RCLCPP_INFO(get_logger(), "RTPReceiver receive operation was canceled.");
    return;
  }

  // If there was another error, log it and return
  if (error) {
    RCLCPP_WARN(get_logger(), "Error receiving RTP packet: %s",
                error.message().c_str());
    return;
  }

  handle_packet(bytes_transferred);

  // Schedule the next receive on the io_context
  receive();
};

// Schedules a receive operation on the io_context
void RTPReceiver::receive() {
  // Zero out the rtp_packet_ buffer
  std::memset(&rtp_packet_, 0, sizeof(rtp_packet_));

  socket_.async_receive_from(
      boost::asio::buffer(&rtp_packet_, sizeof(rtp_packet_)), sender_endpoint_,
      boost::bind(&RTPReceiver::handle_receive, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

// Total RTP packet count
size_t RTPReceiver::packet_count() const noexcept { return packet_count_; }

// Start the RTPReceiver
void RTPReceiver::start(const std::string &ip, const int port) {

  // Open socket
  socket_.open(ip::udp::v4());
  // Bind the socket to the specified IP and port, this is the listen
  // interface/port.
  ip::udp::endpoint endpoint(ip::make_address(ip), port);
  socket_.bind(endpoint);

  // Increase socket_ receive buffer size
  boost::asio::socket_base::receive_buffer_size option(1024 * 1024);
  socket_.set_option(option);

  // Start the async receive
  receive();

  worker_thread_ = std::thread([this]() { io_context_.run(); });
}

// Close socket and stop I/O context
void RTPReceiver::stop() {
  if (socket_.is_open()) {
    // Close socket if open
    socket_.close();
  }

  if (not io_context_.stopped()) {
    io_context_.stop();
  }

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}
