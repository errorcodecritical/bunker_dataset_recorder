// Copyright (c) Sensrad 2025

#include "ArbeReceiver.hpp"
#include "ArbeGenericHeader.hpp"
#include "RTPPacket.hpp"

#include <rclcpp/logging.hpp>

// Constructor
ArbeReceiver::ArbeReceiver(rclcpp::Logger logger)
    : left_to_receive_{0}, current_arbe_generic_header_{}, logger_(logger) {

  current_receive_buffer_.reserve(RECEIVE_BUFFER_RESERVE);
}

// Destructor
ArbeReceiver::~ArbeReceiver() {}

void ArbeReceiver::add_arbe_packet_received_hook(
    ArbePacketReceivedCallback hook) {
  arbe_packet_received_hooks_.push_back(hook);
}

void ArbeReceiver::log_arbe_generic_header(
    const ArbeGenericHeader &arbe_generic_header) const noexcept {

  // TODO.
  (void)arbe_generic_header;
}

// RTP packet received callback
void ArbeReceiver::operator()(const RTPPacket &rtp_packet,
                              std::size_t packet_size) {

  // Offset into the payload to keep track of where we are
  size_t offset = 0;

  // All packets from the Hugin are 1472 bytes This might be a bug but
  // it's the way it is. Not all data is valid.
  if (packet_size != sizeof(RTPPacket)) {

    RCLCPP_WARN(get_logger(),
                "Unexpected packet size: %zu bytes, expected: %zu bytes",
                packet_size, sizeof(RTPPacket));

    // Skip this
    return;
  }

  // Get header of RTP packet
  const auto rtp_header = rtp_packet.header();
  // Get payload of RTP packet, this is a vector with the data after the RTP
  // header (12 bytes).
  const auto rtp_payload = rtp_packet.payload();

  // ArbeGenericHeader is the new header format from at least 2.9.3 It
  // replaces the old header everywhere except the log protocol (I
  // guess that will change).
  ArbeGenericHeader arbe_generic_header{};
  std::memcpy(&arbe_generic_header, rtp_payload.data() + offset,
              sizeof(ArbeGenericHeader));

  // It seems that sometimes a signature can appear at any time.
  if (arbe_generic_header.signature() == ARBE_GENERIC_HEADER_SIGNATURE) {

    // Sanity check
    // Check that header size is 64
    if (arbe_generic_header.header_length() != sizeof(ArbeGenericHeader)) {
      RCLCPP_WARN(get_logger(), "Unexpected header length: %u, expected: %zu",
                  arbe_generic_header.header_length(),
                  sizeof(ArbeGenericHeader));

      return;
    }

    // Store this header
    current_arbe_generic_header_ = arbe_generic_header;
    // This packet contains a ArbeGenericHeader so increment offset
    offset += sizeof(ArbeGenericHeader);

    // Make sure buffer is cleared.
    current_receive_buffer_.clear();
    left_to_receive_ = 0;

    log_arbe_generic_header(current_arbe_generic_header_);

    // If actual_data_len is not zero, this packet contains more than the
    // header
    if (current_arbe_generic_header_.actual_data_len() != 0) {
      // Copy the rest of the data into the receive buffer
      std::copy(rtp_payload.data() + offset,
                rtp_payload.data() + offset +
                    current_arbe_generic_header_.actual_data_len(),
                std::back_inserter(current_receive_buffer_));

      // Increment offset
      offset += current_arbe_generic_header_.actual_data_len();
      (void)offset; // Avoid unused variable warning
    }

    // Subtract what was in this packet
    // The result is often 0.
    left_to_receive_ = current_arbe_generic_header_.data_length() -
                       current_arbe_generic_header_.actual_data_len();

  } else {
    // Are we expecting more data?
    if (left_to_receive_ > 0) {
      // The payload size is the minimum of the remaining data and 1460
      const size_t payload_size =
          rtp_header.ssrc_data_len() ? rtp_header.ssrc_data_len() : 1460;
      left_to_receive_ -= payload_size;

      // Copy into receive buffer
      std::copy(rtp_payload.data(), rtp_payload.data() + payload_size,
                std::back_inserter(current_receive_buffer_));
    }
  }

  // Have we received to much without a new header?
  if (left_to_receive_ < 0) {
    // This should not happen and would indicate an error in the protocol or a
    // bug.
    RCLCPP_ERROR(get_logger(), "left_to_receive_is negative");
    current_receive_buffer_.clear();
    left_to_receive_ = 0;
  }

  // Are we done?
  if (left_to_receive_ == 0 && current_receive_buffer_.size() > 0) {
    // Yes we are.
    // Call all hooks
    for (const auto &hook : arbe_packet_received_hooks_) {
      hook(current_arbe_generic_header_, current_receive_buffer_);
    }
  }
}
