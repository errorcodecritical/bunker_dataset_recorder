// Copyright (c) Sensrad 2025

#pragma once

#include <cstdint>

#include <boost/endian/conversion.hpp>

// This file intentionally doesn't include someip_typedef.h to avoid
// circular dependencies.

// some/ip message types
enum class some_ip_message_type : std::uint8_t {
  request = 0x00,
  request_no_return = 0x01,
  notification = 0x02,
  request_ack = 0x40,
  request_no_return_ack = 0x41,
  notification_ack = 0x42,
  response = 0x80,
  error = 0x81,
  response_ack = 0xc0,
  error_ack = 0xc1
};

// some/ip return codes
enum class some_ip_return_code : std::uint8_t {
  ok = 0x00,
  not_ok = 0x01,
  unknown_service = 0x02,
  unknown_method = 0x03,
  not_ready = 0x04,
  not_reachable = 0x05,
  time_out = 0x06,
  wrong_protocol_version = 0x07,
  wrong_interface_version = 0x08,
  malformed_message = 0x09,
  wrong_message_type = 0x0a
};

// some/ip header stored in network byte order (wire format)
struct __attribute__((packed)) some_ip_header {
  // 32-bit Message ID split up into service_id and method_id.
  // They are individually stored in network byte order.
  // Arbe uses 0x0000 for service_id
  uint16_t service_id_;
  // method_id depends on what is being requested
  uint16_t method_id_;

  // 32-bit length = payload size + 8 bytes (for everything after 'length')
  // Full message size on the wire is (length + 8).
  uint32_t length_;

  // 32-bit Request ID = Client ID (high 16 bits) + Session ID (low 16 bits)
  // Split into:
  uint8_t client_id_prefix_;
  uint8_t client_id_;
  // session_id is used as a message counter and is incremented for each new
  // message.
  uint16_t session_id_;

  // 8-bit Protocol Version
  // Arbe sets this to 0x01
  uint8_t protocol_version_;

  // 8-bit Interface Version
  // Arbe sets this to 0x01
  uint8_t interface_version_;

  // some/ip 8-bit Message Type (e.g., 0x00 = REQUEST, 0x80 = RESPONSE, etc.)
  uint8_t message_type_;

  // some/ip 8-bit Return Code
  // return code is set to 0x00 on requests
  uint8_t return_code_;

  // Constructor that stores in network byte order
  some_ip_header(uint16_t service_id, uint16_t method_id, uint32_t length,
                 uint8_t client_id_prefix, uint8_t client_id,
                 uint16_t session_id, uint8_t protocol_version,
                 uint8_t interface_version, uint8_t message_type,
                 uint8_t return_code) noexcept
      : service_id_(boost::endian::native_to_big(service_id)),
        method_id_(boost::endian::native_to_big(method_id)),
        length_(boost::endian::native_to_big(length)),
        client_id_prefix_(client_id_prefix), client_id_(client_id),
        session_id_(boost::endian::native_to_big(session_id)),
        protocol_version_(protocol_version),
        interface_version_(interface_version), message_type_(message_type),
        return_code_(return_code) {};

  // Zero constructor
  some_ip_header() noexcept
      : service_id_(0), method_id_(0), length_(0), client_id_prefix_(0),
        client_id_(0), session_id_(0), protocol_version_(0),
        interface_version_(0), message_type_(0), return_code_(0) {}

  // Access functions to extract fields in host byte order
  uint32_t service_id() const noexcept {
    // Return in host byte order use
    return boost::endian::big_to_native(service_id_);
  }

  uint32_t method_id() const noexcept {
    // Return in host byte order use
    return boost::endian::big_to_native(method_id_);
  }

  void set_method_id(const uint16_t method_id) noexcept {
    method_id_ = boost::endian::native_to_big(method_id);
  }

  // "Length field shall contain the length in byte starting from
  // Request ID/Client ID until the end of the SOME/IP message."
  uint32_t length() const noexcept {
    return boost::endian::big_to_native(length_);
  }

  void set_length(const uint32_t size) noexcept {
    length_ = boost::endian::native_to_big(size);
  }

  void increment_length(const int32_t size) noexcept {
    length_ = boost::endian::native_to_big(length() + size);
  }

  uint32_t payload_length() const noexcept {
    constexpr uint32_t HEADER_REMAINING_LENGTH = 8;
    return length() - HEADER_REMAINING_LENGTH;
  }

  uint8_t client_id_prefix() const noexcept { return client_id_prefix_; }

  uint8_t client_id() const noexcept { return client_id_; }

  uint16_t session_id() const noexcept {
    return boost::endian::big_to_native(session_id_);
  }

  void set_session_id(const uint16_t session_id) noexcept {
    session_id_ = boost::endian::native_to_big(session_id);
  }

  uint8_t protocol_version() const noexcept { return protocol_version_; }

  uint8_t interface_version() const noexcept { return interface_version_; }

  uint8_t message_type() const noexcept { return message_type_; }

  void set_message_type(const some_ip_message_type message_type) noexcept {
    message_type_ = static_cast<uint8_t>(message_type);
  }

  uint8_t return_code() const noexcept { return return_code_; }
};

// Static assert to ensure that the header is packed
static_assert(sizeof(some_ip_header) == 16, "some_ip_header is not packed");
