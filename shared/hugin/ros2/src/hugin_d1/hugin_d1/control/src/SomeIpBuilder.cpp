// Copyright (c) Sensrad 2025

#include <SomeIpBuilder.hpp>
#include <someip_header.hpp>

#include <someip_typedef.hpp>

#include <boost/endian/conversion.hpp>

SomeIpBuilder::SomeIpBuilder()
    : some_ip_header_(SOMEIP_SERVICE_ID, 0, 8, SOMEIP_CLIENT_ID_PREFIX,
                      SOMEIP_CLIENT_ID, 0, SOMEIP_PROTOCOL_VERSION,
                      SOMEIP_INTERFACE_VERSION,
                      static_cast<uint8_t>(some_ip_message_type::request), 0) {}

SomeIpBuilder &SomeIpBuilder::request(const uint16_t method_id) noexcept {

  some_ip_header_.set_message_type(some_ip_message_type::request);
  some_ip_header_.set_method_id(method_id);

  return *this;
}

SomeIpBuilder &SomeIpBuilder::session_id(const uint16_t session_id) noexcept {
  some_ip_header_.set_session_id(session_id);
  return *this;
}

template <>
SomeIpBuilder &SomeIpBuilder::payload<std::vector<uint8_t>>(
    const std::vector<uint8_t> &payload) {

  // Append bytes
  payload_.insert(payload_.end(), payload.begin(), payload.end());
  // Update length
  some_ip_header_.increment_length(payload.size());

  return *this;
}

std::vector<uint8_t> SomeIpBuilder::build() {

  std::vector<uint8_t> buffer;

  const uint8_t *header_begin =
      reinterpret_cast<const uint8_t *>(&some_ip_header_);
  const uint8_t *header_end = header_begin + sizeof(some_ip_header);

  // Append header bytes
  buffer.insert(buffer.end(), header_begin, header_end);
  // Append payload bytes
  buffer.insert(buffer.end(), payload_.begin(), payload_.end());

  return buffer;
}
