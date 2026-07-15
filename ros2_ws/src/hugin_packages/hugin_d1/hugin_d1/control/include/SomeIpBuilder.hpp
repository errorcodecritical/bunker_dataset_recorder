// Copyright (c) Sensrad 2025

#pragma once

#include <cstdint>
#include <vector>

#include "someip_header.hpp"

class SomeIpBuilder {
private:
  some_ip_header some_ip_header_;
  std::vector<uint8_t> payload_;

public:
  SomeIpBuilder();
  ~SomeIpBuilder() = default;

  SomeIpBuilder &request(const uint16_t method_id) noexcept;

  SomeIpBuilder &session_id(const uint16_t session_id) noexcept;

  // There is a specialized template for std::vector<uint8_t>
  template <typename T> SomeIpBuilder &payload(const T &payload);

  std::vector<uint8_t> build();
};

template <typename T> SomeIpBuilder &SomeIpBuilder::payload(const T &payload) {

  // Assert copyable
  static_assert(std::is_trivially_copyable<T>::value,
                "Payload must be a trivially copyable struct");

  const uint8_t *begin = reinterpret_cast<const uint8_t *>(&payload);
  const uint8_t *end = begin + sizeof(T);

  payload_.insert(payload_.end(), begin, end);
  some_ip_header_.increment_length(sizeof(T));

  return *this;
}
