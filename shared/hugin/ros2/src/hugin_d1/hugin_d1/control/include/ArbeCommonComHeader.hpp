// Copyright (c) Sensrad 2025

#pragma once

#include <boost/endian/conversion.hpp>
#include <cstdint>

// This is the OLD HEADER FORMAT. This is currently only used by the log client.

// Mostly little endian
// We do the prefix check in big-endian for consistency.
struct __attribute__((packed)) ArbeCommonComHeader {

  uint16_t prefix_; // 0xa55a (little endian)
  uint16_t type_;
  uint32_t length_; // data only
  uint32_t time_lsb_;
  uint32_t time_msb_;
  uint32_t message_number_;
  uint16_t crc_16_; // header+data
  uint16_t reserved_;

  uint16_t prefix() const noexcept {
    return boost::endian::big_to_native(prefix_);
  }

  uint16_t type() const noexcept { return boost::endian::big_to_native(type_); }

  uint32_t length() const noexcept {
    return boost::endian::big_to_native(length_);
  }

  uint32_t time_lsb() const noexcept {
    return boost::endian::big_to_native(time_lsb_);
  }

  uint32_t time_msb() const noexcept {
    return boost::endian::big_to_native(time_msb_);
  }

  uint64_t time_ms() const noexcept {
    uint64_t time_ms = 0;
    time_ms = static_cast<uint64_t>(time_msb()) << 32;
    time_ms |= static_cast<uint64_t>(time_lsb());

    return time_ms;
  }

  uint32_t message_number() const noexcept {
    return boost::endian::big_to_native(message_number_);
  }

  uint16_t crc_16() const noexcept {
    return boost::endian::big_to_native(crc_16_);
  }

  uint16_t reserved() const noexcept {
    return boost::endian::big_to_native(reserved_);
  }
};
