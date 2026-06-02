// Copyright (c) Sensrad 2025

#pragma once

#include <boost/endian/conversion.hpp>
#include <cstdint>

#define ARBE_GENERIC_HEADER_SIGNATURE 0xaa55aa55

struct __attribute__((packed)) ArbeGenericHeader {
  uint32_t signature_; // 0xaa55aa55
  uint32_t version_;
  uint32_t header_length_;
  uint32_t data_length_;
  uint32_t seq_num_; // increment every packet type
  uint32_t time_lsb_;
  uint32_t time_msb_;
  uint32_t msg_type_;
  uint32_t msg_num_;
  uint32_t msg_is_last_;
  uint32_t context_id_;  // increment every context
  uint32_t context_num_; // increment every packet type
  uint32_t context_is_last_;
  uint32_t actual_data_len_; // this is used for first packet that contain more
                             // then one data type
  uint32_t header_crc_; // CRC of the header up to usHeaderCrc field and data
  uint32_t data_crc_;   // CRC of the header up to usHeaderCrc field and data

  uint32_t signature() const noexcept {
    return boost::endian::big_to_native(signature_);
  }

  uint32_t version() const noexcept {
    return boost::endian::big_to_native(version_);
  }

  uint32_t header_length() const noexcept {
    return boost::endian::big_to_native(header_length_);
  }

  uint32_t data_length() const noexcept {
    return boost::endian::big_to_native(data_length_);
  }

  uint32_t seq_num() const noexcept {
    return boost::endian::big_to_native(seq_num_);
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

  uint32_t msg_type() const noexcept {
    return boost::endian::big_to_native(msg_type_);
  }

  uint32_t msg_num() const noexcept {
    return boost::endian::big_to_native(msg_num_);
  }

  uint32_t msg_is_last() const noexcept {
    return boost::endian::big_to_native(msg_is_last_);
  }

  uint32_t context_id() const noexcept {
    return boost::endian::big_to_native(context_id_);
  }

  uint32_t context_num() const noexcept {
    return boost::endian::big_to_native(context_num_);
  }

  uint32_t context_is_last() const noexcept {
    return boost::endian::big_to_native(context_is_last_);
  }

  uint32_t actual_data_len() const noexcept {
    return boost::endian::big_to_native(actual_data_len_);
  }

  uint32_t header_crc() const noexcept {
    return boost::endian::big_to_native(header_crc_);
  }

  uint32_t data_crc() const noexcept {
    return boost::endian::big_to_native(data_crc_);
  }
};

// Status assert size
static_assert(sizeof(ArbeGenericHeader) == 64,
              "ArbeGenericHeader is not 64 bytes");
