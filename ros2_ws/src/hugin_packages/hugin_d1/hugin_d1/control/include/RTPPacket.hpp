// Copyright (c) Sensrad 2025

#pragma once

#include <cstdint>
#include <vector>

#include <boost/endian/conversion.hpp>

// #define RDR_MAX_UDP_PACKET_SIZE (1472) //this is our current udp limitation
// of single packet //(0x800-0x40) #define RDR_MAX_TCP_PACKET_SIZE (1460) //this
// is our current udp limitation of single packet //(0x800-0x40) #define
// RDR_MAX_RTP_PACKET_SIZE (1460) //this is our current udp limitation of single
// packet //(0x800-0x40)

// All packets sent from Hugin are 1472 bytes, even if the actual valid data is
// smaller.
constexpr size_t kMAX_RTP_PACKET_SIZE = 1472;

struct __attribute__((packed)) RTPHeader {
  uint8_t version_ : 2;
  uint8_t padding_ : 1;
  uint8_t extension_ : 1;
  uint8_t csrc_count_ : 4;
  uint8_t marker_ : 1;
  uint8_t payload_type_ : 7;
  uint16_t sequence_number_ : 16;
  uint32_t timestamp_ : 32;
  uint32_t ssrc_ : 32;

  uint8_t version() const noexcept { return version_; }

  uint8_t extension() const noexcept { return extension_; }

  uint8_t csrc_count() const noexcept { return csrc_count_; }

  uint8_t marker() const noexcept { return marker_; }

  // Always 63
  uint8_t payload_type() const noexcept { return payload_type_; }

  uint16_t sequence_number() const noexcept {
    return boost::endian::big_to_native(sequence_number_);
  }

  uint32_t timestamp() const noexcept {
    return boost::endian::big_to_native(timestamp_);
  }

  // SSRC encoded data len
  uint32_t ssrc_data_len() const noexcept {
    // Arbe uses the SSRC to encode data_len,frame_id and is_last
    const uint32_t ssrc = boost::endian::big_to_native(ssrc_);
    return ssrc & 0x000007ff;
  }

  // SSRC encoded frame id
  uint32_t ssrc_frame_id() const noexcept {
    // Arbe uses the SSRC to encode data_len,frame_id and is_last
    const uint32_t ssrc = boost::endian::big_to_native(ssrc_);
    return (ssrc & 0xffff0000) >> 16;
  }

  // SSRC is last
  uint32_t ssrc_is_last() const noexcept {
    // Arbe uses the SSRC to encode data_len,frame_id and is_last
    const uint32_t ssrc = boost::endian::big_to_native(ssrc_);
    return (ssrc & 0x00008000) >> 15;
  }
};

// Static assert to ensure that the header is packed and no padding is
// added by the compiler
static_assert(sizeof(RTPHeader) == 12, "RTPHeader is not 12 bytes");

struct __attribute__((packed)) RTPPacket {
  RTPHeader header_;
  uint8_t payload_[kMAX_RTP_PACKET_SIZE - sizeof(RTPHeader)];

  const RTPHeader &header() const noexcept { return header_; }

  // Returns the payload as a vector
  // Currently this is always the whole packet
  std::vector<uint8_t> payload() const noexcept {
    return std::vector<uint8_t>(payload_, payload_ + sizeof(payload_));
  }
};

// Static assert to ensure that the header is packed and no padding is
// added by the compiler
static_assert(sizeof(RTPPacket) == 1472, "RTPPacket is not 1472 bytes");
