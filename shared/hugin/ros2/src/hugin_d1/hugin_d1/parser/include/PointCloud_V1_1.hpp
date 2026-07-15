// Copyright (c) Sensrad 2025

#pragma once

// STL
#include <cassert>
#include <cmath>
#include <cstdint>

// Boost endian conversion library
#include <boost/endian/conversion.hpp>

// Advanced point cloud format
// This is the wire format, the memory layout is important
struct __attribute__((packed)) Point192 {
  uint16_t range;
  int16_t doppler;
  int16_t azimuth;
  int16_t elevation;
  uint16_t power;
  int16_t phase;
  uint32_t confidence;
  uint16_t flags;
  uint16_t future_use;
  uint32_t cluster_assignment;
};

static_assert(sizeof(Point192) == 24,
              "Compiler added unexpected padding to Point192!");

// Compressed point 56 bits total = 7 bytes
// This is the wire format, the memory layout is important
struct __attribute__((packed)) Point56 {
  uint16_t phase : 11;
  uint16_t power : 10;
  uint16_t elevation : 6;
  uint16_t azimuth : 8;
  uint16_t doppler : 12;
  uint16_t range : 9;
};

static_assert(sizeof(Point56) == 7,
              "Compiler added unexpected padding to Point56!");

// Compressed point 48 bits total = 6 bytes
// This is the wire format, the memory layout is important
struct __attribute__((packed)) Point48 {
  uint16_t power : 10;
  uint16_t elevation : 6;
  uint16_t azimuth : 8;
  uint16_t doppler : 12;
  uint16_t range : 9;
  uint16_t padding : 3; // filler bits to align on byte boundary
};

static_assert(sizeof(Point48) == 6,
              "Compiler added unexpected padding to Point48!");

// Point 96 bits total = 12 bytes
// This is the wire format, the memory layout is important
struct __attribute__((packed)) Point96 {
  uint16_t range;
  uint16_t doppler;
  uint16_t azimuth;
  uint16_t elevation;
  uint16_t power;
  uint16_t phase;
};

static_assert(sizeof(Point96) == 12,
              "Compiler added unexpected padding to Point96!");

// Point 80 bits total = 10 bytes
// This is the wire format, the memory layout is important
struct __attribute__((packed)) Point80 {
  uint16_t range;
  uint16_t doppler;
  uint16_t azimuth;
  uint16_t elevation;
  uint16_t power;
};

static_assert(sizeof(Point80) == 10,
              "Compiler added unexpected padding to Point80!");

// Range metadata
struct __attribute__((packed)) Range {
  uint32_t fraction : 16;
  uint32_t integral : 10;
  uint32_t range_zoom : 6;

  float coefficient() const noexcept {
    return static_cast<float>(integral) +
           (static_cast<float>(fraction) / (1 << 16));
  }

  int zoom_factor() const noexcept { return range_zoom + 1; }

  int fft_size() const noexcept { return 128 * zoom_factor(); }

  float range_offset() const noexcept { return (zoom_factor() - 1.f) / 2.f; }
};

static_assert(sizeof(Range) == 4,
              "Compiler added unexpected padding to Range!");

// Doppler metadata
struct __attribute__((packed)) Doppler {
  uint32_t fraction : 16;
  uint32_t integral : 14;
  uint32_t is_dynamic : 1;
  uint32_t is_fine : 1;

  float coefficient() const noexcept {
    return static_cast<float>(integral) +
           (static_cast<float>(fraction) / (1 << 16));
  }

  bool is_4d() const noexcept { return static_cast<bool>(is_fine); }
};

static_assert(sizeof(Doppler) == 4,
              "Compiler added unexpected padding to Doppler!");

// Azimuth metadata
struct __attribute__((packed)) Azimuth {
  uint32_t fraction : 16;
  uint32_t padding : 6;
  uint32_t fft_range : 10;

  float coefficient() const noexcept {
    return static_cast<float>(fraction) / (1 << 16);
  }

  uint32_t fft_padding() const noexcept { return padding; }

  uint32_t fft_size() const noexcept { return fft_range; }
};

static_assert(sizeof(Azimuth) == 4,
              "Compiler added unexpected padding to Azimuth!");

// Elevation metadata
struct __attribute__((packed)) Elevation {
  uint32_t fraction : 16;
  uint32_t padding : 6;
  uint32_t fft_range : 10;

  float coefficient() const noexcept {
    return static_cast<float>(fraction) / (1 << 16);
  }

  uint32_t fft_padding() const noexcept { return padding; }

  uint32_t fft_size() const noexcept { return fft_range; }
};

static_assert(sizeof(Elevation) == 4,
              "Compiler added unexpected padding to Elevation!");

// Metadata container
struct __attribute__((packed)) Metadata {
  Range range;
  Doppler doppler;
  Azimuth azimuth;
  Elevation elevation;
};

static_assert(sizeof(Metadata) == 16,
              "Compiler added unexpected padding to Metadata!");

struct __attribute__((packed)) PointCloud_V1_1 {
  uint16_t prefix_; // 0xa55a
  uint16_t type_;   // 0x0d = normal point cloud, 0x28 additional data header
                    // before points.
  uint32_t length_; // Packet length in bytes including header
  uint32_t time_lsb_;
  uint32_t time_msb_;
  uint16_t frame_counter_;
  uint16_t message_number_;
  uint8_t last_packet_;
  uint8_t trgt_fmt_frm_type_;
  uint16_t crd_count_;
  Metadata metadata_;
  uint16_t header_crc_;
  uint16_t payload_crc_;

  static bool is_out_of_range(const float val, const float min_val,
                              const float max_val) noexcept {
    return (val < min_val) or (val > max_val);
  }

  uint16_t prefix() const noexcept {
    return boost::endian::big_to_native(prefix_);
  }

  uint16_t type() const noexcept { return type_; }

  uint32_t length() const noexcept { return length_; }

  uint32_t time_lsb() const noexcept { return time_lsb_; }

  uint32_t time_msb() const noexcept { return time_msb_; }

  uint16_t frame_counter() const noexcept { return frame_counter_; }

  uint16_t message_number() const noexcept { return message_number_; }

  uint16_t crd_count() const noexcept { return crd_count_; }

  Metadata metadata() const noexcept { return metadata_; }

  uint64_t time_ms() const noexcept {
    uint64_t time_ms = 0;
    time_ms = static_cast<uint64_t>(time_msb()) << 32;
    time_ms |= static_cast<uint64_t>(time_lsb());

    return time_ms;
  }

  bool is_last_packet() const noexcept {
    return static_cast<bool>(last_packet_);
  }

  uint8_t target_format() const noexcept {
    return (((trgt_fmt_frm_type_) >> 6) & 0x03);
  }

  bool is_advanced_pc() const noexcept {
    return (type() == 0x44 || type() == 0x45);
  }

  bool is_pointcloud() const noexcept { return type() == 0x0d; }

  // Get the size of the point/target in bytes.
  size_t point_size() const noexcept {

    if (is_advanced_pc()) {
      return sizeof(Point192);
    } else if (is_pointcloud()) {
      switch (target_format()) {
      case 0:
        return sizeof(Point56);
      case 1:
        return sizeof(Point48);
      case 2:
        return sizeof(Point96);
      case 3:
        return sizeof(Point80);
      default:
        assert(false && "Unknown target format!");
        break;
      }
    }

    return 0;
  }

  // Get the expected number of points in the payload.
  size_t num_points() const noexcept {
    const auto payload_bytes = length() - sizeof(PointCloud_V1_1);
    // Guards against division by zero.
    return point_size() == 0 ? 0 : payload_bytes / point_size();
  }

  // Get the byte offset of the start of the payload.
  size_t payload_offset() const noexcept { return sizeof(PointCloud_V1_1); }

  // Get the byte offset of the i-th point in the payload.
  size_t point_offset(const size_t i) const noexcept {
    const size_t offset = payload_offset() + i * point_size();
    return offset;
  }
};

static_assert(sizeof(PointCloud_V1_1) == 44,
              "Compiler added unexpected padding to PointCloud_V1_1!");
