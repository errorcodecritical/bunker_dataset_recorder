// Copyright (c) Sensrad 2025

#pragma once

// STL
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <span>
#include <vector>

// Boost
#include <boost/numeric/conversion/cast.hpp>
#include <boost/safe_numerics/safe_integer.hpp>

using namespace boost::safe_numerics;

// Contains header
#include <PointCloud_V1_1.hpp>

// Check if float is "valid" (not nan or inf)
template <typename T> bool is_valid_float(const T val) {
  return not std::isnan(val) and not std::isinf(val);
}

// Variadic version
template <typename... Ts> bool are_valid_floats(Ts... vals) {
  // Fold-expression: applies the && check across all arguments
  return ((not std::isnan(vals) and not std::isinf(vals)) && ...);
}

// Check if value is in range [closed interval].
template <typename T>
constexpr bool in_range(const T &x, const T &lower, const T &upper) {
  return lower <= x && x <= upper;
}

// uint32 enum class
enum class PointFlags : uint32_t {
  BadRange = 1 << 0,     // 1
  BadAzimuth = 1 << 1,   // 2
  BadElevation = 1 << 2, // 4
};

struct SignedBins {
  float range;
  float azimuth;
  float elevation;
  float doppler;
  float power;
  float phase;
};

struct Point {
  // Values from radar
  float range;
  float azimuth;
  float elevation;
  float doppler;
  float power;
  float phase;

  uint32_t flags;

  void set_flags(const bool bad_range, const bool bad_azimuth,
                 const bool bad_elevation) {

    flags |= bad_range ? static_cast<uint32_t>(PointFlags::BadRange) : 0;
    flags |= bad_azimuth ? static_cast<uint32_t>(PointFlags::BadAzimuth) : 0;
    flags |=
        bad_elevation ? static_cast<uint32_t>(PointFlags::BadElevation) : 0;
  }

  bool is_bad() const noexcept { return (flags != 0); }

  // Derived values
  float x() const { return range * cos(elevation) * sin(azimuth); }

  float y() const { return range * cos(elevation) * cos(azimuth); }

  float z() const { return range * sin(elevation); }
};

class PointCloud {
private:
  PointCloud_V1_1 header_;

  // Non-owning reference to the data. You are responsible for keeping
  // the data alive while using the PointCloud.
  const std::span<const uint8_t> data_;

  // Function "pointer" to converter function dependent on point type.
  std::function<SignedBins(size_t i)> read_signed_point_;

  // Read a "normal" point
  template <typename PointType>
  SignedBins read_signed_bins(const size_t i) const noexcept;

  // Read super resolution point
  SignedBins read_superresolution_bins(const size_t i) const noexcept;

  // Convert signed point to physical point
  Point signed_point_to_physical(const SignedBins &signed_point) const noexcept;

public:
  explicit PointCloud(const std::vector<uint8_t> &data)
      : header_{}, data_(data) {

    assert(data.size() >= sizeof(PointCloud_V1_1) &&
           "Data size is smaller than the header size");

    // Copy header bytes
    std::memcpy(&header_, data_.data(), sizeof(PointCloud_V1_1));

    // Bind the correct decoder function based on the point size
    switch (header_.point_size()) {
    case sizeof(Point48):
      read_signed_point_ = std::bind(&PointCloud::read_signed_bins<Point48>,
                                     this, std::placeholders::_1);
      break;
    case sizeof(Point56):
      read_signed_point_ = std::bind(&PointCloud::read_signed_bins<Point56>,
                                     this, std::placeholders::_1);
      break;
    case sizeof(Point80):
      read_signed_point_ = std::bind(&PointCloud::read_signed_bins<Point80>,
                                     this, std::placeholders::_1);
      break;
    case sizeof(Point96):
      read_signed_point_ = std::bind(&PointCloud::read_signed_bins<Point96>,
                                     this, std::placeholders::_1);
      break;
    case sizeof(Point192):
      // Advanced point cloud is not supported
      // assert(false && "Point128 is not supported in this version of the
      // API");
      break;
    default:
      assert(false);
    }
  }

  ~PointCloud() = default;

  // Return header
  const PointCloud_V1_1 &header() const noexcept { return header_; }

  // Number of point's in the pointcloud.
  size_t num_points() const noexcept { return header_.num_points(); }

  // Read only reference to the pointcloud packet data.
  const std::span<const uint8_t> data() const noexcept { return data_; }

  // Override []
  Point operator[](const size_t i) const noexcept {
    const auto signed_point = read_signed_point_(i);
    return signed_point_to_physical(signed_point);
  }

  // Add custom iterator
  class const_iterator {
  private:
    // Store a pointer to the owning PointCloud and our index
    const PointCloud *pointcloud_ = nullptr;
    std::size_t index_ = 0;

  public:
    // Standard iterator typedefs
    using iterator_category = std::forward_iterator_tag;
    using value_type = Point; // Return type

    // Constructor
    const_iterator(const PointCloud *pc, std::size_t idx)
        : pointcloud_(pc), index_(idx) {}

    // Pre-increment
    const_iterator &operator++() {
      ++index_;
      return *this;
    }

    // Post-increment
    const_iterator operator++(int) {
      const_iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    // Equality
    bool operator==(const const_iterator &other) const {
      return pointcloud_ == other.pointcloud_ && index_ == other.index_;
    }

    bool operator!=(const const_iterator &other) const {
      return !(*this == other);
    }

    // Dereference: decode and return a point by value.
    value_type operator*() const { return (*pointcloud_)[index_]; }
  };

  const_iterator cbegin() const { return const_iterator(this, 0); }
  const_iterator cend() const {
    return const_iterator(this, header_.num_points());
  }

  const_iterator begin() const { return cbegin(); }
  const_iterator end() const { return cend(); }
};

// Implementations
template <typename PointType>
SignedBins PointCloud::read_signed_bins(const size_t i) const noexcept {

  PointType compressed_point;
  SignedBins signed_bins;

  // Copy from payload to compressed_point
  std::memcpy(&compressed_point, data_.data() + header_.point_offset(i),
              sizeof(PointType));

  // Point48, Point56, Point80, Point96 are packed like this:
  signed_bins.range = boost::numeric_cast<float>(compressed_point.range);

  signed_bins.power = boost::numeric_cast<float>(compressed_point.power);

  // Not all points have phase
  if constexpr (std::is_same_v<PointType, Point56> or
                std::is_same_v<PointType, Point96>) {
    // With phase
    signed_bins.phase = boost::numeric_cast<float>(compressed_point.phase);
  } else {
    // Without phase, set to 0
    signed_bins.phase = 0.f;
  }

  // Convert azimuth bin to signed
  const int signed_azimuth_bin =
      (-1) * (safe<int>(compressed_point.azimuth) -
              safe<int>(header_.metadata_.azimuth.fft_size() >> 1) +
              safe<int>(header_.metadata_.azimuth.padding));

  signed_bins.azimuth = boost::numeric_cast<float>(signed_azimuth_bin);

  // Convert elevation bin to signed
  const int signed_elevation_bin =
      (-1) * (safe<int>(compressed_point.elevation) -
              safe<int>(header_.metadata_.elevation.fft_size() >> 1) +
              safe<int>(header_.metadata_.elevation.padding));

  signed_bins.elevation = boost::numeric_cast<float>(signed_elevation_bin);

  // Convert doppler to signed bin
  float doppler = 0;
  const int bin = safe<int>(compressed_point.doppler);
  if (header_.metadata_.doppler.is_4d()) {
    doppler = static_cast<float>((bin + 0x800) % 0x1000);
    doppler -= 0x800;
  } else {
    doppler = static_cast<float>((bin + 0x200) % 0x400);
    doppler -= 0x200;
  }
  doppler = (-1.0f) * doppler;

  signed_bins.doppler = doppler;

  // Assert that all values are valid (not nan or inf)
  assert(are_valid_floats(signed_bins.range, signed_bins.azimuth,
                          signed_bins.elevation, signed_bins.doppler,
                          signed_bins.power, signed_bins.phase));

  return signed_bins;
}

inline Point PointCloud::signed_point_to_physical(
    const SignedBins &signed_point) const noexcept {
  Point physical_point{};

  // Range checks for range
  const float range_fft_size =
      static_cast<float>(header_.metadata_.range.fft_size());
  const bool bad_range =
      not in_range(signed_point.range, 0.f, range_fft_size - 1.f);

  // Range check for azimuth
  const float az_fft_size_2 =
      static_cast<float>(header_.metadata().azimuth.fft_size()) / 2.f;
  const bool bad_azimuth =
      not in_range(signed_point.azimuth, -az_fft_size_2, az_fft_size_2);

  // Range check for elevation
  const float el_fft_size_2 =
      static_cast<float>(header_.metadata().elevation.fft_size()) / 2.f;
  const bool bad_elevation =
      not in_range(signed_point.elevation, -el_fft_size_2, el_fft_size_2);

  physical_point.set_flags(bad_range, bad_azimuth, bad_elevation);

  physical_point.range =
      (signed_point.range - header_.metadata_.range.range_offset()) *
      header_.metadata_.range.coefficient();

  // alpha = sin(elevation)
  const float alpha =
      signed_point.elevation * header_.metadata_.elevation.coefficient();

#ifndef NDEBUG
  assert(alpha >= -1.0f && alpha <= 1.0f);
#endif
  // elevation - clamp alpha to [-1, 1] to avoid NaN from asinf
  physical_point.elevation = asinf(std::clamp(alpha, -1.0f, 1.0f));

  // beta = sin(azimuth)*cos(elevation)
  const float beta =
      signed_point.azimuth * header_.metadata_.azimuth.coefficient();
  // azimuth - clamp the ratio to [-1, 1] to avoid NaN from asinf
  const float azimuth_ratio = beta / cosf(physical_point.elevation);
#ifndef NDEBUG
  assert(azimuth_ratio >= -1.0f && azimuth_ratio <= 1.0f);
#endif
  physical_point.azimuth = asinf(std::clamp(azimuth_ratio, -1.0f, 1.0f));

  // phase
  const float PHASE_SCALE_FACTOR = 1.0f / 1024.0f;
  physical_point.phase = signed_point.phase * PHASE_SCALE_FACTOR;

  // power
  constexpr float pow_to_db_ratio = 0.1881f; // 3.f / 16.f;
  physical_point.power = signed_point.power * pow_to_db_ratio;

  // Doppler
  physical_point.doppler =
      signed_point.doppler * header_.metadata_.doppler.coefficient();

  // Debug: Check for NaN values in any field and log detailed information
  if (!are_valid_floats(physical_point.range, physical_point.azimuth,
                        physical_point.elevation, physical_point.doppler,
                        physical_point.power, physical_point.phase)) {

    std::cerr << "WARNING: NaN/Inf detected in physical_point!" << std::endl;
    std::cerr << "  Input signed_point values:" << std::endl;
    std::cerr << "    range: " << signed_point.range << std::endl;
    std::cerr << "    azimuth: " << signed_point.azimuth << std::endl;
    std::cerr << "    elevation: " << signed_point.elevation << std::endl;
    std::cerr << "    doppler: " << signed_point.doppler << std::endl;
    std::cerr << "    power: " << signed_point.power << std::endl;
    std::cerr << "    phase: " << signed_point.phase << std::endl;

    std::cerr << "  Output physical_point values:" << std::endl;
    std::cerr << "    range: " << physical_point.range
              << (std::isnan(physical_point.range) ? " [NaN]" : "")
              << (std::isinf(physical_point.range) ? " [Inf]" : "")
              << std::endl;
    std::cerr << "    azimuth: " << physical_point.azimuth
              << (std::isnan(physical_point.azimuth) ? " [NaN]" : "")
              << (std::isinf(physical_point.azimuth) ? " [Inf]" : "")
              << std::endl;
    std::cerr << "    elevation: " << physical_point.elevation
              << (std::isnan(physical_point.elevation) ? " [NaN]" : "")
              << (std::isinf(physical_point.elevation) ? " [Inf]" : "")
              << std::endl;
    std::cerr << "    doppler: " << physical_point.doppler
              << (std::isnan(physical_point.doppler) ? " [NaN]" : "")
              << (std::isinf(physical_point.doppler) ? " [Inf]" : "")
              << std::endl;
    std::cerr << "    power: " << physical_point.power
              << (std::isnan(physical_point.power) ? " [NaN]" : "")
              << (std::isinf(physical_point.power) ? " [Inf]" : "")
              << std::endl;
    std::cerr << "    phase: " << physical_point.phase
              << (std::isnan(physical_point.phase) ? " [NaN]" : "")
              << (std::isinf(physical_point.phase) ? " [Inf]" : "")
              << std::endl;

    std::cerr << "  Intermediate calculations:" << std::endl;
    std::cerr << "    alpha (sin(elevation)): " << alpha << std::endl;
    std::cerr << "    alpha clamped: " << std::clamp(alpha, -1.0f, 1.0f)
              << std::endl;
    std::cerr << "    beta (sin(azimuth)*cos(elevation)): " << beta
              << std::endl;
    std::cerr << "    beta / cos(elevation): " << azimuth_ratio << std::endl;
    std::cerr << "    azimuth_ratio clamped: "
              << std::clamp(azimuth_ratio, -1.0f, 1.0f) << std::endl;

    std::cerr << "  Coefficients:" << std::endl;
    std::cerr << "    range.coefficient: "
              << header_.metadata_.range.coefficient() << std::endl;
    std::cerr << "    azimuth.coefficient: "
              << header_.metadata_.azimuth.coefficient() << std::endl;
    std::cerr << "    elevation.coefficient: "
              << header_.metadata_.elevation.coefficient() << std::endl;
    std::cerr << "    doppler.coefficient: "
              << header_.metadata_.doppler.coefficient() << std::endl;
    std::cerr << "    range.range_offset: "
              << header_.metadata_.range.range_offset() << std::endl;

    // Mark point as bad
    physical_point.flags = static_cast<uint32_t>(PointFlags::BadRange) |
                           static_cast<uint32_t>(PointFlags::BadAzimuth) |
                           static_cast<uint32_t>(PointFlags::BadElevation);
  }

  // If a flag is set, we assume the point is "bad".
  if (!physical_point.flags) {
    assert(are_valid_floats(physical_point.range, physical_point.azimuth,
                            physical_point.elevation, physical_point.doppler,
                            physical_point.power, physical_point.phase));
  }

  return physical_point;
}
