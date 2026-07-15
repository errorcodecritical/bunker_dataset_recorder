// Copyright (c) Sensrad 2025

#pragma once

#include <algorithm>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/safe_numerics/safe_integer.hpp>
#include <cassert>
#include <cstdint>

using namespace boost::safe_numerics;

class DynamicThresholds {

  constexpr static float DEFAULT_BASIC_FARFIELD = 15.0f;
  constexpr static float DEFAULT_AZIMUTH_FARFIELD = 15.0f;
  constexpr static float DEFAULT_ELEVATION_FARFIELD = 15.0f;

  constexpr static float DEFAULT_COARSE_FARFIELD = 0.0f;
  constexpr static float DEFAULT_COARSE_NEARFIELD = 0.0f;

  constexpr static float DEFAULT_NEARFIELD_BACKOFF_BASIC = 5.0f;
  constexpr static float DEFAULT_NEARFIELD_BACKOFF_AZIMUTH = 5.0f;
  constexpr static float DEFAULT_NEARFIELD_BACKOFF_ELEVATION = 10.0f;

  constexpr static float MIN_THRESHOLD = 1.0f;
  constexpr static float MAX_THRESHOLD = 20.0f;

  float basic_farfield_;
  float azimuth_farfield_;
  float elevation_farfield_;

  float coarse_farfield_;
  float coarse_nearfield_;

  float nearfield_backoff_basic_;
  float nearfield_backoff_azimuth_;
  float nearfield_backoff_elevation_;

  // TODO: would it be better with a better dB approximation or just
  // using log?  What happens at the radar side?
  uint32_t to_db(const float value) const noexcept {
    // TODO: Do we need to add a safe range check?
    // TODO: Should we round here, instead of truncating?

    static constexpr float POW_TO_DB_RATIO = 0.1881f; // 3/16

    const auto db = boost::numeric_cast<int>(value / POW_TO_DB_RATIO);
    assert(db >= 0);

    return db;
  }

public:
  DynamicThresholds(
      const float basic_farfield = DEFAULT_BASIC_FARFIELD,
      const float azimuth_farfield = DEFAULT_AZIMUTH_FARFIELD,
      const float elevation_farfield = DEFAULT_ELEVATION_FARFIELD,
      const float coarse_farfield = DEFAULT_COARSE_FARFIELD,
      const float coarse_nearfield = DEFAULT_COARSE_NEARFIELD,
      const float nearfield_backoff_basic = DEFAULT_NEARFIELD_BACKOFF_BASIC,
      const float nearfield_backoff_azimuth = DEFAULT_NEARFIELD_BACKOFF_AZIMUTH,
      const float nearfield_backoff_elevation =
          DEFAULT_NEARFIELD_BACKOFF_ELEVATION)
      : basic_farfield_{basic_farfield}, azimuth_farfield_{azimuth_farfield},
        elevation_farfield_{elevation_farfield},
        coarse_farfield_{coarse_farfield}, coarse_nearfield_{coarse_nearfield},
        nearfield_backoff_basic_(nearfield_backoff_basic),
        nearfield_backoff_azimuth_{nearfield_backoff_azimuth},
        nearfield_backoff_elevation_{nearfield_backoff_elevation} {}

  ~DynamicThresholds() = default;

  float basic_farfield() const noexcept { return basic_farfield_; }

  float azimuth_farfield() const noexcept { return azimuth_farfield_; }

  float elevation_farfield() const noexcept { return elevation_farfield_; }

  uint32_t azimuth_farfield_db() const noexcept {
    return to_db(azimuth_farfield_);
  }

  uint32_t azimuth_nearfield_db() const noexcept {

    // TODO: warning about clamping
    const float value =
        std::clamp(azimuth_farfield_ - nearfield_backoff_azimuth_,
                   MIN_THRESHOLD, MAX_THRESHOLD);

    return to_db(value);
  }

  uint32_t elevation_farfield_db() const noexcept {
    return to_db(elevation_farfield_);
  }

  uint32_t elevation_nearfield_db() const noexcept {

    // TODO: warning about clamping
    const float value =
        std::clamp(elevation_farfield_ - nearfield_backoff_elevation_,
                   MIN_THRESHOLD, MAX_THRESHOLD);

    return to_db(value);
  }

  uint32_t basic_farfield_db() const noexcept { return to_db(basic_farfield_); }

  uint32_t basic_nearfield_db() const noexcept {

    // TODO: warning about clamping
    const float value = std::clamp(basic_farfield_ - nearfield_backoff_basic_,
                                   MIN_THRESHOLD, MAX_THRESHOLD);

    return to_db(value);
  }

  uint32_t coarse_farfield_db() const noexcept {
    return to_db(coarse_farfield_);
  }

  uint32_t coarse_nearfield_db() const noexcept {
    return to_db(coarse_nearfield_);
  }

  void set_basic_farfield(const float new_basic_farfield) noexcept {
    basic_farfield_ = new_basic_farfield;
  }

  void set_azimuth_farfield(const float new_azimuth_farfield) noexcept {
    azimuth_farfield_ = new_azimuth_farfield;
  }

  void set_elevation_farfield(const float new_elevation_farfield) noexcept {
    elevation_farfield_ = new_elevation_farfield;
  }

  void set_coarse_farfield(const float new_coarse_farfield) noexcept {
    coarse_farfield_ = new_coarse_farfield;
  }

  void set_coarse_nearfield(const float new_coarse_nearfield) noexcept {
    coarse_nearfield_ = new_coarse_nearfield;
  }
};
