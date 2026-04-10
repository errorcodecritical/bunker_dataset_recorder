// Copyright (c) Sensrad 2025

#pragma once

#include <boost/numeric/conversion/cast.hpp>
#include <boost/safe_numerics/safe_integer.hpp>
#include <cassert>
#include <cstdint>

using namespace boost::safe_numerics;

class StaticThresholds {

  constexpr static float DEFAULT_THRESHOLD = 10.0f;

  // Warning: If these are dB they have to be applied after to_db.
  constexpr static float DEFAULT_NOISE_LEVEL = 0.0f;
  constexpr static float DEFAULT_BIAS_4D_TO_3D = 0.0f;

private:
  float static_threshold_;
  float noise_level_;
  float bias_4d_to_3d_;

  int32_t to_db(const float value) const noexcept {
    // TODO: Do we need to add a safe range check?
    // TODO: Should we round here, instead of truncating?

    constexpr static float POW_TO_DB_RATIO = 0.1881f; // 3/16

    const auto db = boost::numeric_cast<int>(value / POW_TO_DB_RATIO);

    return db;
  }

public:
  StaticThresholds(const float static_threshold = DEFAULT_THRESHOLD,
                   const float noise_level = DEFAULT_NOISE_LEVEL,
                   const float bias_4d_to_3d = DEFAULT_BIAS_4D_TO_3D)
      : static_threshold_{static_threshold}, noise_level_{noise_level},
        bias_4d_to_3d_{bias_4d_to_3d} {}

  ~StaticThresholds() = default;

  float static_threshold() const noexcept { return static_threshold_; }

  float noise_level() const noexcept { return noise_level_; }

  float bias_4d_to_3d() const noexcept { return bias_4d_to_3d_; }

  float threshold_3d_snr() const noexcept {
    return static_threshold_ - bias_4d_to_3d_ + noise_level_;
  }

  float threshold_4d_snr() const noexcept {
    return static_threshold_ + noise_level_;
  }

  int32_t threshold_3d_db() const noexcept { return to_db(threshold_3d_snr()); }

  int32_t threshold_4d_db() const noexcept { return to_db(threshold_4d_snr()); }

  void set_static_threshold_pow(const float new_static_threshold) noexcept {
    static_threshold_ = new_static_threshold;
  }

  void set_noise_level_pow(const float new_noise_level) noexcept {
    noise_level_ = new_noise_level;
  }

  void set_bias_4d_to_3d_pow(const float new_bias_4d_to_3d) noexcept {
    bias_4d_to_3d_ = new_bias_4d_to_3d;
  }
};
