// Copyright (c) Sensrad 2023-2024
#pragma once

template <typename T> class FirstOrderDiffFilter {
  T alpha_;
  T prev_val_;
  T prev_output_ = 0;
  bool first_ = true;

public:
  explicit FirstOrderDiffFilter(const T alpha) {
    alpha_ = alpha;
    prev_val_ = 0;
  }

  void reset() noexcept { first_ = true; }

  T filter(const T val) noexcept {
    if (first_) {
      first_ = false;
      prev_val_ = val;
      return 0;
    }

    T diff = val - prev_val_;
    prev_val_ = val;

    T output = alpha_ * diff + (1 - alpha_) * prev_output_;
    prev_output_ = output;

    return output;
  }
};
