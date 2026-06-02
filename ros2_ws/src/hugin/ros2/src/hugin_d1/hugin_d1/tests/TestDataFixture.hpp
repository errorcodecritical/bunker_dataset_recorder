// Copyright (c) Sensrad 2025

#pragma once

// STL
#include <cstdint>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "read_test_files.hpp"

class TestDataFixture : public ::testing::Test {
protected:
  // You can use the protected variables in your test case.
  std::vector<uint8_t> pc11_0_binary, pc11_52_binary, pc11_99_binary;

  Eigen::Matrix3Xf cartesian_0, cartesian_52, cartesian_99;

  Eigen::Matrix3Xf spherical_0, spherical_52, spherical_99;

  Eigen::Matrix3Xf phase_power_doppler_0, phase_power_doppler_52,
      phase_power_doppler_99;

  // Setup code (called before each test)
  void SetUp() override {
    pc11_0_binary = read_binary_file("pc11_0.bin");
    pc11_52_binary = read_binary_file("pc11_52.bin");
    pc11_99_binary = read_binary_file("pc11_99.bin");

    cartesian_0 = read_vector3f_csv_file("cartesian_0.txt");
    cartesian_52 = read_vector3f_csv_file("cartesian_52.txt");
    cartesian_99 = read_vector3f_csv_file("cartesian_99.txt");

    spherical_0 = read_vector3f_csv_file("spherical_0.txt");
    spherical_52 = read_vector3f_csv_file("spherical_52.txt");
    spherical_99 = read_vector3f_csv_file("spherical_99.txt");

    phase_power_doppler_0 = read_vector3f_csv_file("phase_power_doppler_0.txt");
    phase_power_doppler_52 =
        read_vector3f_csv_file("phase_power_doppler_52.txt");
    phase_power_doppler_99 =
        read_vector3f_csv_file("phase_power_doppler_99.txt");
  }

  // Teardown code (called after each test)
  void TearDown() override {}
};
