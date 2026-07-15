// Copyright (c) Sensrad 2024-2025

#include <gtest/gtest.h>

#include <PointCloud.hpp>

#include <algorithm>
#include <vector>

#include <Eigen/Dense>

#include "TestDataFixture.hpp"

// You can use the protected variables pc11_0, pc11_52, pc11_99 and
// cartesian_0, cartesian_52, cartesian_99 in your tests. They are
// exposed through TestDataFixture.

TEST_F(TestDataFixture, Read) {
  // Check that the expected file size was read
  ASSERT_EQ(pc11_0_binary.size(), 17732);
  ASSERT_EQ(pc11_52_binary.size(), 16172);
  ASSERT_EQ(pc11_99_binary.size(), 16004);

  // Check that the expected number of cols was read
  ASSERT_EQ(cartesian_0.cols(), 1474);
  ASSERT_EQ(cartesian_52.cols(), 1344);
  ASSERT_EQ(cartesian_99.cols(), 1330);

  // Check that the expected number of cols was read
  ASSERT_EQ(spherical_0.cols(), 1474);
  ASSERT_EQ(spherical_52.cols(), 1344);
  ASSERT_EQ(spherical_99.cols(), 1330);

  // Check that the expected number of cols was read
  ASSERT_EQ(phase_power_doppler_0.cols(), 1474);
  ASSERT_EQ(phase_power_doppler_52.cols(), 1344);
  ASSERT_EQ(phase_power_doppler_99.cols(), 1330);
}

// Test that the number of points in the header is correct.
TEST_F(TestDataFixture, NumTargets) {

  PointCloud pc_0(pc11_0_binary);
  ASSERT_EQ(pc_0.header().num_points(), cartesian_0.cols());

  PointCloud pc_52(pc11_52_binary);
  ASSERT_EQ(pc_52.header().num_points(), cartesian_52.cols());

  PointCloud pc_99(pc11_99_binary);
  ASSERT_EQ(pc_99.header().num_points(), cartesian_99.cols());

  ASSERT_TRUE(true);
}

// Read points using std::transform and check that the values are
// correct.
TEST_F(TestDataFixture, Cartesian0) {

  const float max_abs_error = 1e-4;

  PointCloud pc_0(pc11_0_binary);

  std::vector<Eigen::Vector3f> pc_0_cartesian;
  std::transform(
      pc_0.cbegin(), pc_0.cend(), std::back_inserter(pc_0_cartesian),
      [](const Point &p) { return Eigen::Vector3f(p.x(), p.y(), p.z()); });

  ASSERT_EQ(pc_0.header().num_points(), pc_0_cartesian.size());

  const auto pc_0_mat = to_matrix(pc_0_cartesian);

  ASSERT_EQ(pc_0_mat.cols(), cartesian_0.cols());

  const auto max_abs_error_cartesian =
      (pc_0_mat - cartesian_0).cwiseAbs().maxCoeff();

  ASSERT_LE(max_abs_error_cartesian, max_abs_error);
}

// Read points using std::transform and check that the values are
// correct.
TEST_F(TestDataFixture, Spherical0) {

  PointCloud pc_0(pc11_0_binary);

  std::vector<Eigen::Vector3f> pc_0_spherical;
  std::transform(pc_0.cbegin(), pc_0.cend(), std::back_inserter(pc_0_spherical),
                 [](const Point &p) {
                   return Eigen::Vector3f(p.range, p.azimuth, p.elevation);
                 });

  ASSERT_EQ(pc_0.header().num_points(), pc_0_spherical.size());

  const auto pc_0_mat = to_matrix(pc_0_spherical);

  ASSERT_EQ(pc_0_mat.cols(), spherical_0.cols());
  ASSERT_EQ(pc_0_mat.rows(), spherical_0.rows());

  const auto norm = (pc_0_mat - spherical_0).norm();

  ASSERT_LE(1e-4, norm);
}

// Read points using [] and check that the values are correct.
TEST_F(TestDataFixture, Cartesian52) {

  PointCloud pc_52(pc11_52_binary);

  Eigen::Matrix3Xf pc_52_mat(3, pc_52.header().num_points());

  // make sure they have same shape
  ASSERT_EQ(pc_52_mat.rows(), cartesian_52.rows());
  ASSERT_EQ(pc_52_mat.cols(), cartesian_52.cols());

  for (size_t i = 0; i < pc_52.header().num_points(); ++i) {
    const auto p = pc_52[i];
    pc_52_mat.col(i) << p.x(), p.y(), p.z();
  }

  const auto norm = (pc_52_mat - cartesian_52).norm();

  ASSERT_LE(1e-4, norm);
}

// Read points using [] and check that the values are correct.
TEST_F(TestDataFixture, Spherical52) {

  PointCloud pc_52(pc11_52_binary);

  Eigen::Matrix3Xf pc_52_mat(3, pc_52.header().num_points());

  for (size_t i = 0; i < pc_52.header().num_points(); ++i) {
    const auto p = pc_52[i];
    pc_52_mat.col(i) << p.range, p.azimuth, p.elevation;
  }

  const float max_abs_error = 1e-4;

  const auto max_abs_error_spherical =
      (pc_52_mat - spherical_52).cwiseAbs().maxCoeff();

  ASSERT_EQ(pc_52_mat.cols(), spherical_52.cols());

  ASSERT_LE(max_abs_error_spherical, max_abs_error);
}

// Read points using [] and check that the values are correct.
TEST_F(TestDataFixture, Cartesian99) {

  const float max_abs_error = 1e-4;

  PointCloud pc_99(pc11_99_binary);

  Eigen::Matrix3Xf pc_99_mat(3, pc_99.num_points());

  for (size_t i = 0; i < pc_99.num_points(); ++i) {
    const auto point = pc_99[i];
    pc_99_mat.col(i) << point.x(), point.y(), point.z();
  }

  ASSERT_EQ(pc_99_mat.cols(), cartesian_99.cols());

  const auto max_abs_error_cartesian =
      (pc_99_mat - cartesian_99).cwiseAbs().maxCoeff();

  ASSERT_LE(max_abs_error_cartesian, max_abs_error);
}

// Test phase, power and doppler values for 99th point cloud
TEST_F(TestDataFixture, PhasePowerDoppler0) {

  // The output format was change to scale by 1/1024 but the test data
  // is still not scaled.
  const float PHASE_SCALE_FACTOR = 1024.f;

  const float max_abs_error = 1e-3;

  PointCloud pc_0(pc11_0_binary);

  Eigen::Matrix3Xf pc_00_mat(3, pc_0.num_points());

  for (size_t i = 0; i < pc_0.num_points(); ++i) {
    const auto point = pc_0[i];
    pc_00_mat.col(i) << point.phase * PHASE_SCALE_FACTOR, point.power,
        point.doppler;
  }

  ASSERT_EQ(pc_00_mat.cols(), phase_power_doppler_0.cols());

  const auto max_abs_error_ppd =
      (pc_00_mat - phase_power_doppler_0).cwiseAbs().maxCoeff();

  ASSERT_LE(max_abs_error_ppd, max_abs_error);
}

// Test phase, power and doppler values for 99th point cloud
TEST_F(TestDataFixture, PhasePowerDoppler52) {

  // The output format was change to scale by 1/1024 but the test data
  // is still not scaled.
  const float PHASE_SCALE_FACTOR = 1024.f;

  const float max_abs_error = 1e-3;

  PointCloud pc_52(pc11_52_binary);

  Eigen::Matrix3Xf pc_52_mat(3, pc_52.num_points());

  for (size_t i = 0; i < pc_52.num_points(); ++i) {
    const auto point = pc_52[i];
    pc_52_mat.col(i) << point.phase * PHASE_SCALE_FACTOR, point.power,
        point.doppler;
  }

  ASSERT_EQ(pc_52_mat.cols(), phase_power_doppler_52.cols());

  const auto max_abs_error_ppd =
      (pc_52_mat - phase_power_doppler_52).cwiseAbs().maxCoeff();

  ASSERT_LE(max_abs_error_ppd, max_abs_error);
}

// Test phase, power and doppler values for 99th point cloud
TEST_F(TestDataFixture, PhasePowerDoppler99) {

  // The output format was change to scale by 1/1024 but the test data
  // is still not scaled.
  const float PHASE_SCALE_FACTOR = 1024.f;

  const float max_abs_error = 1e-3;

  PointCloud pc_99(pc11_99_binary);

  Eigen::Matrix3Xf pc_99_mat(3, pc_99.num_points());

  for (size_t i = 0; i < pc_99.num_points(); ++i) {
    const auto point = pc_99[i];
    pc_99_mat.col(i) << point.phase * PHASE_SCALE_FACTOR, point.power,
        point.doppler;
  }

  ASSERT_EQ(pc_99_mat.cols(), phase_power_doppler_99.cols());

  const auto max_abs_error_ppd =
      (pc_99_mat - phase_power_doppler_99).cwiseAbs().maxCoeff();

  ASSERT_LE(max_abs_error_ppd, max_abs_error);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
