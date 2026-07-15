// Copyright (c) Sensrad 2024-2025

#include <gtest/gtest.h>

#include <DynamicThresholds.hpp>
#include <StaticThresholds.hpp>

TEST(thresholds, StaticThresholds) {

  StaticThresholds st;

  ASSERT_EQ(st.static_threshold(), 10.0f);
  ASSERT_EQ(st.noise_level(), 0.0f);
  ASSERT_EQ(st.bias_4d_to_3d(), 0.0f);

  ASSERT_EQ(st.threshold_3d_snr(), 10);
  ASSERT_EQ(st.threshold_4d_snr(), 10);

  ASSERT_EQ(st.threshold_3d_db(), 53);
  ASSERT_EQ(st.threshold_4d_db(), 53);
}

TEST(thresholds, DynamicThresholds) {

  DynamicThresholds dt;

  ASSERT_EQ(dt.basic_farfield(), 15.0f);
  ASSERT_EQ(dt.azimuth_farfield(), 15.0f);
  ASSERT_EQ(dt.elevation_farfield(), 15.0f);

  ASSERT_EQ(dt.azimuth_farfield_db(), 79);
  ASSERT_EQ(dt.azimuth_nearfield_db(), 53);

  ASSERT_EQ(dt.elevation_farfield_db(), 79);
  ASSERT_EQ(dt.elevation_nearfield_db(), 26);

  ASSERT_EQ(dt.basic_farfield_db(), 79);
  ASSERT_EQ(dt.basic_nearfield_db(), 53);

  ASSERT_EQ(dt.coarse_farfield_db(), 0);
  ASSERT_EQ(dt.coarse_nearfield_db(), 0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
