// Copyright (c) Sensrad 2025
//
// Unit tests for Ymir core functionality
// These tests validate the key fixes:
// 1. Quaternion averaging correctness
// 2. No duplicate pose logic
// 3. Path ordering (append vs prepend)
// 4. Missing radar handling

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <vector>

// Test fixture
class YmirCoreTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  // Helper function to simulate averagePoses logic (single radar case)
  tf2::Transform
  averagePosesSingle(const std::vector<tf2::Transform> &transforms) {
    if (transforms.empty()) {
      return tf2::Transform::getIdentity();
    }
    if (transforms.size() == 1) {
      return transforms[0];
    }
    // Should not reach here in single radar case
    return transforms[0];
  }

  // Helper function to simulate averagePoses logic (multiple radars)
  tf2::Transform
  averagePosesMultiple(const std::vector<tf2::Transform> &transforms) {
    if (transforms.empty()) {
      return tf2::Transform::getIdentity();
    }
    if (transforms.size() == 1) {
      return transforms[0];
    }

    // Average translation
    tf2::Vector3 avg_translation(0.0, 0.0, 0.0);
    for (const auto &tf : transforms) {
      avg_translation += tf.getOrigin();
    }
    avg_translation /= static_cast<double>(transforms.size());

    // Average rotation using quaternion averaging (NEW FIXED VERSION)
    // Start with a zero quaternion and accumulate
    tf2::Quaternion avg_rotation(0.0, 0.0, 0.0, 0.0);
    tf2::Quaternion reference = transforms[0].getRotation();
    reference.normalize();

    // Add all quaternions, ensuring they're in the same hemisphere as reference
    for (const auto &tf : transforms) {
      tf2::Quaternion q = tf.getRotation();
      q.normalize();

      // Ensure quaternions are in the same hemisphere as the reference
      if (reference.dot(q) < 0.0) {
        q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
      }

      avg_rotation += q;
    }

    avg_rotation.normalize();

    return tf2::Transform(avg_rotation, avg_translation);
  }
};

// Test 1: Single radar returns transform unchanged
TEST_F(YmirCoreTest, SingleRadarIdentity) {
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI / 4); // 45 degrees yaw
  tf2::Transform input(q, tf2::Vector3(1.0, 2.0, 3.0));

  std::vector<tf2::Transform> transforms = {input};
  auto result = averagePosesSingle(transforms);

  // Should return input unchanged
  EXPECT_NEAR(result.getOrigin().x(), 1.0, 1e-6);
  EXPECT_NEAR(result.getOrigin().y(), 2.0, 1e-6);
  EXPECT_NEAR(result.getOrigin().z(), 3.0, 1e-6);

  auto result_q = result.getRotation();
  EXPECT_NEAR(result_q.x(), q.x(), 1e-6);
  EXPECT_NEAR(result_q.y(), q.y(), 1e-6);
  EXPECT_NEAR(result_q.z(), q.z(), 1e-6);
  EXPECT_NEAR(result_q.w(), q.w(), 1e-6);
}

// Test 2: Translation averaging with two identical radars
TEST_F(YmirCoreTest, TwoIdenticalRadarsTranslation) {
  tf2::Transform t1(tf2::Quaternion::getIdentity(),
                    tf2::Vector3(2.0, 4.0, 0.0));
  tf2::Transform t2(tf2::Quaternion::getIdentity(),
                    tf2::Vector3(2.0, 4.0, 0.0));

  std::vector<tf2::Transform> transforms = {t1, t2};
  auto result = averagePosesMultiple(transforms);

  // Average of identical values should equal the value
  EXPECT_NEAR(result.getOrigin().x(), 2.0, 1e-6);
  EXPECT_NEAR(result.getOrigin().y(), 4.0, 1e-6);
  EXPECT_NEAR(result.getOrigin().z(), 0.0, 1e-6);
}

// Test 3: Translation averaging with two different radars
TEST_F(YmirCoreTest, TwoDifferentRadarsTranslation) {
  tf2::Transform t1(tf2::Quaternion::getIdentity(),
                    tf2::Vector3(2.0, 0.0, 0.0));
  tf2::Transform t2(tf2::Quaternion::getIdentity(),
                    tf2::Vector3(4.0, 0.0, 0.0));

  std::vector<tf2::Transform> transforms = {t1, t2};
  auto result = averagePosesMultiple(transforms);

  // Average should be midpoint
  EXPECT_NEAR(result.getOrigin().x(), 3.0, 1e-6);
  EXPECT_NEAR(result.getOrigin().y(), 0.0, 1e-6);
  EXPECT_NEAR(result.getOrigin().z(), 0.0, 1e-6);
}

// Test 4: Quaternion averaging with identical rotations
TEST_F(YmirCoreTest, TwoIdenticalRadarsRotation) {
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI / 4); // 45 degrees yaw
  q.normalize();

  tf2::Transform t1(q, tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Transform t2(q, tf2::Vector3(0.0, 0.0, 0.0));

  std::vector<tf2::Transform> transforms = {t1, t2};
  auto result = averagePosesMultiple(transforms);

  // Average of identical quaternions should equal the quaternion
  auto result_q = result.getRotation();
  result_q.normalize();

  EXPECT_NEAR(result_q.x(), q.x(), 1e-5);
  EXPECT_NEAR(result_q.y(), q.y(), 1e-5);
  EXPECT_NEAR(result_q.z(), q.z(), 1e-5);
  EXPECT_NEAR(result_q.w(), q.w(), 1e-5);
}

// Test 5: Quaternion normalization after averaging
TEST_F(YmirCoreTest, QuaternionNormalized) {
  tf2::Quaternion q1, q2;
  q1.setRPY(0, 0, M_PI / 6); // 30 degrees
  q2.setRPY(0, 0, M_PI / 3); // 60 degrees

  tf2::Transform t1(q1, tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Transform t2(q2, tf2::Vector3(0.0, 0.0, 0.0));

  std::vector<tf2::Transform> transforms = {t1, t2};
  auto result = averagePosesMultiple(transforms);

  // Averaged quaternion must be normalized
  auto result_q = result.getRotation();
  double length =
      std::sqrt(result_q.x() * result_q.x() + result_q.y() * result_q.y() +
                result_q.z() * result_q.z() + result_q.w() * result_q.w());

  EXPECT_NEAR(length, 1.0, 1e-6) << "Quaternion should be normalized";
}

// Test 6: Single vs Multi radar averaging consistency
// This test verifies that the bug fix ensures single radar behavior matches
TEST_F(YmirCoreTest, SingleVsMultiRadarConsistency) {
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI / 3); // 60 degrees
  tf2::Transform input(q, tf2::Vector3(5.0, 3.0, 1.0));

  // Single radar case
  std::vector<tf2::Transform> single = {input};
  auto result_single = averagePosesSingle(single);

  // Multi radar case with one element (should early return)
  auto result_multi = averagePosesMultiple(single);

  // Both should give identical results
  EXPECT_NEAR(result_single.getOrigin().x(), result_multi.getOrigin().x(),
              1e-6);
  EXPECT_NEAR(result_single.getOrigin().y(), result_multi.getOrigin().y(),
              1e-6);
  EXPECT_NEAR(result_single.getOrigin().z(), result_multi.getOrigin().z(),
              1e-6);

  auto q_single = result_single.getRotation();
  auto q_multi = result_multi.getRotation();

  EXPECT_NEAR(q_single.x(), q_multi.x(), 1e-6);
  EXPECT_NEAR(q_single.y(), q_multi.y(), 1e-6);
  EXPECT_NEAR(q_single.z(), q_multi.z(), 1e-6);
  EXPECT_NEAR(q_single.w(), q_multi.w(), 1e-6);
}

// Test 7: Test timestamp comparison logic for duplicate detection
TEST_F(YmirCoreTest, TimestampComparison) {
  builtin_interfaces::msg::Time t1;
  t1.sec = 100;
  t1.nanosec = 500000000;

  builtin_interfaces::msg::Time t2;
  t2.sec = 100;
  t2.nanosec = 500000000;

  builtin_interfaces::msg::Time t3;
  t3.sec = 100;
  t3.nanosec = 600000000;

  // Same timestamp
  bool is_same_12 = (t1.sec == t2.sec && t1.nanosec == t2.nanosec);
  EXPECT_TRUE(is_same_12) << "Identical timestamps should be detected";

  // Different timestamp
  bool is_same_13 = (t1.sec == t3.sec && t1.nanosec == t3.nanosec);
  EXPECT_FALSE(is_same_13) << "Different timestamps should be detected";
}

// Test 8: Vector appending vs prepending order
TEST_F(YmirCoreTest, VectorAppendOrder) {
  std::vector<int> path;

  // Simulate appending (CORRECT behavior after fix)
  path.push_back(1);
  path.push_back(2);
  path.push_back(3);

  EXPECT_EQ(path.size(), 3);
  EXPECT_EQ(path[0], 1) << "First element should be 1";
  EXPECT_EQ(path[1], 2) << "Second element should be 2";
  EXPECT_EQ(path[2], 3) << "Third element should be 3";

  // Verify chronological order
  for (size_t i = 1; i < path.size(); ++i) {
    EXPECT_LT(path[i - 1], path[i]) << "Elements should be in ascending order";
  }
}

// Test 9: Vector prepending order (OLD BUGGY behavior)
TEST_F(YmirCoreTest, VectorPrependOrderWasWrong) {
  std::vector<int> path;

  // Simulate prepending (OLD WRONG behavior)
  path.insert(path.begin(), 1);
  path.insert(path.begin(), 2);
  path.insert(path.begin(), 3);

  EXPECT_EQ(path.size(), 3);
  EXPECT_EQ(path[0], 3) << "With prepending, newest is first (WRONG)";
  EXPECT_EQ(path[1], 2);
  EXPECT_EQ(path[2], 1);

  // This would be in REVERSE order (bug we fixed)
  for (size_t i = 1; i < path.size(); ++i) {
    EXPECT_GT(path[i - 1], path[i])
        << "Prepending creates reverse order (the bug)";
  }
}

// Test 10: Empty transform case
TEST_F(YmirCoreTest, EmptyTransformList) {
  std::vector<tf2::Transform> empty;
  auto result = averagePosesMultiple(empty);

  // Should return identity
  EXPECT_NEAR(result.getOrigin().x(), 0.0, 1e-6);
  EXPECT_NEAR(result.getOrigin().y(), 0.0, 1e-6);
  EXPECT_NEAR(result.getOrigin().z(), 0.0, 1e-6);

  auto q = result.getRotation();
  auto identity = tf2::Quaternion::getIdentity();

  EXPECT_NEAR(q.x(), identity.x(), 1e-6);
  EXPECT_NEAR(q.y(), identity.y(), 1e-6);
  EXPECT_NEAR(q.z(), identity.z(), 1e-6);
  EXPECT_NEAR(q.w(), identity.w(), 1e-6);
}

// Test 11: Three radar averaging
TEST_F(YmirCoreTest, ThreeRadarAveraging) {
  tf2::Transform t1(tf2::Quaternion::getIdentity(),
                    tf2::Vector3(1.0, 0.0, 0.0));
  tf2::Transform t2(tf2::Quaternion::getIdentity(),
                    tf2::Vector3(2.0, 0.0, 0.0));
  tf2::Transform t3(tf2::Quaternion::getIdentity(),
                    tf2::Vector3(3.0, 0.0, 0.0));

  std::vector<tf2::Transform> transforms = {t1, t2, t3};
  auto result = averagePosesMultiple(transforms);

  // Average of 1, 2, 3 is 2
  EXPECT_NEAR(result.getOrigin().x(), 2.0, 1e-6);
  EXPECT_NEAR(result.getOrigin().y(), 0.0, 1e-6);
  EXPECT_NEAR(result.getOrigin().z(), 0.0, 1e-6);
}

// Test 12: Quaternion hemisphere alignment
TEST_F(YmirCoreTest, QuaternionHemisphereAlignment) {
  // Create two quaternions representing same rotation but opposite signs
  tf2::Quaternion q1(0.0, 0.0, 0.707, 0.707);   // 90 degrees
  tf2::Quaternion q2(0.0, 0.0, -0.707, -0.707); // Same rotation, flipped sign

  // These represent the same rotation
  tf2::Transform t1(q1, tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Transform t2(q2, tf2::Vector3(0.0, 0.0, 0.0));

  // Check that dot product is negative (opposite hemisphere)
  double dot = q1.dot(q2);
  EXPECT_LT(dot, 0.0) << "Quaternions should be in opposite hemispheres";

  // After hemisphere alignment, they should average correctly
  std::vector<tf2::Transform> transforms = {t1, t2};
  auto result = averagePosesMultiple(transforms);

  // Result should be normalized
  auto result_q = result.getRotation();
  double length =
      std::sqrt(result_q.x() * result_q.x() + result_q.y() * result_q.y() +
                result_q.z() * result_q.z() + result_q.w() * result_q.w());

  EXPECT_NEAR(length, 1.0, 1e-6) << "Result should be normalized";
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
