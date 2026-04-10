// Copyright (c) Sensrad 2024-2025

// This file specifies public input and output types for the Oden perception
// module.

#pragma once

// There is a problem in Eigen 3.4.0: it generates a warning in
// TriangularMatrixVector.h:332:12: warning: ‘result’ may be used uninitialized
// [-Wmaybe-uninitialized]
//
// https://github.com/PDAL/PDAL/issues/4102
//
// The GCC command below suppresses this warning.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace common {

constexpr int CLUSTER_UNDEFINED = -2;
constexpr int CLUSTER_NOISE = -1;

constexpr float DEFAULT_TRACK_LOG_LIKELIHOOD = -30.0F;
constexpr float DEFAULT_DISTANCE_GROUNDPLANE = 1000.0F; // [m]
constexpr float DEFAULT_RCS = 1000.0F;                  // [dBsm]

static constexpr uint32_t RGBA_DEFAULT = 0xFFFFFFFF; // White

// Definition of motion_status assessment per point in OutputPoint.
enum MotionStatus : uint8_t { STATIONARY = 0, UNKNOWN = 1, DYNAMIC = 2 };

// Object type; corresponds to the integer "type" in "TrackedObject" and
// "Detection"
enum Type : uint8_t {
  CLASS_UNKNOWN = 0,
  CLASS_PEDESTRIAN = 1,
  CLASS_BICYCLE = 2,
  CLASS_CAR = 3,
  CLASS_UTILITY_VEHICLE = 4,
  CLASS_TRUCK = 5,
  NUM_CLASSES = 6
};

// Definition of track state; corresponds to "track_state" in type TrackedObject
enum TrackState : int { // NOLINT(performance-enum-size) - public API
                        // compatibility
  INITIALIZING = 0,
  TENTATIVE = 1,
  MATURE = 2,
  TERMINATE = 3,
  GHOST = 4
};

// Definition of perception compute status
enum Status : int { // NOLINT(performance-enum-size) - public API compatibility
  NORMAL = 0,
  LARGE_TIME_GAP = 1,
  SHORT_TIME_GAP = 2,
  NEGATIVE_TIME_GAP = 3
};

// Definition of perception modes. In external builds, only "DYNAMIC_DEFAULT"
// and "STATIC_DEFAULT" are available.
enum PERCEPTION_CONFIG : int { // NOLINT(performance-enum-size) - public API
                               // compatibility
  DYNAMIC_DEFAULT = 0,
  STATIC_DEFAULT = 1,
  SURVEILLANCE = 2,
  ROCKFALL = 3,
  DYNAMIC_NO_LOCALMAX = 4,
  HARD_SHOULDER = 5
};

// Disable clang tidy checks on public visibility since we want these data
// carrying structs to directly accessible by the user.

// NOLINTBEGIN(misc-non-private-member-variables-in-classes,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
// Define format of input point-cloud

// Type for storing rotational rate data from an Inertial Measurement Unit
// (IMU). It is assumed that the IMU x-axis is aligned with the radar "right"
// direction, the y-axis is aligned with the radar "forward" direction, and the
// z-axis is aligned with the radar "up" direction. Currently only the angular
// rate is used.
struct IMU {
  bool valid;
  double time_sec;      // [s]
  float angular_rate_x; // [rad/s]
  float angular_rate_y; // [rad/s]
  float angular_rate_z; // [rad/s]

  IMU()
      : valid(false), time_sec(-1.0), angular_rate_x(0.0F),
        angular_rate_y(0.0F), angular_rate_z(0.0F) {}

  IMU(bool valid, double time_sec, float angular_rate_x, float angular_rate_y,
      float angular_rate_z)
      : valid(valid), time_sec(time_sec), angular_rate_x(angular_rate_x),
        angular_rate_y(angular_rate_y), angular_rate_z(angular_rate_z) {}
};

// Type for storing current radar resolution values
struct RadarResolution {
  bool valid;
  float azimuth;   // [rad]
  float elevation; // [rad]
  float range;     // [m]
  float doppler;   // [m/s]

  RadarResolution()
      : valid(false), azimuth(0.0F), elevation(0.0F), range(0.0F),
        doppler(0.0F) {}

  RadarResolution(bool valid, float azimuth, float elevation, float range,
                  float doppler)
      : valid(valid), azimuth(azimuth), elevation(elevation), range(range),
        doppler(doppler) {}
};

// Type for storing data from the radar.
struct InputPoint {
  float range;     // [m]
  float elevation; // [rad]
  float azimuth;   // [rad]
  float power;     // [dB]
  float doppler;   // [m/s]
  float x;         // [m]
  float y;         // [m]
  float z;         // [m]
  std::size_t
      sample_index; // This is only for dealing with replay and annotations.

  InputPoint(float range, float elevation, float azimuth, float power,
             float doppler, float x, float y, float z, std::size_t sample_index)
      : range(range), elevation(elevation), azimuth(azimuth), power(power),
        doppler(doppler), x(x), y(y), z(z), sample_index{sample_index} {}
};
typedef std::vector<InputPoint> IncomingPointCloud;

// Type for storing radar resolution, time, and incoming pointcloud
class RadarData {
private:
  static constexpr std::size_t IO_RESERVE_SIZE = 32768;
  uint64_t timestamp_ms_{0};
  RadarResolution resolution_;
  IncomingPointCloud incoming_pointcloud_;

public:
  RadarData() { incoming_pointcloud_.reserve(IO_RESERVE_SIZE); }

  void clearPointCloud() { incoming_pointcloud_.clear(); }

  void setRadarResolution(const RadarResolution &rr) { resolution_ = rr; }

  void setTimeStamp_ms(uint64_t timestamp_ms) { timestamp_ms_ = timestamp_ms; }
  void addPoint(const InputPoint &point) {
    incoming_pointcloud_.push_back(point);
  }
  std::size_t size() const { return incoming_pointcloud_.size(); }

  const RadarResolution &getRadarResolution() const { return resolution_; }

  uint64_t getRadarTime_ms() const { return timestamp_ms_; }
  const IncomingPointCloud &getPointCloud() const {
    return incoming_pointcloud_;
  }

  // Sort point-cloud with respect to range; this should ideally not be
  // necessary but the radar fails occasionally on this. This should be
  // fast since the point-cloud is "almost" sorted.
  void sortByRange() {
    std::sort(incoming_pointcloud_.begin(), incoming_pointcloud_.end(),
              [](const auto &a, const auto &b) { return a.range < b.range; });
  }
};

// Define format of point-cloud data sent out from perception. Due to
// limitations in pybind11 used in replay applications we cannot easily inherit
// from InputPoint, that's we cannot reuse "InputPoint" here.
struct OutputPoint {
  float range;     // [m]
  float elevation; // [rad]
  float azimuth;   // [rad]
  float power;     // [dB]
  float doppler;   // [m/s]
  float x;         // [m]
  float y;         // [m]
  float z;         // [m]
  std::size_t
      sample_index;      // This is only for dealing with replay and annotations
  uint8_t motion_status; // [0: stationary, 1: unknown, 2: dynamic]
  float delta_velocity;  // Motion compensated doppler [m/s]
  bool
      available_for_tracker; // Whether the point is processed by the clustering
  float track_log_likelihood; // Log likelihood for the mature track closest to
                              // this point
  float track_speed; // [m/s]: track speed for the mature track closest to the
                     // point
  int cluster_idx;   // Index of the cluster this point belongs to
  float distance_to_ground_plane; // Signed distance to the ground plane [m]
  uint8_t multi_path;             // [0: no multi-path, 1: multi-path detected]
  float radar_cross_section;      // Radar cross section [dbSm]
  uint32_t rgba;                  // Color information in RGBA format
  bool ghost_track;               // Whether the point is part of a ghost track
  int track_id; // Unique identifier for the mature track closest to this point
  float local_max_margin; // Distance in dB to neighbors
  bool occluded;          // Whether the point is occluded by another object

  OutputPoint(float range, float elevation, float azimuth, float power,
              float doppler, float x, float y, float z,
              std::size_t sample_index = 0,
              int8_t motion_status = common::MotionStatus::DYNAMIC,
              float delta_velocity = 0.0F, bool available_for_tracker = false,
              float track_log_likelihood = DEFAULT_TRACK_LOG_LIKELIHOOD,
              float track_speed = 0.0F, int cluster_idx = common::CLUSTER_NOISE,
              float distance_to_ground_plane = DEFAULT_DISTANCE_GROUNDPLANE,
              uint8_t multi_path = 0, float radar_cross_section = DEFAULT_RCS,
              uint32_t rgba = RGBA_DEFAULT, bool ghost_track = false,
              int track_id = -1, float local_max_margin = 0.0f,
              bool occluded = false)
      : range(range), elevation(elevation), azimuth(azimuth), power(power),
        doppler(doppler), x(x), y(y), z(z), sample_index(sample_index),
        motion_status(motion_status), delta_velocity(delta_velocity),
        available_for_tracker(available_for_tracker),
        track_log_likelihood(track_log_likelihood), track_speed(track_speed),
        cluster_idx(cluster_idx),
        distance_to_ground_plane(distance_to_ground_plane),
        multi_path(multi_path), radar_cross_section(radar_cross_section),
        rgba(rgba), ghost_track(ghost_track), track_id(track_id),
        local_max_margin(local_max_margin), occluded(occluded) {}
};
typedef std::vector<OutputPoint> OutgoingPointCloud;

// Define format of detections created from clustered data.
struct Detection {
  float range{0};                // [m]
  float radial_speed{0};         // [m/s]
  float azimuth{0};              // [rad]
  float elevation{0};            // [rad]
  float power{0};                // [dB]
  float power_std{0};            // [dB]
  float doppler_std{0};          // [m/s]
  float doppler{0};              // [m/s]
  float rcs{0};                  // [dBsm]
  Eigen::Vector3f position;      // 3x[m]
  Eigen::Matrix3f covariance;    // 3x3[m^2]
  int num_valid_detections{0};   // Number of valid detections used to form this
                                 // cluster
  uint8_t type{0};               // Detection type
  float type_probability{0};     // Probability of detection type
  int label_id{0};               // Label ID for the detection
  float track_log_likelihood{0}; // Average track log likelihood of points
                                 // included in the cluster
  Detection() {
    // Default values to ensure data is populated correctly
    covariance.setIdentity();
    position.setZero();
  }
};
typedef std::vector<Detection> Detections;
typedef std::unordered_map<int, std::vector<int>> cluster_map_t;

// Define output format of tracked objects
struct TrackedObject {
  size_t track_id{0};            // Unique identifier for the tracked object
  int track_state{0};            // Current state of the track
  float yaw{0.0F};               // Yaw angle of the tracked object
  int type{0};                   // Type of the tracked object
  std::array<float, 3> position; // 3D position of the tracked object
  std::array<float, 3> velocity; // 3D velocity of the tracked object
  std::array<float, 9> extent;   // 3x3 row-major size of track

  TrackedObject()
      : position{0.0F, 0.0F, 0.0F}, velocity{0.0F, 0.0F, 0.0F}, extent{0.0F} {};
};
typedef std::vector<TrackedObject> TrackedObjects;

// Define format of 3D region of interest
struct RegionOfInterest {
  float x_center{0.0f}; // X coordinate of the center
  float y_center{0.0f}; // Y coordinate of the center
  float z_center{0.0f}; // Z coordinate of the center
  float width{0.0f};    // Width of the region
  float length{0.0f};   // Length of the region
  float height{0.0f};   // Height of the region
  float yaw{0.0f};      // Yaw angle of the region
  float pitch{0.0f};    // Pitch angle of the region

  RegionOfInterest() = default;
};

} // namespace common

// NOLINTEND(misc-non-private-member-variables-in-classes,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
