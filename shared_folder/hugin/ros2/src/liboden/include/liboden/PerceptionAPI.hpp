// Copyright (c) Sensrad 2023-2025

// This file specifies the public API for the Oden perception module.

#pragma once
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "IO_Types.hpp"

namespace api {
class __attribute__((visibility("default"))) PerceptionAPI {
private:
  class PerceptionImpl;
  std::unique_ptr<PerceptionImpl> pimpl_;

public:
  explicit PerceptionAPI(size_t max_nof_points);

  // Delete unused constructors
  PerceptionAPI(const PerceptionAPI &) = delete;
  PerceptionAPI &operator=(const PerceptionAPI &) = delete;
  PerceptionAPI(PerceptionAPI &&) = delete;
  PerceptionAPI &operator=(PerceptionAPI &&) = delete;

  virtual ~PerceptionAPI();

  // API functions

  // Main update function for processing a point-cloud
  common::OutgoingPointCloud update(const common::RadarData &radar_data);

  // Velocity related data
  typedef Eigen::Vector3f VelocityRadar;
  std::optional<VelocityRadar> getVelocityXYZRadar();
  bool isVelocityAvailable();

  // Rotational rate related data
  typedef Eigen::Vector3f RotationalRateRadar;
  std::optional<RotationalRateRadar> getRotationalRateXYZRadar();

  // Translation vector between two consecutive frames
  Eigen::Vector3f getDeltaPoseTranslation();

  // Quaternion between two consecutive frames
  Eigen::Quaternionf getDeltaPoseQuaternion();

  // Translation vector from start of operation to current frame
  Eigen::Vector3f getPoseTranslation();

  // Quaternion from start of operation to current frame
  Eigen::Quaternionf getPoseQuaternion();

  // Local map used for registration based odometry
  std::vector<Eigen::Vector3f> getOdometryMap();

  // Boolean to check if registration based odometry is active
  bool isOdometryActive();

  // Get odometry prediction error RMS
  float getOdometryPredictionErrorRMS();

  // Ground plane related data
  Eigen::Vector4f groundPlane();
  bool groundPlaneAvailable();

  // Set radar orientation parameters relative ground plane
  void setVerticalHeight(float vertical_height);
  void setElevationAngle(float radar_elevation_angle);

  // Retrieve the detections produced after clustering
  common::Detections getDetections();
  int numClusters();

  // Retrieve the objects in an implementation independent way
  void updateTrackedObjects(common::TrackedObjects &tracked_objects);

  // Set configuration of perception. External configurations are:
  // 0: Dynamic default
  // 1: Static default
  // The output reflects whether the requested configuration change was
  // successful
  bool setConfig(const common::PERCEPTION_CONFIG &config);

  // Return if the current configuration is dynamic or static
  bool isDynamic() const;

  // Get a string representation of the current configuration
  std::string getCurrentConfigName() const;

  // Get all available configurations
  std::map<common::PERCEPTION_CONFIG, std::string> getConfigMap() const;

  // Retrieve the status of the perception module
  common::Status getStatus(void);

  // Reset the perception module
  void reset(void);

  // Set search radius for clustering
  void setClusterGUIScale(const float scale);
  // Set number of stationary updates in tracker
  void setStationaryUpdatesMax(const int stationary_updates);

  // Set parameters for free space ground projection
  void setFreeSpaceRange(const float max_range);
  void setFreeSpaceScreeningMax(const float max_distance);
  void setFreeSpaceScreeningMin(const float min_distance);

  // Interface for setting the region of interest
  void setROI(const common::RegionOfInterest roi);
  void setROIMode(const bool enabled);
  std::optional<common::RegionOfInterest> getROI();

  // Interface for static map
  std::vector<Eigen::Vector3f> staticMapCoordinates();
  std::vector<Eigen::Vector3f> staticMapAnomalies();
  float staticMapResolution();

  // Interface for occupancy grid
  std::vector<Eigen::Vector4f> occupancyGrid();
  float occupancyGridVoxelSize();
  bool isOccupancyGridActive();

  // Interface for free space distance matrix. The free space distance matrix
  // provides information about the distances to the nearest static obstacle in
  // the environment.
  std::vector<std::vector<float>> freeSpaceMatrix();
  std::vector<float> freeSpaceAzimuthAngles();
  std::vector<float> freeSpaceElevationAngles();

  // Interface for free space ground projection. The free space ground
  // projection provides a 3D polygon representing the free space on the ground
  // plane.
  std::vector<Eigen::Vector3f> getFreeSpaceGroundPolygon();
};

} // namespace api
