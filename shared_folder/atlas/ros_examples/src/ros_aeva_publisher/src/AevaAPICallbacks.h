// Copyright 2016-2025 Aeva, Inc. All rights reserved.
// 
// Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
// code for terms and conditions that govern your use of this software. Any use, reproduction,
// disclosure, or distribution of this software and related documentation outside the terms of the
// EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

#pragma once

#include <ros/ros.h>

#include "AevaPublisher.h"

namespace aeva {
namespace api {
namespace ros_examples {

/// \brief Callback function that receives the PointCloud wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void PointCloudCallback(const api::ROSPointCloudWrapper& wrapper,
                        std::vector<std::shared_ptr<AevaPublisher>>& publishers);

/// \brief Callback function that receives the Calibration wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void CalibrationCallback(const api::ROSCalibrationWrapper& wrapper,
                         std::vector<std::shared_ptr<AevaPublisher>>& publishers);

/// \brief Callback function that receives the VehicleStateEstimate wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \note Only supported for Aeries II. See OdometryCallback for Atlas.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
/// \param sensor_fusion_enabled Indicates if sensor fusion is enabled in the Aeva API.
void VehicleStateEstimateCallback(const api::ROSVehicleStateEstimateWrapper& wrapper,
                                  std::vector<std::shared_ptr<AevaPublisher>>& publishers,
                                  const std::string& main_lidar_sensor_id,
                                  bool sensor_fusion_enabled = false);

/// \brief Callback function that receives the SensorFusionVehicleStateEstimate wrapper
///        from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \note Only supported for Aeries II.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
/// \param sensor_fusion_enabled Indicates if sensor fusion is enabled in the Aeva API.
void SensorFusionVehicleStateEstimateCallback(
    const api::ROSVehicleStateEstimateWrapper& wrapper,
    std::vector<std::shared_ptr<AevaPublisher>>& publishers, bool sensor_fusion_enabled = false);

/// \brief Callback function that receives the VelocityEstimate wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void VelocityEstimateCallback(const api::ROSVelocityEstimateWrapper& wrapper,
                              std::vector<std::shared_ptr<AevaPublisher>>& publishers);

/// \brief Callback function that receives the Odometry wrapper from the Aeva API.
/// \note Only supported for Atlas. See VehicleStateEstimateCallback for Aeries II.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void OdometryCallback(const api::ROSOdometryWrapper& wrapper,
                      std::vector<std::shared_ptr<AevaPublisher>>& publishers);

/// \brief Callback function that receives the IMU wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \note Only supported for Aeries II. See OdometryCallback for Atlas.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void IMUCallback(const api::ROSIMUWrapper& wrapper,
                 std::vector<std::shared_ptr<AevaPublisher>>& publishers);

/// \brief Callback function that receives the Health wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void HealthCallback(const api::ROSHealthWrapper& wrapper,
                    std::vector<std::shared_ptr<AevaPublisher>>& publishers);

/// \brief Callback function that receives the NavigationSolution wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void NavigationSolutionCallback(const api::ROSNavigationSolutionWrapper& wrapper,
                                std::vector<std::shared_ptr<AevaPublisher>>& publishers);

/// \brief Callback function that receives the ObjectList wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
/// \param sensor_fusion_enabled Indicates if sensor fusion is enabled in the Aeva API.
void ObjectListCallback(const api::ROSObjectListWrapper& wrapper,
                        std::vector<std::shared_ptr<AevaPublisher>>& publishers,
                        bool sensor_fusion_enabled = false);

/// \brief Callback function that receives the SensorFusionObjectList wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
/// \param sensor_fusion_enabled Indicates if sensor fusion is enabled in the Aeva API.
void SensorFusionObjectListCallback(const api::ROSObjectListWrapper& wrapper,
                                    std::vector<std::shared_ptr<AevaPublisher>>& publishers,
                                    bool sensor_fusion_enabled = false);

/// \brief Callback function that receives the SPCPointGroup wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void SPCPointGroupCallback(const api::ROSRawDataWrapper& wrapper,
                           std::vector<std::shared_ptr<AevaPublisher>>& publishers);

/// \brief Callback function that receives the CPCPointGroup wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \note Only supported for Atlas.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void CPCPointGroupCallback(const api::ROSRawDataWrapper& wrapper,
                           std::vector<std::shared_ptr<AevaPublisher>>& publishers);

/// \brief Callback function that receives the RawPacket wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void RawPacketCallback(const api::ROSRawDataWrapper& wrapper,
                       std::vector<std::shared_ptr<AevaPublisher>>& publishers);

/// \brief Callback function that receives the Reconfig wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void ReconfigCallback(const api::ROSRawDataWrapper& wrapper,
                      std::vector<std::shared_ptr<AevaPublisher>>& publishers);

/// \brief Callback function that receives the Telemetry wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void TelemetryCallback(const api::ROSTelemetryWrapper& wrapper,
                       std::vector<std::shared_ptr<AevaPublisher>>& publishers);

/// \brief Callback function that receives the Config wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void ConfigCallback(const api::ROSRawDataWrapper& wrapper,
                    std::vector<std::shared_ptr<AevaPublisher>>& publishers);

/// \brief Callback function that receives the Log wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void LogCallback(const api::ROSRawDataWrapper& wrapper,
                 std::vector<std::shared_ptr<AevaPublisher>>& publishers);

/// \brief Callback function that receives the Metadata wrapper from the Aeva API.
/// \note This function directs the message to the correct ROS publisher based on the sensor ID.
/// \param publishers A vector of shared pointers to publishers of all connected sensors.
void MetadataCallback(const std_msgs::String& metadata,
                      std::vector<std::shared_ptr<AevaPublisher>>& publishers);

}  // namespace ros_examples
}  // namespace api
}  // namespace aeva
