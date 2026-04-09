// Copyright 2016-2025 Aeva, Inc. All rights reserved.
// 
// Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
// code for terms and conditions that govern your use of this software. Any use, reproduction,
// disclosure, or distribution of this software and related documentation outside the terms of the
// EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

#pragma once

#include <ros/ros.h>

#include "aeva/api/Types.h"

namespace aeva {
namespace api {
namespace ros_examples {

/// \class AevaPublisher
/// \brief This class handles publishing native and Aeva raw data ROS messages for a single sensor.
class AevaPublisher {
 public:
  /// \brief Constructs an AevaPublisher object.
  /// \param sensor_id The unique identifier for the sensor in the Aeva API.
  /// \param sensor_index The index of the sensor in the Aeva API.
  /// \param sensor_index_offset The offset to be added to the sensor index.
  /// \param nh Reference to the ROS NodeHandle.
  /// \param publish_raw_topics Flag to indicate whether to publish aeva_msgs::RawData topics.
  /// \param publish_ros_topics Flag to indicate whether to publish native ROS format topics.
  AevaPublisher(const std::string& sensor_id, const std::size_t sensor_index,
                const int sensor_index_offset, ros::NodeHandle& nh, bool publish_raw_topics = true,
                bool publish_ros_topics = true);

  /// \brief Virtual destructor to make this class polymorphic.
  virtual ~AevaPublisher() = default;

  /// \brief Returns the sensor ID this publisher is linked to.
  /// \return The sensor ID as a std::string.
  inline std::string GetSensorID() const { return sensor_id_; }

  /// \brief Returns the sensor index this publisher is linked to.
  /// \return The sensor index as a std::size_t.
  inline std::size_t GetSensorIndex() const { return sensor_index_; }

  /* ROS native types publish callbacks */

  /// \brief Publishes a PointCloud2 message.
  /// \param message The PointCloud2 message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishPointCloud(const sensor_msgs::PointCloud2& message, const std::string& sensor_id);

  /// \brief Publishes a PointCloudMetadata message.
  /// \param message The PointCloudMetadata message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishPointCloudMetadata(const aeva_msgs::PointCloudMetadata& message,
                                 const std::string& sensor_id);

  /// \brief Publishes an Odometry message.
  /// \param message The Odometry message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishOdometry(const nav_msgs::Odometry& message, const std::string& sensor_id);

  /// \brief Publishes a TwistWithCovarianceStamped message.
  /// \param message The TwistWithCovarianceStamped message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishTwist(const geometry_msgs::TwistWithCovarianceStamped& message,
                    const std::string& sensor_id);

  /// \brief Publishes a TFMessage for transforms.
  /// \note Only supported for Aeries II. Consolidated with PublishOdometry for Atlas.
  /// \param message The TFMessage to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishTransform(const tf2_msgs::TFMessage& message, const std::string& sensor_id);

  /// \brief Publishes an IMU message.
  /// \param message The IMU message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishIMU(const sensor_msgs::Imu& message, const std::string& sensor_id);

  /// \brief Publishes a Health message.
  /// \param message The Health message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishHealth(const aeva_msgs::Health& message, const std::string& sensor_id);

  /// \brief Publishes a Detection3DArray message.
  /// \param message The Detection3DArray message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishDetectionArray(const aeva_msgs::Detection3DArray& message,
                             const std::string& sensor_id);

  /// \brief Publishes a MarkerArray message.
  /// \param message The MarkerArray message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishMarkerArray(const visualization_msgs::MarkerArray& message,
                          const std::string& sensor_id);

  /* ROS Aeva raw data types publish callbacks */

  /// \brief Publishes a raw PointCloud message.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawPointCloud(const aeva_msgs::RawData& message, const std::string& sensor_id);

  /// \brief Publishes a raw Calibration message.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawCalibration(const aeva_msgs::RawData& message, const std::string& sensor_id);

  /// \brief Publishes a raw VehicleStateEstimate message.
  /// \note Only supported for Aeries II. See RawOdometry for Atlas.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawVehicleStateEstimate(const aeva_msgs::RawData& message,
                                      const std::string& sensor_id);

  /// \brief Publishes a raw SensorFusionVehicleStateEstimate message.
  /// \note Only supported for Aeries II. See RawOdometry for Atlas.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawSensorFusionVehicleStateEstimate(const aeva_msgs::RawData& message,
                                                  const std::string& sensor_id);

  /// \brief Publishes a raw VelocityEstimate message.
  /// \note Only supported for Aeries II. See RawOdometry for Atlas.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawVelocityEstimate(const aeva_msgs::RawData& message, const std::string& sensor_id);

  /// \brief Publishes a raw Odometry message.
  /// \note Only supported for Atlas. See RawVehicleStateEstimate for Aeries II.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawOdometry(const aeva_msgs::RawData& message, const std::string& sensor_id);

  /// \brief Publishes a raw IMU message.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawIMU(const aeva_msgs::RawData& message, const std::string& sensor_id);

  /// \brief Publishes a raw NavigationSolution message.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawNavigationSolution(const aeva_msgs::RawData& message,
                                    const std::string& sensor_id);

  /// \brief Publishes a raw ObjectList message.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawObjectList(const aeva_msgs::RawData& message, const std::string& sensor_id);

  /// \brief Publishes a raw SensorFusionObjectList message.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawSensorFusionObjectList(const aeva_msgs::RawData& message,
                                        const std::string& sensor_id);

  /// \brief Publishes a raw SPCPointGroup message.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawSPCPointGroup(const aeva_msgs::RawData& message, const std::string& sensor_id);

  /// \brief Publishes a raw CPCPointGroup message.
  /// \note Only supported for Atlas.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawCPCPointGroup(const aeva_msgs::RawData& message, const std::string& sensor_id);

  /// \brief Publishes a raw Reconfig message.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawReconfig(const aeva_msgs::RawData& message, const std::string& sensor_id);

  /// \brief Publishes a raw Config message.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawConfig(const aeva_msgs::RawData& message, const std::string& sensor_id);

  /// \brief Publishes a raw Log message.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawLog(const aeva_msgs::RawData& message, const std::string& sensor_id);

  /// \brief Publishes a raw Health message.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawHealth(const aeva_msgs::RawData& message, const std::string sensor_id);

  /// \brief Publishes a raw packet message.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawPacket(const aeva_msgs::RawData& message, const std::string sensor_id);

  /// \brief Publishes a raw Telemetry message.
  /// \param message The RawData message to publish.
  /// \param sensor_id The ID of the sensor that generated the data.
  void PublishRawTelemetry(const aeva_msgs::RawData& message, const std::string sensor_id);

  /// \brief Publishes a metadata message.
  /// \param message The metadata string message to publish.
  void PublishMetadata(const std_msgs::String& message);

 protected:
  /// \brief Returns the full topic name based on a topic suffix.
  /// \param topic_suffix The suffix to append to the base topic name.
  /// \return The full topic name as a std::string.
  std::string GetTopicName(const std::string& topic_suffix) const;

  /// \brief Returns the full topic name based on stream index, message type, and stream name.
  /// \param stream_index The index of the data stream.
  /// \param message_type The type of the message.
  /// \param stream_name The name of the data stream.
  /// \return The full topic name as a std::string.
  std::string GetTopicName(std::size_t stream_index, const std::string& message_type,
                           const std::string& stream_name) const;

  /// \brief Returns the topic name for metadata.
  /// \return The metadata topic name as a std::string.
  std::string GetMetadataTopicName() const;

  /// Sensor ID, index and index offset used in the Aeva API.
  std::string sensor_id_;
  std::size_t sensor_index_{0};
  int sensor_index_offset_{0};

  /// Flag to determine whether to advertise raw data topics.
  bool publish_raw_topics_{true};
  /// Flag to determine whether to advertise ROS native topics.
  bool publish_ros_topics_{true};

  /// ROS native type publishers.
  ros::Publisher point_cloud_pub_;
  ros::Publisher point_cloud_metadata_pub_;
  ros::Publisher odometry_pub_;
  ros::Publisher twist_pub_;
  ros::Publisher tf_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher health_pub_;
  ros::Publisher detection_array_pub_;
  ros::Publisher marker_array_pub_;
  ros::Publisher metadata_pub_;

  /// ROS Aeva raw data publishers.
  ros::Publisher raw_point_cloud_pub_;
  ros::Publisher raw_calibration_pub_;
  ros::Publisher raw_vehicle_state_estimate_pub_;
  ros::Publisher raw_sensor_fusion_vehicle_state_estimate_pub_;
  ros::Publisher raw_velocity_estimate_pub_;
  ros::Publisher raw_odometry_pub_;
  ros::Publisher raw_imu_pub_;
  ros::Publisher raw_health_pub_;
  ros::Publisher raw_navigation_solution_pub_;
  ros::Publisher raw_object_list_pub_;
  ros::Publisher raw_packet_pub_;
  ros::Publisher raw_sensor_fusion_object_list_pub_;
  ros::Publisher raw_spc_pointgroup_pub_;
  ros::Publisher raw_cpc_pointgroup_pub_;
  ros::Publisher raw_reconfig_pub_;
  ros::Publisher raw_telemetry_pub_;
  ros::Publisher raw_config_pub_;
  ros::Publisher raw_log_pub_;
};

}  // namespace ros_examples
}  // namespace api
}  // namespace aeva
