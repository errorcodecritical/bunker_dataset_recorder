// Copyright 2016-2025 Aeva, Inc. All rights reserved.
// 
// Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
// code for terms and conditions that govern your use of this software. Any use, reproduction,
// disclosure, or distribution of this software and related documentation outside the terms of the
// EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

#include "AevaPublisher.h"

namespace aeva {
namespace api {
namespace ros_examples {

namespace {

// Queue size for the publishers.
constexpr int64_t kDefaultQueueSize{10};

// Used for topic names for Aeva RawData types.
// These message indices are used to open the ROS bag with Aeviz.
constexpr std::size_t kCalibrationMessageIndex{1};
constexpr std::size_t kTelemetryMessageIndex{3};
constexpr std::size_t kConfigMessageIndex{4};
constexpr std::size_t kLogMessageIndex{5};
constexpr std::size_t kObjectListMessageIndex{6};
constexpr std::size_t kPointCloudMessageIndex{7};
constexpr std::size_t kSPCDataMessageIndex{11};
constexpr std::size_t kHealthMessageIndex{12};
constexpr std::size_t kVelocityEstimateMessageIndex{13};
constexpr std::size_t kIMUMessageIndex{14};
constexpr std::size_t kVSE3DMessageIndex{16};
constexpr std::size_t kReconfigMessageIndex{17};
constexpr std::size_t kRawPacketMessageIndex{20};
constexpr std::size_t kNavigationSolutionMessageIndex{23};
constexpr std::size_t kSensorOdometryMessageIndex{35};
constexpr std::size_t kCPCPointGroupMessageIndex{45};

// Reserved sensor index for sensor fusion pipeline.
constexpr std::size_t kSensorFusionSensorIndex{0xFE};

// Topic names for ROS native types.
constexpr char kTopicPrefix[] = "/aeva/";
constexpr char kTransformTopic[] = "/tf";
// Topic name suffixes.
constexpr char kHealthSuffix[] = "/health";
constexpr char kIMUSuffix[] = "/imu";
constexpr char kDetectionArraySuffix[] = "/object_list/detections";
constexpr char kMarkerArraySuffix[] = "/object_list/markers";
constexpr char kOdometrySuffix[] = "/odometry";
constexpr char kPointCloudSuffix[] = "/point_cloud_compensated";
constexpr char kPointCloudMetadataSuffix[] = "/point_cloud_metadata";
constexpr char kTwistSuffix[] = "/twist";
constexpr char kMetadataTopic[] = "/metadata";

}  // namespace

AevaPublisher::AevaPublisher(const std::string& sensor_id, const std::size_t sensor_index,
                             const int sensor_index_offset, ros::NodeHandle& nh,
                             bool publish_raw_topics, bool publish_ros_topics)
    : sensor_id_(sensor_id),
      sensor_index_(sensor_index),
      sensor_index_offset_(sensor_index_offset),
      publish_raw_topics_(publish_raw_topics),
      publish_ros_topics_(publish_ros_topics) {
  if (api::kSensorFusionSensorId == sensor_id_) {
    ROS_FATAL_COND(kSensorFusionSensorIndex == sensor_index,
                   "Invalid sensor index for sensor fusion publisher, must be 0xFE");
    exit(1);
  }

  if (!publish_raw_topics_ && !publish_ros_topics_) {
    ROS_FATAL("Both raw topics and ROS topics cannot be disabled!");
    exit(1);
  }

  // Initialize ROS native type publishers.
  //
  // NOTE: There may be additional topics that get advertised for external sensors and sensor
  // fusion pipeline, however there will no data published on these topics. Eg: External GNSS may
  // advertise a point cloud topic but will never publish any point cloud data on it.
  if (publish_ros_topics_) {
    point_cloud_pub_ =
        nh.advertise<sensor_msgs::PointCloud2>(GetTopicName(kPointCloudSuffix), kDefaultQueueSize);
    point_cloud_metadata_pub_ = nh.advertise<aeva_msgs::PointCloudMetadata>(
        GetTopicName(kPointCloudMetadataSuffix), kDefaultQueueSize);
    odometry_pub_ =
        nh.advertise<nav_msgs::Odometry>(GetTopicName(kOdometrySuffix), kDefaultQueueSize);
    twist_pub_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(GetTopicName(kTwistSuffix),
                                                                         kDefaultQueueSize);
    tf_pub_ = nh.advertise<tf2_msgs::TFMessage>(kTransformTopic, kDefaultQueueSize);
    imu_pub_ = nh.advertise<sensor_msgs::Imu>(GetTopicName(kIMUSuffix), kDefaultQueueSize);
    health_pub_ = nh.advertise<aeva_msgs::Health>(GetTopicName(kHealthSuffix), kDefaultQueueSize);
    detection_array_pub_ = nh.advertise<aeva_msgs::Detection3DArray>(
        GetTopicName(kDetectionArraySuffix), kDefaultQueueSize);
    marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        GetTopicName(kMarkerArraySuffix), kDefaultQueueSize);
  }

  if (publish_raw_topics_) {
    // Initialize ROS Aeva raw data publishers.
    raw_point_cloud_pub_ = nh.advertise<aeva_msgs::RawData>(
        GetTopicName(kPointCloudMessageIndex, "PointCloud", "point_cloud_compensated"),
        kDefaultQueueSize);
    raw_calibration_pub_ = nh.advertise<aeva_msgs::RawData>(
        GetTopicName(kCalibrationMessageIndex, "Calibration", "calibration"), kDefaultQueueSize);
    raw_vehicle_state_estimate_pub_ = nh.advertise<aeva_msgs::RawData>(
        GetTopicName(kVSE3DMessageIndex, "VehicleStateEstimate", "S3d_vehicle_state_estimate"),
        kDefaultQueueSize);
    raw_sensor_fusion_vehicle_state_estimate_pub_ =
        nh.advertise<aeva_msgs::RawData>(GetTopicName(kVSE3DMessageIndex, "VehicleStateEstimate",
                                                      "sensor_fusion_3d_vehicle_state_estimate"),
                                         kDefaultQueueSize);
    raw_velocity_estimate_pub_ = nh.advertise<aeva_msgs::RawData>(
        GetTopicName(kVelocityEstimateMessageIndex, "VelocityEstimate", "velocity_estimate"),
        kDefaultQueueSize);
    raw_odometry_pub_ = nh.advertise<aeva_msgs::RawData>(
        GetTopicName(kSensorOdometryMessageIndex, "Odometry", "sensor_odometry"),
        kDefaultQueueSize);
    raw_imu_pub_ = nh.advertise<aeva_msgs::RawData>(GetTopicName(kIMUMessageIndex, "IMU", "imu"),
                                                    kDefaultQueueSize);
    raw_health_pub_ = nh.advertise<aeva_msgs::RawData>(
        GetTopicName(kHealthMessageIndex, "Health", "health"), kDefaultQueueSize);
    raw_navigation_solution_pub_ = nh.advertise<aeva_msgs::RawData>(
        GetTopicName(kNavigationSolutionMessageIndex, "NavigationSolution", "navigation_solution"),
        kDefaultQueueSize);
    raw_object_list_pub_ = nh.advertise<aeva_msgs::RawData>(
        GetTopicName(kObjectListMessageIndex, "ObjectList", "lidar_object_list"),
        kDefaultQueueSize);
    raw_sensor_fusion_object_list_pub_ = nh.advertise<aeva_msgs::RawData>(
        GetTopicName(kObjectListMessageIndex, "ObjectList", "sensor_fusion_lidar_object_list"),
        kDefaultQueueSize);
    raw_spc_pointgroup_pub_ = nh.advertise<aeva_msgs::RawData>(
        GetTopicName(kSPCDataMessageIndex, "RawData", "spc_pointgroup"), kDefaultQueueSize);
    raw_cpc_pointgroup_pub_ = nh.advertise<aeva_msgs::RawData>(
        GetTopicName(kCPCPointGroupMessageIndex, "AtlasCpcData", "raw_cpc_pointgroup"),
        kDefaultQueueSize);
    raw_packet_pub_ = nh.advertise<aeva_msgs::RawData>(
        GetTopicName(kRawPacketMessageIndex, "RawData", "raw_packet"), kDefaultQueueSize);
    raw_reconfig_pub_ = nh.advertise<aeva_msgs::RawData>(
        GetTopicName(kReconfigMessageIndex, "Reconfig", "reconfig_client"), kDefaultQueueSize);
    raw_telemetry_pub_ = nh.advertise<aeva_msgs::RawData>(
        GetTopicName(kTelemetryMessageIndex, "Telemetry", "telemetry"), kDefaultQueueSize);
    raw_config_pub_ = nh.advertise<aeva_msgs::RawData>(
        GetTopicName(kConfigMessageIndex, "Config", "config"), kDefaultQueueSize);
    raw_log_pub_ = nh.advertise<aeva_msgs::RawData>(GetTopicName(kLogMessageIndex, "Log", "log"),
                                                    kDefaultQueueSize);
    metadata_pub_ = nh.advertise<std_msgs::String>(GetMetadataTopicName(), kDefaultQueueSize);
  }
}

void AevaPublisher::PublishPointCloud(const sensor_msgs::PointCloud2& message,
                                      const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    point_cloud_pub_.publish(message);
  }
}

void AevaPublisher::PublishPointCloudMetadata(const aeva_msgs::PointCloudMetadata& message,
                                              const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    point_cloud_metadata_pub_.publish(message);
  }
}

void AevaPublisher::PublishOdometry(const nav_msgs::Odometry& message,
                                    const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    odometry_pub_.publish(message);
  }
}

void AevaPublisher::PublishTwist(const geometry_msgs::TwistWithCovarianceStamped& message,
                                 const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    twist_pub_.publish(message);
  }
}

void AevaPublisher::PublishTransform(const tf2_msgs::TFMessage& message,
                                     const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    tf_pub_.publish(message);
  }
}

void AevaPublisher::PublishIMU(const sensor_msgs::Imu& message, const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    imu_pub_.publish(message);
  }
}

void AevaPublisher::PublishHealth(const aeva_msgs::Health& message, const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    health_pub_.publish(message);
  }
}

void AevaPublisher::PublishDetectionArray(const aeva_msgs::Detection3DArray& message,
                                          const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    detection_array_pub_.publish(message);
  }
}

void AevaPublisher::PublishMarkerArray(const visualization_msgs::MarkerArray& message,
                                       const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_ros_topics_) {
    marker_array_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawPointCloud(const aeva_msgs::RawData& message,
                                         const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_point_cloud_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawCalibration(const aeva_msgs::RawData& message,
                                          const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_calibration_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawVehicleStateEstimate(const aeva_msgs::RawData& message,
                                                   const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_vehicle_state_estimate_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawSensorFusionVehicleStateEstimate(const aeva_msgs::RawData& message,
                                                               const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_sensor_fusion_vehicle_state_estimate_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawVelocityEstimate(const aeva_msgs::RawData& message,
                                               const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_velocity_estimate_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawOdometry(const aeva_msgs::RawData& message,
                                       const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_odometry_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawIMU(const aeva_msgs::RawData& message, const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_imu_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawNavigationSolution(const aeva_msgs::RawData& message,
                                                 const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_navigation_solution_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawObjectList(const aeva_msgs::RawData& message,
                                         const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_object_list_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawSensorFusionObjectList(const aeva_msgs::RawData& message,
                                                     const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_sensor_fusion_object_list_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawSPCPointGroup(const aeva_msgs::RawData& message,
                                            const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_spc_pointgroup_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawCPCPointGroup(const aeva_msgs::RawData& message,
                                            const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_cpc_pointgroup_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawReconfig(const aeva_msgs::RawData& message,
                                       const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_reconfig_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawConfig(const aeva_msgs::RawData& message,
                                     const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_config_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawLog(const aeva_msgs::RawData& message, const std::string& sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_log_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawHealth(const aeva_msgs::RawData& message,
                                     const std::string sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_health_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawPacket(const aeva_msgs::RawData& message,
                                     const std::string sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_packet_pub_.publish(message);
  }
}

void AevaPublisher::PublishRawTelemetry(const aeva_msgs::RawData& message,
                                        const std::string sensor_id) {
  if (sensor_id == sensor_id_ && publish_raw_topics_) {
    raw_telemetry_pub_.publish(message);
  }
}

void AevaPublisher::PublishMetadata(const std_msgs::String& message) {
  if (publish_raw_topics_) {
    metadata_pub_.publish(message);
  }
}

std::string AevaPublisher::GetTopicName(const std::string& topic_suffix) const {
  std::stringstream topic_ss;
  topic_ss << kTopicPrefix << sensor_id_ << topic_suffix;
  return topic_ss.str();
}

std::string AevaPublisher::GetTopicName(std::size_t stream_index, const std::string& message_type,
                                        const std::string& stream_name) const {
  std::stringstream topic_ss;
  topic_ss << kTopicPrefix << std::hex << std::setw(6) << std::setfill('0')
           << (sensor_index_ << 16 | stream_index << 8) << "/" << message_type << "/"
           << stream_name;
  return topic_ss.str();
}

std::string AevaPublisher::GetMetadataTopicName() const {
  std::stringstream topic_ss;
  topic_ss << kTopicPrefix << std::hex << std::setw(6) << std::setfill('0')
           << (sensor_index_offset_ << 16) << kMetadataTopic;
  return topic_ss.str();
}

}  // namespace ros_examples
}  // namespace api
}  // namespace aeva
