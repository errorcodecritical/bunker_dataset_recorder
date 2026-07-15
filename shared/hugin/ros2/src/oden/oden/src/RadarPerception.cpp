// Copyright (c) Sensrad 2023-2025
#include <boost/format.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "AnnotationPoint.hpp"
#include "RadarPerception.hpp"

using PointCloud2ConstIterator = sensor_msgs::PointCloud2ConstIterator<float>;
using PointCloud2IteratorINT16 = sensor_msgs::PointCloud2Iterator<int16_t>;
using PointCloud2IteratorUINT8 = sensor_msgs::PointCloud2Iterator<uint8_t>;

rcl_interfaces::msg::SetParametersResult RadarPerception::onParametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;

  RCLCPP_INFO(get_logger(), "Parameters changed");

  // Initial declaration of result. Overwritten if unsuccessful.
  result.successful = true;
  result.reason = "success";

  for (const auto &parameter : parameters) {
    RCLCPP_INFO(get_logger(), "Parameters changed: %s",
                parameter.get_name().c_str());

    if (parameter.get_name() == "vertical_height") {
      const auto vertical_height = parameter.as_double();
      perception_.setVerticalHeight(static_cast<float>(vertical_height));

    } else if (parameter.get_name() == "radar_elevation_angle") {
      const auto radar_elevation_angle = parameter.as_double();
      perception_.setElevationAngle(static_cast<float>(radar_elevation_angle));

    } else if (parameter.get_name() == "stationary_updates") {
      const auto stationary_updates = parameter.as_int();
      perception_.setStationaryUpdatesMax(static_cast<int>(stationary_updates));

    } else if (parameter.get_name() == "cluster_scale") {
      const auto cluster_scale = parameter.as_double();
      perception_.setClusterGUIScale(static_cast<float>(cluster_scale));

    } else if (parameter.get_name() == "free_space_max_range") {
      const auto max_range = parameter.as_double();
      perception_.setFreeSpaceRange(static_cast<float>(max_range));

    } else if (parameter.get_name() == "free_space_upper_bound") {
      const auto screening_max = parameter.as_double();
      perception_.setFreeSpaceScreeningMax(static_cast<float>(screening_max));

    } else if (parameter.get_name() == "free_space_lower_bound") {
      const auto screening_min = parameter.as_double();
      perception_.setFreeSpaceScreeningMin(static_cast<float>(screening_min));
    } else if (parameter.get_name() == "perception_config") {
      const auto perception_config = parameter.as_int();
      const bool success = perception_.setConfig(
          static_cast<common::PERCEPTION_CONFIG>(perception_config));
      if (!success) {
        result.successful = false;
        result.reason = "Invalid perception configuration";
      } else {
        control_state_msg_.perception_config_name =
            perception_.getCurrentConfigName();
        result.reason = "Perception configuration changed";
      }

    } else {
      result.successful = false;
      result.reason = "invalid_parameter";
    }
  }

  publishControlState();
  // Here update class attributes, do some actions, etc.
  return result;
}

// Function that grabs incoming point-cloud and creates
// internal "RadarData" representation
void RadarPerception::createRadarDataFromPointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud2) {
  // Prepare iterators
  PointCloud2ConstIterator in_range_it(*pointcloud2, "range");
  PointCloud2ConstIterator in_elevation_it(*pointcloud2, "elevation");
  PointCloud2ConstIterator in_azimuth_it(*pointcloud2, "azimuth");
  PointCloud2ConstIterator in_power_it(*pointcloud2, "power");
  PointCloud2ConstIterator in_doppler_it(*pointcloud2, "doppler");
  PointCloud2ConstIterator in_x_it(*pointcloud2, "x");
  PointCloud2ConstIterator in_y_it(*pointcloud2, "y");
  PointCloud2ConstIterator in_z_it(*pointcloud2, "z");

  // Get frame id
  frame_id_ = pointcloud2->header.frame_id;

  // Detect if annotation is available
  bool annotation_is_available = std::any_of(
      pointcloud2->fields.begin(), pointcloud2->fields.end(),
      [](const auto &field) { return field.name == "annotation_cluster_idx"; });

  radar_time_sec_ = pointcloud2->header.stamp.sec;
  radar_time_nanosec_ = pointcloud2->header.stamp.nanosec;
  auto radar_time = static_cast<double>(radar_time_sec_) +
                    static_cast<double>(radar_time_nanosec_) / NANOSEC_SCALE;
  radar_data_.setTimeStamp_ms(
      static_cast<uint64_t>(radar_time * MILLISEC_SCALE));
  RCLCPP_DEBUG(get_logger(), "Radar time (msec): %ld",
               radar_data_.getRadarTime_ms());

  // Clear memory.
  radar_data_.clearPointCloud();
  outgoing_pointcloud_.clear();
  annotated_pointcloud_.clear();

  // Convert incoming PointCloud2 to internal "RadarData" format
  std::size_t ctr = 0;
  while (in_range_it != in_range_it.end()) {

    // Check for NaN
    bool is_nan = std::isnan(*in_range_it) || std::isnan(*in_elevation_it) ||
                  std::isnan(*in_azimuth_it) || std::isnan(*in_power_it) ||
                  std::isnan(*in_doppler_it) || std::isnan(*in_x_it) ||
                  std::isnan(*in_y_it) || std::isnan(*in_z_it);

    if (is_nan) {
      RCLCPP_WARN(get_logger(), "NaN in point-cloud, skipping point");
      ++in_range_it;
      ++in_elevation_it;
      ++in_azimuth_it;
      ++in_power_it;
      ++in_doppler_it;
      ++in_x_it;
      ++in_y_it;
      ++in_z_it;
      continue;
    }

    const common::InputPoint incoming_point(
        *in_range_it, *in_elevation_it, *in_azimuth_it, *in_power_it,
        *in_doppler_it, *in_x_it, *in_y_it, *in_z_it, ctr);
    radar_data_.addPoint(incoming_point);

    ++in_range_it;
    ++in_elevation_it;
    ++in_azimuth_it;
    ++in_power_it;
    ++in_doppler_it;
    ++in_x_it;
    ++in_y_it;
    ++in_z_it;
    ctr++;
  }
  radar_data_.sortByRange();

  // Create annotated cloud. If not present in input, fill with default values
  if (annotation_is_available) {

    PointCloud2IteratorINT16 in_annotation_cluster_idx_it(
        *pointcloud2, "annotation_cluster_idx");

    PointCloud2IteratorUINT8 in_annotation_class_it(*pointcloud2,
                                                    "annotation_class");

    while (in_annotation_cluster_idx_it != in_annotation_cluster_idx_it.end()) {
      annotated_pointcloud_.emplace_back(*in_annotation_cluster_idx_it,
                                         *in_annotation_class_it);

      ++in_annotation_cluster_idx_it;
      ++in_annotation_class_it;
    }
  } else {
    const AnnotationPoint default_output{-1, 0};
    annotated_pointcloud_.clear();
    annotated_pointcloud_.resize(radar_data_.size(), default_output);
  }
}

// Callback function for receiving point clouds from the radar
void RadarPerception::radarDataCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud2) {

  auto t1 = std::chrono::high_resolution_clock::now();
  createRadarDataFromPointcloud(pointcloud2);

  // Run perception
  outgoing_pointcloud_ = perception_.update(radar_data_);

  // Get the tracked objects
  perception_.updateTrackedObjects(tracked_objects_);

  // Common header for all messages
  std_msgs::msg::Header header;
  header.stamp.sec = radar_time_sec_;
  header.stamp.nanosec = radar_time_nanosec_;
  header.frame_id = frame_id_;

  // Publish the results
  publisher_->publishGroundplane(perception_.groundPlaneAvailable(),
                                 perception_.groundPlane(), header);
  publisher_->publishDetections(perception_.getDetections(), header);
  publisher_->publishEgoMotion(
      perception_.getVelocityXYZRadar(), perception_.isVelocityAvailable(),
      perception_.getRotationalRateXYZRadar(), perception_.isOdometryActive(),
      perception_.getPoseTranslation(), perception_.getPoseQuaternion(),
      perception_.getDeltaPoseTranslation(),
      perception_.getDeltaPoseQuaternion(), header);
  publisher_->publishOdometryLocalMap(perception_.getOdometryMap(), header);
  publisher_->publishTrackedObjects(tracked_objects_, header);
  publisher_->publishExtendedPointcloud(outgoing_pointcloud_,
                                        annotated_pointcloud_, header);
  publisher_->publishStatistics(tracked_objects_,
                                perception_.getVelocityXYZRadar(), header);
  publisher_->publishRegionOfInterest(perception_.getROI(), header);

  publisher_->publishStaticMap(perception_.isDynamic(),
                               perception_.staticMapResolution(),
                               perception_.staticMapCoordinates(),
                               perception_.staticMapAnomalies(), header);

  publisher_->publishOccupancyGrid(
      perception_.isOccupancyGridActive(), perception_.occupancyGrid(),
      perception_.occupancyGridVoxelSize(), header);

  publisher_->publishFreeSpaceVolume(
      perception_.freeSpaceMatrix(), perception_.freeSpaceAzimuthAngles(),
      perception_.freeSpaceElevationAngles(), header);

  publisher_->publishFreeSpaceVolumeRaw(
      perception_.freeSpaceMatrix(), perception_.freeSpaceAzimuthAngles(),
      perception_.freeSpaceElevationAngles(), header);

  publisher_->publishFreeSpace(perception_.getFreeSpaceGroundPolygon(), header,
                               perception_.groundPlaneAvailable());

  // For debug and profiling
  auto t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> ms_double = t2 - t1;

  RCLCPP_DEBUG(get_logger(), "Received pointcloud with %d points",
               pointcloud2->width * pointcloud2->height);

  RCLCPP_DEBUG(get_logger(), "Execution time: %f (ms)",
               static_cast<double>(ms_double.count()));

  // Check status
  auto status = perception_.getStatus();
  if (status == common::Status::LARGE_TIME_GAP) {
    RCLCPP_WARN(get_logger(), "Reset due to large time gap between updates.");
  } else if (status == common::Status::SHORT_TIME_GAP) {
    RCLCPP_WARN(get_logger(), "Short time gap between updates.");
  } else if (status == common::Status::NEGATIVE_TIME_GAP) {
    RCLCPP_WARN(get_logger(), "Negative time gap between updates, resetting.");
  }
}

// Get radar reolution from  the radar header
void RadarPerception::resolutionDataCallback(
    const raf2_interfaces::msg::Header::SharedPtr header_msg) {
  const common::RadarResolution rr(
      true, header_msg->azimuth_resolution, header_msg->elevation_resolution,
      header_msg->range_resolution, header_msg->doppler_resolution);
  radar_data_.setRadarResolution(rr);
}

void RadarPerception::createSubscriptions() {

  // Radar point cloud - Reliable QoS with keep_last(1) for real-time processing
  // Reduced from depth 10 to depth 1 to prevent queue buildup when processing
  // is slow. This ensures we always process the latest frame instead of
  // building up a backlog. Compatible with publisher's Reliable QoS and RViz.
  auto radar_qos = rclcpp::QoS(1).reliable().durability_volatile();

  // Subscribe to radar_data topic
  pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "radar_data", radar_qos,
      std::bind(&RadarPerception::radarDataCallback, this,
                std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Subscribing to radar data from topic: radar_data "
                            "with Reliable QoS (keep_last=1)");

  // Radar header/resolution - keep default QOS_BACKLOG (infrequent updates)
  resolution_subscription_ = create_subscription<raf2_interfaces::msg::Header>(
      "radar_header", QOS_BACKLOG,
      std::bind(&RadarPerception::resolutionDataCallback, this,
                std::placeholders::_1));
}

// Publish the control state
void RadarPerception::publishControlState() {
  control_state_msg_.header.stamp.sec = radar_time_sec_;
  control_state_msg_.header.stamp.nanosec = radar_time_nanosec_;
  control_state_msg_.header.frame_id = frame_id_;
  control_state_publisher_->publish(control_state_msg_);
}

// Constructor for object running Sensrad radar perception
// Some fields are not necessary for initialization
// NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init)
RadarPerception::RadarPerception()
    : Node("oden"), radar_data_{}, perception_{MAX_POINTS_TO_PROCESS},
      radar_time_sec_{0}, radar_time_nanosec_{0}, frame_id_{"radar_1"} {

  // Default values for oden parameters
  declare_parameter<std::string>("radar_topic", "radar_data");
  declare_parameter("perception_config", 0);
  declare_parameter<float>("vertical_height", 1.0f);
  declare_parameter("radar_elevation_angle", 0.0f);
  declare_parameter("cluster_scale", 1.0f);
  declare_parameter("free_space_max_range", 30.0f);
  declare_parameter("free_space_lower_bound", 0.4f);
  declare_parameter("free_space_upper_bound", 3.0f);
  declare_parameter("stationary_updates", 100);
  declare_parameter("region_of_interest.x_center", 0.0f);
  declare_parameter("region_of_interest.y_center", 0.0f);
  declare_parameter("region_of_interest.z_center", 0.0f);
  declare_parameter("region_of_interest.width", 0.0f);
  declare_parameter("region_of_interest.length", 0.0f);
  declare_parameter("region_of_interest.height", 0.0f);
  declare_parameter("region_of_interest.yaw", 0.0f);
  declare_parameter("region_of_interest.pitch", 0.0f);
  declare_parameter("region_of_interest.enabled", false);

  publisher_ = std::make_unique<RadarPublisher>(this);

  // Get values from launch configuration
  const float vertical_height =
      static_cast<float>(get_parameter("vertical_height").as_double());
  const float radar_elevation_angle =
      static_cast<float>(get_parameter("radar_elevation_angle").as_double());
  const float cluster_scale =
      static_cast<float>(get_parameter("cluster_scale").as_double());
  const float free_space_max_range =
      static_cast<float>(get_parameter("free_space_max_range").as_double());
  const float free_space_lower_bound =
      static_cast<float>(get_parameter("free_space_lower_bound").as_double());
  const float free_space_upper_bound =
      static_cast<float>(get_parameter("free_space_upper_bound").as_double());
  const int stationary_updates =
      static_cast<int>(get_parameter("stationary_updates").as_int());
  const int perception_config =
      static_cast<int>(get_parameter("perception_config").as_int());

  common::RegionOfInterest roi_params;
  roi_params.x_center = static_cast<float>(
      get_parameter("region_of_interest.x_center").as_double());
  roi_params.y_center = static_cast<float>(
      get_parameter("region_of_interest.y_center").as_double());
  roi_params.z_center = static_cast<float>(
      get_parameter("region_of_interest.z_center").as_double());
  roi_params.width =
      static_cast<float>(get_parameter("region_of_interest.width").as_double());
  roi_params.length = static_cast<float>(
      get_parameter("region_of_interest.length").as_double());
  roi_params.height = static_cast<float>(
      get_parameter("region_of_interest.height").as_double());
  roi_params.yaw =
      static_cast<float>(get_parameter("region_of_interest.yaw").as_double());
  roi_params.pitch =
      static_cast<float>(get_parameter("region_of_interest.pitch").as_double());
  const bool roi_enabled =
      get_parameter("region_of_interest.enabled").as_bool();

  const bool success = perception_.setConfig(
      static_cast<common::PERCEPTION_CONFIG>(perception_config));
  const std::string perception_mode_name = perception_.getCurrentConfigName();
  if (!success) {
    RCLCPP_ERROR(get_logger(), "Invalid perception configuration, using %s",
                 perception_mode_name.c_str());
    return;
  }
  perception_.setVerticalHeight(vertical_height);
  perception_.setElevationAngle(radar_elevation_angle);
  perception_.setClusterGUIScale(cluster_scale);
  perception_.setFreeSpaceRange(free_space_max_range);
  perception_.setFreeSpaceScreeningMin(free_space_lower_bound);
  perception_.setFreeSpaceScreeningMax(free_space_upper_bound);
  perception_.setStationaryUpdatesMax(stationary_updates);
  perception_.setROI(roi_params);
  perception_.setROIMode(roi_enabled);

  // Get all

  // Print configuration
  RCLCPP_INFO(get_logger(), "Oden configured with:");
  RCLCPP_INFO(get_logger(), "  config: %d (%s)", perception_config,
              perception_mode_name.c_str());
  RCLCPP_INFO(get_logger(), "  vertical_height: %f", vertical_height);
  RCLCPP_INFO(get_logger(), "  radar_elevation_angle: %f",
              radar_elevation_angle);
  RCLCPP_INFO(get_logger(), "  cluster_scale: %f", cluster_scale);
  RCLCPP_INFO(get_logger(), "  free_space_max_range: %f", free_space_max_range);
  RCLCPP_INFO(get_logger(), "  free_space_lower_bound: %f",
              free_space_lower_bound);
  RCLCPP_INFO(get_logger(), "  free_space_upper_bound: %f",
              free_space_upper_bound);
  RCLCPP_INFO(get_logger(), "  stationary_updates: %d", stationary_updates);

  // Make space for pointclouds and objects
  outgoing_pointcloud_.reserve(POINTCLOUD_RESERVE);
  annotated_pointcloud_.reserve(POINTCLOUD_RESERVE);
  tracked_objects_.reserve(TRACKED_OBJECTS_RESERVE);

  createSubscriptions();

  // Setup publisher with QoS profile durability local
  auto control_state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  control_state_qos.transient_local();
  control_state_publisher_ =
      create_publisher<oden_interfaces::msg::OdenControlState>(
          "oden/control_state", control_state_qos);

  reset_perception_service_ =
      create_service<oden_interfaces::srv::OdenCtrlResetPerception>(
          "oden/reset_perception",
          [&](const std::shared_ptr<
                  oden_interfaces::srv::OdenCtrlResetPerception::Request>,
              std::shared_ptr<
                  oden_interfaces::srv::OdenCtrlResetPerception::Response>
                  response) {
            RCLCPP_WARN(get_logger(), "Resetting perception");
            perception_.reset();
            response->success = true;
            response->message = "reset_perception";
            RCLCPP_INFO(get_logger(), "Success");
            // You might need to set a response here if the service expects
            // one
            publishControlState();
          });

  update_parameters_service_ = create_service<
      oden_interfaces::srv::OdenCtrlSetPerceptionParameters>(
      "oden/set_perception_parameters",
      [&](const std::shared_ptr<
              oden_interfaces::srv::OdenCtrlSetPerceptionParameters::Request>
              request,
          std::shared_ptr<
              oden_interfaces::srv::OdenCtrlSetPerceptionParameters::Response>
              response) {
        RCLCPP_INFO(get_logger(), "Setting perception parameters");

        set_parameters(
            {rclcpp::Parameter("vertical_height", request->vertical_height),
             rclcpp::Parameter("radar_elevation_angle",
                               request->elevation_angle),
             rclcpp::Parameter("cluster_scale", request->cluster_scale),
             rclcpp::Parameter("free_space_max_range",
                               request->free_space_max_range),
             rclcpp::Parameter("free_space_lower_bound",
                               request->free_space_lower_bound),
             rclcpp::Parameter("free_space_upper_bound",
                               request->free_space_upper_bound),
             rclcpp::Parameter("stationary_updates",
                               static_cast<int>(request->stationary_updates)),
             rclcpp::Parameter("free_space_upper_bound",
                               request->free_space_upper_bound),
             rclcpp::Parameter("perception_config",
                               request->perception_config)});

        control_state_msg_.vertical_height = request->vertical_height;
        control_state_msg_.elevation_angle = request->elevation_angle;
        control_state_msg_.cluster_scale = request->cluster_scale;
        control_state_msg_.free_space_range = request->free_space_max_range;
        control_state_msg_.free_space_lower_bound =
            request->free_space_lower_bound;
        control_state_msg_.free_space_upper_bound =
            request->free_space_upper_bound;
        control_state_msg_.stationary_updates = request->stationary_updates;
        control_state_msg_.perception_config = request->perception_config;

        response->success = true;
        response->message = "set_perception_parameters";
        RCLCPP_INFO(get_logger(), "Success");
        // You might need to set a response here if the service expects
        // one
        publishControlState();
      });

  update_roi_service_ =
      create_service<oden_interfaces::srv::OdenCtrlSetRoiParameters>(
          "oden/set_roi_parameters",
          [&](const std::shared_ptr<
                  oden_interfaces::srv::OdenCtrlSetRoiParameters::Request>
                  request,
              std::shared_ptr<
                  oden_interfaces::srv::OdenCtrlSetRoiParameters::Response>
                  response) {
            RCLCPP_INFO(get_logger(), "Setting Region of Interest parameters");

            rclcpp::Parameter roi_x_center("region_of_interest.x_center",
                                           request->roi_x_center);
            rclcpp::Parameter roi_y_center("region_of_interest.y_center",
                                           request->roi_y_center);
            rclcpp::Parameter roi_z_center("region_of_interest.z_center",
                                           request->roi_z_center);
            rclcpp::Parameter roi_width("region_of_interest.width",
                                        request->roi_width);
            rclcpp::Parameter roi_length("region_of_interest.length",
                                         request->roi_length);
            rclcpp::Parameter roi_height("region_of_interest.height",
                                         request->roi_height);
            rclcpp::Parameter roi_yaw("region_of_interest.yaw",
                                      request->roi_yaw);
            rclcpp::Parameter roi_pitch("region_of_interest.pitch",
                                        request->roi_pitch);
            rclcpp::Parameter roi_enabled("region_of_interest.enabled",
                                          request->enabled);

            common::RegionOfInterest roi;
            roi.x_center = request->roi_x_center;
            roi.y_center = request->roi_y_center;
            roi.z_center = request->roi_z_center;
            roi.width = request->roi_width;
            roi.length = request->roi_length;
            roi.height = request->roi_height;
            roi.yaw = request->roi_yaw;
            roi.pitch = request->roi_pitch;
            perception_.setROI(roi);
            perception_.setROIMode(request->enabled);

            // Update the control state message
            control_state_msg_.roi_x_center = request->roi_x_center;
            control_state_msg_.roi_y_center = request->roi_y_center;
            control_state_msg_.roi_z_center = request->roi_z_center;
            control_state_msg_.roi_width = request->roi_width;
            control_state_msg_.roi_length = request->roi_length;
            control_state_msg_.roi_height = request->roi_height;
            control_state_msg_.roi_yaw = request->roi_yaw;
            control_state_msg_.roi_pitch = request->roi_pitch;
            control_state_msg_.roi_enabled = request->enabled;

            response->success = true;
            response->message = "set_roi_parameters";
            RCLCPP_INFO(get_logger(), "Success");
            // You might need to set a response here if the service expects
            // one
            publishControlState();
          });

  // Default values for control state
  control_state_msg_.vertical_height = vertical_height;
  control_state_msg_.elevation_angle = radar_elevation_angle;
  control_state_msg_.cluster_scale = cluster_scale;
  control_state_msg_.free_space_range = free_space_max_range;
  control_state_msg_.free_space_lower_bound = free_space_lower_bound;
  control_state_msg_.free_space_upper_bound = free_space_upper_bound;
  control_state_msg_.stationary_updates = stationary_updates;
  control_state_msg_.perception_config = perception_config;
  control_state_msg_.perception_config_name = perception_mode_name;
  control_state_msg_.roi_x_center = roi_params.x_center;
  control_state_msg_.roi_y_center = roi_params.y_center;
  control_state_msg_.roi_z_center = roi_params.z_center;
  control_state_msg_.roi_width = roi_params.width;
  control_state_msg_.roi_length = roi_params.length;
  control_state_msg_.roi_height = roi_params.height;
  control_state_msg_.roi_yaw = roi_params.yaw;
  control_state_msg_.roi_pitch = roi_params.pitch;
  control_state_msg_.roi_enabled = roi_enabled;

  // Get all perception configurations and add to control state message.
  // Required only at start up of the node.
  const auto config_map = perception_.getConfigMap();
  for (const auto &config : config_map) {
    control_state_msg_.config_identifiers.push_back(
        static_cast<uint8_t>(config.first));
    control_state_msg_.config_names.push_back(config.second);
  }

  publishControlState();

  RCLCPP_INFO(get_logger(), "Oden node started");

  // Launch service for changing parameters
  param_callback_handle_ = add_on_set_parameters_callback(std::bind(
      &RadarPerception::onParametersCallback, this, std::placeholders::_1));
};
