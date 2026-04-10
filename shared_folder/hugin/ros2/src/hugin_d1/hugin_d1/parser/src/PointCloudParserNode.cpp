// Copyright (c) Sensrad 2024-2025

#include <PointCloudParserNode.hpp>

#include <PointCloud.hpp>
#include <iterator>
#include <rclcpp/logging.hpp>

#include <algorithm>

// Constructor for standalone node
PointCloudParserNode::PointCloudParserNode(const std::string &frame_id)
    : Node("pointcloud_parser_node"), pointcloud_modifier_(pointcloud_),
      frame_id_(frame_id) {

  initialize();
}

// Constructor for composable node (accepts NodeOptions)
PointCloudParserNode::PointCloudParserNode(const rclcpp::NodeOptions &options)
    : Node("pointcloud_parser_node", options),
      pointcloud_modifier_(pointcloud_), frame_id_("radar_1") {

  initialize();
}

// Common initialization for all constructors
void PointCloudParserNode::initialize() {
  RCLCPP_INFO(get_logger(), "pointcloud_parser_node has started");

  // Declare and get launch parameters
  frame_id_ = declare_parameter("frame_id", frame_id_);

  // Define pointcloud2 layout
  using PointField = sensor_msgs::msg::PointField;
  pointcloud_modifier_.setPointCloud2Fields(
      NUM_POINCLOUD_FIELDS, "x", 1, PointField::FLOAT32, "y", 1,
      PointField::FLOAT32, "z", 1, PointField::FLOAT32, "range", 1,
      PointField::FLOAT32, "elevation", 1, PointField::FLOAT32, "azimuth", 1,
      PointField::FLOAT32, "power", 1, PointField::FLOAT32, "doppler", 1,
      PointField::FLOAT32, "phase", 1, PointField::FLOAT32);

  // Create pointcloud publisher
  // Publisher qos
  rclcpp::QoS pointcloud_qos((rclcpp::KeepLast(QOS_BACKLOG)));

  pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "radar_data", pointcloud_qos);

  header_publisher_ = create_publisher<raf2_interfaces::msg::Header>(
      "radar_header", pointcloud_qos);

  // Reserve memory for pointcloud
  points_.reserve(INITIAL_POINTCLOUD_SIZE);
  pointcloud_modifier_.reserve(INITIAL_POINTCLOUD_SIZE);
  pointcloud_.height = 1;

  // PointCloud2 frame id
  pointcloud_.header.frame_id = frame_id_;
  // Header msg frame id
  header_msg_.header.frame_id = frame_id_;

  // Publisher qos
  rclcpp::QoS binary_data_qos((rclcpp::KeepLast(QOS_BACKLOG)));

  // Binary data subscription
  binary_data_subscription_ =
      create_subscription<raf2_interfaces::msg::BinaryData>(
          "binary_data", binary_data_qos,
          std::bind(&PointCloudParserNode::onBinaryData, this,
                    std::placeholders::_1));
}

// Destructor
PointCloudParserNode::~PointCloudParserNode() {}

void PointCloudParserNode::updateHeader(const PointCloud &pc) {

  // Convert radar time in milliseconds to ROS2 time format
  const auto radar_time_ms = std::chrono::milliseconds{pc.header().time_ms()};

  const auto secs =
      std::chrono::duration_cast<std::chrono::seconds>(radar_time_ms);
  const auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(
      radar_time_ms - secs);

  const auto radar_time_ros2 =
      rclcpp::Time(static_cast<int32_t>(secs.count()),
                   static_cast<uint32_t>(nsecs.count()), RCL_SYSTEM_TIME);

  header_msg_.header.stamp = radar_time_ros2;

  header_msg_.radar_time_ms = radar_time_ms.count();
  header_msg_.frame_counter = pc.header().frame_counter();
  header_msg_.is_4d = pc.header().metadata().doppler.is_4d();
  header_msg_.frame_type = pc.header().type();
  // This is the number of points in the message before "invalid"
  // points are filtered out.
  header_msg_.number_of_points = pc.num_points();
  header_msg_.crd_count = pc.header().crd_count();

  header_msg_.range_resolution = pc.header().metadata().range.coefficient();
  header_msg_.doppler_resolution = pc.header().metadata().doppler.coefficient();
  header_msg_.azimuth_resolution = pc.header().metadata().azimuth.coefficient();
  header_msg_.elevation_resolution =
      pc.header().metadata().elevation.coefficient();

  header_msg_.range_offset = pc.header().metadata().range.range_offset();
  header_msg_.azimuth_fft_size = pc.header().metadata().azimuth.fft_size();
  header_msg_.elevation_fft_size = pc.header().metadata().elevation.fft_size();
  header_msg_.azimuth_fft_padding =
      pc.header().metadata().azimuth.fft_padding();
  header_msg_.elevation_fft_padding =
      pc.header().metadata().elevation.fft_padding();

  header_msg_.range_fft_size = pc.header().metadata().range.fft_size();
  header_msg_.bytes_per_target = pc.header().point_size();
}

void PointCloudParserNode::publishPointCloud() {

  // Convert to pointcloud2 and publish
  pointcloud_modifier_.resize(points_.size());
  pointcloud_.height = 1;
  pointcloud_.width = points_.size();

  PointCloud2Iterator iter_x(pointcloud_, "x");
  PointCloud2Iterator iter_y(pointcloud_, "y");
  PointCloud2Iterator iter_z(pointcloud_, "z");
  PointCloud2Iterator iter_r(pointcloud_, "range");
  PointCloud2Iterator iter_el(pointcloud_, "elevation");
  PointCloud2Iterator iter_az(pointcloud_, "azimuth");
  PointCloud2Iterator iter_power(pointcloud_, "power");
  PointCloud2Iterator iter_doppler(pointcloud_, "doppler");
  PointCloud2Iterator iter_phase(pointcloud_, "phase");

  for (const auto point : points_) {

    *iter_x = point.x();
    *iter_y = point.y();
    *iter_z = point.z();
    *iter_r = point.range;
    *iter_el = point.elevation;
    *iter_az = point.azimuth;
    *iter_power = point.power;
    *iter_doppler = point.doppler;
    *iter_phase = point.phase;

    // Advance destination iterators
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_el;
    ++iter_az;
    ++iter_power;
    ++iter_doppler;
    ++iter_phase;
  }

  // Use the same timestamp as the header message.
  pointcloud_.header.stamp = header_msg_.header.stamp;

  // Publish pointcloud
  pointcloud_publisher_->publish(pointcloud_);

  // Clear pointcloud for next iteration
  pointcloud_modifier_.clear();
  points_.clear();
}

void PointCloudParserNode::logHeader(const PointCloud &pc) {

  RCLCPP_INFO(get_logger(), "Received point cloud data:");

  RCLCPP_INFO(get_logger(), "  - size: %zu bytes", pc.data().size());

  // Log the point count with different log levels based on the number of
  // points
  if (pc.num_points() == 0) {
    RCLCPP_WARN(get_logger(), "  - num_points: %zu", pc.num_points());
  } else {
    RCLCPP_INFO(get_logger(), "  - num_points: %zu", pc.num_points());
  }

  RCLCPP_INFO(get_logger(), "  - frame_counter: %u",
              pc.header().frame_counter());

  RCLCPP_INFO(get_logger(), "  - last_packet: %s",
              pc.header().is_last_packet() ? "true" : "false");

  RCLCPP_INFO(get_logger(), "  - Radar time: %lu ms", pc.header().time_ms());

  RCLCPP_INFO(get_logger(), "  - Point size: %zu bytes",
              pc.header().point_size());
}

void PointCloudParserNode::handlePointcloud(const PointCloud &pc) {

  // TODO: implement state machine.
  // Now it's just appended until last packet is set.

  // Copy all "valid" points to the pointcloud vector.
  std::copy_if(pc.begin(), pc.end(), std::back_inserter(points_),
               [](const auto &point) { return !point.is_bad(); });

  if (pc.header().is_last_packet()) {
    publishPointCloud();
  }
}

// Callback for incoming binary data
void PointCloudParserNode::onBinaryData(
    const raf2_interfaces::msg::BinaryData::SharedPtr msg) {

  RCLCPP_INFO(get_logger(), "Received binary message with size: %zu bytes",
              msg->data.size());

  // Log ROS2 timestamp
  RCLCPP_INFO(get_logger(), "Received binary message with ROS2 time:  %u:%u",
              msg->header.stamp.sec, msg->header.stamp.nanosec);

  if (msg->type != RDR_COMMON_COM_HEADER_TYPE_POINT_CLOUD) {
    RCLCPP_WARN(get_logger(), "Received non-point-cloud data type: %d",
                msg->type);
    // Not pointcloud data, return early
    RCLCPP_DEBUG(get_logger(), "Ignoring message with type %d, expected %d",
                 msg->type, RDR_COMMON_COM_HEADER_TYPE_POINT_CLOUD);
    return;
  }

  const auto pc = PointCloud(msg->data);

  // Advanced point cloud is not supported
  if (pc.header().is_advanced_pc()) {
    RCLCPP_INFO(get_logger(), "Ignoring point cloud with type 0x%X",
                pc.header().type());
    return;
  }
  logHeader(pc);

  // Update header message.
  updateHeader(pc);

  // Publish header message.
  header_publisher_->publish(header_msg_);

  // Handle point cloud data depending on the current state
  handlePointcloud(pc);
}
