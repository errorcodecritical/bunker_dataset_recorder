// Copyright (c) Sensrad 2024-2025

#include "PolygonGenerator.hpp"
#include "rclcpp/rclcpp.hpp"

namespace perception_visualizer {

// Init of static member variables
const std::string PolygonGenerator::AREA_MARKER_NS = "area_marker";
const std::string PolygonGenerator::BORDER_LINE_MARKER_NS =
    "border_line_marker";

const std::unordered_map<std::string, PolygonGenerator::Color>
    PolygonGenerator::MARKER_COLOR_MAP = {
        {AREA_MARKER_NS, {0.6, 0.0, 0.8, 0.1}},
        {BORDER_LINE_MARKER_NS, {1.0, 0.0, 0.8, 0.1}}};

const std::unordered_map<common::Type, PolygonGenerator::Color>
    PolygonGenerator::CLASS_COLOR_MAP = {
        {common::Type::CLASS_UNKNOWN,
         {CLASS_OPACITY, 0.7, 0.7, 0.7}}, // Light Grey
        {common::Type::CLASS_PEDESTRIAN, {CLASS_OPACITY, 1.0, 0.0, 0.0}}, // Red
        {common::Type::CLASS_BICYCLE, {CLASS_OPACITY, 1.0, 0.5, 0.0}}, // Orange
        {common::Type::CLASS_CAR, {CLASS_OPACITY, 1.0, 0.0, 0.7}},     // Pink
        {common::Type::CLASS_UTILITY_VEHICLE,
         {CLASS_OPACITY, 0.528, 0.808, 1.0}}, // Sky Blue
        {common::Type::CLASS_TRUCK,
         {CLASS_OPACITY, 1.0, 0.863, 0.4}} // Sooty Yellow
};

std::string PolygonGenerator::typeToString(common::Type type) {
  switch (type) {
  case common::Type::CLASS_UNKNOWN:
    return "UNKNOWN";
  case common::Type::CLASS_PEDESTRIAN:
    return "PEDESTRIAN";
  case common::Type::CLASS_BICYCLE:
    return "BICYCLE";
  case common::Type::CLASS_CAR:
    return "CAR";
  case common::Type::CLASS_UTILITY_VEHICLE:
    return "UTILITY_VEHICLE";
  case common::Type::CLASS_TRUCK:
    return "TRUCK";
  default:
    return "Invalid Type";
  }
};

// Populate vertices in a polygon
void PolygonGenerator::populateVertices(geometry_msgs::msg::Polygon &polygon,
                                        const Eigen::Vector3f &point) {
  // Function to add vertices to polygon
  geometry_msgs::msg::Point32 v;
  v.x = point[POINT_DIM_X];
  v.y = point[POINT_DIM_Y];
  v.z = point[POINT_DIM_Z];

  polygon.points.push_back(v);
}

// Create a marker for the AABB (axis aligned bounding box) object
float PolygonGenerator::createAABBMarker(
    const oden_interfaces::msg::ObjectTrack &tracked_object,
    const std_msgs::msg::Header &header,
    visualization_msgs::msg::Marker &aabb_marker_object,
    const float radar_elevation_angle) {

  aabb_marker_object.header = header;
  aabb_marker_object.ns = "aabb_object_tracks";
  aabb_marker_object.id = static_cast<int>(tracked_object.track_id);
  aabb_marker_object.type = visualization_msgs::msg::Marker::CUBE;
  aabb_marker_object.action = visualization_msgs::msg::Marker::ADD;

  aabb_marker_object.pose.orientation.x = 0.0;
  aabb_marker_object.pose.orientation.y = 0.0;
  aabb_marker_object.pose.orientation.z = 0.0;
  aabb_marker_object.pose.orientation.w = 1.0;

  Eigen::Matrix3f rotation_matrix_x;
  Eigen::Matrix3f rotation_matrix_y;
  Eigen::Matrix3f rotation_matrix_z;
  Eigen::Matrix3f rotation_matrix;

  // Yaw angle of the object's heading relative to the z-axis of the radar
  // coordinate system. This z-axis is aligned with the z-axis of the
  // adjustable_frame.
  const float object_yaw_angle = tracked_object.yaw;

  // Radar pitch angle, angle of radar_frame around x-axis relative to
  // adjustable_frame around x-axis
  const float radar_pitch_angle = radar_elevation_angle * DEG_2_RAD;

  rotation_matrix_x << 1, 0, 0, 0, std::cos(radar_pitch_angle),
      -std::sin(radar_pitch_angle), 0, std::sin(radar_pitch_angle),
      std::cos(radar_pitch_angle);                  // Bounding box Yaw
  rotation_matrix_y << Eigen::Matrix3f::Identity(); // Bounding box Roll
  rotation_matrix_z << std::cos(object_yaw_angle), -std::sin(object_yaw_angle),
      0, std::sin(object_yaw_angle), std::cos(object_yaw_angle), 0, 0, 0,
      1; // Bounding box Pitch

  rotation_matrix =
      rotation_matrix_z * rotation_matrix_y *
      rotation_matrix_x; // Extrinsic rotation relative to radar_frame

  Eigen::Matrix3f extent;

  for (int i = 0; i < EXTENT_SIZE * EXTENT_SIZE; ++i) {
    extent(i / EXTENT_SIZE, i % EXTENT_SIZE) = tracked_object.extent[i];
  }

  const Eigen::Matrix3f rotated_extent =
      rotation_matrix * extent * rotation_matrix.transpose();

  // Compute sizes of bounding box. This is a heuristic, would be
  // more accurate to compute eigenvalues of rotated_extent and use those.
  // But this is good enough for now.
  const float x_half_size =
      std::sqrt(std::max(rotated_extent(0, 0), MIN_SIZE)) * NOF_STD_AABB / 2.0F;
  const float y_half_size =
      std::sqrt(std::max(rotated_extent(1, 1), MIN_SIZE)) * NOF_STD_AABB / 2.0F;
  const float z_half_size =
      std::sqrt(std::max(rotated_extent(2, 2), MIN_SIZE)) * NOF_STD_AABB / 2.0F;

  // Compute points on the bounding box and add to the marker
  const Eigen::Vector3f p_center = {tracked_object.position[POINT_DIM_X],
                                    tracked_object.position[POINT_DIM_Y],
                                    tracked_object.position[POINT_DIM_Z]};

  const Eigen::Quaternion pose_quaternion =
      Eigen::Quaternionf(rotation_matrix.transpose()).normalized();
  aabb_marker_object.pose.orientation.x = pose_quaternion.x();
  aabb_marker_object.pose.orientation.y = pose_quaternion.y();
  aabb_marker_object.pose.orientation.z = pose_quaternion.z();
  aabb_marker_object.pose.orientation.w = pose_quaternion.w();

  aabb_marker_object.pose.position.x = p_center[POINT_DIM_X];
  aabb_marker_object.pose.position.y = p_center[POINT_DIM_Y];
  aabb_marker_object.pose.position.z = p_center[POINT_DIM_Z];

  aabb_marker_object.scale.x = 2.0F * x_half_size;
  aabb_marker_object.scale.y = 2.0F * y_half_size;
  aabb_marker_object.scale.z = 2.0F * z_half_size;

  // Set color based on type, and cast to common::Type
  auto type = static_cast<common::Type>(tracked_object.type);
  auto this_color = CLASS_COLOR_MAP.find(type) == CLASS_COLOR_MAP.end()
                        ? CLASS_COLOR_MAP.at(common::Type::CLASS_UNKNOWN)
                        : CLASS_COLOR_MAP.at(type);
  aabb_marker_object.color.a = this_color.a;
  aabb_marker_object.color.r = this_color.r;
  aabb_marker_object.color.g = this_color.g;
  aabb_marker_object.color.b = this_color.b;
  aabb_marker_object.lifetime = rclcpp::Duration::from_seconds(FADE_DURATION);
  return z_half_size;
}

// Create a marker for the label text of the AABB (axis aligned bounding box)
// object
void PolygonGenerator::createTextMarker(
    const oden_interfaces::msg::ObjectTrack &tracked_object,
    const std_msgs::msg::Header &header, float aabb_half_hight,
    visualization_msgs::msg::Marker &text_marker_object,
    const std::unordered_map<uint32_t, TrackIdManager> &trackid_manager_map) {

  const Eigen::Vector3f p_center = {tracked_object.position[POINT_DIM_X],
                                    tracked_object.position[POINT_DIM_Y],
                                    tracked_object.position[POINT_DIM_Z]};

  text_marker_object.header = header;
  text_marker_object.ns = "aabb_object_texts";
  text_marker_object.id = static_cast<int>(tracked_object.track_id);
  text_marker_object.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_object.action = visualization_msgs::msg::Marker::ADD;
  text_marker_object.pose.position.x = p_center[POINT_DIM_X];
  text_marker_object.pose.position.y = p_center[POINT_DIM_Y];
  text_marker_object.pose.position.z =
      p_center[POINT_DIM_Z] + aabb_half_hight + TEXT_OFFSET;
  text_marker_object.scale.z = TEXT_SIZE;

  // Set color based on type, and cast to common::Type
  auto type = static_cast<common::Type>(tracked_object.type);

  auto this_color = CLASS_COLOR_MAP.at(type);
  text_marker_object.color.a = 1.0f; // Fully opaque
  text_marker_object.color.r = this_color.r;
  text_marker_object.color.g = this_color.g;
  text_marker_object.color.b = this_color.b;

  text_marker_object.lifetime = rclcpp::Duration::from_seconds(FADE_DURATION);

  // Calculate the speed of the object
  float speed_kmh =
      MS_2_KMH *
      std::sqrt(tracked_object.velocity[0] * tracked_object.velocity[0] +
                tracked_object.velocity[1] * tracked_object.velocity[1] +
                tracked_object.velocity[2] * tracked_object.velocity[2]);

  std::ostringstream text_stream;

  // Replace the track id with the output track id if the track is mature, if
  // so desired (flag OUTPUT_COUNT_MATURE_ONLY controls this)
  uint32_t displayed_track_id = tracked_object.track_id;
  if (OUTPUT_COUNT_MATURE_ONLY) {
    if (trackid_manager_map.find(tracked_object.track_id) !=
        trackid_manager_map.end()) {
      displayed_track_id =
          trackid_manager_map.at(tracked_object.track_id).output_track_id;
    }
  }

  text_stream << std::to_string(displayed_track_id) << ". " << std::fixed
              << std::setprecision(1) << speed_kmh << "km/h \n"
              << PolygonGenerator::typeToString(type);

  text_marker_object.text = text_stream.str();
}

// Generate AABB (axis aligned bounding box) and text object for visualization
visualization_msgs::msg::MarkerArray PolygonGenerator::getAABBObjectTracks(
    const oden_interfaces::msg::MultiObjectTracking::ConstSharedPtr
        &tracked_objects,
    const std::unordered_map<uint32_t, TrackIdManager> &trackid_manager_map,
    const float radar_elevation_angle) {
  // Define message for visualization
  // visualization_msgs::msg::MarkerArray marker_objects;
  visualization_msgs::msg::MarkerArray marker_objects;

  // Loop over the tracked objects and populate the marker array
  for (unsigned int idx = 0; idx < tracked_objects->object_track_list.size();
       ++idx) {
    const std_msgs::msg::Header &header = tracked_objects->header;
    const auto &tracked_object = tracked_objects->object_track_list[idx];

    // Visualize only mature objects
    if (tracked_object.track_state == common::TrackState::MATURE) {
      // Create a new marker for the AABB
      visualization_msgs::msg::Marker aabb_marker_object;
      visualization_msgs::msg::Marker text_marker_object;

      float aabb_half_hight = PolygonGenerator::createAABBMarker(
          tracked_object, header, aabb_marker_object, radar_elevation_angle);
      PolygonGenerator::createTextMarker(tracked_object, header,
                                         aabb_half_hight, text_marker_object,
                                         trackid_manager_map);

      // Populate the marker array
      marker_objects.markers.push_back(aabb_marker_object);
      marker_objects.markers.push_back(text_marker_object);
    }
  }
  return marker_objects;
}

visualization_msgs::msg::Marker
PolygonGenerator::getFreeSpaceGroundPolygonMarker(
    const std::vector<Eigen::Vector3f> &polygon,
    const std_msgs::msg::Header &header, const bool ground_plane_is_valid) {

  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "free_space_ground";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  // Pose
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Scale (not used for triangle list, but required)
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Set color based on ground plane state
  // Light green (NOMINAL) when ground_plane_is_valid is false
  // Current green (ACTIVE) when ground_plane_is_valid is true
  if (ground_plane_is_valid) {
    // Current green for ACTIVE state
    marker.color.r = 0.0F;
    marker.color.g = 1.0F;
    marker.color.b = 0.0F;
    marker.color.a = 0.8F;
  } else {
    // Light green for NOMINAL state
    marker.color.r = 0.6F;
    marker.color.g = 1.0F;
    marker.color.b = 0.6F;
    marker.color.a = 0.4F;
  }

  // Create triangles by connecting consecutive polygon edges to origin
  // Polygon structure: [origin, p1, p2, ..., pN, origin]
  if (polygon.size() >= 3) {
    std_msgs::msg::ColorRGBA color;
    color.r = 0.0F;
    color.g = 1.0F;
    color.b = 0.0F;
    color.a = 0.8F;

    for (size_t i = 1; i < polygon.size() - 1; ++i) {
      // Triangle: origin, point[i], point[i+1]
      geometry_msgs::msg::Point p0;
      geometry_msgs::msg::Point p1;
      geometry_msgs::msg::Point p2;

      // Polygon is already in radar frame (x, y, z)
      p0.x = polygon[0].x();
      p0.y = polygon[0].y();
      p0.z = polygon[0].z();

      p1.x = polygon[i].x();
      p1.y = polygon[i].y();
      p1.z = polygon[i].z();

      p2.x = polygon[i + 1].x();
      p2.y = polygon[i + 1].y();
      p2.z = polygon[i + 1].z();

      marker.points.push_back(p0);
      marker.points.push_back(p1);
      marker.points.push_back(p2);

      // Add per-vertex colors (one for each vertex)
      marker.colors.push_back(color);
      marker.colors.push_back(color);
      marker.colors.push_back(color);
    }
  }

  return marker;
}
} // namespace perception_visualizer
