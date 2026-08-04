/**
 * ouster_preprocess.cpp
 *
 * Converts PointCloud2 (with 'intensity', 'ring', 'timestamp', 'x', 'y', 'z' fields)
 * into a sensor_msgs/PointCloud2 compatible with A-LOAM's scanRegistration.
 *
 * Remap:  input  → /hesai/points
 *         output → /velodyne_points
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

// Updated per-point struct to match the incoming cloud fields:
// x, y, z, intensity, ring, timestamp
struct OusterPoint {
    PCL_ADD_POINT4D;              // x @ 0, y @ 4, z @ 8, padding @ 12  → 16 bytes
    float    intensity;           // offset 16
    uint16_t ring;                // offset 20
    double   timestamp;           // offset 24 (use double or uint32_t depending on topic format)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPoint,
    (float,    x,         x)
    (float,    y,         y)
    (float,    z,         z)
    (float,    intensity, intensity)
    (uint16_t, ring,      ring)
    (double,   timestamp, timestamp)
)

void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<OusterPoint> cloud_in;
    pcl::fromROSMsg(*msg, cloud_in);

    pcl::PointCloud<pcl::PointXYZI> cloud_out;
    cloud_out.header = cloud_in.header;
    cloud_out.reserve(cloud_in.size());

    // Find scan duration for normalisation (max timestamp value)
    double t_min = std::numeric_limits<double>::max();
    double t_max = 0.0;

    for (const auto &p : cloud_in.points) {
        if (p.timestamp > t_max) t_max = p.timestamp;
        if (p.timestamp < t_min) t_min = p.timestamp;
    }

    double t_diff = t_max - t_min;
    if (t_diff <= 0.0) t_diff = 1.0;   // avoid division by zero

    for (const auto &p : cloud_in.points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;

        pcl::PointXYZI pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        
        // Encode relative time in intensity so scanRegistration can use it
        pt.intensity = p.ring + static_cast<float>((p.timestamp - t_min) / t_diff);
        cloud_out.push_back(pt);
    }

    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(cloud_out, out_msg);
    out_msg.header = msg->header;
    pub.publish(out_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ouster_preprocess");
    ros::NodeHandle nh;
    int queue_size;
    nh.param<int>("queue_size", queue_size, 10);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>
        ("/hesai/points", queue_size, cloudHandler);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", queue_size);

    ROS_INFO("ouster_preprocess: converting /hesai/points → /velodyne_points");
    ros::spin();
    return 0;
}