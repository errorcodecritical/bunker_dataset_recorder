/**
 * ouster_preprocess.cpp
 *
 * Converts Ouster OS1 PointCloud2 (with 't' and 'ring' fields) into a
 * sensor_msgs/PointCloud2 compatible with A-LOAM's scanRegistration.
 *
 * The output cloud uses pcl::PointXYZI where intensity encodes the
 * relative point timestamp within the scan (0.0 – 1.0), which A-LOAM
 * uses for motion undistortion via the 'relTime' mechanism.
 *
 * Remap:  input  → /ouster/points  (or /os_cloud_node/points)
 *         output → /velodyne_points  (so scanRegistration needs no topic change)
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

// Ouster per-point struct matching the driver's binary layout
struct OusterPoint {
    PCL_ADD_POINT4D;              // x @ 0, y @ 4, z @ 8, padding @ 12  → 16 bytes
    float    intensity;           // offset 16
    uint32_t t;                   // offset 20
    uint16_t reflectivity;        // offset 24
    uint16_t ring;                // offset 26  ← was wrongly uint8_t
    uint16_t ambient;             // offset 28
    uint32_t range;               // offset 32
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPoint,
    (float,    x,            x)
    (float,    y,            y)
    (float,    z,            z)
    (float,    intensity,    intensity)
    (uint32_t, t,            t)
    (uint16_t, reflectivity, reflectivity)
    (uint16_t, ring,         ring)          // ← fixed
    (uint16_t, ambient,      ambient)
    (uint32_t, range,        range)
)

void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<OusterPoint> cloud_in;
    pcl::fromROSMsg(*msg, cloud_in);

    pcl::PointCloud<pcl::PointXYZI> cloud_out;
    cloud_out.header = cloud_in.header;
    cloud_out.reserve(cloud_in.size());

    // Find scan duration for normalisation (max t field value)
    uint32_t t_max = 0;
    for (const auto &p : cloud_in.points)
        if (p.t > t_max) t_max = p.t;
    if (t_max == 0) t_max = 1;   // avoid division by zero

    for (const auto &p : cloud_in.points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;
        pcl::PointXYZI pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        // Encode relative time in intensity so scanRegistration can use it
        pt.intensity = p.ring + static_cast<float>(p.t) / static_cast<float>(t_max);
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
	nh.param<int>("queue_size", queue_size, 0);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>
        ("/ouster/points", queue_size, cloudHandler);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", queue_size);

    ROS_INFO("ouster_preprocess: converting /ouster/points → /velodyne_points");
    ros::spin();
    return 0;
}