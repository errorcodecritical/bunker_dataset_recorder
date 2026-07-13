#ifndef _UTILITY_TM_H_
#define _UTILITY_TM_H_

// ROS2 Headers
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

// For interactive markers (if needed)
// #include <interactive_markers/interactive_marker_server.hpp>

// For costmap (if needed with nav2)
// #include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <Eigen/Core>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/common/transforms.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.hpp>

#include <algorithm>
#include <array> // c++11
#include <cfloat>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <iterator>
#include <mutex> // c++11
#include <queue>
#include <sstream>
#include <string>
#include <thread> // c++11
#include <vector>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "planner/cubic_spline_interpolator.h"
#include "planner/kdtree.h"

#include "elevation_msgs/msg/occupancy_elevation.hpp"

using namespace std;

typedef pcl::PointXYZI PointType;
typedef struct kdtree kdtree_t;
typedef struct kdres kdres_t;

// Environment
extern const bool urbanMapping = false;

// Using velodyne cloud "ring" channel for image projection (other lidar may have different name for this channel, change "PointXYZIR" below)
extern const bool useCloudRing = true; // if true, ang_res_y and ang_bottom are not used

extern const float sensorMinimumRange = 0.2; // 1.0 0.2

// Ouster OS1-128 (1024 mode) - was RS-32
// N_SCAN * Horizon_SCAN should match the point cloud size from the sensor
// Ouster in 1024 mode: 128 channels * 1024 points = 131072 points
extern const int N_SCAN = 128;
extern const int Horizon_SCAN = 1024; // Ouster 1024 mode (was 1800 for RS-32)
extern const float ang_res_x = 0.2; //0.2
extern const float ang_res_y = 0.2; //0.2
extern const float ang_bottom = 15.0+0.1; //15.0

// Map Params
extern const float mapResolution = 0.1; // map resolution
extern const float mapCubeLength = 1.0; // the length of a sub-map (meters) 1.0
extern const int mapCubeArrayLength = mapCubeLength / mapResolution; // the grid dimension of a sub-map (mapCubeLength / mapResolution)
extern const int mapArrayLength = 2000 / mapCubeLength; // the sub-map dimension of global map (2000m x 2000m)
extern const int rootCubeIndex = mapArrayLength / 2; // by default, robot is at the center of global map at the beginning

// Filter Ring Params
extern const int scanNumCurbFilter = 8;
extern const int scanNumSlopeFilter = 10;
extern const int scanNumMax = std::max(scanNumCurbFilter, scanNumSlopeFilter);

// Filter Threshold Params
extern const float sensorRangeLimit = 200; // only keep points with in ... 12 | 6
extern const float filterHeightLimit = (urbanMapping == true) ? 0.1 : 0.15; // step diff threshold  0.1 : 0.15 || 0.50
extern const float filterAngleLimit = 20; // slope angle threshold 20
extern const int filterHeightMapArrayLength = sensorRangeLimit * 2 / mapResolution;

// BGK Prediction Params
extern const bool predictionEnableFlag = true;
extern const float predictionKernalSize = 0.2; // predict elevation within x meters 0.2

// Occupancy Params
extern const float p_occupied_when_laser = 0.9; // 0.9 0.65
extern const float p_occupied_when_no_laser = 0.2;
extern const float large_log_odds = 100;
extern const float max_log_odds_for_belief = 20; // 20

// 2D Map Publish Params
extern const int localMapLength = 200.0; // length of the local occupancy grid map (meter)
extern const int localMapArrayLength = localMapLength / mapResolution;

// Visualization Params
extern const float visualizationRadius = 200;
extern const float visualizationFrequency = 2; //2 n, skip n scans then publish, n=0, visualize at each scan

// Robot Params
extern const float robotRadius = 0.22; //0.2
extern const float sensorHeight = 0.65; //0.5

// Traversability Params
extern const int traversabilityObserveTimeTh = 10;
extern const float traversabilityCalculatingDistance = 8.0;

// Planning Cost Params
extern const int NUM_COSTS = 3;
extern const int tmp[] = { 2 };
extern const std::vector<int> costHierarchy(tmp, tmp + sizeof(tmp) / sizeof(int)); // c++11 initialization: costHierarchy{0, 1, 2}

// PRM Planner Settings
extern const bool planningUnknown = true;
extern const float costmapInflationRadius = 0.2; //0.2 0.8
extern const float neighborSampleRadius = 0.5;
extern const float neighborConnectHeight = 1.0; //1.0
extern const float neighborConnectRadius = 2.0; //2.0 2.5
extern const float neighborSearchRadius = localMapLength / 2;

struct grid_t;
struct mapCell_t;
struct childMap_t;
struct state_t;
struct neighbor_t;

/*
    This struct is used to send map from mapping package to prm package
    */
struct grid_t {
    int mapID;
    int cubeX;
    int cubeY;
    int gridX;
    int gridY;
    int gridIndex;
};

/*
    Cell Definition:
    a cell is a member of a grid in a sub-map
    a grid can have several cells in it.
    a cell represent one height information
    */

struct mapCell_t {

    PointType* xyz; // it's a pointer to the corresponding point in the point cloud of submap

    grid_t grid;

    float log_odds;

    int observeTimes;

    float occupancy, occupancyVar;
    float elevation, elevationVar;

    bool updated_in_current_scan;
    float traversability_decay;
    rclcpp::Time last_updated_time;

    mapCell_t()
    {

        log_odds = 0.5;
        observeTimes = 0;

        elevation = -FLT_MAX;
        elevationVar = 1e3;

        occupancy = 0; // initialized as unkown
        occupancyVar = 1e3;

        updated_in_current_scan = false;
        last_updated_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    void updatePoint()
    {
        xyz->z = elevation;
        xyz->intensity = occupancy;
    }
    void updateElevation(float elevIn, float varIn)
    {
        elevation = elevIn;
        elevationVar = varIn;
        updatePoint();
    }
    void updateOccupancy(float occupIn)
    {
        occupancy = occupIn;
        updatePoint();
    }
};

/*
    Sub-map Definition:
    childMap_t is a small square. We call it "cellArray".
    It composes the whole map
    */
struct childMap_t {

    vector<vector<mapCell_t*>> cellArray;
    int subInd; // sub-map's index in 1d mapArray
    int indX; // sub-map's x index in 2d array mapArrayInd
    int indY; // sub-map's y index in 2d array mapArrayInd
    float originX; // sub-map's x root coordinate
    float originY; // sub-map's y root coordinate
    pcl::PointCloud<PointType> cloud;

    childMap_t(int id, int indx, int indy)
    {

        subInd = id;
        indX = indx;
        indY = indy;
        originX = (indX - rootCubeIndex) * mapCubeLength - mapCubeLength / 2.0;
        originY = (indY - rootCubeIndex) * mapCubeLength - mapCubeLength / 2.0;

        // allocate and initialize each cell
        cellArray.resize(mapCubeArrayLength);
        for (int i = 0; i < mapCubeArrayLength; ++i)
            cellArray[i].resize(mapCubeArrayLength);

        for (int i = 0; i < mapCubeArrayLength; ++i)
            for (int j = 0; j < mapCubeArrayLength; ++j)
                cellArray[i][j] = new mapCell_t;
        // allocate point cloud for visualization
        cloud.points.resize(mapCubeArrayLength * mapCubeArrayLength);

        // initialize cell pointer to cloud point
        for (int i = 0; i < mapCubeArrayLength; ++i)
            for (int j = 0; j < mapCubeArrayLength; ++j)
                cellArray[i][j]->xyz = &cloud.points[i + j * mapCubeArrayLength];

        // initialize each point in the point cloud, also each cell
        for (int i = 0; i < mapCubeArrayLength; ++i) {
            for (int j = 0; j < mapCubeArrayLength; ++j) {

                // point cloud initialization
                int index = i + j * mapCubeArrayLength;
                cloud.points[index].x = originX + i * mapResolution;
                cloud.points[index].y = originY + j * mapResolution;
                cloud.points[index].z = std::numeric_limits<float>::quiet_NaN();
                cloud.points[index].intensity = cellArray[i][j]->occupancy;

                // cell position in the array of submap
                cellArray[i][j]->grid.mapID = subInd;
                cellArray[i][j]->grid.cubeX = indX;
                cellArray[i][j]->grid.cubeY = indY;
                cellArray[i][j]->grid.gridX = i;
                cellArray[i][j]->grid.gridY = j;
                cellArray[i][j]->grid.gridIndex = index;
            }
        }
    }
};

/*
    Robot State Defination
    */

struct state_t {
    double x[3]; //  1 - x, 2 - y, 3 - z
    float theta;
    int stateId;
    float cost;
    bool validFlag;
    // # Cost types
    // # 0. obstacle cost
    // # 1. elevation cost
    // # 2. distance cost
    float costsToRoot[NUM_COSTS];
    float costsToParent[NUM_COSTS]; // used in RRT*
    float costsToGo[NUM_COSTS];

    state_t* parentState; // parent for this state in PRM and RRT*
    vector<neighbor_t> neighborList; // PRM adjencency list with edge costs
    vector<state_t*> childList; // RRT*

    // default initialization
    state_t()
    {
        parentState = NULL;
        for (int i = 0; i < NUM_COSTS; ++i) {
            costsToRoot[i] = FLT_MAX;
            costsToParent[i] = FLT_MAX;
            costsToGo[i] = FLT_MAX;
        }
    }
    // use a state input to initialize new state

    state_t(state_t* stateIn)
    {
        // pose initialization
        for (int i = 0; i < 3; ++i)
            x[i] = stateIn->x[i];
        theta = stateIn->theta;
        // regular initialization
        parentState = NULL;
        for (int i = 0; i < NUM_COSTS; ++i) {
            costsToRoot[i] = FLT_MAX;
            costsToParent[i] = stateIn->costsToParent[i];
        }
    }
};

struct neighbor_t {
    state_t* neighbor;
    float edgeCosts[NUM_COSTS]; // the cost from this state to neighbor
    neighbor_t()
    {
        neighbor = NULL;
        for (int i = 0; i < NUM_COSTS; ++i)
            edgeCosts[i] = FLT_MAX;
    }
};

/*
    * A point cloud type that has "ring" channel
    */
struct PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint16_t, ring, ring)
)

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////      Some Functions    ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
state_t* compareState;
bool isStateExsiting(neighbor_t neighborIn)
{
    return neighborIn.neighbor == compareState ? true : false;
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

#endif
