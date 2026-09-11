#ifndef _UTILITY_TM_H_
#define _UTILITY_TM_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <interactive_markers/interactive_marker_server.h>

#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/transforms.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <array> // c++11
#include <thread> // c++11
#include <mutex> // c++11

#include "marker/Marker.h"
#include "marker/MarkerArray.h"

#include "planner/kdtree.h"
#include "planner/cubic_spline_interpolator.h"

#include "elevation_msgs/OccupancyElevation.h"

using namespace std;

typedef pcl::PointXYZI  PointType;
typedef struct kdtree kdtree_t;
typedef struct kdres kdres_t;

// Environment
extern const bool urbanMapping = false;

// Hesai 128-beam (bag: ring 0..127, 230400 = 128*1800)
extern const int N_SCAN = 128;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 360.0/Horizon_SCAN; // horizontal angular resolution
extern const float ang_res_y = 90.0/(N_SCAN - 1);
extern const float ang_bottom = 0.0;
extern const bool useCloudRing = true; // if true, ang_res_y and ang_bottom are not used
extern const float sensorMinimumRange = 0.5; // QT128 minimum range is 0.1 m; near-field ground matters

// Ring cut. The QT128 has a symmetric +/-52.6 deg vertical FOV, so 73 of its 128 channels point
// ABOVE the horizon and contribute nothing but overhead structure (canopy, ceilings, building
// tops) to what is fundamentally a 2.5D ground map - they land in the same (x,y) cell as the
// ground beneath them and blow up its height spread.
//
// Ring -> median elevation angle, measured from the bag (monotonic apart from one bank restart
// at 95->96, so a lower cut "ring <= K" is well defined for any K < 95):
//     ring   0 : -52.6      ring  54 :  -0.2 (horizon)      ring  70 : +13.7
//     ring  50 :  -3.4      ring  64 :  +7.9                ring 127 : +52.6
//
// Shan's original got this for free: a VLP-16 spans only +/-15 deg and extractFilteredCloud
// looped over the lowest 10 of 16 rings, i.e. -15..+3 deg. This restores the same idea.
//
// Two mutually exclusive strategies, selected by useHeightCut:
//
//   RING cut (useHeightCut = false) - discard rings above maxRingIndex, in projectPointCloud.
//     Cheapest possible: runs before projection, filtering or transformation, so ~80% of the
//     scan is gone before any per-point work. But it is RANGE-INDEPENDENT, which is its
//     weakness: ring 64 sits 1.4 m above the sensor at 10 m but 2.8 m at 20 m, so distant
//     overhead structure still leaks in while nearby low obstacles may be cut.
//
//   HEIGHT cut (useHeightCut = true) - discard returns more than heightCutAboveSensor above
//     the sensor, in extractFilteredCloud. Measured in the gravity-aligned map frame after
//     transformCloud, so the threshold means the same thing at every range and stays correct
//     under robot pitch and roll. Semantically it is the real question ("is this within the
//     height envelope my robot cares about?"), but projection and transformation still run
//     on the full cloud, so it costs more CPU.
// Currently HEIGHT cut. Both were tried on the Bunker bag; the height cut is the one in use.
// Flip this to false to switch back to the ring cut - no other change required.
extern const bool useHeightCut = true;
extern const float heightCutAboveSensor = 1.5; // m above the sensor; the sensor sits ~0.60 m
                                               // above ground, so this is a ~1.4 m envelope
extern const int maxRingIndex = 64; // ring cut threshold (~ +7.9 deg); unused when useHeightCut

// Map Params
extern const float mapResolution = 0.1; // map resolution
extern const float mapCubeLength = 1.0; // the length of a sub-map (meters)
extern const int mapCubeArrayLength = mapCubeLength / mapResolution; // the grid dimension of a sub-map (mapCubeLength / mapResolution)
extern const int mapArrayLength = 2000 / mapCubeLength; // the sub-map dimension of global map (2000m x 2000m)
extern const int rootCubeIndex = mapArrayLength / 2; // by default, robot is at the center of global map at the beginning

// Filter Ring Params
extern const int scanNumCurbFilter = 8;
extern const int scanNumSlopeFilter = 10;
extern const int scanNumMax = std::max(scanNumCurbFilter, scanNumSlopeFilter);

// Filter Threshold Params
extern const float sensorRangeLimit = 20; // only keep points with in ...
// Step-diff threshold. Drives BOTH downsampleCloud's per-cell obstacle flag (a 0.1 m cell whose
// max-min height exceeds this is flagged) and the vStep normalization in the traversability
// score, so it is the single most influential knob on how much of the map reads non-traversable.
// 0.15 is far too tight for forest floor - grass, roots and leaf litter clear it constantly.
extern const float filterHeightLimit = (urbanMapping == true) ? 0.1 : 0.25; // step diff threshold
extern const float filterAngleLimit = 25; // slope angle threshold
extern const int filterHeightMapArrayLength = sensorRangeLimit*2 / mapResolution;

// BGK Prediction Params
extern const bool predictionEnableFlag = true;
extern const float predictionKernalSize = 0.2; // predict elevation within x meters

// Occupancy Params
extern const float p_occupied_when_laser = 0.9;
extern const float p_occupied_when_no_laser = 0.2;
extern const float large_log_odds = 100;
extern const float max_log_odds_for_belief = 20;

// Continuous Occupancy Fusion Params
// The original code fused binary (0/100) evidence with a log-odds filter. Traversability is now a
// continuous 0..100 value, so the temporal filter is an exponential moving average instead: same
// role (reject single-scan noise, keep a running belief), no thresholding/saturation.
extern const float occupancyEmaAlpha = 0.3; // smoothing of the continuous traversability estimate
extern const float obstacleBeliefAlpha = 0.3; // smoothing of the per-scan binary obstacle evidence
// Above this, a cell is pinned to 100 regardless of its graded score. Keep it high: the pin is
// binary, so a low value converts large areas of merely-rough terrain into solid red and throws
// away the continuous scale entirely. 80 demands sustained obstacle evidence across scans, not
// a bare majority.
extern const float obstacleBeliefThreshold = 80;
// The other hard-obstacle trigger, as a multiple of filterHeightLimit, tested against the RAW
// max-min elevation across the footprint. This is what catches large PLANAR vertical structure
// - a trunk or a wall fits a plane almost perfectly, so the graded step and roughness terms
// (which measure departure from that plane) both go to zero and only vSlope fires, capping such
// a cell near 50. Raw vertical extent is the only term that sees it.
// At 4x (= 1.0 m) inside a 0.4 m footprint this essentially never fired; 2x (= 0.5 m) is
// already impassable for a Bunker.
extern const float hardObstacleStepFactor = 2.0;

// 2D Map Publish Params
extern const int localMapLength = 20; // length of the local occupancy grid map (meter)
extern const int localMapArrayLength = localMapLength / mapResolution;

// Visualization Params
extern const float visualizationRadius = 50;
extern const float visualizationFrequency = 2; // n, skip n scans then publish, n=0, visualize at each scan

// Robot Params
extern const float robotRadius = 0.2;
extern const float sensorHeight = 0.5;

// Traversability Params
extern const int traversabilityObserveTimeTh = 10;
extern const float traversabilityCalculatingDistance = 15.0;

// Traversability Cost Weights (slope + step + roughness, must sum to 1)
// Weights must sum to 1. Slope carries more than it used to: now that the step and roughness
// terms measure departure from the fitted plane, they say nothing about a smooth steep face,
// and slope is the only graded term that does.
extern const float travWeightSlope = 0.5;
extern const float travWeightStep = 0.3;
extern const float travWeightRoughness = 0.2;
extern const float travSlopeSigmoidWidth = 5.0; // degrees, transition width around filterAngleLimit
extern const float travRoughnessScale = 0.2; // meters of elevation std-dev that counts as fully rough

// Planning Cost Params
extern const int NUM_COSTS = 3;
extern const int tmp[] = {0, 1, 2};
// Which of the three edge costs are computed at all (edgePropagation skips the others, leaving
// them at 0). This used to double as the graph search's tie-breaking order, but the search now
// orders states by the composite cost below, so this list only selects which costs exist.
extern const std::vector<int> costHierarchy(tmp, tmp+sizeof(tmp)/sizeof(int));// c++11 initialization: costHierarchy{0, 1, 2}

// The single scalar the graph search minimizes. Each accumulated cost is divided by a rough
// per-path scale so the three are commensurable, then weighted. Both the relaxation test and
// the priority queue ordering go through this, so they cannot disagree.
// Scales are sized for a ~20 m path, since edge costs are line integrals along the edge: each
// term is then ~1 for a "typical bad" path of that length and the weights below actually
// balance. Traversability = 20 m travelled at occupancy 100.
extern const float pathCostScaleTraversability = 2000.0;
extern const float pathCostScaleElevation = 2.0;        // meters of accumulated |height change|
extern const float pathCostScaleDistance = 20.0;        // meters
extern const float pathCostWeightTraversability = 0.34;
extern const float pathCostWeightElevation = 0.33;
extern const float pathCostWeightDistance = 0.33;

// PRM Planner Settings
// true = samples and edges may cross cells the map has never observed. The original always
// rejected them. In a forest every trunk casts an occlusion shadow, so allowing it lets edges
// be drawn straight across ground nobody has seen - denser coverage, but a lot of it fictional.
// Currently true; set false to restore the original behaviour if the roadmap is too dense.
extern const bool planningUnknown = true;
// Traversability at or above which a cell seeds costmap inflation, i.e. is a HARD obstacle the
// sampler refuses outright. Everything below it stays sampleable and is penalised through the
// edge traversability cost instead, so the planner routes around bad terrain rather than the
// roadmap never being built there. Keep this high: at 70 it rejected ~52% of the map and the
// roadmap could not expand. Graded terrain belongs in the cost function, not in the collision
// check - only genuine no-go cells belong here.
// 90 is unreachable in practice: the score is a WEIGHTED SUM, so 90 demands all three terms
// near saturation at once, and saturated step + roughness with no slope only reaches 60.
// 75 is the point a weighted sum actually gets to when two of the three terms max out.
extern const int obstacleOccupancyThreshold = 75;
// What an unobserved cell counts as when integrating traversability along an edge. Unknown
// reads as -1 in the occupancy grid; left raw it would be cheaper than perfect ground and pull
// the planner into unmapped space. Neutral-to-pessimistic is the safe choice.
extern const int unknownCellTraversability = 50;
extern const float costmapInflationRadius = 0.15; // ~ the Bunker's half width
// Wall-clock budget generateSamples gets per map callback. Controls roadmap COVERAGE: at 10-20
// map callbacks/s this is how fast new ground gets nodes. Shan's value; dropping it to 0.002
// starved coverage. The connect radius below is what to lower if the CPU cost is the concern -
// it is quadratic, whereas this is linear.
extern const double samplingTimeBudget = 0.01;
extern const float neighborSampleRadius  = 0.5;  // minimum spacing between roadmap nodes
extern const float neighborConnectHeight = 1.0;
// Controls roadmap DENSITY: a node connects to every other node within this radius, so the
// neighbour count goes as (neighborConnectRadius / neighborSampleRadius)^2. At Shan's 2.0 that
// is a disc of ~50 nodes. He got away with it because his urban map was full of unknown cells
// and inflated obstacles, and a 2 m edge crosses 20 cells so a single bad one kills it - a
// strongly length-dependent filter. On mostly-observed terrain nothing culls them and the
// roadmap becomes a hairball. 1.0 gives ~11 neighbours; 0.8 gives ~7, matching the original's
// look most closely. Lower this before touching the sampling budget.
extern const float neighborConnectRadius = 1.0;
extern const float neighborSearchRadius = localMapLength / 2;

struct grid_t;
struct mapCell_t;
struct childMap_t;
struct state_t;
struct neighbor_t;

/*
    This struct is used to send map from mapping package to prm package
    */
struct grid_t{
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

struct mapCell_t{

    PointType *xyz; // it's a pointer to the corresponding point in the point cloud of submap

    grid_t grid;

    float log_odds;

    int observeTimes;

    float occupancy, occupancyVar;
    float elevation, elevationVar;

    // Per-scan binary obstacle evidence (0..100), smoothed over time. This is kept separate from
    // "occupancy": the scan pipeline can only say obstacle/free, while occupancy is the continuous
    // traversability score owned solely by TraversabilityMapping::traversabilityMapCalculation().
    // Letting both write occupancy made them overwrite each other every scan.
    float obstacleBelief;
    bool occupancyInit; // has occupancy ever been scored by the traversability thread?

    mapCell_t(){

        log_odds = 0.5;
        observeTimes = 0;

        elevation = -FLT_MAX;
        elevationVar = 1e3;

        occupancy = 0; // initialized as unkown
        occupancyVar = 1e3;

        obstacleBelief = 0;
        occupancyInit = false;
    }

    void updatePoint(){
        // Never publish a cell without a valid elevation: -FLT_MAX in the visualization cloud
        // destroys RViz's autocomputed bounds (and any consumer that reads z).
        if (elevation == -FLT_MAX)
            return;
        xyz->z = elevation;
        xyz->intensity = occupancy;
    }
    void updateElevation(float elevIn, float varIn){
        elevation = elevIn;
        elevationVar = varIn;
        updatePoint();
    }
    // Continuous replacement for the old log-odds filter: exponential moving average, no saturation.
    void updateOccupancy(float occupIn){
        if (occupancyInit == false){
            occupancy = occupIn;
            occupancyInit = true;
        }else{
            occupancy = (1.0f - occupancyEmaAlpha) * occupancy + occupancyEmaAlpha * occupIn;
        }
        updatePoint();
    }
    // Smoothed obstacle evidence from a single scan point (evidenceIn is 0 or 100).
    void updateObstacleBelief(float evidenceIn){
        if (observeTimes == 0)
            obstacleBelief = evidenceIn;
        else
            obstacleBelief = (1.0f - obstacleBeliefAlpha) * obstacleBelief + obstacleBeliefAlpha * evidenceIn;
    }
};


/*
    Sub-map Definition:
    childMap_t is a small square. We call it "cellArray". 
    It composes the whole map
    */
struct childMap_t{

    vector<vector<mapCell_t*> > cellArray;
    int subInd; //sub-map's index in 1d mapArray
    int indX; // sub-map's x index in 2d array mapArrayInd
    int indY; // sub-map's y index in 2d array mapArrayInd
    float originX; // sub-map's x root coordinate
    float originY; // sub-map's y root coordinate
    pcl::PointCloud<PointType> cloud;

    childMap_t(int id, int indx, int indy){

        subInd = id;
        indX = indx;
        indY = indy;
        originX = (indX - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;
        originY = (indY - rootCubeIndex) * mapCubeLength - mapCubeLength/2.0;

        // allocate and initialize each cell
        cellArray.resize(mapCubeArrayLength);
        for (int i = 0; i < mapCubeArrayLength; ++i)
            cellArray[i].resize(mapCubeArrayLength);

        for (int i = 0; i < mapCubeArrayLength; ++i)
            for (int j = 0; j < mapCubeArrayLength; ++j)
                cellArray[i][j] = new mapCell_t;
        // allocate point cloud for visualization
        cloud.points.resize(mapCubeArrayLength*mapCubeArrayLength);

        // initialize cell pointer to cloud point
        for (int i = 0; i < mapCubeArrayLength; ++i)
            for (int j = 0; j < mapCubeArrayLength; ++j)
                cellArray[i][j]->xyz = &cloud.points[i + j*mapCubeArrayLength];

        // initialize each point in the point cloud, also each cell
        for (int i = 0; i < mapCubeArrayLength; ++i){
            for (int j = 0; j < mapCubeArrayLength; ++j){
                
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


struct state_t{
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

    // Set once the graph search has settled this state, i.e. popped it from the priority queue
    // with its final cost. Lives on the state rather than in a separate set because bfsSearch
    // already sweeps every node to reset costs, so clearing it is free. Only valid during a
    // search; meaningless between searches.
    bool closedFlag;

    // default initialization
    state_t(){
        parentState = NULL;
        closedFlag = false;
        for (int i = 0; i < NUM_COSTS; ++i){
            costsToRoot[i] = FLT_MAX;
            costsToParent[i] = FLT_MAX;
            costsToGo[i] = FLT_MAX;
        }
    }
    // use a state input to initialize new state
    
    state_t(state_t* stateIn){
        // pose initialization
        for (int i = 0; i < 3; ++i)
            x[i] = stateIn->x[i];
        theta = stateIn->theta;
        // regular initialization
        parentState = NULL;
        closedFlag = false;
        for (int i = 0; i < NUM_COSTS; ++i){
            costsToRoot[i] = FLT_MAX;
            costsToParent[i] = stateIn->costsToParent[i];
        }
    }
};


struct neighbor_t{
    state_t* neighbor;
    float edgeCosts[NUM_COSTS]; // the cost from this state to neighbor
    neighbor_t(){
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

// RS-BPearl vertical angle ring mapping (top-down)
std::map<int, int> ringToRow = {
    {1, 0}, {9, 1}, {2, 2}, {10, 3}, {3, 4}, {11, 5}, {4, 6}, {12, 7},
    {5, 8}, {13, 9}, {6,10}, {14,11}, {7,12}, {15,13}, {8,14}, {16,15},
    {17,16}, {25,17}, {18,18}, {26,19}, {19,20}, {27,21}, {20,22}, {28,23},
    {21,24}, {29,25}, {22,26}, {30,27}, {23,28}, {31,29}, {24,30}, {32,31}
};







////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////      Some Functions    ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
state_t *compareState;
bool isStateExsiting(neighbor_t neighborIn){
    return neighborIn.neighbor == compareState ? true : false;
}

float pointDistance(PointType p1, PointType p2){
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif
