#include "utility.h"
#include <boost/shared_ptr.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>

class TraversabilityFilter : public rclcpp::Node {

private:
    // ROS2 Node
    // ros::NodeHandle nh;
    // ROS subscriber
    // ros::Subscriber subCloud;
    // ros::Subscriber subOdom;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCloud;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;

    // Mutex Memory Lock
    std::mutex mtx;
    // ROS publisher
    // ros::Publisher pubCloud;
    // ros::Publisher pubCloudVisualHiRes;
    // ros::Publisher pubCloudVisualLowRes;
    // ros::Publisher pubLaserScan;
    // ros::Publisher pubFullCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudVisualHiRes;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudVisualLowRes;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pubLaserScan;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFullCloud;

    laser_geometry::LaserProjection projector_;

    // Point Cloud
    pcl::PointCloud<PointType>::Ptr laserCloudIn; // projected full velodyne cloud
    pcl::PointCloud<PointType>::Ptr laserCloudOut; // filtered and downsampled point cloud
    pcl::PointCloud<PointType>::Ptr laserCloudObstacles; // cloud for saving points that are classified as obstables, convert them to laser scan

    pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;
    pcl::PointCloud<PointType>::Ptr laserCloudInOriginal;

    pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud;

    sensor_msgs::msg::PointCloud2::SharedPtr globalCloudMsg;

    std_msgs::msg::Header cloudHeader;

    PointType nanPoint; // fill in fullCloud at each iteration

    // Transform Listener
    // tf::TransformListener listener;
    // tf::StampedTransform transform;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    geometry_msgs::msg::TransformStamped transform;

    // A few points
    PointType robotPoint;
    PointType localMapOrigin;
    // point cloud saved as N_SCAN * Horizon_SCAN form
    vector<vector<PointType>> laserCloudMatrix;
    // Matrice
    cv::Mat obstacleMatrix; // -1 - invalid, 0 - free, 1 - obstacle
    cv::Mat rangeMatrix; // -1 - invalid, >0 - valid range value
    // laser scan message
    sensor_msgs::msg::LaserScan laserScan;
    // for downsample
    float** minHeight;
    float** maxHeight;
    bool** obstFlag;
    bool** initFlag;

public:
    TraversabilityFilter()
        : Node("traversability_filter")
    {
        // Initialize TF2
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        subCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/hesai/points", rclcpp::SensorDataQoS(),
            std::bind(&TraversabilityFilter::cloudHandler, this, std::placeholders::_1));

        pubFullCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/full_cloud_info", 5);

        pubCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud", 5);
        pubCloudVisualHiRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud_visual_high_res", 5);
        pubCloudVisualLowRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud_visual_low_res", 5);
        pubLaserScan = this->create_publisher<sensor_msgs::msg::LaserScan>("/pointcloud_2_laserscan", 5);

        // RCLCPP_INFO(this->get_logger(), "Publishers created:");
        // RCLCPP_INFO(this->get_logger(), "  /filtered_pointcloud");
        // RCLCPP_INFO(this->get_logger(), "  /filtered_pointcloud_visual_high_res");
        // RCLCPP_INFO(this->get_logger(), "  /filtered_pointcloud_visual_low_res");
        // RCLCPP_INFO(this->get_logger(), "  /pointcloud_2_laserscan");
        // RCLCPP_INFO(this->get_logger(), "  /full_cloud_info");

        subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/gps_waypoint_nav/odometry/navsat", 5,
            std::bind(&TraversabilityFilter::odomHandler, this, std::placeholders::_1));

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();

        pointcloud2laserscanInitialization();
    }

    void allocateMemory()
    {

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudOut.reset(new pcl::PointCloud<PointType>());
        laserCloudObstacles.reset(new pcl::PointCloud<PointType>());
        transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());
        laserCloudInOriginal.reset(new pcl::PointCloud<PointType>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        globalCloudMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        obstacleMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(-1));
        rangeMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(-1));

        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        laserCloudMatrix.resize(N_SCAN);
        for (int i = 0; i < N_SCAN; ++i)
            laserCloudMatrix[i].resize(Horizon_SCAN);

        initFlag = new bool*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            initFlag[i] = new bool[filterHeightMapArrayLength];

        obstFlag = new bool*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            obstFlag[i] = new bool[filterHeightMapArrayLength];

        minHeight = new float*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            minHeight[i] = new float[filterHeightMapArrayLength];

        maxHeight = new float*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            maxHeight[i] = new float[filterHeightMapArrayLength];

        resetParameters();
    }

    void resetParameters()
    {

        laserCloudIn->clear();
        laserCloudOut->clear();
        laserCloudObstacles->clear();

        laserCloudInOriginal->clear();
        transformed_cloud->clear();

        obstacleMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(-1));
        rangeMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(-1));

        for (int i = 0; i < filterHeightMapArrayLength; ++i) {
            for (int j = 0; j < filterHeightMapArrayLength; ++j) {
                initFlag[i][j] = false;
                obstFlag[i][j] = false;
            }
        }

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    ~TraversabilityFilter() { }

    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points, stamp: %.6f",
        //            laserCloudMsg->width * laserCloudMsg->height,
        //            rclcpp::Time(laserCloudMsg->header.stamp).seconds());
        projectPointCloud(laserCloudMsg);

        //RCLCPP_DEBUG(this->get_logger(), "After projectPointCloud, transformed_cloud has %zu points", transformed_cloud->points.size());
        extractRawCloud();

        //RCLCPP_DEBUG(this->get_logger(), "After extractRawCloud, laserCloudIn has %zu points", laserCloudIn->points.size());
        if (transformCloud() == false) {
            // RCLCPP_WARN(this->get_logger(), "transformCloud failed, skipping processing");
            return;
        }

        cloud2Matrix();

        applyFilter();

        extractFilteredCloud();

        downsampleCloud();

        predictCloudBGK();

        publishCloud();

        publishLaserScan();

        resetParameters();
    }

    void projectPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg){
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize;
        PointType thisPoint;

        cloudHeader = laserCloudMsg->header;

        pcl::fromROSMsg(*laserCloudMsg, *laserCloudInOriginal);

        pcl::PointCloud<PointType>::Ptr laserCloudInFiltered(new pcl::PointCloud<PointType>());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudInOriginal, *laserCloudInFiltered, indices);

        if (useCloudRing == true){
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
        }

        cloudSize = laserCloudInFiltered->points.size();

        for (size_t i = 0; i < cloudSize; ++i){

            thisPoint.x = laserCloudInFiltered->points[i].x;
            thisPoint.y = laserCloudInFiltered->points[i].y;
            thisPoint.z = laserCloudInFiltered->points[i].z;
            if (useCloudRing == true){
                if (i >= laserCloudInRing->points.size())
                    continue;
                rowIdn = laserCloudInRing->points[i].ring;
            }
            else{
                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            }
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < sensorMinimumRange)
                continue;

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range;
        }

        geometry_msgs::msg::TransformStamped transform;

        // Always use the point cloud's timestamp for TF lookup
        rclcpp::Time cloud_time = rclcpp::Time(cloudHeader.stamp);

        try {transform = tf_buffer->lookupTransform("base_link", "hesai_lidar", cloud_time, rclcpp::Duration::from_seconds(0.1));}
        catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF lookup failed: base_link -> os_lidar at time %.6f: %s",
                        cloud_time.seconds(), ex.what());
            return;
        }

        Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform.transform);

        pcl::transformPointCloud(*fullInfoCloud, *transformed_cloud, eigen_transform);

        sensor_msgs::msg::PointCloud2 globalCloudMsgTemp;
        pcl::toROSMsg(*transformed_cloud, globalCloudMsgTemp);
        globalCloudMsgTemp.header.frame_id = "base_link";
        globalCloudMsgTemp.header.stamp = cloudHeader.stamp;
        pubFullCloud->publish(globalCloudMsgTemp);
    }

    void extractRawCloud()
    {
        if (transformed_cloud->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "transformed_cloud is empty, skipping extractRawCloud");
            return;
        }

        laserCloudIn->points.assign(N_SCAN * Horizon_SCAN, nanPoint);

        for (const auto & point : transformed_cloud->points) {
            if (!std::isfinite(point.intensity))
                continue;

            int rowIdn = static_cast<int>(point.intensity);
            int columnIdn = static_cast<int>(std::round((point.intensity - rowIdn) * 10000.0));
            if (rowIdn < 0 || rowIdn >= N_SCAN || columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            int index = columnIdn + rowIdn * Horizon_SCAN;
            laserCloudIn->points[index] = point;
        }

        for (int i = 0; i < N_SCAN; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                int index = j  + i * Horizon_SCAN;
                if (!std::isfinite(laserCloudIn->points[index].intensity))
                    continue;
                rangeMatrix.at<float>(i, j) = laserCloudIn->points[index].intensity;
                obstacleMatrix.at<int>(i, j) = 0;
            }
        }
    }

    void odomHandler(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // robotPoint.x = msg->pose.pose.position.x;
        // robotPoint.y = msg->pose.pose.position.y;
        // robotPoint.z = msg->pose.pose.position.z;
    }

    bool transformCloud()
    {
        // Always use the point cloud's timestamp for TF lookup
        rclcpp::Time cloud_time = rclcpp::Time(cloudHeader.stamp);

        try{transform = tf_buffer->lookupTransform("map","base_link", cloud_time, rclcpp::Duration::from_seconds(0.1)); }
        catch (tf2::TransformException ex){
            RCLCPP_ERROR(this->get_logger(), "TF lookup failed: map -> base_link at time %.6f: %s",
                        cloud_time.seconds(), ex.what());
            return false;
        }

        robotPoint.x = transform.transform.translation.x;
        robotPoint.y = transform.transform.translation.y;
        robotPoint.z = transform.transform.translation.z;

        laserCloudIn->header.frame_id = "base_link";
        laserCloudIn->header.stamp = 0;

        pcl::PointCloud<PointType> laserCloudTemp;
        pcl_ros::transformPointCloud("map", *laserCloudIn, laserCloudTemp, *tf_buffer);
        *laserCloudIn = laserCloudTemp;

        // RCLCPP_DEBUG(this->get_logger(), "After transform, laserCloudIn has %zu points (need %d)",
        //              laserCloudIn->points.size(), N_SCAN * Horizon_SCAN);
        if (laserCloudIn->points.size() < static_cast<size_t>(N_SCAN * Horizon_SCAN)) {
            // RCLCPP_WARN(this->get_logger(), "Not enough points after transform: %zu < %d",
            //             laserCloudIn->points.size(), N_SCAN * Horizon_SCAN);
            return false;
        }

        return true;
    }

    void cloud2Matrix(){
        for (int i = 0; i < N_SCAN; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                int index = j  + i * Horizon_SCAN;
                PointType p = laserCloudIn->points[index];
                laserCloudMatrix[i][j] = p;
            }
        }
    }

    void applyFilter()
    {
        if (urbanMapping == true) {
            positiveCurbFilter();
            negativeCurbFilter();
        }
        slopeFilter();
    }

    void positiveCurbFilter()
    {
        int rangeCompareNeighborNum = 3;
        float diff[Horizon_SCAN - 1];

        for (int i = 0; i < scanNumCurbFilter; ++i) {
            for (int j = 0; j < Horizon_SCAN - 1; ++j)
                diff[j] = rangeMatrix.at<float>(i, j) - rangeMatrix.at<float>(i, j + 1);

            for (int j = rangeCompareNeighborNum; j < Horizon_SCAN - rangeCompareNeighborNum; ++j) {
                if (obstacleMatrix.at<int>(i, j) == 1)
                    continue;

                bool breakFlag = false;
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit)
                    continue;
                for (int k = -rangeCompareNeighborNum; k <= rangeCompareNeighborNum; ++k)
                    if (rangeMatrix.at<float>(i, j + k) == -1) {
                        breakFlag = true;
                        break;
                    }
                if (breakFlag == true)
                    continue;
                for (int k = -rangeCompareNeighborNum; k < rangeCompareNeighborNum - 1; ++k)
                    if (diff[j + k] * diff[j + k + 1] <= 0) {
                        breakFlag = true;
                        break;
                    }
                if (breakFlag == true)
                    continue;
                if (abs(rangeMatrix.at<float>(i, j - rangeCompareNeighborNum) - rangeMatrix.at<float>(i, j + rangeCompareNeighborNum)) / rangeMatrix.at<float>(i, j) < 0.03)
                    continue;
                obstacleMatrix.at<int>(i, j) = 1;
            }
        }
    }

    void negativeCurbFilter()
    {
        int rangeCompareNeighborNum = 3;

        for (int i = 0; i < scanNumCurbFilter; ++i) {
            for (int j = 0; j < Horizon_SCAN; ++j) {
                if (obstacleMatrix.at<int>(i, j) == 1)
                    continue;
                if (rangeMatrix.at<float>(i, j) == -1)
                    continue;
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit)
                    continue;
                for (int m = -rangeCompareNeighborNum; m <= rangeCompareNeighborNum; ++m) {
                    int k = j + m;
                    if (k < 0 || k >= Horizon_SCAN)
                        continue;
                    if (rangeMatrix.at<float>(i, k) == -1)
                        continue;
                    if (laserCloudMatrix[i][j].z - laserCloudMatrix[i][k].z > 0.1
                        && pointDistance(laserCloudMatrix[i][j], laserCloudMatrix[i][k]) <= 1.0) {
                        obstacleMatrix.at<int>(i, j) = 1;
                        break;
                    }
                }
            }
        }
    }

    void slopeFilter()
    {
        for (int i = 0; i < scanNumSlopeFilter; ++i) {
            for (int j = 0; j < Horizon_SCAN; ++j) {
                if (obstacleMatrix.at<int>(i, j) == 1)
                    continue;
                if (rangeMatrix.at<float>(i, j) == -1 || rangeMatrix.at<float>(i + 1, j) == -1)
                    continue;
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit)
                    continue;
                float diffX = laserCloudMatrix[i + 1][j].x - laserCloudMatrix[i][j].x;
                float diffY = laserCloudMatrix[i + 1][j].y - laserCloudMatrix[i][j].y;
                float diffZ = laserCloudMatrix[i + 1][j].z - laserCloudMatrix[i][j].z;
                float angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;
                if (angle < -filterAngleLimit || angle > filterAngleLimit) {
                    obstacleMatrix.at<int>(i, j) = 1;
                    continue;
                }
            }
        }
    }

    void extractFilteredCloud()
    {
        for (int i = 0; i < scanNumMax; ++i) {
            for (int j = 0; j < Horizon_SCAN; ++j) {
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit || rangeMatrix.at<float>(i, j) == -1)
                    continue;
                PointType p = laserCloudMatrix[i][j];
                p.intensity = obstacleMatrix.at<int>(i, j) == 1 ? 100 : 0;
                laserCloudOut->push_back(p);
                if (p.intensity == 100)
                    laserCloudObstacles->push_back(p);
            }
        }

        if (pubCloudVisualHiRes->get_subscription_count() != 0) {
            sensor_msgs::msg::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "map";
            pubCloudVisualHiRes->publish(laserCloudTemp);
        }
    }

    void downsampleCloud()
    {
        float roundedX = float(int(robotPoint.x * 10.0f)) / 10.0f;
        float roundedY = float(int(robotPoint.y * 10.0f)) / 10.0f;
        localMapOrigin.x = roundedX - sensorRangeLimit;
        localMapOrigin.y = roundedY - sensorRangeLimit;

        int cloudSize = laserCloudOut->points.size();
        for (int i = 0; i < cloudSize; ++i) {
            int idx = (laserCloudOut->points[i].x - localMapOrigin.x) / mapResolution;
            int idy = (laserCloudOut->points[i].y - localMapOrigin.y) / mapResolution;
            if (idx < 0 || idy < 0 || idx >= filterHeightMapArrayLength || idy >= filterHeightMapArrayLength)
                continue;
            if (laserCloudOut->points[i].intensity == 100)
                obstFlag[idx][idy] = true;
            if (initFlag[idx][idy] == false) {
                minHeight[idx][idy] = laserCloudOut->points[i].z;
                maxHeight[idx][idy] = laserCloudOut->points[i].z;
                initFlag[idx][idy] = true;
            } else {
                minHeight[idx][idy] = std::min(minHeight[idx][idy], laserCloudOut->points[i].z);
                maxHeight[idx][idy] = std::max(maxHeight[idx][idy], laserCloudOut->points[i].z);
            }
        }
        pcl::PointCloud<PointType>::Ptr laserCloudTemp(new pcl::PointCloud<PointType>());
        for (int i = 0; i < filterHeightMapArrayLength; ++i) {
            for (int j = 0; j < filterHeightMapArrayLength; ++j) {
                if (initFlag[i][j] == false)
                    continue;
                PointType thisPoint;
                thisPoint.x = localMapOrigin.x + i * mapResolution + mapResolution / 2.0;
                thisPoint.y = localMapOrigin.y + j * mapResolution + mapResolution / 2.0;
                thisPoint.z = maxHeight[i][j];

                if (obstFlag[i][j] == true || maxHeight[i][j] - minHeight[i][j] > filterHeightLimit) {
                    obstFlag[i][j] = true;
                    thisPoint.intensity = 100;
                    laserCloudTemp->push_back(thisPoint);
                } else {
                    thisPoint.intensity = 0;
                    laserCloudTemp->push_back(thisPoint);
                }
            }
        }

        *laserCloudOut = *laserCloudTemp;

        if (pubCloudVisualLowRes->get_subscription_count() != 0) {
            sensor_msgs::msg::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "map";
            pubCloudVisualLowRes->publish(laserCloudTemp);
        }
    }

    void predictCloudBGK()
    {
        if (predictionEnableFlag == false)
            return;

        int kernelGridLength = int(predictionKernalSize / mapResolution);

        for (int i = 0; i < filterHeightMapArrayLength; ++i) {
            for (int j = 0; j < filterHeightMapArrayLength; ++j) {
                if (initFlag[i][j] == true)
                    continue;
                PointType testPoint;
                testPoint.x = localMapOrigin.x + i * mapResolution + mapResolution / 2.0;
                testPoint.y = localMapOrigin.y + j * mapResolution + mapResolution / 2.0;
                testPoint.z = robotPoint.z;
                if (pointDistance(testPoint, robotPoint) > sensorRangeLimit)
                    continue;
                vector<float> xTrainVec;
                vector<float> yTrainVecElev;
                vector<float> yTrainVecOccu;
                for (int m = -kernelGridLength; m <= kernelGridLength; ++m) {
                    for (int n = -kernelGridLength; n <= kernelGridLength; ++n) {
                        if (std::sqrt(float(m * m + n * n)) * mapResolution > predictionKernalSize)
                            continue;
                        int idx = i + m;
                        int idy = j + n;
                        if (idx < 0 || idy < 0 || idx >= filterHeightMapArrayLength || idy >= filterHeightMapArrayLength)
                            continue;
                        if (initFlag[idx][idy] == true) {
                            xTrainVec.push_back(localMapOrigin.x + idx * mapResolution + mapResolution / 2.0);
                            xTrainVec.push_back(localMapOrigin.y + idy * mapResolution + mapResolution / 2.0);
                            yTrainVecElev.push_back(maxHeight[idx][idy]);
                            yTrainVecOccu.push_back(obstFlag[idx][idy] == true ? 1 : 0);
                        }
                    }
                }
                if (xTrainVec.size() == 0)
                    continue;
                Eigen::MatrixXf xTrain = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTrainVec.data(), xTrainVec.size() / 2, 2);
                Eigen::MatrixXf yTrainElev = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(yTrainVecElev.data(), yTrainVecElev.size(), 1);
                Eigen::MatrixXf yTrainOccu = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(yTrainVecOccu.data(), yTrainVecOccu.size(), 1);
                vector<float> xTestVec;
                xTestVec.push_back(testPoint.x);
                xTestVec.push_back(testPoint.y);
                Eigen::MatrixXf xTest = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTestVec.data(), xTestVec.size() / 2, 2);
                Eigen::MatrixXf Ks;
                covSparse(xTest, xTrain, Ks);

                Eigen::MatrixXf ybarElev = (Ks * yTrainElev).array();
                Eigen::MatrixXf ybarOccu = (Ks * yTrainOccu).array();
                Eigen::MatrixXf kbar = Ks.rowwise().sum().array();

                if (std::isnan(ybarElev(0, 0)) || std::isnan(ybarOccu(0, 0)) || std::isnan(kbar(0, 0)))
                    continue;

                if (kbar(0, 0) == 0)
                    continue;

                float elevation = ybarElev(0, 0) / kbar(0, 0);
                float occupancy = ybarOccu(0, 0) / kbar(0, 0);

                PointType p;
                p.x = xTestVec[0];
                p.y = xTestVec[1];
                p.z = elevation;
                p.intensity = (occupancy > 0.5) ? 100 : 0;

                laserCloudOut->push_back(p);
            }
        }
    }

    void dist(const Eigen::MatrixXf& xStar, const Eigen::MatrixXf& xTrain, Eigen::MatrixXf& d) const
    {
        d = Eigen::MatrixXf::Zero(xStar.rows(), xTrain.rows());
        for (int i = 0; i < xStar.rows(); ++i) {
            d.row(i) = (xTrain.rowwise() - xStar.row(i)).rowwise().norm();
        }
    }

    void covSparse(const Eigen::MatrixXf& xStar, const Eigen::MatrixXf& xTrain, Eigen::MatrixXf& Kxz) const
    {
        dist(xStar / (predictionKernalSize + 0.1), xTrain / (predictionKernalSize + 0.1), Kxz);
        Kxz = (((2.0f + (Kxz * 2.0f * 3.1415926f).array().cos()) * (1.0f - Kxz.array()) / 3.0f) + (Kxz * 2.0f * 3.1415926f).array().sin() / (2.0f * 3.1415926f)).matrix() * 1.0f;
        for (int i = 0; i < Kxz.rows(); ++i)
            for (int j = 0; j < Kxz.cols(); ++j)
                if (Kxz(i, j) < 0)
                    Kxz(i, j) = 0;
    }

    void publishCloud()
    {
        sensor_msgs::msg::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "odom";
        pubCloud->publish(laserCloudTemp);
        //RCLCPP_DEBUG(this->get_logger(), "Published filtered point cloud with %zu points to /filtered_pointcloud", laserCloudOut->points.size());
    }

    void publishLaserScan()
    {
        updateLaserScan();
        laserScan.header.stamp = cloudHeader.stamp;
        pubLaserScan->publish(laserScan);
        std::fill(laserScan.ranges.begin(), laserScan.ranges.end(), laserScan.range_max + 1.0);
    }

    void updateLaserScan()
    {
        // Always use the point cloud's timestamp for TF lookup
        rclcpp::Time cloud_time = rclcpp::Time(cloudHeader.stamp);

        try{transform = tf_buffer->lookupTransform("base_link","map", cloud_time, rclcpp::Duration::from_seconds(0.1));}
        catch (tf2::TransformException ex){
            RCLCPP_ERROR(this->get_logger(), "TF lookup failed: base_link -> map at time %.6f: %s",
                        cloud_time.seconds(), ex.what());
            return;
        }

        laserCloudObstacles->header.frame_id = "map";
        laserCloudObstacles->header.stamp = rclcpp::Time(cloudHeader.stamp).nanoseconds();
        pcl::PointCloud<PointType> laserCloudTemp;
        pcl_ros::transformPointCloud("base_link", *laserCloudObstacles, laserCloudTemp, *tf_buffer);
        int cloudSize = laserCloudTemp.points.size();
        for (int i = 0; i < cloudSize; ++i) {
            PointType* point = &laserCloudTemp.points[i];
            float x = point->x;
            float y = point->y;
            float range = std::sqrt(x * x + y * y);
            float angle = std::atan2(y, x);
            int index = (angle - laserScan.angle_min) / laserScan.angle_increment;
            if (index >= 0 && index < laserScan.ranges.size())
                laserScan.ranges[index] = std::min(laserScan.ranges[index], range);
        }
    }

    void pointcloud2laserscanInitialization()
    {
        laserScan.header.frame_id = "base_link";
        laserScan.angle_min = -M_PI;
        laserScan.angle_max = M_PI;
        laserScan.angle_increment = 1.0f / 180 * M_PI;
        laserScan.time_increment = 0;
        laserScan.scan_time = 0.1;
        laserScan.range_min = 0.3;
        laserScan.range_max = 100;
        int range_size = std::ceil((laserScan.angle_max - laserScan.angle_min) / laserScan.angle_increment);
        laserScan.ranges.assign(range_size, laserScan.range_max + 1.0);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto TFilter = std::make_shared<TraversabilityFilter>();
    RCLCPP_INFO(TFilter->get_logger(), "Traversability Filter Started.");
    rclcpp::spin(TFilter);
    return 0;
}
