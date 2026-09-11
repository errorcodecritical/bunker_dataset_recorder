#include "utility.h"

#include "elevation_msgs/OccupancyElevation.h"


class TraversabilityPRM{
private:

    ros::NodeHandle nh;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber subGoal;
    
    ros::Publisher pubPRMGraph; // publish PRM nodes and edges
    ros::Publisher pubPRMPath; // path extracted from roadmap
    ros::Publisher pubGlobalPath; // path is published in pose array format 
    ros::Publisher pubSingleSourcePaths; // publish paths to al states in roadmap

    ros::Publisher pubCloudPRMNodes;
    ros::Publisher pubCloudPRMGraph;

    ros::Subscriber subElevationMap; // 2d local height map from mapping package

    elevation_msgs::OccupancyElevation elevationMap; // this is received from mapping package. it is a 2d local map that includes height info

    float map_min[3]; // 0 - x, 1 - y, 2 - z
    float map_max[3];
 
    ///////////// Planner ////////////
    vector<state_t*> nodeList;
    vector<state_t*> pathList;

    // (compositeCost, state) entry in bfsSearch's priority queue. std::greater on a pair orders
    // by the float first and only falls back to the pointer to break exact ties, which keeps
    // the ordering total and deterministic.
    typedef std::pair<float, state_t*> QueueEntry;

    nav_msgs::Path globalPath;
    nav_msgs::Path displayGlobalPath;

    double start_time;
    double finish_time;

    bool planningFlag; // true only when a NEW PRM plan is required

    // Final-goal / intermediate-waypoint state.
    // The final goal stays fixed, while bfsSearch() selects one reachable PRM
    // endpoint at a time. That endpoint is latched until the robot reaches it.
    bool haveFinalGoal;
    bool activeWaypoint;
    double waypointX;
    double waypointY;
    double waypointZ;

    // Replan after getting this close to the currently latched PRM endpoint.
    // A repeated /prm_goal closer than newGoalDistanceThreshold to the stored
    // final goal is treated as the same goal and does NOT trigger replanning.
    double waypointReachDistance;
    double newGoalDistanceThreshold;

    state_t *robotState;
    state_t *goalState;
    state_t *nearestGoalState;
    state_t *mapCenter;

    kdtree_t *kdtree;

    bool costUpdateFlag[NUM_COSTS];

    std::mutex mtx;

public:
    TraversabilityPRM():
        nh("~"),
        planningFlag(false),
        haveFinalGoal(false),
        activeWaypoint(false),
        waypointX(0.0),
        waypointY(0.0),
        waypointZ(0.0),
        waypointReachDistance(1.0),
        newGoalDistanceThreshold(0.2){

        robotState = new state_t;
        goalState = new state_t;
        mapCenter = new state_t;

        // Optional ROS parameters; defaults implement the requested behavior.
        nh.param("waypoint_reach_distance", waypointReachDistance, 1.0);
        nh.param("new_goal_distance_threshold", newGoalDistanceThreshold, 0.2);

        subGoal = nh.subscribe<geometry_msgs::PoseStamped>("/prm_goal", 5, &TraversabilityPRM::goalPosHandler, this);
        subElevationMap = nh.subscribe<elevation_msgs::OccupancyElevation>("/occupancy_map_local_height", 5, &TraversabilityPRM::elevationMapHandler, this);     

        pubPRMGraph = nh.advertise<visualization_msgs::MarkerArray>("/prm_graph", 5);
        pubPRMPath = nh.advertise<visualization_msgs::MarkerArray>("/prm_path", 5);
        pubSingleSourcePaths = nh.advertise<visualization_msgs::MarkerArray>("/prm_single_source_paths", 5);

        pubCloudPRMNodes = nh.advertise<sensor_msgs::PointCloud2>("/prm_cloud_nodes", 5);
        pubCloudPRMGraph = nh.advertise<sensor_msgs::PointCloud2>("/prm_cloud_graph", 5);

        pubGlobalPath = nh.advertise<nav_msgs::Path>("/global_path", 5);

        allocateMemory(); 
    }

    ~TraversabilityPRM(){}

    void allocateMemory(){

        // 2D, not 3D. The map is 2.5D - one height per (x,y) - so the roadmap is a ground graph
        // and every nearest/near query on it is really a ground-plane query. This makes
        // getNearestState return the true ground-plane nearest node, which is what
        // stateTooClose needs to enforce node spacing; see the 2D test there.
        kdtree = kd_create(2);

        for (int i = 0; i < NUM_COSTS; ++i)
            costUpdateFlag[i] = false;
        for (int i = 0; i < costHierarchy.size(); ++i)
            costUpdateFlag[costHierarchy[i]] = true;
    }

    void elevationMapHandler(const elevation_msgs::OccupancyElevation::ConstPtr& mapMsg){

        std::lock_guard<std::mutex> lock(mtx);

        elevationMap = *mapMsg;

        updateMapBoundary();

        updateCostMap();

        buildRoadMap();
    }

    void updateMapBoundary(){
        map_min[0] = elevationMap.occupancy.info.origin.position.x; 
        map_min[1] = elevationMap.occupancy.info.origin.position.y;
        map_min[2] = elevationMap.occupancy.info.origin.position.z;
        map_max[0] = elevationMap.occupancy.info.origin.position.x + elevationMap.occupancy.info.resolution * elevationMap.occupancy.info.width; 
        map_max[1] = elevationMap.occupancy.info.origin.position.y + elevationMap.occupancy.info.resolution * elevationMap.occupancy.info.height; 
        map_max[2] = elevationMap.occupancy.info.origin.position.z;
    }

    void updateCostMap(){
        int sizeMap = elevationMap.occupancy.data.size();
        int inflationSize = int(costmapInflationRadius / elevationMap.occupancy.info.resolution);
        for (int i = 0; i < sizeMap; ++i) {
            int idX = int(i % elevationMap.occupancy.info.width);
            int idY = int(i / elevationMap.occupancy.info.width);
            // Threshold before inflating. occupancy is a graded 0..100 score now, so the old
            // "> 0" test fired on virtually every observed cell and would inflate the entire map.
            if (elevationMap.occupancy.data[i] >= obstacleOccupancyThreshold){
                for (int m = -inflationSize; m <= inflationSize; ++m) {
                    for (int n = -inflationSize; n <= inflationSize; ++n) {
                        int newIdX = idX + m;
                        int newIdY = idY + n;
                        if (newIdX < 0 || newIdX >= elevationMap.occupancy.info.width || newIdY < 0 || newIdY >= elevationMap.occupancy.info.height)
                            continue;
                        int index = newIdX + newIdY * elevationMap.occupancy.info.width;
                        elevationMap.costMap[index] = std::max(elevationMap.costMap[index], std::sqrt(float(m*m+n*n)));
                    }
                }
            }
        }
    }

    void buildRoadMap(){
        // 1. Keep expanding/updating the rolling roadmap continuously.
        generateSamples();

        // 2. Update robot pose, node heights and graph edges as the map changes.
        // updateStatesAndEdges() calls getRobotState() first.
        updateStatesAndEdges();

        // No goal has been received yet: only maintain/visualize the roadmap.
        if (!haveFinalGoal){
            publishPRM();
            publishRoadmap2Cloud();
            return;
        }

        // If the robot is already at the actual final goal, finish the whole
        // waypoint sequence instead of selecting yet another PRM endpoint.
        double finalDx = robotState->x[0] - goalState->x[0];
        double finalDy = robotState->x[1] - goalState->x[1];
        double finalGoalDistance = sqrt(finalDx*finalDx + finalDy*finalDy);

        if (finalGoalDistance <= waypointReachDistance){
            if (activeWaypoint || planningFlag){
                ROS_INFO("Final PRM goal reached (%.2f m).", finalGoalDistance);
            }
            activeWaypoint = false;
            planningFlag = false;
        }

        // A PRM endpoint is currently latched. Do NOT run bfsSearch() while
        // travelling toward it. Only unlock planning after reaching it.
        if (activeWaypoint){
            double dx = robotState->x[0] - waypointX;
            double dy = robotState->x[1] - waypointY;
            double waypointDistance = sqrt(dx*dx + dy*dy);

            if (waypointDistance <= waypointReachDistance){
                ROS_INFO("Reached latched PRM waypoint [%.2f, %.2f] (%.2f m). Replanning toward final goal [%.2f, %.2f].",
                         waypointX, waypointY, waypointDistance,
                         goalState->x[0], goalState->x[1]);

                activeWaypoint = false;
                planningFlag = true;
            }
        }

        // Plan only when there is no active intermediate waypoint. On success,
        // bfsSearch() latches the newly selected endpoint before returning.
        if (planningFlag && !activeWaypoint){
            if (bfsSearch()){
                planningFlag = false;
                publishCurrentPath();
            }
            else{
                // Keep planningFlag true so the next map update can try again
                // after more samples / traversability information arrive.
                ROS_WARN_THROTTLE(2.0, "PRM could not find a reachable waypoint yet; will retry on the next map update.");
            }
        }
        else if (activeWaypoint){
            // Re-publish the SAME stored path for downstream consumers, but do
            // not recompute it. The latched waypoint therefore stays fixed.
            publishCurrentPath();
        }

        // 4. Visualize roadmap and currently stored path.
        publishPRM();

        // 5. Convert PRM graph into point cloud for external usage.
        publishRoadmap2Cloud();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////// Planner /////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void goalPosHandler(const geometry_msgs::PoseStampedConstPtr& goal){

        // If /prm_goal is periodically republished, do not let the same final
        // goal cancel the currently latched intermediate waypoint.
        if (haveFinalGoal){
            double dx = goal->pose.position.x - goalState->x[0];
            double dy = goal->pose.position.y - goalState->x[1];
            double goalChange = sqrt(dx*dx + dy*dy);

            if (goalChange < newGoalDistanceThreshold){
                ROS_DEBUG_THROTTLE(2.0, "Ignoring repeated /prm_goal; current final goal is unchanged.");
                return;
            }
        }

        // Store the FINAL requested destination. This remains unchanged while
        // the planner advances through one reachable PRM waypoint at a time.
        goalState->x[0] = goal->pose.position.x;
        goalState->x[1] = goal->pose.position.y;
        goalState->x[2] = goal->pose.position.z;

        haveFinalGoal = true;

        // A genuinely new final goal cancels the old intermediate waypoint and
        // requests one fresh plan on the next elevation-map update.
        activeWaypoint = false;
        planningFlag = true;

        ROS_INFO("New FINAL PRM goal received: [%.2f, %.2f, %.2f]",
                 goalState->x[0], goalState->x[1], goalState->x[2]);
    }

    
    
    bool bfsSearch(){

        pathList.clear();
        globalPath.poses.clear();
        // 1. reset costs, parents and settled flags
        for (int i = 0; i < nodeList.size(); ++i){
            for (int j = 0; j < NUM_COSTS; ++j)
                nodeList[i]->costsToRoot[j] = FLT_MAX;
            nodeList[i]->parentState = NULL;
            nodeList[i]->closedFlag = false;
        }
        // 2. find the state that is the closest to the robot
        state_t *startState = NULL;

        vector<state_t*> nearRobotStates;
        getNearStates(robotState, nearRobotStates, 2);
        if (nearRobotStates.size() == 0)
            return false;

        float nearRobotDist = FLT_MAX;
        for (int i = 0; i < nearRobotStates.size(); ++i){
            float dist = distance(nearRobotStates[i]->x, robotState->x);
            if (dist < nearRobotDist && nearRobotStates[i]->neighborList.size() != 0){
                nearRobotDist = dist;
                startState = nearRobotStates[i];
            }
        }

        if (startState == NULL || startState->neighborList.size() == 0)
            return false;

        for (int i = 0; i < NUM_COSTS; ++i)
            startState->costsToRoot[i] = 0;

        // 3. Resolve the goal node up front, so the search can stop as soon as it is settled.
        // getNearestState minimizes distance to the goal over the entire roadmap, so nothing
        // discovered later can be closer - the refinement in step 5 only ever kicks in when
        // this node turns out to be unreachable.
        nearestGoalState = getNearestState(goalState);
        // Step 1 reset startState->parentState to NULL, so if the goal resolves to the start
        // we would exit on the very first pop and then fail the reachability test in step 5.
        // Degenerate case (goal sitting on the robot); just let the search run.
        bool earlyExitEnabled = (nearestGoalState != NULL && nearestGoalState != startState);

        // 4. Dijkstra over the roadmap, ordered by compositeCost.
        // Lazy-deletion binary heap: a state is pushed once per improvement and may therefore
        // appear several times, but only its first (cheapest) pop is real. Every edge cost is
        // non-negative (edgePropagation returns |differences| and += mapResolution), so a
        // popped state's cost is final and it can be closed - which is what bounds the work.
        std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry> > Queue;
        Queue.push(QueueEntry(0.0f, startState));

        while(!Queue.empty() && ros::ok()){

            state_t *fromState = Queue.top().second;
            Queue.pop();

            if (fromState->closedFlag) // stale heap entry, superseded by a cheaper pop
                continue;
            fromState->closedFlag = true;

            // stop searching if minimum cost path to goal is found
            if (earlyExitEnabled && fromState == nearestGoalState)
                break;

            // loop through all neighbors of this state
            for (int i = 0; i < fromState->neighborList.size(); ++i){
                state_t *toState = fromState->neighborList[i].neighbor;

                if (toState->closedFlag) // already settled, its cost cannot improve
                    continue;

                float newCosts[NUM_COSTS];
                for (int j = 0; j < NUM_COSTS; ++j)
                    newCosts[j] = fromState->costsToRoot[j] + fromState->neighborList[i].edgeCosts[j];

                float newCost = compositeCost(newCosts);

                if (newCost < compositeCost(toState->costsToRoot)) {
                    updateCosts(fromState, toState, i);
                    toState->parentState = fromState;
                    Queue.push(QueueEntry(newCost, toState));
                }
            }
        }

        // 5. If the goal node was never reached, fall back to the reachable roadmap node
        // closest to the goal. When it was reached it is already the best answer, so this
        // whole block is skipped - and with it a 20 m radius query on every plan.
        if (nearestGoalState != NULL && nearestGoalState->parentState == NULL){
            vector<state_t*> nearGoalStates;
            getNearStates(nearestGoalState, nearGoalStates, 20);
            float nearGoalDist = FLT_MAX;
            for (int i = 0; i < nearGoalStates.size(); ++i){
                float dist = distance(nearGoalStates[i]->x, goalState->x);
                if (dist < nearGoalDist && nearGoalStates[i]->parentState != NULL){
                    nearGoalDist = dist;
                    nearestGoalState = nearGoalStates[i];
                }
            }
        }
        // the nearest goal state is invalid
        if (nearestGoalState == NULL || nearestGoalState->parentState == NULL) // no path to the nearestGoalState is found
            return false;

        // 6. Extract path
        state_t *thisState = nearestGoalState;
        while (thisState->parentState != NULL){
            pathList.insert(pathList.begin(), thisState);
            thisState = thisState->parentState;
        }
        pathList.insert(pathList.begin(), robotState); // add current robot state
        // pathList.push_back(goalState); // add goal state

        // 7. Smooth path
        smoothPath();

        // Latch this exact PRM endpoint. Even though the rolling map and PRM
        // continue to update, buildRoadMap() will not call bfsSearch() again
        // until the robot comes within waypointReachDistance of these stored
        // coordinates (or a genuinely new final goal is received).
        waypointX = nearestGoalState->x[0];
        waypointY = nearestGoalState->x[1];
        waypointZ = nearestGoalState->x[2];
        activeWaypoint = true;

        ROS_INFO("Latched PRM waypoint: [%.2f, %.2f, %.2f] -> final goal [%.2f, %.2f, %.2f]",
                 waypointX, waypointY, waypointZ,
                 goalState->x[0], goalState->x[1], goalState->x[2]);

        return true;
    }

    // The single scalar bfsSearch minimizes. Used for BOTH the priority queue ordering and the
    // relaxation test: if those two disagree the heap can pop a state whose cost is not yet
    // final, which breaks Dijkstra's settling invariant. That is exactly what the previous
    // implementation did - it popped lexicographically by costHierarchy while relaxing on a
    // weighted sum - and it is why states were re-expanded over and over.
    static float compositeCost(const float costs[NUM_COSTS]){
        if (costs[0] == FLT_MAX) // unreached; avoid overflowing to inf
            return FLT_MAX;
        return pathCostWeightTraversability * (costs[0] / pathCostScaleTraversability)
             + pathCostWeightElevation      * (costs[1] / pathCostScaleElevation)
             + pathCostWeightDistance       * (costs[2] / pathCostScaleDistance);
    }

    void updateCosts(state_t* fromState, state_t* toState, int neighborInd){
        for (int i = 0; i < NUM_COSTS; ++i)
            toState->costsToRoot[i] = fromState->costsToRoot[i] + fromState->neighborList[neighborInd].edgeCosts[i];
    }

    void smoothPath(){

        if (pathList.size() <= 1)
            return;
        // Cubic Spline
        nav_msgs::Path originPath;
        nav_msgs::Path splinePath;
        
        originPath.header.frame_id = "map";
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";

        originPath.poses.clear();

        for (int i = 0; i < pathList.size(); i++){
            pose.pose.position.x = pathList[i]->x[0];
            pose.pose.position.y = pathList[i]->x[1];
            pose.pose.position.z = pathList[i]->x[2];
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
            originPath.poses.push_back(pose);
        }

        path_smoothing::CubicSplineInterpolator csi("smooth");
        csi.interpolatePath(originPath, splinePath);

        globalPath = splinePath;
        displayGlobalPath = globalPath; // displayGlobalPath is only changed during planning
    }    

    void generateSamples(){

        double sampling_start_time = ros::WallTime::now().toSec();
        while (ros::WallTime::now().toSec() - sampling_start_time < samplingTimeBudget && ros::ok()){

            state_t* newState = new state_t;

            if (sampleState(newState)){
                // 1.1 Too close discard
                if (nodeList.size() != 0 && stateTooClose(newState) == true){
                    delete newState;
                    continue;
                }
                // 1.2 Save new state and insert to KD-tree
                newState->stateId = nodeList.size(); // mark state ID
                nodeList.push_back(newState);
                insertIntoKdtree(newState);
            }
            else
                delete newState;
        }
    }

    bool stateTooClose(state_t *stateIn){
        // Minimum spacing between roadmap nodes, measured on the GROUND PLANE. One node per
        // (x,y) neighbourhood, whatever its elevation - the map is 2.5D, so two nodes in one
        // vertical column are duplicates by construction, and each one carries a full neighbour
        // list, which is what multiplies the edge count.
        //
        // This used the 3D distance(), so a sample 5 cm away in (x,y) but 60 cm away in
        // elevation measured 0.602 m, cleared neighborSampleRadius and was admitted on top of
        // the existing node. ~89% of nodes were stacked this way.
        //
        // getNearestState queries a 2D kdtree, so the node it returns is the true ground-plane
        // nearest and testing it alone is sufficient.
        state_t *nearestState = getNearestState(stateIn);
        if (nearestState == NULL)
            return false;

        return distance2D(stateIn->x, nearestState->x) <= neighborSampleRadius;
    }

    void updateStatesAndEdges(){
        getRobotState();
        // 0. find local map center
        mapCenter->x[0] = (map_min[0] + map_max[0]) / 2;
        mapCenter->x[1] = (map_min[1] + map_max[1]) / 2;
        mapCenter->x[2] = robotState->x[2];
        // 1. add edges for the nodes that are within a certain radius of mapCenter
        vector<state_t*> nearStates;
        getNearStates(mapCenter, nearStates, neighborSearchRadius);
        if (nearStates.size() == 0)
            return;
        // 3. update states height values (because the map is changing all the time)
        for (int i = 0; i < nearStates.size(); ++i){
            float thisHeight = getStateHeight(nearStates[i]);
            if (thisHeight != -FLT_MAX) // new height can be -FLT_MAX since the map is shifted a bit every time (round error)
                nearStates[i]->x[2]  = thisHeight;
        }
        // 4. loop through all neighbors
        float edgeCosts[NUM_COSTS];
        neighbor_t thisNeighbor;
        for (int i = 0; i < nearStates.size(); ++i){
            for (int j = i+1; j < nearStates.size(); ++j){
                // 4.1 height difference larger than threshold, too steep to connect
                if (abs(nearStates[i]->x[2] - nearStates[j]->x[2]) > neighborConnectHeight){
                    deleteEdge(nearStates[i], nearStates[j]);
                    continue;
                }
                // 4.2 distance larger than x, too far to connect
                float distanceBetween = distance(nearStates[i]->x, nearStates[j]->x);
                if (distanceBetween > neighborConnectRadius || distanceBetween < 0.3){
                    deleteEdge(nearStates[i], nearStates[j]);
                    continue;
                }
                // 4.3 this edge is connectable
                if(edgePropagation(nearStates[i], nearStates[j], edgeCosts) == true){
                    // even if edge already exists, we still need to update costs (casuse elevation may change)
                    deleteEdge(nearStates[i], nearStates[j]);
                    for (int k = 0; k < NUM_COSTS; ++k)
                        thisNeighbor.edgeCosts[k] = edgeCosts[k];

                    thisNeighbor.neighbor = nearStates[j];
                    nearStates[i]->neighborList.push_back(thisNeighbor);
                    thisNeighbor.neighbor = nearStates[i];
                    nearStates[j]->neighborList.push_back(thisNeighbor);
                }else{ // edge is not connectable, delete old edge if it exists
                    deleteEdge(nearStates[i], nearStates[j]);
                }
            } 
        }
    }

    void deleteEdge(state_t* stateA, state_t* stateB){
        // "remove" compacts the elements that differ from the value to be removed (state_in) in the beginning of the vector 
        // and returns the iterator to the first element after that range. Then "erase" removes these elements (who's value is unspecified).
        compareState = stateB;
        stateA->neighborList.erase(std::remove_if(stateA->neighborList.begin(), stateA->neighborList.end(), isStateExsiting), stateA->neighborList.end());
        compareState = stateA;
        stateB->neighborList.erase(std::remove_if(stateB->neighborList.begin(), stateB->neighborList.end(), isStateExsiting), stateB->neighborList.end());
    }

    

    bool edgePropagation(state_t *state_from, state_t *state_to, float edgeCosts[NUM_COSTS]){
        // 0. initialize edgeCosts
        for (int i = 0; i < NUM_COSTS; ++i)
            edgeCosts[i] = 0;
        // 1. segment the edge for collision checking
        int steps = floor(distance(state_from->x, state_to->x) / (mapResolution));
        float stepX = (state_to->x[0]-state_from->x[0]) / steps;
        float stepY = (state_to->x[1]-state_from->x[1]) / steps;
        float stepZ = (state_to->x[2]-state_from->x[2]) / steps;
        // 2. allocate memory for a state, this state must be deleted after collision checking
        state_t *stateCurr = new state_t;;
        stateCurr->x[0] = state_from->x[0];
        stateCurr->x[1] = state_from->x[1];
        stateCurr->x[2] = state_from->x[2];

        int rounded_x, rounded_y, indexInLocalMap;

        // Elevation cost is the accumulated |height change| ALONG the edge, so it needs the
        // previous step's height; seed it from the start node's cell.
        int fromIndex = (int)((state_from->x[0] - map_min[0]) / mapResolution)
                      + (int)((state_from->x[1] - map_min[1]) / mapResolution) * elevationMap.occupancy.info.width;
        float prevElevation = (fromIndex >= 0 && fromIndex < (int)elevationMap.height.size())
                            ? elevationMap.height[fromIndex] : -FLT_MAX;

        // 3. collision checking loop
        for (int stepCount = 0; stepCount < steps; ++stepCount){
            stateCurr->x[0] += stepX;
            stateCurr->x[1] += stepY;
            stateCurr->x[2] += stepZ;

            rounded_x = (int)((stateCurr->x[0] - map_min[0]) / mapResolution);
            rounded_y = (int)((stateCurr->x[1] - map_min[1]) / mapResolution);
            indexInLocalMap = rounded_x + rounded_y * elevationMap.occupancy.info.width;

            if (isIncollision(rounded_x, rounded_y, indexInLocalMap)){
                delete stateCurr;
                return false;
            }

            // Costs are LINE INTEGRALS along the edge, not endpoint differences. The previous
            // version used |occupancy(to) - occupancy(from)|, which is zero for an edge that
            // runs entirely through bad terrain at a constant score - exactly the case the
            // planner most needs to avoid. It also recomputed the same endpoint values on every
            // step of this loop. Integrating means cost ~ (how bad) x (how far through it),
            // so the planner trades terrain quality against distance instead of ignoring it.
            if (costUpdateFlag[0]) {
                int occ = elevationMap.occupancy.data[indexInLocalMap];
                // Unknown reads as -1. Left raw it would make unobserved ground the cheapest
                // terrain on the map and actively attract the planner into it.
                if (occ < 0) occ = unknownCellTraversability;
                edgeCosts[0] += float(occ) * mapResolution;
            }

            if (costUpdateFlag[1]) {
                float thisElevation = elevationMap.height[indexInLocalMap];
                if (thisElevation != -FLT_MAX){
                    if (prevElevation != -FLT_MAX)
                        edgeCosts[1] += std::abs(thisElevation - prevElevation);
                    prevElevation = thisElevation;
                }
            }

            // costs propagation
            if (costUpdateFlag[2])
                edgeCosts[2] = edgeCosts[2] + mapResolution; // distance cost
        }
        delete stateCurr;
        return true;
    }

    // Collision check (using rounded index for input)
    bool isIncollision(int rounded_x, int rounded_y, int index){
        if (rounded_x < 0 || rounded_x >= localMapArrayLength ||
            rounded_y < 0 || rounded_y >= localMapArrayLength )
            return true;

        // Test the INFLATED map, not the raw occupancy. updateCostMap spreads every obstacle
        // cell by costmapInflationRadius precisely so the planner keeps the robot's width away
        // from it; reading occupancy.data here bypassed that entirely and made the inflation
        // pass dead code, which is why edges hugged obstacles and flooded the floor.
        if (elevationMap.costMap[index] != 0)
            return true;

        if (planningUnknown == false){
            // stateIn->x is on an unknown grid
            if (elevationMap.height[index] == -FLT_MAX)
                return true;
        }

        return false;
    }

    void getNearStates(state_t *stateIn, vector<state_t*>& vectorNearStatesOut, double radius){
        kdres_t *kdres = kd_nearest_range (kdtree, stateIn->x, radius);
        vectorNearStatesOut.clear();
        // Create the vector data structure for storing the results
        int numNearVertices = kd_res_size (kdres);
        if (numNearVertices == 0) {
            kd_res_free (kdres);
            return;
        }
        // Place pointers to the near vertices into the vector 
        kd_res_rewind (kdres);
        while (!kd_res_end(kdres)) {
            state_t *stateCurr = (state_t *) kd_res_item_data (kdres);
            vectorNearStatesOut.push_back(stateCurr);
            kd_res_next(kdres);
        }
        // Free temporary memory
        kd_res_free (kdres);
    }


    bool sampleState(state_t *stateCurr){
        // random x and y
        for (int i = 0; i < 2; ++i)
            stateCurr->x[i] = (double)rand()/(RAND_MAX + 1.0)*(map_max[i] - map_min[i]) 
                - (map_max[i] - map_min[i])/2.0 + (map_max[i] + map_min[i])/2.0;
        // random heading
        stateCurr->theta = (double)rand()/(RAND_MAX + 1.0) * 2 * M_PI - M_PI;
        // collision checking before getting height info
        if (isIncollision(stateCurr))
            return false;
        // z is not random since the robot is constrained to move on the ground
        stateCurr->x[2] = getStateHeight(stateCurr);
        if (stateCurr->x[2] == -FLT_MAX)
            return false;

        return true;
    }

    double getStateHeight(state_t* stateIn){
        int rounded_x = (int)((stateIn->x[0] - map_min[0]) / mapResolution);
        int rounded_y = (int)((stateIn->x[1] - map_min[1]) / mapResolution);
        return elevationMap.height[rounded_x + rounded_y * elevationMap.occupancy.info.width];
    }

    // Collision check (using state for input)
    bool isIncollision(state_t* stateIn){
        // if the state is outside the map, discard this state
        if (stateIn->x[0] <= map_min[0] || stateIn->x[0] >= map_max[0] 
            || stateIn->x[1] <= map_min[1] || stateIn->x[1] >= map_max[1])
            return true;
        // if the distance to the nearest obstacle is less than xxx, in collision
        int rounded_x = (int)((stateIn->x[0] - map_min[0]) / mapResolution);
        int rounded_y = (int)((stateIn->x[1] - map_min[1]) / mapResolution);
        int index = rounded_x + rounded_y * elevationMap.occupancy.info.width;

        // inflated map - see the index-based overload above
        if (elevationMap.costMap[index] != 0)
            return true;

        if (planningUnknown == false){
            // stateIn->x is on an unknown grid
            if (elevationMap.height[index] == -FLT_MAX)
                return true;
        }

        return false;
    }

    void insertIntoKdtree(state_t *stateCurr){
        kd_insert(kdtree, stateCurr->x, stateCurr);
    }

    state_t* getNearestState(state_t *stateIn){
        kdres_t *kdres = kd_nearest(kdtree, stateIn->x);
        if (kd_res_end (kdres)){
            kd_res_free (kdres);
            return NULL;
        }
        state_t* nearestState = (state_t*) kd_res_item_data(kdres);
        kd_res_free (kdres);
        return nearestState;
    }

    // Euclidean distance between two samples. Note this is 3D despite the original comment;
    // it is the right measure for edge LENGTH (an edge climbing a slope really is longer).
    float distance(double state_from[3], double state_to[3]){
        return sqrt((state_to[0]-state_from[0])*(state_to[0]-state_from[0]) +
                    (state_to[1]-state_from[1])*(state_to[1]-state_from[1]) +
                    (state_to[2]-state_from[2])*(state_to[2]-state_from[2]));
    }

    // Ground-plane distance. The right measure for node SPACING: the roadmap covers a 2.5D
    // surface, so how far apart two nodes are on the ground is what sets sampling density.
    float distance2D(double state_from[3], double state_to[3]){
        return sqrt((state_to[0]-state_from[0])*(state_to[0]-state_from[0]) +
                    (state_to[1]-state_from[1])*(state_to[1]-state_from[1]));
    }


    void publishPRM(){        

        // Path
        if (pubPRMPath.getNumSubscribers() != 0){

            visualization_msgs::MarkerArray markerArray;
            geometry_msgs::Point p;

            // path visualization
            visualization_msgs::Marker markerPath;
            markerPath.header.frame_id = "map";
            markerPath.header.stamp = ros::Time::now();
            markerPath.action = visualization_msgs::Marker::ADD;
            markerPath.type = visualization_msgs::Marker::LINE_STRIP;
            markerPath.ns = "path";
            markerPath.id = 0;
            markerPath.scale.x = 0.2;
            markerPath.color.r = 0.0; markerPath.color.g = 0; markerPath.color.b = 1.0;
            markerPath.color.a = 1.0;

            for (int i = 0; i < displayGlobalPath.poses.size(); ++i){
                p.x = displayGlobalPath.poses[i].pose.position.x;
                p.y = displayGlobalPath.poses[i].pose.position.y;
                p.z = displayGlobalPath.poses[i].pose.position.z + 0.3;
                markerPath.points.push_back(p);
            }
            
            // goal point visualization
            visualization_msgs::Marker markerGoal;
            markerGoal.header.frame_id = "map";
            markerGoal.header.stamp = ros::Time::now();
            markerGoal.action = visualization_msgs::Marker::ADD;
            markerGoal.type= visualization_msgs::Marker::SPHERE_LIST;
            markerGoal.ns = "goal";
            markerGoal.id = 1;
            markerGoal.scale.x = 0.5;
            markerGoal.color.r = 0.0; markerGoal.color.g = 0.0; markerGoal.color.b = 1.0;
            markerGoal.color.a = 1.0;

            if (displayGlobalPath.poses.size() != 0){
                p.x = displayGlobalPath.poses.back().pose.position.x;
                p.y = displayGlobalPath.poses.back().pose.position.y;
                p.z = displayGlobalPath.poses.back().pose.position.z + 0.3;
                markerGoal.points.push_back(p);
            }

            // push to markerarray and publish
            markerArray.markers.push_back(markerPath);
            markerArray.markers.push_back(markerGoal);
            pubPRMPath.publish(markerArray);
        }


        if (pubPRMGraph.getNumSubscribers() != 0){

            visualization_msgs::MarkerArray markerArray;
            geometry_msgs::Point p;

            // PRM nodes visualization
            visualization_msgs::Marker markerNode;
            markerNode.header.frame_id = "map";
            markerNode.header.stamp = ros::Time::now();
            markerNode.action = visualization_msgs::Marker::ADD;
            markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
            markerNode.ns = "nodes";
            markerNode.id = 2;
            markerNode.scale.x = 0.2;
            markerNode.color.r = 0; markerNode.color.g = 1; markerNode.color.b = 1;
            markerNode.color.a = 1;

            for (int i = 0; i < nodeList.size(); ++i){
                if (distance(nodeList[i]->x, robotState->x) >= visualizationRadius)
                    continue;
                p.x = nodeList[i]->x[0];
                p.y = nodeList[i]->x[1];
                p.z = nodeList[i]->x[2] + 0.13;
                markerNode.points.push_back(p);
            }

            // PRM edge visualization with cost-based coloring
            visualization_msgs::Marker markerEdge;
            markerEdge.header.frame_id = "map";
            markerEdge.header.stamp = ros::Time::now();
            markerEdge.action = visualization_msgs::Marker::ADD;
            markerEdge.type = visualization_msgs::Marker::LINE_LIST;
            markerEdge.ns = "edges";
            markerEdge.id = 3;
            markerEdge.scale.x = 0.05;
            markerEdge.pose.orientation.w = 1.0;

            for (int i = 0; i < nodeList.size(); ++i){
                if (distance(nodeList[i]->x, robotState->x) >= visualizationRadius)
                    continue;

                for (int j = 0; j < nodeList[i]->neighborList.size(); ++j){
                    state_t* neighbor = nodeList[i]->neighborList[j].neighbor;
                    float* edgeCosts = nodeList[i]->neighborList[j].edgeCosts;

                    // Combine all 3 costs: traversability (0), elevation (1), and distance (2)
                    float combinedCost = edgeCosts[0] + edgeCosts[1] + edgeCosts[2];

                    // Normalize combined cost (adjust denominator based on expected max total cost)
                    float maxCombinedCost = 100.0f;  // example: max traversability + elevation + distance
                    float t = std::min(std::max(combinedCost / maxCombinedCost, 0.0f), 1.0f);

                    // Use color to reflect combined cost: red = high cost, green = low cost
                    std_msgs::ColorRGBA color;
                    color.a = 1.0;
                    color.r = t;
                    color.g = 1.0 - t;
                    color.b = 0.0;

                    geometry_msgs::Point p1, p2;
                    p1.x = nodeList[i]->x[0]; p1.y = nodeList[i]->x[1]; p1.z = nodeList[i]->x[2] + 0.1;
                    p2.x = neighbor->x[0];    p2.y = neighbor->x[1];    p2.z = neighbor->x[2] + 0.1;

                    markerEdge.points.push_back(p1);
                    markerEdge.colors.push_back(color);
                    markerEdge.points.push_back(p2);
                    markerEdge.colors.push_back(color);
                }
            }


            // push to markerarray and publish
            markerArray.markers.push_back(markerNode);
            markerArray.markers.push_back(markerEdge);
            pubPRMGraph.publish(markerArray);

        }

        // 4. Single Source Shortest Paths
        if (pubSingleSourcePaths.getNumSubscribers() != 0){

            visualization_msgs::MarkerArray markerArray;
            geometry_msgs::Point p;

             // publish empty single-source paths
            if (planningFlag == false){
                pubSingleSourcePaths.publish(markerArray);
                return;
            }

            // single source path visualization
            visualization_msgs::Marker markersPath;
            markersPath.header.frame_id = "map";
            markersPath.header.stamp = ros::Time::now();
            markersPath.action = visualization_msgs::Marker::ADD;
            markersPath.type = visualization_msgs::Marker::LINE_LIST;
            markersPath.ns = "path";
            markersPath.id = 4;
            markersPath.scale.x = 0.05;
            markersPath.color.r = 0.3; markersPath.color.g = 0; markersPath.color.b = 1.0;
            markersPath.color.a = 1.0;

            for (int i = 0; i < nodeList.size(); ++i){
                if (nodeList[i]->parentState == NULL)
                    continue;
                p.x = nodeList[i]->x[0];
                p.y = nodeList[i]->x[1];
                p.z = nodeList[i]->x[2] + 0.2;
                markersPath.points.push_back(p);
                p.x = nodeList[i]->parentState->x[0];
                p.y = nodeList[i]->parentState->x[1];
                p.z = nodeList[i]->parentState->x[2]+0.2;
                markersPath.points.push_back(p);
            }
            // push to markerarray and publish
            markerArray.markers.push_back(markersPath);
            pubSingleSourcePaths.publish(markerArray);
        }
    }

    void publishCurrentPath(){
        // Publishing and planner-state transitions are intentionally separated.
        // Re-publishing this message while activeWaypoint == true does NOT mean
        // replanning; globalPath remains the path produced by the last bfsSearch().
        globalPath.header.frame_id = "map";
        globalPath.header.stamp = ros::Time::now();
        pubGlobalPath.publish(globalPath);
    }

    void publishRoadmap2Cloud(){
        if (pubCloudPRMNodes.getNumSubscribers() == 0 && pubCloudPRMGraph.getNumSubscribers() == 0)
            return;

        int sizeCloud = nodeList.size();
        // PRM nodes to point cloud
        PointType thisPoint;
        pcl::PointCloud<PointType> nodeCloud; nodeCloud.resize(sizeCloud);
        pcl::PointCloud<PointType> adjacencyCloud; adjacencyCloud.resize(sizeCloud*sizeCloud);
        for (int i = 0; i < sizeCloud; ++i){
            // save node
            thisPoint.x = nodeList[i]->x[0];
            thisPoint.y = nodeList[i]->x[1];
            thisPoint.z = nodeList[i]->x[2]+0.15;
            thisPoint.intensity = i;
            nodeCloud.points[i] = thisPoint;
            // extract adjacency matrix into cloud
            int numNeighbors = nodeList[i]->neighborList.size();
            for (int j = 0; j < numNeighbors; ++j){
                int index = nodeList[i]->stateId + sizeCloud * nodeList[i]->neighborList[j].neighbor->stateId;
                adjacencyCloud.points[index].intensity = 1;
            }
        }
        // Publish
        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(nodeCloud, laserCloudTemp);
        laserCloudTemp.header.frame_id = "map";
        laserCloudTemp.header.stamp = ros::Time::now();
        pubCloudPRMNodes.publish(laserCloudTemp);
        pcl::toROSMsg(adjacencyCloud, laserCloudTemp);
        laserCloudTemp.header.frame_id = "map";
        laserCloudTemp.header.stamp = ros::Time::now();
        pubCloudPRMGraph.publish(laserCloudTemp);
    }

    void getRobotState(){
        try{listener.lookupTransform("map","base_link", ros::Time(0), transform); } 
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return; }
        robotState->x[0] = transform.getOrigin().x();
        robotState->x[1] = transform.getOrigin().y();
        robotState->x[2] = transform.getOrigin().z();

        double roll, pitch, yaw;
        tf::Matrix3x3 m(transform.getRotation());
        m.getRPY(roll, pitch, yaw);
        robotState->theta = yaw + M_PI; // change from -PI~PI to 0~1*PI
    }

};


int main(int argc, char** argv){

    ros::init(argc, argv, "traversability_mapping");

    TraversabilityPRM TPRM;

    ROS_INFO("\033[1;32m---->\033[0m Traversability Planner Started.");

    ros::spin();
    return 0;
}
