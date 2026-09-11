/******************************************************************************
 * Copyright (C) 2014 by Todd Tang                                          *
 * todd.j.tang@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "curio_navigation/PurePursuitController.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <geometry_msgs/Twist.h>

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

PurePursuitController::PurePursuitController(const ros::NodeHandle& nh) 
	: _nodeHandle(nh)
	, _nextWayPoint(-1)
	, _odomReceived(false)
	, _actionServer(
		nh,
		"follow_path",
		boost::bind(&PurePursuitController::executeCB, this, _1),
		false)   // auto_start = false
{
	getParameters();

	initMessages();

	_pathSubscriber = _nodeHandle.subscribe(
		_pathTopicName, _queueDepth,
		&PurePursuitController::pathCallback, this);

	_odometrySubscriber = _nodeHandle.subscribe(
		_odometryTopicName, _queueDepth,
		&PurePursuitController::odometryCallback, this);
		
	_cmdVelocityPublisher = _nodeHandle.advertise<geometry_msgs::Twist>(
		_cmdVelocityTopicName, _queueDepth);

	_targetWayPointPublisher = _nodeHandle.advertise<visualization_msgs::Marker>(
		_targetWayPointPubTopicName, _queueDepth);

	_timer = _nodeHandle.createTimer(ros::Duration(1.0/_control_frequency),
		&PurePursuitController::timerCallback, this);
	
	_actionServer.start();
}

PurePursuitController::~PurePursuitController() 
{

}

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

void PurePursuitController::spin() 
{	
	ros::spin();
}


void PurePursuitController::pathCallback(const nav_msgs::Path& msg) 
{

	// Ignore topic path while an action goal is active
	if (_actionServer.isActive()) return;

	if (msg.poses.empty()) return;

	_refPathFrameId = msg.header.frame_id;
	_currentReferencePath = msg;
	_refPathLength = _currentReferencePath.poses.size();
	_nextWayPoint = getClosestWayPoint();
	_pathEndPose = _currentReferencePath.poses.back().pose;
	ROS_INFO_STREAM("Received reference path of size " << _refPathLength);
}


void PurePursuitController::odometryCallback(const nav_msgs::Odometry& msg) 
{	
	_odomReceived = true;

	_currentPose.header = msg.header;
	_currentPose.pose = msg.pose.pose;
	_currentVelocity = msg.twist.twist;
}


void PurePursuitController::timerCallback(const ros::TimerEvent& event)
{
	// Don't act on default-constructed pose data before the first
	// odometry message has actually arrived.
	if (!_odomReceived) {
		ROS_WARN_THROTTLE(2.0, "Waiting for odometry...");
		return;
	}

	if (_nextWayPoint == -1) {
		ROS_WARN_ONCE("No reference path received, will keep waiting...");
		return;
	}
	
	geometry_msgs::Point currentPosition = getCurrentPose().pose.position;
	tf::Vector3 v1(currentPosition.x, currentPosition.y, 0.0);
	tf::Vector3 v2(_pathEndPose.position.x, _pathEndPose.position.y, 0.0);

	if (tf::tfDistance(v1, v2) < 0.50f) {
	    ROS_INFO_STREAM_ONCE("Path Following Finished. Waiting for new path...");

	    resetController();

	    return;  // do nothing until a new path arrives
	}

	geometry_msgs::Twist cmdVelocity;

	if (step(cmdVelocity)) {
		_cmdVelocityPublisher.publish(cmdVelocity);
	}
}


bool PurePursuitController::step(geometry_msgs::Twist& twist) 
{
	twist.linear.x = 0.0;
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;

	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = 0.0;

	_nextWayPoint = getNextWayPoint();

	if (_nextWayPoint >= 0) 
	{
		geometry_msgs::PoseStamped target = _currentReferencePath.poses[_nextWayPoint];

		_target_marker_msg.pose = target.pose;
		_targetWayPointPublisher.publish(_target_marker_msg);

		// ld: delta-distance from current pose to goal pose
		double l_t = getLookAheadThreshold();

		geometry_msgs::PoseStamped origin = getCurrentPose();
		double dy = target.pose.position.y - origin.pose.position.y;
		double dx = target.pose.position.x - origin.pose.position.x;
		// alpha: delta-angle from current pose to goal pose
		double alpha = atan2(dy, dx) - tf::getYaw(origin.pose.orientation);

		ROS_INFO("Alpha (degrees): %.2f", alpha * 180.0 / M_PI);
		
		// ───────────────────────────────
		// Added: rotate in place if |alpha| > 45°
		// ───────────────────────────────
		// Normalize alpha
		alpha = atan2(std::sin(alpha), std::cos(alpha));
		if (std::abs(alpha) > M_PI/4.0) {
		    twist.linear.x = 0.0;
		    twist.angular.z = (alpha > 0 ? 1.0 : -1.0);  // any constant turn rate
		    return true;
		}
		// ───────────────────────────────

		double angularVelocity = 0.0;

		if (std::abs(std::sin(alpha)) >= _epsilon) 
		{
			// r = ld / (2 * sin(alpha))
			double radius = 0.5 * (l_t / std::sin(alpha));
			
			// try to reach the desired linear velocity
			double linearVelocity = _velocity;

			// only steering when delta_angle is larger than error_threshold _epsilon
			if (std::abs(radius) >= _epsilon)
				// omega = v / r
				angularVelocity = linearVelocity / radius;

			twist.linear.x = linearVelocity;
			twist.angular.z = angularVelocity;
			
		}

		return true;
	}

	return false;
}


geometry_msgs::PoseStamped PurePursuitController::getCurrentPose() const 
{
	geometry_msgs::PoseStamped currentPose = _currentPose;

	if (currentPose.pose.orientation.x == 0.0 &&
	    currentPose.pose.orientation.y == 0.0 &&
	    currentPose.pose.orientation.z == 0.0 &&
	    currentPose.pose.orientation.w == 0.0) {
		ROS_WARN_STREAM_ONCE("Current pose has zero quaternion, returning untransformed pose.");
		currentPose.pose.orientation.w = 1.0;
		return currentPose;
	}

	// No transform needed if already in the reference path frame
	if (currentPose.header.frame_id == _refPathFrameId || _refPathFrameId.empty()) {
		return currentPose;
	}

	// Ask TF for the latest available transform instead of the exact stamped time,
	// to avoid extrapolation exceptions from small timing mismatches
	currentPose.header.stamp = ros::Time(0);

	geometry_msgs::PoseStamped transformedPose;
	try {
		_tfListener.transformPose(_refPathFrameId, currentPose, transformedPose);
	}
	catch (tf::TransformException& exception) {
		ROS_ERROR_STREAM_THROTTLE(1.0,
			"PurePursuitController::getCurrentPose: " << exception.what()
			<< " -- falling back to last known untransformed pose");
		return currentPose;   // real, current pose -- never a blank default
	}

	return transformedPose;
}

double PurePursuitController::getDistanceToPose(
	const geometry_msgs::PoseStamped& pose) const 
{
	geometry_msgs::PoseStamped origin = getCurrentPose();
	geometry_msgs::PoseStamped transformedPose;

	try {
		_tfListener.transformPose(_refPathFrameId, pose, transformedPose);
	}
	catch (tf::TransformException& exception) {
		ROS_ERROR_STREAM("PurePursuitController::getDistanceToPose: " << 
		exception.what());
		
		return -1.0;
	}

	tf::Vector3 v1(origin.pose.position.x,
					origin.pose.position.y,
					origin.pose.position.z);
	tf::Vector3 v2(transformedPose.pose.position.x,
					transformedPose.pose.position.y,
					origin.pose.position.z); // use the same Z coordinate as the robot pose to avoid getting trapped in rough terrain

	// Distance from current vehicle pose to current waypoint in ReferencePath frame
	return tf::tfDistance(v1, v2);
}

double PurePursuitController::getAngleToPose(
	const geometry_msgs::PoseStamped& pose) const 
{
	geometry_msgs::PoseStamped origin = getCurrentPose();
	geometry_msgs::PoseStamped transformedPose;

	try {
		_tfListener.transformPose(_refPathFrameId, pose, transformedPose);
	}
	catch (tf::TransformException& exception) {
		ROS_ERROR_STREAM("PurePursuitController::getAngleToPose: " << 
		exception.what());
		
		return -1.0;
	}

	tf::Vector3 v1(origin.pose.position.x,
					origin.pose.position.y,
					origin.pose.position.z);
	tf::Vector3 v2(transformedPose.pose.position.x,
					transformedPose.pose.position.y,
					transformedPose.pose.position.z);

	// Delta Angle from current vehilce pose to current waypoint in ReferencePath frame
	return tf::tfAngle(v1, v2);
}

double PurePursuitController::getLookAheadThreshold()
{
    int startIdx = _nextWayPoint;

    // Find the first point where curvature becomes > threshold
    int curveStart = findEndOfStraightSegment(startIdx, _curvatureThreshold);  // tweak threshold

    // Compute the arc distance from startIdx to curveStart
    double dist = pathArcLength(startIdx, curveStart);

    // If curve is far away → allow long lookahead
    if (dist > _LdMax)        
        return _LdMax;

    // Else → clamp lookahead to start of curve
    return std::max(_LdMin, dist);
}


int PurePursuitController::getNextWayPoint()
{
	if (_currentReferencePath.poses.empty())
		return -1;

	// Re-project the robot onto the path every cycle and pick the target a
	// look-ahead distance further along the arc length. This lets the
	// controller re-acquire the path correctly after an external disturbance
	// (e.g. teleop or local obstacle avoidance steering the robot off-path)
	// instead of continuing to chase the stale waypoint it was tracking
	// before the disturbance.
	const double sTarget = projectOntoPath() + getLookAheadThreshold();

	double s = 0.0;
	for (int i = 0; i + 1 < _refPathLength; ++i)
	{
		const auto& a = _currentReferencePath.poses[i].pose.position;
		const auto& b = _currentReferencePath.poses[i+1].pose.position;
		const double seg = std::hypot(b.x - a.x, b.y - a.y);

		if (s + seg >= sTarget)
			return i + 1;

		s += seg;
	}

	return _refPathLength - 1;
}


double PurePursuitController::projectOntoPath() const
{
	geometry_msgs::PoseStamped robot = getCurrentPose();
	const double rx = robot.pose.position.x;
	const double ry = robot.pose.position.y;

	double bestD2 = std::numeric_limits<double>::max();
	double bestS = 0.0;
	double s = 0.0;

	for (int i = 0; i + 1 < _refPathLength; ++i)
	{
		const auto& a = _currentReferencePath.poses[i].pose.position;
		const auto& b = _currentReferencePath.poses[i+1].pose.position;

		const double dx = b.x - a.x;
		const double dy = b.y - a.y;
		const double seg2 = dx*dx + dy*dy;

		// Project the robot onto this segment, clamped to its extent
		double t = seg2 > 0.0 ? ((rx - a.x)*dx + (ry - a.y)*dy) / seg2 : 0.0;
		t = std::max(0.0, std::min(1.0, t));

		const double fx = a.x + t*dx;
		const double fy = a.y + t*dy;
		const double d2 = (rx - fx)*(rx - fx) + (ry - fy)*(ry - fy);

		if (d2 < bestD2)
		{
			bestD2 = d2;
			bestS = s + t*std::sqrt(seg2);
		}

		s += std::sqrt(seg2);
	}

	return bestS;
}


int PurePursuitController::getClosestWayPoint() const {
	if (!_currentReferencePath.poses.empty()) {
		int closestWaypoint = 0;
		double minDistance = getDistanceToPose(_currentReferencePath.poses.front());
		
		for (int i = closestWaypoint+1; i < _refPathLength; ++i) {
			double distance = getDistanceToPose(_currentReferencePath.poses[i]);
		
			if (distance < minDistance) {
				closestWaypoint = i;
				minDistance = distance;
			}
		}

		return closestWaypoint;
	}

	return -1;
}

double PurePursuitController::estimateCurvatureAhead(int startIndex) const
{
    // Need at least three points
    if (_refPathLength < 2) return 0.0;

    // Choose three points ahead: startIndex, +N1, +N2
    const int idx1 = startIndex;
    const int idx2 = std::min(startIndex + 1, _refPathLength - 1);
    const int idx3 = std::min(startIndex + 2, _refPathLength - 1);

    geometry_msgs::Point p1 = _currentReferencePath.poses[idx1].pose.position;
    geometry_msgs::Point p2 = _currentReferencePath.poses[idx2].pose.position;
    geometry_msgs::Point p3 = _currentReferencePath.poses[idx3].pose.position;

    // Compute 2D curvature
    double x1 = p1.x, y1 = p1.y;
    double x2 = p2.x, y2 = p2.y;
    double x3 = p3.x, y3 = p3.y;

    // Side lengths
    double a = hypot(x2 - x1, y2 - y1);
    double b = hypot(x3 - x2, y3 - y2);
    double c = hypot(x3 - x1, y3 - y1);

    double s = (a + b + c) / 2.0;
    double area = std::max(1e-9, sqrt(std::max(0.0, s*(s-a)*(s-b)*(s-c))));

    // Curvature formula
    double curvature = (4.0 * area) / std::max(1e-6, (a*b*c));
    return curvature;  // positive scalar, no sign
}

int PurePursuitController::findEndOfStraightSegment(int start, double curvatureThreshold)
{
    int i = start;

    while (i + 2 < _refPathLength)
    {
        double k = estimateCurvatureAhead(i);

        if (k > curvatureThreshold)
            break;

        i++;
    }

    return i;   // index of first "curved" segment
}

double PurePursuitController::pathArcLength(int i, int j)
{
    double d = 0.0;
    for (int k = i; k < j; k++)
    {
        auto &p1 = _currentReferencePath.poses[k].pose.position;
        auto &p2 = _currentReferencePath.poses[k+1].pose.position;
        d += hypot(p2.x - p1.x, p2.y - p1.y);
    }
    return d;
}


void PurePursuitController::getParameters() 
{
	std::string param_ns = "pure_pursuit_controller/";

	_nodeHandle.param<std::string>(param_ns + "ref_path_topic_name", 
		_pathTopicName, "/reference_path");
	
	_nodeHandle.param<std::string>(param_ns + "odom_topic_name",
		_odometryTopicName, "/odometry/filtered_map");
	
	_nodeHandle.param<std::string>(param_ns + "cmd_vel_topic_name",
		_cmdVelocityTopicName, "/ackermann_drive_controller/cmd_vel");

	_nodeHandle.param<std::string>("interp_waypoint_topic_name",
		_targetWayPointPubTopicName, "/interpolated_waypoint_pose");
	
	_nodeHandle.param<std::string>("pose_frame_id", _poseFrameId, "base_link");

	_nodeHandle.param<int>(param_ns + "queue_depth", _queueDepth, 100);
	
	_nodeHandle.param<double>(param_ns + "control_frequency", _control_frequency, 20.0);
	
	_nodeHandle.param<double>(param_ns + "target_velocity", _velocity, 0.2);
	
	_nodeHandle.param<double>(param_ns + "epsilon", _epsilon, 1e-6);
	
	_nodeHandle.param<double>(param_ns + "ld_min", _LdMin, 0.5);
	
	_nodeHandle.param<double>(param_ns + "ld_max", _LdMax, 3.0);
	
	_nodeHandle.param<double>(param_ns + "curvature_threshold", _curvatureThreshold, 0.03);
}


void PurePursuitController::initMessages()
{
	// targer marker message
	_target_marker_msg.header.frame_id = "map";
	_target_marker_msg.type = visualization_msgs::Marker::SPHERE;
	_target_marker_msg.action = visualization_msgs::Marker::ADD;
	_target_marker_msg.scale.x = 0.6;
	_target_marker_msg.scale.y = 0.6;
	_target_marker_msg.scale.z = 0.6;
	_target_marker_msg.color.r = 0.5;
	_target_marker_msg.color.g = 0;
	_target_marker_msg.color.b = 1;
	_target_marker_msg.color.a = 1;
}

void PurePursuitController::executeCB(
    const pure_pursuit_controller::FollowPathGoalConstPtr& goal)
{
    ROS_INFO("FollowPathAction: Received new goal.");

    if (goal->path.poses.empty()) {
        ROS_WARN("FollowPathAction: Received empty path, rejecting goal.");
        pure_pursuit_controller::FollowPathResult r;
        r.success = false;
        r.message = "Empty path.";
        _actionServer.setAborted(r);
        return;
    }

    // Accept the goal
    _currentReferencePath = goal->path;
    _refPathFrameId = _currentReferencePath.header.frame_id;
    _refPathLength = _currentReferencePath.poses.size();
    _nextWayPoint = getClosestWayPoint();
    _pathEndPose = _currentReferencePath.poses.back().pose;

    ROS_INFO_STREAM("FollowPathAction: Path accepted, length: " << _refPathLength);

    // Feedback loop (publish current waypoint)
    ros::Rate rate(10); // 10 Hz
    while (_actionServer.isActive() && ros::ok()) {
    
    	if (_actionServer.isPreemptRequested()) {
            ROS_WARN("FollowPathAction: Preempted.");
            _nextWayPoint = -1;
            _currentReferencePath.poses.clear();
            _refPathLength = 0;

            pure_pursuit_controller::FollowPathResult r;
            r.success = false;
            r.message = "Preempted by new goal.";
            _actionServer.setPreempted(r);
            return;
        }
        
        // Send feedback to client
        pure_pursuit_controller::FollowPathFeedback feedback;
        feedback.current_waypoint = _nextWayPoint;
        feedback.distance_to_goal = (_nextWayPoint == -1 || _currentReferencePath.poses.empty()) ? 0.0 : getDistanceToPose(_currentReferencePath.poses.back());
        _actionServer.publishFeedback(feedback);
        
        if (_nextWayPoint == -1) {
            pure_pursuit_controller::FollowPathResult result;
            result.success = true;
            result.message = "Path completed successfully.";
            _actionServer.setSucceeded(result);
            return;
        }

        rate.sleep();
    }

    if (_nextWayPoint == -1) {
        ROS_INFO("FollowPathAction: Path completed!");
        
        pure_pursuit_controller::FollowPathResult result;
	result.success = true;
	result.message = "Path completed successfully.";
	_actionServer.setSucceeded(result);
    } else {
        ROS_WARN("FollowPathAction: Goal aborted (node shutting down?)");
        
        pure_pursuit_controller::FollowPathResult result;
	result.success = false;
	result.message = "Path aborted or interrupted.";
        _actionServer.setAborted(result);
    }
}


void PurePursuitController::resetController() {
    _nextWayPoint = -1;
    _currentReferencePath.poses.clear();
    _refPathLength = 0;

    // Stop the robot
    geometry_msgs::Twist stop_twist;
    stop_twist.linear.x = 0.0;
    stop_twist.linear.y = 0.0;
    stop_twist.linear.z = 0.0;
    stop_twist.angular.x = 0.0;
    stop_twist.angular.y = 0.0;
    stop_twist.angular.z = 0.0;
    _cmdVelocityPublisher.publish(stop_twist);
}