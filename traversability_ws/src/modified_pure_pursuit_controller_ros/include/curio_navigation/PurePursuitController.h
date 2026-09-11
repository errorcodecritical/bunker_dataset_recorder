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

/** \file PurePursuitController.h
    \brief This file defines the PurePursuitController class which
           implements a pure pursuit controller.
  */

#ifndef PURE_PURSUIT_CONTROLLER_H
#define PURE_PURSUIT_CONTROLLER_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>

#include <boost/bind.hpp>
#include <pure_pursuit_controller/FollowPathAction.h>
#include <actionlib/server/simple_action_server.h>


/** The class PurePursuitController implements a pure pursuit controller.
	\brief Pure pursuit controller
*/
class PurePursuitController {
public:
	/** \name Constructors/destructor @{ */
	/// Constructor
	PurePursuitController(const ros::NodeHandle& nh);
	/// Copy constructor
	PurePursuitController(const PurePursuitController& other) = delete;
	/// Copy assignment operator
	PurePursuitController& operator =
		(const PurePursuitController& other) = delete;
	/// Move constructor
	PurePursuitController(PurePursuitController&& other) = delete;
	/// Move assignment operator
	PurePursuitController& operator =
		(PurePursuitController&& other) = delete;
	/// Destructor
	virtual ~PurePursuitController();
	/** @} */

	/** \name Methods @{ */
	/// Spin once
	void spin();
	/// Step once
	bool step(geometry_msgs::Twist& twist);
	/// Returns the current pose of the robot
	geometry_msgs::PoseStamped getCurrentPose() const;
	/// Returns the lookahead distance for the given pose
	double getDistanceToPose(const geometry_msgs::PoseStamped& pose) const;
	/// Returns the lookahead angle for the given pose in [rad]
	double getAngleToPose(const geometry_msgs::PoseStamped& pose) const;
	/// Returns the current lookahead distance threshold
	double getLookAheadThreshold();
	/// Returns the next way point by linear search from the current waypoint
	int getNextWayPoint();    
	/// Returns the current closest waypoint
	int getClosestWayPoint() const;
	/// Projects the current robot pose onto the reference path polyline and
	/// returns the arc-length of the foot point (progress along the path)
	double projectOntoPath() const;
	/// Estimate curvature ahead from path starting at given waypoint index
	double estimateCurvatureAhead(int startIndex) const;
	int findEndOfStraightSegment(int start, double curvatureThreshold);
	double pathArcLength(int i, int j);
	/** @} */

	protected:
	/** \name Protected methods @{ */
	/// Retrieves parameters
	void getParameters();
	/// Initialize messages
	void initMessages();
	/// Reference path message callback
	void pathCallback(const nav_msgs::Path& msg);
	/// Odometry message callback
	void odometryCallback(const nav_msgs::Odometry& msg);
	/// Timer callback
    void timerCallback(const ros::TimerEvent& event);
	/** @} */

	/** \name Protected members @{ */
	/// ROS node handle
	ros::NodeHandle _nodeHandle;
	/// Reference path message subscriber
	ros::Subscriber _pathSubscriber;
	/// Reference path message topic name
	std::string _pathTopicName;
	/// Odometry message subscriber
	ros::Subscriber _odometrySubscriber;
	/// Odometry message topic name
	std::string _odometryTopicName;
	/// Frame id of reference path
	std::string _refPathFrameId;
	/// Frame id of pose estimates
	std::string _poseFrameId;
	/// Queue size for receiving messages
	int _queueDepth;
	/// Current reference path
	nav_msgs::Path _currentReferencePath;
	/// Reference Path Pose length
	int _refPathLength;
	/// Current pose
	geometry_msgs::PoseStamped _currentPose;
	/// Flag to indicate if odometry has been received at least once
	bool _odomReceived;
	/// Current velocity
	geometry_msgs::Twist _currentVelocity;
	/// Controller frequency
	double _control_frequency;
	/// Next way point
	int _nextWayPoint;
	/// Commanded velocity publisher
	ros::Publisher _cmdVelocityPublisher;
	/// Commanded velocity topic name
	std::string _cmdVelocityTopicName;
	/// Interpolated waypoint pose publisher
	ros::Publisher _targetWayPointPublisher;
	/// Interpolated waypoint topic name
	std::string _targetWayPointPubTopicName;
	/// target waypoint marker message
	visualization_msgs::Marker _target_marker_msg;
	/// Path end pose
	geometry_msgs::Pose _pathEndPose;
	/// Velocity
	double _velocity;
	/// Epsilon
	double _epsilon;
	/// Transform listener for robot's pose w.r.t. map
	tf::TransformListener _tfListener;
	/// Main Timer
	ros::Timer _timer;
	
	/* --- New members for adaptive curvature-based look-ahead --- */

	/// Minimum allowable look-ahead distance (Ld_min)
	double _LdMin;
	/// Maximum allowable look-ahead distance (Ld_max)
	double _LdMax;
	/// Maximum curvature threshsold to accept as a straight path (above, its a curved section)
	double _curvatureThreshold;
	/** @} */
	
	actionlib::SimpleActionServer<pure_pursuit_controller::FollowPathAction> _actionServer;

private:
	void executeCB(
        	const pure_pursuit_controller::FollowPathGoalConstPtr& goal);

    void resetController();

};


#endif // PURE_PURSUIT_CONTROLLER_H