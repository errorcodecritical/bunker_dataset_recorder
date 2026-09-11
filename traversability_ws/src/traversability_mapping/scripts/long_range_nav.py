#!/usr/bin/env python
"""
long_range_nav.py

Problem it solves:
    move_base / DWA local planner will only accept a 2D Nav Goal that lies
    within the currently known (mapped) area. If you click a goal far away
    on unexplored / unknown costmap cells, move_base rejects it or aborts.

What this node does:
    1. Subscribes to the normal RViz "2D Nav Goal" topic (/move_base_simple/goal)
       and stores it as the FINAL goal (does NOT forward it to move_base directly).
    2. Subscribes to the global costmap (nav_msgs/OccupancyGrid).
    3. Gets the robot's current pose in the map frame via tf2.
    4. Raycasts in small steps from the robot's current position toward the
       final goal, walking cell-by-cell through the costmap, and finds the
       farthest point along that ray that is still:
         - inside the map bounds
         - "known" (cell value != -1)
         - "free" (cell value < occupied_threshold)
       then backs off a small safety margin from the point where it stopped.
    5. Sends that intermediate point to move_base as a normal MoveBaseGoal
       via actionlib and waits for the result.
    6. On SUCCEEDED, it recomputes the farthest reachable point again (the
       known map may have grown since the robot moved / kept mapping) and
       repeats, walking the "farthest visible waypoint" chain until the
       real final goal is within direct reach, then sends the final goal.
    7. On ABORTED, it retries a limited number of times, then gives up and
       reports failure.

This is intentionally a simple "greedy raycast" long-range goal chainer,
not a full frontier explorer or global planner replacement -- it just lets
you click a distant point in RViz and have the robot walk toward it in
map-limited hops instead of the goal being rejected outright.

Tested against: ROS Melodic (Python 2), should also run under Python 3 /
Noetic unchanged.

Usage:
    1. Drop this file into <your_pkg>/scripts/long_range_nav.py
    2. chmod +x long_range_nav.py
    3. Add it to CMakeLists.txt catkin_install_python (or just rosrun it)
    4. Run alongside your normal move_base + AMCL/SLAM stack:
         rosrun <your_pkg> long_range_nav.py
    5. In RViz, set the "2D Nav Goal" tool's topic (Tool Properties) to
       whatever ~goal_topic below is set to (default /rviz_long_range_goal)
       instead of /move_base_simple/goal, OR just leave RViz's default and
       remap this node's ~goal_topic to /move_base_simple/goal -- see the
       remap note in the __main__ section below.
"""

from __future__ import print_function

import math
import threading

import actionlib
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid


def yaw_to_quaternion(yaw):
    """Minimal yaw -> quaternion (no tf.transformations dependency)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class LongRangeNav(object):
    def __init__(self):
        # ---- parameters ----
        self.costmap_topic = rospy.get_param(
            "~costmap_topic", "/move_base/global_costmap/costmap"
        )
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.move_base_action = rospy.get_param("~move_base_action", "move_base")

        # how far apart (meters) successive raycast samples are taken.
        # smaller = more accurate stop point, more compute. Costmap
        # resolution is a good default step.
        self.step_override = rospy.get_param("~step_size", 0.0)  # 0 = use costmap resolution

        # occupancy grid cell value counts as an obstacle at/above this
        self.occupied_thresh = rospy.get_param("~occupied_thresh", 50)

        # keep this many meters of clearance back from the last free cell
        # before sending it as a waypoint, so we don't send a goal that's
        # sitting right at the edge of known space / next to an obstacle
        self.safety_margin = rospy.get_param("~safety_margin", 0.4)

        # if the straight-line remaining distance to the FINAL goal is
        # less than this, and it is reachable, just send the final goal
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.3)

        # don't bother sending a new waypoint if it's closer than this to
        # the robot (avoids spamming move_base with tiny hops)
        self.min_waypoint_distance = rospy.get_param("~min_waypoint_distance", 1.0)

        self.max_consecutive_aborts = rospy.get_param("~max_consecutive_aborts", 3)

        # ---- state ----
        self.costmap = None  # nav_msgs/OccupancyGrid
        self.final_goal = None  # PoseStamped
        self.nav_thread = None
        self.cancel_requested = threading.Event()
        self.lock = threading.Lock()

        # ---- ROS interfaces ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.client = actionlib.SimpleActionClient(
            self.move_base_action, MoveBaseAction
        )
        rospy.loginfo("long_range_nav: waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("long_range_nav: connected to move_base action server")

        rospy.Subscriber(
            self.costmap_topic, OccupancyGrid, self.costmap_cb, queue_size=1
        )
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_cb, queue_size=1)

        rospy.loginfo(
            "long_range_nav ready. Send goals on '%s', costmap from '%s'.",
            self.goal_topic,
            self.costmap_topic,
        )

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def costmap_cb(self, msg):
        with self.lock:
            self.costmap = msg

    def goal_cb(self, msg):
        rospy.loginfo(
            "long_range_nav: new final goal received: (%.2f, %.2f)",
            msg.pose.position.x,
            msg.pose.position.y,
        )
        with self.lock:
            self.final_goal = msg

        # cancel any in-progress chain and start a fresh one
        self.cancel_requested.set()
        if self.nav_thread is not None and self.nav_thread.is_alive():
            self.nav_thread.join(timeout=2.0)
        self.cancel_requested.clear()

        self.nav_thread = threading.Thread(target=self.navigate_to_final_goal)
        self.nav_thread.daemon = True
        self.nav_thread.start()

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def get_robot_pose(self):
        """Return (x, y) of base_frame in map_frame, or None if unavailable."""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            return (
                trans.transform.translation.x,
                trans.transform.translation.y,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn("long_range_nav: tf lookup failed: %s", str(e))
            return None

    def world_to_grid(self, costmap, x, y):
        ox = costmap.info.origin.position.x
        oy = costmap.info.origin.position.y
        res = costmap.info.resolution
        gx = int((x - ox) / res)
        gy = int((y - oy) / res)
        return gx, gy

    def cell_value(self, costmap, gx, gy):
        w = costmap.info.width
        h = costmap.info.height
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return None  # out of bounds
        idx = gy * w + gx
        return costmap.data[idx]

    def is_free(self, costmap, x, y):
        gx, gy = self.world_to_grid(costmap, x, y)
        val = self.cell_value(costmap, gx, gy)
        if val is None:
            return False  # out of bounds
        if val < 0:
            return False  # unknown (-1)
        if val >= self.occupied_thresh:
            return False  # occupied / inflated
        return True

    def find_farthest_waypoint(self, start_xy, goal_xy, costmap):
        """
        Raycast from start_xy toward goal_xy through the costmap.
        Returns (x, y, reached_goal_directly) where (x, y) is the farthest
        safe point found, and reached_goal_directly is True if the whole
        ray to the goal is free/known (so you can just send the real goal).
        """
        sx, sy = start_xy
        gx, gy = goal_xy
        dx = gx - sx
        dy = gy - sy
        total_dist = math.hypot(dx, dy)

        if total_dist < 1e-3:
            return (gx, gy, True)

        step = self.step_override if self.step_override > 0 else costmap.info.resolution
        step = max(step, costmap.info.resolution)  # never finer than the grid

        ux, uy = dx / total_dist, dy / total_dist

        last_free_dist = 0.0
        n_steps = int(total_dist / step)

        for i in range(1, n_steps + 1):
            d = i * step
            px = sx + ux * d
            py = sy + uy * d
            if self.is_free(costmap, px, py):
                last_free_dist = d
            else:
                break
        else:
            # loop completed without breaking -> entire ray up to n_steps*step is free
            last_free_dist = n_steps * step

        reached_goal_directly = last_free_dist >= (total_dist - step)

        if reached_goal_directly:
            return (gx, gy, True)

        # back off a safety margin from the farthest free point found
        safe_dist = max(last_free_dist - self.safety_margin, 0.0)
        wx = sx + ux * safe_dist
        wy = sy + uy * safe_dist
        return (wx, wy, False)

    def send_move_base_goal(self, x, y, yaw=0.0, frame=None):
        frame = frame or self.map_frame
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw

        rospy.loginfo("long_range_nav: sending waypoint (%.2f, %.2f)", x, y)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return self.client.get_state()

    # ------------------------------------------------------------------
    # main chaining loop
    # ------------------------------------------------------------------
    def navigate_to_final_goal(self):
        with self.lock:
            final = self.final_goal
        if final is None:
            return

        fx = final.pose.position.x
        fy = final.pose.position.y
        # keep the requested final orientation for the LAST hop only
        final_yaw = self.quat_to_yaw(final.pose.orientation)

        consecutive_aborts = 0

        while not rospy.is_shutdown() and not self.cancel_requested.is_set():
            with self.lock:
                costmap = self.costmap

            if costmap is None:
                rospy.logwarn_throttle(5.0, "long_range_nav: no costmap received yet")
                rospy.sleep(0.5)
                continue

            robot_xy = self.get_robot_pose()
            if robot_xy is None:
                rospy.sleep(0.5)
                continue

            dist_to_final = math.hypot(fx - robot_xy[0], fy - robot_xy[1])
            if dist_to_final <= self.goal_tolerance:
                rospy.loginfo("long_range_nav: final goal reached.")
                return

            wx, wy, direct = self.find_farthest_waypoint(
                robot_xy, (fx, fy), costmap
            )

            if direct:
                rospy.loginfo("long_range_nav: final goal is directly reachable, sending it.")
                state = self.send_move_base_goal(fx, fy, final_yaw)
            else:
                hop_dist = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
                if hop_dist < self.min_waypoint_distance:
                    rospy.logwarn(
                        "long_range_nav: farthest reachable point is only %.2fm away "
                        "(< min_waypoint_distance) -- map may not have enough known "
                        "free space in that direction yet. Waiting...",
                        hop_dist,
                    )
                    rospy.sleep(1.0)
                    continue
                # face the direction of travel for intermediate hops
                yaw = math.atan2(wy - robot_xy[1], wx - robot_xy[0])
                state = self.send_move_base_goal(wx, wy, yaw)

            if self.cancel_requested.is_set():
                rospy.loginfo("long_range_nav: navigation cancelled (new goal received).")
                return

            if state == GoalStatus.SUCCEEDED:
                consecutive_aborts = 0
                continue  # loop again: recompute farthest point from new position
            elif state in (GoalStatus.ABORTED, GoalStatus.REJECTED):
                consecutive_aborts += 1
                rospy.logwarn(
                    "long_range_nav: move_base failed (state=%d), attempt %d/%d",
                    state,
                    consecutive_aborts,
                    self.max_consecutive_aborts,
                )
                if consecutive_aborts >= self.max_consecutive_aborts:
                    rospy.logerr(
                        "long_range_nav: giving up after repeated failures reaching intermediate waypoints."
                    )
                    return
                rospy.sleep(1.0)
            else:
                # PREEMPTED, RECALLED, etc.
                rospy.loginfo("long_range_nav: move_base returned state %d, stopping.", state)
                return

    @staticmethod
    def quat_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


if __name__ == "__main__":
    rospy.init_node("long_range_nav")
    # NOTE: if you want RViz's default "2D Nav Goal" button (which publishes
    # straight to /move_base_simple/goal) to feed THIS node instead of going
    # straight into move_base, either:
    #   (a) remap this node's goal topic in your launch file:
    #         <node pkg="your_pkg" type="long_range_nav.py" name="long_range_nav">
    #           <remap from="/move_base_simple/goal" to="/move_base_simple/goal_intercepted"/>
    #           <param name="goal_topic" value="/move_base_simple/goal_intercepted"/>
    #         </node>
    #       and separately remap move_base's own goal subscriber away from
    #       /move_base_simple/goal so the two don't fight over the same topic, OR
    #   (b) simplest: leave move_base subscribed to /move_base_simple/goal as
    #       normal, and set THIS node's ~goal_topic param to a different topic,
    #       then in RViz set the "2D Nav Goal" tool's topic (Panels > Tool
    #       Properties > 2D Nav Goal > Topic) to that same different topic
    #       instead of the default. This cleanly separates "goals for
    #       move_base directly" from "long-range goals for this chainer".
    LongRangeNav()
    rospy.spin()

