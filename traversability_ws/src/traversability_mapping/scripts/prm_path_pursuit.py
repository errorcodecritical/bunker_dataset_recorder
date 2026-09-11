#!/usr/bin/env python
"""
prm_path_pursuit.py

Adaptive pure-pursuit path follower for the Bunker UGV, driven directly by
the PRM planner's path (published as a visualization_msgs/Marker LINE_STRIP
or POINTS, not a nav_msgs/Path). Converts the marker to an internal
arc-length-parameterized polyline, tracks progress with a monotonic,
forward-only closest-point projection (so it never snaps backward to an
earlier node -> no oscillation), interpolates a lookahead point *between*
PRM nodes rather than jumping node-to-node, and rate-limits cmd_vel so
speed/steering changes are smooth.

Assumes a differential/skid-steer drive (Bunker), publishing
geometry_msgs/Twist on /cmd_vel. Change the publish type if your
bunker_base / relay node expects TwistStamped.
"""

import math
import bisect
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion


class PathState(object):
    """Holds the current PRM path as an arc-length parameterized polyline."""

    def __init__(self):
        self.frame_id = None
        self.xs = []
        self.ys = []
        self.s = []          # cumulative arc length at each point
        self.total_length = 0.0

    def update_from_marker(self, marker):
        pts = marker.points
        if len(pts) < 2:
            return False

        xs = [p.x for p in pts]
        ys = [p.y for p in pts]
        s = [0.0]
        for i in range(1, len(xs)):
            d = math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
            s.append(s[-1] + d)

        self.frame_id = marker.header.frame_id
        self.xs, self.ys, self.s = xs, ys, s
        self.total_length = s[-1]
        return True

    def empty(self):
        return len(self.xs) < 2

    def point_at_s(self, s_query):
        """Linearly interpolate an (x, y) point at arc length s_query."""
        s_query = max(0.0, min(s_query, self.total_length))
        i = bisect.bisect_right(self.s, s_query) - 1
        i = max(0, min(i, len(self.s) - 2))
        seg_len = self.s[i + 1] - self.s[i]
        t = 0.0 if seg_len < 1e-9 else (s_query - self.s[i]) / seg_len
        x = self.xs[i] + t * (self.xs[i + 1] - self.xs[i])
        y = self.ys[i] + t * (self.ys[i + 1] - self.ys[i])
        return x, y

    def closest_s(self, x, y, s_lo, s_hi, samples=60):
        """
        Find the arc length of the closest point on the path to (x, y),
        searched only within [s_lo, s_hi]. Coarse sample + local refine.

        Restricting the search window is what prevents the follower from
        jumping backward to an earlier node -- progress is monotonic.
        """
        s_lo = max(0.0, s_lo)
        s_hi = min(self.total_length, s_hi)
        if s_hi <= s_lo:
            s_hi = min(self.total_length, s_lo + 1.0)

        best_s, best_d = s_lo, float('inf')
        n = max(2, samples)
        for k in range(n + 1):
            s_try = s_lo + (s_hi - s_lo) * k / n
            px, py = self.point_at_s(s_try)
            d = math.hypot(px - x, py - y)
            if d < best_d:
                best_d, best_s = d, s_try

        # local refine (ternary-search style bisection around best_s)
        step = (s_hi - s_lo) / n
        lo, hi = max(s_lo, best_s - step), min(s_hi, best_s + step)
        for _ in range(12):
            mid1 = lo + (hi - lo) / 3.0
            mid2 = hi - (hi - lo) / 3.0
            x1, y1 = self.point_at_s(mid1)
            x2, y2 = self.point_at_s(mid2)
            d1 = math.hypot(x1 - x, y1 - y)
            d2 = math.hypot(x2 - x, y2 - y)
            if d1 < d2:
                hi = mid2
            else:
                lo = mid1
        return (lo + hi) / 2.0, best_d


class PRMPathPursuit(object):
    def __init__(self):
        # -- topics / frames --
        self.marker_topic = rospy.get_param("~prm_marker_topic", "/prm_path")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.control_rate = rospy.get_param("~control_rate", 20.0)

        # -- speed / lookahead --
        self.v_max = rospy.get_param("~v_max", 0.5)
        self.v_min_turn = rospy.get_param("~v_min_turn", 0.12)
        self.ld_min = rospy.get_param("~lookahead_min", 0.5)
        self.ld_max = rospy.get_param("~lookahead_max", 1.5)
        self.ld_gain = rospy.get_param("~lookahead_vel_gain", 1.0)  # Ld = ld_min + gain*|v|

        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.3)
        self.max_lin_accel = rospy.get_param("~max_lin_accel", 0.4)   # m/s^2
        self.max_ang_accel = rospy.get_param("~max_ang_accel", 1.2)   # rad/s^2
        self.max_ang_vel = rospy.get_param("~max_ang_vel", 1.0)       # rad/s

        # -- heading-error handling --
        self.rotate_in_place_thresh = rospy.get_param(
            "~rotate_in_place_thresh_deg", 70.0) * math.pi / 180.0
        self.rotate_in_place_gain = rospy.get_param("~rotate_in_place_gain", 1.0)

        # how far ahead/behind (arc length) to search for the closest point
        # each cycle. search_back is small slack for sensor/localization
        # noise; progress never moves backward beyond that.
        self.search_back = rospy.get_param("~search_back", 0.3)
        self.search_forward = rospy.get_param("~search_forward", 4.0)

        self.lock = threading.Lock()
        self.path = PathState()
        self.s_progress = 0.0
        self.have_path = False

        self.last_v = 0.0
        self.last_w = 0.0
        self.last_time = rospy.Time.now()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.target_pub = rospy.Publisher("~lookahead_point", PointStamped, queue_size=1)

        rospy.Subscriber(self.marker_topic, MarkerArray, self.marker_array_cb, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_cb)
        rospy.loginfo("prm_path_pursuit: waiting for PRM path on %s", self.marker_topic)

    # ------------------------------------------------------------------ #
    def marker_array_cb(self, marker_array):
        """
        /prm_path is published as visualization_msgs/MarkerArray.

        Select the path-like marker from the array. Prefer LINE_STRIP because
        that represents an ordered polyline. Fall back to POINTS if needed.
        Ignore graph/node markers that do not contain a usable path.
        """
        path_marker = None

        # Prefer a LINE_STRIP with at least two points.
        for marker in marker_array.markers:
            if marker.type == Marker.LINE_STRIP and len(marker.points) >= 2:
                path_marker = marker
                break

        # Some PRM visualizers may publish the ordered path as POINTS instead.
        if path_marker is None:
            for marker in marker_array.markers:
                if marker.type == Marker.POINTS and len(marker.points) >= 2:
                    path_marker = marker
                    break

        if path_marker is None:
            rospy.logwarn_throttle(
                2.0,
                "prm_path_pursuit: received MarkerArray on %s but found no "
                "LINE_STRIP/POINTS marker with >=2 points",
                self.marker_topic)
            return

        self.marker_cb(path_marker)

    # ------------------------------------------------------------------ #
    def marker_cb(self, marker):
        """Update the internal polyline from the selected path marker."""
        with self.lock:
            old_total = self.path.total_length
            ok = self.path.update_from_marker(marker)
            if not ok:
                return

            if not self.have_path:
                # brand new path: start progress at 0, first control tick
                # will immediately pull it forward via closest_s
                self.s_progress = 0.0
            else:
                # PRM graph grew / replanned: re-anchor progress by carrying
                # the same *fraction* of the path forward onto the new
                # geometry, instead of resetting to 0 (which would look like
                # the robot "restarting" -> visible jerk) or trusting a
                # stale absolute arc length on a changed path.
                frac = 0.0 if old_total < 1e-6 else min(
                    1.0, self.s_progress / old_total)
                self.s_progress = frac * self.path.total_length

            self.have_path = True

    # ------------------------------------------------------------------ #
    def get_pose(self):
        try:
            tf_st = self.tf_buffer.lookup_transform(
                self.path.frame_id or self.map_frame, self.base_frame,
                rospy.Time(0), rospy.Duration(0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, "prm_path_pursuit: TF lookup failed: %s", e)
            return None

        x = tf_st.transform.translation.x
        y = tf_st.transform.translation.y
        q = tf_st.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return x, y, yaw

    # ------------------------------------------------------------------ #
    def control_cb(self, event):
        with self.lock:
            if not self.have_path or self.path.empty():
                return
            path = self.path
            s_progress = self.s_progress

        pose = self.get_pose()
        if pose is None:
            self.publish_cmd(0.0, 0.0)  # don't dead-reckon blind
            return
        x, y, yaw = pose

        # 1) Re-project onto the path within a forward-biased window only.
        #    This is the anti-oscillation step: progress can't jump backward
        #    to an earlier node, so a momentary deviation (e.g. the dodge
        #    controller nudging us around a trunk) can't cause the follower
        #    to re-lock behind itself and reverse direction.
        s_lo = s_progress - self.search_back
        s_hi = s_progress + self.search_forward
        s_closest, dist_to_path = path.closest_s(x, y, s_lo, s_hi)

        # if we've drifted further off-path than the window covers, widen
        # the search once (forward only) instead of forcing a bad lock
        if dist_to_path > self.search_forward and s_hi < path.total_length:
            s_closest, dist_to_path = path.closest_s(x, y, s_progress, path.total_length)

        s_progress = max(s_progress, s_closest)  # monotonic (with search_back slack)

        # 2) goal check
        remaining = path.total_length - s_progress
        if remaining < self.goal_tolerance:
            self.publish_cmd(0.0, 0.0)
            with self.lock:
                self.s_progress = s_progress
            return

        # 3) adaptive lookahead + interpolated target point (sub-node
        #    resolution -> smooth arcs instead of snapping node-to-node)
        ld = min(self.ld_max, max(self.ld_min, self.ld_min + self.ld_gain * abs(self.last_v)))
        s_target = min(path.total_length, s_progress + ld)
        tx, ty = path.point_at_s(s_target)

        self.publish_target_marker(tx, ty, path.frame_id)

        # 4) transform target into robot frame, compute pure-pursuit curvature
        dx = tx - x
        dy = ty - y
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        alpha = math.atan2(local_y, local_x)

        if abs(alpha) > self.rotate_in_place_thresh:
            # target is basically behind/beside us: rotate in place rather
            # than tracing a huge, slow arc
            v_cmd = 0.0
            w_cmd = self.rotate_in_place_gain * (1.0 if alpha > 0 else -1.0)
        else:
            ld_eff = max(1e-3, math.hypot(local_x, local_y))
            curvature = 2.0 * local_y / (ld_eff * ld_eff)

            # slow down smoothly as heading error grows, rather than hard
            # switching between "drive" and "rotate" modes
            heading_scale = max(0.15, 1.0 - abs(alpha) / self.rotate_in_place_thresh)

            v_cmd = self.v_max * heading_scale
            if abs(curvature) > 1e-3:
                v_cmd = max(v_cmd, self.v_min_turn)
            w_cmd = curvature * v_cmd
            w_cmd = max(-self.max_ang_vel, min(self.max_ang_vel, w_cmd))

        self.publish_cmd(v_cmd, w_cmd)

        with self.lock:
            self.s_progress = s_progress

    # ------------------------------------------------------------------ #
    def publish_cmd(self, v_target, w_target):
        now = rospy.Time.now()
        dt = max(1e-3, (now - self.last_time).to_sec())
        self.last_time = now

        dv_max = self.max_lin_accel * dt
        dw_max = self.max_ang_accel * dt

        v = self.last_v + max(-dv_max, min(dv_max, v_target - self.last_v))
        w = self.last_w + max(-dw_max, min(dw_max, w_target - self.last_w))

        self.last_v, self.last_w = v, w

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def publish_target_marker(self, x, y, frame_id):
        pt = PointStamped()
        pt.header.frame_id = frame_id or self.map_frame
        pt.header.stamp = rospy.Time.now()
        pt.point.x = x
        pt.point.y = y
        self.target_pub.publish(pt)


if __name__ == "__main__":
    rospy.init_node("prm_path_pursuit")
    PRMPathPursuit()
    rospy.spin()
