#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import tf2_ros

from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray, Marker


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class PRMSmoothFollower(object):

    def __init__(self):

        rospy.init_node("prm_path_follower")

        # ====================================================
        # Topics / frames
        # ====================================================

        self.path_topic = rospy.get_param(
            "~path_topic", "/prm_path"
        )

        self.cmd_topic = rospy.get_param(
            "~cmd_topic", "/cmd_vel"
        )

        self.robot_frame = rospy.get_param(
            "~robot_frame", "base_link"
        )

        self.control_rate = rospy.get_param(
            "~control_rate", 20.0
        )

        # ====================================================
        # Lookahead
        # ====================================================

        # Base lookahead.
        self.lookahead = rospy.get_param(
            "~lookahead", 0.90
        )

        # Look further ahead if robot is roughly aligned.
        self.max_lookahead = rospy.get_param(
            "~max_lookahead", 1.25
        )

        # ====================================================
        # Speeds
        # ====================================================

        self.max_linear = rospy.get_param(
            "~max_linear", 0.35
        )

        self.max_angular = rospy.get_param(
            "~max_angular", 0.50
        )

        # ====================================================
        # Steering
        # ====================================================

        self.heading_gain = rospy.get_param(
            "~heading_gain", 0.75
        )

        self.heading_deadband = math.radians(
            rospy.get_param(
                "~heading_deadband_deg", 4.0
            )
        )

        # ====================================================
        # Rotate-in-place hysteresis
        # ====================================================

        self.rotate_enter_angle = math.radians(
            rospy.get_param(
                "~rotate_enter_angle_deg", 100.0
            )
        )

        self.rotate_exit_angle = math.radians(
            rospy.get_param(
                "~rotate_exit_angle_deg", 55.0
            )
        )

        self.rotate_mode = False

        # ====================================================
        # Steering smoothing
        # ====================================================

        # Lower = smoother.
        self.angular_filter_alpha = rospy.get_param(
            "~angular_filter_alpha", 0.12
        )

        # Maximum angular change each control cycle.
        self.max_angular_step = rospy.get_param(
            "~max_angular_step", 0.035
        )

        self.filtered_angular = 0.0
        self.previous_angular = 0.0

        # ====================================================
        # Progress tracking
        # ====================================================

        self.path_points = []
        self.path_frame = "map"

        self.progress_index = 0

        # Search slightly backwards so slip/noisy localisation
        # doesn't make progress tracking brittle.
        self.search_back = rospy.get_param(
            "~search_back", 3
        )

        self.search_ahead = rospy.get_param(
            "~search_ahead", 50
        )

        # ====================================================
        # Goal
        # ====================================================

        self.goal_tolerance = rospy.get_param(
            "~goal_tolerance", 0.35
        )

        self.last_path_goal = None
        self.last_path_size = 0

        # ====================================================
        # Oscillation detector
        # ====================================================

        self.oscillation_window = rospy.get_param(
            "~oscillation_window", 2.0
        )

        self.oscillation_sign_changes = rospy.get_param(
            "~oscillation_sign_changes", 5
        )

        self.angular_history = []

        # ====================================================
        # Recovery
        # ====================================================

        self.recovery_time = rospy.get_param(
            "~recovery_time", 1.2
        )

        self.recovery_linear = rospy.get_param(
            "~recovery_linear", 0.12
        )

        self.recovery_angular = rospy.get_param(
            "~recovery_angular", 0.16
        )

        self.recovery_until = rospy.Time(0)
        self.recovery_direction = 1.0

        # ====================================================
        # TF / ROS
        # ====================================================

        self.tf_buffer = tf2_ros.Buffer(
            rospy.Duration(10.0)
        )

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer
        )

        self.cmd_pub = rospy.Publisher(
            self.cmd_topic,
            Twist,
            queue_size=1
        )

        self.path_sub = rospy.Subscriber(
            self.path_topic,
            MarkerArray,
            self.path_callback,
            queue_size=1
        )

        rospy.on_shutdown(self.stop)

        rospy.loginfo("======================================")
        rospy.loginfo(" Smooth PRM follower started")
        rospy.loginfo(" path:       %s", self.path_topic)
        rospy.loginfo(" cmd:        %s", self.cmd_topic)
        rospy.loginfo(" frame:      %s", self.robot_frame)
        rospy.loginfo(
            " lookahead:  %.2f -> %.2f m",
            self.lookahead,
            self.max_lookahead
        )
        rospy.loginfo(
            " max linear: %.2f m/s",
            self.max_linear
        )
        rospy.loginfo(
            " max angular: %.2f rad/s",
            self.max_angular
        )
        rospy.loginfo("======================================")

    # ========================================================
    # Path callback
    # ========================================================

    def path_callback(self, msg):

        chosen = None

        for marker in msg.markers:

            if marker.type != Marker.LINE_STRIP:
                continue

            if len(marker.points) < 2:
                continue

            if marker.ns == "path":
                chosen = marker
                break

            if chosen is None:
                chosen = marker

        if chosen is None:
            rospy.logwarn_throttle(
                2.0,
                "No LINE_STRIP in /prm_path"
            )
            return

        new_points = list(chosen.points)

        if len(new_points) < 2:
            return

        if chosen.header.frame_id:
            self.path_frame = chosen.header.frame_id

        new_goal = new_points[-1]

        # Detect significantly new path.
        new_path = False

        if self.last_path_goal is None:
            new_path = True

        else:

            dx = new_goal.x - self.last_path_goal.x
            dy = new_goal.y - self.last_path_goal.y

            goal_change = math.hypot(dx, dy)

            if goal_change > 0.75:
                new_path = True

            if abs(
                len(new_points) -
                self.last_path_size
            ) > 10:
                new_path = True

        self.path_points = new_points

        if new_path:

            self.progress_index = 0
            self.angular_history = []
            self.rotate_mode = False

            rospy.loginfo(
                "New PRM path: %d points",
                len(new_points)
            )

        self.last_path_goal = new_goal
        self.last_path_size = len(new_points)

    # ========================================================
    # Robot pose
    # ========================================================

    def get_robot_pose(self):

        try:

            tf_msg = self.tf_buffer.lookup_transform(
                self.path_frame,
                self.robot_frame,
                rospy.Time(0),
                rospy.Duration(0.10)
            )

            x = tf_msg.transform.translation.x
            y = tf_msg.transform.translation.y

            yaw = quat_to_yaw(
                tf_msg.transform.rotation
            )

            return x, y, yaw

        except Exception as e:

            rospy.logwarn_throttle(
                2.0,
                "TF %s -> %s unavailable: %s",
                self.path_frame,
                self.robot_frame,
                str(e)
            )

            return None

    # ========================================================
    # Find approximate progress along path
    # ========================================================

    def update_progress(self, rx, ry):

        if len(self.path_points) == 0:
            return 0

        start = max(
            0,
            self.progress_index -
            self.search_back
        )

        end = min(
            len(self.path_points),
            self.progress_index +
            self.search_ahead
        )

        best_index = self.progress_index
        best_dist = float("inf")

        for i in range(start, end):

            p = self.path_points[i]

            dx = p.x - rx
            dy = p.y - ry

            d = dx * dx + dy * dy

            if d < best_dist:
                best_dist = d
                best_index = i

        # Don't allow a large backwards jump.
        min_allowed = max(
            0,
            self.progress_index - 2
        )

        best_index = max(
            best_index,
            min_allowed
        )

        # Generally allow progress forward.
        if best_index > self.progress_index:
            self.progress_index = best_index

        return self.progress_index

    # ========================================================
    # Dynamic lookahead
    # ========================================================

    def get_dynamic_lookahead(self, heading_error):

        deg = abs(
            math.degrees(heading_error)
        )

        if deg < 20.0:
            return self.max_lookahead

        elif deg < 45.0:

            ratio = (
                45.0 - deg
            ) / 25.0

            return (
                self.lookahead +
                ratio *
                (
                    self.max_lookahead -
                    self.lookahead
                )
            )

        else:
            return self.lookahead

    # ========================================================
    # Select target ahead on PRM
    # ========================================================

    def get_target_index(
        self,
        start_index,
        lookahead
    ):

        if start_index >= len(self.path_points) - 1:
            return len(self.path_points) - 1

        accumulated = 0.0

        for i in range(
            start_index,
            len(self.path_points) - 1
        ):

            p1 = self.path_points[i]
            p2 = self.path_points[i + 1]

            segment = math.hypot(
                p2.x - p1.x,
                p2.y - p1.y
            )

            accumulated += segment

            if accumulated >= lookahead:
                return i + 1

        return len(self.path_points) - 1

    # ========================================================
    # Angular smoothing
    # ========================================================

    def smooth_angular(self, desired):

        filtered = (
            self.angular_filter_alpha *
            desired +
            (1.0 - self.angular_filter_alpha) *
            self.filtered_angular
        )

        delta = (
            filtered -
            self.previous_angular
        )

        delta = clamp(
            delta,
            -self.max_angular_step,
            self.max_angular_step
        )

        result = (
            self.previous_angular +
            delta
        )

        result = clamp(
            result,
            -self.max_angular,
            self.max_angular
        )

        self.filtered_angular = result
        self.previous_angular = result

        return result

    # ========================================================
    # Oscillation detection
    # ========================================================

    def detect_oscillation(self, angular):

        if abs(angular) < 0.12:
            return False

        now = rospy.Time.now().to_sec()

        sign = 1 if angular > 0 else -1

        self.angular_history.append(
            (now, sign)
        )

        cutoff = (
            now -
            self.oscillation_window
        )

        self.angular_history = [
            item
            for item in self.angular_history
            if item[0] >= cutoff
        ]

        if len(self.angular_history) < 4:
            return False

        changes = 0

        previous = self.angular_history[0][1]

        for _, current in self.angular_history[1:]:

            if current != previous:
                changes += 1

            previous = current

        return (
            changes >=
            self.oscillation_sign_changes
        )

    # ========================================================
    # Recovery
    # ========================================================

    def start_recovery(self, heading_error):

        self.recovery_until = (
            rospy.Time.now() +
            rospy.Duration(self.recovery_time)
        )

        if heading_error > 0:
            self.recovery_direction = 1.0
        elif heading_error < 0:
            self.recovery_direction = -1.0

        self.angular_history = []

        rospy.logwarn(
            "Oscillation detected -> smooth crawl recovery"
        )

    def recovery_active(self):

        return (
            rospy.Time.now() <
            self.recovery_until
        )

    # ========================================================
    # Goal distance
    # ========================================================

    def goal_distance(self, rx, ry):

        goal = self.path_points[-1]

        return math.hypot(
            goal.x - rx,
            goal.y - ry
        )

    # ========================================================
    # Stop
    # ========================================================

    def stop(self):

        try:
            self.cmd_pub.publish(Twist())
        except:
            pass

    # ========================================================
    # Main control
    # ========================================================

    def control(self):

        if len(self.path_points) < 2:

            self.stop()
            return

        robot = self.get_robot_pose()

        if robot is None:

            self.stop()
            return

        rx, ry, yaw = robot

        # ----------------------------------------------------
        # Final goal
        # ----------------------------------------------------

        goal_dist = self.goal_distance(
            rx,
            ry
        )

        if goal_dist < self.goal_tolerance:

            self.stop()

            rospy.loginfo_throttle(
                1.0,
                "PRM goal reached: %.2f m",
                goal_dist
            )

            return

        # ----------------------------------------------------
        # Current progress along path
        # ----------------------------------------------------

        progress = self.update_progress(
            rx,
            ry
        )

        # ----------------------------------------------------
        # First heading estimate
        # ----------------------------------------------------

        rough_target_index = self.get_target_index(
            progress,
            self.lookahead
        )

        rough_target = self.path_points[
            rough_target_index
        ]

        rough_heading = math.atan2(
            rough_target.y - ry,
            rough_target.x - rx
        )

        rough_error = normalize_angle(
            rough_heading - yaw
        )

        # ----------------------------------------------------
        # Adaptive lookahead
        # ----------------------------------------------------

        lookahead = self.get_dynamic_lookahead(
            rough_error
        )

        target_index = self.get_target_index(
            progress,
            lookahead
        )

        target = self.path_points[
            target_index
        ]

        dx = target.x - rx
        dy = target.y - ry

        target_heading = math.atan2(
            dy,
            dx
        )

        heading_error = normalize_angle(
            target_heading - yaw
        )

        # Ignore tiny steering error.
        if abs(heading_error) < self.heading_deadband:
            heading_error = 0.0

        # ----------------------------------------------------
        # Rotate hysteresis
        # ----------------------------------------------------

        if not self.rotate_mode:

            if abs(heading_error) > \
                    self.rotate_enter_angle:

                self.rotate_mode = True

        else:

            if abs(heading_error) < \
                    self.rotate_exit_angle:

                self.rotate_mode = False

        # ----------------------------------------------------
        # Steering
        # ----------------------------------------------------

        desired_angular = (
            self.heading_gain *
            heading_error
        )

        desired_angular = clamp(
            desired_angular,
            -self.max_angular,
            self.max_angular
        )

        angular = self.smooth_angular(
            desired_angular
        )

        # ----------------------------------------------------
        # Linear velocity
        #
        # Important:
        # NO slowdown for intermediate nodes.
        # NO slowdown simply because goal is close.
        #
        # Only larger turns reduce speed.
        # ----------------------------------------------------

        abs_heading_deg = abs(
            math.degrees(heading_error)
        )

        if self.rotate_mode:

            linear = 0.0

        elif abs_heading_deg < 30.0:

            linear = self.max_linear

        elif abs_heading_deg < 60.0:

            linear = (
                self.max_linear *
                0.85
            )

        else:

            linear = (
                self.max_linear *
                0.65
            )

        # ----------------------------------------------------
        # Oscillation recovery
        # ----------------------------------------------------

        if (
            not self.rotate_mode
            and
            self.detect_oscillation(angular)
        ):

            self.start_recovery(
                heading_error
            )

        if self.recovery_active():

            linear = self.recovery_linear

            angular = (
                self.recovery_direction *
                self.recovery_angular
            )

            self.filtered_angular = angular
            self.previous_angular = angular

        # ----------------------------------------------------
        # Publish
        # ----------------------------------------------------

        cmd = Twist()

        cmd.linear.x = linear
        cmd.angular.z = angular

        self.cmd_pub.publish(cmd)

        rospy.loginfo_throttle(
            0.35,
            "PRM | progress=%d target=%d/%d "
            "LA=%.2f "
            "goal=%.2f "
            "heading=%.1fdeg "
            "rotate=%s "
            "recovery=%s "
            "cmd=(%.2f %.2f)",
            progress,
            target_index,
            len(self.path_points) - 1,
            lookahead,
            goal_dist,
            math.degrees(heading_error),
            str(self.rotate_mode),
            str(self.recovery_active()),
            linear,
            angular
        )

    # ========================================================
    # Run
    # ========================================================

    def run(self):

        rate = rospy.Rate(
            self.control_rate
        )

        while not rospy.is_shutdown():

            self.control()

            rate.sleep()


if __name__ == "__main__":

    try:

        controller = PRMSmoothFollower()
        controller.run()

    except rospy.ROSInterruptException:

        pass
