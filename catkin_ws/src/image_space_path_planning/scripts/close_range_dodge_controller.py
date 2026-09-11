#!/usr/bin/env python3
"""
close_range_dodge_controller.py

Simplified version of the PSTeer red-obstacle dodge controller.

Design goal: DO NOTHING until an obstacle is genuinely close and directly
ahead. No multi-frame target tracking, no scoring, no S-shaped 4-phase
manoeuvre. Every frame, it just asks one question:

    "Is there a big red blob, near the very bottom of the ROI, inside a
     narrow forward corridor, right now?"

If yes -> take over cmd_vel, pick left/right based on which half of the
corridor has less red, turn that way briefly while creeping forward, then
hand control back to move_base/global planner.

This is meant for obstacles that traversability mapping / the global PRM
path did NOT already avoid (i.e. things missed by the map, or obstacles
that appeared too late), so it is intentionally a last-resort, very-close
reflex, not a general avoidance layer.
"""

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class CloseRangeDodgeController:
    def __init__(self):
        rospy.init_node("close_range_dodge_controller")

        self.bridge = CvBridge()

        # ==================================================
        # Topics
        # ==================================================
        self.image_topic = rospy.get_param(
            "~image_topic", "/inference_overlay_smoothed"
        )
        self.debug_topic = rospy.get_param(
            "~debug_topic", "/close_range_dodge/debug"
        )
        self.global_cmd_topic = rospy.get_param("~global_cmd_topic", "/cmd_vel_global")
        self.output_cmd_topic = rospy.get_param("~output_cmd_topic", "/cmd_vel/nav")
        self.global_cmd_timeout = float(rospy.get_param("~global_cmd_timeout", 0.5))

        # ==================================================
        # Region of interest (fraction of image, x0/x1/y0/y1)
        # Kept tight and low in the frame on purpose: we only care about
        # what's about to be hit, not the whole scene.
        # ==================================================
        self.roi_x0 = float(rospy.get_param("~roi_x0", 0.20))
        self.roi_x1 = float(rospy.get_param("~roi_x1", 0.80))
        self.roi_y0 = float(rospy.get_param("~roi_y0", 0.55))
        self.roi_y1 = float(rospy.get_param("~roi_y1", 0.95))

        # ==================================================
        # Red detection thresholds (same idea as before, unchanged math)
        # ==================================================
        self.hue_upper1 = int(rospy.get_param("~red_hue_upper1", 10))
        self.hue_lower2 = int(rospy.get_param("~red_hue_lower2", 170))
        self.sat_min = int(rospy.get_param("~red_sat_min", 60))
        self.val_min = int(rospy.get_param("~red_val_min", 35))
        self.red_margin = int(rospy.get_param("~red_margin", 25))
        self.kernel_size = int(rospy.get_param("~kernel_size", 5))

        # ==================================================
        # "Really close" trigger conditions -- deliberately strict.
        # A blob must satisfy ALL of these to trigger a dodge:
        #   1. area          >= min_trigger_area        (big blob)
        #   2. bbox bottom   >= min_trigger_bottom * ROI height (low in ROI)
        #   3. centroid x    inside the narrow forward corridor
        # ==================================================
        self.min_trigger_area = int(rospy.get_param("~min_trigger_area", 3500))
        self.min_trigger_bottom = float(rospy.get_param("~min_trigger_bottom", 0.80))
        self.corridor_width = float(rospy.get_param("~corridor_width", 0.35))

        # Require the trigger condition to hold for a few consecutive frames
        # before actually taking over, to reject single-frame noise. Kept
        # small on purpose -- this is a close-range reflex, not something
        # that should hesitate once it's real.
        self.confirm_frames = int(rospy.get_param("~confirm_frames", 2))
        self._confirm_count = 0

        # ==================================================
        # Dodge motion: stop -> pivot away in place -> drive straight at
        # the offset heading (this is what actually clears the object) ->
        # pivot back in place -> cooldown -> global.
        #
        # The turn phases are IN-PLACE pivots (linear.x = 0), not arcs, so
        # heading offset doesn't depend on forward speed. The straight-line
        # phase is sized from real geometry so it guarantees clearance
        # around the obstacle instead of relying on a guessed duration.
        # ==================================================
        self.stop_hold_seconds = float(rospy.get_param("~stop_hold_seconds", 0.30))
        self.cooldown_seconds = float(rospy.get_param("~cooldown_seconds", 1.0))

        self.dodge_angular_speed = float(rospy.get_param("~dodge_angular_speed", 0.60))
        self.pass_straight_speed = float(rospy.get_param("~pass_straight_speed", 0.15))

        # Heading offset used while passing the obstacle. Larger = more
        # sideways clearance per metre driven, but a slower net advance.
        self.dodge_turn_angle_deg = float(rospy.get_param("~dodge_turn_angle_deg", 80.0))

        # Real-world geometry used to size the manoeuvre. Defaults assume a
        # ~0.5 m diameter trunk; override ~obstacle_radius_m per obstacle
        # type, or bump ~clearance_margin_m for extra safety buffer.
        self.obstacle_radius_m = float(rospy.get_param("~obstacle_radius_m", 0.25))
        self.robot_half_width_m = float(rospy.get_param("~robot_half_width_m", 0.30))
        self.clearance_margin_m = float(rospy.get_param("~clearance_margin_m", 0.15))

        required_clearance_m = (
            self.obstacle_radius_m + self.robot_half_width_m + self.clearance_margin_m
        )
        angle_rad = np.deg2rad(max(5.0, min(85.0, self.dodge_turn_angle_deg)))

        # Distance to travel at the offset heading so that the *sideways*
        # component of that travel meets the required clearance:
        #   lateral = distance * sin(angle)  =>  distance = clearance / sin(angle)
        pass_distance_m = required_clearance_m / max(1e-3, np.sin(angle_rad))

        self.turn_away_seconds = angle_rad / max(1e-3, self.dodge_angular_speed)
        self.turn_back_seconds = self.turn_away_seconds
        self.pass_straight_seconds = pass_distance_m / max(1e-3, self.pass_straight_speed)

        rospy.loginfo(
            "[close_range_dodge] required_clearance=%.2fm angle=%.0fdeg "
            "-> pass_distance=%.2fm turn_away=%.2fs pass=%.2fs turn_back=%.2fs",
            required_clearance_m,
            self.dodge_turn_angle_deg,
            pass_distance_m,
            self.turn_away_seconds,
            self.pass_straight_seconds,
            self.turn_back_seconds,
        )

        # Don't retrigger on the same still-visible obstacle immediately
        # after cooldown -- wait until the corridor is clear again.
        self.rearm_clear_frames = int(rospy.get_param("~rearm_clear_frames", 8))
        self._rearm_clear_count = 0
        self._armed = True

        # GLOBAL -> STOP_HOLD -> TURN_AWAY -> PASS_STRAIGHT -> TURN_BACK -> COOLDOWN -> GLOBAL
        self._state = "GLOBAL"
        self._state_started = rospy.Time.now()
        self._turn_direction = None     # "LEFT" or "RIGHT"

        self._latest_global_cmd = Twist()
        self._latest_global_cmd_time = rospy.Time(0)

        # ==================================================
        # Debug drawing (optional, cheap)
        # ==================================================
        self.draw_debug = self._get_bool("~draw_debug", True)

        # ==================================================
        # ROS I/O
        # ==================================================
        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.output_cmd_topic, Twist, queue_size=1)

        rospy.Subscriber(
            self.global_cmd_topic, Twist, self._global_cmd_cb, queue_size=1
        )
        rospy.Subscriber(
            self.image_topic, Image, self._image_cb, queue_size=1, buff_size=2 ** 24
        )

        control_rate = float(rospy.get_param("~control_rate", 20.0))
        rospy.Timer(rospy.Duration(1.0 / max(1.0, control_rate)), self._control_timer_cb)

        rospy.loginfo(
            "[close_range_dodge] trigger: area>=%d bottom>=%.2f corridor=%.2f confirm=%d",
            self.min_trigger_area,
            self.min_trigger_bottom,
            self.corridor_width,
            self.confirm_frames,
        )
        rospy.loginfo(
            "[close_range_dodge] global cmd: %s -> output cmd: %s",
            self.global_cmd_topic,
            self.output_cmd_topic,
        )

    @staticmethod
    def _get_bool(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("true", "1", "yes", "on")
        return bool(value)

    # ------------------------------------------------------------------
    # Detection -- one shot, no cross-frame tracking
    # ------------------------------------------------------------------
    def _red_mask(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

        hue_mask = (h <= self.hue_upper1) | (h >= self.hue_lower2)
        sat_val_mask = (s >= self.sat_min) & (v >= self.val_min)

        b = roi[:, :, 0].astype(np.int16)
        g = roi[:, :, 1].astype(np.int16)
        r = roi[:, :, 2].astype(np.int16)
        excess_mask = ((r - g) >= self.red_margin) & ((r - b) >= self.red_margin)

        mask = (hue_mask & sat_val_mask & excess_mask).astype(np.uint8) * 255

        k = max(1, self.kernel_size)
        if k % 2 == 0:
            k += 1
        if k > 1:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        return mask

    def _find_close_blob(self, mask, roi_height, roi_width):
        """
        Return the largest blob that is both inside the forward corridor and
        close enough (low enough) to count as "really close". Returns None
        if nothing qualifies -- this is the whole point of the node.
        """
        corridor_w = np.clip(self.corridor_width, 0.05, 1.0) * roi_width
        corridor_left = (roi_width - corridor_w) / 2.0
        corridor_right = corridor_left + corridor_w
        min_bottom = np.clip(self.min_trigger_bottom, 0.0, 1.0) * roi_height

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8
        )

        best = None
        for i in range(1, num_labels):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < self.min_trigger_area:
                continue

            x = int(stats[i, cv2.CC_STAT_LEFT])
            y = int(stats[i, cv2.CC_STAT_TOP])
            w = int(stats[i, cv2.CC_STAT_WIDTH])
            h = int(stats[i, cv2.CC_STAT_HEIGHT])
            cx = float(centroids[i][0])

            if not (corridor_left <= cx <= corridor_right):
                continue
            if (y + h) < min_bottom:
                continue

            if best is None or area > best["area"]:
                best = {
                    "label_index": i,
                    "area": area,
                    "bbox": (x, y, w, h),
                    "centroid": (cx, float(centroids[i][1])),
                }

        return labels, best

    @staticmethod
    def _choose_side(mask, roi_width):
        """Steer away from whichever half of the ROI has more red pixels."""
        mid = roi_width // 2
        left_count = int(np.count_nonzero(mask[:, :mid] > 0))
        right_count = int(np.count_nonzero(mask[:, mid:] > 0))
        # Turn toward the clearer side.
        return "RIGHT" if left_count > right_count else "LEFT"

    # ------------------------------------------------------------------
    # cmd_vel passthrough / arbitration
    # ------------------------------------------------------------------
    def _global_cmd_cb(self, msg):
        self._latest_global_cmd = msg
        self._latest_global_cmd_time = rospy.Time.now()

    def _fresh_global_cmd(self):
        age = (rospy.Time.now() - self._latest_global_cmd_time).to_sec()
        if self._latest_global_cmd_time == rospy.Time(0) or age > self.global_cmd_timeout:
            return Twist()
        return self._latest_global_cmd

    def _start_dodge(self, direction):
        if self._state != "GLOBAL":
            return
        self._turn_direction = direction
        self._state = "STOP_HOLD"
        self._state_started = rospy.Time.now()
        rospy.logwarn("[close_range_dodge] CLOSE OBSTACLE -> dodging %s", direction)

    def _control_timer_cb(self, _event):
        now = rospy.Time.now()
        elapsed = (now - self._state_started).to_sec()
        cmd = Twist()

        turn_sign = 1.0 if self._turn_direction == "LEFT" else -1.0

        if self._state == "GLOBAL":
            cmd = self._fresh_global_cmd()

        elif self._state == "STOP_HOLD":
            cmd = Twist()
            if elapsed >= self.stop_hold_seconds:
                self._state = "TURN_AWAY"
                self._state_started = now

        elif self._state == "TURN_AWAY":
            # Pure in-place pivot -- no forward motion. This decouples
            # heading offset from creep speed so the turn is genuinely
            # sharp instead of a wide arc.
            cmd.linear.x = 0.0
            cmd.angular.z = turn_sign * abs(self.dodge_angular_speed)
            if elapsed >= self.turn_away_seconds:
                self._state = "PASS_STRAIGHT"
                self._state_started = now

        elif self._state == "PASS_STRAIGHT":
            # Drive straight at the offset heading for a distance sized to
            # actually clear the obstacle (see required_clearance_m calc).
            cmd.linear.x = max(0.0, self.pass_straight_speed)
            cmd.angular.z = 0.0
            if elapsed >= self.pass_straight_seconds:
                self._state = "TURN_BACK"
                self._state_started = now

        elif self._state == "TURN_BACK":
            # Opposite in-place pivot, same duration, to restore heading.
            cmd.linear.x = 0.0
            cmd.angular.z = -turn_sign * abs(self.dodge_angular_speed)
            if elapsed >= self.turn_back_seconds:
                self._state = "COOLDOWN"
                self._state_started = now
                cmd = Twist()

        elif self._state == "COOLDOWN":
            # Hand steering back to the global planner immediately, but keep
            # the trigger disarmed until the corridor is clear again so we
            # don't instantly retrigger on the same obstacle.
            cmd = self._fresh_global_cmd()
            if elapsed >= self.cooldown_seconds:
                self._state = "GLOBAL"
                self._state_started = now
                self._turn_direction = None

        else:
            rospy.logerr("[close_range_dodge] unknown state: %s", self._state)
            self._state = "GLOBAL"
            self._state_started = now

        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------
    # Image callback -- the only place detection happens
    # ------------------------------------------------------------------
    def _image_cb(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logwarn_throttle(2.0, "cv_bridge failed: %s", str(e))
            return

        h_img, w_img = image.shape[:2]
        x0 = int(np.clip(self.roi_x0, 0.0, 1.0) * w_img)
        x1 = int(np.clip(self.roi_x1, 0.0, 1.0) * w_img)
        y0 = int(np.clip(self.roi_y0, 0.0, 1.0) * h_img)
        y1 = int(np.clip(self.roi_y1, 0.0, 1.0) * h_img)
        if x1 <= x0 or y1 <= y0:
            rospy.logwarn_throttle(2.0, "Invalid ROI dimensions")
            return

        roi = image[y0:y1, x0:x1]
        roi_h, roi_w = roi.shape[:2]

        mask = self._red_mask(roi)
        labels, close_blob = self._find_close_blob(mask, roi_h, roi_w)

        # Rearm bookkeeping: only allow a new dodge once the corridor has
        # been clear (no qualifying close blob) for several frames.
        if close_blob is None:
            self._rearm_clear_count += 1
            if self._rearm_clear_count >= self.rearm_clear_frames:
                self._armed = True
        else:
            self._rearm_clear_count = 0

        # Confirm over a couple of frames to reject one-off noise.
        if close_blob is not None:
            self._confirm_count += 1
        else:
            self._confirm_count = 0

        if (
            self._armed
            and self._state == "GLOBAL"
            and close_blob is not None
            and self._confirm_count >= self.confirm_frames
        ):
            direction = self._choose_side(mask, roi_w)
            self._armed = False
            self._start_dodge(direction)

        if self.draw_debug:
            self._publish_debug(image, mask, close_blob, x0, x1, y0, y1, msg.header)

    # ------------------------------------------------------------------
    # Debug visualization
    # ------------------------------------------------------------------
    def _publish_debug(self, image, mask, close_blob, x0, x1, y0, y1, header):
        debug = image.copy()
        cv2.rectangle(debug, (x0, y0), (x1, y1), (255, 255, 255), 2)

        overlay = debug.copy()
        overlay_roi = overlay[y0:y1, x0:x1]
        overlay_roi[mask > 0] = (0, 0, 255)
        overlay[y0:y1, x0:x1] = overlay_roi
        debug = cv2.addWeighted(debug, 0.8, overlay, 0.2, 0)

        if close_blob is not None:
            bx, by, bw, bh = close_blob["bbox"]
            cv2.rectangle(
                debug, (x0 + bx, y0 + by), (x0 + bx + bw, y0 + by + bh), (0, 255, 255), 3
            )
            status = "CLOSE OBSTACLE (confirm {}/{})".format(
                self._confirm_count, self.confirm_frames
            )
            color = (0, 0, 255)
        else:
            status = "clear"
            color = (0, 255, 0)

        cv2.putText(
            debug,
            "state={}  {}".format(self._state, status),
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_msg.header = header
            self.debug_pub.publish(debug_msg)
        except CvBridgeError as e:
            rospy.logwarn_throttle(2.0, "cv_bridge debug publish failed: %s", str(e))


if __name__ == "__main__":
    try:
        CloseRangeDodgeController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
