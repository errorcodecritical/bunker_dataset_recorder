#!/usr/bin/env python3
"""
red_obstacle_overwatch_preview.py

Detects red obstacle regions in a PSTeer overlay, but publishes and draws
ONLY ONE obstacle at a time:

  1. When no target is active, choose the closest obstacle only when it is
     inside the forward/centre corridor and low enough in the ROI.
  2. Lock onto that obstacle and associate it across subsequent frames.
  3. Ignore all other obstacles while the target is locked.
  4. Release the target only after it has been missing for several frames,
     which normally means it has been passed or dodged.

The sector danger output is computed only from the currently observed target,
not from every detected red region.
"""

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge, CvBridgeError


class RedObstaclePerception:
    def __init__(self):
        rospy.init_node("red_obstacle_perception")

        self.bridge = CvBridge()

        # ==================================================
        # Topics
        # ==================================================
        self.image_topic = rospy.get_param(
            "~image_topic", "/inference_overlay_smoothed"
        )
        self.debug_topic = rospy.get_param(
            "~debug_topic", "/red_obstacles/bounding_boxes"
        )
        self.sectors_topic = rospy.get_param(
            "~sectors_topic", "/red_obstacles/sectors"
        )

        # ==================================================
        # Region of interest (fraction of image, x0/x1/y0/y1)
        # ==================================================
        self.roi_x0 = float(rospy.get_param("~roi_x0", 0.15))
        self.roi_x1 = float(rospy.get_param("~roi_x1", 0.85))
        self.roi_y0 = float(rospy.get_param("~roi_y0", 0.42))
        self.roi_y1 = float(rospy.get_param("~roi_y1", 0.88))

        # ==================================================
        # Red detection thresholds
        # ==================================================
        self.hue_upper1 = int(rospy.get_param("~red_hue_upper1", 10))
        self.hue_lower2 = int(rospy.get_param("~red_hue_lower2", 170))
        self.sat_min = int(rospy.get_param("~red_sat_min", 60))
        self.val_min = int(rospy.get_param("~red_val_min", 35))
        self.red_margin = int(rospy.get_param("~red_margin", 25))

        # ==================================================
        # Temporal smoothing (mask-level EMA)
        # ==================================================
        self.temporal_alpha = float(rospy.get_param("~temporal_alpha", 0.5))
        self.temporal_threshold = float(
            rospy.get_param("~temporal_threshold", 0.5)
        )
        self.ema_mask = None

        # ==================================================
        # Blob filtering
        # ==================================================
        self.min_blob_area = int(rospy.get_param("~min_blob_area", 400))
        self.max_blob_area = int(rospy.get_param("~max_blob_area", 1000000))
        self.min_width = int(rospy.get_param("~min_width", 8))
        self.min_height = int(rospy.get_param("~min_height", 8))
        self.min_fill_ratio = float(rospy.get_param("~min_fill_ratio", 0.20))
        self.kernel_size = int(rospy.get_param("~kernel_size", 5))

        # ==================================================
        # Single-target acquisition and tracking
        # ==================================================
        # Width of the forward corridor as a fraction of the ROI width.
        # A new target can only be acquired inside this corridor. Once locked,
        # it may move outside the corridor while the robot drives around it.
        self.target_center_width = float(
            rospy.get_param("~target_center_width", 0.50)
        )

        # A new target must extend below this fraction of the ROI height.
        # This rejects small/far detections near the top of the ROI. For
        # example, 0.60 means the bounding-box bottom must reach the lower
        # 40 percent of the ROI before avoidance starts.
        self.target_min_bottom = float(
            rospy.get_param("~target_min_bottom", 0.60)
        )

        # Draw the horizontal close-enough acquisition line in the debug image.
        self.draw_acquisition_line = self.get_bool_param(
            "~draw_acquisition_line", True
        )

        # Initial target score. Lower in the image is treated as closer.
        self.target_proximity_weight = float(
            rospy.get_param("~target_proximity_weight", 0.65)
        )
        self.target_center_weight = float(
            rospy.get_param("~target_center_weight", 0.25)
        )
        self.target_area_weight = float(
            rospy.get_param("~target_area_weight", 0.10)
        )

        # Association thresholds used after a target has been acquired.
        # A current blob matches the target when it has enough overlap OR its
        # centroid remains sufficiently close to the previous centroid.
        self.target_min_iou = float(rospy.get_param("~target_min_iou", 0.05))
        self.target_max_distance = float(
            rospy.get_param("~target_max_distance", 0.30)
        )
        self.target_max_missed_frames = int(
            rospy.get_param("~target_max_missed_frames", 10)
        )

        self.tracked_blob = None
        self.target_missed_frames = 0
        self.target_id = 0

        # ==================================================
        # Sector danger output
        # ==================================================
        self.num_sectors = int(rospy.get_param("~num_sectors", 7))
        self.danger_gain = float(rospy.get_param("~danger_gain", 3.0))

        # ==================================================
        # Controller preview only (NO velocity commands published)
        # ==================================================
        self.preview_stop_danger = float(
            rospy.get_param("~preview_stop_danger", 0.40)
        )
        self.preview_stop_bottom = float(
            rospy.get_param("~preview_stop_bottom", 0.80)
        )
        self.preview_min_strong_sectors = int(
            rospy.get_param("~preview_min_strong_sectors", 2)
        )
        self.preview_confirm_frames = int(
            rospy.get_param("~preview_confirm_frames", 3)
        )
        self.preview_release_frames = int(
            rospy.get_param("~preview_release_frames", 5)
        )
        self.preview_state = "IDLE"
        self.preview_turn_direction = None
        self.preview_trigger_count = 0
        self.preview_clear_count = 0

        # ==================================================
        # Drawing settings
        # ==================================================
        self.draw_mask_overlay = self.get_bool_param("~draw_mask_overlay", True)
        self.draw_centroid = self.get_bool_param("~draw_centroid", True)
        self.draw_roi = self.get_bool_param("~draw_roi", True)
        self.draw_sector_bars = self.get_bool_param("~draw_sector_bars", True)
        self.draw_forward_corridor = self.get_bool_param(
            "~draw_forward_corridor", True
        )

        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        self.sectors_pub = rospy.Publisher(
            self.sectors_topic, Float32MultiArray, queue_size=1
        )

        rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )

        rospy.loginfo("[red_obstacle_perception] overwatch preview running")
        rospy.loginfo("Input:   %s", self.image_topic)
        rospy.loginfo("Debug:   %s", self.debug_topic)
        rospy.loginfo("Sectors: %s (%d sectors)", self.sectors_topic, self.num_sectors)
        rospy.loginfo(
            "Target: centre_width=%.2f min_bottom=%.2f max_missed=%d",
            self.target_center_width,
            self.target_min_bottom,
            self.target_max_missed_frames,
        )

    @staticmethod
    def get_bool_param(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("true", "1", "yes", "on")
        return bool(value)

    # ------------------------------------------------------------------
    # Detection
    # ------------------------------------------------------------------
    def detect_red(self, roi):
        """Create a robust red mask using HSV and excess-red checks."""
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]

        hue_mask = (h <= self.hue_upper1) | (h >= self.hue_lower2)
        sat_val_mask = (s >= self.sat_min) & (v >= self.val_min)

        b = roi[:, :, 0].astype(np.int16)
        g = roi[:, :, 1].astype(np.int16)
        r = roi[:, :, 2].astype(np.int16)
        excess_mask = ((r - g) >= self.red_margin) & ((r - b) >= self.red_margin)

        mask = hue_mask & sat_val_mask & excess_mask
        return mask.astype(np.uint8) * 255

    def apply_temporal_smoothing(self, raw_mask):
        """Apply a per-pixel EMA to reduce mask flicker."""
        mask_f = raw_mask.astype(np.float32) / 255.0

        if self.ema_mask is None or self.ema_mask.shape != mask_f.shape:
            self.ema_mask = mask_f
        else:
            alpha = self.temporal_alpha
            self.ema_mask = alpha * mask_f + (1.0 - alpha) * self.ema_mask

        return (self.ema_mask > self.temporal_threshold).astype(np.uint8) * 255

    def clean_mask(self, mask):
        kernel_size = max(1, self.kernel_size)
        if kernel_size % 2 == 0:
            kernel_size += 1
        if kernel_size <= 1:
            return mask

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def apply_close_central_gate(self, mask):
        """
        Keep only pixels that are both below the acquisition line and inside
        the central corridor. This prevents a red horizon band from becoming
        one very wide bounding box.
        """
        roi_height, roi_width = mask.shape[:2]
        gated = np.zeros_like(mask)

        corridor_width = int(
            np.clip(self.target_center_width, 0.05, 1.0) * roi_width
        )
        corridor_left = max(0, (roi_width - corridor_width) // 2)
        corridor_right = min(roi_width, corridor_left + corridor_width)
        acquisition_y = int(
            np.clip(self.target_min_bottom, 0.0, 1.0) * roi_height
        )

        gated[acquisition_y:roi_height, corridor_left:corridor_right] = (
            mask[acquisition_y:roi_height, corridor_left:corridor_right]
        )
        return gated

    def find_obstacle_blobs(self, mask):
        """
        Find all valid connected components internally.

        All components are needed only so the tracker can choose or associate
        its single target. They are not all published or drawn.
        """
        number_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8
        )

        blobs = []

        for label_index in range(1, number_labels):
            area = int(stats[label_index, cv2.CC_STAT_AREA])
            x = int(stats[label_index, cv2.CC_STAT_LEFT])
            y = int(stats[label_index, cv2.CC_STAT_TOP])
            width = int(stats[label_index, cv2.CC_STAT_WIDTH])
            height = int(stats[label_index, cv2.CC_STAT_HEIGHT])

            if area < self.min_blob_area or area > self.max_blob_area:
                continue
            if width < self.min_width or height < self.min_height:
                continue

            box_area = max(1, width * height)
            fill_ratio = float(area) / float(box_area)
            if fill_ratio < self.min_fill_ratio:
                continue

            blobs.append(
                {
                    "label_index": label_index,
                    "area": area,
                    "bbox": (x, y, width, height),
                    "centroid": (
                        float(centroids[label_index][0]),
                        float(centroids[label_index][1]),
                    ),
                    "fill_ratio": fill_ratio,
                }
            )

        return labels, blobs

    # ------------------------------------------------------------------
    # Single-target tracking
    # ------------------------------------------------------------------
    @staticmethod
    def bbox_iou(box_a, box_b):
        ax, ay, aw, ah = box_a
        bx, by, bw, bh = box_b

        intersection_x0 = max(ax, bx)
        intersection_y0 = max(ay, by)
        intersection_x1 = min(ax + aw, bx + bw)
        intersection_y1 = min(ay + ah, by + bh)

        intersection_w = max(0, intersection_x1 - intersection_x0)
        intersection_h = max(0, intersection_y1 - intersection_y0)
        intersection_area = intersection_w * intersection_h

        union_area = aw * ah + bw * bh - intersection_area
        if union_area <= 0:
            return 0.0
        return float(intersection_area) / float(union_area)

    @staticmethod
    def centroid_distance(blob_a, blob_b, roi_height, roi_width):
        ax, ay = blob_a["centroid"]
        bx, by = blob_b["centroid"]
        diagonal = max(1.0, float(np.hypot(roi_width, roi_height)))
        return float(np.hypot(ax - bx, ay - by)) / diagonal

    def initial_target_score(self, blob, roi_height, roi_width):
        """Score a new forward target using proximity, centre, and size."""
        x, y, width, height = blob["bbox"]
        centroid_x, _ = blob["centroid"]

        # The bottom of the box is a better monocular proximity cue than the
        # centroid: obstacles closer to the robot normally extend lower.
        bottom_norm = np.clip(float(y + height) / max(1.0, roi_height), 0.0, 1.0)

        half_width = max(1.0, roi_width / 2.0)
        centre_norm = 1.0 - min(
            1.0, abs(centroid_x - roi_width / 2.0) / half_width
        )

        area_fraction = float(blob["area"]) / max(1.0, roi_height * roi_width)
        area_norm = min(1.0, np.sqrt(area_fraction / 0.20))

        return (
            self.target_proximity_weight * bottom_norm
            + self.target_center_weight * centre_norm
            + self.target_area_weight * area_norm
        )

    def acquire_target(self, blobs, roi_height, roi_width):
        """
        Acquire one obstacle only when it is central and close enough.

        Horizontal filtering uses the blob centroid. Distance filtering uses
        the bottom of the bounding box because, for a forward-facing camera,
        nearby ground obstacles normally extend lower in the image.
        """
        corridor_width = np.clip(self.target_center_width, 0.05, 1.0) * roi_width
        corridor_left = (roi_width - corridor_width) / 2.0
        corridor_right = corridor_left + corridor_width
        minimum_bottom = np.clip(self.target_min_bottom, 0.0, 1.0) * roi_height

        candidates = []
        for blob in blobs:
            x, y, width, height = blob["bbox"]
            centroid_x = blob["centroid"][0]
            box_bottom = y + height

            is_central = corridor_left <= centroid_x <= corridor_right
            is_close_enough = box_bottom >= minimum_bottom

            if is_central and is_close_enough:
                candidates.append(blob)

        if not candidates:
            return None

        selected = max(
            candidates,
            key=lambda blob: self.initial_target_score(
                blob, roi_height, roi_width
            ),
        )

        self.target_id += 1
        self.target_missed_frames = 0
        self.tracked_blob = dict(selected)
        return selected

    def associate_target(self, blobs, roi_height, roi_width):
        """Associate the previous target with one current-frame blob."""
        if self.tracked_blob is None or not blobs:
            return None

        matches = []
        for blob in blobs:
            iou = self.bbox_iou(self.tracked_blob["bbox"], blob["bbox"])
            distance = self.centroid_distance(
                self.tracked_blob, blob, roi_height, roi_width
            )

            if iou < self.target_min_iou and distance > self.target_max_distance:
                continue

            distance_score = max(
                0.0,
                1.0 - distance / max(1e-6, self.target_max_distance),
            )
            association_score = 0.70 * iou + 0.30 * distance_score
            matches.append((association_score, blob))

        if not matches:
            return None

        matches.sort(key=lambda item: item[0], reverse=True)
        return matches[0][1]

    def update_target(self, blobs, roi_height, roi_width):
        """
        Return the currently observed target blob, or None.

        The lock is retained through short detection gaps. During that gap the
        danger output is zero, but another obstacle is not selected. This avoids
        target switching caused by one or two flickering frames.
        """
        if self.tracked_blob is None:
            return self.acquire_target(blobs, roi_height, roi_width)

        matched = self.associate_target(blobs, roi_height, roi_width)
        if matched is not None:
            self.tracked_blob = dict(matched)
            self.target_missed_frames = 0
            return matched

        self.target_missed_frames += 1

        if self.target_missed_frames <= self.target_max_missed_frames:
            return None

        rospy.loginfo(
            "[perception] released target #%d after %d missed frames",
            self.target_id,
            self.target_missed_frames,
        )
        self.tracked_blob = None
        self.target_missed_frames = 0

        # The old target has now been considered dodged. A new forward target
        # may be selected immediately from the current frame.
        return self.acquire_target(blobs, roi_height, roi_width)

    @staticmethod
    def make_target_mask(labels, target_blob):
        target_mask = np.zeros(labels.shape, dtype=np.uint8)
        if target_blob is not None:
            target_mask[labels == target_blob["label_index"]] = 255
        return target_mask

    # ------------------------------------------------------------------
    # Danger output
    # ------------------------------------------------------------------
    def compute_sector_dangers(self, target_mask, roi_height, roi_width):
        """Compute sector danger using only the selected target mask."""
        num = max(1, self.num_sectors)
        dangers = np.zeros(num, dtype=np.float32)

        if roi_height <= 1 or roi_width <= 1:
            return dangers

        row_norm = np.arange(roi_height, dtype=np.float32) / (roi_height - 1)
        row_norm = row_norm.reshape(-1, 1)
        proximity_weight = 0.4 + 0.6 * row_norm

        target_pixels = (target_mask > 0).astype(np.float32)
        weighted = target_pixels * proximity_weight

        sector_width = roi_width / float(num)
        for index in range(num):
            sector_x0 = int(round(index * sector_width))
            sector_x1 = (
                int(round((index + 1) * sector_width))
                if index < num - 1
                else roi_width
            )
            sector_x1 = max(sector_x1, sector_x0 + 1)

            strip = weighted[:, sector_x0:sector_x1]
            if strip.size == 0:
                continue

            coverage = float(strip.sum()) / float(strip.shape[0] * strip.shape[1])
            dangers[index] = min(1.0, coverage * self.danger_gain)

        return dangers

    # ------------------------------------------------------------------
    # Controller preview (visualization only)
    # ------------------------------------------------------------------
    def choose_turn_direction(self, dangers):
        """Turn toward the side with less predicted obstacle danger."""
        if len(dangers) == 0:
            return "LEFT"

        center = len(dangers) // 2
        left_danger = float(np.sum(dangers[:center]))
        right_danger = float(np.sum(dangers[center + 1:]))

        # If the left side is more blocked, turn right, and vice versa.
        if left_danger > right_danger:
            return "RIGHT"
        if right_danger > left_danger:
            return "LEFT"

        # Deterministic tie-breaker.
        return self.preview_turn_direction or "LEFT"

    def update_controller_preview(self, target_blob, dangers, roi_height):
        """
        Overwatch preview only. No cmd_vel is published.

        States:
          IDLE             no confirmed close central obstacle
          STOP             close obstacle is being confirmed
          SHARP TURN LEFT  confirmed obstacle; left side is clearer
          SHARP TURN RIGHT confirmed obstacle; right side is clearer
        """
        if target_blob is None:
            self.preview_trigger_count = 0
            self.preview_clear_count += 1
            if self.preview_clear_count >= self.preview_release_frames:
                self.preview_state = "IDLE"
                self.preview_turn_direction = None
            return self.preview_state

        self.preview_clear_count = 0

        _, box_y, _, box_height = target_blob["bbox"]
        bottom_norm = float(box_y + box_height) / max(1.0, float(roi_height))

        center = len(dangers) // 2
        center_start = max(0, center - 1)
        center_end = min(len(dangers), center + 2)
        center_peak = (
            float(np.max(dangers[center_start:center_end]))
            if len(dangers) > 0 else 0.0
        )
        strong_count = int(np.count_nonzero(dangers >= self.preview_stop_danger))

        emergency = (
            bottom_norm >= self.preview_stop_bottom
            and center_peak >= self.preview_stop_danger
            and strong_count >= self.preview_min_strong_sectors
        )

        if not emergency:
            self.preview_trigger_count = 0
            self.preview_turn_direction = None
            self.preview_state = "IDLE"
            return self.preview_state

        self.preview_trigger_count += 1
        if self.preview_turn_direction is None:
            self.preview_turn_direction = self.choose_turn_direction(dangers)

        if self.preview_trigger_count < self.preview_confirm_frames:
            self.preview_state = "STOP"
        else:
            self.preview_state = "SHARP TURN {}".format(
                self.preview_turn_direction
            )
        return self.preview_state

    @staticmethod
    def draw_controller_preview(debug, state, target_blob, dangers, x0, x1, y0, y1):
        """Draw a large controller-preview panel on the image."""
        panel_x0 = x0
        panel_x1 = x1
        panel_y0 = max(0, y0 - 76)
        panel_y1 = max(panel_y0 + 1, y0 - 8)

        overlay = debug.copy()
        cv2.rectangle(overlay, (panel_x0, panel_y0), (panel_x1, panel_y1), (20, 20, 20), -1)
        debug[:] = cv2.addWeighted(debug, 0.35, overlay, 0.65, 0)

        if state == "IDLE":
            color = (180, 180, 180)
        elif state == "STOP":
            color = (0, 80, 255)
        else:
            color = (0, 0, 255)

        cv2.putText(
            debug,
            "OVERWATCH PREVIEW: {}".format(state),
            (panel_x0 + 10, panel_y0 + 29),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            color,
            2,
            cv2.LINE_AA,
        )

        max_danger = float(np.max(dangers)) if len(dangers) else 0.0
        if target_blob is None:
            detail = "No target | no cmd_vel published"
        else:
            _, by, _, bh = target_blob["bbox"]
            bottom_norm = float(by + bh) / max(1.0, float(y1 - y0))
            strong = int(np.count_nonzero(dangers >= 0.40))
            detail = "bottom={:.2f}  strong sectors={}  max danger={:.2f}  [visual only]".format(
                bottom_norm, strong, max_danger
            )

        cv2.putText(
            debug,
            detail,
            (panel_x0 + 10, panel_y0 + 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------
    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, "cv_bridge conversion failed: %s", str(error))
            return

        image_height, image_width = image.shape[:2]

        x0 = int(np.clip(self.roi_x0, 0.0, 1.0) * image_width)
        x1 = int(np.clip(self.roi_x1, 0.0, 1.0) * image_width)
        y0 = int(np.clip(self.roi_y0, 0.0, 1.0) * image_height)
        y1 = int(np.clip(self.roi_y1, 0.0, 1.0) * image_height)

        if x1 <= x0 or y1 <= y0:
            rospy.logwarn_throttle(2.0, "Invalid ROI dimensions")
            return

        roi = image[y0:y1, x0:x1]
        roi_height, roi_width = roi.shape[:2]

        raw_mask = self.detect_red(roi)
        stable_mask = self.apply_temporal_smoothing(raw_mask)
        cleaned_mask = self.clean_mask(stable_mask)
        trigger_mask = self.apply_close_central_gate(cleaned_mask)
        labels, blobs = self.find_obstacle_blobs(trigger_mask)

        target_blob = self.update_target(blobs, roi_height, roi_width)
        target_mask = self.make_target_mask(labels, target_blob)

        dangers = self.compute_sector_dangers(
            target_mask, roi_height, roi_width
        )
        self.publish_sectors(dangers)

        controller_state = self.update_controller_preview(
            target_blob, dangers, roi_height
        )

        debug = self.make_debug_image(
            image=image,
            target_mask=target_mask,
            target_blob=target_blob,
            dangers=dangers,
            x0=x0,
            x1=x1,
            y0=y0,
            y1=y1,
            controller_state=controller_state,
        )

        debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_message.header = message.header
        self.debug_pub.publish(debug_message)

        rospy.loginfo_throttle(
            1.0,
            "[perception] candidates=%d target=%s missed=%d max_danger=%.2f preview=%s",
            len(blobs),
            "#{}".format(self.target_id) if self.tracked_blob is not None else "none",
            self.target_missed_frames,
            float(dangers.max()) if len(dangers) else 0.0,
            controller_state,
        )

    def publish_sectors(self, dangers):
        message = Float32MultiArray()
        dimension = MultiArrayDimension()
        dimension.label = "sector"
        dimension.size = len(dangers)
        dimension.stride = len(dangers)
        message.layout.dim = [dimension]
        message.data = [float(danger) for danger in dangers]
        self.sectors_pub.publish(message)

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------
    def make_debug_image(
        self, image, target_mask, target_blob, dangers, x0, x1, y0, y1,
        controller_state
    ):
        debug = image.copy()

        if self.draw_mask_overlay and target_blob is not None:
            overlay = debug.copy()
            overlay_roi = overlay[y0:y1, x0:x1]
            overlay_roi[target_mask > 0] = (0, 0, 255)
            overlay[y0:y1, x0:x1] = overlay_roi
            debug = cv2.addWeighted(debug, 0.75, overlay, 0.25, 0)

        if self.draw_roi:
            cv2.rectangle(debug, (x0, y0), (x1, y1), (255, 255, 255), 2)

        if self.draw_forward_corridor:
            roi_width = x1 - x0
            corridor_width = int(
                np.clip(self.target_center_width, 0.05, 1.0) * roi_width
            )
            corridor_left = x0 + (roi_width - corridor_width) // 2
            corridor_right = corridor_left + corridor_width
            cv2.line(debug, (corridor_left, y0), (corridor_left, y1), (255, 180, 0), 2)
            cv2.line(debug, (corridor_right, y0), (corridor_right, y1), (255, 180, 0), 2)

        if self.draw_acquisition_line:
            roi_height = y1 - y0
            acquisition_y = y0 + int(
                np.clip(self.target_min_bottom, 0.0, 1.0) * roi_height
            )
            cv2.line(debug, (x0, acquisition_y), (x1, acquisition_y), (0, 165, 255), 2)
            cv2.putText(
                debug,
                "ACQUIRE BELOW",
                (x0 + 8, max(y0 + 20, acquisition_y - 7)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (0, 165, 255),
                2,
                cv2.LINE_AA,
            )

        if target_blob is not None:
            box_x, box_y, box_width, box_height = target_blob["bbox"]
            absolute_x = x0 + box_x
            absolute_y = y0 + box_y

            cv2.rectangle(
                debug,
                (absolute_x, absolute_y),
                (absolute_x + box_width, absolute_y + box_height),
                (0, 255, 255),
                3,
            )

            if self.draw_centroid:
                centroid_x = int(x0 + target_blob["centroid"][0])
                centroid_y = int(y0 + target_blob["centroid"][1])
                cv2.circle(debug, (centroid_x, centroid_y), 6, (255, 0, 255), -1)

            label = "TARGET #{}  A={}  F={:.2f}".format(
                self.target_id,
                target_blob["area"],
                target_blob["fill_ratio"],
            )
            cv2.putText(
                debug,
                label,
                (absolute_x, max(25, absolute_y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        if self.draw_sector_bars and len(dangers) > 0:
            self.draw_danger_bars(debug, dangers, x0, x1, y1)

        if target_blob is not None:
            summary = "Tracking target #{}".format(self.target_id)
        elif self.tracked_blob is not None:
            summary = "Target #{} temporarily lost: {}/{}".format(
                self.target_id,
                self.target_missed_frames,
                self.target_max_missed_frames,
            )
        else:
            summary = "No close central obstacle"

        cv2.putText(
            debug,
            summary,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.80,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )

        self.draw_controller_preview(
            debug,
            controller_state,
            target_blob,
            dangers,
            x0,
            x1,
            y0,
            y1,
        )

        return debug

    @staticmethod
    def draw_danger_bars(debug, dangers, x0, x1, y1, bar_height=30, margin=6):
        num = len(dangers)
        roi_width = x1 - x0
        sector_width = roi_width / float(num)
        bar_top = y1 + margin
        bar_bottom = bar_top + bar_height

        if bar_bottom >= debug.shape[0]:
            return

        for index, danger in enumerate(dangers):
            sector_x0 = int(x0 + index * sector_width)
            sector_x1 = int(x0 + (index + 1) * sector_width)

            color = (0, int(255 * (1.0 - danger)), int(255 * danger))
            filled_top = int(bar_bottom - danger * bar_height)

            cv2.rectangle(
                debug,
                (sector_x0 + 1, bar_top),
                (sector_x1 - 1, bar_bottom),
                (60, 60, 60),
                1,
            )
            cv2.rectangle(
                debug,
                (sector_x0 + 1, filled_top),
                (sector_x1 - 1, bar_bottom),
                color,
                -1,
            )


if __name__ == "__main__":
    try:
        RedObstaclePerception()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
