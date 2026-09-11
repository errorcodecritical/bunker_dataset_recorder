#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from collections import deque
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class TemporalMaskSmoother:
    def __init__(self):
        rospy.init_node("temporal_mask_smoother")

        self.bridge = CvBridge()

        # ------------------------------------------------------
        # Topics
        # ------------------------------------------------------
        self.input_topic = rospy.get_param(
            "~input_topic",
            "/inference_overlay"
        )

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/inference_overlay_smoothed"
        )

        self.mask_topic = rospy.get_param(
            "~mask_topic",
            "/inference_mask_smoothed"
        )

        # ------------------------------------------------------
        # Red-pixel extraction thresholds
        # OpenCV uses BGR ordering.
        # ------------------------------------------------------
        self.red_min = int(
            rospy.get_param("~red_min", 140)
        )

        self.green_max = int(
            rospy.get_param("~green_max", 130)
        )

        self.blue_max = int(
            rospy.get_param("~blue_max", 130)
        )

        self.red_difference = int(
            rospy.get_param("~red_difference", 35)
        )

        # ------------------------------------------------------
        # Temporal sliding-window settings
        # ------------------------------------------------------
        # At approximately 49 Hz, 50 frames is about 1 second.
        self.history_len = int(
            rospy.get_param("~history_len", 50)
        )

        # A new pixel must appear in this many history frames
        # before it is accepted as a stable red prediction.
        self.enter_hits = int(
            rospy.get_param("~enter_hits", 43)
        )

        # Once accepted, the pixel remains stable while at least
        # this many frames in the history still support it.
        self.exit_hits = int(
            rospy.get_param("~exit_hits", 20)
        )

        # Wait until the complete temporal history is available.
        self.wait_for_full_history = self.get_bool_param(
            "~wait_for_full_history",
            True
        )

        # Show stable red only where the current input is also red.
        # This prevents old temporal pixels from being painted bright red.
        self.require_current_red = self.get_bool_param(
            "~require_current_red",
            True
        )

        # ------------------------------------------------------
        # Spatial filtering
        # ------------------------------------------------------
        self.kernel_size = int(
            rospy.get_param("~kernel_size", 3)
        )

        self.min_component_area = int(
            rospy.get_param("~min_component_area", 300)
        )

        # Process only the lower half by default.
        self.roi_y0 = float(
            rospy.get_param("~roi_y0", 0.50)
        )

        self.roi_y1 = float(
            rospy.get_param("~roi_y1", 1.00)
        )

        # ------------------------------------------------------
        # Rejected-pixel display settings
        # ------------------------------------------------------
        # Unconfirmed lower-ROI red pixels are recoloured green.
        self.rejected_green_b = int(
            rospy.get_param("~rejected_green_b", 0)
        )

        self.rejected_green_g = int(
            rospy.get_param("~rejected_green_g", 255)
        )

        self.rejected_green_r = int(
            rospy.get_param("~rejected_green_r", 0)
        )

        # 1.0 means solid green.
        # 0.5 means a softer blend with the current overlay.
        self.green_blend = float(
            rospy.get_param("~green_blend", 1.0)
        )
        self.green_blend = float(
            np.clip(self.green_blend, 0.0, 1.0)
        )

        # ------------------------------------------------------
        # Internal temporal state
        # ------------------------------------------------------
        self.mask_history = deque(maxlen=self.history_len)
        self.mask_sum = None
        self.stable_mask = None

        # ------------------------------------------------------
        # ROS communication
        # ------------------------------------------------------
        self.output_pub = rospy.Publisher(
            self.output_topic,
            Image,
            queue_size=1
        )

        self.mask_pub = rospy.Publisher(
            self.mask_topic,
            Image,
            queue_size=1
        )

        self.subscriber = rospy.Subscriber(
            self.input_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        rospy.loginfo("Temporal mask smoother started")
        rospy.loginfo("Input:  %s", self.input_topic)
        rospy.loginfo("Output: %s", self.output_topic)
        rospy.loginfo("Mask:   %s", self.mask_topic)
        rospy.loginfo(
            "History=%d, enter_hits=%d, exit_hits=%d",
            self.history_len,
            self.enter_hits,
            self.exit_hits
        )
        rospy.loginfo(
            "ROI: %.2f to %.2f",
            self.roi_y0,
            self.roi_y1
        )

    @staticmethod
    def get_bool_param(name, default):
        """
        Read a ROS parameter safely as a boolean.

        This avoids the Python problem where bool("false") is True.
        """
        value = rospy.get_param(name, default)

        if isinstance(value, bool):
            return value

        if isinstance(value, str):
            return value.strip().lower() in (
                "true",
                "1",
                "yes",
                "on"
            )

        return bool(value)

    def extract_red_mask(self, image):
        """
        Extract strongly red pixels from the PSteer overlay.

        Returns:
            uint8 binary mask containing 0 and 255.
        """
        blue = image[:, :, 0].astype(np.int16)
        green = image[:, :, 1].astype(np.int16)
        red = image[:, :, 2].astype(np.int16)

        red_mask = (
            (red >= self.red_min) &
            (green <= self.green_max) &
            (blue <= self.blue_max) &
            ((red - green) >= self.red_difference) &
            ((red - blue) >= self.red_difference)
        )

        return red_mask.astype(np.uint8) * 255

    def get_roi_boolean_mask(self, shape):
        """
        Return a boolean mask covering the selected lower image region.
        """
        height, width = shape

        y0 = int(
            np.clip(self.roi_y0, 0.0, 1.0) * height
        )

        y1 = int(
            np.clip(self.roi_y1, 0.0, 1.0) * height
        )

        y0 = min(y0, height)
        y1 = min(max(y1, y0), height)

        roi = np.zeros(
            (height, width),
            dtype=bool
        )

        roi[y0:y1, :] = True

        return roi

    def apply_roi(self, mask):
        """
        Keep mask pixels only inside the selected lower ROI.
        """
        roi = self.get_roi_boolean_mask(mask.shape)

        output = np.zeros_like(mask)
        output[roi] = mask[roi]

        return output

    def remove_small_components(self, mask):
        """
        Remove connected regions below min_component_area.
        """
        number_labels, labels, stats, _ = (
            cv2.connectedComponentsWithStats(
                mask,
                connectivity=8
            )
        )

        cleaned = np.zeros_like(mask)

        for label_index in range(1, number_labels):
            area = stats[
                label_index,
                cv2.CC_STAT_AREA
            ]

            if area >= self.min_component_area:
                cleaned[labels == label_index] = 255

        return cleaned

    def spatial_cleanup(self, mask):
        """
        Clean the current frame before adding it to temporal history.
        """
        kernel_size = max(1, int(self.kernel_size))

        if kernel_size % 2 == 0:
            kernel_size += 1

        if kernel_size > 1:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (kernel_size, kernel_size)
            )

            # Remove isolated speckles.
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                kernel,
                iterations=1
            )

            # Fill only small internal gaps.
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                kernel,
                iterations=1
            )

        return self.remove_small_components(mask)

    def reset_temporal_state(self, shape):
        """
        Reset temporal memory when image resolution changes.
        """
        self.mask_history.clear()

        self.mask_sum = np.zeros(
            shape,
            dtype=np.uint16
        )

        self.stable_mask = np.zeros(
            shape,
            dtype=bool
        )

        rospy.logwarn(
            "Temporal state reset for image shape %s",
            str(shape)
        )

    def update_temporal_filter(self, current_mask):
        """
        Perform a true sliding-window per-pixel vote.
        """
        current_binary = (
            current_mask > 0
        ).astype(np.uint8)

        if (
            self.mask_sum is None or
            self.mask_sum.shape != current_binary.shape
        ):
            self.reset_temporal_state(
                current_binary.shape
            )

        # Subtract the oldest frame before deque removes it.
        if len(self.mask_history) == self.history_len:
            oldest_mask = self.mask_history[0]

            self.mask_sum -= oldest_mask.astype(
                np.uint16
            )

        self.mask_history.append(
            current_binary.copy()
        )

        self.mask_sum += current_binary.astype(
            np.uint16
        )

        frame_count = len(self.mask_history)

        if (
            self.wait_for_full_history and
            frame_count < self.history_len
        ):
            self.stable_mask[:] = False

            return np.zeros_like(
                current_mask,
                dtype=np.uint8
            )

        # Protect against invalid parameter combinations when testing.
        effective_enter_hits = min(
            max(1, self.enter_hits),
            frame_count
        )

        effective_exit_hits = min(
            max(1, self.exit_hits),
            frame_count
        )

        turn_on = (
            self.mask_sum >= effective_enter_hits
        )

        remain_on = (
            self.mask_sum >= effective_exit_hits
        )

        self.stable_mask = np.where(
            self.stable_mask,
            remain_on,
            turn_on
        )

        return self.stable_mask.astype(
            np.uint8
        ) * 255

    def make_display_mask(
        self,
        stable_mask,
        cleaned_current_mask
    ):
        """
        Restrict displayed red pixels to stable pixels that are also
        red in the current cleaned frame.

        This prevents historical red pixels from being drawn in places
        that are no longer red in the current PSteer output.
        """
        if not self.require_current_red:
            return stable_mask

        return cv2.bitwise_and(
            stable_mask,
            cleaned_current_mask
        )

    def create_clean_overlay(
        self,
        image,
        original_red_mask,
        display_mask
    ):
        """
        Create the cleaned PSteer overlay.

        Outside the lower ROI:
            preserve the original overlay unchanged.

        Inside the lower ROI:
            - temporally confirmed current red pixels retain their
              original PSteer colour;
            - rejected red pixels are changed to green.
        """
        output = image.copy()

        roi_pixels = self.get_roi_boolean_mask(
            original_red_mask.shape
        )

        original_red_pixels = (
            original_red_mask > 0
        )

        confirmed_red_pixels = (
            display_mask > 0
        )

        # Only red pixels inside the lower ROI can be rejected.
        rejected_red_pixels = (
            roi_pixels &
            original_red_pixels &
            ~confirmed_red_pixels
        )

        if np.any(rejected_red_pixels):
            green_colour = np.array(
                [
                    self.rejected_green_b,
                    self.rejected_green_g,
                    self.rejected_green_r
                ],
                dtype=np.float32
            )

            if self.green_blend >= 1.0:
                output[rejected_red_pixels] = (
                    green_colour.astype(np.uint8)
                )
            else:
                original_values = output[
                    rejected_red_pixels
                ].astype(np.float32)

                blended_values = (
                    (1.0 - self.green_blend) *
                    original_values
                    +
                    self.green_blend *
                    green_colour
                )

                output[rejected_red_pixels] = np.clip(
                    blended_values,
                    0,
                    255
                ).astype(np.uint8)

        # Confirmed red pixels are not repainted.
        # Their current PSteer overlay colour is preserved.

        status = (
            f"History {len(self.mask_history)}/"
            f"{self.history_len}  "
            f"Enter {self.enter_hits}  "
            f"Exit {self.exit_hits}"
        )

        cv2.putText(
            output,
            status,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        return output

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8"
            )

        except CvBridgeError as error:
            rospy.logerr_throttle(
                2.0,
                "cv_bridge conversion failed: %s",
                str(error)
            )
            return

        # 1. Extract red pixels from the full PSteer overlay.
        raw_mask = self.extract_red_mask(image)

        # 2. Analyse only the lower image region.
        roi_mask = self.apply_roi(raw_mask)

        # 3. Remove tiny detections before temporal filtering.
        cleaned_current_mask = self.spatial_cleanup(
            roi_mask
        )

        # 4. Perform sliding-window temporal voting.
        stable_mask = self.update_temporal_filter(
            cleaned_current_mask
        )

        # 5. Display only stable pixels that are currently red.
        display_mask = self.make_display_mask(
            stable_mask,
            cleaned_current_mask
        )

        # 6. Recolour rejected lower-ROI red pixels green.
        clean_overlay = self.create_clean_overlay(
            image,
            raw_mask,
            display_mask
        )

        output_message = self.bridge.cv2_to_imgmsg(
            clean_overlay,
            encoding="bgr8"
        )

        mask_message = self.bridge.cv2_to_imgmsg(
            display_mask,
            encoding="mono8"
        )

        output_message.header = message.header
        mask_message.header = message.header

        self.output_pub.publish(output_message)
        self.mask_pub.publish(mask_message)


if __name__ == "__main__":
    try:
        TemporalMaskSmoother()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
