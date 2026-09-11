#!/usr/bin/env python3

import math
import statistics
import sys
from collections import Counter

import rospy
from sensor_msgs.msg import Imu


class ImuIgLio15SecondTest:
    def __init__(self):
        self.topic = rospy.get_param("~imu_topic", "/imu/data")
        self.test_duration = float(rospy.get_param("~duration", 15.0))
        self.expected_rate = float(rospy.get_param("~expected_rate", 400.0))
        self.gravity = float(rospy.get_param("~gravity", 9.80665))

        # Timing thresholds
        self.large_gap_threshold = float(
            rospy.get_param("~large_gap_threshold", 0.02)
        )
        self.max_rate_error_fraction = float(
            rospy.get_param("~max_rate_error_fraction", 0.15)
        )

        # Approximate quality thresholds
        self.stationary_gyro_threshold = float(
            rospy.get_param("~stationary_gyro_threshold", 0.03)
        )
        self.stationary_accel_std_threshold = float(
            rospy.get_param("~stationary_accel_std_threshold", 0.20)
        )
        self.high_gyro_noise_threshold = float(
            rospy.get_param("~high_gyro_noise_threshold", 0.03)
        )
        self.high_accel_noise_threshold = float(
            rospy.get_param("~high_accel_noise_threshold", 0.60)
        )

        self.started = False
        self.finished = False

        self.start_stamp = None
        self.last_stamp = None

        self.frame_ids = Counter()

        self.stamps = []
        self.dts = []

        self.ax = []
        self.ay = []
        self.az = []

        self.gx = []
        self.gy = []
        self.gz = []

        self.acc_norms = []
        self.gyro_norms = []

        self.backward_timestamps = 0
        self.duplicate_timestamps = 0
        self.large_gaps = 0
        self.invalid_values = 0
        self.zero_timestamps = 0

        self.max_gap = 0.0

        self.subscriber = rospy.Subscriber(
            self.topic,
            Imu,
            self.callback,
            queue_size=5000,
            tcp_nodelay=True,
        )

        print("=" * 76)
        print("IG-LIO IMU 15-SECOND DRIVING TEST")
        print("=" * 76)
        print("Waiting for IMU data on:", self.topic)
        print()
        print("When recording starts, drive using this sequence:")
        print("  1. Drive straight")
        print("  2. Turn left")
        print("  3. Turn right")
        print("  4. Stop briefly if possible")
        print()
        print("The script will stop automatically after {:.1f} seconds.".format(
            self.test_duration
        ))
        print("=" * 76)

    @staticmethod
    def norm(x, y, z):
        return math.sqrt(x * x + y * y + z * z)

    @staticmethod
    def safe_mean(values):
        return statistics.mean(values) if values else float("nan")

    @staticmethod
    def safe_std(values):
        if len(values) < 2:
            return 0.0
        return statistics.pstdev(values)

    @staticmethod
    def percentile(values, percentile):
        if not values:
            return float("nan")

        ordered = sorted(values)
        index = (len(ordered) - 1) * percentile
        lower = int(math.floor(index))
        upper = int(math.ceil(index))

        if lower == upper:
            return ordered[lower]

        fraction = index - lower
        return (
            ordered[lower] * (1.0 - fraction)
            + ordered[upper] * fraction
        )

    def callback(self, msg):
        if self.finished:
            return

        stamp = msg.header.stamp.to_sec()

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        values = [ax, ay, az, gx, gy, gz]

        if not all(math.isfinite(value) for value in values):
            self.invalid_values += 1
            return

        if stamp <= 0.0:
            self.zero_timestamps += 1
            return

        if not self.started:
            self.started = True
            self.start_stamp = stamp
            self.last_stamp = stamp

            print()
            print("Recording started.")
            print("Drive the robot now...")
            print()

        elapsed = stamp - self.start_stamp

        if elapsed >= self.test_duration:
            self.finished = True
            self.print_report()
            rospy.signal_shutdown("15-second IMU test complete")
            return

        self.frame_ids[msg.header.frame_id] += 1

        if self.last_stamp is not None:
            dt = stamp - self.last_stamp

            if dt < 0.0:
                self.backward_timestamps += 1
            elif dt == 0.0:
                self.duplicate_timestamps += 1
            else:
                self.dts.append(dt)

                if dt > self.large_gap_threshold:
                    self.large_gaps += 1

                if dt > self.max_gap:
                    self.max_gap = dt

        self.last_stamp = stamp

        self.stamps.append(stamp)

        self.ax.append(ax)
        self.ay.append(ay)
        self.az.append(az)

        self.gx.append(gx)
        self.gy.append(gy)
        self.gz.append(gz)

        self.acc_norms.append(self.norm(ax, ay, az))
        self.gyro_norms.append(self.norm(gx, gy, gz))

        remaining = self.test_duration - elapsed

        if int(elapsed) != int(elapsed - self.dts[-1] if self.dts else -1):
            print(
                "Recording: {:5.1f} s elapsed, {:5.1f} s remaining".format(
                    elapsed,
                    max(0.0, remaining),
                )
            )

    def print_report(self):
        sample_count = len(self.stamps)

        if sample_count < 10:
            print("Not enough valid IMU samples were received.")
            return

        duration = self.stamps[-1] - self.stamps[0]
        mean_dt = self.safe_mean(self.dts)
        std_dt = self.safe_std(self.dts)

        measured_rate = (
            1.0 / mean_dt
            if math.isfinite(mean_dt) and mean_dt > 0.0
            else float("nan")
        )

        rate_from_count = (
            sample_count / duration
            if duration > 0.0
            else float("nan")
        )

        mean_ax = self.safe_mean(self.ax)
        mean_ay = self.safe_mean(self.ay)
        mean_az = self.safe_mean(self.az)

        std_ax = self.safe_std(self.ax)
        std_ay = self.safe_std(self.ay)
        std_az = self.safe_std(self.az)

        mean_gx = self.safe_mean(self.gx)
        mean_gy = self.safe_mean(self.gy)
        mean_gz = self.safe_mean(self.gz)

        std_gx = self.safe_std(self.gx)
        std_gy = self.safe_std(self.gy)
        std_gz = self.safe_std(self.gz)

        mean_acc_norm = self.safe_mean(self.acc_norms)
        std_acc_norm = self.safe_std(self.acc_norms)

        mean_gyro_norm = self.safe_mean(self.gyro_norms)
        std_gyro_norm = self.safe_std(self.gyro_norms)

        max_abs_ax = max(abs(value) for value in self.ax)
        max_abs_ay = max(abs(value) for value in self.ay)
        max_abs_az = max(abs(value) for value in self.az)

        max_abs_gx = max(abs(value) for value in self.gx)
        max_abs_gy = max(abs(value) for value in self.gy)
        max_abs_gz = max(abs(value) for value in self.gz)

        min_acc_norm = min(self.acc_norms)
        max_acc_norm = max(self.acc_norms)

        gyro_norm_p95 = self.percentile(self.gyro_norms, 0.95)
        accel_norm_p95 = self.percentile(self.acc_norms, 0.95)

        dominant_gyro_axis = max(
            [
                ("X", max_abs_gx),
                ("Y", max_abs_gy),
                ("Z", max_abs_gz),
            ],
            key=lambda item: item[1],
        )

        dominant_accel_axis = max(
            [
                ("X", std_ax),
                ("Y", std_ay),
                ("Z", std_az),
            ],
            key=lambda item: item[1],
        )

        frame_id = (
            self.frame_ids.most_common(1)[0][0]
            if self.frame_ids
            else "<empty>"
        )

        checks = []

        def add(level, description):
            checks.append((level, description))

        # Timing checks
        if math.isfinite(measured_rate):
            rate_error = abs(
                measured_rate - self.expected_rate
            ) / self.expected_rate

            if rate_error <= 0.05:
                add("PASS", "IMU frequency matches expected rate")
            elif rate_error <= self.max_rate_error_fraction:
                add("WARN", "IMU frequency differs moderately from expected")
            else:
                add("FAIL", "IMU frequency differs greatly from expected")
        else:
            add("FAIL", "Could not calculate IMU frequency")

        expected_dt = 1.0 / self.expected_rate

        if std_dt < expected_dt * 0.20:
            add("PASS", "Timestamp intervals are consistent")
        elif std_dt < expected_dt:
            add("WARN", "Moderate timestamp jitter")
        else:
            add("FAIL", "High timestamp jitter")

        if self.backward_timestamps == 0:
            add("PASS", "No backward timestamps")
        else:
            add("FAIL", "Backward timestamps detected")

        if self.duplicate_timestamps == 0:
            add("PASS", "No duplicate timestamps")
        else:
            add("WARN", "Duplicate timestamps detected")

        if self.large_gaps == 0:
            add("PASS", "No large IMU message gaps")
        else:
            add(
                "WARN",
                "{} gaps greater than {:.3f} seconds".format(
                    self.large_gaps,
                    self.large_gap_threshold,
                ),
            )

        if self.invalid_values == 0:
            add("PASS", "No NaN or infinite IMU values")
        else:
            add("FAIL", "NaN or infinite IMU values detected")

        # Units and acceleration checks
        if 7.0 <= mean_acc_norm <= 13.0:
            add("PASS", "Acceleration units appear to be m/s²")
        elif 0.7 <= mean_acc_norm <= 1.3:
            add("FAIL", "Acceleration appears to be reported in g units")
        elif 70.0 <= mean_acc_norm <= 130.0:
            add("FAIL", "Acceleration scaling appears about 10x too large")
        else:
            add("WARN", "Acceleration magnitude is unusual")

        if min_acc_norm > 2.0 and max_acc_norm < 30.0:
            add("PASS", "Acceleration magnitude remained physically plausible")
        else:
            add("WARN", "Acceleration magnitude had unusual peaks or drops")

        if max(std_ax, std_ay, std_az) < 0.001:
            add("FAIL", "Accelerometer may be frozen")
        else:
            add("PASS", "Accelerometer responds during motion")

        if max(std_gx, std_gy, std_gz) < 0.00001:
            add("FAIL", "Gyroscope may be frozen")
        else:
            add("PASS", "Gyroscope responds during motion")

        # Yaw-axis check
        if max_abs_gz > 0.10:
            if max_abs_gz > max(max_abs_gx, max_abs_gy):
                add("PASS", "Robot turning is strongest on gyro Z")
            elif max_abs_gz > 0.75 * max(max_abs_gx, max_abs_gy):
                add("WARN", "Gyro Z responds, but another axis is similarly strong")
            else:
                add("FAIL", "Turning does not appear primarily on gyro Z")
        else:
            add(
                "WARN",
                "No sufficiently strong yaw rotation detected during test",
            )

        # Sign reversal check for left/right turning
        gz_positive = max(self.gz)
        gz_negative = min(self.gz)

        if gz_positive > 0.10 and gz_negative < -0.10:
            add("PASS", "Gyro Z changed sign for opposite-direction turns")
        else:
            add(
                "WARN",
                "Both left and right turns were not clearly detected on gyro Z",
            )

        # Noise/vibration interpretation
        if std_acc_norm < self.high_accel_noise_threshold:
            add("PASS", "Acceleration magnitude noise is reasonable")
        else:
            add("WARN", "High acceleration vibration or aggressive motion")

        if std_gyro_norm < self.high_gyro_noise_threshold:
            add("PASS", "Angular-rate noise is reasonable")
        else:
            add("WARN", "High gyro variation or aggressive rotation")

        # Frame check
        if frame_id:
            add("PASS", "IMU frame ID is present: {}".format(frame_id))
        else:
            add("WARN", "IMU frame ID is empty")

        fail_count = sum(level == "FAIL" for level, _ in checks)
        warn_count = sum(level == "WARN" for level, _ in checks)

        print()
        print("=" * 76)
        print("FINAL IG-LIO IMU REPORT")
        print("=" * 76)

        print("Topic                    :", self.topic)
        print("Frame                    :", frame_id)
        print("Recorded duration        : {:.3f} s".format(duration))
        print("Valid samples            :", sample_count)

        print()
        print("TIMING")
        print("Expected rate            : {:.2f} Hz".format(
            self.expected_rate
        ))
        print("Measured rate from dt    : {:.2f} Hz".format(
            measured_rate
        ))
        print("Measured rate from count : {:.2f} Hz".format(
            rate_from_count
        ))
        print("Mean dt                  : {:.8f} s".format(mean_dt))
        print("dt standard deviation    : {:.8f} s".format(std_dt))
        print("Maximum gap              : {:.6f} s".format(self.max_gap))
        print("Backward timestamps      :", self.backward_timestamps)
        print("Duplicate timestamps     :", self.duplicate_timestamps)
        print("Large gaps               :", self.large_gaps)
        print("Invalid values           :", self.invalid_values)

        print()
        print("LINEAR ACCELERATION [m/s²]")
        print(
            "Mean XYZ                 : {:+.5f} {:+.5f} {:+.5f}".format(
                mean_ax,
                mean_ay,
                mean_az,
            )
        )
        print(
            "Std XYZ                  : {:.5f} {:.5f} {:.5f}".format(
                std_ax,
                std_ay,
                std_az,
            )
        )
        print(
            "Peak |XYZ|               : {:.5f} {:.5f} {:.5f}".format(
                max_abs_ax,
                max_abs_ay,
                max_abs_az,
            )
        )
        print(
            "Norm mean/std            : {:.5f} / {:.5f}".format(
                mean_acc_norm,
                std_acc_norm,
            )
        )
        print(
            "Norm min/max             : {:.5f} / {:.5f}".format(
                min_acc_norm,
                max_acc_norm,
            )
        )
        print("Norm 95th percentile     : {:.5f}".format(
            accel_norm_p95
        ))
        print(
            "Most active accel axis   : {} based on standard deviation".format(
                dominant_accel_axis[0]
            )
        )

        print()
        print("ANGULAR VELOCITY [rad/s]")
        print(
            "Mean XYZ                 : {:+.6f} {:+.6f} {:+.6f}".format(
                mean_gx,
                mean_gy,
                mean_gz,
            )
        )
        print(
            "Std XYZ                  : {:.6f} {:.6f} {:.6f}".format(
                std_gx,
                std_gy,
                std_gz,
            )
        )
        print(
            "Peak |XYZ|               : {:.6f} {:.6f} {:.6f}".format(
                max_abs_gx,
                max_abs_gy,
                max_abs_gz,
            )
        )
        print(
            "Norm mean/std            : {:.6f} / {:.6f}".format(
                mean_gyro_norm,
                std_gyro_norm,
            )
        )
        print("Norm 95th percentile     : {:.6f}".format(
            gyro_norm_p95
        ))
        print(
            "Strongest gyro axis      : {} ({:.6f} rad/s)".format(
                dominant_gyro_axis[0],
                dominant_gyro_axis[1],
            )
        )
        print("Gyro Z positive peak     : {:+.6f}".format(gz_positive))
        print("Gyro Z negative peak     : {:+.6f}".format(gz_negative))

        print()
        print("CHECK RESULTS")

        for level, description in checks:
            print("  {:4s}  {}".format(level, description))

        print()
        print("SUMMARY")

        if fail_count > 0:
            print(
                "  RESULT: POSSIBLE IMU PROBLEM THAT COULD BREAK IG-LIO"
            )
        elif warn_count > 0:
            print(
                "  RESULT: NO CRITICAL FAILURE, BUT REVIEW THE WARNINGS"
            )
        else:
            print(
                "  RESULT: NO OBVIOUS IMU DATA-QUALITY PROBLEM"
            )

        print()
        print("  Failures :", fail_count)
        print("  Warnings :", warn_count)

        print()
        print("INTERPRETATION FOR YOUR BUNKER")

        if dominant_gyro_axis[0] == "Z":
            print(
                "  - Turning primarily affected gyro Z, which matches "
                "a vertical IMU Z axis."
            )
        else:
            print(
                "  - Turning was strongest on gyro {}, not Z. Check the "
                "IMU axis convention and R_imu_lidar.".format(
                    dominant_gyro_axis[0]
                )
            )

        if fail_count == 0:
            print(
                "  - If IG-LIO still gets lost, next check LiDAR point "
                "timestamps, ring fields, per-point time, synchronization, "
                "and LiDAR-to-IMU extrinsics."
            )

        print("=" * 76)


if __name__ == "__main__":
    rospy.init_node("imu_ig_lio_15_second_test")

    try:
        ImuIgLio15SecondTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)
