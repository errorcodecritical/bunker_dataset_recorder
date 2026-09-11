#!/usr/bin/env python
"""
imu_reclock_node.py

Live reclocking of bursty Xsens IMU timestamps, for insertion between the
raw driver and IG-LIO.

Problem:
    The Xsens driver appears to buffer ~5 samples internally and flush them
    together, so header stamps show dt ~0.01ms for several messages then a
    ~12.5ms jump. This corrupts per-sample dt-based IMU preintegration in
    IG-LIO. Values are fine, only per-message timing is corrupted.

Fix:
    Maintain a trailing window of (sample_index, raw_header_time) pairs and
    continuously refit a line time = a + b*index via incremental least
    squares (O(1) update per sample, no windowed recompute-from-scratch).
    Because the window spans many burst-flush cycles, the regression
    naturally averages out the local burst jitter while tracking the true
    long-term sample rate (~400 Hz), including any slow clock drift.

    Each outgoing message gets header.stamp = a + b*index instead of the
    raw (bursty) stamp.

Usage (typical wiring):
    Raw Xsens driver publishes on, say, /imu/data_raw (remap it away from
    /imu/data so IG-LIO doesn't see it directly). This node subscribes to
    /imu/data_raw and republishes the corrected stream on /imu/data, which
    IG-LIO subscribes to as normal.

    rosrun your_pkg imu_reclock_node.py \
        _raw_topic:=/imu/data_raw \
        _clean_topic:=/imu/data \
        _window:=400

Caveats:
    - The first `window` samples after startup are published with a
      partially-fit or raw stamp (not enough history yet for a stable fit).
      Expect ~1s of degraded timing at the very start of each run -- make
      sure IG-LIO's own initialization/warmup covers this, or add a short
      static delay before your nav stack starts consuming state estimates.
    - This adds no artificial latency to message delivery: each message is
      published as soon as it arrives, just with a corrected stamp. If you
      need center-windowed (non-causal, more accurate) correction instead,
      that requires buffering and delaying output by window/2 samples --
      only do that if live latency budget allows it.
    - Uses raw header time (not wall-clock arrival time) as the regression
      input, matching the validated offline approach, since the burst
      pattern shows up equally in header and record time in your bag
      report -- the regression cancels it either way.
"""
import collections

import rospy
from sensor_msgs.msg import Imu


class TrailingLeastSquares(object):
    """Incremental windowed least squares fit of time = a + b*index."""

    def __init__(self, window):
        self.window = window
        self.buf = collections.deque()  # holds (idx, y) pairs
        self.sum_x = 0.0
        self.sum_y = 0.0
        self.sum_xx = 0.0
        self.sum_xy = 0.0
        self.ref_time = None  # anchor to keep y values small for numerical stability

    def add(self, idx, raw_time):
        if self.ref_time is None:
            self.ref_time = raw_time
        y = raw_time - self.ref_time  # keep values small (seconds since start)

        self.buf.append((idx, y))
        self.sum_x += idx
        self.sum_y += y
        self.sum_xx += idx * idx
        self.sum_xy += idx * y

        if len(self.buf) > self.window:
            old_idx, old_y = self.buf.popleft()
            self.sum_x -= old_idx
            self.sum_y -= old_y
            self.sum_xx -= old_idx * old_idx
            self.sum_xy -= old_idx * old_y

    def ready(self):
        return len(self.buf) >= max(10, self.window // 4)

    def predict(self, idx):
        n = len(self.buf)
        denom = n * self.sum_xx - self.sum_x * self.sum_x
        if n < 2 or denom == 0:
            return None
        b = (n * self.sum_xy - self.sum_x * self.sum_y) / denom
        a = (self.sum_y - b * self.sum_x) / n
        return self.ref_time + (a + b * idx)


class ImuReclockNode(object):
    def __init__(self):
        raw_topic = rospy.get_param("~raw_topic", "/imu/data_raw")
        clean_topic = rospy.get_param("~clean_topic", "/imu/data")
        window = rospy.get_param("~window", 400)
        queue_size = rospy.get_param("~queue_size", 50)

        self.idx = 0
        self.fitter = TrailingLeastSquares(window)

        self.pub = rospy.Publisher(clean_topic, Imu, queue_size=queue_size)
        self.sub = rospy.Subscriber(raw_topic, Imu, self.callback, queue_size=queue_size)

        rospy.loginfo("imu_reclock_node: %s -> %s (window=%d)",
                       raw_topic, clean_topic, window)

    def callback(self, msg):
        raw_time = msg.header.stamp.to_sec()
        self.fitter.add(self.idx, raw_time)

        if self.fitter.ready():
            corrected = self.fitter.predict(self.idx)
            if corrected is not None:
                msg.header.stamp = rospy.Time.from_sec(corrected)
            # else: degenerate fit (shouldn't normally happen), pass raw stamp through
        # else: still warming up, pass raw stamp through unmodified

        self.pub.publish(msg)
        self.idx += 1


def main():
    rospy.init_node("imu_reclock_node")
    ImuReclockNode()
    rospy.spin()


if __name__ == "__main__":
    main()
