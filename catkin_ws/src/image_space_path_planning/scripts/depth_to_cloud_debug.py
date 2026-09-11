#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge

class DepthToCloud:
    def __init__(self):
        self.bridge = CvBridge()

        # --- fixed intrinsics (from your YAML) ---
        self.K = np.array([
            [612.9004516601562, 0.0, 321.5191345214844],
            [0.0, 613.1563110351562, 276.79446411132812],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        self.frame_id = rospy.get_param("~frame_id", "camera_color_optical_frame")

        # Topics
        self.depth_topic = rospy.get_param("~depth_topic", "/debug/depth")
        self.pub_topic   = rospy.get_param("~cloud_topic", "/debug_depth_cloud")

        # Params (for relative depth images)
        self.step = int(rospy.get_param("~step", 4))
        self.min_depth = float(rospy.get_param("~min_depth", 1.0))
        self.max_depth = float(rospy.get_param("~max_depth", 65000.0))
        self.max_points = int(rospy.get_param("~max_points", 250000))

        self.cloud_pub = rospy.Publisher(self.pub_topic, PointCloud2, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1)

        rospy.loginfo(f"[depth_to_cloud_debug] Subscribing depth: {self.depth_topic}")
        rospy.loginfo(f"[depth_to_cloud_debug] Publishing cloud: {self.pub_topic}")

    def cb_depth(self, msg: Image):
        # Read whatever encoding arrives (mono8/mono16/32FC1)
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"[depth_to_cloud_debug] cv_bridge failed: {e}")
            return

        depth = depth_raw.astype(np.float32)
        H, W = depth.shape[:2]

        fx = float(self.K[0, 0])
        fy = float(self.K[1, 1])
        cx = float(self.K[0, 2])
        cy = float(self.K[1, 2])

        step = max(1, self.step)

        us = np.arange(0, W, step, dtype=np.float32)
        vs = np.arange(0, H, step, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)

        z = depth[vv.astype(np.int32), uu.astype(np.int32)]

        valid = np.isfinite(z) & (z > self.min_depth) & (z < self.max_depth)
        uu = uu[valid]
        vv = vv[valid]
        z  = z[valid]

        if z.size == 0:
            rospy.logwarn_throttle(1.0, "[depth_to_cloud_debug] No valid depth points.")
            return

        x = (uu - cx) * z / fx
        y = (vv - cy) * z / fy
        pts = np.stack([x, y, z], axis=1)

        if pts.shape[0] > self.max_points:
            pts = pts[: self.max_points, :]

        header = msg.header
        header.frame_id = self.frame_id

        cloud_msg = pc2.create_cloud_xyz32(header, pts.tolist())
        self.cloud_pub.publish(cloud_msg)

def main():
    rospy.init_node("depth_to_cloud_debug", anonymous=True)
    DepthToCloud()
    rospy.spin()

if __name__ == "__main__":
    main()
