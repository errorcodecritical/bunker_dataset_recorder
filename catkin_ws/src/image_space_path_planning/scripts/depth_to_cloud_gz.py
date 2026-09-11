#!/usr/bin/env python3
"""
depth_to_cloud.py  (ROS1 / Noetic)

Goal:
  Publish a 3D PointCloud2 from a depth image topic, even if CameraInfo is missing.

Subscribes:
  - ~depth_topic (default: /camera/aligned_depth_to_color/image_raw)  sensor_msgs/Image
  - ~camera_info_topic (default: /camera/aligned_depth_to_color/camera_info) sensor_msgs/CameraInfo (optional)

Publishes:
  - ~cloud_topic (default: /debug_depth_cloud)  sensor_msgs/PointCloud2

Intrinsics source (priority order):
  1) ~K (list of 9 floats, row-major)
  2) ~camera_matrix/data (dict like your YAML: camera_matrix: {data: [..9..]})
  3) /camera_info_topic (sensor_msgs/CameraInfo)  (if published)

Notes:
  - Works with Gazebo depth (32FC1 meters) without camera_info by passing ~K or ~camera_matrix.
  - If depth is 16UC1 (mm), set ~depth_in_meters:=false (it will convert mm->m).
"""

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge


class DepthToCloud:
    def __init__(self):
        self.bridge = CvBridge()
        self.K = None          # 3x3 intrinsics
        self.frame_id = None   # frame for published cloud

        # Params (topics)
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.info_topic  = rospy.get_param("~camera_info_topic", "/camera/aligned_depth_to_color/camera_info")
        self.pub_topic   = rospy.get_param("~cloud_topic", "/debug_depth_cloud")

        # Params (processing)
        self.step = int(rospy.get_param("~step", 4))  # subsample factor (1=all pixels)
        self.min_depth = float(rospy.get_param("~min_depth", 0.2))
        self.max_depth = float(rospy.get_param("~max_depth", 30.0))
        self.max_points = int(rospy.get_param("~max_points", 250000))  # safety cap

        # Depth units handling
        # If True: interpret depth image values as meters (e.g., 32FC1 from Gazebo)
        # If False: interpret as millimeters (e.g., 16UC1) and convert to meters
        self.depth_in_meters = bool(rospy.get_param("~depth_in_meters", True))

        # Optional override for published frame id
        self.override_frame_id = rospy.get_param("~frame_id", "")

        # ---- Fallback intrinsics (if CameraInfo is missing) ----
        # 1) ~K: 9 floats
        K_list = rospy.get_param("~K", [])
        if isinstance(K_list, (list, tuple)) and len(K_list) == 9:
            self.K = np.array(K_list, dtype=np.float64).reshape(3, 3)
            rospy.loginfo("[depth_to_cloud] Using intrinsics from ~K param (CameraInfo not required).")

        # 2) ~camera_matrix: {data: [..9..]}  (matches your YAML style)
        if self.K is None:
            cm = rospy.get_param("~camera_matrix", None)
            if isinstance(cm, dict) and "data" in cm and isinstance(cm["data"], (list, tuple)) and len(cm["data"]) == 9:
                self.K = np.array(cm["data"], dtype=np.float64).reshape(3, 3)
                rospy.loginfo("[depth_to_cloud] Using intrinsics from ~camera_matrix/data param (CameraInfo not required).")

        # Publisher
        self.cloud_pub = rospy.Publisher(self.pub_topic, PointCloud2, queue_size=1)

        # Subscribers
        if self.K is None:
            rospy.Subscriber(self.info_topic, CameraInfo, self.cb_info, queue_size=1)
            rospy.loginfo(f"[depth_to_cloud] Subscribing camera_info: {self.info_topic}")
        else:
            rospy.loginfo("[depth_to_cloud] Skipping CameraInfo subscription (K already provided).")

        rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1)
        rospy.loginfo(f"[depth_to_cloud] Subscribing depth: {self.depth_topic}")

        rospy.loginfo(f"[depth_to_cloud] Publishing cloud: {self.pub_topic}")
        rospy.loginfo(f"[depth_to_cloud] step={self.step} depth_range=[{self.min_depth},{self.max_depth}] "
                      f"max_points={self.max_points} depth_in_meters={self.depth_in_meters}")

    def cb_info(self, msg: CameraInfo):
        # msg.K is length-9 row-major
        K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.K = K
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id

    def cb_depth(self, msg: Image):
        if self.K is None:
            rospy.logwarn_throttle(1.0, "[depth_to_cloud] Waiting for intrinsics (CameraInfo or ~K/~camera_matrix)...")
            return

        # Convert depth image to numpy
        try:
            # If depth is 32FC1, this gives float32 meters
            # If depth is 16UC1, this gives uint16 (often millimeters)
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"[depth_to_cloud] cv_bridge failed: {e}")
            return

        # Normalize depth to float32 meters
        depth = np.asarray(depth)
        if depth.ndim != 2:
            rospy.logwarn_throttle(1.0, f"[depth_to_cloud] Unexpected depth shape: {depth.shape}")
            return

        if depth.dtype == np.uint16:
            # Common: mm in uint16
            depth_m = depth.astype(np.float32) / (1000.0 if not self.depth_in_meters else 1.0)
        else:
            # float32/float64 assumed meters unless user says otherwise
            depth_m = depth.astype(np.float32)
            if not self.depth_in_meters:
                depth_m = depth_m / 1000.0

        H, W = depth_m.shape
        fx = float(self.K[0, 0])
        fy = float(self.K[1, 1])
        cx = float(self.K[0, 2])
        cy = float(self.K[1, 2])

        step = max(1, int(self.step))

        # Subsample grid
        us = np.arange(0, W, step, dtype=np.float32)
        vs = np.arange(0, H, step, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)

        z = depth_m[vv.astype(np.int32), uu.astype(np.int32)]

        # Valid mask
        valid = np.isfinite(z) & (z > self.min_depth) & (z < self.max_depth)

        uu = uu[valid]
        vv = vv[valid]
        z  = z[valid]

        if z.size == 0:
            rospy.logwarn_throttle(1.0, "[depth_to_cloud] No valid depth points in this frame.")
            return

        # Backproject to camera frame
        x = (uu - cx) * z / fx
        y = (vv - cy) * z / fy
        pts = np.stack([x, y, z], axis=1).astype(np.float32)

        # Safety cap (RViz performance)
        if pts.shape[0] > self.max_points:
            pts = pts[: self.max_points, :]

        header = msg.header
        # Choose frame_id
        if self.override_frame_id:
            header.frame_id = self.override_frame_id
        elif self.frame_id:
            header.frame_id = self.frame_id  # from CameraInfo if it existed
        # else: keep msg.header.frame_id (often camera_link)

        cloud_msg = pc2.create_cloud_xyz32(header, pts.tolist())
        self.cloud_pub.publish(cloud_msg)


def main():
    rospy.init_node("depth_to_cloud", anonymous=True)
    DepthToCloud()
    rospy.spin()


if __name__ == "__main__":
    main()
