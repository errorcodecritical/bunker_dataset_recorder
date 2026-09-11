#!/usr/bin/env python3
"""
depth_to_cloud.py  (ROS1 / Noetic)

Subscribes:
  - /camera/aligned_depth_to_color/image_raw   (sensor_msgs/Image, encoding 32FC1, meters)
  - /camera/aligned_depth_to_color/camera_info (sensor_msgs/CameraInfo)

Publishes:
  - /debug_depth_cloud (sensor_msgs/PointCloud2)  [xyz in camera optical frame]

Use RViz:
  Fixed Frame: camera_color_optical_frame (or whatever the depth image header frame_id is)
  Add -> PointCloud2 : /debug_depth_cloud
  Add -> Path        : /debug_projected_path   (your existing debug path)
"""

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge


class DepthToCloud:
    def __init__(self):
        self.bridge = CvBridge()
        self.K = None  # 3x3 intrinsics
        self.frame_id = None

        # Params
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.info_topic  = rospy.get_param("~camera_info_topic", "/camera/aligned_depth_to_color/camera_info")
        self.pub_topic   = rospy.get_param("~cloud_topic", "/debug_depth_cloud")

        self.step = int(rospy.get_param("~step", 4))  # subsample factor (1=all pixels, 4=every 4th pixel)
        self.min_depth = float(rospy.get_param("~min_depth", 0.2))
        self.max_depth = float(rospy.get_param("~max_depth", 30.0))
        self.max_points = int(rospy.get_param("~max_points", 250000))  # safety cap

        self.cloud_pub = rospy.Publisher(self.pub_topic, PointCloud2, queue_size=1)

        rospy.Subscriber(self.info_topic, CameraInfo, self.cb_info, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1)

        rospy.loginfo(f"[depth_to_cloud] Subscribing depth: {self.depth_topic}")
        rospy.loginfo(f"[depth_to_cloud] Subscribing camera_info: {self.info_topic}")
        rospy.loginfo(f"[depth_to_cloud] Publishing cloud: {self.pub_topic}")
        rospy.loginfo(f"[depth_to_cloud] step={self.step} depth_range=[{self.min_depth},{self.max_depth}] max_points={self.max_points}")

    def cb_info(self, msg: CameraInfo):
        # msg.K is length-9 row-major
        K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.K = K
        # Prefer camera_info frame, but depth header is fine too
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id

    def cb_depth(self, msg: Image):
        if self.K is None:
            rospy.logwarn_throttle(1.0, "[depth_to_cloud] Waiting for CameraInfo...")
            return

        # Depth should be 32FC1 meters (your SynPhoRest bag is 32FC1)
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            rospy.logerr(f"[depth_to_cloud] cv_bridge failed: {e}")
            return

        H, W = depth.shape[:2]
        fx = float(self.K[0, 0])
        fy = float(self.K[1, 1])
        cx = float(self.K[0, 2])
        cy = float(self.K[1, 2])

        step = max(1, self.step)

        # Subsample grid
        us = np.arange(0, W, step, dtype=np.float32)
        vs = np.arange(0, H, step, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)

        z = depth[vv.astype(np.int32), uu.astype(np.int32)].astype(np.float32)

        # Valid mask
        valid = np.isfinite(z) & (z > self.min_depth) & (z < self.max_depth)

        uu = uu[valid]
        vv = vv[valid]
        z  = z[valid]

        if z.size == 0:
            rospy.logwarn_throttle(1.0, "[depth_to_cloud] No valid depth points in this frame.")
            return

        # Backproject to camera optical frame
        x = (uu - cx) * z / fx
        y = (vv - cy) * z / fy

        pts = np.stack([x, y, z], axis=1)

        # Safety cap (RViz performance)
        if pts.shape[0] > self.max_points:
            pts = pts[: self.max_points, :]

        header = msg.header
        if self.frame_id:
            header.frame_id = self.frame_id  # keep consistent with camera_info if available

        cloud_msg = pc2.create_cloud_xyz32(header, pts.tolist())
        self.cloud_pub.publish(cloud_msg)


def main():
    rospy.init_node("depth_to_cloud", anonymous=True)
    DepthToCloud()
    rospy.spin()


if __name__ == "__main__":
    main()
