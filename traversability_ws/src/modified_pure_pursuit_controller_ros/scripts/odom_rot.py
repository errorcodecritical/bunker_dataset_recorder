#!/usr/bin/env python

import rospy
import math

from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_multiply


class GlimOdomRepublisher(object):

    def __init__(self):

        self.pub = rospy.Publisher(
            "/odom",
            Odometry,
            queue_size=20
        )

        self.sub = rospy.Subscriber(
            "/glim_ros/odom",
            Odometry,
            self.callback,
            queue_size=20
        )

        # IMU is +90 degrees CCW relative to base_link.
        #
        # Therefore:
        #
        # yaw_base_link = yaw_imu - 90 degrees
        #
        self.correction_q = quaternion_from_euler(
            0.0,
            0.0,
            -math.pi / 2.0
        )

        rospy.loginfo(
            "GLIM odometry correction: IMU -> base_link = -90 deg yaw"
        )

    def rotate_xy_minus_90(self, x, y):

        # Rotation by -90 degrees:
        #
        # [ x' ]   [  0  1 ] [ x ]
        # [ y' ] = [ -1  0 ] [ y ]
        #
        return y, -x

    def callback(self, msg):

        out = Odometry()

        # --------------------------------------------------
        # Header
        # --------------------------------------------------

        out.header = msg.header
        out.header.frame_id = "odom"

        # --------------------------------------------------
        # Child frame
        # --------------------------------------------------

        out.child_frame_id = "base_link"

        # --------------------------------------------------
        # Position
        #
        # Keep the GLIM odom/world position.
        #
        # This is intentional: we are correcting the BODY
        # coordinate convention, not rotating the world frame.
        # --------------------------------------------------

        out.pose.pose.position.x = msg.pose.pose.position.x
        out.pose.pose.position.y = msg.pose.pose.position.y
        out.pose.pose.position.z = msg.pose.pose.position.z

        # --------------------------------------------------
        # Orientation
        #
        # q_odom_base =
        #     q_odom_imu * q_imu_base
        #
        # q_imu_base = rotation of -90 degrees.
        # --------------------------------------------------

        q_imu = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        q_base = quaternion_multiply(
            q_imu,
            self.correction_q
        )

        out.pose.pose.orientation.x = q_base[0]
        out.pose.pose.orientation.y = q_base[1]
        out.pose.pose.orientation.z = q_base[2]
        out.pose.pose.orientation.w = q_base[3]

        # --------------------------------------------------
        # Linear velocity
        #
        # GLIM velocity is expressed in the IMU body frame.
        # Rotate it -90 degrees so +X is forward in base_link.
        # --------------------------------------------------

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        vx_base, vy_base = self.rotate_xy_minus_90(vx, vy)

        out.twist.twist.linear.x = vx_base
        out.twist.twist.linear.y = vy_base
        out.twist.twist.linear.z = vz

        # --------------------------------------------------
        # Angular velocity
        # --------------------------------------------------

        wx = msg.twist.twist.angular.x
        wy = msg.twist.twist.angular.y
        wz = msg.twist.twist.angular.z

        wx_base, wy_base = self.rotate_xy_minus_90(wx, wy)

        out.twist.twist.angular.x = wx_base
        out.twist.twist.angular.y = wy_base
        out.twist.twist.angular.z = wz

        # --------------------------------------------------
        # Covariance
        # --------------------------------------------------

        out.pose.covariance = msg.pose.covariance
        out.twist.covariance = msg.twist.covariance

        self.pub.publish(out)


if __name__ == "__main__":

    rospy.init_node("glim_odom_to_base_link")

    GlimOdomRepublisher()

    rospy.spin()