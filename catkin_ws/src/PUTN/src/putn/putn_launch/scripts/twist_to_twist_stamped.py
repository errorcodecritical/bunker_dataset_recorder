#!/usr/bin/env python
"""
twist_to_twist_stamped.py

Converts geometry_msgs/Twist from PUTN (/cmd_vel)
to geometry_msgs/TwistStamped on /ros1/cmd_vel_stamped.
"""
import rospy
from geometry_msgs.msg import Twist, TwistStamped

def callback(msg):
    stamped = TwistStamped()
    stamped.header.stamp = rospy.Time.now()
    stamped.header.frame_id = "base_link"
    stamped.twist = msg
    pub.publish(stamped)

rospy.init_node('twist_to_twist_stamped')
pub = rospy.Publisher('/ros1/cmd_vel_stamped', TwistStamped, queue_size=10)
rospy.Subscriber('/cmd_vel', Twist, callback)
rospy.loginfo("twist_to_twist_stamped: /cmd_vel -> /ros1/cmd_vel_stamped")
rospy.spin()