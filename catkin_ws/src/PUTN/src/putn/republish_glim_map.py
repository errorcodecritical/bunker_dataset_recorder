#!/usr/bin/env python3
"""Simple republisher for /glim_ros/map to ensure a regular publish rate for PUTN.

Subscribes to a map topic (OccupancyGrid or PointCloud2) and republishes
the last received message at a fixed rate. Updates header.stamp on each
published message so downstream consumers see fresh timestamps.
"""
import rospy
import copy
from threading import Lock

lock = Lock()
last_msg = None
pub = None


def cb(msg):
    global last_msg, pub
    with lock:
        last_msg = msg
        if pub is None:
            # create a latched publisher using the same message type
            pub = rospy.Publisher(rospy.get_param('~output_topic', '/glim_ros/map_republished'),
                                  msg.__class__, queue_size=1, latch=True)
            # immediately publish once when first received
            out = copy.deepcopy(last_msg)
            out.header.stamp = rospy.Time.now()
            pub.publish(out)


def timer_event(event):
    global last_msg, pub
    with lock:
        if last_msg is not None and pub is not None:
            out = copy.deepcopy(last_msg)
            out.header.stamp = rospy.Time.now()
            pub.publish(out)


def main():
    rospy.init_node('glim_map_republisher', anonymous=False)
    input_topic = rospy.get_param('~input_topic', '/glim_ros/map')
    rate_hz = float(rospy.get_param('~rate', 1.0))
    map_type = rospy.get_param('~map_type', 'occupancy')

    if map_type == 'occupancy':
        from nav_msgs.msg import OccupancyGrid as MapMsg
    else:
        from sensor_msgs.msg import PointCloud2 as MapMsg

    rospy.Subscriber(input_topic, MapMsg, cb, queue_size=1)
    rospy.Timer(rospy.Duration(1.0 / rate_hz), timer_event)
    rospy.loginfo('glim_map_republisher: subscribed %s -> publishing %s at %.2f Hz',
                  input_topic, rospy.get_param('~output_topic', '/glim_ros/map_republished'), rate_hz)
    rospy.spin()


if __name__ == '__main__':
    main()
