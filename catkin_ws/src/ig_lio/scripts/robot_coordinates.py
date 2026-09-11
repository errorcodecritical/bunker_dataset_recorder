#!/usr/bin/env python3

import rospy
import tf2_ros


def main():
    rospy.init_node("show_robot_coordinates")

    reference_frame = rospy.get_param("~reference_frame", "odom")
    robot_frame = rospy.get_param("~robot_frame", "base_link")
    rate_hz = rospy.get_param("~rate", 2.0)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.loginfo(
        "Showing position of '%s' in '%s' frame",
        robot_frame,
        reference_frame
    )

    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        try:
            transform = tf_buffer.lookup_transform(
                reference_frame,
                robot_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            rospy.loginfo(
                "Robot position in %s: x=%.3f, y=%.3f, z=%.3f",
                reference_frame,
                x,
                y,
                z
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ) as error:
            rospy.logwarn_throttle(
                2.0,
                "Cannot get transform %s -> %s: %s",
                reference_frame,
                robot_frame,
                str(error)
            )

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
