#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker


def marker_publisher():
    # Initialize the ROS node
    rospy.init_node('marker_publisher')

    # Create a publisher for the marker
    marker_publisher = rospy.Publisher('marker_topic', Marker, queue_size=10)

    # Set the rate at which to publish the marker
    rate = rospy.Rate(10)

    # Initialize the marker message
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.05
    marker.scale.z = 0.01
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    # Set the marker's initial position
    marker.pose.position.x = -1.14
    marker.pose.position.y = 0.33
    marker.pose.position.z = 0.75

    # Set the marker's initial orientation
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.707
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 0.707

    # Loop and publish the marker
    while not rospy.is_shutdown():
        marker_publisher.publish(marker)
        rate.sleep()


if __name__ == '__main__':
    try:
        marker_publisher()
    except rospy.ROSInterruptException:
        print('AAAa')
