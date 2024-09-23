#!/usr/bin/env python3

# This pytyon script is used to convert the odometry data from gym to the odometry data that can be used in slam
# The odometry data from gym is dilivered by the topic /odom. But in slam, the odometry data has to be delivered by transform
# The transform is from odom_frame to base_link
import sys

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped

class GymToSlam(Node):

    def __init__(self):
        super().__init__('gym_to_slam')

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)



    def odom_callback(self, msg):
        br = TransformBroadcaster(self)
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom_frame'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        br.sendTransform(t)
