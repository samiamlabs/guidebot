#!/usr/bin/env python
from __future__ import with_statement
from __future__ import division
from __future__ import unicode_literals

import roslib
import rospy
import rostest
import unittest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from time import sleep
import sys
import threading

from transformations import euler_from_quaternion

from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped

from std_srvs.srv import *

import numpy as np


class TestPathFromPoints(unittest.TestCase):
    def __init__(self, *args):
        super(TestPathFromPoints, self).__init__(*args)

        self._mutex = threading.Lock()

        self._path = Path()

        rospy.init_node('path_from_points_test')
        self._starttime = rospy.get_time()

        self._test_duration = rospy.get_param("~test_duration", 3)
        self._path_received = False

        self._linefollow_subscriber = rospy.Subscriber(
            "linefollow_path", Path, self.path_cb)

        self._point_publisher = rospy.Publisher(
            'clicked_point', PointStamped, queue_size=10)

        rospy.sleep(0)

    def clear_path(self):
        rospy.wait_for_service('clear_linefollow_path')

        try:
            clear_path = rospy.ServiceProxy('clear_linefollow_path', Empty)
            resp = clear_path()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def set_interpolate(self, interpolate):
        rospy.wait_for_service('set_interpolate')

        try:
            set_bool = rospy.ServiceProxy('set_interpolate', SetBool)
            req = SetBoolRequest()
            req.data = interpolate
            resp = set_bool(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def path_cb(self, msg):
        # rospy.logerr("got path, number of poses: %d", len(msg.poses))
        self._path = msg
        self._path_received = True

    def test_all(self):
        self.it_publishes_paths()
        self.it_has_correct_positions()
        self.it_has_correct_angles()
        self.it_interpolates_points()

    def it_publishes_paths(self):
        self.set_interpolate(False)
        self.clear_path()

        point_stamped = create_point_stamped(0, 0)
        self._point_publisher.publish(point_stamped)

        # rospy.sleep(1)

        this_path_received = False
        while not rospy.is_shutdown():
            if self._path_received:
                this_path_received = True
                break

            if rospy.get_time() - self._starttime > self._test_duration:
                break

        self.assertTrue(this_path_received, "No path received")

    def it_has_correct_positions(self):
        rospy.logerr("Starting position test")
        self.clear_path()

        # rospy.sleep(1)

        point_stamped = create_point_stamped(1.0, 0)
        self._point_publisher.publish(point_stamped)

        point_stamped = create_point_stamped(2.0, 3.0)
        self._point_publisher.publish(point_stamped)

        # rospy.sleep(1)

        this_path_received = False
        while not rospy.is_shutdown():
            if self._path_received and len(self._path.poses) == 2:
                this_path_received = True
                break

            if rospy.get_time() - self._starttime > self._test_duration:
                break

        self.assertTrue(this_path_received, "No path received")

        self.assertAlmostEquals(
            self._path.poses[0].pose.position.x, 1.0, 7,
            "First position x is not 2.0")

        self.assertAlmostEquals(
            self._path.poses[1].pose.position.x, 2.0, 7,
            "Second position x is not 2.0")

        self.assertAlmostEquals(
            self._path.poses[1].pose.position.y, 3.0, 7,
            "Second position y is not 3.0")

    def it_has_correct_angles(self):
        rospy.logerr("Starting angle test")
        self.clear_path()

        point_list = []

        point_stamped = create_point_stamped(0, 0)
        point_list.append(point_stamped)

        # Up
        point_stamped = create_point_stamped(1, 0)
        point_list.append(point_stamped)

        # Left
        point_stamped = create_point_stamped(1, 1)
        point_list.append(point_stamped)

        # Down
        point_stamped = create_point_stamped(0, 1)
        point_list.append(point_stamped)

        # Right
        point_stamped = create_point_stamped(0, 0)
        point_list.append(point_stamped)

        # Up Left
        point_stamped = create_point_stamped(1, 1)
        point_list.append(point_stamped)

        # Up Right
        point_stamped = create_point_stamped(2, 0)
        point_list.append(point_stamped)

        # Down Right
        point_stamped = create_point_stamped(1, -1)
        point_list.append(point_stamped)

        # Down Left
        point_stamped = create_point_stamped(0, 0)
        point_list.append(point_stamped)

        for point in point_list:
            self._point_publisher.publish(point)

        number_of_points = len(point_list)

        this_path_received = False
        while not rospy.is_shutdown():
            if self._path_received and len(self._path.poses) == number_of_points:
                this_path_received = True
                break

            if rospy.get_time() - self._starttime > self._test_duration:
                break

        self.assertTrue(this_path_received, "No path received")

        orientation = self._path.poses[0].pose.orientation
        yaw = yaw_from_orientation(orientation)

        self.assertAlmostEquals(yaw, 0, 7, "Going up")

        orientation = self._path.poses[1].pose.orientation
        yaw = yaw_from_orientation(orientation)

        self.assertAlmostEquals(yaw, 90, 7, "Going left")

        orientation = self._path.poses[2].pose.orientation
        yaw = yaw_from_orientation(orientation)

        self.assertAlmostEquals(np.linalg.norm(yaw), 180, 7, "Going down")

        orientation = self._path.poses[3].pose.orientation
        yaw = yaw_from_orientation(orientation)

        self.assertAlmostEquals(yaw, -90, 7, "Going right")

        orientation = self._path.poses[4].pose.orientation
        yaw = yaw_from_orientation(orientation)

        self.assertAlmostEquals(yaw, 45, 7, "Going up left")

        orientation = self._path.poses[5].pose.orientation
        yaw = yaw_from_orientation(orientation)

        self.assertAlmostEquals(yaw, -45, 7, "Going up right")

        orientation = self._path.poses[6].pose.orientation
        yaw = yaw_from_orientation(orientation)

        self.assertAlmostEquals(yaw, -135, 7, "Going down right")

        orientation = self._path.poses[7].pose.orientation
        yaw = yaw_from_orientation(orientation)

        self.assertAlmostEquals(yaw, 135, 7, "Going down left")

    def it_interpolates_points(self):
        rospy.logerr("Starting interpolation test")
        self.set_interpolate(True)
        self.clear_path()

        point_stamped = create_point_stamped(0.0, 0)
        self._point_publisher.publish(point_stamped)

        point_stamped = create_point_stamped(1.0, 0.0)
        self._point_publisher.publish(point_stamped)

        point_stamped = create_point_stamped(1.0, 1.0)
        rospy.logerr("last point sent")
        self._point_publisher.publish(point_stamped)

        this_path_received = False
        while not rospy.is_shutdown():
            if self._path_received and len(self._path.poses) > 2 * 10 - 1:
                this_path_received = True
                break

            if rospy.get_time() - self._starttime > self._test_duration:
                break

        # rospy.logerr("num poses %d", len(self._path.poses))
        self.assertTrue(this_path_received, "No path received")

        for pose_stamped in self._path.poses:
            if(np.linalg.norm(pose_stamped.pose.position.x - 1.0) < 0.1):
                message = "y at x=1.0 is: " + str(pose_stamped.pose.position.y)
                close_enough = np.linalg.norm(pose_stamped.pose.position.y - 0.0) < 0.1
                self.assertTrue(close_enough, message)
                break


def yaw_from_orientation(orientation):
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w)
    euler = euler_from_quaternion(quaternion)
    return euler[2] * 180 / np.pi


def create_point_stamped(x=0, y=0, z=0, frame_id="map"):
    point_stamped = PointStamped()
    point_stamped.header.stamp = rospy.get_rostime()
    point_stamped.header.frame_id = frame_id
    point_stamped.header.seq = 0

    point_stamped.point.x = x
    point_stamped.point.y = y
    point_stamped.point.z = z

    return point_stamped


if __name__ == '__main__':

    rostest.run("guidebot_navigation",
                "test_path_from_points",
                TestPathFromPoints)
