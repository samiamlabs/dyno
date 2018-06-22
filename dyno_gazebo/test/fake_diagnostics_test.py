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

OK = 0
WARN = 1
ERROR = 2
STALE = 3


def get_raw_name(name):
    return name.split(':')[-1]


class DiagnosticItem:
    def __init__(self, msg):
        self.name = get_raw_name(msg.name)
        self.level = msg.level
        self.message = msg.message

        self.update_time = rospy.get_time()

    def is_stale(self):
        return rospy.get_time() - self.update_time > 5

    def update(self, msg):
        self.level = msg.level
        self.message = msg.message

        self.update_time = rospy.get_time()


class TestFakeDiagnostics(unittest.TestCase):
    def __init__(self, *args):
        super(TestFakeDiagnostics, self).__init__(*args)

        self._mutex = threading.Lock()

        self._expecteds = {}

        rospy.init_node('test_fake_diagnostics')
        self._starttime = rospy.get_time()

        self.test_duration = rospy.get_param("~test_duration", 3)
        # rospy.logwarn(self.test_duration)

        sub = rospy.Subscriber("/diagnostics", DiagnosticArray, self.diag_cb)

    def diag_cb(self, msg):
        with self._mutex:
            for stat in msg.status:
                self._expecteds[get_raw_name(stat.name)] = DiagnosticItem(stat)

    def test_diagnostics_received(self):
        while not rospy.is_shutdown():
            if rospy.get_time() - self._starttime > self.test_duration:
                break

        with self._mutex:
            self.assertTrue(len(self._expecteds) > 0,
                            "No items found in raw data!")

    def test_fake_scan_diagnostics(self):
        while not rospy.is_shutdown():
            if rospy.get_time() - self._starttime > self.test_duration:
                break

        with self._mutex:
            self.assertTrue(" scan topic status" in self._expecteds,
                            "scan frequency diagnostic not received")

            if(" scan topic status" in self._expecteds):
                item = self._expecteds[" scan topic status"]

                self.assertEquals(
                    item.level,
                    OK,
                    "Laser Scanner frequency not not within tolerance")

            self.assertTrue(" SweepLidar Connection" in self._expecteds,
                            "Laser Scanner connection diagnostic not received")

    def test_fake_imu_diagnostics(self):
        while not rospy.is_shutdown():
            if rospy.get_time() - self._starttime > self.test_duration:
                break

        with self._mutex:
            self.assertTrue(" imu topic status" in self._expecteds,
                            "imu frequency diagnostic not received")

            if(" imu topic status" in self._expecteds):
                item = self._expecteds[" imu topic status"]

                self.assertEquals(
                    item.level,
                    OK,
                    "imu frequency not not within tolerance")

    def test_fake_battery_diagnostics(self):
        while not rospy.is_shutdown():
            if rospy.get_time() - self._starttime > self.test_duration:
                break

        with self._mutex:
            self.assertTrue(" Battery level" in self._expecteds,
                            "Battery level not received")

            if(" Battery level" in self._expecteds):
                item = self._expecteds[" Battery level"]

                self.assertEquals(
                    item.level,
                    OK,
                    "Battery level low")


if __name__ == '__main__':

    rostest.run("guidebot_gazebo",
                "test_fake_diagnostics",
                TestFakeDiagnostics)
