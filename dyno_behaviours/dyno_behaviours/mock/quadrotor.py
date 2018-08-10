#!/usr/bin/env python
#
# License: BSD
# https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
#
##############################################################################
# Documentation
##############################################################################

"""
Mocks the pick up parcel action for dyno quadrotor
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees_ros
import rospy
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import dyno_msgs.msg as dyno_msgs
from py_trees_ros.mock import action_server

##############################################################################
# Classes
##############################################################################


class PickUpParcel(action_server.ActionServer):
    """
    Simulates:

    * picking up parcel
    * publishing on /cmd_vel
    * publishing on /pose

    Args:
        duration (:obj: `int`): time for a goal to complete (seconds)
    """
    def __init__(self, pose_topic='/pick_up_parcel/pose', twist_topic='/pick_up_parcel/cmd_vel', duration=None):
        super(PickUpParcel, self).__init__(action_name="pick_up_parcel",
                                                    action_type=dyno_msgs.QuadrotorPickUpParcelAction,
                                                    worker=self.worker,
                                                    duration=duration
                                                    )

        self.initial_height = 1.0
        self.goal_height = 0.2

        self.pose = geometry_msgs.Pose()
        self.pose.position.z = self.initial_height

        self.twist = geometry_msgs.Twist()

        latched = True
        queue_size_five = 1
        self.publishers = py_trees_ros.utilities.Publishers(
            [
                ('pose', pose_topic, geometry_msgs.Pose, latched, queue_size_five),
                ('twist', twist_topic, geometry_msgs.Twist, latched, queue_size_five)
            ]
        )

        # self.publishers.pose.publish(self.pose)
        # self.publishers.twist.publish(self.twist)
        self.publishing_timer = rospy.Timer(period=rospy.Duration(0.5), callback=self.publish, oneshot=False)
        self.start()

    def worker(self):
        """
        Lower pose towards the goal and go back up
        """
        percent_completed = self.percent_completed * 0.01

        previous_z = self.pose.position.z

        if (percent_completed < 0.5):
            self.pose.position.z = self.initial_height - 2.0 * percent_completed * (self.initial_height - self.goal_height)
        else:
            self.pose.position.z = self.initial_height - 2.0 * (1 - percent_completed) * (self.initial_height - self.goal_height)

        dt = 1 / 3.0
        self.twist.linear.z = (self.pose.position.z - previous_z) * dt

        self.action.action_feedback = self.pose.position.z

    def publish(self, unused_event):
        self.publishers.pose.publish(self.pose)
        self.publishers.twist.publish(self.twist)


class DropOffParcel(action_server.ActionServer):
    """
    Simulates:

    * dropping off parcel
    * publishing on /cmd_vel
    * publishing on /pose

    Args:
        duration (:obj: `int`): time for a goal to complete (seconds)
    """
    def __init__(self, pose_topic='/drop_off_parcel/pose', twist_topic='/dorp_off_parcel/cmd_vel', duration=None):
        super(DropOffParcel, self).__init__(action_name="drop_off_parcel",
                                                    action_type=dyno_msgs.QuadrotorDropOffParcelAction,
                                                    worker=self.worker,
                                                    duration=duration
                                                    )

        self.initial_height = 1.0
        self.goal_height = 0.2

        self.pose = geometry_msgs.Pose()
        self.pose.position.z = self.initial_height

        self.twist = geometry_msgs.Twist()

        latched = True
        queue_size_five = 1
        self.publishers = py_trees_ros.utilities.Publishers(
            [
                ('pose', pose_topic, geometry_msgs.Pose, latched, queue_size_five),
                ('twist', twist_topic, geometry_msgs.Twist, latched, queue_size_five)
            ]
        )

        # self.publishers.pose.publish(self.pose)
        # self.publishers.twist.publish(self.twist)
        self.publishing_timer = rospy.Timer(period=rospy.Duration(0.5), callback=self.publish, oneshot=False)
        self.start()

    def worker(self):
        """
        Lower pose towards the goal and go back up
        """
        percent_completed = self.percent_completed * 0.01

        previous_z = self.pose.position.z
        if (percent_completed < 0.5):
            self.pose.position.z = self.initial_height - 2.0 * percent_completed * (self.initial_height - self.goal_height)
        else:
            self.pose.position.z = self.initial_height - 2.0 * (1 - percent_completed) * (self.initial_height - self.goal_height)

        dt = 1 / 3.0
        self.twist.linear.z = (self.pose.position.z - previous_z) * dt

        self.action.action_feedback = self.pose.position.z
    def publish(self, unused_event):
        self.publishers.pose.publish(self.pose)
        self.publishers.twist.publish(self.twist)
