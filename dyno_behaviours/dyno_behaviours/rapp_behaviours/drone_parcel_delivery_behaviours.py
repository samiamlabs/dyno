#!/usr/bin/env python
#
# License: BSD
# https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
#
##############################################################################
# Documentation
##############################################################################

"""
A few behaviours for the drone_parcel_delivery rapp.
"""

##############################################################################
# Imports
##############################################################################

import dynamic_reconfigure.client
import py_trees
import py_trees_ros
import rospy
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

##############################################################################
# Behaviours
##############################################################################


class ClearDeliveryQueue(py_trees.behaviour.Behaviour):
    """
    This behavoiur clears the location queue and returns
    :attr:`~py_trees.common.Status.SUCCESS`
    Args:
        name (:obj:`str`): name of the behaviour
    """

    def __init__(self, name):
        super(ClearDeliveryQueue, self).__init__(name=name)

    def setup(self, timeout):
        """
        Args:
            timeout (:obj:`float`): time to wait (0.0 is blocking forever)
        Returns:
            :obj:`bool`: whether it timed out trying to setup
        """
        self.feedback_message = "setup"
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def update(self):
        """
        This behaviour
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)

        self.blackboard.delivery_queue = []
        self.blackboard.should_move = False

        self.feedback_message = "clearing delivery queue"
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status
        """
        self.feedback_message = "cleared"


class RemoveFirstDeliveryFromQueue(py_trees.behaviour.Behaviour):
    """
    This behavoiur removes the delivery at the head of the queue and returns
    :attr:`~py_trees.common.Status.SUCCESS`
    Args:
        name (:obj:`str`): name of the behaviour
    """

    def __init__(self, name):
        super(RemoveFirstDeliveryFromQueue, self).__init__(name=name)

    def setup(self, timeout):
        """
        Args:
            timeout (:obj:`float`): time to wait (0.0 is blocking forever)
        Returns:
            :obj:`bool`: whether it timed out trying to setup
        """
        self.feedback_message = "setup"
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def update(self):
        """
        This behaviour
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)

        delivery = self.blackboard.delivery_queue.pop(0)
        object_name = delivery.object_name
        location_name = delivery.location_name

        self.feedback_message = "Removing delivery: " + object_name + ">>" + location_name + " from queue"

        if (len(self.blackboard.delivery_queue) == 0):
            self.blackboard.empty_queue_warning = True
            self.blackboard.should_move = False

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status
        """
        self.feedback_message = "removed"


class MoveToNextLocation(py_trees_ros.actions.ActionClient):
    def initialise(self):
        """
        Reset the internal variables.
        """
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))

        blackboard = py_trees.blackboard.Blackboard()
        known_locations = blackboard.location_array.locations
        known_location_names = list(map(lambda location: location.name, known_locations))

        delivery_queue = blackboard.delivery_queue

        location_index = known_location_names.index(delivery_queue[0].location_name)

        pose_stamped = geometry_msgs.PoseStamped()
        pose_stamped.pose = known_locations[location_index].pose
        pose_stamped.header.frame_id = "map"

        self.action_goal.target_pose = pose_stamped
        self.sent_goal = False


class MoveToNextObject(py_trees_ros.actions.ActionClient):
    def initialise(self):
        """
        Reset the internal variables.
        """
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))

        blackboard = py_trees.blackboard.Blackboard()
        known_objects = blackboard.object_array.objects
        known_object_names = list(map(lambda object: object.name, known_objects))

        delivery_queue = blackboard.delivery_queue

        object_index = known_object_names.index(delivery_queue[0].object_name)

        pose_stamped = geometry_msgs.PoseStamped()
        pose_stamped.pose = known_objects[object_index].pose
        pose_stamped.header.frame_id = "map"

        self.action_goal.target_pose = pose_stamped
        self.sent_goal = False


class PickUpParcel(py_trees_ros.actions.ActionClient):
    def initialise(self):
        """
        Reset the internal variables.
        """
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))

        blackboard = py_trees.blackboard.Blackboard()

        delivery_queue = blackboard.delivery_queue
        self.action_goal.object_name = delivery_queue[0].object_name
        self.sent_goal = False


class DropOffParcel(py_trees_ros.actions.ActionClient):
    def initialise(self):
        """
        Reset the internal variables.
        """
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))

        blackboard = py_trees.blackboard.Blackboard()

        delivery_queue = blackboard.delivery_queue
        self.action_goal.object_name = delivery_queue[0].object_name
        self.sent_goal = False
