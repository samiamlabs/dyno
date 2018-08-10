#!/usr/bin/env python
#
# License: BSD
#  https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
#
##############################################################################
# Documentation
##############################################################################

"""
Adding location to blackboard
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import rospy
import std_msgs.msg as std_msgs
import dyno_msgs.msg as dyno_msgs

from py_trees_ros import subscribers

##############################################################################
# Behaviours
##############################################################################


class ToBlackboard(subscribers.ToBlackboard):
    """
    Subscribes to the add_location message and appends a location to the
    blackboard if it exists in locations.
    When ticking, updates with :attr:`~py_trees.common.Status.RUNNING` if it got no data,
    :attr:`~py_trees.common.Status.SUCCESS` otherwise.
    Blackboard Variables:
        * last_location (:class:`std_msgs.msg.String`)[w]: The last added location
        * location_queue
        * location_queue_empty (:obj:`bool`)[w]: False there is more than one location in queue, True if not
    Args:
        name (:obj:`str`): name of the behaviour
        topic_name (:obj:`str`) : name of the locations topic
    """

    def __init__(self, name, topic_name="/route_scheduler/add_location"):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=std_msgs.String,
                                           blackboard_variables={
                                               "added_location": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.ON_SUCCESS
                                           )

        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.added_location = ""
        self.blackboard.location_queue = []
        self.blackboard.empty_queue_warning = True   # decision making

        self.location_queue_publisher = rospy.Publisher("/route_scheduler/location_queue", dyno_msgs.LocationArray, queue_size=5);

    def update(self):
        """
        Call the parent to write the raw data to the blackboard and then check
        to determine if the empty queue warning flag should also be updated.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            # we got something

            added_location = self.blackboard.added_location.data
            known_locations = self.blackboard.location_array.locations
            known_location_names = list(map(lambda location: location.name, known_locations))

            location_queue = self.blackboard.location_queue

            location_has_name = added_location != ""
            location_is_known = added_location in known_location_names

            queue_is_empty = len(location_queue) == 0
            if queue_is_empty:
                is_different_from_last_added = True
            else:
                is_different_from_last_added = location_queue[-1] != added_location

            if (location_has_name and is_different_from_last_added and location_is_known):
                self.blackboard.location_queue.append(added_location)

            if self.blackboard.location_queue < 1:
                self.blackboard.empty_location_queue_warning = True
            else:
                self.blackboard.empty_location_queue_warning = False

        self.blackboard.empty_queue_warning = len(self.blackboard.location_queue) == 0

        self.feedback_message = "No locations in queue" if self.blackboard.empty_queue_warning else "Locations queue ok"

        location_queue_msg = dyno_msgs.LocationArray()

        for location_name in self.blackboard.location_queue:
            location_msg = dyno_msgs.Location()
            location_msg.name = location_name
            location_queue_msg.locations.append(location_msg)

        self.location_queue_publisher.publish(location_queue_msg)

        status = py_trees.common.Status.SUCCESS
        return status
