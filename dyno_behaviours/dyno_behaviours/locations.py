#!/usr/bin/env python
#
# License: BSD
#  https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
#
##############################################################################
# Documentation
##############################################################################

"""
Getting the most out of your world state locations.
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import rospy
import dyno_world_state.msg as dyno_world_state

from py_trees_ros import subscribers

##############################################################################
# Behaviours
##############################################################################


class ToBlackboard(subscribers.ToBlackboard):
    """
    Subscribes to the location message and writes location data to the blackboard.
    Also adds a warning flag to the blackboard if there are no
    locations. When ticking, updates with :attr:`~py_trees.common.Status.RUNNING` if it got no data,
    :attr:`~py_trees.common.Status.SUCCESS` otherwise.
    Blackboard Variables:
        * locations (:class:`dyno_world_state.msg.Locations`)[w]: The available locations
        * no_locations_warning (:obj:`bool`)[w]: False there is more than one location, True not
    Args:
        name (:obj:`str`): name of the behaviour
        topic_name (:obj:`str`) : name of the locations topic
    """
    def __init__(self, name, topic_name="/world_state/locations"):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=dyno_world_state.LocationArray,
                                           blackboard_variables={"location_array": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.NEVER
                                           )

        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.location_array = dyno_world_state.LocationArray()
        self.blackboard.no_locations_warning = True   # decision making

    def update(self):
        """
        Call the parent to write the raw data to the blackboard and then check
        to determine if the no locations warning flag should also be updated.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            # we got something
            if len(self.blackboard.location_array.locations) > 0:
                self.blackboard.no_locations_warning = False
            else:
                self.blackboard.no_locations_warning = True

            self.feedback_message = "No locations found" if self.blackboard.no_locations_warning else "Locations are ok"
        return status
