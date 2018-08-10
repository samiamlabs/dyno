#!/usr/bin/env python
#
# License: BSD
#  https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
#
##############################################################################
# Documentation
##############################################################################

"""
Getting the most out of your world state objects.
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import rospy
import dyno_msgs.msg as dyno_msgs

from py_trees_ros import subscribers

##############################################################################
# Behaviours
##############################################################################


class ToBlackboard(subscribers.ToBlackboard):
    """
    Subscribes to the object message and writes object data to the blackboard.
    Also adds a warning flag to the blackboard if there are no
    objects. When ticking, updates with :attr:`~py_trees.common.Status.RUNNING` if it got no data,
    :attr:`~py_trees.common.Status.SUCCESS` otherwise.
    Blackboard Variables:
        * objects (:class:`dyno_msgs.msg.Objects`)[w]: The available objects
        * no_objects_warning (:obj:`bool`)[w]: False there is more than one object, True not
    Args:
        name (:obj:`str`): name of the behaviour
        topic_name (:obj:`str`) : name of the object topic
    """
    def __init__(self, name, topic_name="/world_state/objects"):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=dyno_msgs.ObjectArray,
                                           blackboard_variables={"object_array": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.NEVER
                                           )

        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.object_array = dyno_msgs.ObjectArray()
        self.blackboard.no_objects_warning = True   # decision making

    def update(self):
        """
        Call the parent to write the raw data to the blackboard and then check
        to determine if the no objects warning flag should also be updated.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            # we got something
            if len(self.blackboard.object_array.objects) > 0:
                self.blackboard.no_objects_warning = False
            else:
                self.blackboard.no_objects_warning = True

            self.feedback_message = "No objects found" if self.blackboard.no_objects_warning else "Locations are ok"
        return status
