#!/usr/bin/env python
#
# License: BSD
#  https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
#
##############################################################################
# Documentation
##############################################################################

"""
Adding delivery to blackboard
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
    Subscribes to the add_delivery and appends a delivery to the
    blackboard if both object and location exsist.
    When ticking, updates with :attr:`~py_trees.common.Status.RUNNING` if it got no data,
    :attr:`~py_trees.common.Status.SUCCESS` otherwise.
    Blackboard Variables:
        * last_delivery (:class:`dyno_msgs.msg.Delivery`)[w]: The last added delivery
        * delivery_queue
        * delivery_queue_empty (:obj:`bool`)[w]: False there is more than one delivery in queue, True if not
    Args:
        name (:obj:`str`): name of the behaviour
        topic_name (:obj:`str`) : name of the delivery topic
    """

    def __init__(self, name, topic_name="/drone_parcel_delivery/add_delivery"):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=dyno_msgs.Delivery,
                                           blackboard_variables={
                                               "added_delivery": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.ON_SUCCESS
                                           )

        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.added_delivery = None
        self.blackboard.delivery_queue = []
        self.blackboard.empty_queue_warning = True   # decision making

        self.delivery_queue_publisher = rospy.Publisher("/drone_parcel_delivery/delivery_queue", dyno_msgs.DeliveryArray, queue_size=5);

    def update(self):
        """
        Call the parent to write the raw data to the blackboard and then check
        to determine if the empty queue warning flag should also be updated.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            # we got something

            added_delivery = self.blackboard.added_delivery

            known_locations = self.blackboard.location_array.locations
            known_location_names = list(map(lambda location: location.name, known_locations))

            known_objects = self.blackboard.object_array.objects
            known_object_names = list(map(lambda object: object.name, known_objects))

            delivery_queue = self.blackboard.delivery_queue

            location_has_name = added_delivery.location_name != ""
            location_is_known = added_delivery.location_name in known_location_names

            object_has_name = added_delivery.object_name != ""
            object_is_known = added_delivery.object_name in known_object_names

            has_names = location_has_name and object_has_name
            is_known = location_is_known and object_is_known

            queue_is_empty = len(delivery_queue) == 0
            if queue_is_empty:
                is_different_from_last_added = True
            else:
                is_location_different = delivery_queue[-1].location_name != added_delivery.location_name
                is_object_different = delivery_queue[-1].object_name != added_delivery.object_name
                is_different_from_last_added = is_location_different or is_object_different

            if (has_names and is_known and is_different_from_last_added):
                self.blackboard.delivery_queue.append(added_delivery)

            if self.blackboard.delivery_queue < 1:
                self.blackboard.empty_delivery_queue_warning = True
            else:
                self.blackboard.empty_delivery_queue_warning = False

        self.blackboard.empty_queue_warning = len(self.blackboard.delivery_queue) == 0

        self.feedback_message = "No deliveries in queue" if self.blackboard.empty_queue_warning else "Locations queue ok"

        delivery_queue_msg = dyno_msgs.DeliveryArray()

        for delivery in self.blackboard.delivery_queue:
            delivery_msg = dyno_msgs.Delivery()
            delivery_msg.location_name = delivery.location_name
            delivery_msg.object_name = delivery.object_name
            delivery_queue_msg.deliveries.append(delivery_msg)

        self.delivery_queue_publisher.publish(delivery_queue_msg)

        status = py_trees.common.Status.SUCCESS
        return status
