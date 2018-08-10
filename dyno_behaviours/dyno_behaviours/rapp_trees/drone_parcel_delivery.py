#!/usr/bin/env python
#
# License: BSD
# https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
#
##############################################################################
# Imports
##############################################################################

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys

import dyno_behaviours
import move_base_msgs.msg as move_base_msgs
import std_msgs.msg as std_msgs
import dyno_msgs.msg as dyno_msgs

from dyno_behaviours.rapp_behaviours import drone_parcel_delivery_behaviours

##############################################################################
# Behaviours
##############################################################################


def create_root():
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    takes the asynchronicity out of subscription.
    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the root of the tree
    """

    # Behaviours
    root = py_trees.composites.Parallel("Drone Parcel Delivery")

    start2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Start2BB",
        topic_name="/drone_parcel_delivery/start",
        variable_name="event_start_button"
    )

    stop2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Stop2BB",
        topic_name="/drone_parcel_delivery/stop",
        variable_name="event_stop_button"
    )

    clear2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Clear2BB",
        topic_name="/drone_parcel_delivery/clear",
        variable_name="event_clear_button"
    )

    topics2bb = py_trees.composites.Sequence("Topics2BB")

    add_delivery2bb = dyno_behaviours.add_delivery.ToBlackboard(name="AddObject2BB",
                                                                topic_name="/drone_parcel_delivery/add_delivery"
                                                                )

    locations2bb = dyno_behaviours.locations.ToBlackboard(name="Locations2BB",
                                                          topic_name="/world_state/locations"
                                                          )

    objects2bb = dyno_behaviours.objects.ToBlackboard(name="Objects2BB",
                                                          topic_name="/world_state/objects"
                                                          )

    priorities = py_trees.composites.Selector("Priorities")
    idle = py_trees.behaviours.Running(name="Idle")

    location_check = py_trees.meta.success_is_failure(
        py_trees.composites.Selector)(name="Location problem")

    is_queue_empty = py_trees.blackboard.CheckBlackboardVariable(
        name="Queue empty?",
        variable_name='empty_queue_warning',
        expected_value=True
    )

    start = py_trees.meta.success_is_failure(
        py_trees.composites.Sequence)(name="Start")

    is_start_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="Start?",
        variable_name='event_start_button',
        expected_value=True
    )

    set_should_move = py_trees.blackboard.SetBlackboardVariable(
        name="Set should move",
        variable_name='should_move',
        variable_value=True
    )

    stop = py_trees.meta.success_is_running(
        py_trees.composites.Sequence)(name="Stop")

    cancel = py_trees.composites.Sequence(name="Cancel")

    is_stop_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="Stop?",
        variable_name='event_stop_button',
        expected_value=True
    )

    clear_should_move = py_trees.blackboard.SetBlackboardVariable(
        name="Clear should move",
        variable_name='should_move',
        variable_value=False
    )

    move = py_trees.composites.Sequence(name="Move")

    move_and_remove = py_trees.composites.Sequence(name="Move")

    is_move_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="Should move?",
        variable_name='should_move',
        expected_value=True
    )

    move_to_next_object = drone_parcel_delivery_behaviours.MoveToNextObject(
        name="Move to next object",
        action_namespace="/move_base",
        action_spec=move_base_msgs.MoveBaseAction,
        action_goal=move_base_msgs.MoveBaseGoal()
    )

    move_to_next_location = drone_parcel_delivery_behaviours.MoveToNextLocation(
        name="Move to next location",
        action_namespace="/move_base",
        action_spec=move_base_msgs.MoveBaseAction,
        action_goal=move_base_msgs.MoveBaseGoal()
    )

    pick_up_parcel = py_trees_ros.actions.ActionClient(
        name="Pick up parcel",
        action_namespace="/pick_up_parcel",
        action_spec=dyno_msgs.QuadrotorPickUpParcelAction,
        action_goal=dyno_msgs.QuadrotorPickUpParcelGoal()
    )

    drop_off_parcel = py_trees_ros.actions.ActionClient(
        name="Drop off parcel",
        action_namespace="/drop_off_parcel",
        action_spec=dyno_msgs.QuadrotorPickUpParcelAction,
        action_goal=dyno_msgs.QuadrotorPickUpParcelGoal()
    )

    move_or_cancel = py_trees.composites.Selector(name="Move or be canceled")

    clear = py_trees.meta.success_is_failure(
        py_trees.composites.Selector)(name="Clear Queue")

    is_clear_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="Clear?",
        variable_name='event_clear_button',
        expected_value=False
    )
    clear_deliveries_in_queue = drone_parcel_delivery_behaviours.ClearDeliveryQueue(
        name="Clear deliveries from queue")

    remove_first = py_trees.meta.success_is_failure(
        py_trees.composites.Sequence)(name="Remove first")

    remove_delivery_from_queue = drone_parcel_delivery_behaviours.RemoveFirstDeliveryFromQueue(
        name="Remove first delivery from queue")

    # Tree
    root.add_child(topics2bb)
    topics2bb.add_children([locations2bb, objects2bb, start2bb, stop2bb, clear2bb, add_delivery2bb])
    clear.add_children([is_clear_requested, clear_deliveries_in_queue])
    root.add_child(priorities)
    priorities.add_children([is_queue_empty, clear, start, move, idle])
    start.add_children([is_start_requested, set_should_move])
    cancel.add_children([is_stop_requested, clear_should_move])
    move.add_children([is_move_requested, move_or_cancel])
    move_or_cancel.add_children([cancel, move_and_remove])
    move_and_remove.add_children([move_to_next_object, pick_up_parcel, move_to_next_location, drop_off_parcel, remove_delivery_from_queue])
    return root


class SplinteredReality(object):

    def __init__(self):
        self.tree = py_trees_ros.trees.BehaviourTree(create_root())
        self.tree.add_post_tick_handler(self.publish_reality_report)
        self.report_publisher = rospy.Publisher(
            "drone_parcel_delivery/report", std_msgs.String, queue_size=5)

    def setup(self):
        return self.tree.setup(timeout=15)

    def publish_reality_report(self, tree):
        if tree.tip().name == "Queue empty?":
            self.report_publisher.publish("Delivery Queue is empty")
        elif tree.tip().name == "Remove first delivery from queue":
            self.report_publisher.publish("Removing delivery from queue")
        elif tree.tip().name == "Move to next object":
            self.report_publisher.publish("Moving to next object")
        elif tree.tip().name == "Move to next location":
            self.report_publisher.publish("Moving to next location")
        elif tree.tip().name == "Pick up parcel":
            self.report_publisher.publish("Picking up parcel")
        elif tree.tip().name == "Dropping off parcel":
            self.report_publisher.publish("Drop off parcel")
        else:
            self.report_publisher.publish("Idle, press start to move")

    def tick_tock(self):
        self.tree.tick_tock(500)

    def shutdown(self):
        self.tree.interrupt()

##############################################################################
# Main
##############################################################################


def main():
    """
    Entry point for the script
    """
    rospy.init_node("tree")

    splintered_reality = SplinteredReality()
    rospy.on_shutdown(splintered_reality.shutdown)
    if not splintered_reality.setup():
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    splintered_reality.tick_tock()
