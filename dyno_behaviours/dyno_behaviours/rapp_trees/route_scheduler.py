#!/usr/bin/env python
#
# License: BSD
# https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE
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

from dyno_behaviours.rapp_behaviours import route_scheduler_behaviours

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
    root = py_trees.composites.Parallel("Route Scheduler")

    start2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Start2BB",
        topic_name="/route_scheduler/start",
        variable_name="event_start_button"
    )

    stop2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Stop2BB",
        topic_name="/route_scheduler/stop",
        variable_name="event_stop_button"
    )

    clear2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Clear2BB",
        topic_name="/route_scheduler/clear",
        variable_name="event_clear_button"
    )

    topics2bb = py_trees.composites.Sequence("Topics2BB")

    add_location2bb = dyno_behaviours.add_location.ToBlackboard(name="AddLocation2BB",
                                                                topic_name="/route_scheduler/add_location"
                                                                )

    locations2bb = dyno_behaviours.locations.ToBlackboard(name="Locations2BB",
                                                          topic_name="/world_state/locations"
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

    move_to_next = route_scheduler_behaviours.MoveToNextWaypoint(
        name="Move to next waypoint",
        action_namespace="/move_base",
        action_spec=move_base_msgs.MoveBaseAction,
        action_goal=move_base_msgs.MoveBaseGoal()
    )

    move_or_cancel = py_trees.composites.Selector(name="Move or be canceled")

    clear = py_trees.meta.success_is_failure(
        py_trees.composites.Selector)(name="Clear Queue")

    is_clear_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="Clear?",
        variable_name='event_clear_button',
        expected_value=False
    )
    clear_locations_in_queue = route_scheduler_behaviours.ClearLocationQueue(
        name="Clear locations from queue")

    remove_first = py_trees.meta.success_is_failure(
        py_trees.composites.Sequence)(name="Remove first")

    remove_location_from_queue = route_scheduler_behaviours.RemoveFirstLocationFromQueue(
        name="Remove first location from queue")

    # Tree
    root.add_child(topics2bb)
    topics2bb.add_children([locations2bb, start2bb, stop2bb, clear2bb, add_location2bb])
    clear.add_children([is_clear_requested, clear_locations_in_queue])
    root.add_child(priorities)
    priorities.add_children([is_queue_empty, clear, start, move, idle])
    start.add_children([is_start_requested, set_should_move])
    cancel.add_children([is_stop_requested, clear_should_move])
    move.add_children([is_move_requested, move_or_cancel])
    move_or_cancel.add_children([cancel, move_and_remove])
    move_and_remove.add_children([move_to_next, remove_location_from_queue])
    return root


class SplinteredReality(object):

    def __init__(self):
        self.tree = py_trees_ros.trees.BehaviourTree(create_root())
        self.tree.add_post_tick_handler(self.publish_reality_report)
        self.report_publisher = rospy.Publisher(
            "route_scheduler/report", std_msgs.String, queue_size=5)

    def setup(self):
        return self.tree.setup(timeout=15)

    def publish_reality_report(self, tree):
        if tree.tip().name == "Queue empty?":
            self.report_publisher.publish("Waypoint Queue is empty")
        elif tree.tip().name == "Remove first location from queue":
            self.report_publisher.publish("Removing waypoint from queue")
        elif tree.tip().has_parent_with_name("Move"):
            self.report_publisher.publish("Moving to next waypoint")
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
