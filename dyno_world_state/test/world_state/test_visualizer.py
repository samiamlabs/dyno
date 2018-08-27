# -*- coding: utf-8 -*-
import pytest
import rospy
import time
import os

from geometry_msgs.msg import *
from std_srvs.srv import *
from visualization_msgs.msg import Marker, MarkerArray

from dyno_msgs.msg import *
from dyno_msgs.srv import *

NAME = 'world_state_test'


@pytest.fixture
def node():
    rospy.init_node(NAME, anonymous=True)
    clear_robots()
    clear_locations()
    clear_objects()
    clear_objects_on_robots()


@pytest.fixture
def waiter():
    class Waiter(object):
        def __init__(self):
            self.received = []
            self.condition = lambda x: False

        @property
        def success(self):
            return True in self.received

        def callback(self, data):
            self.received.append(self.condition(data))

        def wait(self, timeout):
            timeout_t = time.time() + timeout
            while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
                time.sleep(0.1)

        def reset(self):
            self.received = []

    return Waiter()


def clear_robots():
    service_name = '/world_state/clear_robots'
    rospy.wait_for_service(service_name)
    try:
        rospy.ServiceProxy(service_name, Empty)(EmptyRequest())
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def set_robots(robots):
    request = SetRobotsRequest(robots=robots)
    service_name = '/world_state/set_robots'
    rospy.wait_for_service(service_name)
    try:
        rospy.ServiceProxy(service_name, SetRobots)(request)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def clear_objects():
    service_name = '/world_state/clear_objects'
    rospy.wait_for_service(service_name)
    try:
        rospy.ServiceProxy(service_name, Empty)(EmptyRequest())
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def set_objects(objects):
    request = SetObjectsRequest(objects=objects)
    service_name = '/world_state/set_objects'
    rospy.wait_for_service(service_name)
    try:
        rospy.ServiceProxy(service_name, SetObjects)(request)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def clear_locations():
    service_name = '/world_state/clear_locations'
    rospy.wait_for_service(service_name)
    try:
        rospy.ServiceProxy(service_name, Empty)(EmptyRequest())
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def set_locations(locations):
    request = SetLocationsRequest(locations=locations)
    service_name = '/world_state/set_locations'
    rospy.wait_for_service(service_name)
    try:
        rospy.ServiceProxy(service_name, SetLocations)(request)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_robots_at_locations():
    service_name = '/world_state/get_robots_at_locations'
    rospy.wait_for_service(service_name)
    try:
        return rospy.ServiceProxy(service_name, GetRobotsAtLocations)().robots_at_locations
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return []


def get_objects_at_locations():
    service_name = '/world_state/get_objects_at_locations'
    rospy.wait_for_service(service_name)
    try:
        return rospy.ServiceProxy(service_name, GetObjectsAtLocations)().objects_at_locations
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return []


def clear_objects_on_robots():
    service_name = '/world_state/clear_objects_on_robots'
    rospy.wait_for_service(service_name)
    try:
        rospy.ServiceProxy(service_name, Empty)()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_objects_on_robots():
    service_name = '/world_state/get_objects_on_robots'
    rospy.wait_for_service(service_name)
    try:
        return rospy.ServiceProxy(service_name, GetObjectsOnRobots)().objects_on_robots
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return []


def add_object_on_robot(object_name, robot_name):
    object_on_robot = ObjectOnRobot(robot_name=robot_name, object_name=object_name)
    request = AddObjectOnRobotRequest(object_on_robot=object_on_robot)
    service_name = '/world_state/add_object_on_robot'
    rospy.wait_for_service(service_name)
    try:
        return rospy.ServiceProxy(service_name, AddObjectOnRobot)(request).success
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    return SetObjectOnRobotResponse()


def remove_object_on_robot(object_name, robot_name):
    object_on_robot = ObjectOnRobot(robot_name=robot_name, object_name=object_name)
    request = RemoveObjectOnRobotRequest(object_on_robot=object_on_robot)
    service_name = '/world_state/remove_object_on_robot'
    rospy.wait_for_service(service_name)
    try:
        return rospy.ServiceProxy(service_name, RemoveObjectOnRobot)(request).success
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    return RemoveObjectOnRobotResponse()


def test_it_publishes_object_marker(node, waiter):
    objects = []
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    objects.append(Object(name='red_box', type='parcel', pose=pose))
    set_objects(objects)

    waiter.condition = lambda data: object_marker_condition(data)
    rospy.Subscriber('/world_state/markers', MarkerArray, waiter.callback)

    waiter.wait(1.0)

    assert waiter.success


def object_marker_condition(data):
    success = False
    for marker in data.markers:
        if marker.ns == 'objects':
            success = True
            success = success and marker.pose.position.x == 0.1
            success = success and marker.pose.position.y == 0.2
            success = success and marker.pose.position.z == 0.3
            success = success and marker.header.frame_id == 'map'

    return success


def test_it_publishes_object_marker_label(node, waiter):
    objects = []
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    objects.append(Object(name='red_box', type='parcel', pose=pose))
    set_objects(objects)

    waiter.condition = lambda data: object_marker_label_condition(data)
    rospy.Subscriber('/world_state/markers', MarkerArray, waiter.callback)

    waiter.wait(1.0)

    assert waiter.success


def object_marker_label_condition(data):
    success = False
    for marker in data.markers:
        if marker.ns == 'object_labels':
            success = True
            success = success and marker.pose.position.x == 0.1
            success = success and marker.pose.position.y == 0.2
            success = success and marker.pose.position.z == 0.3 + 0.25
            success = success and marker.header.frame_id == 'map'

    return success


def test_it_publishes_object_markers(node, waiter):
    objects = []
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    objects.append(Object(name='red_box', type='parcel', pose=pose))
    objects.append(Object(name='blue_box', type='parcel', pose=pose))
    objects.append(Object(name='green_box', type='parcel', pose=pose))
    set_objects(objects)

    waiter.condition = lambda data: object_markers_condition(data)
    rospy.Subscriber('/world_state/markers', MarkerArray, waiter.callback)

    waiter.wait(1.0)

    assert waiter.success


def object_markers_condition(data):
    success = False

    marker_counter = 0
    for marker in data.markers:
        if marker.ns == 'objects':
            success = True
            marker_counter += 1

    success = success and marker_counter == 3
    return success


def test_it_publishes_location_marker(node, waiter):
    locations = []
    pose = Pose(position=Point(x=0.2, y=0.3, z=0.4))
    locations.append(Location(name='start', pose=pose))
    set_locations(locations)

    waiter.condition = lambda data: location_marker_condition(data)
    rospy.Subscriber('/world_state/markers', MarkerArray, waiter.callback)

    waiter.wait(1.0)

    assert waiter.success


def location_marker_condition(data):
    success = False
    for marker in data.markers:
        if marker.ns == 'locations':
            success = True
            success = success and marker.pose.position.x == 0.2
            success = success and marker.pose.position.y == 0.3
            success = success and marker.pose.position.z == 0.0
            success = success and marker.header.frame_id == 'map'

    return success


def test_it_publishes_location_markers(node, waiter):
    locations = []
    pose = Pose(position=Point(x=0.2, y=0.3, z=0.4))
    locations.append(Location(name='start', pose=pose))
    locations.append(Location(name='above', pose=pose))
    locations.append(Location(name='below', pose=pose))
    set_locations(locations)

    waiter.condition = lambda data: location_markers_condition(data)
    rospy.Subscriber('/world_state/markers', MarkerArray, waiter.callback)

    waiter.wait(1.0)

    assert waiter.success


def location_markers_condition(data):
    success = False

    marker_counter = 0
    for marker in data.markers:
        if marker.ns == 'objects':
            success = True
            marker_counter += 1

    success = success and marker_counter == 3
    return success


def test_it_publishes_robot_marker(node, waiter):
    robots = []
    pose = Pose(position=Point(x=0.3, y=0.4, z=0.5))
    robots.append(Robot(name='blue', pose=pose))
    set_robots(robots)

    waiter.condition = lambda data: robot_marker_condition(data)
    rospy.Subscriber('/world_state/markers', MarkerArray, waiter.callback)

    waiter.wait(1.0)

    assert waiter.success


def robot_marker_condition(data):
    success = False
    for marker in data.markers:
        if marker.ns == 'robots':
            success = True
            success = success and marker.pose.position.x == 0.3
            success = success and marker.pose.position.y == 0.4
            success = success and marker.pose.position.z == 0.5
            success = success and marker.header.frame_id == 'map'

    return success


def test_it_publishes_robot_markers(node, waiter):
    robots = []
    pose = Pose(position=Point(x=0.3, y=0.4, z=0.5))
    robots.append(Robot(name='blue', pose=pose))
    robots.append(Robot(name='red', pose=pose))
    robots.append(Robot(name='green', pose=pose))
    set_robots(robots)

    waiter.condition = lambda data: robot_markers_condition(data)
    rospy.Subscriber('/world_state/markers', MarkerArray, waiter.callback)

    waiter.wait(1.0)

    assert waiter.success


def robot_markers_condition(data):
    success = False

    marker_counter = 0
    for marker in data.markers:
        if marker.ns == 'robots':
            success = True
            marker_counter += 1

    success = success and marker_counter == 3
    return success
