# -*- coding: utf-8 -*-
import pytest
import rospy
import time

from geometry_msgs.msg import *
from std_srvs.srv import *

from dyno_msgs.msg import *
from dyno_msgs.srv import *

NAME = 'world_state_test'


@pytest.fixture
def node():
    rospy.init_node(NAME, anonymous=True)
    clear_locations()


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


def clear_locations():
    service_name = '/world_state/clear_locations'
    rospy.wait_for_service(service_name)
    try:
        rospy.ServiceProxy(service_name, Empty)(EmptyRequest())
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_locations():
    service_name = '/world_state/get_locations'
    rospy.wait_for_service(service_name)
    try:
        return rospy.ServiceProxy(service_name, GetLocations)().locations
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return []


def set_locations(locations):
    request = SetLocationsRequest(locations=locations)
    service_name = '/world_state/set_locations'
    rospy.wait_for_service(service_name)
    try:
        rospy.ServiceProxy(service_name, SetLocations)(request)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def test_it_stores_location(node):
    location = Location(name='start')
    set_locations([location])
    stored_locations = get_locations()

    assert len(stored_locations) == 1


def test_it_stores_locations(node):
    locations = []
    locations.append(Location(name='start'))
    locations.append(Location(name='up'))
    locations.append(Location(name='down'))
    set_locations(locations)
    stored_locations = get_locations()

    assert len(stored_locations) == 3


def test_it_clears_locations(node):
    locations = []
    locations.append(Location(name='start'))
    locations.append(Location(name='up'))
    locations.append(Location(name='down'))

    set_locations(locations)
    clear_locations()
    stored_locations = get_locations()

    assert len(stored_locations) == 0


def test_it_has_correct_location_name(node):
    location = Location(name='start')
    set_locations([location])
    stored_locations = get_locations()

    assert stored_locations[0].name == 'start'


def test_it_has_correct_location_pose(node):
    position = Point(x=0.1, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    location = Location(name='start', pose=Pose(position=position, orientation=orientation))
    set_locations([location])
    stored_locations = get_locations()

    assert stored_locations[0].pose.position.x == 0.1
    assert stored_locations[0].pose.position.y == 0.2
    assert stored_locations[0].pose.position.z == 0.3
    assert stored_locations[0].pose.orientation.x == 0.0
    assert stored_locations[0].pose.orientation.w == 1.0


def test_it_overwrites_location_with_same_name(node):
    position = Point(x=0.1, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    location = Location(name='start', pose=Pose(position=position, orientation=orientation))
    set_locations([location])

    second_position = Point(x=0.2, y=0.2, z=0.3)
    second_orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    second_location = Location(name='start', pose=Pose(position=second_position, orientation=second_orientation))
    set_locations([second_location])

    stored_locations = get_locations()

    assert len(stored_locations) == 1
    assert stored_locations[0].pose.position.x == 0.2


def test_it_publishes_locations(node, waiter):
    locations = []
    locations.append(Location(name='start'))
    set_locations(locations)

    waiter.condition = lambda data: True

    rospy.Subscriber('/world_state/locations', LocationArray, waiter.callback)
    waiter.wait(5.0)

    assert waiter.success


def test_it_publishes_the_correct_number_of_locations(node, waiter):
    locations = []
    locations.append(Location(name='start'))
    locations.append(Location(name='up'))
    locations.append(Location(name='down'))
    set_locations(locations)

    waiter.condition = lambda data: len(data.locations) == 3
    rospy.Subscriber('/world_state/locations', LocationArray, waiter.callback)
    waiter.wait(5.0)

    assert waiter.success


def test_it_publishes_the_correct_location_name(node, waiter):
    locations = []
    locations.append(Location(name='start'))
    set_locations(locations)

    waiter.condition = lambda data: data.locations[0].name == 'start'
    rospy.Subscriber('/world_state/locations', LocationArray, waiter.callback)
    waiter.wait(5.0)

    assert waiter.success


def test_it_publishes_location_change_event(node, waiter):

    waiter.condition = lambda data: data.type == 'locations'
    rospy.Subscriber('/world_state/event', WorldStateEvent, waiter.callback)

    rospy.sleep(0.1)

    locations = []
    locations.append(Location(name='start'))
    set_locations(locations)

    waiter.wait(1.0)

    assert waiter.success
