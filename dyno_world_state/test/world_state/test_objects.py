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
    clear_objects()


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


def clear_objects():
    service_name = '/world_state/clear_objects'
    rospy.wait_for_service(service_name)
    try:
        rospy.ServiceProxy(service_name, Empty)(EmptyRequest())
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_objects():
    service_name = '/world_state/get_objects'
    rospy.wait_for_service(service_name)
    try:
        return rospy.ServiceProxy(service_name, GetObjects)().objects
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return []


def set_objects(objects):
    request = SetObjectsRequest(objects=objects)
    service_name = '/world_state/set_objects'
    rospy.wait_for_service(service_name)
    try:
        rospy.ServiceProxy(service_name, SetObjects)(request)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def test_it_stores_object(node):
    object = Object()
    set_objects([object])
    stored_objects = get_objects()

    assert len(stored_objects) == 1


def test_it_clears_objects(node):
    objects = []
    objects.append(Object(name='blue_box'))
    objects.append(Object(name='red_box'))
    objects.append(Object(name='green_box'))
    set_objects(objects)
    clear_objects()
    stored_objects = get_objects()

    assert len(stored_objects) == 0


def test_it_stores_objects(node):
    objects = []
    objects.append(Object(name='blue_box'))
    objects.append(Object(name='red_box'))
    objects.append(Object(name='green_box'))
    set_objects(objects)
    stored_objects = get_objects()

    assert len(stored_objects) == 3


def test_it_has_correct_object_name(node):
    objact = Object(name='start')
    set_objects([objact])
    stored_objects = get_objects()

    assert stored_objects[0].name == 'start'


def test_it_has_correct_object_pose(node):
    position = Point(x=0.1, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    objects = Object(name='start', pose=Pose(position=position, orientation=orientation))
    set_objects([objects])
    stored_objects = get_objects()

    assert stored_objects[0].pose.position.x == 0.1
    assert stored_objects[0].pose.position.y == 0.2
    assert stored_objects[0].pose.position.z == 0.3
    assert stored_objects[0].pose.orientation.x == 0.0
    assert stored_objects[0].pose.orientation.w == 1.0


def test_it_overwrites_objects_with_same_name(node):
    position = Point(x=0.1, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    object = Object(name='start', pose=Pose(position=position, orientation=orientation))
    set_objects([object])

    position = Point(x=0.2, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    second_object = Object(name='start', pose=Pose(position=position, orientation=orientation))
    set_objects([second_object])

    stored_objects = get_objects()

    assert len(stored_objects) == 1
    assert stored_objects[0].pose.position.x == 0.2


def test_it_publishes_objects(node, waiter):
    objects = []
    objects.append(Object(name='blue_box'))
    objects.append(Object(name='green_box'))
    objects.append(Object(name='red_box'))
    set_objects(objects)

    waiter.condition = lambda data: True

    rospy.Subscriber('/world_state/objects', ObjectArray, waiter.callback)
    waiter.wait(5.0)

    assert waiter.success


def test_it_publishes_the_correct_number_of_objects(node, waiter):
    objects = []
    objects.append(Object(name='blue_box'))
    objects.append(Object(name='green_box'))
    objects.append(Object(name='red_box'))
    set_objects(objects)

    waiter.condition = lambda data: len(data.objects) == 3
    rospy.Subscriber('/world_state/objects', ObjectArray, waiter.callback)
    waiter.wait(5.0)

    assert waiter.success


def test_it_publishes_the_correct_object_props(node, waiter):
    objects = []
    objects.append(Object(name='blue_box', type='parcel'))
    set_objects(objects)

    waiter.condition = lambda data: data.objects[0].name == 'blue_box' and data.objects[0].type == 'parcel'
    rospy.Subscriber('/world_state/objects', LocationArray, waiter.callback)
    waiter.wait(5.0)

    assert waiter.success


def test_it_publishes_object_change_event(node, waiter):
    waiter.condition = lambda data: data.type == 'objects'
    rospy.Subscriber('/world_state/event', WorldStateEvent, waiter.callback)

    rospy.sleep(0.1)

    objects = []
    objects.append(Object(name='blue_box', type='parcel'))
    set_objects(objects)

    waiter.wait(1.0)

    assert waiter.success
