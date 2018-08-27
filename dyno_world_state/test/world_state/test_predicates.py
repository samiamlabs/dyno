# -*- coding: utf-8 -*-
import pytest
import rospy
import time
import os

from geometry_msgs.msg import *
from std_srvs.srv import *

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


def test_it_calculates_robot_at_location(node):
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    robot = Robot(name='blue', type='quadrotor', pose=pose)
    set_robots([robot])

    position = Point(x=0.1, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    location = Location(name='start', pose=Pose(position=position, orientation=orientation))
    set_locations([location])

    robots_at_locations = get_robots_at_locations()

    assert len(robots_at_locations) == 1
    assert robots_at_locations[0].robot_name == 'blue'
    assert robots_at_locations[0].location_name == 'start'


def test_it_calculates_no_robots_at_locations(node):
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    robot = Robot(name='blue', type='quadrotor', pose=pose)
    set_robots([robot])

    position = Point(x=0.8, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    location = Location(name='start', pose=Pose(position=position, orientation=orientation))
    set_locations([location])

    robots_at_locations = get_robots_at_locations()

    assert len(robots_at_locations) == 0


def test_it_calculates_robots_at_locations(node):
    robots = []
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    robots.append(Robot(name='blue', type='quadrotor', pose=pose))
    pose = Pose(position=Point(x=0.8, y=0.2, z=0.3))
    robots.append(Robot(name='red', type='quadrotor', pose=pose))
    set_robots(robots)

    locations = []
    position = Point(x=0.1, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    locations.append(Location(name='start', pose=Pose(position=position, orientation=orientation)))
    second_position = Point(x=0.8, y=0.2, z=0.3)
    second_orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    locations.append(Location(name='above', pose=Pose(position=second_position, orientation=second_orientation)))
    set_locations(locations)

    robots_at_locations = get_robots_at_locations()

    assert len(robots_at_locations) == 2


def test_it_publishes_robots_at_locations_change_event(node, waiter):
    waiter.condition = lambda data: data.type == 'robots_at_locations'
    # waiter.condition = lambda data: data.type == 'robots'
    rospy.Subscriber('/world_state/event', WorldStateEvent, waiter.callback)

    rospy.sleep(0.1)

    robots = []
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    robots.append(Robot(name='blue', type='quadrotor', pose=pose))
    set_robots(robots)

    locations = []
    position = Point(x=0.1, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    locations.append(Location(name='start', pose=Pose(position=position, orientation=orientation)))
    set_locations(locations)

    waiter.wait(1.0)

    assert waiter.success

    waiter.received = []

    robots = []
    pose = Pose(position=Point(x=0.8, y=0.2, z=0.3))
    robots.append(Robot(name='red', type='quadrotor', pose=pose))
    set_robots(robots)

    locations = []
    second_position = Point(x=0.8, y=0.2, z=0.3)
    second_orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    locations.append(Location(name='above', pose=Pose(position=second_position, orientation=second_orientation)))
    set_locations(locations)

    waiter.wait(1.0)

    assert waiter.success


def test_it_calculates_object_at_location(node):
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    object = Object(name='blue_box', type='parcel', pose=pose)
    set_objects([object])

    position = Point(x=0.1, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    location = Location(name='start', pose=Pose(position=position, orientation=orientation))
    set_locations([location])

    objects_at_locations = get_objects_at_locations()

    assert len(objects_at_locations) == 1
    assert objects_at_locations[0].object_name == 'blue_box'
    assert objects_at_locations[0].location_name == 'start'


def test_it_calculates_no_objects_at_locations(node):
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    object = Object(name='blue_box', type='parcel', pose=pose)
    set_objects([object])

    position = Point(x=0.8, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    location = Location(name='start', pose=Pose(position=position, orientation=orientation))
    set_locations([location])

    objects_at_locations = get_objects_at_locations()

    assert len(objects_at_locations) == 0


def test_it_calculates_objects_at_locations(node):
    objects = []
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    objects.append(Object(name='blue_box', type='parcel', pose=pose))
    pose = Pose(position=Point(x=0.8, y=0.2, z=0.3))
    objects.append(Robot(name='red_box', type='parcel', pose=pose))
    set_objects(objects)

    locations = []
    position = Point(x=0.1, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    locations.append(Location(name='start', pose=Pose(position=position, orientation=orientation)))
    second_position = Point(x=0.8, y=0.2, z=0.3)
    second_orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    locations.append(Location(name='above', pose=Pose(position=second_position, orientation=second_orientation)))
    set_locations(locations)

    objects_at_locations = get_objects_at_locations()

    assert len(objects_at_locations) == 2


def test_it_publishes_objects_at_locations_change_event(node, waiter):
    waiter.condition = lambda data: data.type == 'objects_at_locations'
    rospy.Subscriber('/world_state/event', WorldStateEvent, waiter.callback)

    rospy.sleep(0.1)

    objects = []
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    objects.append(Object(name='blue_box', type='parcel', pose=pose))
    set_objects(objects)

    locations = []
    position = Point(x=0.1, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    locations.append(Location(name='middle', pose=Pose(position=position, orientation=orientation)))
    set_locations(locations)

    waiter.wait(1.0)

    assert waiter.success

    waiter.received = []

    objects = []
    pose = Pose(position=Point(x=-0.8, y=0.2, z=0.3))
    objects.append(Object(name='red_box', type='parcel', pose=pose))
    set_objects(objects)

    locations = []
    second_position = Point(x=-0.8, y=0.2, z=0.3)
    locations.append(Location(name='below', pose=Pose(position=second_position)))
    set_locations(locations)

    waiter.wait(1.0)

    assert waiter.success


def test_it_adds_object_on_robot(node):
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    robot = Robot(name='blue', type='quadrotor', pose=pose)
    set_robots([robot])

    objects = []
    pose = Pose(position=Point(x=-0.8, y=0.2, z=0.3))
    objects.append(Object(name='red_box', type='parcel', pose=pose))
    set_objects(objects)

    success = add_object_on_robot('red_box', 'blue')
    objects_on_robots = get_objects_on_robots()

    assert success
    assert len(objects_on_robots) == 1
    assert objects_on_robots[0].object_name == 'red_box'
    assert objects_on_robots[0].robot_name == 'blue'


def test_it_clears_objects_on_robots(node):
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    robot = Robot(name='blue', type='quadrotor', pose=pose)
    set_robots([robot])

    objects = []
    pose = Pose(position=Point(x=-0.8, y=0.2, z=0.3))
    objects.append(Object(name='red_box', type='parcel', pose=pose))
    set_objects(objects)

    success = add_object_on_robot('red_box', 'blue')

    objects_on_robots = get_objects_on_robots()
    assert len(objects_on_robots) == 1

    clear_objects_on_robots()

    objects_on_robots = get_objects_on_robots()
    assert len(objects_on_robots) == 0


def test_it_adds_objects_on_robots(node):
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    robot = Robot(name='blue', type='quadrotor', pose=pose)
    second_robot = Robot(name='red', type='quadrotor', pose=pose)
    set_robots([robot, second_robot])

    objects = []
    pose = Pose(position=Point(x=-0.8, y=0.2, z=0.3))
    objects.append(Object(name='red_box', type='parcel', pose=pose))
    objects.append(Object(name='green_box', type='parcel', pose=pose))
    set_objects(objects)

    success = add_object_on_robot('red_box', 'blue')
    success = add_object_on_robot('green_box', 'red')
    objects_on_robots = get_objects_on_robots()

    assert len(objects_on_robots) == 2


def test_it_overwrites_objects_on_robots(node):
    robots = []
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    robots.append(Robot(name='blue', type='quadrotor', pose=pose))
    set_robots(robots)

    objects = []
    pose = Pose(position=Point(x=-0.8, y=0.2, z=0.3))
    objects.append(Object(name='red_box', type='parcel', pose=pose))
    set_objects(objects)

    success = add_object_on_robot('red_box', 'blue')
    success = add_object_on_robot('red_box', 'blue')
    objects_on_robots = get_objects_on_robots()

    assert len(objects_on_robots) == 1
    assert objects_on_robots[0].object_name == 'red_box'
    assert objects_on_robots[0].robot_name == 'blue'


def test_it_removes_object_on_robot(node):
    robots = []
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    robots.append(Robot(name='blue', type='quadrotor', pose=pose))
    robots.append(Robot(name='red', type='quadrotor', pose=pose))
    set_robots(robots)

    objects = []
    pose = Pose(position=Point(x=-0.8, y=0.2, z=0.3))
    objects.append(Object(name='red_box', type='parcel', pose=pose))
    objects.append(Object(name='green_box', type='parcel', pose=pose))
    set_objects(objects)

    add_object_on_robot('red_box', 'blue')
    add_object_on_robot('green_box', 'red')

    success = remove_object_on_robot('red_box', 'blue')

    objects_on_robots = get_objects_on_robots()

    assert success
    assert len(objects_on_robots) == 1


def test_it_fails_to_add_non_exsisting_object_on_robot(node):

    success = add_object_on_robot('red_box', 'blue')

    assert not success


def test_it_fails_to_remove_non_exsisting_object_on_robot(node):
    robots = []
    pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
    robots.append(Robot(name='blue', type='quadrotor', pose=pose))
    robots.append(Robot(name='red', type='quadrotor', pose=pose))
    set_robots(robots)

    objects = []
    pose = Pose(position=Point(x=-0.8, y=0.2, z=0.3))
    objects.append(Object(name='red_box', type='parcel', pose=pose))
    objects.append(Object(name='green_box', type='parcel', pose=pose))
    set_objects(objects)

    add_object_on_robot('red_box', 'blue')
    add_object_on_robot('green_box', 'red')

    success = remove_object_on_robot('blue_box', 'blue')

    objects_on_robots = get_objects_on_robots()

    assert not success
    assert len(objects_on_robots) == 2
