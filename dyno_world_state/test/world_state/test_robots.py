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


def get_robots():
    service_name = '/world_state/get_robots'
    rospy.wait_for_service(service_name)
    try:
        return rospy.ServiceProxy(service_name, GetRobots)().robots
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return []


def set_robots(robots):
    request = SetRobotsRequest(robots=robots)
    service_name = '/world_state/set_robots'
    rospy.wait_for_service(service_name)
    try:
        rospy.ServiceProxy(service_name, SetRobots)(request)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def test_it_stores_robot(node):
    robot = Robot(name='green', type='quadrotor')
    set_robots([robot])
    stored_robots = get_robots()

    assert len(stored_robots) == 1


def test_it_clears_robots(node):
    robots = []
    robots.append(Robot(name='blue'))
    robots.append(Robot(name='red'))
    robots.append(Robot(name='green'))
    set_robots(robots)
    clear_robots()
    stored_robots = get_robots()

    assert len(stored_robots) == 0


def test_it_stores_robots(node):
    robots = []
    robots.append(Robot(name='blue'))
    robots.append(Robot(name='red'))
    robots.append(Robot(name='green'))
    set_robots(robots)
    stored_robots = get_robots()

    assert len(stored_robots) == 3


def test_it_has_correct_robot_name(node):
    robot = Robot(name='blue')
    set_robots([robot])
    stored_robots = get_robots()

    assert stored_robots[0].name == 'blue'


def test_it_has_correct_robot_type(node):
    robot = Robot(name='blue', type='quadrotor')
    set_robots([robot])
    stored_robots = get_robots()

    assert stored_robots[0].type == 'quadrotor'


def test_it_stores_local_robot(node):
    master_name = rospy.get_param("/rocon/master/name", "dyno")
    robot_base_type = os.getenv('DYNO_BASE', 'forklift')

    robot_pose_publisher = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)

    pose = Pose(position=Point(x=0.5))
    pose_stamped_msg = PoseStamped(pose=pose)
    timeout = rospy.Time.now() + rospy.Duration(1.0)
    while rospy.Time.now() < timeout:
        rospy.sleep(0.1)
        robot_pose_publisher.publish(pose_stamped_msg)

    stored_robots = get_robots()
    assert len(stored_robots) == 1
    assert stored_robots[0].name == master_name
    assert stored_robots[0].type == robot_base_type
    assert stored_robots[0].pose.position.x == 0.5


def test_it_has_correct_robot_pose(node):
    position = Point(x=0.1, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    robots = Robot(name='blue', pose=Pose(position=position, orientation=orientation))
    set_robots([robots])
    stored_robots = get_robots()

    assert stored_robots[0].pose.position.x == 0.1
    assert stored_robots[0].pose.position.y == 0.2
    assert stored_robots[0].pose.position.z == 0.3
    assert stored_robots[0].pose.orientation.x == 0.0
    assert stored_robots[0].pose.orientation.w == 1.0


def test_it_overwrites_robots_with_same_name(node):
    position = Point(x=0.1, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    robot = Robot(name='blue', pose=Pose(position=position, orientation=orientation))
    set_robots([robot])

    position = Point(x=0.2, y=0.2, z=0.3)
    orientation = Quaternion(x=0, y=0, z=0, w=1.0)
    second_robot = Robot(name='blue', pose=Pose(position=position, orientation=orientation))
    set_robots([second_robot])

    stored_robots = get_robots()

    assert len(stored_robots) == 1
    assert stored_robots[0].pose.position.x == 0.2


def test_it_publishes_robots(node, waiter):
    robots = []
    robots.append(Robot(name='blue'))
    set_robots(robots)

    waiter.condition = lambda data: True

    rospy.Subscriber('/world_state/robots', RobotArray, waiter.callback)
    waiter.wait(5.0)

    assert waiter.success


def test_it_publishes_the_correct_number_of_robots(node, waiter):
    robots = []
    robots.append(Robot(name='blue_box'))
    robots.append(Robot(name='green_box'))
    robots.append(Robot(name='red_box'))
    set_robots(robots)

    waiter.condition = lambda data: len(data.robots) == 3
    rospy.Subscriber('/world_state/robots', RobotArray, waiter.callback)
    waiter.wait(5.0)

    assert waiter.success


def test_it_publishes_the_correct_robot_props(node, waiter):
    robots = []
    robots.append(Robot(name='blue_box', type='parcel'))
    set_robots(robots)

    waiter.condition = lambda data: len(data.robots) > 0 and data.robots[0].name == 'blue_box' and data.robots[0].type == 'parcel'
    rospy.Subscriber('/world_state/robots', RobotArray, waiter.callback)
    waiter.wait(5.0)

    assert waiter.success


def test_it_publishes_robot_change_event(node, waiter):
    waiter.condition = lambda data: data.type == 'robots'
    rospy.Subscriber('/world_state/event', WorldStateEvent, waiter.callback)

    rospy.sleep(0.1)

    robots = []
    robots.append(Robot(name='blue_box', type='parcel'))
    set_robots(robots)

    waiter.wait(1.0)

    assert waiter.success
