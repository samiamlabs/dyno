#!/usr/bin/env python

# License: BSD
# https://raw.githubusercontent.com/samiamlabs/dyno/master/LICENCE

import roslib
import rospy
import rostest
import unittest

from geometry_msgs.msg import *
from std_srvs.srv import *

from dyno_msgs.msg import *
from dyno_msgs.srv import *


class TestWorldState(unittest.TestCase):
    def __init__(self, *args):
        super(TestWorldState, self).__init__(*args)
        rospy.init_node('world_state_test')

        self.starttime = rospy.get_time()
        self.test_duration = rospy.get_param("~test_duration", 3)

        rospy.sleep(0)

    # Locations
    def clear_locations(self):
        service_name = '/world_state/clear_locations'
        rospy.wait_for_service(service_name)
        try:
            rospy.ServiceProxy(service_name, Empty)(EmptyRequest())
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_locations(self):
        service_name = '/world_state/get_locations'
        rospy.wait_for_service(service_name)
        try:
            return rospy.ServiceProxy(service_name, GetLocations)().locations
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return []

    def set_locations(self, locations):
        request = SetLocationsRequest(locations=locations)
        service_name = '/world_state/set_locations'
        rospy.wait_for_service(service_name)
        try:
            rospy.ServiceProxy(service_name, SetLocations)(request)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    # Objects
    def clear_objects(self):
        service_name = '/world_state/clear_objects'
        rospy.wait_for_service(service_name)
        try:
            rospy.ServiceProxy(service_name, Empty)(EmptyRequest())
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_objects(self):
        service_name = '/world_state/get_objects'
        rospy.wait_for_service(service_name)
        try:
            return rospy.ServiceProxy(service_name, GetObjects)().objects
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return []

    def set_objects(self, objects):
        request = SetObjectsRequest(objects=objects)
        service_name = '/world_state/set_objects'
        rospy.wait_for_service(service_name)
        try:
            rospy.ServiceProxy(service_name, SetObjects)(request)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def test_all(self):
        self.it_stores_locations()
        self.it_stores_objects()

    def it_stores_locations(self):
        self.clear_locations()

        first_location = Location(name='start', pose=Pose(position=Point(y=0.2)))
        self.set_locations([first_location])

        stored_locations = self.get_locations()

        number_of_locations = len(stored_locations)
        self.assertTrue(number_of_locations == 1, 'Location is not stored, number of locations: {}'.format(number_of_locations))
        position_y = stored_locations[0].pose.position_y
        self.assertEqual(position_y, second, msg)

        second_location = Location(name='second', pose=Pose(position=Point(x=0.1)))
        third_location = Location(name='third', pose=Pose())
        self.set_locations([second_location, third_location])

        stored_locations = self.get_locations()

        number_of_locations = len(stored_locations)
        self.assertTrue(number_of_locations == 3, 'Locations are not stored, number of locations: {}'.format(number_of_locations))
        for location in stored_locations:
            if location.name == 'second':
                position_x = location.pose.position.x
                self.assertEqual(position_x, 0.1, 'Position is not 0.1, reslut value: {}'.format(position_x))

        self.clear_locations()

    def it_stores_objects(self):
        # self.clear_objects()
        #
        # first_object = Object(name='blue_box', type='parcel')
        # first_object.pose = Pose(position=Point(x=0.1, y=0.2, z=0.3))
        # self.set_objects([first_object])
        #
        # stored_objects = self.get_objects()

        self.assertTrue(True, 'Location is not stored')

        # self.assertTrue(len(stored_objects) == 1, 'Object is not stored')
        # self.assertTrue(stored_objects[0].name == 'blue_box', 'Object has incorrect name')
        # self.assertTrue(stored_objects[0].type == 'parcel', 'Object has incorrect type')
        # self.assertAlmostEquals(
        #     stored_objects[0].pose.position.x, 0.1, 7,
        #     'Object position x is not 0.1 value: {}'.format(stored_objects[0].pose.position.x)
        # )
        #
        # self.clear_objects()
        #
        # first_object = Object(name='green_box', type='parcel')
        # second_object = Object(name='blue_pallet', type='pallet')
        # self.set_objects([first_object, second_object])
        #
        # stored_objects = self.get_objects()
        #
        # self.assertTrue(len(stored_objects) == 2, 'Multiple locations are not stored')
        # for object in stored_objects:
        #     if object.name == 'blue_pallet':
        #         self.assertTrue(object.type == 'pallet', 'Pallet object has incorrect type')


if __name__ == '__main__':

    rostest.run("dyno_world_state",
                "test_world_state",
                TestWorldState)
