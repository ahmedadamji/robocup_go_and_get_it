#!/usr/bin/env python
import rospy
import actionlib

from smach import State
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
import tf
import math


class ApproachPerson(State):
    def __init__(self, util, move):
        #rospy.loginfo("ApproachPerson state initialized")

        State.__init__(self, outcomes=["outcome1","outcome2"])

        #creates an instance of util class to transform point frames
        self.util = util
        #creates an instance of move class to move robot across the map
        self.move = move



    def move_to_location(self,current_location):
        #location = rospy.get_param("/pointing_person_approach")
        location = current_location.get("approach_location")

        # Sending Move class the location to move to, and stores result in movebase
        movebase = self.move.move_base(location)
        #self.move.rotate_around_base(-20)
        if movebase == True:
            print("Reached the "  + current_location.get("name"))
        else:
            # INSERT HERE THE ACTION IF GOAL NOT ACHIEVED
            self.interaction.talk("I have not been able to reach the " + current_location.get("name") )

    def deliver_object(self):
        self.util.clear_octomap()
        rospy.sleep(1.0)
        self.move.offer_hand()
        self.move.open_gripper()

    def execute(self, userdata, wait=True):
        rospy.loginfo("ApproachPerson state executing")

        rospy.set_param("/message", "apple to person left") #remove later

        # Collects the details of locations in the environment from the util class and saves in self.locations
        self.locations = self.util.locations

        # create the action client:
        self.movebase_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        # wait until the action server has started up and started listening for goals
        self.movebase_client.wait_for_server()

        for location_id in range(0, len(self.locations)):
            location_name = self.locations[location_id].get("name")

            command = rospy.get_param("/message")

            if location_name in command:
                rospy.set_param("/current_location", self.locations[location_id])
                current_location = self.locations[location_id]
                self.move_to_location(current_location)

                return "outcome1"

        rospy.set_param("/current_location", self.locations[0])
        current_location = self.locations[location_id]
        self.move_to_location(current_location)
        self.deliver_object()



        return "outcome1"
