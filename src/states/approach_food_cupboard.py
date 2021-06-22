#!/usr/bin/env python
import rospy
import actionlib

from smach import State
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
from tf import TransformListener
import math
from PIL import Image
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
from std_msgs.msg import Empty

import ros_numpy




class ApproachFoodCupboard(State):
    def __init__(self, util, move, ycb_maskrcnn, segmentfloor):
        #rospy.loginfo("ApproachFoodCupboard state initialized")

        State.__init__(self, outcomes=["outcome1","outcome2"])

        #creates an instance of util class to transform point frames
        self.util = util
        #creates an instance of move class to move robot across the map
        self.move = move

        self.tf = TransformListener()

        self.ycb_maskrcnn = ycb_maskrcnn

        self.segmentfloor = segmentfloor

    def empty_cloud(self):
        data = np.zeros(100, dtype=[('x', np.float32),('y', np.float32),('z', np.float32)])

        msg = ros_numpy.msgify(PointCloud2, data)
        msg.header.frame_id = 'map'
        cloud_pub = rospy.Publisher('/object_aware_cloud', PointCloud2, queue_size=1)
        rate = rospy.Rate(1)

        cloud_pub.publish(msg)



    def identify_obstacles(self, data):
        self.segmentfloor.detect(data)


    def pclmsg_to_pcl_cv2_imgmsg(self, pclmsg):
        # extract the xyz values from 32FC1
        pcl = np.fromstring(pclmsg.data, dtype=np.uint8)
        pcl = pcl.reshape(pclmsg.height, pclmsg.width, -1)

        # extract the rgb values from 32FC1
        frame = np.fromstring(pclmsg.data, dtype=np.uint8)
        frame = frame.reshape(pclmsg.height, pclmsg.width, 32)
        frame = frame[:,:,16:19].copy()

        # imgmsg
        bridge = CvBridge()
        imgmsg = bridge.cv2_to_imgmsg(frame, encoding='rgb8')

        return pcl, frame, imgmsg

    def get_point_cloud(self, data):
        point = self.tf.transformPoint("/base_link", data.i)
        print(point)


    def move_to_location(self,current_location):
        #location = rospy.get_param("/pointing_person_approach")
        location = current_location.get("approach_location")

        # Sending Move class the location to move to, and stores result in movebase
        movebase = self.move.move_base(location)
        #self.move.rotate_around_base(-20)
        if movebase == True:
            print("Reached the "  + current_location.get("name"))
            return True
        else:
            # INSERT HERE THE ACTION IF GOAL NOT ACHIEVED
            #self.interaction.talk("I have not been able to reach the " + current_location.get("name") )
            print("cannot reach destination")
            return False


    def execute(self, userdata, wait=True):
        rospy.loginfo("ApproachFoodCupboard state executing")

        self.empty_cloud()

        # Collects the details of locations in the environment from the util class and saves in self.locations
        self.locations = self.util.locations

        # create the action client:
        self.movebase_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        # wait until the action server has started up and started listening for goals
        self.movebase_client.wait_for_server()

        sub = rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.identify_obstacles, queue_size=1)

        self.move.look_down(-1.0)


        for location_id in range(0, len(self.locations)):
            location_name = self.locations[location_id].get("name")
            print location_name + " is the target location."
            if location_name == "goal":
                rospy.set_param("/current_location", self.locations[location_id])
                current_location = self.locations[location_id]
                if self.move_to_location(current_location) == False:
                    return "outcome2"
        
        sub.unregister()

        return "outcome1"
