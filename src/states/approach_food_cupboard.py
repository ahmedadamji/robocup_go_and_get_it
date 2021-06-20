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


    def identify_obstacles(self, data):
        #self.object_aware_cloud = self.segmentfloor.detect(data)
        print("A")

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

    def segment_cloud(self, cloud, pclmsg):
        mask_confidence = 0.5

        #convert pcl msg
        pcl, img, imgmsg = self.pclmsg_to_pcl_cv2_imgmsg(cloud)

        original_image = img
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)

        #colour hsv values
        colour_low = np.array([10, 30, 20])
        colour_high = np.array([40, 255, 255])
        white = np.array([0,0,0])

        #mask the floor and obstacles
        floor_mask = cv2.inRange(hsv_image, colour_low, colour_high)
        obstacles_mask = floor_mask
        img = cv2.bitwise_and(img, img, mask = obstacles_mask)
        #obstacles = original_image - img
        obstacles = img

        binary_mask = obstacles_mask > mask_confidence
        cv2.imshow('binmask', binary_mask.astype(np.float32))
        indices = np.argwhere(binary_mask)

        # set dist to infinity if x,y in mask = 0
        # otherwise keep dist if x,y in mask = 1
        # complexity is O(3N) so it should b fast
        pcl = pcl.reshape(cloud.height, cloud.width, 32)
        pcl = pcl.view(np.float32)
        temp = {}
        for y,x in indices:
            temp[y,x] = pcl[y,x,2]
        pcl[:,:,2] = float('inf')
        for y,x in indices:
            pcl[y,x,2] = temp[y,x]

        # create pcl
        cloud.data = pcl.flatten().tostring()

        return cloud


    def identify_objects(self):
        #ycb_sub = rospy.Subscriber('segmentations/{}', 1, callback = self.get_point_cloud)
        # colour stuff
        np.random.seed(69)
        COLOURS = np.random.randint(0, 256, (128,3))
        alpha = 0.5
        pclmsg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
        frame, pcl, boxes, clouds, scores, labels, labels_text, masks = self.ycb_maskrcnn.detect(pclmsg, confidence=0.5)

        # output point clouds
        for i, cloud in enumerate(clouds):
            cloud = self.segment_cloud(cloud, pclmsg)
            self.object_aware_cloud = cloud

        for i, mask in enumerate(masks):
            label = labels[i]
            colour = COLOURS[label]

            # segmentation masks
            binary_mask = mask > 0.5
            frame_coloured = np.array((frame * (1-alpha) + colour * alpha), dtype=np.uint8)
            frame = np.where(binary_mask, frame_coloured, frame)

            # bboxes + info
            x1, y1, x2, y2 = [int(v) for v in boxes[i]]
            cv2.putText(frame, 'confidence: {:.2f}'.format(scores[i]), (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
            cv2.putText(frame, 'class: {}'.format(labels_text[i]), (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)


            pub = rospy.Publisher('/object_aware_cloud', PointCloud2, queue_size=1)
            pub.publish(cloud)


        cv2.imshow('test', frame)
        cv2.waitKey(1)

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
        else:
            # INSERT HERE THE ACTION IF GOAL NOT ACHIEVED
            self.interaction.talk("I have not been able to reach the " + current_location.get("name") )



    def execute(self, userdata, wait=True):
        rospy.loginfo("ApproachFoodCupboard state executing")
        rospy.set_param("/message", "apple to person left")

        # Collects the details of locations in the environment from the util class and saves in self.locations
        self.locations = self.util.locations

        # create the action client:
        self.movebase_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        # wait until the action server has started up and started listening for goals
        self.movebase_client.wait_for_server()

        sub = rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.identify_obstacles, queue_size=1)
        self.move.look_down(-1.0)
        self.identify_objects()



        for location_id in range(0, len(self.locations)):
            location_name = self.locations[location_id].get("name")
            print location_name + " is the target location."
            if location_name == "goal":
                rospy.set_param("/current_location", self.locations[location_id])
                current_location = self.locations[location_id]
                self.move_to_location(current_location)

        sub.unregister()
        command = rospy.get_param("/message")
        rospy.set_param("/object", command)
        self.move.look_down(0)

        return "outcome2"
