#!/usr/bin/env python
import rospy
import actionlib

from smach import State
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
from tf import TransformListener
import math
from PIL import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import numpy as np
import cv2
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib


class FindObject(State):
    def __init__(self, util, move, ycb_maskrcnn):
        #rospy.loginfo("FindObject state initialized")
        
        State.__init__(self, outcomes=["outcome1","outcome2"])

        #creates an instance of util class to transform point frames
        self.util = util
        #creates an instance of move class to move robot across the map
        self.move = move

        self.tf = TransformListener()
        
        self.ycb_maskrcnn = ycb_maskrcnn

    def send_joint_trajectory(self, topic_name, joint_names, position, time_from_start):
        print("trying to look down")
        head_trajectory_client = actionlib.SimpleActionClient(topic_name, FollowJointTrajectoryAction)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = joint_names

        #set first and only position
        goal.trajectory.points.append(JointTrajectoryPoint())
        goal.trajectory.points[0].positions = position 
        goal.trajectory.points[0].time_from_start = rospy.Duration(time_from_start)
        
        head_trajectory_client.wait_for_server()
        head_trajectory_client.send_goal(goal)

        return head_trajectory_client.wait_for_result, head_trajectory_client.get_result


    def identify_obstacles(self):
        #ycb_sub = rospy.Subscriber('segmentations/{}', 1, callback = self.get_point_cloud)
        # colour stuff
        np.random.seed(69)
        COLOURS = np.random.randint(0, 256, (128,3))
        alpha = 0.5
        while not rospy.is_shutdown():
            pclmsg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
            frame, pcl, boxes, clouds, scores, labels, labels_text, masks = self.ycb_maskrcnn.detect(pclmsg, confidence=0.5)
        
            # output point clouds
            for i, cloud in enumerate(clouds):
                pub = rospy.Publisher('segmentations/{}'.format(i), PointCloud2, queue_size=1)
                pub.publish(cloud)
                #print(i, cloud)
                #pub = rospy.Publisher('segmentations'.format(i), PointCloud2, queue_size=1)
                #pub.publish(cloud)


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

            cv2.imshow('test', frame)
            cv2.waitKey(1)
            

    def get_point_cloud(self, data):
        point = self.tf.transformPoint("/base_link", data.i)
        print(point)
        ## Do Something

    #def combine_point_cloud_and_laser(self):
        ## Do Something
    
    #def obstacle_avoidance(self):
        ## Do Something

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
        rospy.loginfo("FindObject state executing")
        rospy.set_param("/message", "apple to person left")

        head = -0.75
        torso_lift = np.random.uniform(0.0,0.35)
        self.send_joint_trajectory('/head_controller/follow_joint_trajectory', ['head_1_joint','head_2_joint'], [0, head], 2.0)
        #self.send_joint_trajectory('/torso_controller/folow_joint_trajectory', ['torso_lift_joint'],[torso_lift],2.0)

        # Collects the details of locations in the environment from the util class and saves in self.locations
        self.locations = self.util.locations

        # create the action client:
        self.movebase_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        # wait until the action server has started up and started listening for goals
        self.movebase_client.wait_for_server()

        self.identify_obstacles()

        for location_id in range(0, len(self.locations)):
            location_name = self.locations[location_id].get("name")
            print location_name + " is the target location."
            if location_name == "goal":
                rospy.set_param("/current_location", self.locations[location_id])
                current_location = self.locations[location_id]
                self.move_to_location(current_location)

        command = rospy.get_param("/message")
        rospy.set_param("/object", command)
        
        return "outcome2"