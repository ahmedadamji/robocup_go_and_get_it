#!/usr/bin/env python
import rospy
import actionlib

from smach import State
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import numpy as np
import cv2

from robocup_grasps import main as grasp_main

class FindObject(State):
    def __init__(self, util, move, ycb_maskrcnn):
        #rospy.loginfo("FindObject state initialized")
        
        State.__init__(self, outcomes=["outcome1","outcome2"])

        #creates an instance of util class to transform point frames
        self.util = util
        #creates an instance of move class to move robot across the map
        self.move = move

        
        self.ycb_maskrcnn = ycb_maskrcnn
        self.object_world_coordinate = 0


    def identify_objects(self):
        #ycb_sub = rospy.Subscriber('segmentations/{}', 1, callback = self.get_point_cloud)
        # colour stuff
        np.random.seed(69)
        COLOURS = np.random.randint(0, 256, (128,3))
        alpha = 0.5
        pclmsg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
        frame, pcl, boxes, clouds, scores, labels, labels_text, masks = self.ycb_maskrcnn.detect(pclmsg, confidence=0.5)

        print labels_text
    
        # output point clouds
        for i, cloud in enumerate(clouds):
            pub = rospy.Publisher('segmentations/{}'.format(i), PointCloud2, queue_size=1)
            pub.publish(cloud)
            #print(i, cloud)
            #pub = rospy.Publisher('segmentations'.format(i), PointCloud2, queue_size=1)
            #pub.publish(cloud)

        self.no_matches = 0

        for i, mask in enumerate(masks):
            label = np.array(labels_text[i])
            colour_label = labels[i]
            colour = COLOURS[colour_label]

            # segmentation masks
            binary_mask = mask > 0.5
            frame_coloured = np.array((frame * (1-alpha) + colour * alpha), dtype=np.uint8)
            frame = np.where(binary_mask, frame_coloured, frame)

            # bboxes + info
            x1, y1, x2, y2 = [int(v) for v in boxes[i]]
            cv2.putText(frame, 'confidence: {:.2f}'.format(scores[i]), (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
            cv2.putText(frame, 'class: {}'.format(labels_text[i]), (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)


            if self.object in label:
                x = (x1+x2)/2
                y = (y1+y2)/2
                camera_point_2d = [x,y]
                self.object_world_coordinate = self.util.get_world_coordinate_from_2d_pixel_coordinate()
                print("object found at current torso height")
                return clouds[i]
            else:
                self.no_matches += 1
        if self.no_matches == len(masks):
            print("object not found at current torso height")
            return None


        cv2.imshow('test', frame)
        cv2.waitKey(1)
            
    def grasp_object(self):
        grasp_main()


    def execute(self, userdata, wait=True):
        rospy.loginfo("FindObject state executing")
        ## REMEMBER TO REMOVE THIS BEFORE THE COMPETITION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        pub = rospy.Publisher('/message', std_msgs.msg.String, queue_size=10)
        pub.publish(std_msgs.msg.String("plastic apple"))

        self.move.look_down(-0.80)


        target_name = rospy.wait_for_message("/message", std_msgs.msg.String)
        self.object = target_name.lower()
        self.result = None
        torso_height = 0.0
        while (self.result is None) and (torso_height <= 0.35):
            self.move.set_torso_height(torso_height)
            self.result = self.identify_objects()
            torso_height += 0.13
            
            if self.result is not None:
                rospy.set_param("/object_world_coordinate", self.object_world_coordinate)
                self.grasp_object(self.result)
                self.move.look_down(0)
                return "outcome1"
         
        self.move.look_down(0)
        self.result = self.identify_objects()
              
        if self.result is not None:
            rospy.set_param("/object_world_coordinate", self.object_world_coordinate)
            self.grasp_object(self.result)
            self.move.look_down(0)
            return "outcome1"
        else:
            print "Object not found on shelves"
            self.move.look_down(0)
            return "outcome2"


        