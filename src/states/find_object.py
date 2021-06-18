#!/usr/bin/env python
import rospy
import actionlib

from smach import State
from sensor_msgs.msg import PointCloud2
import numpy as np
import cv2

from robocup_grasps import main as grasp_main

class FindObject(State):
    def __init__(self, util, move, ycb_maskrcnn):
        #rospy.loginfo("FindObject state initialized")
        
        State.__init__(self, outcomes=["outcome1","outcome2"])

        #creates an instance of util class to transform point frames
        self.util = util

        
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
    
        # output point clouds
        for i, cloud in enumerate(clouds):
            pub = rospy.Publisher('segmentations/{}'.format(i), PointCloud2, queue_size=1)
            pub.publish(cloud)
            #print(i, cloud)
            #pub = rospy.Publisher('segmentations'.format(i), PointCloud2, queue_size=1)
            #pub.publish(cloud)

        self.no_matches = 0

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


            if self.object in label:
                x = (x1+x2)/2
                y = (y1+y2)/2
                camera_point_2d = [x,y]
                self.object_world_coordinate = self.util.get_world_coordinate_from_2d_pixel_coordinate()
            else:
                self.no_matches += 1
        if self.no_matches == len(masks):
            print("object not found")

        cv2.imshow('test', frame)
        cv2.waitKey(1)
            
    def grasp_object(self):
        grasp_main()


    def execute(self, userdata, wait=True):
        rospy.loginfo("FindObject state executing")

        self.object = rospy.get_param("/object")

        # self.identify_objects()
        self.grasp_object()

        rospy.set_param("/object_world_coordinate", self.object_world_coordinate)
        
        return "outcome2"