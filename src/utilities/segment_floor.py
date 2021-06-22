#!/usr/bin/env python

import numpy as np
import os
import message_filters
import cv2

from PIL import Image
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge, CvBridgeError
from move import Move

import rospy
import rospkg


class SegmentFloor():
    def __init__(self, object_detector=None):

        self.bridge = CvBridge()
        self.image = None
        self.cloud = None
        self.sensitivity = 10
        self.pub = rospy.Publisher('/object_aware_cloud', PointCloud2, queue_size=1)
        self.object_detector = object_detector
        rospy.sleep(1)


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


    def detect(self, pclmsg):
        mask_confidence = 0.5

        #convert pcl msg
        pcl, img, imgmsg = self.pclmsg_to_pcl_cv2_imgmsg(pclmsg)

        original_image = img
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)

        #colour hsv values
        colour_low = np.array([10, 30, 20])
        colour_high = np.array([40, 255, 255])
        white = np.array([0,0,0])

        #mask the floor and obstacles
        floor_mask = cv2.inRange(hsv_image, colour_low, colour_high)
        obstacles_mask = cv2.bitwise_not(floor_mask)
        img = cv2.bitwise_and(img, img, mask = obstacles_mask)
        binary_mask = obstacles_mask > mask_confidence

        # combine with object detection stuff if we have that
        if self.object_detector is not None:
            frame, pcl, boxes, clouds, scores, labels, labels_text, masks = self.object_detector.detect(pclmsg)
            for i, mask in enumerate(masks):
                mask_temp = mask > 0.5
                mask_temp = mask_temp.reshape(mask_temp.shape[:2])
                binary_mask = np.bitwise_or(binary_mask, mask_temp)

        cv2.imshow('binmask', binary_mask.astype(np.float32))
        indices = np.argwhere(binary_mask)

        # set dist to infinity if x,y in mask = 0
        # otherwise keep dist if x,y in mask = 1
        # complexity is O(3N) so it should b fast
        pcl = pcl.reshape(pclmsg.height, pclmsg.width, 32)
        pcl = pcl.view(np.float32)
        temp = {}
        for y,x in indices:
            temp[y,x] = pcl[y,x,2]
        pcl[:,:,2] = float('inf')
        for y,x in indices:
            pcl[y,x,2] = temp[y,x]

        # create pcl
        pclmsg.data = pcl.flatten().tostring()

        # results
        cloud = pclmsg
        self.pub.publish(cloud)

        cv2.namedWindow('masked_image')
        cv2.imshow('masked_image', img)
        cv2.namedWindow('camera_Feed')
        cv2.imshow('camera_Feed', original_image)

        cv2.waitKey(3)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()


    def callback(self):
        try:
            self.image = rospy.wait_for_message("/xtion/rgb/image_raw",Image)
            img = self.bridge.imgmsg_to_cv2(self.image, 'bgr8')
            original_image = img

            hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            colour_low = np.array([10, 30, 20])
            colour_high = np.array([40, 255, 255])

            floor_mask = cv2.inRange(hsv_image, colour_low, colour_high)
            floor_mask = cv2.bitwise_not(floor_mask)
            img = cv2.bitwise_and(img, img, mask = floor_mask)
            obstacles = original_image - img

            cv2.namedWindow('masked_image')
            cv2.imshow('masked_image', img)
            cv2.namedWindow('camera_Feed')
            cv2.imshow('camera_Feed', original_image)

            cv2.waitKey(3)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()

        except CvBridgeError as ex:
            print(ex)




if __name__ == '__main__':
    from ycb_mask_rcnn import *

    rospy.init_node('floor_test')

    YCB_LABELS_FULL = [
                'marbles', 'clamps', 'plastic lemon',
                'lego duplo', 'cups', 'plate',
                'starkist tuna fish can', 'dice', 'foam brick',
                'plastic peach', 'spam potted meat can', 'windex spray bottle',
                'plastic pear', 'racquetball', 'marbles',
                'baseball', 'scotch brite dobie sponge', 'clamps',
                'cups', 'knife', 'plastic orange',
                'plastic plum', 'cups', 'pitcher base',
                'srub cleanser bottle', 'tennis ball', 'mini soccer ball',
                'adjustable wrench', 'cups', 'toy airplane',
                'toy airplane', 'cups', 'large marker',
                'mug', 'hammer', 'power drill',
                'lego duplo', 'plastic banana', 'cups',
                'lego duplo', 'jell-o chocolate pudding box', 'scissors',
                'wood block', 'domino sugar box', 'toy airplane',
                'lego duplo', 'master chef coffee can', 'golf ball',
                'chain', 'bowl', 'frenchs mustard bottle',
                'plastic strawberry', 'spoon', 'tomato soup can',
                'jell-o strawberry gelatin box', 'lego duplo', 'lego duplo',
                'colored wood blocks', 'cheez-it cracker box', 'soft ball',
                'padlock', 'toy airplane', 'colored wood blocks',
                'lego duplo', 'nine hole peg test', 'spatula',
                'cups', 'skillet lid', 'clamps',
                'cups', 'fork', 'toy airplane',
                'rubiks cube', 'phillips screwdriver', 'cups',
                'flat screwdriver', 'plastic apple', 'skillet',
                'cups'
                ]
    MODEL_PATH = os.path.join(rospkg.RosPack().get_path('execute_grasp_moveit_grasps'), 'src/robocup_grasps/robocup.weights')
    mask_rcnn = YcbMaskRCNN(MODEL_PATH, YCB_LABELS_FULL)

    try:
        MV = Move()
        FL = SegmentFloor(object_detector = mask_rcnn)
        MV.hand_to_default()
        MV.look_down()

        sub = rospy.Subscriber('/xtion/depth_registered/points', PointCloud2, FL.detect, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    #cv2.destroyAllWindows

    mask_rcnn = YcbMaskRCNN(MODEL_PATH, YCB_LABELS_FULL)
