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
    def __init__(self):

        self.bridge = CvBridge()
        self.image = None
        self.cloud = None
        self.sensitivity = 10


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


    def detect(self):
        mask_confidence = 0.5

        #subscriber to depth data
        pclmsg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)

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
        #obstacles = original_image - img
        obstacles = img

        binary_mask = obstacles_mask > mask_confidence
        binary_mask = binary_mask.flatten()
        indices = np.argwhere(binary_mask).flatten()

        pcl_out = np.take(pcl.reshape(pcl.shape[0] * pcl.shape[1], -1), indices, axis=0)

        # create pcl
        pclmsg_out = PointCloud2()
        pclmsg_out.header       = pclmsg.header
        pclmsg_out.height       = pcl_out.shape[0]
        pclmsg_out.width        = 1
        pclmsg_out.fields       = pclmsg.fields
        pclmsg_out.is_bigendian = pclmsg.is_bigendian
        pclmsg_out.point_step   = pclmsg.point_step
        pclmsg_out.row_step     = pcl_out.shape[1]
        pclmsg_out.is_dense     = pclmsg.is_dense
        pclmsg_out.data         = pcl_out.flatten().tostring()

        # results
        cloud = pclmsg_out
        pub = rospy.Publisher('segmentations', PointCloud2, queue_size=1)
        pub.publish(cloud)

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

    rospy.init_node('floor_test')

    while not rospy.is_shutdown():

        MV = Move()
        MV.look_down()
        FL = SegmentFloor()
        FL.detect()

    rospy.spin()
    #cv2.destroyAllWindows
