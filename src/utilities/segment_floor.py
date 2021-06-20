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

import ros_numpy


class SegmentFloor():
    def __init__(self):

        self.bridge = CvBridge()
        self.image = None
        self.cloud = None
        self.sensitivity = 10
        self.pub = rospy.Publisher('/object_aware_cloud', PointCloud2, queue_size=1)
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


    def detect(self, pclmsg, ycb_maskrcnn):
        self.ycb_maskrcnn = ycb_maskrcnn
        mask_confidence = 0.5
        old_pcl_msg = pclmsg

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
        self.object_aware_cloud = pclmsg
        self.identify_objects(self.object_aware_cloud, old_pcl_msg)
        self.pub.publish(self.object_aware_cloud)

        cv2.namedWindow('masked_image')
        cv2.imshow('masked_image', img)
        cv2.namedWindow('camera_Feed')
        cv2.imshow('camera_Feed', original_image)

        cv2.waitKey(3)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()


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


    def identify_objects(self, object_aware_cloud, pclmsg):
        #ycb_sub = rospy.Subscriber('segmentations/{}', 1, callback = self.get_point_cloud)
        # colour stuff
        np.random.seed(69)
        COLOURS = np.random.randint(0, 256, (128,3))
        alpha = 0.5
        frame, pcl, boxes, clouds, scores, labels, labels_text, masks = self.ycb_maskrcnn.detect(pclmsg, confidence=0.5)
        #print(labels_text)


        # output point clouds
        for i, cloud in enumerate(clouds):
            #cloud = self.segment_cloud(cloud, pclmsg)
            #self.object_aware_cloud += cloud
            #print cloud

            pub = rospy.Publisher('segmentations/{}'.format(i), PointCloud2, queue_size=1)
            pub.publish(cloud)
        cloud = clouds[0]
        for i in range(len(clouds)-1):
            cloud += clouds[i+1]
        
        print cloud



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

    try:
        MV = Move()
        FL = SegmentFloor()
        MV.hand_to_default()
        MV.look_down()

        sub = rospy.Subscriber('/xtion/depth_registered/points', PointCloud2, FL.detect, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    #cv2.destroyAllWindows
