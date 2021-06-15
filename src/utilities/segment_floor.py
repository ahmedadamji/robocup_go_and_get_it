#!/usr/bin/env python

import numpy as np
import os
import message_filters
import cv2

from PIL import Image
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge, CvBridgeError

import rospy
import rospkg


class SegmentFloor():
    def __init__(self):
    
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("xtion/rgb/image_raw", Image, self.callback)
        #self.depth_sub = message_filters.Subscriber("/xtion/depth_registered/points", PointCloud2)

        self.image = None
        self.cloud = None

        #self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 1, 0.1)
        #self.ts.registerCallback(self.callback)
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
        imgmsg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        return pcl, frame, imgmsg


    def detect(self):
        pclmsg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)

        pcl, frame, bgr_image = pclmsg_to_pcl_cv2_imgmsg(pclmsg)

        original_image = bgr_image

        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        colour_low = np.array([10, 30, 20])
        colour_high = np.array([40, 255, 255])

        floor_mask = cv2.inRange(hsv_image, colour_low, colour_high)
        floor_mask = cv2.bitwise_not(floor_mask)
        bgr_image = cv2.bitwise_and(bgr_image, bgr_image, mask = floor_mask)
        obstacles = original_image - bgr_image


        # extract segmented detection
        frame_out = np.take(frame_ds.reshape(frame_ds.shape[0] * frame_ds.shape[1], -1), indices, axis=0)
        pcl_out = np.take(pcl_ds.reshape(pcl_ds.shape[0] * pcl_ds.shape[1], -1), indices, axis=0)

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

        # append results
        masks.append(mask)
        boxes.append(pred_boxes[i])
        clouds.append(pclmsg_out)

        cv2.namedWindow('masked_image')
        cv2.imshow('masked_image', bgr_image)
        cv2.namedWindow('camera_Feed')
        cv2.imshow('camera_Feed', original_image)

        cv2.waitKey(3)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
                

    def callback(self):

        try:

            self.image = rospy.wait_for_message("/xtion/rgb/image_raw",Image)
            bgr_image = self.bridge.imgmsg_to_cv2(self.image, 'bgr8')

            original_image = bgr_image

            hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

            colour_low = np.array([10, 30, 20])
            colour_high = np.array([40, 255, 255])


            floor_mask = cv2.inRange(hsv_image, colour_low, colour_high)
            floor_mask = cv2.bitwise_not(floor_mask)
            bgr_image = cv2.bitwise_and(bgr_image, bgr_image, mask = floor_mask)
            obstacles = original_image - bgr_image


            cv2.namedWindow('masked_image')
            cv2.imshow('masked_image', bgr_image)
            cv2.namedWindow('camera_Feed')
            cv2.imshow('camera_Feed', original_image)

            cv2.waitKey(3)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()

        except CvBridgeError as ex:
            print(ex)


   


if __name__ == '__main__':

    rospy.init_node('floor_test')

    segment_mask = SegmentFloor()

    while not rospy.is_shutdown():

        FL = SegmentFloor()
        FL.callback()
        
        # output point clouds
        #for i, cloud in enumerate(clouds):
            #pub = rospy.Publisher('segmentations/{}'.format(i), PointCloud2, queue_size=1)
            #pub.publish(cloud)

            #pub = rospy.Publisher('segmentations'.format(i), PointCloud2, queue_size=1)
            #pub.publish(cloud)

    rospy.spin()
    #cv2.destroyAllWindows
