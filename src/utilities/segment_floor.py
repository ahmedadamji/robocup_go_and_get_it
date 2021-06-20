#!/usr/bin/env python

import numpy as np
import os
import message_filters
import cv2

from PIL import Image
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge, CvBridgeError
from move import Move

from mask_rcnn import MaskRCNN
from ycb_mask_rcnn import YcbMaskRCNN

import rospy
import rospkg

import ros_numpy

YCB_LABELS_FULL = [
            'ycb_063-a_marbles', 'ycb_052_extra_large_clamp', 'ycb_014_lemon',
            'ycb_073-c_lego_duplo', 'ycb_065-f_cups', 'ycb_029_plate',
            'ycb_007_tuna_fish_can', 'ycb_062_dice', 'ycb_061_foam_brick',
            'ycb_015_peach', 'ycb_010_potted_meat_can', 'ycb_022_windex_bottle',
            'ycb_016_pear', 'ycb_057_racquetball', 'ycb_063-b_marbles',
            'ycb_055_baseball', 'ycb_026_sponge', 'ycb_050_medium_clamp',
            'ycb_065-c_cups', 'ycb_032_knife', 'ycb_017_orange',
            'ycb_018_plum', 'ycb_065-d_cups', 'ycb_019_pitcher_base',
            'ycb_021_bleach_cleanser', 'ycb_056_tennis_ball', 'ycb_053_mini_soccer_ball',
            'ycb_042_adjustable_wrench', 'ycb_065-h_cups', 'ycb_072-b_toy_airplane',
            'ycb_072-a_toy_airplane', 'ycb_065-b_cups', 'ycb_040_large_marker',
            'ycb_025_mug', 'ycb_048_hammer', 'ycb_035_power_drill',
            'ycb_073-e_lego_duplo', 'ycb_011_banana', 'ycb_065-a_cups',
            'ycb_073-d_lego_duplo', 'ycb_008_pudding_box', 'ycb_037_scissors',
            'ycb_036_wood_block', 'ycb_004_sugar_box', 'ycb_072-e_toy_airplane',
            'ycb_073-f_lego_duplo', 'ycb_002_master_chef_can', 'ycb_058_golf_ball',
            'ycb_059_chain', 'ycb_024_bowl', 'ycb_006_mustard_bottle',
            'ycb_012_strawberry', 'ycb_031_spoon', 'ycb_005_tomato_soup_can',
            'ycb_009_gelatin_box', 'ycb_073-b_lego_duplo', 'ycb_073-a_lego_duplo',
            'ycb_070-a_colored_wood_blocks', 'ycb_003_cracker_box', 'ycb_054_softball',
            'ycb_038_padlock', 'ycb_072-c_toy_airplane', 'ycb_070-b_colored_wood_blocks',
            'ycb_073-g_lego_duplo', 'ycb_071_nine_hole_peg_test', 'ycb_033_spatula',
            'ycb_065-j_cups', 'ycb_028_skillet_lid', 'ycb_051_large_clamp',
            'ycb_065-e_cups', 'ycb_030_fork', 'ycb_072-d_toy_airplane',
            'ycb_077_rubiks_cube', 'ycb_043_phillips_screwdriver', 'ycb_065-i_cups',
            'ycb_044_flat_screwdriver', 'ycb_013_apple', 'ycb_027_skillet',
            'ycb_065-g_cups'
            ]

class SegmentFloor():
    def __init__(self):

        self.bridge = CvBridge()
        self.image = None
        self.cloud = None
        self.sensitivity = 10
        self.pub = rospy.Publisher('/object_aware_cloud', PointCloud2, queue_size=1)
        rospy.sleep(1)

        MODEL_PATH = os.path.join(rospkg.RosPack().get_path('robocup_go_and_get_it'), 'src/utilities/robocup.weights')
        self.ycb_maskrcnn = YcbMaskRCNN(MODEL_PATH, YCB_LABELS_FULL)


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

        self.identify_objects(pclmsg)

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

        #print np.array(obstacles_mask).shape
        obstacles_mask = cv2.bitwise_or(obstacles_mask, self.objects_mask)

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
        self.pub.publish(self.object_aware_cloud)

        cv2.namedWindow('masked_image')
        cv2.imshow('masked_image', img)
        cv2.namedWindow('camera_Feed')
        cv2.imshow('camera_Feed', original_image)

        cv2.waitKey(3)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()




    def identify_objects(self, pclmsg):
        #ycb_sub = rospy.Subscriber('segmentations/{}', 1, callback = self.get_point_cloud)
        # colour stuff
        np.random.seed(69)
        COLOURS = np.random.randint(0, 256, (128,3))
        alpha = 0.5
        frame, pcl, boxes, clouds, scores, labels, labels_text, masks = self.ycb_maskrcnn.detect(pclmsg, confidence=0.5)
        previous_mask = masks[0]

        #print(labels_text)
        for i, mask in enumerate(masks):
            if i == 0:
                self.previous_mask = mask

        for i, mask in enumerate(masks):
            self.previous_mask = cv2.bitwise_or(mask, self.previous_mask)

            label = labels_text[i]
            #print label

        self.objects_mask = self.previous_mask
        #print np.array(self.objects_mask).shape






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
