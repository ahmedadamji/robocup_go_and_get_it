#!/usr/bin/env python
import rospy
import actionlib

from smach import State
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import numpy as np
import cv2

from robocup_grasps import RobocupGrasps

class FindObject(State):
    def __init__(self, util, move, ycb_maskrcnn):
        #rospy.loginfo("FindObject state initialized")
        
        State.__init__(self, outcomes=["outcome1","outcome2"])

        #creates an instance of util class to transform point frames
        self.util = util
        #creates an instance of move class to move robot across the map
        self.move = move

        self.ycb_maskrcnn = ycb_maskrcnn
        self.rg = RobocupGrasps()


    def identify_objects(self, target_name):
        #ycb_sub = rospy.Subscriber('segmentations/{}', 1, callback = self.get_point_cloud)
        # colour stuff
        np.random.seed(69)
        COLOURS = np.random.randint(0, 256, (128,3))
        alpha = 0.5
        pclmsg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
        frame, pcl, boxes, clouds, scores, labels, labels_text, masks = self.ycb_maskrcnn.detect(pclmsg, confidence=0.5)

        for i, mask in enumerate(masks):
            label = labels_text[i]
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

            print label, target_name
            if target_name in label or label in target_name:
                if clouds[i].height > 0:
                    return clouds[i]

        return None


    def grasp_object(self):
        grasp_main()


    def execute(self, userdata, wait=True):
        OBJECT_LIST = [
            "ycb_005_tomato_soup_can",   # Tomato soup can
            "ycb_010_potted_meat_can",   # Spam potted meat can
            "ycb_006_mustard_bottle",    # French's mustard bottle
            "ycb_004_sugar_box",         # Domino sugar box
            "ycb_013_apple",             # Plastic apple
            "ycb_017_orange",            # Plastic orange
            "ycb_015_peach",             # Plastic peach
            "ycb_016_pear",              # Plastic pear
            "ycb_011_banana",            # Plastic banana
            "ycb_003_cracker_box",       # Cheez-it cracker box
            "ycb_014_lemon",             # Plastic lemon
            "ycb_018_plum"               # Plastic plum
            "ycb_012_strawberry",        # Plastic strawberries
            "ycb_007_tuna_fish_can",     # Starkist tuna fish can
            "ycb_008_pudding_box",       # Jell-o chocolate pudding box
            "ycb_009_gelatin_box",       # Jell-o strawberry gelatin box
            "ycb_002_master_chef_can",   # Master chef coffee can
        ]

        rospy.loginfo("FindObject state executing")
        ## REMEMBER TO REMOVE THIS BEFORE THE COMPETITION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        pub = rospy.Publisher('/message', std_msgs.msg.String, queue_size=10, latch=True)
        # pub.publish(std_msgs.msg.String("mustard"))

        target_name = rospy.wait_for_message("/message", std_msgs.msg.String).data
        target_name = target_name.split(' ')[0]

        for target_name in OBJECT_LIST:
            torso_height = 0.0
            while not rospy.is_shutdown():
                result = None
                for i in xrange(2):
                    # setup moveit, octomap and robot
                    self.rg.detach_object()
                    self.rg.remove_object()
                    self.rg.clear_octomap()
                    self.move.set_torso_height(torso_height)
                    self.move.look_down(-0.80)

                    # detect the target
                    result = self.identify_objects(target_name.lower())
                    if result is not None:
                        # do the moveit and return outcome1 if it works
                        moveit_goal = self.rg.main(result)
                        if moveit_goal and not self.rg.test_grippers_closed():
                            return "outcome1"
                        self.move.hand_to_default(wait=True)

                if torso_height >= 0.35 or result is not None:
                    break
                torso_height = min(torso_height + 0.13, 0.35)

        return "outcome2"
            