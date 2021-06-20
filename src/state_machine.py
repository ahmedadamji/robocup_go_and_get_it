#!/usr/bin/env python
import os
import rospy
import cv2
import rospkg
from smach import State, StateMachine
from utilities import Util, Move, YcbMaskRCNN, MaskRCNN, SegmentFloor
from states import ApproachPositions, ApproachPerson, ApproachRoomTwo, ApproachFoodCupboard, FindObject
import threading
import time

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

# main
def main():
    rospy.init_node("state_machine")
    move = Move()
    util = Util()
    segmentfloor = SegmentFloor()
    MODEL_PATH = os.path.join(rospkg.RosPack().get_path('robocup_go_and_get_it'), 'src/utilities/robocup.weights')
    ycb_maskrcnn = YcbMaskRCNN(MODEL_PATH, YCB_LABELS_FULL)



    # Create a SMACH state machine
    sm = StateMachine(outcomes=["outcome1", "end"])
    # Open the container

    with sm:
        # Add states to the container
        StateMachine.add("approach_room_2", ApproachRoomTwo(util, move), transitions={"outcome1":"approach_food_cupboard", "outcome2": "approach_food_cupboard"})
        StateMachine.add("approach_food_cupboard", ApproachFoodCupboard(util, move, ycb_maskrcnn, segmentfloor), transitions={"outcome1":"find_object", "outcome2": "find_object"})
        StateMachine.add("find_object", FindObject(util, move, ycb_maskrcnn), transitions={"outcome1":"end", "outcome2": "end"})
        #StateMachine.add("approach_person", ApproachPerson(util, move), transitions={"outcome1":"end", "outcome2": "end"})

        sm.execute()

    #cv2.waitKey(0)

    #rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("State Machine terminated...")

