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
import ros_numpy

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

# main
def main():
    rospy.init_node("state_machine")
    MODEL_PATH = os.path.join(rospkg.RosPack().get_path('robocup_go_and_get_it'), 'src/utilities/robocup.weights')
    ycb_maskrcnn = YcbMaskRCNN(MODEL_PATH, YCB_LABELS_FULL)
    move = Move()
    util = Util()
    segmentfloor = SegmentFloor(object_detector=ycb_maskrcnn)


    # Create a SMACH state machine
    sm = StateMachine(outcomes=["outcome1", "end"])
    # Open the container

    with sm:
        # Add states to the container
        StateMachine.add("approach_room_2", ApproachRoomTwo(util, move), transitions={"outcome1":"approach_food_cupboard", "outcome2": "approach_food_cupboard"})
        StateMachine.add("approach_food_cupboard", ApproachFoodCupboard(util, move, ycb_maskrcnn, segmentfloor), transitions={"outcome1":"find_object", "outcome2": "end"})
        StateMachine.add("find_object", FindObject(util, move, ycb_maskrcnn), transitions={"outcome1":"end", "outcome2": "end"})
        StateMachine.add("approach_person", ApproachPerson(util, move), transitions={"outcome1":"end", "outcome2": "end"})

        sm.execute()

    #cv2.waitKey(0)

    #rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("State Machine terminated...")

