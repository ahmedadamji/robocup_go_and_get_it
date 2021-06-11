#!/usr/bin/env python
import rospy
import cv2
from smach import State, StateMachine
from utilities import Util, Move
from states import ApproachPositions, ApproachPerson, ApproachRoomTwo, ApproachFoodCupboard

# main
def main():
    rospy.init_node("state_machine")
    move = Move()
    util = Util()

    # Create a SMACH state machine
    sm = StateMachine(outcomes=["outcome1", "end"])
    # Open the container

    with sm:
        # Add states to the container
        #StateMachine.add("approach_positions", Approachpositions(util, move), transitions={"outcome1":"end", "outcome2": "end"})
        StateMachine.add("approach_room_2", ApproachRoomTwo(util, move), transitions={"outcome1":"approach_food_cupboard", "outcome2": "approach_food_cupboard"})
        StateMachine.add("approach_food_cupboard", ApproachFoodCupboard(util, move), transitions={"outcome1":"approach_person", "outcome2": "approach_person"})
        StateMachine.add("approach_person", ApproachPerson(util, move), transitions={"outcome1":"end", "outcome2": "end"})

        sm.execute()
    
    #cv2.waitKey(0)

    #rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("State Machine terminated...")