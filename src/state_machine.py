#!/usr/bin/env python
import rospy
import cv2
from smach import State, StateMachine
from utilities import Util, Move
from states import ApproachPositions

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
        StateMachine.add("approach_positions", ApproachPositions(util, move), transitions={"outcome1":"end", "outcome2": "end"})

        sm.execute()
    
    #cv2.waitKey(0)

    #rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("State Machine terminated...")