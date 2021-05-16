#!/usr/bin/env python
import rospy
import cv2
from smach import State, StateMachine
#from utilities import 
#from states import 

# main
def main():
    rospy.init_node("state_machine")



    # Create a SMACH state machine
    sm = StateMachine(outcomes=["outcome1", "end"])
    # Open the container

    with sm:
        # Add states to the container
        #StateMachine.add("approach_person", ApproachPerson(classify_objects, interaction, util, move, table), transitions={"outcome1":"detect_pointing_location", "outcome2": "end"})

        sm.execute()
    
    #cv2.waitKey(0)

    #rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("State Machine terminated...")