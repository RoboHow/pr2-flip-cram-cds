#! /usr/bin/env python

# Script for testing CRAM2CDS client 
import roslib; roslib.load_manifest('pr2_example')
import rospy

# Import the SimpleActionClient
import actionlib

# Import the mesagges
import pr2_example.msg
from robohow_common_msgs.msg import MotionPhase
from robohow_common_msgs.msg import MotionModel
from robohow_common_msgs.msg import GaussianMixtureModel
from robohow_common_msgs.msg import GaussianDistribution

def CRAM2CDS_client():
    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('cram2cds', learning_actionlib.msg.CRAM2CDSAction)
    client.wait_for_server()

    # Send goal and wait for result
    desired_motion_phase_model = MotionPhase()
    desired_motion_phase_model.id = 'reaching'
    desired_motion_phase_model.object = 'table'
    desired_motion_phase_model.threshold = 0.01
    desired_motion_phase_model.attractor = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    goal = learning_actionlib.msg.CRAM2CDSGoal(desired_motion_phase_model)
    client.send_goal(goal)
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('cram2cds_client_py')
        result = CRAM2CDS_client()

        print "\n\n= = = = = = = = = = = = = = = = = = = = = "
        print "Result:"
        print result.success

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
