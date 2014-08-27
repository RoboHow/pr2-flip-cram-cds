#! /usr/bin/env python

# Script for testing QueryOWL server 
import roslib; roslib.load_manifest('pr2_example')
import rospy

# Import the SimpleActionClient
import actionlib

# Import the mesagges
import pr2_example.msg

def QueryOWL_client():
    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('queryOWL', learning_actionlib.msg.QueryOWLAction)
    client.wait_for_server()

    # Send goal and wait for result
    goal = learning_actionlib.msg.QueryOWLGoal(desired_motion_phase='reaching')
    client.send_goal(goal)
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('query_client_py')
        result = QueryOWL_client()
        print "Result:"
        # also print some parts to the terminal
        print "\n\n= = = = = = = = = = = = = = = = = = = = = "
        print result.desired_motion_phase_model.described_by_motion_model
	print result.desired_motion_phase_model.id
	print result.desired_motion_phase_model.object
	print result.desired_motion_phase_model.threshold
	print result.desired_motion_phase_model.attractor

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
