#! /usr/bin/env python

import roslib; roslib.load_manifest('pr2_example')
import rospy

import json_prolog
import actionlib
import rospy

from std_msgs.msg import String
from robohow_common_msgs.msg import MotionPhase
from robohow_common_msgs.msg import MotionModel
from robohow_common_msgs.msg import GaussianMixtureModel
from robohow_common_msgs.msg import GaussianDistribution
from pr2_example.msg import *

class QueryOWLAction(object):
  # create messages that are used to publish feedback/result
  _feedback = pr2_example.msg.QueryOWLFeedback()
  _result   = pr2_example.msg.QueryOWLResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, pr2_example.msg.QueryOWLAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(100)
    success = True
    
    # append the seeds for the fibonacci sequence
    self._feedback.found = 1
    
    # publish info to the console for the user
    rospy.loginfo('Retrieving information from Knowledge Base')
    
    # start executing the action
    prolog = json_prolog.Prolog()

    try:
        
        query = prolog.query("""rdfs_subclass_of(Phase, seds:'SEDSMotion'),
                phase_properties(Phase, ID, Object, Threshold, Attractor, Models),
                member(Model, Models),
                motion_properties(Model, Type, GMMs),
                member(GMM, GMMs),
                gmm_properties(GMM, GMMType, InputType, InputDim, OutputType, OutputDim, Gaussians),
                member(Gaussian, Gaussians),
                gaussian_components(Gaussian, Mean, Cov, Prior),
                vector_elements(Mean, MeanVec),
                matrix_elements(Cov, CovMat)""")
        
        phaseModel = MotionPhase()
        for solution in query.solutions():

	    gd = solution['Gaussian'].encode('ascii','ignore')
	    gd = gd[gd.index('#')+1:gd.index('_')]
            gauss = GaussianDistribution(
                gd,
                solution['Prior'],
                solution['MeanVec'],
                solution['CovMat'])

            gmm = GaussianMixtureModel()
            gmm.id = solution['GMM'].encode('ascii','ignore')
	    gmm.id = gmm.id[gmm.id.index('#')+1:gmm.id.index('_')]

            gmm.gaussian_dist.append(gauss)

	    gmm.type = solution['GMMType'].encode('ascii','ignore')
	    gmm.type = gmm.type[gmm.type.index('#')+1:len(gmm.type)]

            gmm.input_type  = solution['InputType'].encode('ascii','ignore')
            gmm.output_type = solution['InputType'].encode('ascii','ignore')
            #gmm.input_dim   = solution['InputDim']
            #gmm.output_dim  = solution['InputDim']

            m = MotionModel()
            m.id = solution['Model'].encode('ascii','ignore')
	    m.id = m.id[m.id.index('#')+1:m.id.index('_')]

            m.type = solution['Type'].encode('ascii','ignore')
	    m.type = m.type[m.type.index('#')+1:len(m.type)]

            m.described_by_GMM.append(gmm)
          
            p = MotionPhase()
            p.id = solution['Phase'].encode('ascii','ignore')
	    p.id = p.id[p.id.index('#')+1:p.id.index('_')]

	    p.object = solution['Object']
	    p.object = p.object[u'term'].pop(1)[u'term'].pop(2)

	    p.threshold = solution['Threshold']
	    p.threshold = p.threshold[u'term'].pop(1)[u'term'].pop(2)

	    p.attractor = solution['Attractor']

            p.described_by_motion_model.append(m)

	    if p.id == goal.desired_motion_phase:
		phaseModel.described_by_motion_model.append(m)
		phaseModel.id = p.id
		phaseModel.object = p.object.encode('ascii','ignore')
		phaseModel.attractor = p.attractor
		phaseModel.threshold = float(p.threshold)

        query.finish()
        
    except rospy.ROSInterruptException:
        pass

    ###########################################################
    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
      rospy.loginfo('%s: Preempted' % self._action_name)
      self._feedback.found = 1
      self._as.set_preempted()
      success = False

    # publish the feedback
    self._as.publish_feedback(self._feedback)
    r.sleep()
      
    if success:
      self._result.desired_motion_phase_model.described_by_motion_model = phaseModel.described_by_motion_model
      self._result.desired_motion_phase_model.id = phaseModel.id
      self._result.desired_motion_phase_model.object = phaseModel.object
      self._result.desired_motion_phase_model.threshold = phaseModel.threshold
      self._result.desired_motion_phase_model.attractor = phaseModel.attractor

      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)

      
if __name__ == '__main__':
  rospy.init_node('queryOWL')
  QueryOWLAction(rospy.get_name())
  rospy.spin()
