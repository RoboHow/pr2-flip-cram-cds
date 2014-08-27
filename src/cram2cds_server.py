#! /usr/bin/env python

import roslib; roslib.load_manifest('pr2_example')
import rospy

import rospkg
import actionlib
import math
import numpy

from std_msgs.msg import String
from robohow_common_msgs.msg import MotionPhase
from robohow_common_msgs.msg import MotionModel
from robohow_common_msgs.msg import GaussianMixtureModel
from robohow_common_msgs.msg import GaussianDistribution
import pr2_example.msg

def write_numpy_array(prior, file):
    data = numpy.mat(prior)
    for data_slice in data:
        #numpy.savetxt(file, data_slice, delimiter='\t', fmt='%-5.6f')
	numpy.savetxt(file, data_slice, fmt='%-5.6f\t')

def write_to_file(name, ind1, ind2, prior, mean, cov):

    file = open(name, "w")
    file.write(str(ind1) + "\n")
    file.write(str(ind2) + "\n\n")
    write_numpy_array(prior, file)
    file.write("\n")

    mean_t = []
    for i in range(len(mean[0])):
	mean_t.append([row[i] for row in mean])
    write_numpy_array(mean_t, file)
    file.write("\n")

    for i in range(ind1):
	i_cov = zip(*[iter(cov[i])]*ind2)
	write_numpy_array(i_cov, file)
	file.write("\n")

    file.close()

def quaternion_from_matrix(matrix):
    q = numpy.empty((4, ), dtype=numpy.float64)
    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:4, :4]
    t = numpy.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    return q

class CRAM2CDSAction(object):
  # create messages that are used to publish feedback/result
  _feedback = pr2_example.msg.CRAM2CDSFeedback()
  _result   = pr2_example.msg.CRAM2CDSResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, pr2_example.msg.CRAM2CDSAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(10)
    success = True
    
    # append the seeds for the fibonacci sequence
    self._feedback.progress = 0
    
    # publish info to the console for the user
    rospy.loginfo('Starting loading of parameters to parameter server')
    
    try:
	rospack = rospkg.RosPack()
	fullpath = rospack.get_path('pr2_example')

	aux = goal.desired_motion_phase_model.attractor
	R = numpy.array( [ [ aux[0],  aux[1],   aux[2],  aux[3]  ],
                           [ aux[4],  aux[5],   aux[6],  aux[7]  ],
                           [ aux[8],  aux[9],   aux[10], aux[11] ],
			   [ aux[12], aux[13],  aux[14], aux[15] ] ], dtype=numpy.float64)
	q = quaternion_from_matrix(R)

	dd = numpy.array([ 0.0, 0.0, 0.38, 0.0 ]);
	rd = numpy.dot(R,dd);

	if goal.desired_motion_phase_model.id == "reaching":
	    print "reach"
	    extra = numpy.array([ 0.2299, 0.1042, -0.2500 ]);
	elif goal.desired_motion_phase_model.id == "swipping":
	    print "swipe"
	    extra = numpy.array([ 0.2299, 0.1042, -0.2500 ]);
	elif goal.desired_motion_phase_model.id == "flipping":
	    print "flip"
	    extra = numpy.array([ 0.2299, 0.1042, -0.2500 ]);
	elif goal.desired_motion_phase_model.id == "back":
	    print "back"
	    extra = numpy.array([ 0.2299, 0.1042, -0.2500 ]);


	cds_params = rospy.get_param("/cds_controller_pr2")
	cds_params['reaching_threshold'] = goal.desired_motion_phase_model.threshold
	cds_params['parent_frame'] = goal.desired_motion_phase_model.object
	cds_params['attractor_frame']['origin']['x'] = float(R[0][3]) + float(extra[0])
	cds_params['attractor_frame']['origin']['y'] = float(R[1][3]) + float(extra[1])
	cds_params['attractor_frame']['origin']['z'] = float(R[2][3]) + float(extra[2]) 
	cds_params['attractor_frame']['orientation']['x'] = float(q[0])
	cds_params['attractor_frame']['orientation']['y'] = float(q[1])
	cds_params['attractor_frame']['orientation']['z'] = float(q[2])
	cds_params['attractor_frame']['orientation']['w'] = float(q[3])

	cds_params['master_gmm_file'] = fullpath + "/data/masterDyn.gmm"
	cds_params['slave_gmm_file'] = fullpath + "/data/slaveDyn.gmm"
	cds_params['coupling_gmm_file'] = fullpath + "/data/couplingDyn.gmm"

	num_mm = len(goal.desired_motion_phase_model.described_by_motion_model)

	num_mm_slave = 0
	num_mm_master = 0
	num_mm_coupling = 0

	ind_mm_master = []
	ind_mm_slave = []
	ind_mm_coupling = []

	for i_mm in range(num_mm):
	    if goal.desired_motion_phase_model.described_by_motion_model[i_mm].described_by_GMM[0].id == 'masterDyn':
		num_mm_master = num_mm_master + 1
		ind_mm_master.append(i_mm)
	    elif goal.desired_motion_phase_model.described_by_motion_model[i_mm].described_by_GMM[0].id == 'slaveDyn':
		num_mm_slave = num_mm_slave + 1
		ind_mm_slave.append(i_mm)
	    elif goal.desired_motion_phase_model.described_by_motion_model[i_mm].described_by_GMM[0].id == 'couplingDyn':
		num_mm_coupling = num_mm_coupling + 1
		ind_mm_coupling.append(i_mm)

	num2_mm_master = len(goal.desired_motion_phase_model.described_by_motion_model[ind_mm_master[0]].described_by_GMM[0].gaussian_dist[0].mean)
	num2_mm_slave = len(goal.desired_motion_phase_model.described_by_motion_model[ind_mm_slave[0]].described_by_GMM[0].gaussian_dist[0].mean)
	num2_mm_coupling = len(goal.desired_motion_phase_model.described_by_motion_model[ind_mm_coupling[0]].described_by_GMM[0].gaussian_dist[0].mean)

	cov_master = []
	mean_master = []
	prior_master = []
	for i in range(num_mm_master):
	    prior_master.append(goal.desired_motion_phase_model.described_by_motion_model[ind_mm_master[i]].described_by_GMM[0].gaussian_dist[0].prior)
	    mean_master.append(goal.desired_motion_phase_model.described_by_motion_model[ind_mm_master[i]].described_by_GMM[0].gaussian_dist[0].mean)
	    cov_master.append(goal.desired_motion_phase_model.described_by_motion_model[ind_mm_master[i]].described_by_GMM[0].gaussian_dist[0].cov)

	cov_slave = []
	mean_slave = []
	prior_slave = []
	for i in range(num_mm_slave):
	    prior_slave.append(goal.desired_motion_phase_model.described_by_motion_model[ind_mm_slave[i]].described_by_GMM[0].gaussian_dist[0].prior)
	    mean_slave.append(goal.desired_motion_phase_model.described_by_motion_model[ind_mm_slave[i]].described_by_GMM[0].gaussian_dist[0].mean)
	    cov_slave.append(goal.desired_motion_phase_model.described_by_motion_model[ind_mm_slave[i]].described_by_GMM[0].gaussian_dist[0].cov)

	cov_coupling = []
	mean_coupling = []
	prior_coupling = []
	for i in range(num_mm_coupling):
	    prior_coupling.append(goal.desired_motion_phase_model.described_by_motion_model[ind_mm_coupling[i]].described_by_GMM[0].gaussian_dist[0].prior)
	    mean_coupling.append(goal.desired_motion_phase_model.described_by_motion_model[ind_mm_coupling[i]].described_by_GMM[0].gaussian_dist[0].mean)
	    cov_coupling.append(goal.desired_motion_phase_model.described_by_motion_model[ind_mm_coupling[i]].described_by_GMM[0].gaussian_dist[0].cov)

    	write_to_file(fullpath + "/data/masterDyn.gmm", num_mm_master, num2_mm_master, prior_master, mean_master, cov_master)
	write_to_file(fullpath + "/data/slaveDyn.gmm", num_mm_slave, num2_mm_slave, prior_slave, mean_slave, cov_slave)
	write_to_file(fullpath + "/data/couplingDyn.gmm", num_mm_coupling, num2_mm_coupling, prior_coupling, mean_coupling, cov_coupling)

	########## TEST ##########
	cds_params['master_gmm_file'] = fullpath + "/data/"+ goal.desired_motion_phase_model.id +"/masterDyn.gmm"
	cds_params['slave_gmm_file'] = fullpath + "/data/"+ goal.desired_motion_phase_model.id +"/slaveDyn.gmm"
	cds_params['coupling_gmm_file'] = fullpath + "/data/"+ goal.desired_motion_phase_model.id +"/couplingDyn.gmm"
	##########################

	cds_params['run_controller'] = 0.0
	cds_params['update_params'] = 1.0
	rospy.set_param("/cds_controller_pr2", cds_params)

	self._feedback.progress = 100      

    except rospy.ROSInterruptException:
        pass

    ###########################################################
    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
      rospy.loginfo('%s: Preempted' % self._action_name)
      self._feedback.progress = 0
      self._as.set_preempted()
      success = False

    # publish the feedback
    self._as.publish_feedback(self._feedback)
    r.sleep()
      
    if success:
      self._result.success = 100
      self._feedback.progress = 100
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)


if __name__ == '__main__':
  rospy.init_node('cram2cds')
  CRAM2CDSAction(rospy.get_name())
  rospy.spin()
