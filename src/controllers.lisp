(in-package :pr2-example)


;;; PR2 ARM POSITION CONTROLLERS
(defparameter *l-arm-position-controller* nil)
(defparameter *r-arm-position-controller* nil)

(defun ensure-left-arm-position-controller ()
  (unless *l-arm-position-controller*
    (setf *l-arm-position-controller*
          (cram-pr2-controllers:make-pr2-arm-position-controller-handle
           *l-arm-position-controller-action-name* *l-arm-joint-names*))))

(defun ensure-right-arm-position-controller ()
  (unless *r-arm-position-controller*
    (setf *r-arm-position-controller*
          (cram-pr2-controllers:make-pr2-arm-position-controller-handle
           *r-arm-position-controller-action-name* *r-arm-joint-names*))))

(defun get-left-arm-position-controller ()
  (ensure-left-arm-position-controller)
  *l-arm-position-controller*)

(defun get-right-arm-position-controller ()
  (ensure-right-arm-position-controller)
  *r-arm-position-controller*)


;;; PR2 TORSO PARAMETERS AND CONTROLLER
(defparameter *spine-action-client* nil)

(defun ensure-spine-controller ()
  (unless *spine-action-client*
    (setf *spine-action-client*
	(actionlib:make-action-client 
	     "/torso_controller/position_joint_action"          
	     "pr2_controllers_msgs/SingleJointPositionAction") ))
	(actionlib:wait-for-server *spine-action-client*) )

(defun get-spine-controller ()
  (ensure-spine-controller)
  *spine-action-client*)


;;; CLIENT FOR QUERYING THE KNOWLEDGE BASE
(defparameter *query-result* nil)
(defparameter *query-action-client* nil)

(defun ensure-query-controller ()
  (unless *query-action-client*
    (setf *query-action-client*
	(actionlib:make-action-client 
	     "queryOWL"
	     "pr2_example/QueryOWLAction") ))
	(actionlib:wait-for-server *query-action-client*) )

(defun get-query-controller ()
  (ensure-query-controller)
  *query-action-client*)


;;; CLIENT FOR SENDING COMMANDS TO CDS_CONTROLLER
(defparameter *cram2cds-goal* nil)
(defparameter *cram2cds-action-client* nil)

(defun ensure-cram2cds-controller ()
  (unless *cram2cds-action-client*
    (setf *cram2cds-action-client*
	(actionlib:make-action-client 
	     "cram2cds"
	     "pr2_example/CRAM2CDSAction") ))
	(actionlib:wait-for-server *cram2cds-action-client*) )

(defun get-cram2cds-controller ()
  (ensure-cram2cds-controller)
  *cram2cds-action-client*)


;;; LISP TRANSFORM LISTENER
(defparameter *transform-listener* nil)

(defun ensure-listener ()
  (unless *transform-listener*
    (setf *transform-listener*
	(make-instance 'cl-tf:transform-listener))))

(defun get-listener ()
  (ensure-listener)
  *transform-listener*)


;;; DEFINITION OF SOME VARIABLES
(defparameter success nil)
(defparameter successF nil)

(defparameter gmm nil)
(defparameter segment nil)
(defparameter index nil)

(defparameter *pose* nil)
(defparameter pos nil)
(defparameter ori nil)
(defparameter poseerror nil)

(defparameter oven-pose nil)
(defparameter pancake-pose nil)
(defparameter oven-pose-old nil)
(defparameter pancake-pose-old nil)

(defparameter aux-oven1 nil)
(defparameter aux-oven2 nil)
