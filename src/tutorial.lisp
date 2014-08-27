(in-package :pr2-example)

;;;  DEFINITION OF OTHER VARIABLES
(defvar *cmd-vel-pub* nil "velocity commands ROS publisher")
(defvar *l-grip-pub* nil "left gripper commands ROS publisher")
(defvar *r-grip-pub* nil "right gripper commands ROS publisher")
(defvar *l-arm-pub* nil "left arm commands ROS publisher")
(defvar *r-arm-pub* nil "right arm commands ROS publisher")
(defvar l-msg nil "right arm commands ROS publisher")
(defvar r-msgt nil "right arm commands ROS publisher")

;;;  INITIALIZATION FUNCTIONS
(defun ini ()
    ;;;(start-ros-node "pr2_client")
    (cram-roslisp-common::startup-ros)
    (init-controllers)
    (move-arms-away)
    (get-ready)
)

(defun init-controllers ()
  "Initializes all the controllers"
  (cram-roslisp-common::ros-time-init)
  (cram-roslisp-common::ros-tf-init)
  (cram-gazebo-utilities::init-cram-gazebo-utilities)
  (location-costmap::location-costmap-vis-init)
  (occupancy-grid-costmap::init-occupancy-grid-costmap)
  (point-head-process-module::init-point-head-action)
  (pr2-navigation-process-module::init-pr2-navigation-process-module)
  (semantic-map-collision-environment::init-semantic-map-collision-environment)
  (pr2-manipulation-process-module::init-collision-environment)
  (pr2-manipulation-process-module::init-pr2-manipulation-process-module)

  (init-larm-controller "l_arm_controller")
  (init-rarm-controller "r_arm_controller")
  (init-left-gripper "l_gripper_controller")
  (init-right-gripper "r_gripper_controller")
  (init-base-controller "base_controller")
  (get-left-arm-position-controller)
  (get-right-arm-position-controller)
  (get-spine-controller)
  (get-query-controller)
  (get-cram2cds-controller)
  (get-listener)

  (roslisp:ros-info (pr2-example) "Basic controllers ON.")
)

(defun init-larm-controller (name)
  "subscribes to topics for the left arm controller."
  (setf *l-arm-pub* (advertise (format nil "~a/command" name)
                                 "trajectory_msgs/JointTrajectory")))

(defun init-rarm-controller (name)
  "subscribes to topics for the right arm controller."
  (setf *r-arm-pub* (advertise (format nil "~a/command" name)
                                 "trajectory_msgs/JointTrajectory"))
)

(defun init-base-controller (name)
  "subscribes to topics for the base controller."
  (setf *cmd-vel-pub* (advertise (format nil "~a/command" name)
                                 "geometry_msgs/Twist")))

(defun init-left-gripper (name)
  "Advertises to topics for the left gripper controller."
  (setf *l-grip-pub* (advertise (format nil "~a/command" name)
                                 "pr2_controllers_msgs/Pr2GripperCommand")))

(defun init-right-gripper (name)
  "Advertises to topics for the right gripper controller."
  (setf *r-grip-pub* (advertise (format nil "~a/command" name)
                                 "pr2_controllers_msgs/Pr2GripperCommand")))

(defun get-ready ()
  (move-spine)
  (move-arms-away)
  (dotimes (i 12) 
    (send-vel-cmd 1.0 1.0)
    (wait-duration 0.1))
  (dotimes (i 25) 
    (send-vel-cmd 1.0 0.0)
    (wait-duration 0.1))
)


;;;  LOW LEVEL CONTROL FUNCTIONS
(defun send-vel-cmd (lin ang)
  "Function to send velocity commands"
  (publish *cmd-vel-pub* (make-message "geometry_msgs/Twist"
                                       :linear (make-msg "geometry_msgs/Vector3"
                                                         :x lin)
                                       :angular (make-msg "geometry_msgs/Vector3"
                                                          :z ang))))

(defun send-left-gripper-cmd (pos force)
  "Function to send commands to left gripper"
  (publish *l-grip-pub* (make-message "pr2_controllers_msgs/Pr2GripperCommand"
                                       :position pos
                                       :max_effort force)))

(defun send-right-gripper-cmd (pos force)
  "Function to send commands to right gripper"
  (publish *r-grip-pub* (make-message "pr2_controllers_msgs/Pr2GripperCommand"
                                       :position pos
                                       :max_effort force)))

(defun move-arms-away ()
  (let* ((joint-values-left
           (list (cons "l_shoulder_pan_joint" 1.1091906456385282d0)
                 (cons "l_shoulder_lift_joint" -0.3516685176176528d0)
                 (cons "l_upper_arm_roll_joint" 0.3357926749691642d0)
                 (cons "l_elbow_flex_joint" -1.919705962915871d0)
                 (cons "l_forearm_roll_joint" 13.518303816702831d0)
                 (cons "l_wrist_flex_joint" -1.403336851649972d0)
                 (cons "l_wrist_roll_joint" -20.099177425140844d0)))
         (joint-values-right
           (list (cons "r_shoulder_pan_joint" -1.1097042291545055d0)
                 (cons "r_shoulder_lift_joint" -0.35204314859086017d0)
                 (cons "r_upper_arm_roll_joint" -0.3710246292874033d0)
                 (cons "r_elbow_flex_joint" -1.920429816472789d0)
                 (cons "r_forearm_roll_joint" 24.297695480641856d0)
                 (cons "r_wrist_flex_joint" -1.4051073464316054d0)
                 (cons "r_wrist_roll_joint" -126.81492027877437d0))))

         (publish *l-arm-pub*
           (roslisp:make-message
            "trajectory_msgs/JointTrajectory"
            (stamp header) (roslisp:ros-time)
            joint_names (map 'vector #'car joint-values-left)
            points (vector
                    (roslisp:make-message
                     "trajectory_msgs/JointTrajectoryPoint"
                     positions (map 'vector #'cdr joint-values-left)
                     velocities (map 'vector #'identity
                                     (make-list (length joint-values-left)
                                                :initial-element 0.0))
                     accelerations (map 'vector #'identity
                                        (make-list (length joint-values-left)
                                                   :initial-element 0.0))
                     time_from_start 5.0))))
         (publish *r-arm-pub*
           (roslisp:make-message
            "trajectory_msgs/JointTrajectory"
            (stamp header) (roslisp:ros-time)
            joint_names (map 'vector #'car joint-values-right)
            points (vector
                    (roslisp:make-message
                     "trajectory_msgs/JointTrajectoryPoint"
                     positions (map 'vector #'cdr joint-values-right)
                     velocities (map 'vector #'identity
                                     (make-list (length joint-values-right)
                                                :initial-element 0.0))
                     accelerations (map 'vector #'identity
                                        (make-list (length joint-values-right)
                                                   :initial-element 0.0))
                     time_from_start 5.0))))

))


;;;  AUXILIAR FUNCTIONS
(defun print-var (var)
  (roslisp:ros-info (pr2-example) "Value: ~a~%" var)
)

(defun mv-lr ()
  "Moving arms"
  (pr2-controllers:move-arm *l-arm-position-controller* *l-arm-pouring-start-config* 5.0)
  (pr2-controllers:move-arm *r-arm-position-controller* *r-arm-pouring-start-config* 5.0))

(defun spine-feedback-cb (msg)
  (roslisp:with-fields (velocity)
      msg
    (format t "~a~%" velocity)))


(defun move-spine ()           
    (actionlib:call-goal *spine-action-client* (actionlib:make-action-goal *spine-action-client*
			position 0.1
			max_velocity 100.0)
        ;;;    :feedback-cb 'spine-feedback-cb
    )
)

(defun mv-spine (pos)
    (actionlib:call-goal *spine-action-client* (actionlib:make-action-goal *spine-action-client*
			position pos
			max_velocity 100.0)
        ;;;    :feedback-cb 'spine-feedback-cb
    )
)

(defun move-custom-position (pos1 pos2 pos3 pos4 pos5 pos6 pos7)
  "Moving right arm to pregrasp position (Without planning, just by hand)"
  (pr2-controllers:move-arm *r-arm-position-controller* 
  (cl-robot-models:make-robot-state
   "LASA" "PR2"
   (list
    (cl-robot-models:make-joint-state
     :joint-name "r_upper_arm_roll_joint" :joint-position pos1)
    (cl-robot-models:make-joint-state
     :joint-name "r_shoulder_pan_joint" :joint-position pos2)
    (cl-robot-models:make-joint-state
     :joint-name "r_shoulder_lift_joint" :joint-position pos3)
    (cl-robot-models:make-joint-state
     :joint-name "r_forearm_roll_joint" :joint-position pos4)
    (cl-robot-models:make-joint-state
     :joint-name "r_elbow_flex_joint" :joint-position pos5)
    (cl-robot-models:make-joint-state
     :joint-name "r_wrist_flex_joint" :joint-position pos6)
    (cl-robot-models:make-joint-state
     :joint-name "r_wrist_roll_joint" :joint-position pos7))) 2.0))

(defun move-custom-position-l (pos1 pos2 pos3 pos4 pos5 pos6 pos7)
  "Moving left arm to pregrasp position (Without planning, just by hand)"
  (pr2-controllers:move-arm *l-arm-position-controller* 
  (cl-robot-models:make-robot-state
   "LASA" "PR2"
   (list
    (cl-robot-models:make-joint-state
     :joint-name "l_upper_arm_roll_joint" :joint-position pos1)
    (cl-robot-models:make-joint-state
     :joint-name "l_shoulder_pan_joint" :joint-position pos2)
    (cl-robot-models:make-joint-state
     :joint-name "l_shoulder_lift_joint" :joint-position pos3)
    (cl-robot-models:make-joint-state
     :joint-name "l_forearm_roll_joint" :joint-position pos4)
    (cl-robot-models:make-joint-state
     :joint-name "l_elbow_flex_joint" :joint-position pos5)
    (cl-robot-models:make-joint-state
     :joint-name "l_wrist_flex_joint" :joint-position pos6)
    (cl-robot-models:make-joint-state
     :joint-name "l_wrist_roll_joint" :joint-position pos7))) 2.0))


;;; FUNCTIONS FOR QUERYING AND SENDING COMMANDS TO CDS CONTROLLER
(defun query-feedback-cb (msg)
  (roslisp:with-fields (found)
      msg
    (format t "~a~%" found)))

(defun ex-query (segment)
  (setf *query-result*
    (actionlib:call-goal
      *query-action-client* 
      (actionlib:make-action-goal *query-action-client* desired_motion_phase segment)
      ;;;:feedback-cb 'query-feedback-cb
    )
  )
)

(defun display-query-result ()
  (roslisp:with-fields (desired_motion_phase_model)
    *query-result*
    (format t "~a~%" desired_motion_phase_model))
  (roslisp:ros-info (pr2-example) "Got result !!")
)

(defun cram2cds-feedback-cb (msg)
  (roslisp:with-fields (progress)
    msg
  (format t "Progress ~a~%" progress))
)

(defun send-cram2cds-msg ()
  (roslisp:with-fields (desired_motion_phase_model)
    *query-result*
    (setf *cram2cds-goal* desired_motion_phase_model)  
  )
  (actionlib:call-goal
    *cram2cds-action-client* 
    (actionlib:make-action-goal *cram2cds-action-client* desired_motion_phase_model *cram2cds-goal*)
    :feedback-cb 'cram2cds-feedback-cb
  )
  (roslisp:ros-info (pr2-example) "Sent message CRAM2CDS.")
)


;;; FUNCTIONS FOR INDIVIDUALLY EXECUTING SEGMENTS
(defun mv (segment)
    (ex-query segment)
    (send-cram2cds-msg)
)

(defun mv-reach ()
    (ex-query "reaching")
    (send-cram2cds-msg)
)

(defun mv-swipe ()
    (ex-query "swipping")
    (send-cram2cds-msg)
)

(defun mv-flip ()
    (ex-query "flipping")
    (send-cram2cds-msg)
)

(defun mv-back ()
    (ex-query "back")
    (send-cram2cds-msg)
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; FUNCTIONS FOR WORKING WITH PRE AND POST CONDITIONS (EXPERIMENTAL)

(defun execute-movement (action-desig)
  (setq gmm (reference action-desig))
  (setq segment (second gmm))
  (setq index (gmmseds-id segment))
  (cond ( (= index 1) (mv-reach))
        ( (= index 2) (mv-swipe))
        ( (= index 3) (mv-flip) )
        ( (= index 4) (mv-back) )
  )
)

(defun store-oven (loc-desig)
  (setq aux-oven (reference loc-desig))
  (setq oven-pose (object-pose aux-oven))
  (setq aux-oven (cl-transforms:origin oven-pose))
  (print-var aux-oven)
)

(defun store-pancake (loc-desig)
  (setq aux-pancake (reference loc-desig))
  (setq pancake-pose (object-pose aux-pancake))
  (setq aux-pancake (cl-transforms:origin pancake-pose))
  (print-var aux-pancake)
)


(defun check-pre-condition (action-desig)
  (setq gmm (reference action-desig))
  (setq segment (second gmm))
  (setq index (gmmseds-id segment))
  (cond ( (= index 1) (pre-condition-reach))
        ( (= index 2) (pre-condition-swipe))
        ( (= index 3) (pre-condition-flip) )
  )
)

(defun check-post-condition (action-desig)
  (setq gmm (reference action-desig))
  (setq segment (second gmm))
  (setq index (gmmseds-id segment))
  (cond ( (= index 1) (post-condition-reach))
        ( (= index 2) (post-condition-swipe))
        ( (= index 3) (post-condition-flip) )
  )
)

;;;                                        ;;;
;;;   REACHING PRE- AND POST- CONDITIONS   ;;;
;;;                                        ;;;

(defun pre-condition-reach ()
  (setq successF 1)
  (find-oven 0)
  (cond ( (= success 1) (find-oven 1) )
        ( (= success 0) (setq successF 0) )
  )

  (find-pancake 0)
  (cond ( (= success 1) (find-pancake 1) )
        ( (= success 0) (setq successF 0) )
  )
)

(defun post-condition-reach ()
  (get-reach-error 0.1)
  (cond ( (= success 1) (setq successF 1) )
        ( (= success 0) (setq successF 0) )
  )
  
)

(defun get-desEE-EE-error ()
  (let ((time (roslisp:ros-time)))
    (cl-tf:wait-for-transform
     *transform-listener* :time time
                          :source-frame "/des_EEframe"
                          :target-frame "/EEframe")
    (cl-tf:transform-pose
     *transform-listener* :pose (tf:make-pose-stamped
                            "/EEframe"
                            time
                            (tf:make-identity-vector)
                            (tf:make-identity-rotation))
                          :target-frame "/des_EEframe")
  )
)

(defun get-reach-error (threshold)
  (setq *pose* (get-desEE-EE-error))
  (setq pos (cl-transforms:origin *pose*))
  (setq ori (cl-transforms:orientation *pose*))
  (setq poseerror (v-norm pos))
  (cond
    ((> poseerror threshold) (setq successF 0))
    ((<= poseerror threshold) (setq successF 1))
  )
  (print-var successF)
)

;;;                                        ;;;
;;;   SWIPPING PRE- AND POST- CONDITIONS   ;;;
;;;                                        ;;;

(defun pre-condition-swipe ()
  (setq oven-pose-old oven-pose)
  (setq pancake-pose-old pancake-pose)

  (setq successF 1)
  (find-oven 0)
  (cond ( (= success 1) (find-oven 1) )
        ( (= success 0) (setq successF 0) )
  )

  (find-pancake 0)
  (cond ( (= success 1) (find-pancake 1) )
        ( (= success 0) (setq successF 0) )
  )

  (setq aux-oven1 (cl-transforms:origin oven-pose))
  (setq aux-oven2 (cl-transforms:origin oven-pose-old))
  (setq aux-oven (resta aux-oven1 aux-oven2))
  (setq poserror (v-norm aux-oven))
  (print-var poserror)
  (cond
    ((> poserror 0.1) (setq successF 0))
    ((<= poserror 0.1) (setq successF 1))
  )
  (print-var successF)

)

(defun resta (v-1 v-2)
  (make-3d-vector (- (x v-1) (x v-2))
                  (- (y v-1) (y v-2))
                  (- (z v-1) (z v-2))
  )
)

(defun post-condition-swipe ()
  (setq successF 1)
  (setq *pose* (get-spatula-pos))
  (find-pancake 0)
  (cond ( (= success 1) (find-pancake 1) )
        ( (= success 0) (setq successF 0) )
  )

)

(defun get-spatula-pos ()
  (let ((time (roslisp:ros-time)))
    (cl-tf:wait-for-transform
     *transform-listener* :time time
                          :source-frame "/map"
                          :target-frame "/spatula")
    (cl-tf:transform-pose
     *transform-listener* :pose (tf:make-pose-stamped
                            "/spatula"
                            time
                            (tf:make-identity-vector)
                            (tf:make-identity-rotation))
                          :target-frame "/map")
  )
)
