(in-package :pr2-example)


;;;  LEFT AND RIGHT ARM JOINT NAMES
(defparameter *l-arm-joint-names*
  '("l_upper_arm_roll_joint"
    "l_shoulder_pan_joint"
    "l_shoulder_lift_joint"
    "l_forearm_roll_joint"
    "l_elbow_flex_joint"
    "l_wrist_flex_joint"
    "l_wrist_roll_joint"))

(defparameter *r-arm-joint-names*
  '("r_upper_arm_roll_joint"
    "r_shoulder_pan_joint"
    "r_shoulder_lift_joint"
    "r_forearm_roll_joint"
    "r_elbow_flex_joint"
    "r_wrist_flex_joint"
    "r_wrist_roll_joint"))


;;; RIGHT AND LEFT ARM CONFIGURATIONS FOR A POSITION
(defparameter *l-arm-pouring-start-config*
  (cl-robot-models:make-robot-state
   "LASA" "PR2"
   (list
    (cl-robot-models:make-joint-state
     :joint-name "l_upper_arm_roll_joint" :joint-position 1.9643297630604963)
    (cl-robot-models:make-joint-state
     :joint-name "l_shoulder_pan_joint" :joint-position 1.265335905500992)
    (cl-robot-models:make-joint-state
     :joint-name "l_shoulder_lift_joint" :joint-position 1.2666995326579538)
    (cl-robot-models:make-joint-state
     :joint-name "l_forearm_roll_joint" :joint-position -5.81991983730232)
    (cl-robot-models:make-joint-state
     :joint-name "l_elbow_flex_joint" :joint-position -0.2625872772879775)
    (cl-robot-models:make-joint-state
     :joint-name "l_wrist_flex_joint" :joint-position -0.13242260444085052)
    (cl-robot-models:make-joint-state
     :joint-name "l_wrist_roll_joint" :joint-position -2.64))))

(defparameter *r-arm-pouring-start-config*
  (cl-robot-models:make-robot-state
   "LASA" "PR2"
   (list
    (cl-robot-models:make-joint-state
     :joint-name "r_upper_arm_roll_joint" :joint-position -1.9643297630604963)
    (cl-robot-models:make-joint-state
     :joint-name "r_shoulder_pan_joint" :joint-position -1.265335905500992)
    (cl-robot-models:make-joint-state
     :joint-name "r_shoulder_lift_joint" :joint-position 1.2666995326579538)
    (cl-robot-models:make-joint-state
     :joint-name "r_forearm_roll_joint" :joint-position 5.81991983730232)
    (cl-robot-models:make-joint-state
     :joint-name "r_elbow_flex_joint" :joint-position -0.2625872772879775)
    (cl-robot-models:make-joint-state
     :joint-name "r_wrist_flex_joint" :joint-position -0.13242260444085052)
    (cl-robot-models:make-joint-state
     :joint-name "r_wrist_roll_joint" :joint-position 2.64))))


;;; PR2 ARM POSITION CONTROLLERS
(defparameter *l-arm-position-controller-action-name* 
  "/l_arm_controller/joint_trajectory_action")

(defparameter *r-arm-position-controller-action-name* 
  "/r_arm_controller/joint_trajectory_action")

