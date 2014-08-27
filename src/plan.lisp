(in-package :pr2-example)

(defmacro with-process-modules (&body body)
  `(cpm:with-process-modules-running
       (pr2-manipulation-process-module:pr2-manipulation-process-module
        pr2-navigation-process-module:pr2-navigation-process-module
        gazebo-perception-process-module:gazebo-perception-process-module
        point-head-process-module:point-head-process-module)
     ,@body))


;;;  SIMPLE TIME-DEPENDENT PLAN
(def-top-level-cram-function make-a-pancake ()
  (with-process-modules
    (cpl-desig-supp:with-designators
      (
        (reaching (action `( (type gmm) (movement reach) )))
        (swipping (action `( (type gmm) (movement swipe) )))
        (flipping (action `( (type gmm) (movement flip) )))
        (back     (action `( (type gmm) (movement back) )))
      )

      ;;;(check-pre-condition reaching)
      (execute-movement reaching)
      (sleep 10)
      ;;;(check-post-condition reaching)
      
      ;;;(check-pre-condition swipping)
      (execute-movement swipping)
      (sleep 10)

      (execute-movement flipping)
      (sleep 10)

      (execute-movement back)
    )
  )
)

;;;  TESTING PERCEPTION PROCESS MODULE
(defun find-oven (param)
  (handler-case
    (find-oven-with-error-handling param)
    (cram-plan-failures:object-not-found (c)
      (print c)
      (setq success 0)
    )
  )
)

(defun find-pancake (param)
  (handler-case
    (find-pancake-with-error-handling param)
    (cram-plan-failures:object-not-found (c)
      (print c)
      (setq success 0)
    )
  )
)

(defun find-oven-with-error-handling (param)
  (cpl-desig-supp:with-designators
    (
      (myoven (object `((desig-props:name "oven_1"))))
    )
    (let ((retry-count 0))
      (with-failure-handling
        ((cram-plan-failures:object-not-found (f)
          (declare (ignore f))
          (when (< (incf retry-count) 2)
            (retry)
          )
        ))
        (setq success 1)
        (cond ( (= param 0) (cram-plan-library:perceive-object 'cram-plan-library:the myoven) )
              ( (= param 1) (let ((oven-perceived
                                    (cram-plan-library:perceive-object 'cram-plan-library:the myoven)
                                 ))
                                 (store-oven oven-perceived)) )
        )
      )
    )
  )
)

(defun find-pancake-with-error-handling (param)
  (cpl-desig-supp:with-designators
    (
      (mypancake (object `((desig-props:name "pancake_1"))))
    )
    (let ((retry-count 0))
      (with-failure-handling
        ((cram-plan-failures:object-not-found (f)
          (declare (ignore f))
          (when (< (incf retry-count) 2)
            (retry)
          )
        ))
        (setq success 1)
        (cond ( (= param 0) (cram-plan-library:perceive-object 'cram-plan-library:the mypancake) )
              ( (= param 1) (let ((pancake-perceived
                                    (cram-plan-library:perceive-object 'cram-plan-library:the mypancake)
                                 ))
                                 (store-pancake pancake-perceived)) )
        )
      )
    )
  )
)

;;;  TESTING PERCEPTION-NAVIGATION PROCESS MODULES
(def-top-level-cram-function get-pancake ()
  (move-arms-away)
  (with-process-modules
    (with-designators
      (
        (mypancake (object `((desig-props:name "pancake_1"))))
      )
      (let (
             (pancake-perceived (cram-plan-library:perceive-object 'cram-plan-library:the mypancake))
           )
           (achieve `(cram-plan-library:object-in-hand ,pancake-perceived))
      )
    )
  )
)

;;;  MORE ELABORATED PLAN - NOT WORKING
;;;(def-top-level-cram-function move-to-pancake ()
;;;  (move-arms-away)
;;;  (with-process-modules
;;;    (cpl-desig-supp:with-designators
;;;      (
;;;        (mypancake (object `((desig-props:name "pancake_1"))))
;;;        (pickup-act-desig (action `( (type trajectory) (to grasp) (obj ,mypancake))))
;;;        (pickup-loc (location `( (to execute) (action , pickup-act-desig))))
;;;      )
;;;      (let (
;;;             (pancake-perceived (cram-plan-library:perceive-object 'cram-plan-library:the mypancake))
;;;           )
;;;	   (at-location (pickup-loc)
;;;	     (roslisp:ros-info (pr2-example) "At location.")
;;;	   )
;;;           (achieve `(cram-plan-library:object-in-hand ,pancake-perceived))
;;;      )
;;;    )
;;;  )
;;;)
