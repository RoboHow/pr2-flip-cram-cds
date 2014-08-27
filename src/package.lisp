(in-package :cl-user)

(desig-props:def-desig-package pr2-example
  (:use #:roslisp #:cram-utilities #:designators-ros
        #:cram-roslisp-common #:cram-designators #:location-costmap
        #:cram-plan-knowledge #:cram-plan-library
	#:pr2-manip-pm #:pr2-nav-pm #:pr2-controllers
	#:cl-user #:cl-tf #:cpl-desig-supp #:cpl #:cpl-impl
  )
  (:import-from #:cram-reasoning #:<- #:def-fact-group #:lisp-fun)
  (:import-from #:cram-plan-library #:at-location)
  (:desig-properties
    #:type
    #:movement
    #:to
  )
)
