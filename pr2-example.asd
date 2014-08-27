(defsystem pr2-example
  :author "Brahayam Ponton"
  :license "BSD"
  :description "Internship at LASA."

  :depends-on (	roslisp
		alexandria
		designators-ros
		cram-roslisp-common
		cram-language
		cram-reasoning
		cram-pr2-knowledge
		cram-plan-knowledge
		cram-environment-representation
		gazebo_msgs-msg
		gazebo_msgs-srv
		pr2-manipulation-knowledge
		pr2-manipulation-process-module
		pr2-reachability-costmap
		pr2-navigation-process-module
		point-head-process-module
		object-location-designators
		physics-utils
		occupancy-grid-costmap
		location-costmap
		semantic-map-costmap
		visibility-costmap
		pr2_controllers_msgs-msg
		robohow_common_msgs-msg
		kinematics_msgs-srv
		trajectory_msgs-msg
		geometry_msgs-msg
		pr2_example-msg
		turtlesim-msg
		actionlib
		cram-pr2-controllers
		cram-plan-library
		cl-robot-models
		cl-tf
		cl-transforms
		gazebo-perception-process-module
		)
  :components
  ((:module "src"
            :components
            ((:file "package")
	     (:file "parameters" :depends-on ("package"))
	     (:file "controllers" :depends-on ("package" "parameters"))
             (:file "tutorial" :depends-on ("package" "parameters" "controllers")) 
             (:file "designators" :depends-on ("package")) 
             (:file "plan" :depends-on ("package" "parameters" "controllers" "tutorial" "designators")) 
))))




