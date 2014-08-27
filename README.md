pr2-flip-cram-cds
=================

Running the simulation
----------------------
-> Changing PR2 definition
  * The file gripper.urdf.xacro in the folder defs has the addition of the spatula and should replace the 
    original one located under pr2_description/urdf/gripper_v0.

-> Adding CDS controller representation file
  * In the folder pr2_example/owl, the file gmm.owl should be copied in knowrob_seds/owl and added in init.pl.

-> In a Shell
  * Bring up Gazebo simulation, prolog, CDS controller, etc.
   roslaunch pr2_example pr2_example.launch

-> In Emacs Shell
  * Run CRAM executive in roslisp_repl
    roslisp_repl
    ,
    r-l-s
    pr2_example
    pr2-example

    (in-package :pr2-example)
    * Initialize Controllers
    * Basic Command to move PR2 close to objects (Hack, should do it automously i.e. with location designator)
    (ini)

-> After Controllers are initiated etc, In another Shell
    * Load right arm parameters to enable comfortable IK for initial reaching 
    $ roslaunch pr2_example rcart_conf.launch

-> In another Shell
    * Run Cartesian Controller (interface between PR2 Cartesian Controller fed by CDS Controller)
    $ rosrun pr2_example cartesian_controller

-> Back to Emacs Shell
    *Execute pancake plan
    (make-a-pancake)
