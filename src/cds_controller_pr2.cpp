#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <seds_control_nodes/cds_cartesian_wrapper.hpp>
#include <tf/transform_listener.h>

class CdsCartesianTestNode
{
  public:
    CdsCartesianTestNode(const ros::NodeHandle& nh) : nh_(nh)
    {
      this->init();
    }

    ~CdsCartesianTestNode() {}

    void run()
    {
      ros::Rate rate(1.0/dt_);

      while(ros::ok())
      {
	nh_.getParamCached("/cds_controller_pr2/update_params", update_flag);
	if (update_flag == 1.0){
	    this->init();
	    ROS_INFO("Parameters have changed: [%f]", update_flag);
	    nh_.setParam("/cds_controller_pr2/update_params", 0.0);
	    nh_.setParam("/cds_controller_pr2/run_controller", 1.0);
	}
	simulateOneCycle();
	ros::spinOnce();
	rate.sleep();
      }
    }
     
  private:
    double run_controller_flag;
    double update_flag;
    ros::NodeHandle nh_;
    tf::TransformBroadcaster br;
    CdsCartesianWrapper cds_;
    double dt_;
    std::string child_frame_, parent_frame_;
    tf::Transform state_;
    tf::TransformListener listener;

    void init()
    {
      nh_.param<double>("dt", dt_, 0.001);  // Read from parameter server, otherwise default 0.001

      KDL::Frame start_pose = readStartPose();
      tf::transformKDLToTF(start_pose, state_);

      cds_.init(readCDSParams(), start_pose);
    } 

    void simulateOneCycle()
    {
	tf::StampedTransform transform;
        try{
	  listener.waitForTransform("map","EEframe", ros::Time(0), ros::Duration(1.0));
          listener.lookupTransform( "map","EEframe", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
	// This is correct, but does not work.
	// state_ = transform;

	KDL::Frame state_copy;
	tf::transformTFToKDL(state_, state_copy);

	nh_.getParamCached("/cds_controller_pr2/run_controller", run_controller_flag);
	if (run_controller_flag == 1.0){  // Controller running -> Update next desired position
	    tf::transformKDLToTF(cds_.update(state_copy), state_);
	} else {
	    state_ = transform;
	}

	broadcastTF();
    }

    void broadcastTF()
    {
      br.sendTransform(tf::StampedTransform(state_, ros::Time::now(),
          "map", child_frame_));
    }

    CDSExecutionParams readCDSParams()
    {
      CDSExecutionParams cds_params;
      unsigned int num_states, num_vars;

      std::string filename = "";

      nh_.getParam("master_gmm_file", filename);
      GMMStates* master_gmm = readGMMStatesFromFile(filename.c_str(),
          num_states, num_vars);
      cds_params.master_dyn_ = new GMRDynamics(master_gmm, num_states, num_vars);

      nh_.getParam("slave_gmm_file", filename);
     
      GMMStates* slave_gmm = readGMMStatesFromFile(filename.c_str(),
          num_states, num_vars);
      cds_params.slave_dyn_ =  new GMRDynamics(slave_gmm, num_states, num_vars);

      nh_.getParam("coupling_gmm_file", filename);
      GMMStates* coupling_gmm = readGMMStatesFromFile(filename.c_str(),
          num_states, num_vars);
      cds_params.coupling_ = new GMR(coupling_gmm, num_states, num_vars);

      nh_.param<double>("alpha", cds_params.alpha_, 10);
      nh_.param<double>("beta", cds_params.beta_, 1); 
      nh_.param<double>("lambda", cds_params.lambda_, 1);
      nh_.param<double>("reaching_threshold", cds_params.reachingThreshold_, 0.001);
      nh_.param<double>("dt", cds_params.dt_, 0.01);

      nh_.getParam("parent_frame", parent_frame_);
      nh_.getParam("child_frame", child_frame_);

      cds_params.object_frame_ = readFrameFromParameterServer("object_frame");  // Could also come from oven-tf
      cds_params.attractor_frame_ = readFrameFromParameterServer("attractor_frame");

      return cds_params;
    }

    KDL::Frame readStartPose()
    {
        tf::StampedTransform transform;
        KDL::Frame transform_copy;
	bool pass = false;

	while (!pass) {
	    pass = true;
            try{
	        listener.waitForTransform("map", "EEframe", ros::Time(0), ros::Duration(5.0));
                listener.lookupTransform( "map", "EEframe", ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
	        std::cout << "Error" << std::endl;
	        pass = false;
                ROS_ERROR("%s",ex.what());
            }
	}

	tf::transformTFToKDL(transform, transform_copy);
	return transform_copy;
    }

    KDL::Frame readFrameFromParameterServer(const std::string& param_name)
    {
      double x, y, z, q_x, q_y, q_z, q_w;

      nh_.param<double>(param_name + "/origin/x", x, 0.0);
      nh_.param<double>(param_name + "/origin/y", y, 0.0);
      nh_.param<double>(param_name + "/origin/z", z, 0.0);
      nh_.param<double>(param_name + "/orientation/x", q_x, 0.0);
      nh_.param<double>(param_name + "/orientation/y", q_y, 0.0);
      nh_.param<double>(param_name + "/orientation/z", q_z, 0.0);
      nh_.param<double>(param_name + "/orientation/w", q_w, 1.0);

      return KDL::Frame(KDL::Rotation::Quaternion(q_x, q_y, q_z, q_w),
          KDL::Vector(x, y, z));
    }

    GMMStates* readGMMStatesFromFile(const char* file, unsigned int& num_states, unsigned int& num_vars)
    {
      FILE *fid;
      int res;
      double dtemp;
      fid = fopen(file, "r");
      if(!fid)
      {
              cout<<"Error opening file \""<<file<<"\"\n";
              throw;
      }
      res=fscanf(fid, "%lf\n", &dtemp);
      num_states = (int)dtemp;
      res=fscanf(fid, "%lf\n", &(dtemp));
      num_vars = (int)dtemp;
      GMMStates* GMMState  = (GMMStates  *)malloc(num_states*sizeof(GMMStates ) );
      for( unsigned int s=0; s<num_states; s++ ){
              GMMState[s].Mu       = svector(num_vars);
              GMMState[s].Sigma    = smatrix(num_vars, num_vars );
      }
      // Read priors
      for( unsigned int s=0; s<num_states; s++ )
              res=fscanf(fid, "%lf", &(GMMState[s].Prio ) );

      // Read Mu
      for( unsigned int i=0; i<num_vars; i++ )
              for( unsigned int s=0; s<num_states; s++ )
                      res=fscanf(fid, "%lf", &(GMMState[s].Mu[i]) );
      // Read Sigmas
      for( unsigned int s=0; s<num_states; s++ )
              for( unsigned int i=0; i<num_vars; i++ )
                      for( unsigned int j=0; j<num_vars; j++ )
                              res=fscanf(fid, "%lf", &(GMMState[s].Sigma[i][j]));
      fclose(fid);

      return GMMState;
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cds_control_node");
  ros::NodeHandle nh("~");
  CdsCartesianTestNode my_test(nh);
  my_test.run();
  return 0;
}
