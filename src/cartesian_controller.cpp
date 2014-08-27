#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

class PR2CartController{
    public:
	PR2CartController(const ros::NodeHandle& node) : node_(node){
	    this->init();
	}

	~PR2CartController(){}

	void run(){
	    tf::StampedTransform transform;
	    try{
		listener.waitForTransform("/torso_lift_link", "/des_EEframe", ros::Time(0), ros::Duration(1.0));
		listener.lookupTransform("/torso_lift_link", "/des_EEframe", ros::Time(0), transform);
	    }
	    catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	    }

	    tf::StampedTransform aux_transform;
	    try{
		listener.waitForTransform("/map", "/des_EEframe", ros::Time(0), ros::Duration(1.0));
		listener.lookupTransform("/map", "/des_EEframe", ros::Time(0), aux_transform);
	    }
	    catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	    }

	    cmd.pose.position.x = transform.getOrigin().x();
	    cmd.pose.position.y = transform.getOrigin().y();
	    cmd.pose.position.z = transform.getOrigin().z();
	    cmd.pose.orientation.x = transform.getRotation().x();
	    cmd.pose.orientation.y = transform.getRotation().y();
	    cmd.pose.orientation.z = transform.getRotation().z();
	    cmd.pose.orientation.w = transform.getRotation().w();

	    if (node_.hasParam("/cds_controller_pr2/run_controller")){
		node_.getParamCached("/cds_controller_pr2/run_controller", run_controller_flag);
		if (run_controller_flag == 1.0){
		    pose_pub_.publish(cmd);
		}
	    }
	}	

    private:

	double run_controller_flag;
	ros::NodeHandle node_;
	tf::TransformListener listener;
	geometry_msgs::PoseStamped cmd;
	ros::Publisher pose_pub_;

	void init(){
	    pose_pub_ = node_.advertise<geometry_msgs::PoseStamped>("r_cart/command_pose", 1);

	    tf::StampedTransform transform;
	    bool pass = false;
	    int times_looping = 0;
	    while (!pass) {
		pass = true;
		times_looping++;
		std::cout << "Times looping " << times_looping << std::endl;
		try{
		    listener.waitForTransform("/torso_lift_link", "/EEframe", ros::Time(0), ros::Duration(5.0));
		    listener.lookupTransform( "/torso_lift_link", "/EEframe", ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
		    pass = false;
		    ROS_ERROR("%s",ex.what());
		}
	    }

	    pass = false;
	    times_looping = 0;
	    while (!pass) {
		pass = true;
		times_looping++;
		std::cout << "Times looping " << times_looping << std::endl;
		try{
		    listener.waitForTransform("/torso_lift_link", "/des_EEframe", ros::Time(0), ros::Duration(5.0));
		    listener.lookupTransform( "/torso_lift_link", "/des_EEframe", ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
		    pass = false;
		    ROS_ERROR("%s",ex.what());
		}
	    }

	    // Load parameters
	    cmd.header.frame_id = "/torso_lift_link";
	    cmd.pose.position.x = transform.getOrigin().x();
	    cmd.pose.position.y = transform.getOrigin().y();
	    cmd.pose.position.z = transform.getOrigin().z();
	    cmd.pose.orientation.x = transform.getRotation().x();
	    cmd.pose.orientation.y = transform.getRotation().y();
	    cmd.pose.orientation.z = transform.getRotation().z();
	    cmd.pose.orientation.w = transform.getRotation().w();
	}
};


int main(int argc, char** argv){
    ros::init(argc, argv, "pr2_cart_controller");
    ros::NodeHandle node;
    PR2CartController my_controller(node);
    ros::Rate rate(100.0);
    while (node.ok()){
	my_controller.run();
	rate.sleep();
    }
    return(0);
}
