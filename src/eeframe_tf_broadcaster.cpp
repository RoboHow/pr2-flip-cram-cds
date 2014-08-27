#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(100.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    //transform.setRotation( tf::Quaternion(0.0, 0.0, 0.0, 1.0) );
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "r_gripper_tool_frame2", "EEframe"));
    rate.sleep();
  }
  return 0;
};
