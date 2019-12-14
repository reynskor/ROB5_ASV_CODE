#include <tf/transform_broadcaster.h>
 
int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ROS_INFO("tf is running (mega happy)");
  ros::NodeHandle n;
 
  ros::Rate r(100);
 
  tf::TransformBroadcaster broadcaster;
 
  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromYaw(0)), tf::Vector3(0.75, 0.0, 0.0)),
        ros::Time::now(),"base_link", "base_IMU"));
    r.sleep();
  }
}
