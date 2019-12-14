#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

float x = 0.0;
float y = 0.0;
float th = 0.0;


void goals(const geometry_msgs::Twist::ConstPtr& vel_goals)
{
  x = vel_goals->linear.x;
  y = vel_goals->linear.y;
  th = vel_goals->angular.z;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "arduino_goal");
  ROS_INFO("move_base is running (heck yea!)");
  ros::NodeHandle n;
  ros::Rate r(100);
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, goals);

  geometry_msgs::Vector3 goal_vel = geometry_msgs::Vector3();

   goal_vel.x = x;
   goal_vel.y = y;
   goal_vel.z = th;

ros::Publisher arduino_pub = n.advertise<geometry_msgs::Vector3>("arduino_goal", 1000);


arduino_pub.publish(goal_vel);

  ros::spin();

  r.sleep();
}
