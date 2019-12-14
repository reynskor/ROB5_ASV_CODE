#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <string>

double x = 0.0; //
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

void IMU_pose_data(const geometry_msgs::Vector3 current_pose)
{
  x = current_pose.x;
  y = current_pose.y;
  th = current_pose.z;
    ROS_DEBUG("I am ODOMETRY recieving an ODOM[%f]",x);
  //ROS_INFO("I heard X: %f ", Current_pose.x);
  //ROS_INFO("I heard Y: %s ", Current_pose.y);
  //ROS_INFO("I heard Theta: %s ", Current_pose.z);

}

void IMU_vel_data(const geometry_msgs::Vector3 current_vel)
{
  vx = current_vel.x;
  vy = current_vel.y;
  vth = current_vel.z;
    ROS_DEBUG("I am ODOMETRY recieving an vel ODOM[%f]",vx);
  //ROS_INFO("I heard Xvel: %s ", Current_vel.x);
  //ROS_INFO("I heard Yvel: %s ", Current_vel.y);
  //ROS_INFO("I heard Thetavel: %s ", Current_vel.z.c_str());

}



int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  ROS_INFO("Odometry is running (fuck yea)");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;


  ros::Subscriber sub1 = n.subscribe("IMU_pose_data", 1000, IMU_pose_data);
  ros::Subscriber sub2 = n.subscribe("IMU_vel_data", 1000, IMU_vel_data);


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate r(10.0);
  while(n.ok()){
              // check for incoming messages
    current_time = ros::Time::now();
    /*
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;
    
    x += delta_x;
    y += delta_y;
    th += delta_th;
    */
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);
    ROS_DEBUG("I am ODOMETRY doing an ODOM[%f]",x);
    last_time = current_time;
	ros::spinOnce();
    r.sleep();
  }
}




