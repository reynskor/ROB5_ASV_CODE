#include "ros/ros.h"
#include <std_msgs/Float32.h>


//float iss = 0.0;



int main(int argc, char **argv)
{
  std_msgs::Float32 piss;
  piss.data = 0.6;

  ros::init(argc, argv, "talker"); 
  

  ros::NodeHandle n;

  ros::Publisher testing_pub = n.advertise<std_msgs::Float32>("ultrasound", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    ROS_INFO("I send something pub");
    testing_pub.publish(piss);
    loop_rate.sleep();
  }


  return 0;
}
