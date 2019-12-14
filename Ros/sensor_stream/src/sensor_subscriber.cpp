#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>

void chatterCallback(const sensor_msgs::PointCloud numb)
{
  ROS_INFO("I heard: [%f]", numb.points[1].x);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("cloud", 1000, chatterCallback);
  ROS_INFO("I send something sub");
  ros::spin();

  return 0;
}

