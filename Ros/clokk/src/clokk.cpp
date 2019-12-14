#include "ros/ros.h"
#include <rosgraph_msgs/Clock.h>


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "clokkpub");

  ros::NodeHandle n;
  ros::Publisher Test_pub = n.advertise<rosgraph_msgs::Clock>("clock", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    rosgraph_msgs::Clock current_time = rosgraph_msgs::Clock();

	current_time.clock = ros::Time::now();
    

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    Test_pub.publish(current_time);
    ROS_INFO("Clokk dun clokked");
    
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
