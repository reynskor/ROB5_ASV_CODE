#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
//#include <math.h>



double l = 0.0;

void numbergetter(const std_msgs::Float32::ConstPtr& length)
{
  l = length->data;
  //ROS_INFO("I send something [%f]",l);
  
}

sensor_msgs::PointCloud getPointCloudData(){
  unsigned int num_points = 100;
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "sensor_frame";

    cloud.points.resize(num_points);

    //we'll also add an intensity channel to the cloud
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(num_points);
    


    cloud.points[1].x = l;
    cloud.points[1].y = 0.0;
    cloud.points[1].z = 0.0;
    
/*
    //generate some fake data for our point cloud
    for(unsigned int i = 0; i < num_points; ++i){
      cloud.points[i].x = 1 + count;
      cloud.points[i].y = 2 + count;
      cloud.points[i].z = 3 + count;
      cloud.channels[0].values[i] = 100 + count;
    }
*/
    return cloud;
}
sensor_msgs::LaserScan getLaserScanData(){
    sensor_msgs::LaserScan laserscan;

    unsigned int num_readings = 1;
	double laser_frequency = 40;
	double ranges[num_readings];
	double intensities[num_readings];

    for (int i=0;i<num_readings;i++){
		ranges[i] = l;
		intensities[i] = 100+l;
	}
	
	ros::Time scantime = ros::Time::now();
	laserscan.header.stamp = scantime;
    laserscan.header.frame_id = "base_IMU";
	laserscan.angle_min = 0;
	laserscan.angle_max = 0;
	laserscan.angle_increment = 0;
	laserscan.time_increment = 0;
	laserscan.range_min = 0.02;
	laserscan.range_max = 4.0;

	laserscan.ranges.resize(num_readings);
	laserscan.intensities.resize(num_readings);
	
	for (int i=0;i<num_readings;i++){
		laserscan.ranges[i] = l;
		laserscan.intensities[i] = 100+l;
	}

    ROS_DEBUG("I send something to laser :)");
    return laserscan;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "point_cloud_publisher");

  ros::NodeHandle n;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 50);
  ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
  ros::Subscriber sub = n.subscribe("ultrasound", 1000,numbergetter);

  ROS_INFO("I send something stream");
  int count = 0;
  ros::Rate r(10.0);
  while(n.ok()){
    
    cloud_pub.publish(getPointCloudData());
    laser_pub.publish(getLaserScanData());
    ++count;
    ros::spinOnce();
    r.sleep();
  }
}

