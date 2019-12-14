#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <location_service/Vec3Long.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include <cmath>

double xgoal=0.0;
double ygoal=0.0;

double gps_x = 57.015200;
double gps_y = 9.984781;
float heading = 0.0;
double goal_distance=0.0;
double goal_angle=0.0;
double home_x;
double home_y;
int state=0;
bool resetHome_lat=true;
bool resetHome_lng=true;
void reset_the_home(const std_msgs::Empty::ConstPtr& nothing){
	resetHome_lat=true;
	resetHome_lng=true;
	ROS_INFO("Recieved RESET HOME signal");
}
	
void gps_updater(const geometry_msgs::Vector3::ConstPtr& vel_goals)
{
	ROS_DEBUG("Recieved New GPS Position @ [%f],[%f]",vel_goals->x,vel_goals->y);
	
	gps_x = floor(vel_goals->x)/1000000.0+50.0;
	gps_y = floor(vel_goals->y)/1000000.0;
	
	if(resetHome_lat && gps_x!=0.0){
		resetHome_lat=false;
		ROS_INFO("Set my home @ [%f],[%f]",gps_x,gps_y);
		home_x=vel_goals->x;
		home_y=vel_goals->y;
	}
}
void lat_updater(const std_msgs::Int32& lat)
{
	ROS_DEBUG("Recieved New lat GPS Position @ [%d]",lat.data);
	
	gps_x = (lat.data)/10000000.0;
	
	if(resetHome_lat && gps_x!=0.0){
		resetHome_lat=false;
		ROS_INFO("Set my lat home @ [%f]",gps_x);
		home_x=gps_x;
	}
}
void lng_updater(const std_msgs::Int32& lng)
{
	ROS_DEBUG("Recieved New lng GPS Position @ [%d]",lng.data);
	
	gps_y = (lng.data)/10000000.0;
	
	if(resetHome_lng && gps_y!=0.0){
		resetHome_lng=false;
		ROS_INFO("Set my lng home @ [%f]",gps_y);
		home_y=gps_y;
	}
}
void updateGoals(){
  double distance=0;
  double a=0;
  double c=0;
  double earth_radius = 6371000;
  double gx_rads = xgoal* M_PI / 180;
  double gy_rads = ygoal* M_PI / 180;
  double lx_rads = gps_x* M_PI / 180;
  double ly_rads = gps_y* M_PI / 180;
  
  double diffN = (xgoal-gps_x) * M_PI / 180;
  double diffE = (ygoal-gps_y) * M_PI / 180;
  
  ROS_DEBUG("DIFF N,E [%.10f],[%.10f]",xgoal,gps_x);
  
  
  a=sin(diffN/2)*sin(diffN/2)+cos(lx_rads)*cos(gx_rads)*sin(diffE/2)*sin(diffE/2);
  c=2*atan2(sqrt(a),sqrt(1-a));
  distance = earth_radius*c;
  
  
  double angle=0;
  angle=atan2(sin(diffE)*cos(gx_rads),cos(lx_rads)*sin(gx_rads)-sin(lx_rads)*cos(gx_rads)*cos(diffE));
  double degreesAngle=std::fmod((angle/M_PI*180)+360,360.0);
  
  
  goal_distance=distance;
  goal_angle=degreesAngle;
}


void distanceBearingGoal(const geometry_msgs::Vector3::ConstPtr& vel_goals)
{
  double earth_radius = 6371000;
  double lx_rads = gps_x* M_PI / 180;
  double ly_rads = gps_y* M_PI / 180;
  double thetarads=vel_goals->y*M_PI/180;
  xgoal = asin(sin(lx_rads)*cos(vel_goals->x/earth_radius)+cos(lx_rads)*sin(vel_goals->x/earth_radius)*cos(thetarads));
  ygoal = ly_rads + atan2(sin(thetarads)*sin(vel_goals->x/earth_radius)*cos(lx_rads),cos(vel_goals->x/earth_radius)-sin(lx_rads)*sin(xgoal));
  ROS_INFO("Recieved New Goal @ [%.7f],[%.7f]",vel_goals->x,vel_goals->y);
  ROS_INFO("Comparing With    @ [%.7f],[%.7f]",gps_x,gps_y);
  xgoal = xgoal *180/M_PI;
  ygoal = ygoal *180/M_PI;
  ROS_INFO("x = [%f]",xgoal);
  ROS_INFO("y = [%f]",ygoal);
  updateGoals();
  ROS_INFO("Distance to new goal = [%f]",goal_distance);
  ROS_INFO("Angle to new goal = [%f]",goal_angle);
}

void goals(const geometry_msgs::Vector3::ConstPtr& vel_goals)
{
  xgoal = vel_goals->x;
  ygoal = vel_goals->y;
  ROS_INFO("Recieved New Offset Goal @ distance: [%.7f] angle: [%.7f]",vel_goals->x,vel_goals->y);
  ROS_INFO("Using present lat and long    @ [%.7f],[%.7f]",gps_x,gps_y);
  
  
  
  updateGoals();
  ROS_INFO("Distance to new goal = [%f]",goal_distance);
  ROS_INFO("Angle to new goal = [%f]",goal_angle);
}

void set_state_func(const std_msgs::Int32 vel_goals){
  state=vel_goals.data;
  ROS_INFO("--NEW STATE RECIEVED--: [%d]",state);
  if (state==3){
  	state=1;
  	xgoal = home_x;
  	ygoal = home_y;
  }
}
int main(int argc, char** argv){
  ros::init(argc, argv, "location_service");
  ROS_INFO("Location Service Initialised");
  ros::NodeHandle n;
  ros::Publisher test_pub = n.advertise<geometry_msgs::Vector3>("control_system_goal", 1000);
  ros::Rate r(100);
  ros::Subscriber sub = n.subscribe("boat_goals", 1000, goals);
  ros::Subscriber sub4 = n.subscribe("bearing_goals", 1000, distanceBearingGoal);
  ros::Subscriber sub2 = n.subscribe("gps_updates", 1000, gps_updater);
  ros::Subscriber latS = n.subscribe("lat_updates", 1000, lat_updater);
  ros::Subscriber lngS = n.subscribe("lng_updates", 1000, lng_updater);
  ros::Subscriber sub3 = n.subscribe("start_program", 1000, reset_the_home);
  ros::Subscriber sub5 = n.subscribe("set_state", 1000, set_state_func);
  
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
	geometry_msgs::Vector3 current_goal = geometry_msgs::Vector3();
	updateGoals();
    current_goal.x = goal_distance;
    current_goal.y = goal_angle;
    current_goal.z = state;
    test_pub.publish(current_goal);
    ros::spinOnce();
	ROS_DEBUG("Sent new goal @ [%f],[%f]",current_goal.x,current_goal.y);

    loop_rate.sleep();
  }
  r.sleep();
}
