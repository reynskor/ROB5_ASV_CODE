/*
   rosserial Subscriber Example
   Blinks an LED on callback
*/

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

#include <std_msgs/Empty.h>
#include <NMEAGPS.h>
#include <math.h>
#include <cmath>
#include <Wire.h>
#include <HMC5883L.h>
#include <HMC5884L.h>
#include <std_msgs/String.h>


int LeftMval = 0, RightMval = 0;


NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values
#define gps_port Serial3
unsigned long t1 = 1000, t2 = 0;

HMC5884L compass;
float x_offset = 260, y_offset = -455, deg = 0.0;
float ang_offset = -25.0;
Servo LeftM, RightM;
ros::NodeHandle  nh;
bool tog_controller = 0;
double current_angle = -30;
long latL = 5, lngL = 5;
void toggle() {
  tog_controller = !tog_controller;
}
void setleftPWM( const std_msgs::Int16& msg) {
  LeftMval = msg.data;
  String buffer;
  buffer.append("Recieved left motor message: ");
  buffer.append(msg.data);
  nh.loginfo(buffer.c_str());
}
void setrightPWM( const std_msgs::Int16& msg) {
  RightMval = msg.data;
  String buffer;
  buffer.append("Recieved right motor message: ");
  buffer.append(msg.data);
  nh.loginfo(buffer.c_str());
}

ros::Subscriber<std_msgs::Int16> subleftPWM("setleftPWM", &setleftPWM );
ros::Subscriber<std_msgs::Int16> subrightPWM("setrightPWM", &setrightPWM );
ros::Subscriber<std_msgs::Empty> subtog("arduino_toggle", &toggle );

std_msgs::String str_msg;
ros::Publisher ardulog("ardg", &str_msg);


void setup()
{
  /*pinMode(25,OUTPUT);
    pinMode(26,OUTPUT);
    pinMode(27,OUTPUT);
    pinMode(24,OUTPUT);*/
  LeftM.attach(9, 1000, 2000); //(pwm pin,puls min, puls max)
  RightM.attach(10, 1000, 2000);
  nh.initNode();
  nh.subscribe(subrightPWM);
  nh.subscribe(subleftPWM);
  nh.subscribe(subtog);
  nh.advertise(ardulog);
  delay(2000);
  gps_port.begin( 9600 );
  gps_port.print( F("$PMTK251,115200*1F\r\n") );
  gps_port.flush();// wait for the command to go out
  delay( 1000 );// wait for the GPS device to change speeds
  gps_port.end();//*/
  gps_port.begin( 115200 );//*/
  gps.send_P( &gps_port, F("PMTK220,1000") );
  delay(100);
  while (!compass.begin())
  {
    nh.loginfo("Could not find a valid HMC5883L sensor, check wiring!");
    nh.spinOnce();
    delay(500);
  }
  compass.setRange(HMC5884L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5884L_CONTINOUS);
  compass.setDataRate(HMC5884L_DATARATE_30HZ);
  compass.setSamples(HMC5884L_SAMPLES_8);
  nh.loginfo("Gyro setup complete... Arming Motors");
  //arm motors
  LeftM.write(90);
  RightM.write(90);
}

void loop()
{
  if (tog_controller) {
    while (gps.available( gps_port )) {
      fix = gps.read();
    }
    //if (fix.valid.location) {
    lngL = fix.longitudeL();
    latL = fix.latitudeL();
    //}
    Vector raw = compass.readRaw();
    current_angle = atan2(raw.YAxis - y_offset, raw.XAxis - x_offset) * 180 / PI - ang_offset;
    if (current_angle < -180.0) {
      current_angle = current_angle - 360;
    }    //*/
    //IMU_data.publish(&current_pose);
    //nh.spinOnce();
    String buffer;
    buffer = "";
    buffer.append(LeftMval);
    buffer.append(",");
    buffer.append(RightMval);
    buffer.append(",");
    buffer.append(current_angle);
    buffer.append(",");
    buffer.append(latL);
    buffer.append(",");
    buffer.append(lngL);
    buffer.append(",");
    buffer.append(fix.valid.location);
    buffer.append(",");
    buffer.append(millis());
    str_msg.data = buffer.c_str();
    ardulog.publish( &str_msg );
    nh.spinOnce();
    LeftM.write(LeftMval);
    RightM.write(RightMval);
    delay(50);
  }
  else {
    LeftM.write(90);
    RightM.write(90);
    nh.spinOnce();
    delay(50);
  }
}
