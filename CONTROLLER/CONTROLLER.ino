#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <NMEAGPS.h>
#include <Servo.h>
#include <math.h>
#include <cmath>
#include <Wire.h>
//#include <HMC5883L.h>
#include <HMC5884L.h>
#include <MPU6050.h>
#include <std_msgs/String.h>
NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values
#define gps_port Serial3
unsigned long t1 = 0, t2 = 0, t3 = 0;
String buffer;
HMC5884L compass;
MPU6050 mpu;
float x_offset = 430, y_offset = -598, deg = 0.0;

int LeftMval = 90, RightMval = 90;
int Leftval = 90, Rightval = 90;
Servo LeftM, RightM;

double x = 0.0, y = 0.0;
ros::NodeHandle  nh;

double goal_distance = 0.0;
double goal_angle = 0.0;


double distance_error = 0.0;
double distance_error_old = 0.0;
double angle_error = 0.0;
double angle_error_old = 0.0;

double current_angle = 0.0;

int current_state = 0;
bool tog_controller = 0;

double kP_distance = 3.0;
double kI_distance = 0.0;
double kD_distance = 0.0;
double distance_controller = 0.0;
double distance_max = 30.0;

double kP_angle = 1.0;
double kI_angle = 0.0;
double kD_angle = 0.5;
double angle_controller = 0.0;
double angle_max = 60.0;

long latL = 570143620, lngL = 99858680;

float ang_offset = 3.0;

void messageCb( const geometry_msgs::Vector3& msg) {
  distance_error = msg.x;
  if (msg.y > 180) {
    goal_angle = (std::fmod(msg.y , 180.0) - 180 );
    current_state = msg.z;
  } else {
    goal_angle = msg.y;
    current_state = msg.z;
  }
  //Leftval=90;
  //Rightval=90;
}
void toggle() {
  tog_controller = !tog_controller;
}
void mag_set(const std_msgs::Int16& off_set) {
  ang_offset = off_set.data;
}
void setleftPWM( const std_msgs::Int16& msg) {
  Leftval = msg.data;
  String buffer;
  buffer.append("Recieved left motor message: ");
  buffer.append(msg.data);
  nh.loginfo(buffer.c_str());
}
void setrightPWM( const std_msgs::Int16& msg) {
  Rightval = 180 - msg.data;
  String buffer;
  buffer.append("Recieved right motor message: ");
  buffer.append(msg.data);
  nh.loginfo(buffer.c_str());
}

ros::Subscriber<std_msgs::Int16> subleftPWM("setleftPWM", &setleftPWM );
ros::Subscriber<std_msgs::Int16> subrightPWM("setrightPWM", &setrightPWM );
ros::Subscriber<geometry_msgs::Vector3> sub("control_system_goal", &messageCb );
ros::Subscriber<std_msgs::Empty> subtog("arduino_toggle", &toggle );
ros::Subscriber<std_msgs::Int16> submagset("mag_set", &mag_set );

geometry_msgs::Vector3 current_pose = geometry_msgs::Vector3();
std_msgs::Int32 latP, lngP;

ros::Publisher IMU_data("gps_updates", &current_pose);
ros::Publisher lat_data("lat_updates", &latP);
ros::Publisher lng_data("lng_updates", &lngP);

std_msgs::String str_msg;
ros::Publisher ardulog("ardg", &str_msg);



void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(IMU_data);
  nh.advertise(lat_data);
  nh.advertise(lng_data);
  nh.advertise(ardulog);
  nh.subscribe(sub);
  nh.subscribe(subtog);
  nh.subscribe(submagset);
  nh.subscribe(subrightPWM);
  nh.subscribe(subleftPWM);
  /*while (tog_controller) {
    nh.spinOnce();
    }*/
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
  while (!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G, 0x68))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  nh.loginfo("Gyro setup complete... Arming Motors");
  LeftM.attach(9, 1000, 2000); //(pwm pin,puls min, puls max)
  RightM.attach(10, 1000, 2000);
  LeftM.write(90);//arm motors
  RightM.write(90);
  delay(1000);
}

void loop() {
  if (tog_controller) {
    while (gps.available( gps_port )) {
      fix = gps.read();
    }
    Vector gyro = mpu.readNormalizeGyro();
    Vector raw = compass.readRaw();
    //Serial.print(raw.YAxis);Serial.print(",");
    //Serial.println(raw.XAxis);
    current_angle = atan2(  -raw.YAxis + y_offset, -raw.XAxis + x_offset ) * 180 / PI - ang_offset;
    if (current_angle < -180.0) {
      current_angle = current_angle + 360;
    }
    else if (current_angle > 180.0) {
      current_angle = current_angle - 360;
    }
    Serial.println(current_angle);//*/
    latP.data = fix.latitudeL();
    lngP.data = fix.longitudeL();
    //current_pose.z = 0;
    //IMU_data.publish(&current_pose);
    lat_data.publish(&latP);
    lng_data.publish(&lngP);

    /*if(millis()>t2+100){
      Serial.println(fix.dateTime_us());
      t2=millis();
      }//*/
    if (current_state == 1) {
      /// Do some controllery stuff;
      angle_error = goal_angle - current_angle;
      if (angle_error > 180) {
        angle_error = angle_error - 360;
      }
      else if (angle_error < -180) {
        angle_error = angle_error + 360;
      }
      //Serial.println(angle_error);

      angle_controller = -kP_angle * angle_error + kD_angle * gyro.ZAxis; //
      //Serial.print(kP_angle *angle_error);Serial.print(",");
      //Serial.println(kD_angle * -gyro.ZAxis);
      //Serial.print(angle_error);Serial.print(",");
      //Serial.println(angle_controller);
      if (angle_controller > angle_max) {
        angle_controller = angle_max;
      }
      else if (angle_controller < -angle_max) {
        angle_controller = -angle_max;
      }
      //Serial.println(angle_error);
      distance_controller = distance_error * kP_distance;
      if (distance_controller > distance_max) {
        distance_controller = distance_max;
      }
      /*if(distance_error<5){
        distance_controller=0;
        }*/
      if (abs(angle_error) > 90) {
        distance_controller = 0;
      }
      /*if (!fix.valid.location) {
        distance_controller = 0;
        }*/
      LeftMval = 90  + angle_controller - distance_controller;
      RightMval = 90  + angle_controller + distance_controller;
      if (LeftMval > 180) {
        LeftMval = 180;
      }
      else if (LeftMval < 0) {
        LeftMval = 0;
      }
      if (RightMval > 180) {
        RightMval = 180;
      }
      else if (RightMval < 0) {
        RightMval = 0;
      }

      buffer = "";
      buffer.append(goal_angle);
      buffer.append(",");
      buffer.append(current_angle);
      buffer.append(",");
      buffer.append(angle_error);
      buffer.append(",");
      buffer.append(gyro.ZAxis);
      buffer.append(",");
      buffer.append(angle_controller);
      buffer.append(",");
      buffer.append(fix.latitudeL());
      buffer.append(",");
      buffer.append(fix.longitudeL());
      buffer.append(",");
      buffer.append(distance_error);
      buffer.append(",");
      buffer.append(distance_controller);
      buffer.append(",");
      buffer.append(LeftMval);
      buffer.append(",");
      buffer.append(RightMval);
      buffer.append(",");
      buffer.append(millis());
      str_msg.data = buffer.c_str();
      ardulog.publish( &str_msg );
    }
    else if (current_state == 2) {
      /// Do some controllery stuff;
      angle_error = goal_angle - current_angle;
      if (angle_error > 180) {
        angle_error = angle_error - 360;
      }
      else if (angle_error < -180) {
        angle_error = angle_error + 360;
      }
      angle_controller = -kP_angle * angle_error + kD_angle * gyro.ZAxis; // + kD_angle * ((angle_error_old - angle_error) / ((millis() - t2))/1000);
      if (angle_controller > angle_max) {
        angle_controller = angle_max;
      }
      else if (angle_controller < -angle_max) {
        angle_controller = -angle_max;
      }
      LeftMval =  Leftval+angle_controller;
      RightMval = Rightval+angle_controller;
      if (LeftMval > 180) {
        LeftMval = 180;
      }
      else if (LeftMval < 0) {
        LeftMval = 0;
      }
      if (RightMval > 180) {
        RightMval = 180;
      }
      else if (RightMval < 0) {
        RightMval = 0;
      }

      buffer = "";
      buffer.append(goal_angle);
      buffer.append(",");
      buffer.append(current_angle);
      buffer.append(",");
      buffer.append(angle_error);
      buffer.append(",");
      buffer.append(angle_controller);
      buffer.append(",");
      buffer.append(LeftMval);
      buffer.append(",");
      buffer.append(RightMval);
      buffer.append(",");
      buffer.append(millis());
      str_msg.data = buffer.c_str();
      ardulog.publish( &str_msg );

    }
    else if (current_state == 3) {
      angle_error = goal_angle - current_angle;
      if (angle_error > 180) {
        angle_error = angle_error - 360;
      }
      else if (angle_error < -180) {
        angle_error = angle_error + 360;
      }
      //Serial.println(angle_error);

      angle_controller = -kP_angle * angle_error; //
      //Serial.print(kP_angle *angle_error);Serial.print(",");
      //Serial.println(kD_angle * -gyro.ZAxis);
      //Serial.print(angle_error);Serial.print(",");
      //Serial.println(angle_controller);
      if (angle_controller > angle_max) {
        angle_controller = angle_max;
      }
      else if (angle_controller < -angle_max) {
        angle_controller = -angle_max;
      }
      LeftMval = 90  + angle_controller;
      RightMval = 90  + angle_controller;
      buffer = "";
      buffer.append(goal_angle);
      buffer.append(",");
      buffer.append(current_angle);
      buffer.append(",");
      buffer.append(angle_error);
      buffer.append(",");
      buffer.append(angle_controller);
      buffer.append(",");
      buffer.append(millis());
      str_msg.data = buffer.c_str();
      ardulog.publish( &str_msg );
    }
    else if (current_state == 4) {
      angle_error = goal_angle - current_angle;
      if (angle_error > 180) {
        angle_error = angle_error - 360;
      }
      else if (angle_error < -180) {
        angle_error = angle_error + 360;
      }
      //Serial.println(angle_error);

      angle_controller = -kP_angle * angle_error + kD_angle * gyro.ZAxis; //
      //Serial.print(kP_angle *angle_error);Serial.print(",");
      //Serial.println(kD_angle * -gyro.ZAxis);
      //Serial.print(angle_error);Serial.print(",");
      //Serial.println(angle_controller);
      if (angle_controller > angle_max) {
        angle_controller = angle_max;
      }
      else if (angle_controller < -angle_max) {
        angle_controller = -angle_max;
      }
      LeftMval = 90  + angle_controller;
      RightMval = 90  + angle_controller;
      buffer = "";
      buffer.append(goal_angle);
      buffer.append(",");
      buffer.append(current_angle);
      buffer.append(",");
      buffer.append(angle_error);
      buffer.append(",");
      buffer.append(gyro.ZAxis);
      buffer.append(",");
      buffer.append(angle_controller);
      buffer.append(",");
      buffer.append(millis());
      str_msg.data = buffer.c_str();
      ardulog.publish( &str_msg );
    }
    nh.spinOnce();
    LeftM.write(LeftMval);
    RightM.write(RightMval);
    delay(50);
  }
  else {
    while (gps.available( gps_port )) {
      fix = gps.read();
    }
    if (millis() > (t1 + 100)) {
      String buffer;
      buffer = "";
      buffer.append(fix.latitudeL());
      buffer.append(",");
      buffer.append(fix.longitudeL());
      nh.loginfo(buffer.c_str());
      nh.spinOnce();
      Serial.println(buffer);
      t1 = millis();

    }
    LeftM.write(90);
    RightM.write(90);

    delay(5);
  }
}
