#include <Wire.h>
#include <HMC5884L.h>
#include <math.h>
#include <cmath>

HMC5884L compass;
float x_offset = 430, y_offset = -598, deg = 0.0;
double current_angle = 0.0;

void setup() {
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  compass.setRange(HMC5884L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5884L_CONTINOUS);
  compass.setDataRate(HMC5884L_DATARATE_30HZ);
  compass.setSamples(HMC5884L_SAMPLES_8);
  Serial.println("x_val,yval");

}

void loop() {
  Vector raw = compass.readRaw();
  /*Serial.print(raw.XAxis);
  Serial.print(",");
  Serial.println(raw.YAxis);//*/
  /*current_angle = atan2(raw.YAxis - y_offset, raw.XAxis - x_offset) * 180 / PI ;
    if (current_angle < -180.0) {
    current_angle = current_angle - 360;
    }
    Serial.println(current_angle);
    Serial.print(",");*/
  current_angle = atan2(  -raw.YAxis + y_offset,-raw.XAxis + x_offset ) * 180 / PI;
    Serial.println(current_angle);//*/
  delay(50);
}
