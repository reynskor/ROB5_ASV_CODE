#include <NMEAGPS.h>
#include <Wire.h>

#define gps_port Serial3

NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values

unsigned long t1 = 0, t2 = 0;
void setup() {
  Serial.begin(115200);
  gps_port.begin( 9600 );
  gps_port.print( F("$PMTK251,115200*1F\r\n") );
  gps_port.flush();// wait for the command to go out
  delay( 1000 );// wait for the GPS device to change speeds
  gps_port.end();//
  gps_port.begin( 115200 );//
  gps.send_P( &gps_port, F("PMTK220,1000") );//*/
  Serial.println("GPS initialized");
}

void loop() {
    while (gps.available( gps_port )) {
      fix = gps.read();
    }
    //Serial.println(fix.dateTime_ms());
    //Serial.println(fix.valid.location);
    if ((millis() > t1 + 1000)) {
      Serial.print( fix.latitudeL()); Serial.print(",");
      Serial.print( fix.longitudeL()); Serial.print(",");
      Serial.println(millis());
      t1 = millis();
    }//*/
   /* String buffer;
    buffer="";
    buffer.append(fix.latitudeL());
    buffer.append(",");
    buffer.append(fix.longitudeL());
    buffer.append(",");
    buffer.append(fix.valid.location);
    buffer.append(",");
    buffer.append(deg);
    Serial.println(buffer.c_str());
    delay(50);
  /*delay(3000);
    Serial.println("GPS log at 10hz");
    Serial.println("Latitude,Longitude,Timestep");
    int i = 0;
    while (i < (10 * 60 * 10)) {
    while (gps.available( gps_port )) {
      fix = gps.read();
    }
    if ((fix.dateTime_ms() != t1)) {
      Serial.print(fix.latitudeL()); Serial.print(",");
      Serial.print(fix.longitudeL()); Serial.print(",");
      Serial.print(fix.dateTime_ms()); Serial.print(",");
      Serial.println(millis());
      t1 = fix.dateTime_ms();
      i++;
    }//
    }
    i = 0;
    gps.send_P( &gps_port, F("PMTK220,500") );
    delay(100);
    Serial.println("GPS log at 2hz");
    Serial.println("Latitude,Longitude,Timestep");
    while (i < (2 * 60 * 10)) {
    while (gps.available( gps_port )) {
      fix = gps.read();
    }
    if (fix.valid.location && (fix.dateTime_ms() != t1)) {
      Serial.print( fix.latitudeL()); Serial.print(",");
      Serial.print( fix.longitudeL()); Serial.print(",");
      Serial.println(millis());
      t1 = fix.dateTime_ms();
      i++;
    }//
    }//
    gps.send_P( &gps_port, F("PMTK220,1000") );
    delay(100);
    Serial.println("GPS log at 1hz");
    Serial.println("Latitude,Longitude,Timestep");
    i = 0;
    while (i < (1 * 60 * 10)) {
    while (gps.available( gps_port )) {
      fix = gps.read();
    }
    //Serial.println(fix.dateTime_ms());
    //Serial.println(fix.valid.location);
    if (fix.valid.location && (millis() > t1 + 1000)) {
      Serial.print( fix.latitudeL()); Serial.print(",");
      Serial.print( fix.longitudeL()); Serial.print(",");
      Serial.println(millis());
      t1 = millis();
      i++;
    }//
    }
    Serial.println("test done");
    while (1) {
    }
  */
}
