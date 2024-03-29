/*****************************************************************************/
//    Function:     Cpp file for HMC5883L
//  Hardware:    Grove - 3-Axis Digital Compass
//    Arduino IDE: Arduino-1.0
//    Author:     FrankieChu
//    Date:      Jan 10,2013
//    Version: v1.0
//    by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
/*******************************************************************************/

#include <Arduino.h>
#include "HMC5883L.h"

HMC5883L::HMC5883L()
{
    m_Scale = 1;
}

void HMC5883L::initCompass()
{
    
    delay(5);
    
    int error = setScale(1.3);                              // Set the scale of the compass.
    
    if(error != 0)                                                  // If there is an error, print it out.
    {
        Serial.println(getErrorText(error));
    }
    
    error = setMeasurementMode(MEASUREMENT_CONTINUOUS);     // Set the measurement mode to Continuous
    
    if(error != 0)                                                  // If there is an error, print it out.
    {
        Serial.println(getErrorText(error));
    }
    
#if __Dbg
    //cout << "val_origin = " << val_origin << endl;
    //cout <<"init ok" << endl;
#endif
}


int HMC5883L::getCompass()
{
    MagnetometerRaw raw = readRawAxis();
    // Retrived the scaled values from the compass (scaled to the configured scale).
    MagnetometerScaled scaled = readScaledAxis();

    // Values are accessed like so:
    int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(scaled.YAxis, scaled.XAxis);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -2??37' which is -2.617 Degrees, or (which we need) -0.0456752665 radians, I will use -0.0457
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    float declinationAngle = -0.0457;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if(heading < 0)
    heading += 2*PI;

    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
    heading -= 2*PI;

    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/M_PI;

    // Output the data via the serial port.
    
    int degree = headingDegrees*10;
    
    return degree;
}



MagnetometerRaw HMC5883L::readRawAxis()
{
    uint8_t* buffer = read(DATA_REGISTER_BEGIN, 6);
    MagnetometerRaw raw = MagnetometerRaw();
    raw.XAxis = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw.ZAxis = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw.YAxis = (int16_t)((buffer[4] << 8) | buffer[5]);
    raw.xbuf1 = buffer[0];
    raw.xbuf2 = buffer[1];
    raw.ybuf1 = buffer[2];
    raw.ybuf2 = buffer[3];
    raw.zbuf1 = buffer[4];
    raw.zbuf2 = buffer[5];
    return raw;
}

MagnetometerScaled HMC5883L::readScaledAxis()
{
    MagnetometerRaw raw = readRawAxis();
    MagnetometerScaled scaled = MagnetometerScaled();
    scaled.XAxis = raw.XAxis * m_Scale;
    scaled.ZAxis = raw.ZAxis * m_Scale;
    scaled.YAxis = raw.YAxis * m_Scale;
    return scaled;
}

short HMC5883L::setScale(float gauss)
{
    uint8_t regValue = 0x00;
    if(gauss == 0.88)
    {
        regValue = 0x00;
        m_Scale = 0.73;
    }
    else if(gauss == 1.3)
    {
        regValue = 0x01;
        m_Scale = 0.92;
    }
    else if(gauss == 1.9)
    {
        regValue = 0x02;
        m_Scale = 1.22;
    }
    else if(gauss == 2.5)
    {
        regValue = 0x03;
        m_Scale = 1.52;
    }
    else if(gauss == 4.0)
    {
        regValue = 0x04;
        m_Scale = 2.27;
    }
    else if(gauss == 4.7)
    {
        regValue = 0x05;
        m_Scale = 2.56;
    }
    else if(gauss == 5.6)
    {
        regValue = 0x06;
        m_Scale = 3.03;
    }
    else if(gauss == 8.1)
    {
        regValue = 0x07;
        m_Scale = 4.35;
    }
    else
    return ERRORCODE_1_NUM;

    // Setting is in the top 3 bits of the register.
    regValue = regValue << 5;
    write(CONFIGURATION_REGISTERB, regValue);
}

short HMC5883L::setMeasurementMode(uint8_t mode)
{
    write(MODE_REGISTER, mode);
}

void HMC5883L::write(short address, short data)
{
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t* HMC5883L::read(short address, short length)
{
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(address);
    Wire.endTransmission();

    //Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.requestFrom(HMC5883L_ADDRESS, length);

    uint8_t buffer[length];
    
    if(Wire.available() == length)
    {
        for(uint8_t i = 0; i < length; i++)
        {
            buffer[i] = Wire.read();
        }
    }
    
    //Wire.endTransmission();
    return buffer;
}

char* HMC5883L::getErrorText(short errorCode)
{
    if(ERRORCODE_1_NUM == 1)
    return ERRORCODE_1;

    return "Error not defined.";
}
