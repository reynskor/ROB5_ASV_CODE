/*
MPU6051.cpp - Class file for the MPU6051 Triple Axis Gyroscope & Accelerometer Arduino Library.

Version: 1.0.3
(c) 2014-2015 Korneliusz Jarzebski
www.jarzebski.pl

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <math.h>

#include <MPU6051.h>

bool MPU6051::begin(MPU6051_dps_t scale, MPU6051_range_t range, int mpua )
{
    // Set Address
    mpuAddress = mpua;

     

    Wire1.begin();
    //Wire1.setSCL(37); 
    //Wire1.setSDA(38);

    // Reset calibrate values
    dg.XAxis = 0;
    dg.YAxis = 0;
    dg.ZAxis = 0;
    useCalibrate = false;

    // Reset threshold values
    tg.XAxis = 0;
    tg.YAxis = 0;
    tg.ZAxis = 0;
    actualThreshold = 0;

    // Check MPU6051 Who Am I Register
    uint8_t address = fastRegister8(MPU6051_REG_WHO_AM_I);
    if ( !( address == 0x68 or address == 0x69 ) )
    {
        Serial.print("ERROR: MPU ADDRESS "); Serial.print(address, HEX); Serial.println(" INCORRECT");
	   return false;
    }

    // Set Clock Source
    setClockSource(MPU6051_CLOCK_PLL_XGYRO);

    // Set Scale & Range
    setScale(scale);
    setRange(range);

    // Disable Sleep Mode
    setSleepEnabled(false);

    return true;
}

void MPU6051::setScale(MPU6051_dps_t scale)
{
    uint8_t value;

    switch (scale)
    {
	case MPU6051_SCALE_250DPS:
	    dpsPerDigit = .007633f;
	    break;
	case MPU6051_SCALE_500DPS:
	    dpsPerDigit = .015267f;
	    break;
	case MPU6051_SCALE_1000DPS:
	    dpsPerDigit = .030487f;
	    break;
	case MPU6051_SCALE_2000DPS:
	    dpsPerDigit = .060975f;
	    break;
	default:
	    break;
    }

    value = readRegister8(MPU6051_REG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (scale << 3);
    writeRegister8(MPU6051_REG_GYRO_CONFIG, value);
}

MPU6051_dps_t MPU6051::getScale(void)
{
    uint8_t value;
    value = readRegister8(MPU6051_REG_GYRO_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (MPU6051_dps_t)value;
}

void MPU6051::setRange(MPU6051_range_t range)
{
    uint8_t value;

    switch (range)
    {
	case MPU6051_RANGE_2G:
	    rangePerDigit = .000061f;
	    break;
	case MPU6051_RANGE_4G:
	    rangePerDigit = .000122f;
	    break;
	case MPU6051_RANGE_8G:
	    rangePerDigit = .000244f;
	    break;
	case MPU6051_RANGE_16G:
	    rangePerDigit = .0004882f;
	    break;
	default:
	    break;
    }

    value = readRegister8(MPU6051_REG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);
    writeRegister8(MPU6051_REG_ACCEL_CONFIG, value);
}

MPU6051_range_t MPU6051::getRange(void)
{
    uint8_t value;
    value = readRegister8(MPU6051_REG_ACCEL_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (MPU6051_range_t)value;
}

void MPU6051::setDHPFMode(MPU6051_dhpf_t dhpf)
{
    uint8_t value;
    value = readRegister8(MPU6051_REG_ACCEL_CONFIG);
    value &= 0b11111000;
    value |= dhpf;
    writeRegister8(MPU6051_REG_ACCEL_CONFIG, value);
}

void MPU6051::setDLPFMode(MPU6051_dlpf_t dlpf)
{
    uint8_t value;
    value = readRegister8(MPU6051_REG_CONFIG);
    value &= 0b11111000;
    value |= dlpf;
    writeRegister8(MPU6051_REG_CONFIG, value);
}

void MPU6051::setClockSource(MPU6051_clockSource_t source)
{
    uint8_t value;
    value = readRegister8(MPU6051_REG_PWR_MGMT_1);
    value &= 0b11111000;
    value |= source;
    writeRegister8(MPU6051_REG_PWR_MGMT_1, value);
}

MPU6051_clockSource_t MPU6051::getClockSource(void)
{
    uint8_t value;
    value = readRegister8(MPU6051_REG_PWR_MGMT_1);
    value &= 0b00000111;
    return (MPU6051_clockSource_t)value;
}

bool MPU6051::getSleepEnabled(void)
{
    return readRegisterBit(MPU6051_REG_PWR_MGMT_1, 6);
}

void MPU6051::setSleepEnabled(bool state)
{
    writeRegisterBit(MPU6051_REG_PWR_MGMT_1, 6, state);
}

bool MPU6051::getIntZeroMotionEnabled(void)
{
    return readRegisterBit(MPU6051_REG_INT_ENABLE, 5);
}

void MPU6051::setIntZeroMotionEnabled(bool state)
{
    writeRegisterBit(MPU6051_REG_INT_ENABLE, 5, state);
}

bool MPU6051::getIntMotionEnabled(void)
{
    return readRegisterBit(MPU6051_REG_INT_ENABLE, 6);
}

void MPU6051::setIntMotionEnabled(bool state)
{
    writeRegisterBit(MPU6051_REG_INT_ENABLE, 6, state);
}

bool MPU6051::getIntFreeFallEnabled(void)
{
    return readRegisterBit(MPU6051_REG_INT_ENABLE, 7);
}

void MPU6051::setIntFreeFallEnabled(bool state)
{
    writeRegisterBit(MPU6051_REG_INT_ENABLE, 7, state);
}

uint8_t MPU6051::getMotionDetectionThreshold(void)
{
    return readRegister8(MPU6051_REG_MOT_THRESHOLD);
}

void MPU6051::setMotionDetectionThreshold(uint8_t threshold)
{
    writeRegister8(MPU6051_REG_MOT_THRESHOLD, threshold);
}

uint8_t MPU6051::getMotionDetectionDuration(void)
{
    return readRegister8(MPU6051_REG_MOT_DURATION);
}

void MPU6051::setMotionDetectionDuration(uint8_t duration)
{
    writeRegister8(MPU6051_REG_MOT_DURATION, duration);
}

uint8_t MPU6051::getZeroMotionDetectionThreshold(void)
{
    return readRegister8(MPU6051_REG_ZMOT_THRESHOLD);
}

void MPU6051::setZeroMotionDetectionThreshold(uint8_t threshold)
{
    writeRegister8(MPU6051_REG_ZMOT_THRESHOLD, threshold);
}

uint8_t MPU6051::getZeroMotionDetectionDuration(void)
{
    return readRegister8(MPU6051_REG_ZMOT_DURATION);
}

void MPU6051::setZeroMotionDetectionDuration(uint8_t duration)
{
    writeRegister8(MPU6051_REG_ZMOT_DURATION, duration);
}

uint8_t MPU6051::getFreeFallDetectionThreshold(void)
{
    return readRegister8(MPU6051_REG_FF_THRESHOLD);
}

void MPU6051::setFreeFallDetectionThreshold(uint8_t threshold)
{
    writeRegister8(MPU6051_REG_FF_THRESHOLD, threshold);
}

uint8_t MPU6051::getFreeFallDetectionDuration(void)
{
    return readRegister8(MPU6051_REG_FF_DURATION);
}

void MPU6051::setFreeFallDetectionDuration(uint8_t duration)
{
    writeRegister8(MPU6051_REG_FF_DURATION, duration);
}

bool MPU6051::getI2CMasterModeEnabled(void)
{
    return readRegisterBit(MPU6051_REG_USER_CTRL, 5);
}

void MPU6051::setI2CMasterModeEnabled(bool state)
{
    writeRegisterBit(MPU6051_REG_USER_CTRL, 5, state);
}

void MPU6051::setI2CBypassEnabled(bool state)
{
    return writeRegisterBit(MPU6051_REG_INT_PIN_CFG, 1, state);
}

bool MPU6051::getI2CBypassEnabled(void)
{
    return readRegisterBit(MPU6051_REG_INT_PIN_CFG, 1);
}

void MPU6051::setAccelPowerOnDelay(MPU6051_onDelay_t delay)
{
    uint8_t value;
    value = readRegister8(MPU6051_REG_MOT_DETECT_CTRL);
    value &= 0b11001111;
    value |= (delay << 4);
    writeRegister8(MPU6051_REG_MOT_DETECT_CTRL, value);
}

MPU6051_onDelay_t MPU6051::getAccelPowerOnDelay(void)
{
    uint8_t value;
    value = readRegister8(MPU6051_REG_MOT_DETECT_CTRL);
    value &= 0b00110000;
    return (MPU6051_onDelay_t)(value >> 4);
}

uint8_t MPU6051::getIntStatus(void)
{
    return readRegister8(MPU6051_REG_INT_STATUS);
}

Activites MPU6051::readActivites(void)
{
    uint8_t data = readRegister8(MPU6051_REG_INT_STATUS);

    a.isOverflow = ((data >> 4) & 1);
    a.isFreeFall = ((data >> 7) & 1);
    a.isInactivity = ((data >> 5) & 1);
    a.isActivity = ((data >> 6) & 1);
    a.isDataReady = ((data >> 0) & 1);

    data = readRegister8(MPU6051_REG_MOT_DETECT_STATUS);

    a.isNegActivityOnX = ((data >> 7) & 1);
    a.isPosActivityOnX = ((data >> 6) & 1);

    a.isNegActivityOnY = ((data >> 5) & 1);
    a.isPosActivityOnY = ((data >> 4) & 1);

    a.isNegActivityOnZ = ((data >> 3) & 1);
    a.isPosActivityOnZ = ((data >> 2) & 1);

    return a;
}

Vector MPU6051::readRawAccel(void)
{
    Wire1.beginTransmission(mpuAddress);
    #if ARDUINO >= 100
	Wire1.write(MPU6051_REG_ACCEL_XOUT_H);
    #else
	Wire1.send(MPU6051_REG_ACCEL_XOUT_H);
    #endif
    Wire1.endTransmission();

    Wire1.beginTransmission(mpuAddress);
    Wire1.requestFrom(mpuAddress, 6);

    while (Wire1.available() < 6);

    #if ARDUINO >= 100
	uint8_t xha = Wire1.read();
	uint8_t xla = Wire1.read();
        uint8_t yha = Wire1.read();
	uint8_t yla = Wire1.read();
	uint8_t zha = Wire1.read();
	uint8_t zla = Wire1.read();
    #else
	uint8_t xha = Wire1.receive();
	uint8_t xla = Wire1.receive();
	uint8_t yha = Wire1.receive();
	uint8_t yla = Wire1.receive();
	uint8_t zha = Wire1.receive();
	uint8_t zla = Wire1.receive();
    #endif

    ra.XAxis = (int16_t)(xha << 8 | xla);
    ra.YAxis = (int16_t)(yha << 8 | yla);
    ra.ZAxis = (int16_t)(zha << 8 | zla);

    return ra;
}

Vector MPU6051::readNormalizeAccel(void)
{
    readRawAccel();

    na.XAxis = ra.XAxis * rangePerDigit * 9.80665f;
    na.YAxis = ra.YAxis * rangePerDigit * 9.80665f;
    na.ZAxis = ra.ZAxis * rangePerDigit * 9.80665f;

    return na;
}

Vector MPU6051::readScaledAccel(void)
{
    readRawAccel();

    na.XAxis = ra.XAxis * rangePerDigit;
    na.YAxis = ra.YAxis * rangePerDigit;
    na.ZAxis = ra.ZAxis * rangePerDigit;

    return na;
}


Vector MPU6051::readRawGyro(void)
{
    Wire1.beginTransmission(mpuAddress);
    #if ARDUINO >= 100
	Wire1.write(MPU6051_REG_GYRO_XOUT_H);
    #else
	Wire1.send(MPU6051_REG_GYRO_XOUT_H);
    #endif
    Wire1.endTransmission();

    Wire1.beginTransmission(mpuAddress);
    Wire1.requestFrom(mpuAddress, 6);

    while (Wire1.available() < 6);

    #if ARDUINO >= 100
	uint8_t xha = Wire1.read();
	uint8_t xla = Wire1.read();
        uint8_t yha = Wire1.read();
	uint8_t yla = Wire1.read();
	uint8_t zha = Wire1.read();
	uint8_t zla = Wire1.read();
    #else
	uint8_t xha = Wire1.receive();
	uint8_t xla = Wire1.receive();
	uint8_t yha = Wire1.receive();
	uint8_t yla = Wire1.receive();
	uint8_t zha = Wire1.receive();
	uint8_t zla = Wire1.receive();
    #endif

    rg.XAxis = (int16_t)(xha << 8 | xla);
    rg.YAxis = (int16_t)(yha << 8 | yla);
    rg.ZAxis = (int16_t)(zha << 8 | zla);

    return rg;
}

Vector MPU6051::readNormalizeGyro(void)
{
    readRawGyro();

    if (useCalibrate)
    {
	ng.XAxis = (rg.XAxis - dg.XAxis) * dpsPerDigit;
	ng.YAxis = (rg.YAxis - dg.YAxis) * dpsPerDigit;
	ng.ZAxis = (rg.ZAxis - dg.ZAxis) * dpsPerDigit;
    } else
    {
	ng.XAxis = rg.XAxis * dpsPerDigit;
	ng.YAxis = rg.YAxis * dpsPerDigit;
	ng.ZAxis = rg.ZAxis * dpsPerDigit;
    }

    if (actualThreshold)
    {
	if (abs(ng.XAxis) < tg.XAxis) ng.XAxis = 0;
	if (abs(ng.YAxis) < tg.YAxis) ng.YAxis = 0;
	if (abs(ng.ZAxis) < tg.ZAxis) ng.ZAxis = 0;
    }

    return ng;
}

float MPU6051::readTemperature(void)
{
    int16_t T;
    T = readRegister16(MPU6051_REG_TEMP_OUT_H);
    return (float)T/340 + 36.53;
}

int16_t MPU6051::getGyroOffsetX(void)
{
    return readRegister16(MPU6051_REG_GYRO_XOFFS_H);
}

int16_t MPU6051::getGyroOffsetY(void)
{
    return readRegister16(MPU6051_REG_GYRO_YOFFS_H);
}

int16_t MPU6051::getGyroOffsetZ(void)
{
    return readRegister16(MPU6051_REG_GYRO_ZOFFS_H);
}

void MPU6051::setGyroOffsetX(int16_t offset)
{
    writeRegister16(MPU6051_REG_GYRO_XOFFS_H, offset);
}

void MPU6051::setGyroOffsetY(int16_t offset)
{
    writeRegister16(MPU6051_REG_GYRO_YOFFS_H, offset);
}

void MPU6051::setGyroOffsetZ(int16_t offset)
{
    writeRegister16(MPU6051_REG_GYRO_ZOFFS_H, offset);
}

int16_t MPU6051::getAccelOffsetX(void)
{
    return readRegister16(MPU6051_REG_ACCEL_XOFFS_H);
}

int16_t MPU6051::getAccelOffsetY(void)
{
    return readRegister16(MPU6051_REG_ACCEL_YOFFS_H);
}

int16_t MPU6051::getAccelOffsetZ(void)
{
    return readRegister16(MPU6051_REG_ACCEL_ZOFFS_H);
}

void MPU6051::setAccelOffsetX(int16_t offset)
{
    writeRegister16(MPU6051_REG_ACCEL_XOFFS_H, offset);
}

void MPU6051::setAccelOffsetY(int16_t offset)
{
    writeRegister16(MPU6051_REG_ACCEL_YOFFS_H, offset);
}

void MPU6051::setAccelOffsetZ(int16_t offset)
{
    writeRegister16(MPU6051_REG_ACCEL_ZOFFS_H, offset);
}

// Calibrate algorithm
void MPU6051::calibrateGyro(uint8_t samples)
{
    // Set calibrate
    useCalibrate = true;

    // Reset values
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    float sigmaX = 0;
    float sigmaY = 0;
    float sigmaZ = 0;

    // Read n-samples
    for (uint8_t i = 0; i < samples; ++i)
    {
	readRawGyro();
	sumX += rg.XAxis;
	sumY += rg.YAxis;
	sumZ += rg.ZAxis;

	sigmaX += rg.XAxis * rg.XAxis;
	sigmaY += rg.YAxis * rg.YAxis;
	sigmaZ += rg.ZAxis * rg.ZAxis;

	delay(5);
    }

    // Calculate delta vectors
    dg.XAxis = sumX / samples;
    dg.YAxis = sumY / samples;
    dg.ZAxis = sumZ / samples;

    // Calculate threshold vectors
    th.XAxis = sqrt((sigmaX / 50) - (dg.XAxis * dg.XAxis));
    th.YAxis = sqrt((sigmaY / 50) - (dg.YAxis * dg.YAxis));
    th.ZAxis = sqrt((sigmaZ / 50) - (dg.ZAxis * dg.ZAxis));

    // If already set threshold, recalculate threshold vectors
    if (actualThreshold > 0)
    {
	setThreshold(actualThreshold);
    }
}

// Get current threshold value
uint8_t MPU6051::getThreshold(void)
{
    return actualThreshold;
}

// Set treshold value
void MPU6051::setThreshold(uint8_t multiple)
{
    if (multiple > 0)
    {
	// If not calibrated, need calibrate
	if (!useCalibrate)
	{
	    calibrateGyro();
	}

	// Calculate threshold vectors
	tg.XAxis = th.XAxis * multiple;
	tg.YAxis = th.YAxis * multiple;
	tg.ZAxis = th.ZAxis * multiple;
    } else
    {
	// No threshold
	tg.XAxis = 0;
	tg.YAxis = 0;
	tg.ZAxis = 0;
    }

    // Remember old threshold value
    actualThreshold = multiple;
}

// Fast read 8-bit from register
uint8_t MPU6051::fastRegister8(uint8_t reg)
{
    uint8_t value;

    Wire1.beginTransmission(mpuAddress);
    #if ARDUINO >= 100
	Wire1.write(reg);
    #else
	Wire1.send(reg);
    #endif
    Wire1.endTransmission();

    Wire1.beginTransmission(mpuAddress);
    Wire1.requestFrom(mpuAddress, 1);
    #if ARDUINO >= 100
	value = Wire1.read();
    #else
	value = Wire1.receive();
    #endif;
    Wire1.endTransmission();

    return value;
}

// Read 8-bit from register
uint8_t MPU6051::readRegister8(uint8_t reg)
{
    uint8_t value;

    Wire1.beginTransmission(mpuAddress);
    #if ARDUINO >= 100
	Wire1.write(reg);
    #else
	Wire1.send(reg);
    #endif
    Wire1.endTransmission();

    Wire1.beginTransmission(mpuAddress);
    Wire1.requestFrom(mpuAddress, 1);
    while(!Wire1.available()) {};
    #if ARDUINO >= 100
	value = Wire1.read();
    #else
	value = Wire1.receive();
    #endif;
    Wire1.endTransmission();

    return value;
}

// Write 8-bit to register
void MPU6051::writeRegister8(uint8_t reg, uint8_t value)
{
    Wire1.beginTransmission(mpuAddress);

    #if ARDUINO >= 100
	Wire1.write(reg);
	Wire1.write(value);
    #else
	Wire1.send(reg);
	Wire1.send(value);
    #endif
    Wire1.endTransmission();
}

int16_t MPU6051::readRegister16(uint8_t reg)
{
    int16_t value;
    Wire1.beginTransmission(mpuAddress);
    #if ARDUINO >= 100
        Wire1.write(reg);
    #else
        Wire1.send(reg);
    #endif
    Wire1.endTransmission();

    Wire1.beginTransmission(mpuAddress);
    Wire1.requestFrom(mpuAddress, 2);
    while(!Wire1.available()) {};
    #if ARDUINO >= 100
        uint8_t vha = Wire1.read();
        uint8_t vla = Wire1.read();
    #else
        uint8_t vha = Wire1.receive();
        uint8_t vla = Wire1.receive();
    #endif;
    Wire1.endTransmission();

    value = vha << 8 | vla;

    return value;
}

void MPU6051::writeRegister16(uint8_t reg, int16_t value)
{
    Wire1.beginTransmission(mpuAddress);

    #if ARDUINO >= 100
	Wire1.write(reg);
	Wire1.write((uint8_t)(value >> 8));
	Wire1.write((uint8_t)value);
    #else
	Wire1.send(reg);
	Wire1.send((uint8_t)(value >> 8));
	Wire1.send((uint8_t)value);
    #endif
    Wire1.endTransmission();
}

// Read register bit
bool MPU6051::readRegisterBit(uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = readRegister8(reg);
    return ((value >> pos) & 1);
}

// Write register bit
void MPU6051::writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = readRegister8(reg);

    if (state)
    {
        value |= (1 << pos);
    } else 
    {
        value &= ~(1 << pos);
    }

    writeRegister8(reg, value);
}
