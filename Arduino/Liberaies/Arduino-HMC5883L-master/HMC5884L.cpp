/*
HMC5884L.cpp - Class file for the HMC5884L Triple Axis Digital Compass Arduino Library.

Version: 1.1.0
(c) 2014 Korneliusz Jarzebski
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

#include "HMC5884L.h"

bool HMC5884L::begin()
{
    Wire1.begin();

    if ((fastRegister8(HMC5884L_REG_IDENT_A) != 0x48)
    || (fastRegister8(HMC5884L_REG_IDENT_B) != 0x34)
    || (fastRegister8(HMC5884L_REG_IDENT_C) != 0x33))
    {
	return false;
    }

    setRange(HMC5884L_RANGE_1_3GA);
    setMeasurementMode(HMC5884L_CONTINOUS);
    setDataRate(HMC5884L_DATARATE_15HZ);
    setSamples(HMC5884L_SAMPLES_1);

    mgPerDigit = 0.92f;

    return true;
}

Vector HMC5884L::readRaw(void)
{
    v.XAxis = readRegister16(HMC5884L_REG_OUT_X_M) - xOffset;
    v.YAxis = readRegister16(HMC5884L_REG_OUT_Y_M) - yOffset;
    v.ZAxis = readRegister16(HMC5884L_REG_OUT_Z_M);

    return v;
}

Vector HMC5884L::readNormalize(void)
{
    v.XAxis = ((float)readRegister16(HMC5884L_REG_OUT_X_M) - xOffset) * mgPerDigit;
    v.YAxis = ((float)readRegister16(HMC5884L_REG_OUT_Y_M) - yOffset) * mgPerDigit;
    v.ZAxis = (float)readRegister16(HMC5884L_REG_OUT_Z_M) * mgPerDigit;

    return v;
}

void HMC5884L::setOffset(int xo, int yo)
{
    xOffset = xo;
    yOffset = yo;
}

void HMC5884L::setRange(hmc5884l_range_t range)
{
    switch(range)
    {
	case HMC5884L_RANGE_0_88GA:
	    mgPerDigit = 0.073f;
	    break;

	case HMC5884L_RANGE_1_3GA:
	    mgPerDigit = 0.92f;
	    break;

	case HMC5884L_RANGE_1_9GA:
	    mgPerDigit = 1.22f;
	    break;

	case HMC5884L_RANGE_2_5GA:
	    mgPerDigit = 1.52f;
	    break;

	case HMC5884L_RANGE_4GA:
	    mgPerDigit = 2.27f;
	    break;

	case HMC5884L_RANGE_4_7GA:
	    mgPerDigit = 2.56f;
	    break;

	case HMC5884L_RANGE_5_6GA:
	    mgPerDigit = 3.03f;
	    break;

	case HMC5884L_RANGE_8_1GA:
	    mgPerDigit = 4.35f;
	    break;

	default:
	    break;
    }

    writeRegister8(HMC5884L_REG_CONFIG_B, range << 5);
}

hmc5884l_range_t HMC5884L::getRange(void)
{
    return (hmc5884l_range_t)((readRegister8(HMC5884L_REG_CONFIG_B) >> 5));
}

void HMC5884L::setMeasurementMode(hmc5884l_mode_t mode)
{
    uint8_t value;

    value = readRegister8(HMC5884L_REG_MODE);
    value &= 0b11111100;
    value |= mode;

    writeRegister8(HMC5884L_REG_MODE, value);
}

hmc5884l_mode_t HMC5884L::getMeasurementMode(void)
{
    uint8_t value;

    value = readRegister8(HMC5884L_REG_MODE);
    value &= 0b00000011;

    return (hmc5884l_mode_t)value;
}

void HMC5884L::setDataRate(hmc5884l_dataRate_t dataRate)
{
    uint8_t value;

    value = readRegister8(HMC5884L_REG_CONFIG_A);
    value &= 0b11100011;
    value |= (dataRate << 2);

    writeRegister8(HMC5884L_REG_CONFIG_A, value);
}

hmc5884l_dataRate_t HMC5884L::getDataRate(void)
{
    uint8_t value;

    value = readRegister8(HMC5884L_REG_CONFIG_A);
    value &= 0b00011100;
    value >>= 2;

    return (hmc5884l_dataRate_t)value;
}

void HMC5884L::setSamples(hmc5884l_samples_t samples)
{
    uint8_t value;

    value = readRegister8(HMC5884L_REG_CONFIG_A);
    value &= 0b10011111;
    value |= (samples << 5);

    writeRegister8(HMC5884L_REG_CONFIG_A, value);
}

hmc5884l_samples_t HMC5884L::getSamples(void)
{
    uint8_t value;

    value = readRegister8(HMC5884L_REG_CONFIG_A);
    value &= 0b01100000;
    value >>= 5;

    return (hmc5884l_samples_t)value;
}

// Write byte to register
void HMC5884L::writeRegister8(uint8_t reg, uint8_t value)
{
    Wire1.beginTransmission(HMC5884L_ADDRESS);
    #if ARDUINO >= 100
        Wire1.write(reg);
        Wire1.write(value);
    #else
        Wire1.send(reg);
        Wire1.send(value);
    #endif
    Wire1.endTransmission();
}

// Read byte to register
uint8_t HMC5884L::fastRegister8(uint8_t reg)
{
    uint8_t value;
    Wire1.beginTransmission(HMC5884L_ADDRESS);
    #if ARDUINO >= 100
        Wire1.write(reg);
    #else
        Wire1.send(reg);
    #endif
    Wire1.endTransmission();

    Wire1.requestFrom(HMC5884L_ADDRESS, 1);
    #if ARDUINO >= 100
        value = Wire1.read();
    #else
        value = Wire1.receive();
    #endif;
    Wire1.endTransmission();

    return value;
}

// Read byte from register
uint8_t HMC5884L::readRegister8(uint8_t reg)
{
    uint8_t value;
    Wire1.beginTransmission(HMC5884L_ADDRESS);
    #if ARDUINO >= 100
        Wire1.write(reg);
    #else
        Wire1.send(reg);
    #endif
    Wire1.endTransmission();

    Wire1.beginTransmission(HMC5884L_ADDRESS);
    Wire1.requestFrom(HMC5884L_ADDRESS, 1);
    while(!Wire1.available()) {};
    #if ARDUINO >= 100
        value = Wire1.read();
    #else
        value = Wire1.receive();
    #endif;
    Wire1.endTransmission();

    return value;
}

// Read word from register
int16_t HMC5884L::readRegister16(uint8_t reg)
{
    int16_t value;
    Wire1.beginTransmission(HMC5884L_ADDRESS);
    #if ARDUINO >= 100
        Wire1.write(reg);
    #else
        Wire1.send(reg);
    #endif
    Wire1.endTransmission();

    Wire1.beginTransmission(HMC5884L_ADDRESS);
    Wire1.requestFrom(HMC5884L_ADDRESS, 2);
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
