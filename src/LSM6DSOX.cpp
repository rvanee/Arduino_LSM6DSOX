/*
  This file is part of the Arduino_LSM6DSOX library.
  Copyright (c) 2021 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "LSM6DSOX.h"

#include <algorithm>  // lower_bound
#include <math.h>     // NAN

#define LSM6DSOX_ADDRESS            0x6A

#define LSM6DSOX_WHO_AM_I_REG       0X0F
#define LSM6DSOX_CTRL1_XL           0X10
#define LSM6DSOX_CTRL2_G            0X11

#define LSM6DSOX_CTRL5_C            0X14
#define LSM6DSOX_CTRL6_C            0X15
#define LSM6DSOX_CTRL7_G            0X16
#define LSM6DSOX_CTRL8_XL           0X17

#define LSM6DSOX_STATUS_REG         0X1E

#define LSM6DSOX_OUT_TEMP_L         0X20
#define LSM6DSOX_OUT_TEMP_H         0X21

#define LSM6DSOX_OUTX_L_G           0X22
#define LSM6DSOX_OUTX_H_G           0X23
#define LSM6DSOX_OUTY_L_G           0X24
#define LSM6DSOX_OUTY_H_G           0X25
#define LSM6DSOX_OUTZ_L_G           0X26
#define LSM6DSOX_OUTZ_H_G           0X27

#define LSM6DSOX_OUTX_L_XL          0X28
#define LSM6DSOX_OUTX_H_XL          0X29
#define LSM6DSOX_OUTY_L_XL          0X2A
#define LSM6DSOX_OUTY_H_XL          0X2B
#define LSM6DSOX_OUTZ_L_XL          0X2C
#define LSM6DSOX_OUTZ_H_XL          0X2D


// Bit masks
#define MASK_ODR_XL                 0xF0  // CTRL1_XL
#define MASK_ODR_G                  0xF0  // CTRL2_G
#define MASK_XL_ULP_EN              0x80  // CTRL5_C
#define MASK_XL_HM_MODE             0x10  // CTRL6_C
#define MASK_G_HM_MODE              0x10  // CTRL7_G


LSM6DSOXClass::LSM6DSOXClass(TwoWire& wire, uint8_t slaveAddress) :
  _wire(&wire),
  _spi(NULL),
  _slaveAddress(slaveAddress)
{
}

LSM6DSOXClass::LSM6DSOXClass(SPIClass& spi, int csPin, int irqPin) :
  _wire(NULL),
  _spi(&spi),
  _csPin(csPin),
  _irqPin(irqPin),
  _spiSettings(10E6, MSBFIRST, SPI_MODE0)
{
}

LSM6DSOXClass::~LSM6DSOXClass()
{
}

int LSM6DSOXClass::begin()
{
  if (_spi != NULL) {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _spi->begin();
  } else {
    _wire->begin();
  }
  // See datasheet: WHO AM I is fixed at 0x6C
  if (readRegister(LSM6DSOX_WHO_AM_I_REG) != 0x6C) {
    end();
    return 0;
  }

  //set the gyroscope control register to work at 104 Hz, 2000 dps and in bypass mode
  writeRegister(LSM6DSOX_CTRL2_G, 0x4C);

  // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass mode and enable ODR/4
  // low pass filter (check figure9 of LSM6DSOX's datasheet)
  writeRegister(LSM6DSOX_CTRL1_XL, 0x4A);

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  writeRegister(LSM6DSOX_CTRL7_G, 0x00);

  // Set the ODR config register to ODR/4
  writeRegister(LSM6DSOX_CTRL8_XL, 0x09);

  return 1;
}

void LSM6DSOXClass::end()
{
  if (_spi != NULL) {
    _spi->end();
    digitalWrite(_csPin, LOW);
    pinMode(_csPin, INPUT);
  } else {
    writeRegister(LSM6DSOX_CTRL2_G, 0x00);
    writeRegister(LSM6DSOX_CTRL1_XL, 0x00);
    _wire->end();
  }
}

int LSM6DSOXClass::setPowerModeXL(PowerModeXL power_XL)
{
  int result = 0;
  uint8_t odr_G;
  switch(power_XL) {
    case XL_POWER_DOWN:
      result = readModifyWriteRegister(LSM6DSOX_CTRL1_XL, 0x00, MASK_ODR_XL);
      if(result < 1) { return result; }
      break;

    case XL_POWER_MODE_ULP:
      // Check if gyroscope in power-down mode (see datasheet par 6.2.1)
      result = readRegister(LSM6DSOX_CTRL2_G);
      if(result < 1) { return result; }
      odr_G = result & MASK_ODR_G;
      if(odr_G != 0b0000) { return -2; }
      // First power down (see datasheet par 6.2.1)
      result = readModifyWriteRegister(LSM6DSOX_CTRL1_XL, 0x00, MASK_ODR_XL);
      if(result < 1) { return result; }
      // Set XL_HM_MODE to 0 (see datasheet par 6.2.1)
      result = readModifyWriteRegister(LSM6DSOX_CTRL6_C, 0x00, MASK_XL_HM_MODE);
      if(result < 1) { return result; }
      // Now set XL_ULP_EN to 1
      result = readModifyWriteRegister(LSM6DSOX_CTRL5_C, 0x80, MASK_XL_ULP_EN);
      if(result < 1) { return result; }
      break;

    case XL_POWER_MODE_LP_NORMAL:
      // Set XL_ULP_EN to 0
      result = readModifyWriteRegister(LSM6DSOX_CTRL5_C, 0x00, MASK_XL_ULP_EN);
      if(result < 1) { return result; }
      // Set XL_HM_MODE to 1
      result = readModifyWriteRegister(LSM6DSOX_CTRL6_C, 0x10, MASK_XL_HM_MODE);
      if(result < 1) { return result; }
      break;

    case XL_POWER_MODE_HP:
      // Set XL_ULP_EN to 0
      result = readModifyWriteRegister(LSM6DSOX_CTRL5_C, 0x00, MASK_XL_ULP_EN);
      if(result < 1) { return result; }
      // Set XL_HM_MODE to 0
      result = readModifyWriteRegister(LSM6DSOX_CTRL6_C, 0x00, MASK_XL_HM_MODE);
      if(result < 1) { return result; }

    default:
      // Do nothing
      break;
  }
  return result;
}

int LSM6DSOXClass::setPowerModeG(PowerModeG power_G)
{
  int result = 0;

  switch(power_G) {
    case G_POWER_DOWN:
      result = readModifyWriteRegister(LSM6DSOX_CTRL2_G, 0x00, MASK_ODR_G);
      if(result < 1) { return result; }
      break;

    case G_POWER_MODE_LP_NORMAL:
      // Set G_HM_MODE to 1
      result = readModifyWriteRegister(LSM6DSOX_CTRL7_G, 0x10, MASK_G_HM_MODE);
      if(result < 1) { return result; }
      break;

    case G_POWER_MODE_HP:
      // Set G_HM_MODE to 0
      result = readModifyWriteRegister(LSM6DSOX_CTRL7_G, 0x00, MASK_G_HM_MODE);
      if(result < 1) { return result; }
      break;

    default:
      // Do nothing
      break;
  }
  return result;
}

int LSM6DSOXClass::setODR_XL(float odr) 
{ 
  // Look up ordinary ODR bit values
  uint8_t odr_bits = nearestODRbits(odr);
  // Handle special case of 1.6Hz, which is unique to XL. It needs to be selected
  // if 0 < odr < (halfway 1.6 and 12.5)
  // NOTE that in High Performance mode 12.5Hz will be automatically selected
  // in this case
  if((odr > 0.0) && (odr < (12.5 - 1.6)/2)) {
      odr_bits = 0b1011;
  } 
  return readModifyWriteRegister(LSM6DSOX_CTRL1_XL, odr_bits << 4, MASK_ODR_XL);
}

int LSM6DSOXClass::setODR_G(float odr) 
{
  uint8_t odr_bits = nearestODRbits(odr);
  return readModifyWriteRegister(LSM6DSOX_CTRL2_G, odr_bits << 4, MASK_ODR_G);
}

int LSM6DSOXClass::readAcceleration(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DSOX_OUTX_L_XL, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 4.0 / 32768.0;
  y = data[1] * 4.0 / 32768.0;
  z = data[2] * 4.0 / 32768.0;

  return 1;
}

int LSM6DSOXClass::accelerationAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x01) {
    return 1;
  }

  return 0;
}

float LSM6DSOXClass::accelerationSampleRate()
{
  int result = readRegister(LSM6DSOX_CTRL1_XL);
  if(result >= 0) {
    uint8_t odr_bits = ((uint8_t)result) >> 4;
    // Handle special case of 1.6/12.5Hz
    if(odr_bits == 0b1011) {
      // 1.6Hz in Low Power mode, 12.5Hz otherwise
      int ctrl6_c = readRegister(LSM6DSOX_CTRL6_C);
      if(ctrl6_c < 1) { return NAN; }
      return (ctrl6_c & MASK_XL_HM_MODE) ? 1.6 : 12.5;
    }
    return getODRFromBits(odr_bits);
  }
  return NAN;
}

int LSM6DSOXClass::readGyroscope(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DSOX_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 2000.0 / 32768.0;
  y = data[1] * 2000.0 / 32768.0;
  z = data[2] * 2000.0 / 32768.0;

  return 1;
}

int LSM6DSOXClass::gyroscopeAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}

int LSM6DSOXClass::readTemperature(int& temperature_deg)
{
  float temperature_float = 0;
  readTemperatureFloat(temperature_float);

  temperature_deg = static_cast<int>(temperature_float);

  return 1;
}

int LSM6DSOXClass::readTemperatureFloat(float& temperature_deg)
{
  /* Read the raw temperature from the sensor. */
  int16_t temperature_raw = 0;

  if (readRegisters(LSM6DSOX_OUT_TEMP_L, reinterpret_cast<uint8_t*>(&temperature_raw), sizeof(temperature_raw)) != 1) {
    return 0;
  }

  /* Convert to °C. */
  static int const TEMPERATURE_LSB_per_DEG = 256;
  static int const TEMPERATURE_OFFSET_DEG = 25;

  temperature_deg = (static_cast<float>(temperature_raw) / TEMPERATURE_LSB_per_DEG) + TEMPERATURE_OFFSET_DEG;

  return 1;
}

int LSM6DSOXClass::temperatureAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x04) {
    return 1;
  }

  return 0;
}

float LSM6DSOXClass::gyroscopeSampleRate()
{
  int result = readRegister(LSM6DSOX_CTRL2_G);
  if(result >= 0) {
    uint8_t odr_bits = ((uint8_t)result) >> 4;
    return getODRFromBits(odr_bits);
  }
  return NAN;
}

int LSM6DSOXClass::readRegister(uint8_t address)
{
  uint8_t value;
  
  if (readRegisters(address, &value, sizeof(value)) != 1) {
    return -1;
  }
  return value;
}

int LSM6DSOXClass::readRegisters(uint8_t address, uint8_t* data, size_t length)
{
  if (_spi != NULL) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(0x80 | address);
    _spi->transfer(data, length);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
  } else {
    _wire->beginTransmission(_slaveAddress);
    _wire->write(address);
    // Do not send a stop message: see data sheet par 5.1.2
    if (_wire->endTransmission(false) != 0) {
      return -1;
    }

    if (_wire->requestFrom(_slaveAddress, length) != length) {
      return 0;
    }

    for (size_t i = 0; i < length; i++) {
      *data++ = _wire->read();
    }
  }
  return 1;
}

int LSM6DSOXClass::writeRegister(uint8_t address, uint8_t value)
{
  if (_spi != NULL) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(address);
    _spi->transfer(value);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
  } else {
    _wire->beginTransmission(_slaveAddress);
    _wire->write(address);
    _wire->write(value);
    if (_wire->endTransmission() != 0) {
      return 0;
    }
  }
  return 1;
}

int LSM6DSOXClass::readModifyWriteRegister(uint8_t address, uint8_t value, uint8_t mask)
{
  int result = readRegister(address);
  if(result >= 0) {
    uint8_t current_value = result;
    return writeRegister(address, (current_value & ~mask) | value);
  }
  return result;
}

/* 
  ODR frequency <-> configuration bits conversion
*/

const std::vector<std::pair<float, uint8_t>> ODR_freq_bits = {
  { 0,    0b0000 }, // means power-down
  { 12.5, 0b0001 },
  { 26,   0b0010 },
  { 52,   0b0011 },
  { 104,  0b0100 },
  { 208,  0b0101 },
  { 416,  0b0110 },
  { 833,  0b0111 },
  { 1667, 0b1000 },
  { 3333, 0b1001 },
  { 6667, 0b1010 }
};

// Note that this is the generic frequency to bits conversion,
// it does not include the XL LP 1.6Hz exception
uint8_t LSM6DSOXClass::nearestODRbits(float odr) {
  auto upper_it = std::lower_bound(
    ODR_freq_bits.begin(),
    ODR_freq_bits.end()-1,
    odr,
    [](const std::pair<float, uint8_t>& p, float value) {return p.first < value;});
  auto lower_it = upper_it->first <= odr ? upper_it : upper_it-1;
  uint8_t odr_bits = abs(odr - lower_it->first) <
                     abs(odr - upper_it->first) ?
                      lower_it->second : upper_it->second;
  return odr_bits;
}

float LSM6DSOXClass::getODRFromBits(uint8_t odr_bits) {
  auto match = std::find_if(
    ODR_freq_bits.begin(),
    ODR_freq_bits.end(),
    [odr_bits](const std::pair<float, uint8_t>& it){ return it.second == odr_bits; });
  if(match != ODR_freq_bits.end()) {
    return match->first;
  }
  return NAN;
}

#ifdef LSM6DS_DEFAULT_SPI
LSM6DSOXClass IMU_LSM6DSOX(LSM6DS_DEFAULT_SPI, PIN_SPI_SS1, LSM6DS_INT);
#else
LSM6DSOXClass IMU_LSM6DSOX(Wire, LSM6DSOX_ADDRESS);
#endif