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
#include <cmath>      // round
#include <math.h>     // NAN
#include <map>

#define LSM6DSOX_ADDRESS            0x6A

#define LSM6DSOX_WHO_AM_I_REG       0X0F
#define LSM6DSOX_CTRL1_XL           0X10
#define LSM6DSOX_CTRL2_G            0X11
#define LSM6DSOX_CTRL3_C            0X12
#define LSM6DSOX_CTRL4_C            0X13
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
#define MASK_FR_XL                  0x0C  // CTRL1_XL
#define MASK_LPF2_XL_EN             0x02  // CTRL1_XL
#define MASK_ODR_G                  0xF0  // CTRL2_G
#define MASK_FR_G                   0x0E  // CTRL2_G
#define MASK_BDU                    0x40  // CTRL3_C
#define MASK_LPF1_SEL_G             0x02  // CTRL4_C
#define MASK_XL_ULP_EN              0x80  // CTRL5_C
#define MASK_XL_HM_MODE             0x10  // CTRL6_C
#define MASK_FTYPE                  0x07  // CTRL6_C
#define MASK_G_HM_MODE              0x80  // CTRL7_G
#define MASK_HPCF_XL                0xE0  // CTRL8_XL

// Output Data Rate configuration bits (excluding 1.6Hz XL LP) 
const vectorOfFloatsAndBits ODR_freq_bits = {
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

// XL Full Range configuration bits
const vectorOfFloatsAndBits FR_XL_bits = { // FS1_XL FS0_XL (CTRL1_XL)
  {  2, 0b00 }, 
  {  4, 0b10 }, 
  {  8, 0b11 }, 
  { 16, 0b01 }
};

// G Full Range configuration bits
const vectorOfFloatsAndBits FR_G_bits = { // FS1_G FS0_G FS_125 (CTRL2_G)
  {  125, 0b001 }, 
  {  250, 0b000 }, 
  {  500, 0b010 }, 
  { 1000, 0b100 }, 
  { 2000, 0b110 }
};

// XL LPF2 ODR divisor configuration bits
const vectorOfFloatsAndBits LPF2_XL_bits = { 
  // bit 0: LPF2_XL_EN (CTRL1_XL)
  // bits 1-3 HPCF_XL_[2:0] (CTRL8_XL)
  {   2, 0b0000 }, // LPF2 disabled -> ODR/2 (LPF1)
  {   4, 0b0001 },
  {  10, 0b0011 }, 
  {  20, 0b0101 }, 
  {  45, 0b0111 }, 
  { 100, 0b1001 },
  { 200, 0b1011 }, 
  { 400, 0b1101 },
  { 800, 0b1111 }
};

// Map of G ODR bits to sorted vector of cutoff frequency and 
// configuration bit pairs:
// bit 0: LPF1_SEL_G (CTRL4_C)
// bits 1..3: FTYPE_2 FTYPE_1 FTYPE_0 (CTRL6_C)
//
// Note that below wherever 'LPF1 disabled' is stated, the cutoff
// frequency is set to an arbitrary number slightly below ODR/2,
// in order for this value to be selected when presented with
// a cutoff of ODR/2. This will lead to G LPF1 being disabled. 
const std::map<uint8_t, vectorOfFloatsAndBits> LPF1_G_bits = {
  { 0b0001,  // ODR G = 12.5 Hz
    {
      {    3.9, 0b1111 },
      {    4.1, 0b1101 },
      {    4.2, 0b0001 },
      {    6.2, 0b0000 } // LPF1 disabled, see above
    }
  },
  { 0b0010,  // ODR G = 26 Hz
    {
      {    6.7, 0b1111 },
      {    7.8, 0b1101 },
      {    8.3, 0b0001 },
      {   12.0, 0b0000 } // LPF1 disabled, see above
    }
  },
  { 0b0011,  // ODR G = 52 Hz
    {
      {    9.7, 0b1111 },
      {   13.4, 0b1101 },
      {   16.6, 0b0001 },
      {   16.7, 0b1001 },
      {   16.8, 0b1011 },
      {   25.0, 0b0000 } // LPF1 disabled, see above
    }
  },
  { 0b0100,  // ODR G = 104 Hz
    {
      {   11.5, 0b1111 },
      {   19.0, 0b1101 },
      {   31.0, 0b1011 },
      {   33.0, 0b0001 },
      {   51.0, 0b0000 } // LPF1 disabled, see above
    }
  },
  { 0b0101,  // ODR G = 208 Hz
    {
      {   12.2, 0b1111 },
      {   23.1, 0b1101 },
      {   43.2, 0b1011 },
      {   62.4, 0b1001 },
      {   67.0, 0b0001 },
      {  103.0, 0b0000 } // LPF1 disabled, see above
    }
  },
  { 0b0110,  // ODR G = 416 Hz
    {
      {   12.4, 0b1111 },
      {   24.6, 0b1101 },
      {   48.0, 0b1011 },
      {   86.7, 0b1001 },
      {  120.3, 0b0101 },
      {  130.5, 0b0011 },
      {  136.6, 0b0001 },
      {  137.1, 0b0111 },
      {  207.0, 0b0000 } // LPF1 disabled, see above
    }
  },
  { 0b0111,  // ODR G = 833 Hz
    {
      {   12.5, 0b1111 },
      {   25.0, 0b1101 },
      {   49.4, 0b1011 },
      {   96.6, 0b1001 },
      {  154.2, 0b0101 },
      {  192.4, 0b0011 },
      {  239.2, 0b0001 },
      {  281.8, 0b0111 },
      {  416.0, 0b0000 } // LPF1 disabled, see above
    }
  },
  { 0b1000, // ODR G = 1667 Hz
    {
      {   12.5, 0b1111 },
      {   25.1, 0b1101 },
      {   49.8, 0b1011 },
      {   99.6, 0b1001 },
      {  166.6, 0b0101 },
      {  220.7, 0b0011 },
      {  304.2, 0b0001 },
      {  453.2, 0b0111 },
      {  833.0, 0b0000 } // LPF1 disabled, see above
    }
  },
  { 0b1001, // ODR G = 3333 Hz
    {
      {  170.1, 0b0101 },
      {  229.6, 0b0011 },
      {  328.5, 0b0001 },
      {  559.2, 0b0111 },
      { 1666.0, 0b0000 } // LPF1 disabled, see above
    }
  },
  { 0b1010, // ODR G = 6667 Hz
    {
      {  171.1, 0b0101 },
      {  232.0, 0b0011 },
      {  335.5, 0b0001 },
      {  609.0, 0b0111 },
      { 3333.0, 0b0000 } // LPF1 disabled, see above
    }
  }
};


LSM6DSOXClass::LSM6DSOXClass(TwoWire& wire, uint8_t slaveAddress) :
  _wire(&wire),
  _spi(NULL),
  _slaveAddress(slaveAddress)
{
  initializeSettings();

  // Full range not yet defined
  fullRange_XL = NAN;
  fullRange_G = NAN;
}

LSM6DSOXClass::LSM6DSOXClass(SPIClass& spi, int csPin, int irqPin) :
  _wire(NULL),
  _spi(&spi),
  _csPin(csPin),
  _irqPin(irqPin),
  _spiSettings(10E6, MSBFIRST, SPI_MODE0)
{
  initializeSettings();

  // Full range not yet defined
  fullRange_XL = NAN;
  fullRange_G = NAN;
}

LSM6DSOXClass::~LSM6DSOXClass()
{
}

void LSM6DSOXClass::initializeSettings(
      float ODR_XL, float ODR_G, 
      float fullRange_XL, float fullRange_G,
      float cutoff_LPF2_XL, float cutoff_LPF1_G,
      PowerModeXL powerMode_XL,
      PowerModeG powerMode_G,
      int i2c_frequency,
      bool BDU)
{
  settings.ODR_XL = ODR_XL;
  settings.ODR_G = ODR_G;
  settings.fullRange_XL = fullRange_XL;
  settings.fullRange_G = fullRange_G;
  settings.cutoff_LPF2_XL = cutoff_LPF2_XL;
  settings.cutoff_LPF1_G = cutoff_LPF1_G;
  settings.powerMode_XL = powerMode_XL;
  settings.powerMode_G = powerMode_G;
  settings.i2c_frequency = i2c_frequency;
  settings.BDU = BDU;
}

int LSM6DSOXClass::begin()
{
  int result = 1;

  if (_spi != NULL) {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _spi->begin();
  } else {
    _wire->begin();
    _wire->setClock(settings.i2c_frequency);
  }
  // See datasheet: WHO AM I is fixed at 0x6C
  if (readRegister(LSM6DSOX_WHO_AM_I_REG) != 0x6C) {
    end();
    return 0;
  }

  // Power modes. First set G power mode, since it should be set
  // to power down prior to setting XL ULP in mode (it doesn't matter
  // for other combinations).
  setPowerModeG(settings.powerMode_G);
  setPowerModeXL(settings.powerMode_XL);

  // Output Data Rates
  setODR_XL(settings.ODR_XL);
  setODR_G(settings.ODR_G);

  // Full ranges
  setFullRange_XL(settings.fullRange_XL);
  setFullRange_G(settings.fullRange_G);

  // XL LPF2
  setLPF2_XL(settings.cutoff_LPF2_XL);

  // G LPF1
  setLPF1_G(settings.cutoff_LPF1_G);

  // BDU
  uint8_t BDU_bit = settings.BDU ? MASK_BDU : 0x00;
  result = readModifyWriteRegister(LSM6DSOX_CTRL3_C, BDU_bit, MASK_BDU);
  
  return result;
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
      result = readModifyWriteRegister(LSM6DSOX_CTRL5_C, MASK_XL_ULP_EN, MASK_XL_ULP_EN);
      if(result < 1) { return result; }
      break;

    case XL_POWER_MODE_LP_NORMAL:
      // Set XL_ULP_EN to 0
      result = readModifyWriteRegister(LSM6DSOX_CTRL5_C, 0x00, MASK_XL_ULP_EN);
      if(result < 1) { return result; }
      // Set XL_HM_MODE to 1
      result = readModifyWriteRegister(LSM6DSOX_CTRL6_C, MASK_XL_HM_MODE, MASK_XL_HM_MODE);
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
      result = readModifyWriteRegister(LSM6DSOX_CTRL7_G, MASK_G_HM_MODE, MASK_G_HM_MODE);
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

int LSM6DSOXClass::setFullRange_XL(float range) 
{
  // Find Full Range value that is at least as large as the specified range,
  // so expected values will always fit. Then select corresponding configuration
  // bits.
  uint8_t fr_bits = largerOrEqualFloatToBits(range, FR_XL_bits);
  int result = readModifyWriteRegister(LSM6DSOX_CTRL1_XL, fr_bits << 2, MASK_FR_XL);
  if(result > 0) {
    // Store selected full range
    fullRange_XL = getFloatFromBits(fr_bits, FR_XL_bits);
  }
  return result;
}

int LSM6DSOXClass::setFullRange_G(float range) 
{
  // Find Full Range value that is at least as large as the specified range,
  // so expected values will always fit. Then select corresponding configuration
  // bits.
  uint8_t fr_bits = largerOrEqualFloatToBits(range, FR_G_bits);
  int result = readModifyWriteRegister(LSM6DSOX_CTRL2_G, fr_bits << 1, MASK_FR_G);
  if(result > 0) {
    // Store selected full range
    fullRange_G = getFloatFromBits(fr_bits, FR_G_bits);
  }
  return result;
}

int LSM6DSOXClass::setLPF2_XL(float cutoff) 
{
  // Cutoff is converted to an ODR divisor. Note that cutoff should be > 0!
  float odr_divisor = std::round(settings.ODR_XL / cutoff);
  // Find the smallest divisor that is larger than or equal to the divisor
  // found above, and extract the corresponding configuration bits.
  uint8_t lpf2_bits = largerOrEqualFloatToBits(odr_divisor, LPF2_XL_bits);
  // Bit 0 of the bit pattern corresponds to LPF2_XL_EN (CTRL1_XL)
  uint8_t lpf2_xl_en = lpf2_bits & 0x01;
  int result = readModifyWriteRegister(LSM6DSOX_CTRL1_XL, lpf2_xl_en << 1, MASK_LPF2_XL_EN);
  if(result > 0) {
    // Bits 1..3 correspond to HPCF_XL (CTRL8_XL)
    uint8_t hpcf_xl = lpf2_bits >> 1;
    result = readModifyWriteRegister(LSM6DSOX_CTRL8_XL, hpcf_xl << 5, MASK_HPCF_XL);
  }
  return result;
}

int LSM6DSOXClass::setLPF1_G(float cutoff) 
{
  // First, look up ODR bits from G ODR settings
  uint8_t odr_bits = nearestODRbits(settings.ODR_G);
  // Find vector of cutoff frequencies and configuration bits corresponding to G ODR
  auto freq_bits_vector = LPF1_G_bits.at(odr_bits);
  // Now find the cutoff that is <= specified cutoff
  uint8_t lpf1_bits = smallerOrEqualFloatToBits(cutoff, freq_bits_vector);
  // Bit 0 of the bit pattern corresponds to LPF1_SEL_G (CTRL4_C)
  uint8_t lpf1_sel_g = lpf1_bits & 0x01;
  int result = readModifyWriteRegister(LSM6DSOX_CTRL4_C, lpf1_sel_g << 1, MASK_LPF1_SEL_G);
  if(result > 0) {
    // Bits 1..3 of the bit pattern correspond to FTYPE_[0-2] (CTRL6_C)
    uint8_t ftype = lpf1_bits >> 1;
    result = readModifyWriteRegister(LSM6DSOX_CTRL6_C, ftype, MASK_FTYPE);
  }
  return result;
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

  x = data[0] * fullRange_XL / 32768.0;
  y = data[1] * fullRange_XL / 32768.0;
  z = data[2] * fullRange_XL / 32768.0;

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
    return getFloatFromBits(odr_bits, ODR_freq_bits);
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

  x = data[0] * fullRange_G / 32768.0;
  y = data[1] * fullRange_G / 32768.0;
  z = data[2] * fullRange_G / 32768.0;

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
    return getFloatFromBits(odr_bits, ODR_freq_bits);
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

// This is the generic frequency to bits conversion,
// it does not include the XL LP 1.6Hz exception
// Find configuration bits for nearest odr value in table.
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

// Find lowest full range in table >= specified value
uint8_t LSM6DSOXClass::largerOrEqualFloatToBits(float value, const vectorOfFloatsAndBits& v) {
  auto value_it = std::lower_bound(
    v.begin(), v.end()-1, value,
    [](const std::pair<float, uint8_t>& p, float val) {return p.first < val;});
  return value_it->second;
}

// Find lowest full range in table <= specified value
uint8_t LSM6DSOXClass::smallerOrEqualFloatToBits(float value, const vectorOfFloatsAndBits& v) {
  auto value_it = std::lower_bound(
    v.begin(), v.end(), value,
    [](const std::pair<float, uint8_t>& p, float val) {return p.first < val;});
  if((value_it != v.begin()) && (value_it->first > value)) {
    value_it = value_it - 1;
  }
  return value_it->second;
}

// Look up range from configuration bits
float LSM6DSOXClass::getFloatFromBits(uint8_t bits, const vectorOfFloatsAndBits& v) {
  auto match = std::find_if(
    v.begin(), v.end(),
    [bits](const std::pair<float, uint8_t>& it){ return it.second == bits; });
  if(match != v.end()) {
    return match->first;
  }
  return NAN;
}

#ifdef LSM6DS_DEFAULT_SPI
LSM6DSOXClass IMU_LSM6DSOX(LSM6DS_DEFAULT_SPI, PIN_SPI_SS1, LSM6DS_INT);
#else
LSM6DSOXClass IMU_LSM6DSOX(Wire, LSM6DSOX_ADDRESS);
#endif