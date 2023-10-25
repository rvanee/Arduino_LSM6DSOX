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
#include "LSM6DSOXTables.h"

#define _USE_MATH_DEFINES
#include <cmath>  // round
#include <math.h> // NAN and M_PI

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
#define LSM6DSOX_CTRL10_C           0X19

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

#define LSM6DSOX_TIMESTAMP0         0x40
#define LSM6DSOX_TIMESTAMP1         0x41
#define LSM6DSOX_TIMESTAMP2         0x42
#define LSM6DSOX_TIMESTAMP3         0x43

#define LSM6DSOX_INTERNAL_FREQ_FINE 0x63

// WHO_AM_I code for LSM6DSOX (see datasheet 9.14)
#define LSM6DSOX_WHO_AM_I_RESPONSE  0x6C

// Bit masks
#define MASK_ODR_XL                 0xF0  // CTRL1_XL
#define MASK_FR_XL                  0x0C  // CTRL1_XL
#define MASK_LPF2_XL_EN             0x02  // CTRL1_XL
#define MASK_ODR_G                  0xF0  // CTRL2_G
#define MASK_FR_G                   0x0E  // CTRL2_G
#define MASK_BDU                    0x40  // CTRL3_C
#define MASK_SW_RESET               0x01  // CTRL3_C
#define MASK_LPF1_SEL_G             0x02  // CTRL4_C
#define MASK_XL_ULP_EN              0x80  // CTRL5_C
#define MASK_XL_HM_MODE             0x10  // CTRL6_C
#define MASK_FTYPE                  0x07  // CTRL6_C
#define MASK_HPM_G                  0x30  // CTRL7_G
#define MASK_HP_EN_G                0x40  // CTRL7_G
#define MASK_G_HM_MODE              0x80  // CTRL7_G
#define MASK_HPCF_XL                0xE0  // CTRL8_XL
#define MASK_TIMESTAMP_EN           0x20  // CTRL10_C

#define INTERNALFREQFACTOR_SHIFT    11
#define INTERNALFREQFACTOR_DEFAULT  ((1000000UL << INTERNALFREQFACTOR_SHIFT) / 40000)


LSM6DSOXClass::LSM6DSOXClass(TwoWire& wire, uint8_t slaveAddress) :
  fifo(this),
  _wire(&wire),
  _spi(NULL),
  _slaveAddress(slaveAddress),
  internalFrequencyFactor(INTERNALFREQFACTOR_DEFAULT)
{
  initializeSettings();

  // Full range not yet defined
  fullRange_XL = 0;
  fullRange_G = 0;

  // Register contents not yet defined
  ctrl1_xl = -1;
  ctrl2_g = -1;
}

LSM6DSOXClass::LSM6DSOXClass(SPIClass& spi, int csPin, int irqPin) :
  fifo(this),
  _wire(NULL),
  _spi(&spi),
  _csPin(csPin),
  _irqPin(irqPin),
  _spiSettings(10E6, MSBFIRST, SPI_MODE0),
  internalFrequencyFactor(INTERNALFREQFACTOR_DEFAULT)
{
  initializeSettings();

  // Full range not yet defined
  fullRange_XL = 0;
  fullRange_G = 0;

  // Register contents not yet defined
  ctrl1_xl = -1;
  ctrl2_g = -1;
}

LSM6DSOXClass::~LSM6DSOXClass()
{
}

void LSM6DSOXClass::initializeSettings(
      float ODR_XL, float ODR_G, 
      float fullRange_XL, float fullRange_G,
      bool rad_G,
      float cutoff_LPF2_XL, float cutoff_LPF1_G,
      float cutoff_HPF_G,
      PowerModeXL powerMode_XL,
      PowerModeG powerMode_G,
      int i2c_frequency,
      bool enable_timestamp,
      bool BDU)
{
  settings.ODR_XL = ODR_XL;
  settings.ODR_G = ODR_G;
  settings.fullRange_XL = fullRange_XL;
  settings.fullRange_G = fullRange_G;
  settings.rad_G = rad_G;
  settings.cutoff_LPF2_XL = cutoff_LPF2_XL;
  settings.cutoff_LPF1_G = cutoff_LPF1_G;
  settings.cutoff_HPF_G = cutoff_HPF_G;
  settings.powerMode_XL = powerMode_XL;
  settings.powerMode_G = powerMode_G;
  settings.i2c_frequency = i2c_frequency;
  settings.enable_timestamp = enable_timestamp;
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
  // See datasheet: LSM6DSOX WHO AM I is fixed at 0x6C
  if (readRegister(LSM6DSOX_WHO_AM_I_REG) != LSM6DSOX_WHO_AM_I_RESPONSE) {
    end();
    return 0;
  }

  // Power modes. First set G power mode, since it should be set
  // to power down prior to setting XL to ULP mode (sequence is
  // irrelevant for other power mode combinations).
  setPowerModeG(settings.powerMode_G);
  setPowerModeXL(settings.powerMode_XL);

  // Output Data Rates
  setODR_XL(settings.ODR_XL);
  setODR_G(settings.ODR_G);

  // Full ranges
  setFullRange_XL(settings.fullRange_XL);
  setFullRange_G(settings.fullRange_G);

  // XL LPF2
  setLPF2_XL(settings.cutoff_LPF2_XL, settings.ODR_XL);

  // G LPF1 and HPF
  setLPF1_G(settings.cutoff_LPF1_G, settings.ODR_G);
  setHPF_G(settings.cutoff_HPF_G);

  // Timestamp enable
  uint8_t timestamp_en = settings.enable_timestamp ? MASK_TIMESTAMP_EN : 0x00;
  readModifyWriteRegister(LSM6DSOX_CTRL10_C, timestamp_en, MASK_TIMESTAMP_EN);

  // BDU
  uint8_t BDU_bit = settings.BDU ? MASK_BDU : 0x00;
  readModifyWriteRegister(LSM6DSOX_CTRL3_C, BDU_bit, MASK_BDU);

  // Used to speed up XL/G full scale autorange
  result = readRegister(LSM6DSOX_CTRL1_XL);
  if(result >= 0) {
    ctrl1_xl = result;
  }
  result = readRegister(LSM6DSOX_CTRL2_G);
  if(result >= 0) {
    ctrl2_g = result;
  }

  // Find internal frequency correction factor
  result = readRegister(LSM6DSOX_INTERNAL_FREQ_FINE);
  if(result >= 0) {
    // 8-bit 2's complement
    int8_t freq_fine = (result & 0x80) ? result - 256 : result;
    // See AN5272, par 6.4: divisor = 40000 + 60*freq_fine
    // To find timestamp in microseconds, multiply it by
    // 1000000 / divisor, or 50000 / (divisor/20).
    // Note that range divisor/20 = [2000-3*128, 2000+3*127].
    uint16_t divisor = 2000 + 3*freq_fine;
    // Fixed point factor. Note that 50000*2^11 / divisor above will
    // yield a range of [42953, 63366], fitting a uint16_t.
    // Multiply the timestamp (in units of 25us) with this factor and
    // divide it by 2^11, and the corrected timestamp in us is found.
    internalFrequencyFactor = (50000UL << INTERNALFREQFACTOR_SHIFT) / divisor;
    result = 1;
  }

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
      if(result < 0) { return result; }
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
  uint8_t odr_bits = LSM6DSOXTables::nearestFloatToBits(odr, LSM6DSOXTables::ODR_freq_bits);
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
  uint8_t odr_bits = LSM6DSOXTables::nearestFloatToBits(odr, LSM6DSOXTables::ODR_freq_bits);
  return readModifyWriteRegister(LSM6DSOX_CTRL2_G, odr_bits << 4, MASK_ODR_G);
}

int LSM6DSOXClass::setFullRange_XL(float range) 
{
  // Find Full Range value that is at least as large as the specified range,
  // so expected values will always fit. Then select corresponding configuration
  // bits.
  uint8_t fr_bits = LSM6DSOXTables::largerOrEqualFloatToBits(range, LSM6DSOXTables::FR_XL_bits);
  return setFullRange_XL_bits(fr_bits);
}

int LSM6DSOXClass::setRelativeFullRange_XL(uint16_t range, int8_t delta_range) 
{
  if((delta_range >= -4) && (delta_range <= 4)) {
    for(auto &range_to_bits: LSM6DSOXTables::FR_XL_bits_down_and_up) 
    {
      if(range == range_to_bits.first) {
        // range_to_bits contains 9 entries for each range:
        // -4/-3/-2/-1, current range (0), +1/+2/+3/+4
        uint8_t fr_bits = range_to_bits.second[delta_range + 4];
        // Only set new scale if the bit pattern differs from the current one
        if(fr_bits != range_to_bits.second[4]) {
          int result = setFullRange_XL_bits(fr_bits);
Serial.print("XL ");
Serial.print(range);
Serial.print(" -> ");
Serial.println(fullRange_XL);
          return result;
        } else {
          return 0; // Nothing set
        }
      }
    }
  }
  return -1; // Wrong parameter (range or delta_range)
}

int LSM6DSOXClass::setFullRange_XL_bits(uint8_t fr_bits) 
{
  int result;
  if(ctrl1_xl >= 0) { // Speed up full scale selection by skipping read
    result = writeRegister(LSM6DSOX_CTRL1_XL, (ctrl1_xl & ~MASK_FR_XL) | fr_bits << 2);
  } else {
    result = readModifyWriteRegister(LSM6DSOX_CTRL1_XL, fr_bits << 2, MASK_FR_XL);
  }
  if(result > 0) {
    // Store selected full range
    fullRange_XL = (uint16_t)(LSM6DSOXTables::getFloatFromBits(fr_bits, LSM6DSOXTables::FR_XL_bits));
  }
  return result;
}

int LSM6DSOXClass::setFullRange_G(float range) 
{
  // Find Full Range value that is at least as large as the specified range,
  // so expected values will always fit. Then select corresponding configuration
  // bits.
  uint8_t fr_bits = LSM6DSOXTables::largerOrEqualFloatToBits(range, LSM6DSOXTables::FR_G_bits);
  return setFullRange_G_bits(fr_bits);
}

int LSM6DSOXClass::setRelativeFullRange_G(uint16_t range, int8_t delta_range) 
{
  if((delta_range >= -4) && (delta_range <= 4)) {
    for(auto &range_to_bits: LSM6DSOXTables::FR_G_bits_down_and_up) 
    {
      if(range == range_to_bits.first) {
        // range_to_bits contains 9 entries for each range:
        // -4/-3/-2/-1, current range (0), +1/+2/+3/+4
        uint8_t fr_bits = range_to_bits.second[delta_range + 4];
        // Only set new scale if the bit pattern differs from the current one
        if(fr_bits != range_to_bits.second[4]) {
          int result = setFullRange_G_bits(fr_bits);
Serial.print("G ");
Serial.print(range);
Serial.print(" -> ");
Serial.println(fullRange_G);
          if(result > )
          return result;
        } else {
          return 0; // Nothing set
        }
      }
    }
  }
  return -1; // Wrong parameter (range or delta_range)
}

int LSM6DSOXClass::setFullRange_G_bits(uint8_t fr_bits) 
{
  int result;
  if(ctrl2_g >= 0) { // Speed up full scale selection by skipping read
    result = writeRegister(LSM6DSOX_CTRL2_G, (ctrl2_g & ~MASK_FR_G) | fr_bits << 1);
  } else {
    result = readModifyWriteRegister(LSM6DSOX_CTRL2_G, fr_bits << 1, MASK_FR_G);
  }
  if(result > 0) {
    // Store selected full range
    fullRange_G = (uint16_t)(LSM6DSOXTables::getFloatFromBits(fr_bits, LSM6DSOXTables::FR_G_bits));
  }
  return result;
}

int LSM6DSOXClass::setLPF2_XL(float cutoff, float odr) 
{
  // Cutoff is converted to an ODR divisor. Note that cutoff should be > 0!
  float odr_divisor = std::round(odr / cutoff);
  // Find the smallest divisor that is larger than or equal to the divisor
  // found above, and extract the corresponding configuration bits.
  uint8_t lpf2_bits = LSM6DSOXTables::largerOrEqualFloatToBits(odr_divisor, LSM6DSOXTables::LPF2_XL_bits);
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

int LSM6DSOXClass::setLPF1_G(float cutoff, float odr) 
{
  // First, look up ODR bits from G ODR settings
  uint8_t odr_bits = LSM6DSOXTables::nearestFloatToBits(odr, LSM6DSOXTables::ODR_freq_bits);
  // Find vector of cutoff frequencies and configuration bits corresponding to G ODR
  auto freq_bits_vector = LSM6DSOXTables::LPF1_G_bits.at(odr_bits);
  // Now find the cutoff that is <= specified cutoff
  uint8_t lpf1_bits = LSM6DSOXTables::smallerOrEqualFloatToBits(cutoff, freq_bits_vector);
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

int LSM6DSOXClass::setHPF_G(float cutoff) 
{
  // Find closest cutoff frequency and corresponding configuration bits
  uint8_t hpf_bits = LSM6DSOXTables::nearestFloatToBits(cutoff, LSM6DSOXTables::HPF_G_bits);
  // Bit 0 of the bit pattern corresponds to HP_EN_G (CTRL7_G)
  uint8_t hp_en_g = hpf_bits & 0x01;
  int result = readModifyWriteRegister(LSM6DSOX_CTRL7_G, hp_en_g << 6, MASK_HP_EN_G);
  if(result > 0) {
    // Bits 1..2 of the bit pattern correspond to MASK_HPM_G_[0-1] (CTRL7_G)
    uint8_t hpm_g = hpf_bits >> 1;
    result = readModifyWriteRegister(LSM6DSOX_CTRL7_G, hpm_g << 4, MASK_HPM_G);
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

  float range_factor = fullRange_XL * (1.0 / 32768);
  x = data[0] * range_factor;
  y = data[1] * range_factor;
  z = data[2] * range_factor;

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
      // 1.6Hz in Ultra Low Power mode
      int ctrl5_c = readRegister(LSM6DSOX_CTRL5_C);
      if(ctrl5_c < 0) { return NAN; }
      if((ctrl5_c & MASK_XL_ULP_EN) == MASK_XL_ULP_EN) { // ULP
        return 1.6;
      }
      // 12.5Hz in High Performance mode, 1.6Hz otherwise
      int ctrl6_c = readRegister(LSM6DSOX_CTRL6_C);
      if(ctrl6_c < 0) { return NAN; }
      return (ctrl6_c & MASK_XL_HM_MODE) ? 1.6 : 12.5;
    }
    return LSM6DSOXTables::getFloatFromBits(odr_bits, LSM6DSOXTables::ODR_freq_bits);
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

  float range_factor = fullRange_G * (1.0 / 32768);
  if(settings.rad_G) {
    range_factor *= M_PI/180.0;
  }
  x = data[0] * range_factor;
  y = data[1] * range_factor;
  z = data[2] * range_factor;

  return 1;
}

int LSM6DSOXClass::gyroscopeAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}

float LSM6DSOXClass::gyroscopeSampleRate()
{
  int result = readRegister(LSM6DSOX_CTRL2_G);
  if(result >= 0) {
    uint8_t odr_bits = ((uint8_t)result) >> 4;
    return LSM6DSOXTables::getFloatFromBits(odr_bits, LSM6DSOXTables::ODR_freq_bits);
  }
  return NAN;
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
  temperature_deg = temperatureIntToCelsius(temperature_raw);

  return 1;
}

float LSM6DSOXClass::temperatureIntToCelsius(int temperature_raw)
{
  /* Convert to °C. */
  static int const TEMPERATURE_LSB_per_DEG = 256;
  static int const TEMPERATURE_OFFSET_DEG = 25;

  float temperature_deg = (static_cast<float>(temperature_raw) / TEMPERATURE_LSB_per_DEG) + TEMPERATURE_OFFSET_DEG;

  return temperature_deg;
}

int LSM6DSOXClass::temperatureAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x04) {
    return 1;
  }

  return 0;
}

int LSM6DSOXClass::readTimestamp(uint32_t& timestamp) {
  uint8_t buffer[4];
  int result = readRegisters(LSM6DSOX_TIMESTAMP0, buffer, 4);
  if(result == 1) {
    timestamp = (((uint32_t)buffer[3]) << 24) | 
                (((uint32_t)buffer[2]) << 16) |
                (((uint32_t)buffer[1]) <<  8) |
                buffer[0];
  }
  return result;
}

int LSM6DSOXClass::readTimestampDouble(double& timestamp) {
  uint32_t t;
  int result = readTimestamp(t);
  if(result == 1) {
    timestamp = correctTimestamp(t);
  }
  return result;
}

int LSM6DSOXClass::resetTimestamp() {
  // See AN5272, par. 6.4
  return writeRegister(LSM6DSOX_TIMESTAMP2, 0xAA);
}

unsigned long long LSM6DSOXClass::correctTimestamp(unsigned long long timestamp64)
{
  // See AN5272, par 6.4
  // Multiply the timestamp (in units of 25us) with this factor and
  // divide it by 2^11, and the corrected timestamp in us is found.
  return (timestamp64 * internalFrequencyFactor) >> INTERNALFREQFACTOR_SHIFT;
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

#ifdef LSM6DS_DEFAULT_SPI
LSM6DSOXClass IMU_LSM6DSOX(LSM6DS_DEFAULT_SPI, PIN_SPI_SS1, LSM6DS_INT);
#else
LSM6DSOXClass IMU_LSM6DSOX(Wire, LSM6DSOX_ADDRESS);
#endif