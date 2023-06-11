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


#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "LSM6DSOXFIFO.h"

enum PowerModeXL {
  XL_POWER_UNDEFINED,
  XL_POWER_DOWN,
  XL_POWER_MODE_ULP,
  XL_POWER_MODE_LP_NORMAL,
  XL_POWER_MODE_HP
};
enum PowerModeG {
  G_POWER_UNDEFINED,
  G_POWER_DOWN,
  G_POWER_MODE_LP_NORMAL,
  G_POWER_MODE_HP
};

struct SensorSettings {
public:
  float       ODR_XL;
  float       ODR_G;
  float       fullRange_XL;
  float       fullRange_G;
  float       cutoff_LPF2_XL;
  float       cutoff_LPF1_G;
  float       cutoff_HPF_G;
  PowerModeXL powerMode_XL;
  PowerModeG  powerMode_G;
  int         i2c_frequency;
  bool        enable_timestamp;
  bool        BDU;
};

class LSM6DSOXClass {
  friend class LSM6DSOXFIFOClass;
  public:
    LSM6DSOXClass(TwoWire& wire, uint8_t slaveAddress);
    LSM6DSOXClass(SPIClass& spi, int csPin, int irqPin);
    ~LSM6DSOXClass();
    
    void initializeSettings(
      float ODR_XL = 104,
      float ODR_G = 104, 
      float fullRange_XL = 4,
      float fullRange_G = 2000,
      float cutoff_LPF2_XL = 52,
      float cutoff_LPF1_G = 52,
      float cutoff_HPF_G = 0,
      PowerModeXL powerMode_XL = XL_POWER_MODE_HP,
      PowerModeG powerMode_G = G_POWER_MODE_HP,
      int i2c_frequency = 100000,
      bool enable_timestamp = true,
      bool BDU = false);

    int begin();
    void end();

    // Configuration
    int setPowerModeXL(PowerModeXL powerMode_XL);
    int setPowerModeG(PowerModeG powerMode_G);

    int setODR_XL(float odr);
    int setODR_G(float odr);

    int setFullRange_XL(float range);
    int setFullRange_G(float range);

    int setLPF2_XL(float cutoff, float odr);
    int setLPF1_G(float cutoff, float odr);
    int setHPF_G(float cutoff);

    // Accelerometer
    int readAcceleration(float& x, float& y, float& z); // Results are in g (earth gravity).
    float accelerationSampleRate(); // Sampling rate of the sensor.
    int accelerationAvailable(); // Check for available data from accelerometer

    // Gyroscope
    int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
    float gyroscopeSampleRate(); // Sampling rate of the sensor.
    int gyroscopeAvailable(); // Check for available data from gyroscope

    // Temperature
    int readTemperature(int& temperature_deg);
    int readTemperatureFloat(float& temperature_deg);
    int temperatureAvailable();

    // Timestamp
    int readTimestamp(uint32_t& timestamp);
    int readTimestampDouble(double& timestamp);
    int resetTimestamp();
    unsigned long long correctTimestamp(unsigned long long timestamp64);

  public:
    //IMU settings
    SensorSettings settings;

    // FIFO
    LSM6DSOXFIFOClass fifo;

    // Actual full range values. These are updated upon each configuration change
    uint16_t fullRange_XL;
    uint16_t fullRange_G;

  private:
    int readRegister(uint8_t address);
    int readRegisters(uint8_t address, uint8_t* data, size_t length);
    int writeRegister(uint8_t address, uint8_t value);
    int readModifyWriteRegister(uint8_t address, uint8_t value, uint8_t mask);

    float temperatureIntToCelsius(int temperature_raw);

  private:
    TwoWire* _wire;
    SPIClass* _spi;
    uint8_t _slaveAddress;
    int _csPin;
    int _irqPin;

    SPISettings _spiSettings;

    uint16_t internalFrequencyFactor;
};

extern LSM6DSOXClass IMU_LSM6DSOX;
#undef IMU
#define IMU IMU_LSM6DSOX

#endif // LSM6DSOX_H
