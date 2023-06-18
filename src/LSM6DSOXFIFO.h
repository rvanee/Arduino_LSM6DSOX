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

#ifndef LSM6DSOXFIFO_H
#define LSM6DSOXFIFO_H

#include <Arduino.h>
#include <limits>
#include "TimestampEstimator.h"


// I2C buffer size is limited to 32 bytes, see link below.
// https://reference.arduino.cc/reference/en/language/functions/communication/wire/
#define I2C_BUFFER_LENGTH     32
#define READ_MAX_WORDS        (I2C_BUFFER_LENGTH / BUFFER_BYTES_PER_WORD)

#define BUFFER_WORDS          READ_MAX_WORDS  // IMU 'word' buffer size
#define BUFFER_BYTES_PER_WORD 7               // Tag + 3 * (2 byte word)
#define SAMPLE_BUFFER_SIZE    16              // Should be a power of 2
#define SAMPLE_BUFFER_MASK    (SAMPLE_BUFFER_SIZE-1)
#define SAMPLE_TAGCNT_MASK    3               // TAGCNT is 2 bits wide

#define FIFO_DATA_OUT_TAG     0
#define FIFO_DATA_OUT_X_L     1
#define FIFO_DATA_OUT_X_H     2
#define FIFO_DATA_OUT_Y_L     3
#define FIFO_DATA_OUT_Y_H     4
#define FIFO_DATA_OUT_Z_L     5
#define FIFO_DATA_OUT_Z_H     6

#define FIFO_INT16_NAN        0xFFFF
#define FIFO_FIXED_POINT_NAN  0xFFFFFFFF
#define FIFO_ULL_NAN          0xFFFFFFFFFFFFFFFF

enum class ReadResult {
  NO_DATA_AVAILABLE,
  DATA_READ,
  READ_ERROR,
  FIFO_OVERFLOW,
  QUEUE_FULL,
  LOGIC_ERROR
};

enum class DecodeTagResult {
  OK,
  TAG_NOT_IMPLEMENTED,
  UNKNOWN_TAG,
};

struct FIFOSettings {
public:
  // Batch Data Rates
  float     BDR_XL;                   // XL Batch Data Rate: 0-6667
  float     BDR_G;                    // G Batch Data Rate: 0-6667
  float     BDR_temperature;          // Temperature Batch Data Rate: 0/1.6/12.5/52
  uint8_t   timestamp_decimation;     // Timestamp every 0/1/8/32 batch data
  
  // Compression
  bool      compression;              // true = enable compression
  uint8_t   uncompressed_decimation;  // Uncompressed data every 0/8/16/32 batch data

  // Watermarks
  uint16_t  watermark_level;          // 9 bits (0-511)
  uint16_t  counter_threshold;        // 11 bits (0-2047)
  bool      counter_gyro;             // false = count XL, true = count G

  // FIFO operation. Do not change unless you know what you are doing...
  uint8_t   fifo_mode;                // 3-bit pattern, see datasheet
  bool      cfg_change;               // Store CFG-change data in FIFO. Default true
  bool      odr_change_en;            // Enables ODR CHANGE to be batched in FIFO
  bool      stop_on_wtm;              // Enables FIFO depth limit to threshold level
};

struct FIFOStatus {
public:
  bool FIFO_WTM_IA;     // FIFO watermark status: 1: FIFO filling >= WTM
  bool FIFO_OVR_IA;     // FIFO overrun status. 1: FIFO is completely filled
  bool FIFO_FULL_IA;    // Smart FIFO full status. 1: FIFO will be full at the next ODR
  bool COUNTER_BDR_IA;  // Counter BDR reaches CNT_BDR_TH_[10:0] threshold
  bool FIFO_OVR_LATCHED;// Latched FIFO overrun status
  uint16_t DIFF_FIFO;   // Number of unread sensor data words (TAG + 6 bytes) stored in FIFO
};

struct SampleData {
  int32_t XYZ[3];       // Fixed point data: decimal point between bits 15 and 16
  int16_t rawXYZ[3];
  uint16_t fullRange;
  bool valid;
};

struct Sample {
  SampleData G;
  SampleData XL;
  unsigned long long timestamp; // Timestamp in microseconds. May be FIFO_ULL_NAN
  float temperature;            // May be NaN
  uint32_t counter;             // Lowest 2 bits provided by tag byte (TAG_CNT)
};

// Utility: sign extension from B bits in 2's complement to int16 
template <const unsigned B>
inline int16_t signextend(const uint8_t x)
{
  struct {int16_t x:B;} s;
  return s.x = (int16_t)x;
}

// Utility: convert 2 bytes to (signed) int16
inline int16_t bytesToInt16(uint8_t hi, uint8_t lo)
{ 
  return ((int16_t)hi << 8) + lo;
}

class LSM6DSOXClass;

class LSM6DSOXFIFOClass {
  public:
    FIFOSettings settings;

    LSM6DSOXFIFOClass(LSM6DSOXClass* imu);
    ~LSM6DSOXFIFOClass();

    void initializeSettings(
      // Batch Data Rates
      float     BDR_XL = 104,             // XL Batch Data Rate: 0-6667
      float     BDR_G = 104,              // G Batch Data Rate: 0-6667
      float     BDR_temperature = 1.6,    // Temperature Batch Data Rate: 0/1.6/12.5/52
      uint8_t   timestamp_decimation = 1, // Timestamp every 0/1/8/32 batch data

      // Compression
      bool      compression = true,       // true = enable compression
      uint8_t   uncompressed_decimation = 32,// Uncompressed data every 0/8/16/32 batch data

      // Watermarks
      uint16_t  watermark_level = 0,      // 9 bits (0-511)
      uint16_t  counter_threshold = 9,    // 11 bits (0-2047)
      bool      counter_gyro = false,     // false = count XL, true = count G

      // FIFO operation. Do not change unless you know what you are doing...
      uint8_t   fifo_mode = 6,            // 3-bit pattern, see datasheet
      bool      odr_change_en = true,     // Enables CFG-Change auxiliary sensor to be batched in FIFO
      bool      stop_on_wtm = false);     // Enables FIFO depth limit to threshold level

    void            begin();
    void            end();

    int             readStatus(FIFOStatus& status);
    ReadResult      fillQueue();

    bool            retrieveSample(Sample& sample);

  private:   
    inline uint8_t  counterToIdx(uint32_t counter) {
      return (uint8_t)(counter & SAMPLE_BUFFER_MASK);
    }

    DecodeTagResult decodeWord(uint8_t *word);
    void            extend5bits(uint8_t hi, uint8_t lo, 
                                int16_t &delta_x, int16_t &delta_y, int16_t &delta_z);

    int32_t         raw2fixedrad(int16_t raw, uint16_t fullRange);

    void            setSampleData(SampleData *s, 
                                  int16_t X, int16_t Y, int16_t Z, uint16_t fullRange,
                                  bool valid, bool to_rad=false);
    void            initializeSample(uint8_t idx, bool setStatusInvalid = false);
    void            invalidateSample(uint8_t idx);

  private:   
    LSM6DSOXClass*  imu;

    // Sample buffer (management)
    Sample          sample_buffer[SAMPLE_BUFFER_SIZE];
    uint32_t        sample_counter; // Actual sample number. Lowest 2 bits resemble TAGCNT
    uint32_t        read_counter;   // First finished sample number that is not yet read

    // Timestamp (reconstruction)
    uint64_t        timestamp64;
    uint64_t        timestamp64_prev;
    uint32_t        timestamp64_counter;
    uint16_t        dt_per_sample;

    // MCU timestamp estimation
    TimestampEstimator MCU_timestamp_estimator;
    bool            use_MCU_timestamp;

    bool            compression_enabled;
    bool            timestamp_reconstruction_enabled;
};

#endif // LSM6DSOXFIFO_H
