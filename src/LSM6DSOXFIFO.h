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

// Buffer size. Define extra slots for compression, timestamp and config change
#define BUFFER_WORDS          512  // Number of 'words'
#define BUFFER_BYTES_PER_WORD 7    // Tag + 3 * (2 byte word)
#define TAGCNT_BUFFER_SIZE    4    // 4 possible TAGCNT values (0-3)

#define FIFO_DATA_OUT_TAG     0
#define FIFO_DATA_OUT_X_L     1
#define FIFO_DATA_OUT_X_H     2
#define FIFO_DATA_OUT_Y_L     3
#define FIFO_DATA_OUT_Y_H     4
#define FIFO_DATA_OUT_Z_L     5
#define FIFO_DATA_OUT_Z_H     6

enum class SampleStatus { 
  OK,
  BUFFER_UNDERRUN,
  BUFFER_OVERRUN,
  TAG_NOT_IMPLEMENTED,
  UNKNOWN_TAG,
  PARITY_ERROR,
  COMMUNICATION_ERROR
};

struct FIFOSettings {
public:
  // Batch Data Rates
  float     BDR_XL;                   // XL Batch Data Rate: 0-6667
  float     BDR_G;                    // G Batch Data Rate: 0-6667
  float     BDR_temperature;          // Temperature Batch Data Rate: 0/1.6/12.5/52
  uint8_t   timestamp_decimation;     // Timestamp every 0/1/8/32 batch data
  bool      timestamp_reconstruction; // Reconstruct unbatched timestamps
  
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

struct Sample {
  int16_t G_X;
  int16_t G_Y;
  int16_t G_Z;
  int16_t XL_X;
  int16_t XL_Y;
  int16_t XL_Z;
  double timestamp;     // May be NaN
  float temperature;    // May be NaN
  uint32_t counter;     // Lowest 2 bits provided by tag byte (TAG_CNT)
  uint8_t fullRange_XL;
  uint16_t fullRange_G;
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
    FIFOSettings  settings;

    LSM6DSOXFIFOClass(LSM6DSOXClass* imu);
    ~LSM6DSOXFIFOClass();

    void initializeSettings(
      // Batch Data Rates
      float     BDR_XL = 104,             // XL Batch Data Rate: 0-6667
      float     BDR_G = 104,              // G Batch Data Rate: 0-6667
      float     BDR_temperature = 1.6,    // Temperature Batch Data Rate: 0/1.6/12.5/52
      uint8_t   timestamp_decimation = 1, // Timestamp every 0/1/8/32 batch data
      bool      timestamp_reconstruction = true, // Reconstruct unbatched timestamps

      // Compression
      bool      compression = true,       // true = enable compression
      uint8_t   uncompressed_decimation = 16,// Uncompressed data every 0/8/16/32 batch data

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
    SampleStatus    getSample(Sample& sample);

  private:   
    SampleStatus    inspectWord(uint16_t idx);
    int             readData(uint16_t& words_read, bool& too_full, FIFOStatus& status);
    int             releaseSample(uint16_t idx, Sample& extracted_sample);
    SampleStatus    decodeWord(uint16_t idx);

    inline uint8_t* buffer_pointer(uint16_t idx) {
      return &buffer[idx * BUFFER_BYTES_PER_WORD];
    }
    void            initializeSample(uint8_t idx);

  private:   
    LSM6DSOXClass*  imu;

    // Sample buffer (management)
    Sample          sample[TAGCNT_BUFFER_SIZE]; // Ring buffer, contains the words at T-3, T-2, T-1 and T
    uint32_t        sample_counter;
    uint32_t        to_release_counter;

    // Timestamp (reconstruction)
    uint64_t        timestamp64;
    uint64_t        timestamp64_prev;
    uint32_t        timestamp_counter;
    float           dt_per_sample;

    // Data buffer (management)
    uint8_t         buffer[BUFFER_WORDS * BUFFER_BYTES_PER_WORD];
    uint16_t        read_idx;
    uint16_t        write_idx;
    bool            buffer_empty;

    bool            compressionEnabled;

    /*/ For debugging purposes
    void displaySamples();
    //*/

  public:
    // The compression algorithm may set the counter to a value of 32 bit unsigned int max
    // value -2/-1/-0.
    // So max uint32 value - 3 is the largest counter value that will not be set by 
    // the algorithm (at least not until after 1 or more weeks of continuous
    // sensing, mere milliseconds before overrun to 0).
    static const uint32_t counter_uninitialized = std::numeric_limits<uint32_t>::max()-3;
};

#endif // LSM6DSOXFIFO_H
