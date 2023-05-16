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

#include "LSM6DSOXFIFO.h"
#include "LSM6DSOX.h"
#include "LSM6DSOXTables.h"

#include <math.h> // isnan


#define LSM6DSOX_ADDRESS            0x6A

#define LSM6DSOX_FUNC_CFG_ACCESS    0x01
#define LSM6DSOX_FIFO_CTRL1         0x07
#define LSM6DSOX_FIFO_CTRL2         0x08
#define LSM6DSOX_FIFO_CTRL3         0x09
#define LSM6DSOX_FIFO_CTRL4         0x0A
#define LSM6DSOX_COUNTER_BDR_REG1   0x0B
#define LSM6DSOX_COUNTER_BDR_REG2   0x0C

#define LSM6DSOX_STATUS1            0x3A
#define LSM6DSOX_STATUS2            0x3B

#define LSM6DSOX_FIFO_DATA_OUT_TAG  0x78

//#define LSM6DSOX_EMB_FUNC_PAGE_SEL  0x02
#define LSM6DSOX_EMB_FUNC_EN_B      0x05
//#define LSM6DSOX_EMB_FUNC_PAGE_RW   0x17

// Bit masks
#define MASK_FUNC_CFG_ACCESS        0x80  // FUNC_CFG_ACCESS
#define MASK_FIFO_COMPR_EN          0x08  // EMB_FUNC_EN_B
#define MASK_WTM7_0                 0xFF  // FIFO_CTRL1
#define MASK_WTM8                   0x01  // FIFO_CTRL2
#define MASK_UNCOPTR_RATE           0x06  // FIFO_CTRL2
#define MASK_ODRCHG_EN              0x10  // FIFO_CTRL2
#define MASK_FIFO_COMPR_RT_EN       0x40  // FIFO_CTRL2
#define MASK_STOP_ON_WTM            0x80  // FIFO_CTRL2
#define MASK_BDR_XL                 0x0F  // FIFO_CTRL3
#define MASK_BDR_GY                 0xF0  // FIFO_CTRL3
#define MASK_FIFO_MODE              0x07  // FIFO_CTRL4
#define MASK_ODR_T_BATCH            0x30  // FIFO_CTRL4
#define MASK_DEC_TS_BATCH           0xC0  // FIFO_CTRL4
#define MASK_CNT_BDR_TH_10_8        0x07  // COUNTER_BDR_REG1
#define MASK_CNT_BDR_TH_7_0         0xFF  // COUNTER_BDR_REG2
#define MASK_DIFF_FIFO_H            0x03  // FIFO_STATUS2
#define MASK_FIFO_OVR_LATCHED       0x08  // FIFO_STATUS2
#define MASK_COUNTER_BDR_IA         0x10  // FIFO_STATUS2
#define MASK_FIFO_FULL_IA           0x20  // FIFO_STATUS2
#define MASK_FIFO_OVR_IA            0x40  // FIFO_STATUS2
#define MASK_FIFO_WTM_IA            0x80  // FIFO_STATUS2
#define MASK_FIFO_TAG_PARITY        0x01  // FIFO_DATA_OUT_TAG
#define MASK_FIFO_TAG_CNT           0x06  // FIFO_DATA_OUT_TAG

// 3D array indexing
#define X_IDX                       0
#define Y_IDX                       1
#define Z_IDX                       2


LSM6DSOXFIFOClass::LSM6DSOXFIFOClass(LSM6DSOXClass* imu) {
  this->imu = imu;

  compressionEnabled = false;

  initializeSettings();
}

LSM6DSOXFIFOClass::~LSM6DSOXFIFOClass() {
  end();
}

void LSM6DSOXFIFOClass::initializeSettings(
  // Batch Data Rates
  float     BDR_XL,                   // XL Batch Data Rate: 0-6667
  float     BDR_G,                    // G Batch Data Rate: 0-6667
  float     BDR_temperature,          // Temperature Batch Data Rate: 0/1.6/12.5/52
  uint8_t   timestamp_decimation,     // Timestamp every 0/1/8/32 batch data
  bool      timestamp_reconstruction, // Reconstruct unbatched timestamps
  
  // Compression
  bool      compression,              // true = enable compression
  uint8_t   uncompressed_decimation,  // Uncompressed data every 0/8/16/32 batch data

  // Watermarks
  uint16_t  watermark_level,          // 9 bits (0-511)
  uint16_t  counter_threshold,        // 11 bits (0-2047)
  bool      counter_gyro,             // false = count XL, true = count G

  // FIFO operation. Do not change unless you know what you are doing...
  uint8_t   fifo_mode,                // 3-bit pattern, see datasheet
  bool      odr_change_en,            // Enables CFG-Change auxiliary sensor to be batched in FIFO
  bool      stop_on_wtm)              // Enables FIFO depth limit to threshold level
{          
  // Batch Data Rates
  settings.BDR_XL = BDR_XL;
  settings.BDR_G = BDR_G;
  settings.BDR_temperature = BDR_temperature;
  settings.timestamp_decimation = timestamp_decimation;
  settings.timestamp_reconstruction = timestamp_reconstruction;
  
  // Compression
  settings.compression = compression;
  settings.uncompressed_decimation = uncompressed_decimation;
    
  // Watermarks
  settings.watermark_level = watermark_level;
  settings.counter_threshold = counter_threshold;
  settings.counter_gyro = counter_gyro;

  // FIFO operation. Do not change unless you know what you are doing...
  settings.fifo_mode = fifo_mode;
  settings.odr_change_en = odr_change_en;
  settings.stop_on_wtm = stop_on_wtm;
}

void LSM6DSOXFIFOClass::begin()
{
  // Turn off FIFO. This will also empty its queue
  end();

  // Process settings

  // Enable or disable compression using FIFO_COMPR_EN in EMB_FUNC_EN_B
  // Note that this requires first enabling, then disabling embedded function registers
  uint8_t fifo_compr_en = settings.compression ? MASK_FIFO_COMPR_EN : 0x00;
  // Enable embedded function registers access
  imu->readModifyWriteRegister(LSM6DSOX_FUNC_CFG_ACCESS, MASK_FUNC_CFG_ACCESS, MASK_FUNC_CFG_ACCESS);
  // Enable or disable compression
  imu->readModifyWriteRegister(LSM6DSOX_EMB_FUNC_EN_B, fifo_compr_en, MASK_FIFO_COMPR_EN);
  // Disable embedded function registers access
  imu->readModifyWriteRegister(LSM6DSOX_FUNC_CFG_ACCESS, 0x00, MASK_FUNC_CFG_ACCESS);

  // FIFO_CTRL1
  uint8_t wtm7_0 = settings.watermark_level & MASK_WTM7_0;
  imu->writeRegister(LSM6DSOX_FIFO_CTRL1, wtm7_0);

  // FIFO_CTRL2
  uint8_t wtm8 = settings.watermark_level >> 8;
  uint8_t uncoptr_rate = LSM6DSOXTables::nearestFloatToBits(settings.uncompressed_decimation, LSM6DSOXTables::UNCOPTR_RATE_bits);
  uint8_t odrchg_en = settings.odr_change_en ? MASK_ODRCHG_EN : 0x00;
  uint8_t fifo_compr_rt_en = settings.compression ? MASK_FIFO_COMPR_RT_EN : 0x00;
  uint8_t stop_on_wtm = settings.stop_on_wtm ? MASK_STOP_ON_WTM : 0x00;
  imu->writeRegister(LSM6DSOX_FIFO_CTRL2, stop_on_wtm | fifo_compr_rt_en | odrchg_en | uncoptr_rate | wtm8);
  
  // CNT_BDR_TH
  uint8_t counter_bdr_th_10_8 = settings.counter_threshold >> 8;
  imu->readModifyWriteRegister(LSM6DSOX_COUNTER_BDR_REG1, counter_bdr_th_10_8, MASK_CNT_BDR_TH_10_8);
  imu->writeRegister(LSM6DSOX_COUNTER_BDR_REG2, settings.counter_threshold & MASK_CNT_BDR_TH_7_0);
  
  // FIFO_CTRL4: ODR T, decimation TS, FIFO MODE
  uint8_t odr_t_batch = LSM6DSOXTables::nearestFloatToBits(settings.BDR_temperature, LSM6DSOXTables::ODR_T_BATCH_bits) << 4;
  uint8_t dec_ts_batch = LSM6DSOXTables::nearestFloatToBits(settings.timestamp_decimation, LSM6DSOXTables::DEC_TS_BATCH_bits) << 6;
  imu->writeRegister(LSM6DSOX_FIFO_CTRL4, dec_ts_batch | odr_t_batch | settings.fifo_mode);

  // FIFO_CTRL3: BDR XL + G
  uint8_t odr_bits_xl = LSM6DSOXTables::nearestFloatToBits(settings.BDR_XL, LSM6DSOXTables::ODR_freq_bits);
  // Handle special case of 1.6Hz, which is unique to XL. It needs to be selected
  // if 0 < odr < (halfway 1.6 and 12.5)
  if((settings.BDR_XL > 0.0) && (settings.BDR_XL < (12.5 - 1.6)/2)) {
      odr_bits_xl = 0b1011;
  } 
  uint8_t odr_bits_g = LSM6DSOXTables::nearestFloatToBits(settings.BDR_G, LSM6DSOXTables::ODR_freq_bits) << 4;
  // Handle special case of 6.5Hz, which is unique to G. It needs to be selected
  // if 0 < odr < (halfway 6.5 and 12.5)
  if((settings.BDR_G > 0.0) && (settings.BDR_G < (12.5 - 6.5)/2)) {
      odr_bits_g = 0b10110000;
  } 
  imu->writeRegister(LSM6DSOX_FIFO_CTRL3, odr_bits_g | odr_bits_xl);

  // Reset internal state

  // Buffer management
  read_idx = 0;
  write_idx = 0;
  buffer_empty = true;

  // Decoder
  compressionEnabled = settings.compression;
  sample_counter = counter_uninitialized;
  to_release_counter = counter_uninitialized;

  // Timestamp (reconstruction)
  timestamp64 = 0;
  timestamp64_prev = 0;
  timestamp_counter = counter_uninitialized;
  dt_per_sample = NAN;

  // Initialize circular tagcnt buffer. Set G and XL data to invalid
  for(uint8_t idx = 0; idx < TAGCNT_BUFFER_SIZE; idx++) {
    initializeSample(idx, true);
  } 
}

void LSM6DSOXFIFOClass::end()
{
  // Disable XL and G batching
  imu->writeRegister(LSM6DSOX_FIFO_CTRL3, 0x00);

  // Disable timestamp and temperature batching, and set FIFO mode=0 (FIFO disabled)
  // Note that this will also reset/empty the FIFO's queue
  imu->writeRegister(LSM6DSOX_FIFO_CTRL4, 0x00);
}

int LSM6DSOXFIFOClass::readStatus(FIFOStatus& status)
{
  uint8_t status_registers[2];
  
  int result = imu->readRegisters(LSM6DSOX_STATUS1, status_registers, 2);
  if(result == 1) {
    status.DIFF_FIFO = ((uint16_t)(status_registers[1] & MASK_DIFF_FIFO_H) << 8) | status_registers[0];
    status.FIFO_OVR_LATCHED = status_registers[1] & MASK_FIFO_OVR_LATCHED;
    status.COUNTER_BDR_IA = status_registers[1] & MASK_COUNTER_BDR_IA;
    status.FIFO_FULL_IA = status_registers[1] & MASK_FIFO_FULL_IA;
    status.FIFO_OVR_IA = status_registers[1] & MASK_FIFO_OVR_IA;
    status.FIFO_WTM_IA = status_registers[1] & MASK_FIFO_WTM_IA;
  }
  return result;
}

ReadResult LSM6DSOXFIFOClass::fillQueue()
{
  FIFOStatus status;
  int result = readStatus(status);
  if(result != 1) return ReadResult::READ_ERROR;

  // Check if new data available
  uint16_t to_read = status.DIFF_FIFO;
  if(to_read == 0) {
    return ReadResult::NO_DATA_AVAILABLE;
  }

  // Check for FIFO overflow
  if(status.FIFO_OVR_IA) {
    return ReadResult::FIFO_OVERFLOW;
  }

  // Read data from FIFO to buffer
  while(to_read > 0) {
    // Break down read operations into a maximum number of words
    uint16_t read_now = min(to_read, READ_MAX_WORDS);

    result = imu->readRegisters(LSM6DSOX_FIFO_DATA_OUT_TAG, buffer, read_now*BUFFER_BYTES_PER_WORD);
    if(result != 1) return ReadResult::READ_ERROR;

    // Process read words
    for(uint16_t wordidx = 0; wordidx < read_now; wordidx++) {
      uint8_t *word = buffer_pointer(wordidx);
      uint8_t tag_byte = word[FIFO_DATA_OUT_TAG];

      // Perform parity check
      uint8_t parity = tag_byte ^ (tag_byte >> 4);
      parity ^= (parity >> 2);
      parity ^= (parity >> 1);
      if(parity & MASK_FIFO_TAG_PARITY) {
        // This is a serious error, probably related to communication with
        // the IMU. Since the tag byte contains TAGCNT as well as TAG data,
        // we don't know if TAGCNT has increased, and if XL or G data is
        // involved.
        // If no compression is enabled, sample data for TAGCNT+0/1 should
        // be invalidated. If compression is enabled, sample data for
        // TAGCNT-1/2 should also be invalidated.
        invalidateSample(0);
        invalidateSample(1);
        if(compressionEnabled) {
          invalidateSample(3); // -1 MOD 4 = 3
          invalidateSample(2); // -2 MOD 4 = 2
        }
        // This word can't be evaluated any further, but we can go on 
        // decoding the next words
        continue;
      }

      // Check if a sample can be released.
      // Also manages counters and timestamps
      uint8_t tagcnt = (tag_byte & MASK_FIFO_TAG_CNT) >> 1;
      Sample extracted_sample;
      if(releaseSample(tagcnt, extracted_sample)) {
        // Store sample in queue
        if (!sampleQueue.putQ(extracted_sample)) {
          return ReadResult::QUEUE_FULL;
        }
      }

      // Decode word, writes to small circular sample buffer
      DecodeTagResult decodeResult = decodeWord(word);
      switch(decodeResult) {
        case DecodeTagResult::OK:
          // All fine
        case DecodeTagResult::TAG_NOT_IMPLEMENTED:
          // Not a problem, this tag doesn't hurt our purpose of
          // decoding samples
          break;
        case DecodeTagResult::UNKNOWN_TAG:
          // This may be a serious problem, arising from a
          // communication error. Or it may indicate using
          // this code on a later version of the IMU, that
          // may or may not be backwards compatible.
          // Mitigation is in decodeWord(), where sample 
          // data is invalidated for both XL and G data.
          break;
        default:
          // We really should not be here, so this is an
          // error in the code logic that we can't
          // recover from
          return ReadResult::LOGIC_ERROR;
      } 
    } // END for(uint16_t wordidx = 0...

    to_read -= read_now;
  } // END while(to_read > 0)

  return ReadResult::DATA_READ;
}

int LSM6DSOXFIFOClass::releaseSample(uint8_t tagcnt, Sample& extracted_sample)
{
  // Note: this function updates previoustagcnt, sample_counter and initializes
  // a new sample in the circular sample buffer

  // sample_counter is undefined at fifo startup
  if(sample_counter == counter_uninitialized) {
    // If still uninitialized, initialize it with tagcnt,
    // so the lower 2 bits always resemble tagcnt
    sample_counter = tagcnt;

    // The first sample to be released is that of
    // the current timestamp counter
    to_release_counter = sample_counter;
  }

  // Update counter based on tagcnt
  uint8_t prev_tagcnt = sample_counter & 0x03;
  if(tagcnt != prev_tagcnt) {
    // TAGCNT rollover?
    if(tagcnt < prev_tagcnt) {
      sample_counter += 4;
    }
    sample_counter &= 0xFFFFFFFC;
    sample_counter |= tagcnt;
  }

  // If compression is enabled, there should be a delay of
  // 2 in releasing samples to account for the compression
  // algorithm modifying data at T-2 and T-1, rather than 
  // just at time T.
  uint8_t delta_cnt = compressionEnabled << 1;
  if((to_release_counter + delta_cnt) < sample_counter) {
    uint8_t releasecnt = (uint8_t)(to_release_counter & 0x03);
    extracted_sample = sample[releasecnt];
    initializeSample(releasecnt);
    to_release_counter++;

    // Timestamp reconstruction
    if(settings.timestamp_reconstruction) {
      // Reconstruction is only necessary if sample's timestamp undefined...
      if(isnan(extracted_sample.timestamp) && 
        // ... and only possible if delta timestamp per sample is known
         !isnan(dt_per_sample)) {
        // Calculate number of samples between newest timestamp and this sample
        int delta_samples = (extracted_sample.counter > timestamp_counter) ?
          extracted_sample.counter - timestamp_counter :
          -int(timestamp_counter - extracted_sample.counter);
        // Set sample's timestamp relative to latest timestamp and correct it
        extracted_sample.timestamp = 
          imu->_internalFrequencyFactor * (timestamp64 + delta_samples*dt_per_sample);
      }
    }

    return 1; // Sample released
  }

  return 0; // No sample released
}

DecodeTagResult LSM6DSOXFIFOClass::decodeWord(uint8_t *word)
{
  // Note: this function updates fullRange_G, fullRange_XL and compressionEnabled,
  // as well as the sample circular buffer
  
  // Tag counters
  uint8_t tag_byte = word[FIFO_DATA_OUT_TAG];
  uint8_t tagcnt = (tag_byte & MASK_FIFO_TAG_CNT) >> 1;  // T
  uint8_t tagcnt_1 = (tagcnt-1) & 0x03; // T-1 mod 4
  uint8_t tagcnt_2 = (tagcnt-2) & 0x03; // T-2 mod 4
  uint8_t tagcnt_3 = (tagcnt-3) & 0x03; // T-3 mod 4

  // Decode tag
  uint8_t tag = tag_byte >> 3;
  switch(tag) {
    case 0x01: // Gyroscope NC Main Gyroscope uncompressed data
    {
      setSampleData(sample[tagcnt].G,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_G, true);
      sample[tagcnt].counter = sample_counter;
      break;
    }

    case 0x02: // Accelerometer NC Main Accelerometer uncompressed data
    {
      setSampleData(sample[tagcnt].XL,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_XL, true);
      sample[tagcnt].counter = sample_counter;
      break;
    }

    case 0x03: // Temperature Auxiliary Temperature data
    {
      sample[tagcnt].temperature = 
        imu->temperatureIntToCelsius(bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]));
      break;
    }

    case 0x04: // Timestamp Auxiliary Timestamp data
    {
      uint32_t timestamp = 
        (((uint32_t)word[FIFO_DATA_OUT_Y_H]) << 24) | 
        (((uint32_t)word[FIFO_DATA_OUT_Y_L]) << 16) |
        (((uint32_t)word[FIFO_DATA_OUT_X_H]) <<  8) |
         ((uint32_t)word[FIFO_DATA_OUT_X_L]);
      
      // For timestamp reconstruction:
      // store old timestamp64 value
      timestamp64_prev = timestamp64;

      // Test for timestamp overrun, i.e. from large 32-bit value in previous step
      // to value close to 0
      if((timestamp64 & 0xFFFFFFFF) > timestamp) {
        timestamp64 += (1ULL << 32); // Add one to highest 32 bits
      }
      // Now replace lower 32 bits of timestamp64 with new 32-bit timestamp value
      timestamp64 = (timestamp64 & 0xFFFFFFFF00000000) | timestamp;

      // Store (corrected) timestamp in current sample
      sample[tagcnt].timestamp = timestamp64 * imu->_internalFrequencyFactor;

      // Calculate delta timestamp per sample (used for timestamp reconstruction)
      dt_per_sample = (sample_counter    == counter_uninitialized) ||
                      (timestamp_counter == counter_uninitialized) ?
                      NAN :
                      float(timestamp64    - timestamp64_prev) / 
                           (sample_counter - timestamp_counter);

      // store current sample counter as last timestamp counter
      timestamp_counter = sample_counter;
      break;
    }

    case 0x05: // CFG_Change Auxiliary Meta-information data
    {
      // Gyro full range
      uint8_t fs_g = word[FIFO_DATA_OUT_X_H] >> 5; // FS1_G FS0_G FS_125
      imu->fullRange_G = LSM6DSOXTables::getFloatFromBits(fs_g, LSM6DSOXTables::FR_G_bits);
      sample[tagcnt].G.fullRange = imu->fullRange_G;

      // Accelerometer full range
      uint8_t fs_xl = word[FIFO_DATA_OUT_Y_L] >> 6; // FS1_XL FS0_XL
      imu->fullRange_XL = LSM6DSOXTables::getFloatFromBits(fs_xl, LSM6DSOXTables::FR_XL_bits);
      sample[tagcnt].XL.fullRange = imu->fullRange_XL;

      // Compression
      compressionEnabled = word[FIFO_DATA_OUT_Y_H] & 0x80;
      break;
    }

    case 0x06: // Accelerometer NC_T_2 Main Accelerometer uncompressed batched at two times the previous time slot
    {
      setSampleData(sample[tagcnt_2].XL,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_XL, true);
      sample[tagcnt_2].counter = sample_counter-2;
      break;
    }

    case 0x07: // Accelerometer NC_T_1 Main Accelerometer uncompressed data batched at the previous time slot
    {
      setSampleData(sample[tagcnt_1].XL,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_XL, true);
      sample[tagcnt_1].counter = sample_counter-1;
      break;
    }

    case 0x08: // Accelerometer 2xC Main Accelerometer 2x compressed data
    {
      int16_t XL_X_2 = sample[tagcnt_3].XL.rawXYZ[X_IDX] + signextend<8>(word[FIFO_DATA_OUT_X_L]);
      int16_t XL_Y_2 = sample[tagcnt_3].XL.rawXYZ[Y_IDX] + signextend<8>(word[FIFO_DATA_OUT_X_H]);
      int16_t XL_Z_2 = sample[tagcnt_3].XL.rawXYZ[Z_IDX] + signextend<8>(word[FIFO_DATA_OUT_Y_L]);
      // Note that TAGCNT-2 sample's validity depends on the old TAGCNT-3 sample's validity
      setSampleData(sample[tagcnt_2].XL, XL_X_2, XL_Y_2, XL_Z_2, imu->fullRange_XL, sample[tagcnt_3].XL.valid);
      sample[tagcnt_2].counter = sample_counter-2;

      int16_t XL_X_1 = XL_X_2 + signextend<8>(word[FIFO_DATA_OUT_Y_H]);
      int16_t XL_Y_1 = XL_Y_2 + signextend<8>(word[FIFO_DATA_OUT_Z_L]);
      int16_t XL_Z_1 = XL_Z_2 + signextend<8>(word[FIFO_DATA_OUT_Z_H]);
      // Note that TAGCNT-1 sample's validity depends on TAGCNT-2 sample's validity
      setSampleData(sample[tagcnt_1].XL, XL_X_1, XL_Y_1, XL_Z_1, imu->fullRange_XL, sample[tagcnt_2].XL.valid);
      sample[tagcnt_1].counter = sample_counter-1;
      break;
    }

    case 0x09: // Accelerometer 3xC Main Accelerometer 3x compressed data
    {
      int16_t dx, dy, dz;

      extend5bits(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L], dx, dy, dz);
      int16_t XL_X_2 = sample[tagcnt_3].XL.rawXYZ[X_IDX] + dx;
      int16_t XL_Y_2 = sample[tagcnt_3].XL.rawXYZ[Y_IDX] + dy;
      int16_t XL_Z_2 = sample[tagcnt_3].XL.rawXYZ[Z_IDX] + dz;
      // Note that TAGCNT-2 sample's validity depends on the old TAGCNT-3 sample's validity
      setSampleData(sample[tagcnt_2].XL, XL_X_2, XL_Y_2, XL_Z_2, imu->fullRange_XL, sample[tagcnt_3].XL.valid);
      sample[tagcnt_2].counter = sample_counter-2;

      extend5bits(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L], dx, dy, dz);
      int16_t XL_X_1 = XL_X_2 + dx;
      int16_t XL_Y_1 = XL_Y_2 + dy;
      int16_t XL_Z_1 = XL_Z_2 + dz;
      // Note that TAGCNT-1 sample's validity depends on TAGCNT-2 sample's validity
      setSampleData(sample[tagcnt_1].XL, XL_X_1, XL_Y_1, XL_Z_1, imu->fullRange_XL, sample[tagcnt_2].XL.valid);
      sample[tagcnt_1].counter = sample_counter-1;

      extend5bits(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L], dx, dy, dz);
      int16_t XL_X = XL_X_1 + dx;
      int16_t XL_Y = XL_Y_1 + dy;
      int16_t XL_Z = XL_Z_1 + dz;
      // Note that TAGCNT sample's validity depends on TAGCNT-1 sample's validity
      setSampleData(sample[tagcnt].XL, XL_X, XL_Y, XL_Z, imu->fullRange_XL, sample[tagcnt_1].XL.valid);
      sample[tagcnt  ].counter = sample_counter;
      break;
    }

    case 0x0A: // Gyroscope NC_T_2 Main Gyroscope uncompressed data batched at two times the previous time slot
    {
      setSampleData(sample[tagcnt_2].G,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_G, true);
      sample[tagcnt_2].counter = sample_counter-2; // Necessary if XL is not recorded in FIFO
      break;    
    }

    case 0x0B: // Gyroscope NC_T_1 Main Gyroscope uncompressed data batched at the previous time slot
    {
      setSampleData(sample[tagcnt_1].G,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_G, true);
      sample[tagcnt_1].counter = sample_counter-1; // Necessary if XL is not recorded in FIFO
      break;
    }

    case 0x0C: // Gyroscope 2xC Main Gyroscope 2x compressed data
    {
      int16_t G_X_2 = sample[tagcnt_3].G.rawXYZ[X_IDX] + signextend<8>(word[FIFO_DATA_OUT_X_L]);
      int16_t G_Y_2 = sample[tagcnt_3].G.rawXYZ[Y_IDX] + signextend<8>(word[FIFO_DATA_OUT_X_H]);
      int16_t G_Z_2 = sample[tagcnt_3].G.rawXYZ[Z_IDX] + signextend<8>(word[FIFO_DATA_OUT_Y_L]);
      // Note that TAGCNT-2 sample's validity depends on the old TAGCNT-3 sample's validity
      setSampleData(sample[tagcnt_2].G, G_X_2, G_Y_2, G_Z_2, imu->fullRange_G, sample[tagcnt_3].G.valid);
      sample[tagcnt_2].counter = sample_counter-2;  // Necessary if XL is not recorded in FIFO

      int16_t G_X_1 = G_X_2 + signextend<8>(word[FIFO_DATA_OUT_Y_H]);
      int16_t G_Y_1 = G_Y_2 + signextend<8>(word[FIFO_DATA_OUT_Z_L]);
      int16_t G_Z_1 = G_Z_2 + signextend<8>(word[FIFO_DATA_OUT_Z_H]);
      // Note that TAGCNT-1 sample's validity depends on TAGCNT-2 sample's validity
      setSampleData(sample[tagcnt_1].G, G_X_1, G_Y_1, G_Z_1, imu->fullRange_G, sample[tagcnt_2].G.valid);
      sample[tagcnt_1].counter = sample_counter-1;  // Necessary if XL is not recorded in FIFO
      break;
    }

    case 0x0D: // Gyroscope 3xC Main Gyroscope 3x compressed data
    {
      int16_t dx, dy, dz;

      extend5bits(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L], dx, dy, dz);
      int16_t G_X_2 = sample[tagcnt_3].G.rawXYZ[X_IDX] + dx;
      int16_t G_Y_2 = sample[tagcnt_3].G.rawXYZ[Y_IDX] + dy;
      int16_t G_Z_2 = sample[tagcnt_3].G.rawXYZ[Z_IDX] + dz;
      // Note that TAGCNT-2 sample's validity depends on the old TAGCNT-3 sample's validity
      setSampleData(sample[tagcnt_2].G, G_X_2, G_Y_2, G_Z_2, imu->fullRange_G, sample[tagcnt_3].G.valid);
      sample[tagcnt_2].counter = sample_counter-2; // Necessary if XL is not recorded in FIFO

      extend5bits(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L], dx, dy, dz);
      int16_t G_X_1 = G_X_2 + dx;
      int16_t G_Y_1 = G_Y_2 + dy;
      int16_t G_Z_1 = G_Z_2 + dz;
      // Note that TAGCNT-1 sample's validity depends on TAGCNT-2 sample's validity
      setSampleData(sample[tagcnt_1].G, G_X_1, G_Y_1, G_Z_1, imu->fullRange_G, sample[tagcnt_2].G.valid);
      sample[tagcnt_1].counter = sample_counter-1;

      extend5bits(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L], dx, dy, dz);
      int16_t G_X = G_X_1 + dx;
      int16_t G_Y = G_Y_1 + dy;
      int16_t G_Z = G_Z_1 + dz;
      // Note that TAGCNT sample's validity depends on TAGCNT-1 sample's validity
      setSampleData(sample[tagcnt].G, G_X, G_Y, G_Z, imu->fullRange_G, sample[tagcnt_1].G.valid);
      sample[tagcnt].counter = sample_counter;
      break;
    }

    case 0x0E: // Sensor Hub Slave 0 Virtual Sensor hub data from slave 0
    case 0x0F: // Sensor Hub Slave 1 Virtual Sensor hub data from slave 1
    case 0x10: // Sensor Hub Slave 2 Virtual Sensor hub data from slave 2
    case 0x11: // Sensor Hub Slave 3 Virtual Sensor hub data from slave 3
    case 0x12: // Step Counter Virtual Step counter data
    case 0x19: // Sensor Hub Nack Virtual Sensor hub nack from slave 0/1/2/3
      return DecodeTagResult::TAG_NOT_IMPLEMENTED;

    default:
      // Unknown tag.
      // This is probably a communication problem. Note that the parity
      // check has passed at this point, so this should basically not
      // occur.
      // The problem is, probably another tag was meant to be sent. It
      // could be any of the tags above, so missing that tag MAY hurt
      // XL or G data, at various tagcnts.
      // If no compression is enabled, sample data for TAGCNT+0/1 should
      // be invalidated. If compression is enabled, sample data for
      // TAGCNT-1/-2 should also be invalidated.
      invalidateSample(0);
      invalidateSample(1);
      if(compressionEnabled) {
        invalidateSample(3); // -1 MOD 4 = 3
        invalidateSample(2); // -2 MOD 4 = 2
      }
      return DecodeTagResult::UNKNOWN_TAG;
  }

  return DecodeTagResult::OK;
}

inline void LSM6DSOXFIFOClass::extend5bits(uint8_t hi, uint8_t lo, 
                                           int16_t &delta_x, int16_t &delta_y, int16_t &delta_z)
{
  delta_x = signextend<5>(lo & 0x1F);
  delta_y = signextend<5>(((hi & 0x03) << 3) | (lo >> 5));
  delta_z = signextend<5>((hi & 0x7C) >> 2);
}

void LSM6DSOXFIFOClass::setSampleData(SampleData &s, 
                                      int16_t X, 
                                      int16_t Y, 
                                      int16_t Z, 
                                      uint16_t fullRange,
                                      bool valid)
{
  s.rawXYZ[X_IDX] = X;
  s.rawXYZ[Y_IDX] = Y;
  s.rawXYZ[Z_IDX] = Z;
  s.fullRange = fullRange;

  s.valid = valid;
  if(valid) {
    float scale = fullRange * (1.0 / 32768);
    s.XYZ[X_IDX] = X * scale;
    s.XYZ[Y_IDX] = Y * scale;
    s.XYZ[Z_IDX] = Z * scale;
  } else {
    // Data is invalid (or undefined)
    s.XYZ[X_IDX] = NAN;
    s.XYZ[Y_IDX] = NAN;
    s.XYZ[Z_IDX] = NAN;
  }
}

void LSM6DSOXFIFOClass::initializeSample(uint8_t idx, bool setStatusInvalid)
{
  // timestamp and temperature are not always sent
  sample[idx].timestamp = NAN;
  sample[idx].temperature = NAN;

  // Set counter and full scale to 'impossible' values
  // to help identifying errors
  sample[idx].counter = counter_uninitialized;

  // If compression is disabled, XL and G data may be
  // set to remarkable values in order to signal errors.
  // If compression is used, older raw values may be
  // used as reference for increments.
  if(!compressionEnabled) {
    int16_t default_value = 0x7FFF;

    bool valid_XL = sample[idx].XL.valid & !setStatusInvalid;
    setSampleData(sample[idx].XL, default_value, default_value, default_value, 0, valid_XL);
    sample[idx].XL.XYZ[X_IDX] = NAN;
    sample[idx].XL.XYZ[Y_IDX] = NAN;
    sample[idx].XL.XYZ[Z_IDX] = NAN;

    bool valid_G = sample[idx].G.valid & !setStatusInvalid;
    setSampleData(sample[idx].G, default_value, default_value, default_value, 0, valid_G);
    sample[idx].G.XYZ[X_IDX] = NAN;
    sample[idx].G.XYZ[Y_IDX] = NAN;
    sample[idx].G.XYZ[Z_IDX] = NAN;
  }
}

void LSM6DSOXFIFOClass::invalidateSample(uint8_t idx)
{
  setSampleData(sample[idx].XL,
                sample[idx].XL.rawXYZ[X_IDX],
                sample[idx].XL.rawXYZ[Y_IDX], 
                sample[idx].XL.rawXYZ[Z_IDX], 
                sample[idx].XL.fullRange, false);
  setSampleData(sample[idx].G,
                sample[idx].G.rawXYZ[X_IDX],
                sample[idx].G.rawXYZ[Y_IDX], 
                sample[idx].G.rawXYZ[Z_IDX], 
                sample[idx].G.fullRange, false);
}

/*/ For debugging purposes
void LSM6DSOXFIFOClass::displaySamples()
{
  Serial.println("---");
  for(uint8_t idx = 0; idx < TAGCNT_BUFFER_SIZE; idx++) {
    Serial.print("["+String(idx)+"] cnt= " + String(sample[idx].counter) + " t="+String(sample[idx].timestamp)+" T="+String(sample[idx].temperature));
    Serial.print(" G=("+String(sample[idx].G_X)+", " + String(sample[idx].G_Y) + ", "+String(sample[idx].G_Z) + ") {FS="+String(sample[idx].fullRange_G)+"}");
    Serial.println(" XL=("+String(sample[idx].XL_X)+", " + String(sample[idx].XL_Y) + ", "+String(sample[idx].XL_Z) + ") {FS="+String(sample[idx].fullRange_XL)+"}");
  }
  Serial.println("---");
} //*/
