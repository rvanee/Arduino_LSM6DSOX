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

#include <Arduino.h>

#include "LSM6DSOXFIFO.h"
#include "LSM6DSOX.h"
#include "LSM6DSOXTables.h"
#include "LSM6DSOXAutoRanger.h"

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


LSM6DSOXFIFOClass::LSM6DSOXFIFOClass(LSM6DSOXClass* imu) :
  imu(imu),
  autoRanger_XL(LSM6DSOXAutoRanger(LSM6DSOXTables::FR_XL_bits)),
  autoRanger_G (LSM6DSOXAutoRanger(LSM6DSOXTables::FR_G_bits ))
{
  compression_enabled = false;
  use_MCU_timestamp = false;

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
  
  // Compression
  bool      compression,              // true = enable compression
  uint8_t   uncompressed_decimation,  // Uncompressed data every 0/8/16/32 batch data

  // Autorange
  bool      autorange,                // Enables XL and G autorange feature
  uint16_t  threshold_up,             // Threshold for 'gearing up'
  uint16_t  threshold_down,           // Threshold for 'gearing down'

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
  
  // Compression
  settings.compression = compression;
  settings.uncompressed_decimation = uncompressed_decimation;

  // Autorange
  settings.autorange = autorange;     // Enables XL and G autorange feature
  settings.threshold_up = threshold_up;       // Threshold for 'gearing up'
  settings.threshold_down = threshold_down;   // Threshold for 'gearing down'
   
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
  // Turn off FIFO. This will also empty its fifo queue
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

  // Sample buffer management
  sample_counter = 0; // Actual sample number. Lowest 2 bits resemble TAGCNT
  read_counter = 0;   // First finished sample number that is not yet read
  // Initialize circular sample buffer. Set G and XL data to invalid
  for(uint8_t idx = 0; idx < SAMPLE_BUFFER_SIZE; idx++) {
    initializeSample(idx, true);
  } 

  // Decoder
  compression_enabled = settings.compression;
    // Timestamp reconstruction is only possible and relevant if
    // timestamp decimation is > 0 and > 1, respectively
  timestamp_reconstruction_enabled = (settings.timestamp_decimation > 1);
  timestamp64 = 0;
  timestamp64_prev = 0;
  timestamp64_counter = 0;
  dt_per_sample = FIFO_INT16_NAN;

  // If IMU fifo timestamping is disabled, use time stamp estimation
  // using the MCU's micros() clock
  use_MCU_timestamp = (settings.timestamp_decimation == 0);
  if(use_MCU_timestamp) {
    // Initialize MCU timestamp estimator
    MCU_timestamp_estimator.reset();
  }

  // Initialize AutoRangers
  autoRanger_XL.reset(settings.threshold_up, settings.threshold_down);
  autoRanger_G.reset(settings.threshold_up, settings.threshold_down);
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
    uint8_t fifo_status2 = status_registers[1];
    status.DIFF_FIFO = ((uint16_t)(fifo_status2 & MASK_DIFF_FIFO_H) << 8) | status_registers[0];
    status.FIFO_OVR_LATCHED = fifo_status2 & MASK_FIFO_OVR_LATCHED;
    status.COUNTER_BDR_IA   = fifo_status2 & MASK_COUNTER_BDR_IA;
    status.FIFO_FULL_IA     = fifo_status2 & MASK_FIFO_FULL_IA;
    status.FIFO_OVR_IA      = fifo_status2 & MASK_FIFO_OVR_IA;
    status.FIFO_WTM_IA      = fifo_status2 & MASK_FIFO_WTM_IA;
  }
  return result;
}

ReadResult LSM6DSOXFIFOClass::fillQueue()
{
  uint16_t to_read;
  uint8_t status_registers[2];
  int result = imu->readRegisters(LSM6DSOX_STATUS1, status_registers, 2);
    
  // If no IMU timestamps are to be read, MCU timestamp should be used.
  // Note that this code should be placed as close as possible to the 
  // read operation above in order to minimize delay between addition
  // of the last word to the fifo, right before the status registers
  // are read, and obtaining the MCU's microsecond-precision clock.
  unsigned long MCU_micros = use_MCU_timestamp ? 
                              micros() :
                              0; // Initialize to prevent warning
  // Keep current sample counter, to be used for timestamping
  uint32_t current_counter = sample_counter;
  // ... and for autoranging
  uint32_t autorange_counter = sample_counter;

  // Now process the status data
  if(result != 1) return ReadResult::READ_ERROR;
  uint8_t fifo_status2 = status_registers[1];
  // Find number of words currently in fifo
  to_read = ((uint16_t)(fifo_status2 & MASK_DIFF_FIFO_H) << 8) | status_registers[0];
  // Check if new data available
  if(to_read == 0) return ReadResult::NO_DATA_AVAILABLE;
  // Check for FIFO overflow
  if(fifo_status2 & MASK_FIFO_OVR_IA) return ReadResult::FIFO_OVERFLOW;

  // Used in autoranging:
  // If compression is enabled, there should be a delay of 2 in
  // releasing samples to account for the compression algorithm
  // modifying data at T-2 and T-1, rather than just at time T.
  uint8_t compression_offset = compression_enabled << 1;

  // Read data from FIFO to local word buffer
  do {
    // Break down read operations into a maximum number of 'fifo words'
    uint8_t  read_buffer[READ_MAX_WORDS * BUFFER_BYTES_PER_WORD];
    uint16_t read_now = min(to_read, READ_MAX_WORDS);
    result = imu->readRegisters(LSM6DSOX_FIFO_DATA_OUT_TAG, 
                                read_buffer,
                                read_now*BUFFER_BYTES_PER_WORD);
    if(result != 1) return ReadResult::READ_ERROR;
    to_read -= read_now;

    // Process words read above
    for(uint8_t *word = read_buffer; read_now--; word += BUFFER_BYTES_PER_WORD) {
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
        // the previous 2 samples should also be invalidated.
        invalidateSample(counterToIdx(sample_counter));
        invalidateSample(counterToIdx(sample_counter+1));
        if(compression_enabled) {
          invalidateSample(counterToIdx(sample_counter-1));
          invalidateSample(counterToIdx(sample_counter-2));
        }
        // This word can't be evaluated any further, but we can go on 
        // decoding the next words
        continue;
      }

      // Update sample counter based on tagcnt
      uint8_t tagcnt = (tag_byte & MASK_FIFO_TAG_CNT) >> 1;
      uint8_t prev_tagcnt = sample_counter & SAMPLE_TAGCNT_MASK;
      int8_t delta_tagcnt = static_cast<int8_t>(tagcnt) - static_cast<int8_t>(prev_tagcnt);
      if(delta_tagcnt != 0) {
        // Add delta tagcnt and rollover (+4) if tagcnt < prev_tagcnt
        sample_counter += delta_tagcnt + (delta_tagcnt & 0x04);

        // Test for sample buffer overflow. This will happen when
        // new sample data would occupy the same position as the
        // one pointed to by the read counter.
        if(sample_counter >= (read_counter + SAMPLE_BUFFER_SIZE)) {
          return ReadResult::QUEUE_FULL;
        }
      }

      // Decode word, writes to circular sample buffer
      switch(decodeWord(word)) {
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

      if(settings.autorange && (sample_counter > compression_offset)) {
        uint32_t finished_sample_counter = sample_counter - compression_offset - 1;
        for(; autorange_counter <= finished_sample_counter; autorange_counter++) {
          Sample &sample = sample_buffer[counterToIdx(autorange_counter)];

          // Check for need of emergency full range increase: overflow would lead 
          // to loss of most significant bit(s)
          uint16_t new_full_range_XL =
            autoRanger_XL.add_and_check_sample(autorange_counter, sample.XL);
          if(new_full_range_XL > 0) imu->setFullRange_XL(new_full_range_XL);

          uint16_t new_full_range_G =
            autoRanger_G.add_and_check_sample(autorange_counter, sample.G);
          if(new_full_range_G > 0) imu->setFullRange_G(new_full_range_G);
        } // END for
      } // END if(settings.autorange && (sample_counter > compression_offset))
    } // END for(uint8_t *word = &read_buffer; ...
  } while(to_read > 0);

  if(settings.autorange) {
    // Now check if the XL and G full ranges may be scaled
    // down, in order to gain accuracy in the least significant
    // bits of the XL/G raw X/Y/Z values
    uint16_t new_full_range_XL = autoRanger_XL.check_underflow();
    if(new_full_range_XL > 0) imu->setFullRange_XL(new_full_range_XL);

    uint16_t new_full_range_G = autoRanger_G.check_underflow();
    if(new_full_range_G > 0) imu->setFullRange_G(new_full_range_G);
  }

  // IMU timestamp reconstruction enabled and possible?
  // (It won't be possible before at least two timestamp/counter
  // combinations are available.)
  if((timestamp_reconstruction_enabled) && 
     (dt_per_sample != FIFO_INT16_NAN)) {
    // Find delta samples between the 'current' sample and the sample
    // with the most recent timestamp64 data. Note that this number
    // will be in range [-7, 7] or [-31, 31], depending on 
    // settings.timestamp_decimation.
    // Then use linear inter/extrapolation of the current samples
    // timestamp using the most recent (64 bit) timestamp and its
    // corresponding sample counter.  
    uint64_t t = timestamp64;
    if(current_counter > timestamp64_counter) {
      uint8_t delta_samples = current_counter - timestamp64_counter;
      t += delta_samples*(uint32_t)dt_per_sample;
    } else {
      uint8_t delta_samples = timestamp64_counter - current_counter;
      t -= delta_samples*(uint32_t)dt_per_sample;
    }
    // Now calculate timestamps for all new samples read and decoded
    // above, including the current sample. That one may still receive
    // timing information later, but in that case the timestamp will
    // simply be overwritten; otherwise it may never be written.
    for(; current_counter <= sample_counter; current_counter++) {
      // The current sample's timestamp is in timestamp units,
      // i.e. approximate 25us 'clicks'. It is converted into
      // microseconds and corrected using a clock multiplier, then
      // stored in the sample buffer.
      sample_buffer[counterToIdx(current_counter)].timestamp =
        imu->correctTimestamp(t);
      // Advance t one sample, so add dt_per_sample
      t += dt_per_sample;
    }
  }

  // If MCU timestamps are to be used, they should be correlated with
  // the last sample retrieved, since that will be closest to the MCU
  // micros() found above. This corresponds to the sample with the
  // most recent sample counter.
  if(use_MCU_timestamp) {
    MCU_timestamp_estimator.add(sample_counter, MCU_micros);

    // Now calculate timestamps for all new samples read and decoded
    // above, including the current sample. That one may be revisited
    // later, but in that case the timestamp will simply be overwritten
    // with more recent timing information; otherwise it may never be
    // written.
    for(; current_counter <= sample_counter; current_counter++) {
      // Calculate timestamp estimate in microseconds, then store it
      // in the current sample in the sample buffer
      sample_buffer[counterToIdx(current_counter)].timestamp =
        MCU_timestamp_estimator.estimate(current_counter);
    }
  }

  return ReadResult::DATA_READ;
}

bool LSM6DSOXFIFOClass::retrieveSample(Sample& sample)
{
  // If compression is enabled, there should be a delay of 2 in
  // releasing samples to account for the compression algorithm
  // modifying data at T-2 and T-1, rather than just at time T.
  uint8_t compression_offset = compression_enabled << 1;
  // Check if a sample is available
  if((read_counter + compression_offset) < sample_counter) {
    // Copy sample from the circular sample buffer
    uint8_t idx = counterToIdx(read_counter);
    sample = sample_buffer[idx];
    read_counter++;
    // Set released buffer sample to some 'invalid' values.
    // This may help debugging problems in decoding logic,
    // or in MCU<->IMU communication.
    // NOTE that the raw XL and G values and the 'valid' flag
    // will not be overwritten if compression is enabled,
    // since this data may still be needed (in the case of 3xC
    // decoding below).
    initializeSample(idx, false);
    return true; // Sample retrieved
  }
  return false; // No sample retrieved (buffer empty)
}

DecodeTagResult LSM6DSOXFIFOClass::decodeWord(uint8_t *word)
{
  // Note: this function updates fullRange_G, fullRange_XL and compression_enabled,
  // as well as the sample circular buffer
  
  // Sample counters
  uint8_t current   = counterToIdx(sample_counter);   // T
  uint8_t current_1 = counterToIdx(sample_counter-1); // T-1
  uint8_t current_2 = counterToIdx(sample_counter-2); // T-2
  uint8_t current_3 = counterToIdx(sample_counter-3); // T-3

  // Convert G data from deg/s to rad/s?
  bool rad_G = imu->settings.rad_G;

  // Decode tag
  uint8_t tag = word[FIFO_DATA_OUT_TAG] >> 3;
  switch(tag) {
    case 0x01: // Gyroscope NC Main Gyroscope uncompressed data
    {
      setLSM6DSOXSampleData(&sample_buffer[current].G,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_G, true, rad_G);
      sample_buffer[current].counter = sample_counter;
      break;
    }

    case 0x02: // Accelerometer NC Main Accelerometer uncompressed data
    {
      setLSM6DSOXSampleData(&sample_buffer[current].XL,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_XL, true);
      sample_buffer[current].counter = sample_counter;
      break;
    }

    case 0x03: // Temperature Auxiliary Temperature data
    {
      sample_buffer[current].temperature = 
        imu->temperatureIntToCelsius(bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]));
      break;
    }

    case 0x04: // Timestamp Auxiliary Timestamp data
    {
      uint32_t timestamp = 
        (((uint32_t)word[FIFO_DATA_OUT_Y_H]) << 24) | 
        (((uint32_t)word[FIFO_DATA_OUT_Y_L]) << 16) |
        (((uint16_t)word[FIFO_DATA_OUT_X_H]) <<  8) |
         (          word[FIFO_DATA_OUT_X_L]);
      
      // For timestamp reconstruction:
      // store old timestamp64 value
      timestamp64_prev = timestamp64;

      // Test for timestamp overrun, i.e. from large 32-bit value in previous step
      // to value close to 0
      if(static_cast<uint32_t>(timestamp64) > timestamp) {
        timestamp64 += (1ULL << 32); // Add one to highest 32 bits
      }
      // Now replace lower 32 bits of timestamp64 with new 32-bit timestamp value
      timestamp64 = (timestamp64 & 0xFFFFFFFF00000000) | timestamp;

      // Store (corrected) timestamp in current sample
      sample_buffer[current].timestamp = imu->correctTimestamp(timestamp64);

      // Calculate delta timestamp per sample (used for timestamp reconstruction).
      // Note that this should always be an integer value, because the IMU
      // uses a base clock of ~25us and all ODR time intervals are integer
      // multiples of this.
      uint8_t delta_counter = sample_counter - timestamp64_counter;
      dt_per_sample = (delta_counter == 0) ?
                      FIFO_INT16_NAN :
                      uint16_t(timestamp64 - timestamp64_prev) / delta_counter;

      // store current sample counter as last timestamp counter
      timestamp64_counter = sample_counter;
      break;
    }

    case 0x05: // CFG_Change Auxiliary Meta-information data
    {
      // Accelerometer full range
      uint8_t fs_xl = word[FIFO_DATA_OUT_Y_L] >> 6; // FS1_XL FS0_XL
      uint16_t fr_XL = LSM6DSOXTables::getFloatFromBits(fs_xl, LSM6DSOXTables::FR_XL_bits);
      sample_buffer[current].XL.fullRange = fr_XL;
      if(settings.autorange && (imu->fullRange_XL != fr_XL)) {
        autoRanger_XL.notify_new_full_range(sample_counter, fr_XL);
      }
      imu->fullRange_XL = fr_XL;

      // Gyro full range
      uint8_t fs_g = word[FIFO_DATA_OUT_X_H] >> 5; // FS1_G FS0_G FS_125
      uint16_t fr_G = LSM6DSOXTables::getFloatFromBits(fs_g, LSM6DSOXTables::FR_G_bits);
      sample_buffer[current].G.fullRange = fr_G;
      if(settings.autorange && (imu->fullRange_G != fr_G)) {
        autoRanger_G.notify_new_full_range(sample_counter, fr_G);
      }
      imu->fullRange_G = fr_G;

      // Compression
      compression_enabled = word[FIFO_DATA_OUT_Y_H] & 0x80;
      break;
    }

    case 0x06: // Accelerometer NC_T_2 Main Accelerometer uncompressed batched at two times the previous time slot
    {
      setLSM6DSOXSampleData(&sample_buffer[current_2].XL,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_XL, true);
      sample_buffer[current_2].counter = sample_counter-2;
      break;
    }

    case 0x07: // Accelerometer NC_T_1 Main Accelerometer uncompressed data batched at the previous time slot
    {
      setLSM6DSOXSampleData(&sample_buffer[current_1].XL,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_XL, true);
      sample_buffer[current_1].counter = sample_counter-1;
      break;
    }

    case 0x08: // Accelerometer 2xC Main Accelerometer 2x compressed data
    {
      int16_t XL_X_2 = sample_buffer[current_3].XL.rawXYZ[X_IDX] + signextend<8>(word[FIFO_DATA_OUT_X_L]);
      int16_t XL_Y_2 = sample_buffer[current_3].XL.rawXYZ[Y_IDX] + signextend<8>(word[FIFO_DATA_OUT_X_H]);
      int16_t XL_Z_2 = sample_buffer[current_3].XL.rawXYZ[Z_IDX] + signextend<8>(word[FIFO_DATA_OUT_Y_L]);
      // Note that TAGCNT-2 sample's validity depends on the old TAGCNT-3 sample's validity
      setLSM6DSOXSampleData(&sample_buffer[current_2].XL, XL_X_2, XL_Y_2, XL_Z_2, imu->fullRange_XL, sample_buffer[current_3].XL.valid);
      sample_buffer[current_2].counter = sample_counter-2;

      int16_t XL_X_1 = XL_X_2 + signextend<8>(word[FIFO_DATA_OUT_Y_H]);
      int16_t XL_Y_1 = XL_Y_2 + signextend<8>(word[FIFO_DATA_OUT_Z_L]);
      int16_t XL_Z_1 = XL_Z_2 + signextend<8>(word[FIFO_DATA_OUT_Z_H]);
      // Note that TAGCNT-1 sample's validity depends on TAGCNT-2 sample's validity
      setLSM6DSOXSampleData(&sample_buffer[current_1].XL, XL_X_1, XL_Y_1, XL_Z_1, imu->fullRange_XL, sample_buffer[current_2].XL.valid);
      sample_buffer[current_1].counter = sample_counter-1;
      break;
    }

    case 0x09: // Accelerometer 3xC Main Accelerometer 3x compressed data
    {
      int16_t dx, dy, dz;

      extend5bits(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L], dx, dy, dz);
      int16_t XL_X_2 = sample_buffer[current_3].XL.rawXYZ[X_IDX] + dx;
      int16_t XL_Y_2 = sample_buffer[current_3].XL.rawXYZ[Y_IDX] + dy;
      int16_t XL_Z_2 = sample_buffer[current_3].XL.rawXYZ[Z_IDX] + dz;
      // Note that TAGCNT-2 sample's validity depends on the old TAGCNT-3 sample's validity
      setLSM6DSOXSampleData(&sample_buffer[current_2].XL, XL_X_2, XL_Y_2, XL_Z_2, imu->fullRange_XL, sample_buffer[current_3].XL.valid);
      sample_buffer[current_2].counter = sample_counter-2;

      extend5bits(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L], dx, dy, dz);
      int16_t XL_X_1 = XL_X_2 + dx;
      int16_t XL_Y_1 = XL_Y_2 + dy;
      int16_t XL_Z_1 = XL_Z_2 + dz;
      // Note that TAGCNT-1 sample's validity depends on TAGCNT-2 sample's validity
      setLSM6DSOXSampleData(&sample_buffer[current_1].XL, XL_X_1, XL_Y_1, XL_Z_1, imu->fullRange_XL, sample_buffer[current_2].XL.valid);
      sample_buffer[current_1].counter = sample_counter-1;

      extend5bits(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L], dx, dy, dz);
      int16_t XL_X = XL_X_1 + dx;
      int16_t XL_Y = XL_Y_1 + dy;
      int16_t XL_Z = XL_Z_1 + dz;
      // Note that TAGCNT sample's validity depends on TAGCNT-1 sample's validity
      setLSM6DSOXSampleData(&sample_buffer[current].XL, XL_X, XL_Y, XL_Z, imu->fullRange_XL, sample_buffer[current_1].XL.valid);
      sample_buffer[current].counter = sample_counter;
      break;
    }

    case 0x0A: // Gyroscope NC_T_2 Main Gyroscope uncompressed data batched at two times the previous time slot
    {
      setLSM6DSOXSampleData(&sample_buffer[current_2].G,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_G, true, rad_G);
      sample_buffer[current_2].counter = sample_counter-2; // Necessary if XL is not recorded in FIFO
      break;    
    }

    case 0x0B: // Gyroscope NC_T_1 Main Gyroscope uncompressed data batched at the previous time slot
    {
      setLSM6DSOXSampleData(&sample_buffer[current_1].G,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_G, true, rad_G);
      sample_buffer[current_1].counter = sample_counter-1; // Necessary if XL is not recorded in FIFO
      break;
    }

    case 0x0C: // Gyroscope 2xC Main Gyroscope 2x compressed data
    {
      int16_t G_X_2 = sample_buffer[current_3].G.rawXYZ[X_IDX] + signextend<8>(word[FIFO_DATA_OUT_X_L]);
      int16_t G_Y_2 = sample_buffer[current_3].G.rawXYZ[Y_IDX] + signextend<8>(word[FIFO_DATA_OUT_X_H]);
      int16_t G_Z_2 = sample_buffer[current_3].G.rawXYZ[Z_IDX] + signextend<8>(word[FIFO_DATA_OUT_Y_L]);
      // Note that TAGCNT-2 sample's validity depends on the old TAGCNT-3 sample's validity
      setLSM6DSOXSampleData(&sample_buffer[current_2].G, G_X_2, G_Y_2, G_Z_2, 
                    imu->fullRange_G, sample_buffer[current_3].G.valid, rad_G);
      sample_buffer[current_2].counter = sample_counter-2;  // Necessary if XL is not recorded in FIFO

      int16_t G_X_1 = G_X_2 + signextend<8>(word[FIFO_DATA_OUT_Y_H]);
      int16_t G_Y_1 = G_Y_2 + signextend<8>(word[FIFO_DATA_OUT_Z_L]);
      int16_t G_Z_1 = G_Z_2 + signextend<8>(word[FIFO_DATA_OUT_Z_H]);
      // Note that TAGCNT-1 sample's validity depends on TAGCNT-2 sample's validity
      setLSM6DSOXSampleData(&sample_buffer[current_1].G, G_X_1, G_Y_1, G_Z_1,
                    imu->fullRange_G, sample_buffer[current_2].G.valid, rad_G);
      sample_buffer[current_1].counter = sample_counter-1;  // Necessary if XL is not recorded in FIFO
      break;
    }

    case 0x0D: // Gyroscope 3xC Main Gyroscope 3x compressed data
    {
      int16_t dx, dy, dz;

      extend5bits(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L], dx, dy, dz);
      int16_t G_X_2 = sample_buffer[current_3].G.rawXYZ[X_IDX] + dx;
      int16_t G_Y_2 = sample_buffer[current_3].G.rawXYZ[Y_IDX] + dy;
      int16_t G_Z_2 = sample_buffer[current_3].G.rawXYZ[Z_IDX] + dz;
      // Note that TAGCNT-2 sample's validity depends on the old TAGCNT-3 sample's validity
      setLSM6DSOXSampleData(&sample_buffer[current_2].G, G_X_2, G_Y_2, G_Z_2, imu->fullRange_G, sample_buffer[current_3].G.valid, rad_G);
      sample_buffer[current_2].counter = sample_counter-2; // Necessary if XL is not recorded in FIFO

      extend5bits(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L], dx, dy, dz);
      int16_t G_X_1 = G_X_2 + dx;
      int16_t G_Y_1 = G_Y_2 + dy;
      int16_t G_Z_1 = G_Z_2 + dz;
      // Note that TAGCNT-1 sample's validity depends on TAGCNT-2 sample's validity
      setLSM6DSOXSampleData(&sample_buffer[current_1].G, G_X_1, G_Y_1, G_Z_1, 
                    imu->fullRange_G, sample_buffer[current_2].G.valid, rad_G);
      sample_buffer[current_1].counter = sample_counter-1;

      extend5bits(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L], dx, dy, dz);
      int16_t G_X = G_X_1 + dx;
      int16_t G_Y = G_Y_1 + dy;
      int16_t G_Z = G_Z_1 + dz;
      // Note that TAGCNT sample's validity depends on TAGCNT-1 sample's validity
      setLSM6DSOXSampleData(&sample_buffer[current].G, G_X, G_Y, G_Z,
                    imu->fullRange_G, sample_buffer[current_1].G.valid, rad_G);
      sample_buffer[current].counter = sample_counter;
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
      // If no compression is enabled, sample data for the current sample
      // should be invalidated. If compression is enabled, sample data for
      // current sample -1/-2 should also be invalidated.
      invalidateSample(current);
      if(compression_enabled) {
        invalidateSample(current_1);
        invalidateSample(current_2);
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

inline int32_t LSM6DSOXFIFOClass::raw2fixedrad(int16_t raw, uint16_t fullRange)
{
  /* This function converts raw, unscaled signed 16-bit deg/s gyroscope
     data to signed 16.16 fixed point rad/s, by multiplication with
     fullRange (2000/1000/500/250/125) and with pi/180.
     It first calculates raw * 2000*pi/180, then uses the lower 4 bits
     of fullRange to scale the result back in case of smaller fullRange.
  */
 
  /* Gyroscope full range to right shift positions.
     Full range's lower 4 bits determine right shift.
     All other combinations are invalid and shift all data out. */
  static const uint8_t shift_lut[16] = {
     8, // 0b0000 2000 = 7D0
    32, // 0b0001
    32, // 0b0010
    32, // 0b0011
    10, // 0b0100  500 = 1F4
    32, // 0b0101
    32, // 0b0110
    32, // 0b0111
     9, // 0b1000 1000 = 3E8
    32, // 0b1001
    11, // 0b1010  250 = 0FA
    32, // 0b1011
    32, // 0b1100
    12, // 0b1101  125 = 07D
    32, // 0b1110
    32, // 0b1111
  };

  // The basic concept here is to scale the multiplicand such that it
  // contains as many significant bits as possible, while preventing
  // overflow. The resulting signed 16 x unsigned 32 bit multiplication
  // is split into two signed 16x16->32 bit multiplications, whose
  // results are added after proper alignment. Then scaling for fullRange
  // is combined with postprocessing to a signed 16.16 fixed point number.
  // int(2000*pi/180 * 2^25) = 1171270634 = 17872*65536 + 11242
  int32_t rad_hi = 17872L * raw;
  int32_t rad_lo = (11242L * raw) >> 16;
  int32_t rad = (rad_hi + rad_lo) >> shift_lut[(uint8_t)fullRange & 0x0F];
 
  return rad;
}

void LSM6DSOXFIFOClass::setLSM6DSOXSampleData(LSM6DSOXSampleData *s, 
                                      int16_t X, 
                                      int16_t Y, 
                                      int16_t Z, 
                                      uint16_t fullRange,
                                      bool valid,
                                      bool to_rad)
{
  s->rawXYZ[X_IDX] = X;
  s->rawXYZ[Y_IDX] = Y;
  s->rawXYZ[Z_IDX] = Z;
  s->fullRange = fullRange;

  s->valid = valid;
  if(valid) {
    if(to_rad) {
      // Convert from 16 bit signed deg/s to signed 16.16
      // bit fixed point numbers in rad/s, by multiplication
      // with 2 * fullRange * pi/180
      // (raw data is in range [-32768, 32767]).
      s->XYZ[X_IDX] = raw2fixedrad(X, fullRange);
      s->XYZ[Y_IDX] = raw2fixedrad(Y, fullRange);
      s->XYZ[Z_IDX] = raw2fixedrad(Z, fullRange);
    } else {
      // Transform into signed 16.16 fixed point numbers,
      // by multiplication with 2*fullRange
      // (raw data is in range [-32768, 32767]).
      int16_t doubleRange = (int16_t)fullRange << 1;
      s->XYZ[X_IDX] = (int32_t)X * doubleRange;
      s->XYZ[Y_IDX] = (int32_t)Y * doubleRange;
      s->XYZ[Z_IDX] = (int32_t)Z * doubleRange;
    }
  } else {
    // Data is invalid (or undefined)
    s->XYZ[X_IDX] = FIFO_FIXED_POINT_NAN;
    s->XYZ[Y_IDX] = FIFO_FIXED_POINT_NAN;
    s->XYZ[Z_IDX] = FIFO_FIXED_POINT_NAN;
  }
}

void LSM6DSOXFIFOClass::initializeSample(uint8_t idx, bool setStatusInvalid)
{
  // timestamp and temperature are not always se(n)t
  sample_buffer[idx].timestamp = 0;
  sample_buffer[idx].temperature = NAN;

  // Set counter to 'impossible' value to help identify errors
  sample_buffer[idx].counter = 0;

  // If compression is disabled, XL and G data may be
  // set to remarkable values in order to signal errors.
  // If compression is used, older raw values may be
  // used as reference for increments.
  LSM6DSOXSampleData *XL = &sample_buffer[idx].XL;
  LSM6DSOXSampleData *G  = &sample_buffer[idx].G;
  if(!compression_enabled) {
    bool valid_XL = XL->valid & !setStatusInvalid;
    setLSM6DSOXSampleData(XL,
                  FIFO_INT16_NAN, FIFO_INT16_NAN, FIFO_INT16_NAN,
                  0, valid_XL);

    bool valid_G = G->valid & !setStatusInvalid;
    setLSM6DSOXSampleData(G, 
                  FIFO_INT16_NAN, FIFO_INT16_NAN, FIFO_INT16_NAN,
                  0, valid_G);
  }

  // XL and G 16.16 bit fixed point values may be set to Not A Number
  XL->XYZ[X_IDX] = FIFO_FIXED_POINT_NAN;
  XL->XYZ[Y_IDX] = FIFO_FIXED_POINT_NAN;
  XL->XYZ[Z_IDX] = FIFO_FIXED_POINT_NAN;

  G->XYZ[X_IDX] = FIFO_FIXED_POINT_NAN;
  G->XYZ[Y_IDX] = FIFO_FIXED_POINT_NAN;
  G->XYZ[Z_IDX] = FIFO_FIXED_POINT_NAN;
}

void LSM6DSOXFIFOClass::invalidateSample(uint8_t idx)
{
  LSM6DSOXSampleData *XL = &sample_buffer[idx].XL;
  setLSM6DSOXSampleData(XL,
                XL->rawXYZ[X_IDX],
                XL->rawXYZ[Y_IDX], 
                XL->rawXYZ[Z_IDX], 
                XL->fullRange, false);
  LSM6DSOXSampleData *G = &sample_buffer[idx].G;
  setLSM6DSOXSampleData(G,
                G->rawXYZ[X_IDX],
                G->rawXYZ[Y_IDX], 
                G->rawXYZ[Z_IDX], 
                G->fullRange, false);
}