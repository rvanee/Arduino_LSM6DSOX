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

// I2C buffer size is limited to 32 bytes, see link below.
// https://reference.arduino.cc/reference/en/language/functions/communication/wire/
#define I2C_BUFFER_LENGTH           32
#define READ_MAX_WORDS              (I2C_BUFFER_LENGTH / BUFFER_BYTES_PER_WORD)

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

  // Initialize circular tagcnt buffer
  for(uint8_t idx = 0; idx < TAGCNT_BUFFER_SIZE; idx++) {
    initializeSample(idx);
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

// Read as much data as possible in one multiple byte/word read from sensor fifo
// to our own buffer
int LSM6DSOXFIFOClass::readData(uint16_t& words_read, bool& too_full, FIFOStatus& status)
{
  words_read = 0;
  too_full = false;

  int result = readStatus(status);
  if(result == 1) {
    // The I2C/SPI multibyte read requires contiguous memory. Therefore fifo reading 
    // operations can not run past the end of the buffer. They can also not run
    // up to the current read pointer, in order to prevent data overrun.
    uint16_t to_read = status.DIFF_FIFO;
    if(read_idx > write_idx) {
      uint16_t readable = read_idx - write_idx;
      if(to_read > readable) {
        to_read = readable;
        too_full = true;
      }
    // Check special case where read and write pointer coincide: the buffer
    // is either empty or completely full
    } else if((read_idx == write_idx) && !buffer_empty) {
        to_read = 0;
        too_full = true;
    } else { // Now read_idx < write_idx, so we can write up all the way to buffer end
      uint16_t to_end = BUFFER_WORDS - write_idx;
      if(to_read > to_end) to_read = to_end;
    }
    if(to_read == 0) return 2; // No data read, but other reason than communication problem (<= 0)

    // Break down read operations into a maximum number of words
    while(to_read > 0) {
      uint16_t read_now = min(to_read, READ_MAX_WORDS);

      result = imu->readRegisters(LSM6DSOX_FIFO_DATA_OUT_TAG, buffer_pointer(write_idx), read_now*BUFFER_BYTES_PER_WORD);
      if(result != 1) return result;

      words_read += read_now;
      to_read -= read_now;
      if((write_idx += read_now) >= BUFFER_WORDS) write_idx -= BUFFER_WORDS; // Wrap around to buffer start
      buffer_empty = false;
    } // END while(to_read > 0)
  }
  return result;
}

SampleStatus LSM6DSOXFIFOClass::getSample(Sample& sample)
{
  // First process words in the local buffer, return as soon as a sample
  // can be released (lazy approach).
  // If the local buffer runs empty, fill it from the fifo, then
  // again decode words until a sample may be released.
  uint16_t words_read = 0;
  do {
    // Process all words available in the local buffer,
    // until a sample is produced or no more words are
    // available in the local buffer.
    while(!buffer_empty) {
      // Inspect word at read idx pointer
      SampleStatus inspectStatus = inspectWord(read_idx);
      switch(inspectStatus) {
        case SampleStatus::OK:
          // Nothing wrong, continue below
          break;

        case SampleStatus::PARITY_ERROR: // Parity error -> communication problem?
        default:
          return inspectStatus;
      }

      // Extract sample if available
      int samples_released = releaseSample(read_idx, sample);
      if(samples_released > 0) {
        // Ready: sample released, no errors
        return SampleStatus::OK;
      }
      
      // Decode word, then update read pointer and buffer empty flag
      SampleStatus decodeStatus = decodeWord(read_idx);
      if(++read_idx >= BUFFER_WORDS) read_idx -= BUFFER_WORDS;
      buffer_empty = (read_idx == write_idx);
      switch(decodeStatus) {
        case SampleStatus::OK:
          // Continue below, i.e. enter next !buffer_empty loop iteration
          break;

        case SampleStatus::TAG_NOT_IMPLEMENTED:
        case SampleStatus::UNKNOWN_TAG:
        default:
          return decodeStatus;
      }
    } // END while(!buffer_empty)

    // If no sample was produced, read a fresh batch of
    // words from the IMU to the local buffer. Then resume
    // processing them, again until a sample is produced.
    bool too_full = false;
    // Read block of data. Note that too_full will always be false,
    // since the buffer was emptied above.
    FIFOStatus status;
    int read_result = readData(words_read, too_full, status);
    // If an error occurred (result <= 0), report communication error
    if(read_result <= 0) {
      return SampleStatus::COMMUNICATION_ERROR;
    }
    // Buffer overrun qualifies as an error too
    if(status.FIFO_OVR_LATCHED) {
      return SampleStatus::BUFFER_OVERRUN;
    }
  } while (words_read > 0);

  // No words were read (so fifo is empty, 'underrun')
  return SampleStatus::BUFFER_UNDERRUN;
}

SampleStatus LSM6DSOXFIFOClass::inspectWord(uint16_t idx)
{
  uint8_t *word = buffer_pointer(idx);
  
  // Perform parity check
  uint8_t parity = word[FIFO_DATA_OUT_TAG] ^ (word[FIFO_DATA_OUT_TAG] >> 4);
  parity ^= (parity >> 2);
  parity ^= (parity >> 1);
  if(parity & MASK_FIFO_TAG_PARITY) {
    return SampleStatus::PARITY_ERROR; // Parity error -> communication problem?
  }

  return SampleStatus::OK;
}

int LSM6DSOXFIFOClass::releaseSample(uint16_t idx, Sample& extracted_sample)
{
  // Note: this function updates previoustagcnt, sample_counter and initializes
  // a new sample in the circular sample buffer

  // Retrieve word from word buffer
  uint8_t *word = buffer_pointer(idx);

  // Tag counter
  uint8_t tagcnt = (word[FIFO_DATA_OUT_TAG] & MASK_FIFO_TAG_CNT) >> 1;  // T

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

  return 0; // No sample released, keep decoding
}

SampleStatus LSM6DSOXFIFOClass::decodeWord(uint16_t idx)
{
  // Note: this function updates fullRange_G, fullRange_XL and compressionEnabled,
  // as well as the sample circular buffer
  uint8_t *word = buffer_pointer(idx);

  // Tag counters
  uint8_t tagcnt = (word[FIFO_DATA_OUT_TAG] & MASK_FIFO_TAG_CNT) >> 1;  // T
  uint8_t tagcnt_1 = (tagcnt-1) & 0x03; // T-1
  uint8_t tagcnt_2 = (tagcnt-2) & 0x03; // T-2
  uint8_t tagcnt_3 = (tagcnt-3) & 0x03; // T-3

  // Decode tag
  uint8_t tag = word[FIFO_DATA_OUT_TAG] >> 3;
  switch(tag) {
    case 0x01: // Gyroscope NC Main Gyroscope uncompressed data
    {
      setSampleData(sample[tagcnt].G,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_G);
      sample[tagcnt].counter = sample_counter;
      break;
    }

    case 0x02: // Accelerometer NC Main Accelerometer uncompressed data
    {
      setSampleData(sample[tagcnt].XL,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_XL);
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
                    imu->fullRange_XL);
      sample[tagcnt_2].counter = sample_counter-2;
      break;
    }

    case 0x07: // Accelerometer NC_T_1 Main Accelerometer uncompressed data batched at the previous time slot
    {
      setSampleData(sample[tagcnt_1].XL,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_XL);
      sample[tagcnt_1].counter = sample_counter-1;
      break;
    }

    case 0x08: // Accelerometer 2xC Main Accelerometer 2x compressed data
    {
      int16_t XL_X_2 = sample[tagcnt_3].XL.rawXYZ[X_IDX] + signextend<8>(word[FIFO_DATA_OUT_X_L]);
      int16_t XL_Y_2 = sample[tagcnt_3].XL.rawXYZ[Y_IDX] + signextend<8>(word[FIFO_DATA_OUT_X_H]);
      int16_t XL_Z_2 = sample[tagcnt_3].XL.rawXYZ[Z_IDX] + signextend<8>(word[FIFO_DATA_OUT_Y_L]);
      setSampleData(sample[tagcnt_2].XL, XL_X_2, XL_Y_2, XL_Z_2, imu->fullRange_XL);
      sample[tagcnt_2].counter = sample_counter-2;

      int16_t XL_X_1 = XL_X_2 + signextend<8>(word[FIFO_DATA_OUT_Y_H]);
      int16_t XL_Y_1 = XL_Y_2 + signextend<8>(word[FIFO_DATA_OUT_Z_L]);
      int16_t XL_Z_1 = XL_Z_2 + signextend<8>(word[FIFO_DATA_OUT_Z_H]);
      setSampleData(sample[tagcnt_1].XL, XL_X_1, XL_Y_1, XL_Z_1, imu->fullRange_XL);
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
      setSampleData(sample[tagcnt_2].XL, XL_X_2, XL_Y_2, XL_Z_2, imu->fullRange_XL);
      sample[tagcnt_2].counter = sample_counter-2;

      extend5bits(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L], dx, dy, dz);
      int16_t XL_X_1 = XL_X_2 + dx;
      int16_t XL_Y_1 = XL_Y_2 + dy;
      int16_t XL_Z_1 = XL_Z_2 + dz;
      setSampleData(sample[tagcnt_1].XL, XL_X_1, XL_Y_1, XL_Z_1, imu->fullRange_XL);
      sample[tagcnt_1].counter = sample_counter-1;

      extend5bits(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L], dx, dy, dz);
      int16_t XL_X = XL_X_1 + dx;
      int16_t XL_Y = XL_Y_1 + dy;
      int16_t XL_Z = XL_Z_1 + dz;
      setSampleData(sample[tagcnt].XL, XL_X, XL_Y, XL_Z, imu->fullRange_XL);
      sample[tagcnt  ].counter = sample_counter;
      break;
    }

    case 0x0A: // Gyroscope NC_T_2 Main Gyroscope uncompressed data batched at two times the previous time slot
    {
      setSampleData(sample[tagcnt_2].G,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_G);
      sample[tagcnt_2].counter = sample_counter-2; // Necessary if XL is not recorded in FIFO
      break;    
    }

    case 0x0B: // Gyroscope NC_T_1 Main Gyroscope uncompressed data batched at the previous time slot
    {
      setSampleData(sample[tagcnt_1].G,
                    bytesToInt16(word[FIFO_DATA_OUT_X_H], word[FIFO_DATA_OUT_X_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L]),
                    bytesToInt16(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L]),
                    imu->fullRange_G);
      sample[tagcnt_1].counter = sample_counter-1; // Necessary if XL is not recorded in FIFO
      break;
    }

    case 0x0C: // Gyroscope 2xC Main Gyroscope 2x compressed data
    {
      int16_t G_X_2 = sample[tagcnt_3].G.rawXYZ[X_IDX] + signextend<8>(word[FIFO_DATA_OUT_X_L]);
      int16_t G_Y_2 = sample[tagcnt_3].G.rawXYZ[Y_IDX] + signextend<8>(word[FIFO_DATA_OUT_X_H]);
      int16_t G_Z_2 = sample[tagcnt_3].G.rawXYZ[Z_IDX] + signextend<8>(word[FIFO_DATA_OUT_Y_L]);
      setSampleData(sample[tagcnt_2].G, G_X_2, G_Y_2, G_Z_2, imu->fullRange_G);
      sample[tagcnt_2].counter = sample_counter-2;  // Necessary if XL is not recorded in FIFO

      int16_t G_X_1 = G_X_2 + signextend<8>(word[FIFO_DATA_OUT_Y_H]);
      int16_t G_Y_1 = G_Y_2 + signextend<8>(word[FIFO_DATA_OUT_Z_L]);
      int16_t G_Z_1 = G_Z_2 + signextend<8>(word[FIFO_DATA_OUT_Z_H]);
      setSampleData(sample[tagcnt_1].G, G_X_1, G_Y_1, G_Z_1, imu->fullRange_G);
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
      setSampleData(sample[tagcnt_2].G, G_X_2, G_Y_2, G_Z_2, imu->fullRange_G);
      sample[tagcnt_2].counter = sample_counter-2; // Necessary if XL is not recorded in FIFO

      extend5bits(word[FIFO_DATA_OUT_Y_H], word[FIFO_DATA_OUT_Y_L], dx, dy, dz);
      int16_t G_X_1 = G_X_2 + dx;
      int16_t G_Y_1 = G_Y_2 + dy;
      int16_t G_Z_1 = G_Z_2 + dz;
      setSampleData(sample[tagcnt_1].G, G_X_1, G_Y_1, G_Z_1, imu->fullRange_G);
      sample[tagcnt_1].counter = sample_counter-1;

      extend5bits(word[FIFO_DATA_OUT_Z_H], word[FIFO_DATA_OUT_Z_L], dx, dy, dz);
      int16_t G_X = G_X_1 + dx;
      int16_t G_Y = G_Y_1 + dy;
      int16_t G_Z = G_Z_1 + dz;
      setSampleData(sample[tagcnt].G, G_X, G_Y, G_Z, imu->fullRange_G);
      sample[tagcnt].counter = sample_counter;
      break;
    }

    case 0x0E: // Sensor Hub Slave 0 Virtual Sensor hub data from slave 0
    case 0x0F: // Sensor Hub Slave 1 Virtual Sensor hub data from slave 1
    case 0x10: // Sensor Hub Slave 2 Virtual Sensor hub data from slave 2
    case 0x11: // Sensor Hub Slave 3 Virtual Sensor hub data from slave 3
    case 0x12: // Step Counter Virtual Step counter data
    case 0x19: // Sensor Hub Nack Virtual Sensor hub nack from slave 0/1/2/3
      return SampleStatus::TAG_NOT_IMPLEMENTED;

    default:
      return SampleStatus::UNKNOWN_TAG;
  }

  return SampleStatus::OK;
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
                                      uint16_t fullRange)
{
  s.rawXYZ[X_IDX] = X;
  s.rawXYZ[Y_IDX] = Y;
  s.rawXYZ[Z_IDX] = Z;
  s.fullRange = fullRange;

  float scale = fullRange * (1.0 / 32768);
  s.XYZ[X_IDX] = X * scale;
  s.XYZ[Y_IDX] = Y * scale;
  s.XYZ[Z_IDX] = Z * scale;
}

void LSM6DSOXFIFOClass::initializeSample(uint8_t idx)
{
  // timestamp and temperature are not always sent
  sample[idx].timestamp = NAN;
  sample[idx].temperature = NAN;

  // Set counter and full scale to 'impossible' values
  // to help identifying errors
  sample[idx].counter = counter_uninitialized;
  sample[idx].XL.fullRange = 0;
  sample[idx].G.fullRange = 0;

  // If compression is disabled, XL and G data may be
  // set to remarkable values in order to signal errors
  if(!compressionEnabled) {
    int16_t default_value = 0x7FFF;

    setSampleData(sample[idx].XL, default_value, default_value, default_value, 0);
    sample[idx].XL.XYZ[X_IDX] = NAN;
    sample[idx].XL.XYZ[Y_IDX] = NAN;
    sample[idx].XL.XYZ[Z_IDX] = NAN;

    setSampleData(sample[idx].G, default_value, default_value, default_value, 0);
    sample[idx].G.XYZ[X_IDX] = NAN;
    sample[idx].G.XYZ[Y_IDX] = NAN;
    sample[idx].G.XYZ[Z_IDX] = NAN;
  }
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
