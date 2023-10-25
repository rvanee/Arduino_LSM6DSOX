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

#include "LSM6DSOXTables.h"

#include <algorithm>  // find_if
#include <math.h>     // NAN


// Output Data Rate configuration bits (excluding 1.6Hz XL LP) 
const vectorOfFloatsAndBits LSM6DSOXTables::ODR_freq_bits = {
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
const vectorOfFloatsAndBits LSM6DSOXTables::FR_XL_bits = { // FS1_XL FS0_XL (CTRL1_XL)
  {  2, 0b00 }, 
  {  4, 0b10 }, 
  {  8, 0b11 }, 
  { 16, 0b01 }
};
const vectorOfIntsAndDownUpBits LSM6DSOXTables::FR_XL_bits_down_and_up = {
  {  2, {0b00, 0b00, 0b00, 0b00, 0b00, 0b10, 0b11, 0b01, 0b01 } },
  {  4, {0b00, 0b00, 0b00, 0b00, 0b10, 0b11, 0b01, 0b01, 0b01 } },
  {  8, {0b00, 0b00, 0b00, 0b10, 0b11, 0b01, 0b01, 0b01, 0b01 } },
  { 16, {0b00, 0b00, 0b10, 0b11, 0b01, 0b01, 0b01, 0b01, 0b01 } }
};

// G Full Range configuration bits
const vectorOfFloatsAndBits LSM6DSOXTables::FR_G_bits = { // FS1_G FS0_G FS_125 (CTRL2_G)
  {  125, 0b001 }, 
  {  250, 0b000 }, 
  {  500, 0b010 }, 
  { 1000, 0b100 }, 
  { 2000, 0b110 }
};
const vectorOfIntsAndDownUpBits LSM6DSOXTables::FR_G_bits_down_and_up = { // FS1_G FS0_G FS_125 (CTRL2_G)
  {  125, { 0b001, 0b001, 0b001, 0b001, 0b001, 0b000, 0b010, 0b100, 0b110 } }, 
  {  250, { 0b001, 0b001, 0b001, 0b001, 0b000, 0b010, 0b100, 0b110, 0b110 } }, 
  {  500, { 0b001, 0b001, 0b001, 0b000, 0b010, 0b100, 0b110, 0b110, 0b110 } }, 
  { 1000, { 0b001, 0b001, 0b000, 0b010, 0b100, 0b110, 0b110, 0b110, 0b110 } }, 
  { 2000, { 0b001, 0b000, 0b010, 0b100, 0b110, 0b110, 0b110, 0b110, 0b110 } }
};

// XL LPF2 ODR divisor configuration bits
const vectorOfFloatsAndBits LSM6DSOXTables::LPF2_XL_bits = { 
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
const std::map<uint8_t, vectorOfFloatsAndBits> LSM6DSOXTables::LPF1_G_bits = {
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

// G HPF cutoff / configuration bits
const vectorOfFloatsAndBits LSM6DSOXTables::HPF_G_bits = { 
  // bit 0: HP_EN_G (CTRL7_G)
  // bits 1-2 HPM_G[1:0] (CTRL7_G)
  { 0,     0b000 }, // HPF disabled
  { 0.016, 0b001 },
  { 0.065, 0b011 }, 
  { 0.260, 0b101 }, 
  { 1.04,  0b111 }
};

// Configuration bits for rate at which the compression algorithm 
// writes non-compressed data
const vectorOfFloatsAndBits LSM6DSOXTables::UNCOPTR_RATE_bits = { 
  // bits 0-1: UNCOPTR_RATE_[1:0] (FIFO_CTRL2)
  {  0, 0b00 }, //  Non-compressed data writing is not forced
  {  8, 0b01 },
  { 16, 0b10 }, 
  { 32, 0b11 }
};

// Timestamp decimation configuration bits
const vectorOfFloatsAndBits LSM6DSOXTables::DEC_TS_BATCH_bits = { 
  // bits 0-1: DEC_TS_BATCH_[1:0] (FIFO_CTRL4)
  {  0, 0b00 }, // Timestamp not batched in FIFO (default)
  {  1, 0b01 },
  {  8, 0b10 }, 
  { 32, 0b11 }
};

// Temperature batching data rate configuration bits
const vectorOfFloatsAndBits LSM6DSOXTables::ODR_T_BATCH_bits = { 
  // bits 0-1: ODR_T_BATCH_[1:0] (FIFO_CTRL4)
  {  0,   0b00 }, // Temperature not batched in FIFO (default)
  {  1.6, 0b01 },
  { 12.5, 0b10 }, 
  { 52,   0b11 }
};

/* For debugging purposes
std::map<uint8_t, String> LSM6DSOXTables::mapTagToStr = {
  { 0x01, "G_NC" }, 
  { 0x02, "XL_NC" }, 
  { 0x03, "TEMP" }, 
  { 0x04, "TIME" },
  { 0x05, "CFG" }, 
  { 0x06, "XL_NC_T_2" }, 
  { 0x07, "XL_NC_T_1" }, 
  { 0x08, "XL_2C" },
  { 0x09, "XL_3C" }, 
  { 0x0A, "G_NC_T_2" }, 
  { 0x0B, "G_NC_T_1" }, 
  { 0x0C, "G_2C" }, 
  { 0x0D, "G_3C" },
  { 0x0E, "SENS_HUB_0" }, 
  { 0x0F, "SENS_HUB_1" }, 
  { 0x10, "SENS_HUB_2" }, 
  { 0x11, "SENS_HUB_3" },
  { 0x12, "STEP_CNT" },
  { 0x19, "SENS_HUB_NACK" }
}; */

// Find configuration bits for nearest value in table.
uint8_t LSM6DSOXTables::nearestFloatToBits(float value, const vectorOfFloatsAndBits& v) {
  auto upper_it = lower_bound(
    v.begin(), v.end()-1, value,
    [](const pair<float, uint8_t>& p, float val) {return p.first < val;});
  auto lower_it = upper_it->first <= value ? upper_it : upper_it-1;
  uint8_t bits = abs(value - lower_it->first) <
                 abs(value - upper_it->first) ?
                    lower_it->second : upper_it->second;
  return bits;
}

// Find lowest full range in table >= specified value
uint8_t LSM6DSOXTables::largerOrEqualFloatToBits(float value, const vectorOfFloatsAndBits& v) {
  auto value_it = lower_bound(
    v.begin(), v.end()-1, value,
    [](const pair<float, uint8_t>& p, float val) {return p.first < val;});
  return value_it->second;
}

// Find lowest full range in table <= specified value
uint8_t LSM6DSOXTables::smallerOrEqualFloatToBits(float value, const vectorOfFloatsAndBits& v) {
  auto value_it = lower_bound(
    v.begin(), v.end(), value,
    [](const pair<float, uint8_t>& p, float val) {return p.first < val;});
  if((value_it != v.begin()) && (value_it->first > value)) {
    value_it = value_it - 1;
  }
  return value_it->second;
}

// Look up range from configuration bits
float LSM6DSOXTables::getFloatFromBits(uint8_t bits, const vectorOfFloatsAndBits& v) {
  auto match = find_if(
    v.begin(), v.end(),
    [bits](const pair<float, uint8_t>& it){ return it.second == bits; });
  if(match != v.end()) {
    return match->first;
  }
  return NAN;
}