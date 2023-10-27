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

#ifndef LSM6DSOXTABLES_H
#define LSM6DSOXTABLES_H

#include <vector>
#include <utility> // std::pair
#include <map>

typedef std::vector<std::pair<float, uint8_t>> vectorOfFloatsAndBits;

class LSM6DSOXTables {
  public:
    // Output Data Rate configuration bits (excluding 1.6Hz XL LP) 
    static const vectorOfFloatsAndBits ODR_freq_bits;

    // XL Full Range configuration bits
    static const vectorOfFloatsAndBits FR_XL_bits;

    // G Full Range configuration bits
    static const vectorOfFloatsAndBits FR_G_bits;

    // XL LPF2 ODR divisor configuration bits
    static const vectorOfFloatsAndBits LPF2_XL_bits;

    // Map of G ODR bits to sorted vector of cutoff frequency and 
    // configuration bit pairs:
    // bit 0: LPF1_SEL_G (CTRL4_C)
    // bits 1..3: FTYPE_2 FTYPE_1 FTYPE_0 (CTRL6_C)
    //
    // Note that below wherever 'LPF1 disabled' is stated, the cutoff
    // frequency is set to an arbitrary number slightly below ODR/2,
    // in order for this value to be selected when presented with
    // a cutoff of ODR/2. This will lead to G LPF1 being disabled. 
    static const std::map<uint8_t, vectorOfFloatsAndBits> LPF1_G_bits;

    // G HPF cutoff / configuration bits
    static const vectorOfFloatsAndBits HPF_G_bits;

    // Configuration bits for rate at which the compression algorithm 
    // writes non-compressed data
    static const vectorOfFloatsAndBits UNCOPTR_RATE_bits;

    // Timestamp decimation configuration bits
    static const vectorOfFloatsAndBits DEC_TS_BATCH_bits;

    // Temperature batching data rate configuration bits
    static const vectorOfFloatsAndBits ODR_T_BATCH_bits;

    /* For debugging purposes
    static const std::map<uint8_t, String> mapTagToStr;
    */

  public:
    static uint8_t nearestFloatToBits(float value, const vectorOfFloatsAndBits& v);
    static uint8_t largerOrEqualFloatToBits(float value, const vectorOfFloatsAndBits& v);
    static uint8_t smallerOrEqualFloatToBits(float value, const vectorOfFloatsAndBits& v);
    static float getFloatFromBits(uint8_t bits, const vectorOfFloatsAndBits& v);
};

#endif // LSM6DSOXTABLES_H