/*
  This file is part of the Arduino_LSM6DSOX library.
  It was created by Rene F. van Ee, August 2023.
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

#include "AutoRanger.h"
#include "LSM6DSOXTables.h"


AutoRanger::AutoRanger() {
  reset();
}

void AutoRanger::reset(uint8_t alpha)
{
  this->alpha = alpha;
}

int8_t AutoRanger::predict_range(SampleData *prev_sample, SampleData *curr_sample)
{
  // Predict absolute values for X, Y and Z, and perform logical OR on them
  uint32_t orred_abs_predictions = abs_prediction(prev_sample->rawXYZ[0], curr_sample->rawXYZ[0]);
  orred_abs_predictions |= abs_prediction(prev_sample->rawXYZ[1], curr_sample->rawXYZ[1]);
  orred_abs_predictions |= abs_prediction(prev_sample->rawXYZ[2], curr_sample->rawXYZ[2]);

  // Perform logical OR on current values
  uint32_t orred_abs_current = static_cast<uint32_t>(abs(curr_sample->rawXYZ[0]));
  orred_abs_current |= static_cast<uint32_t>(abs(curr_sample->rawXYZ[1]));
  orred_abs_current |= static_cast<uint32_t>(abs(curr_sample->rawXYZ[2]));

  // Combined current and predicted values
  uint32_t combined_bits = orred_abs_predictions | orred_abs_current;

  // Find highest bit, after removing the lowest 14 bits (keeping ~4
  // significant bits).
  // Rationale: if b14 is set, either current or predicted values are
  // close to overflow. Any higher bit (b15, b16 etc.) set will
  // originate from predicted values and requires extra scaling up.
  uint8_t upper_bits_combined = static_cast<uint8_t>(combined_bits >> 14);
  int8_t scale_up;
  for(scale_up = 0; upper_bits_combined; scale_up++) {
    upper_bits_combined >>= 1;
  } // scale_up is now 0..4
  if(scale_up) { // Scale up if > 0
    return scale_up;
  }

  // Now see if we can scale down. For this we use a lower range of bits.
  // We know that b14 and up are 0 at this point. Since we can scale
  // 4 bits at most, we will used the 'combined' bits from b10 and up.
  uint8_t upper_bits_combined = static_cast<uint8_t>(combined_bits >> 10);
  int8_t scale_down;
  for(scale_down = 0; upper_bits_combined; scale_down++) {
    upper_bits_combined >>= 1;
  } // scale_down is now 0..4
  return -scale_down;
}

inline uint32_t AutoRanger::abs_prediction(int16_t prev_value, int16_t curr_value)
{
  int32_t curr_value_32 = static_cast<int32_t>(curr_value);
  int32_t delta = curr_value_32 - static_cast<int32_t>(prev_value);
  int32_t prediction = curr_value_32 + alpha * delta;

  return static_cast<uint32_t>(abs(prediction));
}