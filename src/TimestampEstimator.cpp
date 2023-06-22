/*
  This file is part of the Arduino_LSM6DSOX library.
  It was created by Rene F. van Ee, June 2023.
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

#include "TimestampEstimator.h"
#include <algorithm>


TimestampEstimator::TimestampEstimator() {
  reset();
}

TimestampEstimator::~TimestampEstimator() {
}

void TimestampEstimator::reset()
{
    N = 0;
    buf_idx = 0;

    // Reset buffers
    std::fill(IMU_delta_counter,   IMU_delta_counter  +TIMESTAMP_BUFFER_SIZE, 0);
    std::fill(MCU_delta_timestamp, MCU_delta_timestamp+TIMESTAMP_BUFFER_SIZE, 0);

    // IMU sample counter variables
    IMU_sum = 0;
    IMU_sum_sq = 0;
    IMU_ref = 0;
    IMU_prev = 0;
    IMU_sum_sq_max = 0;

    // MCU micros variables
    MCU_sum = 0;
    MCU_ref = 0;
    MCU_sum_IMU = 0;
    MCU_prev = 0;
    MCU_sum_IMU_max = 0;

    // b is a fixed point number scaled by 2**TIMESTAMP_BSCALE_POWER2
    b_scaled = 0;
}

void TimestampEstimator::add(uint32_t IMU_counter, uint32_t MCU_micros)
{
  // Initialize if first sample
  if(N == 0) {
    IMU_ref = IMU_counter;
    IMU_prev = IMU_counter;

    MCU_ref = MCU_micros;
    MCU_prev = MCU_micros;
  }

  uint8_t delta_IMU_counter = IMU_counter - IMU_prev;
  // Only store new counter/micros pair if counter has changed,
  // otherwise we may end up with multiple micros data points at
  // the same counter. Except for the first sample.
  if((delta_IMU_counter > 31) || (N == 0)) {
    IMU_prev = IMU_counter;

    // Note that this code will take care of wrap-around of MCU_micros
    uint32_t delta_MCU_micros = MCU_micros - (uint32_t)MCU_prev;
    MCU_prev += delta_MCU_micros;

    // Retrieve oldest data point
    uint8_t  delta_IMU_0 = IMU_delta_counter[buf_idx];
    uint32_t delta_MCU_0 = MCU_delta_timestamp[buf_idx];

    // Store new deltas
    IMU_delta_counter[buf_idx]   = delta_IMU_counter;
    MCU_delta_timestamp[buf_idx] = delta_MCU_micros;
    buf_idx = (buf_idx+1) & TIMESTAMP_BUFFER_MASK;

    // Adjust reference values
    IMU_ref += delta_IMU_0;
    MCU_ref += delta_MCU_0;

    // Use differences between new and *reference* values to add
    // 'normalized' new data to sums below
    uint32_t normalized_delta_IMU = IMU_counter - IMU_ref;
    uint64_t normalized_delta_MCU = MCU_micros  - MCU_ref;

    if(N < TIMESTAMP_BUFFER_SIZE) { // Still filling the buffer (at startup)?
      N++;

      // Add 'normalized' new data to sums
      IMU_sum += normalized_delta_IMU;
      IMU_sum_sq += normalized_delta_IMU*normalized_delta_IMU;
      MCU_sum += normalized_delta_MCU;
      MCU_sum_IMU += normalized_delta_IMU*normalized_delta_MCU;

      // Calculate b as a scaled 64 bit unsigned integer
      uint64_t b_denominator = N*IMU_sum_sq - IMU_sum*IMU_sum;
      if(b_denominator > 0) {
        uint64_t b_numerator = (N*MCU_sum_IMU - IMU_sum*MCU_sum) << TIMESTAMP_BSCALE_POWER2;
        b_scaled = b_numerator / b_denominator;
      } else {
        b_scaled = 0;
      }
    } else {
      // Subtract the old deltas from the sums.
      // This will keep the sums relatively small, preventing overflow.
      IMU_sum_sq  -= uint64_t(IMU_sum)*(2*delta_IMU_0);
      IMU_sum_sq  += (uint32_t(delta_IMU_0)*delta_IMU_0) << TIMESTAMP_BUFFER_POWER2;
      MCU_sum_IMU -= uint64_t(IMU_sum)*delta_MCU_0;
      MCU_sum_IMU -= MCU_sum*delta_IMU_0;
      MCU_sum_IMU += (uint64_t(delta_MCU_0)*delta_IMU_0) << TIMESTAMP_BUFFER_POWER2;
      IMU_sum     -= (uint32_t(delta_IMU_0)) << TIMESTAMP_BUFFER_POWER2;
      MCU_sum     -= delta_MCU_0 << TIMESTAMP_BUFFER_POWER2;

      // Add 'normalized' new data to sums
      IMU_sum     += normalized_delta_IMU;
      IMU_sum_sq  += normalized_delta_IMU*normalized_delta_IMU;
      MCU_sum     += normalized_delta_MCU;
      MCU_sum_IMU += normalized_delta_IMU*normalized_delta_MCU;

      // Calculate b as a scaled 64 bit unsigned integer
      uint64_t b_numerator   = ((MCU_sum_IMU << TIMESTAMP_BUFFER_POWER2) - IMU_sum*MCU_sum) << TIMESTAMP_BSCALE_POWER2;
      uint64_t b_denominator = (IMU_sum_sq << TIMESTAMP_BUFFER_POWER2) - uint64_t(IMU_sum)*IMU_sum;
      b_scaled = b_numerator / b_denominator;
    //Serial.print("normalized_delta_IMU=");
    //Serial.print(normalized_delta_IMU);
    //Serial.print(", normalized_delta_MCU=");
    //Serial.println(normalized_delta_MCU);
    //Serial.print(", b_scaled=");
    //Serial.println(b_scaled);
    }

    if(IMU_sum_sq_max < IMU_sum_sq) IMU_sum_sq_max = IMU_sum_sq;
    if(MCU_sum_IMU_max < MCU_sum_IMU) MCU_sum_IMU_max = MCU_sum_IMU;
  } // END if(delta_IMU_counter == 0)
}
