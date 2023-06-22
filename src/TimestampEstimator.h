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

#ifndef TIMESTAMPESTIMATOR_H
#define TIMESTAMPESTIMATOR_H

#include <Arduino.h>


// Number of value pairs in buffer. This should be a (small)
// power of 2, <8. Buffer size should be inversely proportional to 
// the correlation between IMU sample counter and MCU microseconds.
#define TIMESTAMP_BUFFER_POWER2   6
#define TIMESTAMP_BUFFER_SIZE     (1 << TIMESTAMP_BUFFER_POWER2)
#define TIMESTAMP_BUFFER_MASK     (TIMESTAMP_BUFFER_SIZE-1)

// The b factor is a fixed point (rational) number relating an
// increase in sample counts to an increase in microseconds. It 
// should be scaled up ideally to the first power of 2 larger than
// the largest value of (MCU_micros - IMU_ref). It should also be
// small enough to have b * (MCU_micros - IMU_ref) fit in a 
// 64 bit integer. A reasonable value seems to be the number of bits
// in the buffer size plus two, requiring decoding such that at most
// 4 samples are read from the fifo on average for each decoder loop.
#define TIMESTAMP_BSCALE_POWER2   (2)

class SimpleUInt128 {
    uint64_t            hi64;
    uint64_t            lo64;

    //void                add(uint32_t x32);
    //void                add(uint64_t x64);
    //void                sub(uint32_t x32);
    //void                sub(uint64_t x64);
    //void                mul(uint64_t x32);
    //void                mul(uint64_t x64);

};

class TimestampEstimator {
  public:
    TimestampEstimator();
    ~TimestampEstimator();

    void                reset();

    void                add(uint32_t IMU_counter, uint32_t MCU_micros);
    inline uint64_t     estimate_micros(uint32_t IMU_counter)
    {
      return MCU_ref + ((b_scaled * (IMU_counter - IMU_ref)) >> TIMESTAMP_BSCALE_POWER2);
    }
    uint32_t            IMU_sum_sq_max;
    uint64_t            MCU_sum_IMU_max;

  private:
    uint8_t             N;
    uint8_t             buf_idx;

    uint8_t             IMU_delta_counter[TIMESTAMP_BUFFER_SIZE];
    uint32_t            IMU_sum;
    uint64_t            IMU_sum_sq;
    uint32_t            IMU_ref;
    uint32_t            IMU_prev;

    uint32_t            MCU_delta_timestamp[TIMESTAMP_BUFFER_SIZE];
    uint64_t            MCU_sum;
    uint64_t            MCU_sum_IMU;
    uint64_t            MCU_ref;
    uint64_t            MCU_prev;

    // b is a fixed point number scaled by 2**TIMESTAMP_BSCALE_POWER2
    uint64_t            b_scaled;
};

#endif // TIMESTAMPESTIMATOR_H
