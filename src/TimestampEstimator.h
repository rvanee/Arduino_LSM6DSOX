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


// Stage 1 is a downsampling stage that takes care of the
// wide range of sample frequencies that the IMU offers.
// It collects MCU timestamps in microseconds related to
// the IMU sample counter, for a short interval (~0.16s).
// Setting this interval too large will cause overflow
// errors in the filter when using high sample rates.
#define TIMESTAMP_STAGE1_DURATION 160000

// Stage 2 is the main stage. It contains a deque of stage 1
// output timestamp/counter pairs, implemented as a circular
// buffer with a power-of-2 size for performance reasons.
// This size should be reasonably small (~2**8) to limit
// memory use and prevent overflow.
#define TIMESTAMP_STAGE2_POWER2   6
#define TIMESTAMP_STAGE2_SIZE     (1 << TIMESTAMP_STAGE2_POWER2)
#define TIMESTAMP_STAGE2_MASK     (TIMESTAMP_STAGE2_SIZE-1)


class TEFirstStage {
  public:
    TEFirstStage(uint32_t duration = TIMESTAMP_STAGE1_DURATION);

    void                reset();
    void                initialize(uint32_t IMU_counter, uint64_t MCU_micros_64);
    uint64_t            add(uint32_t IMU_counter, uint64_t MCU_micros_64);

  private:
    bool                initialized;

    uint32_t            duration;
    uint16_t            N;

    uint32_t            IMU_prev;
    uint32_t            IMU_ref;
    uint32_t            IMU_sum;
    uint64_t            IMU_sum_sq;

    uint64_t            MCU_ref;
    uint64_t            MCU_sum;
    uint64_t            MCU_sum_IMU;
};

class TESecondStage {
  public:
    TESecondStage();
  
    void                reset();
    void                initialize(uint32_t IMU_counter, uint64_t MCU_micros_64);
    void                add(uint32_t IMU_counter, uint64_t MCU_micros_64);
    uint64_t            estimate(uint32_t IMU_counter);

  private:
    uint16_t            N;
    uint16_t            buf_idx;

    uint32_t            IMU_prev;
    uint32_t            IMU_ref;
    uint32_t            IMU_sum;
    uint64_t            IMU_sum_sq;
    uint16_t            IMU_delta_counter[TIMESTAMP_STAGE2_SIZE];

    uint64_t            MCU_prev;
    uint64_t            MCU_ref;
    uint64_t            MCU_sum;
    uint64_t            MCU_sum_IMU;
    uint32_t            MCU_delta_timestamp[TIMESTAMP_STAGE2_SIZE];

    uint64_t            a;
    uint64_t            b_denominator;
    lldiv_t             b;

    uint64_t            estimate_micros;
    uint64_t            estimate_fraction;
    uint32_t            estimate_IMU;
};

class TimestampEstimator {
  public:
    TimestampEstimator();
  
    void                reset();

    void                add(uint32_t IMU_counter, uint32_t MCU_micros);
    uint64_t            estimate(uint32_t IMU_counter);

  private:
    uint64_t            MCU_current;

    // Stages
    TEFirstStage        firstStage;
    TESecondStage       secondStage;
};

#endif // TIMESTAMPESTIMATOR_H
