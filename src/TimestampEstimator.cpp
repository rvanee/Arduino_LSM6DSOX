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
#include <cstdlib>


TimestampEstimator::TimestampEstimator() {
  reset();
}

TimestampEstimator::~TimestampEstimator() {
}

void TimestampEstimator::reset()
{
    MCU_current = 0;

    // Init stages
    firstStage.reset();
    secondStage.reset();
}

uint64_t TimestampEstimator::add(uint32_t IMU_counter, uint32_t MCU_micros)
{
  // Take care of wrap-around of MCU_micros
  uint32_t MCU_diff =
    static_cast<uint32_t>((1ULL << 32) + MCU_micros - static_cast<uint32_t>(MCU_current));
  MCU_current += MCU_diff;

  uint64_t estimate = firstStage.add(IMU_counter, MCU_current);
  if(estimate) {
    estimate = secondStage.add(IMU_counter, estimate);
    Serial.print("IMU_counter="+String(IMU_counter)+", MCU_current=");
    Serial.print(MCU_current);
    Serial.print(", estimate=");
    Serial.println(estimate);
  } else {
    // TODO
  }

  return estimate;

/*

  uint16_t delta_IMU_counter = IMU_counter - IMU_prev;
  IMU_prev = IMU_counter;
  // Only store new counter/micros pair if counter has changed,
  // otherwise we may end up with multiple micros data points at
  // the same counter. Except for the first sample.
  if((delta_IMU_counter > 0) || (N == 0)) {
    N++;

    // Note that this code will take care of wrap-around of MCU_micros
    uint32_t delta_MCU_micros = MCU_micros - (uint32_t)MCU_prev;
    uint64_t MCU_micros_64 = MCU_prev + delta_MCU_micros;

    // Stage 1 filling
    uint32_t delta_IMU_counter_stage1 = IMU_counter - IMU_ref_stage1;
    IMU_sum_stage1     += delta_IMU_counter_stage1;
    IMU_sum_sq_stage1  += delta_IMU_counter_stage1*delta_IMU_counter_stage1;
    uint32_t delta_MCU_micros_stage1  = MCU_micros_64 - MCU_ref_stage1;
    MCU_sum_stage1     += delta_MCU_micros_stage1;
    MCU_sum_IMU_stage1 += uint64_t(delta_MCU_micros_stage1)*delta_IMU_counter_stage1;
    N_stage1++;
    if(delta_MCU_micros_stage1 >= TIMESTAMP_STAGE1_DURATION) {
      // Stage 1 filled -> proceed to stage 2
      uint64_t b_denominator = (IMU_sum_sq_stage1 * N_stage1) - uint64_t(IMU_sum_stage1)*IMU_sum_stage1;
      uint64_t b_numerator = (((MCU_sum_IMU_stage1 * N_stage1) - MCU_sum_stage1*IMU_sum_stage1));
      Serial.print("IMU_sum_stage1="+String(IMU_sum_stage1)+", IMU_sum_sq_stage1=");
      Serial.print(IMU_sum_sq_stage1);
      Serial.print(", MCU_sum_stage1=");
      Serial.print(MCU_sum_stage1);
      Serial.print(", MCU_sum_IMU_stage1=");
      Serial.println(MCU_sum_IMU_stage1);
      Serial.print("b_denominator=");
      Serial.print(b_denominator);
      Serial.print(", b_numerator=");
      Serial.print(b_numerator);
      Serial.print(", N_stage1=");
      Serial.print(N_stage1);

      uint64_t MCU_estimate_stage1 = 0;
      if(b_denominator > 0) {
        MCU_estimate_stage1 = (b_numerator*(delta_IMU_counter_stage1*N_stage1 - IMU_sum_stage1))/(b_denominator*N_stage1) +
          (MCU_ref_stage1*N_stage1 + MCU_sum_stage1) / N_stage1;
        //MCU_estimate_stage1 = (b_numerator * delta_IMU_counter_stage1) / b_denominator;
        //int64_t a64 = (MCU_ref_stage1*N_stage1 + MCU_sum_stage1 - ((b_numerator * IMU_sum_stage1) / b_denominator)) / N_stage1;
       // Serial.print(", a=");
        //Serial.print(a64);
        //MCU_estimate_stage1 += a64;
        Serial.print(", b_numerator*(delta_IMU_counter_stage1*N_stage1 - IMU_sum_stage1)=");
        Serial.print(b_numerator*(delta_IMU_counter_stage1*N_stage1 - IMU_sum_stage1));
        Serial.print(", b=");
        Serial.print(b_numerator / b_denominator);
        Serial.print(", MCU_estimate_stage1=");
        Serial.print(MCU_estimate_stage1);
        Serial.print(", MCU_micros_64=");
        Serial.print(MCU_micros_64);

        b = uint64_t(b_numerator / b_denominator);
      }
      Serial.println("");
        
      // Reset stage 1
      N_stage1 = 0;
      IMU_ref_stage1 = IMU_counter;
      IMU_sum_stage1 = 0;
      IMU_sum_sq_stage1 = 0;
      MCU_ref_stage1 = MCU_micros_64;
      MCU_sum_stage1 = 0;
      MCU_sum_IMU_stage1 = 0;
    }

    // Retrieve oldest data point
    uint16_t delta_IMU_0 = IMU_delta_counter[buf_idx];
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
      IMU_sum_sq += uint64_t(normalized_delta_IMU)*normalized_delta_IMU;
      MCU_sum += normalized_delta_MCU;
      MCU_sum_IMU += uint128_t(normalized_delta_IMU)*normalized_delta_MCU;

      // Calculate b as a scaled 64 bit unsigned integer
      uint128_t b_denominator = (uint128_t(IMU_sum_sq) * N) - uint128_t(IMU_sum)*IMU_sum;
      uint128_t b_numerator = (((MCU_sum_IMU * N) - MCU_sum*IMU_sum) << 32);
      //double b_denominator = double((uint128_t(IMU_sum_sq) * N) - uint128_t(IMU_sum)*IMU_sum);
      if(b_denominator > 0) {
        b = uint64_t(b_numerator / b_denominator)/4294967296.0;
      } else {
        b = 0.0;
      }
      a = (MCU_sum - uint64_t(b*IMU_sum)).lower() / N;
    } else {
      // Subtract the old deltas from the sums.
      // This will keep the sums relatively small, preventing overflow.
      IMU_sum_sq  -= IMU_sum*(delta_IMU_0 << 1);
      IMU_sum_sq  += ((uint32_t(delta_IMU_0)*delta_IMU_0) << TIMESTAMP_BUFFER_POWER2);
      //MCU_sum_IMU -= double(IMU_sum)*delta_MCU_0 + MCU_sum*delta_IMU_0;
      //MCU_sum_IMU += ((uint64_t(delta_MCU_0)*delta_IMU_0) << TIMESTAMP_BUFFER_POWER2);
      //IMU_sum     -= ((uint32_t(delta_IMU_0)) << TIMESTAMP_BUFFER_POWER2);
      //MCU_sum     -= (uint64_t(delta_MCU_0) << TIMESTAMP_BUFFER_POWER2);
      //IMU_sum_sq  += TIMESTAMP_BUFFER_SIZE*double(delta_IMU_0)*delta_IMU_0;
      MCU_sum_IMU -= (uint128_t(IMU_sum)*delta_MCU_0 + MCU_sum*delta_IMU_0);
      MCU_sum_IMU += ((uint64_t(delta_MCU_0)*delta_IMU_0) << TIMESTAMP_BUFFER_POWER2);
      IMU_sum     -= (uint32_t(delta_IMU_0) << TIMESTAMP_BUFFER_POWER2);
      MCU_sum     -= (uint64_t(delta_MCU_0) << TIMESTAMP_BUFFER_POWER2);

      // Add 'normalized' new data to sums
      IMU_sum     += normalized_delta_IMU;
      IMU_sum_sq  += uint64_t(normalized_delta_IMU)*normalized_delta_IMU;
      MCU_sum     += normalized_delta_MCU;
      MCU_sum_IMU += uint128_t(normalized_delta_IMU)*normalized_delta_MCU;

      // Calculate b as a scaled 64 bit unsigned integer
      //uint64_t b_numerator   = ((MCU_sum_IMU << TIMESTAMP_BUFFER_POWER2) - IMU_sum*MCU_sum) << TIMESTAMP_BSCALE_POWER2;
      uint128_t b_denominator = (uint128_t(IMU_sum_sq) << TIMESTAMP_BUFFER_POWER2) - uint128_t(IMU_sum)*IMU_sum;
      uint128_t b_numerator = (((MCU_sum_IMU << TIMESTAMP_BUFFER_POWER2) - MCU_sum*IMU_sum) << TIMESTAMP_BSCALE_POWER2);
      //uint8_t shift = 128 - b_numerator.bits();
      //b_numerator <<= shift;
      b = uint64_t(b_numerator / b_denominator) / double(TIMESTAMP_BSCALE);
      //b = uint64_t(b_numerator / b_denominator)/double(1ULL << shift);

      // y = a + bx <=> MCU = a + b*IMU => MCU_sum/N = a + b* IMU_sum/N <=> a = (MCU_sum + b*IMU_sum) / N
      //a = mean(y) - b * mean(x)
      double bx = b*IMU_sum;
      a = ((MCU_sum - uint128_t(bx)) >> TIMESTAMP_BUFFER_POWER2).lower();

    //Serial.print("normalized_delta_IMU=");
    //Serial.print(normalized_delta_IMU);
    //Serial.print(", normalized_delta_MCU=");
    //Serial.println(normalized_delta_MCU);
    //Serial.print(", b_scaled=");
    //Serial.println(b_scaled);
    }

    if(IMU_sum_sq_max < IMU_sum_sq) IMU_sum_sq_max = IMU_sum_sq;
  } // END if(delta_IMU_counter == 0)
  */
}

uint64_t TimestampEstimator::next()
{
  return secondStage.estimate_next();
}

TEFirstStage::TEFirstStage(uint32_t duration) {
  this->duration = duration;
  reset();
}

TEFirstStage::~TEFirstStage() {
}

void TEFirstStage::reset()
{
  initialized = false;
  N = 0;

  IMU_sum = 0;
  IMU_sum_sq = 0;

  MCU_sum = 0;
  MCU_sum_IMU = 0;
}

void TEFirstStage::initialize(uint32_t IMU_counter, uint64_t MCU_micros_64)
{
  IMU_prev = IMU_counter;
  IMU_ref  = IMU_counter;

  MCU_ref = MCU_micros_64;

  initialized = true;
}

uint64_t TEFirstStage::add(uint32_t IMU_counter, uint64_t MCU_micros_64)
{
  if(!initialized) { // Initialize if first sample
    initialize(IMU_counter, MCU_micros_64);
  } else { // Fill
    // Only store new counter/micros pair if counter has changed,
    // otherwise we may end up with multiple micros data points at
    // the same counter.
    if(IMU_prev < IMU_counter) {
      N++;
      IMU_prev = IMU_counter;
  
      uint32_t delta_IMU_counter = IMU_counter - IMU_ref;
      IMU_sum    += delta_IMU_counter;
      IMU_sum_sq += delta_IMU_counter*delta_IMU_counter;

      uint64_t delta_MCU_micros = MCU_micros_64 - MCU_ref;
      MCU_sum     += delta_MCU_micros;
      MCU_sum_IMU += delta_MCU_micros*delta_IMU_counter;
      
      // Specified amount of time elapsed?
      if(delta_MCU_micros >= duration) {
        uint64_t b_denominator = N*IMU_sum_sq  - IMU_sum*static_cast<uint64_t>(IMU_sum);
        uint64_t b_numerator   = N*MCU_sum_IMU - IMU_sum*MCU_sum;

        // Estimate MCU_micros_64 using linear regression for the current
        // IMU_counter.
        // Note that by definition (N*delta_IMU_counter - IMU_sum) >= 0
        uint64_t MCU_micros_64_est = MCU_ref;
        if(b_denominator > 0) { 
          MCU_micros_64_est +=
            (MCU_sum +
            (b_numerator*(N*delta_IMU_counter - IMU_sum)) / b_denominator) / N;
        }

        // Reset, then immediately reuse current IMU and MCU data to
        // set references and IMU_prev
        reset();
        initialize(IMU_counter, MCU_micros_64);

        return MCU_micros_64_est;
      } // END if(delta_MCU_micros >= duration)
    } // END if(IMU_counter > IMU_prev)
  } // END else                   

  return 0ULL;
}

TESecondStage::TESecondStage() {
  reset();
}

TESecondStage::~TESecondStage() {
}

void TESecondStage::reset()
{
  N = 0;

  IMU_sum = 0;
  IMU_sum_sq = 0;
  std::fill(IMU_delta_counter,   IMU_delta_counter  +TIMESTAMP_BUFFER_SIZE, 0);

  MCU_sum = 0;
  MCU_sum_IMU = 0;
  std::fill(MCU_delta_timestamp, MCU_delta_timestamp+TIMESTAMP_BUFFER_SIZE, 0);

  a = 0;
  b_denominator = 0;
  b = {0, 0};
  estimate = 0;
  estimate_fraction = 0;
}

void TESecondStage::initialize(uint32_t IMU_counter, uint64_t MCU_micros_64)
{
  IMU_prev = IMU_counter;
  IMU_ref  = IMU_counter;

  MCU_prev = MCU_micros_64;
  MCU_ref  = MCU_micros_64;
}

uint64_t TESecondStage::add(uint32_t IMU_counter, uint64_t MCU_micros_64)
{
  if(N == 0) { // Initialize if first sample
    initialize(IMU_counter, MCU_micros_64);
  }

  uint16_t delta_IMU_counter = IMU_counter - IMU_prev;
  IMU_prev = IMU_counter;
  uint32_t delta_MCU_micros = MCU_micros_64 - MCU_prev;
  MCU_prev = MCU_micros_64;

  // Retrieve oldest data point
  uint16_t delta_IMU_0 = IMU_delta_counter[buf_idx];
  uint32_t delta_MCU_0 = MCU_delta_timestamp[buf_idx];

  // Store new deltas
  IMU_delta_counter[buf_idx]   = delta_IMU_counter;
  MCU_delta_timestamp[buf_idx] = delta_MCU_micros;
  buf_idx = (buf_idx+1) & TIMESTAMP_BUFFER_MASK;

  // Adjust reference values
  IMU_ref += delta_IMU_0;
  MCU_ref += delta_MCU_0;

  // Use differences between new and *reference* values to add
  // 'referenced' new data to sums below
  uint16_t ref_delta_IMU = IMU_counter   - IMU_ref;
  uint32_t ref_delta_MCU = MCU_micros_64 - MCU_ref;

  if(N < TIMESTAMP_BUFFER_SIZE) { // Still filling the buffer (at startup)?
    N++;
  } else { // So N == TIMESTAMP_BUFFER_SIZE, which is 1 << TIMESTAMP_BUFFER_POWER2
    // Subtract the old deltas from the sums.
    // This will keep the sums relatively small, preventing overflow.
    IMU_sum_sq  -= static_cast<uint64_t>(IMU_sum)*(delta_IMU_0 << 1);
    IMU_sum_sq  += ((static_cast<uint32_t>(delta_IMU_0)*delta_IMU_0) << TIMESTAMP_BUFFER_POWER2);
    MCU_sum_IMU -= (static_cast<uint64_t>(IMU_sum)*delta_MCU_0 + MCU_sum*delta_IMU_0);
    MCU_sum_IMU += ((static_cast<uint64_t>(delta_MCU_0)*delta_IMU_0) << TIMESTAMP_BUFFER_POWER2);
    IMU_sum     -= (static_cast<uint32_t>(delta_IMU_0) << TIMESTAMP_BUFFER_POWER2);
    MCU_sum     -= (static_cast<uint64_t>(delta_MCU_0) << TIMESTAMP_BUFFER_POWER2);
  }

  // Add 'referenced' new data to sums
  IMU_sum     += ref_delta_IMU;
  IMU_sum_sq  += ref_delta_IMU*static_cast<uint32_t>(ref_delta_IMU);
  MCU_sum     += ref_delta_MCU;
  MCU_sum_IMU += ref_delta_IMU*static_cast<uint64_t>(ref_delta_MCU);

  // Linear regression: y = a + bx
  // Calculate a and b (b in denominator and numerator parts).
  // x = (counter - counter_ref), y = (MCU - MCU_ref)
  // Note that a includes MCU_ref, so y = MCU
  uint64_t b_numerator = IMU_sum*MCU_sum;
  b_denominator = IMU_sum*static_cast<uint64_t>(IMU_sum);
  if(N < TIMESTAMP_BUFFER_SIZE) {
    b_numerator   = N*MCU_sum_IMU - b_numerator;
    b_denominator = N*IMU_sum_sq  - b_denominator;
    b = div(static_cast<long long>(b_numerator), 
            static_cast<long long>(b_denominator));
    uint64_t b_IMU_sum = static_cast<uint64_t>(b.quot)*IMU_sum + 
                        (static_cast<uint64_t>(b.rem) *IMU_sum)/b_denominator;
    a = (MCU_sum >= b_IMU_sum) ? // Only use unsigned arithmetic
          MCU_ref + ((MCU_sum - b_IMU_sum) / N) :
          MCU_ref - ((b_IMU_sum - MCU_sum) / N);
  } else { // So N == TIMESTAMP_BUFFER_SIZE, which is 1 << TIMESTAMP_BUFFER_POWER2
    b_numerator   = (MCU_sum_IMU << TIMESTAMP_BUFFER_POWER2) - b_numerator;
    b_denominator = (IMU_sum_sq  << TIMESTAMP_BUFFER_POWER2) - b_denominator;
    b = div(static_cast<long long>(b_numerator), 
            static_cast<long long>(b_denominator));
    uint64_t b_IMU_sum = static_cast<uint64_t>(b.quot)*IMU_sum + 
                        (static_cast<uint64_t>(b.rem) *IMU_sum)/b_denominator;
    a = (MCU_sum >= b_IMU_sum) ? // Only use unsigned arithmetic
          MCU_ref + ((MCU_sum - b_IMU_sum) >> TIMESTAMP_BUFFER_POWER2) :
          MCU_ref - ((b_IMU_sum - MCU_sum) >> TIMESTAMP_BUFFER_POWER2);
  }

  Serial.print("N="+String(N));
  Serial.print(", b_denominator=");
  Serial.print(b_denominator);
  Serial.print(", b_numerator=");
  Serial.print(b_numerator);
  Serial.print(", b_integer=");
  Serial.print(b.quot);
  Serial.print(", b_remainder=");
  Serial.print(b.rem);
  Serial.print(", a=");
  Serial.println(a);

  // MCU estimate = a + b*delta_IMU
  // Note that b consists of an integer part (b.quot) and a fractional
  // part. The fractional part is a rational number: b.rem / b_denominator.
  lldiv_t remainder_product = 
    div(static_cast<long long>(b.rem)*delta_IMU_counter,
        static_cast<long long>(b_denominator));
  estimate = a +
             static_cast<uint64_t>(b.quot)*delta_IMU_counter +
             static_cast<uint64_t>(remainder_product.quot);

  // Keep a running account of the fraction for use in estimate_next()
  estimate_fraction = static_cast<uint64_t>(remainder_product.rem);

  return estimate;
}

// Linear extrapolation based on linear regression above
uint64_t TESecondStage::estimate_next()
{
  // Add b to current estimate:
  // + integer part
  estimate += static_cast<uint64_t>(b.quot);

  // + fractional/rational part (= b_remainder / b_denominator)
  estimate_fraction += static_cast<uint64_t>(b.rem);
  if(estimate_fraction >= b_denominator) {
    estimate++;
    estimate_fraction -= b_denominator;
  }
  
  return estimate;
}
