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

#include "LSM6DSOXAutoRanger.h"
#include "LSM6DSOXFIFO.h"
#include "LSM6DSOXTables.h"

//#include <bitset>
//#include <algorithm>


LSM6DSOXAutoRanger::LSM6DSOXAutoRanger(const vectorOfFloatsAndBits& v) {
  // Copy full ranges from full range to bits transformation
  // table, and sort it in ascending order
  for(int i = 0; i < v.size(); i++) {
    v_ranges.push_back(static_cast<uint16_t>(v[i].first))
  }
  sort(v_ranges.begin(), v_ranges.end()); 

  reset();
}

void LSM6DSOXAutoRanger::reset(uint16_t threshold_up, uint16_t threshold_down)
{
  // Configuration
  this->threshold_up = threshold_up;
  this->threshold_down = threshold_down;

  // Short history of samples
  samples_deque.clear();

  // Last reported full range
  current_full_range = 0;

  // Pending full range change
  new_full_range_pending = false;
  new_full_range = 0;
  counter_at_change = 0;
  delay_to_change = LSM6DSOXAUTORANGER_INITIAL_DELAY;
}

void LSM6DSOXAutoRanger::notify_set_full_range(uint32_t current_counter, uint16_t new_full_range)
{
  counter_at_change = current_counter;
  this->new_full_range = new_full_range;
  new_full_range_pending = true;
}

void LSM6DSOXAutoRanger::notify_new_full_range(uint32_t current_counter, uint16_t new_full_range)
{
  // Always update our full range state
  current_full_range = new_full_range;
  
  // Notifications are not always related to a change in full range that
  // we initiated, so check if there is a change pending.
  // Moreover, check that the updated full range matches the one that
  // was initiated. It is possible for gyro and acc full range changes
  // to interfere.
  if(new_full_range_pending && (this->new_full_range == new_full_range)) {
    // This is our best estimate of the delay in sample numbers between
    // initiating a change in full range and noticing the effect of it.
    delay_to_change = current_counter - counter_at_change;
    new_full_range_pending = false;
  }
}

uint16_t LSM6DSOXAutoRanger::add_and_check_sample(uint32_t counter, LSM6DSOXSampleData &sample)
{
  uint16_t full_range = sample.fullRange;

  // Compare with last sample in deque
  if(samples_deque.size() > 0) {
    if(counter == samples_deque.back().counter) {
      // Do nothing, we already processed this sample
      return 0;
    }
    if(full_range != samples_deque.back().sample.fullRange) {
      // Change in full range -> clear deque, then continue below
      samples_deque.clear();
    }
  }

  // Get max absolute raw (16-bit signed) value in the
  // 3 channels of the sample provided
  uint32_t max_abs = max_abs_value(sample);

  // Find full_range value that would bring the max_abs
  // value into range [threshold_down, threshold_up], if
  // possible
  int s = v_ranges.size();
  // Linear search for matching full_range
  for(int idx = 0; idx < s; idx++) {
    if(v_ranges[idx] == full_range) { // Found
      int idx_scaled;
      // Find range for which max_abs < threshold_up
      // NOTE that this assumes that each step up in range
      // doubles it by a factor of 2
      for(idx_scaled = idx; 
          (max_abs >= threshold_up) && (idx_scaled < s); 
          idx_scaled++, max_abs >>= 1);
      if(idx_scaled > idx) {
        full_range = v_ranges[idx_scaled];
        break;
      }

      // No matching higher range found, so try and see
      // if a lower range would do: find range for which 
      // max_abs >= threshold_down
      // NOTE that this assumes that each step up in range
      // doubles it by a factor of 2
      for(idx_scaled = idx; 
          (max_abs < threshold_down) && (idx_scaled >= 0); 
          idx_scaled--, max_abs <<= 1);
      if(idx_scaled < idx) {
        full_range = v_ranges[idx_scaled];
      }
      break;
    }
  }

  // Store new sample in deque
  samples_deque.push_back(
    LSM6DSOXAutoRangerSample(sample, counter, full_range));

  // Check if full range should change. This should only happen
  // when the optimal full range found above is larger than the
  // currently set full range (emergency gear up to prevent overflow).
  if(full_range > current_full_range) {
    // Now check if a full range change is already pending. If so, 
    // only override that if the new optimal full range is larger 
    // than the one being set right now.
    if(!new_full_range_pending || (full_range > new_full_range)) {
      notify_set_full_range(counter, full_range);
      return full_range;
    }
  }

  // No emergency change necessary
  return 0;
}

uint16_t LSM6DSOXAutoRanger::check_underflow()
{
  uint16_t full_range = 0;

  // Of course, no full range change should be pending.
  // The deque should contain more samples than are being
  // produced during full range change set to effect.
  if(!new_full_range_pending &&
     (samples_deque.size() > delay_to_change)) {
    // Find largest optimal full range in deque
    uint16_t fr = 0;
    for(auto s: samples_deque) {
      if(fr < s.optimal_full_range) fr = s.optimal_full_range;
    }
    if(fr < current_full_range) {
      // It is safe to decrease the full range to fr
      full_range = fr;
      notify_set_full_range(samples_deque.back().counter, full_range);
    }
  }

  // Reduce deque size to no more than delay_to_change+1
  for(int n_remove = samples_deque.size() - delay_to_change - 1; 
      n_remove >= 0;
      n_remove--) {
    // Remove samples from the front of the queue
    samples_deque.pop_front();
  }

  // Signal full scale range change (if )
  return full_range;
}


/*
uint16_t LSM6DSOXAutoRanger::predict_range(LSM6DSOXSampleData *prev_sample, LSM6DSOXSampleData *curr_sample)
{
  // Predict absolute values for X, Y and Z, and find max.
  // Absolute value of current sample will always be <= 32768,
  // prediction may be up to delta_counter * 32768.
  uint32_t max_abs_prediction = abs_prediction(prev_sample->rawXYZ[0], 
                                               curr_sample->rawXYZ[0]);
  max_abs_prediction = std::max(max_abs_prediction, 
                                abs_prediction(prev_sample->rawXYZ[1],
                                               curr_sample->rawXYZ[1]));
  max_abs_prediction = std::max(max_abs_prediction,
                                abs_prediction(prev_sample->rawXYZ[2],
                                               curr_sample->rawXYZ[2]));

  // Scaling up should be performed as soon as possible if the absolute max
  // determined above could overflow the measurement range
  int8_t scale_up_down = 0;
  uint32_t threshold = static_cast<uint32_t>(threshold_up);
  for(; (scale_up_down <= 4) && (max_abs_prediction >= threshold); scale_up_down++) {
    threshold <<= 1;
  }
  if(scale_up_down == 0) { 
    // TODO
    // Scaling down involves finding the max value of a number of consecutive
    // samples, then scale that into the desired range [threshold_down, threshold_up].
    // Restart history after scale change.
    uint32_t threshold = static_cast<uint32_t>(threshold_down);
    for(; (scale_up_down >= -4) && (max_abs_prediction <= threshold); scale_up_down--) {
      threshold >>= 1;
    } 
    
  }
  return scale_up_down;

          Serial.print("LSM6DSOXAutoRanger::predict_range pred=");
          Serial.print(orred_abs_predictions);
          Serial.print(" curr=");
          Serial.print(orred_abs_current);
          Serial.print(" combined=");
          Serial.print(combined_bits, HEX);
          Serial.print(" leading_one=");
          Serial.print(leading_one);
}
*/

inline uint32_t LSM6DSOXAutoRanger::max_abs_value(LSM6DSOXSampleData &sample)
{
  // First cast to int32_t, since abs(-32768) can not be represented in
  // 16 bit 2's complement (signed) integer
  uint32_t max_abs = max(abs(static_cast<int32_t>(sample.rawXYZ[0])), 
                         abs(static_cast<int32_t>(sample.rawXYZ[1])));
  return max(max_abs,    abs(static_cast<int32_t>(sample.rawXYZ[2])));
}

/*
inline uint32_t LSM6DSOXAutoRanger::abs_prediction(int16_t prev_value, int16_t curr_value)
{
  int32_t curr_value_32 = static_cast<int32_t>(curr_value);
  int32_t delta = curr_value_32 - static_cast<int32_t>(prev_value);
  int32_t prediction = curr_value_32 + delta_counter * delta;

  return static_cast<uint32_t>(abs(prediction));
}
*/

/*
uint16_t LSM6DSOXAutoRanger::adjusted_full_range(uint16_t full_range, int8_t range_factor)
{
  int s = v_ranges.size();
  // Linear search for matching full_range
  for(int idx = 0; idx < s; idx++) {
    if(v_ranges[idx] == full_range) { // Found
      // Check for underflow (idx+range_factor < 0)
      if(idx <= (-range_factor))  return v_ranges[0];
      // Check for overflow (idx+range_factor >= s)
      if(idx >= (s-range_factor)) return v_ranges[s-1];
      // Somewhere in range
      return v_ranges[idx + range_factor];
    }
  }
  return 0; // full_range not found in v_ranges (this should not happen!)
}
*/