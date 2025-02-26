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

#include <algorithm> // std::sort


LSM6DSOXAutoRanger::LSM6DSOXAutoRanger(const vectorOfFloatsAndBits& v) {
  v_ranges = &v;

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

  // If current full range is uninitialized, use full range from
  // the first sample provided to do that
  if(current_full_range == 0) {
    current_full_range = full_range;
  }

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
  int s = (*v_ranges).size();
  // Linear search for matching full_range
  for(int idx = 0; idx < s; idx++) {
    if(static_cast<uint16_t>((*v_ranges)[idx].first) == full_range) { // Found
      int idx_scaled;
      // Find range for which max_abs < threshold_up
      // NOTE that this assumes that each step up in range
      // doubles it by a factor of 2
      for(idx_scaled = idx; 
          (max_abs >= threshold_up) && (idx_scaled < (s-1));
          idx_scaled++, max_abs >>= 1);
      if(idx_scaled > idx) {
        full_range = static_cast<uint16_t>((*v_ranges)[idx_scaled].first);
        break;
      }

      // No matching higher range found, so try and see
      // if a lower range would do: find range for which 
      // max_abs >= threshold_down
      // NOTE that this assumes that each step up in range
      // doubles it by a factor of 2
      for(idx_scaled = idx; 
          (max_abs < threshold_down) && (idx_scaled > 0); 
          idx_scaled--, max_abs <<= 1);
      if(idx_scaled < idx) {
        full_range = static_cast<uint16_t>((*v_ranges)[idx_scaled].first);
      }
      break;
    }
  }

  // Store new sample in deque
  samples_deque.push_back(
    LSM6DSOXAutoRangerSample{sample, counter, full_range});

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

  // Signal full scale range change (0 means no change)
  return full_range;
}

inline uint32_t LSM6DSOXAutoRanger::max_abs_value(LSM6DSOXSampleData &sample)
{
  // First cast to int32_t, since abs(-32768) can not be represented in
  // 16 bit 2's complement (signed) integer
  uint32_t max_abs = max(static_cast<uint32_t>(abs(static_cast<int32_t>(sample.rawXYZ[0]))), 
                         static_cast<uint32_t>(abs(static_cast<int32_t>(sample.rawXYZ[1]))));
  return max(max_abs,    static_cast<uint32_t>(abs(static_cast<int32_t>(sample.rawXYZ[2]))));
}