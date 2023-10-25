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

#ifndef LSM6DSOXAUTORANGER_H
#define LSM6DSOXAUTORANGER_H

#include <vector>
#include <deque>
using namespace std;

#include <Arduino.h>


/* These constants define autorange behaviour. They require some trial
   and error to optimize.
   Initial delay is a guess for the roundtrip delay, expressed in number
   of samples, between a range change and the first measurement using
   that range.
   Alpha is a small positive integer used for linear extrapolation
   (estimate of next value). Make it too small and the LSM6DSOXAutoRanger will
   adapt too slowly, resulting in overflow/saturation. Too high, and
   the system may oscillate.
   Threshold up is the absolute value for the estimated value above
   which autorange will 'gear up'.
   Threshold down is the absolute value for the estimated value below
   which autorange will 'gear down'. It should be smaller than half
   threshold up, to allow for some hysteresis to prevent oscillation.
 */
// 
#define LSM6DSOXAUTORANGER_INITIAL_DELAY  5
#define LSM6DSOXAUTORANGER_THRESHOLD_UP   32000
#define LSM6DSOXAUTORANGER_THRESHOLD_DOWN 15000

// Forward declarations to break dependency cycle
struct LSM6DSOXSampleData;

struct LSM6DSOXAutoRangerSample {
    LSM6DSOXSampleData  sample;
    uint32_t            counter;
    uint16_t            optimal_full_range;
}

class LSM6DSOXAutoRanger {
  public:
    LSM6DSOXAutoRanger(const vectorOfFloatsAndBits& v);

    void reset(
      uint16_t threshold_up = LSM6DSOXAUTORANGER_THRESHOLD_UP,
      uint16_t threshold_down = LSM6DSOXAUTORANGER_THRESHOLD_DOWN
    );

    void notify_set_full_range(uint32_t current_counter, uint16_t new_full_range);
    void notify_new_full_range(uint32_t current_counter, uint16_t new_full_range);

    uint16_t add_and_check_sample(uint32_t counter, LSM6DSOXSampleData &sample);
    uint16_t check_underflow();
    //uint16_t predict_range(LSM6DSOXSampleData *prev_sample, LSM6DSOXSampleData *curr_sample);

  private:
    uint32_t max_abs_value(LSM6DSOXSampleData &sample);
    //uint32_t abs_prediction(int16_t prev_value, int16_t curr_value);
    //uint16_t adjusted_full_range(uint16_t full_range, int8_t range_factor);

  private:
    // Configuration
    uint16_t  threshold_up;
    uint16_t  threshold_down;

    // Short history of samples
    deque<LSM6DSOXAutoRangerSample> samples_deque;

    // Last reported full range
    uint16_t  current_full_range;

    // Pending full range change
    bool      new_full_range_pending;
    uint16_t  new_full_range;
    uint32_t  counter_at_change;
    uint16_t  delay_to_change;
};

#endif // LSM6DSOXAUTORANGER_H
