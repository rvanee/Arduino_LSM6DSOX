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

#ifndef AUTORANGER_H
#define AUTORANGER_H

#include <Arduino.h>


#define AUTORANGE_ALPHA_DEFAULT   3

class AutoRanger {
  public:
    AutoRanger();

    int8_t predict_range(SampleData *prev_sample, SampleData *curr_sample);

  private:
    void reset(uint8_t alpha = AUTORANGE_ALPHA_DEFAULT);

    uint32_t abs_prediction(int16_t prev_value, int16_t curr_value);

  private:
    uint8_t alpha;
};

#endif // AUTORANGER_H
