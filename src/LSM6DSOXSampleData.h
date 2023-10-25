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

#ifndef LSM6DSOXSampleData_H
#define LSM6DSOXSampleData_H

#include <Arduino.h>

struct LSM6DSOXSampleData {
  int32_t XYZ[3];       // Fixed point data: decimal point between bits 15 and 16
  int16_t rawXYZ[3];
  uint16_t fullRange;
  bool valid;
};

#endif // LSM6DSOXSampleData_H
