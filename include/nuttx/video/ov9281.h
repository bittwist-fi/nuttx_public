/****************************************************************************
 * include/nuttx/video/ov9281.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* OmniVision OV9281 1-Mpixel global-shutter monochrome image sensor,
 * MIPI CSI-2 (2 data lanes, RAW10).
 *
 * The sensor is controlled only through V4L2: ov9281_initialize() returns
 * an imgsensor to pass to capture_register() together with the SoC's
 * MIPI CSI-2 receiver (imgdata).
 *
 * Preconditions before calling ov9281_initialize():
 *   - EXTCLK (24 MHz) is running to the sensor.
 *   - Sensor XSHUTDOWN has been released.
 *   - I2C bus master is initialized and passed in.
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_VIDEO_OV9281_H
#define __INCLUDE_NUTTX_VIDEO_OV9281_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include <nuttx/video/imgsensor.h>

struct i2c_master_s;

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* V4L2_CID_TEST_PATTERN values (sensor register 0x5e00 bit 7). */

enum ov9281_test_pattern_e
{
  OV9281_TP_OFF        = 0,
  OV9281_TP_COLOR_BARS = 1,
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Probe the chip ID at (i2c, addr), load the default mode with streaming
 * off and return the sensor as an imgsensor. Returns NULL on failure.
 * Pass the sensor input clock frequency as "inck". This is typically 24MHz
 * on RPi modules, which have an on-board oscillator.
 */

FAR struct imgsensor_s *ov9281_initialize(FAR struct i2c_master_s *i2c,
                                          uint32_t inck);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_OV9281_H */
