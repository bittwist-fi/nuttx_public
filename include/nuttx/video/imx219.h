/****************************************************************************
 * include/nuttx/video/imx219.h
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

/* Sony IMX219 8-Mpx CMOS image sensor, MIPI CSI-2.
 *
 * The sensor is controlled only through V4L2: imx219_initialize() returns
 * an imgsensor to pass to capture_register() together with the SoC's
 * MIPI CSI-2 receiver (imgdata).
 *
 * Preconditions before calling imx219_initialize():
 *   - INCK (24 MHz) is running to the sensor.
 *   - Sensor XCLR reset has been released.
 *   - I2C bus master is initialized and passed in.
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_VIDEO_IMX219_H
#define __INCLUDE_NUTTX_VIDEO_IMX219_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

struct i2c_master_s;

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* V4L2_CID_TEST_PATTERN values (sensor register 0x0600). */

enum imx219_test_pattern_e
{
  IMX219_TP_OFF                    = 0,
  IMX219_TP_SOLID_COLOR            = 1,
  IMX219_TP_COLOR_BARS             = 2,
  IMX219_TP_FADE_TO_GREY           = 3,
  IMX219_TP_PN9                    = 4,
  IMX219_TP_16_SPLIT_COLOR_BAR     = 5,
  IMX219_TP_16_SPLIT_INV_COLOR_BAR = 6,
  IMX219_TP_COLUMN_COUNTER         = 7,
  IMX219_TP_INV_COLUMN_COUNTER     = 8,
  IMX219_TP_PN31                   = 9,
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Probe the chip ID on the i2c bus, load the default mode with streaming
 * off and return the sensor as an imgsensor. Returns NULL on failure.
 * Pass the sensor input clock frequency as "inck". This is typically 24MHz
 * on RPi modules, which have an on-board oscillator.
 */

FAR struct imgsensor_s *imx219_initialize(FAR struct i2c_master_s *i2c,
                                          uint32_t inck);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_IMX219_H */
