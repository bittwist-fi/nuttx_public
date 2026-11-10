/****************************************************************************
 * drivers/video/ov9281/ov9281_regs.h
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

/* OV9281 SCCB register map (OV9281 datasheet, preliminary v1.01).
 * Register addresses are 16-bit big-endian, data is 8-bit. Multi-byte
 * fields are split over consecutive registers, MSB first.
 */

#ifndef __DRIVERS_VIDEO_OV9281_OV9281_REGS_H
#define __DRIVERS_VIDEO_OV9281_OV9281_REGS_H

/* --- Identification --------------------------------------------------- */

#define OV9281_CHIP_ID                  0x9281u

#define OV9281_REG_CHIP_ID_HIGH         0x300a
#define OV9281_REG_CHIP_ID_LOW          0x300b
#define OV9281_REG_REVISION             0x300c

/* --- Mode / stream control ------------------------------------------- */

#define OV9281_REG_MODE_SELECT          0x0100  /* 0 = SW-standby, 1 = stream */
#define OV9281_REG_SW_RESET             0x0103  /* 1 = soft reset */

/* --- Exposure / gain ------------------------------------------------- */

/* Exposure is a 20-bit value with 4 fraction bits: rows << 4, spread over
 * 0x3500[3:0], 0x3501 and 0x3502.
 */

#define OV9281_REG_EXPO_0               0x3500

/* Real-gain format (0x3503[2] = 0): 0x3508[4:0]:0x3509, 1.0x = 0x10. */

#define OV9281_REG_GAIN_HIGH            0x3508

/* --- Timing ----------------------------------------------------------- */

#define OV9281_REG_VTS_HIGH             0x380e  /* 0x380e/0x380f */

/* --- Test pattern ----------------------------------------------------- */

#define OV9281_REG_PRE_CTRL00           0x5e00
#define OV9281_PRE_CTRL00_COLOR_BAR     0x80

/* --- Static sensor limits --------------------------------------------- */

#define OV9281_GAIN_CODE_MIN            0x10u   /* 1.0x */
#define OV9281_GAIN_CODE_MAX            0xffu   /* gain[7:0] field, 15.9x */

#define OV9281_EXPOSURE_LINES_MIN       1u

/* The datasheet gives no exposure limit; keep clear of the frame end. */

#define OV9281_FRAME_LENGTH_LINES_MAX   0xffffu

/* Black target level register default (0x4002:0x4003), 10-bit data. */

#define OV9281_BLACK_LEVEL              16

#endif /* __DRIVERS_VIDEO_OV9281_OV9281_REGS_H */
