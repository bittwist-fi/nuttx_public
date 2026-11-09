/****************************************************************************
 * drivers/video/ov7251/ov7251_regs.h
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

/* OV7251 SCCB register map (OV7750/OV7251 datasheet rev 1F, v2.12).
 * Register addresses are 16-bit big-endian, data is 8-bit. Multi-byte
 * fields are split over consecutive registers, MSB first.
 */

#ifndef __DRIVERS_VIDEO_OV7251_OV7251_REGS_H
#define __DRIVERS_VIDEO_OV7251_OV7251_REGS_H

/* --- Identification --------------------------------------------------- */

#define OV7251_CHIP_ID                  0x7750u  /* OV7750 reads the same */

#define OV7251_REG_CHIP_ID_HIGH         0x300a
#define OV7251_REG_CHIP_ID_LOW          0x300b
#define OV7251_REG_REVISION             0x300c

/* --- Mode / stream control ------------------------------------------- */

#define OV7251_REG_MODE_SELECT          0x0100  /* 0 = SW-standby, 1 = stream */
#define OV7251_REG_SW_RESET             0x0103  /* 1 = soft reset */

/* --- Exposure / gain ------------------------------------------------- */

/* Exposure in rows, left-shifted by 4: [3:0] of 0x3500, 0x3501, [7:4] of
 * 0x3502 hold exposure[15:12], [11:4], [3:0].
 */

#define OV7251_REG_EXPO_0               0x3500
#define OV7251_REG_EXPO_1               0x3501
#define OV7251_REG_EXPO_2               0x3502

/* Linear gain, 1.0x = 0x10 (0x3509[4] = 1, set by the mode table). */

#define OV7251_REG_GAIN_HIGH            0x350a  /* [1:0] = gain[9:8] */
#define OV7251_REG_GAIN_LOW             0x350b

/* --- Timing ----------------------------------------------------------- */

#define OV7251_REG_HTS_HIGH             0x380c
#define OV7251_REG_VTS_HIGH             0x380e  /* 0x380e/0x380f */

/* --- BLC / test pattern ---------------------------------------------- */

#define OV7251_REG_BLC_TARGET           0x4009
#define OV7251_REG_PRE_ISP_00           0x5e00
#define OV7251_PRE_ISP_00_COLOR_BAR     0x80

/* --- PLL -------------------------------------------------------------- */

#define OV7251_REG_PLL2_PRE_DIV         0x3098
#define OV7251_REG_PLL2_MULT            0x3099
#define OV7251_REG_PLL2_SYS_DIV         0x309a
#define OV7251_REG_PLL2_ADC_DIV         0x309b
#define OV7251_REG_PLL2_DIVIDER         0x309d
#define OV7251_REG_PLL1_PIX_DIV         0x30b0
#define OV7251_REG_PLL1_DIVIDER         0x30b1
#define OV7251_REG_PLL1_MULT            0x30b3
#define OV7251_REG_PLL1_PRE_DIV         0x30b4
#define OV7251_REG_PLL1_MIPI_DIV        0x30b5

/* --- Static sensor limits (data sheet) ------------------------------- */

#define OV7251_GAIN_CODE_MIN            0x10u   /* 1.0x */
#define OV7251_GAIN_CODE_MAX            0x3ffu

#define OV7251_EXPOSURE_LINES_MIN       1u
#define OV7251_EXPOSURE_LINES_MARGIN    20u     /* max = frame_length - 20 */
#define OV7251_FRAME_LENGTH_LINES_MAX   0xffffu

/* BLC target is 10-bit data; register default 0x10. */

#define OV7251_BLACK_LEVEL              16

#endif /* __DRIVERS_VIDEO_OV7251_OV7251_REGS_H */
