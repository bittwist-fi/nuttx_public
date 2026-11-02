/****************************************************************************
 * drivers/video/imx219/imx219_regs.h
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

/* IMX219 CCI (I2C) register map. All register addresses are 16-bit,
 * big-endian on the wire. Register data is 8-bit; multi-byte fields
 * are split into consecutive 8-bit registers, MSB first.
 *
 * References:
 *   - Sony IMX219 datasheet
 *   - Sony IMX219 sensor register reference (public)
 ****************************************************************************/

#ifndef __DRIVERS_VIDEO_IMX219_IMX219_REGS_H
#define __DRIVERS_VIDEO_IMX219_IMX219_REGS_H

/* --- Identification --------------------------------------------------- */

#define IMX219_CHIP_ID                  0x0219u

#define IMX219_REG_CHIP_ID_MSB          0x0000
#define IMX219_REG_CHIP_ID_LSB          0x0001

/* --- Mode / stream control ------------------------------------------- */

#define IMX219_REG_MODE_SELECT          0x0100  /* 0 = SW-standby, 1 = stream */
#define IMX219_REG_SW_RESET             0x0103  /* 1 = soft reset */

/* --- Group hold (atomic register-set update) ------------------------- */

#define IMX219_REG_GROUPED_PAR_HOLD     0x0104  /* 1 = hold, 0 = release */

/* --- CSI / MIPI ------------------------------------------------------- */

#define IMX219_REG_CSI_LANE_MODE        0x0114  /* 0 = 4-lane, 1 = 2-lane */
#define IMX219_REG_DPHY_CTRL            0x0128  /* 0 = auto */
#define IMX219_REG_EXCK_FREQ_MSB        0x012a  /* INCK freq in MHz, Q8 */
#define IMX219_REG_EXCK_FREQ_LSB        0x012b

/* --- Exposure / gain ------------------------------------------------- */

#define IMX219_REG_FRAME_LENGTH_MSB     0x0160
#define IMX219_REG_FRAME_LENGTH_LSB     0x0161
#define IMX219_REG_LINE_LENGTH_MSB      0x0162
#define IMX219_REG_LINE_LENGTH_LSB      0x0163

#define IMX219_REG_COARSE_INT_MSB       0x015a  /* exposure, in lines   */
#define IMX219_REG_COARSE_INT_LSB       0x015b

#define IMX219_REG_ANA_GAIN_GLOBAL      0x0157  /* code 0..232 */

#define IMX219_REG_DIG_GAIN_GLOBAL_MSB  0x0158  /* Q8.8, 1.0 = 0x0100 */
#define IMX219_REG_DIG_GAIN_GLOBAL_LSB  0x0159

/* --- Windowing / output size ----------------------------------------- */

#define IMX219_REG_X_ADD_STA_MSB        0x0164
#define IMX219_REG_X_ADD_STA_LSB        0x0165
#define IMX219_REG_X_ADD_END_MSB        0x0166
#define IMX219_REG_X_ADD_END_LSB        0x0167
#define IMX219_REG_Y_ADD_STA_MSB        0x0168
#define IMX219_REG_Y_ADD_STA_LSB        0x0169
#define IMX219_REG_Y_ADD_END_MSB        0x016a
#define IMX219_REG_Y_ADD_END_LSB        0x016b
#define IMX219_REG_X_OUT_SIZE_MSB       0x016c
#define IMX219_REG_X_OUT_SIZE_LSB       0x016d
#define IMX219_REG_Y_OUT_SIZE_MSB       0x016e
#define IMX219_REG_Y_OUT_SIZE_LSB       0x016f

#define IMX219_REG_X_ODD_INC            0x0170
#define IMX219_REG_Y_ODD_INC            0x0171
#define IMX219_REG_BINNING_MODE_H       0x0174
#define IMX219_REG_BINNING_MODE_V       0x0175

#define IMX219_BINNING_NONE             0x00
#define IMX219_BINNING_X2               0x01
#define IMX219_BINNING_X2_ANALOG        0x03  /* 2 output lines per line time */

/* --- Format ---------------------------------------------------------- */

#define IMX219_REG_CSI_DATA_FORMAT_MSB  0x018c  /* 0x0a0a for RAW10 */
#define IMX219_REG_CSI_DATA_FORMAT_LSB  0x018d

/* --- PLL ------------------------------------------------------------- */

#define IMX219_REG_VTPXCK_DIV           0x0301
#define IMX219_REG_VTSYCK_DIV           0x0303
#define IMX219_REG_PREPLLCK_VT_DIV      0x0304
#define IMX219_REG_PREPLLCK_OP_DIV      0x0305
#define IMX219_REG_PLL_VT_MPY_MSB       0x0306
#define IMX219_REG_PLL_VT_MPY_LSB       0x0307
#define IMX219_REG_OPPXCK_DIV           0x0309
#define IMX219_REG_OPSYCK_DIV           0x030b
#define IMX219_REG_PLL_OP_MPY_MSB       0x030c
#define IMX219_REG_PLL_OP_MPY_LSB       0x030d

/* --- Vendor access magic (must be applied before mode regs) ---------- */

#define IMX219_REG_ACCESS_CODE_A        0x30eb
#define IMX219_REG_ACCESS_CODE_B        0x300a
#define IMX219_REG_ACCESS_CODE_C        0x300b

/* --- Test pattern ---------------------------------------------------- */

#define IMX219_REG_TEST_PATTERN_MSB     0x0600  /* pattern id, MSB first */
#define IMX219_REG_TEST_PATTERN_LSB     0x0601
#define IMX219_REG_TP_RED_MSB           0x0602  /* solid color, RAW10 code */
#define IMX219_REG_TP_RED_LSB           0x0603
#define IMX219_REG_TP_GREENR_MSB        0x0604
#define IMX219_REG_TP_GREENR_LSB        0x0605
#define IMX219_REG_TP_BLUE_MSB          0x0606
#define IMX219_REG_TP_BLUE_LSB          0x0607
#define IMX219_REG_TP_GREENB_MSB        0x0608
#define IMX219_REG_TP_GREENB_LSB        0x0609
#define IMX219_REG_TP_WINDOW_X_MSB      0x0620
#define IMX219_REG_TP_WINDOW_X_LSB      0x0621
#define IMX219_REG_TP_WINDOW_Y_MSB      0x0622
#define IMX219_REG_TP_WINDOW_Y_LSB      0x0623
#define IMX219_REG_TP_WINDOW_W_MSB      0x0624
#define IMX219_REG_TP_WINDOW_W_LSB      0x0625
#define IMX219_REG_TP_WINDOW_H_MSB      0x0626
#define IMX219_REG_TP_WINDOW_H_LSB      0x0627
#define IMX219_REG_TP_GREENB_LSB        0x0609

/* --- Static sensor limits (data sheet) ------------------------------- */

#define IMX219_ANA_GAIN_CODE_MIN        0u
#define IMX219_ANA_GAIN_CODE_MAX        232u   /* 10.66x */

#define IMX219_DIG_GAIN_MIN             0x0100u /* Q8.8 => 1.0 */
#define IMX219_DIG_GAIN_MAX             0x0fffu /* ~15.99x       */

#define IMX219_EXPOSURE_LINES_MIN       4u
#define IMX219_EXPOSURE_LINES_MARGIN    4u    /* max = frame_length - margin */
#define IMX219_FRAME_LENGTH_LINES_MAX   0xfffeu

/* Native pixel array is 3280x2464 but active area is 3280x2464 minus OB. */

#define IMX219_NATIVE_WIDTH             3280
#define IMX219_NATIVE_HEIGHT            2464

/* RAW10 white/black levels */

#define IMX219_BLACK_LEVEL              64
#define IMX219_WHITE_LEVEL              1023

#endif /* __DRIVERS_VIDEO_IMX219_IMX219_REGS_H */
