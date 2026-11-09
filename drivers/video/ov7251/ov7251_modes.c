/****************************************************************************
 * drivers/video/ov7251/ov7251_modes.c
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

/* OV7251 register tables.
 *
 * Timing for the 640x480 RAW10 1-lane mode, EXTCLK = 24 MHz:
 *
 *   PLL1: 24 MHz / 2 * 50 = 600 MHz VCO -> 600 Mbit/s per lane
 *   PLL2: 24 MHz / 2 * 40 = 480 MHz VCO -> SYS_CLK 48 MHz
 *   HTS                = 928 SYS_CLK -> 19.33 us/line
 *   VTS                = 516 lines   -> 9.98 ms/frame (~100 fps)
 *
 * The frame time of table 2-2 (478,848 SYS_CLK) is HTS x VTS.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>

#include "ov7251_regs.h"
#include "ov7251_modes.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct csi_sensor_reg_s g_ov7251_global_regs[] =
{
  { 0x303b, 0x02 },
  { 0x3503, 0x03 },  /* AEC/AGC manual: no auto module, bits must stay 1 */
  { 0x3509, 0x10 },  /* gain convert enable: linear gain, 0x10 = 1.0x */
};

const uint16_t g_ov7251_global_nregs =
  sizeof(g_ov7251_global_regs) / sizeof(g_ov7251_global_regs[0]);

/* Table 2-10, "VGA @ 100 fps with 1 lane, 10-bit output" */

const struct csi_sensor_reg_s g_ov7251_pll_regs[] =
{
  { OV7251_REG_PLL1_PRE_DIV,  0x02 },  /* /2 */
  { OV7251_REG_PLL1_MULT,     0x32 },  /* x50 */
  { OV7251_REG_PLL1_DIVIDER,  0x01 },
  { OV7251_REG_PLL1_PIX_DIV,  0x0a },  /* /10 */
  { OV7251_REG_PLL1_MIPI_DIV, 0x01 },  /* /1 */
  { OV7251_REG_PLL2_PRE_DIV,  0x04 },  /* /2 */
  { OV7251_REG_PLL2_MULT,     0x28 },  /* x40 */
  { OV7251_REG_PLL2_DIVIDER,  0x00 },
  { OV7251_REG_PLL2_SYS_DIV,  0x05 },  /* /10 */
  { OV7251_REG_PLL2_ADC_DIV,  0x04 },  /* /2 */
};

const uint16_t g_ov7251_pll_nregs =
  sizeof(g_ov7251_pll_regs) / sizeof(g_ov7251_pll_regs[0]);

static const struct csi_sensor_reg_s ov7251_mode_640x480_raw10_regs[] =
{
  { 0x3005, 0x00 },
  { 0x3012, 0xc0 },
  { 0x3013, 0xd2 },
  { 0x3014, 0x04 },
  { 0x3016, 0x10 },
  { 0x3017, 0x00 },
  { 0x3018, 0x00 },
  { 0x301a, 0x00 },
  { 0x301b, 0x00 },
  { 0x301c, 0x00 },
  { 0x3023, 0x05 },
  { 0x3037, 0xf0 },
  { 0x3106, 0xda },
  { 0x3600, 0x1c },
  { 0x3636, 0x00 },
  { 0x3662, 0x01 },  /* bit 1 = 0: RAW10 */
  { 0x3663, 0x70 },
  { 0x3666, 0x0a },
  { 0x37a8, 0x01 },
  { 0x37a9, 0xc0 },
  { 0x3800, 0x00 },  /* array window X 4..651, Y 4..491 */
  { 0x3801, 0x04 },
  { 0x3802, 0x00 },
  { 0x3803, 0x04 },
  { 0x3804, 0x02 },
  { 0x3805, 0x8b },
  { 0x3806, 0x01 },
  { 0x3807, 0xeb },
  { 0x3808, 0x02 },  /* output 640 */
  { 0x3809, 0x80 },
  { 0x380a, 0x01 },  /* output 480 */
  { 0x380b, 0xe0 },
  { 0x380c, 0x03 },  /* HTS 928 */
  { 0x380d, 0xa0 },
  { 0x380e, OV7251_FRAME_LINES_DEF >> 8 },
  { 0x380f, OV7251_FRAME_LINES_DEF & 0xff },
  { 0x3810, 0x00 },
  { 0x3811, 0x04 },
  { 0x3812, 0x00 },
  { 0x3813, 0x05 },
  { 0x3814, 0x11 },
  { 0x3815, 0x11 },
  { 0x3820, 0x40 },
  { 0x3821, 0x00 },
  { 0x382f, 0x0e },
  { 0x3832, 0x00 },
  { 0x3833, 0x05 },
  { 0x3834, 0x00 },
  { 0x3835, 0x0c },
  { 0x3837, 0x00 },
  { 0x3b80, 0x00 },
  { 0x3b81, 0xa5 },
  { 0x3b82, 0x10 },
  { 0x3b83, 0x00 },
  { 0x3b84, 0x08 },
  { 0x3b85, 0x00 },
  { 0x3b86, 0x01 },
  { 0x3b87, 0x00 },
  { 0x3b88, 0x00 },
  { 0x3b89, 0x00 },
  { 0x3b8a, 0x00 },
  { 0x3b8b, 0x05 },
  { 0x3b8c, 0x00 },
  { 0x3b8d, 0x00 },
  { 0x3b8e, 0x00 },
  { 0x3b8f, 0x1a },
  { 0x3b94, 0x05 },
  { 0x3b95, 0xf2 },
  { 0x3b96, 0x40 },
  { 0x3c00, 0x89 },
  { 0x3c01, 0x63 },
  { 0x3c02, 0x01 },
  { 0x3c03, 0x00 },
  { 0x3c04, 0x00 },
  { 0x3c05, 0x03 },
  { 0x3c06, 0x00 },
  { 0x3c07, 0x06 },
  { 0x3c0c, 0x01 },
  { 0x3c0d, 0xd0 },
  { 0x3c0e, 0x02 },
  { 0x3c0f, 0x0a },
  { 0x4001, 0x42 },
  { 0x4004, 0x04 },
  { 0x4005, 0x00 },
  { 0x404e, 0x01 },
  { 0x4300, 0xff },
  { 0x4301, 0x00 },
  { 0x4315, 0x00 },
  { 0x4501, 0x48 },
  { 0x4600, 0x00 },
  { 0x4601, 0x4e },
  { 0x4800, 0x54 },  /* MIPI CTRL 00: send LS/LE short packets */
  { 0x4801, 0x0f },
  { 0x4806, 0x0f },
  { 0x4819, 0xaa },
  { 0x4823, 0x3e },
  { 0x4a47, 0x7f },
  { 0x4a49, 0xf0 },
  { 0x5000, 0x85 },
  { 0x5001, 0x80 },
};

const struct ov7251_mode_s g_ov7251_mode_640x480_raw10 =
{
  .name                   = "640x480-RAW10-1L",
  .width                  = 640,
  .height                 = 480,
  .line_length_pck        = OV7251_LINE_LENGTH_PCK,
  .frame_length_lines     = OV7251_FRAME_LINES_DEF,
  .min_frame_length_lines = OV7251_FRAME_LINES_MIN,
  .vt_pix_clk_hz          = OV7251_PIX_CLK_HZ,
  .link_mbps_per_lane     = OV7251_LINK_MBPS,
  .regs                   = ov7251_mode_640x480_raw10_regs,
  .nregs                  = sizeof(ov7251_mode_640x480_raw10_regs) /
                            sizeof(ov7251_mode_640x480_raw10_regs[0]),
};
