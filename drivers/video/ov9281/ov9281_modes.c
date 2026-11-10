/****************************************************************************
 * drivers/video/ov9281/ov9281_modes.c
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

/* OV9281 register tables.
 *
 * Timing for the RAW10 2-lane modes, EXTCLK = 24 MHz:
 *
 *   PLL1: 24 MHz / 1.5 * 50 = 800 MHz VCO -> 800 Mbit/s per lane,
 *         SYS_CLK = 800 MHz / 5 / 2 = 80 MHz, MIPI_PCLK 100 MHz
 *   HTS                = 728 SYS_CLK -> 9.1 us/line
 *   VTS                = 910 lines for 1280x800 -> 8.28 ms/frame (~120.8 fps)
 *
 * The frame time of table 2-2 (663,208 SYS_CLK) is HTS x 911. The VTS of
 * the other modes is derived from the maximum frame rates of table 2-1.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>

#include "ov9281_regs.h"
#include "ov9281_modes.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct csi_sensor_reg_s g_ov9281_global_regs[] =
{
  { 0x3022, 0x51 },  /* mipi_bit_sel = 10-bit */
  { 0x3039, 0x32 },  /* MIPI enabled, two-lane mode */
  { 0x3662, 0x05 },  /* RAW10 readout (0x07 = RAW8) */
  { 0x4800, 0x10 },  /* MIPI CTRL00: free-running clock, LS/LE packets */
  { 0x4814, 0x6b },  /* long packet data type forced to RAW10 (0x2b) */
  { 0x3001, 0x00 },
  { 0x3004, 0x00 },
  { 0x3005, 0x00 },
  { 0x3006, 0x04 },  /* output enable: ILPWM */
  { 0x3011, 0x0a },
  { 0x3013, 0x18 },
  { 0x301c, 0xf0 },
  { 0x3030, 0x10 },
  { 0x303a, 0x00 },
  { 0x3503, 0x08 },  /* manual AEC/AGC, gain in 1/16 steps */
  { 0x3505, 0x8c },
  { 0x3507, 0x03 },
  { 0x3508, 0x00 },
  { 0x3666, 0x00 },
  { 0x3920, 0xff },
  { 0x4010, 0x40 },
  { 0x4043, 0x40 },
  { 0x4307, 0x30 },
  { 0x4317, 0x00 },
  { 0x4601, 0x04 },
  { 0x470f, 0x00 },
  { 0x4f07, 0x00 },
  { 0x5000, 0x9f },
  { 0x5001, 0x00 },
  { 0x5e00, 0x00 },
  { 0x0101, 0x01 },
  { 0x5a08, 0x84 },
};

const uint16_t g_ov9281_global_nregs =
  sizeof(g_ov9281_global_regs) / sizeof(g_ov9281_global_regs[0]);

/* Table 2-10, "1 megapixel @ 120 fps with 2-lane, 10-bit output". */

const struct csi_sensor_reg_s g_ov9281_pll_regs[] =
{
  { 0x030a, 0x00 },  /* PLL1_pre_div0 /1 */
  { 0x0300, 0x01 },  /* PLL1_pre_div /1.5 */
  { 0x0301, 0x00 },  /* PLL1_loop_div[9:8] */
  { 0x0302, 0x32 },  /* PLL1_loop_div[7:0] = 50 */
  { 0x0305, 0x02 },  /* PLL1_sys_pre_div /5 */
  { 0x0306, 0x01 },  /* PLL1_sys_div /2 */
  { 0x0303, 0x00 },  /* PLL1_M_div /1 */
  { 0x0304, 0x03 },  /* PLL1_MIPI_div /8 */
  { 0x0314, 0x00 },  /* PLL2_pre_div0 */
  { 0x030b, 0x04 },  /* PLL2_pre_div /3 */
  { 0x030c, 0x00 },  /* PLL2_loop_div[9:8] */
  { 0x030d, 0x50 },  /* PLL2_loop_div[7:0] = 80 */
  { 0x030f, 0x03 },  /* PLL2_sys_pre_div /4 */
  { 0x030e, 0x02 },  /* PLL2_sys_div /2 */
  { 0x0313, 0x01 },  /* PLL2_DAC_div /2 */
  { 0x0312, 0x07 },  /* PLL2_SA1_div /8 */
};

const uint16_t g_ov9281_pll_nregs =
  sizeof(g_ov9281_pll_regs) / sizeof(g_ov9281_pll_regs[0]);

/* Window, output size, HTS/VTS, ISP offset, sub-sampling and binning.
 * The array is 1296x816 around the 1280x800 output; the ISP window offsets
 * (in output pixels) select the output inside the array window.
 */

#define OV9281_MODE_REGS(xs, ys, xe, ye, w, h, vts, xo, yo, xinc, yinc, f1, f2) \
  { 0x3800, (xs) >> 8 }, { 0x3801, (xs) & 0xff },     \
  { 0x3802, (ys) >> 8 }, { 0x3803, (ys) & 0xff },     \
  { 0x3804, (xe) >> 8 }, { 0x3805, (xe) & 0xff },     \
  { 0x3806, (ye) >> 8 }, { 0x3807, (ye) & 0xff },     \
  { 0x3808, (w) >> 8 },  { 0x3809, (w) & 0xff },      \
  { 0x380a, (h) >> 8 },  { 0x380b, (h) & 0xff },      \
  { 0x380c, OV9281_LINE_LENGTH_PCK >> 8 },            \
  { 0x380d, OV9281_LINE_LENGTH_PCK & 0xff },          \
  { 0x380e, (vts) >> 8 }, { 0x380f, (vts) & 0xff },   \
  { 0x3810, (xo) >> 8 }, { 0x3811, (xo) & 0xff },     \
  { 0x3812, (yo) >> 8 }, { 0x3813, (yo) & 0xff },     \
  { 0x3814, (xinc) }, { 0x3815, (yinc) },             \
  { 0x3820, (f1) }, { 0x3821, (f2) }

/* Black-level settings for full-resolution and 2x2-binned readout */

#define OV9281_FULL_REGS                              \
  { 0x4003, 0x40 },                                   \
  { 0x4008, 0x04 }, { 0x4009, 0x0b },                 \
  { 0x400c, 0x00 }, { 0x400d, 0x07 }

#define OV9281_BIN_REGS                               \
  { 0x4008, 0x02 }, { 0x4009, 0x05 },                 \
  { 0x400c, 0x00 }, { 0x400d, 0x03 }

/* Full array, no sub-sampling: X 0..1295, Y 0..815. */

static const struct csi_sensor_reg_s ov9281_mode_1280x800_regs[] =
{
  OV9281_MODE_REGS(0, 0, 1295, 815, 1280, 800, OV9281_VTS_1280X800,
                   8, 8, 0x11, 0x11, 0x40, 0x00),
  OV9281_FULL_REGS
};

/* Crop to rows 40..775, centred; fewer rows give a shorter frame. */

static const struct csi_sensor_reg_s ov9281_mode_1280x720_regs[] =
{
  OV9281_MODE_REGS(0, 40, 1295, 775, 1280, 720, OV9281_VTS_1280X720,
                   8, 8, 0x11, 0x11, 0x40, 0x00),
  OV9281_FULL_REGS
};

/* Centre crop: columns 320..975, rows 160..655 of the array. */

static const struct csi_sensor_reg_s ov9281_mode_640x480_regs[] =
{
  OV9281_MODE_REGS(320, 160, 975, 655, 640, 480, OV9281_VTS_640X480,
                   8, 8, 0x11, 0x11, 0x40, 0x00),
  OV9281_FULL_REGS
};

/* Full field of view, 2x2 binning; ISP offsets are half of the full mode */

static const struct csi_sensor_reg_s ov9281_mode_640x400_regs[] =
{
  OV9281_MODE_REGS(0, 0, 1295, 815, 640, 400, OV9281_VTS_640X400,
                   4, 4, 0x31, 0x22, 0x60, 0x01),
  OV9281_BIN_REGS
};

#define OV9281_MODE(n, w, h, vts, r)                       \
  {                                                        \
    .name                   = n,                           \
    .width                  = w,                           \
    .height                 = h,                           \
    .line_length_pck        = OV9281_LINE_LENGTH_PCK,      \
    .frame_length_lines     = vts,                         \
    .min_frame_length_lines = vts,                         \
    .vt_pix_clk_hz          = OV9281_PIX_CLK_HZ,           \
    .link_mbps_per_lane     = OV9281_LINK_MBPS,            \
    .regs                   = r,                           \
    .nregs                  = sizeof(r) / sizeof((r)[0]),  \
  }

/* The first entry is the default mode. */

const struct ov9281_mode_s g_ov9281_modes[] =
{
  OV9281_MODE("1280x800-RAW10-2L", 1280, 800, OV9281_VTS_1280X800,
              ov9281_mode_1280x800_regs),
  OV9281_MODE("1280x720-RAW10-2L", 1280, 720, OV9281_VTS_1280X720,
              ov9281_mode_1280x720_regs),
  OV9281_MODE("640x480-RAW10-2L", 640, 480, OV9281_VTS_640X480,
              ov9281_mode_640x480_regs),
  OV9281_MODE("640x400-RAW10-2L-bin2x2", 640, 400, OV9281_VTS_640X400,
              ov9281_mode_640x400_regs),
};

const unsigned int g_ov9281_nmodes =
  sizeof(g_ov9281_modes) / sizeof(g_ov9281_modes[0]);
