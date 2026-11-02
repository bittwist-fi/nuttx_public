/****************************************************************************
 * drivers/video/imx219/imx219_modes.c
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

/* IMX219 register tables.
 *
 * Register addresses and field encodings are taken from the public Sony
 * IMX219 register reference. Timing math for the default 1920x1080
 * RAW10 2-lane mode:
 *
 *   INCK               = 24 MHz
 *   PREPLLCK_VT_DIV    = 3       -> 8 MHz
 *   PLL_VT_MPY         = 57      -> 456 MHz
 *   VTSYCK_DIV         = 1
 *   VTPXCK_DIV         = 5       -> VT_PIX_CLK = 91.2 MHz, 2 px/clk
 *   LINE_LENGTH_PCK    = 3448    -> 18.90 us/line; analog 2x2 binning
 *                                   sends 2 CSI-2 lines per line time
 *   FRAME_LENGTH_LINES = 1113    -> 21.04 ms/frame  (~47.5 fps)
 *
 *   PREPLLCK_OP_DIV    = 3
 *   PLL_OP_MPY         = 114     -> 912 MHz
 *   OPSYCK_DIV         = 1
 *   OPPXCK_DIV         = 10      -> OP_PIX_CLK = 91.2 MHz  (RAW10)
 *   MIPI D-PHY         = 912 Mbit/s per lane
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>

#include "imx219_regs.h"
#include "imx219_modes.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Common bring-up: vendor magic access code, MIPI lane count and INCK
 * frequency. Applied once from imx219_init() before any mode-specific
 * register list.
 */

const struct csi_sensor_reg_s g_imx219_common_regs[] =
{
  { IMX219_REG_ACCESS_CODE_A, 0x05 },
  { IMX219_REG_ACCESS_CODE_A, 0x0c },
  { IMX219_REG_ACCESS_CODE_B, 0xff },
  { IMX219_REG_ACCESS_CODE_C, 0xff },
  { IMX219_REG_ACCESS_CODE_A, 0x05 },
  { IMX219_REG_ACCESS_CODE_A, 0x09 },
  { IMX219_REG_CSI_LANE_MODE, 0x01 },
  { IMX219_REG_DPHY_CTRL,     0x00 },
  { IMX219_REG_EXCK_FREQ_MSB, 0x18 },
  { IMX219_REG_EXCK_FREQ_LSB, 0x00 },
};

const uint16_t g_imx219_common_nregs =
  sizeof(g_imx219_common_regs) / sizeof(g_imx219_common_regs[0]);

static const struct csi_sensor_reg_s
imx219_mode_1920x1080_raw10_2lane_regs[] =
{
  { IMX219_REG_X_ADD_STA_MSB, 0x02 },
  { IMX219_REG_X_ADD_STA_LSB, 0xa8 },
  { IMX219_REG_X_ADD_END_MSB, 0x0a },
  { IMX219_REG_X_ADD_END_LSB, 0x27 },
  { IMX219_REG_Y_ADD_STA_MSB, 0x02 },
  { IMX219_REG_Y_ADD_STA_LSB, 0xb4 },
  { IMX219_REG_Y_ADD_END_MSB, 0x06 },
  { IMX219_REG_Y_ADD_END_LSB, 0xeb },
  { IMX219_REG_X_OUT_SIZE_MSB, 0x07 },
  { IMX219_REG_X_OUT_SIZE_LSB, 0x80 },
  { IMX219_REG_Y_OUT_SIZE_MSB, 0x04 },
  { IMX219_REG_Y_OUT_SIZE_LSB, 0x38 },
  { IMX219_REG_X_ODD_INC,      0x01 },
  { IMX219_REG_Y_ODD_INC,      0x01 },
  { IMX219_REG_BINNING_MODE_H, 0x00 },
  { IMX219_REG_BINNING_MODE_V, 0x00 },
  { IMX219_REG_LINE_LENGTH_MSB,  0x0d },
  { IMX219_REG_LINE_LENGTH_LSB,  0x78 },
  { IMX219_REG_FRAME_LENGTH_MSB, 0x04 },
  { IMX219_REG_FRAME_LENGTH_LSB, 0x59 },
  { IMX219_REG_VTPXCK_DIV,        0x05 },
  { IMX219_REG_VTSYCK_DIV,        0x01 },
  { IMX219_REG_PREPLLCK_VT_DIV,   0x03 },
  { IMX219_REG_PLL_VT_MPY_MSB,    0x00 },
  { IMX219_REG_PLL_VT_MPY_LSB,    0x39 },
  { IMX219_REG_PREPLLCK_OP_DIV,   0x03 },
  { IMX219_REG_OPPXCK_DIV,        0x0a },
  { IMX219_REG_OPSYCK_DIV,        0x01 },
  { IMX219_REG_PLL_OP_MPY_MSB,    0x00 },
  { IMX219_REG_PLL_OP_MPY_LSB,    0x72 },
  { IMX219_REG_CSI_DATA_FORMAT_MSB, 0x0a },
  { IMX219_REG_CSI_DATA_FORMAT_LSB, 0x0a },
  { IMX219_REG_COARSE_INT_MSB,  0x02 },
  { IMX219_REG_COARSE_INT_LSB,  0x58 },
  { IMX219_REG_ANA_GAIN_GLOBAL,  0x00 },
  { IMX219_REG_DIG_GAIN_GLOBAL_MSB, 0x01 },
  { IMX219_REG_DIG_GAIN_GLOBAL_LSB, 0x00 },
};

const struct imx219_mode_s g_imx219_mode_1920x1080_raw10_2lane =
{
  .name               = "1920x1080-RAW10-2L",
  .width              = 1920,
  .height             = 1080,
  .bit_depth          = 10,
  .n_lanes            = 2,
  .line_length_pck    = 3448,
  .frame_length_lines = 1113,
  .max_frame_length_lines = IMX219_FRAME_LENGTH_LINES_MAX,
  .vt_pix_clk_hz      = 182400000u,  /* 2 px/clk */
  .rate_factor        = 1,
  .regs               = imx219_mode_1920x1080_raw10_2lane_regs,
  .nregs              = sizeof(imx219_mode_1920x1080_raw10_2lane_regs) /
                        sizeof(imx219_mode_1920x1080_raw10_2lane_regs[0]),
};

static const struct csi_sensor_reg_s
imx219_mode_1640x1232_raw10_2lane_regs[] =
{
  { IMX219_REG_X_ADD_STA_MSB, 0x00 },
  { IMX219_REG_X_ADD_STA_LSB, 0x00 },
  { IMX219_REG_X_ADD_END_MSB, 0x0c },
  { IMX219_REG_X_ADD_END_LSB, 0xcf },
  { IMX219_REG_Y_ADD_STA_MSB, 0x00 },
  { IMX219_REG_Y_ADD_STA_LSB, 0x00 },
  { IMX219_REG_Y_ADD_END_MSB, 0x09 },
  { IMX219_REG_Y_ADD_END_LSB, 0x9f },
  { IMX219_REG_X_OUT_SIZE_MSB, 0x06 },
  { IMX219_REG_X_OUT_SIZE_LSB, 0x68 },
  { IMX219_REG_Y_OUT_SIZE_MSB, 0x04 },
  { IMX219_REG_Y_OUT_SIZE_LSB, 0xd0 },
  { IMX219_REG_X_ODD_INC,      0x01 },
  { IMX219_REG_Y_ODD_INC,      0x01 },
  { IMX219_REG_BINNING_MODE_H, IMX219_BINNING_X2_ANALOG },
  { IMX219_REG_BINNING_MODE_V, IMX219_BINNING_X2_ANALOG },
  { IMX219_REG_LINE_LENGTH_MSB,  0x0d },
  { IMX219_REG_LINE_LENGTH_LSB,  0x78 },
  { IMX219_REG_FRAME_LENGTH_MSB, 0x06 },
  { IMX219_REG_FRAME_LENGTH_LSB, 0xe3 },
  { IMX219_REG_VTPXCK_DIV,        0x05 },
  { IMX219_REG_VTSYCK_DIV,        0x01 },
  { IMX219_REG_PREPLLCK_VT_DIV,   0x03 },
  { IMX219_REG_PLL_VT_MPY_MSB,    0x00 },
  { IMX219_REG_PLL_VT_MPY_LSB,    0x39 },
  { IMX219_REG_PREPLLCK_OP_DIV,   0x03 },
  { IMX219_REG_OPPXCK_DIV,        0x0a },
  { IMX219_REG_OPSYCK_DIV,        0x01 },
  { IMX219_REG_PLL_OP_MPY_MSB,    0x00 },
  { IMX219_REG_PLL_OP_MPY_LSB,    0x72 },
  { IMX219_REG_CSI_DATA_FORMAT_MSB, 0x0a },
  { IMX219_REG_CSI_DATA_FORMAT_LSB, 0x0a },
  { IMX219_REG_COARSE_INT_MSB,   0x03 },
  { IMX219_REG_COARSE_INT_LSB,   0x84 },
  { IMX219_REG_ANA_GAIN_GLOBAL,     0x00 },
  { IMX219_REG_DIG_GAIN_GLOBAL_MSB, 0x01 },
  { IMX219_REG_DIG_GAIN_GLOBAL_LSB, 0x00 },
};

const struct imx219_mode_s g_imx219_mode_1640x1232_raw10_2lane =
{
  .name               = "1640x1232-RAW10-2L-bin2x2",
  .width              = 1640,
  .height             = 1232,
  .bit_depth          = 10,
  .n_lanes            = 2,
  .line_length_pck    = 3448,
  .frame_length_lines = 1763,
  .max_frame_length_lines = IMX219_FRAME_LENGTH_LINES_MAX,
  .vt_pix_clk_hz      = 182400000u,  /* 2 px/clk */
  .rate_factor        = 2,           /* analog binning */
  .regs               = imx219_mode_1640x1232_raw10_2lane_regs,
  .nregs              = sizeof(imx219_mode_1640x1232_raw10_2lane_regs) /
                        sizeof(imx219_mode_1640x1232_raw10_2lane_regs[0]),
};

static const struct csi_sensor_reg_s
imx219_mode_640x480_raw10_2lane_regs[] =
{
  { IMX219_REG_X_ADD_STA_MSB, 0x03 },
  { IMX219_REG_X_ADD_STA_LSB, 0xe8 },
  { IMX219_REG_X_ADD_END_MSB, 0x08 },
  { IMX219_REG_X_ADD_END_LSB, 0xe7 },
  { IMX219_REG_Y_ADD_STA_MSB, 0x02 },
  { IMX219_REG_Y_ADD_STA_LSB, 0xf0 },
  { IMX219_REG_Y_ADD_END_MSB, 0x06 },
  { IMX219_REG_Y_ADD_END_LSB, 0xaf },
  { IMX219_REG_X_OUT_SIZE_MSB, 0x02 },
  { IMX219_REG_X_OUT_SIZE_LSB, 0x80 },
  { IMX219_REG_Y_OUT_SIZE_MSB, 0x01 },
  { IMX219_REG_Y_OUT_SIZE_LSB, 0xe0 },
  { IMX219_REG_X_ODD_INC,      0x01 },
  { IMX219_REG_Y_ODD_INC,      0x01 },
  { IMX219_REG_BINNING_MODE_H, IMX219_BINNING_X2_ANALOG },
  { IMX219_REG_BINNING_MODE_V, IMX219_BINNING_X2_ANALOG },
  { IMX219_REG_LINE_LENGTH_MSB,  0x0d },
  { IMX219_REG_LINE_LENGTH_LSB,  0x78 },
  { IMX219_REG_FRAME_LENGTH_MSB, 0x03 },
  { IMX219_REG_FRAME_LENGTH_LSB, 0x72 },
  { IMX219_REG_VTPXCK_DIV,        0x05 },
  { IMX219_REG_VTSYCK_DIV,        0x01 },
  { IMX219_REG_PREPLLCK_VT_DIV,   0x03 },
  { IMX219_REG_PLL_VT_MPY_MSB,    0x00 },
  { IMX219_REG_PLL_VT_MPY_LSB,    0x39 },
  { IMX219_REG_PREPLLCK_OP_DIV,   0x03 },
  { IMX219_REG_OPPXCK_DIV,        0x0a },
  { IMX219_REG_OPSYCK_DIV,        0x01 },
  { IMX219_REG_PLL_OP_MPY_MSB,    0x00 },
  { IMX219_REG_PLL_OP_MPY_LSB,    0x72 },
  { IMX219_REG_CSI_DATA_FORMAT_MSB, 0x0a },
  { IMX219_REG_CSI_DATA_FORMAT_LSB, 0x0a },
  { IMX219_REG_COARSE_INT_MSB,   0x02 },
  { IMX219_REG_COARSE_INT_LSB,   0x58 },
  { IMX219_REG_ANA_GAIN_GLOBAL,     0x00 },
  { IMX219_REG_DIG_GAIN_GLOBAL_MSB, 0x01 },
  { IMX219_REG_DIG_GAIN_GLOBAL_LSB, 0x00 },
};

const struct imx219_mode_s g_imx219_mode_640x480_raw10_2lane =
{
  .name               = "640x480-RAW10-2L-bin2x2",
  .width              = 640,
  .height             = 480,
  .bit_depth          = 10,
  .n_lanes            = 2,
  .line_length_pck    = 3448,
  .frame_length_lines = 882,
  .max_frame_length_lines = IMX219_FRAME_LENGTH_LINES_MAX,
  .vt_pix_clk_hz      = 182400000u,  /* 2 px/clk */
  .rate_factor        = 2,           /* analog binning */
  .regs               = imx219_mode_640x480_raw10_2lane_regs,
  .nregs              = sizeof(imx219_mode_640x480_raw10_2lane_regs) /
                        sizeof(imx219_mode_640x480_raw10_2lane_regs[0]),
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

const struct imx219_mode_s *imx219_mode_by_id(int id)
{
  switch (id)
    {
      case 0:
        return &g_imx219_mode_1920x1080_raw10_2lane;

      case 1:
        return &g_imx219_mode_1640x1232_raw10_2lane;

      case 2:
        return &g_imx219_mode_640x480_raw10_2lane;

      default:
        return NULL;
    }
}
