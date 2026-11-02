/****************************************************************************
 * drivers/video/imx219/imx219_modes.h
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

#ifndef __DRIVERS_VIDEO_IMX219_IMX219_MODES_H
#define __DRIVERS_VIDEO_IMX219_IMX219_MODES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/video/csi_sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Compile-time description of a sensor mode. The register list must set
 * up windowing, PLL, format, line/frame length and exposure such that
 * only `MODE_SELECT = 1` is required to start streaming afterwards.
 */

struct imx219_mode_s
{
  const char             *name;
  uint16_t                width;
  uint16_t                height;
  uint8_t                 bit_depth;
  uint8_t                 n_lanes;
  uint16_t                line_length_pck;
  uint16_t                frame_length_lines;
  uint16_t                max_frame_length_lines;
  uint32_t                vt_pix_clk_hz;
  uint8_t                 rate_factor;  /* CSI-2 lines per sensor line */
  const struct csi_sensor_reg_s *regs;
  uint16_t                nregs;
};

/* Common (magic-access / lane / INCK) register block, applied before any
 * mode-specific register list.
 */

extern const struct csi_sensor_reg_s g_imx219_common_regs[];
extern const uint16_t                g_imx219_common_nregs;

/* All modes below are RAW10 on 2 MIPI lanes, INCK = 24 MHz. */

extern const struct imx219_mode_s g_imx219_mode_1920x1080_raw10_2lane;
extern const struct imx219_mode_s g_imx219_mode_1640x1232_raw10_2lane;
extern const struct imx219_mode_s g_imx219_mode_640x480_raw10_2lane;

enum imx219_mode_id_e
{
  IMX219_MODE_1920X1080 = 0,   /* full-res 1080p crop, no binning       */
  IMX219_MODE_1640X1232,       /* 2x2 analog binning, full field-of-view */
  IMX219_MODE_VGA_640X480,     /* 2x2 analog binning, centre crop        */
  IMX219_MODE_COUNT
};

/* Look up a mode by id. Returns NULL if the id is out of range. */

const struct imx219_mode_s *imx219_mode_by_id(int id);

#endif /* __DRIVERS_VIDEO_IMX219_IMX219_MODES_H */
