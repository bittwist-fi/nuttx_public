/****************************************************************************
 * drivers/video/ov7251/ov7251_modes.h
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

#ifndef __DRIVERS_VIDEO_OV7251_OV7251_MODES_H
#define __DRIVERS_VIDEO_OV7251_OV7251_MODES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/video/csi_sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 640x480 RAW10 timing; must match HTS/VTS in the mode register table. */

#define OV7251_LINE_LENGTH_PCK   928u         /* HTS, SYS_CLK cycles */
#define OV7251_FRAME_LINES_DEF   516u         /* VTS, ~100 fps */
#define OV7251_FRAME_LINES_MIN   516u         /* fastest rate in table 2-1 */
#define OV7251_PIX_CLK_HZ        48000000u    /* SYS_CLK */
#define OV7251_LINK_MBPS         600u         /* per lane */

/* Compile-time description of a sensor mode. The register list sets up
 * windowing, format, timing and BLC such that only `MODE_SELECT = 1` is
 * required to start streaming afterwards (PLLs are applied before it).
 */

struct ov7251_mode_s
{
  const char                    *name;
  uint16_t                       width;
  uint16_t                       height;
  uint16_t                       line_length_pck;
  uint16_t                       frame_length_lines;
  uint16_t                       min_frame_length_lines;
  uint32_t                       vt_pix_clk_hz;
  uint32_t                       link_mbps_per_lane;
  const struct csi_sensor_reg_s *regs;
  uint16_t                       nregs;
};

/* Vendor setup applied once right after the soft reset. */

extern const struct csi_sensor_reg_s g_ov7251_global_regs[];
extern const uint16_t                g_ov7251_global_nregs;

/* PLL1/PLL2 for EXTCLK = 24 MHz: 600 Mbps per lane, 48 MHz pixel clock. */

extern const struct csi_sensor_reg_s g_ov7251_pll_regs[];
extern const uint16_t                g_ov7251_pll_nregs;

extern const struct ov7251_mode_s g_ov7251_mode_640x480_raw10;

#endif /* __DRIVERS_VIDEO_OV7251_OV7251_MODES_H */
