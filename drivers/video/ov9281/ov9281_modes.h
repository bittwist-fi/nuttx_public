/****************************************************************************
 * drivers/video/ov9281/ov9281_modes.h
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

#ifndef __DRIVERS_VIDEO_OV9281_OV9281_MODES_H
#define __DRIVERS_VIDEO_OV9281_OV9281_MODES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/video/csi_sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RAW10 timing common to all modes; HTS must match the mode register
 * tables. VTS gives the maximum frame rate of table 2-1 of the datasheet.
 */

#define OV9281_LINE_LENGTH_PCK   1456u        /* HTS, SYS_CLK cycles */
#define OV9281_PIX_CLK_HZ        80000000u    /* SYS_CLK */
#define OV9281_LINK_MBPS         800u         /* per lane */

#define OV9281_VTS_1280X800      910u         /* ~120 fps */
#define OV9281_VTS_1280X720      846u         /* ~130 fps */
#define OV9281_VTS_640X480       611u         /* ~180 fps */
#define OV9281_VTS_640X400       524u         /* ~210 fps */

/* Compile-time description of a sensor mode. The register list sets up
 * windowing, timing and format such that only `MODE_SELECT = 1` is
 * required to start streaming afterwards (PLLs are applied before it).
 */

struct ov9281_mode_s
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

/* Setup applied once right after the soft reset. */

extern const struct csi_sensor_reg_s g_ov9281_global_regs[];
extern const uint16_t                g_ov9281_global_nregs;

/* PLL1/PLL2 for EXTCLK = 24 MHz: 800 Mbps per lane, 80 MHz SYS_CLK. */

extern const struct csi_sensor_reg_s g_ov9281_pll_regs[];
extern const uint16_t                g_ov9281_pll_nregs;

extern const struct ov9281_mode_s g_ov9281_modes[];
extern const unsigned int         g_ov9281_nmodes;

#endif /* __DRIVERS_VIDEO_OV9281_OV9281_MODES_H */
