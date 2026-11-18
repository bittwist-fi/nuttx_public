/****************************************************************************
 * drivers/video/imx335.c
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

/* Sony IMX335 5 MP image sensor, 2-lane MIPI CSI-2, RAW10 RGGB Bayer.
 *
 * The register values come from the Zephyr IMX335 driver
 * (drivers/video/imx335.c, Apache-2.0): 2 data lanes, 10 bits and the
 * default VMAX of 4500 lines.
 *
 * Two RAW10 (V4L2_PIX_FMT_SRGGB10) modes are offered:
 *
 *   1296x972    2/2 binning, 25, 30, 50 or 60 frames per second; the default
 *   2592x1944   all-pixel scan, 25 or 30 frames per second
 *
 * The frame rate is set with the line length (HMAX).  A capture interface
 * with an image signal processor (for example the STM32N6 DCMIPP)
 * demosaics and scales the frame and registers its own image sensor in
 * front of this one.  There is no automatic exposure in the sensor; the
 * exposure time and the gain can be changed with
 * V4L2_CID_EXPOSURE_ABSOLUTE and V4L2_CID_GAIN (0.3 dB steps) or
 * V4L2_CID_ANALOGUE_GAIN (linear Q16.16, analogue range only).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/csi_sensor.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imx335.h>
#include <nuttx/video/video.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMX335_I2C_ADDR          0x1a
#define IMX335_I2C_FREQ          400000

/* Output sizes of the two modes */

#define IMX335_WIDTH             2592
#define IMX335_HEIGHT            1944
#define IMX335_BIN_WIDTH         1296
#define IMX335_BIN_HEIGHT        972

/* Frame timing: VMAX (reset value, never changed) lines per frame; the
 * line length HMAX sets the frame rate.  The first shutter line that is
 * allowed is lower without binning.
 */

#define IMX335_FPS_DEFAULT       30
#define IMX335_VMAX              4500
#define IMX335_SHR0_MIN          9
#define IMX335_SHR0_MIN_BIN      17

/* The pixel clock of the line length is 74.25 MHz. */

#define IMX335_VT_PIX_CLK_HZ     74250000

/* Number of line lengths in an exposure time unit of 100 us */

#define IMX335_CLKS_PER_EXPOSURE (IMX335_VT_PIX_CLK_HZ / 10000)

/* Lane rate measured from the line burst length; equals the PLL */

#define IMX335_LINK_MBPS         1188
#define IMX335_LANES             2
#define IMX335_CSI_DT_RAW10      0x2b

/* Gain in 0.3 dB steps, 0 to 72 dB (Zephyr IMX335_GAIN_UNIT_MDB) */

#define IMX335_GAIN_MAX          240

/* Steps up to 30 dB are analogue; one step is 10^(0.3/20) in Q16.16 */

#define IMX335_AGAIN_MAX         100
#define IMX335_GAIN_STEP_Q16     67840

/* Registers */

#define IMX335_REG_STANDBY       0x3000  /* Bit 0: standby */
#define IMX335_REG_REGHOLD       0x3001  /* Bit 0: hold register updates */
#define IMX335_REG_VMAX          0x3030  /* 24 bits: lines per frame */
#define IMX335_REG_HMAX          0x3034  /* 16 bits: line length */
#define IMX335_REG_SHR0          0x3058  /* 24 bits: shutter line */
#define IMX335_REG_GAIN          0x30e8  /* 16 bits: gain, 0.3 dB steps */

#define IMX335_STANDBY           0x01
#define IMX335_OPERATING         0x00

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imx335_inck_s
{
  uint32_t freq;
  struct csi_sensor_reg_s regs[7];
};

struct imx335_mode_s
{
  uint16_t width;
  uint16_t height;
  uint8_t max_fps;
  uint8_t shr0_min;
  uint8_t hmax_per_line;              /* HMAX periods per output line */
  uint8_t pre_active_lines;           /* Line periods from FS to image */
  FAR const struct csi_sensor_reg_s *regs;
  size_t nregs;
};

struct imx335_rate_s
{
  uint8_t fps;
  uint16_t hmax;                      /* Line length, 74.25 MHz clocks */
};

struct imx335_dev_s
{
  struct csi_sensor_s csi;            /* Bound sensor */
  struct csi_sensor_cci_s cci;        /* Little endian registers */
  mutex_t lock;                       /* Serializes the sensor access */
  FAR const struct imx335_inck_s *inck;
  FAR const struct imx335_mode_s *mode;
  FAR const struct imx335_rate_s *rate;
  uint32_t exposure;                  /* Exposure time in 100 us units */
  uint32_t gain;                      /* Gain in 0.3 dB steps */
  bool streaming;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool imx335_is_available(FAR void *arg);
static int imx335_init(FAR void *arg);
static int imx335_uninit(FAR void *arg);
static int imx335_start_capture(FAR void *arg,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                FAR imgsensor_format_t *datafmts,
                                FAR imgsensor_interval_t *interval);
static int imx335_stop_capture(FAR void *arg,
                               imgsensor_stream_type_t type);
static int imx335_get_mipi_csi_config(FAR void *arg,
                                      FAR const imgsensor_format_t *fmt,
                                      FAR const imgsensor_interval_t *ival,
                                      FAR imgsensor_mipi_csi_cfg_t *config);
static int imx335_get_supported_value(FAR void *arg, uint32_t id,
                                      FAR imgsensor_supported_value_t *val);
static int imx335_get_value(FAR void *arg, uint32_t id, uint32_t size,
                            FAR imgsensor_value_t *value);
static int imx335_set_value(FAR void *arg, uint32_t id, uint32_t size,
                            imgsensor_value_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Initial settings with the sensor in standby (Zephyr
 * imx335_init_params), then the fixed value registers (Zephyr
 * imx335_fixed_regs).  Multi-byte registers are little endian.
 */

static const struct csi_sensor_reg_s g_imx335_init_regs[] =
{
  { 0x3000, 0x01 },
  { 0x3002, 0x00 },
  { 0x302c, 0x3c },
  { 0x302d, 0x00 },
  { 0x302e, 0x20 },
  { 0x302f, 0x0a },
  { 0x3056, 0x98 },
  { 0x3057, 0x07 },
  { 0x3074, 0xc8 },
  { 0x3075, 0x00 },
  { 0x3076, 0x30 },
  { 0x3077, 0x0f },
  { 0x31a1, 0x00 },
  { 0x3288, 0x21 },
  { 0x328a, 0x02 },
  { 0x3414, 0x05 },
  { 0x3416, 0x18 },
  { 0x3648, 0x01 },
  { 0x364a, 0x04 },
  { 0x364c, 0x04 },
  { 0x3678, 0x01 },
  { 0x367c, 0x31 },
  { 0x367e, 0x31 },
  { 0x3706, 0x10 },
  { 0x3708, 0x03 },
  { 0x3714, 0x02 },
  { 0x3715, 0x02 },
  { 0x3716, 0x01 },
  { 0x3717, 0x03 },
  { 0x371c, 0x3d },
  { 0x371d, 0x3f },
  { 0x372c, 0x00 },
  { 0x372d, 0x00 },
  { 0x372e, 0x46 },
  { 0x372f, 0x00 },
  { 0x3730, 0x89 },
  { 0x3731, 0x00 },
  { 0x3732, 0x08 },
  { 0x3733, 0x01 },
  { 0x3734, 0xfe },
  { 0x3735, 0x05 },
  { 0x3740, 0x02 },
  { 0x375d, 0x00 },
  { 0x375e, 0x00 },
  { 0x375f, 0x11 },
  { 0x3760, 0x01 },
  { 0x3768, 0x1b },
  { 0x3769, 0x1b },
  { 0x376a, 0x1b },
  { 0x376b, 0x1b },
  { 0x376c, 0x1a },
  { 0x376d, 0x17 },
  { 0x376e, 0x0f },
  { 0x3776, 0x00 },
  { 0x3777, 0x00 },
  { 0x3778, 0x46 },
  { 0x3779, 0x00 },
  { 0x377a, 0x89 },
  { 0x377b, 0x00 },
  { 0x377c, 0x08 },
  { 0x377d, 0x01 },
  { 0x377e, 0x23 },
  { 0x377f, 0x02 },
  { 0x3780, 0xd9 },
  { 0x3781, 0x03 },
  { 0x3782, 0xf5 },
  { 0x3783, 0x06 },
  { 0x3784, 0xa5 },
  { 0x3788, 0x0f },
  { 0x378a, 0xd9 },
  { 0x378b, 0x03 },
  { 0x378c, 0xeb },
  { 0x378d, 0x05 },
  { 0x378e, 0x87 },
  { 0x378f, 0x06 },
  { 0x3790, 0xf5 },
  { 0x3792, 0x43 },
  { 0x3794, 0x7a },
  { 0x3796, 0xa1 },
  { 0x37b0, 0x36 },
  { 0x3a00, 0x01 },
};

/* All-pixel scan without binning (Zephyr imx335_bin_none), which also
 * resets everything that the binned mode changes.
 */

static const struct csi_sensor_reg_s g_imx335_full_regs[] =
{
  { 0x3018, 0x00 },
  { 0x302e, 0x20 },
  { 0x302f, 0x0a },
  { 0x3056, 0x98 },
  { 0x3057, 0x07 },
  { 0x3072, 0x28 },
  { 0x3073, 0x00 },
  { 0x3074, 0xc8 },
  { 0x3075, 0x00 },
  { 0x3076, 0x30 },
  { 0x3077, 0x0f },
  { 0x3199, 0x00 },
  { 0x3300, 0x00 },
  { 0x3078, 0x01 },
  { 0x3079, 0x02 },
  { 0x307a, 0xff },
  { 0x307b, 0x02 },
  { 0x307c, 0x00 },
  { 0x307d, 0x00 },
  { 0x307e, 0x00 },
  { 0x307f, 0x00 },
  { 0x3080, 0x01 },
  { 0x3081, 0x02 },
  { 0x3082, 0xff },
  { 0x3083, 0x02 },
  { 0x3084, 0x00 },
  { 0x3085, 0x00 },
  { 0x3086, 0x00 },
  { 0x3087, 0x00 },
  { 0x30a4, 0x33 },
  { 0x30a8, 0x10 },
  { 0x30a9, 0x04 },
  { 0x30ac, 0x00 },
  { 0x30ad, 0x00 },
  { 0x30b0, 0x10 },
  { 0x30b1, 0x08 },
  { 0x30b4, 0x00 },
  { 0x30b5, 0x00 },
  { 0x30b6, 0x00 },
  { 0x30b7, 0x00 },
  { 0x3112, 0x08 },
  { 0x3113, 0x00 },
  { 0x3116, 0x08 },
  { 0x3117, 0x00 },
};

/* 2/2 binning (Zephyr imx335_bin_2x2), 1296x972 */

static const struct csi_sensor_reg_s g_imx335_bin_regs[] =
{
  { 0x3018, 0x01 },
  { 0x302e, 0x08 },
  { 0x302f, 0x0a },
  { 0x3056, 0xcc },
  { 0x3057, 0x03 },
  { 0x3072, 0x30 },
  { 0x3073, 0x00 },
  { 0x3074, 0xc8 },
  { 0x3075, 0x00 },
  { 0x3076, 0x30 },
  { 0x3077, 0x0f },
  { 0x3199, 0x30 },
  { 0x3300, 0x01 },
  { 0x3078, 0x04 },
  { 0x3079, 0xfd },
  { 0x307a, 0x04 },
  { 0x307b, 0xfe },
  { 0x307c, 0x04 },
  { 0x307d, 0xfb },
  { 0x307e, 0x04 },
  { 0x307f, 0x02 },
  { 0x3080, 0x04 },
  { 0x3081, 0xfd },
  { 0x3082, 0x04 },
  { 0x3083, 0xfe },
  { 0x3084, 0x04 },
  { 0x3085, 0xfb },
  { 0x3086, 0x04 },
  { 0x3087, 0x02 },
  { 0x30a4, 0x77 },
  { 0x30a8, 0x20 },
  { 0x30a9, 0x00 },
  { 0x30ac, 0x08 },
  { 0x30ad, 0x08 },
  { 0x30b0, 0x20 },
  { 0x30b1, 0x00 },
  { 0x30b4, 0x10 },
  { 0x30b5, 0x10 },
  { 0x30b6, 0x00 },
  { 0x30b7, 0x00 },
  { 0x3112, 0x10 },
  { 0x3113, 0x00 },
  { 0x3116, 0x10 },
  { 0x3117, 0x00 },
};

/* 2 lanes with 10 bits (Zephyr imx335_mode_2l_10b) */

static const struct csi_sensor_reg_s g_imx335_lane_regs[] =
{
  { 0x3050, 0x00 },
  { 0x319d, 0x00 },
  { 0x341c, 0xff },
  { 0x341d, 0x01 },
  { 0x3a01, 0x01 },
};

/* Binned mode first: it is the default */

static const struct imx335_mode_s g_imx335_mode[] =
{
  {
    IMX335_BIN_WIDTH, IMX335_BIN_HEIGHT, 60, IMX335_SHR0_MIN_BIN, 4, 14,
    g_imx335_bin_regs, nitems(g_imx335_bin_regs),
  },
  {
    IMX335_WIDTH, IMX335_HEIGHT, 30, IMX335_SHR0_MIN, 2, 22,
    g_imx335_full_regs, nitems(g_imx335_full_regs),
  },
};

/* The line length is the same with and without binning; the first is the
 * default.
 */

static const struct imx335_rate_s g_imx335_rate[] =
{
  { 30, 0x0226 },
  { 25, 0x0294 },
  { 50, 0x014a },
  { 60, 0x0113 },
};

/* Input clock (INCK) dependent settings (Zephyr imx335_inck_*) */

static const struct imx335_inck_s g_imx335_inck[] =
{
  {
    74250000,
    {
      { 0x300c, 0xb6 },
      { 0x300d, 0x7f },
      { 0x314c, 0x80 },
      { 0x314d, 0x00 },
      { 0x315a, 0x03 },
      { 0x3168, 0x68 },
      { 0x316a, 0x7f },
    },
  },
  {
    27000000,
    {
      { 0x300c, 0x42 },
      { 0x300d, 0x2e },
      { 0x314c, 0xb0 },
      { 0x314d, 0x00 },
      { 0x315a, 0x02 },
      { 0x3168, 0x8f },
      { 0x316a, 0x7e },
    },
  },
  {
    24000000,
    {
      { 0x300c, 0x3b },
      { 0x300d, 0x2a },
      { 0x314c, 0xc6 },
      { 0x314d, 0x00 },
      { 0x315a, 0x02 },
      { 0x3168, 0xa0 },
      { 0x316a, 0x7e },
    },
  },
  {
    18000000,
    {
      { 0x300c, 0x2d },
      { 0x300d, 0x1f },
      { 0x314c, 0x84 },
      { 0x314d, 0x00 },
      { 0x315a, 0x01 },
      { 0x3168, 0x6b },
      { 0x316a, 0x7d },
    },
  },
  {
    6000000,
    {
      { 0x300c, 0x0f },
      { 0x300d, 0x0b },
      { 0x314c, 0xc6 },
      { 0x314d, 0x00 },
      { 0x315a, 0x00 },
      { 0x3168, 0xa0 },
      { 0x316a, 0x7c },
    },
  },
};

static const struct csi_sensor_ops_s g_imx335_ops =
{
  .is_available        = imx335_is_available,
  .init                = imx335_init,
  .uninit              = imx335_uninit,
  .start_capture       = imx335_start_capture,
  .stop_capture        = imx335_stop_capture,
  .get_mipi_csi_config = imx335_get_mipi_csi_config,
  .get_supported_value = imx335_get_supported_value,
  .get_value           = imx335_get_value,
  .set_value           = imx335_set_value,
};

static const imgsensor_mipi_csi_mode_t g_imx335_modes[] =
{
  {
    .pixelformat = V4L2_PIX_FMT_SRGGB10,
    .width       = IMX335_BIN_WIDTH,
    .height      = IMX335_BIN_HEIGHT,
    .interval    =
      {
        .numerator   = 1,
        .denominator = IMX335_FPS_DEFAULT,
      },
  },
  {
    .pixelformat = V4L2_PIX_FMT_SRGGB10,
    .width       = IMX335_WIDTH,
    .height      = IMX335_HEIGHT,
    .interval    =
      {
        .numerator   = 1,
        .denominator = IMX335_FPS_DEFAULT,
      },
  },
};

static const struct v4l2_fmtdesc g_imx335_fmtdescs[] =
{
  {
    .pixelformat = V4L2_PIX_FMT_SRGGB10,
    .description = "RAW10 RGGB",
  },
};

static const struct v4l2_frmsizeenum g_imx335_frmsizes[] =
{
  {
    .type = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete =
      {
        .width  = IMX335_BIN_WIDTH,
        .height = IMX335_BIN_HEIGHT,
      },
  },
  {
    .type = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete =
      {
        .width  = IMX335_WIDTH,
        .height = IMX335_HEIGHT,
      },
  },
};

/* All rates of g_imx335_rate; 50 and 60 are only possible when binning */

static const struct v4l2_frmivalenum g_imx335_frmintervals[] =
{
  {
    .type = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete = { .numerator = 1, .denominator = 30 },
  },
  {
    .type = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete = { .numerator = 1, .denominator = 25 },
  },
  {
    .type = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete = { .numerator = 1, .denominator = 50 },
  },
  {
    .type = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete = { .numerator = 1, .denominator = 60 },
  },
};

static const struct csi_sensor_config_s g_imx335_config =
{
  .name             = "IMX335",
  .modes            = g_imx335_modes,
  .nmodes           = nitems(g_imx335_modes),
  .fmtdescs         = g_imx335_fmtdescs,
  .fmtdescs_num     = nitems(g_imx335_fmtdescs),
  .frmsizes         = g_imx335_frmsizes,
  .frmsizes_num     = nitems(g_imx335_frmsizes),
  .frmintervals     = g_imx335_frmintervals,
  .frmintervals_num = nitems(g_imx335_frmintervals),
  .ops              = &g_imx335_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx335_find_mode
 ****************************************************************************/

static FAR const struct imx335_mode_s *imx335_find_mode(uint32_t width,
                                                        uint32_t height)
{
  int i;

  for (i = 0; i < nitems(g_imx335_mode); i++)
    {
      if (g_imx335_mode[i].width == width &&
          g_imx335_mode[i].height == height)
        {
          return &g_imx335_mode[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: imx335_find_rate
 *
 * Description:
 *   The line length of a frame interval that the mode can do; the default
 *   rate when no interval is given.
 *
 ****************************************************************************/

static FAR const struct imx335_rate_s *
imx335_find_rate(FAR const struct imx335_mode_s *mode,
                 FAR const imgsensor_interval_t *interval)
{
  int i;

  if (interval == NULL || interval->numerator == 0 ||
      interval->denominator == 0)
    {
      return &g_imx335_rate[0];
    }

  for (i = 0; i < nitems(g_imx335_rate); i++)
    {
      if (g_imx335_rate[i].fps <= mode->max_fps &&
          interval->numerator * g_imx335_rate[i].fps ==
          interval->denominator)
        {
          return &g_imx335_rate[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: imx335_maxexposure
 *
 * Description:
 *   The longest exposure time in 100 us units: the frame less the first
 *   shutter line.
 *
 ****************************************************************************/

static uint32_t imx335_maxexposure(FAR const struct imx335_dev_s *priv)
{
  return (IMX335_VMAX - priv->mode->shr0_min) * priv->rate->hmax /
         IMX335_CLKS_PER_EXPOSURE;
}

/****************************************************************************
 * Name: imx335_setexposure
 *
 * Description:
 *   Program the shutter for an exposure time in 100 us units.  One line
 *   lasts HMAX clocks of 74.25 MHz.
 *
 ****************************************************************************/

static int imx335_setexposure(FAR struct imx335_dev_s *priv,
                              uint32_t exposure)
{
  uint32_t lines;
  uint32_t shr;
  int ret;

  lines = exposure * IMX335_CLKS_PER_EXPOSURE / priv->rate->hmax;
  if (lines < 1)
    {
      lines = 1;
    }

  if (lines > IMX335_VMAX - priv->mode->shr0_min)
    {
      lines = IMX335_VMAX - priv->mode->shr0_min;
    }

  shr = IMX335_VMAX - lines;

  ret = csi_sensor_cci_write8(&priv->cci, IMX335_REG_REGHOLD, 1);
  if (ret >= 0)
    {
      ret = csi_sensor_cci_write24(&priv->cci, IMX335_REG_SHR0, shr);
      csi_sensor_cci_write8(&priv->cci, IMX335_REG_REGHOLD, 0);
    }

  if (ret >= 0)
    {
      priv->exposure = exposure;
    }

  return ret;
}

/****************************************************************************
 * Name: imx335_gain_q16
 ****************************************************************************/

static uint32_t imx335_gain_q16(uint32_t code)
{
  uint64_t gain = 1 << 16;

  while (code-- > 0)
    {
      gain = gain * IMX335_GAIN_STEP_Q16 >> 16;
    }

  return (uint32_t)gain;
}

/****************************************************************************
 * Name: imx335_gain_code
 *
 * Description:
 *   The analogue gain step nearest to a linear Q16.16 gain.
 *
 ****************************************************************************/

static uint32_t imx335_gain_code(uint32_t gain_q16)
{
  uint64_t gain = 1 << 16;
  uint64_t next;
  uint32_t code;

  for (code = 0; code < IMX335_AGAIN_MAX; code++)
    {
      next = gain * IMX335_GAIN_STEP_Q16 >> 16;
      if (2 * (uint64_t)gain_q16 < gain + next)
        {
          break;
        }

      gain = next;
    }

  return code;
}

/****************************************************************************
 * Name: imx335_setgain
 ****************************************************************************/

static int imx335_setgain(FAR struct imx335_dev_s *priv, uint32_t gain)
{
  int ret;

  ret = csi_sensor_cci_write8(&priv->cci, IMX335_REG_REGHOLD, 1);
  if (ret >= 0)
    {
      ret = csi_sensor_cci_write16(&priv->cci, IMX335_REG_GAIN, gain);
      csi_sensor_cci_write8(&priv->cci, IMX335_REG_REGHOLD, 0);
    }

  if (ret >= 0)
    {
      priv->gain = gain;
    }

  return ret;
}

/****************************************************************************
 * Name: imx335_setmode
 *
 * Description:
 *   Program the readout mode and the line length of the frame rate with
 *   the sensor in standby, then the exposure again, as the line length
 *   changes the unit of the shutter.
 *
 ****************************************************************************/

static int imx335_setmode(FAR struct imx335_dev_s *priv,
                          FAR const struct imx335_mode_s *mode,
                          FAR const struct imx335_rate_s *rate)
{
  int ret;

  ret = csi_sensor_cci_write_list(&priv->cci, mode->regs, mode->nregs);
  if (ret >= 0)
    {
      ret = csi_sensor_cci_write8(&priv->cci, IMX335_REG_REGHOLD, 1);
    }

  if (ret >= 0)
    {
      ret = csi_sensor_cci_write16(&priv->cci, IMX335_REG_HMAX, rate->hmax);
      csi_sensor_cci_write8(&priv->cci, IMX335_REG_REGHOLD, 0);
    }

  if (ret >= 0)
    {
      priv->mode = mode;
      priv->rate = rate;
      ret = imx335_setexposure(priv, priv->exposure);
    }

  return ret;
}

/****************************************************************************
 * Name: imx335_is_available
 ****************************************************************************/

static bool imx335_is_available(FAR void *arg)
{
  FAR struct imx335_dev_s *priv = arg;
  uint32_t vmax = 0;
  int ret;

  /* There is no ID register; a successful read of VMAX shows the sensor
   * answers on the bus.
   */

  nxmutex_lock(&priv->lock);
  ret = csi_sensor_cci_read24(&priv->cci, IMX335_REG_VMAX, &vmax);
  nxmutex_unlock(&priv->lock);
  return ret >= 0;
}

/****************************************************************************
 * Name: imx335_init
 ****************************************************************************/

static int imx335_init(FAR void *arg)
{
  FAR struct imx335_dev_s *priv = arg;
  int ret;

  nxmutex_lock(&priv->lock);
  ret = csi_sensor_cci_write_list(&priv->cci, g_imx335_init_regs,
                                  nitems(g_imx335_init_regs));
  if (ret >= 0)
    {
      ret = csi_sensor_cci_write_list(&priv->cci, g_imx335_lane_regs,
                                      nitems(g_imx335_lane_regs));
    }

  if (ret >= 0)
    {
      ret = csi_sensor_cci_write_list(&priv->cci, priv->inck->regs,
                                      nitems(priv->inck->regs));
    }

  if (ret >= 0)
    {
      ret = imx335_setgain(priv, priv->gain);
    }

  if (ret >= 0)
    {
      ret = imx335_setmode(priv, &g_imx335_mode[0], &g_imx335_rate[0]);
    }

  if (ret < 0)
    {
      snerr("ERROR: IMX335 initialization failed: %d\n", ret);
    }

  priv->streaming = false;
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: imx335_uninit
 ****************************************************************************/

static int imx335_uninit(FAR void *arg)
{
  FAR struct imx335_dev_s *priv = arg;
  int ret;

  nxmutex_lock(&priv->lock);
  priv->streaming = false;
  ret = csi_sensor_cci_write8(&priv->cci, IMX335_REG_STANDBY,
                              IMX335_STANDBY);
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: imx335_get_mipi_csi_config
 ****************************************************************************/

static int imx335_get_mipi_csi_config(FAR void *arg,
                                      FAR const imgsensor_format_t *fmt,
                                      FAR const imgsensor_interval_t *ival,
                                      FAR imgsensor_mipi_csi_cfg_t *config)
{
  FAR const struct imx335_mode_s *mode;
  FAR const struct imx335_rate_s *rate;

  mode = fmt == NULL ? NULL : imx335_find_mode(fmt->width, fmt->height);
  if (mode == NULL)
    {
      return -ENOTSUP;
    }

  rate = imx335_find_rate(mode, ival);
  if (rate == NULL)
    {
      return -ERANGE;
    }

  memset(config, 0, sizeof(*config));
  config->link_mbps_per_lane = IMX335_LINK_MBPS;
  config->line_length_pck    = mode->hmax_per_line * rate->hmax;
  config->frame_length_lines = IMX335_VMAX / mode->hmax_per_line;
  config->vt_pix_clk_hz      = IMX335_VT_PIX_CLK_HZ;
  config->n_lanes            = IMX335_LANES;
  config->data_type          = IMX335_CSI_DT_RAW10;
  config->pre_active_lines   = mode->pre_active_lines;

  config->line_sync          = IMGSENSOR_LINE_SYNC_NONE;
  return OK;
}

/****************************************************************************
 * Name: imx335_start_capture
 ****************************************************************************/

static int imx335_start_capture(FAR void *arg,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmts,
                                FAR imgsensor_format_t *datafmts,
                                FAR imgsensor_interval_t *interval)
{
  FAR struct imx335_dev_s *priv = arg;
  FAR const struct imx335_mode_s *mode;
  FAR const struct imx335_rate_s *rate;
  int ret = OK;

  if (nr_datafmts < 1 || datafmts == NULL)
    {
      return -EINVAL;
    }

  mode = imx335_find_mode(datafmts[IMGSENSOR_FMT_MAIN].width,
                          datafmts[IMGSENSOR_FMT_MAIN].height);
  if (mode == NULL)
    {
      return -ENOTSUP;
    }

  rate = imx335_find_rate(mode, interval);
  if (rate == NULL)
    {
      return -ERANGE;
    }

  nxmutex_lock(&priv->lock);

  /* The mode is only changed in standby; the capture framework may have
   * left the sensor streaming (see imx335_stop_capture).
   */

  if (priv->streaming && (mode != priv->mode || rate != priv->rate))
    {
      ret = csi_sensor_cci_write8(&priv->cci, IMX335_REG_STANDBY,
                                  IMX335_STANDBY);
      priv->streaming = false;
    }

  if (ret >= 0 && !priv->streaming)
    {
      if (mode != priv->mode || rate != priv->rate)
        {
          ret = imx335_setmode(priv, mode, rate);
        }

      if (ret >= 0)
        {
          ret = csi_sensor_cci_write8(&priv->cci, IMX335_REG_STANDBY,
                                      IMX335_OPERATING);
        }

      if (ret >= 0)
        {
          /* Settling time after leaving standby, as in the Zephyr
           * driver.
           */

          nxsig_usleep(20);
          priv->streaming = true;
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: imx335_stop_capture
 ****************************************************************************/

static int imx335_stop_capture(FAR void *arg,
                               imgsensor_stream_type_t type)
{
  FAR struct imx335_dev_s *priv = arg;
  int ret = OK;

  /* The capture framework may stop the stream from the frame interrupt
   * when it runs out of buffers.  I2C is not possible there; keep the
   * sensor running, the capture interface drops the frames.
   */

  if (up_interrupt_context())
    {
      return OK;
    }

  nxmutex_lock(&priv->lock);
  if (priv->streaming)
    {
      priv->streaming = false;
      ret = csi_sensor_cci_write8(&priv->cci, IMX335_REG_STANDBY,
                                  IMX335_STANDBY);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: imx335_get_supported_value
 ****************************************************************************/

static int imx335_get_supported_value(FAR void *arg, uint32_t id,
                                      FAR imgsensor_supported_value_t *val)
{
  FAR struct imx335_dev_s *priv = arg;
  uint32_t maximum;

  switch (id)
    {
      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        nxmutex_lock(&priv->lock);
        maximum = imx335_maxexposure(priv);
        nxmutex_unlock(&priv->lock);

        val->type                  = IMGSENSOR_CTRL_TYPE_INTEGER;
        val->u.range.minimum       = 1;
        val->u.range.maximum       = maximum;
        val->u.range.step          = 1;
        val->u.range.default_value = MIN(CONFIG_VIDEO_IMX335_EXPOSURE,
                                         maximum);
        return OK;

      case IMGSENSOR_ID_GAIN:
        val->type                  = IMGSENSOR_CTRL_TYPE_INTEGER;
        val->u.range.minimum       = 0;
        val->u.range.maximum       = IMX335_GAIN_MAX;
        val->u.range.step          = 1;
        val->u.range.default_value = CONFIG_VIDEO_IMX335_GAIN;
        return OK;

      case IMGSENSOR_ID_ANALOGUE_GAIN:
        val->type                  = IMGSENSOR_CTRL_TYPE_INTEGER;
        val->u.range.minimum       = imx335_gain_q16(0);
        val->u.range.maximum       = imx335_gain_q16(IMX335_AGAIN_MAX);
        val->u.range.step          = 1;
        val->u.range.default_value =
          imx335_gain_q16(MIN(CONFIG_VIDEO_IMX335_GAIN, IMX335_AGAIN_MAX));
        return OK;

      default:
        return -EINVAL;
    }
}

/****************************************************************************
 * Name: imx335_get_value
 ****************************************************************************/

static int imx335_get_value(FAR void *arg, uint32_t id, uint32_t size,
                            FAR imgsensor_value_t *value)
{
  FAR struct imx335_dev_s *priv = arg;
  int ret = OK;

  nxmutex_lock(&priv->lock);
  switch (id)
    {
      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        value->value32 = priv->exposure;
        break;

      case IMGSENSOR_ID_GAIN:
        value->value32 = priv->gain;
        break;

      case IMGSENSOR_ID_ANALOGUE_GAIN:
        value->value32 = imx335_gain_q16(priv->gain);
        break;

      default:
        ret = -EINVAL;
        break;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: imx335_set_value
 ****************************************************************************/

static int imx335_set_value(FAR void *arg, uint32_t id, uint32_t size,
                            imgsensor_value_t value)
{
  FAR struct imx335_dev_s *priv = arg;
  int ret;

  switch (id)
    {
      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        nxmutex_lock(&priv->lock);
        if (value.value32 < 1 || value.value32 > imx335_maxexposure(priv))
          {
            nxmutex_unlock(&priv->lock);
            return -EINVAL;
          }

        ret = imx335_setexposure(priv, value.value32);
        nxmutex_unlock(&priv->lock);
        return ret;

      case IMGSENSOR_ID_GAIN:
        if (value.value32 < 0 || value.value32 > IMX335_GAIN_MAX)
          {
            return -EINVAL;
          }

        nxmutex_lock(&priv->lock);
        ret = imx335_setgain(priv, value.value32);
        nxmutex_unlock(&priv->lock);
        return ret;

      case IMGSENSOR_ID_ANALOGUE_GAIN:
        if (value.value32 <= 0)
          {
            return -EINVAL;
          }

        nxmutex_lock(&priv->lock);
        ret = imx335_setgain(priv, imx335_gain_code(value.value32));
        nxmutex_unlock(&priv->lock);
        return ret;

      default:
        return -EINVAL;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx335_initialize
 ****************************************************************************/

FAR struct imgsensor_s *imx335_initialize(FAR struct i2c_master_s *i2c,
                                          uint32_t inck)
{
  FAR struct imx335_dev_s *priv;
  FAR struct imgsensor_s *sensor;
  int i;

  if (i2c == NULL)
    {
      return NULL;
    }

  priv = kmm_zalloc(sizeof(struct imx335_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  for (i = 0; i < nitems(g_imx335_inck); i++)
    {
      if (g_imx335_inck[i].freq == inck)
        {
          priv->inck = &g_imx335_inck[i];
          break;
        }
    }

  if (priv->inck == NULL)
    {
      snerr("ERROR: unsupported input clock %" PRIu32 "\n", inck);
      kmm_free(priv);
      return NULL;
    }

  priv->cci.i2c  = i2c;
  priv->cci.addr = IMX335_I2C_ADDR;
  priv->cci.freq = IMX335_I2C_FREQ;
  priv->cci.le   = true;
  priv->mode     = &g_imx335_mode[0];
  priv->rate     = &g_imx335_rate[0];
  nxmutex_init(&priv->lock);
  priv->exposure = CONFIG_VIDEO_IMX335_EXPOSURE;
  priv->gain     = CONFIG_VIDEO_IMX335_GAIN;

  sensor = imx335_is_available(priv) ?
           csi_sensor_bind(&g_imx335_config, priv, &priv->csi) : NULL;
  if (sensor == NULL)
    {
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
    }

  return sensor;
}
