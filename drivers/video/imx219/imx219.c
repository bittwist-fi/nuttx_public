/****************************************************************************
 * drivers/video/imx219/imx219.c
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

/* Sony IMX219 MIPI CSI-2 image sensor, exposed as a V4L2 imgsensor.
 * See include/nuttx/video/imx219.h for the public API.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <sys/videoio.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/video/csi_sensor.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imx219.h>

#include "imx219_regs.h"
#include "imx219_modes.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMX219_I2C_ADDR          0x10
#define IMX219_I2C_FREQ          400000

/* Compile-time default startup mode, selected via Kconfig choice. */

#if defined(CONFIG_VIDEO_IMX219_DEFMODE_1640x1232)
#  define IMX219_DEFAULT_MODE_ID  IMX219_MODE_1640X1232
#elif defined(CONFIG_VIDEO_IMX219_DEFMODE_VGA)
#  define IMX219_DEFAULT_MODE_ID  IMX219_MODE_VGA_640X480
#else
#  define IMX219_DEFAULT_MODE_ID  IMX219_MODE_1920X1080
#endif

/* Native RGGB: no mirror/flip (0x0172 = 0) and all crop starts are even.
 * Verified on real hardware.
 */

#define IMX219_ADV_FOURCC       V4L2_PIX_FMT_SRGGB10

/* CSI-2 data type for RAW10. */

#define IMX219_CSI_DT_RAW10     0x2b

/* Lines from FS to the first image line: 2 embedded-data lines (DT 0x12)
 * plus one more, measured with real images (the test pattern starts earlier)
 */

#define IMX219_PRE_ACTIVE_LINES 3u

/* MIPI D-PHY per-lane link rate for the 2-lane RAW10 modes. */

#define IMX219_LINK_MBPS_PER_LANE  912u

/* Timing for the 2-lane RAW10 modes. */

#define IMX219_LINE_LENGTH_PCK       3448u

/* VT_PIX_CLK 91.2 MHz at 2 px/clk */

#define IMX219_PIX_CLK_HZ            182400000u

#define IMX219_1080P_WIDTH           1920
#define IMX219_1080P_HEIGHT          1080
#define IMX219_1080P_FRAME_LINES     1113u
#define IMX219_1080P_FRAME_INTERVAL_NUM \
  (IMX219_LINE_LENGTH_PCK * IMX219_1080P_FRAME_LINES)
#define IMX219_1080P_FRAME_INTERVAL_DEN \
  IMX219_PIX_CLK_HZ

#define IMX219_VGA_WIDTH             640
#define IMX219_VGA_HEIGHT            480
#define IMX219_VGA_FRAME_LINES       882u
#define IMX219_VGA_FRAME_INTERVAL_NUM \
  (IMX219_LINE_LENGTH_PCK * IMX219_VGA_FRAME_LINES)
#define IMX219_VGA_FRAME_INTERVAL_DEN \
  IMX219_PIX_CLK_HZ

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Capabilities of the currently loaded mode */

struct imx219_caps_s
{
  uint32_t min_exposure_us;
  uint32_t max_exposure_us;
  uint32_t min_again_q16;
  uint32_t max_again_q16;
  uint32_t min_dgain_q16;
  uint32_t max_dgain_q16;
};

struct imx219_dev_s
{
  struct imgsensor_s          sensor;   /* must be first */
  struct csi_sensor_cci_s     cci;
  bool                        initialised;
  bool                        streaming;
  mutex_t                     lock;

  const struct imx219_mode_s *mode;
  uint32_t                    frame_length_lines;
  uint8_t                     test_pattern;   /* last requested 0x0601 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  imx219_soft_reset(struct imx219_dev_s *dev);
static int  imx219_apply_mode(struct imx219_dev_s *dev,
                              const struct imx219_mode_s *mode);
static uint8_t imx219_gain_q16_to_code(uint32_t gain_q16);
static uint32_t imx219_us_to_lines(const struct imx219_mode_s *mode,
                                   uint32_t frame_length_lines,
                                   uint32_t us);
static uint32_t imx219_interval_to_frame_length(
                              const struct imx219_mode_s *mode,
                              uint32_t numerator,
                              uint32_t denominator);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* --- Software reset & mode application ------------------------------ */

static int imx219_soft_reset(struct imx219_dev_s *dev)
{
  int ret;

  ret = csi_sensor_cci_write8(&dev->cci, IMX219_REG_SW_RESET, 0x01);
  if (ret < 0)
    {
      return ret;
    }

  up_mdelay(10);
  return OK;
}

static int imx219_apply_mode(struct imx219_dev_s *dev,
                             const struct imx219_mode_s *mode)
{
  int ret;

  /* Ensure stream is off before touching mode registers. */

  ret = csi_sensor_cci_write8(&dev->cci, IMX219_REG_MODE_SELECT, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  ret = csi_sensor_cci_write_list(&dev->cci, g_imx219_common_regs,
                                  g_imx219_common_nregs);
  if (ret < 0)
    {
      return ret;
    }

  ret = csi_sensor_cci_write_list(&dev->cci, mode->regs, mode->nregs);
  if (ret < 0)
    {
      return ret;
    }

  /* Per-mode register tables intentionally do not touch the test-pattern
   * registers. Always program TEST_PATTERN (0x0600/0x0601) here from the
   * cached driver state so that a value selected via
   * imx219_set_test_pattern() survives a mode switch (e.g. selecting the
   * pattern before VIDIOC_STREAMON -> start_capture -> set_mode).
   */

  ret = csi_sensor_cci_write8(&dev->cci, IMX219_REG_TEST_PATTERN_MSB, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  ret = csi_sensor_cci_write8(&dev->cci, IMX219_REG_TEST_PATTERN_LSB,
                              dev->test_pattern);
  if (ret < 0)
    {
      return ret;
    }

  /* The test-pattern window must match the output size, otherwise the
   * sensor emits a pattern frame larger than the configured mode.
   */

  {
    const struct csi_sensor_reg_s tp_window[] =
    {
      { IMX219_REG_TP_WINDOW_X_MSB, 0x00 },
      { IMX219_REG_TP_WINDOW_X_LSB, 0x00 },
      { IMX219_REG_TP_WINDOW_Y_MSB, 0x00 },
      { IMX219_REG_TP_WINDOW_Y_LSB, 0x00 },
      { IMX219_REG_TP_WINDOW_W_MSB, (uint8_t)(mode->width >> 8) },
      { IMX219_REG_TP_WINDOW_W_LSB, (uint8_t)(mode->width & 0xff) },
      { IMX219_REG_TP_WINDOW_H_MSB, (uint8_t)(mode->height >> 8) },
      { IMX219_REG_TP_WINDOW_H_LSB, (uint8_t)(mode->height & 0xff) },
    };

    ret = csi_sensor_cci_write_list(&dev->cci, tp_window,
                                    sizeof(tp_window) /
                                    sizeof(tp_window[0]));
    if (ret < 0)
      {
        return ret;
      }
  }

  dev->mode = mode;
  dev->frame_length_lines = mode->frame_length_lines;
  return OK;
}

/* --- Unit conversions ---------------------------------------------- */

/* Analog gain register value is:  code = 256 - 256/gain  (gain >= 1.0)
 * inverted:                       gain = 256 / (256 - code)
 * Range: code = 0..232  -> gain = 1.0 .. 10.67
 */

static uint8_t imx219_gain_q16_to_code(uint32_t gain_q16)
{
  uint32_t gain_x256;
  uint32_t code;

  /* Clamp to >= 1.0 */

  if (gain_q16 < (1u << 16))
    {
      return 0;
    }

  /* code = 256 - (256 * 65536 / gain_q16) */

  gain_x256 = ((uint64_t)256u * (uint64_t)(1u << 16)) / gain_q16;
  if (gain_x256 >= 256u)
    {
      return 0;
    }

  code = 256u - gain_x256;
  if (code > IMX219_ANA_GAIN_CODE_MAX)
    {
      code = IMX219_ANA_GAIN_CODE_MAX;
    }

  return (uint8_t)code;
}

static uint32_t imx219_us_to_lines(const struct imx219_mode_s *mode,
                                   uint32_t frame_length_lines,
                                   uint32_t us)
{
  /* line_time_us = line_length_pck / (vt_pix_clk_hz / 1e6)
   *              = line_length_pck * 1e6 / vt_pix_clk_hz
   * lines        = us / line_time_us
   *              = us * vt_pix_clk_hz / (line_length_pck * 1e6)
   */

  uint64_t num = (uint64_t)us * (uint64_t)mode->vt_pix_clk_hz;
  uint64_t den = (uint64_t)mode->line_length_pck * 1000000ull;
  uint64_t lines = num / den;

  if (lines < IMX219_EXPOSURE_LINES_MIN)
    {
      lines = IMX219_EXPOSURE_LINES_MIN;
    }

  if (lines + IMX219_EXPOSURE_LINES_MARGIN > frame_length_lines)
    {
      lines = frame_length_lines - IMX219_EXPOSURE_LINES_MARGIN;
    }

  return (uint32_t)lines;
}

static uint32_t imx219_interval_to_frame_length(
                              const struct imx219_mode_s *mode,
                              uint32_t numerator,
                              uint32_t denominator)
{
  uint64_t num;
  uint64_t den;

  num = (uint64_t)mode->vt_pix_clk_hz * (uint64_t)numerator;
  den = (uint64_t)mode->line_length_pck * (uint64_t)denominator;

  return (uint32_t)((num + den / 2u) / den);
}

/* --- Sensor control -------------------------------------------------- */

static int imx219_probe(struct imx219_dev_s *dev)
{
  uint8_t msb = 0xff;
  uint8_t lsb = 0xff;
  uint16_t id;
  int ret;

  if (dev == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  ret = csi_sensor_cci_read8(&dev->cci, IMX219_REG_CHIP_ID_MSB, &msb);
  if (ret < 0)
    {
      goto out;
    }

  ret = csi_sensor_cci_read8(&dev->cci, IMX219_REG_CHIP_ID_LSB, &lsb);
  if (ret < 0)
    {
      goto out;
    }

  id = ((uint16_t)msb << 8) | lsb;

  vinfo("imx219: chip ID 0x%04x (expected 0x%04x)%s\n",
         id, IMX219_CHIP_ID,
         (id == IMX219_CHIP_ID) ? " OK" : " MISMATCH");

  ret = (id == IMX219_CHIP_ID) ? OK : -ENODEV;

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int imx219_init(struct imx219_dev_s *dev)
{
  int ret;

  if (dev == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  ret = imx219_soft_reset(dev);
  if (ret < 0)
    {
      goto out;
    }

  ret = imx219_apply_mode(dev, imx219_mode_by_id(IMX219_DEFAULT_MODE_ID));
  if (ret < 0)
    {
      goto out;
    }

  dev->initialised = true;
  dev->streaming   = false;

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int imx219_start_stream(struct imx219_dev_s *dev)
{
  int ret;

  if (dev == NULL || !dev->initialised)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);
  ret = csi_sensor_cci_write8(&dev->cci, IMX219_REG_MODE_SELECT, 0x01);
  if (ret == OK)
    {
      dev->streaming = true;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

static int imx219_stop_stream(struct imx219_dev_s *dev)
{
  int ret;

  if (dev == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);
  ret = csi_sensor_cci_write8(&dev->cci, IMX219_REG_MODE_SELECT, 0x00);
  if (ret == OK)
    {
      dev->streaming = false;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

static int imx219_set_mode(struct imx219_dev_s *dev,
                           enum imx219_mode_id_e id)
{
  const struct imx219_mode_s *mode;
  int ret;

  if (dev == NULL || !dev->initialised)
    {
      return -EINVAL;
    }

  mode = imx219_mode_by_id((int)id);
  if (mode == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  if (dev->streaming)
    {
      ret = -EBUSY;
      goto out;
    }

  ret = imx219_apply_mode(dev, mode);
  if (ret == OK)
    {
      vinfo("imx219: switched to mode %s\n", mode->name);
    }

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int imx219_set_frame_interval(struct imx219_dev_s *dev,
                                     uint32_t numerator,
                                     uint32_t denominator)
{
  uint32_t lines;
  uint16_t coarse;
  int ret;

  if (dev == NULL || !dev->initialised || dev->mode == NULL ||
      numerator == 0 || denominator == 0)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  if (dev->streaming)
    {
      ret = -EBUSY;
      goto unlock;
    }

  lines = imx219_interval_to_frame_length(dev->mode,
                                          numerator, denominator);
  if (lines <= dev->mode->height + IMX219_EXPOSURE_LINES_MARGIN ||
      lines > dev->mode->max_frame_length_lines)
    {
      ret = -ERANGE;
      goto unlock;
    }

  ret = csi_sensor_cci_read16(&dev->cci, IMX219_REG_COARSE_INT_MSB, &coarse);
  if (ret < 0)
    {
      goto unlock;
    }

  if ((uint32_t)coarse + IMX219_EXPOSURE_LINES_MARGIN > lines)
    {
      coarse = (uint16_t)(lines - IMX219_EXPOSURE_LINES_MARGIN);
    }

  ret = csi_sensor_cci_write8(&dev->cci, IMX219_REG_GROUPED_PAR_HOLD, 0x01);
  if (ret < 0)
    {
      goto unlock;
    }

  ret = csi_sensor_cci_write16(&dev->cci, IMX219_REG_FRAME_LENGTH_MSB,
                               (uint16_t)lines);
  if (ret < 0)
    {
      goto release_hold;
    }

  ret = csi_sensor_cci_write16(&dev->cci, IMX219_REG_COARSE_INT_MSB,
                               coarse);
  if (ret == OK)
    {
      dev->frame_length_lines = lines;
    }

release_hold:
  (void)csi_sensor_cci_write8(&dev->cci, IMX219_REG_GROUPED_PAR_HOLD, 0x00);

unlock:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int imx219_set_exposure_us(struct imx219_dev_s *dev,
                                  uint32_t exposure_us)
{
  uint32_t lines;
  int ret;

  if (dev == NULL || !dev->initialised)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  lines = imx219_us_to_lines(dev->mode, dev->frame_length_lines,
                             exposure_us);

  /* Group-hold ensures MSB and LSB are latched together. */

  ret = csi_sensor_cci_write8(&dev->cci, IMX219_REG_GROUPED_PAR_HOLD, 0x01);
  if (ret < 0)
    {
      goto unlock;
    }

  ret = csi_sensor_cci_write16(&dev->cci, IMX219_REG_COARSE_INT_MSB,
                               (uint16_t)lines);

  (void)csi_sensor_cci_write8(&dev->cci, IMX219_REG_GROUPED_PAR_HOLD, 0x00);

unlock:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int imx219_set_analog_gain_q16(struct imx219_dev_s *dev,
                                      uint32_t gain_q16)
{
  uint8_t code;
  int ret;

  if (dev == NULL || !dev->initialised)
    {
      return -EINVAL;
    }

  code = imx219_gain_q16_to_code(gain_q16);

  nxmutex_lock(&dev->lock);
  ret = csi_sensor_cci_write8(&dev->cci, IMX219_REG_ANA_GAIN_GLOBAL, code);
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int imx219_set_digital_gain_q16(struct imx219_dev_s *dev,
                                       uint32_t gain_q16)
{
  uint32_t code;
  int ret;

  if (dev == NULL || !dev->initialised)
    {
      return -EINVAL;
    }

  /* The register is Q8.8, the control Q16.16 */

  code = gain_q16 >> 8;
  if (code < IMX219_DIG_GAIN_MIN)
    {
      code = IMX219_DIG_GAIN_MIN;
    }
  else if (code > IMX219_DIG_GAIN_MAX)
    {
      code = IMX219_DIG_GAIN_MAX;
    }

  nxmutex_lock(&dev->lock);
  ret = csi_sensor_cci_write16(&dev->cci, IMX219_REG_DIG_GAIN_GLOBAL_MSB,
                               (uint16_t)code);
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int imx219_set_test_pattern(struct imx219_dev_s *dev,
                                   enum imx219_test_pattern_e pat)
{
  int ret;

  if (dev == NULL || !dev->initialised)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  /* Cache selection so imx219_apply_mode() can restore it after a mode
   * switch that would otherwise clobber 0x0600/0x0601.
   */

  dev->test_pattern = (uint8_t)pat;

  ret = csi_sensor_cci_write8(&dev->cci, IMX219_REG_TEST_PATTERN_MSB, 0x00);
  if (ret < 0)
    {
      goto out;
    }

  ret = csi_sensor_cci_write8(&dev->cci, IMX219_REG_TEST_PATTERN_LSB,
                              (uint8_t)pat);

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int imx219_get_caps(struct imx219_dev_s *dev,
                           struct imx219_caps_s *caps)
{
  const struct imx219_mode_s *m;
  uint64_t line_us_num;
  uint32_t line_us_x1e6;

  if (dev == NULL || caps == NULL || dev->mode == NULL)
    {
      return -EINVAL;
    }

  m = dev->mode;

  /* Exposure limits in us, matching imx219_us_to_lines() clamps. */

  line_us_num  = (uint64_t)m->line_length_pck * 1000000ull;
  line_us_x1e6 = (uint32_t)(line_us_num / (uint64_t)m->vt_pix_clk_hz);

  caps->min_exposure_us = IMX219_EXPOSURE_LINES_MIN * line_us_x1e6;
  caps->max_exposure_us =
    (dev->frame_length_lines - IMX219_EXPOSURE_LINES_MARGIN) * line_us_x1e6;

  caps->min_again_q16 = 1u << 16;                          /* 1.0x */
  caps->max_again_q16 = (256u << 16) / (256u - IMX219_ANA_GAIN_CODE_MAX);
  caps->min_dgain_q16 = IMX219_DIG_GAIN_MIN << 8;
  caps->max_dgain_q16 = IMX219_DIG_GAIN_MAX << 8;

  return OK;
}

/* --- V4L2 imgsensor adapter ------------------------------------------ */

static const struct v4l2_fmtdesc g_imx219_fmtdescs[] =
{
    {
    .index       = 0,
    .type        = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .flags       = 0,
    .description = "RGGB Bayer 10-bit",
    .pixelformat = IMX219_ADV_FOURCC,
    },
};

static const struct v4l2_frmsizeenum g_imx219_frmsizes[] =
{
    {
    .index                 = 0,
    .pixel_format          = IMX219_ADV_FOURCC,
    .type                  = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete.width        = IMX219_1080P_WIDTH,
    .discrete.height       = IMX219_1080P_HEIGHT,
    },

    {
    .index                 = 1,
    .pixel_format          = IMX219_ADV_FOURCC,
    .type                  = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete.width        = IMX219_VGA_WIDTH,
    .discrete.height       = IMX219_VGA_HEIGHT,
    },
};

static const struct v4l2_frmivalenum g_imx219_frmintervals[] =
{
    {
    .index                    = 0,
    .pixel_format             = IMX219_ADV_FOURCC,
    .width                    = IMX219_1080P_WIDTH,
    .height                   = IMX219_1080P_HEIGHT,
    .type                     = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete.numerator       = IMX219_1080P_FRAME_INTERVAL_NUM,
    .discrete.denominator     = IMX219_1080P_FRAME_INTERVAL_DEN,
    },

    {
    .index                    = 1,
    .pixel_format             = IMX219_ADV_FOURCC,
    .width                    = IMX219_VGA_WIDTH,
    .height                   = IMX219_VGA_HEIGHT,
    .type                     = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete.numerator       = IMX219_VGA_FRAME_INTERVAL_NUM,
    .discrete.denominator     = IMX219_VGA_FRAME_INTERVAL_DEN,
    },
};

static const imgsensor_mipi_csi_mode_t g_imx219_csi_modes[] =
{
    {
    .pixelformat = IMX219_ADV_FOURCC,
    .width       = IMX219_1080P_WIDTH,
    .height      = IMX219_1080P_HEIGHT,
    .interval    =
        {
        .numerator   = IMX219_1080P_FRAME_INTERVAL_NUM,
        .denominator = IMX219_1080P_FRAME_INTERVAL_DEN,
        },
    },

    {
    .pixelformat = IMX219_ADV_FOURCC,
    .width       = IMX219_VGA_WIDTH,
    .height      = IMX219_VGA_HEIGHT,
    .interval    =
        {
        .numerator   = IMX219_VGA_FRAME_INTERVAL_NUM,
        .denominator = IMX219_VGA_FRAME_INTERVAL_DEN,
        },
    },
};

static const struct imx219_mode_s *imx219_find_mode(
  FAR const imgsensor_format_t *format, FAR enum imx219_mode_id_e *id)
{
  enum imx219_mode_id_e mode_id;

  if (format == NULL || format->pixelformat != IMX219_ADV_FOURCC)
    {
      return NULL;
    }

  if (format->width == IMX219_1080P_WIDTH &&
      format->height == IMX219_1080P_HEIGHT)
    {
      mode_id = IMX219_MODE_1920X1080;
    }
  else if (format->width == IMX219_VGA_WIDTH &&
           format->height == IMX219_VGA_HEIGHT)
    {
      mode_id = IMX219_MODE_VGA_640X480;
    }
  else
    {
      return NULL;
    }

  if (id != NULL)
    {
      *id = mode_id;
    }

  return imx219_mode_by_id(mode_id);
}

static int imx219_op_get_mipi_csi_config(
  FAR struct imgsensor_s *sensor, FAR const imgsensor_format_t *format,
  FAR const imgsensor_interval_t *interval,
  FAR imgsensor_mipi_csi_cfg_t *config)
{
  FAR const struct imx219_mode_s *mode;
  uint32_t frame_length_lines;

  (void)sensor;

  if (config == NULL)
    {
      return -EINVAL;
    }

  mode = imx219_find_mode(format, NULL);
  if (mode == NULL)
    {
      return -ENOTSUP;
    }

  frame_length_lines = mode->frame_length_lines;
  if (interval != NULL && interval->numerator != 0 &&
      interval->denominator != 0)
    {
      frame_length_lines =
        imx219_interval_to_frame_length(mode, interval->numerator,
                                        interval->denominator);
      if (frame_length_lines <= mode->height +
                                IMX219_EXPOSURE_LINES_MARGIN ||
          frame_length_lines > mode->max_frame_length_lines)
        {
          return -ERANGE;
        }
    }

  /* The receiver times CSI-2 output lines, not sensor line times */

  memset(config, 0, sizeof(*config));
  config->link_mbps_per_lane = IMX219_LINK_MBPS_PER_LANE;
  config->line_length_pck    = mode->line_length_pck;
  config->frame_length_lines = frame_length_lines * mode->rate_factor;
  config->vt_pix_clk_hz      = mode->vt_pix_clk_hz * mode->rate_factor;
  config->n_lanes            = mode->n_lanes;
  config->data_type          = IMX219_CSI_DT_RAW10;
  config->pre_active_lines   = IMX219_PRE_ACTIVE_LINES;

  return OK;
}

static bool imx219_op_is_available(FAR struct imgsensor_s *sensor)
{
  return sensor != NULL;
}

static int imx219_op_init(FAR struct imgsensor_s *sensor)
{
  (void)sensor;
  return OK;
}

static int imx219_op_uninit(FAR struct imgsensor_s *sensor)
{
  (void)sensor;
  return OK;
}

static FAR const char *
imx219_op_get_driver_name(FAR struct imgsensor_s *sensor)
{
  (void)sensor;
  return "imx219";
}

static int imx219_op_validate_frame_setting(
  FAR struct imgsensor_s *sensor, imgsensor_stream_type_t type,
  uint8_t nr_datafmts, FAR imgsensor_format_t *datafmts,
  FAR imgsensor_interval_t *interval)
{
  imgsensor_mipi_csi_cfg_t cfg;

  (void)type;

  if (nr_datafmts < 1 || datafmts == NULL)
    {
      return -EINVAL;
    }

  return imx219_op_get_mipi_csi_config(sensor,
                                       &datafmts[IMGSENSOR_FMT_MAIN],
                                       interval, &cfg);
}

static int imx219_op_start_capture(FAR struct imgsensor_s *sensor,
                                   imgsensor_stream_type_t type,
                                   uint8_t nr_datafmts,
                                   FAR imgsensor_format_t *datafmts,
                                   FAR imgsensor_interval_t *ival)
{
  FAR struct imx219_dev_s *dev = (FAR struct imx219_dev_s *)sensor;
  enum imx219_mode_id_e mode_id;
  int ret;

  (void)type;

  if (nr_datafmts < 1 || datafmts == NULL)
    {
      return -EINVAL;
    }

  if (imx219_find_mode(&datafmts[IMGSENSOR_FMT_MAIN], &mode_id) == NULL)
    {
      return -ENOTSUP;
    }

  ret = imx219_set_mode(dev, mode_id);
  if (ret < 0)
    {
      return ret;
    }

  if (ival != NULL && ival->numerator != 0 && ival->denominator != 0)
    {
      ret = imx219_set_frame_interval(dev, ival->numerator,
                                      ival->denominator);
      if (ret < 0)
        {
          return ret;
        }
    }

  return imx219_start_stream(dev);
}

static int imx219_op_stop_capture(FAR struct imgsensor_s *sensor,
                                  imgsensor_stream_type_t type)
{
  FAR struct imx219_dev_s *dev = (FAR struct imx219_dev_s *)sensor;

  (void)type;
  return imx219_stop_stream(dev);
}

static int imx219_op_get_supported_value(FAR struct imgsensor_s *sensor,
                                         uint32_t id,
                                         FAR imgsensor_supported_value_t *v)
{
  FAR struct imx219_dev_s *dev = (FAR struct imx219_dev_s *)sensor;
  struct imx219_caps_s caps;
  int ret;

  if (v == NULL)
    {
      return -EINVAL;
    }

  ret = imx219_get_caps(dev, &caps);
  if (ret < 0)
    {
      return ret;
    }

  memset(v, 0, sizeof(*v));

  switch (id)
    {
      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:

        /* V4L2 unit: 100 us steps. */

        v->type                = IMGSENSOR_CTRL_TYPE_INTEGER;
        v->u.range.minimum     = (caps.min_exposure_us + 99) / 100;
        v->u.range.maximum     = caps.max_exposure_us / 100;
        v->u.range.step        = 1;
        v->u.range.default_value =
          (int64_t)((caps.min_exposure_us + caps.max_exposure_us) / 200);
        return OK;

      case IMGSENSOR_ID_ANALOGUE_GAIN:
        v->type                = IMGSENSOR_CTRL_TYPE_INTEGER;
        v->u.range.minimum     = caps.min_again_q16;
        v->u.range.maximum     = caps.max_again_q16;
        v->u.range.step        = 1;
        v->u.range.default_value = caps.min_again_q16;
        return OK;

      case IMGSENSOR_ID_DIGITAL_GAIN:
        v->type                = IMGSENSOR_CTRL_TYPE_INTEGER;
        v->u.range.minimum     = caps.min_dgain_q16;
        v->u.range.maximum     = caps.max_dgain_q16;
        v->u.range.step        = 1;
        v->u.range.default_value = caps.min_dgain_q16;
        return OK;

      case IMGSENSOR_ID_TEST_PATTERN:
        v->type                = IMGSENSOR_CTRL_TYPE_INTEGER;
        v->u.range.minimum     = IMX219_TP_OFF;
        v->u.range.maximum     = IMX219_TP_PN31;
        v->u.range.step        = 1;
        v->u.range.default_value = IMX219_TP_OFF;
        return OK;

      case IMGSENSOR_ID_BLACK_LEVEL:

        /* All supported modes are RAW10. */

        v->type                = IMGSENSOR_CTRL_TYPE_INTEGER;
        v->u.range.minimum     = IMX219_BLACK_LEVEL;
        v->u.range.maximum     = IMX219_BLACK_LEVEL;
        v->u.range.step        = 1;
        v->u.range.default_value = IMX219_BLACK_LEVEL;
        return OK;

      default:
        return -EINVAL;
    }
}

static int imx219_op_set_value(FAR struct imgsensor_s *sensor,
                               uint32_t id, uint32_t size,
                               imgsensor_value_t value)
{
  FAR struct imx219_dev_s *dev = (FAR struct imx219_dev_s *)sensor;

  (void)size;

  switch (id)
    {
      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:

        /* V4L2 units: 100 us steps -> convert back to us. */

        return imx219_set_exposure_us(dev,
                                      (uint32_t)value.value32 * 100u);

      case IMGSENSOR_ID_ANALOGUE_GAIN:
        return imx219_set_analog_gain_q16(dev, (uint32_t)value.value32);

      case IMGSENSOR_ID_DIGITAL_GAIN:
        return imx219_set_digital_gain_q16(dev, (uint32_t)value.value32);

      case IMGSENSOR_ID_TEST_PATTERN:
        if ((unsigned)value.value32 > (unsigned)IMX219_TP_PN31)
          {
            return -EINVAL;
          }

        return imx219_set_test_pattern(dev,
                                       (enum imx219_test_pattern_e)
                                       value.value32);

      default:
        return -EINVAL;
    }
}

static const struct imgsensor_ops_s g_imx219_ops =
{
  .is_available           = imx219_op_is_available,
  .init                   = imx219_op_init,
  .uninit                 = imx219_op_uninit,
  .get_driver_name        = imx219_op_get_driver_name,
  .validate_frame_setting = imx219_op_validate_frame_setting,
  .start_capture          = imx219_op_start_capture,
  .stop_capture           = imx219_op_stop_capture,
  .get_supported_value    = imx219_op_get_supported_value,
  .set_value              = imx219_op_set_value,
  .get_mipi_csi_config    = imx219_op_get_mipi_csi_config,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct imgsensor_s *imx219_initialize(FAR struct i2c_master_s *i2c,
                                          uint32_t inck)
{
  FAR struct imx219_dev_s *dev;

  if (i2c == NULL || inck != 24000000)
    {
      return NULL;
    }

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return NULL;
    }

  dev->sensor.ops                = &g_imx219_ops;
  dev->sensor.fmtdescs_num       = nitems(g_imx219_fmtdescs);
  dev->sensor.fmtdescs           = g_imx219_fmtdescs;
  dev->sensor.frmsizes_num       = nitems(g_imx219_frmsizes);
  dev->sensor.frmsizes           = g_imx219_frmsizes;
  dev->sensor.frmintervals_num   = nitems(g_imx219_frmintervals);
  dev->sensor.frmintervals       = g_imx219_frmintervals;
  dev->sensor.mipi_csi_modes_num = nitems(g_imx219_csi_modes);
  dev->sensor.mipi_csi_modes     = g_imx219_csi_modes;

  dev->cci.i2c  = i2c;
  dev->cci.addr = IMX219_I2C_ADDR;
  dev->cci.freq = IMX219_I2C_FREQ;
  dev->mode     = imx219_mode_by_id(IMX219_DEFAULT_MODE_ID);
  nxmutex_init(&dev->lock);

  if (imx219_probe(dev) == OK && imx219_init(dev) == OK)
    {
      return &dev->sensor;
    }

  nxmutex_destroy(&dev->lock);
  kmm_free(dev);
  return NULL;
}
