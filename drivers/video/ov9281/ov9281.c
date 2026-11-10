/****************************************************************************
 * drivers/video/ov9281/ov9281.c
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

/* OmniVision OV9281 MIPI CSI-2 monochrome image sensor, exposed as a V4L2
 * imgsensor. See include/nuttx/video/ov9281.h for the public API.
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
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/video/csi_sensor.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/ov9281.h>

#include "ov9281_regs.h"
#include "ov9281_modes.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OV9281_I2C_FREQ           400000
#define OV9281_I2C_ADDR           0x60

#define OV9281_FOURCC             V4L2_PIX_FMT_Y10

/* CSI-2 data type for RAW10. */

#define OV9281_CSI_DT_RAW10       0x2b

/* The sensor sends no embedded-data lines, so the first image line follows
 * the frame start directly.
 */

#define OV9281_PRE_ACTIVE_LINES   1u

#define OV9281_N_LANES            2u

/* Default frame interval of a mode with the given VTS. */

#define OV9281_INTERVAL_NUM(vts)  (OV9281_LINE_LENGTH_PCK * (vts))
#define OV9281_INTERVAL_DEN       OV9281_PIX_CLK_HZ

/* Exposure applied at start-up, in rows. */

#define OV9281_EXPOSURE_LINES_DEF 500u

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Capabilities of the currently loaded mode */

struct ov9281_caps_s
{
  uint32_t min_exposure_us;
  uint32_t max_exposure_us;
  uint32_t min_again_q16;
  uint32_t max_again_q16;
};

struct ov9281_dev_s
{
  struct imgsensor_s          sensor;   /* must be first */
  struct csi_sensor_cci_s     cci;
  bool                        initialised;
  bool                        streaming;
  mutex_t                     lock;

  const struct ov9281_mode_s *mode;
  uint32_t                    frame_length_lines;
  uint32_t                    exposure_lines;
  uint32_t                    gain_code;
  uint8_t                     test_pattern;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* --- Unit conversions ---------------------------------------------- */

static uint32_t ov9281_lines_to_us(const struct ov9281_mode_s *mode,
                                   uint32_t lines)
{
  uint64_t us = (uint64_t)lines * mode->line_length_pck * 1000000ull /
                mode->vt_pix_clk_hz;

  return us > UINT32_MAX ? UINT32_MAX : (uint32_t)us;
}

static uint32_t ov9281_us_to_lines(const struct ov9281_mode_s *mode,
                                   uint32_t frame_length_lines,
                                   uint32_t us)
{
  uint64_t lines = (uint64_t)us * mode->vt_pix_clk_hz /
                   ((uint64_t)mode->line_length_pck * 1000000ull);
  uint32_t max_lines = frame_length_lines;

  if (lines < OV9281_EXPOSURE_LINES_MIN)
    {
      lines = OV9281_EXPOSURE_LINES_MIN;
    }

  if (lines > max_lines)
    {
      lines = max_lines;
    }

  return (uint32_t)lines;
}

/* Real-gain format: code = gain * 16 (1.0x = 0x10). */

static uint32_t ov9281_gain_q16_to_code(uint32_t gain_q16)
{
  uint32_t code = gain_q16 >> 12;

  if (code < OV9281_GAIN_CODE_MIN)
    {
      code = OV9281_GAIN_CODE_MIN;
    }

  if (code > OV9281_GAIN_CODE_MAX)
    {
      code = OV9281_GAIN_CODE_MAX;
    }

  return code;
}

/* Saturates instead of wrapping so callers can range-check the result. */

static uint32_t ov9281_interval_to_frame_length(
                              const struct ov9281_mode_s *mode,
                              uint32_t numerator,
                              uint32_t denominator)
{
  uint64_t num = (uint64_t)mode->vt_pix_clk_hz * numerator;
  uint64_t den = (uint64_t)mode->line_length_pck * denominator;
  uint64_t lines = (num + den / 2u) / den;

  return lines > UINT32_MAX ? UINT32_MAX : (uint32_t)lines;
}

/* --- Sensor control -------------------------------------------------- */

static int ov9281_write_exposure(struct ov9281_dev_s *dev, uint32_t lines)
{
  /* 20-bit exposure in bits [19:4] of the 24-bit register field */

  return csi_sensor_cci_write24(&dev->cci, OV9281_REG_EXPO_0,
                                (lines << 4) & 0x0ffff0);
}

static int ov9281_write_gain(struct ov9281_dev_s *dev, uint32_t code)
{
  return csi_sensor_cci_write16(&dev->cci, OV9281_REG_GAIN_HIGH,
                                code & 0x1fff);
}

static int ov9281_write_vts(struct ov9281_dev_s *dev, uint32_t lines)
{
  return csi_sensor_cci_write16(&dev->cci, OV9281_REG_VTS_HIGH,
                                lines & 0xffff);
}

static int ov9281_apply_mode(struct ov9281_dev_s *dev,
                             const struct ov9281_mode_s *mode)
{
  uint32_t exposure;
  int ret;

  /* PLL settings can only be changed in software standby. */

  ret = csi_sensor_cci_write8(&dev->cci, OV9281_REG_MODE_SELECT, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  ret = csi_sensor_cci_write_list(&dev->cci, g_ov9281_pll_regs,
                                  g_ov9281_pll_nregs);
  if (ret < 0)
    {
      return ret;
    }

  ret = csi_sensor_cci_write_list(&dev->cci, mode->regs, mode->nregs);
  if (ret < 0)
    {
      return ret;
    }

  dev->mode               = mode;
  dev->frame_length_lines = mode->frame_length_lines;

  /* The mode table does not touch exposure/gain; reapply the cached values
   * clamped to this mode so a pre-STREAMON setting survives.
   */

  exposure = dev->exposure_lines;
  if (exposure > dev->frame_length_lines)
    {
      exposure = dev->frame_length_lines;
    }

  ret = ov9281_write_exposure(dev, exposure);
  if (ret < 0)
    {
      return ret;
    }

  return ov9281_write_gain(dev, dev->gain_code);
}

static int ov9281_probe(struct ov9281_dev_s *dev)
{
  uint8_t msb = 0xff;
  uint8_t lsb = 0xff;
  uint8_t rev = 0xff;
  uint16_t id;
  int ret;

  nxmutex_lock(&dev->lock);

  ret = csi_sensor_cci_read8(&dev->cci, OV9281_REG_CHIP_ID_HIGH, &msb);
  if (ret < 0)
    {
      goto out;
    }

  ret = csi_sensor_cci_read8(&dev->cci, OV9281_REG_CHIP_ID_LOW, &lsb);
  if (ret < 0)
    {
      goto out;
    }

  (void)csi_sensor_cci_read8(&dev->cci, OV9281_REG_REVISION, &rev);

  id = ((uint16_t)msb << 8) | lsb;

  vinfo("ov9281: chip ID 0x%04x rev 0x%02x (expected 0x%04x)%s\n",
         id, rev, OV9281_CHIP_ID,
         (id == OV9281_CHIP_ID) ? " OK" : " MISMATCH");

  ret = (id == OV9281_CHIP_ID) ? OK : -ENODEV;

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int ov9281_init(struct ov9281_dev_s *dev)
{
  int ret;

  nxmutex_lock(&dev->lock);

  ret = csi_sensor_cci_write8(&dev->cci, OV9281_REG_SW_RESET, 0x01);
  if (ret < 0)
    {
      goto out;
    }

  up_mdelay(10);

  ret = csi_sensor_cci_write_list(&dev->cci, g_ov9281_global_regs,
                                  g_ov9281_global_nregs);
  if (ret < 0)
    {
      goto out;
    }

  dev->exposure_lines = OV9281_EXPOSURE_LINES_DEF;
  dev->gain_code      = OV9281_GAIN_CODE_MIN;

  ret = ov9281_apply_mode(dev, &g_ov9281_modes[0]);
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

static int ov9281_start_stream(struct ov9281_dev_s *dev)
{
  int ret;

  if (!dev->initialised)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);
  ret = csi_sensor_cci_write8(&dev->cci, OV9281_REG_MODE_SELECT, 0x01);
  if (ret == OK)
    {
      dev->streaming = true;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

static int ov9281_stop_stream(struct ov9281_dev_s *dev)
{
  int ret;

  nxmutex_lock(&dev->lock);
  ret = csi_sensor_cci_write8(&dev->cci, OV9281_REG_MODE_SELECT, 0x00);
  if (ret == OK)
    {
      dev->streaming = false;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

static int ov9281_set_frame_interval(struct ov9281_dev_s *dev,
                                     uint32_t numerator,
                                     uint32_t denominator)
{
  uint32_t lines;
  int ret;

  if (!dev->initialised || numerator == 0 || denominator == 0)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  if (dev->streaming)
    {
      ret = -EBUSY;
      goto out;
    }

  lines = ov9281_interval_to_frame_length(dev->mode, numerator, denominator);
  if (lines < dev->mode->min_frame_length_lines ||
      lines > OV9281_FRAME_LENGTH_LINES_MAX)
    {
      ret = -ERANGE;
      goto out;
    }

  ret = ov9281_write_vts(dev, lines);
  if (ret < 0)
    {
      goto out;
    }

  dev->frame_length_lines = lines;

  if (dev->exposure_lines > lines)
    {
      dev->exposure_lines = lines;
      ret = ov9281_write_exposure(dev, lines);
    }

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int ov9281_set_exposure_us(struct ov9281_dev_s *dev,
                                  uint32_t exposure_us)
{
  uint32_t lines;
  int ret;

  if (!dev->initialised)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  lines = ov9281_us_to_lines(dev->mode, dev->frame_length_lines,
                             exposure_us);
  ret = ov9281_write_exposure(dev, lines);
  if (ret == OK)
    {
      dev->exposure_lines = lines;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

static int ov9281_set_analog_gain_q16(struct ov9281_dev_s *dev,
                                      uint32_t gain_q16)
{
  uint32_t code;
  int ret;

  if (!dev->initialised)
    {
      return -EINVAL;
    }

  code = ov9281_gain_q16_to_code(gain_q16);

  nxmutex_lock(&dev->lock);
  ret = ov9281_write_gain(dev, code);
  if (ret == OK)
    {
      dev->gain_code = code;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

static int ov9281_set_test_pattern(struct ov9281_dev_s *dev,
                                   enum ov9281_test_pattern_e pat)
{
  uint8_t val;
  int ret;

  if (!dev->initialised)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  ret = csi_sensor_cci_read8(&dev->cci, OV9281_REG_PRE_CTRL00, &val);
  if (ret < 0)
    {
      goto out;
    }

  if (pat == OV9281_TP_COLOR_BARS)
    {
      val |= OV9281_PRE_CTRL00_COLOR_BAR;
    }
  else
    {
      val &= (uint8_t)~OV9281_PRE_CTRL00_COLOR_BAR;
    }

  ret = csi_sensor_cci_write8(&dev->cci, OV9281_REG_PRE_CTRL00, val);
  if (ret == OK)
    {
      dev->test_pattern = (uint8_t)pat;
    }

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int ov9281_get_caps(struct ov9281_dev_s *dev,
                           struct ov9281_caps_s *caps)
{
  if (dev->mode == NULL)
    {
      return -EINVAL;
    }

  caps->min_exposure_us =
    ov9281_lines_to_us(dev->mode, OV9281_EXPOSURE_LINES_MIN);
  caps->max_exposure_us =
    ov9281_lines_to_us(dev->mode, dev->frame_length_lines);

  caps->min_again_q16 = OV9281_GAIN_CODE_MIN << 12;
  caps->max_again_q16 = OV9281_GAIN_CODE_MAX << 12;

  return OK;
}

/* --- V4L2 imgsensor adapter ------------------------------------------ */

static const struct v4l2_fmtdesc g_ov9281_fmtdescs[] =
{
  {
    .index       = 0,
    .type        = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .flags       = 0,
    .description = "Y 10-bit (monochrome)",
    .pixelformat = OV9281_FOURCC,
  },
};

static const struct v4l2_frmsizeenum g_ov9281_frmsizes[] =
{
  {
    .index                 = 0,
    .pixel_format          = OV9281_FOURCC,
    .type                  = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete.width        = 1280,
    .discrete.height       = 800,
  },
  {
    .index                 = 1,
    .pixel_format          = OV9281_FOURCC,
    .type                  = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete.width        = 1280,
    .discrete.height       = 720,
  },
  {
    .index                 = 2,
    .pixel_format          = OV9281_FOURCC,
    .type                  = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete.width        = 640,
    .discrete.height       = 480,
  },
  {
    .index                 = 3,
    .pixel_format          = OV9281_FOURCC,
    .type                  = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete.width        = 640,
    .discrete.height       = 400,
  },
};

#define OV9281_FRMIVAL(i, w, h, vts)                          \
  {                                                           \
    .index                    = i,                            \
    .pixel_format             = OV9281_FOURCC,                \
    .width                    = w,                            \
    .height                   = h,                            \
    .type                     = V4L2_FRMIVAL_TYPE_DISCRETE,   \
    .discrete.numerator       = OV9281_INTERVAL_NUM(vts),     \
    .discrete.denominator     = OV9281_INTERVAL_DEN,          \
  }

static const struct v4l2_frmivalenum g_ov9281_frmintervals[] =
{
  OV9281_FRMIVAL(0, 1280, 800, OV9281_VTS_1280X800),
  OV9281_FRMIVAL(1, 1280, 720, OV9281_VTS_1280X720),
  OV9281_FRMIVAL(2, 640, 480, OV9281_VTS_640X480),
  OV9281_FRMIVAL(3, 640, 400, OV9281_VTS_640X400),
};

#define OV9281_CSI_MODE(w, h, vts)                            \
  {                                                           \
    .pixelformat = OV9281_FOURCC,                             \
    .width       = w,                                         \
    .height      = h,                                         \
    .interval    =                                            \
      {                                                       \
        .numerator   = OV9281_INTERVAL_NUM(vts),              \
        .denominator = OV9281_INTERVAL_DEN,                   \
      },                                                      \
  }

static const imgsensor_mipi_csi_mode_t g_ov9281_csi_modes[] =
{
  OV9281_CSI_MODE(1280, 800, OV9281_VTS_1280X800),
  OV9281_CSI_MODE(1280, 720, OV9281_VTS_1280X720),
  OV9281_CSI_MODE(640, 480, OV9281_VTS_640X480),
  OV9281_CSI_MODE(640, 400, OV9281_VTS_640X400),
};

static const struct ov9281_mode_s *
ov9281_find_mode(FAR const imgsensor_format_t *format)
{
  unsigned int i;

  if (format == NULL || format->pixelformat != OV9281_FOURCC)
    {
      return NULL;
    }

  for (i = 0; i < g_ov9281_nmodes; i++)
    {
      if (g_ov9281_modes[i].width == format->width &&
          g_ov9281_modes[i].height == format->height)
        {
          return &g_ov9281_modes[i];
        }
    }

  return NULL;
}

static int ov9281_op_get_mipi_csi_config(
  FAR struct imgsensor_s *sensor, FAR const imgsensor_format_t *format,
  FAR const imgsensor_interval_t *interval,
  FAR imgsensor_mipi_csi_cfg_t *config)
{
  FAR const struct ov9281_mode_s *mode;
  uint32_t frame_length_lines;

  (void)sensor;

  if (config == NULL)
    {
      return -EINVAL;
    }

  mode = ov9281_find_mode(format);
  if (mode == NULL)
    {
      return -ENOTSUP;
    }

  frame_length_lines = mode->frame_length_lines;
  if (interval != NULL && interval->numerator != 0 &&
      interval->denominator != 0)
    {
      frame_length_lines =
        ov9281_interval_to_frame_length(mode, interval->numerator,
                                        interval->denominator);
      if (frame_length_lines < mode->min_frame_length_lines ||
          frame_length_lines > OV9281_FRAME_LENGTH_LINES_MAX)
        {
          return -ERANGE;
        }
    }

  memset(config, 0, sizeof(*config));
  config->link_mbps_per_lane = mode->link_mbps_per_lane;
  config->line_length_pck    = mode->line_length_pck;
  config->frame_length_lines = frame_length_lines;
  config->vt_pix_clk_hz      = mode->vt_pix_clk_hz;
  config->n_lanes            = OV9281_N_LANES;
  config->data_type          = OV9281_CSI_DT_RAW10;
  config->pre_active_lines   = OV9281_PRE_ACTIVE_LINES;
  config->line_sync          = IMGSENSOR_LINE_SYNC_LS;

  return OK;
}

static bool ov9281_op_is_available(FAR struct imgsensor_s *sensor)
{
  return sensor != NULL;
}

static int ov9281_op_init(FAR struct imgsensor_s *sensor)
{
  (void)sensor;
  return OK;
}

static int ov9281_op_uninit(FAR struct imgsensor_s *sensor)
{
  (void)sensor;
  return OK;
}

static FAR const char *
ov9281_op_get_driver_name(FAR struct imgsensor_s *sensor)
{
  (void)sensor;
  return "ov9281";
}

static int ov9281_op_validate_frame_setting(
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

  return ov9281_op_get_mipi_csi_config(sensor,
                                       &datafmts[IMGSENSOR_FMT_MAIN],
                                       interval, &cfg);
}

static int ov9281_op_start_capture(FAR struct imgsensor_s *sensor,
                                   imgsensor_stream_type_t type,
                                   uint8_t nr_datafmts,
                                   FAR imgsensor_format_t *datafmts,
                                   FAR imgsensor_interval_t *ival)
{
  FAR struct ov9281_dev_s *dev = (FAR struct ov9281_dev_s *)sensor;
  FAR const struct ov9281_mode_s *mode;
  int ret;

  (void)type;

  if (nr_datafmts < 1 || datafmts == NULL)
    {
      return -EINVAL;
    }

  mode = ov9281_find_mode(&datafmts[IMGSENSOR_FMT_MAIN]);
  if (mode == NULL)
    {
      return -ENOTSUP;
    }

  nxmutex_lock(&dev->lock);

  if (dev->streaming)
    {
      nxmutex_unlock(&dev->lock);
      return -EBUSY;
    }

  ret = ov9281_apply_mode(dev, mode);
  nxmutex_unlock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (ival != NULL && ival->numerator != 0 && ival->denominator != 0)
    {
      ret = ov9281_set_frame_interval(dev, ival->numerator,
                                      ival->denominator);
      if (ret < 0)
        {
          return ret;
        }
    }

  return ov9281_start_stream(dev);
}

static int ov9281_op_stop_capture(FAR struct imgsensor_s *sensor,
                                  imgsensor_stream_type_t type)
{
  (void)type;
  return ov9281_stop_stream((FAR struct ov9281_dev_s *)sensor);
}

static int ov9281_op_get_supported_value(FAR struct imgsensor_s *sensor,
                                         uint32_t id,
                                         FAR imgsensor_supported_value_t *v)
{
  FAR struct ov9281_dev_s *dev = (FAR struct ov9281_dev_s *)sensor;
  struct ov9281_caps_s caps;
  int ret;

  if (v == NULL)
    {
      return -EINVAL;
    }

  ret = ov9281_get_caps(dev, &caps);
  if (ret < 0)
    {
      return ret;
    }

  memset(v, 0, sizeof(*v));

  switch (id)
    {
      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:

        /* V4L2 unit: 100 us steps. */

        v->type                  = IMGSENSOR_CTRL_TYPE_INTEGER;
        v->u.range.minimum       = (caps.min_exposure_us + 99) / 100;
        v->u.range.maximum       = caps.max_exposure_us / 100;
        v->u.range.step          = 1;
        v->u.range.default_value =
          (int64_t)((caps.min_exposure_us + caps.max_exposure_us) / 200);
        return OK;

      case IMGSENSOR_ID_ANALOGUE_GAIN:
        v->type                  = IMGSENSOR_CTRL_TYPE_INTEGER;
        v->u.range.minimum       = caps.min_again_q16;
        v->u.range.maximum       = caps.max_again_q16;
        v->u.range.step          = 1;
        v->u.range.default_value = caps.min_again_q16;
        return OK;

      case IMGSENSOR_ID_TEST_PATTERN:
        v->type                  = IMGSENSOR_CTRL_TYPE_INTEGER;
        v->u.range.minimum       = OV9281_TP_OFF;
        v->u.range.maximum       = OV9281_TP_COLOR_BARS;
        v->u.range.step          = 1;
        v->u.range.default_value = OV9281_TP_OFF;
        return OK;

      case IMGSENSOR_ID_BLACK_LEVEL:
        v->type                  = IMGSENSOR_CTRL_TYPE_INTEGER;
        v->u.range.minimum       = OV9281_BLACK_LEVEL;
        v->u.range.maximum       = OV9281_BLACK_LEVEL;
        v->u.range.step          = 1;
        v->u.range.default_value = OV9281_BLACK_LEVEL;
        return OK;

      default:
        return -EINVAL;
    }
}

static int ov9281_op_set_value(FAR struct imgsensor_s *sensor,
                               uint32_t id, uint32_t size,
                               imgsensor_value_t value)
{
  FAR struct ov9281_dev_s *dev = (FAR struct ov9281_dev_s *)sensor;

  (void)size;

  switch (id)
    {
      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:

        /* V4L2 units: 100 us steps -> convert back to us. */

        return ov9281_set_exposure_us(dev, (uint32_t)value.value32 * 100u);

      case IMGSENSOR_ID_ANALOGUE_GAIN:
        return ov9281_set_analog_gain_q16(dev, (uint32_t)value.value32);

      case IMGSENSOR_ID_TEST_PATTERN:
        if ((unsigned)value.value32 > (unsigned)OV9281_TP_COLOR_BARS)
          {
            return -EINVAL;
          }

        return ov9281_set_test_pattern(dev,
                                       (enum ov9281_test_pattern_e)
                                       value.value32);

      default:
        return -EINVAL;
    }
}

static const struct imgsensor_ops_s g_ov9281_ops =
{
  .is_available           = ov9281_op_is_available,
  .init                   = ov9281_op_init,
  .uninit                 = ov9281_op_uninit,
  .get_driver_name        = ov9281_op_get_driver_name,
  .validate_frame_setting = ov9281_op_validate_frame_setting,
  .start_capture          = ov9281_op_start_capture,
  .stop_capture           = ov9281_op_stop_capture,
  .get_supported_value    = ov9281_op_get_supported_value,
  .set_value              = ov9281_op_set_value,
  .get_mipi_csi_config    = ov9281_op_get_mipi_csi_config,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct imgsensor_s *ov9281_initialize(FAR struct i2c_master_s *i2c,
                                          uint32_t inck)
{
  FAR struct ov9281_dev_s *dev;

  if (i2c == NULL || inck != 24000000)
    {
      return NULL;
    }

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return NULL;
    }

  dev->sensor.ops                = &g_ov9281_ops;
  dev->sensor.fmtdescs_num       = nitems(g_ov9281_fmtdescs);
  dev->sensor.fmtdescs           = g_ov9281_fmtdescs;
  dev->sensor.frmsizes_num       = nitems(g_ov9281_frmsizes);
  dev->sensor.frmsizes           = g_ov9281_frmsizes;
  dev->sensor.frmintervals_num   = nitems(g_ov9281_frmintervals);
  dev->sensor.frmintervals       = g_ov9281_frmintervals;
  dev->sensor.mipi_csi_modes_num = nitems(g_ov9281_csi_modes);
  dev->sensor.mipi_csi_modes     = g_ov9281_csi_modes;

  dev->cci.i2c  = i2c;
  dev->cci.addr = OV9281_I2C_ADDR;
  dev->cci.freq = OV9281_I2C_FREQ;
  dev->mode     = &g_ov9281_modes[0];
  nxmutex_init(&dev->lock);

  if (ov9281_probe(dev) == OK && ov9281_init(dev) == OK)
    {
      return &dev->sensor;
    }

  nxmutex_destroy(&dev->lock);
  kmm_free(dev);
  return NULL;
}
