/****************************************************************************
 * drivers/video/ov7251/ov7251.c
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

/* OmniVision OV7251 MIPI CSI-2 monochrome image sensor, exposed as a V4L2
 * imgsensor. See include/nuttx/video/ov7251.h for the public API.
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
#include <nuttx/video/ov7251.h>

#include "ov7251_regs.h"
#include "ov7251_modes.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OV7251_I2C_FREQ           400000
#define OV7251_I2C_ADDR           0x60

#define OV7251_FOURCC             V4L2_PIX_FMT_Y10

/* CSI-2 data type for RAW10. */

#define OV7251_CSI_DT_RAW10       0x2b

/* The sensor sends no embedded-data lines, so the first image line follows
 * the frame start directly.
 */

#define OV7251_PRE_ACTIVE_LINES   1u

#define OV7251_N_LANES            1u

#define OV7251_WIDTH              640
#define OV7251_HEIGHT             480

#define OV7251_FRAME_INTERVAL_NUM \
  (OV7251_LINE_LENGTH_PCK * OV7251_FRAME_LINES_DEF)
#define OV7251_FRAME_INTERVAL_DEN OV7251_PIX_CLK_HZ

/* Exposure applied at start-up, in rows. */

#define OV7251_EXPOSURE_LINES_DEF 400u

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Capabilities of the currently loaded mode */

struct ov7251_caps_s
{
  uint32_t min_exposure_us;
  uint32_t max_exposure_us;
  uint32_t min_again_q16;
  uint32_t max_again_q16;
};

struct ov7251_dev_s
{
  struct imgsensor_s          sensor;   /* must be first */
  struct csi_sensor_cci_s     cci;
  bool                        initialised;
  bool                        streaming;
  mutex_t                     lock;

  const struct ov7251_mode_s *mode;
  uint32_t                    frame_length_lines;
  uint32_t                    exposure_lines;
  uint32_t                    gain_code;
  uint8_t                     test_pattern;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* --- Unit conversions ---------------------------------------------- */

static uint32_t ov7251_lines_to_us(const struct ov7251_mode_s *mode,
                                   uint32_t lines)
{
  uint64_t us = (uint64_t)lines * mode->line_length_pck * 1000000ull /
                mode->vt_pix_clk_hz;

  return us > UINT32_MAX ? UINT32_MAX : (uint32_t)us;
}

static uint32_t ov7251_us_to_lines(const struct ov7251_mode_s *mode,
                                   uint32_t frame_length_lines,
                                   uint32_t us)
{
  uint64_t lines = (uint64_t)us * mode->vt_pix_clk_hz /
                   ((uint64_t)mode->line_length_pck * 1000000ull);
  uint32_t max_lines = frame_length_lines - OV7251_EXPOSURE_LINES_MARGIN;

  if (lines < OV7251_EXPOSURE_LINES_MIN)
    {
      lines = OV7251_EXPOSURE_LINES_MIN;
    }

  if (lines > max_lines)
    {
      lines = max_lines;
    }

  return (uint32_t)lines;
}

/* Linear gain: code = gain * 16 (1.0x = 0x10). */

static uint32_t ov7251_gain_q16_to_code(uint32_t gain_q16)
{
  uint32_t code = gain_q16 >> 12;

  if (code < OV7251_GAIN_CODE_MIN)
    {
      code = OV7251_GAIN_CODE_MIN;
    }

  if (code > OV7251_GAIN_CODE_MAX)
    {
      code = OV7251_GAIN_CODE_MAX;
    }

  return code;
}

/* Saturates instead of wrapping so callers can range-check the result. */

static uint32_t ov7251_interval_to_frame_length(
                              const struct ov7251_mode_s *mode,
                              uint32_t numerator,
                              uint32_t denominator)
{
  uint64_t num = (uint64_t)mode->vt_pix_clk_hz * numerator;
  uint64_t den = (uint64_t)mode->line_length_pck * denominator;
  uint64_t lines = (num + den / 2u) / den;

  return lines > UINT32_MAX ? UINT32_MAX : (uint32_t)lines;
}

/* --- Sensor control -------------------------------------------------- */

/* Multi-byte fields go out in one transaction so the sensor never latches
 * them half-updated.
 */

static int ov7251_write_exposure(struct ov7251_dev_s *dev, uint32_t lines)
{
  /* 20-bit exposure in bits [19:4] of the 24-bit register field */

  return csi_sensor_cci_write24(&dev->cci, OV7251_REG_EXPO_0,
                                (lines << 4) & 0x0ffff0);
}

static int ov7251_write_gain(struct ov7251_dev_s *dev, uint32_t code)
{
  return csi_sensor_cci_write16(&dev->cci, OV7251_REG_GAIN_HIGH,
                                code & 0x03ff);
}

static int ov7251_write_vts(struct ov7251_dev_s *dev, uint32_t lines)
{
  return csi_sensor_cci_write16(&dev->cci, OV7251_REG_VTS_HIGH,
                                lines & 0xffff);
}

static int ov7251_apply_mode(struct ov7251_dev_s *dev,
                             const struct ov7251_mode_s *mode)
{
  uint32_t exposure;
  int ret;

  /* Ensure stream is off before touching mode registers. */

  ret = csi_sensor_cci_write8(&dev->cci, OV7251_REG_MODE_SELECT, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  ret = csi_sensor_cci_write_list(&dev->cci, g_ov7251_pll_regs,
                                  g_ov7251_pll_nregs);
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
  if (exposure > dev->frame_length_lines - OV7251_EXPOSURE_LINES_MARGIN)
    {
      exposure = dev->frame_length_lines - OV7251_EXPOSURE_LINES_MARGIN;
    }

  ret = ov7251_write_exposure(dev, exposure);
  if (ret < 0)
    {
      return ret;
    }

  return ov7251_write_gain(dev, dev->gain_code);
}

static int ov7251_probe(struct ov7251_dev_s *dev)
{
  uint8_t msb = 0xff;
  uint8_t lsb = 0xff;
  uint8_t rev = 0xff;
  uint16_t id;
  int ret;

  nxmutex_lock(&dev->lock);

  ret = csi_sensor_cci_read8(&dev->cci, OV7251_REG_CHIP_ID_HIGH, &msb);
  if (ret < 0)
    {
      goto out;
    }

  ret = csi_sensor_cci_read8(&dev->cci, OV7251_REG_CHIP_ID_LOW, &lsb);
  if (ret < 0)
    {
      goto out;
    }

  (void)csi_sensor_cci_read8(&dev->cci, OV7251_REG_REVISION, &rev);

  id = ((uint16_t)msb << 8) | lsb;

  vinfo("ov7251: chip ID 0x%04x rev 0x%02x (expected 0x%04x)%s\n",
         id, rev, OV7251_CHIP_ID,
         (id == OV7251_CHIP_ID) ? " OK" : " MISMATCH");

  ret = (id == OV7251_CHIP_ID) ? OK : -ENODEV;

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int ov7251_init(struct ov7251_dev_s *dev)
{
  int ret;

  nxmutex_lock(&dev->lock);

  ret = csi_sensor_cci_write8(&dev->cci, OV7251_REG_SW_RESET, 0x01);
  if (ret < 0)
    {
      goto out;
    }

  up_mdelay(10);

  ret = csi_sensor_cci_write_list(&dev->cci, g_ov7251_global_regs,
                                  g_ov7251_global_nregs);
  if (ret < 0)
    {
      goto out;
    }

  dev->exposure_lines = OV7251_EXPOSURE_LINES_DEF;
  dev->gain_code      = OV7251_GAIN_CODE_MIN;

  ret = ov7251_apply_mode(dev, &g_ov7251_mode_640x480_raw10);
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

static int ov7251_start_stream(struct ov7251_dev_s *dev)
{
  int ret;

  if (!dev->initialised)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);
  ret = csi_sensor_cci_write8(&dev->cci, OV7251_REG_MODE_SELECT, 0x01);
  if (ret == OK)
    {
      dev->streaming = true;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

static int ov7251_stop_stream(struct ov7251_dev_s *dev)
{
  int ret;

  nxmutex_lock(&dev->lock);
  ret = csi_sensor_cci_write8(&dev->cci, OV7251_REG_MODE_SELECT, 0x00);
  if (ret == OK)
    {
      dev->streaming = false;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

static int ov7251_set_frame_interval(struct ov7251_dev_s *dev,
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

  lines = ov7251_interval_to_frame_length(dev->mode, numerator, denominator);
  if (lines < dev->mode->min_frame_length_lines ||
      lines > OV7251_FRAME_LENGTH_LINES_MAX)
    {
      ret = -ERANGE;
      goto out;
    }

  ret = ov7251_write_vts(dev, lines);
  if (ret < 0)
    {
      goto out;
    }

  dev->frame_length_lines = lines;

  if (dev->exposure_lines > lines - OV7251_EXPOSURE_LINES_MARGIN)
    {
      dev->exposure_lines = lines - OV7251_EXPOSURE_LINES_MARGIN;
      ret = ov7251_write_exposure(dev, dev->exposure_lines);
    }

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int ov7251_set_exposure_us(struct ov7251_dev_s *dev,
                                  uint32_t exposure_us)
{
  uint32_t lines;
  int ret;

  if (!dev->initialised)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  lines = ov7251_us_to_lines(dev->mode, dev->frame_length_lines,
                             exposure_us);
  ret = ov7251_write_exposure(dev, lines);
  if (ret == OK)
    {
      dev->exposure_lines = lines;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

static int ov7251_set_analog_gain_q16(struct ov7251_dev_s *dev,
                                      uint32_t gain_q16)
{
  uint32_t code;
  int ret;

  if (!dev->initialised)
    {
      return -EINVAL;
    }

  code = ov7251_gain_q16_to_code(gain_q16);

  nxmutex_lock(&dev->lock);
  ret = ov7251_write_gain(dev, code);
  if (ret == OK)
    {
      dev->gain_code = code;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

static int ov7251_set_test_pattern(struct ov7251_dev_s *dev,
                                   enum ov7251_test_pattern_e pat)
{
  uint8_t val;
  int ret;

  if (!dev->initialised)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  ret = csi_sensor_cci_read8(&dev->cci, OV7251_REG_PRE_ISP_00, &val);
  if (ret < 0)
    {
      goto out;
    }

  if (pat == OV7251_TP_COLOR_BARS)
    {
      val |= OV7251_PRE_ISP_00_COLOR_BAR;
    }
  else
    {
      val &= (uint8_t)~OV7251_PRE_ISP_00_COLOR_BAR;
    }

  ret = csi_sensor_cci_write8(&dev->cci, OV7251_REG_PRE_ISP_00, val);
  if (ret == OK)
    {
      dev->test_pattern = (uint8_t)pat;
    }

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int ov7251_get_caps(struct ov7251_dev_s *dev,
                           struct ov7251_caps_s *caps)
{
  if (dev->mode == NULL)
    {
      return -EINVAL;
    }

  caps->min_exposure_us =
    ov7251_lines_to_us(dev->mode, OV7251_EXPOSURE_LINES_MIN);
  caps->max_exposure_us =
    ov7251_lines_to_us(dev->mode,
                       dev->frame_length_lines -
                       OV7251_EXPOSURE_LINES_MARGIN);

  caps->min_again_q16 = OV7251_GAIN_CODE_MIN << 12;
  caps->max_again_q16 = OV7251_GAIN_CODE_MAX << 12;

  return OK;
}

/* --- V4L2 imgsensor adapter ------------------------------------------ */

static const struct v4l2_fmtdesc g_ov7251_fmtdescs[] =
{
    {
    .index       = 0,
    .type        = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .flags       = 0,
    .description = "Y 10-bit (monochrome)",
    .pixelformat = OV7251_FOURCC,
    },
};

static const struct v4l2_frmsizeenum g_ov7251_frmsizes[] =
{
    {
    .index                 = 0,
    .pixel_format          = OV7251_FOURCC,
    .type                  = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete.width        = OV7251_WIDTH,
    .discrete.height       = OV7251_HEIGHT,
    },
};

static const struct v4l2_frmivalenum g_ov7251_frmintervals[] =
{
    {
    .index                    = 0,
    .pixel_format             = OV7251_FOURCC,
    .width                    = OV7251_WIDTH,
    .height                   = OV7251_HEIGHT,
    .type                     = V4L2_FRMIVAL_TYPE_DISCRETE,
    .discrete.numerator       = OV7251_FRAME_INTERVAL_NUM,
    .discrete.denominator     = OV7251_FRAME_INTERVAL_DEN,
    },
};

static const imgsensor_mipi_csi_mode_t g_ov7251_csi_modes[] =
{
    {
    .pixelformat = OV7251_FOURCC,
    .width       = OV7251_WIDTH,
    .height      = OV7251_HEIGHT,
    .interval    =
        {
        .numerator   = OV7251_FRAME_INTERVAL_NUM,
        .denominator = OV7251_FRAME_INTERVAL_DEN,
        },
    },
};

static bool ov7251_format_supported(FAR const imgsensor_format_t *format)
{
  return format != NULL && format->pixelformat == OV7251_FOURCC &&
         format->width == OV7251_WIDTH && format->height == OV7251_HEIGHT;
}

static int ov7251_op_get_mipi_csi_config(
  FAR struct imgsensor_s *sensor, FAR const imgsensor_format_t *format,
  FAR const imgsensor_interval_t *interval,
  FAR imgsensor_mipi_csi_cfg_t *config)
{
  FAR const struct ov7251_mode_s *mode = &g_ov7251_mode_640x480_raw10;
  uint32_t frame_length_lines = mode->frame_length_lines;

  (void)sensor;

  if (config == NULL)
    {
      return -EINVAL;
    }

  if (!ov7251_format_supported(format))
    {
      return -ENOTSUP;
    }

  if (interval != NULL && interval->numerator != 0 &&
      interval->denominator != 0)
    {
      frame_length_lines =
        ov7251_interval_to_frame_length(mode, interval->numerator,
                                        interval->denominator);
      if (frame_length_lines < mode->min_frame_length_lines ||
          frame_length_lines > OV7251_FRAME_LENGTH_LINES_MAX)
        {
          return -ERANGE;
        }
    }

  memset(config, 0, sizeof(*config));
  config->link_mbps_per_lane = mode->link_mbps_per_lane;
  config->line_length_pck    = mode->line_length_pck;
  config->frame_length_lines = frame_length_lines;
  config->vt_pix_clk_hz      = mode->vt_pix_clk_hz;
  config->n_lanes            = OV7251_N_LANES;
  config->data_type          = OV7251_CSI_DT_RAW10;
  config->pre_active_lines   = OV7251_PRE_ACTIVE_LINES;
  config->line_sync          = IMGSENSOR_LINE_SYNC_LS;

  return OK;
}

static bool ov7251_op_is_available(FAR struct imgsensor_s *sensor)
{
  return sensor != NULL;
}

static int ov7251_op_init(FAR struct imgsensor_s *sensor)
{
  (void)sensor;
  return OK;
}

static int ov7251_op_uninit(FAR struct imgsensor_s *sensor)
{
  (void)sensor;
  return OK;
}

static FAR const char *
ov7251_op_get_driver_name(FAR struct imgsensor_s *sensor)
{
  (void)sensor;
  return "ov7251";
}

static int ov7251_op_validate_frame_setting(
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

  return ov7251_op_get_mipi_csi_config(sensor,
                                       &datafmts[IMGSENSOR_FMT_MAIN],
                                       interval, &cfg);
}

static int ov7251_op_start_capture(FAR struct imgsensor_s *sensor,
                                   imgsensor_stream_type_t type,
                                   uint8_t nr_datafmts,
                                   FAR imgsensor_format_t *datafmts,
                                   FAR imgsensor_interval_t *ival)
{
  FAR struct ov7251_dev_s *dev = (FAR struct ov7251_dev_s *)sensor;
  int ret;

  (void)type;

  if (nr_datafmts < 1 || datafmts == NULL)
    {
      return -EINVAL;
    }

  if (!ov7251_format_supported(&datafmts[IMGSENSOR_FMT_MAIN]))
    {
      return -ENOTSUP;
    }

  nxmutex_lock(&dev->lock);

  if (dev->streaming)
    {
      nxmutex_unlock(&dev->lock);
      return -EBUSY;
    }

  ret = ov7251_apply_mode(dev, &g_ov7251_mode_640x480_raw10);
  nxmutex_unlock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (ival != NULL && ival->numerator != 0 && ival->denominator != 0)
    {
      ret = ov7251_set_frame_interval(dev, ival->numerator,
                                      ival->denominator);
      if (ret < 0)
        {
          return ret;
        }
    }

  return ov7251_start_stream(dev);
}

static int ov7251_op_stop_capture(FAR struct imgsensor_s *sensor,
                                  imgsensor_stream_type_t type)
{
  (void)type;
  return ov7251_stop_stream((FAR struct ov7251_dev_s *)sensor);
}

static int ov7251_op_get_supported_value(FAR struct imgsensor_s *sensor,
                                         uint32_t id,
                                         FAR imgsensor_supported_value_t *v)
{
  FAR struct ov7251_dev_s *dev = (FAR struct ov7251_dev_s *)sensor;
  struct ov7251_caps_s caps;
  int ret;

  if (v == NULL)
    {
      return -EINVAL;
    }

  ret = ov7251_get_caps(dev, &caps);
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
        v->u.range.minimum       = OV7251_TP_OFF;
        v->u.range.maximum       = OV7251_TP_COLOR_BARS;
        v->u.range.step          = 1;
        v->u.range.default_value = OV7251_TP_OFF;
        return OK;

      case IMGSENSOR_ID_BLACK_LEVEL:
        v->type                  = IMGSENSOR_CTRL_TYPE_INTEGER;
        v->u.range.minimum       = OV7251_BLACK_LEVEL;
        v->u.range.maximum       = OV7251_BLACK_LEVEL;
        v->u.range.step          = 1;
        v->u.range.default_value = OV7251_BLACK_LEVEL;
        return OK;

      default:
        return -EINVAL;
    }
}

static int ov7251_op_set_value(FAR struct imgsensor_s *sensor,
                               uint32_t id, uint32_t size,
                               imgsensor_value_t value)
{
  FAR struct ov7251_dev_s *dev = (FAR struct ov7251_dev_s *)sensor;

  (void)size;

  switch (id)
    {
      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:

        /* V4L2 units: 100 us steps -> convert back to us. */

        return ov7251_set_exposure_us(dev, (uint32_t)value.value32 * 100u);

      case IMGSENSOR_ID_ANALOGUE_GAIN:
        return ov7251_set_analog_gain_q16(dev, (uint32_t)value.value32);

      case IMGSENSOR_ID_TEST_PATTERN:
        if ((unsigned)value.value32 > (unsigned)OV7251_TP_COLOR_BARS)
          {
            return -EINVAL;
          }

        return ov7251_set_test_pattern(dev,
                                       (enum ov7251_test_pattern_e)
                                       value.value32);

      default:
        return -EINVAL;
    }
}

static const struct imgsensor_ops_s g_ov7251_ops =
{
  .is_available           = ov7251_op_is_available,
  .init                   = ov7251_op_init,
  .uninit                 = ov7251_op_uninit,
  .get_driver_name        = ov7251_op_get_driver_name,
  .validate_frame_setting = ov7251_op_validate_frame_setting,
  .start_capture          = ov7251_op_start_capture,
  .stop_capture           = ov7251_op_stop_capture,
  .get_supported_value    = ov7251_op_get_supported_value,
  .set_value              = ov7251_op_set_value,
  .get_mipi_csi_config    = ov7251_op_get_mipi_csi_config,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct imgsensor_s *ov7251_initialize(FAR struct i2c_master_s *i2c,
                                          uint32_t inck)
{
  FAR struct ov7251_dev_s *dev;

  if (i2c == NULL || inck != 24000000)
    {
      return NULL;
    }

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return NULL;
    }

  dev->sensor.ops                = &g_ov7251_ops;
  dev->sensor.fmtdescs_num       = nitems(g_ov7251_fmtdescs);
  dev->sensor.fmtdescs           = g_ov7251_fmtdescs;
  dev->sensor.frmsizes_num       = nitems(g_ov7251_frmsizes);
  dev->sensor.frmsizes           = g_ov7251_frmsizes;
  dev->sensor.frmintervals_num   = nitems(g_ov7251_frmintervals);
  dev->sensor.frmintervals       = g_ov7251_frmintervals;
  dev->sensor.mipi_csi_modes_num = nitems(g_ov7251_csi_modes);
  dev->sensor.mipi_csi_modes     = g_ov7251_csi_modes;

  dev->cci.i2c  = i2c;
  dev->cci.addr = OV7251_I2C_ADDR;
  dev->cci.freq = OV7251_I2C_FREQ;
  dev->mode     = &g_ov7251_mode_640x480_raw10;
  nxmutex_init(&dev->lock);

  if (ov7251_probe(dev) == OK && ov7251_init(dev) == OK)
    {
      return &dev->sensor;
    }

  nxmutex_destroy(&dev->lock);
  kmm_free(dev);
  return NULL;
}
