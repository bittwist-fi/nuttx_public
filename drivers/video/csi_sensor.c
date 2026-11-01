/****************************************************************************
 * drivers/video/csi_sensor.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_DRIVERS_VIDEO) && defined(CONFIG_VIDEO_STREAM) && \
    defined(CONFIG_VIDEO_MIPI_CSI)

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/nuttx.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/csi_sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_VIDEO_MIPI_CSI_REGDEBUG
#  define cci_regdbg(...) syslog(LOG_INFO, "cci: " __VA_ARGS__)
#else
#  define cci_regdbg(...)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool csi_sensor_is_available(FAR struct imgsensor_s *sensor);
static int  csi_sensor_init(FAR struct imgsensor_s *sensor);
static int  csi_sensor_uninit(FAR struct imgsensor_s *sensor);
static FAR const char *
            csi_sensor_get_driver_name(FAR struct imgsensor_s *sensor);
static int  csi_sensor_validate_frame_setting(
              FAR struct imgsensor_s *sensor,
              imgsensor_stream_type_t type,
              uint8_t nr_datafmts,
              FAR imgsensor_format_t *datafmts,
              FAR imgsensor_interval_t *interval);
static int  csi_sensor_start_capture(FAR struct imgsensor_s *sensor,
              imgsensor_stream_type_t type,
              uint8_t nr_datafmts,
              FAR imgsensor_format_t *datafmts,
              FAR imgsensor_interval_t *interval);
static int  csi_sensor_stop_capture(FAR struct imgsensor_s *sensor,
              imgsensor_stream_type_t type);
static int  csi_sensor_get_frame_interval(FAR struct imgsensor_s *sensor,
              imgsensor_stream_type_t type,
              FAR imgsensor_interval_t *interval);
static int  csi_sensor_get_supported_value(FAR struct imgsensor_s *sensor,
              uint32_t id,
              FAR imgsensor_supported_value_t *value);
static int  csi_sensor_get_value(FAR struct imgsensor_s *sensor,
              uint32_t id, uint32_t size,
              FAR imgsensor_value_t *value);
static int  csi_sensor_set_value(FAR struct imgsensor_s *sensor,
              uint32_t id, uint32_t size,
              imgsensor_value_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imgsensor_ops_s g_csi_sensor_ops =
{
  .is_available           = csi_sensor_is_available,
  .init                   = csi_sensor_init,
  .uninit                 = csi_sensor_uninit,
  .get_driver_name        = csi_sensor_get_driver_name,
  .validate_frame_setting = csi_sensor_validate_frame_setting,
  .start_capture          = csi_sensor_start_capture,
  .stop_capture           = csi_sensor_stop_capture,
  .get_frame_interval     = csi_sensor_get_frame_interval,
  .get_supported_value    = csi_sensor_get_supported_value,
  .get_value              = csi_sensor_get_value,
  .set_value              = csi_sensor_set_value,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR const imgsensor_mipi_csi_mode_t *
csi_sensor_find_mipi_csi_mode(FAR const struct imgsensor_s *sensor,
                              FAR const imgsensor_format_t *format)
{
  size_t i;
#ifdef CONFIG_VIDEO_MIPI_CSI_SCALING
  FAR const imgsensor_mipi_csi_mode_t *best = NULL;
#endif

  if (sensor == NULL || format == NULL || sensor->mipi_csi_modes == NULL)
    {
      return NULL;
    }

#ifdef CONFIG_VIDEO_MIPI_CSI_SCALING
  for (i = 0; i < sensor->mipi_csi_modes_num; i++)
    {
      FAR const imgsensor_mipi_csi_mode_t *m = &sensor->mipi_csi_modes[i];

      if (m->pixelformat == format->pixelformat &&
          m->width >= format->width && m->height >= format->height &&
          (best == NULL ||
           (uint32_t)m->width * m->height <
           (uint32_t)best->width * best->height))
        {
          best = m;
        }
    }

  return best;
#else
  for (i = 0; i < sensor->mipi_csi_modes_num; i++)
    {
      if (sensor->mipi_csi_modes[i].pixelformat == format->pixelformat &&
          sensor->mipi_csi_modes[i].width == format->width &&
          sensor->mipi_csi_modes[i].height == format->height)
        {
          return &sensor->mipi_csi_modes[i];
        }
    }

  return NULL;
#endif
}

static bool csi_sensor_is_available(FAR struct imgsensor_s *sensor)
{
  FAR struct csi_sensor_s *p =
    container_of(sensor, struct csi_sensor_s, sensor);

  DEBUGASSERT(p->config->ops != NULL);

  if (p->config->ops->is_available == NULL)
    {
      return p->priv != NULL;
    }

  return p->config->ops->is_available(p->priv);
}

static int csi_sensor_init(FAR struct imgsensor_s *sensor)
{
  FAR struct csi_sensor_s *p =
    container_of(sensor, struct csi_sensor_s, sensor);

  DEBUGASSERT(p->config->ops != NULL);

  if (p->config->ops->init != NULL)
    {
      return p->config->ops->init(p->priv);
    }

  return OK;
}

static int csi_sensor_uninit(FAR struct imgsensor_s *sensor)
{
  FAR struct csi_sensor_s *p =
    container_of(sensor, struct csi_sensor_s, sensor);

  DEBUGASSERT(p->config->ops != NULL);

  if (p->config->ops->uninit != NULL)
    {
      return p->config->ops->uninit(p->priv);
    }

  return OK;
}

static FAR const char *
csi_sensor_get_driver_name(FAR struct imgsensor_s *sensor)
{
  FAR struct csi_sensor_s *p =
    container_of(sensor, struct csi_sensor_s, sensor);

  return p->config->name;
}

static int csi_sensor_validate_frame_setting(
  FAR struct imgsensor_s *sensor, imgsensor_stream_type_t type,
  uint8_t nr_datafmts, FAR imgsensor_format_t *datafmts,
  FAR imgsensor_interval_t *interval)
{
  FAR struct csi_sensor_s *p =
    container_of(sensor, struct csi_sensor_s, sensor);
  FAR const imgsensor_mipi_csi_mode_t *mode;
  int ret;

  (void)type;

  if (nr_datafmts < 1 || datafmts == NULL)
    {
      return -EINVAL;
    }

  mode = csi_sensor_find_mipi_csi_mode(sensor,
                                      &datafmts[IMGSENSOR_FMT_MAIN]);
  if (mode == NULL)
    {
      return -ENOTSUP;
    }

  DEBUGASSERT(p->config->ops != NULL);
  DEBUGASSERT(p->config->ops->get_mipi_csi_config != NULL);

  ret = p->config->ops->get_mipi_csi_config(p->priv,
                                            &datafmts[IMGSENSOR_FMT_MAIN],
                                            interval, &p->mipi_csi);
  if (ret < 0)
    {
      memset(&p->mipi_csi, 0, sizeof(p->mipi_csi));
    }

  return ret;
}

static int csi_sensor_start_capture(FAR struct imgsensor_s *sensor,
  imgsensor_stream_type_t type, uint8_t nr_datafmts,
  FAR imgsensor_format_t *datafmts, FAR imgsensor_interval_t *interval)
{
  FAR struct csi_sensor_s *p =
    container_of(sensor, struct csi_sensor_s, sensor);

  DEBUGASSERT(p->config->ops != NULL);
  DEBUGASSERT(p->config->ops->start_capture != NULL);

  return p->config->ops->start_capture(p->priv, type, nr_datafmts,
                                       datafmts, interval);
}

static int csi_sensor_stop_capture(FAR struct imgsensor_s *sensor,
                                      imgsensor_stream_type_t type)
{
  FAR struct csi_sensor_s *p =
    container_of(sensor, struct csi_sensor_s, sensor);

  DEBUGASSERT(p->config->ops != NULL);
  DEBUGASSERT(p->config->ops->stop_capture != NULL);

  return p->config->ops->stop_capture(p->priv, type);
}

static int csi_sensor_get_frame_interval(FAR struct imgsensor_s *sensor,
  imgsensor_stream_type_t type, FAR imgsensor_interval_t *interval)
{
  FAR struct csi_sensor_s *p =
    container_of(sensor, struct csi_sensor_s, sensor);

  (void)type;

  if (interval == NULL)
    {
      return -EINVAL;
    }

  else if (p->config->nmodes < 1)
    {
      return -ENOTSUP;
    }

  *interval = p->config->modes[0].interval;
  return OK;
}

static int csi_sensor_get_supported_value(FAR struct imgsensor_s *sensor,
  uint32_t id, FAR imgsensor_supported_value_t *value)
{
  FAR struct csi_sensor_s *p =
    container_of(sensor, struct csi_sensor_s, sensor);

  DEBUGASSERT(p->config->ops != NULL);

  if (p->config->ops->get_supported_value != NULL)
    {
      return p->config->ops->get_supported_value(p->priv, id, value);
    }

  return -EINVAL;
}

static int csi_sensor_get_value(FAR struct imgsensor_s *sensor,
  uint32_t id, uint32_t size, FAR imgsensor_value_t *value)
{
  FAR struct csi_sensor_s *p =
    container_of(sensor, struct csi_sensor_s, sensor);

  DEBUGASSERT(p->config->ops != NULL);

  if (p->config->ops->get_value != NULL)
    {
      return p->config->ops->get_value(p->priv, id, size, value);
    }

  return -EINVAL;
}

static int csi_sensor_set_value(FAR struct imgsensor_s *sensor,
  uint32_t id, uint32_t size, imgsensor_value_t value)
{
  FAR struct csi_sensor_s *p =
    container_of(sensor, struct csi_sensor_s, sensor);

  DEBUGASSERT(p->config->ops != NULL);

  if (p->config->ops->set_value != NULL)
    {
      return p->config->ops->set_value(p->priv, id, size, value);
    }

  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct imgsensor_s *
csi_sensor_bind(FAR const struct csi_sensor_config_s *config,
                FAR void *priv,
                FAR struct csi_sensor_s *storage)
{
  FAR struct csi_sensor_s *p = storage;

  if (config == NULL || priv == NULL || storage == NULL ||
      config->name == NULL || config->modes == NULL || config->nmodes < 1 ||
      config->ops == NULL || config->ops->start_capture == NULL ||
      config->ops->stop_capture == NULL ||
      config->ops->get_mipi_csi_config == NULL)
    {
      return NULL;
    }

  memset(p, 0, sizeof(*p));
  p->config = config;
  p->priv   = priv;

  p->sensor.ops                = &g_csi_sensor_ops;
  p->sensor.fmtdescs_num       = config->fmtdescs_num;
  p->sensor.fmtdescs           = config->fmtdescs;
  p->sensor.frmsizes_num       = config->frmsizes_num;
  p->sensor.frmsizes           = config->frmsizes;
  p->sensor.frmintervals_num   = config->frmintervals_num;
  p->sensor.frmintervals       = config->frmintervals;
  p->sensor.mipi_csi_modes_num = config->nmodes;
  p->sensor.mipi_csi_modes     = config->modes;

  return &p->sensor;
}

int csi_sensor_set_mipi_csi_mode(FAR const struct imgsensor_s *sensor,
                                 FAR const imgsensor_format_t *sensor_fmt,
                                 FAR imgdata_format_t *data_fmt)
{
  FAR const struct csi_sensor_s *p;
  FAR const imgsensor_mipi_csi_mode_t *mode;

  DEBUGASSERT(sensor != NULL);
  DEBUGASSERT(data_fmt != NULL);

  data_fmt->mipi_csi = NULL;

  if (sensor->mipi_csi_modes == NULL || sensor_fmt == NULL)
    {
      return -ENODATA;
    }

  p = container_of(sensor, struct csi_sensor_s, sensor);
  mode = csi_sensor_find_mipi_csi_mode(sensor, sensor_fmt);
  if (mode != NULL)
    {
      data_fmt->mipi_csi = &p->mipi_csi;
      data_fmt->pixelformat = mode->pixelformat;
      return OK;
    }

  return -ENOTSUP;
}

#ifdef CONFIG_I2C
int csi_sensor_cci_write8(FAR const struct csi_sensor_cci_s *cci,
                          uint16_t reg, uint8_t val)
{
  struct i2c_msg_s msg;
  uint8_t buf[3];
  int ret;

  buf[0] = (uint8_t)(reg >> 8);
  buf[1] = (uint8_t)(reg & 0xff);
  buf[2] = val;

  msg.frequency = cci->freq;
  msg.addr      = cci->addr;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = 3;

  ret = I2C_TRANSFER(cci->i2c, &msg, 1);
  if (ret < 0)
    {
      _err("cci 0x%02x: write 0x%04x = 0x%02x -> %d\n",
           cci->addr, reg, val, ret);
    }
  else
    {
      cci_regdbg("0x%02x W 0x%04x = 0x%02x\n", cci->addr, reg, val);
    }

  return ret;
}

int csi_sensor_cci_read8(FAR const struct csi_sensor_cci_s *cci,
                         uint16_t reg, FAR uint8_t *val)
{
  struct i2c_msg_s msgs[2];
  uint8_t regbuf[2];
  int ret;

  regbuf[0] = (uint8_t)(reg >> 8);
  regbuf[1] = (uint8_t)(reg & 0xff);

  msgs[0].frequency = cci->freq;
  msgs[0].addr      = cci->addr;
  msgs[0].flags     = 0;
  msgs[0].buffer    = regbuf;
  msgs[0].length    = 2;

  msgs[1].frequency = cci->freq;
  msgs[1].addr      = cci->addr;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = val;
  msgs[1].length    = 1;

  ret = I2C_TRANSFER(cci->i2c, msgs, 2);
  if (ret < 0)
    {
      _err("cci 0x%02x: read 0x%04x -> %d\n", cci->addr, reg, ret);
    }
  else
    {
      cci_regdbg("0x%02x R 0x%04x = 0x%02x\n", cci->addr, reg, *val);
    }

  return ret;
}

/* Multi-byte values travel in one transaction, in the byte order selected
 * by cci->le.
 */

static int csi_sensor_cci_write(FAR const struct csi_sensor_cci_s *cci,
                                uint16_t reg, uint32_t val,
                                unsigned int len)
{
  struct i2c_msg_s msg;
  uint8_t buf[2 + 4];
  unsigned int i;
  int ret;

  buf[0] = (uint8_t)(reg >> 8);
  buf[1] = (uint8_t)(reg & 0xff);

  for (i = 0; i < len; i++)
    {
      buf[2 + (cci->le ? i : len - 1 - i)] = (uint8_t)(val >> (8 * i));
    }

  msg.frequency = cci->freq;
  msg.addr      = cci->addr;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = 2 + len;

  ret = I2C_TRANSFER(cci->i2c, &msg, 1);
  if (ret < 0)
    {
      _err("cci 0x%02x: write 0x%04x = 0x%0*" PRIx32 " -> %d\n",
           cci->addr, reg, (int)(2 * len), val, ret);
    }
  else
    {
      cci_regdbg("0x%02x W 0x%04x = 0x%0*" PRIx32 "\n",
                 cci->addr, reg, (int)(2 * len), val);
    }

  return ret;
}

static int csi_sensor_cci_read(FAR const struct csi_sensor_cci_s *cci,
                               uint16_t reg, FAR uint32_t *val,
                               unsigned int len)
{
  struct i2c_msg_s msgs[2];
  uint8_t regbuf[2];
  uint8_t data[4];
  unsigned int i;
  int ret;

  regbuf[0] = (uint8_t)(reg >> 8);
  regbuf[1] = (uint8_t)(reg & 0xff);

  msgs[0].frequency = cci->freq;
  msgs[0].addr      = cci->addr;
  msgs[0].flags     = 0;
  msgs[0].buffer    = regbuf;
  msgs[0].length    = 2;

  msgs[1].frequency = cci->freq;
  msgs[1].addr      = cci->addr;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = data;
  msgs[1].length    = len;

  ret = I2C_TRANSFER(cci->i2c, msgs, 2);
  if (ret < 0)
    {
      _err("cci 0x%02x: read 0x%04x -> %d\n", cci->addr, reg, ret);
      return ret;
    }

  *val = 0;
  for (i = 0; i < len; i++)
    {
      *val |= (uint32_t)data[cci->le ? i : len - 1 - i] << (8 * i);
    }

  cci_regdbg("0x%02x R 0x%04x = 0x%0*" PRIx32 "\n",
             cci->addr, reg, (int)(2 * len), *val);

  return ret;
}

int csi_sensor_cci_write16(FAR const struct csi_sensor_cci_s *cci,
                           uint16_t reg, uint16_t val)
{
  return csi_sensor_cci_write(cci, reg, val, 2);
}

int csi_sensor_cci_write24(FAR const struct csi_sensor_cci_s *cci,
                           uint16_t reg, uint32_t val)
{
  return csi_sensor_cci_write(cci, reg, val & 0xffffff, 3);
}

int csi_sensor_cci_write32(FAR const struct csi_sensor_cci_s *cci,
                           uint16_t reg, uint32_t val)
{
  return csi_sensor_cci_write(cci, reg, val, 4);
}

int csi_sensor_cci_read16(FAR const struct csi_sensor_cci_s *cci,
                          uint16_t reg, FAR uint16_t *val)
{
  uint32_t v;
  int ret;

  ret = csi_sensor_cci_read(cci, reg, &v, 2);
  if (ret >= 0)
    {
      *val = (uint16_t)v;
    }

  return ret;
}

int csi_sensor_cci_read24(FAR const struct csi_sensor_cci_s *cci,
                          uint16_t reg, FAR uint32_t *val)
{
  return csi_sensor_cci_read(cci, reg, val, 3);
}

int csi_sensor_cci_read32(FAR const struct csi_sensor_cci_s *cci,
                          uint16_t reg, FAR uint32_t *val)
{
  return csi_sensor_cci_read(cci, reg, val, 4);
}

int csi_sensor_cci_write_list(FAR const struct csi_sensor_cci_s *cci,
                              FAR const struct csi_sensor_reg_s *regs,
                              size_t nregs)
{
  size_t i;
  int ret;

  for (i = 0; i < nregs; i++)
    {
      if (regs[i].addr == CSI_SENSOR_REG_DELAY)
        {
          up_mdelay(regs[i].val);
          continue;
        }

      ret = csi_sensor_cci_write8(cci, regs[i].addr, regs[i].val);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}
#endif /* CONFIG_I2C */

#endif /* CONFIG_DRIVERS_VIDEO && CONFIG_VIDEO_STREAM && CONFIG_VIDEO_MIPI_CSI */
