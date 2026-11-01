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
 * Public Functions
 ****************************************************************************/

FAR const imgsensor_mipi_csi_mode_t *
csi_sensor_find_mode(FAR const imgsensor_mipi_csi_mode_t *modes,
                     size_t nmodes, FAR const imgsensor_format_t *format)
{
  size_t i;
#ifdef CONFIG_VIDEO_MIPI_CSI_SCALING
  FAR const imgsensor_mipi_csi_mode_t *best = NULL;
#endif

  if (modes == NULL || format == NULL)
    {
      return NULL;
    }

#ifdef CONFIG_VIDEO_MIPI_CSI_SCALING
  for (i = 0; i < nmodes; i++)
    {
      FAR const imgsensor_mipi_csi_mode_t *m = &modes[i];

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
  for (i = 0; i < nmodes; i++)
    {
      if (modes[i].pixelformat == format->pixelformat &&
          modes[i].width == format->width &&
          modes[i].height == format->height)
        {
          return &modes[i];
        }
    }

  return NULL;
#endif
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
