/****************************************************************************
 * include/nuttx/video/csi_sensor.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_CSI_SENSOR_H
#define __INCLUDE_NUTTX_VIDEO_CSI_SENSOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/videoio.h>

#include <nuttx/video/imgdata.h>
#include <nuttx/video/imgsensor.h>

struct csi_sensor_config_s;

struct csi_sensor_s
{
  struct imgsensor_s sensor;
  FAR const struct csi_sensor_config_s *config;
  FAR void *priv;
#ifdef CONFIG_VIDEO_MIPI_CSI
  imgsensor_mipi_csi_cfg_t mipi_csi;
#endif
};

#ifdef CONFIG_VIDEO_MIPI_CSI

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* MIPI CCI: I2C with 16-bit big-endian register addresses and 8 to 32-bit
 * data; the data byte order is big-endian unless le is set.
 */

struct i2c_master_s;

struct csi_sensor_cci_s
{
  FAR struct i2c_master_s *i2c;
  uint32_t freq;
  uint8_t  addr;
  bool     le;
};

/* Register table entry; addr CSI_SENSOR_REG_DELAY means "wait val ms" */

struct csi_sensor_reg_s
{
  uint16_t addr;
  uint8_t  val;
};

#define CSI_SENSOR_REG_DELAY  0xffffu

struct csi_sensor_ops_s
{
  CODE bool (*is_available)(FAR void *priv);
  CODE int  (*init)(FAR void *priv);
  CODE int  (*uninit)(FAR void *priv);
  CODE int  (*start_capture)(FAR void *priv,
                             imgsensor_stream_type_t type,
                             uint8_t nr_datafmts,
                             FAR imgsensor_format_t *datafmts,
                             FAR imgsensor_interval_t *interval);
  CODE int  (*stop_capture)(FAR void *priv,
                            imgsensor_stream_type_t type);
  CODE int  (*get_mipi_csi_config)(FAR void *priv,
                                   FAR const imgsensor_format_t *format,
                                   FAR const imgsensor_interval_t *interval,
                                   FAR imgsensor_mipi_csi_cfg_t *config);
  CODE int  (*get_supported_value)(FAR void *priv, uint32_t id,
                                   FAR imgsensor_supported_value_t *value);
  CODE int  (*get_value)(FAR void *priv, uint32_t id, uint32_t size,
                         FAR imgsensor_value_t *value);
  CODE int  (*set_value)(FAR void *priv, uint32_t id, uint32_t size,
                         imgsensor_value_t value);
};

struct csi_sensor_config_s
{
  FAR const char *name;
  FAR const imgsensor_mipi_csi_mode_t *modes;
  size_t nmodes;
  FAR const struct v4l2_fmtdesc *fmtdescs;
  size_t fmtdescs_num;
  FAR const struct v4l2_frmsizeenum *frmsizes;
  size_t frmsizes_num;
  FAR const struct v4l2_frmivalenum *frmintervals;
  size_t frmintervals_num;
  FAR const struct csi_sensor_ops_s *ops;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct imgsensor_s *
csi_sensor_bind(FAR const struct csi_sensor_config_s *config,
                FAR void *priv,
                FAR struct csi_sensor_s *storage);

int csi_sensor_set_mipi_csi_mode(FAR const struct imgsensor_s *sensor,
                                 FAR const imgsensor_format_t *sensor_fmt,
                                 FAR imgdata_format_t *data_fmt);

#ifdef CONFIG_I2C
int csi_sensor_cci_write8(FAR const struct csi_sensor_cci_s *cci,
                          uint16_t reg, uint8_t val);
int csi_sensor_cci_read8(FAR const struct csi_sensor_cci_s *cci,
                         uint16_t reg, FAR uint8_t *val);
int csi_sensor_cci_write16(FAR const struct csi_sensor_cci_s *cci,
                           uint16_t reg, uint16_t val);
int csi_sensor_cci_read16(FAR const struct csi_sensor_cci_s *cci,
                          uint16_t reg, FAR uint16_t *val);
int csi_sensor_cci_write24(FAR const struct csi_sensor_cci_s *cci,
                           uint16_t reg, uint32_t val);
int csi_sensor_cci_read24(FAR const struct csi_sensor_cci_s *cci,
                          uint16_t reg, FAR uint32_t *val);
int csi_sensor_cci_write32(FAR const struct csi_sensor_cci_s *cci,
                           uint16_t reg, uint32_t val);
int csi_sensor_cci_read32(FAR const struct csi_sensor_cci_s *cci,
                          uint16_t reg, FAR uint32_t *val);
int csi_sensor_cci_write_list(FAR const struct csi_sensor_cci_s *cci,
                              FAR const struct csi_sensor_reg_s *regs,
                              size_t nregs);
#endif

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_VIDEO_MIPI_CSI */

#endif /* __INCLUDE_NUTTX_VIDEO_CSI_SENSOR_H */
