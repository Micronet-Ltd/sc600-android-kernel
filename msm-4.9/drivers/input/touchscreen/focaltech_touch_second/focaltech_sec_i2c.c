/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2019, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/************************************************************************
*
* File Name: focaltech_i2c.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-04
*
* Abstract: i2c communication with TP
*
* Version: v1.0
*
* Revision History:
*
************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_sec_core.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define I2C_RETRY_NUMBER                    3

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/

/*****************************************************************************
* Static variables
*****************************************************************************/

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/

/*****************************************************************************
* functions body
*****************************************************************************/
int fts_sec_read(u8 *cmd, u32 cmdlen, u8 *data, u32 datalen)
{
    int ret = 0;
    int i = 0;
    struct i2c_client *client = NULL;
    struct i2c_msg msg_list[2];
    struct i2c_msg *msg = NULL;
    int msg_num = 0;

    /* must have data when read */
    if (!fts_sec_data || !fts_sec_data->client || !data || !datalen) {
        FTS_ERROR("fts_sec_data/client/data/datalen(%d) is invalid", datalen);
        return -EINVAL;
    }
    client = fts_sec_data->client;

    mutex_lock(&fts_sec_data->bus_lock);
    memset(&msg_list[0], 0, sizeof(struct i2c_msg));
    memset(&msg_list[1], 0, sizeof(struct i2c_msg));
    msg_list[0].addr = client->addr;
    msg_list[0].flags = 0;
    msg_list[0].len = cmdlen;
    msg_list[0].buf = cmd;
    msg_list[1].addr = client->addr;
    msg_list[1].flags = I2C_M_RD;
    msg_list[1].len = datalen;
    msg_list[1].buf = data;
    if (cmd && cmdlen) {
        msg = &msg_list[0];
        msg_num = 2;
    } else {
        msg = &msg_list[1];
        msg_num = 1;
    }

    for (i = 0; i < I2C_RETRY_NUMBER; i++) {
        ret = i2c_transfer(client->adapter, msg, msg_num);
        if (ret < 0) {
            FTS_ERROR("i2c_transfer(read) fail,ret:%d", ret);
        } else {
            break;
        }
    }

    mutex_unlock(&fts_sec_data->bus_lock);
    return ret;
}

int fts_sec_write(u8 *writebuf, u32 writelen)
{
    int ret = 0;
    int i = 0;
    struct i2c_client *client = NULL;
    struct i2c_msg msgs;

    if (!fts_sec_data || !fts_sec_data->client || !writebuf || !writelen) {
        FTS_ERROR("fts_sec_data/client/data/datalen(%d) is invalid", writelen);
        return -EINVAL;
    }
    client = fts_sec_data->client;

    mutex_lock(&fts_sec_data->bus_lock);
    memset(&msgs, 0, sizeof(struct i2c_msg));
    msgs.addr = client->addr;
    msgs.flags = 0;
    msgs.len = writelen;
    msgs.buf = writebuf;
    for (i = 0; i < I2C_RETRY_NUMBER; i++) {
        ret = i2c_transfer(client->adapter, &msgs, 1);
        if (ret < 0) {
            FTS_ERROR("i2c_transfer(write) fail,reg:%d", ret);
        } else {
            break;
        }
    }
    mutex_unlock(&fts_sec_data->bus_lock);
    return ret;
}

int fts_sec_read_reg(u8 addr, u8 *value)
{
    return fts_sec_read(&addr, 1, value, 1);
}

int fts_sec_write_reg(u8 addr, u8 value)
{
    u8 buf[2] = { 0 };

    buf[0] = addr;
    buf[1] = value;
    return fts_sec_write(buf, sizeof(buf));
}

int fts_sec_bus_init(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    ts_data->bus_buf = NULL;
    FTS_FUNC_EXIT();
    return 0;
}

int fts_sec_bus_exit(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    FTS_FUNC_EXIT();
    return 0;
}