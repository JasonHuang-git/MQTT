/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-23     flybreak     first version
 */

#include <board.h>
#include "pcf8574.h"

#define RESET_IO        GET_PIN(G, 10)
//#define I2C_BUS         "i2c2"
//#define ETH_RESET_IO    7
pcf8574_device_t dev = RT_NULL;

void phy_reset(void)
{
    rt_pin_write(RESET_IO, PIN_LOW);
    rt_thread_mdelay(50);
    rt_pin_write(RESET_IO, PIN_HIGH);
//    pcf8574_pin_write(dev, ETH_RESET_IO, 1);
//    rt_thread_mdelay(10);
//    pcf8574_pin_write(dev, ETH_RESET_IO, 0);
}

int phy_init(void)
{
    rt_pin_mode(RESET_IO, PIN_MODE_OUTPUT);
    rt_pin_write(RESET_IO, PIN_HIGH);   
//    dev = pcf8574_init(I2C_BUS, RT_NULL);

//    if (dev == RT_NULL)
//        return -1;

    return RT_EOK;
}
INIT_DEVICE_EXPORT(phy_init);
