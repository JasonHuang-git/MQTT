/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-05     whj4674672   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "arpa/inet.h"
#include "netdev.h"

/* defined the LED0 pin: PB1 */
#define LED0_PIN    GET_PIN(H, 1)

int main(void)
{
    int count = 1;
    /* set LED0 pin mode to output */	
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);	
	
	rt_thread_t sta = RT_NULL;
	
	while(sta == RT_NULL)
	{
		sta = rt_thread_find("tcpip");
		rt_thread_mdelay(10);
	}
	
	struct netdev *netdev = RT_NULL;
    ip_addr_t dns_addr;

    netdev = netdev_get_by_name("e0");
    if (netdev == RT_NULL)
    {
        rt_kprintf("bad network interface device name(%s).\n", "e0");
    }

    inet_aton("192.168.1.1", &dns_addr);
    netdev_set_dns_server(netdev, 0, &dns_addr);
	
	inet_aton("114.114.114.114", &dns_addr);
    netdev_set_dns_server(netdev, 1, &dns_addr);
	
    while (count++)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
		//rt_kprintf("led on\r\n");
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
		//rt_kprintf("led off\r\n");
        rt_thread_mdelay(500);
    }
    return RT_EOK;
}
