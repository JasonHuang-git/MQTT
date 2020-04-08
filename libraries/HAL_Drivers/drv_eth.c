/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-19     SummerGift   first version
 * 2018-12-25     zylx         fix some bugs
 */

#include "board.h"
#include "drv_config.h"
#include <netif/ethernetif.h>
#include "lwipopts.h"
#include "drv_eth.h"

/*
* Emac driver uses CubeMX tool to generate emac and phy's configuration,
* the configuration files can be found in CubeMX_Config floder.
*/

/* debug option */
//#define ETH_RX_DUMP
//#define ETH_TX_DUMP
//#define DRV_DEBUG
#define LOG_TAG             "drv.emac"
#include <drv_log.h>

#define MAX_ADDR_LEN 6
/* ETH Setting  */
#define ETH_RX_BUFFER_SIZE                     ( 1536UL )
#define ETH_DMA_TRANSMIT_TIMEOUT               ( 20U )

struct rt_stm32_eth
{
    /* inherit from ethernet device */
    struct eth_device parent;

    /* interface address info, hw address */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN];
    /* ETH_Speed */
    uint32_t    ETH_Speed;
    /* ETH_Duplex_Mode */
    uint32_t    ETH_Mode;
};

//static ETH_DMADescTypeDef *DMARxDscrTab, *DMATxDscrTab;
//static rt_uint8_t *Rx_Buff, *Tx_Buff;
static  ETH_HandleTypeDef EthHandle;
static struct rt_stm32_eth stm32_eth_device;
ETH_TxPacketConfig TxConfig;
uint32_t current_pbuf_idx =0;

#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */ 

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

/* Memory Pool Declaration */
LWIP_MEMPOOL_DECLARE(RX_POOL, 10, sizeof(struct pbuf_custom), "Zero-copy RX PBUF pool");
//struct pbuf_custom rx_pbuf[ETH_RX_DESC_CNT];

void pbuf_free_custom(struct pbuf *p);

#if defined(ETH_RX_DUMP) || defined(ETH_TX_DUMP)
#define __is_print(ch) ((unsigned int)((ch) - ' ') < 127u - ' ')
static void dump_hex(const rt_uint8_t *ptr, rt_size_t buflen)
{
    unsigned char *buf = (unsigned char *)ptr;
    int i, j;

    for (i = 0; i < buflen; i += 16)
    {
        rt_kprintf("%08X: ", i);

        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                rt_kprintf("%02X ", buf[i + j]);
            else
                rt_kprintf("   ");
        rt_kprintf(" ");

        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                rt_kprintf("%c", __is_print(buf[i + j]) ? buf[i + j] : '.');
        rt_kprintf("\n");
    }
}
#endif

extern void phy_reset(void);
/* EMAC initialization function */
static rt_err_t rt_stm32_eth_init(rt_device_t dev)
{
    rt_uint32_t idx = 0;
    ETH_MACConfigTypeDef MACConf;
//    __HAL_RCC_ETH_CLK_ENABLE();

    phy_reset();

    /* ETHERNET Configuration */
    EthHandle.Instance = ETH;
    EthHandle.Init.MACAddr = (rt_uint8_t *)&stm32_eth_device.dev_addr[0];
//    EthHandle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_DISABLE;
//    EthHandle.Init.Speed = ETH_SPEED_100M;
//    EthHandle.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
    EthHandle.Init.MediaInterface = HAL_ETH_RMII_MODE;
//    EthHandle.Init.RxMode = ETH_RXINTERRUPT_MODE;
//#ifdef RT_LWIP_USING_HW_CHECKSUM
//    EthHandle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
//#else
//    EthHandle.Init.ChecksumMode = ETH_CHECKSUM_BY_SOFTWARE;
//#endif
    EthHandle.Init.TxDesc = DMATxDscrTab;
    EthHandle.Init.RxDesc = DMARxDscrTab;
    EthHandle.Init.RxBuffLen = ETH_RX_BUFFER_SIZE;

//    HAL_ETH_DeInit(&EthHandle);

    /* configure ethernet peripheral (GPIOs, clocks, MAC, DMA) */
    if (HAL_ETH_Init(&EthHandle) != HAL_OK)
    {
        LOG_E("eth hardware init failed");
        return -RT_ERROR;
    }
    else
    {
        LOG_D("eth hardware init success");
    }
//    HAL_ETH_SetMDIOClockRange(&EthHandle);

//    /* Initialize Tx Descriptors list: Chain Mode */
//    HAL_ETH_DMATxDescListInit(&EthHandle, DMATxDscrTab, Tx_Buff, ETH_TXBUFNB);

//    /* Initialize Rx Descriptors list: Chain Mode  */
//    HAL_ETH_DMARxDescListInit(&EthHandle, DMARxDscrTab, Rx_Buff, ETH_RXBUFNB);

    memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
    TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
    TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

    /* End ETH HAL Init */

    /* Initialize the RX POOL */
    LWIP_MEMPOOL_INIT(RX_POOL);
    
    for(idx = 0; idx < ETH_RX_DESC_CNT; idx ++)
    {
        HAL_ETH_DescAssignMemory(&EthHandle, idx, Rx_Buff[idx], NULL);
//        rx_pbuf[idx].custom_free_function=pbuf_free_custom;
    } 
    
    /* ETH interrupt Init */
    HAL_NVIC_SetPriority(ETH_IRQn, 0x07, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
    
    HAL_ETH_GetMACConfig(&EthHandle, &MACConf); 
    MACConf.DuplexMode = ETH_FULLDUPLEX_MODE;
    MACConf.Speed = ETH_SPEED_100M;
    HAL_ETH_SetMACConfig(&EthHandle, &MACConf);

    /* Enable MAC and DMA transmission and reception */
    if (HAL_ETH_Start_IT(&EthHandle) == HAL_OK)
    {
        LOG_D("emac hardware start");
    }
    else
    {
        LOG_E("emac hardware start faild");
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t rt_stm32_eth_open(rt_device_t dev, rt_uint16_t oflag)
{
    LOG_D("emac open");
    return RT_EOK;
}

static rt_err_t rt_stm32_eth_close(rt_device_t dev)
{
    LOG_D("emac close");
    return RT_EOK;
}

static rt_size_t rt_stm32_eth_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    LOG_D("emac read");
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_size_t rt_stm32_eth_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    LOG_D("emac write");
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_err_t rt_stm32_eth_control(rt_device_t dev, int cmd, void *args)
{
    switch (cmd)
    {
    case NIOCTL_GADDR:
        /* get mac address */
        if (args) rt_memcpy(args, stm32_eth_device.dev_addr, 6);
        else return -RT_ERROR;
        break;

    default :
        break;
    }

    return RT_EOK;
}

/* ethernet device interface */
/* transmit data*/
rt_err_t rt_stm32_eth_tx(rt_device_t dev, struct pbuf *p)
{
  uint32_t i=0, framelen = 0;
  struct pbuf *q;
  err_t errval = ERR_OK;
  ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT];
  
  memset(Txbuffer, 0 , ETH_TX_DESC_CNT*sizeof(ETH_BufferTypeDef));
  
  for(q = p; q != NULL; q = q->next)
  {
    if(i >= ETH_TX_DESC_CNT)	
      return ERR_IF;
    
    Txbuffer[i].buffer = q->payload;
    Txbuffer[i].len = q->len;
    framelen += q->len;
    
    if(i>0)
    {
      Txbuffer[i-1].next = &Txbuffer[i];
    }
    
    if(q->next == NULL)
    {
      Txbuffer[i].next = NULL;
    }
    
    i++;
  }

  TxConfig.Length = framelen;
  TxConfig.TxBuffer = Txbuffer;
#ifdef ETH_TX_DUMP
    dump_hex(p->payload, p->tot_len);
#endif
  SCB_CleanInvalidateDCache();
  HAL_ETH_Transmit(&EthHandle, &TxConfig, ETH_DMA_TRANSMIT_TIMEOUT);
  
  return errval;
}

/**
  * @brief  Custom Rx pbuf free callback
  * @param  pbuf: pbuf to be freed
  * @retval None
  */
void pbuf_free_custom(struct pbuf *p)
{
  struct pbuf_custom* custom_pbuf = (struct pbuf_custom*)p;
  
#if !defined(DUAL_CORE) || defined(CORE_CM7)
  /* Invalidate data cache: lwIP and/or application may have written into buffer */
  SCB_InvalidateDCache_by_Addr((uint32_t *)p->payload, p->tot_len);
#endif
  
  LWIP_MEMPOOL_FREE(RX_POOL, custom_pbuf);
}

/* receive data*/
struct pbuf *rt_stm32_eth_rx(rt_device_t dev)
{
  struct pbuf *p = NULL;
  ETH_BufferTypeDef RxBuff;
  uint32_t framelength = 0;
  struct pbuf_custom* custom_pbuf;
  
  
  if (HAL_ETH_IsRxDataAvailable(&EthHandle))
  {
      SCB_CleanInvalidateDCache();
    HAL_ETH_GetRxDataBuffer(&EthHandle, &RxBuff);
    HAL_ETH_GetRxDataLength(&EthHandle, &framelength);
    
    /* Build Rx descriptor to be ready for next data reception */
    HAL_ETH_BuildRxDescriptors(&EthHandle);

#if !defined(DUAL_CORE) || defined(CORE_CM7)
    /* Invalidate data cache for ETH Rx Buffers */
    SCB_InvalidateDCache_by_Addr((uint32_t *)RxBuff.buffer, framelength);
#endif
    
    custom_pbuf  = (struct pbuf_custom*)LWIP_MEMPOOL_ALLOC(RX_POOL);
    custom_pbuf->custom_free_function = pbuf_free_custom;
    
    p = pbuf_alloced_custom(PBUF_RAW, framelength, PBUF_REF, custom_pbuf, RxBuff.buffer, ETH_RX_BUFFER_SIZE);
#ifdef ETH_TX_DUMP
    dump_hex(p->payload, p->tot_len);
#endif    
    return p;
  }
  else
  {
    return NULL;
  }
}

/* interrupt service routine */
void ETH_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_ETH_IRQHandler(&EthHandle);

    /* leave interrupt */
    rt_interrupt_leave();
}

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
    rt_err_t result;
    result = eth_device_ready(&(stm32_eth_device.parent));
    if (result != RT_EOK)
        LOG_E("RX err = %d", result);
}

void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth)
{
    LOG_E("eth err");
}

#ifdef PHY_USING_INTERRUPT_MODE
static void eth_phy_isr(void *args)
{
    rt_uint32_t status = 0;
    static rt_uint8_t link_status = 1;

    HAL_ETH_ReadPHYRegister(&EthHandle, PHY_INTERRUPT_FLAG_REG, (uint32_t *)&status);
    LOG_D("phy interrupt status reg is 0x%X", status);
    HAL_ETH_ReadPHYRegister(&EthHandle, PHY_BASIC_STATUS_REG, (uint32_t *)&status);
    LOG_D("phy basic status reg is 0x%X", status);

    if (status & PHY_LINKED_STATUS_MASK)
    {
        if (link_status == 0)
        {
            link_status = 1;
            LOG_D("link up");
            /* send link up. */
            eth_device_linkchange(&stm32_eth_device.parent, RT_TRUE);
        }
    }
    else
    {
        if (link_status == 1)
        {
            link_status = 0;
            LOG_I("link down");
            /* send link down. */
            eth_device_linkchange(&stm32_eth_device.parent, RT_FALSE);
        }
    }
}
#endif /* PHY_USING_INTERRUPT_MODE */

static uint8_t phy_speed = 0;
#define PHY_LINK_MASK       (1<<0)
static void phy_monitor_thread_entry(void *parameter)
{
    uint8_t phy_addr = 0xFF;
    uint8_t phy_speed_new = 0;
    rt_uint32_t status = 0;
    uint8_t detected_count = 0;

    while(phy_addr == 0xFF)
    {
        /* phy search */
        rt_uint32_t i, temp;
        for (i = 0; i <= 0x1F; i++)
        {
//            EthHandle.Init.PhyAddress = i;
            HAL_ETH_ReadPHYRegister(&EthHandle, i, PHY_ID1_REG, (uint32_t *)&temp);

            if (temp != 0xFFFF && temp != 0x00)
            {
                phy_addr = i;
                break;
            }
        }

        detected_count++;
        rt_thread_mdelay(1000);

        if (detected_count > 10)
        {
            LOG_E("No PHY device was detected, please check hardware!");
        }
    }

    LOG_D("Found a phy, address:0x%02X", phy_addr);

    /* RESET PHY */
    LOG_D("RESET PHY!");
    HAL_ETH_WritePHYRegister(&EthHandle, phy_addr, PHY_BASIC_CONTROL_REG, PHY_RESET_MASK);
    rt_thread_mdelay(2000);
    HAL_ETH_WritePHYRegister(&EthHandle, phy_addr, PHY_BASIC_CONTROL_REG, PHY_AUTO_NEGOTIATION_MASK);

    while (1)
    {
        HAL_ETH_ReadPHYRegister(&EthHandle, phy_addr, PHY_BASIC_STATUS_REG, (uint32_t *)&status);
        LOG_D("PHY BASIC STATUS REG:0x%04X", status);

        phy_speed_new = 0;

        if (status & (PHY_AUTONEGO_COMPLETE_MASK | PHY_LINKED_STATUS_MASK))
        {
            rt_uint32_t SR;

            phy_speed_new = PHY_LINK_MASK;

//            SR = HAL_ETH_ReadPHYRegister(&EthHandle, PHY_Status_REG, (uint32_t *)&SR);
            HAL_ETH_ReadPHYRegister(&EthHandle, phy_addr, PHY_Status_REG, (uint32_t *)&SR);
            LOG_D("PHY Control/Status REG:0x%04X ", SR);

            if (SR & PHY_100M_MASK)
            {
                phy_speed_new |= PHY_100M_MASK;
            }
            else if (SR & PHY_10M_MASK)
            {
                phy_speed_new |= PHY_10M_MASK;
            }

            if (SR & PHY_FULL_DUPLEX_MASK)
            {
                phy_speed_new |= PHY_FULL_DUPLEX_MASK;
            }
        }

        /* linkchange */
        if (phy_speed_new != phy_speed)
        {
            if (phy_speed_new & PHY_LINK_MASK)
            {
                LOG_D("link up ");

                if (phy_speed_new & PHY_100M_MASK)
                {
                    LOG_D("100Mbps");
                    stm32_eth_device.ETH_Speed = ETH_SPEED_100M;
                }
                else
                {
                    stm32_eth_device.ETH_Speed = ETH_SPEED_10M;
                    LOG_D("10Mbps");
                }

                if (phy_speed_new & PHY_FULL_DUPLEX_MASK)
                {
                    LOG_D("full-duplex");
                    stm32_eth_device.ETH_Mode = ETH_FULLDUPLEX_MODE;
                }
                else
                {
                    LOG_D("half-duplex");
                    stm32_eth_device.ETH_Mode = ETH_HALFDUPLEX_MODE;
                }

                /* send link up. */
                eth_device_linkchange(&stm32_eth_device.parent, RT_TRUE);

#ifdef PHY_USING_INTERRUPT_MODE
                /* configuration intterrupt pin */
                rt_pin_mode(PHY_INT_PIN, PIN_MODE_INPUT_PULLUP);
                rt_pin_attach_irq(PHY_INT_PIN, PIN_IRQ_MODE_FALLING, eth_phy_isr, (void *)"callbackargs");
                rt_pin_irq_enable(PHY_INT_PIN, PIN_IRQ_ENABLE);

                /* enable phy interrupt */
                HAL_ETH_WritePHYRegister(&EthHandle, PHY_INTERRUPT_MSAK_REG, PHY_INT_MASK);

                break;
#endif
            } /* link up. */
            else
            {
                LOG_I("link down");
                /* send link down. */
                eth_device_linkchange(&stm32_eth_device.parent, RT_FALSE);
            }
            phy_speed = phy_speed_new;
        }

        rt_thread_delay(RT_TICK_PER_SECOND);
    }
}

/* Register the EMAC device */
static int rt_hw_stm32_eth_init(void)
{
    rt_err_t state = RT_EOK;

//    /* Prepare receive and send buffers */
//    Rx_Buff = (rt_uint8_t *)rt_calloc(ETH_RXBUFNB, ETH_MAX_PACKET_SIZE);
//    if (Rx_Buff == RT_NULL)
//    {
//        LOG_E("No memory");
//        state = -RT_ENOMEM;
//        goto __exit;
//    }

//    Tx_Buff = (rt_uint8_t *)rt_calloc(ETH_TXBUFNB, ETH_MAX_PACKET_SIZE);
//    if (Rx_Buff == RT_NULL)
//    {
//        LOG_E("No memory");
//        state = -RT_ENOMEM;
//        goto __exit;
//    }

//    DMARxDscrTab = (ETH_DMADescTypeDef *)rt_calloc(ETH_RXBUFNB, sizeof(ETH_DMADescTypeDef));
//    if (DMARxDscrTab == RT_NULL)
//    {
//        LOG_E("No memory");
//        state = -RT_ENOMEM;
//        goto __exit;
//    }

//    DMATxDscrTab = (ETH_DMADescTypeDef *)rt_calloc(ETH_TXBUFNB, sizeof(ETH_DMADescTypeDef));
//    if (DMATxDscrTab == RT_NULL)
//    {
//        LOG_E("No memory");
//        state = -RT_ENOMEM;
//        goto __exit;
//    }

    stm32_eth_device.ETH_Speed = ETH_SPEED_100M;
    stm32_eth_device.ETH_Mode  = ETH_FULLDUPLEX_MODE;

    /* OUI 00-80-E1 STMICROELECTRONICS. */
    stm32_eth_device.dev_addr[0] = 0x98;
    stm32_eth_device.dev_addr[1] = 0xFA;
    stm32_eth_device.dev_addr[2] = 0x9B;
    /* generate MAC addr from 96bit unique ID (only for test). */
    stm32_eth_device.dev_addr[3] = 0x64;
    stm32_eth_device.dev_addr[4] = 0x4C;
    stm32_eth_device.dev_addr[5] = 0xA2;

    stm32_eth_device.parent.parent.init       = rt_stm32_eth_init;
    stm32_eth_device.parent.parent.open       = rt_stm32_eth_open;
    stm32_eth_device.parent.parent.close      = rt_stm32_eth_close;
    stm32_eth_device.parent.parent.read       = rt_stm32_eth_read;
    stm32_eth_device.parent.parent.write      = rt_stm32_eth_write;
    stm32_eth_device.parent.parent.control    = rt_stm32_eth_control;
    stm32_eth_device.parent.parent.user_data  = RT_NULL;

    stm32_eth_device.parent.eth_rx     = rt_stm32_eth_rx;
    stm32_eth_device.parent.eth_tx     = rt_stm32_eth_tx;

    /* register eth device */
    state = eth_device_init(&(stm32_eth_device.parent), "e0");
    if (RT_EOK == state)
    {
        LOG_D("emac device init success");
    }
    else
    {
        LOG_E("emac device init faild: %d", state);
        state = -RT_ERROR;
        goto __exit;
    }

    /* start phy monitor */
    rt_thread_t tid;
    tid = rt_thread_create("phy",
                           phy_monitor_thread_entry,
                           RT_NULL,
                           1024,
                           RT_THREAD_PRIORITY_MAX - 2,
                           2);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        state = -RT_ERROR;
    }
__exit:
    if (state != RT_EOK)
    {
//        if (Rx_Buff)
//        {
//            rt_free(Rx_Buff);
//        }

//        if (Tx_Buff)
//        {
//            rt_free(Tx_Buff);
//        }

//        if (DMARxDscrTab)
//        {
//            rt_free(DMARxDscrTab);
//        }

//        if (DMATxDscrTab)
//        {
//            rt_free(DMATxDscrTab);
//        }
    }

    return state;
}
INIT_COMPONENT_EXPORT(rt_hw_stm32_eth_init);
