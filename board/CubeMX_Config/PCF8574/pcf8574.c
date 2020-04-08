#include "pcf8574.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F7开发板
//PCF8574驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/1/13
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
extern I2C_HandleTypeDef hi2c2;
//初始化PCF8574
uint8_t PCF8574_Init(void)
{
    uint8_t temp=0;
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();           //使能GPIOB时钟
	
    GPIO_Initure.Pin=GPIO_PIN_12;           //PB12
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);     //初始化
    
    PCF8574_WriteOneByte(0XFF);	            //默认情况下所有IO输出高电平
	return temp;
}

//读取PCF8574的8位IO值
//返回值:读到的数据
uint8_t PCF8574_ReadOneByte(void)
{				  
	uint8_t temp=0;		  	    																 
    HAL_I2C_Master_Receive(&hi2c2, PCF8574_ADDR, &temp, 1, 3000);
	return temp;
}
//向PCF8574写入8位IO值  
//DataToWrite:要写入的数据
void PCF8574_WriteOneByte(uint8_t DataToWrite)
{				   	  	    																 
    HAL_I2C_Master_Transmit(&hi2c2, PCF8574_ADDR, &DataToWrite, 1, 3000);
	HAL_Delay(10);	 
}

//设置PCF8574某个IO的高低电平
//bit:要设置的IO编号,0~7
//sta:IO的状态;0或1
void PCF8574_WriteBit(uint8_t bit,uint8_t sta)
{
    uint8_t data;
    data=PCF8574_ReadOneByte(); //先读出原来的设置
    if(sta==0)data&=~(1<<bit);     
    else data|=1<<bit;
    PCF8574_WriteOneByte(data); //写入新的数据
}

//读取PCF8574的某个IO的值
//bit：要读取的IO编号,0~7
//返回值:此IO的值,0或1
uint8_t PCF8574_ReadBit(uint8_t bit)
{
    uint8_t data;
    data=PCF8574_ReadOneByte(); //先读取这个8位IO的值 
    if(data&(1<<bit))return 1;
    else return 0;   
}  
    
