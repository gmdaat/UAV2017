/******************** (C) COPYRIGHT 2015 FTC ***************************
 * 作者		 ：FTC
 * 文件名  ：FTC_Drv_LED.cpp
 * 描述    ：LED
**********************************************************************************/
#include "FTC_Drv_LED.h"

FTC_LED led;

void FTC_LED::Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure1;
	GPIO_InitTypeDef  GPIO_InitStructure2;

	RCC_APB2PeriphClockCmd( FTC_RCC_LED1,ENABLE);
	RCC_APB2PeriphClockCmd( FTC_RCC_LED2,ENABLE);

	GPIO_InitStructure1.GPIO_Pin  = FTC_Pin_LED1;
	GPIO_InitStructure2.GPIO_Pin  = FTC_Pin_LED2;
	
	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_Out_PP;
	
	GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
		
	GPIO_Init(FTC_GPIO_LED1, &GPIO_InitStructure1);
	GPIO_Init(FTC_GPIO_LED2, &GPIO_InitStructure2);
}

void FTC_LED::ON1(void)
{
	GPIO_SetBits(FTC_GPIO_LED1, FTC_Pin_LED1);	
}

void FTC_LED::ON2(void)
{		
	GPIO_SetBits(FTC_GPIO_LED2, FTC_Pin_LED2);	
}

void FTC_LED::OFF1(void)
{
	GPIO_ResetBits(FTC_GPIO_LED1, FTC_Pin_LED1);
}

void FTC_LED::OFF2(void)
{
	GPIO_ResetBits(FTC_GPIO_LED2, FTC_Pin_LED2);
}


/******************* (C) COPYRIGHT 2015 FTC *****END OF FILE************/

