#include "led.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////
// ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
// ALIENTEK mini?SSTM32������
// LED��������
// ����ԭ��@ALIENTEK
// ������̳:www.openedv.com
// �޸�����:2012/9/2
// �汾��V1.0
// ��Ȩ���У�����ؾ���
// Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
// All rights reserved
//////////////////////////////////////////////////////////////////////////////////

// ��ʼ��PB5��PE5Ϊ�����.��ʹ���������ڵ�ʱ��
// LED IO��ʼ��
void LED_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;		  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // �������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOE, &GPIO_InitStructure);			  // �����趨������ʼ��GPIOA.8
	GPIO_SetBits(GPIOE, GPIO_Pin_5);				  // PA.8 �����
}

void LED_Blink(void)
{
	GPIO_SetBits(GPIOE, GPIO_Pin_5);
	delay_ms(200);
	GPIO_ResetBits(GPIOE, GPIO_Pin_5);
	delay_ms(200);
}
