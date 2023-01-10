/*******************************************************************************
 *				 _ _                                             _ _
				|   |                                           (_ _)
				|   |        _ _     _ _   _ _ _ _ _ _ _ _ _ _   _ _
				|   |       |   |   |   | |    _ _     _ _    | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |_ _ _  |   |_ _|   | |   |   |   |   |   | |   |
				|_ _ _ _ _| |_ _ _ _ _ _| |_ _|   |_ _|   |_ _| |_ _|
								(C)2021 Lumi
 * Copyright (c) 2021
 * Lumi, JSC.
 * All Rights Reserved
 *
 * File name: main.c
 *
 * Description: Control bright led theo gia tri anh sang ma sensor do duoc.
 *
 * Author: CuuNV
 *
 * Last Changed By:  $Author: CuuNV$
 * Revision:         $Revision: $
 * Last Changed:     $Date: $January 9, 2023
 *
 * Code sample:
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include <stm32f401re_gpio.h>
#include <stm32f401re_rcc.h>
#include <stm32f401re_adc.h>
#include <stm32f401re_tim.h>
#include <kalman_filter.h>
#include <timer.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
void appInitCommon(void);
static void ledControlTimerOCInit (void);
static void lightSensorAdcInit(void);
static void ledControlTimerOcSetPwm(uint8_t byDuty);
static uint16_t lightSensorAdcPollingRead(void);
static uint16_t lightValueOfFilter(void);
static void ablStepBrightness(uint16_t wLightValue);
static void ablProcess(void);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/



int main(void)
{
	appInitCommon();

    /* Loop forever */
	while(1)
	{
		ablProcess();
	}
}
/*
 *
 *
 */
void appInitCommon(void)
{
	SystemCoreClockUpdate();
	TimerInit();
	ledControlTimerOCInit();
	lightSensorAdcInit();
	// (mea_e, est_e, q)
	KalmanFilterInit(2,2,0.001);
}
/*
 * Ham khoi tao chan PA1
 * Chuc nang su dung TIM2_CH2
 * Tan so cap PWM la 10kHz
 *
 */
static void ledControlTimerOCInit (void)
{
	GPIO_InitTypeDef 		GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef		TIM_OCInitStructure;

	// Led: PA1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);

	//Timer: TIM2_CH2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	// Timer dem len voi tan so 10kHz
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 4199;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	// dau ra se dao trang thai khi dem den auto-reload
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	//Trang thai dau ra se ve muc low neu dem den gia tri Compare
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//Khoi tao gia tri auto-reload bang 0
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC2Init(TIM2, &TIM_OCInitStructure);

	TIM_Cmd(TIM2, ENABLE);

	TIM_CtrlPWMOutputs(TIM2, ENABLE);
}
/*
 * Ham khoi tao chan PC5
 * Chuc nang ADC su dung ADC1_IN15
 * Mode doc du lieu lien tuc va don kenh
 * Do phan giai 12bit voi thoi gian lay mau la 15cycle
 * Thoi gian giua 2 lan lay mau la 5cycle
 */
static void lightSensorAdcInit(void)
{
	//ADC1_IN15-PC5
	GPIO_InitTypeDef 		GPIO_InitStructure;
	ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
	ADC_InitTypeDef 		ADC_InitStructure;
	//Config GPIO Pin:PC5,Mode:AN,Bus:AHP1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;

	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//Config ADC1 Bus: APB2
	ADC_DeInit();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	//Thoi gian tre giua 2 lay mau (84/2)/5 = 8.4uS
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;

	ADC_CommonInit(&ADC_CommonInitStructure);
	//Config Mode, ghi du lieu,do phan giai
		//1. Lay mau lien tuc
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
		//2.Ghi du lieu tu ben phai thanh ghi
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;

	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;

	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
		//3.So kenh chuyen doi = 1
	ADC_InitStructure.ADC_NbrOfConversion = 1;
		//4.Do phan giai bo ADC la 12bit
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
		//5.Chuyen doi don kenh
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;

	ADC_Init(ADC1, &ADC_InitStructure);

		//6.Khoi tao kenh dau vao cho bo ADC
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_15Cycles);

	ADC_Cmd(ADC1, ENABLE);

}
/*
 * Ham dieu khien do rong xung
 * Gia tri duty = 0~100;
 */
static void ledControlTimerOcSetPwm(uint8_t byDuty)
{
	static uint16_t wPulseLength = 0;

	wPulseLength = (uint16_t)((byDuty * 4199)/100);

	TIM_SetCompare2(TIM2, wPulseLength);
}
/*
 * Ham doc gia tri sensor tu bo ADC
 * Gia tri tra ve 0~4095
 */
static uint16_t lightSensorAdcPollingRead(void)
{
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)== RESET);

	return ADC_GetConversionValue(ADC1);
}
/*
 * Ham loc nhieu su dung bo loc kalman
 * Ham loc gia tri anh sang do duoc tu ham doc ADC
 * Gia tri tra ve 0~4095
 */
static uint16_t lightValueOfFilter(void)
{
	uint16_t wLight = lightSensorAdcPollingRead();
	return KalmanFilter_updateEstimate(wLight);
}
/*
 * Ham dieu khien muc sang LED
 * Dau vao la gia tri anh sang ma bo ADC doc duoc
 */
static void ablStepBrightness(uint16_t wLightValue)
{
	// value_in/4095 = value_out/100
	uint8_t byDuty;
	byDuty = (uint8_t)((wLightValue*100)/4095);
	ledControlTimerOcSetPwm(byDuty);
}
/*
 * Ham tao chu ky quet sensor
 * Thoi gian moi lan quet la 100ms
 * Sau khi quet duoc do sang tu bo loc kalman thi dieu khien muc
 * sang cua led
 * Anh sang cang lon thi muc sang cua led cang cao
 */
static void ablProcess(void)
{
	uint32_t dwTimeCurrent;
	static uint32_t dwTimeTotal, dwTimeInit;
	dwTimeCurrent = GetMilSecTick();

	if(dwTimeCurrent >= dwTimeInit)
	{
		dwTimeTotal += dwTimeCurrent - dwTimeInit;
	}else
	{
		dwTimeTotal += 0xFFFFFFFF - dwTimeCurrent + dwTimeInit;
	}

	if(dwTimeTotal >=100)
	{
		uint16_t wLight;
		wLight = lightValueOfFilter();
		ablStepBrightness(wLight);
		dwTimeTotal = 0;
	}
	dwTimeInit = dwTimeCurrent;
}
