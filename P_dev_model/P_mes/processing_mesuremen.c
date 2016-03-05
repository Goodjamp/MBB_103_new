/*
 * processing_mesuremen.c
 *
 *  Created on:
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "processing_mesuremen.h"
u8 mem_from[10]={1,2,3,4,5,1,2,3,4,5};
u8 mem_to[10]={[0 ... 9] =0};
S_dma_run_param  s_adc_rez_buf;


//---------------задача t_processing_mesuremen---------------
// звдача t_processing_mesuremen - выполняет измерения частоты и напряжения
void t_processing_mesuremen(void) {

	S_config_ADC s_config_adc={0,
			                   {[0 ... 15]=0},
			                   {[0 ... 15]=0},
			                   0
	                          };
    RCC_ClocksTypeDef rcc_clocksTypeDef;
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	RCC_GetClocksFreq(&rcc_clocksTypeDef);
	processing_mesuremen_dma_init(); // конфигурация каналов ПДП


	s_config_adc.comon_adc_psc=RCC_PCLK2_Div8;
	s_config_adc.quantity_chanel=5;
	// channel №1
	s_config_adc.num_chanel[0]=1;
	s_config_adc.time_mes[0]=ADC_SampleTime_239Cycles5;
	// channel №2
	s_config_adc.num_chanel[1]=3;
	s_config_adc.time_mes[1]=ADC_SampleTime_239Cycles5;
	// channel №3
	s_config_adc.num_chanel[2]=5;
	s_config_adc.time_mes[2]=ADC_SampleTime_239Cycles5;
	// channel №4
	s_config_adc.num_chanel[3]=6;
	s_config_adc.time_mes[3]=ADC_SampleTime_239Cycles5;
	// channel №5
	s_config_adc.num_chanel[4]=8;
	s_config_adc.time_mes[4]=ADC_SampleTime_239Cycles5;

	processing_mesuremen_adc_conf_channel(ADC1,s_config_adc);
	processing_mesuremen_adc_conf_mod(&s_adc_rez_buf);
	processing_mesuremen_adc_run();


	while(1) // main loop mes task
	{

	}
}
