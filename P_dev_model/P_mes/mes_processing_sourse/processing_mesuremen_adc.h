/*
 processing_mesuremen_adc.h
 *
 *  Created on: December 21, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#ifndef  PROCESSING_MESUREMENT_ADC_H_
#define  PROCESSING_MESUREMENT_ADC_H_

#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "processing_mesuremen_dma.h"

//параметры АЦП из ДШ
#define MAX_ADC_C_PSC          3
#define MAX_QUANTITY_CHANEL 16
#define MIN_QUANTITY_CHANEL  1
#define MAX_SAMPLE_TIME      7
#define SIZE_1_SQR           5
#define SIZE_1_SMPR          3

// max time wait reset CALIB bit in CR2 register
#define WAIT_RESET_CALIB    10000


//ADC ERROR RETURN
typedef enum{
	REZ_OK=0,
	ERROR_C_PSC=1,
	ERROR_MAX_QUANTITY_CHANEL=2,
	ERROR_POIN_ADC=3,
	ERROR_SAMPLE_TIME=5,
	ERROR_NUMBER_CHANEL=6,
	ERROR_COMON_ADC_PSC=7,
	ERROR_CALIB=8
}ADC_CONF_ERROR;





// структура конфигурации АЦП
typedef struct{
	u8  quantity_chanel;    // количество каналов для преобразования
	u8 num_chanel[16];     // номера каналов
	u8  time_mes[16];       // количество тактов для 1-го преобразования, поканально из поля 'num_mes'
	u16 comon_adc_psc;     // предделитель тактирования АЦП
}S_config_ADC;



ADC_CONF_ERROR processing_mesuremen_adc_conf_channel(ADC_TypeDef *ADC_IN, S_config_ADC s_config_ADC);
ADC_CONF_ERROR processing_mesuremen_adc_conf_mod(S_dma_run_param *ps_adc_buf_par);
void processing_mesuremen_adc_run(void);

#endif
