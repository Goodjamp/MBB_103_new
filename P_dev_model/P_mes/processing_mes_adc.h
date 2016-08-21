/*
 * processing_mes_adc.h
 *
 *  Created on: March 17, 2016
 *      Author: Gerasimchuk
 */

#ifndef PROCESSING_MES_ADC_H_
#define PROCESSING_MES_ADC_H_


#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"


#define ADC_DMA_CHANNEL     DMA1_Channel1

//
typedef void (*adc_dma_callback)(void);

typedef struct{
	adc_dma_callback callbac_fun;
	u8               fun_is_set;
}S_processing_mes_adc_dma_callback;

typedef struct{
	u32 F_adc;           //Period ADC conversion, us
	ADC_TypeDef *ADC;
	u16 channel_number;
}S_ADC_init;

typedef struct{
	u16 adc_buffer_size;    // full buffer results size
	u16 rez_buffer_size;    // half buffer results size
	u16 *p_buf_0;       // pointer FIREST part ADC buffer
	u16 *p_buf_1;       // pointer SECOND part ADC buffer
	u16 *p_valid_buf;   // pointer temporary buffer (buffer with valid valid results)
	volatile u8 f_mes_complete;
}S_Buffer_result;


typedef enum{
	ADC_OK=0,
	ADC_ERROR_ADC=1
}PROCESSING_MES_ADC_STATUS;


PROCESSING_MES_ADC_STATUS processing_mes_adc_config(S_ADC_init const* const ps_adc_init,S_Buffer_result *pconstps_buffer_result );
PROCESSING_MES_ADC_STATUS processing_mes_adc_config_adc(S_ADC_init const* const ps_adc_init);
PROCESSING_MES_ADC_STATUS processing_mes_adc_config_tim(S_ADC_init const* const ps_adc_init);
PROCESSING_MES_ADC_STATUS processing_mes_adc_config_dma_adc(S_Buffer_result const* const ps_buffer_result);
void processing_mes_adc_set_dma_callback(adc_dma_callback callback_fun);




#endif
