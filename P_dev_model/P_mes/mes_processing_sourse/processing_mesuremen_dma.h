/*
 processing_mesuremen_adc_dma.h
 *
 *  Created on: December 21, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#ifndef  PROCESSING_MESUREMENT_DMA_H_
#define  PROCESSING_MESUREMENT_DMA_H_

#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "FreeRTOS.h"
#include "semphr.h"


typedef enum{
	COPY_NOT_FINISHED=0,
	COPY_OK=1,
	COPY_ERROR=2
}REZ_COPY_DATA;


typedef struct{
	u16 *p_mem_addres; // pointer to target mem
	u16 num_data;      // size mem array
}S_dma_run_param;


void processing_mesuremen_dma_init(void);
void processing_mesuremen_dma_conf_adc(void);
void processing_mesuremen_dma_conf_m2m(void);
REZ_COPY_DATA copy_data_dma(u8 *p_data_from, u8 *p_data_to, u16 length);
void processing_mesuremen_dma_mem(S_dma_run_param *ps_dma_par);
void processing_mesuremen_dma_run_adc(void);



#endif
