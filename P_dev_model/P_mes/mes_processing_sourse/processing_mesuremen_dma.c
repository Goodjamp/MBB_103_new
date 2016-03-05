/*
 *  processing_mesuremen_dma.h
 *
 *  Created on: December 21, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "processing_mesuremen_dma.h"
u16 data;
REZ_COPY_DATA rez_copy;
xSemaphoreHandle Semaphor_copy;





void processing_mesuremen_dma_init(void){
	processing_mesuremen_dma_conf_adc();
	processing_mesuremen_dma_conf_m2m();
}


//*********************************************************************//
//-----------------processing_mesuremen_init_dma_adc------------------
//функция processing_mesuremen_init_dma_adc - выполняет конфигурацию канала 1 DMA1
// в режим переноса результатов ADC1  в память
void processing_mesuremen_dma_conf_adc(void){
	DMA_InitTypeDef dma_InitTypeDef;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	// -----------------НАСТРОЙКА ПДП ДЛЯ РАБОТЫ С АЦП-------------------
	// Настрока адресов и режима работы DMA
	// Read from peripheral copy to memory
	dma_InitTypeDef.DMA_BufferSize=0;
	dma_InitTypeDef.DMA_DIR=DMA_DIR_PeripheralSRC;
	dma_InitTypeDef.DMA_M2M=DMA_M2M_Disable;
	dma_InitTypeDef.DMA_MemoryBaseAddr=(u32)0;
	dma_InitTypeDef.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
	dma_InitTypeDef.DMA_MemoryInc=DMA_MemoryInc_Enable;
	dma_InitTypeDef.DMA_Mode=DMA_Mode_Circular;
	dma_InitTypeDef.DMA_PeripheralBaseAddr=(u32)&ADC1->DR;
	dma_InitTypeDef.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	dma_InitTypeDef.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	//
	DMA_Init(DMA1_Channel1,&dma_InitTypeDef);
	// настройка прерываний DMA
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE); // буффер заполнен полностью
	DMA_ITConfig(DMA1_Channel1,DMA_IT_HT,ENABLE); // буффер заполнен на половину
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);           //
}

// Установка адреса памяти и к-ва данных к копированию
void processing_mesuremen_dma_mem(S_dma_run_param *ps_dma_par){
	DMA1_Channel1->CNDTR=ps_dma_par->num_data;
	DMA1_Channel1->CMAR=(u32)ps_dma_par->p_mem_addres;
}

void processing_mesuremen_dma_run_adc(void){
	DMA_Cmd(DMA1_Channel1,ENABLE);
}



void DMA1_Channel1_IRQHandler(void){
	static u8 counter_int=0;
	if(counter_int<=1){
		counter_int++;
		return ;
	}
	if(DMA_GetITStatus(DMA1_IT_HT1)) // fill first part ADC buffer
	{
		DMA_ClearFlag(DMA1_IT_HT1);
		// copy data to ADC buffer result
		//copy_data_dma(,,);
	}
	else if(DMA_GetITStatus(DMA1_IT_TC1))// fill first part ADC buffer
	{
		DMA_ClearFlag(DMA1_IT_TC1);
	}
};




//*********************************************************************//
//-----------------processing_mesuremen_init_dma_m2m------------------
//функция processing_mesuremen_init_dma_m2m - выполняет конфигурацию канала 2 DMA1
// в режим копирования "из памяти в память"
void processing_mesuremen_dma_conf_m2m(void){
	DMA_InitTypeDef dma_InitTypeDef;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	// -----------------НАСТРОЙКА ПДП ДЛЯ КОПИРОВАНИЯ ИЗ ОДНОЙ ОБЛАСТИ ПАМЯТИ В ДРУГУЮ-----------
	// источник копирования - память
	// цель копирования     - переферия
	// Настрока адресов и режима работы DMA
	dma_InitTypeDef.DMA_BufferSize=0;
	dma_InitTypeDef.DMA_DIR=DMA_DIR_PeripheralDST;//  "Read from memory copy to peripheral"
	dma_InitTypeDef.DMA_M2M=DMA_M2M_Enable;
	dma_InitTypeDef.DMA_MemoryBaseAddr=(u32)0;
	dma_InitTypeDef.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	dma_InitTypeDef.DMA_MemoryInc=DMA_MemoryInc_Enable;
	dma_InitTypeDef.DMA_Mode=DMA_Mode_Normal;
	dma_InitTypeDef.DMA_PeripheralBaseAddr=(u32)0;
	dma_InitTypeDef.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
	dma_InitTypeDef.DMA_PeripheralInc=DMA_PeripheralInc_Enable;
	dma_InitTypeDef.DMA_Priority=DMA_Priority_High;
	DMA_Init(DMA1_Channel2,&dma_InitTypeDef);
	// настройка прерываний DMA
	DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE); // буффер заполнен полностью
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
};

void DMA1_Channel2_IRQHandler(void){
	if(DMA_GetITStatus(DMA1_IT_TC2)) // copy_sucsesful
	{
		rez_copy=COPY_OK;
	}
	else{
		rez_copy=COPY_ERROR;
	}
	DMA_Cmd(DMA1_Channel2,DISABLE);
	DMA_ClearITPendingBit(DMA1_IT_GL2); //global clear interrupt flag
}



//*********************************************************************//
//-----------------copy_data_dma------------------
//функция copy_data_dma - выполняет копирование данных и одной области памяти в другую
// входные аргументы:
// p_data_from - указатель на область ОТКУДА копирования
// p_data_to - указатель на область КУДА копирования
// length    - к-во данных к копированию
//выходные аргументы:
// REZ_COPY_DATA  - результат копирования
REZ_COPY_DATA copy_dma_data(u8 *p_data_from, u8 *p_data_to, u16 length){
	DMA1_Channel2->CNDTR=length;
	DMA1_Channel2->CMAR=(u32)p_data_from;
	DMA1_Channel2->CPAR=(u32)p_data_to;
	DMA_Cmd(DMA1_Channel2,ENABLE);
	rez_copy=COPY_NOT_FINISHED;
	while(!(rez_copy)){} // жду окончания копирования, прерывания DMA1
	return rez_copy;
}



