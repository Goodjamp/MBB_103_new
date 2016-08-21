/*
 * processing_mes_adc.c
 *
 *  Created on: March 17, 2016
 *      Author: Gerasimchuk
 */


#include "processing_mes_adc.h"
S_Buffer_result *p_buffer_result;
static volatile S_processing_mes_adc_dma_callback s_processing_mes_adc_dma_callback={.fun_is_set=0};


PROCESSING_MES_ADC_STATUS processing_mes_adc_config(S_ADC_init const* const ps_adc_init, S_Buffer_result *pconstps_buffer_result ){
	p_buffer_result=pconstps_buffer_result;
	processing_mes_adc_config_adc(ps_adc_init);
	processing_mes_adc_config_dma_adc(pconstps_buffer_result);
	processing_mes_adc_config_tim(ps_adc_init);

	return ADC_OK;
}

// --------------processing_mes_adc_config_adc--------------------------
//function processing_mes_adc_config_adc - full configuration ADC
// input arg:
// ps_adc_init - pointer of structure that combine ADC configuration parameters
PROCESSING_MES_ADC_STATUS processing_mes_adc_config_adc(S_ADC_init const* const ps_adc_init){
	u32 counter_calib=0;
	GPIO_InitTypeDef gpio_init;
	ADC_InitTypeDef adc_init_typedef;
    //Check input parameters
	if(ps_adc_init->ADC==ADC1){	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);}
	else if(ps_adc_init->ADC==ADC2){RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,ENABLE);}
	else{return ADC_ERROR_ADC;}

	// Initialization ADC_1
	ADC_Cmd(ps_adc_init->ADC,ENABLE);

	//Calibration ADC
	ADC_StartCalibration(ps_adc_init->ADC);
	// Waiting calibration is complete
	while(ADC_GetCalibrationStatus(ps_adc_init->ADC)){
		counter_calib++;
		if(counter_calib>100000){return 0;}
	};
	// Triggering by Timer1_CC1, one channel, single mode
	adc_init_typedef.ADC_ContinuousConvMode=DISABLE;     // single conversion mode
	adc_init_typedef.ADC_DataAlign=ADC_DataAlign_Right;  //
	adc_init_typedef.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T2_CC2;  // source of triggering
	adc_init_typedef.ADC_Mode=ADC_Mode_Independent;  // Independent (not dual)mode
	adc_init_typedef.ADC_NbrOfChannel=1;   // 1 channel
	adc_init_typedef.ADC_ScanConvMode=DISABLE;       // 2

	ADC_Init(ps_adc_init->ADC,&adc_init_typedef);
	ADC_ExternalTrigConvCmd(ps_adc_init->ADC,ENABLE);

	// configuration channel
	ADC_RegularChannelConfig(ps_adc_init->ADC,ps_adc_init->channel_number,1,ADC_SampleTime_239Cycles5);
	ps_adc_init->ADC->SQR1&=~(ADC_SQR1_L);
	//ADC  GPIO configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	gpio_init.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	gpio_init.GPIO_Speed=GPIO_Speed_2MHz;
	// select gpio according with number of ADC input and making configuration it
	switch (ps_adc_init->channel_number){
	case 0 ... 7:
				gpio_init.GPIO_Pin=(1<<ps_adc_init->channel_number);
				GPIO_Init(GPIOA,&gpio_init);
				break;
	case 8 ... 9:
				gpio_init.GPIO_Pin=(1<<(ps_adc_init->channel_number-8));
				GPIO_Init(GPIOB,&gpio_init);
				break;
	case 10 ... 15:
				gpio_init.GPIO_Pin=(1<<(ps_adc_init->channel_number-10));
				GPIO_Init(GPIOC,&gpio_init);
				break;
	}
	//Enable DMA
	ADC_DMACmd(ps_adc_init->ADC,ENABLE);
	return ADC_OK;
}



PROCESSING_MES_ADC_STATUS processing_mes_adc_config_tim(S_ADC_init const* const ps_adc_init){
	u16 ccr2_calc;
	RCC_ClocksTypeDef rcc_clock;

	RCC_GetClocksFreq(&rcc_clock);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM2->PSC=(rcc_clock.PCLK1_Frequency*2)/1000000-1;
	ccr2_calc=1000000/ps_adc_init->F_adc;
	TIM2->CCR2=ccr2_calc-1;
	TIM2->CR1|=TIM_CR1_ARPE;
	TIM2->ARR=TIM2->CCR2;
	TIM2->CCMR1 =0;
    TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // allow output channel operation
    //TIM2->CR2 |= TIM_CR2_OIS1;
    TIM2->CCER|=TIM_CCER_CC2E;
    TIM2->BDTR|=TIM_BDTR_MOE;


	TIM_ITConfig(TIM2,TIM_IT_CC2,ENABLE);
	TIM2->EGR|=TIM_EGR_UG;
	TIM2->SR=0;
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM_Cmd(TIM2,ENABLE);
	return ADC_OK;
}


void TIM2_IRQHandler(void){
	TIM2->SR=0;
}

// --------------function processing_mes_adc_config_dma_adc--------------------------
//function processing_mes_adc_config_dma_adc - configuration DMA for transmit ADC samples
// input arg:
// ps_buffer_result - pointer of structure that combine buffer configuration parameters
PROCESSING_MES_ADC_STATUS processing_mes_adc_config_dma_adc(S_Buffer_result const* const ps_buffer_result){
	DMA_InitTypeDef dma_inittypedef;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	// Configuration DMA:
	// dma mode:      circular
	// dma direction: from peripheral to memory
	// size transmit: 16 bit (half word)
	// priority:      highest
	dma_inittypedef.DMA_BufferSize=ps_buffer_result->adc_buffer_size;
	dma_inittypedef.DMA_DIR=DMA_DIR_PeripheralSRC;
	dma_inittypedef.DMA_M2M=DMA_M2M_Disable;
	dma_inittypedef.DMA_Mode=DMA_Mode_Circular;
	dma_inittypedef.DMA_MemoryBaseAddr=(u32)ps_buffer_result->p_buf_0;
	dma_inittypedef.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
	dma_inittypedef.DMA_MemoryInc=DMA_MemoryInc_Enable;
	dma_inittypedef.DMA_PeripheralBaseAddr=(u32)&ADC1->DR;
	dma_inittypedef.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	dma_inittypedef.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	dma_inittypedef.DMA_Priority=DMA_Priority_VeryHigh;
	// init DMA
	DMA_Init(ADC_DMA_CHANNEL,&dma_inittypedef);
	//Config interupt:
	DMA_ITConfig(ADC_DMA_CHANNEL,DMA_IT_TC,ENABLE);  // transmit complete
 	DMA_ITConfig(ADC_DMA_CHANNEL,DMA_IT_HT,ENABLE);  // half transmit complete
	NVIC_SetPriority(DMA1_Channel1_IRQn,13);
 	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	// Enable DMA
	DMA_Cmd(ADC_DMA_CHANNEL,ENABLE);

	return ADC_OK;
}


void processing_mes_adc_set_dma_callback(adc_dma_callback callback_fun){
	s_processing_mes_adc_dma_callback.callbac_fun=callback_fun;
	s_processing_mes_adc_dma_callback.fun_is_set=1;
}

//----------interrupt DMA1_Channel1_IRQHandler-------------
// interrupt DMA1_Channel1_IRQHandler - set pointer on last ADC result
void DMA1_Channel1_IRQHandler(void){
	if(DMA1->ISR & DMA_ISR_TCIF1){
		p_buffer_result->p_valid_buf=p_buffer_result->p_buf_1;
	}
	else if(DMA1->ISR & DMA_ISR_HTIF1){
		p_buffer_result->p_valid_buf=p_buffer_result->p_buf_0;
	}
	p_buffer_result->f_mes_complete=1;
	// Global clear interupt flag
	DMA_ClearITPendingBit(DMA1_IT_GL1);
	// start transmit last ADC data to filtration buffer
	if(s_processing_mes_adc_dma_callback.fun_is_set){
		s_processing_mes_adc_dma_callback.callbac_fun();
	}
}


