#include "dma_m2m.h"
#include "stm32f10x_gpio.h"
 S_dma_m2m dma_m2m_status={0};
static  DMA_Channel_TypeDef *volatile sel_dma_m2m;
static volatile u8 status=0;

// --------------processing_mes_adc_config_dma_m2m--------------------------
DMA_M2M_STATYS dma_m2m_init(DMA_Channel_TypeDef *dma_ch){
	DMA_InitTypeDef dma_inittypedef;
	if(!IS_DMA_CH(dma_ch)){return DMA_M2M_BAD_CHANNEL;}

	sel_dma_m2m=dma_ch;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	// Configuration DMA:
	// dma mode:      normal
	// dma direction: mem2mem
	// size transmit: 16 bit (half word)
	// priority:      highest
	dma_inittypedef.DMA_BufferSize=0;
	dma_inittypedef.DMA_DIR=DMA_DIR_PeripheralSRC; // from peripheral
	dma_inittypedef.DMA_M2M=DMA_M2M_Enable;
	dma_inittypedef.DMA_Mode=DMA_Mode_Normal;
	dma_inittypedef.DMA_MemoryBaseAddr=(u32)0;
	dma_inittypedef.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	dma_inittypedef.DMA_MemoryInc=DMA_MemoryInc_Enable;
	dma_inittypedef.DMA_PeripheralBaseAddr=(u32)0;
	dma_inittypedef.DMA_PeripheralDataSize=DMA_MemoryDataSize_Byte;
	dma_inittypedef.DMA_PeripheralInc=DMA_PeripheralInc_Enable;
	dma_inittypedef.DMA_Priority=DMA_Priority_VeryHigh;
	// init DMA
	DMA_Init(sel_dma_m2m,&dma_inittypedef);
	//Config interupt:
	DMA_ITConfig(sel_dma_m2m,DMA_IT_TC,ENABLE);  // transmit complete
 	NVIC_EnableIRQ(DMA_M2M_IRQ);
 	return DMA_M2M_OK;
}


void DMA1_Channel2_IRQHandler(void){
	DMA_ClearITPendingBit(DMA1_IT_GL2);
	DMA_Cmd(sel_dma_m2m,DISABLE);
	dma_m2m_status.f_transmit_status=DMA_M2M_OK;
	status=0;
	GPIO_ResetBits(GPIOB,GPIO_Pin_9);
}


DMA_M2M_STATYS dma_m2m_get_statys(void){
	//NVIC_DisableIRQ(DMA_M2M_IRQ);
	__disable_irq();
	DMA_M2M_STATYS temp_dma_statys=dma_m2m_status.f_transmit_status;
	//NVIC_EnableIRQ(DMA_M2M_IRQ);
	__enable_irq();

	return temp_dma_statys;
}

//

DMA_M2M_STATYS memcopy_dma(DMA_M2M_TX_SIZE size_var,u16 size_data,void *pdsrc, void *pdst){

	GPIO_SetBits(GPIOB,GPIO_Pin_9);

	if(dma_m2m_status.f_transmit_status){return DMA_M2M_BUSY;}

	DMA1_Channel2->CNDTR=size_data;
	DMA1_Channel2->CPAR=(u32)pdsrc;
	DMA1_Channel2->CMAR=(u32)pdst;
	DMA1_Channel2->CCR &=~(DMA_CCR1_MSIZE | DMA_CCR1_PSIZE);
	switch(size_var){
	case DMA_TX_1BYTE: break;
	case DMA_TX_2BYTE:
		DMA1_Channel2->CCR |= (DMA_CCR1_MSIZE_0 | DMA_CCR1_PSIZE_0);
		break;
	case DMA_TX_3BYTE:
		DMA1_Channel2->CCR |= (DMA_CCR1_MSIZE_1 | DMA_CCR1_PSIZE_1);
		break;
	}
	// Enable DMA
	DMA_Cmd(sel_dma_m2m,ENABLE);
	// set busy flag
	dma_m2m_status.f_transmit_status=DMA_M2M_BUSY;
	status=1;
 	while(status){}
	return DMA_M2M_OK;
}
