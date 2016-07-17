/*
 * dma_m2m.h
 *
 *  Created on: March 20, 2016
 *      Author: Gerasimchuk
 */

#ifndef DMA_M2M_H_
#define DMA_M2M_H_

#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"

#define DMA_M2M_CHANNEL DMA1_Channel2
#define DMA_M2M_IRQ DMA1_Channel2_IRQn


#define 	DMA1_Ch1 DMA1_Channel1
#define 	DMA1_Ch2 DMA1_Channel2
#define 	DMA1_Ch3 DMA1_Channel3
#define 	DMA1_Ch4 DMA1_Channel4
#define 	DMA1_Ch5 DMA1_Channel5
#define 	DMA1_Ch6 DMA1_Channel6
#define 	DMA1_Ch7 DMA1_Channel7
#define 	DMA2_Ch1 DMA2_Channel1
#define 	DMA2_Ch2 DMA2_Channel2
#define 	DMA2_Ch3 DMA2_Channel3
#define 	DMA2_Ch4 DMA2_Channel4
#define 	DMA2_Ch5 DMA2_Channel5


#define IS_DMA_CH(X) ((X==DMA1_Channel1)||(X==DMA1_Channel2)||(X==DMA1_Channel3)||(X==DMA1_Channel4)||\
					  (X==DMA1_Channel5)||(X==DMA1_Channel6)||(X==DMA1_Channel7)||\
					  (X==DMA2_Channel1)||(X==DMA2_Channel2)||(X==DMA2_Channel3)||(X==DMA2_Channel4)||\
					  (X==DMA2_Channel5))



typedef struct{
	u8 f_transmit_status:4;
	u8 f_reserved_1:1;
	u8 f_reserved_2:1;
	u8 f_reserved_3:1;
	u8 f_reserved_4:1;
}S_dma_m2m;

typedef enum{
	DMA_TX_1BYTE=0,
	DMA_TX_2BYTE=1,
	DMA_TX_3BYTE=2
}DMA_M2M_TX_SIZE;


typedef enum{
	DMA_M2M_OK=0,
	DMA_M2M_BUSY=1,
	DMA_M2M_BAD_CHANNEL=2
}DMA_M2M_STATYS;

DMA_M2M_STATYS dma_m2m_init(DMA_Channel_TypeDef *dma_ch);
void DMA1_Channel2_IRQHandler(void);
DMA_M2M_STATYS memcopy_dma(DMA_M2M_TX_SIZE size_var,u16 size_data,void *pdsrc, void *pdst);
DMA_M2M_STATYS dma_m2m_get_statys(void);


#endif
