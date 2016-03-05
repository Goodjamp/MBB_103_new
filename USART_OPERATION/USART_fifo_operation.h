/*
 * USART_fifo_operation
 *
 *  Created on: Mach 27, 2015
 *      Author: Gerasimchuk
 */

#ifndef USART_FIFO_OPERATION_H_
#define USART_FIFO_OPERATION_H_
#include "stm32f10x_usart.h"
#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"


#define IT_TXEIE_USART						0x80
#define NUMBER_TXEIE_USART					7
#define TRANSMITION_IS_NOT_COMPLETE			-1

//--------------------------------MYSTRUCT-----------------------------------------------
struct point_wrdirobject
{
	unsigned char	*pBuf_UsartOut;
	unsigned short	size_wait_point_read;
	unsigned short	size_buffer;
	unsigned short	pointwrite;
	unsigned short	pointread;
	unsigned short	num_write;
	unsigned short	num_read;

};




 //----------------------MYFUNCTION---------------------------------------------------
 int 	write_fastdirstruct_usart(struct point_wrdirobject *pwrdirobject,unsigned char singlchar);
 int	read_fastdirstruct_usart(struct point_wrdirobject *pwrdirobject,uint8_t *singlchar);
 char 	UsartGetStatusCR1(USART_TypeDef *USARTx,uint16_t USART_IT);
 void 	UsartSetStatusCR1(USART_TypeDef* USARTx,uint16_t USART_IT);
 void 	UsartClearStatusCR1(USART_TypeDef* USARTx,uint16_t USART_IT);
 uint16_t write_Usart_Buf(USART_TypeDef* USARTx,uint8_t *bufuchar,uint16_t lenmassive,struct point_wrdirobject *pwrdirobject);
 int    transmit_Usart_buf(USART_TypeDef* USARTx,uint8_t *bufuchar,int lenstring);
 int	ReadUSART(struct point_wrdirobject *pwrdirobject,unsigned char *buffer,int numbyte,int MaxTime);
 int    Read_From(struct point_wrdirobject *pwrdirobject,uint8_t *buffer,int numbyte);

#endif
