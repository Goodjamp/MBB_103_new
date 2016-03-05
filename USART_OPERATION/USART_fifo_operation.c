#include "USART_fifo_operation.h"


//extern S_Task_parameters *ptask_parameters;
extern int len, posbuf;
extern uint8_t *bufUSART;
extern volatile u16 counter_ms;




//----------------------------------MY	FUNCTION-------------------------------------------
char UsartGetStatusCR1(USART_TypeDef *USARTx, uint16_t USART_IT) {

	if (((uint16_t) (USARTx->CR1 & USART_IT)) != 0)
		return 1;
	else
		return 0;

}

void UsartSetStatusCR1(USART_TypeDef *USARTx, uint16_t USART_IT) {
	USARTx->CR1 = USARTx->CR1 | USART_IT;
}

void UsartClearStatusCR1(USART_TypeDef *USARTx, uint16_t USART_IT) {
	USARTx->CR1 = (USARTx->CR1 | USART_IT) ^ USART_IT;
}

uint16_t write_Usart_Buf(USART_TypeDef* USARTx, uint8_t *bufuchar,
		uint16_t lenmassive, struct point_wrdirobject *pwrdirobject) {
	unsigned char singlchar;
	uint16_t i;
	uint16_t usingle;
	int b;

	i = 0;
	while (i < lenmassive) {
		do {
			if ((*(bufuchar + i) < 0x30) || (*(bufuchar + i) > 0x39)) {
			}
			if ((b = write_fastdirstruct_usart(pwrdirobject, *(bufuchar + i)))
					!= 0)
				i++;

		} while ((b != 0) && (i < lenmassive));

		if ((UsartGetStatusCR1(USARTx, IT_TXEIE_USART)) != RESET) {

			//return i;
		} else {
			b = read_fastdirstruct_usart(pwrdirobject, &singlchar);
			//if()
			//if((singlchar<0x30)||(singlchar>0x39))
			if (b == 0) {
			}
			if (b == 1) {
				usingle = ((uint16_t) singlchar) & 0x00ff;
				//	__disable_irq();
				USART_SendData(USARTx, usingle);
				UsartSetStatusCR1(USARTx, IT_TXEIE_USART);
				//	__enable_irq();
			}

		}

	}
	return i;
}

int write_fastdirstruct_usart(struct point_wrdirobject *pwrdirobject,
		unsigned char singlchar) {
	unsigned char *pobject;
	unsigned short pwrite;
	unsigned short ocenka;

	pobject = pwrdirobject->pBuf_UsartOut;
	pwrite = pwrdirobject->pointwrite;
	ocenka = pwrite;
	if ((pwrdirobject->num_write >= pwrdirobject->size_wait_point_read)
			&& (pwrdirobject->pointwrite != pwrdirobject->pointread)) {
		ocenka++;
		if (ocenka >= (pwrdirobject->size_buffer)) //Proverka na konec bufera
			ocenka = 0;
		if (ocenka == pwrdirobject->pointread)
			return 0;
		pwrdirobject->num_write = pwrdirobject->num_write - 1;
	}
	if (pwrdirobject->pointwrite == pwrdirobject->pointread)
		pwrdirobject->num_write = 0;
	//---
	pobject = pobject + pwrite;
	//-----
	*pobject = singlchar;
	//------
	pwrdirobject->num_write = pwrdirobject->num_write + 1;
	pwrite++;
	if (pwrite >= (pwrdirobject->size_buffer)) //Proverka na konec bufera
		pwrite = 0;
	pwrdirobject->pointwrite = pwrite;
	return 1;
}

int read_fastdirstruct_usart(struct point_wrdirobject *pwrdirobject, uint8_t *singlchar) {
	unsigned char *pobject;
	unsigned short pread; //
	unsigned short pwrite; //

	pobject = pwrdirobject->pBuf_UsartOut;
	pwrite = pwrdirobject->pointwrite;
	pread = pwrdirobject->pointread;
	if (pread != pwrite) {
		pobject = pobject + pread;
		*singlchar = *pobject;
		pread++;
		if (pread >= (pwrdirobject->size_buffer))
			pread = 0;
		pwrdirobject->pointread = pread;

		return 1;
	} else
		return 0;
}

int transmit_Usart_buf(USART_TypeDef* USARTx, uint8_t *bufuchar, int lenstring) {
	unsigned char singlchar;
	uint16_t usingle;
	len = lenstring;
	//while((UsartGetStatusCR1(USARTx,IT_TXEIE_USART))!=RESET);

	if ((UsartGetStatusCR1(USARTx, IT_TXEIE_USART)) != RESET) {
		return TRANSMITION_IS_NOT_COMPLETE;
	} else if (posbuf == 0) {
		bufUSART = bufuchar;
		singlchar = *bufuchar;
		posbuf++;
		usingle = ((uint16_t) singlchar) & 0x00ff;
		__disable_irq();
		USART_SendData(USARTx, usingle);
		UsartSetStatusCR1(USARTx, IT_TXEIE_USART);
		__enable_irq();
	} else
		return -1;

	return lenstring;
}

int Read_From(struct point_wrdirobject *pwrdirobject, uint8_t *buffer,
		int numbyte) {
	int i;
	for (i = 0; i < numbyte; i++) {
		if (read_fastdirstruct_usart(pwrdirobject, (buffer + i)) == 0) {
			return i;
		}
	}
	return numbyte;
}

int ReadUSART(struct point_wrdirobject *pwrdirobject, unsigned char *buffer,int numbyte, int MaxTime) {
	uint16_t start_counter;
	int n = 0; // time counter
	int	SymbRead, Numost = 0;
	//	int	npr;
	int endtimesec = 30000; //the endtimesec need too ReadCom parameters
	unsigned char *timebuf;
	timebuf = buffer;
	start_counter = counter_ms; // время входа в функцию считівания

	//MaxTime=MaxTime*10000;

	while ((n < MaxTime) && (n <= endtimesec) && (n >= 0)) {
		if ((SymbRead = Read_From(pwrdirobject, timebuf, numbyte - Numost))<= 0) // если USART не считано ни одного символа
		{
			n = counter_ms - start_counter; // increment time counter
			if (n < 0)
			{
				n = n * (-1);
			}
			vTaskDelay(2);   //  wait ms
		}
		else  // if read succesful
		{
			timebuf = timebuf + SymbRead;
			Numost = Numost + SymbRead;
			start_counter = counter_ms;

			if ((numbyte - Numost) <= 0) {
				n = MaxTime + 5;
			}
		}

	}
	return Numost;
}


//----------------------------------END MY	FUNCTION-------------------------------------------
