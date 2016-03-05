/*
 * processing_USART.c
 *
 *  Created on: Feb 6, 2012
 *      Author: Gerasimchuk
 */

#include "processing_USART.h"


//----выделяем память бля буфферов и настроек USART---------------------------------------------------
unsigned char Buf_USART_Transmit[NUMBER_USART][SIZE_OUT_BUFER];
unsigned char Buf_USART_Read[NUMBER_USART][SIZE_READ_BUFER];
// -------структуры обслуживания стека приема/передачи---------------------------------
struct point_wrdirobject WrUSART_CODE[NUMBER_USART]; // Масив структур обслуживания стека передачи. Индекс соответствует номеру юзарта
struct point_wrdirobject RdUSART_CODE[NUMBER_USART]; //Масив структур обслуживания стека приема. Индекс соответствует номеру юзарта

//------------указатели на ЮЗАРТы указывают на начальные структуры из масива структур ЮЗАРТов
struct point_wrdirobject *WrUSART=&WrUSART_CODE[0];;
struct point_wrdirobject *RdUSART=&RdUSART_CODE[0];;

//------------масив структру в которой записаны указатели на буферы приема/передачи ---------------
S_Task_parameters task_parameters[NUMBER_MY_PROCES];
S_Task_parameters *ptask_parameters=&task_parameters[0];;
//--------------------------------------------------------------------

//------------масив указателей на USART которые пользователь разрешил использовать, и под которые выделенная память для стеков прийома/передачи-----------------------
USART_TypeDef *present_usart[NUMBER_USART]={PRESENT_USART};

volatile u32 counter_ms=0;
static u8 f_tim_init=0;

//--------------функция init_USART()-----------
// функция init_USART() - выполняет програмную настройку модуля USART - связывает программно конкретный USART
// из его буфером приема/передачи
void init_USART(USART_TypeDef *USART_in, u8 num_usart){

	// заполняю структуры БУФЕРОВ ЮЗАРТа (ТОЛЬКО БУФЕРОВ заданными параметрами)
	(WrUSART+num_usart)->pBuf_UsartOut=Buf_USART_Transmit[num_usart];
	(WrUSART+num_usart)->size_wait_point_read=SIZE_WAIT_POINT_OUT_READ;
	(WrUSART+num_usart)->size_buffer=REAL_SIZE_OUT_BUFER;
	(WrUSART+num_usart)->pointwrite=0;
	(WrUSART+num_usart)->pointread=0;
	(WrUSART+num_usart)->num_write=0;
	(WrUSART+num_usart)->num_read=0;


	(RdUSART+num_usart)->pBuf_UsartOut=Buf_USART_Read[num_usart];
	(RdUSART+num_usart)->size_wait_point_read=SIZE_WAIT_POINT_READ_READ;
	(RdUSART+num_usart)->size_buffer=REAL_SIZE_READ_BUFER;
	(RdUSART+num_usart)->pointwrite=0;
	(RdUSART+num_usart)->pointread=0;
	(RdUSART+num_usart)->num_write=0;
	(RdUSART+num_usart)->num_read=0;

   //связывает задачу с конкретным ЮЗАРТОМ (номером ЮЗАРТа, буфером узарта)
	(ptask_parameters+num_usart)->USARTx=USART_in;
	(ptask_parameters+num_usart)->WrUSART=WrUSART+num_usart;
	(ptask_parameters+num_usart)->RdUSART=RdUSART+num_usart;

 // настройка таймера для USART
	if(!f_tim_init){
		init_timer();
	}

};


void init_timer(){

	TIM_TimeBaseInitTypeDef user_timer_1;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	/*Делитель 72 (с учетом значения регистра захвата/сравнения=999 получаем период прерывания = 1 мс) */

	user_timer_1.TIM_Period = 999;			 //чтобы получить период генерации прерываний 1000 мкс, количество отсчетов	должно быть 999 ???
	user_timer_1.TIM_Prescaler = 72 - 1;	 //Делитель считается как TIM_Prescaler+1  !!!
	user_timer_1.TIM_ClockDivision = TIM_CKD_DIV1;	//на вход таймера частота шины APB1 не делится
	user_timer_1.TIM_CounterMode = TIM_CounterMode_Up;
	user_timer_1.TIM_RepetitionCounter = 0x0000;

	TIM_TimeBaseInit(TIM4, &user_timer_1);

	/* Прерывание по переполнению счётчика TIM4*/
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	NVIC_EnableIRQ(TIM4_IRQn);
}

//------------------------------------MY HANDLER INTERRAPTS---------------------------------------------

void USART1_IRQHandler(void)
{
	unsigned char singlchar;
	uint16_t	usingle;
	//--Proveryaem TXE

	if(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == SET)
	{
		if((read_fastdirstruct_usart(WrUSART,&singlchar))!=0)
		{
			if((singlchar<0x30)||(singlchar>0x39))
			{
			}
			usingle=((uint16_t)singlchar)&0x00ff;
			USART_SendData(USART1,usingle);
		}
		else
		{
			UsartClearStatusCR1(USART1,IT_TXEIE_USART);			//Nado zakrit port
		}
	}

	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == SET)
	{

		usingle=USART_ReceiveData(USART1);
		singlchar=(uint8_t)(usingle&0xff);
		write_fastdirstruct_usart(RdUSART,singlchar);
	}

}

void USART2_IRQHandler(void)
{
	uint8_t singlchar;
	uint16_t	usingle;
	if(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == SET)
	{
		if((read_fastdirstruct_usart(WrUSART+1,&singlchar))!=0)
		{
			usingle=((uint16_t)singlchar)&0x00ff;
			USART_SendData(USART2,usingle);
		}
		else
			UsartClearStatusCR1(USART2,IT_TXEIE_USART);			//Nado zakrit port
	}

	if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE) == SET)
	{
		usingle=USART_ReceiveData(USART2);
		singlchar=(uint8_t)(usingle&0xff);
		write_fastdirstruct_usart(RdUSART+1,singlchar);
	}
}

void USART3_IRQHandler(void)
{
	unsigned char singlchar;
	uint16_t	usingle;
	//--Proveryaem TXE

	if(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == SET)
	{
		if((read_fastdirstruct_usart(WrUSART+2,&singlchar))!=0)
		{
			usingle=((uint16_t)singlchar)&0x00ff;
			USART_SendData(USART3,usingle);
		}
		else
			UsartClearStatusCR1(USART3,IT_TXEIE_USART);			//Nado zakrit port
	}

	if(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) == SET)
	{
		usingle=USART_ReceiveData(USART3);
		singlchar=(uint8_t)(usingle&0xff);
		write_fastdirstruct_usart(RdUSART+2,singlchar);

	}
}
void TIM4_IRQHandler(void)
{
	//uint8_t	usinglechar;
	/*Очищаем бит обрабатываемого прерывания*/
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	counter_ms++;  //счетчик миллисекунд через таймер TIM7
}




//
// Configure UART
//
USART_errort  ConfigureUSART(const S_port_config *ps_port_unit_config, USART_TypeDef *USART_init){
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_TypeDef  *GPIO_USART_port_TX;
	GPIO_TypeDef  *GPIO_USART_port_RX;
	uint16_t GPIO_USART_pin_TX;
	uint16_t GPIO_USART_pin_RX;
	u8 counter;

	// ищу запрашиваимый USART в списке доступных
	for(counter=0;counter<NUMBER_USART;counter++){
		if(USART_init==present_usart[counter]){break;}
	}
	if(counter>=NUMBER_USART){return USART_ERROR_CONFIG;} // если запрашиваимого USART нету в списке - выход с ошибкой
// конфигурю стек приема/передачи выбранного USART
	init_USART(USART_init, counter);

	//Включаю тактирование альтернативных ф-й
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);


	// выбираем и настраеваем нужные ножки

	//USART1: Tx (PA9)  Rx (PA10)
	//USART2: Tx (PA2)  Rx (PA3)
	//USART3: Tx (PB10) Rx (PB11)

	if (USART_init==USART1){
		GPIO_USART_port_TX=USART1_PORT_TX;
		GPIO_USART_port_RX=USART1_PORT_RX;
		GPIO_USART_pin_TX=USART1_PIN_TX;
		GPIO_USART_pin_RX=USART1_PIN_RX;
		// Включаю тактирование USART1
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
		// Включаю тактирование порта А
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		// включаю прерывание
		NVIC_EnableIRQ(USART1_IRQn);
	}
	else if(USART_init==USART2){
		GPIO_USART_port_TX=USART2_PORT_TX;
		GPIO_USART_port_RX=USART2_PORT_RX;
		GPIO_USART_pin_TX=USART2_PIN_TX;
		GPIO_USART_pin_RX=USART2_PIN_RX;
		// Включаю тактирование USART2
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
		// Включаю тактирование порта А
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		// включаю прерывание
		NVIC_EnableIRQ(USART2_IRQn);
	}
	else if(USART_init==USART3){
		GPIO_USART_port_TX=USART3_PORT_TX;
		GPIO_USART_port_RX=USART3_PORT_RX;
		GPIO_USART_pin_TX=USART3_PIN_TX;
		GPIO_USART_pin_RX=USART3_PIN_RX;
		// Включаю тактирование USART3
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
		// Включаю тактирование порта В
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		// включаю прерывание
		NVIC_EnableIRQ(USART3_IRQn);
	}
/*	else if(USART_init==USART4){
			GPIO_USART_port_TX=USART4_PORT_TX;
			GPIO_USART_port_RX=USART4_PORT_RX;
			GPIO_USART_pin_TX=USART4_PIN_TX;
			GPIO_USART_pin_RX=USART4_PIN_RX;
			// Включаю тактирование USART4
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART4,ENABLE);
			// Включаю тактирование порта В
			RCC_APB2PeriphClockCmd(RCC_APB4Periph_GPIOB,ENABLE);
			// включаю прерывание
			NVIC_EnableIRQ(USART4_IRQn);
		}
	else if(USART_init==USART5){
			GPIO_USART_port_TX=USART5_PORT_TX;
			GPIO_USART_port_RX=USART5_PORT_RX;
			GPIO_USART_pin_TX=USART5_PIN_TX;
			GPIO_USART_pin_RX=USART5_PIN_RX;
			// Включаю тактирование USART5
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART5,ENABLE);
			// Включаю тактирование порта В
			RCC_APB2PeriphClockCmd(RCC_APB1Periph_GPIOB,ENABLE);
			// включаю прерывание
			NVIC_EnableIRQ(USART5_IRQn);
		}
	else if(USART_init==USART6){
			GPIO_USART_port_TX=USART6_PORT_TX;
			GPIO_USART_port_RX=USART6_PORT_RX;
			GPIO_USART_pin_TX=USART6_PIN_TX;
			GPIO_USART_pin_RX=USART6_PIN_RX;
			// Включаю тактирование USART6
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART6,ENABLE);
			// Включаю тактирование порта В
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
			// включаю прерывание
			NVIC_EnableIRQ(USART6_IRQn);
		}
*/
	/* Configure USART Tx  as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_USART_pin_TX;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIO_USART_port_TX, &GPIO_InitStructure);

	/* Configure USART Rx  as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_USART_pin_RX;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIO_USART_port_RX, &GPIO_InitStructure);

	//------------------------------------------------------------------------


	//----------------------------- настройка USART----------------------------
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable;

	USART_InitStructure.USART_BaudRate = ps_port_unit_config->baudrate;
	// размер слова
	switch(ps_port_unit_config->amountbyte){
	case 8:USART_InitStructure.USART_WordLength = USART_WordLength_8b;break;
	case 9:USART_InitStructure.USART_WordLength = USART_WordLength_9b;break;
	default:break;
	}
	// к-во стоп битов
	switch(ps_port_unit_config->stopbits){
	case 1:USART_InitStructure.USART_StopBits = USART_StopBits_1;break;
	case 2:USART_InitStructure.USART_StopBits = USART_StopBits_2;break;
	default:break;
	}
	// тип паритета
	switch(ps_port_unit_config->parity){
	case 0:USART_InitStructure.USART_Parity = USART_Parity_No;break;
	case 1:USART_InitStructure.USART_Parity = USART_Parity_Odd;
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;break;
	case 2:USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;break;
	default:break;
	}

	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode =USART_Mode_Rx|USART_Mode_Tx;

	USART_Init(USART_init, &USART_InitStructure);
	USART_ClockInit(USART_init,&USART_ClockInitStruct);

	USART_ITConfig(USART_init, USART_IT_TXE, ENABLE);

	USART_ITConfig(USART_init, USART_IT_RXNE, ENABLE);

	/* Enable USART */
	USART_Cmd(USART_init, ENABLE);

	return USART_OK;
}

void Clrinbuf(S_Task_parameters *ptaskparameters,int MaxTime)
{
	unsigned int iRead=1;
	 uint8_t achar;
	while (iRead!=0)
	{
		iRead=ReadUSART(ptaskparameters->RdUSART,&achar,1,MaxTime);
	}
}

void Clrinbuf_without_time(S_Task_parameters *ptaskparameters)
{
	uint8_t achar;
	while ((Read_From(ptaskparameters->RdUSART,&achar,1))!=0)
	{

	}
}


