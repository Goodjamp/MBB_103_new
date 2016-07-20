/*
 * processing_TC_signal.h
 *
 *  Created on: January 28, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#ifndef PROCESSING_TC_SINAL
#define  PROCESSING_TC_SINAL

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "processing_mem_map.h"
#include "processing_modbus.h"
#include "init_system.h"
#include "processing_TC_extern.h"

#include "processing_reset_control.h"
// --------- define ПРОВЕРКИ--------------------
//#define TC_SOFT_CHECK

#define PIN_SOFT_CHECK_B2  GPIO_Pin_2
#define PORT_SOFT_CHECK_B2 GPIOB

#define PIN_SOFT_CHECK_B12  GPIO_Pin_12
#define PORT_SOFT_CHECK_B12 GPIOB
//----------------------------------------------


#define LD_PORT          GPIOB
#define LD_PIN           GPIO_Pin_0

#define TOK_PORT         GPIOB
#define TOK_PIN          GPIO_Pin_1
#define TOK_ERROR_STATE  0

#define V_PORT           GPIOA
#define V_PIN            GPIO_Pin_4
#define V_ERROR_STATE    1

#define LD_SET     GPIO_SetBits(LD_PORT,LD_PIN);
#define LD_CLEAR   GPIO_ResetBits(LD_PORT,LD_PIN);

// Период обновления значений ТС, мc
#define PERIOD_GET_TC   2

// Квант счета таймера, в отсчетах
#define TIM_PRESCALER   1000

#define CHECK_SERIAL_BYTE   0b01010101

#define TYPE_SIGNAL_TC       1

#define MAX_CYCLE_FLASH_INFLUANCE  3

// считывание бита из байта; x - байт, y - номер бита [0 - 7]
#define READ_BIT_TC(x,y) ((x>>y) & 0b1)

// Состояние дискретного входа
typedef enum{
	IS_PROCESSING,
	IS_NOT_PROCESSING,
	TS_SET,
	TS_CLEAR
}ts_state;

// Возможные исключения статус - регистра
typedef enum{
	TOK_ERROR=0,
	V_ERROR,
	CHECK_ERROR
}ts_type_processing;



#pragma pack(push, 1)


// структура результатов считывания данных с сериалайзера
typedef struct{
	u16 data_cheack;
	u16 data_TC;
}S_reaz_read_TC;

// Структура обработки единого ТС
typedef struct{
	ts_state tc_is_procssing;
	u16 tc_counter;
	u16 tc_state;
} S_one_TC_processing;


// структура обработки ТС
typedef struct{
	S_one_TC_processing as_one_TC_processing[TOTAL_NUM_TC];
	u8 num_TC;
} S_total_TC_processing;


#pragma pack(pop)

void processing_TC_init(void);
void processing_TC_init_struct(void);
void processing_TC_init_gpio(void);
void processing_TC_init_tim3(void);
void processing_TC_init_spi1(void);
void SPI1_IRQHandler(void);
void processing_TC_signal_mem_update(void);
REZ_REQ_CHEACK_SLAVE processing_TC_signal_check(void* data_chek);
REZ_REQ_CHEACK_SLAVE processing_TC_check_is_input_status_modbus(void* p_check_address);
INIT_MBB_Rezult processing_TC_fill_S_TS(u8 *read_data);

//// 	ТОЛКЬО ДЛЯ ПРОВЕРКИ!!!!
#ifdef TC_SOFT_CHECK
void check(void);
#endif


#endif // PROCESSING_TC_SINAL


/*
 *
 *
 *

//--------------функция processing_TC_signal_read_TC-----------------
//функция processing_TC_signal_read_TC - запись и считывание сигналов ТС и проверочного байта
void processing_TC_signal_read_TC(S_reaz_read_TC *ps_reaz_read_TC){
	u32 k1=0;
	// загружаю данные в сдвиговый регистр
	LD_CLEAR;
	while(k1<10){
		k1++;
	}
	LD_SET;
	while(k1<10){
		k1++;
	}
	// передаю проверочный байт
	SPI_I2S_SendData(SPI1,(u8)ps_reaz_read_TC->data_cheack);
	// жду приема данных ТС
	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)){}
	ps_reaz_read_TC->data_TC=SPI_I2S_ReceiveData(SPI1);
	// передаю пустую посылку чтобы принять проверочный байт
	SPI_I2S_SendData(SPI1,0x00);
	// жду приема проверочного байта
	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)){}
	ps_reaz_read_TC->data_cheack=SPI_I2S_ReceiveData(SPI1);
}

*/
