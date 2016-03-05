/*
 * processing_modbus_extern.h
 *
 *  Created on: May 26, 2015
 *      Author: Gerasimchuk Alexandr
 */
#ifndef P_MODBUS_EXTERN_H_
#define P_MODBUS_EXTERN_H_

#include "stm32f10x.h"
#include "processing_mem_map.h"
#include "processing_USART.h"




// Максимальное количество запросов modbus по одном порту
#define MAX_NUM_MODBUS_REQ       2


// -----допустимые функции MODBUS_(MASTER/SLAVE)--------------------------------
typedef enum{
	READ_COIL_STATUS=0x01,
	READ_INPUT_STATUS=0x02,
	READ_HOLDING_STATUS=0x03,
	READ_INPUT_REGISTERS=0x04,
	FORCE_SINGLE_COIL=0x05,
	PRESET_SINGLE_REGISTERS=0x06,
	FORSE_MULTIPLE_COILS=0x0f,
	PRESET_MULTIPLE_REGISTERS=0x10
}MODBUS_FUNCTION;

//------------Результаты проверки запроса слейв-----------------
typedef enum{
	REQ_SLAVE_OK=0,
	ILLEGAL_FUNCTION=1,
	ILLEGAL_DATA_ADRESS=2,
	ILLEGAL_DATA_VALUE=3,
	SLAVE_DEVICE_FALIURE=4,
	ACKNOWLEDGE=5,
	SLAVE_DEVICE_BUSY=6,
	NEGATIVE_ACKNOWLEDGE=7,
	MEMORY_PARITY_ERROR=8,
	REQ_SLAVE_ERROR=9
}REZ_REQ_CHEACK_SLAVE;


// -----------Протокол даного порта--------------------
typedef enum {
	PROTOCOL_MODBUS_MASTER,
	PROTOCOL_MODBUS_SLAVE,
} PORT_PROTOCOL;

#pragma pack(push, 1)

// адреса оперативных регистров ПОРОТОВ
typedef struct{
	u16 status_PORT;
} S_modbus_address;

// оперативные регистры ПОРТОВ
typedef struct{
	S_proces_object_modbus status_PORT[NUM_PORT_MODBUS];
} S_modbus_oper_data;


// настройки одного запроса
typedef struct {
	u16	pribor;         // ??
	u8	adress;         // Адрес КП
	u8	function;       // функция опроса
	u16	start_adres;    // арес карты памяти
	u16	number;         // к-во данных к приему
	u16	adress_pmz;     //адрес в карте памяти куда нужно ложить принятую посылку (почему "pmz", это капец.)
	u16	adres_status;   // адрес в области данных статуса запроса
}S_ulist_modbus;


// Структура настроек Modbus Master|Slave
typedef struct {
	u16 state;         // состояние програмного модуля: ENABLE/DISABLE
	S_port_config s_port_config;   //
	PORT_PROTOCOL type;            // (A)    протокол для порта

	//---настройки Master (глобальные для всех запросов на даном порту)

	u16 waitresp;                  // (M)    время ожидания ответа
	//unsigned int nextmessage;    // (S)    таймаут на считывание одного символа в ф-и очистки буфера приема
	u8 number_of_pribor;           // (M)    количество приборов (имееться в виду к-во разных запросов, а не приборов, у нас всегда 1)
	u8 number_no_answer;           // (M)    допустимое к-во неотвеченых запросов для установки флагоа обрыва связ

	//---настройки Slave

	u8 adress_kp;                   // (S)    адрес моего КП

} S_connectmodbus;

#pragma pack(pop)


#endif /* MODBUS_H_ */
