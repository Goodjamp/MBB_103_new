/*
 * processing_mem_map_extern.h
 *
 *  Created on: December 29, 2014
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#ifndef PROCESSING_MEM_MAP_EXTERN_H_
#define PROCESSING_MEM_MAP_EXTERN_H_

#include "stm32f10x.h"
#include "global_dev.h"
#include "processing_mem_map.h"

// подключаимые модули
#include "processing_includ.h"


//------ОПИСАНИЕ ОБЩЕПОЛЬЗОВАТЕЛЬСКИХ СТРУКТУР-----------------------

#pragma pack(push, 1)

// Структура адресов статус регистров устройства
typedef struct{
	u16 address_device_rcr;              //  device reset_control_register адрес
	u16 address_device_statys;	         // device status register адрес
} S_address_devise_reset_status;

// Структура  статус регистров устройства
typedef struct{
	S_proces_object_modbus device_rcr;              // device reset_control_register
	S_proces_object_modbus device_statys;	         // device status register
} S_devise_reset_status;

// структура адресов оперативных регистров
typedef struct{
	S_address_devise_reset_status s_address_devise_reset_status; // структура регистров статуса устройства
	S_modbus_address s_modbus_address; // структура регистров портов

#ifdef DEV_0
	USER_ADDRESS_FIELD(S,DEV_0) USER_ADDRESS_FIELD(s,DEV_0);
#endif
#ifdef DEV_1
	USER_ADDRESS_FIELD(S,DEV_1) USER_ADDRESS_FIELD(s,DEV_1);
#endif
#ifdef DEV_2
	USER_ADDRESS_FIELD(S,DEV_2) USER_ADDRESS_FIELD(s,DEV_2);
#endif
#ifdef DEV_3
	USER_ADDRESS_FIELD(S,DEV_3) USER_ADDRESS_FIELD(s,DEV_3);
#endif
#ifdef DEV_4
	USER_ADDRESS_FIELD(S,DEV_4) USER_ADDRESS_FIELD(s,DEV_4);
#endif
#ifdef DEV_5
	USER_ADDRESS_FIELD(S,DEV_5) USER_ADDRESS_FIELD(s,DEV_5);
#endif
#ifdef DEV_6
	USER_ADDRESS_FIELD(S,DEV_6) USER_ADDRESS_FIELD(s,DEV_6);
#endif
#ifdef DEV_7
	USER_ADDRESS_FIELD(S,DEV_7) USER_ADDRESS_FIELD(s,DEV_7);
#endif
#ifdef DEV_8
	USER_ADDRESS_FIELD(S,DEV_8) USER_ADDRESS_FIELD(s,DEV_8);
#endif
#ifdef DEV_9
	USER_ADDRESS_FIELD(S,DEV_9) USER_ADDRESS_FIELD(s,DEV_9);
#endif
#ifdef DEV_10
	USER_ADDRESS_FIELD(S,DEV_10) USER_ADDRESS_FIELD(s,DEV_10);
#endif
#ifdef DEV_11
	USER_ADDRESS_FIELD(S,DEV_11) USER_ADDRESS_FIELD(s,DEV_11);
#endif
#ifdef DEV_12
	USER_ADDRESS_FIELD(S,DEV_12) USER_ADDRESS_FIELD(s,DEV_12);
#endif
#ifdef DEV_13
	USER_ADDRESS_FIELD(S,DEV_13) USER_ADDRESS_FIELD(s,DEV_13);
#endif
#ifdef DEV_14
	USER_ADDRESS_FIELD(S,DEV_14) USER_ADDRESS_FIELD(s,DEV_14);
#endif
#ifdef DEV_15
	USER_ADDRESS_FIELD(S,DEV_15) USER_ADDRESS_FIELD(s,DEV_15);
#endif
}S_address_oper_data;


// структура оперативных регистров
// ИСПОЛЬЗУЕТЬСЯ ДЛЯ РАСЧЕТА РАЗМЕРА ОБЛАСТИ ОПЕРАТИВНЫХ РЕГИСТРОВ, ПЕРЕМЕННАЯ ДАНОГО ТИПА НЕ СОЗДАЕТЬСЯ !!!
typedef struct{
	S_devise_reset_status s_devise_reset_status;

	S_modbus_oper_data s_modbus_oper_data; // структура регистров портов

#ifdef DEV_0
	USER_OPER_DATA_FIELD(S,DEV_0) USER_OPER_DATA_FIELD(s,DEV_0);
#endif
#ifdef DEV_1
	USER_OPER_DATA_FIELD(S,DEV_1) USER_OPER_DATA_FIELD(s,DEV_1);
#endif
#ifdef DEV_2
	USER_OPER_DATA_FIELD(S,DEV_2) USER_OPER_DATA_FIELD(s,DEV_2);
#endif
#ifdef DEV_3
	USER_OPER_DATA_FIELD(S,DEV_3) USER_OPER_DATA_FIELD(s,DEV_3);
#endif
#ifdef DEV_4
	USER_OPER_DATA_FIELD(S,DEV_4) USER_OPER_DATA_FIELD(s,DEV_4);
#endif
#ifdef DEV_5
	USER_OPER_DATA_FIELD(S,DEV_5) USER_OPER_DATA_FIELD(s,DEV_5);
#endif
#ifdef DEV_6
	USER_OPER_DATA_FIELD(S,DEV_6) USER_OPER_DATA_FIELD(s,DEV_6);
#endif
#ifdef DEV_7
	USER_OPER_DATA_FIELD(S,DEV_7) USER_OPER_DATA_FIELD(s,DEV_7);
#endif
#ifdef DEV_8
	USER_OPER_DATA_FIELD(S,DEV_8) USER_OPER_DATA_FIELD(s,DEV_8);
#endif
#ifdef DEV_9
	USER_OPER_DATA_FIELD(S,DEV_9) USER_OPER_DATA_FIELD(s,DEV_9);
#endif
#ifdef DEV_10
	USER_OPER_DATA_FIELD(S,DEV_10) USER_OPER_DATA_FIELD(s,DEV_10);
#endif
#ifdef DEV_11
	USER_OPER_DATA_FIELD(S,DEV_11) USER_OPER_DATA_FIELD(s,DEV_11);
#endif
#ifdef DEV_12
	USER_OPER_DATA_FIELD(S,DEV_12) USER_OPER_DATA_FIELD(s,DEV_12);
#endif
#ifdef DEV_13
	USER_OPER_DATA_FIELD(S,DEV_13) USER_OPER_DATA_FIELD(s,DEV_13);
#endif
#ifdef DEV_14
	USER_OPER_DATA_FIELD(S,DEV_14) USER_OPER_DATA_FIELD(s,DEV_14);
#endif
#ifdef DEV_15
	USER_OPER_DATA_FIELD(S,DEV_15) USER_OPER_DATA_FIELD(s,DEV_15);
#endif
}S_oper_data;

#pragma pack(pop)

// размер карты памяти (лперативные регистры и регистры данных)
#define SIZE_OPER_DATA            sizeof(S_oper_data)
// размер области оперативных регистров в единицах размера типового proces_object ( для Герасимчук: все поля структур S_oper_data
// имеют тип S_proces_object_modbus, поэтому и размер S_oper_data кратен размеру S_proces_object_modbus !!! )
#define NUM_REG_OPER_DATA         sizeof(S_oper_data)/SIZE_PROCES_OBJECT

#endif
