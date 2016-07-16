/*
 * processing_mesurement_extern.h
 *
 *  Created on:
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#ifndef PROCESSING_MESUREMENT_EXTERN_H_
#define PROCESSING_MESUREMENT_EXTERN_H_

#include "stm32f10x.h"
#include "processing_mem_map.h"

#define DEV_4                mesurement
// git

// Общее количество аналоговых входов модуля МВВ
#define TOTAL_NUM_MES            1
// Количекство статус-регистров MES
#define NUM_REG_STATUS_MES      1
// Общее количество регистров MES (имееться в виду регистров разных типов, без status регистра)
#define NUM_REG_MES             1

#pragma pack(push,1)

//-----------------------------Адреса оперативных регистров процесса ТС---------------------------------------------------------------
typedef struct{
	u16  status_mesurement; // статус регистры mesurement
	u16  rez_mesurement;  // адрес в памяти МК регистров состояния mesurement
} S_mesurement_address;
//--------------------------------------------------------------------------------------------------------------------------------------


//----------------------------- Оперативные регистры процесса ТС------------------------------------------------------------------------
typedef struct{
	S_proces_object_modbus  status_mesurement; // статус регистры mesurement
	S_proces_object_modbus  rez_mesurement[TOTAL_NUM_MES];  // адрес в памяти МК регистров состояния mesurement
} S_mesurement_oper_data;
//--------------------------------------------------------------------------------------------------------------------------------------


//----------------------------- Конфигурация процесса ТС--------------------------------------------------------------------------------
typedef struct{
	FunctionalState state;                           // состояние програмного модуля: ENABLE/DISABLE
	u16 time_gist;                                   // время гистерезисса прийома ТС
    u16 a_r[5];                                       // место настрок для будущего пользования
}S_mesurement_user_config;
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(pop)

u16 mesurement_calc_address_oper_reg(S_mesurement_address *ps_mesurement_address, u16 adres_start);
void t_processing_mesurement(void *pvParameters);




#endif
