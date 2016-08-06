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


// Количекство статус-регистров процнсса messurement
#define NUM_REG_STATUS_MES      1
// к-во регистров для нормированного результата измерения
#define NUM_REG_REZ_NORM       sizeof(u16)/sizeof(S_proces_object_modbus)


#pragma pack(push,1)

//-----------------------------Адреса оперативных регистров процесса ТС---------------------------------------------------------------
typedef struct{
	u16  status_mesurement; // статус регистры mesurement
	u16  rez_mes_current;    // адрес в памяти МК регистров результата измерения тока
	u16  rez_mes_frequency;  // адрес в памяти МК регистров результата измерения частоты
} S_mesurement_address;
//--------------------------------------------------------------------------------------------------------------------------------------


//----------------------------- Оперативные регистры процесса ТС------------------------------------------------------------------------
typedef struct{
	S_proces_object_modbus  status_mesurement; // статус регистры mesurement
	S_proces_object_modbus  rez_mes_current[NUM_REG_REZ_NORM];  // результат измерений тока
	S_proces_object_modbus  rez_mes_frequency[NUM_REG_REZ_NORM]; // результат измерений частоты х1000
} S_mesurement_oper_data;
//--------------------------------------------------------------------------------------------------------------------------------------


//----------------------------- Конфигурация процесса ТС--------------------------------------------------------------------------------
typedef struct{
	FunctionalState state;                           // состояние програмного модуля: ENABLE/DISABLE
	u16 frequency;                                   // время гистерезисса прийома ТС frequency
    u16 a_r[5];                                       // место настрок для будущего пользования
}S_mesurement_user_config;
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(pop)

u16 mesurement_calc_address_oper_reg(S_mesurement_address *ps_mesurement_address, u16 adres_start);
void t_processing_mesurement(void *pvParameters);




#endif
