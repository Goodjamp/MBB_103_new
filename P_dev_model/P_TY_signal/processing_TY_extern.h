/*
 * processing_TY_signal_extern.h
 *
 *  Created on: February 15, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#ifndef PROCESSING_TY_SINAL_EXTERN_H_
#define  PROCESSING_TY_SINAL_EXTERN_H_

#define DEV_2                TY
//#define DEV_2_BIT_STATUS_REG 2

//------ОПИСАНИЕ ОБЩЕПОЛЬЗОВАТЕЛЬСКИХ СТРУКТУР-----------------------

// Общее количество выходных контактов ТY
#define TOTAL_NUM_TY             4
// Количекство статус-регистров ТУ
#define NUM_REG_STATUS_TY        1
// Общее количество регистров ТY (имееться в виду регистров разных типов, без status регистра)
#define NUM_REG_TY               4

//конфигурация типа ТУ
typedef enum{
	NO_OUT=0,
	SINGLE_POSITION,
	DOUBLE_POSITION,
	PARALLEL_CHANNEL
}TY_MODE;



#pragma pack(push, 1)

//-----------------------------Адреса оперативных регистров процесса ТY---------------------------------------------------------------
typedef struct{
	u16  status_TY;        // общий статус модуля ТУ
	u16  present_state_TY; // поточное состояние региcтра ТУ
	u16  set_state_TY;     // состояние регистра ТУ которое необходимо установить
	u16  operation_TY_statys;     // результат цикличесткой проверки состояния готовности регистра ТУ (не совсем понятно, дублтрует )
} S_TY_address;
//-------------------------------------------------------------------------------------------------------------------------------------


//----------------------------- Оперативные регистры процесса ТY------------------------------------------------------------------------
typedef struct{
	S_proces_object_modbus  status_TY;
	S_proces_object_modbus  present_state_TY[TOTAL_NUM_TY];   // поточное состояние региcтра ТУ
	S_proces_object_modbus  set_state_TY[TOTAL_NUM_TY];       // состояние регистра ТУ которое необходимо установить
	S_proces_object_modbus  operation_TY_statys[TOTAL_NUM_TY];
} S_TY_oper_data;
//--------------------------------------------------------------------------------------------------------------------------------------


//----------------------------- Конфигурация процесса ТY--------------------------------------------------------------------------------
typedef struct{
	TY_MODE  mode_TY;                                   // режим работы выхода
	u8 f_paralel_out;                                   // флаг наличия паралельного выхода
	u16 num_paralel;                                     // номер паралельного выхода (если флаг установлен)
}S_TY_out_config;


// Общая структура настроек пользователя модуля ТУ
typedef struct{
	FunctionalState state;                             // состояние програмного модуля: ENABLE/DISABLE
	u16 puls_with;                                     // длина импульса для выходов типа DP, мс
	FunctionalState f_use_V_check;                     // флаг использования проверки наличия оперативного напряжения
	S_TY_out_config s_TY_out_config[TOTAL_NUM_TY];     // структура настроек выходов ТУ
}S_TY_user_config;
//------------------------------------------------------------------------------------------------------------------------------

#pragma pack(pop)

u16 TY_calc_address_oper_reg(S_TY_address *ps_TY_address, u16 adres_start);
void t_processing_TY(void *pvParameters);

#endif
