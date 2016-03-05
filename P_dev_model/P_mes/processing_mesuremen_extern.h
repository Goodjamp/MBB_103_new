/*
 * processing_mesuremen_extern.h
 *
 *  Created on:
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#ifndef PROCESSING_MESUREMENT_EXTERN_H_
#define PROCESSING_MESUREMENT_EXTERN_H_


// Общее количество аналоговых входов модуля МВВ
#define TOTAL_NUM_MES            1
// Количекство статус-регистров MES
#define NUM_REG_STATUS_MES      1
// Общее количество регистров MES (имееться в виду регистров разных типов, без status регистра)
#define NUM_REG_MES             1

// адреса оперативных регистров MES
typedef struct{
	u16  status_MES;
	u16  present_MES;
} S_mes_address;

// оперативные регистры MES
typedef struct{
	S_proces_object_modbus  status_MES;
	S_proces_object_modbus  present_MES[TOTAL_NUM_MES];
} S_oper_data_MES;
//--------------------------------------------------------------------------------------------------------------------------------

#endif
