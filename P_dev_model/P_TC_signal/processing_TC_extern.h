/*
* processing_TC_signal_extern.h
 *
 *  Created on: May 24, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#ifndef PROCESSING_TC_SINAL_EXTERN_H_
#define PROCESSING_TC_SINAL_EXTERN_H_

#include "processing_mem_map.h"

#define DEV_1                TC
//#define DEV_1_BIT_STATUS_REG 1

// ����� ���������� �������� ��
#define TOTAL_NUM_TC            8
// ����������� ������-��������� ��
#define NUM_REG_STATUS_TC       1
// ����� ���������� ��������� �� (�������� � ���� ��������� ������ �����, ��� status ��������)
#define NUM_REG_TC              1


#pragma pack(push,1)

//-----------------------------������ ����������� ��������� �������� ��---------------------------------------------------------------
typedef struct{
	u16  status_TC; // ������ �������� ��
	u16  rez_TC;  // ����� � ������ �� ��������� ��������� TC
} S_TC_address;
//--------------------------------------------------------------------------------------------------------------------------------------


//----------------------------- ����������� �������� �������� ��------------------------------------------------------------------------
typedef struct{
	S_proces_object_modbus  status_TC; // ������ �������� ��
	S_proces_object_modbus  rez_TC[TOTAL_NUM_TC];  // ����� � ������ �� ��������� ��������� TC
} S_TC_oper_data;
//--------------------------------------------------------------------------------------------------------------------------------------


//----------------------------- ������������ �������� ��--------------------------------------------------------------------------------
typedef struct{
	FunctionalState state;                           // ��������� ����������� ������: ENABLE/DISABLE
	u16 time_gist;                                    // ����� ������������ ������� ��
    u8 a_r[5];                                       // ����� ������� ��� �������� �����������
}S_TC_user_config;
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(pop)

u16 TC_calc_address_oper_reg(S_TC_address *ps_TC_address, u16 adres_start);
void t_processing_TC(void *pvParameters);


#endif // PROCESSING_TC_SINAL

