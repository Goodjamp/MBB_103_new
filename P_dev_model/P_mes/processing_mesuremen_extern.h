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

// ����� ���������� ���������� ������ ������ ���
#define TOTAL_NUM_MES            1
// ����������� ������-��������� MES
#define NUM_REG_STATUS_MES      1
// ����� ���������� ��������� MES (�������� � ���� ��������� ������ �����, ��� status ��������)
#define NUM_REG_MES             1

#pragma pack(push,1)

//-----------------------------������ ����������� ��������� �������� ��---------------------------------------------------------------
typedef struct{
	u16  status_mesurement; // ������ �������� mesurement
	u16  rez_mesurement;  // ����� � ������ �� ��������� ��������� mesurement
} S_mesurement_address;
//--------------------------------------------------------------------------------------------------------------------------------------


//----------------------------- ����������� �������� �������� ��------------------------------------------------------------------------
typedef struct{
	S_proces_object_modbus  status_mesurement; // ������ �������� mesurement
	S_proces_object_modbus  rez_mesurement[TOTAL_NUM_MES];  // ����� � ������ �� ��������� ��������� mesurement
} S_mesurement_oper_data;
//--------------------------------------------------------------------------------------------------------------------------------------


//----------------------------- ������������ �������� ��--------------------------------------------------------------------------------
typedef struct{
	FunctionalState state;                           // ��������� ����������� ������: ENABLE/DISABLE
	u16 time_gist;                                   // ����� ������������ ������� ��
    u16 a_r[5];                                       // ����� ������� ��� �������� �����������
}S_mesurement_user_config;
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(pop)

u16 mesurement_calc_address_oper_reg(S_mesurement_address *ps_mesurement_address, u16 adres_start);
void t_processing_mesurement(void *pvParameters);




#endif
