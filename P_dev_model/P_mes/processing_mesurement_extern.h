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


// ����������� ������-��������� �������� messurement
#define NUM_REG_STATUS_MES      1
// �-�� ��������� ��� �������������� ���������� ��������� ���� U16
#define NUM_REG_REZ_U16       sizeof(u16)/sizeof(S_proces_object_modbus)
// �-�� ��������� ��� �������������� ���������� ��������� ���� DOUBLE
#define NUM_REG_REZ_DOUBLE       sizeof(double)/sizeof(S_proces_object_modbus)


#pragma pack(push,1)

//-----------------------------������ ����������� ��������� �������� ��---------------------------------------------------------------
typedef struct{
	u16  status_mesurement; // ������ �������� mesurement
	u16  rez_mes_current;    // ����� � ����� ������ ���������� ��������� ����
	u16  rez_mes_frequency;  // ����� � ����� ������ ���������� ��������� �������
	u16  mes_current_double; // ����� � ����� ������ ���� ����������� ��������� ����
} S_mesurement_address;
//--------------------------------------------------------------------------------------------------------------------------------------


//----------------------------- ����������� �������� �������� ��------------------------------------------------------------------------
typedef struct{
	S_proces_object_modbus  status_mesurement; // ������ �������� mesurement
	S_proces_object_modbus  rez_mes_current[NUM_REG_REZ_U16];  // ��������� ��������� ����
	S_proces_object_modbus  rez_mes_frequency[NUM_REG_REZ_U16]; // ��������� ��������� ������� �1000
	S_proces_object_modbus  rez_mes_current_kod[NUM_REG_REZ_DOUBLE]; // ��������� ��������� ������� �1000
} S_mesurement_oper_data;
//--------------------------------------------------------------------------------------------------------------------------------------


//----------------------------- ������������ �������� ��--------------------------------------------------------------------------------
typedef struct{
	u16 state;                           // ��������� ����������� ������: ENABLE/DISABLE
	u16 frequency;                                   // ������� ����������� ����
    u16 a_r[3];                                      // ����� ������� ��� �������� �����������
}S_mesurement_user_config;
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(pop)

u16 mesurement_calc_address_oper_reg(S_mesurement_address *ps_mesurement_address, u16 adres_start);
void t_processing_mesurement(void *pvParameters);




#endif
