/*
 * processing_mesuremen_extern.h
 *
 *  Created on:
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#ifndef PROCESSING_MESUREMENT_EXTERN_H_
#define PROCESSING_MESUREMENT_EXTERN_H_


// ����� ���������� ���������� ������ ������ ���
#define TOTAL_NUM_MES            1
// ����������� ������-��������� MES
#define NUM_REG_STATUS_MES      1
// ����� ���������� ��������� MES (�������� � ���� ��������� ������ �����, ��� status ��������)
#define NUM_REG_MES             1

// ������ ����������� ��������� MES
typedef struct{
	u16  status_MES;
	u16  present_MES;
} S_mes_address;

// ����������� �������� MES
typedef struct{
	S_proces_object_modbus  status_MES;
	S_proces_object_modbus  present_MES[TOTAL_NUM_MES];
} S_oper_data_MES;
//--------------------------------------------------------------------------------------------------------------------------------

#endif
