/*
 * processing_routing_data_extern.h
 *
 *  Created on: May 24, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#ifndef PROCESSING_ROUTING_DATA_EXTERN_H_
#define PROCESSING_ROUTING_DATA_EXTERN_H_


#include "processing_mem_map.h"

#define DEV_3                routing_data
#define DEV_3_BIT_STATUS_REG 3

#pragma pack(push, 1)

//-----------------------------������ ����������� ��������� �������� routing_data-------------------------------------------------------
typedef struct{
	u16  status_routing_data;
} S_routing_data_address;
//--------------------------------------------------------------------------------------------------------------------------------------

//----------------------------- ����������� �������� �������� routing_data--------------------------------------------------------------
typedef struct{
	S_proces_object_modbus  status_routing_data;
} S_routing_data_oper_data;
//--------------------------------------------------------------------------------------------------------------------------------------


//----------------------------- ������������ �������� routing_data----------------------------------------------------------------------
// ��������� ������������� ������� ���������� (������ ��������)
typedef struct{
	u16 address_from;
	u16 address_to;
}S_routing_data;


// ������� �������������
typedef struct{
	FunctionalState state;                      // ��������� ����������� ������: ENABLE/DISABLE
	u8              byte_alignt;                // �������������� ���� ��� ������������ ������
	u16             num_rout;                   // ���������� ��������� � ������������
	S_routing_data  s_data_rout[NUM_REG_DATA ]; // ������ ������� ������������� ����� ���������� ��������� �����/������, ��������� ������������ ����� ��������� ������ � ��� �������
}S_routing_data_user_config;
//--------------------------------------------------------------------------------------------------------------------------------------

#pragma pack(pop)

u16 routing_data_calc_address_oper_reg(S_routing_data_address *ps_routing_data_address, u16 adres_start);
void t_processing_routing_data(void *pvParameters);


#endif
