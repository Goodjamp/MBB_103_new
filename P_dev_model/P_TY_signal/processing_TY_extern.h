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

//------�������� �������������������� ��������-----------------------

// ����� ���������� �������� ��������� �Y
#define TOTAL_NUM_TY             4
// ����������� ������-��������� ��
#define NUM_REG_STATUS_TY        1
// ����� ���������� ��������� �Y (�������� � ���� ��������� ������ �����, ��� status ��������)
#define NUM_REG_TY               4

//������������ ���� ��
typedef enum{
	NO_OUT=0,
	SINGLE_POSITION,
	DOUBLE_POSITION,
	PARALLEL_CHANNEL
}TY_MODE;



#pragma pack(push, 1)

//-----------------------------������ ����������� ��������� �������� �Y---------------------------------------------------------------
typedef struct{
	u16  status_TY;        // ����� ������ ������ ��
	u16  present_state_TY; // �������� ��������� ����c��� ��
	u16  set_state_TY;     // ��������� �������� �� ������� ���������� ����������
	u16  operation_TY_statys;     // ��������� ������������ �������� ��������� ���������� �������� �� (�� ������ �������, ��������� )
} S_TY_address;
//-------------------------------------------------------------------------------------------------------------------------------------


//----------------------------- ����������� �������� �������� �Y------------------------------------------------------------------------
typedef struct{
	S_proces_object_modbus  status_TY;
	S_proces_object_modbus  present_state_TY[TOTAL_NUM_TY];   // �������� ��������� ����c��� ��
	S_proces_object_modbus  set_state_TY[TOTAL_NUM_TY];       // ��������� �������� �� ������� ���������� ����������
	S_proces_object_modbus  operation_TY_statys[TOTAL_NUM_TY];
} S_TY_oper_data;
//--------------------------------------------------------------------------------------------------------------------------------------


//----------------------------- ������������ �������� �Y--------------------------------------------------------------------------------
typedef struct{
	TY_MODE  mode_TY;                                   // ����� ������ ������
	u8 f_paralel_out;                                   // ���� ������� ������������ ������
	u16 num_paralel;                                     // ����� ������������ ������ (���� ���� ����������)
}S_TY_out_config;


// ����� ��������� �������� ������������ ������ ��
typedef struct{
	FunctionalState state;                             // ��������� ����������� ������: ENABLE/DISABLE
	u16 puls_with;                                     // ����� �������� ��� ������� ���� DP, ��
	FunctionalState f_use_V_check;                     // ���� ������������� �������� ������� ������������ ����������
	S_TY_out_config s_TY_out_config[TOTAL_NUM_TY];     // ��������� �������� ������� ��
}S_TY_user_config;
//------------------------------------------------------------------------------------------------------------------------------

#pragma pack(pop)

u16 TY_calc_address_oper_reg(S_TY_address *ps_TY_address, u16 adres_start);
void t_processing_TY(void *pvParameters);

#endif
