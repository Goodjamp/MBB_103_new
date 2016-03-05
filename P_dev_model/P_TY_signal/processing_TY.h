/*
 * processing_TY_signal.h
 *
 *  Created on: February 15, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#ifndef PROCESSING_TY_SINAL
#define  PROCESSING_TY_SINAL


#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "flash_operation.h"
#include "processing_mem_map.h"
#include "processing_modbus.h"
#include "init_system.h"

#include "processing_TY_extern.h"
#include "processing_mem_map_extern.h"
#include "processing_reset_control.h"
// -----����� �����/������ ���������� ��------------------------

// ���� ��������� �����
#define PORT_GRYP_REL      GPIOB
// ���� ����� ���������� ��������
#define PORT_V_REL         GPIOB
// ����� ����� ��
#define PORT_TY_REL1       GPIOA
#define PORT_TY_REL2       GPIOA
#define PORT_TY_REL3       GPIOA
#define PORT_TY_REL4       GPIOB
// ���� ����� �������� ������� ���������� ������������ ����
// (����� �� ��������� ������ PA1)
#define PORT_V_INPUT       GPIOB
//#define PORT_V_INPUT       GPIOA
// ����� ������� �������� ��������� ������� �����
#define PORT_CHK_COIL1      GPIOB
#define PORT_CHK_COIL2      GPIOB
#define PORT_CHK_COIL3      GPIOB
#define PORT_CHK_COIL4      GPIOB

// -----���� �����/������ ���������� ��------------------------
// ��� ��������� �����
#define PIN_GRYP_REL      GPIO_Pin_13
// ��� ����� ���������� ��������
#define PIN_V_REL         GPIO_Pin_14
// ��� ����� ��
#define PIN_TY_REL1       GPIO_Pin_12
#define PIN_TY_REL2       GPIO_Pin_11
#define PIN_TY_REL3       GPIO_Pin_8
#define PIN_TY_REL4       GPIO_Pin_15
// ��� ����� �������� ������� ���������� ������������ ����
// (����� �� ��������� ������ PA1)
#define PIN_V_INPUT       GPIO_Pin_12
//#define PIN_V_INPUT       GPIO_Pin_1

// ���� ������� �������� ��������� ������� �����
#define PIN_CHK_COIL1      GPIO_Pin_7
#define PIN_CHK_COIL2      GPIO_Pin_6
#define PIN_CHK_COIL3      GPIO_Pin_5
#define PIN_CHK_COIL4      GPIO_Pin_4


//-----------------------------------------------------------------------------------------------------------------------------------
//---------------------------------��������� ������� ��������� ������� ���������� ������������ ����----------------------------------
// ������������ �-�� �������� �������� ����-���� � ���������������� ���� ����
#define NUM_MES_V        20

// �������� ������� ����� �������� ���� �������� ������� ���������� ������������ ����, ��, � ������� ��������� ���� ����
#define TIME_MES_V        1
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
//---------------------------------��������� ������������� ������������ ������������ �����-------------------------------------------
// ����� ����������� ������� ��������� ������������� �����, ��
#define TIME_REL_COMUTATION   10

// ����� ����������� ������� ��������� �������������� �������� �����, ��
#define TIME_ELEKTRIK_KEY_COMUTATION   15
//-----------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------
//----��������� ��������� ������������� ������������ ������������� ������ ��� �������� ��������� �������-----------------------------------
//
//                                       read_coil_check|            read_coil_check |
// TIME_BEFORE_CHECK_COIL  | TIME_TRANSIENT_CHECK_COIL  | TIME_TRANSIENT_CHECK_COIL  |
//                          ____________________________
//                         |                            |
//_________________________|                            |____________________________
//
// ����� ����� ����� ������� �������� �������� �����, *10 ���
#define TIME_BEFORE_CHECK_COIL_       100
#define TIME_BEFORE_CHECK_COIL        1*TIME_BEFORE_CHECK_COIL_

//����� ����������� ������� ��������� �������������� ����� ������ �� ��� �������� ��������� ������� �����
// (����� ��������� �������� ������ �� �����), *10 ���
// ��������!!!!! ����� ������ ���� �� ����� ������� ���������� ����� < 500 ��� !!!!
#define TIME_TRANSIENT_CHECK_COIL_    2
#define TIME_TRANSIENT_CHECK_COIL     1*TIME_TRANSIENT_CHECK_COIL_


//������ �������� ������� �� ����� ���������� ������� DP, ������� 1 ��
#define TIME_TOTAL_CHECK_COIL 10

//������ �������� ������� ���������� ������������ ���� �� ����� ���������� ������� DP, ������� 1 ��
#define TIME_TOTAL_CHECK_V 50

// �-�� ������� ������� ��������� ��������
#define NUM_CHECK_FUN   7

//-----------------------------------------------------------------------------------------------------------------------------------


// ����������� ���������� �-�� ����������� ��������
#define MAX_NUM_CYCLING_CHECK    3


// ����� �������� ���� ��� ������ �������� ��
#define TY_FLASH_PAGE 124

// ---------- ������� ���������� �������� --------------------------------------
#define TY_OUT_INVERSE
#ifdef TY_OUT_INVERSE
// ���� ���������� ����� ��������� (��� 0 ������������� ��� �����)
#define TY_CLEAR_OUT(PORT,PIN) GPIO_SetBits(PORT,PIN)
#define TY_SET_OUT(PORT,PIN) GPIO_ResetBits(PORT,PIN)
#else
// ���� ���������� ����� ���������� (��� 1 ������������� ���� �����)
#define TY_SET_OUT(PORT,PIN) GPIO_SetBits(PORT,PIN)
#define TY_CLEAR_OUT(PORT,PIN) GPIO_ResetBits(PORT,PIN)
#endif
#define TY_INVERT_OUT(PORT,PIN) PORT->ODR^=PIN;
// --------- ������� ������ �������----------------
//
#define TY_IN_INVERSE
#ifdef TY_IN_INVERSE
// ���� ������ ��������� ������������ ����� ��������� (��� 0 ������������� ��� �����)
#define TY_READ_IN_1 1
#define TY_READ_IN_0 0
#else
// ���� ���������� ����� ���������� (��� 1 ������������� ���� �����)
#define TY_READ_IN_1 1
#define TY_READ_IN_0 0
#endif

// ������ ������� ������-�������� ������ ��
#define STATUS_STATE(X) X&((ERROR_TY_REL_CONTACT)|\
                           (ERROR_V_INPUT)|\
		                   (ERROR_TY_COIL)|\
		                   (ERROR_GRYP_REL_ON)|\
		                   (ERROR_GRYP_REL_OFF)|\
                           (ERROR_TY_REL_ON)|\
                           (ERROR_TY_REL_OFF))

// ������ ������� ������� �������� � ������ ����������
#define IS_FUN_EXCEPTION(X)  (X==a_check_fun_exception[0])||\
		                     (X==a_check_fun_exception[1])||\
		                     (X==a_check_fun_exception[2])||\
		                     (X==a_check_fun_exception[3])||\
		                     (X==a_check_fun_exception[4])||\
		                     (X==a_check_fun_exception[5])||\
		                     (X==a_check_fun_exception[6])


// ��������� ����������� �������� ������� ���������� ������� ����� ����
#define CHECK_V_OPER_TOK
// ��������� ����������� �������� ��������� ���������
#define CHECK_RELAY_CONTACT
// ��������� ����������� �������� ��������� ������� �����
#define CHECK_RELAY_COIL
//-----------------------------------------------------------------

//���� ������� �������
typedef enum{
	NO_COMAND=0,
	REDY_COMAND=1
}TY_COMAND_FLAG;

//��������� ����� ��������� ���������� ���� ����
typedef enum{
	V_INPUT_SET,
	V_INPUT_RESET
}TY_STATE_V_INPUT;

//��������� ����������� ����� ��������
typedef enum{
	REZ_CHECK_OK=(0),
	ERROR_TY_MODUL=(1<<0),
	ERROR_TY_REL_CONTACT=(1<<1),
	ERROR_V_INPUT=(1<<2),
	ERROR_TY_REL_ON=(1<<3),
	ERROR_GRYP_REL_ON=(1<<4),
	ERROR_GRYP_REL_OFF=(1<<5),
	ERROR_TY_REL_OFF=(1<<6),
	ERROR_TY_COIL=(1<<7)
}TY_REZ_CHECK;


//��������� ��������� ����� ������ ��
typedef enum{
	TY_OFF=0x0000,
	TY_ON=0xFF00,
	TY_NOT_SET=0xFFFF
}TY_STATE;

/*
//��������� ��������� ����� ������ ��
typedef enum{
	TY_SET_ERROR=0x0000,
	TY_SET_OK=0xFF00,
	TY_NOT_DEF=0xFFFF
}TY_REZ_SET;
*/

// ��������� �� ������� ��������
typedef TY_REZ_CHECK (*F_check)(void);


#pragma pack(push, 1) // ������������ 1 ����


// ��������� ������-��������� ��
typedef struct{
	u16  status_TY;
	TY_STATE  present_state_TY[TOTAL_NUM_TY];
	TY_STATE  set_state_TY[TOTAL_NUM_TY];
	u16  operation_TY_statys[TOTAL_NUM_TY];
} S_state_TY;


//��������� ��������� ����������� �������� ��������� ������ ��
typedef struct{
	F_check pf_check;
	TY_REZ_CHECK maska_error;
}S_ty_cycling_check_fun;

typedef struct{
	S_ty_cycling_check_fun pf_fun[MAX_NUM_CYCLING_CHECK];
	u8 num_cycling_check;
}S_ty_cycling_check;

// ��������� ����������� gpio TY
typedef struct{
	GPIO_TypeDef *port_operation;
	u16  pin_operation;
}S_ty_rel_out;

/*
// ��������� DOBLE_POINT TY
typedef struct{
	GPIO_TypeDef *enable_gpio;
	u16  enable_pin;
	GPIO_TypeDef *disable_gpio;
    u16  disable_pin;
}S_ty_double_point;

// ��������� SIMPLE_POINT TY
typedef struct{
	GPIO_TypeDef *enable_gpio;
    u16  enable_pin;
}S_ty_simple_point;
*/

// C�������� ��� �������� � ���������� ������� ��
typedef struct{
	u8 num_TY;             // ����� ������ ��
	TY_COMAND_FLAG f_TY;   // ���� �� ���������� ��
	TY_STATE state_TY;     // ��������� �� ������� ����� ���������� (0xFF00 - enable, 0x0000 - disable)
}S_ty_operation;


// ��������� ������� ������� �� ���������� ��
typedef struct{
	u16 number;
	u16 state;
} S_ty_comand;

// ------------------------��������� ������������ �������� �������� ������� �����------------------------------------------
typedef enum{
	ERROR_XX=0b00,
	ERROR_K3=0b11,
}TY_ERROR_COIL;
//��������� ����������
typedef struct{
	u8 num_check_TY: 6;
	u8 rez_check_TY: 2;
}S_ty_coil_check;

//���������-������ �������� �� ��� �������� �������� �����
typedef struct{
	u8 num_check_TY;                // �-�� ��������
	u8 f_check_end: 4;                 // ���� ��������� ��������
	u8 f_limit_check: 4;                //  ����� �������� (���_�������� ������� �����/�� ����������)
	S_ty_coil_check s_TY_is_check[TOTAL_NUM_TY]; // ����� �������� �����: ����� ������/��������� ��������
}S_ty_coil_check_grup;
// ---------------------------------------------------------------------------------------------------------------------------


#pragma pack(pop)



// ��������� �������
void processing_TY_signal_init(void);
INIT_MBB_Rezult processing_TY_signal_analis_user_config(void);
void processing_TY_signal_init_state(void);
void processing_TY_signal_init_tim(void);
void TIM6_DAC_IRQHandler(void);
void processing_TY_signal_init_gpio(void);
void gpio_enable(GPIO_TypeDef *IN_GPIO);
void processing_TY_signal_set_check_par(void);
u8 processing_TY_signal_operation(void* req,u8 num_peyload_data, u16 addres_data);
REZ_REQ_CHEACK_SLAVE  processing_TY_check_is_coil_status_address_modbus(void* p_check_address);
REZ_REQ_CHEACK_SLAVE  processing_TY_check_force_single_coil_address_modbus(void* p_check_address);
void processing_TY_signal_DP_TY(u8 num_ty, S_state_TY* status_TY);
void processing_TY_signal_SP_TY(u8 num_ty,S_state_TY* status_TY);
REZ_REQ_CHEACK_SLAVE processing_TY_signal_modbus_check(void* req);

TY_STATE_V_INPUT processing_TY_signal_read_v_input(void);

TY_REZ_CHECK processing_TY_signal_check_v(void);
TY_REZ_CHECK processing_TY_signal_check_rel_contact(void);
TY_REZ_CHECK processing_TY_signal_check_ty_rel_on(void);
TY_REZ_CHECK processing_TY_signal_check_gryp_rel_on(void);
TY_REZ_CHECK processing_TY_signal_check_gryp_rel_off(void);
TY_REZ_CHECK processing_TY_signal_check_ty_rel_off(void);
TY_REZ_CHECK processing_TY_signal_check_coil(void);
u8 processing_TY_signal_chek_processing(F_check p_fun_chek, TY_REZ_CHECK paramiter_chek, S_state_TY* status,u8 num_ty);
void processing_TY_signal_write_rel_status(u16 set_state_TY, u16 status, u8 num_ty);
void processing_TY_signal_clear(void);
void processing_TY_signal_disable_rellay(u8 num_ty);
void processing_TY_signal_update_TY_state(S_state_TY *present);
INIT_MBB_Rezult processing_TY_fill_S_TY(u8 *read_data);




#endif
