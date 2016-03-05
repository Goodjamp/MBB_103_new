 /*
 * modbus.h
 *
 *  Created on: Apr 10, 2015
 *      Author: Gerasimchuk Alexandr
 */
#ifndef MODBUS_H_
#define MODBUS_H_

#include "stm32f10x_tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "processing_USART.h"
#include "USART_fifo_operation.h"
#include "funct.h"
#include "string.h"
#include "timers.h"
#include "init_system.h"
#include "processing_modbus_extern.h"
#include "processing_reset_control.h"

// РАЗМЕР БУФФЕРА НА RX (SLAVE ЧИТАЕТ, MASTER )
#define MODBUS_SIZE_BUFF_IN 256
// РАЗМЕР БУФФЕРА НА TX (MASTER ЗАПИСЫВАЕТ)
#define MODBUS_SIZE_BUFF_OUT 256


// К-во пропусков между передачей запроса когда выставлен статус запроса
#define TIME_BETWEEN_ERROR_REQ_MASTER     10

// размер посылки исключения
#define	SIZE_EXCEPTION                     5


//-----------------МАКРОСЫ-----------------------------------
#define	P_SHIFT(X,Y)   (X+Y)


//------------Результаты проверки запроса мастер-----------------
typedef enum{
	REQ_MASTER_OK=0,
	REQ_MASTER_ERROR=10,
	REQ_MASTER_FUN_ADDRES_ERROR=11,
	REQ_MASTER_CRC_ERROR=12
}REZ_REQ_CHEACK_MASTER;


#pragma pack(push, 1)

// структура-аргумент, которая передаеться в задачи t_Modbus_MASTER и t_Modbus_SALVE
typedef struct {
	S_Task_parameters *ps_task_parameters; // указатель на структуру настроек буфферов порта USART, который используеться для данного  MODBUS
	u8                 num_modbus; // номер структуры настроек MODBUS_MASTER, MODBUS_SLAVE
}S_modbus_tsk_par;

typedef struct{
	S_ulist_modbus         *ps_ulist_modbus;
	REZ_REQ_CHEACK_MASTER  rez_zapros;     // результат выполнения запроса
	u8                     error_req_counter; // счетчик времени между передачей запроса при выставленом статусе запроса
	u8                     non_req_count;   // счетчик количества запрсов без ответа
} S_ulist_req_modbus;


// Структура настроек запросов Nodbus Master ДЛЯ ОДНОГО ПОРТА
typedef struct{
	S_ulist_req_modbus s_ulist_req_modbus[MAX_NUM_MODBUS_REQ]; // структуры настроек запросов
}S_list_madbus_req;



typedef struct {
	u8 byte_count;
	u8 payload;
}S_req_f16;

// варианты одного из полей MODBUS в запросе
// (описывает: запрос команды 1-5 - поля контрольной суммы, запрос команды 15-16 - поле к-ва байт полезной нагрузки и первый байт подезной нагрузки)
typedef union{
	u16 CRC_HiLo;
	S_req_f16 s_req_f16;
} UN_modbus_req_var;

// Структура описывающая первые 2 поля запроса modbus
typedef struct {
	u8 address_dev;
	u8 function;
	u8 address_mem_H;
	u8 address_mem_L;
    u8 number_reg_H;
	u8 number_reg_L;
	UN_modbus_req_var req_field_var;
}S_modbus_req_input;

// Структура описывающая поля запроса modbus на чтение (регистров) (ком. 1-4)
typedef struct {
	u8 address_dev;
	u8 function;
	u8 address_mem_L;
	u8 address_mem_H;
	u8 num_reg_L;
	u8 num_reg_H;
	u8 CRC_H;
	u8 CRC_L;
}S_modbus_req_read;

// Структура описывающая поля запроса modbus на установку (котушек) (ком. 5-6)
typedef struct {
	u8 address_dev;
	u8 function;
	u8 address_mem_L;
	u8 address_mem_H;
	u8 forse_data_L;
	u8 forse_data_H;
	u8 CRC_H;
	u8 CRC_L;
}S_modbus_req_write;


// варианты формирования одного из полей MODBUS в ответе
// (описывает: нормальный ответ - к-во байт в ответе, исключение - тип исключения)
typedef union{
	u8 type_exception;
	u8 num_byte_res;
} UN_modbus_excep_num;

// Структура описывающая поля формирования ответа modbus
typedef struct {
	u8 address_dev;
	u8 function;
	UN_modbus_excep_num un_excep_num;
	u8 resp_payload;
}S_modbus_make_res;

// указатель на функцию-обработчик для слейва (тоесть в теле драйвера  modbus ) !!
typedef u16 (*PF_procesing_req_slave)(void *, S_modbus_make_res *);


// указатель на функцию-обработчик дополнительных проверок и дополнительных внешних обработок
typedef REZ_REQ_CHEACK_SLAVE (*PF_modbus_callback_check)(void*);

// Структура установок дополнительной проверки параметров запроса
typedef struct{
	PF_modbus_callback_check pf_callback_check;
	u8 flag_callback_check;
} S_modbus_callback_check;

// указатель на функцию-обработчик дополнительных внешних обработок
typedef u8 (*PF_add_processing_req_slave)(void*,u8,u16);

// Структура установок дополнительной обработки параметров запроса
typedef struct{
	PF_add_processing_req_slave f_add_req_procesing;
	u8 flag_req_procesing;
} S_add_procesing_req;


#pragma pack(pop)

u8 modbus_fill_S_connectmodbus(u8 *p_read_data);
// ---ФУНКЦИИ ОБСЛУЖИВАНИЯ MODBUS MASTER------------------------
INIT_MBB_Rezult modbus_fill_S_modbus_req(u8* p_req_config_data);
u8 Put_MODBUS(u8 *buf_in, S_connectmodbus *s_connectmodbus_port,S_ulist_req_modbus *ps_ulist_req_modbus, u16 status);
u16 ModMasterResTrs(S_Task_parameters *ptaskparameters,\
		u8 *buf_in, S_connectmodbus *pconnectmodbus, \
		S_ulist_req_modbus *ps_ulist_req);
void t_Modbus_MASTER(void *pvParameters);
void modbus_master_req(S_ulist_req_modbus *ps_ulist_req_modbus,S_connectmodbus *ps_connectmodbus,S_Task_parameters* ps_task_parameters);
// ---ФУНКЦИИ ОБСЛУЖИВАНИЯ MODBUS SLAVE------------------------
REZ_REQ_CHEACK_SLAVE cheack_req_modbus(S_Task_parameters *p_iparameters, S_modbus_req_input *ps_modbus_req_cheack,u8 address_kp);
u16 func_0102(void *req, S_modbus_make_res *buf_respons);
u16 func_0304(void *req, S_modbus_make_res *buf_respons);
u16 func_05(void *req, S_modbus_make_res *buf_respons);
u16 func_06(void *p_req, S_modbus_make_res *p_buf_respons);
u16 func_16(void *req, S_modbus_make_res *buf_respons);
void t_Modbus_SLAVE(void *pvParameters);
u8 modbus_exception(S_modbus_req_input *ps_modbus_req, S_modbus_make_res *buf_respons,u8 exeption);
void modbus_callback_add_check(PF_modbus_callback_check p_modbus_callback_check_TY, u8 number_req);
void modbus_callback_address_check(PF_modbus_callback_check pf_modbus_callback_address_check, u8 number_req);
void modbus_callback_add_processing(PF_add_processing_req_slave p_modbus_callback_req, u8 number_req);


#endif /* MODBUS_H_ */
