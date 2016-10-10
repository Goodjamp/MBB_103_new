/*
 * processing_config_dev.h
 *
 *  Created on: April 17, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#ifndef  CONFIG_DEV_H_
#define  CONFIG_DEV_H_
#include "stm32f10x.h"
#include "funct.h"
#include "global_dev.h"
#include "string.h"
#include "processing_modbus.h"
#include "processing_mem_map_extern.h"
#include "processing_reset_control.h"
//#include "check_init.h"

// подключаимые модули

#include "processing_includ.h"

// максимаольное к-во устройств
#define MAX_NUM_MODUL       16

//адрес начала области конфигурации пользоватея (после поля CRC)
#define USER_CONFIG_START_ADRESS  (USER_ADDRESS_CONFIG_DATA+sizeof(S_dev_staff)/2)
//Размер области конфигурации пользоватея (после поля CRC)
#define USER_CONFIG_DATA_SIZE  (NUM_REG_CONFIG-sizeof(S_dev_staff)/2)


// размер области конфигураций карты памяти
#define NUM_REG_CONFIG        ((sizeof(S_global_config)+1)/2)

// Размер регистра конфигурации
#define SIZE_REG_CONFIG        2

#pragma pack(push, 1)


// битовое поле состава устройства
typedef struct{
	u16 MODULE_0:1;
	u16 MODULE_1:1;
	u16 MODULE_2:1 ;
	u16 MODULE_3:1 ;
	u16 MODULE_4:1 ;
	u16 MODULE_5:1 ;
	u16 MODULE_6:1 ;
	u16 MODULE_7:1 ;
	u16 MODULE_8:1 ;
	u16 MODULE_9:1 ;
	u16 MODULE_10:1 ;
	u16 MODULE_11:1 ;
	u16 MODULE_12:1 ;
	u16 MODULE_13:1 ;
	u16 MODULE_14:1 ;
	u16 MODULE_15:1 ;
}BF_dev_staff;


// битовое поле ДАТЫ
typedef struct{
	u32 day:5;      //31 -  0b11111   5 bit
	u32 mounth:4;   //12 -  0b1100    4 bit
	u32 year:6 ;    //63 -  0b111111  6 bit
	u32 hour:5 ;    //23    0b10111   5 bit
	u32 minute:6;   //60 -  0b111100  6 bit
	u32 second:6;   //60 -  0b111100  6 bit
}BF_date_config;




// структура состава устройства (все програмные модули и данные необходимые для конфигурации)
typedef struct{
	BF_dev_staff    bf_dev_staff;        // состав устройства
	u16             num_use_uart;        // к-во USART которые используються для передачи/приема данных в протоколе MODBUS
	u16             num_reg_data;        // размер пользовательской области карты памяти (в 2-х байтных регистрах)
	u16             modbus_req_user;     // максимально к-во запросов modbus_master на канал
	u16             program_version;     // версия микропрограмы
	u16             CRC_dev;             // размер области конфигурации                  (в 2-х байтных регистрах)
}S_dev_staff;


// Структура конфигураций всех програмных модулей, остается неизменной на
// протежении всего процеса работы програмы до перезагрузки (в отличии от  области s_config_moduls карты памяти, в которую может быть
// записана новая конфигурация в процесе переконфигурирования устройства).
// С полями даной структуры работают все переферийные модули (процесы) через указатели на них
typedef struct{
	S_connectmodbus s_connectmodbus_global[NUM_PORT_MODBUS];

#ifdef DEV_0
#define	DEV_0_CONTROL_BIT   0+NUM_PORT_MODBUS
	    USER_CONFIG_FIELD(S,DEV_0) USER_CONFIG_FIELD(s,DEV_0);
#endif
#ifdef DEV_1
#define	DEV_1_CONTROL_BIT   1+NUM_PORT_MODBUS
	    USER_CONFIG_FIELD(S,DEV_1) USER_CONFIG_FIELD(s,DEV_1);
#endif
#ifdef DEV_2
#define	DEV_2_CONTROL_BIT   2+NUM_PORT_MODBUS
	    USER_CONFIG_FIELD(S,DEV_2) USER_CONFIG_FIELD(s,DEV_2);
#endif
#ifdef DEV_3
#define	DEV_3_CONTROL_BIT   3+NUM_PORT_MODBUS
	    USER_CONFIG_FIELD(S,DEV_3) USER_CONFIG_FIELD(s,DEV_3);
#endif
#ifdef DEV_4
#define	DEV_4_CONTROL_BIT   4+NUM_PORT_MODBUS
	    USER_CONFIG_FIELD(S,DEV_4) USER_CONFIG_FIELD(s,DEV_4);
#endif
#ifdef DEV_5
#define	DEV_5_CONTROL_BIT   5+NUM_PORT_MODBUS
	USER_CONFIG_FIELD(S,DEV_5) USER_CONFIG_FIELD(s,DEV_5);
#endif
#ifdef DEV_6
#define	DEV_6_CONTROL_BIT   6+NUM_PORT_MODBUS
	    USER_CONFIG_FIELD(S,DEV_6) USER_CONFIG_FIELD(s,DEV_6);
#endif
#ifdef DEV_7
#define	DEV_7_CONTROL_BIT   7+NUM_PORT_MODBUS
	    USER_CONFIG_FIELD(S,DEV_7) USER_CONFIG_FIELD(s,DEV_7);
#endif
#ifdef DEV_8
#define	DEV_8_CONTROL_BIT   8+NUM_PORT_MODBUS
	    USER_CONFIG_FIELD(S,DEV_8) USER_CONFIG_FIELD(s,DEV_8);
#endif
#ifdef DEV_9
#define	DEV_9_CONTROL_BIT   9+NUM_PORT_MODBUS
	    USER_CONFIG_FIELD(S,DEV_9) USER_CONFIG_FIELD(s,DEV_9);
#endif
#ifdef DEV_10
#define	DEV_10_CONTROL_BIT   10+NUM_PORT_MODBUS
	    USER_CONFIG_FIELD(S,DEV_10) USER_CONFIG_FIELD(s,DEV_10);
#endif
#ifdef DEV_11
#define	DEV_11_CONTROL_BIT   11+NUM_PORT_MODBUS
	    USER_CONFIG_FIELD(S,DEV_11) USER_CONFIG_FIELD(s,DEV_11);
#endif
#ifdef DEV_12
#define	DEV_12_CONTROL_BIT   12+NUM_PORT_MODBUS
	    USER_CONFIG_FIELD(S,DEV_12) USER_CONFIG_FIELD(s,DEV_12);
#endif
#ifdef DEV_13
#define	DEV_13_CONTROL_BIT   13+NUM_PORT_MODBUS
	    USER_CONFIG_FIELD(S,DEV_13) USER_CONFIG_FIELD(s,DEV_13);
#endif
#ifdef DEV_14
#define	DEV_14_CONTROL_BIT   14+NUM_PORT_MODBUS
	    USER_CONFIG_FIELD(S,DEV_14) USER_CONFIG_FIELD(s,DEV_14);
#endif
#ifdef DEV_15
#define	DEV_15_CONTROL_BIT   15+NUM_PORT_MODBUS
	    USER_CONFIG_FIELD(S,DEV_15) USER_CONFIG_FIELD(s,DEV_15);
#endif

	S_ulist_modbus s_ulist_modbus[NUM_PORT_MODBUS*MAX_NUM_MODBUS_REQ];
} S_config_moduls;

// структура конфигурации устройства
typedef struct{
	S_dev_staff     s_dev_staff;           //основные парамет
	BF_date_config  bf_date_config;        // дата конфигурации
	S_config_moduls s_config_moduls;       // конфигурационные структуры составаных частей устройства
}S_global_config;


// Наполнение данных состава устройства:
// состав устройства (програмные модули) - битовое полле (2 байта)
// дата последней конфигурации           - битовое полле (2 байта)
// к-во задействованных USART            -               (1 байт)
// размер карты памяти                   -               (2 байта)
// к-во регстров пользователя            -               (1 байт)  равно количеству возможных ретрянсляций
// к-во оперативных регистров            -               (2 байта)
// к-во регистров конфигурации           -               (2 байта)


#pragma pack(pop)

//--------------------Прототипы ф-й конфигурации------------------
void processing_config_firest_on(void);
void processing_config_init_not_user_config(S_dev_staff *ps_dev_staff);
u8 update_config_data(void* req,u8 num_peyload_data, u16 addres_data);
void processing_config_dev_init(void);
REZ_REQ_CHEACK_SLAVE processing_config_check_is_holding_reg(void* p_check_address);
REZ_REQ_CHEACK_SLAVE processing_config_check_is_input_reg(void* p_check_address);
REZ_REQ_CHEACK_SLAVE processing_config_check_is_preset_multiple_reg(void* p_check_address);
// ----------------Прототипы функций файла конфигурации------------------
void default_config_table(S_global_config *ps_mem_data_set);

#endif
