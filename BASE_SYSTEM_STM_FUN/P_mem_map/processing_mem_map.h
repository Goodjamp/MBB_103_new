/*
 * processing_mem_map.h
 *
 *  Created on: December 29, 2014
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#ifndef PROCESSING_MEM_MAP_H_
#define PROCESSING_MEM_MAP_H_

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32f10x_usart.h"

//***********************************  МАКРООПРЕДИЛЕНИЯ НАСТРОЙКИ ПРОГРАМИСТА  ******************************
//--------------------------------------------------начало----------------------------------------------------------

//--------ЗАДАЮ ОБЩЕЕ К-ВО И УКАЗАТЕЛИ USART-тов в используемом МК---------
// даное опредиление должно быть описано именнов ЭТОМ файле (а не в processing_USART.h),
// так как processing_USART.h - это часть драйвера, а в этом хидере описаны макроопредиления настроек пользователя
//к-во USART
#define	NUMBER_USART		3
//номера USART
#define	PRESENT_USART     USART1,\
	                      USART2,\
	                      USART3\

//---ВЫБОР К-ВА И НОМЕРОВ USART КОТОРЫЕ ИСПОЛЬЗУЮТЬСЯ ДЛЯ ПЕРЕДАЧИ/ПРИЕМА ДАННЫХ В ПРОТОКОЛЕ MODBUS---------
// К-во портов на которых возможно задействовать протокол
#define NUM_PORT_MODBUS     3
//номера USART-тов
#define MODBUS_USART        (WORKED_USART1|WORKED_USART2|WORKED_USART3)
//#define MODBUS_USART          (WORKED_USART3)

// максимально - возможное к-во одновременно задействованых независимых модулей
#define MAX_NUM_MODULS     NUM_PORT_MODBUS+3

//Общее количество регистров ввода/вывода общего назначения
#define NUM_REG_DATA              50

//--------------------------------------------------конец----------------------------------------------------------
//***********************************  МАКРООПРЕДИЛЕНИЯ НАСТРОЙКИ ПРОГРАМИСТА  ******************************

// Относительный (видимый пользователю) адрес начала области регистров ввода/вывода общего назначения
#define USER_ADDRESS_DATA         0
// Относительный (видимый пользователю) адрес начала области оперативных регистров
#define USER_ADDRESS_OPER_DATA    500
// Относительный (видимый пользователю) адрес начала области настроек
#define USER_ADDRESS_CONFIG_DATA  1000


// Размер типового proces_object
#define SIZE_PROCES_OBJECT        sizeof(S_proces_object_modbus)
// размер поля DATA структуры S_proces_object_modbus
#define MAX_DATA_OBJ_MODBUS					0x02




// Параметрические макросы
#define PPROCOBJ_2DATA(x)        *(S_proces_object_modbus*)(x)
#define POINT_TO_USER_ADDRESS(X) ((u16)X)/sizeof(S_proces_object_modbus)
//+USER_ADDRESS_OPER_DATA


//------ОПИСАНИЕ ИМЕННОВАНЫХ ТИПОВ ДАННЫХ-----------------------
// результат обращения к карте памяти
typedef enum{
	MEM_ERROR,
	MEM_OK
}ERROR_mem;

// типы принадлежности регистров по типу данных
typedef enum{
	ERROR_DATA=0,
	OPER_DATA=1,
	PMZ_DATA=2,
	CONFIG_DATA=3
}TYPE_REG_MEM_MAP;



//----------ОПИСАНИЕ ТИПОВ УКАЗАТЕЛЕЙ НА Ф-И--------------------------
typedef ERROR_mem (*P_mem_map_processing_check_address)(void*);


//------ОПИСАНИЕ ОБЩЕПОЛЬЗОВАТЕЛЬСКИХ СТРУКТУР-----------------------

#pragma pack(push, 1)

// единицы информации карты памяти (СЕЙЧАС ОБЩАЯ ДЛЯ ВСЕХ (ТС, ТУ, ТИ) ТИПОВ ДАННЫХ)
typedef struct
{
	unsigned char date[MAX_DATA_OBJ_MODBUS];
	unsigned char flag;
} S_proces_object_modbus;


// разлинениваю карту памяти
typedef struct{
	u16 size_mem_map;  // размер карты памяти= сума размеров первой (регистры ввода/вывода), второй (оперативные регистры) и третей (конфигурация) частей карты памяти
	u8 *ps_proces_object_mem_map; // указатель на начало карты памяти
	u8 *p_start_DATA;             // адрес начала настраиваимой части карты памяти
	u8 *p_start_oper_data;        // адрес начала оперативных регистров
	u8 *p_start_config_data;      // адрес начала данных конфигурации
} S_mem_map;



#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------

void processing_mem_map_init(void);
u8 processing_mem_map_init_mem_map(void);
ERROR_mem processing_mem_map_write_s_proces_object_modbus( const u16 *p_data,const u16 num_data, const u16 address);
ERROR_mem processing_mem_map_read_s_proces_object_modbus( u16 *p_data, u16 num_data,u16 address);
TYPE_REG_MEM_MAP processing_mem_map_detect(u16 data_user_addres, u8 **p_addres_mem_map);

#endif
