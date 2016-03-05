/*
 * global_dev.h
 *
 *  Created on: March 26, 2015
 *      Author: Gerasimchuk
 */

#ifndef GLOBAL_DEV_H_
#define  GLOBAL_DEV_H_

// макрос для подключения хидера нового устройства по макроопредилению
#define MY_FAIL(X)        _MY_FAIL(X)
#define _MY_FAIL(NAME)   <processing_##NAME.h>

// макрос для конструирования имени структуры конфигурации
#define USER_CONFIG_FIELD(Y,X)         _USER_CONFIG_FIELD(Y,X)
#define _USER_CONFIG_FIELD(TYPE,NAME)     TYPE##_##NAME##_user_config

// макрос для конструирования имени структуры адресов оперативных регистров
#define USER_ADDRESS_FIELD(Y,X)         _USER_ADDRESS_FIELD(Y,X)
#define _USER_ADDRESS_FIELD(TYPE,NAME)     TYPE##_##NAME##_address

// макрос для конструирования имени структуры оперативных регистров
#define USER_OPER_DATA_FIELD(Y,X)         _USER_OPER_DATA_FIELD(Y,X)
#define _USER_OPER_DATA_FIELD(TYPE,NAME)     TYPE##_##NAME##_oper_data

// макрос для конструирования имени функции-потока
#define TASK_PROCESSING(X)         _TASK_PROCESSING(X)
#define _TASK_PROCESSING(NAME)     t_processing_##NAME

// макрос для конструирования имени функции-рассчета адрессов оперативных регистров подключаимого устройства
#define CALC_ADDRESS_OPER_REG_PROCESSING(X,Y)         _CALC_ADDRESS_OPER_REG_PROCESSING(X,Y)
#define _CALC_ADDRESS_OPER_REG_PROCESSING(NAME,Y)     NAME##_calc_address_oper_reg(&s_address_oper_data.USER_ADDRESS_FIELD(s,NAME),Y)

// макрос для конструирования строки в скобках
#define TASK_IDENT(X)    _TASK_IDENT(X)
#define _TASK_IDENT(X)    #X

//макрос для конструирования строки приоритета задачи
#define TASK_PRIORITY(X)   X##_T_PRIORITY

//макрос для конструирования имени-указателя на USART
#define USART(X)   _USART(X)
#define _USART(NUM)   USART##NUM


#endif
