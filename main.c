/*
    FreeRTOS V7.0.2 - Copyright (C) 2011 Real Time Engineers Ltd.

*/
/* Kernel includes. */
#include <stdlib.h>
#include <stddef.h>
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

#include "funct.h"
#include "USART_fifo_operation.h"
#include "init_system.h"
#include "global_dev.h"
#include "processing_config_dev.h"
#include "processing_mem_map.h"
#include "processing_modbus.h"
#include "processing_simple_gpio.h"
#include "processing_reset_control.h"


// файл с хидерами процессов устройства
#include "processing_includ.h"

//My experements
#include "my_task.h"



/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		1

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
#define mainQUEUE_SEND_FREQUENCY_MS			( 200 / portTICK_RATE_MS )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 1 )

//static void prvSetupHardware( void );

//указатель на структуру с указателями на буфера приема и передачи USART. из processing_USART.с
extern S_Task_parameters *ptask_parameters;
extern S_connectmodbus *ps_connectmodbus_global;
//структура описания карты памяти (из  mem_map_processing.c)
extern S_mem_map s_mem_map;
//масив указателей на USART которые пользователь разрешил использовать (из  processing_USART.c)
extern USART_TypeDef *present_usart[NUMBER_USART];
S_config_moduls s_config_moduls; // глобальная структура настроек МОДУЛЕЙ устройства, которая используеться програмными модулями в процесе работы програмы,
// в отличии от карты памяти (в карте памяти область настроек можно изменить, но до перезагрузки изменения не
// активизируються !! )


void delay_loop(){
	u32 coutner=0;
	while(coutner<1000000){
		coutner++;
	}
}

S_modbus_tsk_par s_modbus_tsk_par[NUM_PORT_MODBUS];
//---------------------------------------------------------------------------
int main(void)
{
	u8 k1;


	//t_processing_mesuremen();


	delay_loop(); // ожидаю пока стабилизируеться питание
	// настройка портов логического управления
	processing_simple_gpio();
	// настраиваю карту памяти и механизм доступа к ней
	processing_mem_map_init();
	// -----------------------------ЕСЛИ ПЕРВОЕ ВКЛЮЧЕНИЕ------------------------------------
	// если конфигурация по умолчанию и конфигурация пользователя не записаны - записываю
	processing_config_firest_on();
	// Считываю конфигурационные данные из FLESH памяти
	if(STATE_JAMPER1) // если джампер №1 НЕ установлен (лог 1 на входе) - считать конфигурацию пользователя
	{
		// Считываю конфигурацию пользователя
		INIT_MBB_read_addjust_table(s_mem_map.p_start_config_data,sizeof(S_global_config), PAGE_USER_CONFIG);
	}
	else // если джампер установлен ((лог 0 на входе))- считать конфигурацию по умолчанию
	{
		// Считываю конфигурацию по-умолчанию
		INIT_MBB_read_addjust_table(s_mem_map.p_start_config_data,sizeof(S_global_config), PAGE_DEFAULT_CONFIG);

	}


	// продолжаем конфигурацию устройства (см. описание функции)
	processing_config_dev_init();
	// Если устройство находиться в режиме "конфигурация по умолчанию", нужно в карту памяти считать к-цию пользователя
	// Это зделано для того, чтобы при считывании конфигурации, когда устройство находиться в режиме
	// "конфигурация по умолчанию", пользователь считывал к-цию пользователя, а не к-цию по умолчанию (к-ция
	// по умолчанию известна из документации на устройство).
	if(!STATE_JAMPER1){ // если джампер №1 установлен (лог 0 на входе) - считать конфигурацию пользователя
		INIT_MBB_read_addjust_table(s_mem_map.p_start_config_data,sizeof(S_global_config), PAGE_USER_CONFIG);
	}


	/*******************************************************************************/
	//---------------ЗАПУСКАЮ ЗАДАЧИ ПРОТОКОЛОВ СОГЛАСНО НАСТРОЕК--------------------
	//*******************************************************************************/
	// заполняю конфигурационные данные настройки портов и протоколов
	modbus_fill_S_connectmodbus((u8*)&s_config_moduls.s_connectmodbus_global);
	modbus_fill_S_modbus_req((u8*)&s_config_moduls.s_ulist_modbus);
	u8 count_modbus=0;
	for(k1=0;k1<NUMBER_USART;k1++){

		// если в конфигурации поточный порт включен, выполняю настройку modbus
		if(MODBUS_USART & (1<<k1)){

			if(!(s_config_moduls.s_connectmodbus_global[count_modbus].state)){// если в конфигурации поточный порт выключен
				count_modbus++;
				continue;
			}

			ConfigureUSART(&P_SHIFT(ps_connectmodbus_global,count_modbus)->s_port_config, present_usart[k1]); // настраиваю USART
			// описываю параметры задачи
			s_modbus_tsk_par[count_modbus].num_modbus=count_modbus;
			s_modbus_tsk_par[count_modbus].ps_task_parameters=ptask_parameters+k1;
			// Описываю задачу для заданого порта
			if(s_config_moduls.s_connectmodbus_global[count_modbus].type==PROTOCOL_MODBUS_MASTER){
				xTaskCreate( t_Modbus_MASTER, ( signed char * ) "Modbus_MASTER", 230, (void*)(&s_modbus_tsk_par[count_modbus]), mainQUEUE_RECEIVE_TASK_PRIORITY, NULL );
			}
			else if(s_config_moduls.s_connectmodbus_global[count_modbus].type==PROTOCOL_MODBUS_SLAVE){
				xTaskCreate( t_Modbus_SLAVE, ( signed char * ) "Modbus_SLAVE", 230, (void*)(&s_modbus_tsk_par[count_modbus]), mainQUEUE_RECEIVE_TASK_PRIORITY, NULL );
			}
			count_modbus++;
		};
	}
	//------------------------------------------------------------------------------------------------------------------------------------

	/*******************************************************************************/
	//---------------ЗАПУСКАЮ ЗАДАЧИ ПРОГРАМНЫХ ПРОЦЕСОВ СОГЛАСНО НАСТРОЕК----------/
	/*******************************************************************************/

#ifdef DEV_0
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_0).state){// если в конфигурации поточный порт выключен
		xTaskCreate(  TASK_PROCESSING(DEV_0), ( signed char * ) TASK_IDENT(DEV_1), 100,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_0), TASK_PRIORITY(DEV_0), NULL );
	}
#endif

#ifdef DEV_1
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_1).state){// если в конфигурации поточный модуль выключен
		xTaskCreate(  TASK_PROCESSING(DEV_1), ( signed char * ) TASK_IDENT(DEV_1), 100,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_1), TASK_PRIORITY(DEV_1), NULL );
	}
#endif

#ifdef DEV_2
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_2).state){// если в конфигурации поточный модуль выключен
		xTaskCreate(  TASK_PROCESSING(DEV_2), ( signed char * ) TASK_IDENT(DEV_2), 100,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_2), TASK_PRIORITY(DEV_2), NULL );
	}
#endif

#ifdef DEV_3
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_3).state){// если в конфигурации поточный модуль выключен
		xTaskCreate(  TASK_PROCESSING(DEV_3), ( signed char * ) TASK_IDENT(DEV_3), 100,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_3), TASK_PRIORITY(DEV_3), NULL );
	}
#endif

#ifdef DEV_4
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_4).state){// если в конфигурации поточный модуль выключен
		xTaskCreate(  TASK_PROCESSING(DEV_4), ( signed char * ) TASK_IDENT(DEV_4), 400,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_4), TASK_PRIORITY(DEV_4), NULL );
	}
#endif

#ifdef DEV_5
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_5).state){// если в конфигурации поточный модуль выключен
		xTaskCreate(  TASK_PROCESSING(DEV_5), ( signed char * ) TASK_IDENT(DEV_5), 100,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_5), TASK_PRIORITY(DEV_5), NULL );
	}
#endif

#ifdef DEV_6
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_6).state){// если в конфигурации поточный модуль выключен
		xTaskCreate(  TASK_PROCESSING(DEV_6), ( signed char * ) TASK_IDENT(DEV_6), 100,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_6), TASK_PRIORITY(DEV_6), NULL );
	}
#endif

#ifdef DEV_7
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_7).state){// если в конфигурации поточный модуль выключен
		xTaskCreate(  TASK_PROCESSING(DEV_7), ( signed char * ) TASK_IDENT(DEV_7), 100,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_7), TASK_PRIORITY(DEV_7), NULL );
	}
#endif

#ifdef DEV_8
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_8).state){// если в конфигурации поточный модуль выключен
		xTaskCreate(  TASK_PROCESSING(DEV_8), ( signed char * ) TASK_IDENT(DEV_8), 100,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_8), TASK_PRIORITY(DEV_8), NULL );
	}
#endif

#ifdef DEV_9
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_9).state){// если в конфигурации поточный модуль выключен
		xTaskCreate(  TASK_PROCESSING(DEV_9), ( signed char * ) TASK_IDENT(DEV_9), 100,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_9), TASK_PRIORITY(DEV_9), NULL );
	}
#endif

#ifdef DEV_10
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_10).state){// если в конфигурации поточный модуль выключен
		xTaskCreate(  TASK_PROCESSING(DEV_10), ( signed char * ) TASK_IDENT(DEV_10), 100,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_10), TASK_PRIORITY(DEV_10), NULL );
	}
#endif

#ifdef DEV_11
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_11).state){// если в конфигурации поточный модуль выключен
		xTaskCreate(  TASK_PROCESSING(DEV_11), ( signed char * ) TASK_IDENT(DEV_11), 100,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_11), TASK_PRIORITY(DEV_11), NULL );
	}
#endif

#ifdef DEV_12
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_12).state){// если в конфигурации поточный модуль выключен
		xTaskCreate(  TASK_PROCESSING(DEV_12), ( signed char * ) TASK_IDENT(DEV_12), 100,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_12), TASK_PRIORITY(DEV_12), NULL );
	}
#endif

#ifdef DEV_13
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_13).state){// если в конфигурации поточный модуль выключен
		xTaskCreate(  TASK_PROCESSING(DEV_13), ( signed char * ) TASK_IDENT(DEV_13), 100,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_13), TASK_PRIORITY(DEV_13), NULL );
	}
#endif

#ifdef DEV_14
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_14).state){// если в конфигурации поточный модуль выключен
		xTaskCreate(  TASK_PROCESSING(DEV_14), ( signed char * ) TASK_IDENT(DEV_14), 100,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_14), TASK_PRIORITY(DEV_14), NULL );
	}
#endif

#ifdef DEV_15
#if (TASK_PRIORITY(DEV_1)<MIN_T_PRIORITY)||(TASK_PRIORITY(DEV_1)>MAX_T_PRIORITY)
#error  Inavalide task DEV_1 priopity (Ger)
#endif
	if(s_config_moduls.USER_CONFIG_FIELD(s,DEV_15).state){// если в конфигурации поточный модуль выключен
		xTaskCreate(  TASK_PROCESSING(DEV_15), ( signed char * ) TASK_IDENT(DEV_15), 100,(void *)&s_config_moduls.USER_CONFIG_FIELD(s,DEV_15), TASK_PRIORITY(DEV_15), NULL );
	}
#endif

	//xTaskCreate( tskTS, ( signed char * ) "M1", 50, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL );
	//xTaskCreate( t_tsk_BLINK, ( signed char * ) "M1", 50, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL );

// задача управления процесом програмной презагрузки и световой индикации режима работы
	xTaskCreate(  t_processing_reset_control, ( signed char * ) "WatcDogTask", 70,	NULL, 4, NULL );

	//--------------НУ ..... ЗАПУСКАЮ ПЛАНИРОВЩИК-------------И ПОЛЕТЕЛИ!!!!!!!!!!-----------------------------
	vTaskStartScheduler();

	return 0;
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	volatile size_t xFreeStackSpace;

	/* This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amout of FreeRTOS heap that 
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
