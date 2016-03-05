/*
 * P_watchdog.c
 *
 *  Created on: Jun 2, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "processing_reset_control.h"

// структура адресов оперативных регистров (из processing_mem_map.c)
extern S_address_oper_data s_address_oper_data;
extern S_mem_map s_mem_map;
// Семафор для обработки структуры флагов состояния процесов
xSemaphoreHandle SemaphoreHandle_watchdog;
// к-во циклов ожидания приема второго шага для перезагрузки
u16 count_reset=MAX_TIME_RESET/PERIOD_TASK;
// счетчик к-ва циклов после приема первого шага до перезагрузки
BF_processing_reset_control_ bf_counter_reset_control={0,0};
// копия регистра device_rcr;
u16 data_reset_reg;
//параметры режимов индикации
S_mode_blink s_mode_blink={
		MOD_1_ON*10/PERIOD_TASK,MOD_1_OFF*10/PERIOD_TASK,
		MOD_2_ON*10/PERIOD_TASK,MOD_2_OFF*10/PERIOD_TASK,
		MOD_3_ON*10/PERIOD_TASK,MOD_3_OFF*10/PERIOD_TASK,
		0,WORK_OK
};
// асоциативный масив для установки битов статуса
u8 a_bit_status[MAX_NUM_MODULS]={[0 ... (MAX_NUM_MODULS-1)]=0};

//-------------p_watchdog_init------------------
//функция p_watchdog_init - выполняет конфигурацию watchdog
// входные аргументы:
// prescale               - делитель частоты LSI счетчика watchdog,
// prelode                - величина счета watchdog (макс 4095)
u8 p_watchdog_init(WATCHDOG_PRESCALER prescale, u16 prelode ){
	if(prelode>0x0FFF){return 1;}
	// запускаю внутрений низкоскоростной генератор LSI
	RCC_LSICmd(ENABLE);
	// выбираю LSI как источник тактирования IWD
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
	// КОНФИГУРИРУЮ WATCHDOG. Источник тактового сигнала LSI, частота - 40 кГц
	// Получаем доступ к регистрам PR и RLR
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(prescale);  // утанавливаем передделитель
	IWDG_SetReload(prelode); // устанавливаю перезагружаимую величину счета (WATCHDOG считает вниз от этого числа)
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable); // закрываю доступ к регистрам PR и RLR
	return 0;
}


//-------------p_watchdog_enable------------------
//функция p_watchdog_enable - включает watchdog
void p_watchdog_enable(void){
	IWDG_Enable();
};


//-------------p_watchdog_reset------------------
//функция p_watchdog_reset - сбрасывает watchdog
void p_watchdog_reset(void){
	IWDG_ReloadCounter();
}

void p_watchdog_set_flag(void){
	xSemaphoreTake(SemaphoreHandle_watchdog, portMAX_DELAY);
	xSemaphoreGive(SemaphoreHandle_watchdog);
}

//------------функция processing_reset_control_single_registr-----------------------------------
// функция processing_reset_control_single_registr - выполняет проверку адресса карты памяти на принадлежность регистрам конфигурации устройства
// даная функция соответствует проверке команде №6 modbus
// входные аргументы:
// *p_check_address - указатель на начало последовательности (u16)address, (u16)num_reg
// выходные аргументы:
// REQ_SLAVE_OK - проверка выполнена успешно
// ILLEGAL_DATA_ADRESS - недопустимый адрес
static REZ_REQ_CHEACK_SLAVE processing_reset_control_single_registr(void* p_check_address){
	if(s_address_oper_data.s_address_devise_reset_status.address_device_rcr!=*((u16*)p_check_address)){
		return ILLEGAL_DATA_ADRESS;
	}
	return REQ_SLAVE_OK;
}


//------------функция processing_config_set_dev_error-----------------------------------
// функция processing_config_set_dev_error - выполняет установку аварийного бита устройства в глобальном статус регистре устройства
//                                           и включает аварийную сигнализацию
// входные аргументы:
//num_bit - номер бита который нужно установить
void processing_reset_control_set_dev_error(u8 num_bit, FlagStatus state){
	u16 state_status_reg, counter;
	// нахожу порядочный номер бита модуля, который вызвал данную функцию
	for(counter=0;counter<MAX_NUM_MODULS;counter++){
		if(a_bit_status[counter]==num_bit){break;}
	}
	// считываю поточный статус устройства
	processing_mem_map_read_s_proces_object_modbus(&state_status_reg,1, s_address_oper_data.s_address_devise_reset_status.address_device_statys);
	//установить статус-бит в глобальном статус регистре
	if(state==SET){
		SET_BIT_(state_status_reg,counter);
	}
	//очистить статус-бит в глобальном статус регистре
	else if(state==RESET){
		CLEAR_BIT_(state_status_reg,counter);
	}
	// Записываю обновленный статус регистр устройства
	processing_mem_map_write_s_proces_object_modbus(&state_status_reg,1, s_address_oper_data.s_address_devise_reset_status.address_device_statys);
	if(state_status_reg){
		s_mode_blink.work_mode=ERROR_MODE;
	}
	else{
		s_mode_blink.work_mode=WORK_OK;
	}
}


//--------функция processing_reset_control_reset_control--------------
//функция processing_reset_control_reset_control - выполняет управления програмной перезагрузкой
//(следит за состоянием регистра перезагрузки, и при выполнении условий перезагрузки -
// сбрасывает устройство)
// АЛГОРИТМ ПЕРЕЗАГРУЗКИ:
// 1) записать в регистр по адресу address_device_rcr число RESET_FIREST_STEP,
// 2) на протяжении времени не более MAX_TIME_RESET мс  записать в регистр по
//    адресу address_device_rcr число RESET_SECOND_STEP
static void processing_reset_control_reset_control(void){
	u16 zero=0;
	// считываю регистр управления перезагрузкой
	processing_mem_map_read_s_proces_object_modbus(&data_reset_reg,1,s_address_oper_data.s_address_devise_reset_status.address_device_rcr);
	// анализирую поточную ситуацию:
	if(bf_counter_reset_control.f_rx_1_step){ // если флаг установлен, был принят первый шаг
		bf_counter_reset_control.counter++;
		if(data_reset_reg==RESET_SECOND_STEP){ // если принят второй шаг ПЕРЕЗАГРУЗКА
			NVIC_SystemReset();// выполняю програмную перезагрузку для обновления конфигурационных данных
			return;
		}
		if(bf_counter_reset_control.counter>count_reset){ // если превышен лимит времени ожидания второго шага
			bf_counter_reset_control.f_rx_1_step=0;
			bf_counter_reset_control.counter=0;
			processing_mem_map_write_s_proces_object_modbus(&zero,1,s_address_oper_data.s_address_devise_reset_status.address_device_rcr);
		}
		else{
			return;
		}
	}
	else{// если флаг не установлен, первый шаг не принят
		if(data_reset_reg==RESET_FIREST_STEP){ // если принят прервый шаг
			bf_counter_reset_control.f_rx_1_step=1;
			return;
		}
		return;
	}
}


//-------------функция processing_reset_control_blink----------------
// функция processing_reset_control_blink - выполняет управление индикацией
// режима работы устройства:
// - устройство функционирует нормально:
// - неправильная конфигурация от одного до всех модулей
// - ошибка от одного до всех модулей
static void processing_reset_control_blink(void){
	u8 mode=s_mode_blink.work_mode;
	u8 counter_per=s_mode_blink.counter;
	u8 state=s_mode_blink.state_blink;
	if(state){
		if(counter_per>=s_mode_blink.as_blink_par[mode].time_on){
			// выключить индикацию
			IND_ERROR_RESET;
			s_mode_blink.state_blink=0;
			s_mode_blink.counter=0;
			return;
		}
	}
	else{
		if(counter_per>=s_mode_blink.as_blink_par[mode].time_off){
			// включить индикацию
			IND_ERROR_SET;
			s_mode_blink.state_blink=1;
			s_mode_blink.counter=0;
			return;
		}
	}
	s_mode_blink.counter++;
}

//-------------функция init_a_bit_status----------------
// функция init_a_bit_status - заполняет асоциативный масив статус-битов устройства используя следующий принцип:
// из битового поля состава устройства выбираються номера позиции битов заданных в даном устройстве модулей и
// последовательно переносяться в масив a_bit_status. Номер позиции слаживаеться с NUM_PORT_MODBUS (первые NUM_PORT_MODBUS
// битов - это биты портов MODBUS)
void init_a_bit_status(void){
	u8 counter_bit, counter_dev;
	u16 *p_dev_staff=(u16*)&((S_global_config*)s_mem_map.p_start_config_data)->s_dev_staff.bf_dev_staff;
	// если в составе устройства присутствует MODBUS (первый или последний биты состава устройства), значит первые позиции
	// статус масива - номер полей статуса порта
	counter_dev=0;
	counter_bit=0;
	if(((0b1)& (*p_dev_staff))||((0b1000000000000000)& (*p_dev_staff)))
	{
		for(counter_bit=0;counter_bit<NUM_PORT_MODBUS;counter_bit++)
			{
			a_bit_status[counter_bit]=counter_bit;
			}
		counter_bit=1;
		counter_dev=NUM_PORT_MODBUS;
	}
	// лписываю позиции статус - битов других переферийных модулей
	// от counter_bit=1 - потому что на первом месте позиция  MODBUS (проанализированно выше)
	// до MAX_NUM_MODUL-1 - потому что на последнем месте позиция MODBUS (проанализированно выше)
	for(;counter_bit<(MAX_NUM_MODUL-1);counter_bit++)
	{
		if((1<<counter_bit)& (*p_dev_staff)){
			a_bit_status[counter_dev]=counter_bit+NUM_PORT_MODBUS;
			counter_dev++;
		}
	}
}


//---------------задача t_processing_watchdog---------------
// звдача t_processing_watchdog - выполняет настройку и обработку сторожевого таймера
void t_processing_reset_control(void* in_Data){
	// задаю ф-ю callback которая вызываться процессом modbus_slave для проверки запрошиваимого адресса запросом №6
	modbus_callback_address_check(&processing_reset_control_single_registr,PRESET_SINGLE_REGISTERS);
	// заполняю асоциативный масив статус-битов устройства
	init_a_bit_status();
	//Создаю семафор доступа к флагоам состояния процесов
	vSemaphoreCreateBinary(SemaphoreHandle_watchdog);
	// инициилизирую СТОРОЖЕВОЙ ТАЙМЕР (watchdog)
	p_watchdog_init(WDG_PRESCALER_32,4000);
	p_watchdog_enable();
	while(1){
		//xSemaphoreTake(SemaphoreHandle_watchdog, portMAX_DELAY);
		// анализирую состояние задач
		//xSemaphoreGive(SemaphoreHandle_watchdog);
		vTaskDelay(PERIOD_TASK);
		// выполняю управление процесом индикации
		processing_reset_control_blink();
		//выполняб управление процесом перезагрузки
		processing_reset_control_reset_control();
		// сбрасываю сторожевой таймер
		p_watchdog_reset();
	}
}
