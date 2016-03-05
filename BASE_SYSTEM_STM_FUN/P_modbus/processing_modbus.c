/*
 * modbus.c
 *
 *  Created on: Apr 10, 2012
 *      Author: Vitaliy Tikhenko
 */

#include <stddef.h>
#include "processing_modbus.h"
#include "processing_mem_map_extern.h"

extern S_proces_object_modbus *p_proces_object;
// структура оперативных регистров (из mem_map_processing.c)
//extern S_oper_data *ps_oper_data;
// структура адресов оперативных регистров (из mem_map_processing.c)
extern S_address_oper_data s_address_oper_data;
// масив указателей на функции проверки допустимых адресов, (из mem_map_processing.c)
extern P_mem_map_processing_check_address p_mem_map_processing_check[8];
//конст НОЛЬ
const u16 ZERO=0;
// масив структур настроек каждого порта
S_connectmodbus *ps_connectmodbus_global; //[NUM_PORT_MODBUS];
// структура настроек запросов modbas_master  для каждого из портов (порядочный номер порта - (индекс масива+1 ))
S_list_madbus_req s_list_madbus_req[NUM_PORT_MODBUS];
// масив указателей на функции проверки адресации запросов
PF_procesing_req_slave p_procesing_slave[17] = { &func_0102, &func_0102,
		&func_0304, &func_0304, &func_05, &func_06, 0, 0, 0, 0, 0, 0, 0, 0, &func_16,
		&func_16 };
//-------------------------

// Масив структур для функций проверки адресов в запросе (типа callback, задает пользователь)
S_modbus_callback_check s_modbus_check_address_req[16];
// масив указателей на функции дополнительных проверок
S_modbus_callback_check s_modbus_check_add_check[16];
// масив указателей на функции обработчики запросов слейва
S_add_procesing_req s_modbus_add_procesing_req[16];

//------------функция INIT_MBB_fill_S_connectmodbus---------------
// функция INIT_MBB_fill_S_connectmodbus - заполняет структуры настройки портов
u8 modbus_fill_S_connectmodbus(u8 *p_read_data) {
	u8 k1;
	ps_connectmodbus_global = (S_connectmodbus*) p_read_data;

	for (k1 = 0; k1 < NUM_PORT_MODBUS; k1++) {
		if (!((ps_connectmodbus_global + k1)->state)) {
			continue;
		} // если порт отключен, перехожу к анализу следующего порта
		//Анализ конфигурации порта
	}

	return MBB_INIT_OK;
}

// -----------функция  INIT_MBB_fill_S_modbus_master------------
// функция  INIT_MBB_fill_S_modbus_master - заполнение структур настройки запросов modbus_master
INIT_MBB_Rezult modbus_fill_S_modbus_req(u8* p_req_config_data) {
	u8 k1, k2;
	S_ulist_modbus* ps_ulist_modbus = (S_ulist_modbus*) p_req_config_data;
	for (k1 = 0; k1 < NUM_PORT_MODBUS; k1++) {
		for (k2 = 0; k2 < MAX_NUM_MODBUS_REQ; k2++) {
			s_list_madbus_req[k1].s_ulist_req_modbus[k2].ps_ulist_modbus =	ps_ulist_modbus;
			ps_ulist_modbus++;
		}
	}

	return MBB_INIT_OK;
}

//-------------------------END CONST-------------------------------------------------
//----------------------MODBUS MASTER---------------------------------------------------------------------------
//-----функция Put_MODBUS--------------------------------
// функция Put_MODBUS - выполняет обработку ответа, заполняет регистры данных и статус регистры
// входные аргументы:
//
//выходные аргументы:
//
u16 bit_value;
u8 Put_MODBUS(u8 *buf_in, S_connectmodbus *s_connectmodbus_port,S_ulist_req_modbus *ps_ulist_req_modbus, u16 status) { // НУЖНО  ЕЩЕ ПЕРЕДАТЬ СТАТУС ПРИЕМА ОТВЕТА, ДЛЯ ЗАПОЛНЕНИЯ СТАТУС РЕГИСТРА
	u16 *p_payload;
	u8 counter;

	u16 rez_status;
	if (ps_ulist_req_modbus->rez_zapros) {
		ps_ulist_req_modbus->non_req_count++; // увеличиваю счетчик запросов без ответов
		if (ps_ulist_req_modbus->non_req_count >= s_connectmodbus_port->number_no_answer) //если к-во запросов без ответа превысило допустимое - выставить соответствующий статус
		{
			//rez_status = (1<<ps_ulist_req_modbus->rez_zapros);
			rez_status = (ps_ulist_req_modbus->rez_zapros);
			//выставляю статус запроса
			processing_mem_map_write_s_proces_object_modbus(&rez_status, 1,	ps_ulist_req_modbus->ps_ulist_modbus->adres_status);
		}
		return 1;
	}
	// если обрабатываимый запрос был из выставленым статусам (slave начал отвечать) убрать все признаки ошибок
	if(status){
		ps_ulist_req_modbus->non_req_count=0;
		processing_mem_map_write_s_proces_object_modbus(&ZERO, 1,	ps_ulist_req_modbus->ps_ulist_modbus->adres_status);
	}
	//--------------ЕСЛИ ОТВЕТ ПРИНЯТ------------------------------------------------
	ps_ulist_req_modbus->non_req_count = 0; // обнуляю счетчик запросов без ответов для данного запроса
	//-------------------------------------------------------------------------------
	p_payload=(u16*) &buf_in[3];  //указатель на начало полезных данных
	// сохраняю полезную нагрузку согласно настройкам запроса
	switch (ps_ulist_req_modbus->ps_ulist_modbus->function) {
	case (READ_COIL_STATUS): // Если запрашивал функции 1 или 2
	case (READ_INPUT_STATUS):
		for(counter=0;counter<ps_ulist_req_modbus->ps_ulist_modbus->number;counter++){
			bit_value=(u16)get_array_bit((u8*)p_payload,counter);
			processing_mem_map_write_s_proces_object_modbus((u16*)&bit_value,(u16)1,ps_ulist_req_modbus->ps_ulist_modbus->adress_pmz+counter);
		}
	break;
	case (READ_HOLDING_STATUS): // Если запрашивал функции 3 или 4
	case (READ_INPUT_REGISTERS):
	// меняю местами байты: Hi->Lo, Lo-Hi
		for(counter=0;counter<ps_ulist_req_modbus->ps_ulist_modbus->number;counter++){
			Convert_ShortTooCharHiLo((*p_payload),(u8*)p_payload);
			p_payload++;
		}
		processing_mem_map_write_s_proces_object_modbus((u16*) &buf_in[3],ps_ulist_req_modbus->ps_ulist_modbus->number,	ps_ulist_req_modbus->ps_ulist_modbus->adress_pmz);
	break;
	default:
		break;
	};
	return 0;
}

//-----------функция ModMasterResTrs---------------------
// функция ModMasterResTrs - выполняет передачу запроса МодбасМастера в сторону слейва, прием ответа и обработку принятого сообщания на предмет ошибок (СRC)
// входные аргументы:
//                 *ptaskparameters     - указатель на структуру с описанием порта в который передаеиться запрос
//                 *buf_in              - буфер ответа
//                 *pconnectmodbus      - настройки даного запроса в контексте настроек порта
//                 *ps_ulist_req        - настройки поточного запроса
// выходные аргументы

u16 ModMasterResTrs(S_Task_parameters *ptaskparameters, u8 *buf_in,
		S_connectmodbus *pconnectmodbus, S_ulist_req_modbus *ps_ulist_req) {
	u16 NumberByteResponse;
	u16 NumberRead;
	S_modbus_make_res *ps_modbus_res = (S_modbus_make_res*) buf_in;
	S_modbus_req_write s_modbus_req;
	union {
		u8 rez_u8[2];
		u16 rez_u16;
	} resCRC16;

	//формирую запрос
	s_modbus_req.address_dev = ps_ulist_req->ps_ulist_modbus->adress; //адрес слейв
	s_modbus_req.function = ps_ulist_req->ps_ulist_modbus->function; //номер функции
	Convert_ShortTooCharHiLo(ps_ulist_req->ps_ulist_modbus->start_adres,&s_modbus_req.address_mem_L);
	//адрес памяти
	Convert_ShortTooCharHiLo(ps_ulist_req->ps_ulist_modbus->number,	&s_modbus_req.forse_data_L);
	// к-во байт данных
	modus_add_CRC((u8*) &s_modbus_req, 8); // рассчитываю и добавляю контрольную сумму
	// передаю запрос
	write_Usart_Buf(ptaskparameters->USARTx, (u8*) &s_modbus_req, 8,ptaskparameters->WrUSART); // передаю запрос


	// если запрашивалась функция 1 или 2, перещет битов в к-во байтов полезной нагрузки
	if ((s_modbus_req.function == READ_COIL_STATUS) || (s_modbus_req.function == READ_INPUT_STATUS)) // если запрошивали функции 1 или 2
	{
		NumberByteResponse = ps_ulist_req->ps_ulist_modbus->number / 8;
		if ((NumberByteResponse * 8) != ps_ulist_req->ps_ulist_modbus->number) {
			NumberByteResponse++;
		}
	}
	else{
		// Расчитываю размер полезной нагрузки ответа в БАЙТАХ для ф-и № 3-6
		NumberByteResponse = ps_ulist_req->ps_ulist_modbus->number * 2; // к-во БАЙТ полезной нагрузки к приему
	}

	NumberByteResponse += 5; // общая длина ответа в БАЙТАХ с учетом шапки и контрольной суммы

	// ожидаю прием ответа и нализирую к-во принятых байт
	//--------!!!!!!!!!!!!!!!!!!!!!!ДОБАВИТЬ АНАЛИЗ НА ПРИЕМ ИСКЛЮЧЕНИЯ !!!!!!!!!!!!!!!!!!!!!!------------------
	NumberRead= ReadUSART(ptaskparameters->RdUSART, (u8*) ps_modbus_res,NumberByteResponse, pconnectmodbus->waitresp);
	if ((NumberByteResponse!= NumberRead)&&(SIZE_EXCEPTION!= NumberRead)) {// если ожжидаемое к-во считанных байт не равно реальнго считанным, два варианта:
		//ОШИБКА
		return REQ_MASTER_ERROR; // ответ - ошибка
	};

	// проверяю поля ответа
	if ((ps_modbus_res->address_dev != ps_ulist_req->ps_ulist_modbus->adress)||
			((ps_modbus_res->function!= ps_ulist_req->ps_ulist_modbus->function)&&
			 (ps_modbus_res->function-0x80!= ps_ulist_req->ps_ulist_modbus->function)) // в ответе-исключении поле "функция" передаеться в формате "функция + 0х80"
			 ) // если в принятом ответе не правильное поле адрес или поле функция
	{
		return REQ_MASTER_FUN_ADDRES_ERROR;
	}

	// расчитываю и проверяю контрольную суму
	resCRC16.rez_u16 = CRC16((u8*) ps_modbus_res, NumberRead - 2);
	if (resCRC16.rez_u8[1] != *((u8*) ps_modbus_res + NumberRead - 2)&& (resCRC16.rez_u8[0]!= *((u8*) ps_modbus_res + NumberRead - 1))) // если контрольная сума  совпала
	{
		return REQ_MASTER_CRC_ERROR; // ошибка контрольной суммы
	}
	//--------------------------------------------
	// ПРОВЕРЯЮ НА ОТВЕТ-ИСКЛЮЧЕНИЕ
	if(NumberRead==SIZE_EXCEPTION){
		return ps_modbus_res->un_excep_num.type_exception; // ответ SLAVE - исключение
	}
	return REQ_MASTER_OK; // ответ на запрос верный
}



void t_Modbus_MASTER(void *p_task_par) {
	u8 k1;
	u16 req_status, port_status;
	u8 number_port, number_req;
	//u16 check_no_answer = 0;
	u8 buf_in[MODBUS_SIZE_BUFF_IN]; // Глобальный буффер приема
	S_Task_parameters *ptaskparameters;
	ptaskparameters = ((S_modbus_tsk_par*)p_task_par)->ps_task_parameters;
	number_port =  ((S_modbus_tsk_par*)p_task_par)->num_modbus;
	// основной цикл задачи
	while (1) {

		for (number_req = 0;number_req< ps_connectmodbus_global[number_port].number_of_pribor;number_req++) //циклический опрос запросами порта
		{
			// получаю статус запроса
			processing_mem_map_read_s_proces_object_modbus(&req_status, 1,s_list_madbus_req[number_port].s_ulist_req_modbus[number_req].ps_ulist_modbus->adres_status);

			if (req_status && (s_list_madbus_req[number_port].s_ulist_req_modbus[number_req].error_req_counter<TIME_BETWEEN_ERROR_REQ_MASTER)) { // если по данному запросу выставлен исключающий статус - запрос не передавать
				s_list_madbus_req[number_port].s_ulist_req_modbus[number_req].error_req_counter++; // увеличиваю счетчик к-ва пропусков
				continue;
			};
			s_list_madbus_req[number_port].s_ulist_req_modbus[number_req].error_req_counter=0;

			Clrinbuf(ptaskparameters, 200); // очищаю входной буфер ЮЗАРТа (Архитектура - ОДИН ПОРТА -> ОДНА ЗАДАЧА), перейти к (ОДИН ПРОТОКОЛ -> ОДНА ЗАДАЧА)

			// передаю запрос и анализирую ответ
			// Результат запроса записываю в структуру настроек запроса
			s_list_madbus_req[number_port].s_ulist_req_modbus[number_req].rez_zapros = ModMasterResTrs(ptaskparameters, &buf_in[0],	P_SHIFT(ps_connectmodbus_global,number_port),
							&s_list_madbus_req[number_port].s_ulist_req_modbus[number_req]); // формируем запрос, передаю, жду и получаю ответ

			Put_MODBUS(&buf_in[0], P_SHIFT(ps_connectmodbus_global,number_port),&s_list_madbus_req[number_port].s_ulist_req_modbus[number_req],req_status);

		}
		// анализирую статусы всех запросов и обновляю статус порта (когда заканчиваеться цикл опроса всех устройств на линии)
		port_status = 0xffff;
		for (k1 = 0; k1 < ps_connectmodbus_global[number_port].number_of_pribor; k1++) // опрашиваю статусы всех запросов порта
		{
			processing_mem_map_read_s_proces_object_modbus(&req_status, 1,s_list_madbus_req[number_port].s_ulist_req_modbus[k1].ps_ulist_modbus->adres_status);
			port_status = (req_status & port_status);
		}

		// если все устройсва на данной линии не отвечают, выставляю статус порта
		if (port_status) {
			processing_mem_map_write_s_proces_object_modbus(&port_status, 1,s_address_oper_data.s_modbus_address.status_PORT + number_port);
		//выполнить запись в общий статус-регистр состояния устройства
			processing_reset_control_set_dev_error(number_port,SET);
		}
		else{
			processing_mem_map_write_s_proces_object_modbus(&ZERO, 1,s_address_oper_data.s_modbus_address.status_PORT + number_port);
			//выполнить запись в общий статус-регистр состояния устройства
			processing_reset_control_set_dev_error(number_port,RESET);
		}
	}
}

//----------------------------------------------------------------------------
//----------------------START MODBUS SLAVE------------------------------------
//----------------------------------------------------------------------------
//---------функции MODBUS
// - ф. 1 - чтение состояния обмоток выходов
// - ф. 2 - чтение Состояния дискретных входов
// - ф. 3 - Чтение Регистров Хранения данных (конфигурация)
// - ф. 4 - Чтение Входных Регистров данных
// - ф. 5 - Установка Отдельной Обмотки
// - ф. 6 - Запись Отдельного Регистра
// - ф. 15 - Группы Обмоток
// - ф. 16 - Запись Группы Регистров

//---------------функция func_0102---------------
// функция func_0102 - формирует ответ на запрос по функции 1 и 2, копируя запрашиваимые элементы из карты памяти в бинарном формате
// входные аргументы:
// S_modbus_req *ps_modbus_req  - указатель на строку с запросом от мастера
// S_modbus_make_res *buf_respons - указатель на строку с сформированым ответом
// выходные аргументы:
// u16 - к-во байт ответа (размер ответа)
u16 func_0102(void *req, S_modbus_make_res *buf_respons) {
	S_modbus_req_read *ps_modbus_req = req;
	u8 *p_buf_rez_req = &buf_respons->resp_payload;
	u8 k1;
	u8 coun_TS;
	u8 numTS, num_byte_TS;
	u16 val_TS, address_TS;

	address_TS = CharTooshort(ps_modbus_req->address_mem_L,
			ps_modbus_req->address_mem_H); // адрес
	numTS = CharTooshort(ps_modbus_req->num_reg_L, ps_modbus_req->num_reg_H); // к-во
	buf_respons->address_dev = ps_modbus_req->address_dev; // формируем ответ - адрес КП
	buf_respons->function = ps_modbus_req->function; // формируем ответ - адрес функция

	// рассчитывааю к-во байт которые занимает ТС
	num_byte_TS = numTS / 8;
	if ((num_byte_TS * 8) != numTS) {
		num_byte_TS++;
	}
	//обнуляю байты ТС в ответе
	for (k1 = 0; k1 < num_byte_TS; k1++) {
		*(p_buf_rez_req + k1) = 0;
	}
	// заполняю посылку значениями ТС
	p_buf_rez_req = &buf_respons->resp_payload;
	*(p_buf_rez_req) = 0;
	for (coun_TS = 0; coun_TS < numTS; coun_TS++) {
		if (processing_mem_map_read_s_proces_object_modbus(&val_TS, 1,address_TS) != MEM_OK)
		{// если обратились к несуществующему адресу - исключительная ситуация
			break;
		}
		if (val_TS >= 1) { // ЕСЛИ  регистре значения больше >0 - возвращаем 1, если 0 - возвращаем 0
			*(p_buf_rez_req + (u8) (coun_TS / 8)) |= (1
					<< (coun_TS - (u8) (coun_TS / 8) * 8));
		}
		address_TS++;
	}
	// биты скопированы
	buf_respons->un_excep_num.num_byte_res = num_byte_TS;
	modus_add_CRC((u8*) buf_respons, num_byte_TS + 5);
	return buf_respons->un_excep_num.num_byte_res + 5;
}

//---------------функция func_0304---------------
// функция func_0304 - формирует ответ на запрос по функции 3 и 4, копируя запрашиваимые элементы из карты памяти
// входные аргументы:
// S_modbus_req *ps_modbus_req  - указатель на строку с запросом от мастера
// S_modbus_make_res *buf_respons - указатель на строку с сформированым ответом
// выходные аргументы:
// u16 - к-во байт ответа (размер ответа)
u16 func_0304(void *req, S_modbus_make_res *p_buf_respons) {
	S_modbus_req_read *ps_modbus_req = req;
	u16 num_reg_TI;
	u16 address_TI;
	u8 counter;
	u16 *p_payload;

	address_TI = CharTooshort(ps_modbus_req->address_mem_L,	ps_modbus_req->address_mem_H); // адрес
	num_reg_TI = CharTooshort(ps_modbus_req->num_reg_L,	ps_modbus_req->num_reg_H); // к-во регистров ТИ
	p_buf_respons->address_dev = ps_modbus_req->address_dev; // адрес КП
	p_buf_respons->function = ps_modbus_req->function; // функция
	// Копирую данные из карты памяти
	processing_mem_map_read_s_proces_object_modbus((u16*) &p_buf_respons->resp_payload, num_reg_TI, address_TI);
	// меняю местами байты: Hi->Lo, Lo-Hi
	p_payload=(u16*)&p_buf_respons->resp_payload;
	for(counter=0;counter<num_reg_TI;counter++){
		Convert_ShortTooCharHiLo((*p_payload),(u8*)p_payload);
		p_payload++;
	}

	// заполняю поле "к-во" в ответе
	p_buf_respons->un_excep_num.num_byte_res = (num_reg_TI * 2);
	modus_add_CRC((u8*) p_buf_respons,
			p_buf_respons->un_excep_num.num_byte_res + 5);
	return (p_buf_respons->un_excep_num.num_byte_res + 5);
}

//---------------функция func_05---------------
// функция func_05 - установка указанной обмотки
// входные аргументы:
// S_modbus_req *ps_modbus_req  - указатель на строку с заданым ТУ для установки
// S_modbus_make_res *buf_respons - указатель на строку с результатом выполнения ТУ
// выходные аргументы:
// u16 - к-во байт ответа (размер ответа)
u16 func_05(void *req, S_modbus_make_res *buf_respons) {
	S_modbus_req_write *ps_modbus_req = req;
	u16 payload_modbus_req[2];
	//u32 k1=0;
	// если задана дополнительная функция - обработчик запроса
	if (s_modbus_add_procesing_req[FORCE_SINGLE_COIL - 1].flag_req_procesing) {
		// форматирую полезную нагрузку
		payload_modbus_req[0] = CharTooshort(ps_modbus_req->address_mem_L,
				ps_modbus_req->address_mem_H);
		payload_modbus_req[1] = CharTooshort(ps_modbus_req->forse_data_L,
				ps_modbus_req->forse_data_H);
		// выполняю доп. обработку
		s_modbus_add_procesing_req[FORCE_SINGLE_COIL - 1].f_add_req_procesing(
				(void*) &payload_modbus_req[0],2,\
				CharTooshort(ps_modbus_req->address_mem_H,ps_modbus_req->address_mem_L)); // выполнить функцию обработчик
	}
	else{
		// записать в карту памяти pay_load приннятой посылки  !!!!!!!!!!!!!! ДОБАВИТЬ !!!!!!!!!!!!

	}

	// отвeт совпадает из запросом, копирую запрос в облать ответа
	memcpy(buf_respons, ps_modbus_req, sizeof(S_modbus_req_write));
	return sizeof(S_modbus_req_write);
}

//---------------функция func_06---------------
// функция func_06 - установка одного регистра
// входные аргументы:
// S_modbus_req *ps_modbus_req  - указатель на строку с данными для записи
// S_modbus_make_res *buf_respons - указатель на строку с результатом записи
// выходные аргументы:
// u16 - к-во байт ответа (размер ответа)
u16 func_06(void *p_req, S_modbus_make_res *p_buf_respons) {
	S_modbus_req_input *ps_req=(S_modbus_req_input*)p_req;
	u16 temp_store;
	// нужно привести данные запроса к нормальному виду: младший бай, старший байт
	Convert_CharTooShortHiLo((u16*)&(ps_req->number_reg_H),&temp_store);

	// копирую принятые данные в карту памяти
	processing_mem_map_write_s_proces_object_modbus( &temp_store,
			1,
			CharTooshort(ps_req->address_mem_H,ps_req->address_mem_L)
			);

	// если задана дополнительная функция - обработчик запроса
	if (s_modbus_add_procesing_req[ps_req->function - 1].flag_req_procesing) {
		// выполняю доп. обработку
		s_modbus_add_procesing_req[ps_req->function - 1].f_add_req_procesing(
				(void*) &(ps_req->req_field_var.s_req_f16.payload),\
				CharTooshort(ps_req->number_reg_H,ps_req->number_reg_L),\
				/*ps_req->req_field_var.s_req_f16.byte_count,\*/
				CharTooshort(ps_req->address_mem_H,ps_req->address_mem_L));
	}

	// формирую отвeт
	memcpy(p_buf_respons, (u8*) p_req, sizeof(S_modbus_req_write));
	//добавляю контрольную сумму
	//modus_add_CRC((u8*) p_buf_respons, sizeof(S_modbus_req_write));
	return sizeof(S_modbus_req_write);
}



//---------------функция func_16---------------
// функция func_16 - установка групы регистров
// входные аргументы:
// S_modbus_req *ps_modbus_req  - указатель на строку с данными для записи
// S_modbus_make_res *buf_respons - указатель на строку с результатом записи
// выходные аргументы:
// u16 - к-во байт ответа (размер ответа)
u16 func_16(void *p_req, S_modbus_make_res *p_buf_respons) {
	S_modbus_req_input *ps_req=(S_modbus_req_input*)p_req;
	u8 *p_peyload = (u8*)&(ps_req->req_field_var.s_req_f16.payload);
	u8 temp_store, k1;
	// нужно привести данные запроса к нормальному виду: младший бай, старший байт
	for (k1 = 0; k1 < ps_req->number_reg_L; k1++) {
		temp_store=*p_peyload;
		*p_peyload=*(p_peyload+1);
		p_peyload++;
		*p_peyload=temp_store;
		p_peyload++;
	}
	// копирую принятые данные в карту памяти
	processing_mem_map_write_s_proces_object_modbus( (u16*)&ps_req->req_field_var.s_req_f16.payload,ps_req->number_reg_L,
			CharTooshort(ps_req->address_mem_H,ps_req->address_mem_L)
			);

	// если задана дополнительная функция - обработчик запроса
	if (s_modbus_add_procesing_req[ps_req->function - 1].flag_req_procesing)
	{
		// выполняю доп. обработку
		s_modbus_add_procesing_req[ps_req->function - 1].f_add_req_procesing((void*) &(ps_req->req_field_var.s_req_f16.payload),\
				CharTooshort(ps_req->number_reg_H,ps_req->number_reg_L),\
				/*ps_req->req_field_var.s_req_f16.byte_count,\*/
				CharTooshort(ps_req->address_mem_H,ps_req->address_mem_L));
	}

	// формирую отвeт
	memcpy(p_buf_respons, (u8*) p_req, sizeof(S_modbus_req_write) - 2); //-2 байты контрольной суммы
	//добавляю контрольную сумму
	modus_add_CRC((u8*) p_buf_respons, sizeof(S_modbus_req_write));
	return sizeof(S_modbus_req_write);
}

void t_Modbus_SLAVE(void *p_task_par) {
	S_modbus_req_input *ps_modbus_req_input; // указатель на начало буфера приема

	u16 num_TSTI;
	u8 buf_out_slave[MODBUS_SIZE_BUFF_OUT]; // Глобальный буффер передачи
	u8 buf_in_slave[MODBUS_SIZE_BUFF_IN]; // Глобальный буффер приема
	REZ_REQ_CHEACK_SLAVE rez_chack_req;

    u8 address_KP = P_SHIFT(ps_connectmodbus_global,((S_modbus_tsk_par*)p_task_par)->num_modbus)->adress_kp;
	S_Task_parameters *ptaskparameters= ((S_modbus_tsk_par*)p_task_par)->ps_task_parameters;;

	ps_modbus_req_input = (S_modbus_req_input*) &buf_in_slave[0];


	while (ReadUSART(ptaskparameters->RdUSART, (u8*) ps_modbus_req_input, 1, 1)) {	} // очищаю буффер приема

	while (1) {

		if (ReadUSART(ptaskparameters->RdUSART, (u8*) ps_modbus_req_input, 8,20) != 8) { //считываю из стека USART. Если размер прочитаной посылки не 8 байт, повторяю чтение
			vTaskDelay(50);
			continue;
		}

		// выполняю проверку
		rez_chack_req = cheack_req_modbus(ptaskparameters, ps_modbus_req_input,address_KP); //
		// анализирую результаты проверки
		if (rez_chack_req == REQ_SLAVE_ERROR) { // ошибка полей или контрольной суммы
			continue;
		} else if (rez_chack_req) { // ошибка из кодов исключения
			num_TSTI = modbus_exception(ps_modbus_req_input,(S_modbus_make_res*) buf_out_slave, rez_chack_req);
		} else { // запрос коректный, вызываю ф-ю обработки команды
			if((ps_modbus_req_input->function - 1)==16){
				ps_modbus_req_input->function =ps_modbus_req_input->function;
			}
			num_TSTI = p_procesing_slave[ps_modbus_req_input->function - 1]((void*) ps_modbus_req_input,(S_modbus_make_res*) buf_out_slave);
		}

		write_Usart_Buf(ptaskparameters->USARTx, buf_out_slave, num_TSTI,
				ptaskparameters->WrUSART); // передача ответа или исключения

		vTaskDelay(50);
	}
}

// ----------------функция cheack_req_modbus------------------------
// функция cheack_req_modbus выполняет проверку принятого запроса по следующим пунктам:
// - адрес КП
// - допустимые значения поля функции
// - допустимый диапазон адресов регистров для заданой ф-и через функцию callback пользователя
// - соответствие контрольной суммы
REZ_REQ_CHEACK_SLAVE cheack_req_modbus(S_Task_parameters *p_iparameters,
		S_modbus_req_input *ps_modbus_req_cheack, u8 address_kp) {
	u8 rez_addition_chek;
	u16 rez_CRC16, *p_req_CRC16;
	u16 payload_modbus_req[2]; //адрес начала запрашиваимых регистров - payload_modbus_req[0],
	                           //количество запрашиваимых регистров   - payload_modbus_req[1],
	u16 byte_count; // к-во сбайт полезнеой нагрузки

	//   ----ТИПОВАЯ ЧАСТЬ ПРОВЕРКИ (ДЛЯ ВСЕХ ТИПОВ ЗАПРОСОВ)-------------
	// проверка на допустимые функции запроса
	if ((ps_modbus_req_cheack->function != READ_COIL_STATUS)
			& (ps_modbus_req_cheack->function != READ_INPUT_STATUS)
			& (ps_modbus_req_cheack->function != READ_HOLDING_STATUS)
			& (ps_modbus_req_cheack->function != READ_INPUT_REGISTERS)
			& (ps_modbus_req_cheack->function != FORCE_SINGLE_COIL)
			& (ps_modbus_req_cheack->function != FORSE_MULTIPLE_COILS)
			& (ps_modbus_req_cheack->function != PRESET_SINGLE_REGISTERS)
			& (ps_modbus_req_cheack->function != PRESET_MULTIPLE_REGISTERS)) {
		return REQ_SLAVE_ERROR;
	}
	// проверяю адрес кому адресовано сообщение
	if (ps_modbus_req_cheack->address_dev != address_kp) {
		return REQ_SLAVE_ERROR;
	}
	//Два варианта: или было принято начало посылки №15-16 или другие типы
	if ((ps_modbus_req_cheack->function == FORSE_MULTIPLE_COILS) // если принято начало посылки 15 или 16
			|| (ps_modbus_req_cheack->function == PRESET_MULTIPLE_REGISTERS)) {
		// к-во байт к риему
		byte_count = CharTooshort(ps_modbus_req_cheack->number_reg_H,ps_modbus_req_cheack->number_reg_L) * 2;
		// Парсим принятую посылку
		if ((u16) (ps_modbus_req_cheack->req_field_var.s_req_f16.byte_count)!= byte_count) {
			return REQ_SLAVE_ERROR;
		}
		// принимаю остальную часть посылки
		if (ReadUSART(p_iparameters->RdUSART, (u8*) (ps_modbus_req_cheack + 1),	byte_count - 1 + 2, 20) != (byte_count - 1 + 2)) { // -1: один байт полезной нагрузки уже считан. +2: два байта контрольной суммы
			return REQ_SLAVE_ERROR;
		};
		byte_count = byte_count + 7; // длина запроса без контрольной суммы
	} else {
		byte_count = 8 - 2; // длина запроса без контрольной суммы
	}

	// рассчитываю контрольную сумму приннятого запроса
	rez_CRC16 = CRC16((u8*) ps_modbus_req_cheack, byte_count); // +7 это заголовок команды запроса №16
	Convert_ShortTooCharHiLo(rez_CRC16, (u8*)&rez_CRC16);
	// меняю местами байты в CRC чтобы были как в запросе
	p_req_CRC16 = (u16*) ((u8*) ps_modbus_req_cheack + byte_count); // беру указатель на приннятую контрольную сумму
	// проверка контрольной суммы посылки
	if ((*p_req_CRC16) != rez_CRC16) {
		return REQ_SLAVE_ERROR;
	}
    // ---------ЗАКЛЮЧЕНИЕ: посылка адресована даному устройству, формат посылки выдержан и без ошибок, проверка полезной нагрузки

    //*********************************************************************************************************
	//  -----------------------ПРОВЕРЯЕМ КОРЕКТНОСТЬ АДРЕСАЦИИ ПРИННЯТОЙ КОМАНДЫ------------------------------*
	//*********************************************************************************************************

	// выбираю из запроса адрес и к-во запрашиваимых регистров
	payload_modbus_req[0] = CharTooshort(ps_modbus_req_cheack->address_mem_H,
			ps_modbus_req_cheack->address_mem_L);
	payload_modbus_req[1] = CharTooshort(ps_modbus_req_cheack->number_reg_H,
			ps_modbus_req_cheack->number_reg_L);

	// проверка принадлежности адреса запрашиваимых регистров команды области допустимых адресов согласно номеру команды
	// через функцию callback пользователя
	if (!s_modbus_check_address_req[ps_modbus_req_cheack->function - 1].flag_callback_check) { // если для запрашиваимой команды функция не задана
		return ILLEGAL_DATA_ADRESS;                                                              // значит запрос по данной команде не допустим
	}
	else{
		if( s_modbus_check_address_req[ps_modbus_req_cheack->function - 1].pf_callback_check(&payload_modbus_req[0])){
			return ILLEGAL_DATA_ADRESS;
		};
	}

	//*********************************************************************************************************
	//  -----------------------ИНДИИДУАЛЬНЯ ЧАСТЬ ПРОВЕРКИ----------------------------------------------------*
	//*********************************************************************************************************

	// если НЕ_НУЖНО выполнять проверку работоспособности для выдачи ответа по заданой ф-и
	if (!s_modbus_check_add_check[ps_modbus_req_cheack->function - 1].flag_callback_check) {
		return REQ_SLAVE_OK;
	}
	// если НУЖНО выполнить ДОПОЛНИТЕЛЬНУЮ проверкудля выдачи ответа по заданой ф-и (для разных запросов аргументы имеют разный смысл)
	rez_addition_chek = s_modbus_check_add_check[ps_modbus_req_cheack->function - 1].pf_callback_check(&payload_modbus_req[0]);

	// если функция дополнительной проверки возвратила недопустимый аргумент
	if ((rez_addition_chek != REQ_SLAVE_OK)
			&& (rez_addition_chek != REQ_SLAVE_ERROR)
			&& (rez_addition_chek != ILLEGAL_FUNCTION)
			&& (rez_addition_chek != ILLEGAL_DATA_ADRESS)
			&& (rez_addition_chek != ILLEGAL_DATA_VALUE)
			&& (rez_addition_chek != SLAVE_DEVICE_FALIURE)
			&& (rez_addition_chek != ACKNOWLEDGE)
			&& (rez_addition_chek != SLAVE_DEVICE_BUSY)
			&& (rez_addition_chek != NEGATIVE_ACKNOWLEDGE)
			&& (rez_addition_chek != MEMORY_PARITY_ERROR)) {
		return REQ_SLAVE_ERROR;
	}
	//----------------------------------------------------------------------------------------------------------------------


	// возвращаю код исключения
	if (rez_addition_chek) {
		return rez_addition_chek;
	}
	//выход
	return REQ_SLAVE_OK;
}

// ----------------функция modbus_exception------------------------
// функция modbus_exception формирует ответ-исключение
u8 modbus_exception(S_modbus_req_input *ps_modbus_req,
		S_modbus_make_res *buf_respons, u8 exeption) {
	buf_respons->address_dev = ps_modbus_req->address_dev;
	buf_respons->function = ((0b10000000) | (ps_modbus_req->function)); // устанавливаю "1" в старший бит типа ф-и, указывая на передачу исключения
	buf_respons->un_excep_num.type_exception = exeption;
	modus_add_CRC((u8*) buf_respons, SIZE_EXCEPTION);
	return SIZE_EXCEPTION;
}

// ----------------функция modbus_callback_add_check------------------------
// функция modbus_callback_add_check - задает ф-ю для дополнительной проверки параметров запросу
// входные аргументы:
// p_modbus_callback_add_check - указательна на функцию дополнительной обработки
// number_req - типа запроса (номер)
void modbus_callback_add_check(PF_modbus_callback_check p_modbus_callback_add_check, u8 number_req) {
	s_modbus_check_add_check[number_req - 1].pf_callback_check =
			p_modbus_callback_add_check;
	s_modbus_check_add_check[number_req - 1].flag_callback_check = 1;
}


// ----------------функция modbus_callback_address_check------------------------
// функция modbus_callback_address_check - задает ф-ю для для проверки адреса запроса
// входные аргументы:
// pf_modbus_callback_address_check - указательна на функцию дополнительной обработки
// number_req - типа запроса (номер)
void modbus_callback_address_check(PF_modbus_callback_check pf_modbus_callback_address_check, u8 number_req) {
	s_modbus_check_address_req[number_req - 1].pf_callback_check =
			pf_modbus_callback_address_check;
	s_modbus_check_address_req[number_req - 1].flag_callback_check = 1;
}


// ----------------функция modbus_add_processing_callback------------------------
// функция modbus_add_processing_callback - задает дополнительную ф-ю обработки запроса
// входные аргументы:
// p_modbus_callback_req - указательна на функцию дополнительной обработки
// number_req - типа запроса (номер)
void  modbus_callback_add_processing(PF_add_processing_req_slave p_modbus_callback_req, u8 number_req) {
	s_modbus_add_procesing_req[number_req - 1].f_add_req_procesing =
			p_modbus_callback_req;
	s_modbus_add_procesing_req[number_req - 1].flag_req_procesing = 1;
}

