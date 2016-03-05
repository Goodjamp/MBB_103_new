/*
 * processing_config_dev.c
 *
 *  Created on: April 25, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "processing_config_dev.h"

//структура адресов оперативных регистров
extern S_address_oper_data s_address_oper_data;
// глобальная структура настроек устройства (дублировання из карты памяти, не изменяеться в процесе работы устройства
// до перезагрузки, если была задана новая конфигурация)), из main
extern S_config_moduls s_config_moduls;
//структура описания карты памяти (из  mem_map_processing.c)
extern S_mem_map s_mem_map;

//-------------processing_config_firest_on------------------
// функция processing_config_firest_on - если конфигурация по умолчанию не записана - записать
//                                       если конфигурация пользователя не записана - записать конфигурацию по умолчанию
// При первом включении выполняеться запись конфигураций пользователя и по умолчанию
void processing_config_firest_on(void){
	S_global_config s_mem_data_set;
	u16 CRC_dev;
	// ----------------------КОНФИГУРАЦИЯ "ПО УМОЛЧАНИЮ"-------------------------
	INIT_MBB_read_addjust_table(s_mem_map.p_start_config_data,sizeof(S_dev_staff), PAGE_DEFAULT_CONFIG);
    // расчитываю контрольную сумму считанных данных и сравниваю с считанной контрольной суммой
	CRC_dev=CRC16((u8*)&(((S_global_config*)s_mem_map.p_start_config_data)->s_dev_staff),(sizeof(S_dev_staff)-2));
	if(CRC_dev !=((S_global_config*)s_mem_map.p_start_config_data)->s_dev_staff.CRC_dev)
	{
		default_config_table(&s_mem_data_set);  // записываю конфигурацию по умолчанию
		// очистка конфигурационной области флеш
		FLASH_OPERATION_erase_page(PAGE_DEFAULT_CONFIG);
		FLASH_OPERATION_write_flash_16b((u16*) &s_mem_data_set,(sizeof(s_mem_data_set)+1) / 2 , PAGE(PAGE_DEFAULT_CONFIG ));
	}
	// ----------------------КОНФИГУРАЦИЯ "ПОЛЬЗОВАТЕЛЯ"-------------------------
	INIT_MBB_read_addjust_table(s_mem_map.p_start_config_data,sizeof(S_dev_staff), PAGE_USER_CONFIG);
	// расчитываю контрольную сумму считанных данных и сравниваю с считанной контрольной суммой
	CRC_dev=CRC16((u8*)&(((S_global_config*)s_mem_map.p_start_config_data)->s_dev_staff),(sizeof(S_dev_staff)-2));
	if(CRC_dev != ((S_global_config*)s_mem_map.p_start_config_data)->s_dev_staff.CRC_dev)
	{
		default_config_table(&s_mem_data_set);  // записываю конфигурацию по умолчанию
		// очистка конфигурационной области флеш
		FLASH_OPERATION_erase_page(PAGE_USER_CONFIG);
		FLASH_OPERATION_write_flash_16b((u16*) &s_mem_data_set,(sizeof(s_mem_data_set)+1) / 2 , PAGE(PAGE_USER_CONFIG ));

	}
}


//-------------processing_config_init_not_user_comfig------------------
//функция processing_config_init_not_user_comfig - заполняет поля "не пользовательских" настроек, а именно:
//                                               - состав устройства
//                                               - к-во портов MODBUS
//                                               - регистров пользователя
//                                               - максимальное к-во запросов MODBUS MASTER для одного порта
//                                               - версия внутренего ПО
//                                               - CRC - идентефикатор устройства
void processing_config_init_not_user_config(S_dev_staff *ps_dev_staff){
	// заполняю поля конфигурации устройства
	ps_dev_staff->bf_dev_staff=(BF_dev_staff){1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
	// Состав устройства
#ifdef DEV_0
	ps_dev_staff->bf_dev_staff.MODULE_0=ENABLE;
#endif
#ifdef DEV_1
	ps_dev_staff->bf_dev_staff.MODULE_1=ENABLE;
#endif
#ifdef DEV_2
	ps_dev_staff->bf_dev_staff.MODULE_2=ENABLE;
#endif
#ifdef DEV_3
	ps_dev_staff->bf_dev_staff.MODULE_3=ENABLE;
#endif
#ifdef DEV_4
	ps_dev_staff->bf_dev_staff.MODULE_4=ENABLE;
#endif
#ifdef DEV_5
	ps_dev_staff->bf_dev_staff.MODULE_5=ENABLE;
#endif
#ifdef DEV_6
	ps_dev_staff->bf_dev_staff.MODULE_6=ENABLE;
#endif
#ifdef DEV_7
	ps_dev_staff->bf_dev_staff.MODULE_7=ENABLE;
#endif
#ifdef DEV_8
	ps_dev_staff->bf_dev_staff.MODULE_8=ENABLE;
#endif
#ifdef DEV_9
	ps_dev_staff->bf_dev_staff.MODULE_9=ENABLE;
#endif
#ifdef DEV_10
	ps_dev_staff->bf_dev_staff.MODULE_10=ENABLE;
#endif
#ifdef DEV_11
	ps_dev_staff->bf_dev_staff.MODULE_11=ENABLE;
#endif
#ifdef DEV_12
	ps_dev_staff->bf_dev_staff.MODULE_12=ENABLE;
#endif
#ifdef DEV_13
	ps_dev_staff->bf_dev_staff.MODULE_13=ENABLE;
#endif
#ifdef DEV_14
	ps_dev_staff->bf_dev_staff.MODULE_14=ENABLE;
#endif
#ifdef DEV_15
	ps_dev_staff->bf_dev_staff.MODULE_15=ENABLE;
#endif

     // заполняю поля состава и свойства устройства. По идее эти значения должны храниться во флеш - памяти, и считываться вместе с конфигурацией,
	 // но как сделать удобный интерфейс записи во флеш настроек, которые не меняються на протяжении всей жизни прибора (состав и т. д.) еще не придумал

	ps_dev_staff->num_use_uart=NUM_PORT_MODBUS;//
	ps_dev_staff->num_reg_data=NUM_REG_DATA;
	ps_dev_staff->modbus_req_user=MAX_NUM_MODBUS_REQ;
	ps_dev_staff->program_version=1;
	ps_dev_staff->CRC_dev=CRC16((u8*)ps_dev_staff,(sizeof(S_dev_staff)-2));
}



//-------------processing_config_dev_init------------------
//функция processing_config_dev_init - выполняет заполнение битового поля состава устройства согласно настройкам пользователя
//                                     копирует данные конфигурации в рабочую структуру для доступа програмных модулей,
//                                     задает callback приема функции 16 modbus
void processing_config_dev_init(void){
	// копирую настройки МОДУЛЕЙ устройства из карты памяти, в структуру настроек, которая будет использоваться програмными модулями.
	// даная структура не изменяеться в процесе работы устройства (до презагрузки).
	// НАСТРОЙКИ ПОЛЬЗОВАТЕЛЯ (НОВЫЕ) В ПРОЦЕСЕ КОНФИГУРАЦИИ СОХРАНЯЮТЬСЯ В КАРТЕ ПАМЯТИ !!!
	// (u8*)&s_config_moduls - глобальная структура настроек МОДУЛЕЙ устройства, которая используеться програмными модулями в процесе работы програмы
	//(S_global_config*)s_mem_map.p_start_config_data ->s_config_moduls - структура настроек устройства в карте памяти, не используеться модуляями
	//                                                                    в процесе работы, но при обращении к карте памяти передаються/записываються
	//                                                                    данные именно в эту область
	memcpy((u8*)&s_config_moduls,
			(u8*)&((S_global_config*)s_mem_map.p_start_config_data)->s_config_moduls,
			sizeof(S_config_moduls)
			);

	// задаю ф-ю callback которая вызываться процессом modbus_slave для проверки запрашиваимого адресса запросом №3
	modbus_callback_address_check(&processing_config_check_is_holding_reg,READ_HOLDING_STATUS);
	// задаю ф-ю callback которая вызываться процессом modbus_slave для проверки запрашиваимого адресса запросом №4
	modbus_callback_address_check(&processing_config_check_is_input_reg,READ_INPUT_REGISTERS);
	// задаю ф-ю callback которая вызываться процессом modbus_slave для проверки запрашиваимого адресса запросом №16
	modbus_callback_address_check(&processing_config_check_is_preset_multiple_reg,PRESET_MULTIPLE_REGISTERS);
	// задаю ф-ю callback которая вызываться процессом modbus_slave после приема сообщения  №16
	modbus_callback_add_processing(&update_config_data,PRESET_MULTIPLE_REGISTERS);
	// Понижаю частоту шины APB2 для коректной работы SPI1 модуля ТС
	RCC_ClocksTypeDef rcc_clock;
	RCC_GetClocksFreq(&rcc_clock);
	RCC_PCLK2Config(RCC_HCLK_Div2);
	RCC_GetClocksFreq(&rcc_clock);
}


//------------функция processing_config_is_holding_reg-----------------------------------
// функция processing_config_is_holding_reg - выполняет проверку адресса карты памяти на принадлежность регистрам конфигурации устройства
// даная функция соответствует проверке команде №3 modbus
// входные аргументы:
// *p_check_address - указатель на начало последовательности (u16)address, (u16)num_reg
// выходные аргументы:
// REQ_SLAVE_OK - проверка выполнена успешно
// ILLEGAL_DATA_ADRESS - недопустимый адрес
REZ_REQ_CHEACK_SLAVE processing_config_check_is_holding_reg(void* p_check_address){
	u16 address= *(u16*)p_check_address;
	u16 num_reg= *(u16*)((u16*)p_check_address+1);
	// адрес принадлежит области оперативных регистров
	if((address>=USER_ADDRESS_OPER_DATA)&&((address+num_reg-1)<(USER_ADDRESS_OPER_DATA+NUM_REG_OPER_DATA)))
	{
		return REQ_SLAVE_OK;
	}
	// адрес принадлежит области настроек
	if((address>=USER_ADDRESS_CONFIG_DATA)&&((address+num_reg-1)<(USER_ADDRESS_CONFIG_DATA+NUM_REG_CONFIG)))
	{
		return REQ_SLAVE_OK;
	}
	// адрес находиться за границами региcтров настройки
	return ILLEGAL_DATA_ADRESS;
}




//------------функция processing_config_is_input_reg-----------------------------------
// функция processing_config_is_input_reg - выполняет проверку адресса карты памяти на принадлежность любому регистру устройства, за исключением рег. конфигурации
// даная функция соответствует проверке команде №4 modbus
// входные аргументы:
// *p_check_address - указатель на начало последовательности (u16)address, (u16)num_reg
// выходные аргументы:
// REQ_SLAVE_OK - проверкаа выполнена успешно
// ILLEGAL_DATA_ADRESS - недопустимый адрес
REZ_REQ_CHEACK_SLAVE processing_config_check_is_input_reg(void* p_check_address){
	u16 address= *(u16*)p_check_address;
	u16 num_reg= *(u16*)((u16*)p_check_address+1);
	// адрес принадлежит области регистров пользователя
	if((address>=USER_ADDRESS_DATA)&&
			((address+num_reg-1)<(USER_ADDRESS_DATA+NUM_REG_DATA)))
	{
		return REQ_SLAVE_OK;
	}
	// адрес находиться за границами допустимых адресов
	return ILLEGAL_DATA_ADRESS;
}


//------------функция processing_config_check_is_preset_multiple_register-----------------------------------
// функция processing_mem_map_is_input_reg - выполняет проверку адресса карты памяти на принадлежность регистрам
// даная функция соответствует проверке команде №16 modbus
// входные аргументы:
// *check_data - указатель на начало последовательности (u16)address
// выходные аргументы:
// REQ_SLAVE_OK - проверкаа выполнена успешно
// ILLEGAL_DATA_ADRESS - недопустимый адрес

REZ_REQ_CHEACK_SLAVE processing_config_check_is_preset_multiple_reg(void* p_check_address){
	u16 address_reg=(*((u16*)p_check_address)); // адрес начала запрашиваимых регистров
	u16 num_reg=(*((u16*)p_check_address++)); // количество запрашиваимых регистров

	return REQ_SLAVE_OK;
}


//-------------update_config_data------------------
//функция update_config_data - выполняет управление конфигурацией.
// входные аргументы:
// req               - указатель на информационную часть сообщения с правильным следованием байт
// num_peyload_data  -
// После приема всех конфигурационных сообщений, выполняеться проверка конфигурации
// и запись во FLESH.
// Процес приема конфигурации:
// 1 - жду приема сообщения с адресом равным адресу начала области данных конфигурации, устанавливаю флаг начала приема конфигурации
// 2 - после каждого приннятого пакета увеличиваю следующий адреса записи
// 3 - если адрес в следующем пакете не совпадает со счетчиком адресов (п.2) перехожу к п.1
// 4 - после приема всей конфигурационной информации, выполняю проверку и записываю к-цию во флеш
u8 update_config_data(void* req,u8 num_peyload_data, u16 addres_data){
	static u16 count_config_data=0;
	static u8 f_rx_config_data=0; // флаг начаоа приема к-ции
	// если принята случайная посылка
	if((f_rx_config_data==0)&(addres_data!=USER_CONFIG_START_ADRESS))
	{
		return 1;
	}
	//если принят первый пакет посылки конфигурации
	if(f_rx_config_data==0)
	{
		f_rx_config_data=1;
		count_config_data=num_peyload_data;
		return 0;
	}
	// если приннятый пакет данных конфигурации не соответствует проверочным точкам
	if((addres_data-USER_CONFIG_START_ADRESS)!=count_config_data)
	{
		f_rx_config_data=0;
		count_config_data=0;
		return 1;
	}
	// увеличиваю счетчик адреса следующей посылки
	count_config_data+=num_peyload_data;
	// если принята вся конфигурационная посылка - выполнить ПРОВЕРКУ и запись во FLESH
	if(count_config_data==(USER_CONFIG_DATA_SIZE))
	{
		vTaskSuspendAll(); // остановить планировщик
		count_config_data=0;
		f_rx_config_data=0; //
		FLASH_OPERATION_erase_page(PAGE_USER_CONFIG);
		FLASH_OPERATION_write_flash_16b((u16*)s_mem_map.p_start_config_data,sizeof(S_global_config) / 2 + 1, START_ADDRESS_DATA);
		xTaskResumeAll();
		// ---------здесь выполнить проверку приннятой посылки----------

		// можно так: вызов функций конфигурации из "main.c" сгрупировать в одну функцию перенести

	}

	if(count_config_data>NUM_REG_CONFIG)
	{
		return 1;
	}
		return 0;
}
