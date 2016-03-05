#include "processing_TC.h"
#include "processing_mem_map_extern.h"
#include "flash_operation.h"

xSemaphoreHandle SemaphoreHandle_spi;
portBASE_TYPE prioryty_take_semaphore;
// Общая структура настроек пользователя модуля ТY
S_TC_user_config *ps_TC_user_config;
// структрура содеожащая проверчный байт и состояние ТС
S_reaz_read_TC s_reaz_read_TC={0b10101010,0};
// Структура обработки состояний ТС
S_total_TC_processing s_TC_processing;
// структура адресов оперативных регистров (из mem_map_processing.c)
extern S_address_oper_data s_address_oper_data;


//----------функция processing_TC_init----------------
//функция processing_TC_init - конфигурация переферии которая используеться для
//                             считывания сигналов ТС
void processing_TC_init(void){
	// создаю семафор для прерывания tim3
	vSemaphoreCreateBinary(SemaphoreHandle_spi);
	// конфигурация структур данных сигналов ТС
	processing_TC_init_struct();
	// конфигурирую SPI для считывания сигналов ТС
	processing_TC_init_spi1();
	// инициилизация выходных портов
	processing_TC_init_gpio();
	// конффигурирую таймер для отсчета интервалов времени считвания сигналов ТС
	processing_TC_init_tim3();
	// задаю функцию callback проверки запрашиваимого адреса данных команды READ_INPUT_STATUS (№2) для Modbus
	modbus_callback_address_check(&processing_TC_check_is_input_status_modbus,READ_INPUT_STATUS);
	// выполнять проверку работоспособности блока ТУ при запросе статуса ТУ протоколом MODBUS
	modbus_callback_add_check(&processing_TC_signal_check, READ_INPUT_STATUS);

	//// 	ТОЛКЬО ДЛЯ ПРОВЕРКИ!!!!
#ifdef TC_SOFT_CHECK
	check();
#endif

}

#ifdef TC_SOFT_CHECK
// 	ТОЛКЬО ДЛЯ ПРОВЕРКИ!!!!
void check(void){
	GPIO_InitTypeDef GPIO_InitTypeDef;
	// настройка порта програмной проверки модуля ТС
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitTypeDef.GPIO_Pin=PIN_SOFT_CHECK_B2;
	GPIO_InitTypeDef.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(PORT_SOFT_CHECK_B2, &GPIO_InitTypeDef);
	GPIO_ResetBits(PORT_SOFT_CHECK_B2,PIN_SOFT_CHECK_B2);
	// настройка порта програмной проверки модуля ТС
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitTypeDef.GPIO_Pin=PIN_SOFT_CHECK_B12;
	GPIO_InitTypeDef.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(PORT_SOFT_CHECK_B12, &GPIO_InitTypeDef);
	GPIO_ResetBits(PORT_SOFT_CHECK_B12,PIN_SOFT_CHECK_B12);
}
#endif


//------------функция processing_TC_check_is_input_status_modbus-----------------------------------
// функция processing_TC_check_is_input_status_modbus - выполняет проверку адресса карты памяти на принадлежность регистрам состояния ТС
// даная функция соответствует проверке команде №2 modbus
// входные аргументы:
// *check_data - указатель на начало последовательности (u16)address, (u16)num_reg
// выходные аргументы:
// REQ_SLAVE_OK - проверкаа выполнена успешно
// ILLEGAL_DATA_ADRESS - недопустимый адрес
REZ_REQ_CHEACK_SLAVE processing_TC_check_is_input_status_modbus(void* p_check_address){
	u16 address= *(u16*)p_check_address;
	u16 num_reg= *(u16*)((u16*)p_check_address+1);
	// адрес принадлежит области регистров ТС
	if((address>=s_address_oper_data.s_TC_address.rez_TC)&&
			((address+num_reg-1)<(s_address_oper_data.s_TC_address.rez_TC+TOTAL_NUM_TC)))
	{
		return REQ_SLAVE_OK;
	}
	// адрес находиться за границами допустимых адресов регистров ТС
	return ILLEGAL_DATA_ADRESS;
}


INIT_MBB_Rezult processing_TC_fill_S_TS(u8 *read_data) {
	// копирую настройки TC
	ps_TC_user_config=(S_TC_user_config*)read_data;
	//memcpy(&ps_TC_user_config->read_data,sizeof(S_tc_user_config));
	return MBB_INIT_OK;
}


//----------функция processing_TC_init_struct----------------
//функция processing_TC_init_struct - конфигурация структур данных сигналов ТС
void processing_TC_init_struct(void){
	u8 k1;
	for(k1=0;k1<TOTAL_NUM_TC;k1++){
		s_TC_processing.as_one_TC_processing[k1].tc_counter=0;
		s_TC_processing.as_one_TC_processing[k1].tc_is_procssing=IS_NOT_PROCESSING;
		s_TC_processing.as_one_TC_processing[k1].tc_state=0;
	}
}


//----функция processing_TC_init_gpio--------------
//функция processing_TC_init_gpio - конффигурирую gpio для работы из сериалайзером
void processing_TC_init_gpio(void){
	GPIO_InitTypeDef GPIO_InitTypeDef_TC;
	// настройка SN66HVS882 LD
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef_TC.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitTypeDef_TC.GPIO_Pin=LD_PIN;
	GPIO_InitTypeDef_TC.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(LD_PORT, &GPIO_InitTypeDef_TC);
	// настройка SN66HVS882 TOK
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef_TC.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitTypeDef_TC.GPIO_Pin=TOK_PIN;
	GPIO_InitTypeDef_TC.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(TOK_PORT, &GPIO_InitTypeDef_TC);
	// настройка 24 v IN
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef_TC.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitTypeDef_TC.GPIO_Pin=V_PIN;
	GPIO_InitTypeDef_TC.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(V_PORT, &GPIO_InitTypeDef_TC);
}

//----функция processing_TC_tim3--------------
//функция processing_TC_tim3 - конффигурирую таймер для отсчета интервалов времени считвания сигналов ТС
void processing_TC_init_tim3(void){
	u32 tim3_com=0;
	TIM_TimeBaseInitTypeDef tim3_InitTypeDef;
	RCC_ClocksTypeDef rcc_ClocksTypeDef;
	// вычисляю частоты всех шин МК
	RCC_GetClocksFreq(&rcc_ClocksTypeDef);
	// запускаю тактирование TIM3, шина APB2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	TIM_DeInit(TIM3);
	// заполняю структуру конфигурации таймера
	rcc_ClocksTypeDef.PCLK1_Frequency/=1000;
	tim3_com=(rcc_ClocksTypeDef.PCLK1_Frequency*2/TIM_PRESCALER)*PERIOD_GET_TC-1;
	tim3_InitTypeDef.TIM_CounterMode=TIM_CounterMode_Up;
	tim3_InitTypeDef.TIM_Period=tim3_com;
	tim3_InitTypeDef.TIM_Prescaler=TIM_PRESCALER;
	tim3_InitTypeDef.TIM_ClockDivision=TIM_CKD_DIV1;
	tim3_InitTypeDef.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM3,&tim3_InitTypeDef);
	// прерывания по переполнению счетного регистра
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn,14);
	// включаю тактировани таймера
	TIM_Cmd(TIM3,ENABLE);
}


//-----------функция processing_TC_init_spi1---------------
//функция processing_TC_init_spi1 - конфигурирую SPI для считывания сигналов ТС
void processing_TC_init_spi1(void){
	GPIO_InitTypeDef GPIO_InitTypeDef_TC;
	SPI_InitTypeDef SPI_InitTypeDef_TC;


	// --------Настройка GPIO для работы с SN66HVS882-------
	// SCK   -  PA5  Alternate function push-pull
	// MISO  -  PA6  Input floating
	// MOSI  -  PA7  Alternate function push-pull
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	// настройка SPI SCK
	GPIO_InitTypeDef_TC.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitTypeDef_TC.GPIO_Pin=GPIO_Pin_5;
	GPIO_InitTypeDef_TC.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitTypeDef_TC);
	// настройка SPI MISO
	GPIO_InitTypeDef_TC.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitTypeDef_TC.GPIO_Pin=GPIO_Pin_6;
	GPIO_InitTypeDef_TC.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitTypeDef_TC);
	// настройка SPI MOSI
	GPIO_InitTypeDef_TC.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitTypeDef_TC.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitTypeDef_TC.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitTypeDef_TC);

	// ------------Настройка SPI для работы с SN66HVS882
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);

	SPI_InitTypeDef_TC.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_256;
	SPI_InitTypeDef_TC.SPI_CPHA=SPI_CPHA_1Edge;
	SPI_InitTypeDef_TC.SPI_CPOL=SPI_CPOL_Low;
	SPI_InitTypeDef_TC.SPI_DataSize=SPI_DataSize_8b;
	SPI_InitTypeDef_TC.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	SPI_InitTypeDef_TC.SPI_FirstBit=SPI_FirstBit_MSB;
	SPI_InitTypeDef_TC.SPI_Mode=SPI_Mode_Master;
	SPI_InitTypeDef_TC.SPI_NSS=SPI_NSS_Soft;
	// настройка
	SPI_Init(SPI1,&SPI_InitTypeDef_TC);
	// Включаю прерывание по приему байта
	SPI_I2S_ITConfig(SPI1,SPI_I2S_IT_RXNE,ENABLE);
	// включаю прерывание SPI
	NVIC_EnableIRQ(SPI1_IRQn);
	//Меняю приоритет прерывания SPI для возможности вызова из него функци FreeRTOS API
	NVIC_SetPriority(SPI1_IRQn,13);
	// включаю SPI
	SPI_Cmd(SPI1,ENABLE);
}



//--------------прерывание TIM3----------------
//прерывание TIM3 выполняеться периодически из заданым в PERIOD_GET_TC мс периодом.
// В даном прерывании инициируеться процес считывания данных из сериалайзера (захватываються данные состояния и запускаеться процес приема данных)
void TIM3_IRQHandler(void){
	u8 k1;
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	// генерирую импульс захвата ТС
	LD_CLEAR;
	while(k1<(30*4)){
		k1++;
	}
	LD_SET;
	while(k1<(30*4)){
		k1++;
	}
	// передаю проверочный байт, должен принять состояние ТС (в прерывании)
	SPI_I2S_SendData(SPI1,CHECK_SERIAL_BYTE); // передаю пустую посылку чтобы принять проверочный байт
	s_reaz_read_TC.data_TC=0;
	//Проверяю, какие входы ТС находяться в процесе рбработки (петле гистерезиса) и увеличиваю для них счетчик мс
	for(k1=0;k1<TOTAL_NUM_TC;k1++){
		// если сигнал ТС находится в петле гистерезиса
		if(s_TC_processing.as_one_TC_processing[k1].tc_is_procssing==IS_PROCESSING){
			s_TC_processing.as_one_TC_processing[k1].tc_counter++;
		}
	}
	/*
#ifdef TC_SOFT_CHECK
	if(GPIO_ReadOutputDataBit(PORT_SOFT_CHECK,PIN_SOFT_CHECK)){
		GPIO_ResetBits(PORT_SOFT_CHECK,PIN_SOFT_CHECK);
	}
	else{
		GPIO_SetBits(PORT_SOFT_CHECK,PIN_SOFT_CHECK);
	}
#endif
*/
}




//--------------прерывание SPI1----------------
//прерывание SPI1 выполняеться по приему байта от сериалайзера
void SPI1_IRQHandler(void){
	static u8 st_count_spi_int=0; // счетчик входов в прерывание
/*  u8 k1=0;
	GPIO_SetBits(GPIOB,GPIO_Pin_8);
	for(k1=0;k1<20;k1++);
	GPIO_ResetBits(GPIOB,GPIO_Pin_8);*/
	/*
	 *   - OVR (OverRun Error) interrupt pending bit is cleared by software
	 *     sequence: a read operation to SPI_DR register (SPI_I2S_ReceiveData())
	 *     followed by a read operation to SPI_SR register (SPI_I2S_GetITStatus()).
	 */
	#ifdef TC_SOFT_CHECK
	if(GPIO_ReadOutputDataBit(PORT_SOFT_CHECK_B2,PIN_SOFT_CHECK_B2)){
		GPIO_ResetBits(PORT_SOFT_CHECK_B2,PIN_SOFT_CHECK_B2);
	}
	else{
		GPIO_SetBits(PORT_SOFT_CHECK_B2,PIN_SOFT_CHECK_B2);
	}
    #endif


	if(st_count_spi_int==0){                                                // прерывание №1 - принял валидное состояние входов ТС
		st_count_spi_int++;
		s_reaz_read_TC.data_TC=SPI_I2S_ReceiveData(SPI1);
		SPI_I2S_SendData(SPI1,0x00); // передаю пустую посылку чтобы принять проверочный байт
		return;
	}
	st_count_spi_int=0;                                                    // прерывание №2 - принял проверочный байт состояния схемы сериалайзера
	s_reaz_read_TC.data_cheack=SPI_I2S_ReceiveData(SPI1);
	xSemaphoreGiveFromISR(SemaphoreHandle_spi,&prioryty_take_semaphore);
	taskYIELD(); // выхожу из прерывания, переключив задачу, и сразу попадаю в задачу анализа ТС

}


// ---------функция  TC_calc_address_oper_reg ---------
// функция TC_calc_address_oper_reg - выполняет рассчет адресов регистров (область оперативных регистров) устройства маршрутизации данных
// входные аргументы:
// ps_TC_address           - указатель на структуру адресов регистров устройства (поле в глобальной структуре адресов устройства);
// adres_start             - начальный адрес оперативных регистров устройства в карте памяти. относительно него расчитываються адрес а вех остальных регистров устройства
// выходные аргументы:
//                         - адрес последнего регистра устройства
u16 TC_calc_address_oper_reg(S_TC_address *ps_TC_address, u16 adres_start){
	ps_TC_address->status_TC=adres_start;
	ps_TC_address->rez_TC=ps_TC_address->status_TC+NUM_REG_STATUS_TC;
	adres_start=ps_TC_address->rez_TC+TOTAL_NUM_TC;
	return adres_start;
};


//------------задача processing_TC_signal--------------------
// задача processing_TC_signal выполняет обработку сигналов ТС
void t_processing_TC(void *pvParameters){
	u8 firest_flag=0;
	u8 counter_flesh_influence=0; // счетчик цикдлв опроса после операции из flesh памятью
	u16 tc_status_temp, tc_status_;
	// сохраняю конфигурационные данные в указателе на структуру конфигурации,
	// а также выпоняб проверку
	if(processing_TC_fill_S_TS((u8*)pvParameters)){
		SET_GLOBAL_STATUS(DEV_2);
		vTaskDelete(NULL); // если проверка не пройдена - удалить задачу + аварийная сигнализация + ЗАПИСЬ В КАРТУ ПАМЯТИ !!!!
	};

	// продолжаю конфигурацию устройства
	processing_TC_init();
	LD_SET;
	// эта задача имеет наивышший приоритет
	// - запускаю задачу, запускаеться процес считывания ТС
	// - после считывания записываю данные в регистры МЕМ_МAP
	// - блокирую задачу на 500 мс.
	// а может лучше зделать выполнение задачи по таймеру ???
	while(1){
		xSemaphoreTake(SemaphoreHandle_spi,portMAX_DELAY); // захватываю семафор и ожидаю максимально допустимое время выполнения чтения ТС из сериалайзера
		// проверяю работоспособность модуля ТС по следующим составляющим:
		// 1 - соответствие принятого проверочного бита переданому
		// 2 - сигнал из ножки ТОК сериалайзера (превышение температуры микросхемы)
		// 3 - наличие 24 В на дискретных входах
		// результат отображаеться в статус регистре

		if(firest_flag==0){ // если первый раз ( после подачи питания !!!!) считал, может быть ошибка, повтрить
			firest_flag=1;
			continue;
		}
		//
		tc_status_temp=0;
		// опрашиваю статус TOK
		if(GPIO_ReadInputDataBit(TOK_PORT,TOK_PIN)==TOK_ERROR_STATE){
			// bit 0 stattu
			tc_status_temp|=(1<<TOK_ERROR);
		}
		// опрашиваю статус напряжения
		if(GPIO_ReadInputDataBit(V_PORT,V_PIN)==V_ERROR_STATE){
			// bit 1 stattu
			tc_status_temp|=(1<<V_ERROR);
		}
		// проверяю, сходится ли контрольный принятый байт с переданым
		if(s_reaz_read_TC.data_cheack!=CHECK_SERIAL_BYTE){

			tc_status_temp|=(1<<CHECK_ERROR);
		}
		s_reaz_read_TC.data_cheack=0;
		// обновить статус-регистры
		if(tc_status_!=tc_status_temp){
			tc_status_=tc_status_temp;
			// обновляю статус регистр
			processing_mem_map_write_s_proces_object_modbus(&tc_status_,1,s_address_oper_data.s_TC_address.status_TC);
			// если в статус регистр записано хоть одно значение, данные ТС не обновлять
			if(tc_status_){
				// выставить статус модуля ТС в глобальном статус-регистре
				SET_GLOBAL_STATUS(DEV_2);
				continue;
			}
			else{
				// очистить статус модуля ТС в глобальном статус-регистре
				RESET_GLOBAL_STATUS(DEV_2);
			}
		}
		// анализирую выполнялись ли операции с flesh памятью между опросами сериалайзера, если ДА
		// пропустить MAX_CYCLE_FLASH_INFLUANCE результатов опроса, так как они могут быть
		// неправельны в результате влияния здержки, вызванной прерыванием операциями из flesh памятью
		if(FLASH_OPERATION_flash_state()!=DATA_NON){
		    counter_flesh_influence=MAX_CYCLE_FLASH_INFLUANCE;
			continue;
		}
		// если была выполнена какая то операция из flesh памятью, пропускаю MAX_CYCLE_FLASH_INFLUANCE тактов считывания сериалайзера
		if((counter_flesh_influence>=0)&&(counter_flesh_influence<=MAX_CYCLE_FLASH_INFLUANCE)){
			counter_flesh_influence--;
			continue;
		}
		// анализирую входы телесигнализации
		processing_TC_signal_mem_update();
	}
}

//-------------------------функция processing_TC_signal_mem_update------------------------------
// функция processing_TC_signal_mem_update - выполняет обработку сигналов ТС принятых от сериалайзера и обновление данных ТС в карте памяти
void processing_TC_signal_mem_update(void){
	u8 k1;
	for(k1=0;k1<TOTAL_NUM_TC;k1++){
		//	READ_BIT_TC(s_reaz_read_TC.data_TC,k1);
		// если сигнал ТС Не находится в петле гистерезиса
		if(s_TC_processing.as_one_TC_processing[k1].tc_is_procssing==IS_NOT_PROCESSING){
			// Состояние ТС не изменилось
			if(s_TC_processing.as_one_TC_processing[k1].tc_state==READ_BIT_TC(s_reaz_read_TC.data_TC,k1))
			{
				continue;
			}
			//состояние изменилось
			s_TC_processing.as_one_TC_processing[k1].tc_state=READ_BIT_TC(s_reaz_read_TC.data_TC,k1);
			s_TC_processing.as_one_TC_processing[k1].tc_is_procssing=IS_PROCESSING;
			// записать новое состояние в карту памяти
			processing_mem_map_write_s_proces_object_modbus(&s_TC_processing.as_one_TC_processing[k1].tc_state,
					1,
					s_address_oper_data.s_TC_address.rez_TC+k1);
			continue;
		}
		// если сигнал ТС находится в петле гистерезиса
		//s_TC_processing.as_one_TC_processing[k1].tc_counter++;
		if(s_TC_processing.as_one_TC_processing[k1].tc_counter>=ps_TC_user_config->time_gist){
			s_TC_processing.as_one_TC_processing[k1].tc_counter=0;
			s_TC_processing.as_one_TC_processing[k1].tc_is_procssing=IS_NOT_PROCESSING;
			continue;
		}
	}
}


//------------функция processing_TC_signal_check-----------------------------------
// функция processing_TC_signal_check - выполняет проверку необходимых параметров модуля ТС в MODBUS перед выдачей ТС
// входные аргументы:
//  data_chek - в ф-и не используеться
// выходные аргументы:
// 0 - модуль ТС функционирует нормально, выдача ТС возможна
// 1 - ошибка модуля (причина ошибки не уточняеться) выдача ТС невозможна
REZ_REQ_CHEACK_SLAVE processing_TC_signal_check(void* data_chek){
	u16 status_TC;
	processing_mem_map_read_s_proces_object_modbus(&status_TC,1,s_address_oper_data.s_TC_address.status_TC);
	if(status_TC){ return SLAVE_DEVICE_FALIURE;} // исключение типа "НЕИСПРАВНОСТЬ ПОДЧИНЕННОГО"
	return 0;
}




