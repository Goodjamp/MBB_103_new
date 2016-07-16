/*
 * processing_TY_signal.c
 *
 *  Created on: February 15, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "processing_TY.h"

// Семафор для отсчета времени импульса ТУ типа DP
xSemaphoreHandle SemaphoreHandle_ty;
portBASE_TYPE prioryty_take_semaphore_ty;
//масив структур пинов и портов управления релл
S_ty_rel_out s_rel_out[TOTAL_NUM_TY];
//масив структур пинов и портов проверки обмоток релле
S_ty_rel_out s_coil_check[TOTAL_NUM_TY];
//Структура глобальных настроек пользователя модуля ТС
S_TY_user_config *ps_TY_user_config;
// Структура описания и выполнения команды ТУ
S_ty_operation s_TY_operation;
//Структура настройки циклической проверки состояния иодуля ТУ
S_ty_cycling_check s_cycling_check;
// Структура регистров-статусов поточного состояния модуля ТУ
S_state_TY s_oper_data_TY_present;
// Масив функций которые, в поточной конфигурации, нельзя использовать для проверки
F_check a_check_fun_exception[NUM_CHECK_FUN]={[0 ... (NUM_CHECK_FUN-1)]=0};
// Структура обработки проверки обмоток релле
static S_ty_coil_check_grup s_ty_coil_check_grup;
// флаг-признак окончания отсчета времени для выхода типа DP
static u8 f_dp_time_complite=RESET;
// структура адресов оперативных регистров (из mem_map_processing.c)
extern S_address_oper_data s_address_oper_data;

//---------------функция processing_TY_signal_init---------------
// функция processing_TY_signal_init -выполняет конфигурацию програмного блока ТУ
void processing_TY_signal_init(void) {
	// опредиляю,согласно установленному джамперу, режим проверки (без_проверок внешних цепей/из проверками)
	if(STATE_JAMPER2){ //если джампер НЕ установлен (лог 1) - выполнять полную проверку
		s_ty_coil_check_grup.f_limit_check=RESET;
	}
	else //если джампер установлен (лог 0) - выполнять выборочную проверку
	{
		s_ty_coil_check_grup.f_limit_check=SET;
	}
	// конфигурация портов ввода вывдо задействованых модулем ТУ
	processing_TY_signal_init_gpio();
	// конфигурация таймера
	processing_TY_signal_init_tim();
	// Конфигурация процеса выполнения ТУ и циклических проверок
	processing_TY_signal_set_check_par();
	// задаю функцию callback проверки запрашиваимого адреса данных команды READ_COIL_STATUS (№1) для Modbus
	modbus_callback_address_check(
			&processing_TY_check_is_coil_status_address_modbus,
			READ_COIL_STATUS);
	// задаю функцию callback проверки запрашиваимого адреса данных команды FORCE_SINGLE_COIL (№5) для Modbus
	modbus_callback_address_check(
			&processing_TY_check_force_single_coil_address_modbus,
			FORCE_SINGLE_COIL);
	// задаю дополнительную функция проверки команды FORCE_SINGLE_COIL (№5) для Modbus
	modbus_callback_add_check(&processing_TY_signal_modbus_check, FORCE_SINGLE_COIL);
	// задаю дополнительную функцию callback обработки команды FORCE_SINGLE_COIL (№5) для Modbus
	modbus_callback_add_processing(&processing_TY_signal_operation,	FORCE_SINGLE_COIL);
	// принудительно очистить очередь на выполнение команды ТУ
	processing_TY_signal_clear();
	//Выполняю конфигурацию статус регистров (считываю из флеш и т. д.)
	processing_TY_signal_init_state();
	//Создаю семафор
	vSemaphoreCreateBinary(SemaphoreHandle_ty);
}

//------------функция INIT_MBB_fill_S_TY---------------
// функция INIT_MBB_fill_S_TY - заполняет структуры настройки ТУ
INIT_MBB_Rezult processing_TY_fill_S_TY(u8 *read_data) {
	// копирую настройки TY
	ps_TY_user_config = (S_TY_user_config*) read_data;
	//выполняем анализ и обработку конфигурации пользователя
	return processing_TY_signal_analis_user_config();
}

//---------------функция processing_TY_signal_analis_user_config---------------
// функция processing_TY_signal_analis_user_config -выполняет анализ конфигурации модуля ТУ пользователем
// Выполгяеться анализ следующих параметров:
// - тип выхода DP/SP
// - - DP может быть присвоен только парным номерам ТУ (номер выхода ТУ в карте памяти - номер первого ТУ).
// Тип DP присвоен выходу ON, выход OFF (следующий номер выхода) конфигурируется автоматически на тип DP.
// Номер паралельного канала в настройках присваиваеться ножке ОFF, далее этот
// выход не должен быть использован (в настройках выхода паралельного канала должен быть указан тип PARALLEL_CHANNEL=3)
// - - SP может быть задан любому свободному выходу (парный / не парный)
INIT_MBB_Rezult processing_TY_signal_analis_user_config(void) {
	u8 k1;
	//--------------------------вложенная функция analis_user_config_check_paralel-----------------------------------
	INIT_MBB_Rezult analis_user_config_check_paralel(u8 num_paralel_sourse,
			u16 num_paralel_check, TY_MODE type) {
		//существует ли выход паралельного канала согласно конфигурации
		if ((ps_TY_user_config->s_TY_out_config[num_paralel_check].num_paralel < s_address_oper_data.s_TY_address.set_state_TY)||
				(ps_TY_user_config->s_TY_out_config[num_paralel_check].num_paralel >=
				(s_address_oper_data.s_TY_address.set_state_TY + TOTAL_NUM_TY))) // номер должен быть менше TOTAL_NUM_TY
		{
			return MBB_INIT_ERROR;
		};
		// проверка типа паралельного выхода
		if (ps_TY_user_config->s_TY_out_config[num_paralel_check].mode_TY!= type) // тип должен быть PARALLEL_CHANNEL
		{
			return MBB_INIT_ERROR;
		};
		//В поле "номер паралельного канала" паралельного канала должен быть номер управляющего выхода OFF DP
		if (ps_TY_user_config->s_TY_out_config[num_paralel_check].num_paralel!=
				(num_paralel_sourse+ s_address_oper_data.s_TY_address.set_state_TY)) // (k1+1) - потому что управляющим выходом есть выход OFF DP
		{
			return MBB_INIT_ERROR;
		};
		// Проверка конфигурации модуля ТУ выполнена успешно
		{
			return MBB_INIT_OK;
		};
	}
	;

	for (k1 = 0; k1 < TOTAL_NUM_TY; k1++) {

		if ((ps_TY_user_config->s_TY_out_config[k1].mode_TY != DOUBLE_POSITION)
			&& (ps_TY_user_config->s_TY_out_config[k1].mode_TY!= SINGLE_POSITION)
			&& (ps_TY_user_config->s_TY_out_config[k1].mode_TY!= PARALLEL_CHANNEL)
			&& (ps_TY_user_config->s_TY_out_config[k1].mode_TY != NO_OUT))
		{
			return MBB_INIT_ERROR;
		}
		// выход ТУ не задействован
		if (ps_TY_user_config->s_TY_out_config[k1].mode_TY == NO_OUT)
		{
			continue;
		};

		//-----------ТИП ВЫХОДА ТУ - DOUBLE POSITION

		if (ps_TY_user_config->s_TY_out_config[k1].mode_TY == DOUBLE_POSITION)
		{
			if (k1 % 2)
			{
				return MBB_INIT_ERROR;
			} // только парные номера выходов доступны для конфигурации DP
			// существует ли выход [k1+1] (выход OFF)
			if ((k1 + 1) >= TOTAL_NUM_TY)
			{
				return MBB_INIT_ERROR;
			}; // если не существует, ИСКЛЮЧЕНИЕ
			// проверка типа выхода [k1+1] (выход OFF DP)
			if (ps_TY_user_config->s_TY_out_config[k1 + 1].mode_TY!= DOUBLE_POSITION)
			{
				return MBB_INIT_ERROR;
			}; // если тип следующего выхода не DOUBLE_POSITION ИСКЛЮЧЕНИЕ
			//проверка присутствия паралельного выхода
			if (!(ps_TY_user_config->s_TY_out_config[k1 + 1].f_paralel_out))
			{
				k1++;
				continue;
			}; // если паралельного выхода нету, перехожу к проверке выхода k1+2
			//если паралельный выход есть
			//проверяю согласованость управляющего выхода и паралельного
			if (analis_user_config_check_paralel(k1 + 1,
				ps_TY_user_config->s_TY_out_config[k1 + 1].num_paralel- s_address_oper_data.s_TY_address.set_state_TY,
				PARALLEL_CHANNEL))
			{
				return MBB_INIT_ERROR;
			};

			//проверка выполнена без исключений
			k1++;
			continue;
		}; // перехожу к проверке выхода k1+2 ()

		//-----------ТИП ВЫХОДА ТУ - SINGLE POSITIN

		if (ps_TY_user_config->s_TY_out_config[k1].mode_TY == SINGLE_POSITION)
		{
			continue;
		}

		//-----------ТИП ВЫХОДА ТУ - PARALLEL_CHANNEL

		if (ps_TY_user_config->s_TY_out_config[k1].mode_TY== PARALLEL_CHANNEL)
		{
			//проверяю согласованость паралельного выхода и управляющего
			if (analis_user_config_check_paralel(k1,
					ps_TY_user_config->s_TY_out_config[k1].num_paralel
					- s_address_oper_data.s_TY_address.set_state_TY,
					DOUBLE_POSITION))
			{
				return MBB_INIT_ERROR;
			};
		};
	};
	return MBB_INIT_OK;
}

//---------------функция processing_TY_signal_init_state---------------
// функция processing_TY_signal_init_state - считывает последние значения состояни из ФЛЕШ,
// заполняет поля статуса ТУ и устанавливает их
//
void processing_TY_signal_init_state(void) {
	S_state_TY s_state_TY;
	u8 count_ty_out;
	// Считываю данные состояния ТУ из ФЛЕШ
	FLASH_OPERATION_read_flash_8b((u8*) &s_state_TY, sizeof(S_state_TY),
			PAGE(TY_FLASH_PAGE));

	// устанавливаю все статус регистры выходов ТУ в неопредиленное состояние
	s_oper_data_TY_present.status_TY = 0;
	for (count_ty_out = 0; count_ty_out < TOTAL_NUM_TY; count_ty_out++)
	{
		s_oper_data_TY_present.present_state_TY[count_ty_out] = TY_NOT_SET;
		s_oper_data_TY_present.set_state_TY[count_ty_out] = TY_NOT_SET;
		s_oper_data_TY_present.operation_TY_statys[count_ty_out] = REZ_CHECK_OK;
	}
	/*
	 // если статус перед перезагрузкой или выключением был исключением - значит устанавливать значения не нужно,
	 // все выходы в состояние НЕОПРЕДИЛЕН
	 if(s_state_TY.status_TY){
	 return;
	 }
	 */

	//memcpy((u8*)&s_oper_data_TY_present,(u8*)&s_state_TY,sizeof(S_state_TY));
	// если статус не был установлен (модуль ТУ функционировал) анализирую
	// - если выход ТУ типа SINGLE_POSITION - нужно установить то значения ТУ, которое было до перезагрузки, и выставить аналогичные статтусы
	// - если выход ТУ типа DOUBLE_POSITION - значения состояния не определино (не анализирую)
	for (count_ty_out = 0; count_ty_out < TOTAL_NUM_TY; count_ty_out++)
	{ //
		if (ps_TY_user_config->s_TY_out_config[count_ty_out].mode_TY== SINGLE_POSITION)
		{ // если выход ТУ типа SINGLE_POSITION
			if (s_state_TY.operation_TY_statys[count_ty_out] != REZ_CHECK_OK)
			{
				continue;
			}; // если перед перезагрузкой последняя операция ТУ
			// была выпонена с исключением - не анализировать состояние
			if (s_state_TY.present_state_TY[count_ty_out] == TY_ON) // если перед перезагрузкой или выключением ТУ был установлен успешно
			{
				TY_SET_OUT(s_rel_out[count_ty_out].port_operation,
						s_rel_out[count_ty_out].pin_operation);
				// Записываю значения поточного состояния в статус регистры выхода ТУ
				s_oper_data_TY_present.present_state_TY[count_ty_out] = TY_ON;
				s_oper_data_TY_present.set_state_TY[count_ty_out] = TY_ON;
				s_oper_data_TY_present.operation_TY_statys[count_ty_out] =
						REZ_CHECK_OK;
			}
			if (s_state_TY.present_state_TY[count_ty_out] == TY_OFF)// если перед перезагрузкой или выключением ТУ был сброшен успешно
			{
				TY_CLEAR_OUT(s_rel_out[count_ty_out].port_operation,
						s_rel_out[count_ty_out].pin_operation);
				// Записываю значения поточного состояния в статус регистры выхода ТУ
				s_oper_data_TY_present.present_state_TY[count_ty_out] = TY_OFF;
				s_oper_data_TY_present.set_state_TY[count_ty_out] = TY_OFF;
				s_oper_data_TY_present.operation_TY_statys[count_ty_out] =
						REZ_CHECK_OK;

			}
		}
	}
	// записываю в карту памяти начальное состояние регистров
	processing_mem_map_write_s_proces_object_modbus((u16*)&s_oper_data_TY_present, (sizeof(S_state_TY) / 2),s_address_oper_data.s_TY_address.status_TY);
}

//---------------функция processing_TY_signal_init_tim---------------
// функция processing_TY_signal_init_tim -выполняет конфигурацию таймеров
// TIM2 - циклический отсчет для выполнения проверок наличия напряжения оперативного тока и состояния обмоток
// TIM1 - для отсчета времени импульса и отсчета периодов запуска проверки котушек релле
//        во время выполнения команды DP
void processing_TY_signal_init_tim(void) {
	//----------------------------------Конфигурация таймера TIM2------------------------
	RCC_ClocksTypeDef rcc_ClocksTypeDef;
	RCC_GetClocksFreq(&rcc_ClocksTypeDef);
	//----------------------------------Конфигурация таймера TIM1------------------------
	// кратность счета одного такта таймера - 10 мкс
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	TIM1->PSC = rcc_ClocksTypeDef.PCLK1_Frequency / 100000;
	TIM1->PSC = TIM1->PSC * 2 - 1;
	TIM1->CCR1 = TIME_BEFORE_CHECK_COIL;
	TIM1->SR = 0;
	TIM1->CNT = 0;
	TIM1->DIER |= (TIM_DIER_CC1IE); // использую прерывание по сравнению
	TIM1->EGR |= TIM_EGR_UG;
	// максимально допустимый приоритет прерывания
	//NVIC_SetPriority(TIM2_IRQn,12);
	NVIC_EnableIRQ(TIM1_CC_IRQn);
	//----------------------------------Конфигурация таймера TIM2------------------------
	// Кратность счета - 0.1 мс (100 мкс)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM2->CR1 |= TIM_CR1_ARPE;
	TIM2->PSC = rcc_ClocksTypeDef.PCLK1_Frequency / 10000;
	TIM2->PSC *= 2; // см документ.(если APB1_PSC>1, частота тактирования таймера f_APB1 x 2)
	TIM2->ARR = ps_TY_user_config->puls_with * 10;
	TIM2->CCR1=1;
	TIM2->EGR |= (TIM_EGR_UG);
	TIM2->SR = 0;
	TIM2->CNT = 0;
	TIM2->DIER |= ((TIM_DIER_UIE)|(TIM_DIER_CC1IE)); // использую прерывание по переполнению таймера (конец отсчета импульса DP)
	                                                 // и по переполнению (начать проверку состояния котушек релле)
	// максимально допустимый приоритет прерывания
	NVIC_SetPriority(TIM2_IRQn, 12);
	NVIC_EnableIRQ(TIM2_IRQn);

}

//---------------функция processing_TY_signal_check_coil---------------
// функция processing_TY_signal_check_coil -проверку котушек релле
//Даная ф-я выполняет анализ какие котушки нужно проверять:
// - если находимся в процесе выполнения команды ТУ- это котушки релле выходов типа SP+DP+parallel
// - если находимся в данный момент не выполняеться команда - проверка всех котушек
TY_REZ_CHECK processing_TY_signal_check_coil(void) {
	u8 counter;
	u8 num_out; // номер выхода ТУ, который стоит в очереди на комутацию
	TY_REZ_CHECK rez_grup_coil_check =REZ_CHECK_OK;

	// Если в очереди на выполнение стоит команда, "И"
	// тип выхода ТУ который нужно каомутировать DOUBLE_POSITION, "И"
	// состояние которое нужно установить - TY_OFF ТО
	// ПЕРЕЙТИ НА ВЫХОД num_TY+1

	if((s_TY_operation.f_TY)&&
		(ps_TY_user_config->s_TY_out_config[s_TY_operation.num_TY].mode_TY==DOUBLE_POSITION)&&
		(s_TY_operation.state_TY==TY_OFF)	)
	{
		num_out=s_TY_operation.num_TY+1;
	}
	else
	{
		num_out=s_TY_operation.num_TY;
	}

	// выполнить поиск котушек которые необходимо проверить согласно поточному состоянию модуля ТУ:
	// - если данная ф-я вызвана в процесе выполнения команды DP - проверять только котушки
	//   релле которые задействованны в процесе выполнения даной команды (а именно - если команда ВКЛ
	//   проверять только котушку ВКЛ, если команда ВЫКЛ - проверять котушку ВЫКЛ, если задействованный
	//   паралельны канал - то котушку паралельного канала)
	// - если данная ф-я вызвана между выполнениями команд - проверять все котушки.
	//
	// Выбираю котушки исходя из условия от обратного: если поточный выход ТУ соответствует условиям НЕ попадания в
	// список - перехожу к проверке следующего выхода

	for (counter = 0,s_ty_coil_check_grup.num_check_TY=0; counter < TOTAL_NUM_TY; counter++) {
		if ( //---если в очереди на выполнение стоит команда,  "И"
		     //---тип поточного проверяимого выхода ТУ DOUBLE_POSITION, "И"
			 //---поточный проверяимый выход ТУ не стоит в списке на выполнения
			((s_TY_operation.f_TY)&&\
			(ps_TY_user_config->s_TY_out_config[counter].mode_TY == DOUBLE_POSITION) && \
			(counter!=num_out))||\
			//---если в очереди на выполнение стоит команда,  "И"
			//---"И" если тип поточного проверяимого выхода, PARALLEL_CHANNEL
			//---"И" управляющий им выход ТУ не стоит в списке на выполнение
			((s_TY_operation.f_TY)&&\
			(ps_TY_user_config->s_TY_out_config[counter].mode_TY == PARALLEL_CHANNEL) && \
			((num_out+s_address_oper_data.s_TY_address.set_state_TY)!=ps_TY_user_config->s_TY_out_config[counter].num_paralel)))
		{
			continue;
		}
		// во всех остальных случаях - поставить поточный выход ТУ в список на проверку
		else
		{
			s_ty_coil_check_grup.s_TY_is_check[s_ty_coil_check_grup.num_check_TY].num_check_TY=counter;
			s_ty_coil_check_grup.s_TY_is_check[s_ty_coil_check_grup.num_check_TY].rez_check_TY=0;
			s_ty_coil_check_grup.num_check_TY++;
		}
	}
	// обнуляю флаг процеса выполнения проверки котушки
	s_ty_coil_check_grup.f_check_end=RESET;
	// запустить таймер управления проверкой
	TIM_Cmd(TIM1, ENABLE);
	// ждем окончания проверки
	while(!s_ty_coil_check_grup.f_check_end){}
	// выполняем анализ результатов проверки
	// - проверка выполняеться в два такта: инвертируем выходы управления ТУ (инвертируем -
	//    потому, что мы выполняем проверку котушек релле как между командами, тогда состояние
	//    выходов ТУ 0-1-0, так и во время команды, тогда состояние выходов управления ТУ 1-0-1)
	// - после прохождения времени переходного процеса, считываю значения на входах проверки состояния
	//   котушек, инвертирую выходы управления ТУ (возвращаю в начальное положения)
	// - после прохождения времени переходного процеса, считываю значения на входах проверки состояния
	//   котушек.
	// результаты считывания записываю в поле rez_check_TY. если в результате проверки
	// получены значени 0b01 или 0b10 - котушка исправне, если 0b00(ERROR_XX) или 0b11(ERROR_K3) - котушкаа
	// повреждена
	for(counter=0;counter<s_ty_coil_check_grup.num_check_TY;counter++){
		if((s_ty_coil_check_grup.s_TY_is_check[counter].rez_check_TY==ERROR_XX)||
		   (s_ty_coil_check_grup.s_TY_is_check[counter].rez_check_TY==ERROR_K3))
		{
		// если проверка для поточного выхода ТУ закончилась ошибкой - запись в статус-регистр выхода и возврат
		// ERROR_TY_COIL функцией
			rez_grup_coil_check =ERROR_TY_COIL;
			s_oper_data_TY_present.operation_TY_statys[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY]|=ERROR_TY_COIL;
		}
		else  // если проверка закончилась удачно - очистить статус проверки в поточном статус-регистре выхода ТУ
		{
			s_oper_data_TY_present.operation_TY_statys[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY]&=~ERROR_TY_COIL;
		}
	}

	return rez_grup_coil_check;
}

//------------прерывание  TIM1_CC_IRQHandler----------------
// прерывание  TIM1_CC_IRQHandler - прерывание по TIM1
// Даное прерывание вызывает события совпадения счетного регистра и регистра захвата/сравнения ССR1
// В теле даного прерывания выполняються все комyтации проверки котушек релле
void TIM1_CC_IRQHandler(void) {
	static u8 count_interupt = 0;
	u8 counter;
	// обрабатываю только прерывания регистра захвата/сраванения
	if ((TIM1->SR) && (TIM_SR_CC1IF))
	{
		TIM1->SR &= ~(TIM_SR_CC1IF);
	} else
	{
		return;
	}
	TIM1->CNT = 0;
	// прерывание №1 - инвертировать выходы управления ТУ
	if (count_interupt == 0)
	{
		TIM1->CCR1 = TIME_TRANSIENT_CHECK_COIL;
		TIM1->EGR |= TIM_EGR_UG;
		// инвертирую все выходы из списка
		for (counter = 0; counter < s_ty_coil_check_grup.num_check_TY; counter++) {
			TY_INVERT_OUT(s_rel_out[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].port_operation,
				        s_rel_out[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].pin_operation);
		}
		count_interupt++;
		return;
	}
	// прерывание №2 - считать значения из проверочных ножек и выходы управления ТУ
	else if (count_interupt == 1) {
		TIM1->CCR1 = TIME_TRANSIENT_CHECK_COIL;
		TIM1->EGR |= TIM_EGR_UG;
		for (counter = 0; counter < s_ty_coil_check_grup.num_check_TY; counter++)
		{
			// считываю состояние на проверочных ножках и выполняю запись
			if (GPIO_ReadInputDataBit(s_coil_check[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].port_operation,
					                  s_coil_check[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].pin_operation))
			{
				s_ty_coil_check_grup.s_TY_is_check[counter].rez_check_TY |= (1 << (count_interupt - 1));
			}
			// инвертирую все выходы из списка
			TY_INVERT_OUT(s_rel_out[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].port_operation,
					      s_rel_out[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].pin_operation);

		}
		count_interupt++;
		return;
	}
	// прерывание №3 (последнее)- считать значения из проверочных ножек
	else if (count_interupt == 2)
	{
		TIM_Cmd(TIM1, DISABLE);            // ВЫКЛЮЧАЮ ТАЙМЕР
		TIM1->CCR1 = TIME_BEFORE_CHECK_COIL;
		TIM1->EGR |= TIM_EGR_UG;
		s_ty_coil_check_grup.f_check_end=1;  // устанавливаю флаг окончания проверки состояния обмоток
		for (counter = 0; counter < s_ty_coil_check_grup.num_check_TY; counter++) {
			// считываю состояние на проверочных ножках и выполняю запись
			if (GPIO_ReadInputDataBit(s_coil_check[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].port_operation,
								      s_coil_check[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].pin_operation))
			{
				s_ty_coil_check_grup.s_TY_is_check[counter].rez_check_TY |= (1 << (count_interupt - 1));
			}
		}
		count_interupt = 0;
		return;
	}
}

//------------прерывание  TIM2_IRQHandler----------------
// прерывание  TIM2_IRQHandler - прерывание по переполнению таймера №6.
// Данное прерывание возникает после отсчета длины импульса срабатывания ТУ типа DP
void TIM2_IRQHandler(void) {
	if(TIM_GetITStatus(TIM2,TIM_IT_Update))
	{
		TIM_ClearFlag(TIM2,TIM_FLAG_Update);
		TIM_Cmd(TIM2,DISABLE); // выключаю таймер
		TIM_SetCounter(TIM2,0);
		TY_CLEAR_OUT(PORT_GRYP_REL, PIN_GRYP_REL);// отключить груповое релле
		// отдаю семафор
		f_dp_time_complite=1; // устанавливаю флаг-признак окончания времени отсчета
		TIM2->CCR1=(TIME_TOTAL_CHECK_COIL*10); // устанавливаю начальное время проверки для следующего включения
	}
	else if(TIM_GetITStatus(TIM2,TIM_IT_CC1))
	{
		TIM_ClearFlag(TIM2,TIM_FLAG_CC1);
		if((TIM2->CCR1+TIME_TOTAL_CHECK_COIL*10*2)<TIM2->ARR)// если следующее прерывание по сравнению с регистром захвата/сравнения наступит до события "UPDATE" + запас
		{                                                    // значит можно выполнить еще один такт проверки состояния котушек
			TIM2->CCR1+=(TIME_TOTAL_CHECK_COIL*10);
		}
	}
	else
	{
		TIM2->SR=0;
	}
	// отдаю семафор
	xSemaphoreGiveFromISR(SemaphoreHandle_ty, &prioryty_take_semaphore_ty);
}

//---------------функция init_TY_gpio---------------
// функция init_TY_gpio -выполняет конфигурацию всех портов ввода/вывода которые используються
// програмным блоком ТУ
void processing_TY_signal_init_gpio(void) {
	GPIO_InitTypeDef gpio_TY_init;
	u8 k1;

	// порты ТУ
	s_rel_out[0].port_operation = PORT_TY_REL1;
	s_rel_out[1].port_operation = PORT_TY_REL2;
	s_rel_out[2].port_operation = PORT_TY_REL3;
	s_rel_out[3].port_operation = PORT_TY_REL4;
	// пины ТУ
	s_rel_out[0].pin_operation = PIN_TY_REL1;
	s_rel_out[1].pin_operation = PIN_TY_REL2;
	s_rel_out[2].pin_operation = PIN_TY_REL3;
	s_rel_out[3].pin_operation = PIN_TY_REL4;

	// конфигурирую ножки ТУ на выход
	for (k1 = 0; k1 < TOTAL_NUM_TY; k1++)
	{
		gpio_enable(s_rel_out[k1].port_operation); // включаю выбраный порт
		gpio_TY_init.GPIO_Mode = GPIO_Mode_Out_OD;
		gpio_TY_init.GPIO_Speed = GPIO_Speed_2MHz;
		gpio_TY_init.GPIO_Pin = s_rel_out[k1].pin_operation;
		GPIO_Init(s_rel_out[k1].port_operation, &gpio_TY_init);
		TY_CLEAR_OUT(s_rel_out[k1].port_operation, s_rel_out[k1].pin_operation);
	}

	// конфигурирую порт управления групой реле ТУ
	gpio_enable(PORT_GRYP_REL); // включаю выбраный порт
	gpio_TY_init.GPIO_Mode = GPIO_Mode_Out_OD;
	gpio_TY_init.GPIO_Speed = GPIO_Speed_2MHz;
	gpio_TY_init.GPIO_Pin = PIN_GRYP_REL;
	GPIO_Init(PORT_GRYP_REL, &gpio_TY_init);
	TY_CLEAR_OUT(PORT_GRYP_REL, PIN_GRYP_REL);

	// конфигурирую порт управления диагностикой
	gpio_enable(PORT_V_REL); // включаю выбраный порт
	gpio_TY_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_TY_init.GPIO_Speed = GPIO_Speed_2MHz;
	gpio_TY_init.GPIO_Pin = PIN_V_REL;
	GPIO_Init(PORT_V_REL, &gpio_TY_init);
	TY_CLEAR_OUT(PORT_V_REL, PIN_V_REL);

	 // конфигурирую порт проверки наличия напряжения оперативного тока
	 gpio_enable(PORT_V_INPUT);// включаю выбраный порт
	 gpio_TY_init.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	 gpio_TY_init.GPIO_Speed=GPIO_Speed_2MHz;
	 gpio_TY_init.GPIO_Pin=PIN_V_INPUT;
	 GPIO_Init(PORT_V_INPUT,&gpio_TY_init);


	// конфигураци портов проверки состояния обмотки релле
	// порты проверки состояния обмотки релле
	s_coil_check[0].port_operation = PORT_CHK_COIL1;
	s_coil_check[1].port_operation = PORT_CHK_COIL2;
	s_coil_check[2].port_operation = PORT_CHK_COIL3;
	s_coil_check[3].port_operation = PORT_CHK_COIL4;
	// пины проверки состояния обмотки релле
	s_coil_check[0].pin_operation = PIN_CHK_COIL1;
	s_coil_check[1].pin_operation = PIN_CHK_COIL2;
	s_coil_check[2].pin_operation = PIN_CHK_COIL3;
	s_coil_check[3].pin_operation = PIN_CHK_COIL4;

	// конфигурирую пинов проверки состояния обмотки релле на вход
	for (k1 = 0; k1 < TOTAL_NUM_TY; k1++)
	{
		gpio_enable(s_rel_out[k1].port_operation); // включаю выбраный порт
		gpio_TY_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		gpio_TY_init.GPIO_Speed = GPIO_Speed_10MHz;
		gpio_TY_init.GPIO_Pin = s_coil_check[k1].pin_operation;
		GPIO_Init(s_coil_check[k1].port_operation, &gpio_TY_init);
	}
}

// включить тактирование заданного GPIO
void gpio_enable(GPIO_TypeDef *IN_GPIO){
	if (IN_GPIO == GPIOA) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	} else if (IN_GPIO == GPIOB)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	} else if (IN_GPIO == GPIOC)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	} else if (IN_GPIO == GPIOD)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	}
}

//---------------функция processing_TY_signal_set_check_par---------------
// функция processing_TY_signal_set_check_par -выполняет:
// - заполнение спика функций проверок цепей устройства которые можно использовать в поточном режиме работы
// - заполнение спика функций циклической проверки
void processing_TY_signal_set_check_par(void) {
	s_cycling_check.num_cycling_check = 0;
    //заполнение спика функций проверок цепей устройства, которые можно использовать в поточном режиме работы
	if(s_ty_coil_check_grup.f_limit_check)//если выбран ограниченный режим проверок
	{
		a_check_fun_exception[0]=&processing_TY_signal_check_v;
		a_check_fun_exception[1]=&processing_TY_signal_check_rel_contact;
		a_check_fun_exception[2]=&processing_TY_signal_check_ty_rel_on;
		a_check_fun_exception[3]=&processing_TY_signal_check_gryp_rel_on;
		a_check_fun_exception[4]=&processing_TY_signal_check_gryp_rel_off;
		a_check_fun_exception[5]=&processing_TY_signal_check_ty_rel_off;
	}

	// заполнение спика функций циклической проверки

	// ПРОВЕРКА - проверка напряжения оперативного тока
	s_cycling_check.pf_fun[s_cycling_check.num_cycling_check].pf_check = &processing_TY_signal_check_v;
	s_cycling_check.pf_fun[s_cycling_check.num_cycling_check].maska_error = ERROR_V_INPUT;
	s_cycling_check.num_cycling_check++;

	// ПРОВЕРКА - проверка залипания контактов
	s_cycling_check.pf_fun[s_cycling_check.num_cycling_check].pf_check =	&processing_TY_signal_check_rel_contact;
	s_cycling_check.pf_fun[s_cycling_check.num_cycling_check].maska_error = ERROR_TY_REL_CONTACT;
	s_cycling_check.num_cycling_check++;

	// ПРОВЕРКА - проверка котушек релле
	s_cycling_check.pf_fun[s_cycling_check.num_cycling_check].pf_check =	&processing_TY_signal_check_coil;
	s_cycling_check.pf_fun[s_cycling_check.num_cycling_check].maska_error = ERROR_TY_COIL;
	s_cycling_check.num_cycling_check++;

}

//---------------функция processing_TY_signal_check---------------
// функция processing_TY_signal_check - выполняет проверку параметров запроса на выполнение ТУ (для MODBUS)
// Выполняеться проверка следующих парметров модуля ТУ:
// - статус регистр;
// - согласовываеться возможность выполнения указаного ТУ выходом ТУ из адреса запроса
REZ_REQ_CHEACK_SLAVE processing_TY_signal_modbus_check(void* req) {
	S_ty_comand *ps_comand = req;
	u8 number_out_TY;
	u16 status_TY;
	// Если модуль ТУ отключен програмно
	if (ps_TY_user_config->state == DISABLE)
	{
		return ILLEGAL_FUNCTION; // исключение типа "НЕДОСТУПНАЯ ФУНКЦИЯ"
	}
	// проверка аргумента команды (он должен соответствовать ВКЛ/ВЫКЛ 0хFF00/0x0000 )
	//если значение ТУ которое нужно установить не соответствует допустимым - выход с ошибкой
	if ((ps_comand->state != TY_ON) && (ps_comand->state != TY_OFF))
	{
		return ILLEGAL_DATA_VALUE;
	}
	// Порядочныйномер ТУ
	number_out_TY = ps_comand->number
			- s_address_oper_data.s_TY_address.set_state_TY;
	//проверка выхода ТУ согласно конфигурации
	if ((ps_TY_user_config->s_TY_out_config[number_out_TY].mode_TY == NO_OUT)
			|| (ps_TY_user_config->s_TY_out_config[number_out_TY].mode_TY== PARALLEL_CHANNEL))
	{
		return ILLEGAL_DATA_ADRESS; // исключение типа "НЕДОСТУПНЫЙ АДРЕС"
	}
	// Проверка допустимости выполнения ТУ указанного типа заданым выходом
	if ((number_out_TY % 2)	&& (ps_TY_user_config->s_TY_out_config[number_out_TY].mode_TY== DOUBLE_POSITION))
	{
		return ILLEGAL_DATA_ADRESS; // исключение типа "НЕДОСТУПНЫЙ АДРЕС"
	}

	// Проверка статус-регистра
	processing_mem_map_read_s_proces_object_modbus(&status_TY, 1,s_address_oper_data.s_TY_address.status_TY);
	if ( STATUS_STATE(status_TY))
	{
		return SLAVE_DEVICE_FALIURE; // исключение типа "НЕИСПРАВНОСТЬ ПОДЧИНЕННОГО"
	}
	// проверка на выполнение в данный момент команды ТУ
	if (s_TY_operation.f_TY)
	{
		return SLAVE_DEVICE_BUSY; // исключение типа "ПОДЧИНЕННЫЙ ЗАНЯТ ВЫПОЛНЕНИЕМ ПРЕДЫДУЩЕЙ КОМАНДЫ"
	}
	return (0);
}

//------------функция processing_TY_signal_out_check-----------------------------------
// функция processing_TY_signal_out_check - выполняет проверку адресса карты памяти на принадлежность расширенным регистрам состояния выходов ТУ
// даная функция соответствует проверке команде №1 modbus
// входные аргументы:
// *p_check_address - указатель на начало последовательности (u16)address, (u16)num_reg
// выходные аргументы:
// REQ_SLAVE_OK - проверкаа выполнена успешно
// ILLEGAL_DATA_ADRESS - недопустимый адрес
REZ_REQ_CHEACK_SLAVE processing_TY_check_is_coil_status_address_modbus(void* p_check_address) {
	u16 address = *(u16*) p_check_address;
	u16 num_reg = *(u16*) ((u16*) p_check_address + 1);
	// адрес принадлежит области регистров ТY
	if ((address >= s_address_oper_data.s_TY_address.present_state_TY)&&
			((address + num_reg - 1)< (s_address_oper_data.s_TY_address.present_state_TY + TOTAL_NUM_TY * NUM_REG_TY)))
	{
		return REQ_SLAVE_OK;
	}
	// адрес находиться за границами допустимых адресов регистров ТС
	return ILLEGAL_DATA_ADRESS;
}

//------------функция processing_TY_signal_address_check-----------------------------------
// функция processing_TY_signal_address_modbus_check - выполняет проверку адресса карты памяти на принадлежность регистрам установки ТУ
// даная функция соответствует проверке команде №5 modbus
// входные аргументы:
// *p_check_address - указатель на начало последовательности (u16)address
// выходные аргументы:
// REQ_SLAVE_OK - проверкаа выполнена успешно
// ILLEGAL_DATA_ADRESS - недопустимый адрес
REZ_REQ_CHEACK_SLAVE processing_TY_check_force_single_coil_address_modbus(void* p_check_address) {
	u16 address = *(u16*) p_check_address;
	// адрес принадлежит области установки регистров ТУ
	if ((address >= s_address_oper_data.s_TY_address.set_state_TY)
			&& (address	< (s_address_oper_data.s_TY_address.set_state_TY+ TOTAL_NUM_TY)))
	{
		return REQ_SLAVE_OK;
	}
	// адрес находиться за границами допустимых адресов регистров
	return ILLEGAL_DATA_ADRESS;
}

//---------------функция processing_TY_signal_operation---------------
// функция processing_TY_signal_operation - ставит в очередь на выполнение команду ТУ
u8 processing_TY_signal_operation(void* req, u8 num_peyload_data,u16 addres_data) {
	S_ty_comand *ps_comand = req;
	// ВСЕ ПРОВЕРКИ ВЫПОЛНЕНЫ КОГДА БЫЛА ПРИНЯТА КОМАНДА №5. !!!!
	// ставим в очередь
	s_TY_operation.num_TY = ps_comand->number - s_address_oper_data.s_TY_address.set_state_TY; // вычисляем порядочный номер выхода ТУ (0-(TOTAL_NUM_TY-1))
	s_TY_operation.state_TY = ps_comand->state;
	s_TY_operation.f_TY = REDY_COMAND;
	return (0);
}

//---------------функция processing_TY_signal_clear---------------
// функция processing_TY_signal_clear - очистить очередь на выполнение команду ТУ
void processing_TY_signal_clear(void) {
	s_TY_operation.num_TY = 0;
	s_TY_operation.state_TY = 0;
	s_TY_operation.f_TY = 0;
}

//---------------функция processing_TY_signal_DP_TY---------------
void processing_TY_signal_DP_TY(u8 num_ty, S_state_TY* p_status_TY) {
	u8 num_paralel_ty;
	// Если установлен хотябы один не допускающий выполнение команды флаг - выход
	if (STATUS_STATE(p_status_TY->status_TY))
	{
		return;
	}
	// переходим к выполнению команды
	if (s_TY_operation.state_TY == TY_OFF) // если необходимо выполнить операцию выключения
	{
		num_ty++; // перейти на выход выключения
	}
	//
	//--------- ВЫПОЛНИТЬ ОПЕРАЦИЮ УСТАНОВКИ РЕЛЛЕ ТУ ---------------
	// включить заданное релле
	TY_SET_OUT(s_rel_out[num_ty].port_operation,
			s_rel_out[num_ty].pin_operation);
	// паралельный канал комутируеться только с операцией ВЫКЛЮЧЕНИЯ
	if ((ps_TY_user_config->s_TY_out_config[num_ty].f_paralel_out)&& (s_TY_operation.state_TY == TY_OFF))
	{
		num_paralel_ty = ps_TY_user_config->s_TY_out_config[num_ty].num_paralel	- s_address_oper_data.s_TY_address.set_state_TY;
		TY_SET_OUT(s_rel_out[num_paralel_ty].port_operation,s_rel_out[num_paralel_ty].pin_operation);// включаю релле паралельного канала !!!!!!
	}
	// выполнить проверку включения релле ТУ
	if (processing_TY_signal_chek_processing(&processing_TY_signal_check_ty_rel_on, ERROR_TY_REL_ON, p_status_TY,num_ty))
	{
		processing_TY_signal_disable_rellay(num_ty); // отключаю выходное релле ТУ и груповое релле
		return;
	}

	//--------- ВЫПОЛНИТЬ ОПЕРАЦИЮ ВКЛЮЧЕНИЯ ГУПОВОГО РЕЛЛЕ ---------------
	// включить груповое релле
	TY_SET_OUT(PORT_GRYP_REL, PIN_GRYP_REL);
	// выполнить проверку включения групового релле
	if (processing_TY_signal_chek_processing(&processing_TY_signal_check_gryp_rel_on, ERROR_GRYP_REL_ON,p_status_TY, num_ty))
	{
		processing_TY_signal_disable_rellay(num_ty); // отключаю выходное релле ТУ и груповое релле
		return;
	}

	// Запускаю таймер для отсчета времени равного длине импульса ТУ типа DP
	TIM_Cmd(TIM2,ENABLE);
	while(1){
		// пытаюсь захватить семафор, который отдаеться из TIM1. Период ожидания равен
		// времени между проверками котушки
		xSemaphoreTake(SemaphoreHandle_ty, portMAX_DELAY); // жду следующего акта проверки котушек
		if(f_dp_time_complite)// если пройшло время импульса, выход
		{
			f_dp_time_complite=0;
			break;
		}
		// запускаю ф-ю проверки состояния котушек релле. Если проверка неудачная - выход из ф-и
		if(processing_TY_signal_chek_processing(&processing_TY_signal_check_coil,ERROR_TY_COIL,p_status_TY,num_ty))
		{
			processing_TY_signal_disable_rellay(num_ty);
			return;
		}
	}

	//--------- ОПЕРАЦИЯ ВЫКЛЮЧЕННИЯ НРУПОВОГО РЕЛЛЕ ВЫПОЛНЯЕТЬСЯ В ПРЕРЫВАНИИ ТАЙМЕНА ---------------
	// выполнить проверку выключения групового релле
	if (processing_TY_signal_chek_processing(&processing_TY_signal_check_gryp_rel_off, ERROR_GRYP_REL_OFF,p_status_TY, num_ty))
	{
		processing_TY_signal_disable_rellay(num_ty); // отключаю выходное релле ТУ и груповое релле
		return;
	}

	//--------- ВЫПОЛНИТЬ ОПЕРАЦИЮ СБРАСЫВАНИЯ РЕЛЛЕ ТУ --------------
	// выключить заданное релле, и паралельное (если задано пользователем и пришла команда DP_OFF)
	TY_CLEAR_OUT(s_rel_out[num_ty].port_operation,
			s_rel_out[num_ty].pin_operation);
	if ((ps_TY_user_config->s_TY_out_config[num_ty].f_paralel_out)
			&& (s_TY_operation.state_TY == TY_OFF))
	{
		TY_CLEAR_OUT(s_rel_out[num_paralel_ty].port_operation,
				s_rel_out[num_paralel_ty].pin_operation);
	}
	// выполнить проверку выключения релле ТУ
	if (processing_TY_signal_chek_processing(&processing_TY_signal_check_ty_rel_off, ERROR_TY_REL_OFF,p_status_TY, num_ty))
	{
		processing_TY_signal_disable_rellay(num_ty); // отключаю выходное релле ТУ и груповое релле
		return;
	}
}

//---------------функция processing_TY_signal_chek_processing---------------
// функция processing_TY_signal_chek_processing - выполняет операцию проверки выполнения одного из ступеней ТУ и обработку результатов выполнеия
// Входные аргументы:
// p_fun_chek      - указатель на ф-ю проверки;
// paramiter_chek  - флаг на бит результата проверки
// status          - указатель на переменную в которой будет установлен флаг результата проверки
// Выходные аргументы:
// 0 - проверка окончилась успешно
// 1 - проверка окончилась исключением
u8 processing_TY_signal_chek_processing(F_check p_fun_chek,TY_REZ_CHECK paramiter_chek, S_state_TY* p_status, u8 num_ty) {
	TY_REZ_CHECK rez_chek_ty;
	// если ф-я проверки находиться ы списке исключения - положительный выход
	if(IS_FUN_EXCEPTION(p_fun_chek)){return (0);}
	// выполнить проверку
	rez_chek_ty = p_fun_chek();
	(p_status->status_TY) &= ~(paramiter_chek); // обнуляю статус проверяимого параметра
	(p_status->status_TY) |= rez_chek_ty; // устанавливаю статус согласно результатам проверки

	// если была передана команда на выключение, значит номер ТУ был увеличен на 1. Если выполняеться
	// команда типа DP, для коректной записи статуса данного ТУ нужно уменшить номер ТУ
	if (s_TY_operation.state_TY == TY_OFF) { // если необходимо выполнить операцию выключения
		num_ty--; // уменшить номер ТУ
	}
	if (rez_chek_ty)// если проверка окончилась исключением
	{
		// Выполнить запись в регистры состояния выходов ТУ
		p_status->present_state_TY[num_ty] = TY_NOT_SET; //состояние не опредилено ОШИБКА
		p_status->operation_TY_statys[num_ty] |= rez_chek_ty;
		return (1);

	} else {
		// Выполнить запись в регистры состояния выходов ТУ
		p_status->present_state_TY[num_ty] = p_status->set_state_TY[num_ty]; //поточное состояние выхода ТУ
		//p_status->operation_TY_statys[num_ty] |= rez_chek_ty;
		return (0);
	}
}

//---------------функция processing_TY_signal_disable_rellay---------------
// функция processing_TY_signal_disable_rellay - выполняет отключение всех релле которые комутируються в процесе выполнения ТУ и групового релле
// Входные аргументы:
// num_ty      - номер комутируемого релле ТУ;
void processing_TY_signal_disable_rellay(u8 num_ty) {
	u8 num_paralel_ty;
	TY_CLEAR_OUT(s_rel_out[num_ty].port_operation,
			s_rel_out[num_ty].pin_operation);
	// паралельный канал комутируеться только с операцией ВЫКЛЮЧЕНИЯ
	if ((ps_TY_user_config->s_TY_out_config[num_ty].f_paralel_out)
			&& (s_TY_operation.state_TY == TY_OFF))
	{
		num_paralel_ty = ps_TY_user_config->s_TY_out_config[num_ty].num_paralel
				- s_address_oper_data.s_TY_address.set_state_TY;
		TY_CLEAR_OUT(s_rel_out[num_paralel_ty].port_operation,
				s_rel_out[num_paralel_ty].pin_operation);
		// включаю релле паралельного канала !!!!!!
	}

	TY_CLEAR_OUT(PORT_GRYP_REL, PIN_GRYP_REL);
	// отключить груповое релле
}

//---------------функция processing_TY_signal_SP_TY---------------
// функция processing_TY_signal_SP_TY - выполняет TY типа SP
void processing_TY_signal_SP_TY(u8 num_ty, S_state_TY* status_TY) {
	//ДЛЯ ТУ ВЫХОДОВ ТИПА SP ВЫПОЛНЯЕТЬСЯ ПРОВЕРКА:
	// - для режима с проверками НАЛИЧИЯ НАПРЯЖЕНИЯ ОППЕРАТИВНОГО ТОКА, СОСТОЯНИЯ КОТУШЕК РЕЛЛЕ
	// - для режима с проверками СОСТОЯНИЯ КОТУШЕК РЕЛЛЕ
	if (status_TY->status_TY & ((ERROR_V_INPUT) | (ERROR_TY_COIL))) {
		return;
	}
	if (s_TY_operation.state_TY == TY_ON) { // если необходимо выполнить операцию включения
		TY_SET_OUT(s_rel_out[num_ty].port_operation,s_rel_out[num_ty].pin_operation);
	}
	else
	{
		TY_CLEAR_OUT(s_rel_out[num_ty].port_operation,s_rel_out[num_ty].pin_operation);
	}
	status_TY->present_state_TY[num_ty] = status_TY->set_state_TY[num_ty];
	status_TY->operation_TY_statys[num_ty] = REZ_CHECK_OK;
}

//---------------функция processing_TY_signal_check_V---------------
// функция processing_TY_signal_check_V - выполняет проверку наличия оперативного тока в линии
TY_REZ_CHECK processing_TY_signal_check_v(void) {
	u8 state = 0;
	// подключаю измерительную цепь
	TY_SET_OUT(PORT_V_REL, PIN_V_REL);
	vTaskDelay(2); // переходной процес подключения
    state = processing_TY_signal_read_v_input(); // считваю состояние напряжение опер тока
	TY_CLEAR_OUT(PORT_V_REL, PIN_V_REL);
	// отключаю измерительную цепь
	vTaskDelay(1); // переходной процес отключения
	if (state)
	{
		return REZ_CHECK_OK;
	} // если напряжение опер тока установлено - положительный результат
	else
	{
		return ERROR_V_INPUT;
	}
}

//---------------функция processing_TY_signal_read_v_input---------------
// функция processing_TY_signal_read_v_input - выполняет считывание пина напряжения опер тока (как переменного так и постоянного)
TY_STATE_V_INPUT processing_TY_signal_read_v_input(void) {
	u8 state = TY_READ_IN_0;
	u8 count_ms;
	u8 count_set_mes = 0; // счетчик к-ва измерений из  из положительным исходом (напряжение опер тока присутствует)
	for (count_ms = 0; count_ms < NUM_MES_V; count_ms++)
	{
		//                  нету есть
		// на ножке          1    0
		// выход ф-и         0    1
		if (!GPIO_ReadInputDataBit(PORT_V_INPUT, PIN_V_INPUT)) // лог 1 на ножке (напряжения нету)
		{
			count_set_mes++;
		}
		// если количество измерений напряжения из положительным исходом больше заданого - значит
		// значит напряжение опер тока присутствует
		if (count_set_mes >= 4 )
		{
			state = TY_READ_IN_1;
			return state;
			break;
		}
		vTaskDelay(TIME_MES_V);
	}
	return state;
}

//---------------функция processing_TY_signal_check_rel_contact---------------
// функция processing_TY_signal_check_rel_contact - выполняет проверку залипания контактов релле
TY_REZ_CHECK processing_TY_signal_check_rel_contact(void) {
	u8 state = 0;
	TY_CLEAR_OUT(PORT_V_REL, PIN_V_REL);
	// отключаю измерительную цепь
	vTaskDelay(2); // переходной процес подключения
	state = processing_TY_signal_read_v_input(); // считваю состояние напряжение опер тока
	if (!(state))
	{
		return REZ_CHECK_OK;
	} // если напряжение опер тока не установлено - положительный результат
	else
	{
		return ERROR_TY_REL_CONTACT;
	}
}

//---------------функция processing_TY_signal_check_ty_rel_on---------------
// функция processing_TY_signal_check_ty_rel_on - выполняет проверку включения релле управления TY
TY_REZ_CHECK processing_TY_signal_check_ty_rel_on(void) {
	u8 state = 0;
	TY_CLEAR_OUT(PORT_V_REL, PIN_V_REL);
	// отключаю измерительную цепь
	vTaskDelay(TIME_REL_COMUTATION); // переходной процес подключения
	state = processing_TY_signal_read_v_input(); // считваю состояние напряжение опер тока
	if (state)
	{
		return REZ_CHECK_OK;
	} // если напряжение опер тока установлено - положительный результат
	else
	{
		return ERROR_TY_REL_ON;
	}
}

//---------------функция processing_TY_signal_check_gryp_rel---------------
// функция processing_TY_signal_check_gryp_rel - выполняет проверку включения групового релле
TY_REZ_CHECK processing_TY_signal_check_gryp_rel_on(void) {
	u8 state = 0;
	vTaskDelay(TIME_ELEKTRIK_KEY_COMUTATION); // переходной процес подключения групового релле
	state = processing_TY_signal_read_v_input(); // считваю состояние напряжение опер тока
	if (!(state))
	{
		return REZ_CHECK_OK;
	} // если напряжение опер тока не установлено - положительный результат
	else
	{
		return ERROR_GRYP_REL_ON;
	}
}

//---------------функция processing_TY_signal_check_gryp_rel---------------
// функция processing_TY_signal_check_gryp_rel - выполняет проверку выключения групового релле
TY_REZ_CHECK processing_TY_signal_check_gryp_rel_off(void) {
	u8 state = 0;
	vTaskDelay(TIME_ELEKTRIK_KEY_COMUTATION); // переходной процес подключения
	state = processing_TY_signal_read_v_input(); // считваю состояние напряжение опер тока
	if (state)
	{
		return REZ_CHECK_OK;
	} // если напряжение опер тока установлено - положительный результат
	else
	{
		return ERROR_GRYP_REL_OFF;
	}
}

//---------------функция processing_TY_signal_check_gryp_rel---------------
// функция processing_TY_signal_check_gryp_rel - выполняет проверку выключения релле
TY_REZ_CHECK processing_TY_signal_check_ty_rel_off(void) {
	u8 state = 0;
	vTaskDelay(TIME_REL_COMUTATION); // переходной процес подключения
	state = processing_TY_signal_read_v_input(); // считваю состояние напряжение опер тока
	if (!(state))
	{
		return REZ_CHECK_OK;
	} // если напряжение опер тока не установлено - положительный результат
	else
	{
		return ERROR_TY_REL_OFF;
	}
}

// ---------функция  TY_calc_address_oper_reg ---------
// функция TY_calc_address_oper_reg - выполняет рассчет адресов регистров (область оперативных регистров) устройства маршрутизации данных
// входные аргументы:
// ps_TY_address           - указатель на структуру адресов регистров устройства (поле в глобальной структуре адресов устройства);
// adres_start             - начальный адрес оперативных регистров устройства в карте памяти. относительно него расчитываються адрес а вех остальных регистров устройства
// выходные аргументы:
//                         - адрес последнего регистра устройства
u16 TY_calc_address_oper_reg(S_TY_address *ps_TY_address, u16 adres_start) {
	ps_TY_address->status_TY = adres_start;
	ps_TY_address->present_state_TY = ps_TY_address->status_TY
			+ NUM_REG_STATUS_TY;
	ps_TY_address->set_state_TY = ps_TY_address->present_state_TY + TOTAL_NUM_TY;
	ps_TY_address->operation_TY_statys = ps_TY_address->set_state_TY
			+ TOTAL_NUM_TY;
	adres_start = ps_TY_address->operation_TY_statys + TOTAL_NUM_TY;
	return adres_start;
}
;

//---------------задача processing_TY---------------
// звдача processing_TY - выполняет:циклическую проверку наличия опер тока (согласно настройкам),
// обработку команды ТУ...
// состояние ON – константа 0xFF00;
// состояние OFF – константа 0x0000.
// (все остальные значения констант являются некорректными,
// не приведут к изменению состояния обмотки,  вызовут ответ об ошибке значения в поле данных запроса).
void t_processing_TY(void *pvParameters) {
	u8 k1;
	TY_REZ_CHECK rez_chek_ty_modul;

	// сохраняю конфигурационные данные в указателе на структуру конфигурации,
	// а также выпоняю проверку
	if (processing_TY_fill_S_TY((u8*) pvParameters))
	{
		// делаю запись в статус регистр устройства
		SET_GLOBAL_STATUS(DEV_3);
		// выполняю запись в статус регистр ТУ

		vTaskDelete(NULL); // если проверка не пройдена - удалить задачу + аврийная сигнализация + ЗАПИСЬ В КАРТУ ПАМЯТИ !!!!
	};

	// продолжаю конфигурацию устройства
	//конфигурирую програмный блок ТУ
	processing_TY_signal_init();
	while (1)
	{
		//status_TY=0;
		//mem_map_processing_read_s_proces_object_modbus(&status_TY,1,s_address_oper_data.s_oper_data_TY.status_TY);
		//стандартная циклическая проверка указаными функциями
		for (k1 = 0; k1 < s_cycling_check.num_cycling_check; k1++) {
			if(IS_FUN_EXCEPTION( s_cycling_check.pf_fun[k1].pf_check)){continue;}
			rez_chek_ty_modul = s_cycling_check.pf_fun[k1].pf_check();
			s_oper_data_TY_present.status_TY &=~(s_cycling_check.pf_fun[k1].maska_error); // обнуляю статус проверяимого параметра
			s_oper_data_TY_present.status_TY |= rez_chek_ty_modul; // устанавливаю статус результата проверки
		}
		// если была принята команда ТУ и статус регистр пустой (блок функционирует нормально)
		if ((!(s_oper_data_TY_present.status_TY)) && (s_TY_operation.f_TY)) {
			//if((s_TY_operation.f_TY)){ // ТОЛЬКО ДЛЯ ПРОВЕРКИ РАСКОМЕНТИРОВАТЬ ВЕРХНЮЮ СТРОКУ
			// выставляю статус неопредиленного сотояния выхода ТУ, в промежутке времени между
			// началом выполнения команды ТУ, и окончанием выполнения
			s_oper_data_TY_present.set_state_TY[s_TY_operation.num_TY] =s_TY_operation.state_TY;
			s_oper_data_TY_present.present_state_TY[s_TY_operation.num_TY] =TY_NOT_SET;
			//s_oper_data_TY_present.operation_TY_statys[s_TY_operation.num_TY]=TY_NOT_DEF;
			processing_TY_signal_update_TY_state(&s_oper_data_TY_present);
			// выбираю тип ТУ из установок пользователя и выполняю переключения
			switch (ps_TY_user_config->s_TY_out_config[s_TY_operation.num_TY].mode_TY)
			{
				case SINGLE_POSITION:processing_TY_signal_SP_TY(s_TY_operation.num_TY,&s_oper_data_TY_present);
					break;
				case DOUBLE_POSITION:processing_TY_signal_DP_TY(s_TY_operation.num_TY,&s_oper_data_TY_present);
					break;
				default:
					break;
			}
			processing_TY_signal_clear();
		}
		// обновляю статус регистр модуля ТУ в карте памяти и флеш памяти
		processing_TY_signal_update_TY_state(&s_oper_data_TY_present);

	}
}

//----------------функция processing_TY_signal_update_TY_state-------------------------------
// функция processing_TY_signal_update_TY_state - обновляет статус регистры модуля ТУ
// Перезапись регистроЕсли регистры модуля ТУ до
void processing_TY_signal_update_TY_state(S_state_TY *ps_present) {
	u8 counter;
	u16 *p_present = (u16*) ps_present; // предыидущее состояние регистра
	u16 *p_previous; // поточное состояние регистра (которое хочу записать)
	S_state_TY s_oper_data_TY_previous;

	// считываю из карты памяти статус-регистры состояния модуля ТУ
	processing_mem_map_read_s_proces_object_modbus((u16*)&s_oper_data_TY_previous, sizeof(S_state_TY) / 2,s_address_oper_data.s_TY_address.status_TY);
	p_previous=(u16*)&s_oper_data_TY_previous;
	// сравниваю статус-регисстры поточного состояния и предидущего.
	// Если состояние изменилось  - запись в глобальный статус-регистр устройства
	if(s_oper_data_TY_previous.status_TY!=ps_present->status_TY)
	{
		if(ps_present->status_TY)
		{
			SET_GLOBAL_STATUS(DEV_3);
		}
		else{
			RESET_GLOBAL_STATUS(DEV_3);
		}
	}
	// Сравниваю регистры предыдущего состояния модуля ТУ и поточного, если найдено не совпадение - перезаписать ВСЕ регистры модуля ТУ
	for (counter = 0; counter < (sizeof(S_state_TY) / 2); counter++) { // /2 - потому, что sizeof вовращает размер в байтах, а регистры мы проверяем в 2-х байтном формате
		if ((*p_present) != (*p_previous)) { // если регистр поточного статуса и установленного статуса не равны
			// перезапись в карту памяти всех регистров модуля ТУ
			processing_mem_map_write_s_proces_object_modbus((u16*)ps_present, (sizeof(S_state_TY) / 2),s_address_oper_data.s_TY_address.status_TY );
            break;
		}
		p_present++; // перехожу к следуюющим регистрам
		p_previous++;
	}
    //Сравниваю предыидущее и поточное состояние выходов ТУ типа SP
	for(counter=0;counter<TOTAL_NUM_TY;counter++)
	{
		// если изменилось состояние выхода SP "И" напряжение оперативного тока в норме - перезапись
		// Перезаписываеться вся страница
		if((!((ps_present->status_TY)&(ERROR_V_INPUT)))&&\
				(ps_present->present_state_TY[counter]!=s_oper_data_TY_previous.present_state_TY[counter])&&\
				(ps_TY_user_config->s_TY_out_config[counter].mode_TY==SINGLE_POSITION)
				)
		{
			// нужно устранить влияние записи во флеш на другие прерывания ((
			FLASH_OPERATION_write_flash_page_16b((u16*) ps_present,sizeof(S_state_TY) / 2, TY_FLASH_PAGE);
			break;
		}

	}
}

