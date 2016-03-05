/*
 *  processing_mesuremen_adc.h
 *
 *  Created on: December 21, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "processing_mesuremen_adc.h"



//----------u8 config_ADC(ADC_TypeDef *ADC_IN, S_config_ADC s_config_ADC)------------------
//функция config_ADC. Выполняет начальную конфигурация АЦП и портов ввода/вывда согласно списку входных аргументов.
//по умолчанию настраиваються:
// - устанавливаеться коэффициент деления для всех АЦП
// - отбор данных с помощью DMA
// - непрерывный режим (для количества каналов равному 1), режим сканирования (для количества каналов >1)
// - последовательные преобразования без остановки
// - конфигурируеться только група регулярных каналов
// - НЕ конфигурируеться поток DMA для отбора данных (конфигурацию выполнить в отдельной ф-и)
// ***********************************ВХОДНЫЕ АРГУМЕНТЫ******************************
// - ADC_IN - указатель на АЦП который необходимо конфигурировать. принимает значения ADC1...3
// - s_config_ADC - структура с настроечными параметрами.
//                  Поля структуры s_config_ADC:
//              - u16 ADCPRE (0-3     - коэффициент деления частоты тактирования АЦП (частота тактирования АЦП называеться ADCCLK)
//                                      АЦП тактируються от шины APB2 (максимальная частота F_APB2 = 84 мГц). Используя  ADCPRE можно уменшить частоту тактирования АЦП
//                                      и, соответствено, скорость поступления отсчетов. К-т деления задаеться для всех АЦП
//                                      ADCPRE задаеться из списка:
//                                     к-т деления (ADCPRE)      аргумент
//                                              2                @ref    RCC_PCLK2_Div2
//                                              4                @ref    RCC_PCLK2_Div4
//                                              6                @ref    RCC_PCLK2_Div6
//                                              8                @ref    RCC_PCLK2_Div8
//              - u8  quantity_chanel (1-16) - количество регулярных каналов.
//              - u16 num_chanel[16] (0-16)  - масив с номерами регулярных каналов. Соответстыие номера канала и пина контроллера приведено ниже
//              ____________________________________________________________________________________________________________________________________________
//-номер канала |  0  |  1  |  2  |  3  |    4    |    5    |    6    |    7    |    8    |    9    | 10  |  11  |  12  |  13  |  14  |  15  |  16  |  17  |  18  |
//-пин          | PA0 | PA1 | PA2 |PA3  | PA4(1,2)| PA5(1,2)| PA6(1,2)| PA7(1,2)| PB0(1,2)| PB1(1,2)| PC0 |  PC1 | PC2  | PC3  | PC4  | PC5  |tempr | Vref | Vbat |
//
//              - u16 time_mes[16] (0-7) - количество тактов ADCCLK для ка4ждого канала из списка num_chanel.
//                                      Количество тактов задаеться из списка:
//                                     к-тво (CH_SMPR)     аргумент
//                                       1,5+12,5          @ref ADC_SampleTime_1Cycles5
//                                       7,5+12,5          @ref ADC_SampleTime_7Cycles5
//                                      13,5+12,5          @ref ADC_SampleTime_13Cycles5
//                                      28,5+12,5          @ref ADC_SampleTime_28Cycles5
//                                      41,5+12,5          @ref ADC_SampleTime_41Cycles5
//                                      55,5+12,5          @ref ADC_SampleTime_55Cycles5
//                                      71,5+12,5          @ref ADC_SampleTime_71Cycles5
//                                     239,5+12,5          @ref ADC_SampleTime_239Cycles5
//                                      Номер элемента масива time_mes соответствует количеству тактов ADCCLK
//                                      каналу под таким же номером элемента в масиве num_chanel. Напрмер: num_chanel[3]={3,7,2}
//                                                                                                         time_mes[3]  ={2,4,7}
//                                                                         время преобразование для канала 3 (первый в списке) равно 2 и т. д.
//                                      Абсолютное время преобразование для одного отсчета расчитываетьсяз формулой:
//                                        Time_mes=(CH_SMPR)/(F_APB2/ADCPRE)
//                                      соответствено частота преобразования:
//                                        F_Time_mes=1/Time_mes
// ***********************************ВЫХОДНЫЕ АРГУМЕНТЫ******************************
// 1 - ERROR_PSC                  - превышен предделитель АЦП
// 2 - ERROR_MAX_QUANTITY_CHANEL  - неверно задано максимальное количество каналов. Диапазон 1-16
// 3 - ERROR_POIN_ADC             - неверный указатель на блок АЦП. Диапазон ADC1, ADC2, ADC3
// 4 - ERROR_NUMBER_CHANEL        - превышен номер канала. Диапазон 0-16
// 5 - ERROR_SAMPLE_TIME          - превышено время преобразования канала диапазон 0-7

ADC_CONF_ERROR processing_mesuremen_adc_conf_channel(ADC_TypeDef *ADC_IN, S_config_ADC s_config_ADC) {
	u8 counter;
	u8 time_mesurem;
	u8 number_chanel;
	GPIO_TypeDef *adc_gpio;
	GPIO_InitTypeDef gpio_InitTypeDef;
	//*********проверка входных аргументов************************
	// проверка количества каналов. ДОПУСТИМЫЕ ЗНАЧЕНИЯ: 1 - 16
	if ((s_config_ADC.quantity_chanel > MAX_QUANTITY_CHANEL)|| (s_config_ADC.quantity_chanel < MIN_QUANTITY_CHANEL))
	{
		return ERROR_MAX_QUANTITY_CHANEL;
	}
	// Устанавливаю общий предделитель АЦП
	if(!IS_RCC_ADCCLK(s_config_ADC.comon_adc_psc)){return ERROR_COMON_ADC_PSC;};
	RCC_ADCCLKConfig(s_config_ADC.comon_adc_psc);
	//включаю тактирование альтернативных ф-й
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	//*********конфигурация АЦП************************
	// Включаем тактирование АЦП

	if (ADC_IN == ADC1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	}
	else if (ADC_IN == ADC2)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,ENABLE);
	}
#ifdef STM32F10X_HD
	else if (ADC_IN == ADC3)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3,ENABLE);
	}
#endif
	else
	{
		return ERROR_POIN_ADC;
	};
	ADC_IN->CR2 |= (ADC_CR2_ADON);
	// Обнуляем все настраиваимые регистры
	ADC_IN->CR1 = 0;
	ADC_IN->SMPR1 = 0;
	ADC_IN->SMPR2 = 0;
	ADC_IN->SQR1 = 0;
	ADC_IN->SQR2 = 0;
	ADC_IN->SQR3 = 0;
	//---------устанавливаем заданое количество преобразований---------------------
	ADC_IN->SQR1 |= ((s_config_ADC.quantity_chanel - 1) << 20); // на 1 менше количества каналов
	//--------------------------------------------------------------------------------
	// ----------КОНФИГУРИРУЕМ ПОРЯДОК СЛЕДОВАНИЯ КАНАЛОВ-----ВРЕМЯ ПРЕОБРАЗОВАНИЯ ДЛЯ КАЖДОГО КАНАЛА-----------------------
	for (counter = 1; counter <= s_config_ADC.quantity_chanel; counter++) {
		number_chanel = s_config_ADC.num_chanel[counter - 1];
		time_mesurem = s_config_ADC.time_mes[counter - 1];
		if (number_chanel > MAX_QUANTITY_CHANEL) { // проверка коректности номера канала
			return ERROR_NUMBER_CHANEL;
		};
		if(!IS_ADC_SAMPLE_TIME(time_mesurem ))// проверка коректности время преобразования
		{
			return ERROR_SAMPLE_TIME;
		};
		//Настраиваю номер каналов
		if ((counter >= 1) && (counter <= 6))
		{
			ADC_IN->SQR3 |= (number_chanel << ((counter - 1) * SIZE_1_SQR));
		}
		else if ((counter >= 7) && (counter <= 12))
		{
			ADC_IN->SQR2 |= (number_chanel << ((counter - 7) * SIZE_1_SQR));
		}
		else if ((counter >= 13) && (counter <= 16))
		{
			ADC_IN->SQR1 |= (number_chanel << ((counter - 13) * SIZE_1_SQR));
		}
		//Настраиваю время преобразования каналов
		if ((number_chanel >= 0) && (number_chanel <= 9))
		{
			ADC_IN->SMPR2 |= (time_mesurem << (number_chanel * SIZE_1_SMPR));
		}
		else if ((number_chanel >= 10) && (number_chanel <= 18))
		{
			ADC_IN->SMPR1 |= (time_mesurem << ((number_chanel - 10) * SIZE_1_SMPR));
		};
	}; // end for(counter)
	//--------------------------------------------------------------------------------
	// ----------КОНФИГУРИРУЕМ GPIO---------------------------------------------------
	for (counter = 1; counter <= s_config_ADC.quantity_chanel; counter++) {
		number_chanel = s_config_ADC.num_chanel[counter - 1];
		switch (number_chanel) {
		case (0): // Каналы порта А
		case (1):
		case (2):
		case (3):
		case (4):
		case (5):
		case (6):
		case (7):
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		adc_gpio = GPIOA;
		if (number_chanel == 0)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_0;
		}
		else if (number_chanel == 1)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_1;
		}
		else if (number_chanel == 2)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_2;
		}
		else if (number_chanel == 3)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_3;
		}
		else if (number_chanel == 4)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_4;
		}
		else if (number_chanel == 5)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_5;
		}
		else if (number_chanel == 6)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_6;
		}
		else if (number_chanel == 7)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_7;
		};
		break;
		case (8): // Каналы порта В
		case (9):
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		adc_gpio = GPIOB;
		if (number_chanel == 8) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_1;
		} else if (number_chanel == 9) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_0;
		}
		;
		break;
		case (10): // Каналы порта С
		case (11):
		case (12):
		case (13):
		case (14):
		case (15):
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
		adc_gpio = GPIOC;
		if (number_chanel == 10) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_0;
		} else if (number_chanel == 11) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_1;
		} else if (number_chanel == 12) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_2;
		} else if (number_chanel == 13) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_3;
		} else if (number_chanel == 14) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_4;
		} else if (number_chanel == 15) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_5;
		};
		break;
		default:
			break;
		};
		gpio_InitTypeDef.GPIO_Speed=GPIO_Speed_2MHz;
		gpio_InitTypeDef.GPIO_Mode=GPIO_Mode_IN_FLOATING;
		GPIO_Init(adc_gpio,&gpio_InitTypeDef);
	} // end for(counter)

	return REZ_OK;
}

//-----------processing_mesuremen_conf_mod_adc---------------
// FUNCTION processing_mesuremen_conf_mod_adc - make ADC mode configuration:
// - ADC continious mode
ADC_CONF_ERROR processing_mesuremen_adc_conf_mod(S_dma_run_param *ps_adc_buf_par){
	u32 count_wait_calib_reset=0;
	ADC1->CR1=0;
	ADC1->CR2=0;
	// calibration ADC
	ADC1->CR2|=(ADC_CR2_CAL);
	//wait reset calibration bit reset
	while(((ADC1->CR2)|(ADC_CR2_CAL))&&(count_wait_calib_reset<WAIT_RESET_CALIB)){
		count_wait_calib_reset++;
	}
	// if self calibration not successful
	if(count_wait_calib_reset>=WAIT_RESET_CALIB)
	{
		return ERROR_CALIB;
	}
	//config mode
	ADC1->CR2|=((ADC_CR2_CONT)|(ADC_CR2_DMA));
	// set DMA num data and mem address
	processing_mesuremen_dma_mem(ps_adc_buf_par);
	return REZ_OK;
}


//-----------processing_mesuremen_run_adc---------------
// function processing_mesuremen_run_adc - run ADC1
void processing_mesuremen_adc_run(void){
	processing_mesuremen_dma_run_adc(); // run ADC DMA

	ADC1->CR2|=ADC_CR2_ADON;            // run ADC
}




