/*
 * processing_mesurement.c
 *
 *  Created on: March 17, 2016
 *      Author: Gerasimchuk
 */

#include "processing_mesurement.h"
#include "processing_mesurement_extern.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "math.h"
#include "string.h"

// Проверяем допустимые настройки соотношения частоты АЦП и размера буффера АЦП
#if (BPF_F/ADC_BUFFER_SIZE_HALF)*ADC_BUFFER_SIZE_HALF!=BPF_F
#error "processing_mesurement" init: BPF_F/ADC_BUFFER_SIZE_HALF must bee integer
#endif

S_ADC_init s_adc;
S_Buffer_result s_buff_adc;
u16 buff_adc[ADC_BUFFER_SIZE_FULL]={[0 ... (ADC_BUFFER_SIZE_FULL-1)]=0}; // буффер результатов АЦП
u16 buff_to_filtring[SIZE_BUFF_TO_FILTRING]={[0 ... (SIZE_BUFF_TO_FILTRING-1)]=0}; // буффер данных к фильтрации
s32 buff_rez_filtring[ADC_BUFFER_SIZE_HALF*2]={[0 ... (ADC_BUFFER_SIZE_HALF*2-1)] = 0};// буффер результатов фильтрации действительная и мнимая части перемежаются
S_buff_rez s_buff_rez;      // буффер результатов измерений


//структура параметров фильтра
S_par_filters filter_par;
S_coef_filter_integer s_coef_filter;
// для проверки
    #define SIZE_ARRAY_TIME   10
	#define SIZE_SIN      120
	u16 a_sin[SIZE_SIN];
	s32 rez_filt[SIZE_SIN*3];



void check_filter(void) {
	// create sin-signal array
	data_operation_sin(&a_sin[0],SIZE_SIN,50,1000,0,filter_par.F_adc);

	//---------made few cycles filtration ------------------

	u16 counter_time;
	u16 array_time[SIZE_ARRAY_TIME];
	//filtration
	for(counter_time=0;counter_time<SIZE_ARRAY_TIME;counter_time++){
			fun_proces_filterin_integer(&a_sin[0],SIZE_SIN, 1,&rez_filt[0], filter_par.Q1, &s_coef_filter);
	}
}



//---------------функция data_operation_sin ------------------
// функция data_operation_sin - генерирует синус-выборку
// входные аргументы:
//*pa_sin - указатель на начльные элемент масива
// length - размер выборки
// f1     - частота синус-функции
// Am     - амплитуда синус-функции
// fi     - начальная фаза синус-функции
// Fadc   - частота дискретизации выборки
void data_operation_sin(u16 *pa_sin, u16 length, float f1, float Am, float fi,float Fadc){
	u16 counter;
	float Dt=1/Fadc;
	for(counter=0;counter<length;counter++){
		(*pa_sin)=(u16)(Am*sinf(2*M_PI*f1*(counter*Dt)+fi)+Am);
		pa_sin++;
	}
}




//---------------задача processing_mesurement_global_config ---------
void processing_mesurement_global_config(){
//---------------------------------------------------------------
//---------------------НАСТРОЙКА FIFO-----------------------------
//---------------------------------------------------------------
	fifo_init(&s_buff_rez.steck_rez,
			  &s_buff_rez.buff_summ[0],
			  sizeof(s_buff_rez.buff_summ[0]),
			  BPF_F/ADC_BUFFER_SIZE_HALF
			  );
//---------------------------------------------------------------
//---------------------НАСТРОЙКА ЦОС-----------------------------
//---------------------------------------------------------------
	filter_par.DF_filt = BPF_BEND_PASS;   // frequency band
	filter_par.F_0_filt = BPF_MIDDLE_F50;  // central frequency
	filter_par.Q1 = BPF_Q;        // ordering of FIR filter
	filter_par.F_adc=BPF_F;      // discretization frequency
	//рассчитываю целочисленные 2-байтные коэфициэнты фильтра
	fun_proces_config_BPF_integer(&filter_par,&s_coef_filter,WINDOW_HAMMING );
	check_filter();
//---------------------------------------------------------------
//---------------------DMA M2M-----------------------------
//---------------------------------------------------------------
	dma_m2m_init(DMA1_Ch2);
//---------------------------------------------------------------
//---------------------НАСТРОЙКА АЦП-----------------------------
//---------------------------------------------------------------
	//Параметры АЦП
	s_adc.ADC=ADC1;
	s_adc.T_adc=BPF_F;
	s_adc.channel_number=0;
	// параметры буффера АЦП
	s_buff_adc.adc_buffer_size=ADC_BUFFER_SIZE_FULL;
	s_buff_adc.rez_buffer_size=ADC_BUFFER_SIZE_HALF;
	s_buff_adc.p_buf_0=&buff_adc[0];
	s_buff_adc.p_buf_1=&buff_adc[ADC_BUFFER_SIZE_HALF];
	s_buff_adc.p_valid_buf=&buff_adc[0];
	s_buff_adc.f_mes_complete=0;
	// инициилизация АЦП + ДМА
	processing_mes_adc_config(&s_adc,&s_buff_adc);

}



//расчитываю ток с учетом нового окна выборок
void processing_mesurement_calc(void){
	double_t temp_var=0;
	double_t temp_sum=0;
	double_t last_item;
	s32 *p_last_item=&buff_rez_filtring[ADC_BUFFER_SIZE_HALF*2-1];
	s32 *p_re_rez=&buff_rez_filtring[0];
	s32 *p_im_rez=&buff_rez_filtring[1];
	for(;p_im_rez<=p_last_item;){
		temp_var=(double_t)(*p_re_rez)*(double_t)(*p_re_rez);//+(double_t)(*p_im_rez)*(double_t)(*p_im_rez);
		temp_var+=(double_t)(*p_im_rez)*(double_t)(*p_im_rez);
		temp_var=sqrt(temp_var);
		temp_sum+=temp_var;
		p_re_rez+=2;
		p_im_rez+=2;
	}
	if(fifo_read_available(&s_buff_rez.steck_rez) < (BPF_F/ADC_BUFFER_SIZE_HALF)){
		fifo_write(&s_buff_rez.steck_rez,1,&temp_sum);
		s_buff_rez.temp_sum+=temp_sum;
		return;
	}
	fifo_read(&s_buff_rez.steck_rez,1,&last_item);
	s_buff_rez.temp_sum-=last_item;
	s_buff_rez.temp_sum+=temp_sum;
	fifo_write(&s_buff_rez.steck_rez,1,&temp_sum);
	s_buff_rez.rez_mes=(double_t)((double_t)s_buff_rez.temp_sum/(double_t)(BPF_F));
}



//---------------задача processing_mesurement_task ------------------
void processing_mesurement_task(){
	u8 qw=1;
	volatile static uint8_t f_begin_mesurement=1;
	//new rezult redy
	if(s_buff_adc.f_mes_complete){   //

		GPIO_SetBits(GPIOB,GPIO_Pin_9);
		//vTaskSuspendAll();

		s_buff_adc.f_mes_complete=0;
		//copy temp data ОТКУДА - КУДА

/*		memcopy_dma(DMA_TX_2BYTE,ADC_BUFFER_SIZE_HALF,s_buff_adc.p_valid_buf,&buff_to_filtring[BPF_Q-1]);
		while(dma_m2m_get_statys())
		{
			qw++;
		}// wait complete copy*/
		memcpy(&buff_to_filtring[BPF_Q-1],s_buff_adc.p_valid_buf,ADC_BUFFER_SIZE_HALF*DMA_TX_2BYTE);
		// Выполняю фильтрацию
		fun_proces_filterin_integer(&buff_to_filtring[0],
				                     SIZE_BUFF_TO_FILTRING,
				                     DEC_STEP,
				                     &buff_rez_filtring[0],
				                     BPF_Q,
				                     &s_coef_filter);
		// копирую результаты АЦП размером (порядок_фильтра - 1) из конца буффера в начало
				memcopy_dma(DMA_TX_2BYTE,
				(BPF_Q-1),
				&buff_to_filtring[SIZE_BUFF_TO_FILTRING-(BPF_Q-1)],
				&buff_to_filtring[0]);
		memcpy(&buff_to_filtring[0],&buff_to_filtring[SIZE_BUFF_TO_FILTRING-(BPF_Q-1)],(BPF_Q-1)*DMA_TX_2BYTE);
		// первую последовательность нужно пропустить, потому что буффер данных к фильтрации в начале работы НЕ заполнен
		if(f_begin_mesurement){
			f_begin_mesurement=0;
			//xTaskResumeAll();
			return;
		}

		// выполнить обработку результатов измерений, расчитать ток и частоту
		processing_mesurement_calc();
		//xTaskResumeAll();
		GPIO_ResetBits(GPIOB,GPIO_Pin_9);


	}

}

u16 mesurement_calc_address_oper_reg(S_mesurement_address *ps_mesurement_address, u16 adres_start){
	ps_mesurement_address->status_mesurement=adres_start;
	ps_mesurement_address->rez_mrez_float=ps_mesurement_address->status_mesurement+NUM_REG_STATUS_MES;
	ps_mesurement_address->rez_norm=ps_mesurement_address->rez_mrez_float+NUM_REG_REZ_FLOAT;
	return (ps_mesurement_address->rez_norm+NUM_REG_REZ_NORM);
}


void t_processing_mesurement(void *pvParameters){

	GPIO_InitTypeDef gpio_service;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	gpio_service.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio_service.GPIO_Pin=GPIO_Pin_9;
	gpio_service.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&gpio_service);
	GPIO_ResetBits(GPIOB,GPIO_Pin_9);

	//конфигурация АЦП+ДМА, ДМА(М2М), ЦОС(коєффициенты фильтра)
	processing_mesurement_global_config();

	while(1)
    {
		//GPIO_SetBits(GPIOB,GPIO_Pin_9);
    	processing_mesurement_task();
    	//GPIO_ResetBits(GPIOB,GPIO_Pin_9);
    }

}

