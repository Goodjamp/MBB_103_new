/*
 * processing_mesurement.c
 *
 *  Created on: March 17, 2016
 *      Author: Gerasimchuk
 */

#include "processing_mesurement.h"
#include "processing_mesurement_extern.h"
#include "processing_mem_map_extern.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "math.h"
#include "string.h"
#include "semphr.h"

// Проверяем допустимые настройки соотношения частоты АЦП и размера буффера АЦП
#if (BPF_F/ADC_BUFFER_SIZE_HALF)*ADC_BUFFER_SIZE_HALF!=BPF_F
#error "processing_mesurement" init: BPF_F/ADC_BUFFER_SIZE_HALF must bee integer
#endif
/*
S_ADC_init s_adc;
S_Buffer_result s_buff_adc;
u16 buff_adc[ADC_BUFFER_SIZE_FULL]={[0 ... (ADC_BUFFER_SIZE_FULL-1)]=0}; // буффер результатов АЦП
u16 buff_to_filtring[SIZE_BUFF_TO_FILTRING]={[0 ... (SIZE_BUFF_TO_FILTRING-1)]=0}; // буффер данных к фильтрации
s32 buff_rez_filtring[ADC_BUFFER_SIZE_HALF*2]={[0 ... (ADC_BUFFER_SIZE_HALF*2-1)] = 0};// буффер результатов фильтрации действительная и мнимая части перемежаются
S_buff_rez s_buff_rez;      // буффер результатов измерений

*/
//u8 filtring_fun[0x175];
//p_fun_proces_filterin_integer ram_filtring=(p_fun_proces_filterin_integer)&filtring_fun[1];

/*
//структура параметров фильтра
S_par_filters filter_par;
S_coef_filter_integer s_coef_filter;
*/
static xSemaphoreHandle SemphrADCBuffFull;
extern S_address_oper_data s_address_oper_data;

// для проверки
    #define SIZE_ARRAY_TIME   10
	#define SIZE_SIN      120
	u16 a_sin[SIZE_SIN];
	s32 rez_filt[SIZE_SIN*3];



void check_filter(S_globall_buff * ps_globall_buff ) {
	// create sin-signal array
	data_operation_sin(&a_sin[0],SIZE_SIN,50,1000,0,ps_globall_buff->filter_par.F_adc);

	//---------made few cycles filtration ------------------
//
	u16 counter_time;
	u16 array_time[SIZE_ARRAY_TIME];
	//filtration
	for(counter_time=0;counter_time<SIZE_ARRAY_TIME;counter_time++){
			fun_proces_filterin_integer(&a_sin[0],SIZE_SIN, 1,&rez_filt[0], ps_globall_buff->filter_par.Q1, &ps_globall_buff->s_coef_filter);
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
void processing_mesurement_global_config(S_globall_buff * ps_globall_buff){
//---------------------------------------------------------------
//---------------------НАСТРОЙКА FIFO-----------------------------
//---------------------------------------------------------------
	fifo_init(&ps_globall_buff->s_buff_rez.steck_rez,
			  &ps_globall_buff->s_buff_rez.buff_summ[0],
			  sizeof(ps_globall_buff->s_buff_rez.buff_summ[0]),
			  BPF_F/ADC_BUFFER_SIZE_HALF
			  );
//---------------------------------------------------------------
//---------------------НАСТРОЙКА ЦОС-----------------------------
//---------------------------------------------------------------
	ps_globall_buff->filter_par.DF_filt = BPF_BEND_PASS;   // frequency band
	ps_globall_buff->filter_par.F_0_filt = BPF_MIDDLE_F50;  // central frequency
	ps_globall_buff->filter_par.Q1 = BPF_Q;        // ordering of FIR filter
	ps_globall_buff->filter_par.F_adc=BPF_F;      // discretization frequency
	//рассчитываю целочисленные 2-байтные коэфициэнты фильтра
	fun_proces_config_BPF_integer(&ps_globall_buff->filter_par,&ps_globall_buff->s_coef_filter,WINDOW_HAMMING );
	// проверкаа фильтра
	check_filter(ps_globall_buff);
//---------------------------------------------------------------
//---------------------DMA M2M-----------------------------
//---------------------------------------------------------------
	dma_m2m_init(DMA1_Ch2);
//---------------------------------------------------------------
//---------------------НАСТРОЙКА АЦП-----------------------------
//---------------------------------------------------------------
	//Параметры АЦП
	ps_globall_buff->s_adc.ADC=ADC1;
	ps_globall_buff->s_adc.T_adc=BPF_F;
	ps_globall_buff->s_adc.channel_number=0;
	// параметры буффера АЦП
	ps_globall_buff->s_buff_adc.adc_buffer_size=ADC_BUFFER_SIZE_FULL;
	ps_globall_buff->s_buff_adc.rez_buffer_size=ADC_BUFFER_SIZE_HALF;
	ps_globall_buff->s_buff_adc.p_buf_0=&ps_globall_buff->buff_adc[0];
	ps_globall_buff->s_buff_adc.p_buf_1=&ps_globall_buff->buff_adc[ADC_BUFFER_SIZE_HALF];
	ps_globall_buff->s_buff_adc.p_valid_buf=&ps_globall_buff->buff_adc[0];
	ps_globall_buff->s_buff_adc.f_mes_complete=0;
	// инициилизация АЦП + ДМА
	processing_mes_adc_config(&ps_globall_buff->s_adc,&ps_globall_buff->s_buff_adc);
}



//расчитываю ток с учетом нового окна выборок
void processing_mesurement_calc(S_globall_buff * ps_globall_buff){
	double_t temp_var=0;
	double_t temp_sum=0;
	double_t last_item;
	s32 *p_last_item=&ps_globall_buff->buff_rez_filtring[ADC_BUFFER_SIZE_HALF*2-1];
	s32 *p_re_rez=&ps_globall_buff->buff_rez_filtring[0];
	s32 *p_im_rez=&ps_globall_buff->buff_rez_filtring[1];
	for(;p_im_rez<=p_last_item;){
		temp_var=(double_t)(*p_re_rez)*(double_t)(*p_re_rez);//+(double_t)(*p_im_rez)*(double_t)(*p_im_rez);
		temp_var+=(double_t)(*p_im_rez)*(double_t)(*p_im_rez);
		temp_var=sqrt(temp_var);
		temp_sum+=temp_var;
		p_re_rez+=2;
		p_im_rez+=2;
	}
	if(fifo_read_available(&ps_globall_buff->s_buff_rez.steck_rez) < (BPF_F/ADC_BUFFER_SIZE_HALF)){
		fifo_write(&ps_globall_buff->s_buff_rez.steck_rez,1,&temp_sum);
		ps_globall_buff->s_buff_rez.temp_sum+=temp_sum;
		return;
	}
	fifo_read(&ps_globall_buff->s_buff_rez.steck_rez,1,&last_item);
	ps_globall_buff->s_buff_rez.temp_sum-=last_item;
	ps_globall_buff->s_buff_rez.temp_sum+=temp_sum;
	fifo_write(&ps_globall_buff->s_buff_rez.steck_rez,1,&temp_sum);
	ps_globall_buff->s_buff_rez.rez_mes=(double_t)((double_t)ps_globall_buff->s_buff_rez.temp_sum/(double_t)(BPF_F));
	double_t middle_rez;
	u16 rez;
	middle_rez=65.535*ps_globall_buff->s_buff_rez.rez_mes;
	middle_rez=middle_rez/100000;
	rez=(u16)middle_rez;
	processing_mem_map_write_s_proces_object_modbus(&rez,1,s_address_oper_data.s_mesurement_address.rez_norm);
}

extern  S_dma_m2m dma_m2m_status;

//---------------задача processing_mesurement_task ------------------
void processing_mesurement_task(S_globall_buff * ps_globall_buff){
	DMA_M2M_STATYS qw=1;
	DMA_M2M_STATYS qw1=1;
	u8 asd=0;
	volatile static uint8_t f_begin_mesurement=1;
	//new rezult redy
		GPIO_SetBits(GPIOB,GPIO_Pin_9);

		ps_globall_buff->s_buff_adc.f_mes_complete=0;
		//copy temp data ОТКУДА - КУДА
/*
		memcopy_dma(DMA_TX_2BYTE,\
				ADC_BUFFER_SIZE_HALF,\
				ps_globall_buff->s_buff_adc.p_valid_buf,
				&ps_globall_buff->buff_to_filtring[BPF_Q-1]
				);
		//while(qw)
		while(dma_m2m_get_statys())
		{
			//qw1=dma_m2m_status.f_transmit_status;
			//qw=dma_m2m_get_statys();
			asd++;
		}// wait complete copy*/
		memcpy(&ps_globall_buff->buff_to_filtring[BPF_Q-1],ps_globall_buff->s_buff_adc.p_valid_buf,ADC_BUFFER_SIZE_HALF*2);
		// Выполняю фильтрацию

		fun_proces_filterin_integer(&ps_globall_buff->buff_to_filtring[0],
				                     SIZE_BUFF_TO_FILTRING,
				                     DEC_STEP,
				                     &ps_globall_buff->buff_rez_filtring[0],
				                     BPF_Q,
				                     &ps_globall_buff->s_coef_filter
				                     );

		// копирую результаты АЦП размером (порядок_фильтра - 1) из конца буффера в начало
		/*		memcopy_dma(DMA_TX_2BYTE,
				(BPF_Q-1),
				&buff_to_filtring[SIZE_BUFF_TO_FILTRING-(BPF_Q-1)],
				&buff_to_filtring[0]);*/
		memcpy(&ps_globall_buff->buff_to_filtring[0],\
				&ps_globall_buff->buff_to_filtring[SIZE_BUFF_TO_FILTRING-(BPF_Q-1)],\
				(BPF_Q-1)*2);
		// первую последовательность нужно пропустить, потому что буффер данных к фильтрации в начале работы НЕ заполнен
		if(f_begin_mesurement){
			f_begin_mesurement=0;
			return;
		}

		// выполнить обработку результатов измерений, расчитать ток и частоту
		processing_mesurement_calc(ps_globall_buff);

		//GPIO_ResetBits(GPIOB,GPIO_Pin_9);

}


u16 mesurement_calc_address_oper_reg(S_mesurement_address *ps_mesurement_address, u16 adres_start){
	ps_mesurement_address->status_mesurement=adres_start;
	ps_mesurement_address->rez_mrez_float=ps_mesurement_address->status_mesurement+NUM_REG_STATUS_MES;
	ps_mesurement_address->rez_norm=ps_mesurement_address->rez_mrez_float+NUM_REG_REZ_FLOAT;
	return (ps_mesurement_address->rez_norm+NUM_REG_REZ_NORM);
}


void processing_mes_adc_buff_full_callback(void){
	portBASE_TYPE priority;
	xSemaphoreGiveFromISR(SemphrADCBuffFull,&priority);
}

void t_processing_mesurement(void *pvParameters){

	GPIO_InitTypeDef gpio_service;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	gpio_service.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio_service.GPIO_Pin=GPIO_Pin_9;
	gpio_service.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&gpio_service);
	GPIO_ResetBits(GPIOB,GPIO_Pin_9);

	S_globall_buff s_global_buff;
	// Семафор для разблокировки задачи обработки результатов АЦП по факту
    // заполнения буффера АЦП
	 vSemaphoreCreateBinary(SemphrADCBuffFull);
	 processing_mes_adc_set_dma_callback(&processing_mes_adc_buff_full_callback);
	//конфигурация АЦП+ДМА, ДМА(М2М), ЦОС(коєффициенты фильтра)
	processing_mesurement_global_config(&s_global_buff);


	while(1)
    {
		// жду пока заполниться буффер АЦП
		xSemaphoreTake(SemphrADCBuffFull,portMAX_DELAY);
		GPIO_SetBits(GPIOB,GPIO_Pin_9);
    	processing_mesurement_task(&s_global_buff);
    	GPIO_ResetBits(GPIOB,GPIO_Pin_9);
    }

}

