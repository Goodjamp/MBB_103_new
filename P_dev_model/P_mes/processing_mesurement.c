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

// ��������� ���������� ��������� ����������� ������� ��� � ������� ������� ���
#if (BPF_F/ADC_BUFFER_SIZE_HALF)*ADC_BUFFER_SIZE_HALF!=BPF_F
#error "processing_mesurement" init: BPF_F/ADC_BUFFER_SIZE_HALF must bee integer
#endif
//������������� ������ ��� ���������� ������� ����������� �������� ����
const S_calib s_calib_current={
	#include "processing_mes_calib_current.h"
};

static xSemaphoreHandle SemphrADCBuffFull;
extern S_address_oper_data s_address_oper_data;

// ��� ��������
    #define SIZE_ARRAY_TIME   10
	#define SIZE_SIN          120
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



//---------------������� data_operation_sin ------------------
// ������� data_operation_sin - ���������� �����-�������
// ������� ���������:
//*pa_sin - ��������� �� �������� ������� ������
// length - ������ �������
// f1     - ������� �����-�������
// Am     - ��������� �����-�������
// fi     - ��������� ���� �����-�������
// Fadc   - ������� ������������� �������
void data_operation_sin(u16 *pa_sin, u16 length, float f1, float Am, float fi,float Fadc){
	u16 counter;
	float Dt=1/Fadc;
	for(counter=0;counter<length;counter++){
		(*pa_sin)=(u16)(Am*sinf(2*M_PI*f1*(counter*Dt)+fi)+Am);
		pa_sin++;
	}
}




//---------------������� processing_mesurement_global_config ------------------
// ������� processing_mesurement_global_config - ��������� ������������ ��� ��������� ������ ���������
// ������� ���������:
//*ps_globall_buff - ��������� �� ��������� � ������� ������� ��� �������� ������ ���������
void processing_mesurement_global_config(S_globall_buff * ps_globall_buff){
//---------------------------------------------------------------
//---------------------��������� FIFO ��� ��������� ����---------
//---------------------------------------------------------------
	fifo_init(&ps_globall_buff->s_buff_rez_current.steck_rez,
			  &ps_globall_buff->s_buff_rez_current.buff_summ[0],
			  sizeof(ps_globall_buff->s_buff_rez_current.buff_summ[0]),
			  REZ_BUFF_SIZE
			  );
//---------------------------------------------------------------
//---------------------��������� FIFO ��� ��������� �������------
//---------------------------------------------------------------
	fifo_init(&ps_globall_buff->s_buff_rez_frequency.steck_rez,
			&ps_globall_buff->s_buff_rez_frequency.buff_summ[0],
			sizeof(ps_globall_buff->s_buff_rez_frequency.buff_summ[0]),
			REZ_BUFF_SIZE
	);
//---------------------------------------------------------------
//---------------------��������� ���-----------------------------
//---------------------------------------------------------------
	ps_globall_buff->filter_par.DF_filt = BPF_BEND_PASS;   // frequency band
	ps_globall_buff->filter_par.F_0_filt = BPF_MIDDLE_F50;  // central frequency
	ps_globall_buff->filter_par.Q1 = BPF_Q;        // ordering of FIR filter
	ps_globall_buff->filter_par.F_adc=BPF_F;      // discretization frequency
	//����������� ������������� 2-������� ����������� �������
	fun_proces_config_BPF_integer(&ps_globall_buff->filter_par,&ps_globall_buff->s_coef_filter,WINDOW_HAMMING );
	// ��������� �������
	//check_filter(ps_globall_buff);
//---------------------------------------------------------------
//---------------------DMA M2M-----------------------------
//---------------------------------------------------------------
	dma_m2m_init(DMA1_Ch2);
//---------------------------------------------------------------
//---------------------��������� ���-----------------------------
//---------------------------------------------------------------
	//��������� ���
	ps_globall_buff->s_adc.ADC=ADC1;
	ps_globall_buff->s_adc.F_adc=BPF_F;
	ps_globall_buff->s_adc.channel_number=0;
	// ��������� ������� ���
	ps_globall_buff->s_buff_adc.adc_buffer_size=ADC_BUFFER_SIZE_FULL;
	ps_globall_buff->s_buff_adc.rez_buffer_size=ADC_BUFFER_SIZE_HALF;
	ps_globall_buff->s_buff_adc.p_buf_0=&ps_globall_buff->buff_adc[0];
	ps_globall_buff->s_buff_adc.p_buf_1=&ps_globall_buff->buff_adc[ADC_BUFFER_SIZE_HALF];
	ps_globall_buff->s_buff_adc.p_valid_buf=&ps_globall_buff->buff_adc[0];
	ps_globall_buff->s_buff_adc.f_mes_complete=0;
	// ������������� ��� + ���
	processing_mes_adc_config(&ps_globall_buff->s_adc,&ps_globall_buff->s_buff_adc);
}



//---------------������� processing_mesurement_calc ------------------
// ������� processing_mesurement_calc - ��������� ������ �������� ���������� ������� (��� � �������)
// �� ����������� ����������
// ������� ���������:
//*ps_globall_buff - ��������� �� ��������� � ������� ������� ��� �������� ������ ���������
void processing_mesurement_calc(S_globall_buff * ps_globall_buff){
	u8 secon_order=0;
	double_t x_0, x_1, y_0, y_1, dx, dy, dl, temp_sum_dl=0;
	u8 k_1=0;
	double_t temp_var=0;
	double_t temp_sum_current=0;
	double_t last_item;
	s32 *p_last_item=&ps_globall_buff->buff_rez_filtring[ADC_BUFFER_SIZE_HALF*2-1];
	s32 *p_re_rez=&ps_globall_buff->buff_rez_filtring[0];
	s32 *p_im_rez=&ps_globall_buff->buff_rez_filtring[1];
	double_t middle_rez;
	u16 rez;
	for(;p_im_rez<=p_last_item;){
		//����������� ���
		temp_var=(double_t)(*p_re_rez)*(double_t)(*p_re_rez);//+(double_t)(*p_im_rez)*(double_t)(*p_im_rez);
		temp_var+=(double_t)(*p_im_rez)*(double_t)(*p_im_rez);
		temp_var=sqrt(temp_var);
		temp_sum_current+=temp_var;
		// ����������� �������
		x_0=x_1;
		y_0=y_1;
		x_1=(double_t)(*p_re_rez)/temp_var;
		y_1=(double_t)(*p_im_rez)/temp_var;

		if(secon_order)
		{
			dx=(double_t)x_1-(double_t)x_0;
			dy=(double_t)y_1-(double_t)y_0;
			dx*=dx;
			dy*=dy;
			dl=dy+dx;
			dl=sqrt(dl);
			temp_sum_dl+=dl;
			k_1++;
		}
		secon_order=1;
		p_re_rez+=2;
		p_im_rez+=2;
	}

	if(fifo_read_available(&ps_globall_buff->s_buff_rez_current.steck_rez) < REZ_BUFF_SIZE){
		// ---------������������� ������ ����-----------------------------
		fifo_write(&ps_globall_buff->s_buff_rez_current.steck_rez,1,&temp_sum_current);
		ps_globall_buff->s_buff_rez_current.temp_sum+=temp_sum_current;
		// ---------������������� ������ �������-----------------------------
		fifo_write(&ps_globall_buff->s_buff_rez_frequency.steck_rez,1,&temp_sum_dl);
		ps_globall_buff->s_buff_rez_frequency.temp_sum+=temp_sum_dl;
		return;
	}
	// --------------------���������� ��� --------------------
	fifo_read(&ps_globall_buff->s_buff_rez_current.steck_rez,1,&last_item);
	ps_globall_buff->s_buff_rez_current.temp_sum-=last_item;
	ps_globall_buff->s_buff_rez_current.temp_sum+=temp_sum_current;
	fifo_write(&ps_globall_buff->s_buff_rez_current.steck_rez,1,&temp_sum_current);
	ps_globall_buff->s_buff_rez_current.rez_mes=(double_t)((double_t)ps_globall_buff->s_buff_rez_current.temp_sum/(double_t)(REZ_BUFF_SIZE*ADC_BUFFER_SIZE_HALF));
	//��������� "�����", �� �������������, ��� ���������� ����
	processing_mem_map_write_s_proces_object_modbus((u16*)&ps_globall_buff->s_buff_rez_current.rez_mes,NUM_REG_REZ_DOUBLE,s_address_oper_data.s_mesurement_address.mes_current_double);
	middle_rez=65.535*ps_globall_buff->s_buff_rez_current.rez_mes;
	middle_rez=middle_rez/1000000;
	rez=(u16)middle_rez;
	processing_mem_map_write_s_proces_object_modbus(&rez,NUM_REG_REZ_U16,s_address_oper_data.s_mesurement_address.rez_mes_current);
	// --------------------���������� ������� --------------------
	fifo_read(&ps_globall_buff->s_buff_rez_frequency.steck_rez,1,&last_item);
	ps_globall_buff->s_buff_rez_frequency.temp_sum-=last_item;
	ps_globall_buff->s_buff_rez_frequency.temp_sum+=temp_sum_dl;
	fifo_write(&ps_globall_buff->s_buff_rez_frequency.steck_rez,1,&temp_sum_dl);
	middle_rez=(double_t)ps_globall_buff->s_buff_rez_frequency.temp_sum/(double_t)(REZ_BUFF_SIZE*ADC_BUFFER_SIZE_HALF-REZ_BUFF_SIZE);
	// ����� ������������ ����
	middle_rez=0.5*middle_rez;
	middle_rez=asin(middle_rez)*2; // ������� ��������
	middle_rez*=ps_globall_buff->filter_par.F_adc/(2*M_PI);
	rez=middle_rez*1000;
	processing_mem_map_write_s_proces_object_modbus(&rez,NUM_REG_REZ_U16,s_address_oper_data.s_mesurement_address.rez_mes_frequency);
}


//---------------������� processing_mesurement_calc_clib_data ------------------
// ������� processing_mesurement_calc_clib_data - ��������� ������ ����������� �������� ���� ��
// ������������� ������
// ������� ���������:
//
MES_STATUS processing_mesurement_calc_clib_data(double mes_cod, double *rez_current){
	u8 counter;
	for(counter=0;counter<(s_calib_current.num_point-1);counter++){
		//���� �������� �������� ���������� ���� � ������� ���������� ������������� ���
		if((mes_cod >= KOD_VAL(counter))&&(mes_cod < KOD_VAL(counter+1))){
			// ������� ��� �������� ((x1-x2)/(y1-y2))*(mes_cod-x1)+y1
			(*rez_current)=(KOD_VAL(counter)-KOD_VAL(counter+1))/(CURRENT_VAL(counter)-CURRENT_VAL(counter+1))*\
					       (mes_cod-KOD_VAL(counter))+CURRENT_VAL(counter);
			return MES_OK;
		}
	}
	return MES_OUT_OF_CALIB_RAMGE;
}



extern  S_dma_m2m dma_m2m_status;
//---------------������ processing_mesurement_task ------------------
void processing_mesurement_task(S_globall_buff * ps_globall_buff){
	DMA_M2M_STATYS qw=1;
	DMA_M2M_STATYS qw1=1;
	u8 asd=0;
	volatile static uint8_t f_begin_mesurement=1;
	//new rezult redy
		GPIO_SetBits(GPIOB,GPIO_Pin_9);

		ps_globall_buff->s_buff_adc.f_mes_complete=0;
		//copy temp data ������ - ����
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
		// �������� ����������

		fun_proces_filterin_integer(&ps_globall_buff->buff_to_filtring[0],
				                     SIZE_BUFF_TO_FILTRING,
				                     DEC_STEP,
				                     &ps_globall_buff->buff_rez_filtring[0],
				                     BPF_Q,
				                     &ps_globall_buff->s_coef_filter
				                     );

		// ������� ���������� ��� �������� (�������_������� - 1) �� ����� ������� � ������
		/*		memcopy_dma(DMA_TX_2BYTE,
				(BPF_Q-1),
				&buff_to_filtring[SIZE_BUFF_TO_FILTRING-(BPF_Q-1)],
				&buff_to_filtring[0]);*/
		memcpy(&ps_globall_buff->buff_to_filtring[0],\
				&ps_globall_buff->buff_to_filtring[SIZE_BUFF_TO_FILTRING-(BPF_Q-1)],\
				(BPF_Q-1)*2);
		// ������ ������������������ ����� ����������, ������ ��� ������ ������ � ���������� � ������ ������ �� ��������
		if(f_begin_mesurement){
			f_begin_mesurement=0;
			return;
		}

		// ��������� ��������� ����������� ���������, ��������� ��� � �������
		processing_mesurement_calc(ps_globall_buff);

		//GPIO_ResetBits(GPIOB,GPIO_Pin_9);

}


u16 mesurement_calc_address_oper_reg(S_mesurement_address *ps_mesurement_address, u16 adres_start){
	ps_mesurement_address->status_mesurement=adres_start;
	ps_mesurement_address->rez_mes_current=ps_mesurement_address->status_mesurement+NUM_REG_STATUS_MES;
	ps_mesurement_address->rez_mes_frequency=ps_mesurement_address->rez_mes_current+NUM_REG_REZ_U16;
	ps_mesurement_address->mes_current_double=ps_mesurement_address->rez_mes_frequency+NUM_REG_REZ_U16;
	return (ps_mesurement_address->rez_mes_frequency+NUM_REG_REZ_DOUBLE);
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
	// ������� ��� ������������� ������ ��������� ����������� ��� �� �����
    // ���������� ������� ���
	 vSemaphoreCreateBinary(SemphrADCBuffFull);
	 processing_mes_adc_set_dma_callback(&processing_mes_adc_buff_full_callback);
	//������������ ���+���, ���(�2�), ���(����������� �������)
	processing_mesurement_global_config(&s_global_buff);


	while(1)
    {
		// ��� ���� ����������� ������ ���
		xSemaphoreTake(SemphrADCBuffFull,portMAX_DELAY);
		GPIO_SetBits(GPIOB,GPIO_Pin_9);

    	processing_mesurement_task(&s_global_buff);

    	GPIO_ResetBits(GPIOB,GPIO_Pin_9);
    }

}

