/*
 * processing_mesurement.h
 *
 *  Created on: March 17, 2016
 *      Author: Gerasimchuk
 */

#ifndef PROCESSING_MESUREMENT_H_
#define PROCESSING_MESUREMENT_H_

#include "stm32f10x.h"
#include "processing_mes_dsp.h"
#include "processing_mes_adc.h"
//#include "counter_time.h"
//#include "arm_math.h"
#include "dma_m2m.h"
#include "fifo.h"


// --------------��������� �������� ���----------------
//������ ������ �������
#define ADC_BUFFER_SIZE_FULL  40
//�-�� ����������� ���������� ����������
#define ADC_BUFFER_SIZE_HALF  20

//---------------��������� �������--------------------
//����������� ������� �������
#define BPF_MIDDLE_F50        50
#define BPF_MIDDLE_F60        60
//������ ����������� �������
#define BPF_BEND_PASS         50
//������� �������
#define BPF_Q                 Q_MAX
//������� �������������
#define BPF_F                 1000
//��� ���� ������������
#define DEC_STEP              1
// ������ ������ � ����������
#define SIZE_BUFF_TO_FILTRING BPF_Q-1+ADC_BUFFER_SIZE_HALF

// ������� ����������
#define V_REF         3
// ������������ ���������� - 12 ���
#define BIT_REF       4095


typedef struct{
	S_fifo_steak steck_rez;
	double_t buff_summ[BPF_F/ADC_BUFFER_SIZE_HALF];
	double_t temp_sum;
	double_t rez_mes;
}S_buff_rez;

typedef struct{
	S_ADC_init s_adc;
	S_Buffer_result s_buff_adc;
	u16 buff_adc[ADC_BUFFER_SIZE_FULL];//={[0 ... (ADC_BUFFER_SIZE_FULL-1)]=0}; // ������ ����������� ���
	u16 buff_to_filtring[SIZE_BUFF_TO_FILTRING];//={[0 ... (SIZE_BUFF_TO_FILTRING-1)]=0}; // ������ ������ � ����������
	s32 buff_rez_filtring[ADC_BUFFER_SIZE_HALF*2];//={[0 ... (ADC_BUFFER_SIZE_HALF*2-1)] = 0};// ������ ����������� ���������� �������������� � ������ ����� ������������
	S_buff_rez s_buff_rez;      // ������ ����������� ���������
	//��������� ���������� �������
	S_par_filters filter_par;
	S_coef_filter_integer s_coef_filter;
}S_globall_buff;


void check_filter(S_globall_buff * ps_globall_buff);
void data_operation_sin(u16 *pa_sin, u16 length, float f1, float Am, float fi,float Fadc);
void processing_mesurement_global_config(S_globall_buff * ps_globall_buff );
void processing_mesurement_calc(S_globall_buff * ps_globall_buff);
void processing_mesurement_task(S_globall_buff * ps_globall_buff);

#endif
