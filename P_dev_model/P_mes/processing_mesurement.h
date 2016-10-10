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


//����� ������� ����������� ���������
#define CALC_LINE

// --------------��������� �������� ���----------------
//������ ������ �������
#define ADC_BUFFER_SIZE_FULL  40
//�-�� ����������� ���������� ����������
#define ADC_BUFFER_SIZE_HALF  20
//������ ������� ����������� ��������� (���� ���������) � �-�� ADC_BUFFER_SIZE_HALF
// ������ ���������� ����������� � ����, ������� ����� (��������) REZ_BUFF_SIZE*ADC_BUFFER_SIZE_HALF
//                                                     (�����)    REZ_BUFF_SIZE*ADC_BUFFER_SIZE_HALF/BPF_F
#define REZ_BUFF_SIZE 10
//������ ������� ���������� ������ � ����� ������� �� ��������� ���������� ��� ���������� ����������
#define MIDDLE_REZ 1
//---------------��������� �������--------------------
//����������� ������� �������
#define BPF_MIDDLE_F50        50
#define BPF_MIDDLE_F60        60
//������ ����������� �������
#define BPF_BEND_PASS         32
//������� �������
#define BPF_Q                 Q_MAX
//������� �������������
#define BPF_F                 1000
//��� ���� ������������
#define DEC_STEP              1
// ������ ������ � ����������
#define SIZE_BUFF_TO_FILTRING BPF_Q-1+ADC_BUFFER_SIZE_HALF
// 3*���
#define _3_SIGMA  			  90638
// ������� ����������
#define V_REF         3
// ����������� ���������� - 12 ���
#define BIT_REF       4095
// ������������ ������ ��������� ������������� ������
#define CALIB_DATA_MAX_SIZE   100

#define KOD_VAL(X)     s_calib_current.calib_curve[X][0]
#define CURRENT_VAL(X) s_calib_current.calib_curve[X][1]

//
typedef enum{
	MES_OK=0,
	MES_OUT_OF_CALIB_RAMGE=1
}MES_STATUS;


typedef struct{
	S_fifo_steak steck_rez;
	//double_t *p_buff_summ;//[REZ_BUFF_SIZE];
	double_t temp_sum;
	double_t rez_mes;
}S_buff_rez;



// ��������� �� ����� �����������, ��������
typedef struct{
	S_ADC_init s_adc;
	S_Buffer_result s_buff_adc;
	u16 buff_adc[ADC_BUFFER_SIZE_FULL];//={[0 ... (ADC_BUFFER_SIZE_FULL-1)]=0}; // ������ ����������� ���
	u16 buff_to_filtring[SIZE_BUFF_TO_FILTRING];//={[0 ... (SIZE_BUFF_TO_FILTRING-1)]=0}; // ������ ������ � ����������
	s32 buff_rez_filtring[ADC_BUFFER_SIZE_HALF*2];//={[0 ... (ADC_BUFFER_SIZE_HALF*2-1)] = 0};// ������ ����������� ���������� �������������� � ������ ����� ������������
	S_buff_rez s_buff_rez_current;       // ������ ����������� ����
	S_buff_rez s_buff_rez_frequency;     // ������ ����������� �������
	//��������� ���������� �������
	S_par_filters filter_par;
	S_coef_filter_integer s_coef_filter;
}S_globall_buff;

// ��������� � �������������� �������
typedef struct{
	u32 num_point;
	double calib_curve[CALIB_DATA_MAX_SIZE][2];
}S_calib;


void check_filter(S_globall_buff * ps_globall_buff);
void data_operation_sin(u16 *pa_sin, u16 length, float f1, float Am, float fi,float Fadc);
void processing_mesurement_task(S_globall_buff * ps_globall_buff);

#endif
