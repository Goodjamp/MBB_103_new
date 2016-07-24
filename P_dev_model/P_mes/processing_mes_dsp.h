/*
 * processing_mes_dsp.h
 *
 *  Created on: March 17, 2016
 *      Author: Gerasimchuk
 */

#ifndef PROCESSING_MES_DSP_H_
#define PROCESSING_MES_DSP_H_

#include "stm32f10x.h"
#include "math.h"
//#include "arm_math.h"
//#include "processing_mesurement.h"


//������ ����������
#define ERROR_FEW_SAMPLES 1
#define ERROR_BAD_TYPE    2

// ������� �������
#define Q_MAX  120

// ---------------�������-----------------
#define MODUL(X)     X>=0 ? (X):((-1)*X)

//---------���� ���� ��������----------------
typedef enum{
	WINDOW_NO=0,
	WINDOW_BLEKMAN=1,
	WINDOW_HAMMING=2,
	WINDOW_NUTAL=3,
	WINDOW_PERSONAL=4
}WINDOW;

typedef union{
	float float_array[Q_MAX];
	s32   s32_array[Q_MAX];
}float_s32;



// �������������, ��� � ��������� ������ 4-� ������� ����������� �������
typedef struct{
	float_s32 re_koef_filtr; // �������������� ����������� �������
	float_s32 im_koef_filtr; // ������ ����������� �������
} S_coef_filter;


// ������������� 2-� ������� ����������� �������
typedef struct{
	s16 re_koef_filtr[Q_MAX]; // �������������� ����������� �������
	s16 im_koef_filtr[Q_MAX]; // ������ ����������� �������
} S_coef_filter_integer;


typedef struct{
	u16 Q1;       // ������� �������
	u16 F_0_filt; // ����������� �������
	u16 DF_filt;  // ������ �������
	float F_adc;  // ������� �������������
} S_par_filters;

// ��������� �� ������� ����������
typedef u8 (*p_fun_proces_filterin_integer)(u16*, u16 , u8 ,s32*,u16 , S_coef_filter_integer*);


// ��������� �������
u8 fun_proces_config_BPF_float(S_par_filters const *const p_filter_par,S_coef_filter *ps_coef_filter);
u8 fun_proces_config_BPF_integer(S_par_filters const *const p_filter_par,S_coef_filter_integer *ps_coef_filter, WINDOW type_window);
void fun_proces_norm_fill_koeff(S_par_filters const *const p_filter_par,S_coef_filter *ps_coef_filter, u16 coeff);
void fun_proces_bessel_w(float *p_fil_coef, u16 length, S_par_filters *p_filter_par);
void fun_proces_heming_w(float *p_fil_coef, u16 length, S_par_filters const *p_filter_par);
u8 fun_proces_filterin(s16 const *samples, u16 length_samples, u8 step_window,s32 *rez_samples_processing,u16 Q, S_coef_filter const *ps_coef_filter);
u8 fun_proces_filterin_integer(u16 *p_samples, u16 length_samples, u8 step_window,s32 *rez_samples_processing, u16 Q, S_coef_filter_integer *ps_coef_filter);
u8 fun_proces_geterodin(u16 length_samples, u8 step_window,float *rez_samples_processing);
u8 fun_proces_FFT(float *fft_samples, u16 length_fft_samples);

u16 data_processing_calc_num_shift_data(u8 w, u16 Q, u16 N);
u16 data_processing_calc_num_rez(u8 w, u16 Q, u16 N);
u16 data_processing_calc_num_need_adc_rez(u8 w, u16 Q, u16 N);

#endif

/* ��� ���������� ������� ��������� �������� �������, ������� �������, �������, ���������� �������
 * ����������� �� �������������� ���������� ����
 * ����������� ������������� ����:
 * F_xxx  -  ������� ������� ���
 * W_xxx  -  ������� ������� ������� ���
 * �_xxx  -  ������ ������������ ������� ���
 * D_xxx  -  ��������� ���������� �������������� ���
 *
 */





