/*
 * filtring.h
 *
 *  Created on: May 5, 2013
 *      Author: Gerasimchu
 */

#ifndef FILTRING_H_
#define FILTRING_H_

#include "stm32f10x.h"


//������ ����������
#define ERROR_FEW_SAMPLES 1
#define ERROR_BAD_TYPE    2

// ������� �������
#define Q_MAX  256



typedef struct s_filtring_1{
	float re_koef_filtr[Q_MAX]; // �������������� ����������� �������
	float im_koef_filtr[Q_MAX]; // ������ ����������� �������
} S_coef_filter;

typedef struct s_filtring_2{
	u16 Q1;   // ������� �������
	u16 F_0_filt;   // ����������� �������
	u16 DF_filt;   // ������ �������
	u16 F_geter;   // ������� ����������
	float F_adc; //// ������� �������������
} S_par_filters;

// ��������� �������
u8 config_band_pass_filter(S_par_filters );
u8 filtering_processing(float *, u16 l, u8 ,float *);
u8 fft_processing(float *, u16 );
u8 geterodin_processing(u16, u8,float *);

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





