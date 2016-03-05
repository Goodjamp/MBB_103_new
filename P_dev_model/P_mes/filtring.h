/*
 * filtring.h
 *
 *  Created on: May 5, 2013
 *      Author: Gerasimchu
 */

#ifndef FILTRING_H_
#define FILTRING_H_

#include "stm32f10x.h"


//ошибки фильтрации
#define ERROR_FEW_SAMPLES 1
#define ERROR_BAD_TYPE    2

// Порядок фильтра
#define Q_MAX  256



typedef struct s_filtring_1{
	float re_koef_filtr[Q_MAX]; // действительные коэфициенты фильтра
	float im_koef_filtr[Q_MAX]; // мнимые коэфициенты фильтра
} S_coef_filter;

typedef struct s_filtring_2{
	u16 Q1;   // порядок фильтра
	u16 F_0_filt;   // центральная частота
	u16 DF_filt;   // ширина фильтра
	u16 F_geter;   // частота гетеродина
	float F_adc; //// Частота дискретизации
} S_par_filters;

// Прототипы функций
u8 config_band_pass_filter(S_par_filters );
u8 filtering_processing(float *, u16 l, u8 ,float *);
u8 fft_processing(float *, u16 );
u8 geterodin_processing(u16, u8,float *);

#endif

/* Все переменные которые принимают значение частоты, угловой частоты, периода, промежутка времени
 * начинаються из спецификаторов приведеных ниже
 * Сьандартные спецификаторы типа:
 * F_xxx  -  частота процеса ххх
 * W_xxx  -  угловая частота процеса ххх
 * Т_xxx  -  период гармоничного процеса ххх
 * D_xxx  -  временной промежуток характеристики ххх
 *
 */





