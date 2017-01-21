/*
 * processing_mes_dsp.c
 *
 *  Created on: March 17, 2016
 *      Author: Gerasimchuk
 */


#include "processing_mes_dsp.h"
//---------function blackman_w_coef-------------------
// функция blackman_w_coef - выполняет коэфициэнта для оконного усечение коэффициэнтов фильтра
// окном БЛЕУМАНА
// входные аргументы:
// - p_fil_coef - указатель на масив коэффициэнтов фильтра
// - length - количество отсчетов для усечения
// - index  - индекс требуимого коэффициента окна
// выходные аргументы:
// - = -1 если lengthБ=1
// - =  коэфициент окна
static inline float blackman_w_coef(u16 length, u16 index){
	if(length<=1){return 10;}
	return 0.42-0.5*cosf(2*M_PI*index/(length-1))+0.08*cosf(4*M_PI*index/(length-1));
}

//---------function hamming_w_coef-------------------
// функция hamming_w_coef - выполняет коэфициэнта для оконного усечение коэффициэнтов фильтра
// окном ХЕМИНГА
// входные аргументы:
// - p_fil_coef - указатель на масив коэффициэнтов фильтра
// - length - количество отсчетов для усечения
// - index  - индекс требуимого коэффициента окна
// выходные аргументы:
// - = -1 если lengthБ=1
// - =  коэфициент окна
static inline float hamming_w_coef(u16 length, u16 index){
	if(length<=1){return 10;}
	return 0.54-0.46*cosf(2*M_PI*index/(length-1));
}

//---------function nuttall_w_coef-------------------
// функция nuttall_w_coef - выполняет коэфициэнта для оконного усечение коэффициэнтов фильтра
// окном НАТАЛА
// входные аргументы:
// - p_fil_coef - указатель на масив коэффициэнтов фильтра
// - length - количество отсчетов для усечения
// - index  - индекс требуимого коэффициента окна
// выходные аргументы:
// - = -1 если lengthБ=1
// - =  коэфициент окна
static inline float nuttall_w_coef(u16 length, u16 index){
	if(length<=1){return 10;}
	return 0.355768-0.48829*cosf(2*M_PI*index/(length-1))+0.14128*cosf(4*M_PI*index/(length-1))-0.01168*cosf(6*M_PI*index/(length-1));
}


//---------function my_w_coef-------------------
// функция my_w_coef - выполняет коэфициэнта для оконного усечение коэффициэнтов фильтра
// окном НАТАЛА
// входные аргументы:
// - p_fil_coef - указатель на масив коэффициэнтов фильтра
// - length - количество отсчетов для усечения
// - index  - индекс требуимого коэффициента окна
// выходные аргументы:
// - = -1 если lengthБ=1
// - =  коэфициент окна
static inline float personal_w_coef(u16 length, u16 index){
	if(length<=1){return 10;}
	return 0.355768-0.48829*cosf(2*M_PI*index/(length))+0.14128*cosf(4*M_PI*index/(length-1))-0.01168*cosf(6*M_PI*index/(length-1));
}




float W_0_filt;  // угловая характеристика центральной частоты фильтра
float W_dand_filter;  // угловая характеристика полосы пропусканя
float W_geter;  // угловая характеристика частоты гетеродина
float T_adc; // время одного преобразования (обратно частоте дискретизации)

static inline void bpf_coeff(u16 index, float *p_re_coeff, float *p_im_coeff,float arg_1,float arg_2,float arg_3, u16 Q_int, u8 type_window ){
		float coeff=0;
		(*p_re_coeff) = (sinf(arg_1 * (index - arg_2))* cosf(arg_3 * (index - arg_2))) / (arg_1 * (index - arg_2)); // действительная часть
		(*p_im_coeff) = (sinf(arg_1 * (index - arg_2))* sinf(arg_3 * (index - arg_2))) / (arg_1 * (index - arg_2)); // мнимая часть
		switch(type_window){
			case WINDOW_NO: return;
			case WINDOW_BLEKMAN:
				coeff=blackman_w_coef(Q_int,index);
				break;
			case WINDOW_HAMMING:
				coeff=hamming_w_coef(Q_int,index);
				break;
			case WINDOW_NUTAL:
				coeff=nuttall_w_coef(Q_int,index);
				break;
			case WINDOW_PERSONAL:
				coeff=personal_w_coef(Q_int,index);
				break;
		}
		(*p_re_coeff) *=coeff;
		(*p_im_coeff) *=coeff;
}


//---------function config_band_pass_filter-------------------
//функция config_band_pass_filter. Выполняет расчет коэфициентов полосового фильтра
//входные аргументы:
// сруктура типа S_par_filtersсо следующими полями:
//       - Q1 - порядок (количество коэфициентов) фильтра
//       - F_0_filt - центральная чатота фильтра
//       - DF_filt - ширина полосы фильтрации
//выходные аргументы и переменные:
//глобальная структура типа S_coef_filter со следующими полями:
//       - re_filtr - действительная часть коэфициента
//       - im_filtr - мнимая часть коэфициента
u8 fun_proces_config_BPF_float(S_par_filters const *const p_filter_par,S_coef_filter *ps_coef_filter) {
	u16 k_1;
	u16 middle_index;
	//переопредиляем параметры фильтра для локального пользщования (внутри файла фильтра)
	float F_0_filt;
	float DF_filt;
	float F_adc;
	float Q;
	float arg_1; // T_adc*W_dand_filter/2
	float arg_2; // Q/2-1/2
	float arg_3; // W_0_filt*T_adc

	//переопредиляем параметры фильтра для локального пользования (внутри файла фильтра)
	F_0_filt = p_filter_par->F_0_filt;
	DF_filt = p_filter_par->DF_filt;
	Q = p_filter_par->Q1;
	F_adc = p_filter_par->F_adc;
//	F_geter= p_filter_par->F_geter;
	// расчитываем угловые характеристики фильтра (см. описание переменных)
	W_0_filt = 2 * M_PI * F_0_filt;
	W_dand_filter = 2 * M_PI * DF_filt;
	T_adc = 1 / F_adc;
	// расчитываем угловые характеристики гетеродина (см. описание переменных)
	//W_geter = 2 * M_PI * F_geter;
	// Расчитываем промежуточные параметры аргументов (см. описание переменных)
	arg_1 = T_adc * W_dand_filter / 2;
	arg_2 = (float)p_filter_par->Q1 / (float)2 - (float)1/2;
	arg_3 = T_adc*W_0_filt;
	// Расчитываем коефициенты фильтра (действительную и мнимую часть)
	for (k_1 = 0; k_1 < Q; k_1++) {
		ps_coef_filter->re_koef_filtr.float_array[k_1] = (sinf(arg_1 * (k_1 - arg_2))* cosf(arg_3 * (k_1 - arg_2)))	/ (arg_1 * (k_1 - arg_2)); // действительная часть
		ps_coef_filter->im_koef_filtr.float_array[k_1] = (sinf(arg_1 * (k_1 - arg_2))* sinf(arg_3 * (k_1 - arg_2)))	/ (arg_1 * (k_1 - arg_2)); // мнимая часть
	};
	middle_index=Q/2;
	ps_coef_filter->re_koef_filtr.float_array[middle_index] = 1;
	ps_coef_filter->im_koef_filtr.float_array[middle_index] = 0;

	return 0;
}




//---------function config_band_pass_filter-------------------
//функция config_band_pass_filter. Выполняет расчет коэфициентов полосового фильтра
//входные аргументы:
// сруктура типа S_par_filtersсо следующими полями:
//       - Q1 - порядок (количество коэфициентов) фильтра
//       - F_0_filt - центральная чатота фильтра
//       - DF_filt - ширина полосы фильтрации
//выходные аргументы и переменные:
//глобальная структура типа S_coef_filter со следующими полями:
//       - re_filtr - действительная часть коэфициента
//       - im_filtr - мнимая часть коэфициента
u8 fun_proces_config_BPF_integer(S_par_filters const *const p_filter_par,S_coef_filter_integer *ps_coef_filter, WINDOW type_window) {
	u16 k_1;
	u16 middle_index;
	float temp_re_coef, temp_im_coef, max_coeff;
	//переопредиляем параметры фильтра для локального пользщования (внутри файла фильтра)
	float F_0_filt;
	float DF_filt;
	float F_adc;
	float Q;
	float arg_1; // T_adc*W_dand_filter/2
	float arg_2; // Q/2-1/2
	float arg_3; // W_0_filt*T_adc

	//переопредиляем параметры фильтра для локального пользования (внутри файла фильтра)
	F_0_filt = p_filter_par->F_0_filt;
	DF_filt = p_filter_par->DF_filt;
	Q = p_filter_par->Q1;
	F_adc = p_filter_par->F_adc;
//	F_geter= p_filter_par->F_geter;
	// расчитываем угловые характеристики фильтра (см. описание переменных)
	W_0_filt = 2 * M_PI * F_0_filt;
	W_dand_filter = 2 * M_PI * DF_filt;
	T_adc = 1 / F_adc;
	// расчитываем угловые характеристики гетеродина (см. описание переменных)
	//W_geter = 2 * M_PI * F_geter;
	// Расчитываем промежуточные параметры аргументов (см. описание переменных)
	arg_1 = T_adc * W_dand_filter / 2;
	arg_2 = (float)p_filter_par->Q1 / (float)2 - (float)1/2;
	arg_3 = T_adc*W_0_filt;
	// ---------ТОНКИЙ МОМЕНТ: если порядок фильтра парный    - центрального коэффициента нету
	//                         если порялок фильтра НЕ парный - центральный коэффициент ЕСТЬ и равен 1+i0
	middle_index=Q/2; // центральный отсчет фильтра
	// Расчитываем коефициенты фильтра (действительную и мнимую часть) и ищем макисмальный коэффициент
	for (k_1 = 0; k_1 < middle_index; k_1++) {
		bpf_coeff(k_1,&temp_re_coef, &temp_im_coef,arg_1,arg_2,arg_3,(u16)Q ,type_window);
		temp_re_coef=MODUL(temp_re_coef);
		if(temp_re_coef>max_coeff){max_coeff=temp_re_coef;}
		temp_im_coef=MODUL(temp_im_coef);
		if(temp_im_coef>max_coeff){max_coeff=temp_im_coef;}
	};
	if(middle_index*2 != Q){max_coeff=1;} //если порядок не парный, максимальный коэффициет равен 1
	// Раситываем коэффициенты фильтра и нормируем их относительно максимального коэффициента
	for (k_1 = 0; k_1 < middle_index; k_1++) {
		bpf_coeff(k_1,&temp_re_coef, &temp_im_coef,arg_1,arg_2,arg_3,(u16)Q ,type_window);
		ps_coef_filter->re_koef_filtr[k_1]=(s16)(temp_re_coef*4300/max_coeff);
		ps_coef_filter->im_koef_filtr[k_1]=(s16)(temp_im_coef*4300/max_coeff);
		// симетрическая часть фильра
		ps_coef_filter->re_koef_filtr[p_filter_par->Q1-1-k_1]=ps_coef_filter->re_koef_filtr[k_1];
		ps_coef_filter->im_koef_filtr[p_filter_par->Q1-1-k_1]=(-1)*ps_coef_filter->im_koef_filtr[k_1];
	};
	//
	if(middle_index*2 != Q){ // если порядок не парный - задаем центральный коэффициент
	ps_coef_filter->re_koef_filtr[middle_index] = 1;
	ps_coef_filter->im_koef_filtr[middle_index] = 0;
	}

	return 0;
}




void fun_proces_norm_fill_koeff(S_par_filters const *const p_filter_par,S_coef_filter *ps_coef_filter, u16 coeff){
	u16 counter;
	float max_coef=ps_coef_filter->im_koef_filtr.float_array[0];
	// find maximum coefficient
	for(counter=0;counter<p_filter_par->Q1;counter++){
		if(ps_coef_filter->im_koef_filtr.float_array[counter]>max_coef){
			max_coef=ps_coef_filter->im_koef_filtr.float_array[counter];
		}
		if(ps_coef_filter->re_koef_filtr.float_array[counter]>max_coef){
			max_coef=ps_coef_filter->re_koef_filtr.float_array[counter];
		}
	}
	// Normalization coefficient
	for(counter=0;counter<p_filter_par->Q1;counter++){
		ps_coef_filter->im_koef_filtr.float_array[counter]=(ps_coef_filter->im_koef_filtr.float_array[counter]*coeff/max_coef);
		ps_coef_filter->re_koef_filtr.float_array[counter]=(ps_coef_filter->re_koef_filtr.float_array[counter]*coeff/max_coef);
		ps_coef_filter->im_koef_filtr.s32_array[counter]=(s16)(ps_coef_filter->im_koef_filtr.float_array[counter]);
		ps_coef_filter->re_koef_filtr.s32_array[counter]=(s16)(ps_coef_filter->re_koef_filtr.float_array[counter]);
	}

}




//---------function fun_proces_filterin-------------------
// функция filtering. Выполняет фильтрацию входной выборки
// входные аргументы:
// - samples - указатель на масив с отсчетами
// - length_samples - количество отсчетов
// - step_window - шаг окна прореживания
// - rez_samples_processing - указатель на масив результатов фильтрации.
//                            результаты слажываються ПЕРЕМЕЖАЮЩИМСЯ ОБРАЗОМ (re(0), im(0),re(1), im(1)re(2), im(2))
u8 fun_proces_filterin(s16 const *samples, u16 length_samples, u8 step_window,s32 *rez_samples_processing,\
		                      u16 Q, S_coef_filter const *ps_coef_filter) {
	u16 k_1; // номер отфильтрованого отсчета
	u16 k_2; //номер отсчета входной выборки
	u16 k_3; // для перебора коэфициентов фильтра
	u16 k_4; // для перебора входной выборки в петле фильтрации
	u16 max_number_rez_filt;
	s32 re_fil_sum = 0; // промежуточный буфер сумирования фильтра
	s32 im_fil_sum = 0; // промежуточный буфер сумирования фильтра

	// проверка входных аргументов
	if (length_samples < Q) {
		return ERROR_FEW_SAMPLES;
	}; // если количество отсчетов менше порядка фильтра - выход с аргументом
	   //расчитываю максимальный номер выходного отсчета

	max_number_rez_filt = (length_samples - Q + step_window) * 2 / step_window; // *2 - потому что есть действительная и мнимая части
	// количество циклов расчета результатов фильтрации
	// выполняем фильтрацию (свертку)
	for (k_1 = 0, k_2 = 0; k_1 <= (max_number_rez_filt); k_1 += 2, k_2 +=step_window) { // номер отфильтрованого отсчета
		re_fil_sum = 0;
		im_fil_sum = 0;
		for (k_3 =(Q-1),k_4=k_2; k_3 <= (Q-1); k_3--, k_4++)  // k_2 - номер отсчета выборки, относительно которого начинаеться рассчет
			                                                  // следующего результата, k_3 - номер коэфициента
            {
			re_fil_sum +=(s32)(ps_coef_filter->re_koef_filtr.s32_array[k_3]* samples[k_4]);
			im_fil_sum +=(s32)(ps_coef_filter->im_koef_filtr.s32_array[k_3]* samples[k_4]);
			//
			k_3--;
			k_4++;
			re_fil_sum +=(s32)(ps_coef_filter->re_koef_filtr.s32_array[k_3]* samples[k_4]);
			im_fil_sum +=(s32)(ps_coef_filter->im_koef_filtr.s32_array[k_3]* samples[k_4]);
			//
			k_3--;
			k_4++;
			re_fil_sum +=(s32)(ps_coef_filter->re_koef_filtr.s32_array[k_3]* samples[k_4]);
			im_fil_sum +=(s32)(ps_coef_filter->im_koef_filtr.s32_array[k_3]* samples[k_4]);
			//
			//
			k_3--;
			k_4++;
			re_fil_sum +=(s32)(ps_coef_filter->re_koef_filtr.s32_array[k_3]* samples[k_4]);
			im_fil_sum +=(s32)(ps_coef_filter->im_koef_filtr.s32_array[k_3]* samples[k_4]);

		};
		rez_samples_processing[k_1]     = re_fil_sum;
		rez_samples_processing[k_1 + 1] = im_fil_sum;
	}

	return 0;
}


//---------function fun_proces_filterin_new-------------------
// функция filtering. Выполняет фильтрацию входной выборки
// входные аргументы:
// - samples - указатель на масив с отсчетами
// - length_samples - количество отсчетов
// - step_window - шаг окна прореживания
// - rez_samples_processing - указатель на масив результатов фильтрации.
//                            результаты слажываються ПЕРЕМЕЖАЮЩИМСЯ ОБРАЗОМ (re(0), im(0),re(1), im(1)re(2), im(2))
u8 fun_proces_filterin_integer(u16 *p_samples, u16 length_samples, u8 step_window,s32 *rez_samples_processing,\
		                      u16 Q, S_coef_filter_integer *ps_coef_filter) {
	s32 re_fil_sum = 0; // промежуточный буфер сумирования фильтра
	s32 im_fil_sum = 0; // промежуточный буфер сумирования фильтра

	// проверка входных аргументов
	if (length_samples < Q) {
		return ERROR_FEW_SAMPLES; // если количество отсчетов менше порядка фильтра - выход с аргументом
	};

	u16 *p_rez=p_samples; //указатель на ПОТОЧНЫЙ ОТСЧЕТ, относительно которого НАЧИНАЕТСЯ фильтрация
	u16 *p_rez_temp=0; //указатель на ОТСЧЕТ, с которым выполняется МАС операция
	u16 *p_rez_last=p_rez + (data_processing_calc_num_rez(step_window, Q, length_samples)-1)*step_window; // указатель на ПОСЛЕДНИЙ ОТСЧЕТ, относительно которого НАЧИНАЕТСЯ фильтрация
	s16 *p_re_fill_coeff, *p_im_fill_coeff; //указатель на действительный (im) и мнимый ПОТОЧНЫЕ коэффициенты фильтра, которыми выполняеться фильтоация
	s16 *p_firest_re_fill_coeff=&ps_coef_filter->re_koef_filtr[0];
	s16 *p_last_re_fill_coeff=&ps_coef_filter->re_koef_filtr[Q-1];
	s16 *p_last_im_fill_coeff=&ps_coef_filter->im_koef_filtr[Q-1];

	for (; p_samples <= p_rez_last; p_samples +=step_window) { // номер отфильтрованого отсчета
		p_rez_temp=p_samples;
		p_re_fill_coeff=p_last_re_fill_coeff;
		p_im_fill_coeff=p_last_im_fill_coeff;
		re_fil_sum = 0;
		im_fil_sum = 0;
       // Выполняем фильтрацию относительно элемента p_rez_temp
		__asm volatile(
				// -1-
				"start:                                    \n\t"
				"ldrsh.w %[asm_rez_adc], [%[asm_p_rez_adc]]       \n\t"
				"ldrsh.w %[asm_fill],    [%[asm_p_fill_re]]       \n\t"
				"mla     %[asm_rez_re],%[asm_rez_adc],%[asm_fill],%[asm_rez_re] \n\t"
				"ldrsh.w %[asm_fill],    [%[asm_p_fill_im]]       \n\t"
				"mla     %[asm_rez_im],%[asm_rez_adc],%[asm_fill],%[asm_rez_im] \n\t"
				// -2-
				"ldrsh.w %[asm_rez_adc], [%[asm_p_rez_adc],#2]       \n\t"
				"ldrsh.w %[asm_fill],    [%[asm_p_fill_re],#-2]       \n\t"
				"mla     %[asm_rez_re],%[asm_rez_adc],%[asm_fill],%[asm_rez_re] \n\t"
				"ldrsh.w %[asm_fill],    [%[asm_p_fill_im],#-2]       \n\t"
				"mla     %[asm_rez_im],%[asm_rez_adc],%[asm_fill],%[asm_rez_im] \n\t"
				// -3-
				"ldrsh.w %[asm_rez_adc], [%[asm_p_rez_adc],#4]       \n\t"
				"ldrsh.w %[asm_fill],    [%[asm_p_fill_re],#-4]       \n\t"
				"mla     %[asm_rez_re],%[asm_rez_adc],%[asm_fill],%[asm_rez_re] \n\t"
				"ldrsh.w %[asm_fill],    [%[asm_p_fill_im],#-4]       \n\t"
				"mla     %[asm_rez_im],%[asm_rez_adc],%[asm_fill],%[asm_rez_im] \n\t"
				// -4-
				"ldrsh %[asm_rez_adc], [%[asm_p_rez_adc],#6]       \n\t"
				"ldrsh %[asm_fill],    [%[asm_p_fill_re],#-6]       \n\t"
				"mla     %[asm_rez_re],%[asm_rez_adc],%[asm_fill],%[asm_rez_re] \n\t"
				"ldrsh %[asm_fill],    [%[asm_p_fill_im],#-6]       \n\t"
				"mla     %[asm_rez_im],%[asm_rez_adc],%[asm_fill],%[asm_rez_im] \n\t"

				"add %[asm_p_rez_adc], 8                           \n\t"
				"sub %[asm_p_fill_re], 8                          \n\t"
				"sub %[asm_p_fill_im], 8                          \n\t"
				"cmp  %[asm_p_fill_re], %[asm_p_first_re_coef]      \n\t"
				"bhi.n start                                        \n\t"

				"str  %[asm_rez_re],[%[asm_p_rez_re]]              \n\t"
				"str  %[asm_rez_im],[%[asm_p_rez_im]]              \n\t"
				:: [asm_rez_adc]   "r" (0),          // регистр для загрузки отсчетов
				 [asm_p_rez_adc] "r" (p_rez_temp),  // указатель на начало отсчетов к фильтрации
				 [asm_rez_im]    "r" (im_fil_sum),
				 [asm_p_rez_im]  "r" (&im_fil_sum),
				 [asm_rez_re]    "r" (re_fil_sum),
				 [asm_p_rez_re]  "r" (&re_fil_sum),
				 [asm_fill]      "r" (0),
				 [asm_p_fill_re] "r" (p_re_fill_coeff),
				 [asm_p_fill_im] "r" (p_im_fill_coeff),
				 [asm_p_first_re_coef] "r" (p_firest_re_fill_coeff)
		);

		(*rez_samples_processing) = re_fil_sum;
		rez_samples_processing++;
		(*rez_samples_processing) = im_fil_sum;
		rez_samples_processing++;
	}

	return 0;
}




//---------function geterodin-------------------
// функция geterodin. Выполняет гетеродинирование (перенос частоты)
// входные аргументы:
// - length_samples - количество отсчетов
// - step_window - шаг окна прореживания
// - rez_samples_processing - двунеправленный масив: данные до гетеродинирования и после
//                            результаты слажываються ПЕРЕМЕЖАЮЩИМСЯ ОБРАЗОМ (re(0), im(0),re(1), im(1)re(2), im(2))
u8 fun_proces_geterodin(u16 length_samples, u8 step_window,float *rez_samples_processing) {
	u16 k_1,k_2;
	float rez_geterod_k_1_re; // промежуточний действительный k_1 - й результат гетеродинирования
	float rez_geterod_k_1_im; // промежуточний мнимый k_1 - й результат гетеродинирования
	float Wg_Tadc=W_geter*T_adc*step_window;
	for(k_1=0,k_2=0;k_2<=(length_samples-1);k_1+=2,k_2++){
		rez_geterod_k_1_re=sinf(Wg_Tadc*k_2)*rez_samples_processing[k_1]+\
				sinf(Wg_Tadc*k_2)*rez_samples_processing[k_1+1];
		rez_geterod_k_1_im=cosf(Wg_Tadc*k_2)*rez_samples_processing[k_1+1]-\
				sinf(Wg_Tadc*k_2)*rez_samples_processing[k_1];
		rez_samples_processing[k_1]=rez_geterod_k_1_re;
		rez_samples_processing[k_1+1]=rez_geterod_k_1_im;
	}
	return 0;
}


//*****************************************************************************************************************************
//**********************ФУНКЦИИ КОНФИГУРАЦИИ БУФЕРОВ ОТСЧЕТОВ******************************************************************
//*****************************************************************************************************************************

// -----------data_processing_calc_num_shift_data-----------
// функция data_processing_calc_num_shift_data - расчет к-ва отсчетов,которые нужно перенести из конца буфера,
// который фильтруеться, в начало буффера, который заполняеться
//входные аргументы:
// w - окно прореживания (децимации)
// Q - порядок фильтра
// N - размер выборки
// выходные аргументы
// - к-во отсчетов, которые нужно перенести
u16 data_processing_calc_num_shift_data(u8 w, u16 Q, u16 N){
	if(Q<w){return 0;}
	return N-w*(u16)((N-Q)/w)-w;
}

// -----------calc_num_rez-----------
// функция calc_num_rez - расчет к-ва результатов фильтрации с учетом параметра фильтра
//входные аргументы:
// w - окно прореживания (децимации)
// Q - порядок фильтра
// N - размер выборки
// выходные аргументы
// - к-во результатов
u16 data_processing_calc_num_rez(u8 w, u16 Q, u16 N){
	if(Q<w){return 0;}
	return (N - Q )/w + 1;
}

// -----------calc_num_need_adc_rez-----------
// функция calc_num_need_adc_rez - расче к-ва отсчетов, которые необходимо передать в ф-ю фильтрации
// для получения заданого к-ва результатов
//входные аргументы:
// w - окно прореживания (децимации)
// Q - порядок фильтра
// N - желаимое к-во результатов
// выходные аргументы
// - необходимое к-во отсчетов
u16 data_processing_calc_num_need_adc_rez(u8 w, u16 Q, u16 N){
	if(Q<w){return 0;}
	return Q-1+(N-1)*w;
}







