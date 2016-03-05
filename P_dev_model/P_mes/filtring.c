/*
 * filtring.h
 *
 *  Created on: May 5, 2013
 *      Author: Gerasimchu
 */
#include "filtring.h"
//#include <math.h>
#include "arm_math.h"
#include "data_operation_workspace.h"

S_coef_filter s_coef_filter;
float W_0_filt;  // угловая характеристика центральной частоты фильтра
float W_dand_filter;  // угловая характеристика полосы пропусканя
float W_geter;  // угловая характеристика частоты гетеродина
float T_adc; // время одного преобразования (обратно частоте дискретизации)
//переопредиляем параметры фильтра для локального пользщования (внутри файла фильтра)
float F_0_filt;
float F_geter;
float DF_filt;
float F_adc;
u16 Q;
arm_cfft_radix4_instance_f32 my_fft; //СТРУКТУРА НАСТРОЕК ПРЕОБРАЗОВАНИЯ ФУРЬЕ



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
u8 config_band_pass_filter(S_par_filters filter_par) {
	u16 k_1;
	float arg_1; // T_adc*W_dand_filter/2
	float arg_2; // Q/2-1
	float arg_3; // W_0_filt*T_adc

	//переопредиляем параметры фильтра для локального пользования (внутри файла фильтра)
	F_0_filt = filter_par.F_0_filt;
	DF_filt = filter_par.DF_filt;
	Q = filter_par.Q1;
	F_adc = filter_par.F_adc;
	F_geter= filter_par.F_geter;
	// расчитываем угловые характеристики фильтра (см. описание переменных)
	W_0_filt = 2 * M_PI * F_0_filt;
	W_dand_filter = 2 * M_PI * DF_filt;
	T_adc = 1 / F_adc;
	// расчитываем угловые характеристики гетеродина (см. описание переменных)
	W_geter = 2 * M_PI * F_geter;
	// Расчитываем промежуточные параметры аргументов (см. описание переменных)
	arg_1 = T_adc * W_dand_filter / 2;
	arg_2 = filter_par.Q1 / 2 - 1;
	arg_3 = W_0_filt * T_adc;
	// Расчитываем коефициенты фильтра (действительную и мнимую часть)
	for (k_1 = 0; k_1 < Q; k_1++) {
		s_coef_filter.re_koef_filtr[k_1] = (arm_sin_f32(arg_1 * (k_1 - arg_2))
                 * arm_cos_f32(arg_3 * (k_1 - arg_2)))
				/\
 (arg_1 * (k_1 - arg_2)); // действительная часть
		s_coef_filter.im_koef_filtr[k_1] = (arm_sin_f32(arg_1 * (k_1 - arg_2))
				* arm_sin_f32(arg_3 * (k_1 - arg_2)))
				/\
 (arg_1 * (k_1 - arg_2)); // мнимая часть
	};

	s_coef_filter.re_koef_filtr[127] = 1;
	s_coef_filter.im_koef_filtr[127] = 0;

	return 0;
}

//---------function filtering_processing-------------------
// функция filtering. Выполняет фильтрацию входной выборки
// входные аргументы:
// - samples - указатель на масив с отсчетами
// - length_samples - количество отсчетов
// - step_window - шаг окна прореживания
// - rez_samples_processing - указатель на масив результатов фильтрации.
//                            результаты слажываються ПЕРЕМЕЖАЮЩИМСЯ ОБРАЗОМ (re(0), im(0),re(1), im(1)re(2), im(2))
u8 filtering_processing(float *samples, u16 length_samples, u8 step_window,
		float *rez_samples_processing) {
	u16 k_1; // номер отфильтрованого отсчета
	u16 k_2; //номер отсчета входной выборки
	u16 k_3; // для перебора коэфициентов фильтра
	u16 k_4; // для перебора входной выборки в петле фильтрации
	u16 max_number_rez_filt;
	float re_fil_sum = 0; // промежуточный буфер сумирования фильтра
	float im_fil_sum = 0; // промежуточный буфер сумирования фильтра

	// проверка входных аргументов
	if (length_samples < Q) {
		return ERROR_FEW_SAMPLES;
	}; // если количество отсчетов менше порядка фильтра - выход с аргументом
	   //расчитываю максимальный носер ыходного отсчета
	max_number_rez_filt = (length_samples - Q + step_window) * 2 / step_window;
	// количество циклов расчета результатов фильтрации
	// выполняем фильтрацию
	for (k_1 = 0, k_2 = 0; k_1 <= (max_number_rez_filt); k_1 += 2, k_2 +=step_window) { // номер отфильтрованого отсчета
		re_fil_sum = 0;
		im_fil_sum = 0;
		for (k_3 =(Q-1),k_4=k_2; k_3 <= (Q-1); k_3--, k_4++)  // k_2 - номер отсчета, k_3 - номер коэфициента
            {
			re_fil_sum +=s_coef_filter.re_koef_filtr[k_3]* samples[k_4];
			im_fil_sum +=s_coef_filter.im_koef_filtr[k_3]* samples[k_4];

		};
		rez_samples_processing[k_1] = re_fil_sum;
		rez_samples_processing[k_1 + 1] = im_fil_sum;
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
u8 geterodin_processing(u16 length_samples, u8 step_window,float *rez_samples_processing) {
	u16 k_1,k_2;
	float rez_geterod_k_1_re; // промежуточний действительный k_1 - й результат гетеродинирования
	float rez_geterod_k_1_im; // промежуточний мнимый k_1 - й результат гетеродинирования
	float Wg_Tadc=W_geter*T_adc*step_window;
	for(k_1=0,k_2=0;k_2<=(length_samples-1);k_1+=2,k_2++){
		rez_geterod_k_1_re=arm_cos_f32(Wg_Tadc*k_2)*rez_samples_processing[k_1]+\
				arm_sin_f32(Wg_Tadc*k_2)*rez_samples_processing[k_1+1];
		rez_geterod_k_1_im=arm_cos_f32(Wg_Tadc*k_2)*rez_samples_processing[k_1+1]-\
				arm_sin_f32(Wg_Tadc*k_2)*rez_samples_processing[k_1];
		rez_samples_processing[k_1]=rez_geterod_k_1_re;
		rez_samples_processing[k_1+1]=rez_geterod_k_1_im;
	}
	return 0;
}


//---------function fft_processing-------------------
// функция geterodin. Выполняет гетеродинирование (перенос частоты)
// входные аргументы:
// - fft_samples - указатель на масив с из комплексными отсчетами. Отсчеты должны быть упорядочены следуюющим образом: re(0),im(0),re(1),im(1),re(2),im(2)....
//             Этот масив также являеться выходным, результаты преобразования сложены таким же образом как и до преобразования
// - length_fft_samples - количество отсчетов, выбираеться из ряда:
// выходные аргументы:
//0 - удачное выполнение фукции
u8 fft_processing(float *fft_samples, u16 length_fft_samples) {
	arm_status error_init_fft; //структура ошибок процеса инициилизации структуры настроек fft
	//инициилизируем настройки быстрого преобразования Фурье
	error_init_fft = arm_cfft_radix4_init_f32(&my_fft, length_fft_samples, 0,1); //ИНИЦИИЛИЗАЦИЯ СТРУКУТРЫ НАСТРОЕК
	arm_cfft_radix4_f32(&my_fft, fft_samples);
	return 0;
}

