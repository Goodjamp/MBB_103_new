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
float W_0_filt;  // ������� �������������� ����������� ������� �������
float W_dand_filter;  // ������� �������������� ������ ����������
float W_geter;  // ������� �������������� ������� ����������
float T_adc; // ����� ������ �������������� (������� ������� �������������)
//�������������� ��������� ������� ��� ���������� ������������ (������ ����� �������)
float F_0_filt;
float F_geter;
float DF_filt;
float F_adc;
u16 Q;
arm_cfft_radix4_instance_f32 my_fft; //��������� �������� �������������� �����



//---------function config_band_pass_filter-------------------
//������� config_band_pass_filter. ��������� ������ ������������ ���������� �������
//������� ���������:
// �������� ���� S_par_filters�� ���������� ������:
//       - Q1 - ������� (���������� ������������) �������
//       - F_0_filt - ����������� ������ �������
//       - DF_filt - ������ ������ ����������
//�������� ��������� � ����������:
//���������� ��������� ���� S_coef_filter �� ���������� ������:
//       - re_filtr - �������������� ����� �����������
//       - im_filtr - ������ ����� �����������
u8 config_band_pass_filter(S_par_filters filter_par) {
	u16 k_1;
	float arg_1; // T_adc*W_dand_filter/2
	float arg_2; // Q/2-1
	float arg_3; // W_0_filt*T_adc

	//�������������� ��������� ������� ��� ���������� ����������� (������ ����� �������)
	F_0_filt = filter_par.F_0_filt;
	DF_filt = filter_par.DF_filt;
	Q = filter_par.Q1;
	F_adc = filter_par.F_adc;
	F_geter= filter_par.F_geter;
	// ����������� ������� �������������� ������� (��. �������� ����������)
	W_0_filt = 2 * M_PI * F_0_filt;
	W_dand_filter = 2 * M_PI * DF_filt;
	T_adc = 1 / F_adc;
	// ����������� ������� �������������� ���������� (��. �������� ����������)
	W_geter = 2 * M_PI * F_geter;
	// ����������� ������������� ��������� ���������� (��. �������� ����������)
	arg_1 = T_adc * W_dand_filter / 2;
	arg_2 = filter_par.Q1 / 2 - 1;
	arg_3 = W_0_filt * T_adc;
	// ����������� ����������� ������� (�������������� � ������ �����)
	for (k_1 = 0; k_1 < Q; k_1++) {
		s_coef_filter.re_koef_filtr[k_1] = (arm_sin_f32(arg_1 * (k_1 - arg_2))
                 * arm_cos_f32(arg_3 * (k_1 - arg_2)))
				/\
 (arg_1 * (k_1 - arg_2)); // �������������� �����
		s_coef_filter.im_koef_filtr[k_1] = (arm_sin_f32(arg_1 * (k_1 - arg_2))
				* arm_sin_f32(arg_3 * (k_1 - arg_2)))
				/\
 (arg_1 * (k_1 - arg_2)); // ������ �����
	};

	s_coef_filter.re_koef_filtr[127] = 1;
	s_coef_filter.im_koef_filtr[127] = 0;

	return 0;
}

//---------function filtering_processing-------------------
// ������� filtering. ��������� ���������� ������� �������
// ������� ���������:
// - samples - ��������� �� ����� � ���������
// - length_samples - ���������� ��������
// - step_window - ��� ���� ������������
// - rez_samples_processing - ��������� �� ����� ����������� ����������.
//                            ���������� ������������ �������������� ������� (re(0), im(0),re(1), im(1)re(2), im(2))
u8 filtering_processing(float *samples, u16 length_samples, u8 step_window,
		float *rez_samples_processing) {
	u16 k_1; // ����� ��������������� �������
	u16 k_2; //����� ������� ������� �������
	u16 k_3; // ��� �������� ������������ �������
	u16 k_4; // ��� �������� ������� ������� � ����� ����������
	u16 max_number_rez_filt;
	float re_fil_sum = 0; // ������������� ����� ����������� �������
	float im_fil_sum = 0; // ������������� ����� ����������� �������

	// �������� ������� ����������
	if (length_samples < Q) {
		return ERROR_FEW_SAMPLES;
	}; // ���� ���������� �������� ����� ������� ������� - ����� � ����������
	   //���������� ������������ ����� �������� �������
	max_number_rez_filt = (length_samples - Q + step_window) * 2 / step_window;
	// ���������� ������ ������� ����������� ����������
	// ��������� ����������
	for (k_1 = 0, k_2 = 0; k_1 <= (max_number_rez_filt); k_1 += 2, k_2 +=step_window) { // ����� ��������������� �������
		re_fil_sum = 0;
		im_fil_sum = 0;
		for (k_3 =(Q-1),k_4=k_2; k_3 <= (Q-1); k_3--, k_4++)  // k_2 - ����� �������, k_3 - ����� �����������
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
// ������� geterodin. ��������� ����������������� (������� �������)
// ������� ���������:
// - length_samples - ���������� ��������
// - step_window - ��� ���� ������������
// - rez_samples_processing - ��������������� �����: ������ �� ����������������� � �����
//                            ���������� ������������ �������������� ������� (re(0), im(0),re(1), im(1)re(2), im(2))
u8 geterodin_processing(u16 length_samples, u8 step_window,float *rez_samples_processing) {
	u16 k_1,k_2;
	float rez_geterod_k_1_re; // ������������� �������������� k_1 - � ��������� �����������������
	float rez_geterod_k_1_im; // ������������� ������ k_1 - � ��������� �����������������
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
// ������� geterodin. ��������� ����������������� (������� �������)
// ������� ���������:
// - fft_samples - ��������� �� ����� � �� ������������ ���������. ������� ������ ���� ����������� ���������� �������: re(0),im(0),re(1),im(1),re(2),im(2)....
//             ���� ����� ����� ��������� ��������, ���������� �������������� ������� ����� �� ������� ��� � �� ��������������
// - length_fft_samples - ���������� ��������, ����������� �� ����:
// �������� ���������:
//0 - ������� ���������� ������
u8 fft_processing(float *fft_samples, u16 length_fft_samples) {
	arm_status error_init_fft; //��������� ������ ������� ������������� ��������� �������� fft
	//�������������� ��������� �������� �������������� �����
	error_init_fft = arm_cfft_radix4_init_f32(&my_fft, length_fft_samples, 0,1); //������������� ��������� ��������
	arm_cfft_radix4_f32(&my_fft, fft_samples);
	return 0;
}

