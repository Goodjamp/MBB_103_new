/*
 * processing_TY_signal.c
 *
 *  Created on: February 15, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "processing_TY.h"

// ������� ��� ������� ������� �������� �� ���� DP
xSemaphoreHandle SemaphoreHandle_ty;
portBASE_TYPE prioryty_take_semaphore_ty;
//����� �������� ����� � ������ ���������� ����
S_ty_rel_out s_rel_out[TOTAL_NUM_TY];
//����� �������� ����� � ������ �������� ������� �����
S_ty_rel_out s_coil_check[TOTAL_NUM_TY];
//��������� ���������� �������� ������������ ������ ��
S_TY_user_config *ps_TY_user_config;
// ��������� �������� � ���������� ������� ��
S_ty_operation s_TY_operation;
//��������� ��������� ����������� �������� ��������� ������ ��
S_ty_cycling_check s_cycling_check;
// ��������� ���������-�������� ��������� ��������� ������ ��
S_state_TY s_oper_data_TY_present;
// ����� ������� �������, � �������� ������������, ������ ������������ ��� ��������
F_check a_check_fun_exception[NUM_CHECK_FUN]={[0 ... (NUM_CHECK_FUN-1)]=0};
// ��������� ��������� �������� ������� �����
static S_ty_coil_check_grup s_ty_coil_check_grup;
// ����-������� ��������� ������� ������� ��� ������ ���� DP
static u8 f_dp_time_complite=RESET;
// ��������� ������� ����������� ��������� (�� mem_map_processing.c)
extern S_address_oper_data s_address_oper_data;

//---------------������� processing_TY_signal_init---------------
// ������� processing_TY_signal_init -��������� ������������ ����������� ����� ��
void processing_TY_signal_init(void) {
	// ���������,�������� �������������� ��������, ����� �������� (���_�������� ������� �����/�� ����������)
	if(STATE_JAMPER2){ //���� ������� �� ���������� (��� 1) - ��������� ������ ��������
		s_ty_coil_check_grup.f_limit_check=RESET;
	}
	else //���� ������� ���������� (��� 0) - ��������� ���������� ��������
	{
		s_ty_coil_check_grup.f_limit_check=SET;
	}
	// ������������ ������ ����� ����� �������������� ������� ��
	processing_TY_signal_init_gpio();
	// ������������ �������
	processing_TY_signal_init_tim();
	// ������������ ������� ���������� �� � ����������� ��������
	processing_TY_signal_set_check_par();
	// ����� ������� callback �������� �������������� ������ ������ ������� READ_COIL_STATUS (�1) ��� Modbus
	modbus_callback_address_check(
			&processing_TY_check_is_coil_status_address_modbus,
			READ_COIL_STATUS);
	// ����� ������� callback �������� �������������� ������ ������ ������� FORCE_SINGLE_COIL (�5) ��� Modbus
	modbus_callback_address_check(
			&processing_TY_check_force_single_coil_address_modbus,
			FORCE_SINGLE_COIL);
	// ����� �������������� ������� �������� ������� FORCE_SINGLE_COIL (�5) ��� Modbus
	modbus_callback_add_check(&processing_TY_signal_modbus_check, FORCE_SINGLE_COIL);
	// ����� �������������� ������� callback ��������� ������� FORCE_SINGLE_COIL (�5) ��� Modbus
	modbus_callback_add_processing(&processing_TY_signal_operation,	FORCE_SINGLE_COIL);
	// ������������� �������� ������� �� ���������� ������� ��
	processing_TY_signal_clear();
	//�������� ������������ ������ ��������� (�������� �� ���� � �. �.)
	processing_TY_signal_init_state();
	//������ �������
	vSemaphoreCreateBinary(SemaphoreHandle_ty);
}

//------------������� INIT_MBB_fill_S_TY---------------
// ������� INIT_MBB_fill_S_TY - ��������� ��������� ��������� ��
INIT_MBB_Rezult processing_TY_fill_S_TY(u8 *read_data) {
	// ������� ��������� TY
	ps_TY_user_config = (S_TY_user_config*) read_data;
	//��������� ������ � ��������� ������������ ������������
	return processing_TY_signal_analis_user_config();
}

//---------------������� processing_TY_signal_analis_user_config---------------
// ������� processing_TY_signal_analis_user_config -��������� ������ ������������ ������ �� �������������
// ������������ ������ ��������� ����������:
// - ��� ������ DP/SP
// - - DP ����� ���� �������� ������ ������ ������� �� (����� ������ �� � ����� ������ - ����� ������� ��).
// ��� DP �������� ������ ON, ����� OFF (��������� ����� ������) ��������������� ������������� �� ��� DP.
// ����� ������������ ������ � ���������� �������������� ����� �FF, ����� ����
// ����� �� ������ ���� ����������� (� ���������� ������ ������������ ������ ������ ���� ������ ��� PARALLEL_CHANNEL=3)
// - - SP ����� ���� ����� ������ ���������� ������ (������ / �� ������)
INIT_MBB_Rezult processing_TY_signal_analis_user_config(void) {
	u8 k1;
	//--------------------------��������� ������� analis_user_config_check_paralel-----------------------------------
	INIT_MBB_Rezult analis_user_config_check_paralel(u8 num_paralel_sourse,
			u16 num_paralel_check, TY_MODE type) {
		//���������� �� ����� ������������ ������ �������� ������������
		if ((ps_TY_user_config->s_TY_out_config[num_paralel_check].num_paralel < s_address_oper_data.s_TY_address.set_state_TY)||
				(ps_TY_user_config->s_TY_out_config[num_paralel_check].num_paralel >=
				(s_address_oper_data.s_TY_address.set_state_TY + TOTAL_NUM_TY))) // ����� ������ ���� ����� TOTAL_NUM_TY
		{
			return MBB_INIT_ERROR;
		};
		// �������� ���� ������������ ������
		if (ps_TY_user_config->s_TY_out_config[num_paralel_check].mode_TY!= type) // ��� ������ ���� PARALLEL_CHANNEL
		{
			return MBB_INIT_ERROR;
		};
		//� ���� "����� ������������ ������" ������������ ������ ������ ���� ����� ������������ ������ OFF DP
		if (ps_TY_user_config->s_TY_out_config[num_paralel_check].num_paralel!=
				(num_paralel_sourse+ s_address_oper_data.s_TY_address.set_state_TY)) // (k1+1) - ������ ��� ����������� ������� ���� ����� OFF DP
		{
			return MBB_INIT_ERROR;
		};
		// �������� ������������ ������ �� ��������� �������
		{
			return MBB_INIT_OK;
		};
	}
	;

	for (k1 = 0; k1 < TOTAL_NUM_TY; k1++) {

		if ((ps_TY_user_config->s_TY_out_config[k1].mode_TY != DOUBLE_POSITION)
			&& (ps_TY_user_config->s_TY_out_config[k1].mode_TY!= SINGLE_POSITION)
			&& (ps_TY_user_config->s_TY_out_config[k1].mode_TY!= PARALLEL_CHANNEL)
			&& (ps_TY_user_config->s_TY_out_config[k1].mode_TY != NO_OUT))
		{
			return MBB_INIT_ERROR;
		}
		// ����� �� �� ������������
		if (ps_TY_user_config->s_TY_out_config[k1].mode_TY == NO_OUT)
		{
			continue;
		};

		//-----------��� ������ �� - DOUBLE POSITION

		if (ps_TY_user_config->s_TY_out_config[k1].mode_TY == DOUBLE_POSITION)
		{
			if (k1 % 2)
			{
				return MBB_INIT_ERROR;
			} // ������ ������ ������ ������� �������� ��� ������������ DP
			// ���������� �� ����� [k1+1] (����� OFF)
			if ((k1 + 1) >= TOTAL_NUM_TY)
			{
				return MBB_INIT_ERROR;
			}; // ���� �� ����������, ����������
			// �������� ���� ������ [k1+1] (����� OFF DP)
			if (ps_TY_user_config->s_TY_out_config[k1 + 1].mode_TY!= DOUBLE_POSITION)
			{
				return MBB_INIT_ERROR;
			}; // ���� ��� ���������� ������ �� DOUBLE_POSITION ����������
			//�������� ����������� ������������ ������
			if (!(ps_TY_user_config->s_TY_out_config[k1 + 1].f_paralel_out))
			{
				k1++;
				continue;
			}; // ���� ������������ ������ ����, �������� � �������� ������ k1+2
			//���� ����������� ����� ����
			//�������� �������������� ������������ ������ � ������������
			if (analis_user_config_check_paralel(k1 + 1,
				ps_TY_user_config->s_TY_out_config[k1 + 1].num_paralel- s_address_oper_data.s_TY_address.set_state_TY,
				PARALLEL_CHANNEL))
			{
				return MBB_INIT_ERROR;
			};

			//�������� ��������� ��� ����������
			k1++;
			continue;
		}; // �������� � �������� ������ k1+2 ()

		//-----------��� ������ �� - SINGLE POSITIN

		if (ps_TY_user_config->s_TY_out_config[k1].mode_TY == SINGLE_POSITION)
		{
			continue;
		}

		//-----------��� ������ �� - PARALLEL_CHANNEL

		if (ps_TY_user_config->s_TY_out_config[k1].mode_TY== PARALLEL_CHANNEL)
		{
			//�������� �������������� ������������ ������ � ������������
			if (analis_user_config_check_paralel(k1,
					ps_TY_user_config->s_TY_out_config[k1].num_paralel
					- s_address_oper_data.s_TY_address.set_state_TY,
					DOUBLE_POSITION))
			{
				return MBB_INIT_ERROR;
			};
		};
	};
	return MBB_INIT_OK;
}

//---------------������� processing_TY_signal_init_state---------------
// ������� processing_TY_signal_init_state - ��������� ��������� �������� �������� �� ����,
// ��������� ���� ������� �� � ������������� ��
//
void processing_TY_signal_init_state(void) {
	S_state_TY s_state_TY;
	u8 count_ty_out;
	// �������� ������ ��������� �� �� ����
	FLASH_OPERATION_read_flash_8b((u8*) &s_state_TY, sizeof(S_state_TY),
			PAGE(TY_FLASH_PAGE));

	// ������������ ��� ������ �������� ������� �� � �������������� ���������
	s_oper_data_TY_present.status_TY = 0;
	for (count_ty_out = 0; count_ty_out < TOTAL_NUM_TY; count_ty_out++)
	{
		s_oper_data_TY_present.present_state_TY[count_ty_out] = TY_NOT_SET;
		s_oper_data_TY_present.set_state_TY[count_ty_out] = TY_NOT_SET;
		s_oper_data_TY_present.operation_TY_statys[count_ty_out] = REZ_CHECK_OK;
	}
	/*
	 // ���� ������ ����� ������������� ��� ����������� ��� ����������� - ������ ������������� �������� �� �����,
	 // ��� ������ � ��������� �����������
	 if(s_state_TY.status_TY){
	 return;
	 }
	 */

	//memcpy((u8*)&s_oper_data_TY_present,(u8*)&s_state_TY,sizeof(S_state_TY));
	// ���� ������ �� ��� ���������� (������ �� ��������������) ����������
	// - ���� ����� �� ���� SINGLE_POSITION - ����� ���������� �� �������� ��, ������� ���� �� ������������, � ��������� ����������� ��������
	// - ���� ����� �� ���� DOUBLE_POSITION - �������� ��������� �� ���������� (�� ����������)
	for (count_ty_out = 0; count_ty_out < TOTAL_NUM_TY; count_ty_out++)
	{ //
		if (ps_TY_user_config->s_TY_out_config[count_ty_out].mode_TY== SINGLE_POSITION)
		{ // ���� ����� �� ���� SINGLE_POSITION
			if (s_state_TY.operation_TY_statys[count_ty_out] != REZ_CHECK_OK)
			{
				continue;
			}; // ���� ����� ������������� ��������� �������� ��
			// ���� �������� � ����������� - �� ������������� ���������
			if (s_state_TY.present_state_TY[count_ty_out] == TY_ON) // ���� ����� ������������� ��� ����������� �� ��� ���������� �������
			{
				TY_SET_OUT(s_rel_out[count_ty_out].port_operation,
						s_rel_out[count_ty_out].pin_operation);
				// ��������� �������� ��������� ��������� � ������ �������� ������ ��
				s_oper_data_TY_present.present_state_TY[count_ty_out] = TY_ON;
				s_oper_data_TY_present.set_state_TY[count_ty_out] = TY_ON;
				s_oper_data_TY_present.operation_TY_statys[count_ty_out] =
						REZ_CHECK_OK;
			}
			if (s_state_TY.present_state_TY[count_ty_out] == TY_OFF)// ���� ����� ������������� ��� ����������� �� ��� ������� �������
			{
				TY_CLEAR_OUT(s_rel_out[count_ty_out].port_operation,
						s_rel_out[count_ty_out].pin_operation);
				// ��������� �������� ��������� ��������� � ������ �������� ������ ��
				s_oper_data_TY_present.present_state_TY[count_ty_out] = TY_OFF;
				s_oper_data_TY_present.set_state_TY[count_ty_out] = TY_OFF;
				s_oper_data_TY_present.operation_TY_statys[count_ty_out] =
						REZ_CHECK_OK;

			}
		}
	}
	// ��������� � ����� ������ ��������� ��������� ���������
	processing_mem_map_write_s_proces_object_modbus((u16*)&s_oper_data_TY_present, (sizeof(S_state_TY) / 2),s_address_oper_data.s_TY_address.status_TY);
}

//---------------������� processing_TY_signal_init_tim---------------
// ������� processing_TY_signal_init_tim -��������� ������������ ��������
// TIM2 - ����������� ������ ��� ���������� �������� ������� ���������� ������������ ���� � ��������� �������
// TIM1 - ��� ������� ������� �������� � ������� �������� ������� �������� ������� �����
//        �� ����� ���������� ������� DP
void processing_TY_signal_init_tim(void) {
	//----------------------------------������������ ������� TIM2------------------------
	RCC_ClocksTypeDef rcc_ClocksTypeDef;
	RCC_GetClocksFreq(&rcc_ClocksTypeDef);
	//----------------------------------������������ ������� TIM1------------------------
	// ��������� ����� ������ ����� ������� - 10 ���
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	TIM1->PSC = rcc_ClocksTypeDef.PCLK1_Frequency / 100000;
	TIM1->PSC = TIM1->PSC * 2 - 1;
	TIM1->CCR1 = TIME_BEFORE_CHECK_COIL;
	TIM1->SR = 0;
	TIM1->CNT = 0;
	TIM1->DIER |= (TIM_DIER_CC1IE); // ��������� ���������� �� ���������
	TIM1->EGR |= TIM_EGR_UG;
	// ����������� ���������� ��������� ����������
	//NVIC_SetPriority(TIM2_IRQn,12);
	NVIC_EnableIRQ(TIM1_CC_IRQn);
	//----------------------------------������������ ������� TIM2------------------------
	// ��������� ����� - 0.1 �� (100 ���)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM2->CR1 |= TIM_CR1_ARPE;
	TIM2->PSC = rcc_ClocksTypeDef.PCLK1_Frequency / 10000;
	TIM2->PSC *= 2; // �� ��������.(���� APB1_PSC>1, ������� ������������ ������� f_APB1 x 2)
	TIM2->ARR = ps_TY_user_config->puls_with * 10;
	TIM2->CCR1=1;
	TIM2->EGR |= (TIM_EGR_UG);
	TIM2->SR = 0;
	TIM2->CNT = 0;
	TIM2->DIER |= ((TIM_DIER_UIE)|(TIM_DIER_CC1IE)); // ��������� ���������� �� ������������ ������� (����� ������� �������� DP)
	                                                 // � �� ������������ (������ �������� ��������� ������� �����)
	// ����������� ���������� ��������� ����������
	NVIC_SetPriority(TIM2_IRQn, 12);
	NVIC_EnableIRQ(TIM2_IRQn);

}

//---------------������� processing_TY_signal_check_coil---------------
// ������� processing_TY_signal_check_coil -�������� ������� �����
//����� �-� ��������� ������ ����� ������� ����� ���������:
// - ���� ��������� � ������� ���������� ������� ��- ��� ������� ����� ������� ���� SP+DP+parallel
// - ���� ��������� � ������ ������ �� ������������ ������� - �������� ���� �������
TY_REZ_CHECK processing_TY_signal_check_coil(void) {
	u8 counter;
	u8 num_out; // ����� ������ ��, ������� ����� � ������� �� ���������
	TY_REZ_CHECK rez_grup_coil_check =REZ_CHECK_OK;

	// ���� � ������� �� ���������� ����� �������, "�"
	// ��� ������ �� ������� ����� ������������� DOUBLE_POSITION, "�"
	// ��������� ������� ����� ���������� - TY_OFF ��
	// ������� �� ����� num_TY+1

	if((s_TY_operation.f_TY)&&
		(ps_TY_user_config->s_TY_out_config[s_TY_operation.num_TY].mode_TY==DOUBLE_POSITION)&&
		(s_TY_operation.state_TY==TY_OFF)	)
	{
		num_out=s_TY_operation.num_TY+1;
	}
	else
	{
		num_out=s_TY_operation.num_TY;
	}

	// ��������� ����� ������� ������� ���������� ��������� �������� ��������� ��������� ������ ��:
	// - ���� ������ �-� ������� � ������� ���������� ������� DP - ��������� ������ �������
	//   ����� ������� �������������� � ������� ���������� ����� ������� (� ������ - ���� ������� ���
	//   ��������� ������ ������� ���, ���� ������� ���� - ��������� ������� ����, ���� ���������������
	//   ���������� ����� - �� ������� ������������ ������)
	// - ���� ������ �-� ������� ����� ������������ ������ - ��������� ��� �������.
	//
	// ������� ������� ������ �� ������� �� ���������: ���� �������� ����� �� ������������� �������� �� ��������� �
	// ������ - �������� � �������� ���������� ������

	for (counter = 0,s_ty_coil_check_grup.num_check_TY=0; counter < TOTAL_NUM_TY; counter++) {
		if ( //---���� � ������� �� ���������� ����� �������,  "�"
		     //---��� ��������� ������������ ������ �� DOUBLE_POSITION, "�"
			 //---�������� ����������� ����� �� �� ����� � ������ �� ����������
			((s_TY_operation.f_TY)&&\
			(ps_TY_user_config->s_TY_out_config[counter].mode_TY == DOUBLE_POSITION) && \
			(counter!=num_out))||\
			//---���� � ������� �� ���������� ����� �������,  "�"
			//---"�" ���� ��� ��������� ������������ ������, PARALLEL_CHANNEL
			//---"�" ����������� �� ����� �� �� ����� � ������ �� ����������
			((s_TY_operation.f_TY)&&\
			(ps_TY_user_config->s_TY_out_config[counter].mode_TY == PARALLEL_CHANNEL) && \
			((num_out+s_address_oper_data.s_TY_address.set_state_TY)!=ps_TY_user_config->s_TY_out_config[counter].num_paralel)))
		{
			continue;
		}
		// �� ���� ��������� ������� - ��������� �������� ����� �� � ������ �� ��������
		else
		{
			s_ty_coil_check_grup.s_TY_is_check[s_ty_coil_check_grup.num_check_TY].num_check_TY=counter;
			s_ty_coil_check_grup.s_TY_is_check[s_ty_coil_check_grup.num_check_TY].rez_check_TY=0;
			s_ty_coil_check_grup.num_check_TY++;
		}
	}
	// ������� ���� ������� ���������� �������� �������
	s_ty_coil_check_grup.f_check_end=RESET;
	// ��������� ������ ���������� ���������
	TIM_Cmd(TIM1, ENABLE);
	// ���� ��������� ��������
	while(!s_ty_coil_check_grup.f_check_end){}
	// ��������� ������ ����������� ��������
	// - �������� ������������ � ��� �����: ����������� ������ ���������� �� (����������� -
	//    ������, ��� �� ��������� �������� ������� ����� ��� ����� ���������, ����� ���������
	//    ������� �� 0-1-0, ��� � �� ����� �������, ����� ��������� ������� ���������� �� 1-0-1)
	// - ����� ����������� ������� ����������� �������, �������� �������� �� ������ �������� ���������
	//   �������, ���������� ������ ���������� �� (��������� � ��������� ���������)
	// - ����� ����������� ������� ����������� �������, �������� �������� �� ������ �������� ���������
	//   �������.
	// ���������� ���������� ��������� � ���� rez_check_TY. ���� � ���������� ��������
	// �������� ������� 0b01 ��� 0b10 - ������� ��������, ���� 0b00(ERROR_XX) ��� 0b11(ERROR_K3) - ��������
	// ����������
	for(counter=0;counter<s_ty_coil_check_grup.num_check_TY;counter++){
		if((s_ty_coil_check_grup.s_TY_is_check[counter].rez_check_TY==ERROR_XX)||
		   (s_ty_coil_check_grup.s_TY_is_check[counter].rez_check_TY==ERROR_K3))
		{
		// ���� �������� ��� ��������� ������ �� ����������� ������� - ������ � ������-������� ������ � �������
		// ERROR_TY_COIL ��������
			rez_grup_coil_check =ERROR_TY_COIL;
			s_oper_data_TY_present.operation_TY_statys[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY]|=ERROR_TY_COIL;
		}
		else  // ���� �������� ����������� ������ - �������� ������ �������� � �������� ������-�������� ������ ��
		{
			s_oper_data_TY_present.operation_TY_statys[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY]&=~ERROR_TY_COIL;
		}
	}

	return rez_grup_coil_check;
}

//------------����������  TIM1_CC_IRQHandler----------------
// ����������  TIM1_CC_IRQHandler - ���������� �� TIM1
// ����� ���������� �������� ������� ���������� �������� �������� � �������� �������/��������� ��R1
// � ���� ������ ���������� ������������ ��� ���y����� �������� ������� �����
void TIM1_CC_IRQHandler(void) {
	static u8 count_interupt = 0;
	u8 counter;
	// ����������� ������ ���������� �������� �������/����������
	if ((TIM1->SR) && (TIM_SR_CC1IF))
	{
		TIM1->SR &= ~(TIM_SR_CC1IF);
	} else
	{
		return;
	}
	TIM1->CNT = 0;
	// ���������� �1 - ������������� ������ ���������� ��
	if (count_interupt == 0)
	{
		TIM1->CCR1 = TIME_TRANSIENT_CHECK_COIL;
		TIM1->EGR |= TIM_EGR_UG;
		// ���������� ��� ������ �� ������
		for (counter = 0; counter < s_ty_coil_check_grup.num_check_TY; counter++) {
			TY_INVERT_OUT(s_rel_out[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].port_operation,
				        s_rel_out[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].pin_operation);
		}
		count_interupt++;
		return;
	}
	// ���������� �2 - ������� �������� �� ����������� ����� � ������ ���������� ��
	else if (count_interupt == 1) {
		TIM1->CCR1 = TIME_TRANSIENT_CHECK_COIL;
		TIM1->EGR |= TIM_EGR_UG;
		for (counter = 0; counter < s_ty_coil_check_grup.num_check_TY; counter++)
		{
			// �������� ��������� �� ����������� ������ � �������� ������
			if (GPIO_ReadInputDataBit(s_coil_check[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].port_operation,
					                  s_coil_check[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].pin_operation))
			{
				s_ty_coil_check_grup.s_TY_is_check[counter].rez_check_TY |= (1 << (count_interupt - 1));
			}
			// ���������� ��� ������ �� ������
			TY_INVERT_OUT(s_rel_out[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].port_operation,
					      s_rel_out[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].pin_operation);

		}
		count_interupt++;
		return;
	}
	// ���������� �3 (���������)- ������� �������� �� ����������� �����
	else if (count_interupt == 2)
	{
		TIM_Cmd(TIM1, DISABLE);            // �������� ������
		TIM1->CCR1 = TIME_BEFORE_CHECK_COIL;
		TIM1->EGR |= TIM_EGR_UG;
		s_ty_coil_check_grup.f_check_end=1;  // ������������ ���� ��������� �������� ��������� �������
		for (counter = 0; counter < s_ty_coil_check_grup.num_check_TY; counter++) {
			// �������� ��������� �� ����������� ������ � �������� ������
			if (GPIO_ReadInputDataBit(s_coil_check[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].port_operation,
								      s_coil_check[s_ty_coil_check_grup.s_TY_is_check[counter].num_check_TY].pin_operation))
			{
				s_ty_coil_check_grup.s_TY_is_check[counter].rez_check_TY |= (1 << (count_interupt - 1));
			}
		}
		count_interupt = 0;
		return;
	}
}

//------------����������  TIM2_IRQHandler----------------
// ����������  TIM2_IRQHandler - ���������� �� ������������ ������� �6.
// ������ ���������� ��������� ����� ������� ����� �������� ������������ �� ���� DP
void TIM2_IRQHandler(void) {
	if(TIM_GetITStatus(TIM2,TIM_IT_Update))
	{
		TIM_ClearFlag(TIM2,TIM_FLAG_Update);
		TIM_Cmd(TIM2,DISABLE); // �������� ������
		TIM_SetCounter(TIM2,0);
		TY_CLEAR_OUT(PORT_GRYP_REL, PIN_GRYP_REL);// ��������� �������� �����
		// ����� �������
		f_dp_time_complite=1; // ������������ ����-������� ��������� ������� �������
		TIM2->CCR1=(TIME_TOTAL_CHECK_COIL*10); // ������������ ��������� ����� �������� ��� ���������� ���������
	}
	else if(TIM_GetITStatus(TIM2,TIM_IT_CC1))
	{
		TIM_ClearFlag(TIM2,TIM_FLAG_CC1);
		if((TIM2->CCR1+TIME_TOTAL_CHECK_COIL*10*2)<TIM2->ARR)// ���� ��������� ���������� �� ��������� � ��������� �������/��������� �������� �� ������� "UPDATE" + �����
		{                                                    // ������ ����� ��������� ��� ���� ���� �������� ��������� �������
			TIM2->CCR1+=(TIME_TOTAL_CHECK_COIL*10);
		}
	}
	else
	{
		TIM2->SR=0;
	}
	// ����� �������
	xSemaphoreGiveFromISR(SemaphoreHandle_ty, &prioryty_take_semaphore_ty);
}

//---------------������� init_TY_gpio---------------
// ������� init_TY_gpio -��������� ������������ ���� ������ �����/������ ������� �������������
// ���������� ������ ��
void processing_TY_signal_init_gpio(void) {
	GPIO_InitTypeDef gpio_TY_init;
	u8 k1;

	// ����� ��
	s_rel_out[0].port_operation = PORT_TY_REL1;
	s_rel_out[1].port_operation = PORT_TY_REL2;
	s_rel_out[2].port_operation = PORT_TY_REL3;
	s_rel_out[3].port_operation = PORT_TY_REL4;
	// ���� ��
	s_rel_out[0].pin_operation = PIN_TY_REL1;
	s_rel_out[1].pin_operation = PIN_TY_REL2;
	s_rel_out[2].pin_operation = PIN_TY_REL3;
	s_rel_out[3].pin_operation = PIN_TY_REL4;

	// ������������ ����� �� �� �����
	for (k1 = 0; k1 < TOTAL_NUM_TY; k1++)
	{
		gpio_enable(s_rel_out[k1].port_operation); // ������� �������� ����
		gpio_TY_init.GPIO_Mode = GPIO_Mode_Out_OD;
		gpio_TY_init.GPIO_Speed = GPIO_Speed_2MHz;
		gpio_TY_init.GPIO_Pin = s_rel_out[k1].pin_operation;
		GPIO_Init(s_rel_out[k1].port_operation, &gpio_TY_init);
		TY_CLEAR_OUT(s_rel_out[k1].port_operation, s_rel_out[k1].pin_operation);
	}

	// ������������ ���� ���������� ������ ���� ��
	gpio_enable(PORT_GRYP_REL); // ������� �������� ����
	gpio_TY_init.GPIO_Mode = GPIO_Mode_Out_OD;
	gpio_TY_init.GPIO_Speed = GPIO_Speed_2MHz;
	gpio_TY_init.GPIO_Pin = PIN_GRYP_REL;
	GPIO_Init(PORT_GRYP_REL, &gpio_TY_init);
	TY_CLEAR_OUT(PORT_GRYP_REL, PIN_GRYP_REL);

	// ������������ ���� ���������� ������������
	gpio_enable(PORT_V_REL); // ������� �������� ����
	gpio_TY_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_TY_init.GPIO_Speed = GPIO_Speed_2MHz;
	gpio_TY_init.GPIO_Pin = PIN_V_REL;
	GPIO_Init(PORT_V_REL, &gpio_TY_init);
	TY_CLEAR_OUT(PORT_V_REL, PIN_V_REL);

	 // ������������ ���� �������� ������� ���������� ������������ ����
	 gpio_enable(PORT_V_INPUT);// ������� �������� ����
	 gpio_TY_init.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	 gpio_TY_init.GPIO_Speed=GPIO_Speed_2MHz;
	 gpio_TY_init.GPIO_Pin=PIN_V_INPUT;
	 GPIO_Init(PORT_V_INPUT,&gpio_TY_init);


	// ����������� ������ �������� ��������� ������� �����
	// ����� �������� ��������� ������� �����
	s_coil_check[0].port_operation = PORT_CHK_COIL1;
	s_coil_check[1].port_operation = PORT_CHK_COIL2;
	s_coil_check[2].port_operation = PORT_CHK_COIL3;
	s_coil_check[3].port_operation = PORT_CHK_COIL4;
	// ���� �������� ��������� ������� �����
	s_coil_check[0].pin_operation = PIN_CHK_COIL1;
	s_coil_check[1].pin_operation = PIN_CHK_COIL2;
	s_coil_check[2].pin_operation = PIN_CHK_COIL3;
	s_coil_check[3].pin_operation = PIN_CHK_COIL4;

	// ������������ ����� �������� ��������� ������� ����� �� ����
	for (k1 = 0; k1 < TOTAL_NUM_TY; k1++)
	{
		gpio_enable(s_rel_out[k1].port_operation); // ������� �������� ����
		gpio_TY_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		gpio_TY_init.GPIO_Speed = GPIO_Speed_10MHz;
		gpio_TY_init.GPIO_Pin = s_coil_check[k1].pin_operation;
		GPIO_Init(s_coil_check[k1].port_operation, &gpio_TY_init);
	}
}

// �������� ������������ ��������� GPIO
void gpio_enable(GPIO_TypeDef *IN_GPIO){
	if (IN_GPIO == GPIOA) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	} else if (IN_GPIO == GPIOB)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	} else if (IN_GPIO == GPIOC)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	} else if (IN_GPIO == GPIOD)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	}
}

//---------------������� processing_TY_signal_set_check_par---------------
// ������� processing_TY_signal_set_check_par -���������:
// - ���������� ����� ������� �������� ����� ���������� ������� ����� ������������ � �������� ������ ������
// - ���������� ����� ������� ����������� ��������
void processing_TY_signal_set_check_par(void) {
	s_cycling_check.num_cycling_check = 0;
    //���������� ����� ������� �������� ����� ����������, ������� ����� ������������ � �������� ������ ������
	if(s_ty_coil_check_grup.f_limit_check)//���� ������ ������������ ����� ��������
	{
		a_check_fun_exception[0]=&processing_TY_signal_check_v;
		a_check_fun_exception[1]=&processing_TY_signal_check_rel_contact;
		a_check_fun_exception[2]=&processing_TY_signal_check_ty_rel_on;
		a_check_fun_exception[3]=&processing_TY_signal_check_gryp_rel_on;
		a_check_fun_exception[4]=&processing_TY_signal_check_gryp_rel_off;
		a_check_fun_exception[5]=&processing_TY_signal_check_ty_rel_off;
	}

	// ���������� ����� ������� ����������� ��������

	// �������� - �������� ���������� ������������ ����
	s_cycling_check.pf_fun[s_cycling_check.num_cycling_check].pf_check = &processing_TY_signal_check_v;
	s_cycling_check.pf_fun[s_cycling_check.num_cycling_check].maska_error = ERROR_V_INPUT;
	s_cycling_check.num_cycling_check++;

	// �������� - �������� ��������� ���������
	s_cycling_check.pf_fun[s_cycling_check.num_cycling_check].pf_check =	&processing_TY_signal_check_rel_contact;
	s_cycling_check.pf_fun[s_cycling_check.num_cycling_check].maska_error = ERROR_TY_REL_CONTACT;
	s_cycling_check.num_cycling_check++;

	// �������� - �������� ������� �����
	s_cycling_check.pf_fun[s_cycling_check.num_cycling_check].pf_check =	&processing_TY_signal_check_coil;
	s_cycling_check.pf_fun[s_cycling_check.num_cycling_check].maska_error = ERROR_TY_COIL;
	s_cycling_check.num_cycling_check++;

}

//---------------������� processing_TY_signal_check---------------
// ������� processing_TY_signal_check - ��������� �������� ���������� ������� �� ���������� �� (��� MODBUS)
// ������������ �������� ��������� ��������� ������ ��:
// - ������ �������;
// - ���������������� ����������� ���������� ��������� �� ������� �� �� ������ �������
REZ_REQ_CHEACK_SLAVE processing_TY_signal_modbus_check(void* req) {
	S_ty_comand *ps_comand = req;
	u8 number_out_TY;
	u16 status_TY;
	// ���� ������ �� �������� ���������
	if (ps_TY_user_config->state == DISABLE)
	{
		return ILLEGAL_FUNCTION; // ���������� ���� "����������� �������"
	}
	// �������� ��������� ������� (�� ������ ��������������� ���/���� 0�FF00/0x0000 )
	//���� �������� �� ������� ����� ���������� �� ������������� ���������� - ����� � �������
	if ((ps_comand->state != TY_ON) && (ps_comand->state != TY_OFF))
	{
		return ILLEGAL_DATA_VALUE;
	}
	// ��������������� ��
	number_out_TY = ps_comand->number
			- s_address_oper_data.s_TY_address.set_state_TY;
	//�������� ������ �� �������� ������������
	if ((ps_TY_user_config->s_TY_out_config[number_out_TY].mode_TY == NO_OUT)
			|| (ps_TY_user_config->s_TY_out_config[number_out_TY].mode_TY== PARALLEL_CHANNEL))
	{
		return ILLEGAL_DATA_ADRESS; // ���������� ���� "����������� �����"
	}
	// �������� ������������ ���������� �� ���������� ���� ������� �������
	if ((number_out_TY % 2)	&& (ps_TY_user_config->s_TY_out_config[number_out_TY].mode_TY== DOUBLE_POSITION))
	{
		return ILLEGAL_DATA_ADRESS; // ���������� ���� "����������� �����"
	}

	// �������� ������-��������
	processing_mem_map_read_s_proces_object_modbus(&status_TY, 1,s_address_oper_data.s_TY_address.status_TY);
	if ( STATUS_STATE(status_TY))
	{
		return SLAVE_DEVICE_FALIURE; // ���������� ���� "������������� ������������"
	}
	// �������� �� ���������� � ������ ������ ������� ��
	if (s_TY_operation.f_TY)
	{
		return SLAVE_DEVICE_BUSY; // ���������� ���� "����������� ����� ����������� ���������� �������"
	}
	return (0);
}

//------------������� processing_TY_signal_out_check-----------------------------------
// ������� processing_TY_signal_out_check - ��������� �������� ������� ����� ������ �� �������������� ����������� ��������� ��������� ������� ��
// ����� ������� ������������� �������� ������� �1 modbus
// ������� ���������:
// *p_check_address - ��������� �� ������ ������������������ (u16)address, (u16)num_reg
// �������� ���������:
// REQ_SLAVE_OK - ��������� ��������� �������
// ILLEGAL_DATA_ADRESS - ������������ �����
REZ_REQ_CHEACK_SLAVE processing_TY_check_is_coil_status_address_modbus(void* p_check_address) {
	u16 address = *(u16*) p_check_address;
	u16 num_reg = *(u16*) ((u16*) p_check_address + 1);
	// ����� ����������� ������� ��������� �Y
	if ((address >= s_address_oper_data.s_TY_address.present_state_TY)&&
			((address + num_reg - 1)< (s_address_oper_data.s_TY_address.present_state_TY + TOTAL_NUM_TY * NUM_REG_TY)))
	{
		return REQ_SLAVE_OK;
	}
	// ����� ���������� �� ��������� ���������� ������� ��������� ��
	return ILLEGAL_DATA_ADRESS;
}

//------------������� processing_TY_signal_address_check-----------------------------------
// ������� processing_TY_signal_address_modbus_check - ��������� �������� ������� ����� ������ �� �������������� ��������� ��������� ��
// ����� ������� ������������� �������� ������� �5 modbus
// ������� ���������:
// *p_check_address - ��������� �� ������ ������������������ (u16)address
// �������� ���������:
// REQ_SLAVE_OK - ��������� ��������� �������
// ILLEGAL_DATA_ADRESS - ������������ �����
REZ_REQ_CHEACK_SLAVE processing_TY_check_force_single_coil_address_modbus(void* p_check_address) {
	u16 address = *(u16*) p_check_address;
	// ����� ����������� ������� ��������� ��������� ��
	if ((address >= s_address_oper_data.s_TY_address.set_state_TY)
			&& (address	< (s_address_oper_data.s_TY_address.set_state_TY+ TOTAL_NUM_TY)))
	{
		return REQ_SLAVE_OK;
	}
	// ����� ���������� �� ��������� ���������� ������� ���������
	return ILLEGAL_DATA_ADRESS;
}

//---------------������� processing_TY_signal_operation---------------
// ������� processing_TY_signal_operation - ������ � ������� �� ���������� ������� ��
u8 processing_TY_signal_operation(void* req, u8 num_peyload_data,u16 addres_data) {
	S_ty_comand *ps_comand = req;
	// ��� �������� ��������� ����� ���� ������� ������� �5. !!!!
	// ������ � �������
	s_TY_operation.num_TY = ps_comand->number - s_address_oper_data.s_TY_address.set_state_TY; // ��������� ���������� ����� ������ �� (0-(TOTAL_NUM_TY-1))
	s_TY_operation.state_TY = ps_comand->state;
	s_TY_operation.f_TY = REDY_COMAND;
	return (0);
}

//---------------������� processing_TY_signal_clear---------------
// ������� processing_TY_signal_clear - �������� ������� �� ���������� ������� ��
void processing_TY_signal_clear(void) {
	s_TY_operation.num_TY = 0;
	s_TY_operation.state_TY = 0;
	s_TY_operation.f_TY = 0;
}

//---------------������� processing_TY_signal_DP_TY---------------
void processing_TY_signal_DP_TY(u8 num_ty, S_state_TY* p_status_TY) {
	u8 num_paralel_ty;
	// ���� ���������� ������ ���� �� ����������� ���������� ������� ���� - �����
	if (STATUS_STATE(p_status_TY->status_TY))
	{
		return;
	}
	// ��������� � ���������� �������
	if (s_TY_operation.state_TY == TY_OFF) // ���� ���������� ��������� �������� ����������
	{
		num_ty++; // ������� �� ����� ����������
	}
	//
	//--------- ��������� �������� ��������� ����� �� ---------------
	// �������� �������� �����
	TY_SET_OUT(s_rel_out[num_ty].port_operation,
			s_rel_out[num_ty].pin_operation);
	// ����������� ����� ������������� ������ � ��������� ����������
	if ((ps_TY_user_config->s_TY_out_config[num_ty].f_paralel_out)&& (s_TY_operation.state_TY == TY_OFF))
	{
		num_paralel_ty = ps_TY_user_config->s_TY_out_config[num_ty].num_paralel	- s_address_oper_data.s_TY_address.set_state_TY;
		TY_SET_OUT(s_rel_out[num_paralel_ty].port_operation,s_rel_out[num_paralel_ty].pin_operation);// ������� ����� ������������ ������ !!!!!!
	}
	// ��������� �������� ��������� ����� ��
	if (processing_TY_signal_chek_processing(&processing_TY_signal_check_ty_rel_on, ERROR_TY_REL_ON, p_status_TY,num_ty))
	{
		processing_TY_signal_disable_rellay(num_ty); // �������� �������� ����� �� � �������� �����
		return;
	}

	//--------- ��������� �������� ��������� �������� ����� ---------------
	// �������� �������� �����
	TY_SET_OUT(PORT_GRYP_REL, PIN_GRYP_REL);
	// ��������� �������� ��������� ��������� �����
	if (processing_TY_signal_chek_processing(&processing_TY_signal_check_gryp_rel_on, ERROR_GRYP_REL_ON,p_status_TY, num_ty))
	{
		processing_TY_signal_disable_rellay(num_ty); // �������� �������� ����� �� � �������� �����
		return;
	}

	// �������� ������ ��� ������� ������� ������� ����� �������� �� ���� DP
	TIM_Cmd(TIM2,ENABLE);
	while(1){
		// ������� ��������� �������, ������� ��������� �� TIM1. ������ �������� �����
		// ������� ����� ���������� �������
		xSemaphoreTake(SemaphoreHandle_ty, portMAX_DELAY); // ��� ���������� ���� �������� �������
		if(f_dp_time_complite)// ���� ������� ����� ��������, �����
		{
			f_dp_time_complite=0;
			break;
		}
		// �������� �-� �������� ��������� ������� �����. ���� �������� ��������� - ����� �� �-�
		if(processing_TY_signal_chek_processing(&processing_TY_signal_check_coil,ERROR_TY_COIL,p_status_TY,num_ty))
		{
			processing_TY_signal_disable_rellay(num_ty);
			return;
		}
	}

	//--------- �������� ����������� ��������� ����� ������������ � ���������� ������� ---------------
	// ��������� �������� ���������� ��������� �����
	if (processing_TY_signal_chek_processing(&processing_TY_signal_check_gryp_rel_off, ERROR_GRYP_REL_OFF,p_status_TY, num_ty))
	{
		processing_TY_signal_disable_rellay(num_ty); // �������� �������� ����� �� � �������� �����
		return;
	}

	//--------- ��������� �������� ����������� ����� �� --------------
	// ��������� �������� �����, � ����������� (���� ������ ������������� � ������ ������� DP_OFF)
	TY_CLEAR_OUT(s_rel_out[num_ty].port_operation,
			s_rel_out[num_ty].pin_operation);
	if ((ps_TY_user_config->s_TY_out_config[num_ty].f_paralel_out)
			&& (s_TY_operation.state_TY == TY_OFF))
	{
		TY_CLEAR_OUT(s_rel_out[num_paralel_ty].port_operation,
				s_rel_out[num_paralel_ty].pin_operation);
	}
	// ��������� �������� ���������� ����� ��
	if (processing_TY_signal_chek_processing(&processing_TY_signal_check_ty_rel_off, ERROR_TY_REL_OFF,p_status_TY, num_ty))
	{
		processing_TY_signal_disable_rellay(num_ty); // �������� �������� ����� �� � �������� �����
		return;
	}
}

//---------------������� processing_TY_signal_chek_processing---------------
// ������� processing_TY_signal_chek_processing - ��������� �������� �������� ���������� ������ �� �������� �� � ��������� ����������� ���������
// ������� ���������:
// p_fun_chek      - ��������� �� �-� ��������;
// paramiter_chek  - ���� �� ��� ���������� ��������
// status          - ��������� �� ���������� � ������� ����� ���������� ���� ���������� ��������
// �������� ���������:
// 0 - �������� ���������� �������
// 1 - �������� ���������� �����������
u8 processing_TY_signal_chek_processing(F_check p_fun_chek,TY_REZ_CHECK paramiter_chek, S_state_TY* p_status, u8 num_ty) {
	TY_REZ_CHECK rez_chek_ty;
	// ���� �-� �������� ���������� � ������ ���������� - ������������� �����
	if(IS_FUN_EXCEPTION(p_fun_chek)){return (0);}
	// ��������� ��������
	rez_chek_ty = p_fun_chek();
	(p_status->status_TY) &= ~(paramiter_chek); // ������� ������ ������������ ���������
	(p_status->status_TY) |= rez_chek_ty; // ������������ ������ �������� ����������� ��������

	// ���� ���� �������� ������� �� ����������, ������ ����� �� ��� �������� �� 1. ���� ������������
	// ������� ���� DP, ��� ��������� ������ ������� ������� �� ����� �������� ����� ��
	if (s_TY_operation.state_TY == TY_OFF) { // ���� ���������� ��������� �������� ����������
		num_ty--; // �������� ����� ��
	}
	if (rez_chek_ty)// ���� �������� ���������� �����������
	{
		// ��������� ������ � �������� ��������� ������� ��
		p_status->present_state_TY[num_ty] = TY_NOT_SET; //��������� �� ���������� ������
		p_status->operation_TY_statys[num_ty] |= rez_chek_ty;
		return (1);

	} else {
		// ��������� ������ � �������� ��������� ������� ��
		p_status->present_state_TY[num_ty] = p_status->set_state_TY[num_ty]; //�������� ��������� ������ ��
		//p_status->operation_TY_statys[num_ty] |= rez_chek_ty;
		return (0);
	}
}

//---------------������� processing_TY_signal_disable_rellay---------------
// ������� processing_TY_signal_disable_rellay - ��������� ���������� ���� ����� ������� ������������� � ������� ���������� �� � ��������� �����
// ������� ���������:
// num_ty      - ����� ������������� ����� ��;
void processing_TY_signal_disable_rellay(u8 num_ty) {
	u8 num_paralel_ty;
	TY_CLEAR_OUT(s_rel_out[num_ty].port_operation,
			s_rel_out[num_ty].pin_operation);
	// ����������� ����� ������������� ������ � ��������� ����������
	if ((ps_TY_user_config->s_TY_out_config[num_ty].f_paralel_out)
			&& (s_TY_operation.state_TY == TY_OFF))
	{
		num_paralel_ty = ps_TY_user_config->s_TY_out_config[num_ty].num_paralel
				- s_address_oper_data.s_TY_address.set_state_TY;
		TY_CLEAR_OUT(s_rel_out[num_paralel_ty].port_operation,
				s_rel_out[num_paralel_ty].pin_operation);
		// ������� ����� ������������ ������ !!!!!!
	}

	TY_CLEAR_OUT(PORT_GRYP_REL, PIN_GRYP_REL);
	// ��������� �������� �����
}

//---------------������� processing_TY_signal_SP_TY---------------
// ������� processing_TY_signal_SP_TY - ��������� TY ���� SP
void processing_TY_signal_SP_TY(u8 num_ty, S_state_TY* status_TY) {
	//��� �� ������� ���� SP ������������ ��������:
	// - ��� ������ � ���������� ������� ���������� ������������� ����, ��������� ������� �����
	// - ��� ������ � ���������� ��������� ������� �����
	if (status_TY->status_TY & ((ERROR_V_INPUT) | (ERROR_TY_COIL))) {
		return;
	}
	if (s_TY_operation.state_TY == TY_ON) { // ���� ���������� ��������� �������� ���������
		TY_SET_OUT(s_rel_out[num_ty].port_operation,s_rel_out[num_ty].pin_operation);
	}
	else
	{
		TY_CLEAR_OUT(s_rel_out[num_ty].port_operation,s_rel_out[num_ty].pin_operation);
	}
	status_TY->present_state_TY[num_ty] = status_TY->set_state_TY[num_ty];
	status_TY->operation_TY_statys[num_ty] = REZ_CHECK_OK;
}

//---------------������� processing_TY_signal_check_V---------------
// ������� processing_TY_signal_check_V - ��������� �������� ������� ������������ ���� � �����
TY_REZ_CHECK processing_TY_signal_check_v(void) {
	u8 state = 0;
	// ��������� ������������� ����
	TY_SET_OUT(PORT_V_REL, PIN_V_REL);
	vTaskDelay(2); // ���������� ������ �����������
    state = processing_TY_signal_read_v_input(); // ������� ��������� ���������� ���� ����
	TY_CLEAR_OUT(PORT_V_REL, PIN_V_REL);
	// �������� ������������� ����
	vTaskDelay(1); // ���������� ������ ����������
	if (state)
	{
		return REZ_CHECK_OK;
	} // ���� ���������� ���� ���� ����������� - ������������� ���������
	else
	{
		return ERROR_V_INPUT;
	}
}

//---------------������� processing_TY_signal_read_v_input---------------
// ������� processing_TY_signal_read_v_input - ��������� ���������� ���� ���������� ���� ���� (��� ����������� ��� � �����������)
TY_STATE_V_INPUT processing_TY_signal_read_v_input(void) {
	u8 state = TY_READ_IN_0;
	u8 count_ms;
	u8 count_set_mes = 0; // ������� �-�� ��������� ��  �� ������������� ������� (���������� ���� ���� ������������)
	for (count_ms = 0; count_ms < NUM_MES_V; count_ms++)
	{
		//                  ���� ����
		// �� �����          1    0
		// ����� �-�         0    1
		if (!GPIO_ReadInputDataBit(PORT_V_INPUT, PIN_V_INPUT)) // ��� 1 �� ����� (���������� ����)
		{
			count_set_mes++;
		}
		// ���� ���������� ��������� ���������� �� ������������� ������� ������ �������� - ������
		// ������ ���������� ���� ���� ������������
		if (count_set_mes >= 4 )
		{
			state = TY_READ_IN_1;
			return state;
			break;
		}
		vTaskDelay(TIME_MES_V);
	}
	return state;
}

//---------------������� processing_TY_signal_check_rel_contact---------------
// ������� processing_TY_signal_check_rel_contact - ��������� �������� ��������� ��������� �����
TY_REZ_CHECK processing_TY_signal_check_rel_contact(void) {
	u8 state = 0;
	TY_CLEAR_OUT(PORT_V_REL, PIN_V_REL);
	// �������� ������������� ����
	vTaskDelay(2); // ���������� ������ �����������
	state = processing_TY_signal_read_v_input(); // ������� ��������� ���������� ���� ����
	if (!(state))
	{
		return REZ_CHECK_OK;
	} // ���� ���������� ���� ���� �� ����������� - ������������� ���������
	else
	{
		return ERROR_TY_REL_CONTACT;
	}
}

//---------------������� processing_TY_signal_check_ty_rel_on---------------
// ������� processing_TY_signal_check_ty_rel_on - ��������� �������� ��������� ����� ���������� TY
TY_REZ_CHECK processing_TY_signal_check_ty_rel_on(void) {
	u8 state = 0;
	TY_CLEAR_OUT(PORT_V_REL, PIN_V_REL);
	// �������� ������������� ����
	vTaskDelay(TIME_REL_COMUTATION); // ���������� ������ �����������
	state = processing_TY_signal_read_v_input(); // ������� ��������� ���������� ���� ����
	if (state)
	{
		return REZ_CHECK_OK;
	} // ���� ���������� ���� ���� ����������� - ������������� ���������
	else
	{
		return ERROR_TY_REL_ON;
	}
}

//---------------������� processing_TY_signal_check_gryp_rel---------------
// ������� processing_TY_signal_check_gryp_rel - ��������� �������� ��������� ��������� �����
TY_REZ_CHECK processing_TY_signal_check_gryp_rel_on(void) {
	u8 state = 0;
	vTaskDelay(TIME_ELEKTRIK_KEY_COMUTATION); // ���������� ������ ����������� ��������� �����
	state = processing_TY_signal_read_v_input(); // ������� ��������� ���������� ���� ����
	if (!(state))
	{
		return REZ_CHECK_OK;
	} // ���� ���������� ���� ���� �� ����������� - ������������� ���������
	else
	{
		return ERROR_GRYP_REL_ON;
	}
}

//---------------������� processing_TY_signal_check_gryp_rel---------------
// ������� processing_TY_signal_check_gryp_rel - ��������� �������� ���������� ��������� �����
TY_REZ_CHECK processing_TY_signal_check_gryp_rel_off(void) {
	u8 state = 0;
	vTaskDelay(TIME_ELEKTRIK_KEY_COMUTATION); // ���������� ������ �����������
	state = processing_TY_signal_read_v_input(); // ������� ��������� ���������� ���� ����
	if (state)
	{
		return REZ_CHECK_OK;
	} // ���� ���������� ���� ���� ����������� - ������������� ���������
	else
	{
		return ERROR_GRYP_REL_OFF;
	}
}

//---------------������� processing_TY_signal_check_gryp_rel---------------
// ������� processing_TY_signal_check_gryp_rel - ��������� �������� ���������� �����
TY_REZ_CHECK processing_TY_signal_check_ty_rel_off(void) {
	u8 state = 0;
	vTaskDelay(TIME_REL_COMUTATION); // ���������� ������ �����������
	state = processing_TY_signal_read_v_input(); // ������� ��������� ���������� ���� ����
	if (!(state))
	{
		return REZ_CHECK_OK;
	} // ���� ���������� ���� ���� �� ����������� - ������������� ���������
	else
	{
		return ERROR_TY_REL_OFF;
	}
}

// ---------�������  TY_calc_address_oper_reg ---------
// ������� TY_calc_address_oper_reg - ��������� ������� ������� ��������� (������� ����������� ���������) ���������� ������������� ������
// ������� ���������:
// ps_TY_address           - ��������� �� ��������� ������� ��������� ���������� (���� � ���������� ��������� ������� ����������);
// adres_start             - ��������� ����� ����������� ��������� ���������� � ����� ������. ������������ ���� �������������� ����� � ��� ��������� ��������� ����������
// �������� ���������:
//                         - ����� ���������� �������� ����������
u16 TY_calc_address_oper_reg(S_TY_address *ps_TY_address, u16 adres_start) {
	ps_TY_address->status_TY = adres_start;
	ps_TY_address->present_state_TY = ps_TY_address->status_TY
			+ NUM_REG_STATUS_TY;
	ps_TY_address->set_state_TY = ps_TY_address->present_state_TY + TOTAL_NUM_TY;
	ps_TY_address->operation_TY_statys = ps_TY_address->set_state_TY
			+ TOTAL_NUM_TY;
	adres_start = ps_TY_address->operation_TY_statys + TOTAL_NUM_TY;
	return adres_start;
}
;

//---------------������ processing_TY---------------
// ������ processing_TY - ���������:����������� �������� ������� ���� ���� (�������� ����������),
// ��������� ������� ��...
// ��������� ON � ��������� 0xFF00;
// ��������� OFF � ��������� 0x0000.
// (��� ��������� �������� �������� �������� �������������,
// �� �������� � ��������� ��������� �������,  ������� ����� �� ������ �������� � ���� ������ �������).
void t_processing_TY(void *pvParameters) {
	u8 k1;
	TY_REZ_CHECK rez_chek_ty_modul;

	// �������� ���������������� ������ � ��������� �� ��������� ������������,
	// � ����� ������� ��������
	if (processing_TY_fill_S_TY((u8*) pvParameters))
	{
		// ����� ������ � ������ ������� ����������
		SET_GLOBAL_STATUS(DEV_3);
		// �������� ������ � ������ ������� ��

		vTaskDelete(NULL); // ���� �������� �� �������� - ������� ������ + �������� ������������ + ������ � ����� ������ !!!!
	};

	// ��������� ������������ ����������
	//������������ ���������� ���� ��
	processing_TY_signal_init();
	while (1)
	{
		//status_TY=0;
		//mem_map_processing_read_s_proces_object_modbus(&status_TY,1,s_address_oper_data.s_oper_data_TY.status_TY);
		//����������� ����������� �������� ��������� ���������
		for (k1 = 0; k1 < s_cycling_check.num_cycling_check; k1++) {
			if(IS_FUN_EXCEPTION( s_cycling_check.pf_fun[k1].pf_check)){continue;}
			rez_chek_ty_modul = s_cycling_check.pf_fun[k1].pf_check();
			s_oper_data_TY_present.status_TY &=~(s_cycling_check.pf_fun[k1].maska_error); // ������� ������ ������������ ���������
			s_oper_data_TY_present.status_TY |= rez_chek_ty_modul; // ������������ ������ ���������� ��������
		}
		// ���� ���� ������� ������� �� � ������ ������� ������ (���� ������������� ���������)
		if ((!(s_oper_data_TY_present.status_TY)) && (s_TY_operation.f_TY)) {
			//if((s_TY_operation.f_TY)){ // ������ ��� �������� ���������������� ������� ������
			// ��������� ������ ��������������� �������� ������ ��, � ���������� ������� �����
			// ������� ���������� ������� ��, � ���������� ����������
			s_oper_data_TY_present.set_state_TY[s_TY_operation.num_TY] =s_TY_operation.state_TY;
			s_oper_data_TY_present.present_state_TY[s_TY_operation.num_TY] =TY_NOT_SET;
			//s_oper_data_TY_present.operation_TY_statys[s_TY_operation.num_TY]=TY_NOT_DEF;
			processing_TY_signal_update_TY_state(&s_oper_data_TY_present);
			// ������� ��� �� �� ��������� ������������ � �������� ������������
			switch (ps_TY_user_config->s_TY_out_config[s_TY_operation.num_TY].mode_TY)
			{
				case SINGLE_POSITION:processing_TY_signal_SP_TY(s_TY_operation.num_TY,&s_oper_data_TY_present);
					break;
				case DOUBLE_POSITION:processing_TY_signal_DP_TY(s_TY_operation.num_TY,&s_oper_data_TY_present);
					break;
				default:
					break;
			}
			processing_TY_signal_clear();
		}
		// �������� ������ ������� ������ �� � ����� ������ � ���� ������
		processing_TY_signal_update_TY_state(&s_oper_data_TY_present);

	}
}

//----------------������� processing_TY_signal_update_TY_state-------------------------------
// ������� processing_TY_signal_update_TY_state - ��������� ������ �������� ������ ��
// ���������� ������������ �������� ������ �� ��
void processing_TY_signal_update_TY_state(S_state_TY *ps_present) {
	u8 counter;
	u16 *p_present = (u16*) ps_present; // ����������� ��������� ��������
	u16 *p_previous; // �������� ��������� �������� (������� ���� ��������)
	S_state_TY s_oper_data_TY_previous;

	// �������� �� ����� ������ ������-�������� ��������� ������ ��
	processing_mem_map_read_s_proces_object_modbus((u16*)&s_oper_data_TY_previous, sizeof(S_state_TY) / 2,s_address_oper_data.s_TY_address.status_TY);
	p_previous=(u16*)&s_oper_data_TY_previous;
	// ��������� ������-��������� ��������� ��������� � �����������.
	// ���� ��������� ����������  - ������ � ���������� ������-������� ����������
	if(s_oper_data_TY_previous.status_TY!=ps_present->status_TY)
	{
		if(ps_present->status_TY)
		{
			SET_GLOBAL_STATUS(DEV_3);
		}
		else{
			RESET_GLOBAL_STATUS(DEV_3);
		}
	}
	// ��������� �������� ����������� ��������� ������ �� � ���������, ���� ������� �� ���������� - ������������ ��� �������� ������ ��
	for (counter = 0; counter < (sizeof(S_state_TY) / 2); counter++) { // /2 - ������, ��� sizeof ��������� ������ � ������, � �������� �� ��������� � 2-� ������� �������
		if ((*p_present) != (*p_previous)) { // ���� ������� ��������� ������� � �������������� ������� �� �����
			// ���������� � ����� ������ ���� ��������� ������ ��
			processing_mem_map_write_s_proces_object_modbus((u16*)ps_present, (sizeof(S_state_TY) / 2),s_address_oper_data.s_TY_address.status_TY );
            break;
		}
		p_present++; // �������� � ���������� ���������
		p_previous++;
	}
    //��������� ����������� � �������� ��������� ������� �� ���� SP
	for(counter=0;counter<TOTAL_NUM_TY;counter++)
	{
		// ���� ���������� ��������� ������ SP "�" ���������� ������������ ���� � ����� - ����������
		// ����������������� ��� ��������
		if((!((ps_present->status_TY)&(ERROR_V_INPUT)))&&\
				(ps_present->present_state_TY[counter]!=s_oper_data_TY_previous.present_state_TY[counter])&&\
				(ps_TY_user_config->s_TY_out_config[counter].mode_TY==SINGLE_POSITION)
				)
		{
			// ����� ��������� ������� ������ �� ���� �� ������ ���������� ((
			FLASH_OPERATION_write_flash_page_16b((u16*) ps_present,sizeof(S_state_TY) / 2, TY_FLASH_PAGE);
			break;
		}

	}
}

