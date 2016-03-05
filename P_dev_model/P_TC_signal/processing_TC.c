#include "processing_TC.h"
#include "processing_mem_map_extern.h"
#include "flash_operation.h"

xSemaphoreHandle SemaphoreHandle_spi;
portBASE_TYPE prioryty_take_semaphore;
// ����� ��������� �������� ������������ ������ �Y
S_TC_user_config *ps_TC_user_config;
// ���������� ���������� ���������� ���� � ��������� ��
S_reaz_read_TC s_reaz_read_TC={0b10101010,0};
// ��������� ��������� ��������� ��
S_total_TC_processing s_TC_processing;
// ��������� ������� ����������� ��������� (�� mem_map_processing.c)
extern S_address_oper_data s_address_oper_data;


//----------������� processing_TC_init----------------
//������� processing_TC_init - ������������ ��������� ������� ������������� ���
//                             ���������� �������� ��
void processing_TC_init(void){
	// ������ ������� ��� ���������� tim3
	vSemaphoreCreateBinary(SemaphoreHandle_spi);
	// ������������ �������� ������ �������� ��
	processing_TC_init_struct();
	// ������������ SPI ��� ���������� �������� ��
	processing_TC_init_spi1();
	// ������������� �������� ������
	processing_TC_init_gpio();
	// ������������� ������ ��� ������� ���������� ������� ��������� �������� ��
	processing_TC_init_tim3();
	// ����� ������� callback �������� �������������� ������ ������ ������� READ_INPUT_STATUS (�2) ��� Modbus
	modbus_callback_address_check(&processing_TC_check_is_input_status_modbus,READ_INPUT_STATUS);
	// ��������� �������� ����������������� ����� �� ��� ������� ������� �� ���������� MODBUS
	modbus_callback_add_check(&processing_TC_signal_check, READ_INPUT_STATUS);

	//// 	������ ��� ��������!!!!
#ifdef TC_SOFT_CHECK
	check();
#endif

}

#ifdef TC_SOFT_CHECK
// 	������ ��� ��������!!!!
void check(void){
	GPIO_InitTypeDef GPIO_InitTypeDef;
	// ��������� ����� ���������� �������� ������ ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitTypeDef.GPIO_Pin=PIN_SOFT_CHECK_B2;
	GPIO_InitTypeDef.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(PORT_SOFT_CHECK_B2, &GPIO_InitTypeDef);
	GPIO_ResetBits(PORT_SOFT_CHECK_B2,PIN_SOFT_CHECK_B2);
	// ��������� ����� ���������� �������� ������ ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitTypeDef.GPIO_Pin=PIN_SOFT_CHECK_B12;
	GPIO_InitTypeDef.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(PORT_SOFT_CHECK_B12, &GPIO_InitTypeDef);
	GPIO_ResetBits(PORT_SOFT_CHECK_B12,PIN_SOFT_CHECK_B12);
}
#endif


//------------������� processing_TC_check_is_input_status_modbus-----------------------------------
// ������� processing_TC_check_is_input_status_modbus - ��������� �������� ������� ����� ������ �� �������������� ��������� ��������� ��
// ����� ������� ������������� �������� ������� �2 modbus
// ������� ���������:
// *check_data - ��������� �� ������ ������������������ (u16)address, (u16)num_reg
// �������� ���������:
// REQ_SLAVE_OK - ��������� ��������� �������
// ILLEGAL_DATA_ADRESS - ������������ �����
REZ_REQ_CHEACK_SLAVE processing_TC_check_is_input_status_modbus(void* p_check_address){
	u16 address= *(u16*)p_check_address;
	u16 num_reg= *(u16*)((u16*)p_check_address+1);
	// ����� ����������� ������� ��������� ��
	if((address>=s_address_oper_data.s_TC_address.rez_TC)&&
			((address+num_reg-1)<(s_address_oper_data.s_TC_address.rez_TC+TOTAL_NUM_TC)))
	{
		return REQ_SLAVE_OK;
	}
	// ����� ���������� �� ��������� ���������� ������� ��������� ��
	return ILLEGAL_DATA_ADRESS;
}


INIT_MBB_Rezult processing_TC_fill_S_TS(u8 *read_data) {
	// ������� ��������� TC
	ps_TC_user_config=(S_TC_user_config*)read_data;
	//memcpy(&ps_TC_user_config->read_data,sizeof(S_tc_user_config));
	return MBB_INIT_OK;
}


//----------������� processing_TC_init_struct----------------
//������� processing_TC_init_struct - ������������ �������� ������ �������� ��
void processing_TC_init_struct(void){
	u8 k1;
	for(k1=0;k1<TOTAL_NUM_TC;k1++){
		s_TC_processing.as_one_TC_processing[k1].tc_counter=0;
		s_TC_processing.as_one_TC_processing[k1].tc_is_procssing=IS_NOT_PROCESSING;
		s_TC_processing.as_one_TC_processing[k1].tc_state=0;
	}
}


//----������� processing_TC_init_gpio--------------
//������� processing_TC_init_gpio - ������������� gpio ��� ������ �� �������������
void processing_TC_init_gpio(void){
	GPIO_InitTypeDef GPIO_InitTypeDef_TC;
	// ��������� SN66HVS882 LD
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef_TC.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitTypeDef_TC.GPIO_Pin=LD_PIN;
	GPIO_InitTypeDef_TC.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(LD_PORT, &GPIO_InitTypeDef_TC);
	// ��������� SN66HVS882 TOK
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef_TC.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitTypeDef_TC.GPIO_Pin=TOK_PIN;
	GPIO_InitTypeDef_TC.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(TOK_PORT, &GPIO_InitTypeDef_TC);
	// ��������� 24 v IN
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef_TC.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitTypeDef_TC.GPIO_Pin=V_PIN;
	GPIO_InitTypeDef_TC.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(V_PORT, &GPIO_InitTypeDef_TC);
}

//----������� processing_TC_tim3--------------
//������� processing_TC_tim3 - ������������� ������ ��� ������� ���������� ������� ��������� �������� ��
void processing_TC_init_tim3(void){
	u32 tim3_com=0;
	TIM_TimeBaseInitTypeDef tim3_InitTypeDef;
	RCC_ClocksTypeDef rcc_ClocksTypeDef;
	// �������� ������� ���� ��� ��
	RCC_GetClocksFreq(&rcc_ClocksTypeDef);
	// �������� ������������ TIM3, ���� APB2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	TIM_DeInit(TIM3);
	// �������� ��������� ������������ �������
	rcc_ClocksTypeDef.PCLK1_Frequency/=1000;
	tim3_com=(rcc_ClocksTypeDef.PCLK1_Frequency*2/TIM_PRESCALER)*PERIOD_GET_TC-1;
	tim3_InitTypeDef.TIM_CounterMode=TIM_CounterMode_Up;
	tim3_InitTypeDef.TIM_Period=tim3_com;
	tim3_InitTypeDef.TIM_Prescaler=TIM_PRESCALER;
	tim3_InitTypeDef.TIM_ClockDivision=TIM_CKD_DIV1;
	tim3_InitTypeDef.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM3,&tim3_InitTypeDef);
	// ���������� �� ������������ �������� ��������
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn,14);
	// ������� ����������� �������
	TIM_Cmd(TIM3,ENABLE);
}


//-----------������� processing_TC_init_spi1---------------
//������� processing_TC_init_spi1 - ������������ SPI ��� ���������� �������� ��
void processing_TC_init_spi1(void){
	GPIO_InitTypeDef GPIO_InitTypeDef_TC;
	SPI_InitTypeDef SPI_InitTypeDef_TC;


	// --------��������� GPIO ��� ������ � SN66HVS882-------
	// SCK   -  PA5  Alternate function push-pull
	// MISO  -  PA6  Input floating
	// MOSI  -  PA7  Alternate function push-pull
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	// ��������� SPI SCK
	GPIO_InitTypeDef_TC.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitTypeDef_TC.GPIO_Pin=GPIO_Pin_5;
	GPIO_InitTypeDef_TC.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitTypeDef_TC);
	// ��������� SPI MISO
	GPIO_InitTypeDef_TC.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitTypeDef_TC.GPIO_Pin=GPIO_Pin_6;
	GPIO_InitTypeDef_TC.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitTypeDef_TC);
	// ��������� SPI MOSI
	GPIO_InitTypeDef_TC.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitTypeDef_TC.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitTypeDef_TC.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitTypeDef_TC);

	// ------------��������� SPI ��� ������ � SN66HVS882
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);

	SPI_InitTypeDef_TC.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_256;
	SPI_InitTypeDef_TC.SPI_CPHA=SPI_CPHA_1Edge;
	SPI_InitTypeDef_TC.SPI_CPOL=SPI_CPOL_Low;
	SPI_InitTypeDef_TC.SPI_DataSize=SPI_DataSize_8b;
	SPI_InitTypeDef_TC.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	SPI_InitTypeDef_TC.SPI_FirstBit=SPI_FirstBit_MSB;
	SPI_InitTypeDef_TC.SPI_Mode=SPI_Mode_Master;
	SPI_InitTypeDef_TC.SPI_NSS=SPI_NSS_Soft;
	// ���������
	SPI_Init(SPI1,&SPI_InitTypeDef_TC);
	// ������� ���������� �� ������ �����
	SPI_I2S_ITConfig(SPI1,SPI_I2S_IT_RXNE,ENABLE);
	// ������� ���������� SPI
	NVIC_EnableIRQ(SPI1_IRQn);
	//����� ��������� ���������� SPI ��� ����������� ������ �� ���� ������ FreeRTOS API
	NVIC_SetPriority(SPI1_IRQn,13);
	// ������� SPI
	SPI_Cmd(SPI1,ENABLE);
}



//--------------���������� TIM3----------------
//���������� TIM3 ������������ ������������ �� ������� � PERIOD_GET_TC �� ��������.
// � ����� ���������� ������������� ������ ���������� ������ �� ������������ (�������������� ������ ��������� � ������������ ������ ������ ������)
void TIM3_IRQHandler(void){
	u8 k1;
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	// ��������� ������� ������� ��
	LD_CLEAR;
	while(k1<(30*4)){
		k1++;
	}
	LD_SET;
	while(k1<(30*4)){
		k1++;
	}
	// ������� ����������� ����, ������ ������� ��������� �� (� ����������)
	SPI_I2S_SendData(SPI1,CHECK_SERIAL_BYTE); // ������� ������ ������� ����� ������� ����������� ����
	s_reaz_read_TC.data_TC=0;
	//��������, ����� ����� �� ���������� � ������� ��������� (����� �����������) � ���������� ��� ��� ������� ��
	for(k1=0;k1<TOTAL_NUM_TC;k1++){
		// ���� ������ �� ��������� � ����� �����������
		if(s_TC_processing.as_one_TC_processing[k1].tc_is_procssing==IS_PROCESSING){
			s_TC_processing.as_one_TC_processing[k1].tc_counter++;
		}
	}
	/*
#ifdef TC_SOFT_CHECK
	if(GPIO_ReadOutputDataBit(PORT_SOFT_CHECK,PIN_SOFT_CHECK)){
		GPIO_ResetBits(PORT_SOFT_CHECK,PIN_SOFT_CHECK);
	}
	else{
		GPIO_SetBits(PORT_SOFT_CHECK,PIN_SOFT_CHECK);
	}
#endif
*/
}




//--------------���������� SPI1----------------
//���������� SPI1 ������������ �� ������ ����� �� ������������
void SPI1_IRQHandler(void){
	static u8 st_count_spi_int=0; // ������� ������ � ����������
/*  u8 k1=0;
	GPIO_SetBits(GPIOB,GPIO_Pin_8);
	for(k1=0;k1<20;k1++);
	GPIO_ResetBits(GPIOB,GPIO_Pin_8);*/
	/*
	 *   - OVR (OverRun Error) interrupt pending bit is cleared by software
	 *     sequence: a read operation to SPI_DR register (SPI_I2S_ReceiveData())
	 *     followed by a read operation to SPI_SR register (SPI_I2S_GetITStatus()).
	 */
	#ifdef TC_SOFT_CHECK
	if(GPIO_ReadOutputDataBit(PORT_SOFT_CHECK_B2,PIN_SOFT_CHECK_B2)){
		GPIO_ResetBits(PORT_SOFT_CHECK_B2,PIN_SOFT_CHECK_B2);
	}
	else{
		GPIO_SetBits(PORT_SOFT_CHECK_B2,PIN_SOFT_CHECK_B2);
	}
    #endif


	if(st_count_spi_int==0){                                                // ���������� �1 - ������ �������� ��������� ������ ��
		st_count_spi_int++;
		s_reaz_read_TC.data_TC=SPI_I2S_ReceiveData(SPI1);
		SPI_I2S_SendData(SPI1,0x00); // ������� ������ ������� ����� ������� ����������� ����
		return;
	}
	st_count_spi_int=0;                                                    // ���������� �2 - ������ ����������� ���� ��������� ����� ������������
	s_reaz_read_TC.data_cheack=SPI_I2S_ReceiveData(SPI1);
	xSemaphoreGiveFromISR(SemaphoreHandle_spi,&prioryty_take_semaphore);
	taskYIELD(); // ������ �� ����������, ���������� ������, � ����� ������� � ������ ������� ��

}


// ---------�������  TC_calc_address_oper_reg ---------
// ������� TC_calc_address_oper_reg - ��������� ������� ������� ��������� (������� ����������� ���������) ���������� ������������� ������
// ������� ���������:
// ps_TC_address           - ��������� �� ��������� ������� ��������� ���������� (���� � ���������� ��������� ������� ����������);
// adres_start             - ��������� ����� ����������� ��������� ���������� � ����� ������. ������������ ���� �������������� ����� � ��� ��������� ��������� ����������
// �������� ���������:
//                         - ����� ���������� �������� ����������
u16 TC_calc_address_oper_reg(S_TC_address *ps_TC_address, u16 adres_start){
	ps_TC_address->status_TC=adres_start;
	ps_TC_address->rez_TC=ps_TC_address->status_TC+NUM_REG_STATUS_TC;
	adres_start=ps_TC_address->rez_TC+TOTAL_NUM_TC;
	return adres_start;
};


//------------������ processing_TC_signal--------------------
// ������ processing_TC_signal ��������� ��������� �������� ��
void t_processing_TC(void *pvParameters){
	u8 firest_flag=0;
	u8 counter_flesh_influence=0; // ������� ������ ������ ����� �������� �� flesh �������
	u16 tc_status_temp, tc_status_;
	// �������� ���������������� ������ � ��������� �� ��������� ������������,
	// � ����� ������� ��������
	if(processing_TC_fill_S_TS((u8*)pvParameters)){
		SET_GLOBAL_STATUS(DEV_2);
		vTaskDelete(NULL); // ���� �������� �� �������� - ������� ������ + ��������� ������������ + ������ � ����� ������ !!!!
	};

	// ��������� ������������ ����������
	processing_TC_init();
	LD_SET;
	// ��� ������ ����� ��������� ���������
	// - �������� ������, ������������ ������ ���������� ��
	// - ����� ���������� ��������� ������ � �������� ���_�AP
	// - �������� ������ �� 500 ��.
	// � ����� ����� ������� ���������� ������ �� ������� ???
	while(1){
		xSemaphoreTake(SemaphoreHandle_spi,portMAX_DELAY); // ���������� ������� � ������ ����������� ���������� ����� ���������� ������ �� �� ������������
		// �������� ����������������� ������ �� �� ��������� ������������:
		// 1 - ������������ ��������� ������������ ���� ����������
		// 2 - ������ �� ����� ��� ������������ (���������� ����������� ����������)
		// 3 - ������� 24 � �� ���������� ������
		// ��������� ������������� � ������ ��������

		if(firest_flag==0){ // ���� ������ ��� ( ����� ������ ������� !!!!) ������, ����� ���� ������, ��������
			firest_flag=1;
			continue;
		}
		//
		tc_status_temp=0;
		// ��������� ������ TOK
		if(GPIO_ReadInputDataBit(TOK_PORT,TOK_PIN)==TOK_ERROR_STATE){
			// bit 0 stattu
			tc_status_temp|=(1<<TOK_ERROR);
		}
		// ��������� ������ ����������
		if(GPIO_ReadInputDataBit(V_PORT,V_PIN)==V_ERROR_STATE){
			// bit 1 stattu
			tc_status_temp|=(1<<V_ERROR);
		}
		// ��������, �������� �� ����������� �������� ���� � ���������
		if(s_reaz_read_TC.data_cheack!=CHECK_SERIAL_BYTE){

			tc_status_temp|=(1<<CHECK_ERROR);
		}
		s_reaz_read_TC.data_cheack=0;
		// �������� ������-��������
		if(tc_status_!=tc_status_temp){
			tc_status_=tc_status_temp;
			// �������� ������ �������
			processing_mem_map_write_s_proces_object_modbus(&tc_status_,1,s_address_oper_data.s_TC_address.status_TC);
			// ���� � ������ ������� �������� ���� ���� ��������, ������ �� �� ���������
			if(tc_status_){
				// ��������� ������ ������ �� � ���������� ������-��������
				SET_GLOBAL_STATUS(DEV_2);
				continue;
			}
			else{
				// �������� ������ ������ �� � ���������� ������-��������
				RESET_GLOBAL_STATUS(DEV_2);
			}
		}
		// ���������� ����������� �� �������� � flesh ������� ����� �������� ������������, ���� ��
		// ���������� MAX_CYCLE_FLASH_INFLUANCE ����������� ������, ��� ��� ��� ����� ����
		// ����������� � ���������� ������� �������, ��������� ����������� ���������� �� flesh �������
		if(FLASH_OPERATION_flash_state()!=DATA_NON){
		    counter_flesh_influence=MAX_CYCLE_FLASH_INFLUANCE;
			continue;
		}
		// ���� ���� ��������� ����� �� �������� �� flesh �������, ��������� MAX_CYCLE_FLASH_INFLUANCE ������ ���������� ������������
		if((counter_flesh_influence>=0)&&(counter_flesh_influence<=MAX_CYCLE_FLASH_INFLUANCE)){
			counter_flesh_influence--;
			continue;
		}
		// ���������� ����� ����������������
		processing_TC_signal_mem_update();
	}
}

//-------------------------������� processing_TC_signal_mem_update------------------------------
// ������� processing_TC_signal_mem_update - ��������� ��������� �������� �� �������� �� ������������ � ���������� ������ �� � ����� ������
void processing_TC_signal_mem_update(void){
	u8 k1;
	for(k1=0;k1<TOTAL_NUM_TC;k1++){
		//	READ_BIT_TC(s_reaz_read_TC.data_TC,k1);
		// ���� ������ �� �� ��������� � ����� �����������
		if(s_TC_processing.as_one_TC_processing[k1].tc_is_procssing==IS_NOT_PROCESSING){
			// ��������� �� �� ����������
			if(s_TC_processing.as_one_TC_processing[k1].tc_state==READ_BIT_TC(s_reaz_read_TC.data_TC,k1))
			{
				continue;
			}
			//��������� ����������
			s_TC_processing.as_one_TC_processing[k1].tc_state=READ_BIT_TC(s_reaz_read_TC.data_TC,k1);
			s_TC_processing.as_one_TC_processing[k1].tc_is_procssing=IS_PROCESSING;
			// �������� ����� ��������� � ����� ������
			processing_mem_map_write_s_proces_object_modbus(&s_TC_processing.as_one_TC_processing[k1].tc_state,
					1,
					s_address_oper_data.s_TC_address.rez_TC+k1);
			continue;
		}
		// ���� ������ �� ��������� � ����� �����������
		//s_TC_processing.as_one_TC_processing[k1].tc_counter++;
		if(s_TC_processing.as_one_TC_processing[k1].tc_counter>=ps_TC_user_config->time_gist){
			s_TC_processing.as_one_TC_processing[k1].tc_counter=0;
			s_TC_processing.as_one_TC_processing[k1].tc_is_procssing=IS_NOT_PROCESSING;
			continue;
		}
	}
}


//------------������� processing_TC_signal_check-----------------------------------
// ������� processing_TC_signal_check - ��������� �������� ����������� ���������� ������ �� � MODBUS ����� ������� ��
// ������� ���������:
//  data_chek - � �-� �� �������������
// �������� ���������:
// 0 - ������ �� ������������� ���������, ������ �� ��������
// 1 - ������ ������ (������� ������ �� �����������) ������ �� ����������
REZ_REQ_CHEACK_SLAVE processing_TC_signal_check(void* data_chek){
	u16 status_TC;
	processing_mem_map_read_s_proces_object_modbus(&status_TC,1,s_address_oper_data.s_TC_address.status_TC);
	if(status_TC){ return SLAVE_DEVICE_FALIURE;} // ���������� ���� "������������� ������������"
	return 0;
}




