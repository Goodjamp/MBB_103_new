/*
 *  processing_mesuremen_adc.h
 *
 *  Created on: December 21, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "processing_mesuremen_adc.h"



//----------u8 config_ADC(ADC_TypeDef *ADC_IN, S_config_ADC s_config_ADC)------------------
//������� config_ADC. ��������� ��������� ������������ ��� � ������ �����/����� �������� ������ ������� ����������.
//�� ��������� ��������������:
// - ���������������� ����������� ������� ��� ���� ���
// - ����� ������ � ������� DMA
// - ����������� ����� (��� ���������� ������� ������� 1), ����� ������������ (��� ���������� ������� >1)
// - ���������������� �������������� ��� ���������
// - ���������������� ������ ����� ���������� �������
// - �� ���������������� ����� DMA ��� ������ ������ (������������ ��������� � ��������� �-�)
// ***********************************������� ���������******************************
// - ADC_IN - ��������� �� ��� ������� ���������� ���������������. ��������� �������� ADC1...3
// - s_config_ADC - ��������� � ������������ �����������.
//                  ���� ��������� s_config_ADC:
//              - u16 ADCPRE (0-3     - ����������� ������� ������� ������������ ��� (������� ������������ ��� ����������� ADCCLK)
//                                      ��� ������������ �� ���� APB2 (������������ ������� F_APB2 = 84 ���). ���������  ADCPRE ����� �������� ������� ������������ ���
//                                      �, �������������, �������� ����������� ��������. �-� ������� ��������� ��� ���� ���
//                                      ADCPRE ��������� �� ������:
//                                     �-� ������� (ADCPRE)      ��������
//                                              2                @ref    RCC_PCLK2_Div2
//                                              4                @ref    RCC_PCLK2_Div4
//                                              6                @ref    RCC_PCLK2_Div6
//                                              8                @ref    RCC_PCLK2_Div8
//              - u8  quantity_chanel (1-16) - ���������� ���������� �������.
//              - u16 num_chanel[16] (0-16)  - ����� � �������� ���������� �������. ������������ ������ ������ � ���� ����������� ��������� ����
//              ____________________________________________________________________________________________________________________________________________
//-����� ������ |  0  |  1  |  2  |  3  |    4    |    5    |    6    |    7    |    8    |    9    | 10  |  11  |  12  |  13  |  14  |  15  |  16  |  17  |  18  |
//-���          | PA0 | PA1 | PA2 |PA3  | PA4(1,2)| PA5(1,2)| PA6(1,2)| PA7(1,2)| PB0(1,2)| PB1(1,2)| PC0 |  PC1 | PC2  | PC3  | PC4  | PC5  |tempr | Vref | Vbat |
//
//              - u16 time_mes[16] (0-7) - ���������� ������ ADCCLK ��� ��4����� ������ �� ������ num_chanel.
//                                      ���������� ������ ��������� �� ������:
//                                     �-��� (CH_SMPR)     ��������
//                                       1,5+12,5          @ref ADC_SampleTime_1Cycles5
//                                       7,5+12,5          @ref ADC_SampleTime_7Cycles5
//                                      13,5+12,5          @ref ADC_SampleTime_13Cycles5
//                                      28,5+12,5          @ref ADC_SampleTime_28Cycles5
//                                      41,5+12,5          @ref ADC_SampleTime_41Cycles5
//                                      55,5+12,5          @ref ADC_SampleTime_55Cycles5
//                                      71,5+12,5          @ref ADC_SampleTime_71Cycles5
//                                     239,5+12,5          @ref ADC_SampleTime_239Cycles5
//                                      ����� �������� ������ time_mes ������������� ���������� ������ ADCCLK
//                                      ������ ��� ����� �� ������� �������� � ������ num_chanel. �������: num_chanel[3]={3,7,2}
//                                                                                                         time_mes[3]  ={2,4,7}
//                                                                         ����� �������������� ��� ������ 3 (������ � ������) ����� 2 � �. �.
//                                      ���������� ����� �������������� ��� ������ ������� ��������������� ��������:
//                                        Time_mes=(CH_SMPR)/(F_APB2/ADCPRE)
//                                      ������������� ������� ��������������:
//                                        F_Time_mes=1/Time_mes
// ***********************************�������� ���������******************************
// 1 - ERROR_PSC                  - �������� ������������ ���
// 2 - ERROR_MAX_QUANTITY_CHANEL  - ������� ������ ������������ ���������� �������. �������� 1-16
// 3 - ERROR_POIN_ADC             - �������� ��������� �� ���� ���. �������� ADC1, ADC2, ADC3
// 4 - ERROR_NUMBER_CHANEL        - �������� ����� ������. �������� 0-16
// 5 - ERROR_SAMPLE_TIME          - ��������� ����� �������������� ������ �������� 0-7

ADC_CONF_ERROR processing_mesuremen_adc_conf_channel(ADC_TypeDef *ADC_IN, S_config_ADC s_config_ADC) {
	u8 counter;
	u8 time_mesurem;
	u8 number_chanel;
	GPIO_TypeDef *adc_gpio;
	GPIO_InitTypeDef gpio_InitTypeDef;
	//*********�������� ������� ����������************************
	// �������� ���������� �������. ���������� ��������: 1 - 16
	if ((s_config_ADC.quantity_chanel > MAX_QUANTITY_CHANEL)|| (s_config_ADC.quantity_chanel < MIN_QUANTITY_CHANEL))
	{
		return ERROR_MAX_QUANTITY_CHANEL;
	}
	// ������������ ����� ������������ ���
	if(!IS_RCC_ADCCLK(s_config_ADC.comon_adc_psc)){return ERROR_COMON_ADC_PSC;};
	RCC_ADCCLKConfig(s_config_ADC.comon_adc_psc);
	//������� ������������ �������������� �-�
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	//*********������������ ���************************
	// �������� ������������ ���

	if (ADC_IN == ADC1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	}
	else if (ADC_IN == ADC2)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,ENABLE);
	}
#ifdef STM32F10X_HD
	else if (ADC_IN == ADC3)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3,ENABLE);
	}
#endif
	else
	{
		return ERROR_POIN_ADC;
	};
	ADC_IN->CR2 |= (ADC_CR2_ADON);
	// �������� ��� ������������� ��������
	ADC_IN->CR1 = 0;
	ADC_IN->SMPR1 = 0;
	ADC_IN->SMPR2 = 0;
	ADC_IN->SQR1 = 0;
	ADC_IN->SQR2 = 0;
	ADC_IN->SQR3 = 0;
	//---------������������� ������� ���������� ��������������---------------------
	ADC_IN->SQR1 |= ((s_config_ADC.quantity_chanel - 1) << 20); // �� 1 ����� ���������� �������
	//--------------------------------------------------------------------------------
	// ----------������������� ������� ���������� �������-----����� �������������� ��� ������� ������-----------------------
	for (counter = 1; counter <= s_config_ADC.quantity_chanel; counter++) {
		number_chanel = s_config_ADC.num_chanel[counter - 1];
		time_mesurem = s_config_ADC.time_mes[counter - 1];
		if (number_chanel > MAX_QUANTITY_CHANEL) { // �������� ����������� ������ ������
			return ERROR_NUMBER_CHANEL;
		};
		if(!IS_ADC_SAMPLE_TIME(time_mesurem ))// �������� ����������� ����� ��������������
		{
			return ERROR_SAMPLE_TIME;
		};
		//���������� ����� �������
		if ((counter >= 1) && (counter <= 6))
		{
			ADC_IN->SQR3 |= (number_chanel << ((counter - 1) * SIZE_1_SQR));
		}
		else if ((counter >= 7) && (counter <= 12))
		{
			ADC_IN->SQR2 |= (number_chanel << ((counter - 7) * SIZE_1_SQR));
		}
		else if ((counter >= 13) && (counter <= 16))
		{
			ADC_IN->SQR1 |= (number_chanel << ((counter - 13) * SIZE_1_SQR));
		}
		//���������� ����� �������������� �������
		if ((number_chanel >= 0) && (number_chanel <= 9))
		{
			ADC_IN->SMPR2 |= (time_mesurem << (number_chanel * SIZE_1_SMPR));
		}
		else if ((number_chanel >= 10) && (number_chanel <= 18))
		{
			ADC_IN->SMPR1 |= (time_mesurem << ((number_chanel - 10) * SIZE_1_SMPR));
		};
	}; // end for(counter)
	//--------------------------------------------------------------------------------
	// ----------������������� GPIO---------------------------------------------------
	for (counter = 1; counter <= s_config_ADC.quantity_chanel; counter++) {
		number_chanel = s_config_ADC.num_chanel[counter - 1];
		switch (number_chanel) {
		case (0): // ������ ����� �
		case (1):
		case (2):
		case (3):
		case (4):
		case (5):
		case (6):
		case (7):
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		adc_gpio = GPIOA;
		if (number_chanel == 0)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_0;
		}
		else if (number_chanel == 1)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_1;
		}
		else if (number_chanel == 2)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_2;
		}
		else if (number_chanel == 3)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_3;
		}
		else if (number_chanel == 4)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_4;
		}
		else if (number_chanel == 5)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_5;
		}
		else if (number_chanel == 6)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_6;
		}
		else if (number_chanel == 7)
		{
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_7;
		};
		break;
		case (8): // ������ ����� �
		case (9):
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		adc_gpio = GPIOB;
		if (number_chanel == 8) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_1;
		} else if (number_chanel == 9) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_0;
		}
		;
		break;
		case (10): // ������ ����� �
		case (11):
		case (12):
		case (13):
		case (14):
		case (15):
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
		adc_gpio = GPIOC;
		if (number_chanel == 10) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_0;
		} else if (number_chanel == 11) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_1;
		} else if (number_chanel == 12) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_2;
		} else if (number_chanel == 13) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_3;
		} else if (number_chanel == 14) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_4;
		} else if (number_chanel == 15) {
			gpio_InitTypeDef.GPIO_Pin=GPIO_Pin_5;
		};
		break;
		default:
			break;
		};
		gpio_InitTypeDef.GPIO_Speed=GPIO_Speed_2MHz;
		gpio_InitTypeDef.GPIO_Mode=GPIO_Mode_IN_FLOATING;
		GPIO_Init(adc_gpio,&gpio_InitTypeDef);
	} // end for(counter)

	return REZ_OK;
}

//-----------processing_mesuremen_conf_mod_adc---------------
// FUNCTION processing_mesuremen_conf_mod_adc - make ADC mode configuration:
// - ADC continious mode
ADC_CONF_ERROR processing_mesuremen_adc_conf_mod(S_dma_run_param *ps_adc_buf_par){
	u32 count_wait_calib_reset=0;
	ADC1->CR1=0;
	ADC1->CR2=0;
	// calibration ADC
	ADC1->CR2|=(ADC_CR2_CAL);
	//wait reset calibration bit reset
	while(((ADC1->CR2)|(ADC_CR2_CAL))&&(count_wait_calib_reset<WAIT_RESET_CALIB)){
		count_wait_calib_reset++;
	}
	// if self calibration not successful
	if(count_wait_calib_reset>=WAIT_RESET_CALIB)
	{
		return ERROR_CALIB;
	}
	//config mode
	ADC1->CR2|=((ADC_CR2_CONT)|(ADC_CR2_DMA));
	// set DMA num data and mem address
	processing_mesuremen_dma_mem(ps_adc_buf_par);
	return REZ_OK;
}


//-----------processing_mesuremen_run_adc---------------
// function processing_mesuremen_run_adc - run ADC1
void processing_mesuremen_adc_run(void){
	processing_mesuremen_dma_run_adc(); // run ADC DMA

	ADC1->CR2|=ADC_CR2_ADON;            // run ADC
}




