/*
 * processing_simple_gpio.c
 *
 *  Created on: Jun 8, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "processing_simple_gpio.h"


//------- функция processing_simple_gpio -------------
// функция processing_simple_gpio - выполняет настройку портов индиувции, джамперов (всех портов логического управления)
void processing_simple_gpio(void){
	remap_out_pin();
	work_jamper_init();
	work_ind_init();
}

//------- функция work_jamper_init -------------
// функция work_jamper_init - выполняет настройку служебных портов и пинов джамперов
void work_jamper_init(void){
	GPIO_InitTypeDef gpio_InitTypeDef;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	// настройка джампера 1
	gpio_InitTypeDef.GPIO_Mode=GPIO_Mode_IPU;
	gpio_InitTypeDef.GPIO_Pin=PIN_JAMP1;
	gpio_InitTypeDef.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(PORT_JAMP1,&gpio_InitTypeDef);
	// настройка джампера 2
	gpio_InitTypeDef.GPIO_Mode=GPIO_Mode_IPU;
	gpio_InitTypeDef.GPIO_Pin=PIN_JAMP2;
	gpio_InitTypeDef.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(PORT_JAMP2,&gpio_InitTypeDef);
}

//------- функция remap_out_pin -------------
// функция remap_out_pin - настраиваю rermap ножек JTAG
void remap_out_pin(void){
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Необходтио настроить ножки PB3 и PB4
	// выключить JTAG, включить SWD
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);
	// выполнить remap TIM3 (PB4), TIM2 (PB3)
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2,ENABLE);
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}

//------- функция work_ind_init -------------
// функция work_ind_init - выполняет настройку портов и пинов служебных светодиодов режма работы
void work_ind_init(void){
	GPIO_InitTypeDef gpio_InitTypeDef;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	gpio_InitTypeDef.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio_InitTypeDef.GPIO_Pin=PIN_IND_ERROR;
	gpio_InitTypeDef.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(PORT_IND_ERROR,&gpio_InitTypeDef);
	IND_ERROR_RESET;
}

