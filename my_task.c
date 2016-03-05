#include "my_task.h"
//---------функция init_TS-----------
//
extern u8 mem_map[500];


void init_TS(void){
	GPIO_InitTypeDef s_gpio_TS, s_gpio_LED_Init_portC;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

	//------Init port GPIOB pin 1 input
	GPIO_StructInit(&s_gpio_TS);
	s_gpio_TS.GPIO_Mode=GPIO_Mode_IPD;
	s_gpio_TS.GPIO_Pin=PIN_TS;
	s_gpio_TS.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(PORT_TS,&s_gpio_TS);

	//------Init port GPIOC pin 9 output
	GPIO_StructInit(&s_gpio_LED_Init_portC);
	s_gpio_LED_Init_portC.GPIO_Mode=GPIO_Mode_Out_PP;
	s_gpio_LED_Init_portC.GPIO_Pin=GPIO_Pin_9;
	s_gpio_LED_Init_portC.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOC,&s_gpio_LED_Init_portC);
	GPIO_WriteBit(GPIOC,GPIO_Pin_9, Bit_SET);
}

u8 rez_read_TS(){
	return GPIO_ReadInputDataBit(PORT_TS,PIN_TS);
}


void tskTS(void *pvParameters){


	while(1){
		if(rez_read_TS()){
			vTaskSuspendAll();
			//mem_map[TS_ADRES*2+1]=0xFF;
			GPIO_WriteBit(GPIOC,GPIO_Pin_9, Bit_SET);
			xTaskResumeAll();

			vTaskDelay(10);
		}
		else{
			GPIO_WriteBit(GPIOC,GPIO_Pin_9, Bit_RESET);
			//mem_map[TS_ADRES*2+1]=15;
		}
	}
}

void tsk_BLINK(void *pvParameters){
	GPIO_InitTypeDef gpio_InitTypeDef_blink1;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

	GPIO_StructInit(&gpio_InitTypeDef_blink1);

	gpio_InitTypeDef_blink1.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio_InitTypeDef_blink1.GPIO_Pin=GPIO_Pin_8;
	gpio_InitTypeDef_blink1.GPIO_Speed=GPIO_Speed_2MHz;

	GPIO_Init(GPIOC,&gpio_InitTypeDef_blink1);


	while(1){
		GPIOC->ODR^=GPIO_ODR_ODR9;
		vTaskDelay(500);
	}
}
