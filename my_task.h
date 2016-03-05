#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"


#define PORT_TS GPIOA
#define PIN_TS  GPIO_Pin_0

#define TS_ADRES  10

void init_TS(void);
u8 rez_read_TS();
void tskTS(void *);
void tsk_BLINK(void *pvParameters);
