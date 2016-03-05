/*
 * flash_operation.c
 *
 *  Created on: December 25, 2014
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#include "stm32f10x_flash.h"
#include "stm32f10x.h"

#ifndef FLASH_OPERATION
#define FLASH_OPERATION

typedef enum flash_1{
	DATA_NON,
	DATA_READ,
	DATA_WRITE,
	DATA_ERASE
}DATA_READ_WRITE_STATE;


// Начальный адрес области flash памяти
#define START_MEM_ADRESS 0x08000000
// Конечный адрес области flash памяти
#define STOP_MEM_ADRESS 0x08000000+127*1024

// Макрос для опредиления начала адреса страницы по номеру страницы
#define PAGE(NUM_PAGE) NUM_PAGE*0x0400+START_MEM_ADRESS


// ----------------Прототипы функций файла работы с flash-памятью------------------
void FLASH_OPERATION_read_flash_8b(u8 *p_buff, u32 length, u32 address);
void FLASH_OPERATION_read_flash_16b(u16 *data, u32 length, u32 address);
FLASH_Status FLASH_OPERATION_write_flash_page_16b(u16 *data, u32 length, u16 num_page);
FLASH_Status FLASH_OPERATION_write_flash_16b(u16 *data, u32 length, u32 address);
void FLASH_OPERATION_erase_page(u16 num_page);
DATA_READ_WRITE_STATE FLASH_OPERATION_flash_state(void);

#endif
