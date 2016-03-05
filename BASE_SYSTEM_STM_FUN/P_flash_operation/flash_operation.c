/*
 * flash_operation.c
 *
 *  Created on: December 25, 2014
 *      Author: Gerasimchuk
 *      Versin: 1
 */
 
#include "flash_operation.h"

DATA_READ_WRITE_STATE state_read_write=DATA_NON;


//------- функция FLASH_OPERATION_read_flash_8b -------------
// функция FLASH_OPERATION_read_flash_8b - считывание заданого к-ва 8-ми байтных чисел из flash-памяти
// входные аргументы:
//                 *p_buff - указатель на буффер считанных 8-ми битных значений
//                  length - к-во данных к считыванию
//                 address - начальный адрес считываимых данных
void FLASH_OPERATION_read_flash_8b(u8 *p_buff, u32 length, u32 address) {
	u32 k1;

	state_read_write=DATA_READ;

	for (k1 = 0; k1 < length; k1++) {
		(*p_buff) = (*(__IO u8*) address);
		address += 1;
		p_buff++;
	}
	//state_read_write=DATA_NON;
}


//------- функция FLASH_OPERATION_read_flash_16b -------------
// функция FLASH_OPERATION_read_flash_16b - считывание заданого к-ва 8-ми байтных чисел из flash-памяти
// входные аргументы:
//                 *p_buff - указатель на буффер считанных 16-ти битных значений
//                  length - к-во данных к считыванию
//                 address - начальный адрес считываимых данных
void FLASH_OPERATION_read_flash_16b(u16 *p_buff, u32 length, u32 address) {
	u32 k1;
	state_read_write=DATA_READ;

	for (k1 = 0; k1 < length; k1++) {
		(*p_buff) = (*(__IO u16*) address);
		address += 2;
		p_buff++;
	}
	//state_read_write=DATA_NON;
}

//------- функция FLASH_OPERATION_write_flash_16b -------------
// функция FLASH_OPERATION_write_flashb - запись в flash-память заданное к-во 16-ти битных значений,
//                                 запись выполняеться строго с начала страницы. Перед записью страница стираеться.
// входные аргументы:
//                 *data - указатель на записываимые данные
//                  length - к-во данных к записи
//                  num_page - номер страницы памяти, принмсает значения 0-127
FLASH_Status FLASH_OPERATION_write_flash_page_16b(u16 *data, u32 length, u16 num_page) {
	FLASH_Status rez_write;
	u32 address;
	u32 k1;
	state_read_write=DATA_WRITE;
	address = PAGE(num_page);
	FLASH_Unlock();
	FLASH_ErasePage(address);
	for (k1 = 0; k1 < length; k1++) {
		rez_write = FLASH_ProgramHalfWord(address, (*data));
		data++;
		address += 2;
	}
	FLASH_Lock();
	//state_read_write=DATA_NON;
	return rez_write;
}
 

//------- функция FLASH_OPERATION_write_flash_16b -------------
// функция FLASH_OPERATION_write_flashb - запись в flash-память заданное к-во 16-ти битных значений
// входные аргументы:
//                 *data - указатель на записываимые данные
//                  length - к-во данных к записи
//                 address - начальный адрес записи, принимает значения START_MEM_ADRESS - STOP_MEM_ADRESS
u32 k1_count;
FLASH_Status FLASH_OPERATION_write_flash_16b(u16 *data, u32 length, u32 address) {
	FLASH_Status rez_write;
	u32 k1;
	state_read_write=DATA_WRITE;
	FLASH_Unlock();
	for (k1_count = 0; k1_count < length; k1_count ++) {
		if(k1_count>=135){
			k1++;}
		rez_write = FLASH_ProgramHalfWord(address, (*data));
		data++;
		address += 2;
	}
	FLASH_Lock();
	//state_read_write=DATA_NON;
	return rez_write;
}


//------- функция FLASH_OPERATION_erase_page -------------
// функция FLASH_OPERATION_erase_page - стирание страницы
// входные аргументы:
//                 num_page  - номер страницы памяти, принмсает значения 0-127
void FLASH_OPERATION_erase_page(u16 num_page) {
	u32 address;
	state_read_write=DATA_ERASE;
	address = PAGE(num_page);
	FLASH_Unlock();
	FLASH_ErasePage(address);
	FLASH_Lock();
	//state_read_write=DATA_NON;
}

//------- функция flash_state -------------
// функция flash_state - возращает поточное состояние (запись, считывание, стирание, простой)
DATA_READ_WRITE_STATE FLASH_OPERATION_flash_state(void){
	DATA_READ_WRITE_STATE naw_state=state_read_write;
	state_read_write=DATA_NON;
	return naw_state;
}
