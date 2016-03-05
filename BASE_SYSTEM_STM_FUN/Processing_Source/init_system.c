/*
 * init_MBB.c
 *
 *  Created on: December 25, 2014
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#include "init_system.h"

//------- функция INIT_MBB_read_addjust_table -------------
// функция INIT_MBB_read_addjust_table - считывание данных конфигурации
// входные аргументы:
//                 *p_read_array - указатель на буффер считанных 8-ми битных значений
//                  size_read_array - к-во данных к считыванию
void INIT_MBB_read_addjust_table(u8 *p_read_array, u16 size_read_array, u8 num_read_page){
	u32 addres_read;
	addres_read = PAGE(num_read_page);
	FLASH_OPERATION_read_flash_8b(p_read_array,size_read_array, addres_read);
}

