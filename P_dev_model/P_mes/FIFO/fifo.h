/*
 * fifo.h- "first in first out" steak operation
 *
 *  Created on: June 12, 2016
 *      Author: Gerasimchuk
 */
#ifndef FIFO_H_
#define FIFO_H_

#include "stdint.h"

#define INDEX_INCREMENT(X,Y)    ((X+1) < Y ? (X+1):(0))

typedef enum{
	FIFO_OK=0,
	FIFO_NO_DATA=1,
	FIFO_OVERWRITE=2
}FIFO_ERROR;

volatile typedef struct{
	uint16_t steak_size_data; // размер буффера стека заданная пользователесм
	uint8_t *p_steak;        // указатель на начало стека
	uint8_t item_size;       // размер одного члена стека
	uint16_t steak_size;     // размер буфера стека в байтах
	uint16_t pos_read;       // поточная позиция на считывание
	uint16_t pos_write;      // поточная позиция на запись
	FIFO_ERROR status;       // статус стека
}S_fifo_steak;




FIFO_ERROR fifo_init(S_fifo_steak *ps_steak,void *p_steak_begin,uint8_t item_size,uint16_t full_steak_size);
uint8_t fifo_write(S_fifo_steak *ps_steak, uint16_t length_data, void *p_data);
uint8_t fifo_read(S_fifo_steak *ps_steak, uint16_t length_data, void *p_data);
uint8_t fifo_read_available(S_fifo_steak *ps_steak);
FIFO_ERROR fifo_get_status(S_fifo_steak *ps_steak);


#endif
