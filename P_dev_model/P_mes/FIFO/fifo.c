/*
 * fifo.c - "first in first out" steak operation
 *
 *  Created on: June 12, 2016
 *      Author: Gerasimchuk
 */
#include "fifo.h"


// Ïîçèöèÿ íà ñ÷èòûâàíèå - ìåñòî ñ êîòîğîãî ÍÀ×ÈÍÀÒÜ Ñ×ÈÒÛÂÀÒÜ
// Ïîçèöèÿ íà çàïèñü     - ìåñòî ñ êîòîğîãî ÍÀ×ÈÍÀÒÜ ÇÀÏÈÑÛÂÀÒÜ

FIFO_ERROR fifo_init(S_fifo_steak *ps_steak,void *p_steak_begin,uint8_t item_size,uint16_t full_steak_size)
{
	// init steak
	ps_steak->steak_size_data=full_steak_size;
	ps_steak->p_steak=(uint8_t*)p_steak_begin;
	ps_steak->item_size=item_size;
	ps_steak->steak_size=full_steak_size*item_size;
	ps_steak->pos_read=0;
	ps_steak->pos_write=0;
	ps_steak->status=FIFO_OK;
	return FIFO_OK;
}



uint8_t fifo_write(S_fifo_steak *ps_steak, uint16_t length_data, void *p_data){
	uint16_t num_copy=length_data*ps_steak->item_size;
	uint8_t *p_read=(uint8_t*)p_data;
	uint16_t counter=0;
	for(;counter<num_copy;counter++,ps_steak->pos_write=INDEX_INCREMENT(ps_steak->pos_write,ps_steak->steak_size)){
		if((ps_steak->pos_write==ps_steak->pos_read)&&(counter)){ps_steak->status=FIFO_OVERWRITE;}
		ps_steak->p_steak[ps_steak->pos_write]=p_read[counter];
	}
	if(ps_steak->pos_write==ps_steak->pos_read){ps_steak->status=FIFO_OVERWRITE;}
	return length_data;
}


uint8_t fifo_read(S_fifo_steak *ps_steak, uint16_t length_data, void *p_data){
	uint8_t *p_buff_in=(uint8_t*)p_data;
	uint8_t num_read=fifo_read_available(ps_steak)*ps_steak->item_size;
	uint16_t byte_to_read=length_data*ps_steak->item_size;
	uint8_t counter=0;
	if(ps_steak->status==FIFO_OVERWRITE){
		ps_steak->status=FIFO_OK;
		ps_steak->pos_read=ps_steak->pos_write;
	}
	if(num_read>byte_to_read){num_read=byte_to_read;}

	for(;counter<num_read;counter++,ps_steak->pos_read=INDEX_INCREMENT(ps_steak->pos_read,ps_steak->steak_size)){
		p_buff_in[counter]=ps_steak->p_steak[ps_steak->pos_read];
	}
	return num_read/ps_steak->item_size;
}


uint8_t fifo_read_available(S_fifo_steak *ps_steak){
	if(ps_steak->status==FIFO_OVERWRITE){
		return ps_steak->steak_size_data;
	}
	if(ps_steak->pos_write >= ps_steak->pos_read){
		return (ps_steak->pos_write - ps_steak->pos_read)/ps_steak->item_size;
	}
	else{
		return ps_steak->steak_size_data-(ps_steak->pos_read-ps_steak->pos_write)/ps_steak->item_size;
	}
}


FIFO_ERROR fifo_get_status(S_fifo_steak *ps_steak){
	return 	ps_steak->status;
}
