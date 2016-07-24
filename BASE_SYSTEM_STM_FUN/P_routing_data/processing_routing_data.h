/*
 * processing_routing_data.h
 *
 *  Created on: May 24, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#ifndef PROCESSING_ROUTING_DATA_H_
#define PROCESSING_ROUTING_DATA_H_

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "init_system.h"
#include "processing_reset_control.h"
#include "processing_routing_data_extern.h"

#define SIZE_SYAYUS_REG_OPER_ROUTING 1

typedef enum{
	CONFIG_OK=0,
	ERROR_OUT_MEM=1<<0,
	ERROR_SAME_MEM=1<<1
}ROUTING_ERROR;

INIT_MBB_Rezult processing_mem_map_fill_S_route_table(u8 *p_read_rout_table);


#endif
