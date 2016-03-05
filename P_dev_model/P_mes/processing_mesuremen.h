/*
 * processing_mesuremen.h
 *
 *  Created on:
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#ifndef PROCESSING_MESUREMENT_H_
#define PROCESSING_MESUREMENT_H_
#include "processing_mem_map.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include <mes_processing_sourse/processing_mesuremen_dma.h>
#include <mes_processing_sourse/processing_mesuremen_adc.h>





void t_processing_mesuremen(void);

#endif
