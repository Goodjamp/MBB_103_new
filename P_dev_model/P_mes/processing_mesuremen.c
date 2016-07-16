/*
 * processing_mesuremen.c
 *
 *  Created on:
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#include "processing_mesuremen_extern.h"
#include "processing_mesuremen.h"



u16 mesurement_calc_address_oper_reg(S_mesurement_address *ps_mesurement_address, u16 adres_start){
	ps_mesurement_address->status_mesurement=adres_start;
	ps_mesurement_address->rez_mrez_float=ps_mesurement_address->status_mesurement+NUM_REG_STATUS_MES;
	ps_mesurement_address->rez_norm=ps_mesurement_address->rez_mrez_float+NUM_REG_REZ_FLOAT;
	return (ps_mesurement_address->rez_norm+NUM_REG_REZ_NORM);
}
void t_processing_mesurement(void *pvParameters){

	u8 var[150];
	while(1){

	}

}
