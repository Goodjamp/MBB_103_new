/*
 * processing_routing_data.c
 *
 *  Created on: May 24, 2015
 *      Author: Gerasimchuk
 *      Version: 1
 */
#include "processing_routing_data.h"

//-------------------����������-----------------------
// �������� �� ������ ����� ������, ����������� � proces_object (�� main.c)
extern S_proces_object_modbus *p_proces_object;
// ������ ������� � ����� ������, ����������
extern xSemaphoreHandle h_mutex_mem_map_axes;
// ��������� �������� �������������
S_routing_data_user_config *ps_rout_table;
extern S_address_oper_data s_address_oper_data;
//---------------------------------------------------------------------------



// ----������� INIT_MBB_fill_S_ROUT_TABLE--------
//������� processing_mem_map_fill_S_route_table - ��������� ���������� ������ ������������� ������
INIT_MBB_Rezult processing_mem_map_fill_S_route_table(u8 *p_read_rout_table) {
	u16 k1;
	TYPE_REG_MEM_MAP type_mem;
	u8 *p_address;
	ps_rout_table=(S_routing_data_user_config*)p_read_rout_table;

	for (k1 = 0; k1 < ps_rout_table->num_rout; k1++) {
		// ����������� ������ "������ ����������"
		type_mem=processing_mem_map_detect(ps_rout_table->s_data_rout[k1].address_from,&p_address );
		if(type_mem==ERROR_DATA){
			return MBB_INIT_ERROR;
		}
		// ����������� ������ "���� ����������"
		type_mem=processing_mem_map_detect(ps_rout_table->s_data_rout[k1].address_to,&p_address );
		if(type_mem!=PMZ_DATA){
			return MBB_INIT_ERROR;
		}
		// ���� ������� ����� - ���� ������
		if(ps_rout_table->s_data_rout[k1].address_from == ps_rout_table->s_data_rout[k1].address_to){
			return MBB_INIT_ERROR;
		}
	}
	return MBB_INIT_OK;
}


// ---------�������  routing_data_calc_address_oper_reg ---------
// ������� routing_data_calc_address_oper_reg - ��������� ������� ������� ��������� (������� ����������� ���������) ���������� ������������� ������
// ������� ���������:
// ps_routing_data_address - ��������� �� ��������� ������� ��������� ���������� (���� � ���������� ��������� ������� ����������);
// adres_start             - ��������� ����� ����������� ��������� ���������� � ����� ������. ������������ ���� �������������� ����� � ��� ��������� ��������� ����������
// �������� ���������:
//                         - ����� ���������� �������� ����������
u16 routing_data_calc_address_oper_reg(S_routing_data_address *ps_routing_data_address, u16 adres_start){
	ps_routing_data_address->status_routing_data=adres_start;
	return adres_start;
};


// ---------�������  retransmit_task ---------
// ������� retransmit_task - ������ ������������ ��������� ������������ ������ �� ����� ������
void t_processing_routing_data(void *pvParameters) {
	u8 k1;
	u8 *p_addres_from;
	TYPE_REG_MEM_MAP type_reg;
	// �������� ���������������� ������ � ��������� �� ��������� ������������,
	// � ����� ������� ��������
	if(processing_mem_map_fill_S_route_table((u8*)pvParameters))// ���� �������� �� �������� - ������� ������ + ��������� ������������ + ������ � ����� ������ !!!!
	{
		//processing_mem_map_write_s_proces_object_modbus(ERROR_OUT_MEM,1,s_address_oper_data.s_routing_data_address.status_routing_data);
		SET_GLOBAL_STATUS(DEV_1);
		vTaskDelete(NULL);
	};

	// ��������� ������������ ����������
	//������������ ���������� ���� ��
	while (1) {
		// ���� ������, ������� ����������� ���������� �����
		xSemaphoreTake(h_mutex_mem_map_axes, portMAX_DELAY);
		// ������������ ��� ����������� ������������
		for (k1 = 0; k1 < ps_rout_table->num_rout; k1++) {
			// ����� ��� ������ ������������ � ������ ���������������
			type_reg=processing_mem_map_detect(ps_rout_table->s_data_rout[k1].address_from, &p_addres_from);
			if(type_reg==ERROR_DATA)
			{
				continue;
			}
			else if(type_reg==CONFIG_DATA)
			{
				*(u16*)(ps_rout_table->s_data_rout[k1].address_to+p_proces_object)=*(u16*)p_addres_from;
			}
			else{
				PPROCOBJ_2DATA(ps_rout_table->s_data_rout[k1].address_to+p_proces_object) = PPROCOBJ_2DATA(p_addres_from);
			}
		}

		xSemaphoreGive(h_mutex_mem_map_axes);
		// ����� ������
		taskYIELD(); // ������������� ���������� ��������
	} // end while(1){};
}





