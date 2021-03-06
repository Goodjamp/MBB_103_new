/*
 * processing_config_dev.c
 *
 *  Created on: April 25, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */
#include "processing_config_dev.h"

//��������� ������� ����������� ���������
extern S_address_oper_data s_address_oper_data;
// ���������� ��������� �������� ���������� (������������ �� ����� ������, �� ����������� � ������� ������ ����������
// �� ������������, ���� ���� ������ ����� ������������)), �� main
extern S_config_moduls s_config_moduls;
//��������� �������� ����� ������ (��  mem_map_processing.c)
extern S_mem_map s_mem_map;

//-------------processing_config_firest_on------------------
// ������� processing_config_firest_on - ���� ������������ �� ��������� �� �������� - ��������
//                                       ���� ������������ ������������ �� �������� - �������� ������������ �� ���������
// ��� ������ ��������� ������������ ������ ������������ ������������ � �� ���������
void processing_config_firest_on(void){
	S_global_config s_mem_data_set;
	u16 CRC_dev;
	// ----------------------������������ "�� ���������"-------------------------
	INIT_MBB_read_addjust_table(s_mem_map.p_start_config_data,sizeof(S_dev_staff), PAGE_DEFAULT_CONFIG);
    // ���������� ����������� ����� ��������� ������ � ��������� � ��������� ����������� ������
	CRC_dev=CRC16((u8*)&(((S_global_config*)s_mem_map.p_start_config_data)->s_dev_staff),(sizeof(S_dev_staff)-2));
	if(CRC_dev !=((S_global_config*)s_mem_map.p_start_config_data)->s_dev_staff.CRC_dev)
	{
		default_config_table(&s_mem_data_set);  // ��������� ������������ �� ���������
		// ������� ���������������� ������� ����
		FLASH_OPERATION_erase_page(PAGE_DEFAULT_CONFIG);
		FLASH_OPERATION_write_flash_16b((u16*) &s_mem_data_set,(sizeof(s_mem_data_set)+1) / 2 , PAGE(PAGE_DEFAULT_CONFIG ));
	}
	// ----------------------������������ "������������"-------------------------
	INIT_MBB_read_addjust_table(s_mem_map.p_start_config_data,sizeof(S_dev_staff), PAGE_USER_CONFIG);
	// ���������� ����������� ����� ��������� ������ � ��������� � ��������� ����������� ������
	CRC_dev=CRC16((u8*)&(((S_global_config*)s_mem_map.p_start_config_data)->s_dev_staff),(sizeof(S_dev_staff)-2));
	if(CRC_dev != ((S_global_config*)s_mem_map.p_start_config_data)->s_dev_staff.CRC_dev)
	{
		default_config_table(&s_mem_data_set);  // ��������� ������������ �� ���������
		// ������� ���������������� ������� ����
		FLASH_OPERATION_erase_page(PAGE_USER_CONFIG);
		FLASH_OPERATION_write_flash_16b((u16*) &s_mem_data_set,(sizeof(s_mem_data_set)+1) / 2 , PAGE(PAGE_USER_CONFIG ));

	}
}


//-------------processing_config_init_not_user_comfig------------------
//������� processing_config_init_not_user_comfig - ��������� ���� "�� ����������������" ��������, � ������:
//                                               - ������ ����������
//                                               - �-�� ������ MODBUS
//                                               - ��������� ������������
//                                               - ������������ �-�� �������� MODBUS MASTER ��� ������ �����
//                                               - ������ ���������� ��
//                                               - CRC - ������������� ����������
void processing_config_init_not_user_config(S_dev_staff *ps_dev_staff){
	// �������� ���� ������������ ����������
	ps_dev_staff->bf_dev_staff=(BF_dev_staff){1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
	// ������ ����������
#ifdef DEV_0
	ps_dev_staff->bf_dev_staff.MODULE_0=ENABLE;
#endif
#ifdef DEV_1
	ps_dev_staff->bf_dev_staff.MODULE_1=ENABLE;
#endif
#ifdef DEV_2
	ps_dev_staff->bf_dev_staff.MODULE_2=ENABLE;
#endif
#ifdef DEV_3
	ps_dev_staff->bf_dev_staff.MODULE_3=ENABLE;
#endif
#ifdef DEV_4
	ps_dev_staff->bf_dev_staff.MODULE_4=ENABLE;
#endif
#ifdef DEV_5
	ps_dev_staff->bf_dev_staff.MODULE_5=ENABLE;
#endif
#ifdef DEV_6
	ps_dev_staff->bf_dev_staff.MODULE_6=ENABLE;
#endif
#ifdef DEV_7
	ps_dev_staff->bf_dev_staff.MODULE_7=ENABLE;
#endif
#ifdef DEV_8
	ps_dev_staff->bf_dev_staff.MODULE_8=ENABLE;
#endif
#ifdef DEV_9
	ps_dev_staff->bf_dev_staff.MODULE_9=ENABLE;
#endif
#ifdef DEV_10
	ps_dev_staff->bf_dev_staff.MODULE_10=ENABLE;
#endif
#ifdef DEV_11
	ps_dev_staff->bf_dev_staff.MODULE_11=ENABLE;
#endif
#ifdef DEV_12
	ps_dev_staff->bf_dev_staff.MODULE_12=ENABLE;
#endif
#ifdef DEV_13
	ps_dev_staff->bf_dev_staff.MODULE_13=ENABLE;
#endif
#ifdef DEV_14
	ps_dev_staff->bf_dev_staff.MODULE_14=ENABLE;
#endif
#ifdef DEV_15
	ps_dev_staff->bf_dev_staff.MODULE_15=ENABLE;
#endif

     // �������� ���� ������� � �������� ����������. �� ���� ��� �������� ������ ��������� �� ���� - ������, � ����������� ������ � �������������,
	 // �� ��� ������� ������� ��������� ������ �� ���� ��������, ������� �� ��������� �� ���������� ���� ����� ������� (������ � �. �.) ��� �� ��������

	ps_dev_staff->num_use_uart=NUM_PORT_MODBUS;//
	ps_dev_staff->num_reg_data=NUM_REG_DATA;
	ps_dev_staff->modbus_req_user=MAX_NUM_MODBUS_REQ;
	ps_dev_staff->program_version=1;
	ps_dev_staff->CRC_dev=CRC16((u8*)ps_dev_staff,(sizeof(S_dev_staff)-2));
}



//-------------processing_config_dev_init------------------
//������� processing_config_dev_init - ��������� ���������� �������� ���� ������� ���������� �������� ���������� ������������
//                                     �������� ������ ������������ � ������� ��������� ��� ������� ���������� �������,
//                                     ������ callback ������ ������� 16 modbus
void processing_config_dev_init(void){
	// ������� ��������� ������� ���������� �� ����� ������, � ��������� ��������, ������� ����� �������������� ����������� ��������.
	// ����� ��������� �� ����������� � ������� ������ ���������� (�� �����������).
	// ��������� ������������ (�����) � ������� ������������ ������������ � ����� ������ !!!
	// (u8*)&s_config_moduls - ���������� ��������� �������� ������� ����������, ������� ������������� ����������� �������� � ������� ������ ��������
	//(S_global_config*)s_mem_map.p_start_config_data ->s_config_moduls - ��������� �������� ���������� � ����� ������, �� ������������� ���������
	//                                                                    � ������� ������, �� ��� ��������� � ����� ������ �����������/�������������
	//                                                                    ������ ������ � ��� �������
	memcpy((u8*)&s_config_moduls,
			(u8*)&((S_global_config*)s_mem_map.p_start_config_data)->s_config_moduls,
			sizeof(S_config_moduls)
			);

	// ����� �-� callback ������� ���������� ��������� modbus_slave ��� �������� �������������� ������� �������� �3
	modbus_callback_address_check(&processing_config_check_is_holding_reg,READ_HOLDING_STATUS);
	// ����� �-� callback ������� ���������� ��������� modbus_slave ��� �������� �������������� ������� �������� �4
	modbus_callback_address_check(&processing_config_check_is_input_reg,READ_INPUT_REGISTERS);
	// ����� �-� callback ������� ���������� ��������� modbus_slave ��� �������� �������������� ������� �������� �16
	modbus_callback_address_check(&processing_config_check_is_preset_multiple_reg,PRESET_MULTIPLE_REGISTERS);
	// ����� �-� callback ������� ���������� ��������� modbus_slave ����� ������ ���������  �16
	modbus_callback_add_processing(&update_config_data,PRESET_MULTIPLE_REGISTERS);
	// ������� ������� ���� APB2 ��� ��������� ������ SPI1 ������ ��
	RCC_ClocksTypeDef rcc_clock;
	RCC_GetClocksFreq(&rcc_clock);
	RCC_PCLK2Config(RCC_HCLK_Div2);
	RCC_GetClocksFreq(&rcc_clock);
}


//------------������� processing_config_is_holding_reg-----------------------------------
// ������� processing_config_is_holding_reg - ��������� �������� ������� ����� ������ �� �������������� ��������� ������������ ����������
// ����� ������� ������������� �������� ������� �3 modbus
// ������� ���������:
// *p_check_address - ��������� �� ������ ������������������ (u16)address, (u16)num_reg
// �������� ���������:
// REQ_SLAVE_OK - �������� ��������� �������
// ILLEGAL_DATA_ADRESS - ������������ �����
REZ_REQ_CHEACK_SLAVE processing_config_check_is_holding_reg(void* p_check_address){
	u16 address= *(u16*)p_check_address;
	u16 num_reg= *(u16*)((u16*)p_check_address+1);
	// ����� ����������� ������� ����������� ���������
	if((address>=USER_ADDRESS_OPER_DATA)&&((address+num_reg-1)<(USER_ADDRESS_OPER_DATA+NUM_REG_OPER_DATA)))
	{
		return REQ_SLAVE_OK;
	}
	// ����� ����������� ������� ��������
	if((address>=USER_ADDRESS_CONFIG_DATA)&&((address+num_reg-1)<(USER_ADDRESS_CONFIG_DATA+NUM_REG_CONFIG)))
	{
		return REQ_SLAVE_OK;
	}
	// ����� ���������� �� ��������� ����c���� ���������
	return ILLEGAL_DATA_ADRESS;
}




//------------������� processing_config_is_input_reg-----------------------------------
// ������� processing_config_is_input_reg - ��������� �������� ������� ����� ������ �� �������������� ������ �������� ����������, �� ����������� ���. ������������
// ����� ������� ������������� �������� ������� �4 modbus
// ������� ���������:
// *p_check_address - ��������� �� ������ ������������������ (u16)address, (u16)num_reg
// �������� ���������:
// REQ_SLAVE_OK - ��������� ��������� �������
// ILLEGAL_DATA_ADRESS - ������������ �����
REZ_REQ_CHEACK_SLAVE processing_config_check_is_input_reg(void* p_check_address){
	u16 address= *(u16*)p_check_address;
	u16 num_reg= *(u16*)((u16*)p_check_address+1);
	// ����� ����������� ������� ��������� ������������
	if((address>=USER_ADDRESS_DATA)&&
			((address+num_reg-1)<(USER_ADDRESS_DATA+NUM_REG_DATA)))
	{
		return REQ_SLAVE_OK;
	}
	// ����� ���������� �� ��������� ���������� �������
	return ILLEGAL_DATA_ADRESS;
}


//------------������� processing_config_check_is_preset_multiple_register-----------------------------------
// ������� processing_mem_map_is_input_reg - ��������� �������� ������� ����� ������ �� �������������� ���������
// ����� ������� ������������� �������� ������� �16 modbus
// ������� ���������:
// *check_data - ��������� �� ������ ������������������ (u16)address
// �������� ���������:
// REQ_SLAVE_OK - ��������� ��������� �������
// ILLEGAL_DATA_ADRESS - ������������ �����

REZ_REQ_CHEACK_SLAVE processing_config_check_is_preset_multiple_reg(void* p_check_address){
	u16 address_reg=(*((u16*)p_check_address)); // ����� ������ ������������� ���������
	u16 num_reg=(*((u16*)p_check_address++)); // ���������� ������������� ���������

	return REQ_SLAVE_OK;
}


//-------------update_config_data------------------
//������� update_config_data - ��������� ���������� �������������.
// ������� ���������:
// req               - ��������� �� �������������� ����� ��������� � ���������� ����������� ����
// num_peyload_data  -
// ����� ������ ���� ���������������� ���������, ������������ �������� ������������
// � ������ �� FLESH.
// ������ ������ ������������:
// 1 - ��� ������ ��������� � ������� ������ ������ ������ ������� ������ ������������, ������������ ���� ������ ������ ������������
// 2 - ����� ������� ���������� ������ ���������� ��������� ������ ������
// 3 - ���� ����� � ��������� ������ �� ��������� �� ��������� ������� (�.2) �������� � �.1
// 4 - ����� ������ ���� ���������������� ����������, �������� �������� � ��������� �-��� �� ����
u8 update_config_data(void* req,u8 num_peyload_data, u16 addres_data){
	static u16 count_config_data=0;
	static u8 f_rx_config_data=0; // ���� ������ ������ �-���
	// ���� ������� ��������� �������
	if((f_rx_config_data==0)&(addres_data!=USER_CONFIG_START_ADRESS))
	{
		return 1;
	}
	//���� ������ ������ ����� ������� ������������
	if(f_rx_config_data==0)
	{
		f_rx_config_data=1;
		count_config_data=num_peyload_data;
		return 0;
	}
	// ���� ��������� ����� ������ ������������ �� ������������� ����������� ������
	if((addres_data-USER_CONFIG_START_ADRESS)!=count_config_data)
	{
		f_rx_config_data=0;
		count_config_data=0;
		return 1;
	}
	// ���������� ������� ������ ��������� �������
	count_config_data+=num_peyload_data;
	// ���� ������� ��� ���������������� ������� - ��������� �������� � ������ �� FLESH
	if(count_config_data==(USER_CONFIG_DATA_SIZE))
	{
		vTaskSuspendAll(); // ���������� �����������
		count_config_data=0;
		f_rx_config_data=0; //
		FLASH_OPERATION_erase_page(PAGE_USER_CONFIG);
		FLASH_OPERATION_write_flash_16b((u16*)s_mem_map.p_start_config_data,sizeof(S_global_config) / 2 + 1, START_ADDRESS_DATA);
		xTaskResumeAll();
		// ---------����� ��������� �������� ��������� �������----------

		// ����� ���: ����� ������� ������������ �� "main.c" ������������ � ���� ������� ���������

	}

	if(count_config_data>NUM_REG_CONFIG)
	{
		return 1;
	}
		return 0;
}
