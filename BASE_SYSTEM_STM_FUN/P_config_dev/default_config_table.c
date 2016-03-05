/*
 * default_config_table.c
 *
 *  Created on:  4 January, 2016
 *      Author: Gerasimchuk
 *      Versin: 1
 */

//#include "check_init.h"

#include "processing_config_dev.h"

// -----------функция  config_check------------
// функция  config_check - для записи конфигурации по умолчанию.
// все поля конфигурации заполняються вручную
void default_config_table(S_global_config *ps_mem_data_set) {
	// очистка конфигурационной области флеш
	//FLASH_OPERATION_erase_page(num_page);


	// Поля настройки состава устройства
	ps_mem_data_set->s_dev_staff=(S_dev_staff){0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	                                0,
	                                0,
	                                0,
	                                0,
	                                0
	};
    // дата конфигурации по умолчанию
	ps_mem_data_set->bf_date_config=(BF_date_config){21,4,15,14,13,12};

	//-------------------------------------------------------------------------------
	//----------------настройки ПОРТОВ И ПРОТОКОЛОВ----------------------------------
	//-------------------------------------------------------------------------------


	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[0].state = ENABLE;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[0].s_port_config.baudrate= 9600;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[0].s_port_config.stopbits = 1;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[0].s_port_config.parity = 0;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[0].s_port_config.amountbyte = 8;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[0].s_port_config.controlpotok=0;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[0].type = PROTOCOL_MODBUS_SLAVE; // master
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[0].waitresp = 200;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[0].number_of_pribor = 2;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[0].number_no_answer = 200;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[0].adress_kp = 1;


	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[1].state = ENABLE;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[1].s_port_config.baudrate=9600;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[1].s_port_config.stopbits = 1;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[1].s_port_config.parity = 0;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[1].s_port_config.amountbyte = 8;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[1].s_port_config.controlpotok=0;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[1].type = PROTOCOL_MODBUS_SLAVE;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[1].waitresp = 200;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[1].number_of_pribor = 1;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[1].number_no_answer = 10;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[1].adress_kp = 1;


	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[2].state = ENABLE;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[2].s_port_config.baudrate= 9600;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[2].s_port_config.stopbits = 1;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[2].s_port_config.parity = 0;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[2].s_port_config.amountbyte = 8;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[2].s_port_config.controlpotok=0;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[2].type = PROTOCOL_MODBUS_MASTER; // master
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[2].waitresp = 200;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[2].number_of_pribor = 2;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[2].number_no_answer = 20;
	ps_mem_data_set->s_config_moduls.s_connectmodbus_global[2].adress_kp = 1;


	//-------------------------------------------------------------------------------
	// ----------------настройки МОДУЛЯ ТС-------------------------------------------
	//-------------------------------------------------------------------------------
	ps_mem_data_set->s_config_moduls.s_TC_user_config.state=ENABLE;
	ps_mem_data_set->s_config_moduls.s_TC_user_config.time_gist=1000;
	ps_mem_data_set->s_config_moduls.s_TC_user_config.a_r[0]=1;
	ps_mem_data_set->s_config_moduls.s_TC_user_config.a_r[1]=1;
	ps_mem_data_set->s_config_moduls.s_TC_user_config.a_r[2]=1;
	ps_mem_data_set->s_config_moduls.s_TC_user_config.a_r[3]=1;
	//ps_mem_data_set->s_config_moduls.s_TC_user_config.a_r[4]=1;
	//ps_mem_data_set->s_config_moduls.s_TC_user_config.a_r[5]=1;

	//-------------------------------------------------------------------------------
	// ----------------настройки МОДУЛЯ ТУ-------------------------------------------
	//-------------------------------------------------------------------------------
/*	ps_mem_data_set->s_config_moduls.s_TY_user_config.state=ENABLE;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.puls_with=5000;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.f_use_V_check=1;
	//ТУ №1
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[0].mode_TY=2;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[0].f_paralel_out=0;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[0].num_paralel=0;
	//ТУ №2
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[1].mode_TY=2;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[1].f_paralel_out=0;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[1].num_paralel=0;
	//ТУ №3
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[2].mode_TY=1;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[2].f_paralel_out=0;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[2].num_paralel=0;
	//ТУ №4
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[3].mode_TY=0;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[3].f_paralel_out=0;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[3].num_paralel=0;
*/

	ps_mem_data_set->s_config_moduls.s_TY_user_config.state=ENABLE;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.puls_with=4000;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.f_use_V_check=1;
	//ТУ №1 518
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[0].mode_TY=2;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[0].f_paralel_out=0;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[0].num_paralel=0;
	//ТУ №2 519
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[1].mode_TY=2;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[1].f_paralel_out=1;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[1].num_paralel=522;
	//ТУ №3 520
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[2].mode_TY=1;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[2].f_paralel_out=0;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[2].num_paralel=0;
	//ТУ №4 521
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[3].mode_TY=3;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[3].f_paralel_out=1;
	ps_mem_data_set->s_config_moduls.s_TY_user_config.s_TY_out_config[3].num_paralel=520;

	//-------------------------------------------------------------------------------
	// ----------------настройки РЕТРЯНСЛЯЦИИ----------------------------------------
	//-------------------------------------------------------------------------------
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.state=ENABLE;
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.byte_alignt=0;
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.num_rout = 0;

	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[0].address_from = 0; //1
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[0].address_to = 0;

	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[1].address_from = 0; //2
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[1].address_to = 0;

	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[2].address_from = 0; //3
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[2].address_to = 0;

	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[3].address_from = 0; //4
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[3].address_to = 0;

	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[4].address_from = 0; //5
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[4].address_to = 0;

	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[5].address_from = 0; //6
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[5].address_to = 0;

	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[6].address_from = 0; //7
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[6].address_to = 0;

	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[7].address_from = 0; //8
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[7].address_to = 0;

	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[8].address_from = 0; //9
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[8].address_to = 0;

	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[9].address_from = 0; //10
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[9].address_to = 0;

	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[10].address_from = 0; //11
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[10].address_to = 0;

	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[11].address_from = 0;  //12
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[11].address_to = 0;

	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[12].address_from = 0;  //13
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[12].address_to = 0;

    ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[13].address_from = 0;  //14
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[13].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[14].address_from = 0;  //15
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[14].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[15].address_from = 0;  //16
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[15].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[16].address_from = 0;  //17
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[16].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[17].address_from = 0;  //18
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[17].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[18].address_from = 0; //19
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[18].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[19].address_from = 0;  //20
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[19].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[20].address_from = 0;  //21
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[20].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[21].address_from = 0;  //22
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[21].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[22].address_from = 0;  //23
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[22].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[23].address_from = 0;  //24
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[23].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[24].address_from = 0;  //25
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[24].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[25].address_from = 0;  //26
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[25].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[26].address_from = 0;  //27
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[26].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[27].address_from = 0;  //28
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[27].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[28].address_from = 0;  //29
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[28].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[29].address_from = 0;  //30
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[29].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[30].address_from = 0;  //31
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[30].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[31].address_from = 0;  //32
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[31].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[32].address_from = 0;  //33
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[32].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[33].address_from = 0;  //34
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[33].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[34].address_from = 0;  //35
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[34].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[35].address_from = 0;  //36
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[35].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[36].address_from = 0;  //37
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[36].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[37].address_from = 0;  //38
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[37].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[38].address_from = 0;  //39
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[38].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[39].address_from = 0;  //40
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[39].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[40].address_from = 0;  //41
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[40].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[41].address_from = 0;  //42
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[41].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[42].address_from = 0;  //43
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[42].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[43].address_from = 0;  //44
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[43].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[44].address_from = 0;  //45
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[44].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[45].address_from = 0;  //46
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[45].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[46].address_from = 0;  //47
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[46].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[47].address_from = 0;  //48
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[47].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[48].address_from = 0;  //49
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[48].address_to = 0;
	//
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[49].address_from = 0;  //50
	ps_mem_data_set->s_config_moduls.s_routing_data_user_config.s_data_rout[49].address_to = 0;




	//-------------------------------------------------------------------------------
	// ----------------настройки запроса ПОРТА №1------------------------------------
	//-------------------------------------------------------------------------------
	// настройка запроса №1
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[0].pribor = 1;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[0].adress = 2;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[0].function = 4;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[0].start_adres = 1001;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[0].number =8;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[0].adress_pmz = 01;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[0].adres_status = 00;

	// настройка запроса №2
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[1].pribor = 1;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[1].adress = 2;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[1].function = 4;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[1].start_adres = 1002;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[1].number =8;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[1].adress_pmz = 01;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[1].adres_status = 00;

	// настройка запроса №3
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[2].pribor = 1;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[2].adress = 2;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[2].function = 4;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[2].start_adres = 1003;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[2].number =8;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[2].adress_pmz = 01;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[2].adres_status = 00;

	// настройка запроса №4
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[3].pribor = 1;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[3].adress = 2;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[3].function = 4;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[3].start_adres = 1004;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[3].number =3;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[3].adress_pmz = 01;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[3].adres_status = 00;

	// настройка запроса №5
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[4].pribor = 1;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[4].adress = 2;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[4].function = 4;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[4].start_adres = 1004;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[4].number =4;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[4].adress_pmz = 02;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[4].adres_status = 00;

	// настройка запроса №6
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[5].pribor = 1;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[5].adress = 2;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[5].function = 4;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[5].start_adres = 108;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[5].number =4;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[5].adress_pmz = 06;
	ps_mem_data_set->s_config_moduls.s_ulist_modbus[5].adres_status = 01;


	//

	// заполняю НЕ пользовательские настройки
	processing_config_init_not_user_config(&ps_mem_data_set->s_dev_staff);


	//FLASH_OPERATION_write_flash_16b((u16*) &s_mem_data_set,(sizeof(s_mem_data_set)+1) / 2 , address_write);

}





