/*
 * processing_routing_data.c
 *
 *  Created on: May 24, 2015
 *      Author: Gerasimchuk
 *      Version: 1
 */
#include "processing_routing_data.h"

//-------------------√ерасимчук-----------------------
// ”кзатель на начало карты пам€ти, приведенный к proces_object (из main.c)
extern S_proces_object_modbus *p_proces_object;
// ћютекс доступа к карте пам€ти, глобальный
extern xSemaphoreHandle h_mutex_mem_map_axes;
// структура настроек маршрутизации
S_routing_data_user_config *ps_rout_table;
extern S_address_oper_data s_address_oper_data;
//---------------------------------------------------------------------------



// ----функци€ INIT_MBB_fill_S_ROUT_TABLE--------
//функци€ processing_mem_map_fill_S_route_table - выполн€ет заполнение таблиц маршрутизации данных
INIT_MBB_Rezult processing_mem_map_fill_S_route_table(u8 *p_read_rout_table) {
	u16 k1;
	TYPE_REG_MEM_MAP type_mem;
	u8 *p_address;
	ps_rout_table=(S_routing_data_user_config*)p_read_rout_table;

	for (k1 = 0; k1 < ps_rout_table->num_rout; k1++) {
		// локализаци€ адреса "откуда копировать"
		type_mem=processing_mem_map_detect(ps_rout_table->s_data_rout[k1].address_from,&p_address );
		if(type_mem==ERROR_DATA){
			return MBB_INIT_ERROR;
		}
		// локализаци€ алреса "куда копировать"
		type_mem=processing_mem_map_detect(ps_rout_table->s_data_rout[k1].address_to,&p_address );
		if(type_mem!=PMZ_DATA){
			return MBB_INIT_ERROR;
		}
		// если адресса равны - тоже ошибка
		if(ps_rout_table->s_data_rout[k1].address_from == ps_rout_table->s_data_rout[k1].address_to){
			return MBB_INIT_ERROR;
		}
	}
	return MBB_INIT_OK;
}


// ---------функци€  routing_data_calc_address_oper_reg ---------
// функци€ routing_data_calc_address_oper_reg - выполн€ет рассчет адресов регистров (область оперативных регистров) устройства маршрутизации данных
// входные аргументы:
// ps_routing_data_address - указатель на структуру адресов регистров устройства (поле в глобальной структуре адресов устройства);
// adres_start             - начальный адрес оперативных регистров устройства в карте пам€ти. относительно него расчитываютьс€ адрес а вех остальных регистров устройства
// выходные аргументы:
//                         - адрес последнего регистра устройства
u16 routing_data_calc_address_oper_reg(S_routing_data_address *ps_routing_data_address, u16 adres_start){
	ps_routing_data_address->status_routing_data=adres_start;
	return adres_start;
};


// ---------функци€  retransmit_task ---------
// функци€ retransmit_task - «јƒј„ј –≈“–яЌ—Ћя÷»» выполн€ет ретр€нсл€цию данных по карте пам€ти
void t_processing_routing_data(void *pvParameters) {
	u8 k1;
	u8 *p_addres_from;
	TYPE_REG_MEM_MAP type_reg;
	// сохран€ю конфигурационные данные в указателе на структуру конфигурации,
	// а также выпон€б проверку
	if(processing_mem_map_fill_S_route_table((u8*)pvParameters))// если проверка не пройдена - удалить задачу + аварийна€ сигнализаци€ + «јѕ»—№ ¬  ј–“” ѕјћя“» !!!!
	{
		//processing_mem_map_write_s_proces_object_modbus(ERROR_OUT_MEM,1,s_address_oper_data.s_routing_data_address.status_routing_data);
		SET_GLOBAL_STATUS(DEV_1);
		vTaskDelete(NULL);
	};

	// продолжаю конфигурацию устройства
	//конфигурирую програмный блок “”
	while (1) {
		// беру мутекс, ожидать максимально допустимое врем€
		xSemaphoreTake(h_mutex_mem_map_axes, portMAX_DELAY);
		// выполн€ютьс€ все необходимые ретрансл€ции
		for (k1 = 0; k1 < ps_rout_table->num_rout; k1++) {
			// узнаю тип данных ретр€нсл€ции и откуда ретранслировать
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
		// отдаю мютекс
		taskYIELD(); // принудительно переключаю контекст
	} // end while(1){};
}





