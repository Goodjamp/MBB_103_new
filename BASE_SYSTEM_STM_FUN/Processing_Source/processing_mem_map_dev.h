/*
 * processing_mem_map_dev.h
 *
 *  Created on: April 19, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */

// щрнр тюик янгдюм дкъ рнцн врнаш хгаефюрэ оепейпеярмшу ухдепнб, ю хлеммн
// люйпннопедхкемхе "SIZE_MEM_MAP" хяонкэгсерэяъ б тюике "processing_mem_map.c", мн дкъ
// ецн пюяверю мсфмш люйпннопедхкемхъ х тюикю "processing_config_dev.h" х "processing_mem_map.h"
// мн б тюике "processing_config_dev.h" ондйкчвем тюик "processing_mem_map.h", онщрнлс
// еякх люйпннопедхкемхе "SIZE_MEM_MAP" нохяюрэ б тюике "processing_mem_map.h", рн мсфмн ондйкчвхрэ тюик
// "processing_config_dev.h", б йнрнпнл ондйкчвем тюик "processing_mem_map.h" - онксвюел оепейпеярмше
// ухдепш х ньхайс (йнлохкърнпс ме хгбеярмн йрн йнцн ондйкчвхк оепбшл)

#ifndef PROCESSING_MEM_MAP_DEV_H_
#define PROCESSING_MEM_MAP_DEV_H_

#include "stm32f10x.h"
#include "processing_mem_map.h"
#include "processing_config_dev.h"

// ПЮГЛЕП ЙЮПРШ ОЮЛЪРХ (ПЕЦХЯРПШ ДЮММШУ,НОЕПЮРХБМШЕ ПЕЦХЯРПШ Х ПЕЦХЯРПШ ЙНМТХЦСПЮЖХХ)
#define SIZE_MEM_MAP             NUM_REG_DATA*SIZE_PROCES_OBJECT+sizeof(S_oper_data)+NUM_REG_CONFIG*SIZE_REG_CONFIG

#endif
