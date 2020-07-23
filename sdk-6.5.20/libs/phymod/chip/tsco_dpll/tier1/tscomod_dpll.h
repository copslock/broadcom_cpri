/*
 *
 * This license is set out in https://raw.githubusercontent.com/Broadcom-Network-Switching-Software/OpenBCM/master/Legal/LICENSE file.
 * 
 * Copyright 2007-2020 Broadcom Inc. All rights reserved.
 */
/*
 * tscomod_dpll.h
 * Description:  Define enumration and macro
 */

#ifndef _TSCOMOD_DPLL_H_
#define _TSCOMOD_DPLL_H_

#include <phymod/phymod.h>
#include <phymod/phymod_debug.h>
#include "tsco/tier1/tscomod_sc_lkup_table.h"
#include "osprey7_v2l8p1/tier1/osprey7_v2l8p1_enum.h"


#define TSCO_DPLL_MODEL_NUMBER 0x2A

extern uint32_t tsco_dpll_spd_id_entry_53[TSCOMOD_SPEED_ID_TABLE_SIZE][TSCOMOD_SPEED_ID_ENTRY_SIZE];
extern uint32_t tsco_dpll_spd_id_entry_51[TSCOMOD_SPEED_ID_TABLE_SIZE][TSCOMOD_SPEED_ID_ENTRY_SIZE];
extern uint32_t tsco_dpll_spd_id_entry_41[TSCOMOD_SPEED_ID_TABLE_SIZE][TSCOMOD_SPEED_ID_ENTRY_SIZE];
extern uint32_t tsco_dpll_spd_id_entry_53_gsh[TSCOMOD_SPEED_ID_TABLE_SIZE][TSCOMOD_SPEED_ID_ENTRY_SIZE];
extern uint32_t tsco_dpll_spd_id_entry_51_gsh[TSCOMOD_SPEED_ID_TABLE_SIZE][TSCOMOD_SPEED_ID_ENTRY_SIZE];
extern uint32_t tsco_dpll_spd_id_entry_41_gsh[TSCOMOD_SPEED_ID_TABLE_SIZE][TSCOMOD_SPEED_ID_ENTRY_SIZE];
extern uint32_t tsco_dpll_am_table_entry[TSCOMOD_UM_TABLE_SIZE][TSCOMOD_AM_ENTRY_SIZE];
extern uint32_t tsco_dpll_um_table_entry[TSCOMOD_UM_TABLE_SIZE][TSCOMOD_UM_ENTRY_SIZE];
extern uint32_t tsco_dpll_speed_priority_mapping_table[TSCOMOD_SPEED_PRIORITY_MAPPING_TABLE_SIZE][TSCOMOD_SPEED_PRIORITY_MAPPING_ENTRY_SIZE];

static inline int
tscomod_dpll_pll_div_convert(phymod_tscbh_pll_multiplier_t pll_div, enum osprey7_v2l8p1_pll_div_enum *target_pll_div)
{
    switch(pll_div) {
        case phymod_TSCBH_PLL_DIV132:
            *target_pll_div = OSPREY7_V2L8P1_PLL_DIV_132;
            break;
        case phymod_TSCBH_PLL_DIV165:
            *target_pll_div = OSPREY7_V2L8P1_PLL_DIV_165;
            break;
        case phymod_TSCBH_PLL_DIV170:
            *target_pll_div = OSPREY7_V2L8P1_PLL_DIV_170;
            break;
        default:
            return PHYMOD_E_FAIL;
    }
    return PHYMOD_E_NONE;
}



#endif /* _TSCOMOD_DPLL_H_ */
