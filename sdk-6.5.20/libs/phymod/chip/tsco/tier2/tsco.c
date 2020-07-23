/*
 *
 * 
 *
 * This license is set out in https://raw.githubusercontent.com/Broadcom-Network-Switching-Software/OpenBCM/master/Legal/LICENSE file.
 * 
 * Copyright 2007-2020 Broadcom Inc. All rights reserved.
 */

#include <phymod/phymod.h>
#include <phymod/phymod_system.h>
#include <phymod/phymod_util.h>
#include <phymod/phymod_dispatch.h>
#include <phymod/chip/tsco.h>
#include <phymod/chip/bcmi_tsco_xgxs_defs.h>
#include "tsco/tier1/tscomod.h"
#include "tsco/tier1/tscomod_sc_lkup_table.h"
#include "osprey7_v2l8p1/tier2/osprey7_v2l8p1_ucode.h"
#include "osprey7_v2l8p1/tier1/osprey7_v2l8p1_cfg_seq.h"
#include "osprey7_v2l8p1/tier1/osprey7_v2l8p1_enum.h"
#include "osprey7_v2l8p1/tier1/osprey7_v2l8p1_common.h"
#include "osprey7_v2l8p1/tier1/osprey7_v2l8p1_interface.h"
#include "osprey7_v2l8p1/tier1/osprey7_v2l8p1_dependencies.h"
#include "osprey7_v2l8p1/tier1/osprey7_v2l8p1_internal.h"
#include "osprey7_v2l8p1/tier1/public/osprey7_api_uc_vars_rdwr_defns_public.h"
#include "osprey7_v2l8p1/tier1/osprey7_v2l8p1_access.h"

extern uint32_t tsco_spd_id_entry_53[TSCOMOD_SPEED_ID_TABLE_SIZE][TSCOMOD_SPEED_ID_ENTRY_SIZE];
extern uint32_t tsco_spd_id_entry_51[TSCOMOD_SPEED_ID_TABLE_SIZE][TSCOMOD_SPEED_ID_ENTRY_SIZE];
extern uint32_t tsco_spd_id_entry_41[TSCOMOD_SPEED_ID_TABLE_SIZE][TSCOMOD_SPEED_ID_ENTRY_SIZE];
extern uint32_t tsco_spd_id_entry_53_gsh[TSCOMOD_SPEED_ID_TABLE_SIZE][TSCOMOD_SPEED_ID_ENTRY_SIZE];
extern uint32_t tsco_spd_id_entry_51_gsh[TSCOMOD_SPEED_ID_TABLE_SIZE][TSCOMOD_SPEED_ID_ENTRY_SIZE];
extern uint32_t tsco_spd_id_entry_41_gsh[TSCOMOD_SPEED_ID_TABLE_SIZE][TSCOMOD_SPEED_ID_ENTRY_SIZE];
extern uint32_t tsco_am_table_entry[TSCOMOD_UM_TABLE_SIZE][TSCOMOD_AM_ENTRY_SIZE];
extern uint32_t tsco_um_table_entry[TSCOMOD_UM_TABLE_SIZE][TSCOMOD_UM_ENTRY_SIZE];
extern uint32_t tsco_speed_priority_mapping_table[TSCOMOD_SPEED_PRIORITY_MAPPING_TABLE_SIZE][TSCOMOD_SPEED_PRIORITY_MAPPING_ENTRY_SIZE];

#define TSCO_SERDES_ID           0x29 /* 0x9008 Main0_serdesID - Serdes ID Register */


int tsco_core_identify(const phymod_core_access_t* core, uint32_t core_id,
                       uint32_t* is_identified)
{
    PHYID2r_t id2;
    PHYID3r_t id3;
    MAIN0_SERDESIDr_t serdesid;
    int ioerr = 0;

    *is_identified = 0;
    ioerr += READ_PHYID2r(&core->access, &id2);
    ioerr += READ_PHYID3r(&core->access, &id3);

    if (PHYID2r_REGID1f_GET(id2) == TSCOMOD_ID0 &&
       (PHYID3r_REGID2f_GET(id3) == TSCOMOD_ID1)) {
        /* PHY IDs match - now check PCS model */
        ioerr += READ_MAIN0_SERDESIDr(&core->access, &serdesid);
        if ( (MAIN0_SERDESIDr_MODEL_NUMBERf_GET(serdesid)) == TSCO_SERDES_ID)  {
            *is_identified = 1;
        }
    }
    return ioerr ? PHYMOD_E_IO : PHYMOD_E_NONE;
    return PHYMOD_E_NONE;
}

int tsco_core_info_get(const phymod_core_access_t* core, phymod_core_info_t* info)
{
    int rv = 0;
    MAIN0_SERDESIDr_t serdes_id;
    char core_name[15] = "Tsco";
    PHYID2r_t id2;
    PHYID3r_t id3;
    const phymod_access_t *pm_acc = &core->access;

    rv = READ_MAIN0_SERDESIDr(&core->access, &serdes_id);

    info->serdes_id = MAIN0_SERDESIDr_GET(serdes_id);
    PHYMOD_IF_ERR_RETURN
        (phymod_core_name_get(core, MAIN0_SERDESIDr_GET(serdes_id), core_name, info));
    info->serdes_id = MAIN0_SERDESIDr_GET(serdes_id);
    info->core_version = phymodCoreVersionTsco;

    PHYMOD_IF_ERR_RETURN(READ_PHYID2r(pm_acc, &id2));
    PHYMOD_IF_ERR_RETURN(READ_PHYID3r(pm_acc, &id3));

    info->phy_id0 = (uint16_t) id2.v[0];
    info->phy_id1 = (uint16_t) id3.v[0];

    return rv;
}

int tsco_phy_firmware_lane_config_get(const phymod_phy_access_t* phy,
                                      phymod_firmware_lane_config_t* fw_config)
{
    struct osprey7_v2l8p1_uc_lane_config_st lane_config;
    phymod_phy_access_t pm_phy_copy;

    PHYMOD_MEMCPY(&pm_phy_copy, phy, sizeof(pm_phy_copy));

    PHYMOD_MEMSET(&lane_config, 0x0, sizeof(lane_config));
    PHYMOD_MEMSET(fw_config, 0, sizeof(*fw_config));

    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_get_uc_lane_cfg(&pm_phy_copy.access, &lane_config));

    fw_config->LaneConfigFromPCS     = lane_config.field.lane_cfg_from_pcs;
    fw_config->AnEnabled             = lane_config.field.an_enabled;
    fw_config->DfeOn                 = lane_config.field.dfe_on;
    fw_config->LpDfeOn               = lane_config.field.dfe_lp_mode;
    fw_config->ForceBrDfe            = lane_config.field.force_brdfe_on;
    fw_config->MediaType             = lane_config.field.media_type;
    fw_config->UnreliableLos         = lane_config.field.unreliable_los;
    fw_config->Cl72AutoPolEn         = lane_config.field.cl72_auto_polarity_en;
    fw_config->ScramblingDisable     = lane_config.field.scrambling_dis;
    fw_config->Cl72RestTO            = lane_config.field.cl72_restart_timeout_en;
    fw_config->ForceExtenedReach     = lane_config.field.force_er;
    fw_config->ForceNormalReach      = lane_config.field.force_nr;
    fw_config->LpPrecoderEnabled     = lane_config.field.lp_has_prec_en;
    fw_config->ForcePAM4Mode         = lane_config.field.force_pam4_mode;
    fw_config->ForceNRZMode          = lane_config.field.force_nrz_mode;

    return PHYMOD_E_NONE;
}

int tsco_phy_tx_lane_control_set(const phymod_phy_access_t* phy,
                                 phymod_phy_tx_lane_control_t tx_control)
{
    phymod_firmware_lane_config_t fw_lane_config;
    phymod_phy_access_t phy_copy;

    PHYMOD_IF_ERR_RETURN (tsco_phy_firmware_lane_config_get(phy, &fw_lane_config));
    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    switch (tx_control) {
        case phymodTxElectricalIdleEnable:
            /* idle is the same as tx disable */
            if (fw_lane_config.LaneConfigFromPCS == 0) {
                PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_tx_disable(&phy_copy.access, 1));
            } else {
                return PHYMOD_E_PARAM;
            }
            break;
        case phymodTxElectricalIdleDisable:
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_tx_disable(&phy_copy.access, 0));
            break;
        case phymodTxSquelchOn:
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_tx_disable(&phy_copy.access, 1));
            break;
        case phymodTxSquelchOff:
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_tx_disable(&phy_copy.access, 0));
            break;
        default:
            return PHYMOD_E_PARAM;
    }
    return PHYMOD_E_NONE;
}

int tsco_phy_tx_lane_control_get(const phymod_phy_access_t* phy,
                                 phymod_phy_tx_lane_control_t *tx_control)
{
    uint8_t tx_disable;
    uint32_t lb_enable;
    phymod_phy_access_t pm_phy_copy;
    int start_lane, num_lane;

    PHYMOD_MEMCPY(&pm_phy_copy, phy, sizeof(pm_phy_copy));
    /* next program the tx fir taps and driver current based on the input */
    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));

    pm_phy_copy.access.lane_mask = 0x1 << start_lane;

    PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_tx_disable_get(&pm_phy_copy.access, &tx_disable));

    /* next check if PMD loopback is on */
    if (tx_disable) {
        PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_dig_lpbk_get(&pm_phy_copy.access, &lb_enable));
        if (lb_enable) tx_disable = 0;
    }

    if (tx_disable) {
        *tx_control = phymodTxSquelchOn;
    } else {
        *tx_control = phymodTxSquelchOff;
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_rx_lane_control_set(const phymod_phy_access_t* phy,
                                 phymod_phy_rx_lane_control_t rx_control)
{
    phymod_phy_access_t pm_phy_copy;

    PHYMOD_MEMCPY(&pm_phy_copy, phy, sizeof(pm_phy_copy));

    switch (rx_control) {
        case phymodRxSquelchOn:
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_pmd_force_signal_detect(&pm_phy_copy.access, 1, 0));
            break;
        case phymodRxSquelchOff:
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_pmd_force_signal_detect(&pm_phy_copy.access, 0, 0));
            break;
        default:
            return PHYMOD_E_PARAM;
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_rx_lane_control_get(const phymod_phy_access_t* phy,
                                 phymod_phy_rx_lane_control_t* rx_control)
{
    int rx_squelch_enable;
    uint32_t lb_enable;
    uint8_t force_en, force_val;
    phymod_phy_access_t pm_phy_copy;

    PHYMOD_MEMCPY(&pm_phy_copy, phy, sizeof(pm_phy_copy));

    /* first get the force enabled bit and forced value */
    PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_pmd_force_signal_detect_get(&pm_phy_copy.access, &force_en, &force_val));

    if (force_en & (!force_val)) {
        rx_squelch_enable = 1;
    } else {
        rx_squelch_enable = 0;
    }

    /* next check if PMD loopback is on */
    if (rx_squelch_enable) {
        PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_dig_lpbk_get(&pm_phy_copy.access, &lb_enable));
        if (lb_enable) rx_squelch_enable = 0;
    }
    if(rx_squelch_enable) {
        *rx_control = phymodRxSquelchOn;
    } else {
        *rx_control = phymodRxSquelchOff;
    }
    return PHYMOD_E_NONE;
}


STATIC
int _tsco_core_firmware_load(const phymod_core_access_t* core, const phymod_core_init_config_t* init_config)
{
    phymod_core_access_t  core_copy;
    phymod_phy_access_t phy_access;
    unsigned int osprey_ucode_len;
    unsigned char *osprey_ucode;

    PHYMOD_MEMCPY(&core_copy, core, sizeof(core_copy));
    TSCOMOD_CORE_TO_PHY_ACCESS(&phy_access, core);

    phy_access.access.lane_mask = 0x1;
    osprey_ucode = osprey7_v2l8p1_ucode_get();
    osprey_ucode_len = OSPREY7_V2L8P1_UCODE_IMAGE_SIZE;

    switch(init_config->firmware_load_method){
    case phymodFirmwareLoadMethodInternal:
        PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_ucode_load(&core_copy.access, osprey_ucode, osprey_ucode_len));
        break;
    case phymodFirmwareLoadMethodExternal:
        PHYMOD_NULL_CHECK(init_config->firmware_loader);
        PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_ucode_init(&core_copy.access));
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_pram_firmware_enable(&core_copy.access, 1, 0));
        PHYMOD_IF_ERR_RETURN(init_config->firmware_loader(core, osprey_ucode_len, osprey_ucode));
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_pram_firmware_enable(&core_copy.access, 0, 0));
        break;
    case phymodFirmwareLoadMethodNone:
        break;
    default:
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_CONFIG, (_PHYMOD_MSG("illegal fw load method %u"), init_config->firmware_load_method));
    }

    return PHYMOD_E_NONE;
}


int tsco_phy_tx_set(const phymod_phy_access_t* phy, const phymod_tx_t* tx)
{
    phymod_phy_access_t phy_copy;
    int start_lane, num_lane, i, port_start_lane, port_num_lane;
    enum osprey7_v2l8p1_txfir_tap_enable_enum enable_taps = OSPREY7_V2L8P1_NRZ_6TAP;
    uint32_t lane_reset, pcs_lane_enable, port_lane_mask = 0;

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));
    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    /*get the start lane of the port lane mask */
    PHYMOD_IF_ERR_RETURN
        (tscomod_port_start_lane_get(&phy_copy.access, &port_start_lane, &port_num_lane));

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_mask_get(port_start_lane, port_num_lane, &port_lane_mask));

    phy_copy.access.lane_mask = 1 << port_start_lane;

    /*next check if PCS lane is in reset */
    PHYMOD_IF_ERR_RETURN
        (tscomod_enable_get(&phy_copy.access, &pcs_lane_enable));

    /*first check if lane is in reset */
    phy_copy.access.lane_mask = 1 << start_lane;
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_lane_soft_reset_get(&phy_copy.access, &lane_reset));

    /* disable pcs lane if pcs lane not in rset */
    if (pcs_lane_enable) {
        phy_copy.access.lane_mask = 1 << port_start_lane;
        PHYMOD_IF_ERR_RETURN
            (tscomod_disable_set(&phy_copy.access));
    }
    if (!lane_reset) {
        PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_lane_soft_reset(&phy_copy.access, 1));
    }

    for (i = 0; i < num_lane; i++) {
        if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
            continue;
        }
        phy_copy.access.lane_mask = 1 << (start_lane + i);
        /*next check 3 tap mode or 6 tap mode */
        if (tx->tap_mode == phymodTxTapMode3Tap) {
            if (tx->sig_method == phymodSignallingMethodNRZ) {
                enable_taps= OSPREY7_V2L8P1_NRZ_LP_3TAP;
            } else {
                enable_taps= OSPREY7_V2L8P1_PAM4_LP_3TAP;
            }
            PHYMOD_IF_ERR_RETURN
                (osprey7_v2l8p1_apply_txfir_cfg(&phy_copy.access,
                                                 enable_taps,
                                                 tx->pre3,
                                                 0,
                                                 tx->pre,
                                                 tx->main,
                                                 tx->post,
                                                 0));
        } else {
            if (tx->sig_method == phymodSignallingMethodNRZ) {
                enable_taps= OSPREY7_V2L8P1_NRZ_6TAP;
            } else {
                enable_taps= OSPREY7_V2L8P1_PAM4_6TAP;
            }
            PHYMOD_IF_ERR_RETURN
                (osprey7_v2l8p1_apply_txfir_cfg(&phy_copy.access,
                                                 enable_taps,
                                                 tx->pre3,
                                                 tx->pre2,
                                                 tx->pre,
                                                 tx->main,
                                                 tx->post,
                                                 tx->post2));
        }
    }

    if (!lane_reset) {
        PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_lane_soft_reset(&phy_copy.access, 0));
    }

    /* re-enable pcs lane if pcs lane not in rset */
    if (pcs_lane_enable) {
        phy_copy.access.lane_mask = 1 << port_start_lane;
        PHYMOD_IF_ERR_RETURN
            (tscomod_enable_set(&phy_copy.access));
    }
    return PHYMOD_E_NONE;
}

STATIC
int _tsco_core_tx_rx_mux_select_set(const phymod_core_access_t* core, const phymod_lane_map_t* lane_map)
{
    uint32_t lane, pcs_tx_swap = 0, pcs_rx_swap = 0;
    phymod_core_access_t  core_copy;

    PHYMOD_MEMCPY(&core_copy, core, sizeof(core_copy));

    if (lane_map->num_of_lanes != TSCOMOD_NOF_LANES_IN_CORE){
        return PHYMOD_E_CONFIG;
    }

    for (lane = 0; lane < TSCOMOD_NOF_LANES_IN_CORE; lane++){
        if ((lane_map->lane_map_tx[lane] >= TSCOMOD_NOF_LANES_IN_CORE)||
             (lane_map->lane_map_rx[lane] >= TSCOMOD_NOF_LANES_IN_CORE)){
            return PHYMOD_E_CONFIG;
        }
        /*encode each lane as four bits*/
        pcs_tx_swap += lane_map->lane_map_tx[lane]<<(lane*4);
        pcs_rx_swap += lane_map->lane_map_rx[lane]<<(lane*4);
    }
    PHYMOD_IF_ERR_RETURN
        (tscomod_pcs_tx_mux_select(&core_copy.access, pcs_tx_swap));
    PHYMOD_IF_ERR_RETURN
        (tscomod_pcs_rx_mux_select(&core_copy.access, pcs_rx_swap));

    return PHYMOD_E_NONE;
}

int tsco_core_lane_map_set(const phymod_core_access_t* core, const phymod_lane_map_t* lane_map)
{
    uint32_t lane, pcs_tx_swap = 0, pcs_rx_swap = 0;
    uint8_t pmd_tx_addr[8], pmd_rx_addr[8];
    phymod_core_access_t  core_copy;

    PHYMOD_MEMCPY(&core_copy, core, sizeof(core_copy));

    if (lane_map->num_of_lanes != TSCOMOD_NOF_LANES_IN_CORE){
        return PHYMOD_E_CONFIG;
    }

    for (lane = 0; lane < TSCOMOD_NOF_LANES_IN_CORE; lane++){
        if ((lane_map->lane_map_tx[lane] >= TSCOMOD_NOF_LANES_IN_CORE)||
             (lane_map->lane_map_rx[lane] >= TSCOMOD_NOF_LANES_IN_CORE)){
            return PHYMOD_E_CONFIG;
        }
        /*encode each lane as four bits*/
        
        pcs_tx_swap += lane_map->lane_map_tx[lane]<<(lane*4);
        pcs_rx_swap += lane_map->lane_map_rx[lane]<<(lane*4);
    }
    /* PMD lane addr is based on PCS logical to physical mapping*/
    for (lane = 0; lane < TSCOMOD_NOF_LANES_IN_CORE; lane++){
        pmd_tx_addr[((pcs_tx_swap >> (lane*4)) & 0xf)] = lane;
        pmd_rx_addr[((pcs_rx_swap >> (lane*4)) & 0xf)] = lane;
    }

    PHYMOD_IF_ERR_RETURN
        (tscomod_pcs_tx_m1_lane_swap(&core_copy.access, pcs_tx_swap));
    PHYMOD_IF_ERR_RETURN
        (tscomod_pcs_tx_m0_lane_swap(&core_copy.access, pcs_tx_swap));
    PHYMOD_IF_ERR_RETURN
        (tscomod_pcs_rx_lane_swap(&core_copy.access, pcs_rx_swap));

    COMPILER_REFERENCE(pmd_tx_addr);
    COMPILER_REFERENCE(pmd_rx_addr);
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_map_lanes(&core_copy.access, TSCOMOD_NOF_LANES_IN_CORE, pmd_tx_addr, pmd_rx_addr));

    return PHYMOD_E_NONE;
}

int tsco_core_lane_map_get(const phymod_core_access_t* core, phymod_lane_map_t* lane_map)
{

    return PHYMOD_E_NONE;
}

static 
int _tsco_phy_firmware_lane_config_set(const phymod_phy_access_t* phy, phymod_firmware_lane_config_t fw_config)
{
    uint32_t is_warm_boot;
    struct osprey7_v2l8p1_uc_lane_config_st serdes_firmware_config;
    phymod_phy_access_t phy_copy;
    int start_lane, num_lane, i;

    PHYMOD_MEMSET(&serdes_firmware_config, 0x0, sizeof(serdes_firmware_config));
    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));
    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    for (i = 0; i < num_lane; i++) {
        if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
            continue;
        }
        phy_copy.access.lane_mask = 1 << (start_lane + i);
        serdes_firmware_config.field.lane_cfg_from_pcs      = fw_config.LaneConfigFromPCS;
        serdes_firmware_config.field.an_enabled             = fw_config.AnEnabled;
        serdes_firmware_config.field.dfe_on                 = fw_config.DfeOn;
        serdes_firmware_config.field.force_brdfe_on         = fw_config.ForceBrDfe;
        serdes_firmware_config.field.scrambling_dis         = fw_config.ScramblingDisable;
        serdes_firmware_config.field.unreliable_los         = fw_config.UnreliableLos;
        serdes_firmware_config.field.media_type             = fw_config.MediaType;
        serdes_firmware_config.field.dfe_lp_mode            = fw_config.LpDfeOn;
        serdes_firmware_config.field.cl72_auto_polarity_en  = fw_config.Cl72AutoPolEn;
        serdes_firmware_config.field.cl72_restart_timeout_en = fw_config.Cl72RestTO;
        serdes_firmware_config.field.force_er               = fw_config.ForceExtenedReach;
        serdes_firmware_config.field.force_nr               = fw_config.ForceNormalReach;
        serdes_firmware_config.field.force_nrz_mode         = fw_config.ForceNRZMode;
        serdes_firmware_config.field.force_pam4_mode        = fw_config.ForcePAM4Mode;
        serdes_firmware_config.field.lp_has_prec_en         = fw_config.LpPrecoderEnabled;

        PHYMOD_IF_ERR_RETURN(PHYMOD_IS_WRITE_DISABLED(&phy->access, &is_warm_boot));

        if (!is_warm_boot) {
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_set_uc_lane_cfg(&phy_copy.access, serdes_firmware_config));
        }
    }

    return PHYMOD_E_NONE;
}


int tsco_phy_firmware_lane_config_set(const phymod_phy_access_t* phy,
                                      phymod_firmware_lane_config_t fw_config)
{
    phymod_phy_access_t phy_copy;
    uint32_t lane_reset, pcs_lane_enable, port_lane_mask = 0;
    int start_lane, num_lane, port_start_lane, port_num_lane;

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));

    /*get the start lane of the port lane mask */
    PHYMOD_IF_ERR_RETURN
        (tscomod_port_start_lane_get(&phy_copy.access, &port_start_lane, &port_num_lane));

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_mask_get(port_start_lane, port_num_lane, &port_lane_mask));

    /*first check if lane is in reset */
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_lane_soft_reset_get(&phy_copy.access, &lane_reset));

    /*next check if PCS lane is in reset */
    phy_copy.access.lane_mask = 1 << port_start_lane;
    PHYMOD_IF_ERR_RETURN
        (tscomod_enable_get(&phy_copy.access, &pcs_lane_enable));

    /* disable pcs lane if pcs lane not in rset */
    if (pcs_lane_enable) {
        PHYMOD_IF_ERR_RETURN
            (tscomod_disable_set(&phy_copy.access));
    }

    /* if lane is not in reset, then reset the lane first */
    if (!lane_reset) {
        PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_lane_soft_reset(&phy_copy.access, 1));
    }

    PHYMOD_IF_ERR_RETURN
         (_tsco_phy_firmware_lane_config_set(phy, fw_config));

    if (!lane_reset) {
        PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_lane_soft_reset(&phy_copy.access, 0));
    }

    /* re-enable pcs lane if pcs lane not in rset */
    if (pcs_lane_enable) {
        phy_copy.access.lane_mask = 1 << port_start_lane;
        PHYMOD_IF_ERR_RETURN
            (tscomod_enable_set(&phy_copy.access));
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_polarity_set(const phymod_phy_access_t* phy, const phymod_polarity_t* polarity)
{
    phymod_phy_access_t phy_copy;
    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_tx_rx_polarity_set(&phy_copy.access, polarity->tx_polarity, polarity->rx_polarity));

    return PHYMOD_E_NONE;
}

int tsco_phy_polarity_get(const phymod_phy_access_t* phy, phymod_polarity_t* polarity)
{
    int start_lane, num_lane, i;
    phymod_polarity_t temp_pol;
    phymod_phy_access_t phy_copy;
    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    /* figure out the lane num and start_lane based on the input */
    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));

    polarity->tx_polarity = 0;
    polarity->rx_polarity = 0;
    for (i = 0; i < num_lane; i++) {
        if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
            continue;
        }
        phy_copy.access.lane_mask = 0x1 << (i + start_lane);
        temp_pol.tx_polarity = 0;
        temp_pol.rx_polarity = 0;
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_tx_rx_polarity_get(&phy_copy.access, &temp_pol.tx_polarity, &temp_pol.rx_polarity));
        polarity->tx_polarity |= ((temp_pol.tx_polarity & 0x1) << i);
        polarity->rx_polarity |= ((temp_pol.rx_polarity & 0x1) << i);
    }

    return PHYMOD_E_NONE;
}

int tsco_port_enable_set(const phymod_phy_access_t* phy, uint32_t enable)
{
    phymod_phy_access_t phy_copy;
    uint32_t pcs_enable;
    int start_lane, num_lane, port_an_enable, port_enable;

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));

    /* first read port an enable bit */
    PHYMOD_IF_ERR_RETURN
        (tscomod_port_an_mode_enable_get(&phy_copy.access, &port_an_enable));

    /* next read current port enable bit */
    PHYMOD_IF_ERR_RETURN
        (tscomod_port_enable_get(&phy_copy.access, &port_enable));

    /*next check if PCS lane is in reset */
    PHYMOD_IF_ERR_RETURN
        (tscomod_enable_get(&phy_copy.access, &pcs_enable));

    if ((port_an_enable) || (port_enable && !pcs_enable)) {
        /* cuurent port is in An mode mode */
        if (enable == 1) {
            /* next release both tx/rx squelch */
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_tx_disable(&phy_copy.access, 0));
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_pmd_force_signal_detect(&phy_copy.access, 0, 0));
            phy_copy.access.lane_mask = 1 << start_lane;
        }
        PHYMOD_IF_ERR_RETURN
            (tscomod_port_cl73_enable_set(&phy_copy.access, enable));
        /*check if enable ==0 */
        if (!enable) {
            /* next set both tx/rx squelch */
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_tx_disable(&phy_copy.access, 1));
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_pmd_force_signal_detect(&phy_copy.access, 1, 0));
        }

    } else {
        /* cuurent port is in forced speed mode */
        if (enable == 1) {
            /* next release both tx/rx squelch */
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_tx_disable(&phy_copy.access, 0));
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_pmd_force_signal_detect(&phy_copy.access, 0, 0));
            phy_copy.access.lane_mask = 1 << start_lane;
            /* enable speed control bit */
            PHYMOD_IF_ERR_RETURN
                (tscomod_enable_set(&phy_copy.access));
        } else if (enable == 0) {
            /* disable speed control bit */
            PHYMOD_IF_ERR_RETURN
                (tscomod_disable_set(&phy_copy.access));
            /* next set both tx/rx squelch */
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_tx_disable(&phy_copy.access, 1));
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_pmd_force_signal_detect(&phy_copy.access, 1, 0));
        }
    }

    /* next set port enable bit */
    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
    PHYMOD_IF_ERR_RETURN
        (tscomod_port_enable_set(&phy_copy.access, enable));


    return PHYMOD_E_NONE;
}

int tsco_port_enable_get(const phymod_phy_access_t* phy, uint32_t* enable)
{
    phymod_phy_access_t phy_copy;
    int temp_enable;

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    /* first read port an enable bit */
    PHYMOD_IF_ERR_RETURN
        (tscomod_port_enable_get(&phy_copy.access, &temp_enable));

    *enable = (uint32_t) temp_enable;

    return PHYMOD_E_NONE;
}

int tsco_phy_tx_get(const phymod_phy_access_t* phy, phymod_tx_t* tx)
{
    uint8_t pmd_tx_tap_mode;
    uint16_t tx_tap_nrz_mode = 0;
    int16_t val;
    phymod_phy_access_t phy_copy;
    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    /* read current tx tap mode */
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_tx_tap_mode_get(&phy_copy.access, &pmd_tx_tap_mode));

    /*read current tx NRZ mode control info */
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_tx_nrz_mode_get(&phy_copy.access, &tx_tap_nrz_mode));

    if (pmd_tx_tap_mode == 0) {
        /* 3 tap mode */
        tx->tap_mode = phymodTxTapMode3Tap;
    } else {
        tx->tap_mode = phymodTxTapMode6Tap;
    }

    if (tx_tap_nrz_mode) {
        tx->sig_method = phymodSignallingMethodNRZ;
    } else {
        tx->sig_method = phymodSignallingMethodPAM4;
    }

    /*next check 3 tap mode or 6 tap mode */
    if (tx->tap_mode == phymodTxTapMode3Tap) {
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_read_tx_afe(&phy_copy.access, OSPREY7_V2L8P1_TX_AFE_TAP0, &val));
        tx->pre = val;
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_read_tx_afe(&phy_copy.access, OSPREY7_V2L8P1_TX_AFE_TAP1, &val));
        tx->main = val;
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_read_tx_afe(&phy_copy.access, OSPREY7_V2L8P1_TX_AFE_TAP2, &val));
        tx->post = val;
        tx->pre2 = 0;
        tx->post2 = 0;
        tx->post3 = 0;
        tx->pre3 = 0;
    } else {
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_read_tx_afe(&phy_copy.access, OSPREY7_V2L8P1_TX_AFE_TAP0, &val));
        tx->pre3 = val;
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_read_tx_afe(&phy_copy.access, OSPREY7_V2L8P1_TX_AFE_TAP1, &val));
        tx->pre2 = val;
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_read_tx_afe(&phy_copy.access, OSPREY7_V2L8P1_TX_AFE_TAP2, &val));
        tx->pre = val;
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_read_tx_afe(&phy_copy.access, OSPREY7_V2L8P1_TX_AFE_TAP3, &val));
        tx->main = val;
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_read_tx_afe(&phy_copy.access, OSPREY7_V2L8P1_TX_AFE_TAP4, &val));
        tx->post = val;
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_read_tx_afe(&phy_copy.access, OSPREY7_V2L8P1_TX_AFE_TAP5, &val));
        tx->post2 = val;
        tx->post3 = 0;
    }

    return PHYMOD_E_NONE;
}

/* This function based on num_lane, data_rate and fec_type
 * assign force speed SW speed_id.
 */

STATIC
int _tsco_phy_speed_id_set(int num_lane,
                            uint32_t data_rate,
                            phymod_fec_type_t fec_type,
                            tscomod_spd_intfc_type_t* spd_intf)
{
    if (num_lane == 1) {
        switch (data_rate) {
            case 10000:
                if (fec_type == phymod_fec_None) {
                    *spd_intf = TSCOMOD_SPD_10G_IEEE_KR1;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
                break;
            case 20000:
                if (fec_type == phymod_fec_None) {
                    *spd_intf = TSCOMOD_SPD_20G_BRCM_KR1;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
                break;
            case 25000:
                if (fec_type == phymod_fec_None) {
                    *spd_intf = TSCOMOD_SPD_25G_BRCM_NO_FEC_KR1_CR1;
                } else if (fec_type == phymod_fec_CL91) {
                    *spd_intf = TSCOMOD_SPD_25G_BRCM_FEC_528_KR1_CR1;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
                break;
            case 50000:
                if (fec_type == phymod_fec_CL91) {
                    *spd_intf = TSCOMOD_SPD_50G_BRCM_FEC_528_CR1_KR1;
                } else if (fec_type == phymod_fec_RS544) {
                    *spd_intf = TSCOMOD_SPD_50G_IEEE_KR1_CR1;
                } else if (fec_type == phymod_fec_RS272) {
                    *spd_intf = TSCOMOD_SPD_50G_BRCM_FEC_272_CR1_KR1;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
                break;
            case 100000:
                if (fec_type == phymod_fec_RS544_2XN) {
                    *spd_intf = TSCOMOD_SPD_100G_IEEE_KR1_CR1;
                } else if (fec_type == phymod_fec_RS544) {
                    *spd_intf = TSCOMOD_SPD_100G_BRCM_KR1_CR1;
                } else if (fec_type == phymod_fec_RS272) {
                    *spd_intf = TSCOMOD_SPD_100G_BRCM_FEC_272_KR1_CR1;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
                break;
            default:
                return PHYMOD_E_UNAVAIL;
        }
    } else if (num_lane == 2) {
        switch (data_rate) {
            case 40000:
                if (fec_type == phymod_fec_None) {
                    *spd_intf = TSCOMOD_SPD_40G_BRCM_CR2;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
                break;
            case 50000:
                if (fec_type == phymod_fec_None) {
                    *spd_intf = TSCOMOD_SPD_50G_BRCM_CR2_KR2_NO_FEC;
                } else if (fec_type == phymod_fec_CL91) {
                    *spd_intf = TSCOMOD_SPD_50G_BRCM_CR2_KR2_RS_FEC;
                } else if (fec_type == phymod_fec_RS544) {
                    *spd_intf = TSCOMOD_SPD_50G_BRCM_FEC_544_CR2_KR2;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
                break;
            case 100000:
                if (fec_type == phymod_fec_None) {
                    *spd_intf = TSCOMOD_SPD_100G_BRCM_NO_FEC_KR2_CR2;
                } else if (fec_type == phymod_fec_CL91) {
                    *spd_intf = TSCOMOD_SPD_100G_BRCM_FEC_528_KR2_CR2;
                } else if (fec_type == phymod_fec_RS544) {
                    *spd_intf = TSCOMOD_SPD_100G_IEEE_KR2_CR2;
                } else if (fec_type == phymod_fec_RS272) {
                    *spd_intf = TSCOMOD_SPD_100G_BRCM_FEC_272_CR2_KR2;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
                break;
            case 200000:
                if (fec_type == phymod_fec_RS544_2XN) {
                    *spd_intf = TSCOMOD_SPD_200G_IEEE_KR2_CR2;
                } else if (fec_type == phymod_fec_RS272_2XN) {
                    *spd_intf = TSCOMOD_SPD_200G_BRCM_FEC_272_KR2_CR2;
                } else if (fec_type == phymod_fec_RS544) {
                    *spd_intf = TSCOMOD_SPD_200G_BRCM_FEC_544_KR2_CR2;
                } else if (fec_type == phymod_fec_RS272) {
                    *spd_intf = TSCOMOD_SPD_200G_BRCM_FEC_272_N2;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
                break;
            default:
                return PHYMOD_E_UNAVAIL;
        }
    } else if (num_lane == 4) {
        switch (data_rate) {
            case 40000:
                if (fec_type == phymod_fec_None) {
                    *spd_intf = TSCOMOD_SPD_40G_IEEE_KR4;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
                break;
            case 100000:
                if (fec_type == phymod_fec_None) {
                    *spd_intf = TSCOMOD_SPD_100G_BRCM_NO_FEC_X4;
                } else if (fec_type == phymod_fec_CL91) {
                    *spd_intf = TSCOMOD_SPD_100G_IEEE_KR4;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
                break;
            case 200000:
                if (fec_type == phymod_fec_None) {
                    *spd_intf = TSCOMOD_SPD_200G_BRCM_NO_FEC_KR4_CR4;
                } else if (fec_type == phymod_fec_RS544) {
                    *spd_intf = TSCOMOD_SPD_200G_BRCM_KR4_CR4;
                } else if (fec_type == phymod_fec_RS544_2XN) {
                    *spd_intf = TSCOMOD_SPD_200G_IEEE_KR4_CR4;
                } else if (fec_type == phymod_fec_RS272) {
                    *spd_intf = TSCOMOD_SPD_200G_BRCM_FEC_272_N4;
                } else if (fec_type == phymod_fec_RS272_2XN) {
                    *spd_intf = TSCOMOD_SPD_200G_BRCM_FEC_272_CR4_KR4;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
                break;
            case 400000:
                if (fec_type == phymod_fec_RS544_2XN) {
                    *spd_intf = TSCOMOD_SPD_400G_IEEE_KR4_CR4;
                } else if (fec_type == phymod_fec_RS272_2XN) {
                    *spd_intf = TSCOMOD_SPD_400G_BRCM_FEC_272_KR4_CR4;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
                break;
            default:
                return PHYMOD_E_UNAVAIL;
        }
    } else if (num_lane == 8) {
        if (data_rate == 400000 && fec_type == phymod_fec_RS544_2XN) {
            *spd_intf = TSCOMOD_SPD_400G_BRCM_FEC_KR8_CR8;
        } else if (data_rate == 400000 && fec_type == phymod_fec_RS272_2XN) {
            *spd_intf = TSCOMOD_SPD_400G_BRCM_FEC_272_N8;
        } else if (data_rate == 200000 && fec_type == phymod_fec_RS544_2XN) {
            *spd_intf = TSCOMOD_SPD_200G_BRCM_FEC_544_CR8_KR8;
        } else {
            return PHYMOD_E_UNAVAIL;
        }
    } else {
        return PHYMOD_E_UNAVAIL;
    }

    return PHYMOD_E_NONE;
}

STATIC
int _tsco_per_lane_data_rate_get(uint32_t vco_rate, int osr_mode, int is_pam4, uint32_t *data_rate)
{
    switch (vco_rate) {
        case 41250000:
            if (osr_mode == TSCOMOD_OS_MODE_2) {
            /* NRZ speed, if OSR is by 2 */
                *data_rate = 20000;
            } else if (osr_mode == TSCOMOD_OS_MODE_4) {
            /*OSR  by 4 */
                *data_rate = 10000;
            } else {
                return PHYMOD_E_UNAVAIL;
            }
            break;
        case 51562500:
            if (is_pam4) {
                if (osr_mode == TSCOMOD_OS_MODE_1) {
                /* PAM4 speed 100G per lane */
                    *data_rate = 100000;
                } else if (osr_mode == TSCOMOD_OS_MODE_2) {
                /* PAM4 speed 50G per lane */
                    *data_rate = 50000;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
           } else {
                /* NRZ speed */
                if (osr_mode == TSCOMOD_OS_MODE_2) {
                    *data_rate = 25000;
                } else if (osr_mode == TSCOMOD_OS_MODE_5) {
                    *data_rate = 10000;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
            }
            break;
        case 53125000:
            if (is_pam4) {
                /* PAM4 speed */
                if (osr_mode == TSCOMOD_OS_MODE_1) {
                /* PAM4 speed 100G per lane */
                    *data_rate = 100000;
                } else if (osr_mode == TSCOMOD_OS_MODE_2) {
                /* PAM4 speed 50G per lane */
                    *data_rate = 50000;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
            } else {
                /* NRZ speed */
                if (osr_mode == TSCOMOD_OS_MODE_2) {
                    *data_rate = 25000;
                } else {
                    return PHYMOD_E_UNAVAIL;
                }
            }
            break;
        default:
            return PHYMOD_E_UNAVAIL;
    }
    return PHYMOD_E_NONE;
}

STATIC
int _tsco_speed_table_entry_to_speed_config_get(const phymod_phy_access_t *phy,
                                        tscomod_spd_id_tbl_entry_t *speed_config_entry,
                                        phymod_phy_speed_config_t *speed_config)
{
    uint32_t pll_div, refclk_in_hz, data_rate_lane = 0;
    int osr_mode = 0;
    tscomod_refclk_t ref_clk;
    uint32_t vco_rate = 41250000;
    phymod_phy_access_t phy_copy;
    phymod_firmware_lane_config_t firmware_lane_config = {0};
    int num_lane;

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    switch (speed_config_entry->num_lanes) {
        case 0: num_lane = 1;
            break;
        case 1: num_lane = 2;
            break;
        case 2: num_lane = 4;
            break;
        case 3: num_lane = 8;
            break;
        case 4: num_lane = 3;
            break;
        case 5: num_lane = 6;
            break;
        case 6: num_lane = 7;
            break;
        default:
            PHYMOD_DEBUG_ERROR(("Unsupported number of lane \n"));
            return PHYMOD_E_UNAVAIL;
    }

    phy_copy.access.pll_idx = 0;
    PHYMOD_MEMSET(&firmware_lane_config,0,sizeof(firmware_lane_config));
    PHYMOD_IF_ERR_RETURN
        (tsco_phy_firmware_lane_config_get(phy, &firmware_lane_config));

    /* get the PLL div from HW */
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_INTERNAL_read_pll_div(&phy_copy.access, &pll_div));

    PHYMOD_IF_ERR_RETURN
        (tscomod_refclk_get(&phy_copy.access, &ref_clk));

    if (ref_clk == TSCOMOD_REF_CLK_312P5MHZ) {
        refclk_in_hz = 312500000;
    } else {
        refclk_in_hz = 156250000;
    }
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_get_vco_from_refclk_div(&phy_copy.access, refclk_in_hz, pll_div, &vco_rate, 0));

    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_osr_mode_get(&phy_copy.access, &osr_mode));

    PHYMOD_IF_ERR_RETURN
        (_tsco_per_lane_data_rate_get(vco_rate, osr_mode, firmware_lane_config.ForcePAM4Mode, &data_rate_lane));

    speed_config->data_rate = data_rate_lane * num_lane;

    PHYMOD_IF_ERR_RETURN
        (tscomod_fec_arch_decode_get(speed_config_entry->fec_arch, &(speed_config->fec_type)));

    return PHYMOD_E_NONE;
}



int tsco_phy_speed_config_set(const phymod_phy_access_t* phy,
                              const phymod_phy_speed_config_t* speed_config,
                              const phymod_phy_pll_state_t* old_pll_state,
                              phymod_phy_pll_state_t* new_pll_state)
{
    phymod_phy_access_t pm_phy_copy;
    uint32_t lane_mask_backup;
    uint32_t tvco_pll_div = TSCOMOD_PLL_MODE_DIV_170, request_pll_div = 0;
    int i, start_lane, num_lane, mapped_speed_id;
    phymod_firmware_lane_config_t firmware_lane_config;
    phymod_firmware_core_config_t firmware_core_config;
    tscomod_spd_intfc_type_t spd_intf = 0;
    tscomod_refclk_t ref_clk;
    int port_enable;
    uint32_t osr5_is_required = 0;

    firmware_lane_config = speed_config->pmd_lane_config;
    /*first make sure that tvco pll index is valid */
    if (phy->access.tvco_pll_index > 0) {
        PHYMOD_DEBUG_ERROR(("Unsupported tvco index\n"));
        return PHYMOD_E_UNAVAIL;
    }

    PHYMOD_MEMSET(&firmware_core_config, 0x0, sizeof(firmware_core_config));

    /* Copy the PLL state */
    *new_pll_state = *old_pll_state;

    PHYMOD_MEMCPY(&pm_phy_copy, phy, sizeof(pm_phy_copy));
    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));
    lane_mask_backup = phy->access.lane_mask;

    /* get current port enable bit */
    PHYMOD_IF_ERR_RETURN
        (tscomod_port_enable_get(&pm_phy_copy.access, &port_enable));

    /* then clear the port an mode enable bit */
    PHYMOD_IF_ERR_RETURN
        (tscomod_port_an_mode_enable_set(&pm_phy_copy.access, 0));

    /* Hold the pcs lane reset */
    pm_phy_copy.access.lane_mask = 1 << start_lane;
    PHYMOD_IF_ERR_RETURN
        (tscomod_disable_set(&pm_phy_copy.access));

    /* write this port forced speed id entry */
    PHYMOD_IF_ERR_RETURN
      (tscomod_set_sc_speed(&pm_phy_copy.access, TSCOMOD_FORCED_SPEED_ID_OFFSET + start_lane, 0));

    /*Hold the per lane PMD soft reset bit*/
    pm_phy_copy.access.lane_mask = lane_mask_backup;
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_lane_soft_reset(&pm_phy_copy.access, 1));

    /* update the port mode */
    PHYMOD_IF_ERR_RETURN
        (tscomod_update_port_mode(&pm_phy_copy.access, speed_config->data_rate));

    /*for speed mode config set */
    PHYMOD_IF_ERR_RETURN(_tsco_phy_speed_id_set(num_lane, speed_config->data_rate,
                                                 speed_config->fec_type, &spd_intf));

    PHYMOD_IF_ERR_RETURN
        (tscomod_get_mapped_speed(spd_intf, &mapped_speed_id));

    /* set the rs fec CW properly */
    if ((speed_config->fec_type == phymod_fec_RS544) ||
        (speed_config->fec_type == phymod_fec_RS544_2XN)) {
        if (start_lane < 2) {
            pm_phy_copy.access.lane_mask = 1 << 0;
        } else if (start_lane < 4)  {
            pm_phy_copy.access.lane_mask = 1 << 2;
        } else if (start_lane < 6)  {
            pm_phy_copy.access.lane_mask = 1 << 4;
        } else {
            pm_phy_copy.access.lane_mask = 1 << 6;
        }
        PHYMOD_IF_ERR_RETURN
            (tscomod_rsfec_cw_type_set(&pm_phy_copy.access, TSCOMOD_RS_FEC_CW_TYPE_544, 0));

        /* next check if 8 lane port, need to set MPP1 as well */
        if (num_lane == 8) {
            pm_phy_copy.access.lane_mask = 1 << 2;
            PHYMOD_IF_ERR_RETURN
                (tscomod_rsfec_cw_type_set(&pm_phy_copy.access, TSCOMOD_RS_FEC_CW_TYPE_544, 0));
        }

    } else if ((speed_config->fec_type == phymod_fec_RS272) ||
               (speed_config->fec_type == phymod_fec_RS272_2XN)) {
        if (start_lane < 2) {
            pm_phy_copy.access.lane_mask = 1 << 0;
        } else if (start_lane < 4)  {
            pm_phy_copy.access.lane_mask = 1 << 2;
        } else if (start_lane < 6)  {
            pm_phy_copy.access.lane_mask = 1 << 4;
        } else {
            pm_phy_copy.access.lane_mask = 1 << 6;
        }
        PHYMOD_IF_ERR_RETURN
            (tscomod_rsfec_cw_type_set(&pm_phy_copy.access, TSCOMOD_RS_FEC_CW_TYPE_272, 0));
        /* next check if 8 lane port, need to set MPP1 as well */
        if (num_lane == 8) {
            pm_phy_copy.access.lane_mask = 1 << 2;
            PHYMOD_IF_ERR_RETURN
                (tscomod_rsfec_cw_type_set(&pm_phy_copy.access, TSCOMOD_RS_FEC_CW_TYPE_272, 0));
        }
    }

    /* Get TVCO because it's not allowed to change during speed set */
    pm_phy_copy.access.pll_idx = 0;
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_INTERNAL_read_pll_div(&pm_phy_copy.access, &tvco_pll_div));

    /* based on the current TVCO PLL div, decide which copy of speed id entry to load */
    pm_phy_copy.access.lane_mask = 1 << 0;

    if (tvco_pll_div == (uint32_t) TSCOMOD_PLL_MODE_DIV_170)  {
        /* then load 53G TVCO speed id entry */
        PHYMOD_IF_ERR_RETURN
            (phymod_mem_write(&pm_phy_copy.access, phymodMemSpeedIdTable, TSCOMOD_FORCED_SPEED_ID_OFFSET + start_lane, &tsco_spd_id_entry_53[mapped_speed_id][0]));
    } else if ((tvco_pll_div == (uint32_t) TSCOMOD_PLL_MODE_DIV_165)) {
        /* then load 51G TVCO speed id entry */
        PHYMOD_IF_ERR_RETURN
            (phymod_mem_write(&pm_phy_copy.access, phymodMemSpeedIdTable, TSCOMOD_FORCED_SPEED_ID_OFFSET + start_lane, &tsco_spd_id_entry_51[mapped_speed_id][0]));
    } else {
        /* then load 41G TVCO speed id entry */
        PHYMOD_IF_ERR_RETURN
            (phymod_mem_write(&pm_phy_copy.access, phymodMemSpeedIdTable, TSCOMOD_FORCED_SPEED_ID_OFFSET + start_lane, &tsco_spd_id_entry_41[mapped_speed_id][0]));
    }

    /* Check the request speed VCO */
    pm_phy_copy.access.lane_mask = 1 << start_lane;
    PHYMOD_IF_ERR_RETURN
        (tscomod_refclk_get(&pm_phy_copy.access, &ref_clk));

    /* Get requested PLL */
    /*for ethernet speed mode config set */
    PHYMOD_IF_ERR_RETURN
        (tscomod_plldiv_lkup_get(&pm_phy_copy.access, mapped_speed_id, ref_clk, &request_pll_div));

    /* for the 10G single lane and 40G 4 lane case, either 41G or 51G VCO can work */
    /* first check if requested 40G VCO is present in the current PLL config, if NOT, need
     to try 51G VCO present or not */
    if (request_pll_div != tvco_pll_div) {
        if  ((int)tvco_pll_div == TSCOMOD_PLL_MODE_DIV_165) {
            request_pll_div = tvco_pll_div;
            osr5_is_required = 1;
        }
    }

    pm_phy_copy.access.lane_mask = lane_mask_backup;
    /* Program OS mode */
    /* need to check if osr5 is required */
    if (osr5_is_required) {
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_osr_mode_set(&pm_phy_copy.access, TSCOMOD_OS_MODE_5));
    } else {
        PHYMOD_IF_ERR_RETURN
            (tscomod_pmd_osmode_set(&pm_phy_copy.access, mapped_speed_id, ref_clk));
    }

    /*next need to set certain firmware lane config to be zero*/
    firmware_lane_config.LaneConfigFromPCS = 0;
    firmware_lane_config.AnEnabled = 0;

   for (i = 0; i < num_lane; i++) {
        pm_phy_copy.access.lane_mask = 0x1 << (start_lane + i);
        if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
            continue;
        }
        PHYMOD_IF_ERR_RETURN
             (_tsco_phy_firmware_lane_config_set(&pm_phy_copy, firmware_lane_config));
    }

    /* next need to enable/disable link training based on the input */
    if (speed_config->linkTraining) {
        PHYMOD_IF_ERR_RETURN
            (tsco_phy_cl72_set(phy, speed_config->linkTraining));
    } else {
        /* disable cl72 and avoid overwriting the value from above _firmware_lane_config_set */
        for (i = 0; i < num_lane; i++) {
            pm_phy_copy.access.lane_mask = 1 << (start_lane + i);
            if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
                continue;
            }
            PHYMOD_IF_ERR_RETURN
                (osprey7_v2l8p1_clause72_control(&pm_phy_copy.access, speed_config->linkTraining));
        }
    }

    /*release the lane soft reset bit*/
    PHYMOD_MEMCPY(&pm_phy_copy, phy, sizeof(pm_phy_copy));
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_lane_soft_reset(&pm_phy_copy.access, 0));

    /* Release the pcs lane reset */
    if (port_enable) {
        pm_phy_copy.access.lane_mask = 1 << start_lane;
        PHYMOD_IF_ERR_RETURN
            (tscomod_enable_set(&pm_phy_copy.access));
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_speed_config_get(const phymod_phy_access_t* phy,
                              phymod_phy_speed_config_t* speed_config)
{
    uint32_t cl72_enable = 0;
    phymod_phy_access_t phy_copy;
    phymod_firmware_lane_config_t firmware_lane_config = {0};
    int start_lane, num_lane, speed_id;
    uint32_t packed_entry[20];
    tscomod_spd_id_tbl_entry_t speed_config_entry;
    int an_en, an_done /*, osr_mode */;

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));
    PHYMOD_MEMSET(&firmware_lane_config,0,sizeof(firmware_lane_config));
    PHYMOD_IF_ERR_RETURN
        (tsco_phy_firmware_lane_config_get(phy, &firmware_lane_config));

    speed_config->pmd_lane_config.AnEnabled          = firmware_lane_config.AnEnabled;
    speed_config->pmd_lane_config.Cl72AutoPolEn      = firmware_lane_config.Cl72AutoPolEn;
    speed_config->pmd_lane_config.Cl72RestTO         = firmware_lane_config.Cl72RestTO;
    speed_config->pmd_lane_config.DfeOn              = firmware_lane_config.DfeOn;
    speed_config->pmd_lane_config.ForceBrDfe         = firmware_lane_config.ForceBrDfe;
    speed_config->pmd_lane_config.ForceExtenedReach  = firmware_lane_config.ForceExtenedReach;
    speed_config->pmd_lane_config.ForceNormalReach   = firmware_lane_config.ForceNormalReach;
    speed_config->pmd_lane_config.ForceNRZMode       = firmware_lane_config.ForceNRZMode;
    speed_config->pmd_lane_config.ForcePAM4Mode      = firmware_lane_config.ForcePAM4Mode;
    speed_config->pmd_lane_config.LaneConfigFromPCS  = firmware_lane_config.LaneConfigFromPCS;
    speed_config->pmd_lane_config.LpDfeOn            = firmware_lane_config.LpDfeOn;
    speed_config->pmd_lane_config.LpPrecoderEnabled  = firmware_lane_config.LpPrecoderEnabled;
    speed_config->pmd_lane_config.MediaType          = firmware_lane_config.MediaType;
    speed_config->pmd_lane_config.ScramblingDisable  = firmware_lane_config.ScramblingDisable;
    speed_config->pmd_lane_config.UnreliableLos      = firmware_lane_config.UnreliableLos;

    /* for ethernet port */
    if (!PHYMOD_DEVICE_OP_MODE_PCS_BYPASS_GET(phy->device_op_mode)) {
        /* first read speed id from resolved status */
        PHYMOD_IF_ERR_RETURN
            (tscomod_speed_id_get(&phy_copy.access, &speed_id));

        /* next check check if AN enabled */
        PHYMOD_IF_ERR_RETURN
           (tscomod_autoneg_status_get(&phy_copy.access, &an_en, &an_done));

        /* first read the speed entry and then decode the speed and FEC type */
        phy_copy.access.lane_mask = 1 << 0;
        PHYMOD_IF_ERR_RETURN
            (phymod_mem_read(&phy_copy.access, phymodMemSpeedIdTable, speed_id, packed_entry));

        /*decode speed entry */
        tscomod_spd_ctrl_unpack_spd_id_tbl_entry(packed_entry, &speed_config_entry);


        PHYMOD_IF_ERR_RETURN
            (_tsco_speed_table_entry_to_speed_config_get(phy, &speed_config_entry, speed_config));

        /* if autoneg enabled, needs to update the FEC_ARCH based on the An resolved status */
        if (an_en && an_done) {
            uint8_t fec_arch;
            phy_copy.access.lane_mask = 0x1 << start_lane;
            PHYMOD_IF_ERR_RETURN
                (tscomod_autoneg_fec_status_get(&phy_copy.access, &fec_arch));
            PHYMOD_IF_ERR_RETURN
                (tscomod_fec_arch_decode_get(fec_arch, &(speed_config->fec_type)));
        }
    }

    /* next get the cl72 enable status */
    phy_copy.access.lane_mask = 0x1 << start_lane;
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_clause72_control_get(&phy_copy.access, &cl72_enable));
    speed_config->linkTraining = cl72_enable;

    return PHYMOD_E_NONE;
}

int tsco_phy_cl72_set(const phymod_phy_access_t* phy, uint32_t cl72_en)
{
    struct osprey7_v2l8p1_uc_lane_config_st serdes_firmware_config;
    phymod_firmware_lane_config_t firmware_lane_config;
    int start_lane, num_lane, i, precoder_en;
    uint32_t lane_reset, pcs_lane_enable;
    phymod_phy_access_t pm_phy_copy;

    PHYMOD_MEMCPY(&pm_phy_copy, phy, sizeof(pm_phy_copy));
    PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_get_uc_lane_cfg(&pm_phy_copy.access, &serdes_firmware_config));

    if ((serdes_firmware_config.field.dfe_on == 0) && cl72_en) {
      PHYMOD_DEBUG_ERROR(("ERROR :: DFE is off : Can not start CL72/CL93 with no DFE\n"));
      return PHYMOD_E_CONFIG;
    }

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));

    /*first check if lane is in reset */
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_lane_soft_reset_get(&pm_phy_copy.access, &lane_reset));

    /*next check if PCS lane is in reset */
    PHYMOD_IF_ERR_RETURN
        (tscomod_enable_get(&pm_phy_copy.access, &pcs_lane_enable));

    /* disable pcs lane if pcs lane not in rset */
    if (pcs_lane_enable) {
        pm_phy_copy.access.lane_mask = 1 << start_lane;
        PHYMOD_IF_ERR_RETURN
            (tscomod_disable_set(&pm_phy_copy.access));
    }

    /* if lane is not in reset, then reset the lane first */
    if (!lane_reset) {
        PHYMOD_MEMCPY(&pm_phy_copy, phy, sizeof(pm_phy_copy));
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_lane_soft_reset(&pm_phy_copy.access, 1));
    }

    /* next need to clear both force ER and NR config on the firmware lane config side
    if link training enable is set */
    if (cl72_en) {
        PHYMOD_IF_ERR_RETURN
            (tsco_phy_firmware_lane_config_get(phy, &firmware_lane_config));

        firmware_lane_config.ForceNormalReach = 0;
        firmware_lane_config.ForceExtenedReach = 0;

         PHYMOD_IF_ERR_RETURN
            (_tsco_phy_firmware_lane_config_set(phy, firmware_lane_config));
    } else {
        /* disable Tx pre-coding and set Rx in NR mode */
        for (i = 0; i < num_lane; i++) {
            pm_phy_copy.access.lane_mask = 1 << (start_lane + i);
            precoder_en = 0;
            PHYMOD_IF_ERR_RETURN
                (tsco_phy_tx_pam4_precoder_enable_get(&pm_phy_copy, &precoder_en));
            if (precoder_en) {
                PHYMOD_IF_ERR_RETURN
                    (tsco_phy_tx_pam4_precoder_enable_set(&pm_phy_copy, 0));
            }
            PHYMOD_IF_ERR_RETURN
                (tsco_phy_firmware_lane_config_get(&pm_phy_copy, &firmware_lane_config));
            if (firmware_lane_config.ForcePAM4Mode) {
                firmware_lane_config.ForceNormalReach = 1;
                firmware_lane_config.ForceExtenedReach = 0;
                PHYMOD_IF_ERR_RETURN
                    (_tsco_phy_firmware_lane_config_set(&pm_phy_copy, firmware_lane_config));
            }
        }
    }
    for (i = 0; i < num_lane; i++) {
        pm_phy_copy.access.lane_mask = 1 << (start_lane + i);
        if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
            continue;
        }
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_clause72_control(&pm_phy_copy.access, cl72_en));
    }

    /* release the ln dp reset */
    if (!lane_reset) {
        PHYMOD_MEMCPY(&pm_phy_copy, phy, sizeof(pm_phy_copy));
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_lane_soft_reset(&pm_phy_copy.access, 0));
    }

    /* re-enable pcs lane if pcs lane not in rset */
    if (pcs_lane_enable) {
        pm_phy_copy.access.lane_mask = 1 << start_lane;
        PHYMOD_IF_ERR_RETURN
            (tscomod_enable_set(&pm_phy_copy.access));
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_cl72_get(const phymod_phy_access_t* phy, uint32_t* cl72_en)
{
    phymod_phy_access_t pm_phy_copy;

    PHYMOD_MEMCPY(&pm_phy_copy, phy, sizeof(pm_phy_copy));

    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_clause72_control_get(&pm_phy_copy.access, cl72_en));

    return PHYMOD_E_NONE;
}

int tsco_phy_cl72_status_get(const phymod_phy_access_t* phy, phymod_cl72_status_t* status)
{
    int i;
    uint32_t tmp_status;
    int start_lane, num_lane;
    phymod_phy_access_t phy_copy;
    int an_en, an_done, speed_id;
    tscomod_spd_id_tbl_entry_t speed_config_entry;
    uint32_t packed_entry[5];

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
    status->locked = 1;

    /* next figure out the lane num and start_lane based on the input */
    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));
    /* check check if AN enabled */
    PHYMOD_IF_ERR_RETURN
       (tscomod_autoneg_status_get(&phy_copy.access, &an_en, &an_done));

    if (an_en && an_done) {
        PHYMOD_IF_ERR_RETURN
            (tscomod_speed_id_get(&phy_copy.access, &speed_id));
        PHYMOD_IF_ERR_RETURN
            (phymod_mem_read(&phy_copy.access, phymodMemSpeedIdTable, speed_id, packed_entry));
        tscomod_spd_ctrl_unpack_spd_id_tbl_entry(packed_entry, &speed_config_entry);
        /* Update num_lane for AN port */
        num_lane = 1 << speed_config_entry.num_lanes;
    }

    for (i = 0; i < num_lane; i++) {
        if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
            continue;
        }
        phy_copy.access.lane_mask = 0x1 << (i + start_lane);
        tmp_status = 1;
        PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_pmd_cl72_receiver_status(&phy_copy.access, &tmp_status));
        if (tmp_status == 0) {
            status->locked = 0;
            return PHYMOD_E_NONE;
        }
    }
    return PHYMOD_E_NONE;
}

int tsco_phy_loopback_set(const phymod_phy_access_t* phy,
                          phymod_loopback_mode_t loopback, uint32_t enable)
{
    int i;
    int start_lane, num_lane;
    phymod_phy_access_t phy_copy;

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    /* next figure out the lane num and start_lane based on the input */
    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));

    switch (loopback) {
    case phymodLoopbackGlobal :
    case phymodLoopbackGlobalPMD :
        if (enable) {
            phy_copy.access.lane_mask = 1 << start_lane;
            PHYMOD_IF_ERR_RETURN
                (tscomod_disable_set(&phy_copy.access));
            /*first squelch rx */
            for (i = 0; i < num_lane; i++) {
                if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
                    continue;
                }
                phy_copy.access.lane_mask = 0x1 << (i + start_lane);
                PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_pmd_force_signal_detect(&phy_copy.access,  (int) enable, (int) 0));
            }
            phy_copy.access.lane_mask = 1 << start_lane;
            PHYMOD_IF_ERR_RETURN
                (tscomod_enable_set(&phy_copy.access));
        }
        for (i = 0; i < num_lane; i++) {
            if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
                continue;
            }
            phy_copy.access.lane_mask = 0x1 << (i + start_lane);
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_tx_disable(&phy_copy.access, enable));
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_dig_lpbk(&phy_copy.access, (uint8_t) enable));
        }
        if (!enable) {
            for (i = 0; i < num_lane; i++) {
                if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
                    continue;
                }
                phy_copy.access.lane_mask = 0x1 << (i + start_lane);
                PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_pmd_force_signal_detect(&phy_copy.access,  (int) enable, (int) 0));
            }
        }
        break;
    case phymodLoopbackRemotePMD :
        for (i = 0; i < num_lane; i++) {
            if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
                continue;
            }
            phy_copy.access.lane_mask = 0x1 << (i + start_lane);
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_rmt_lpbk(&phy_copy.access, (uint8_t)enable));
        }
        break;
    case phymodLoopbackRemotePCS :
    default :
        PHYMOD_RETURN_WITH_ERR(PHYMOD_E_UNAVAIL,
                               (_PHYMOD_MSG("This mode is not supported\n")));
        break;
    }
    return PHYMOD_E_NONE;
}

int tsco_phy_loopback_get(const phymod_phy_access_t* phy,
                          phymod_loopback_mode_t loopback, uint32_t* enable)
{
    int start_lane, num_lane;
    phymod_phy_access_t phy_copy;

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    /*next figure out the lane num and start_lane based on the input*/
    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));

    switch (loopback) {
    case phymodLoopbackGlobal :
    case phymodLoopbackGlobalPMD :
        PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_dig_lpbk_get(&phy_copy.access, enable));
        break;
    case phymodLoopbackRemotePMD :
        PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_rmt_lpbk_get(&phy_copy.access, enable));
        break;
    case phymodLoopbackRemotePCS :
    default :
        return PHYMOD_E_UNAVAIL;
    }
    return PHYMOD_E_NONE;
}

/* Core initialization
 * (PASS1)
 * 1.  De-assert PMD core and PMD lane reset
 * 2.  Set heartbeat for comclk
 * 3.  Micro code load and verify
 * (PASS2)
 * 4.  Configure PMD lane mapping and PCS lane swap
 * 5.  De-assert micro reset
 * 6.  Wait for uc_active = 1
 * 7.  Initialize software information table for the micro
 * 8.  Config PMD polarity
 * 9. AFE/PLL configuration
 * 10. Set core_from_pcs_config
 * 11. Program AN default timer
 * 12. Load sd_id_table, am_table and um_table into TSC memory
 * 13. Release core DP soft reset
 */
STATIC
int _tsco_core_init_pass1(const phymod_core_access_t* core, const phymod_core_init_config_t* init_config, const phymod_core_status_t* core_status)
{
    int  rv, lane;
    phymod_phy_access_t phy_access;
    phymod_core_access_t  core_copy;
    /* need to fix this later */
    uint32_t uc_enable = 0;
    ucode_info_t ucode;
    unsigned int osprey7_ucode_len;
    unsigned char *osprey7_ucode;

    TSCOMOD_CORE_TO_PHY_ACCESS(&phy_access, core);
    PHYMOD_MEMCPY(&core_copy, core, sizeof(core_copy));
    core_copy.access.lane_mask = 0x1;

    /* 1. De-assert PMD core power and core data path reset */
    PHYMOD_IF_ERR_RETURN
        (tscomod_pmd_reset_seq(&core_copy.access));

    core_copy.access.pll_idx = 0;
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_core_dp_reset(&core_copy.access, 1));

    /* De-assert PMD lane reset */
    
    for (lane = 0; lane < TSCOMOD_NOF_LANES_IN_CORE; lane++) {
        phy_access.access.lane_mask = 1 << lane;
        PHYMOD_IF_ERR_RETURN
          (tscomod_pmd_x4_reset(&phy_access.access));
    }

    if (!PHYMOD_CORE_INIT_F_SERDES_FW_BCAST_GET(init_config)) {
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_uc_active_get(&phy_access.access, &uc_enable));
        if (uc_enable) return PHYMOD_E_NONE;
    }

    /* 2. Set the heart beat COM Clock, default is 312.5M */
    if (init_config->interface.ref_clock == phymodRefClk312Mhz) {
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_comclk_set(&core_copy.access, init_config->interface.ref_clock));
    } else {
        PHYMOD_DEBUG_ERROR(("Unsupported reference clock.\n"));
        return PHYMOD_E_UNAVAIL;
    }

    /* set the micro stack size */
    phy_access.access.lane_mask = 0x1;
    ucode.stack_size = OSPREY7_V2L8P1_UCODE_STACK_SIZE;
    ucode.ucode_size = OSPREY7_V2L8P1_UCODE_IMAGE_SIZE;
    osprey7_ucode_len = OSPREY7_V2L8P1_UCODE_IMAGE_SIZE;
    osprey7_ucode = osprey7_v2l8p1_ucode_get();

    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_uc_reset_with_info(&core_copy.access , 1, ucode));

    /* 3. Micro code load and verify */
    rv = _tsco_core_firmware_load(&core_copy, init_config);
    if (rv != PHYMOD_E_NONE) {
        PHYMOD_DEBUG_ERROR(("devad 0x%"PRIx32" lane 0x%"PRIx32": UC firmware-load failed\n", core->access.addr, core->access.lane_mask));
        PHYMOD_IF_ERR_RETURN(rv);
    }

     /* need to check if the ucode load is correct or not */
    if (init_config->firmware_load_method != phymodFirmwareLoadMethodNone) {
        /* Read-back uCode from Program RAM and verify against ucode_image.
         * The comparison is 4-byte at a time, which is time-consuming.
         * Thus, this code is only needed for debug purpose.
         */
        if (PHYMOD_CORE_INIT_F_FIRMWARE_LOAD_VERIFY_GET(init_config)) {
            rv = osprey7_v2l8p1_ucode_load_verify(&core_copy.access, (uint8_t *) osprey7_ucode, osprey7_ucode_len);
            if (rv != PHYMOD_E_NONE) {
                PHYMOD_DEBUG_ERROR(("devad 0x%x lane 0x%x: UC load-verify failed\n", core->access.addr, core->access.lane_mask));
                PHYMOD_IF_ERR_RETURN(rv);
            }
        }
    }

    return PHYMOD_E_NONE;
}


STATIC
int _tsco_core_init_pass2(const phymod_core_access_t* core, const phymod_core_init_config_t* init_config, const phymod_core_status_t* core_status)
{
    phymod_phy_access_t phy_access, phy_access_copy;
    phymod_core_access_t  core_copy;
    enum osprey7_v2l8p1_pll_refclk_enum refclk;
    phymod_polarity_t tmp_pol;
    int lane, pmd_config_bypass = 0;
    uint32_t am_table_load_size, um_table_load_size;
    uint32_t i, fclk_div_mode = 1;

    TSCOMOD_CORE_TO_PHY_ACCESS(&phy_access, core);
    phy_access_copy = phy_access;
    PHYMOD_MEMCPY(&core_copy, core, sizeof(core_copy));

    /* first check if PMD config needs to be skipped */
    if (PHYMOD_CORE_INIT_F_BYPASS_PMD_CONFIGURATION_GET(init_config)) {
        pmd_config_bypass = 1;
        core_copy.access.lane_mask = 0x10;
        phy_access_copy = phy_access;
        phy_access_copy.access = core->access;
        phy_access_copy.access.lane_mask = 0x10;
        phy_access_copy.type = core->type;
    } else {
        core_copy.access.lane_mask = 0x1;
        phy_access_copy = phy_access;
        phy_access_copy.access = core->access;
        phy_access_copy.access.lane_mask = 0x1;
        phy_access_copy.type = core->type;
    }

    PHYMOD_MEMSET(&tmp_pol, 0x0, sizeof(tmp_pol));

    /* 4. Configure PMD lane mapping and PCS lane swap */
    PHYMOD_IF_ERR_RETURN
        (tsco_core_lane_map_set(&core_copy, &init_config->lane_map));

    /* need to set PML0 tx/rx mux selection */
    if (!pmd_config_bypass) {
        PHYMOD_IF_ERR_RETURN
           ( _tsco_core_tx_rx_mux_select_set(&core_copy, &init_config->lane_map));
    }

    if (init_config->interface.ref_clock == phymodRefClk312Mhz) {
        refclk = OSPREY7_V2L8P1_PLL_REFCLK_312P5MHZ;
        PHYMOD_IF_ERR_RETURN
            (tscomod_refclk_set(&phy_access.access, TSCOMOD_REF_CLK_312P5MHZ));
    } else {
        PHYMOD_DEBUG_ERROR(("Unsupported reference clock.\n"));
        return PHYMOD_E_UNAVAIL;
    }

    /*next need to load UM table and AM table */
    am_table_load_size = TSCOMOD_AM_TABLE_SIZE > TSCOMOD_HW_AM_TABLE_SIZE ? TSCOMOD_HW_AM_TABLE_SIZE : TSCOMOD_AM_TABLE_SIZE;
    um_table_load_size = TSCOMOD_UM_TABLE_SIZE > TSCOMOD_HW_UM_TABLE_SIZE ? TSCOMOD_HW_UM_TABLE_SIZE : TSCOMOD_UM_TABLE_SIZE;

    for (i = 0; i < am_table_load_size; i++) {
        PHYMOD_IF_ERR_RETURN
            (phymod_mem_write(&core_copy.access, phymodMemAMTable, i,  &tsco_am_table_entry[i][0]));
    }

    for (i = 0; i < um_table_load_size; i++) {
        PHYMOD_IF_ERR_RETURN
            (phymod_mem_write(&core_copy.access, phymodMemUMTable, i,  &tsco_um_table_entry[i][0]));
    }

    /*need to update speed_priority_mapping_table with correct speed id */
    PHYMOD_IF_ERR_RETURN
        (phymod_mem_write(&core_copy.access, phymodMemSpeedPriorityMapTable, 0,  &tsco_speed_priority_mapping_table[0][0]));

    if ((init_config->firmware_load_method != phymodFirmwareLoadMethodNone) && (!pmd_config_bypass)) {
        ucode_info_t ucode;
        ucode.stack_size = OSPREY7_V2L8P1_UCODE_STACK_SIZE;
        ucode.ucode_size = OSPREY7_V2L8P1_UCODE_IMAGE_SIZE;
        ucode.crc_value  = OSPREY7_V2L8P1_UCODE_IMAGE_CRC;

        /* 6. Release uc reset */
        PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_uc_reset_with_info(&core_copy.access , 0, ucode));

        /* 7. Wait for uc_active = 1 */
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_wait_uc_active(&phy_access.access));

        for (lane = 0; lane < TSCOMOD_NOF_LANES_IN_CORE; lane++) {
            phy_access_copy.access.lane_mask = 1 << lane;
            PHYMOD_IF_ERR_RETURN
                (osprey7_v2l8p1_pmd_ln_h_rstb_pkill_override(&phy_access_copy.access, 0x1));
        }
        /* we need to wait at least 10ms for the uc to settle */
        PHYMOD_USLEEP(10000);

        /* 8. Initialize software information table for the macro */
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_init_osprey7_v2l8p1_info(&core_copy.access));

        /* release pmd lane hard reset */
        for (lane = 0; lane < TSCOMOD_NOF_LANES_IN_CORE; lane++) {
            phy_access_copy.access.lane_mask = 1 << lane;
            PHYMOD_IF_ERR_RETURN
                (osprey7_v2l8p1_pmd_ln_h_rstb_pkill_override(&phy_access_copy.access, 0x0));
        }
    }

    /* 9. RX/TX polarity configuration */
    if (!pmd_config_bypass) {
        for (lane = 0; lane < TSCOMOD_NOF_LANES_IN_CORE; lane++) {
            phy_access_copy.access.lane_mask = 1 << lane;
            tmp_pol.tx_polarity = (init_config->polarity_map.tx_polarity) >> lane & 0x1;
            tmp_pol.rx_polarity = (init_config->polarity_map.rx_polarity) >> lane & 0x1;
            PHYMOD_IF_ERR_RETURN
                (tsco_phy_polarity_set(&phy_access_copy, &tmp_pol));
            /* clear the tmp vairiable */
            PHYMOD_MEMSET(&tmp_pol, 0x0, sizeof(tmp_pol));
        }
    }

    /* Enable PCS clock block on the PMD side */
    if (!pmd_config_bypass) {
        core_copy.access.pll_idx = 0;
        PHYMOD_IF_ERR_RETURN
            (tscomod_pcs_clk_blk_en(&core_copy.access, 1));
    }

    if (!pmd_config_bypass) {
        if ((int) (init_config->pll0_div_init_value) != phymod_TSCBH_PLL_DIVNONE) {
            core_copy.access.pll_idx = 0;
            PHYMOD_IF_ERR_RETURN
                (osprey7_v2l8p1_configure_pll_refclk_div(&core_copy.access,
                                                          refclk,
                                                          init_config->pll0_div_init_value));
        }
    }

    COMPILER_REFERENCE(fclk_div_mode);
    /* Set FCLK period. */
    
    /* PHYMOD_IF_ERR_RETURN
        (tscomod_set_fclk_period(&core_copy.access, tvco_rate, fclk_div_mode));
     */

    /* 12. Program AN default timer  for both MMP0 and MMP1*/
    /* PML0 config */
    if (!pmd_config_bypass) {
        core_copy.access.lane_mask = 0x1;
        core_copy.access.pll_idx = 0;
        PHYMOD_IF_ERR_RETURN
            (tscomod_set_an_timers(&core_copy.access, init_config->interface.ref_clock, NULL));
        /* enable FEC COBRA as default */
        PHYMOD_IF_ERR_RETURN
            (tscomod_fec_cobra_enable(&core_copy.access, 1));

        core_copy.access.lane_mask = 0x4;
        core_copy.access.pll_idx = 0;
        PHYMOD_IF_ERR_RETURN
            (tscomod_set_an_timers(&core_copy.access, init_config->interface.ref_clock, NULL));
        /* enable FEC COBRA as default */
        PHYMOD_IF_ERR_RETURN
            (tscomod_fec_cobra_enable(&core_copy.access, 1));
    } else {
        /* PML1 config */
        core_copy.access.lane_mask = 0x10;
        core_copy.access.pll_idx = 0;
        PHYMOD_IF_ERR_RETURN
            (tscomod_set_an_timers(&core_copy.access, init_config->interface.ref_clock, NULL));
        /* enable FEC COBRA as default */
        PHYMOD_IF_ERR_RETURN
            (tscomod_fec_cobra_enable(&core_copy.access, 1));

        core_copy.access.lane_mask = 0x40;
        core_copy.access.pll_idx = 0;
        PHYMOD_IF_ERR_RETURN
            (tscomod_set_an_timers(&core_copy.access, init_config->interface.ref_clock, NULL));
        /* enable FEC COBRA as default */
        PHYMOD_IF_ERR_RETURN
            (tscomod_fec_cobra_enable(&core_copy.access, 1));
    }

    /* set the PMD debug level to be 2 as default */
    /* need to re-enable later */
    if (!pmd_config_bypass) {
        for (lane = 0; lane < TSCOMOD_NOF_LANES_IN_CORE; lane++) {
            phy_access_copy.access.lane_mask = 1 << lane;
            PHYMOD_IF_ERR_RETURN
                (osprey7_v2l8p1_set_usr_ctrl_lane_event_log_level(&phy_access_copy.access, 2));
        }
    }

    /* 14. Release core DP soft reset for PLL0 */
    if (!pmd_config_bypass) {
        core_copy.access.lane_mask = 0x1;
        core_copy.access.pll_idx = 0;
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_core_dp_reset(&core_copy.access, 0));
    }

    return PHYMOD_E_NONE;

}


int tsco_core_init(const phymod_core_access_t* core,
                   const phymod_core_init_config_t* init_config,
                   const phymod_core_status_t* core_status)
{
    if ( (!PHYMOD_CORE_INIT_F_EXECUTE_PASS1_GET(init_config) &&
          !PHYMOD_CORE_INIT_F_EXECUTE_PASS2_GET(init_config)) ||
        PHYMOD_CORE_INIT_F_EXECUTE_PASS1_GET(init_config)) {
        PHYMOD_IF_ERR_RETURN
            (_tsco_core_init_pass1(core, init_config, core_status));

        if (PHYMOD_CORE_INIT_F_EXECUTE_PASS1_GET(init_config)) {
            return PHYMOD_E_NONE;
        }
    }

    if ( (!PHYMOD_CORE_INIT_F_EXECUTE_PASS1_GET(init_config) &&
          !PHYMOD_CORE_INIT_F_EXECUTE_PASS2_GET(init_config)) ||
        PHYMOD_CORE_INIT_F_EXECUTE_PASS2_GET(init_config)) {
        PHYMOD_IF_ERR_RETURN
            (_tsco_core_init_pass2(core, init_config, core_status));
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_init(const phymod_phy_access_t* phy, const phymod_phy_init_config_t* init_config)
{
    phymod_phy_access_t pm_phy_copy;
    int start_lane, num_lane, i;
    phymod_firmware_lane_config_t firmware_lane_config;
    uint32_t pll_power_down = 0;

    PHYMOD_MEMCPY(&pm_phy_copy, phy, sizeof(pm_phy_copy));
    PHYMOD_MEMSET(&firmware_lane_config, 0x0, sizeof(firmware_lane_config));

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&pm_phy_copy.access, &start_lane, &num_lane));
    /* per lane based reset release */
    PHYMOD_IF_ERR_RETURN
        (tscomod_pmd_x4_reset(&pm_phy_copy.access));

    /* next check if 8 lane port, if so need to modify PMD tx/rx mux selection */
    if (num_lane == 8) {
        PHYMOD_IF_ERR_RETURN
            (tscomod_pcs_rx_mux_select(&pm_phy_copy.access, 0x0));
        PHYMOD_IF_ERR_RETURN
            (tscomod_pcs_tx_mux_select(&pm_phy_copy.access, 0x0));
    }

    /* Put PMD lane into soft reset */
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_lane_soft_reset(&pm_phy_copy.access, 1));

    
    /* clearing all the lane config */
    PHYMOD_MEMSET(&firmware_lane_config, 0x0, sizeof(firmware_lane_config));

    for (i = 0; i < num_lane; i++) {
        if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
            continue;
        }
        pm_phy_copy.access.lane_mask = 0x1 << (i + start_lane);
        /* set tx parameters */
        PHYMOD_IF_ERR_RETURN
            (tsco_phy_tx_set(&pm_phy_copy, &init_config->tx[i]));
    }

    
    for (i = 0; i < num_lane; i++) {
        if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
            continue;
        }
        pm_phy_copy.access.lane_mask = 0x1 << (i + start_lane);
        PHYMOD_IF_ERR_RETURN
             (_tsco_phy_firmware_lane_config_set(&pm_phy_copy, firmware_lane_config));
        /* FROM PMD team:
         * The best initial analog calibration is achieved using the information
         * from the PMD RX PRBS checker. Using the FW register value of 3 enables
         * a continuous version that does not rely on PRBS traffic anymore.
         */
        
        /* PHYMOD_IF_ERR_RETURN
             (osprey7_v2l8p1_lane_cfg_fwapi_data1_set(&pm_phy_copy.access, 3));
         */
    }

    /* make sure that power up PLL is locked */
    pm_phy_copy.access.pll_idx = 0;
    pm_phy_copy.access.lane_mask = 1 << 0;
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_pll_pwrdn_get(&pm_phy_copy.access, &pll_power_down));

    /* need to check pll0 lock if not power up */
    /* put the check here is to save on boot up time */
    if (!pll_power_down) {
        uint32_t cnt = 0, pll_lock = 0;
        cnt = 0;
        while (cnt < 1000) {
            PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_pll_lock_get(&pm_phy_copy.access, &pll_lock));
            cnt = cnt + 1;
            if (pll_lock) {
                break;
            } else {
                if(cnt == 1000) {
                    PHYMOD_DEBUG_ERROR(("WARNING :: core 0x%x PLL0 is not locked within 5 milli second \n", pm_phy_copy.access.addr));
                    break;
                }
            }
            PHYMOD_USLEEP(10);
        }
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_link_status_get(const phymod_phy_access_t* phy, uint32_t* link_status)
{
    phymod_phy_access_t pm_phy_copy;
    PHYMOD_MEMCPY(&pm_phy_copy, phy, sizeof(pm_phy_copy));

    PHYMOD_IF_ERR_RETURN(tscomod_get_pcs_latched_link_status(&pm_phy_copy.access, link_status));
    return PHYMOD_E_NONE;
}

int tsco_phy_tx_taps_default_get(const phymod_phy_access_t* phy,
                                 phymod_phy_signalling_method_t mode,
                                 phymod_tx_t* tx)
{
    /*always default to 6-taps mode */
    tx->tap_mode = phymodTxTapMode6Tap;
    tx->sig_method = mode;
    if (mode == phymodSignallingMethodNRZ) {
        tx->pre3 = 0;
        tx->pre2 = 0;
        tx->pre = -12;
        tx->main = 88;
        tx->post = -26;
        tx->post2 = 0;
        tx->post3 = 0;
    } else {
        tx->pre3 = 0;
        tx->pre2 = 0;
        tx->pre = -24;
        tx->main = 132;
        tx->post = -12;
        tx->post2 = 0;
        tx->post3 = 0;
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_lane_config_default_get(const phymod_phy_access_t* phy,
                                     phymod_phy_signalling_method_t mode,
                                     phymod_firmware_lane_config_t* lane_config)
{
    /* default always assume backplane as the medium type and with dfe on */
    if (mode == phymodSignallingMethodNRZ) {
        lane_config->ForceNRZMode = 1;
        lane_config->ForcePAM4Mode = 0;
        lane_config->ForceNormalReach  = 0;
    } else {
        lane_config->ForceNRZMode = 0;
        lane_config->ForcePAM4Mode = 1;
        lane_config->ForceNormalReach  = 1;
    }
    lane_config->LaneConfigFromPCS = 0;
    lane_config->AnEnabled = 0;
    lane_config->DfeOn = 1;
    lane_config->LpDfeOn = 0;
    lane_config->ForceBrDfe = 0;
    lane_config->MediaType = 0;
    lane_config->ScramblingDisable = 0;
    lane_config->Cl72AutoPolEn = 0;
    lane_config->Cl72RestTO = 0;
    lane_config->ForceExtenedReach = 0;
    lane_config->LpPrecoderEnabled = 0;
    lane_config->UnreliableLos = 0;

    return PHYMOD_E_NONE;
}

int tsco_phy_reg_read(const phymod_phy_access_t* phy, uint32_t reg_addr, uint32_t* val)
{
    int pcs_num_copy = 0;
    /* first check if PMD register */
    if (reg_addr & 0x10000) {
        PHYMOD_IF_ERR_RETURN(phymod_tscbh_iblk_read(&phy->access, reg_addr, val));
    } else {
        /* need to figure out 1 /2 or 4 copy register */
        PHYMOD_IF_ERR_RETURN
            (tscomod_pcs_reg_num_copy_get(reg_addr, &pcs_num_copy));
        if (pcs_num_copy == 1) {
            PHYMOD_IF_ERR_RETURN(phymod_tsco1_iblk_read(&phy->access, reg_addr, val));
        } else if (pcs_num_copy == 2) {
            PHYMOD_IF_ERR_RETURN(phymod_tsco2_iblk_read(&phy->access, reg_addr, val));
        } else if (pcs_num_copy == 4) {
            PHYMOD_IF_ERR_RETURN(phymod_tsco4_iblk_read(&phy->access, reg_addr, val));
        } else {
            PHYMOD_DEBUG_ERROR(("ERROR :: invalid pcs register address 0x%x \n", reg_addr));
            return PHYMOD_E_UNAVAIL;
        }
    }
    return PHYMOD_E_NONE;
}

int tsco_phy_reg_write(const phymod_phy_access_t* phy, uint32_t reg_addr, uint32_t val)
{
    int pcs_num_copy = 0;
    /* first check if PMD register */
    if (reg_addr & 0x10000) {
        PHYMOD_IF_ERR_RETURN(phymod_tscbh_iblk_write(&phy->access, reg_addr, val));
    } else {
        /* need to figure out 1 /2 or 4 copy register */
        PHYMOD_IF_ERR_RETURN
            (tscomod_pcs_reg_num_copy_get(reg_addr, &pcs_num_copy));
        if (pcs_num_copy == 1) {
            PHYMOD_IF_ERR_RETURN(phymod_tsco1_iblk_write(&phy->access, reg_addr, val));
        } else if (pcs_num_copy == 2) {
            PHYMOD_IF_ERR_RETURN(phymod_tsco2_iblk_write(&phy->access, reg_addr, val));
        } else if (pcs_num_copy == 4) {
            PHYMOD_IF_ERR_RETURN(phymod_tsco4_iblk_write(&phy->access, reg_addr, val));
        } else {
            PHYMOD_DEBUG_ERROR(("ERROR :: invalid pcs register address 0x%x \n", reg_addr));
            return PHYMOD_E_UNAVAIL;
        }
    }
    return PHYMOD_E_NONE;
}

int tsco_phy_firmware_load_info_get(const phymod_phy_access_t* phy, phymod_firmware_load_info_t* info)
{
    return PHYMOD_E_NONE;
}

int tsco_phy_autoneg_advert_ability_set(const phymod_phy_access_t* phy,
                                        const phymod_autoneg_advert_abilities_t* an_advert_abilities,
                                        const phymod_phy_pll_state_t* old_pll_adv_state,
                                        phymod_phy_pll_state_t* new_pll_adv_state)
{
    int start_lane, num_lane;
    phymod_phy_access_t phy_copy;

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
    phy_copy.access.lane_mask = 0x1 << start_lane;

    /* Program local advert abilitiy registers */
    PHYMOD_IF_ERR_RETURN
        (tscomod_autoneg_ability_set(&phy_copy.access, an_advert_abilities));

    return PHYMOD_E_NONE;
}

int tsco_phy_autoneg_advert_ability_get(const phymod_phy_access_t* phy,
                                        phymod_autoneg_advert_abilities_t* an_advert_abilities)
{
    phymod_phy_access_t phy_copy;
    int start_lane, num_lane;
    uint32_t i;
    phymod_firmware_lane_config_t firmware_lane_config;

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));
    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
    phy_copy.access.lane_mask = 0x1 << start_lane;

    PHYMOD_IF_ERR_RETURN
        (tscomod_autoneg_ability_get(&phy_copy.access, an_advert_abilities));

    /* Get Medium type from fw_lane_config */
    PHYMOD_IF_ERR_RETURN
        (tsco_phy_firmware_lane_config_get(phy, &firmware_lane_config));

    for (i = 0; i < an_advert_abilities->num_abilities; i++) {
        an_advert_abilities->autoneg_abilities[i].medium = firmware_lane_config.MediaType;
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_autoneg_remote_advert_ability_get(const phymod_phy_access_t* phy,
                                               phymod_autoneg_advert_abilities_t* an_advert_abilities)
{

    return PHYMOD_E_NONE;
}

int tsco_phy_autoneg_set(const phymod_phy_access_t* phy, const phymod_autoneg_control_t* an)
{
    int num_lane_adv_encoded, mapped_speed_id;
    int start_lane, num_lane;
    int i, do_lane_config_set;
    uint32_t pll_div, vco_rate, refclk_in_hz;
    phymod_firmware_lane_config_t firmware_lane_config;
    tscomod_an_control_t an_control;
    phymod_phy_access_t phy_copy;
    tscomod_refclk_t ref_clk;
    tscomod_spd_intfc_type_t spd_intf = 0;

    PHYMOD_MEMSET(&firmware_lane_config, 0x0, sizeof(firmware_lane_config));

    PHYMOD_MEMSET(&an_control, 0x0, sizeof(an_control));
    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
    /* first set the port an mode enable bit */
    PHYMOD_IF_ERR_RETURN
        (tscomod_port_an_mode_enable_set(&phy_copy.access, (int) an->enable));

    phy_copy.access.lane_mask = 0x1 << start_lane;

    if (an->enable) {
        PHYMOD_IF_ERR_RETURN(tscomod_disable_set(&phy_copy.access));
        /* adjust FEC arch for RS544/RS272 */
        if (PHYMOD_AN_F_FEC_RS272_CLR_GET(an)) {
            if (start_lane < 2) {
                phy_copy.access.lane_mask = 0x1;
            } else if (start_lane < 4) {
                phy_copy.access.lane_mask = 0x4;
            } else if (start_lane < 6) {
                phy_copy.access.lane_mask = 0x10;
            } else if (start_lane < 8) {
                phy_copy.access.lane_mask = 0x40;
            }
            PHYMOD_IF_ERR_RETURN(tscomod_rsfec_cw_type_set(&phy_copy.access, TSCOMOD_RS_FEC_CW_TYPE_544, 0));
            if (num_lane == 8) {
                phy_copy.access.lane_mask = 0x4;
                PHYMOD_IF_ERR_RETURN(tscomod_rsfec_cw_type_set(&phy_copy.access, TSCOMOD_RS_FEC_CW_TYPE_544, 0));
            }
        } else if (PHYMOD_AN_F_FEC_RS272_REQ_GET(an)) {
            if (start_lane < 2) {
                phy_copy.access.lane_mask = 0x1;
            } else if (start_lane < 4) {
                phy_copy.access.lane_mask = 0x4;
            } else if (start_lane < 6) {
                phy_copy.access.lane_mask = 0x10;
            } else if (start_lane < 8) {
                phy_copy.access.lane_mask = 0x40;
            }
            PHYMOD_IF_ERR_RETURN(tscomod_rsfec_cw_type_set(&phy_copy.access, TSCOMOD_RS_FEC_CW_TYPE_272, 0));
            if (num_lane == 8) {
                phy_copy.access.lane_mask = 0x4;
                PHYMOD_IF_ERR_RETURN(tscomod_rsfec_cw_type_set(&phy_copy.access, TSCOMOD_RS_FEC_CW_TYPE_272, 0));
            }
        }

    }

    switch (an->num_lane_adv) {
        case 1:
            num_lane_adv_encoded = 0;
            break;
        case 2:
            num_lane_adv_encoded = 1;
            break;
        case 4:
            num_lane_adv_encoded = 2;
            break;
        case 8:
            num_lane_adv_encoded = 3;
            break;
        default:
            return PHYMOD_E_PARAM;
    }

    an_control.num_lane_adv = num_lane_adv_encoded;
    an_control.enable       = an->enable;
    switch (an->an_mode) {
        case phymod_AN_MODE_CL73:
            an_control.an_type = TSCOMOD_AN_MODE_CL73;
            break;
        case phymod_AN_MODE_CL73BAM:
            an_control.an_type = TSCOMOD_AN_MODE_CL73_BAM;
            break;
        case phymod_AN_MODE_CL73_MSA:
            an_control.an_type = TSCOMOD_AN_MODE_CL73_MSA;
            break;
        default:
            return PHYMOD_E_PARAM;
            break;
    }

    if (an->enable) {
        PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
        /* Set AN port mode */
        PHYMOD_IF_ERR_RETURN
            (tscomod_set_an_port_mode(&phy_copy.access, start_lane));

        /* Get TVCO rate (PLL1 for now) */
        PHYMOD_IF_ERR_RETURN
            (tscomod_refclk_get(&phy_copy.access, &ref_clk));

        if (ref_clk == TSCOMOD_REF_CLK_312P5MHZ) {
            refclk_in_hz = 312500000;
        } else {
            refclk_in_hz = 156250000;
        }

        /* next read current TVCO pll divider*/
        phy_copy.access.pll_idx = 0;
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_INTERNAL_read_pll_div(&phy_copy.access, &pll_div));

        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_get_vco_from_refclk_div(&phy_copy.access, refclk_in_hz, pll_div, &vco_rate, 0));

        if (vco_rate == 41250000) {
            /* load 41G VCO spd_id */
            spd_intf = TSCOMOD_SPD_CL73_IEEE_41G;
        } else if (vco_rate == 51562500) {
            /* load 51G VCO spd_id */
            spd_intf = TSCOMOD_SPD_CL73_IEEE_51G;
        } else if (vco_rate == 53125000) {
            /* load 53G VCO spd_id */
            spd_intf = TSCOMOD_SPD_CL73_IEEE_53G;
        } else {
            return PHYMOD_E_PARAM;
        }

        PHYMOD_IF_ERR_RETURN
            (tscomod_get_mapped_speed(spd_intf, &mapped_speed_id));

        phy_copy.access.lane_mask = 0x1 << start_lane;

        /* Load 1G speed ID */
        PHYMOD_IF_ERR_RETURN
            (tscomod_set_sc_speed(&phy_copy.access, mapped_speed_id, 0));
    }

    do_lane_config_set = 0;
    if (an->enable) {
        /* make sure the firmware config is set to an enabled */
        PHYMOD_IF_ERR_RETURN
            (tsco_phy_firmware_lane_config_get(&phy_copy, &firmware_lane_config));
        /* make sure the firmware config is set to an enabled */
        if (firmware_lane_config.AnEnabled != 1) {
          firmware_lane_config.AnEnabled = 1;
          do_lane_config_set = 1;
        }
        if (firmware_lane_config.LaneConfigFromPCS != 1) {
          firmware_lane_config.LaneConfigFromPCS = 1;
          do_lane_config_set = 1;
        }
        firmware_lane_config.Cl72RestTO = 0;
        firmware_lane_config.ForceNormalReach = 0;
        firmware_lane_config.ForceExtenedReach = 0;
        firmware_lane_config.ForceNRZMode= 0;
        firmware_lane_config.ForcePAM4Mode = 0;
    }

    if (do_lane_config_set) {
        for (i = 0; i < num_lane; i++) {
            if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
                continue;
            }
            phy_copy.access.lane_mask = 0x1 << (i + start_lane);
            PHYMOD_IF_ERR_RETURN
                (osprey7_v2l8p1_lane_soft_reset(&phy_copy.access, 1));
        }
        PHYMOD_USLEEP(1000);
        for (i = 0; i < num_lane; i++) {
            if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
                continue;
            }
            phy_copy.access.lane_mask = 0x1 << (i + start_lane);
            PHYMOD_IF_ERR_RETURN
                (_tsco_phy_firmware_lane_config_set(&phy_copy, firmware_lane_config));
        }
        for (i = 0; i < num_lane; i++) {
            if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
                continue;
            }
            phy_copy.access.lane_mask = 0x1 << (i + start_lane);
            PHYMOD_IF_ERR_RETURN
                (osprey7_v2l8p1_lane_soft_reset(&phy_copy.access, 0));
        }
    }

    phy_copy.access.lane_mask = 0x1 << start_lane;

    PHYMOD_IF_ERR_RETURN
        (tscomod_autoneg_control(&phy_copy.access, &an_control));

    return PHYMOD_E_NONE;
}

int tsco_phy_autoneg_get(const phymod_phy_access_t* phy,
                         phymod_autoneg_control_t* an, uint32_t* an_done)
{
    tscomod_an_control_t an_control;
    phymod_phy_access_t phy_copy;
    int start_lane, num_lane;
    int an_complete = 0;

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
    phy_copy.access.lane_mask = 0x1 << start_lane;

    PHYMOD_MEMSET(&an_control, 0x0,  sizeof(tscomod_an_control_t));
    PHYMOD_IF_ERR_RETURN
        (tscomod_autoneg_control_get(&phy_copy.access, &an_control, &an_complete));

    if (an_control.enable) {
        an->enable = 1;
        *an_done = an_complete;
    } else {
        an->enable = 0;
    }

    switch (an_control.num_lane_adv) {
        case 0:
            an->num_lane_adv = 1;
            break;
        case 1:
            an->num_lane_adv = 2;
            break;
        case 2:
            an->num_lane_adv = 4;
            break;
        case 3:
            an->num_lane_adv = 8;
            break;
        default:
            an->num_lane_adv = 0;
            break;
    }

    switch (an_control.an_type) {
        case TSCOMOD_AN_MODE_CL73:
            an->an_mode = phymod_AN_MODE_CL73;
            break;
        case TSCOMOD_AN_MODE_CL73_BAM:
            an->an_mode = phymod_AN_MODE_CL73BAM;
            break;
        case TSCOMOD_AN_MODE_MSA:
            an->an_mode = phymod_AN_MODE_MSA;
            break;
        case TSCOMOD_AN_MODE_CL73_MSA:
            an->an_mode = phymod_AN_MODE_CL73_MSA;
            break;
        default:
            an->an_mode = phymod_AN_MODE_NONE;
            break;
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_autoneg_status_get(const phymod_phy_access_t* phy, phymod_autoneg_status_t* status)
{
    int an_en, an_done;
    phymod_phy_speed_config_t speed_config;
    phymod_phy_access_t phy_copy;
    int start_lane, num_lane, speed_id;
    uint32_t packed_entry[5];
    tscomod_spd_id_tbl_entry_t speed_config_entry;

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
    phy_copy.access.lane_mask = 0x1 << start_lane;

    PHYMOD_IF_ERR_RETURN
       (tscomod_autoneg_status_get(&phy_copy.access, &an_en, &an_done));

    PHYMOD_IF_ERR_RETURN
       (tsco_phy_speed_config_get(phy, &speed_config));

    if (an_en && an_done) {
        uint32_t an_resolved_mode;
        /* if an resolves and link up */
        PHYMOD_IF_ERR_RETURN
            (tscomod_speed_id_get(&phy_copy.access, &speed_id));
        /*read the speed id entry and get the num_lane info */
        phy_copy.access.lane_mask = 1 << start_lane;
        PHYMOD_IF_ERR_RETURN
            (phymod_mem_read(&phy_copy.access, phymodMemSpeedIdTable, speed_id, packed_entry));
        tscomod_spd_ctrl_unpack_spd_id_tbl_entry(packed_entry, &speed_config_entry);
        num_lane = 1 << speed_config_entry.num_lanes;
        /* read the AN final resolved port mode */
        PHYMOD_IF_ERR_RETURN
            (tscomod_resolved_port_mode_get(&phy_copy.access, &an_resolved_mode));
        status->resolved_port_mode = an_resolved_mode;
    }

    status->enabled   = an_en;
    status->locked    = an_done;
    status->data_rate = speed_config.data_rate;
    status->resolved_num_lane = num_lane;

    return PHYMOD_E_NONE;
}

int tsco_phy_pll_reconfig(const phymod_phy_access_t* phy,
                          uint8_t pll_index,
                          uint32_t pll_div,
                          phymod_ref_clk_t ref_clk1)
{
    phymod_phy_access_t pm_phy_copy;
    tscomod_refclk_t ref_clk;
    uint32_t tvco_rate = 0;
    enum osprey7_v2l8p1_pll_refclk_enum refclk;
    uint32_t cnt = 0, pll_lock = 0, fclk_div_mode = 1;

    PHYMOD_MEMCPY(&pm_phy_copy, phy, sizeof(pm_phy_copy));

    if (pll_index > 0) {
        PHYMOD_DEBUG_ERROR(("Unsupported PLL index\n"));
        return PHYMOD_E_UNAVAIL;
    }

    pm_phy_copy.access.lane_mask = 1 << 0;
    /* first needs to read the ref clock from main reg*/
    PHYMOD_IF_ERR_RETURN
        (tscomod_refclk_get(&pm_phy_copy.access, &ref_clk));

    if (ref_clk == TSCOMOD_REF_CLK_312P5MHZ) {
        refclk = OSPREY7_V2L8P1_PLL_REFCLK_312P5MHZ;
    } else {
        PHYMOD_DEBUG_ERROR(("Unsupported reference clock.\n"));
        return PHYMOD_E_UNAVAIL;
    }

    PHYMOD_IF_ERR_RETURN
        (tscomod_pll_to_vco_get(ref_clk, pll_div, &tvco_rate));

    /* next disable pcs datapath only if TVCO re-config*/
    PHYMOD_MEMCPY(&pm_phy_copy, phy, sizeof(pm_phy_copy));
    PHYMOD_IF_ERR_RETURN
        (tscomod_disable_set(&pm_phy_copy.access));

    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_core_dp_reset(&pm_phy_copy.access, 1));

    /*next re-config pll divider */
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_configure_pll_refclk_div(&pm_phy_copy.access,
                                                  refclk,
                                                  pll_div));

    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_core_dp_reset(&pm_phy_copy.access, 0));


    COMPILER_REFERENCE(fclk_div_mode);
    /* Configure FCLK period. */
    
    /* 
    PHYMOD_IF_ERR_RETURN
        (tscomod_set_fclk_period(&pm_phy_copy.access, tvco_rate, fclk_div_mode));
     */

    /* need to wait for the PLL lock */
    cnt = 0;
    while (cnt < 1000) {
        PHYMOD_IF_ERR_RETURN(osprey7_v2l8p1_pll_lock_get(&pm_phy_copy.access, &pll_lock));
        cnt = cnt + 1;
        if (pll_lock) {
            break;
        } else {
            if (cnt == 1000) {
                PHYMOD_DEBUG_ERROR(("WARNING :: core 0x%x PLL Index %d is not locked within 10 milli second \n", pm_phy_copy.access.addr, pll_index));
                break;
            }
        }
        PHYMOD_USLEEP(10);
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_tx_pam4_precoder_enable_set(const phymod_phy_access_t* phy, int enable)
{
    phymod_phy_access_t phy_copy;
    int start_lane, num_lane, i;
    uint32_t lane_reset, pcs_lane_enable;

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));
    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    /*first check if lane is in reset */
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_lane_soft_reset_get(&phy_copy.access, &lane_reset));

    /*next check if PCS lane is in reset */
    PHYMOD_IF_ERR_RETURN
        (tscomod_enable_get(&phy_copy.access, &pcs_lane_enable));

    /* disable pcs lane if pcs lane not in rset */
    if (pcs_lane_enable) {
        phy_copy.access.lane_mask = 1 << start_lane;
        PHYMOD_IF_ERR_RETURN
            (tscomod_disable_set(&phy_copy.access));
    }

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
    /* if lane is not in reset, then reset the lane first */
    if (!lane_reset) {
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_lane_soft_reset(&phy_copy.access, 1));
    }

    for (i = 0; i < num_lane; i++) {
        if (!PHYMOD_LANEPBMP_MEMBER(phy->access.lane_mask, start_lane + i)) {
            continue;
        }
        phy_copy.access.lane_mask = 1 << (start_lane + i);
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_tx_pam4_precoder_enable_set(&phy_copy.access, enable));
    }

    /* release the ln dp reset */
    if (!lane_reset) {
        PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_lane_soft_reset(&phy_copy.access, 0));
    }

    /* re-enable pcs lane if pcs lane not in rset */
    if (pcs_lane_enable) {
        phy_copy.access.lane_mask = 1 << start_lane;
        PHYMOD_IF_ERR_RETURN
            (tscomod_enable_set(&phy_copy.access));
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_tx_pam4_precoder_enable_get(const phymod_phy_access_t* phy, int *enable)
{
    phymod_phy_access_t pm_phy_copy;

    PHYMOD_MEMCPY(&pm_phy_copy, phy, sizeof(pm_phy_copy));
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_tx_pam4_precoder_enable_get(&pm_phy_copy.access, enable));
    return PHYMOD_E_NONE;
}

int tsco_timesync_enable_set(const phymod_phy_access_t* phy, uint32_t flags, uint32_t enable)
{

    return PHYMOD_E_NONE;
}

int tsco_timesync_enable_get(const phymod_phy_access_t* phy, uint32_t flags, uint32_t* enable)
{

    return PHYMOD_E_NONE;
}

int tsco_timesync_offset_set(const phymod_core_access_t* core, uint32_t ts_offset)
{

    return PHYMOD_E_NONE;
}

int tsco_timesync_adjust_set(const phymod_phy_access_t* phy,
                             uint32_t flags, uint32_t ts_am_norm_mode)
{

    return PHYMOD_E_NONE;
}

int tsco_phy_pcs_lane_swap_adjust(const phymod_phy_access_t* phy,
                                  uint32_t active_lane_map,
                                  uint32_t original_tx_lane_map,
                                  uint32_t original_rx_lane_map)
{
    phymod_phy_access_t phy_copy;
    int start_lane, num_lane, i, disable_lane[2]= {0,0}, disable_lane_num = 0;
    int tx_physical_lane, tx_logical_lane, rx_physical_lane, rx_logical_lane;
    uint32_t new_tx_lane_map, new_rx_lane_map, swap_mask, tmp_tx_lane_map, tmp_rx_lane_map;

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));
    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    /* next based on active_lane_map, figure out the disable lane num  and index*/
    for (i = 0; i < num_lane; i++)
    {
        if(active_lane_map & (1 << i))
        {
            continue;
        } else {
            disable_lane[disable_lane_num] = i + start_lane;
            disable_lane_num++;
        }
    }

    /* next check if disable_lane_num 0 */
    if (disable_lane_num == 0) {
        new_tx_lane_map = original_tx_lane_map;
        new_rx_lane_map = original_rx_lane_map;

        /* next check if 4 lane or 8 lane port */
        if (num_lane == 8) {
            PHYMOD_IF_ERR_RETURN
                (tscomod_pcs_tx_m1_lane_swap(&phy_copy.access, new_tx_lane_map));
            PHYMOD_IF_ERR_RETURN
                (tscomod_pcs_rx_lane_swap(&phy_copy.access, new_rx_lane_map));
        } else { /* 4 lane port */
            for (i = 0; i < num_lane; i++) {
                tx_logical_lane = i + start_lane;
                rx_logical_lane = i + start_lane;
                tx_physical_lane = (original_tx_lane_map & (0xf << (tx_logical_lane * 4)) ) >> (tx_logical_lane * 4);
                rx_physical_lane = (original_rx_lane_map & (0xf << (rx_logical_lane * 4)) ) >> (rx_logical_lane * 4);
                tx_physical_lane &= 0xf;
                rx_physical_lane &= 0xf;
                PHYMOD_IF_ERR_RETURN
                    (tscomod_pcs_single_tx_lane_swap_set(&phy_copy.access, tx_physical_lane, tx_logical_lane));
                PHYMOD_IF_ERR_RETURN
                    (tscomod_pcs_single_rx_lane_swap_set(&phy_copy.access, rx_physical_lane, rx_logical_lane));
            }
        }
    } else  {
        if (num_lane == 4) {
            /* first check if disable lane is the last, if yes, then need to adjust */
            if ((disable_lane[0] == 3) || (disable_lane[0] == 7)) {
                return PHYMOD_E_NONE;
            } else if (disable_lane[0] < 4) {
                /* mpp0 */
                tx_logical_lane = disable_lane[0];
                rx_logical_lane = disable_lane[0];
                tx_physical_lane = (original_tx_lane_map & (0xf << (3 * 4)) & 0xffff) >> (3 * 4);
                rx_physical_lane = (original_rx_lane_map & (0xf << (3 * 4)) & 0xffff) >> (3 * 4);
                tx_physical_lane &= 0xf;
                rx_physical_lane &= 0xf;
                PHYMOD_IF_ERR_RETURN
                    (tscomod_pcs_single_tx_lane_swap_set(&phy_copy.access, tx_physical_lane, tx_logical_lane));
                PHYMOD_IF_ERR_RETURN
                    (tscomod_pcs_single_rx_lane_swap_set(&phy_copy.access, rx_physical_lane, rx_logical_lane));
                /* mpp0 */
                tx_logical_lane = 3;
                rx_logical_lane = 3;
                tx_physical_lane = (original_tx_lane_map & (0xf << (disable_lane[0] * 4)) & 0xffff) >> (disable_lane[0] * 4);
                rx_physical_lane = (original_rx_lane_map & (0xf << (disable_lane[0] * 4)) & 0xffff) >> (disable_lane[0] * 4);
                tx_physical_lane &= 0xf;
                rx_physical_lane &= 0xf;
                PHYMOD_IF_ERR_RETURN
                    (tscomod_pcs_single_tx_lane_swap_set(&phy_copy.access, tx_physical_lane, tx_logical_lane));
                PHYMOD_IF_ERR_RETURN
                    (tscomod_pcs_single_rx_lane_swap_set(&phy_copy.access, rx_physical_lane, rx_logical_lane));
            } else {
                /* mpp1 */
                tx_logical_lane = disable_lane[0];
                rx_logical_lane = disable_lane[0];
                tx_physical_lane = (original_tx_lane_map & (0xf << (7 * 4)) & 0xffff0000) >> (7 * 4);
                rx_physical_lane = (original_rx_lane_map & (0xf << (7 * 4)) & 0xffff0000) >> (7 * 4);
                tx_physical_lane &= 0xf;
                rx_physical_lane &= 0xf;
                PHYMOD_IF_ERR_RETURN
                    (tscomod_pcs_single_tx_lane_swap_set(&phy_copy.access, tx_physical_lane, tx_logical_lane));
                PHYMOD_IF_ERR_RETURN
                    (tscomod_pcs_single_rx_lane_swap_set(&phy_copy.access, rx_physical_lane, rx_logical_lane));
                /* mpp1 */
                tx_logical_lane = 7;
                rx_logical_lane = 7;
                tx_physical_lane = (original_tx_lane_map & (0xf << (disable_lane[0] * 4)) & 0xffff0000) >> (disable_lane[0] * 4);
                rx_physical_lane = (original_rx_lane_map & (0xf << (disable_lane[0] * 4)) & 0xffff0000) >> (disable_lane[0] * 4);
                tx_physical_lane &= 0xf;
                rx_physical_lane &= 0xf;
                PHYMOD_IF_ERR_RETURN
                    (tscomod_pcs_single_tx_lane_swap_set(&phy_copy.access, tx_physical_lane, tx_logical_lane));
                PHYMOD_IF_ERR_RETURN
                    (tscomod_pcs_single_rx_lane_swap_set(&phy_copy.access, rx_physical_lane, rx_logical_lane));
            }
        } else if (num_lane == 8) {
            /* first check if disable lane is the last, if yes, then no need to adjust */
            if (disable_lane_num == 1) {
                /* no need to do pcs adjust */
                if (disable_lane[0] == 7) {
                    return PHYMOD_E_NONE;
                } else {
                    new_tx_lane_map = (original_tx_lane_map & (0xf << (7 * 4)) & 0xffffffff) >> (4 * (7 - disable_lane[0]));
                    new_tx_lane_map |= (original_tx_lane_map & (0xf << (disable_lane[0] * 4)) & 0xffffffff) << (4 * (7 - disable_lane[0]));
                    new_rx_lane_map = (original_rx_lane_map & (0xf << (7 * 4)) & 0xffffffff) >> (4 * (7 - disable_lane[0]));
                    new_rx_lane_map |= (original_rx_lane_map & (0xf << (disable_lane[0] * 4)) & 0xffffffff) << (4 * (7 - disable_lane[0]));
                    swap_mask = ~(0xf << (4 * 7) | 0xf << (4 * disable_lane[0]));
                    new_tx_lane_map |=  (original_tx_lane_map & swap_mask);
                    new_rx_lane_map |=  (original_rx_lane_map & swap_mask);
                }
            } else { /*2 lane disabled */
                if ((disable_lane[0] == 6)  && (disable_lane[1] == 7)) {
                    /* no need to do pcs adjust */
                    return PHYMOD_E_NONE;
                } else if (disable_lane[1] != 6) {
                    new_tx_lane_map = (original_tx_lane_map & (0xf << (6 * 4)) & 0xffffffff) >> (4 * (6 - disable_lane[0]));
                    new_tx_lane_map |= (original_tx_lane_map & (0xf << (disable_lane[0] * 4)) & 0xffffffff) << (4 * (6 - disable_lane[0]));
                    tmp_tx_lane_map = (original_tx_lane_map & (0xf << (7 * 4)) & 0xffffffff) >> (4 * (7 - disable_lane[1]));
                    tmp_tx_lane_map |= (original_tx_lane_map & (0xf << (disable_lane[1] * 4)) & 0xffffffff) <<  (4 * (7 - disable_lane[1]));
                    new_tx_lane_map |= tmp_tx_lane_map;
                    new_rx_lane_map = (original_rx_lane_map & (0xf << (6 * 4)) & 0xffffffff) >> (4 * (6 - disable_lane[0]));
                    new_rx_lane_map |= (original_rx_lane_map & (0xf << (disable_lane[0] * 4)) & 0xffffffff) << (4 * (6 - disable_lane[0]));
                    tmp_rx_lane_map = (original_rx_lane_map & (0xf << (7 * 4)) & 0xffffffff) >> (4 * (7 - disable_lane[1]));
                    tmp_rx_lane_map |= (original_rx_lane_map & (0xf << (disable_lane[1] * 4)) & 0xffffffff) << (4 * (7 - disable_lane[1]));
                    new_rx_lane_map |= tmp_rx_lane_map;
                    swap_mask = ~(0xff << (4 * 6) | 0xf << (4 * disable_lane[0]) | 0xf << (4 * disable_lane[1]));
                    new_tx_lane_map |=  (original_tx_lane_map & swap_mask);
                    new_rx_lane_map |=  (original_rx_lane_map & swap_mask);
                } else {
                    tmp_tx_lane_map = (original_tx_lane_map & (0xf << (7 * 4)) & 0xffffffff) >> (4 * (7 - disable_lane[0]));
                    tmp_tx_lane_map |= (original_tx_lane_map & (0xf << (disable_lane[0] * 4)) & 0xffffffff) <<  (4 * (7 - disable_lane[0]));
                    new_tx_lane_map = tmp_tx_lane_map;
                    tmp_rx_lane_map = (original_rx_lane_map & (0xf << (7 * 4)) & 0xffffffff) >> (4 * (7 - disable_lane[0]));
                    tmp_rx_lane_map |= (original_rx_lane_map & (0xf << (disable_lane[0] * 4)) & 0xffffffff) << (4 * (7 - disable_lane[0]));
                    new_rx_lane_map = tmp_rx_lane_map;
                    swap_mask = ~(0xf << (4 * 7) | 0xf << (4 * disable_lane[0]));
                    new_tx_lane_map |=  (original_tx_lane_map & swap_mask);
                    new_rx_lane_map |=  (original_rx_lane_map & swap_mask);
                }
            }

            PHYMOD_IF_ERR_RETURN
                (tscomod_pcs_tx_m1_lane_swap(&phy_copy.access, new_tx_lane_map));
            PHYMOD_IF_ERR_RETURN
                (tscomod_pcs_rx_lane_swap(&phy_copy.access, new_rx_lane_map));
        }
    }
    return PHYMOD_E_NONE;
}

int tsco_phy_load_speed_id_entry(const phymod_phy_access_t* phy,
                                 uint32_t speed,
                                 uint32_t num_lane,
                                 phymod_fec_type_t fec_type)
{
    phymod_phy_access_t phy_copy;
    int start_lane, local_num_lane, mapped_speed_id;
    tscomod_spd_intfc_type_t spd_intf;
    uint32_t tvco_pll_div;

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &local_num_lane));
    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    /* first check the speed */
    switch (speed) {
    case 150000:
        spd_intf = TSCOMOD_SPD_150G_FEC_544_2XN_N3;
        break;
    case 350000:
        spd_intf = TSCOMOD_SPD_350G_FEC_544_2XN_N7;
        break;
    case 300000:
        spd_intf = TSCOMOD_SPD_300G_FEC_544_2XN_N6;
        break;
    case 400000:
        if (fec_type == phymod_fec_RS544_2XN) {
            spd_intf = TSCOMOD_SPD_400G_BRCM_FEC_KR8_CR8;
        } else if (fec_type == phymod_fec_RS272_2XN) {
            spd_intf = TSCOMOD_SPD_400G_BRCM_FEC_272_N8;
        } else {
            PHYMOD_DEBUG_ERROR(("Unsupported 200G fec type\n"));
            return PHYMOD_E_UNAVAIL;
        }
        break;
    case 200000:
        if (fec_type == phymod_fec_RS544_2XN) {
            spd_intf = TSCOMOD_SPD_200G_IEEE_KR4_CR4;
        } else if (fec_type == phymod_fec_RS544) {
            spd_intf = TSCOMOD_SPD_200G_BRCM_KR4_CR4;
        } else if (fec_type == phymod_fec_RS272) {
            spd_intf = TSCOMOD_SPD_200G_BRCM_FEC_272_N4;
        } else if (fec_type == phymod_fec_RS272_2XN) {
            spd_intf = TSCOMOD_SPD_200G_BRCM_FEC_272_CR4_KR4;
        } else {
            PHYMOD_DEBUG_ERROR(("Unsupported 200G fec type\n"));
            return PHYMOD_E_UNAVAIL;
        }
        break;
    default:
        PHYMOD_DEBUG_ERROR(("Unsupported speed for speed id load function \n"));
        return PHYMOD_E_UNAVAIL;
    }

    PHYMOD_IF_ERR_RETURN
        (tscomod_get_mapped_speed(spd_intf, &mapped_speed_id));

    /* Get TVCO because it's not allowed to change during speed set */
    phy_copy.access.pll_idx = 0;

    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_INTERNAL_read_pll_div(&phy_copy.access, &tvco_pll_div));

    /* based on the current TVCO PLL div, decide which copy of speed id entry to load */
    /* first set the lane mask to be 0x1 */
    phy_copy.access.lane_mask = 1 << 0;
    if ((int) tvco_pll_div == TSCOMOD_PLL_MODE_DIV_170)  {
        /* then load 53G TVCO speed id entry */
        PHYMOD_IF_ERR_RETURN
            (phymod_mem_write(&phy_copy.access, phymodMemSpeedIdTable, TSCOMOD_FORCED_SPEED_ID_OFFSET + start_lane, &tsco_spd_id_entry_53[mapped_speed_id][0]));
    } else if ((int)tvco_pll_div == TSCOMOD_PLL_MODE_DIV_165) {
        /* then load 51G TVCO speed id entry */
        PHYMOD_IF_ERR_RETURN
            (phymod_mem_write(&phy_copy.access, phymodMemSpeedIdTable, TSCOMOD_FORCED_SPEED_ID_OFFSET + start_lane, &tsco_spd_id_entry_51[mapped_speed_id][0]));
    } else {
        /* then load 41G TVCO speed id entry */
        PHYMOD_IF_ERR_RETURN
            (phymod_mem_write(&phy_copy.access, phymodMemSpeedIdTable, TSCOMOD_FORCED_SPEED_ID_OFFSET + start_lane, &tsco_spd_id_entry_41[mapped_speed_id][0]));
    }

    return PHYMOD_E_NONE;
}

int tsco_timesync_tx_info_get(const phymod_phy_access_t* phy, phymod_ts_fifo_status_t* ts_tx_info)
{

    return PHYMOD_E_NONE;
}

int tsco_phy_autoneg_speed_id_table_reload(const phymod_phy_access_t* phy, uint32_t gsh_header_enable)
{
    phymod_phy_access_t phy_copy;
    uint32_t tvco_pll_div, i;

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    /*first read the current TVCO rate */
    phy_copy.access.pll_idx = 0;

    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_INTERNAL_read_pll_div(&phy_copy.access, &tvco_pll_div));

    /* based on the current TVCO PLL div, decide which copy of speed id entry to load */
    /* first set the lane mask to be 0x1 */
    phy_copy.access.lane_mask = 1 << 0;
    /* need to reload autoneg speed id table based on gsh enable or not */
    if ((int) tvco_pll_div == TSCOMOD_PLL_MODE_DIV_170)  {
        /* then load 53G TVCO speed id entry */
        for (i = 0; i < TSCOMOD_AUTONEG_SPEED_ID_COUNT; i++) {
            if (gsh_header_enable) {
                PHYMOD_IF_ERR_RETURN
                    (phymod_mem_write(&phy_copy.access, phymodMemSpeedIdTable, i, &tsco_spd_id_entry_53_gsh[i][0]));
            } else {
                PHYMOD_IF_ERR_RETURN
                    (phymod_mem_write(&phy_copy.access, phymodMemSpeedIdTable, i, &tsco_spd_id_entry_53[i][0]));
            }
        }
    } else if ((int) tvco_pll_div == TSCOMOD_PLL_MODE_DIV_165) {
        /* then load 51G TVCO speed id entry */
        for (i = 0; i < TSCOMOD_AUTONEG_SPEED_ID_COUNT; i++) {
            if (gsh_header_enable) {
                PHYMOD_IF_ERR_RETURN
                    (phymod_mem_write(&phy_copy.access, phymodMemSpeedIdTable, i, &tsco_spd_id_entry_51_gsh[i][0]));
            } else {
                PHYMOD_IF_ERR_RETURN
                    (phymod_mem_write(&phy_copy.access, phymodMemSpeedIdTable, i, &tsco_spd_id_entry_51[i][0]));
            }
        }
    } else {
        /* then load 41G TVCO speed id entry */
        for (i = 0; i < TSCOMOD_AUTONEG_SPEED_ID_COUNT; i++) {
            if (gsh_header_enable) {
                PHYMOD_IF_ERR_RETURN
                    (phymod_mem_write(&phy_copy.access, phymodMemSpeedIdTable, i, &tsco_spd_id_entry_41_gsh[i][0]));
            } else {
                PHYMOD_IF_ERR_RETURN
                    (phymod_mem_write(&phy_copy.access, phymodMemSpeedIdTable, i, &tsco_spd_id_entry_41[i][0]));
            }
        }
    }
    return PHYMOD_E_NONE;
}

int tsco_phy_pcs_enable_set(const phymod_phy_access_t* phy, uint32_t enable)
{
    phymod_phy_access_t phy_copy;
    int start_lane, num_lane;

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));
    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    if (enable == 1) {
        phy_copy.access.lane_mask = 1 << start_lane;
        PHYMOD_IF_ERR_RETURN
            (tscomod_enable_set(&phy_copy.access));
    } else if (enable == 0) {
        PHYMOD_IF_ERR_RETURN
            (tscomod_disable_set(&phy_copy.access));
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_synce_clk_ctrl_set(const phymod_phy_access_t* phy,
                                phymod_synce_clk_ctrl_t cfg)
{
    phymod_phy_access_t phy_copy;
    uint32_t sdm_val, data_rate_lane = 0;
    int osr_mode;
    uint32_t pll_div;
    phymod_firmware_lane_config_t firmware_lane_config = {0};
    tscomod_refclk_t ref_clk;
    uint32_t vco_rate = 41250000, refclk_in_hz;

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    PHYMOD_IF_ERR_RETURN
         (tscomod_synce_mode_set(&phy_copy.access, cfg.stg0_mode, cfg.stg1_mode));

    /* next check if SDM mode, if yes, needs to figure out the SDM value based on the current */
    if ((cfg.stg0_mode == 0x2) && (cfg.stg1_mode == 0x0)) {
        /* next figure out per lane speed */
        PHYMOD_MEMSET(&firmware_lane_config, 0, sizeof(firmware_lane_config));
        PHYMOD_IF_ERR_RETURN
            (tsco_phy_firmware_lane_config_get(phy, &firmware_lane_config));

        /* get the PLL div from HW */
        phy_copy.access.pll_idx = 0;
        phy_copy.access.lane_mask = 0x1;
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_INTERNAL_read_pll_div(&phy_copy.access, &pll_div));

        PHYMOD_IF_ERR_RETURN
            (tscomod_refclk_get(&phy_copy.access, &ref_clk));

        if (ref_clk == TSCOMOD_REF_CLK_312P5MHZ) {
            refclk_in_hz = 312500000;
        } else {
            refclk_in_hz = 156250000;
        }
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_get_vco_from_refclk_div(&phy_copy.access, refclk_in_hz, pll_div, &vco_rate, 0));

        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_osr_mode_get(&phy_copy.access, &osr_mode));

        PHYMOD_IF_ERR_RETURN
            (_tsco_per_lane_data_rate_get(vco_rate, osr_mode, firmware_lane_config.ForcePAM4Mode, &data_rate_lane));

        if (data_rate_lane == 10000) {
            sdm_val = TSCOMOD_SYNCE_SDM_DIVISOR_10G_PER_LANE;
        } else if (data_rate_lane == 20000) {
            sdm_val = TSCOMOD_SYNCE_SDM_DIVISOR_20G_PER_LANE;
        } else if (data_rate_lane == 25000) {
            sdm_val = TSCOMOD_SYNCE_SDM_DIVISOR_25G_PER_LANE;
        } else if ((firmware_lane_config.ForcePAM4Mode) && (pll_div == TSCOMOD_PLL_MODE_DIV_170)) {
            sdm_val = TSCOMOD_SYNCE_SDM_DIVISOR_53G_VCO_PAM4;
        } else if ((firmware_lane_config.ForcePAM4Mode) && (pll_div == TSCOMOD_PLL_MODE_DIV_165)) {
            sdm_val = TSCOMOD_SYNCE_SDM_DIVISOR_51G_VCO_PAM4;
        } else {
            PHYMOD_DEBUG_ERROR(("Unsupported speeds\n"));
            return PHYMOD_E_UNAVAIL;
        }

        /* next configure the SDM value */
        PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
        PHYMOD_IF_ERR_RETURN
            (tscomod_synce_clk_ctrl_set(&phy_copy.access, sdm_val));
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_synce_clk_ctrl_get(const phymod_phy_access_t* phy,
                                phymod_synce_clk_ctrl_t *cfg)
{
    phymod_phy_access_t phy_copy;

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    PHYMOD_IF_ERR_RETURN
        (tscomod_synce_mode_get(&phy_copy.access, &(cfg->stg0_mode), &(cfg->stg1_mode)));

    PHYMOD_IF_ERR_RETURN
        (tscomod_synce_clk_ctrl_get(&phy_copy.access, &(cfg->sdm_val)));

    return PHYMOD_E_NONE;
}

int tsco_intr_handler(const phymod_phy_access_t* phy,
                      phymod_interrupt_type_t type,
                      uint32_t* is_handled)
{
    return PHYMOD_E_NONE;
}

int tsco_phy_codec_mode_set(const phymod_phy_access_t* phy, phymod_phy_codec_mode_t codec_type)
{
    phymod_phy_access_t phy_copy;
    tscomod_spd_id_tbl_entry_t speed_config_entry;
    uint32_t packed_entry[5];
    int start_lane, num_lane;
    uint32_t lane_reset, pcs_lane_enable;

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));
    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    /*first check if lane is in reset */
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_lane_soft_reset_get(&phy_copy.access, &lane_reset));

    /*next check if PCS lane is in reset */
    PHYMOD_IF_ERR_RETURN
        (tscomod_enable_get(&phy_copy.access, &pcs_lane_enable));

    /* disable pcs lane if pcs lane not in rset */
    if (pcs_lane_enable) {
        phy_copy.access.lane_mask = 1 << start_lane;
        PHYMOD_IF_ERR_RETURN
            (tscomod_disable_set(&phy_copy.access));
    }

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
    /* if lane is not in reset, then reset the lane first */
    if (!lane_reset) {
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_lane_soft_reset(&phy_copy.access, 1));
    }

    /* first read the current speed id entry for this port */
    PHYMOD_IF_ERR_RETURN
        (phymod_mem_read(&phy_copy.access, phymodMemSpeedIdTable, TSCOMOD_FORCED_SPEED_ID_OFFSET + start_lane, packed_entry));

    tscomod_spd_ctrl_unpack_spd_id_tbl_entry(packed_entry, &speed_config_entry);

    /* update the codec field */
    speed_config_entry.codec_mode = codec_type;

    /* next pack the speed config entry into 5 word format */
    tscomod_spd_ctrl_pack_spd_id_tbl_entry(&speed_config_entry, &packed_entry[0]);

    /* write back to the speed id HW table */
    PHYMOD_IF_ERR_RETURN
        (phymod_mem_write(&phy_copy.access, phymodMemSpeedIdTable, TSCOMOD_FORCED_SPEED_ID_OFFSET + start_lane, &packed_entry[0]));

    /* release the ln dp reset */
    if (!lane_reset) {
        PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_lane_soft_reset(&phy_copy.access, 0));
    }

    /* re-enable pcs lane if pcs lane not in rset */
    if (pcs_lane_enable) {
        phy_copy.access.lane_mask = 1 << start_lane;
        PHYMOD_IF_ERR_RETURN
            (tscomod_enable_set(&phy_copy.access));
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_codec_mode_get(const phymod_phy_access_t* phy, phymod_phy_codec_mode_t* codec_type)
{
    phymod_phy_access_t phy_copy;
    tscomod_spd_id_tbl_entry_t speed_config_entry;
    uint32_t packed_entry[5];
    int start_lane, num_lane, speed_id;

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));
    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    phy_copy.access.lane_mask = 1 << start_lane;

    /* first read speed id from resolved status */
    PHYMOD_IF_ERR_RETURN
        (tscomod_speed_id_get(&phy_copy.access, &speed_id));

    /* first read the current speed id entry for this port */
    PHYMOD_IF_ERR_RETURN
        (phymod_mem_read(&phy_copy.access, phymodMemSpeedIdTable, speed_id, packed_entry));

    tscomod_spd_ctrl_unpack_spd_id_tbl_entry(packed_entry, &speed_config_entry);

    /* the codeec type */
    *codec_type = speed_config_entry.codec_mode;

    return PHYMOD_E_NONE;
}

int tsco_phy_fec_bypass_indication_set(const phymod_phy_access_t* phy,
                                       uint32_t enable)
{
    int start_lane, num_lane;
    phymod_phy_access_t phy_copy;
    uint32_t lane_reset, pcs_lane_enable;

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));

    /*first check if lane is in reset */
    PHYMOD_IF_ERR_RETURN
        (osprey7_v2l8p1_lane_soft_reset_get(&phy_copy.access, &lane_reset));

    /*next check if PCS lane is in reset */
    PHYMOD_IF_ERR_RETURN
        (tscomod_enable_get(&phy_copy.access, &pcs_lane_enable));

    /* disable pcs lane if pcs lane not in rset */
    if (pcs_lane_enable) {
        phy_copy.access.lane_mask = 1 << start_lane;
        PHYMOD_IF_ERR_RETURN
            (tscomod_disable_set(&phy_copy.access));
    }

    /* if lane is not in reset, then reset the lane first */
    if (!lane_reset) {
        PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_lane_soft_reset(&phy_copy.access, 1));
    }

    phy_copy.access.lane_mask = 1 << start_lane;
    PHYMOD_IF_ERR_RETURN
      (tscomod_fec_bypass_indication_set(&phy_copy.access, enable));

    /* release the lane soft reset bit */
    if (!lane_reset) {
        PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
        PHYMOD_IF_ERR_RETURN
            (osprey7_v2l8p1_lane_soft_reset(&phy_copy.access, 0));
    }

    /* re-enable pcs lane if pcs lane not in rset */
    if (pcs_lane_enable) {
        phy_copy.access.lane_mask = 1 << start_lane;
        PHYMOD_IF_ERR_RETURN
            (tscomod_enable_set(&phy_copy.access));
    }

    return PHYMOD_E_NONE;
}

int tsco_phy_fec_bypass_indication_get(const phymod_phy_access_t* phy,
                                       uint32_t *enable)
{
    int start_lane, num_lane;
    phymod_phy_access_t phy_copy;

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));

    phy_copy.access.lane_mask = 1 << start_lane;

    PHYMOD_IF_ERR_RETURN
      (tscomod_fec_bypass_indication_get(&phy_copy.access, enable));

    return PHYMOD_E_NONE;
}

int tsco_phy_rs_fec_rxp_get(const phymod_phy_access_t* phy, uint32_t* hi_ser_lh, uint32_t* hi_ser_live)
{
    int start_lane, num_lane;
    phymod_phy_access_t phy_copy;

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));

    PHYMOD_IF_ERR_RETURN
        (phymod_util_lane_config_get(&phy->access, &start_lane, &num_lane));

    phy_copy.access.lane_mask = 1 << start_lane;

    PHYMOD_IF_ERR_RETURN
        (tscomod_rs_fec_hi_ser_get(&phy_copy.access, hi_ser_lh, hi_ser_live));

    return PHYMOD_E_NONE;
}

int tsco_phy_pmd_override_enable_set(const phymod_phy_access_t* phy,
                                           phymod_override_type_t pmd_override_type,
                                           uint32_t override_enable,
                                           uint32_t override_val)
{
    phymod_phy_access_t phy_copy;

    PHYMOD_MEMCPY(&phy_copy, phy, sizeof(phy_copy));
    PHYMOD_IF_ERR_RETURN
        (tscomod_pmd_override_enable_set(&phy_copy.access, pmd_override_type, override_enable, override_val));

    return PHYMOD_E_NONE;
}

