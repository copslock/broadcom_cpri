/*******************************************************************************
 *
 * DO NOT EDIT THIS FILE!
 * This file is auto-generated by fltg from
 *    INTERNAL/fltg/xgs/flex_digest/bcm56780_a0/bcm56780_a0_FLEX_DIGEST_CONTROL.map.ltl for
 *      bcm56780_a0
 *
 * Tool: $SDK/INTERNAL/fltg/bin/fltg
 *
 * Edits to this file will be lost when it is regenerated.
 *
 * This license is set out in https://raw.githubusercontent.com/Broadcom-Network-Switching-Software/OpenBCM/master/Legal/LICENSE file.
 * 
 * Copyright 2007-2020 Broadcom Inc. All rights reserved.
 */
#include <bcmlrd/bcmlrd_internal.h>
#include <bcmlrd/chip/bcmlrd_id.h>
#include <bcmlrd/chip/bcm56780_a0/bcm56780_a0_lrd_field_data.h>
#include <bcmlrd/chip/bcm56780_a0/bcm56780_a0_lrd_ltm_intf.h>
#include <bcmlrd/chip/bcm56780_a0/bcm56780_a0_lrd_xfrm_field_desc.h>
#include <bcmdrd/chip/bcm56780_a0_enum.h>
#include "bcmltd/chip/bcmltd_common_enumpool.h"
#include "bcm56780_a0_lrd_enumpool.h"
/* FLEX_DIGEST_CONTROL field init */
static const bcmlrd_field_data_t bcm56780_a0_lrd_flex_digest_control_map_field_data_mmd[] = {
    { /* 0 VERSATILE_HASH_RANDOM_SEED_A */
      .flags = 0,
      .min = &bcm56780_a0_lrd_ifd_u32_0x0,
      .def = &bcm56780_a0_lrd_ifd_u32_0x0,
      .max = &bcm56780_a0_lrd_ifd_u32_0xffffffff,
      .depth = 0,
      .width = 32,
      .edata = NULL,
    },
    { /* 1 VERSATILE_HASH_RANDOM_SEED_B */
      .flags = 0,
      .min = &bcm56780_a0_lrd_ifd_u32_0x0,
      .def = &bcm56780_a0_lrd_ifd_u32_0x0,
      .max = &bcm56780_a0_lrd_ifd_u32_0xffffffff,
      .depth = 0,
      .width = 32,
      .edata = NULL,
    },
    { /* 2 VERSATILE_HASH_RANDOM_SEED_C */
      .flags = 0,
      .min = &bcm56780_a0_lrd_ifd_u32_0x0,
      .def = &bcm56780_a0_lrd_ifd_u32_0x0,
      .max = &bcm56780_a0_lrd_ifd_u32_0xffffffff,
      .depth = 0,
      .width = 32,
      .edata = NULL,
    },
};
const bcmlrd_map_field_data_t bcm56780_a0_lrd_flex_digest_control_map_field_data = {
    .fields = 3,
    .field = bcm56780_a0_lrd_flex_digest_control_map_field_data_mmd
};

static const bcmlrd_map_table_attr_t bcm56780_a0_lrd_flex_digest_controlt_attr_entry[] = {
    { /* 0 */
        .key   = BCMLRD_MAP_TABLE_ATTRIBUTE_INTERACTIVE,
        .value = FALSE,
    },
};

static const bcmlrd_map_attr_t bcm56780_a0_lrd_flex_digest_controlt_attr_group = {
    .attributes = 1,
    .attr = bcm56780_a0_lrd_flex_digest_controlt_attr_entry,
};

static const bcmlrd_map_entry_t bcm56780_a0_lrd_flex_digest_controlt_flex_digest_hash_vh_init_a_map_entry[] = {
    { /* 0 */
        .entry_type = BCMLRD_MAP_ENTRY_MAPPED_VALUE,
        .desc = {
            .field_id  = VALUEf,
            .field_idx = 0,
            .minbit    = 0,
            .maxbit    = 31,
            .entry_idx = 0,
            .reserved  = 0
        },
        .u = {
            .mapped = {
                .src = {
                    .field_id = FLEX_DIGEST_CONTROLt_VERSATILE_HASH_RANDOM_SEED_Af,
                    .field_idx = 0,
                    .minbit    = 0,
                    .maxbit    = 31,
                    .entry_idx = 0,
                    .reserved  = 0
                }
            }
        },
    },
};
static const bcmlrd_map_entry_t bcm56780_a0_lrd_flex_digest_controlt_flex_digest_hash_vh_init_b_map_entry[] = {
    { /* 0 */
        .entry_type = BCMLRD_MAP_ENTRY_MAPPED_VALUE,
        .desc = {
            .field_id  = VALUEf,
            .field_idx = 0,
            .minbit    = 0,
            .maxbit    = 31,
            .entry_idx = 0,
            .reserved  = 0
        },
        .u = {
            .mapped = {
                .src = {
                    .field_id = FLEX_DIGEST_CONTROLt_VERSATILE_HASH_RANDOM_SEED_Bf,
                    .field_idx = 0,
                    .minbit    = 0,
                    .maxbit    = 31,
                    .entry_idx = 0,
                    .reserved  = 0
                }
            }
        },
    },
};
static const bcmlrd_map_entry_t bcm56780_a0_lrd_flex_digest_controlt_flex_digest_hash_vh_init_c_map_entry[] = {
    { /* 0 */
        .entry_type = BCMLRD_MAP_ENTRY_MAPPED_VALUE,
        .desc = {
            .field_id  = VALUEf,
            .field_idx = 0,
            .minbit    = 0,
            .maxbit    = 31,
            .entry_idx = 0,
            .reserved  = 0
        },
        .u = {
            .mapped = {
                .src = {
                    .field_id = FLEX_DIGEST_CONTROLt_VERSATILE_HASH_RANDOM_SEED_Cf,
                    .field_idx = 0,
                    .minbit    = 0,
                    .maxbit    = 31,
                    .entry_idx = 0,
                    .reserved  = 0
                }
            }
        },
    },
};
static const bcmlrd_map_group_t bcm56780_a0_lrd_flex_digest_control_map_group[] = {
    {
        .dest = {
            .kind = BCMLRD_MAP_PHYSICAL,
            .id = FLEX_DIGEST_HASH_VH_INIT_Ar,
        },
        .entries = 1,
        .entry = bcm56780_a0_lrd_flex_digest_controlt_flex_digest_hash_vh_init_a_map_entry
    },
    {
        .dest = {
            .kind = BCMLRD_MAP_PHYSICAL,
            .id = FLEX_DIGEST_HASH_VH_INIT_Br,
        },
        .entries = 1,
        .entry = bcm56780_a0_lrd_flex_digest_controlt_flex_digest_hash_vh_init_b_map_entry
    },
    {
        .dest = {
            .kind = BCMLRD_MAP_PHYSICAL,
            .id = FLEX_DIGEST_HASH_VH_INIT_Cr,
        },
        .entries = 1,
        .entry = bcm56780_a0_lrd_flex_digest_controlt_flex_digest_hash_vh_init_c_map_entry
    },
};
const bcmlrd_map_t bcm56780_a0_lrd_flex_digest_control_map = {
    .src_id = FLEX_DIGEST_CONTROLt,
    .field_data = &bcm56780_a0_lrd_flex_digest_control_map_field_data,
    .groups = 3,
    .group  = bcm56780_a0_lrd_flex_digest_control_map_group,
    .table_attr = &bcm56780_a0_lrd_flex_digest_controlt_attr_group,
    .entry_ops = BCMLRD_MAP_TABLE_ENTRY_OPERATION_LOOKUP | BCMLRD_MAP_TABLE_ENTRY_OPERATION_TRAVERSE | BCMLRD_MAP_TABLE_ENTRY_OPERATION_INSERT | BCMLRD_MAP_TABLE_ENTRY_OPERATION_UPDATE | BCMLRD_MAP_TABLE_ENTRY_OPERATION_DELETE
};