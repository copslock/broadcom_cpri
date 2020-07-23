/*******************************************************************************
 *
 * DO NOT EDIT THIS FILE!
 * This file is auto-generated by fltg from Logical Table mapping files.
 *
 * Tool: $SDK/INTERNAL/fltg/bin/fltg
 *
 * Edits to this file will be lost when it is regenerated.
 *
 * This license is set out in https://raw.githubusercontent.com/Broadcom-Network-Switching-Software/OpenBCM/master/Legal/LICENSE file.
 * 
 * Copyright 2007-2020 Broadcom Inc. All rights reserved.
 */
/* Logical Table Adaptor for component bcmltx */
/* Handler: bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_xfrm_handler */
#include <bcmlrd/bcmlrd_types.h>
#include <bcmltd/chip/bcmltd_id.h>
#include <bcmltx/bcmltx_field_demux.h>
#include <bcmdrd/chip/bcm56880_a0_enum.h>
#include <bcmlrd/chip/bcm56880_a0/dna_4_6_6/bcm56880_a0_dna_4_6_6_lrd_xfrm_field_desc.h>


extern const bcmltd_field_desc_t
bcm56880_a0_dna_4_6_6_lrd_field_demux_r_ing_outer_tpid_0t_tpidf_1_src_field_desc_s1[];

static const bcmltd_field_desc_t
bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_dst_field_desc[18] = {
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 300,
        .field_idx = 0,
        .minbit    = 186,
        .maxbit    = 193,
        .entry_idx = 0,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY0[191:184]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 301,
        .field_idx = 0,
        .minbit    = 178,
        .maxbit    = 185,
        .entry_idx = 0,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY0[183:176]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 302,
        .field_idx = 0,
        .minbit    = 186,
        .maxbit    = 193,
        .entry_idx = 1,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY1[191:184]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 303,
        .field_idx = 0,
        .minbit    = 178,
        .maxbit    = 185,
        .entry_idx = 1,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY1[183:176]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 304,
        .field_idx = 0,
        .minbit    = 138,
        .maxbit    = 145,
        .entry_idx = 2,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY2[143:136]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 305,
        .field_idx = 0,
        .minbit    = 130,
        .maxbit    = 137,
        .entry_idx = 2,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY2[135:128]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 306,
        .field_idx = 0,
        .minbit    = 138,
        .maxbit    = 145,
        .entry_idx = 3,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY3[143:136]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 307,
        .field_idx = 0,
        .minbit    = 130,
        .maxbit    = 137,
        .entry_idx = 3,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY3[135:128]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 308,
        .field_idx = 0,
        .minbit    = 138,
        .maxbit    = 145,
        .entry_idx = 4,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY4[143:136]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 309,
        .field_idx = 0,
        .minbit    = 130,
        .maxbit    = 137,
        .entry_idx = 4,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY4[135:128]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 310,
        .field_idx = 0,
        .minbit    = 186,
        .maxbit    = 193,
        .entry_idx = 5,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY5[191:184]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 311,
        .field_idx = 0,
        .minbit    = 178,
        .maxbit    = 185,
        .entry_idx = 5,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY5[183:176]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 312,
        .field_idx = 0,
        .minbit    = 122,
        .maxbit    = 129,
        .entry_idx = 6,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY6[127:120]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 313,
        .field_idx = 0,
        .minbit    = 114,
        .maxbit    = 121,
        .entry_idx = 6,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY6[119:112]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 314,
        .field_idx = 0,
        .minbit    = 122,
        .maxbit    = 129,
        .entry_idx = 7,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY7[127:120]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 315,
        .field_idx = 0,
        .minbit    = 114,
        .maxbit    = 121,
        .entry_idx = 7,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY7[119:112]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 316,
        .field_idx = 0,
        .minbit    = 122,
        .maxbit    = 129,
        .entry_idx = 8,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY8[127:120]",
        .reserved  = 0
    },
    {
        .field_id  = BCMLTD_INTERNAL_FIELD_BASE + 317,
        .field_idx = 0,
        .minbit    = 114,
        .maxbit    = 121,
        .entry_idx = 8,
        .sym       = "IPARSER2_HME_STAGE0_TCAM_ONLYm.KEYf.ENTRY8[119:112]",
        .reserved  = 0
    },
};

static const
bcmltd_field_list_t
bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_src_list_s0 = {
    .field_num = 18,
    .field_array = bcm56880_a0_dna_4_6_6_lrd_field_demux_r_ing_outer_tpid_0t_tpidf_1_src_field_desc_s1
};

static const bcmltd_field_list_t
bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_dst_list_d0 = {
    .field_num = 18,
    .field_array = bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_dst_field_desc
};

static const uint32_t
bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_transform_src_s0[1] = {
    BCM56880_A0_DNA_4_6_6_R_ING_OUTER_TPID_0t_TPIDf,
};

static const uint32_t
bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_transform_dst_d0[18] = {
    BCMLTD_INTERNAL_FIELD_BASE + 300,
    BCMLTD_INTERNAL_FIELD_BASE + 301,
    BCMLTD_INTERNAL_FIELD_BASE + 302,
    BCMLTD_INTERNAL_FIELD_BASE + 303,
    BCMLTD_INTERNAL_FIELD_BASE + 304,
    BCMLTD_INTERNAL_FIELD_BASE + 305,
    BCMLTD_INTERNAL_FIELD_BASE + 306,
    BCMLTD_INTERNAL_FIELD_BASE + 307,
    BCMLTD_INTERNAL_FIELD_BASE + 308,
    BCMLTD_INTERNAL_FIELD_BASE + 309,
    BCMLTD_INTERNAL_FIELD_BASE + 310,
    BCMLTD_INTERNAL_FIELD_BASE + 311,
    BCMLTD_INTERNAL_FIELD_BASE + 312,
    BCMLTD_INTERNAL_FIELD_BASE + 313,
    BCMLTD_INTERNAL_FIELD_BASE + 314,
    BCMLTD_INTERNAL_FIELD_BASE + 315,
    BCMLTD_INTERNAL_FIELD_BASE + 316,
    BCMLTD_INTERNAL_FIELD_BASE + 317,
};

static const bcmltd_generic_arg_t
bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_comp_data = {
    .sid       = BCM56880_A0_DNA_4_6_6_R_ING_OUTER_TPID_0t,
    .values    = 0,
    .value     = NULL,
    .user_data = NULL
};

static const bcmltd_transform_arg_t
bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_xfrm_handler_fwd_arg_s0_d0 = {
    .values      = 0,
    .value       = NULL,
    .tables      = 0,
    .table       = NULL,
    .fields      = 1,
    .field       = bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_transform_src_s0,
    .field_list  = &bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_src_list_s0,
    .rfields     = 18,
    .rfield      = bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_transform_dst_d0,
    .rfield_list = &bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_dst_list_d0,
    .comp_data   = &bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_comp_data
};

const bcmltd_xfrm_handler_t
bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_xfrm_handler_fwd_s0_d0 = {
    .transform     = bcmltx_field_demux_transform,
    .arg           = &bcm56880_a0_dna_4_6_6_lta_bcmltx_field_demux_r_ing_outer_tpid_0t_tpidf_1_xfrm_handler_fwd_arg_s0_d0
};

