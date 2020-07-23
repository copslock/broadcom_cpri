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
/* Handler: bcm56780_a0_lta_bcmltx_sec_track_index_xfrm_handler */
#include <bcmlrd/bcmlrd_types.h>
#include <bcmltd/chip/bcmltd_id.h>
#include <bcmltx/bcmsec/bcmltx_sec_track_index.h>
#include <bcmdrd/chip/bcm56780_a0_enum.h>
#include <bcmlrd/chip/bcm56780_a0/bcm56780_a0_lrd_xfrm_field_desc.h>

extern const bcmltd_field_desc_t
bcm56780_a0_lrd_bcmltx_sec_track_index_src_field_desc_s0[];

extern const bcmltd_field_desc_t
bcm56780_a0_lrd_bcmltx_sec_track_index_src_field_desc_s1[];

extern const bcmltd_field_desc_t
bcm56780_a0_lrd_bcmltx_sec_track_index_sec_encrypt_thd_mask_dst_field_desc_d0[];

extern const bcmltd_field_desc_t
bcm56780_a0_lrd_bcmltx_sec_track_index_sec_macsec_sectag_ethertype_dst_field_desc_d0[];

static const
bcmltd_field_list_t
bcm56780_a0_lta_bcmltx_sec_track_index_src_list_s0 = {
    .field_num = 2,
    .field_array = bcm56780_a0_lrd_bcmltx_sec_track_index_src_field_desc_s0
};

static const
bcmltd_field_list_t
bcm56780_a0_lta_bcmltx_sec_track_index_src_list_s1 = {
    .field_num = 2,
    .field_array = bcm56780_a0_lrd_bcmltx_sec_track_index_src_field_desc_s1
};

static const bcmltd_field_list_t
bcm56780_a0_lrd_bcmltx_sec_track_index_sec_encrypt_thd_mask_dst_list_d0 = {
    .field_num = 1,
    .field_array = bcm56780_a0_lrd_bcmltx_sec_track_index_sec_encrypt_thd_mask_dst_field_desc_d0
};

static const bcmltd_field_list_t
bcm56780_a0_lrd_bcmltx_sec_track_index_sec_macsec_sectag_ethertype_dst_list_d0 = {
    .field_num = 1,
    .field_array = bcm56780_a0_lrd_bcmltx_sec_track_index_sec_macsec_sectag_ethertype_dst_field_desc_d0
};

static const uint32_t
bcm56780_a0_lta_bcmltx_sec_track_index_transform_src_s0[2] = {
    SEC_ENCRYPT_THD_MASKt_SEC_BLOCK_IDf,
    SEC_ENCRYPT_THD_MASKt_SEC_ENCRYPT_THD_MASK_IDf,
};

static const uint32_t
bcm56780_a0_lta_bcmltx_sec_track_index_transform_src_s1[2] = {
    SEC_MACSEC_SECTAG_ETHERTYPEt_SEC_BLOCK_IDf,
    SEC_MACSEC_SECTAG_ETHERTYPEt_SEC_MACSEC_SECTAG_ETHERTYPE_IDf,
};

static const uint32_t
bcm56780_a0_lrd_bcmltx_sec_track_index_sec_encrypt_thd_mask_transform_dst_d0[1] = {
    BCMLRD_FIELD_TRACK_INDEX,
};

static const uint32_t
bcm56780_a0_lrd_bcmltx_sec_track_index_sec_macsec_sectag_ethertype_transform_dst_d0[1] = {
    BCMLRD_FIELD_TRACK_INDEX,
};

static const bcmltd_generic_arg_t
bcm56780_a0_lta_bcmltx_sec_track_index_comp_data = {
    .sid       = SEC_ENCRYPT_THD_MASKt,
    .values    = 0,
    .value     = NULL,
    .user_data = NULL
};

static const bcmltd_generic_arg_t
bcm56780_a0_lta_bcmltx_sec_track_index_comp_data1 = {
    .sid       = SEC_MACSEC_SECTAG_ETHERTYPEt,
    .values    = 0,
    .value     = NULL,
    .user_data = NULL
};

static const bcmltd_transform_arg_t
bcm56780_a0_lta_bcmltx_sec_track_index_xfrm_handler_fwd_arg_s0_d0 = {
    .values      = 0,
    .value       = NULL,
    .tables      = 0,
    .table       = NULL,
    .fields      = 2,
    .field       = bcm56780_a0_lta_bcmltx_sec_track_index_transform_src_s0,
    .field_list  = &bcm56780_a0_lta_bcmltx_sec_track_index_src_list_s0,
    .rfields     = 1,
    .rfield      = bcm56780_a0_lrd_bcmltx_sec_track_index_sec_encrypt_thd_mask_transform_dst_d0,
    .rfield_list = &bcm56780_a0_lrd_bcmltx_sec_track_index_sec_encrypt_thd_mask_dst_list_d0,
    .comp_data   = &bcm56780_a0_lta_bcmltx_sec_track_index_comp_data
};

static const bcmltd_transform_arg_t
bcm56780_a0_lta_bcmltx_sec_track_index_xfrm_handler_rev_arg_s0_d0 = {
    .values      = 0,
    .value       = NULL,
    .tables      = 0,
    .table       = NULL,
    .fields      = 1,
    .field       = bcm56780_a0_lrd_bcmltx_sec_track_index_sec_encrypt_thd_mask_transform_dst_d0,
    .field_list  = &bcm56780_a0_lrd_bcmltx_sec_track_index_sec_encrypt_thd_mask_dst_list_d0,
    .rfields     = 2,
    .rfield      = bcm56780_a0_lta_bcmltx_sec_track_index_transform_src_s0,
    .rfield_list = &bcm56780_a0_lta_bcmltx_sec_track_index_src_list_s0,
    .comp_data   = &bcm56780_a0_lta_bcmltx_sec_track_index_comp_data
};

static const bcmltd_transform_arg_t
bcm56780_a0_lta_bcmltx_sec_track_index_xfrm_handler_fwd_arg_s1_d0 = {
    .values      = 0,
    .value       = NULL,
    .tables      = 0,
    .table       = NULL,
    .fields      = 2,
    .field       = bcm56780_a0_lta_bcmltx_sec_track_index_transform_src_s1,
    .field_list  = &bcm56780_a0_lta_bcmltx_sec_track_index_src_list_s1,
    .rfields     = 1,
    .rfield      = bcm56780_a0_lrd_bcmltx_sec_track_index_sec_macsec_sectag_ethertype_transform_dst_d0,
    .rfield_list = &bcm56780_a0_lrd_bcmltx_sec_track_index_sec_macsec_sectag_ethertype_dst_list_d0,
    .comp_data   = &bcm56780_a0_lta_bcmltx_sec_track_index_comp_data1
};

static const bcmltd_transform_arg_t
bcm56780_a0_lta_bcmltx_sec_track_index_xfrm_handler_rev_arg_s1_d0 = {
    .values      = 0,
    .value       = NULL,
    .tables      = 0,
    .table       = NULL,
    .fields      = 1,
    .field       = bcm56780_a0_lrd_bcmltx_sec_track_index_sec_macsec_sectag_ethertype_transform_dst_d0,
    .field_list  = &bcm56780_a0_lrd_bcmltx_sec_track_index_sec_macsec_sectag_ethertype_dst_list_d0,
    .rfields     = 2,
    .rfield      = bcm56780_a0_lta_bcmltx_sec_track_index_transform_src_s1,
    .rfield_list = &bcm56780_a0_lta_bcmltx_sec_track_index_src_list_s1,
    .comp_data   = &bcm56780_a0_lta_bcmltx_sec_track_index_comp_data1
};

const bcmltd_xfrm_handler_t
bcm56780_a0_lta_bcmltx_sec_track_index_xfrm_handler_fwd_s0_d0 = {
    .transform     = bcmltx_sec_track_index_transform,
    .arg           = &bcm56780_a0_lta_bcmltx_sec_track_index_xfrm_handler_fwd_arg_s0_d0
};

const bcmltd_xfrm_handler_t
bcm56780_a0_lta_bcmltx_sec_track_index_xfrm_handler_fwd_s1_d0 = {
    .transform     = bcmltx_sec_track_index_transform,
    .arg           = &bcm56780_a0_lta_bcmltx_sec_track_index_xfrm_handler_fwd_arg_s1_d0
};

const bcmltd_xfrm_handler_t
bcm56780_a0_lta_bcmltx_sec_track_index_xfrm_handler_rev_s0_d0 = {
    .transform     = bcmltx_sec_track_index_rev_transform,
    .arg           = &bcm56780_a0_lta_bcmltx_sec_track_index_xfrm_handler_rev_arg_s0_d0
};

const bcmltd_xfrm_handler_t
bcm56780_a0_lta_bcmltx_sec_track_index_xfrm_handler_rev_s1_d0 = {
    .transform     = bcmltx_sec_track_index_rev_transform,
    .arg           = &bcm56780_a0_lta_bcmltx_sec_track_index_xfrm_handler_rev_arg_s1_d0
};

