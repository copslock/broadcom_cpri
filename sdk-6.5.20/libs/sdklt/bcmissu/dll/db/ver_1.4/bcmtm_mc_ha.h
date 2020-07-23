/**************************************************************************
 *
 * DO NOT EDIT THIS FILE!
 * This file is auto-generated by HA parser from YAML formated file.
 * Edits to this file will be lost when it is regenerated.
 * Tool: bcmha/scripts/ha_yml_parser.py
 *
 * $Copyright:.$
 *
 **************************************************************************/
#ifndef BCMTM_MC_HA_H
#define BCMTM_MC_HA_H

#include <shr/shr_ha.h>

#define MAX_REPL_LIST_NHOP 6
#define MAX_REPL_HEAD_COUNT 147456
#define MAX_REPL_LIST_WORD_COUNT 4609
#define MAX_REPL_HEAD_WORD_COUNT 4609
#define MAX_REPL_GROUP_COUNT 16384

/*!
 * Multicast replication head information table.
 */
typedef struct {
   /*! Repl head index. */
   int index;
   /*! Replication count. */
   uint32_t repl_count;
   /*! Port Agg ID valid. */
   bool port_agg_valid;
   /*! Port Aggregate ID. */
   uint32_t port_agg_id;
   /*! MC Group ID valid. */
   bool mc_group_valid;
   /*! MC group id. */
   uint32_t mc_group_id;
   /*! Repl List Info chain. */
   uint32_t repl_chain;
   /*! Maximum number of configured replication per index. */
   int num_cfg_repl;
   /*! Previous head ID sharing the replication chain. */
   int prev_shared_head;
   /*! Next head ID sharing the replication chain. */
   int next_shared_head;
} bcmtm_mc_repl_head_info_t;

#define BCMTM_MC_REPL_HEAD_INFO_T_ID 0xbd0b0c1271e27acf

/*!
 * Multicast replication list entry.
 */
typedef struct {
   /*! The current index. */
   int entry_ptr;
   /*! Sparse Mode. */
   bool sparse_mode;
   /*! Lower 64 NHOP bitmap for the entry. */
   uint32_t nhop[MAX_REPL_LIST_NHOP];
   /*! MSB in multiples of 64. */
   uint32_t msb;
   /*! Sparse mode bitmap. */
   uint32_t mode_bitmap;
   /*! Remaining replication count. Invalid during a get operation. */
   int remaining_reps;
   /*! Next index in the chain. */
   int next_ptr;
} bcmtm_mc_repl_list_entry_t;

#define BCMTM_MC_REPL_LIST_ENTRY_T_ID 0xb9511fb26b308b52

/*!
 * Multicast replication list chain.
 */
typedef struct {
   /*! Repl List start ptr. */
   int start_ptr;
   /*! Reference count per Repl list chain. */
   int ref_count;
   /*! Hash value of all NHOP IDs in this repl list chain. */
   uint32_t hash;
   /*! Previous replication list chain */
   uint32_t prev;
   /*! Next replication list chain. */
   uint32_t next;
   /*! Previous shared head using this replication list. */
   uint32_t prev_shared_head;
   /*! Number of list used for the chain. */
   uint32_t num_list_used;
} bcmtm_repl_list_chain_t;

#define BCMTM_REPL_LIST_CHAIN_T_ID 0xfcbc75d8d36b80e1

/*!
 * Multicast device information.
 */
typedef struct {
   /*! Max Next Hop indices (EGR_L3_NEXT_HOP). */
   int max_mc_nhop;
   /*! Max Repl Head entries (REPL_HEAD_INFO). */
   int max_repl_head;
   /*! List of active Repl list chains. */
   uint32_t active_list_chain;
   /*! List of recycled Repl list chains. */
   uint32_t recycle_list_chain;
   /*! List of free Repl list chains. */
   uint32_t free_list_chain;
   /*! Size of the recycle list. */
   int recycle_list_size;
   /*! Number of entries allocated in the Repl List chain block. */
   int repl_chain_alloc_entries;
   /*! Count of initial list of chains in use. Once it is exhausted, free chains should be derived from free_list_chain. */
   int repl_chain_initial_use_count;
   /*! Used Replication list entries bitmap. */
   uint32_t used_list_bmp[MAX_REPL_LIST_WORD_COUNT];
   /*! Array of Repl Head information. */
   bcmtm_mc_repl_head_info_t head_info_list[MAX_REPL_HEAD_COUNT];
   /*! No of nexthops that can be programmed in on repl_list table index. */
   int nhops_per_repl_index;
   /*! Maximum replication count for a mc group. */
   int repl_count_mc_grp[MAX_REPL_GROUP_COUNT];
   /*! Maximum number of replication for a multicast group ID. */
   int max_packet_replications;
} bcmtm_mc_dev_info_t;

#define BCMTM_MC_DEV_INFO_T_ID 0x4f1b724f8cee310a

#endif /* BCMTM_MC_HA_H */