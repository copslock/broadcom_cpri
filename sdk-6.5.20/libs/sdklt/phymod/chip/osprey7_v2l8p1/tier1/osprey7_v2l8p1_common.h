/***********************************************************************************
 *                                                                                 *
 * Copyright: (c) 2020 Broadcom.                                                   *
 * Broadcom Proprietary and Confidential. All rights reserved.                     *
 */

/********************************************************************************
 ********************************************************************************
 *                                                                              *
 *  Revision      :   *
 *                                                                              *
 *  Description   :  Defines and Enumerations required by Serdes APIs           *
 */

/** @file osprey7_v2l8p1_common.h
 * Defines and Enumerations shared across MER7/BHK7/OSP7 APIs but NOT uCode
 */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef OSPREY7_V2L8P1_API_COMMON_H
#define OSPREY7_V2L8P1_API_COMMON_H

#include "osprey7_v2l8p1_ipconfig.h"

/** Macro to determine sign of a value */
#define sign(x) ((x>=0) ? 1 : -1)

#define UCODE_MAX_SIZE  (160*1024)    /* 160K CODE RAM */

/*
 * IP-Specific Iteration Bounds
 */
# define NUM_PLLS 1

#endif
#ifdef __cplusplus
}
#endif
