/**
 ****************************************************************************************
 *
 * @file user_custs1_def.h
 *
 * @brief Custom Server 1 (CUSTS1) profile database definitions.
 *
 * Copyright (C) 2016-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _USER_CUSTS1_DEF_H_
#define _USER_CUSTS1_DEF_H_

/**
 ****************************************************************************************
 * @defgroup USER_CONFIG
 * @ingroup USER
 * @brief Custom Server 1 (CUSTS1) profile database definitions.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "attm_db_128.h"

/*
 * DEFINES
 ****************************************************************************************
 */
//  chen 2021-7-7
// Service 1 of the custom server 1
#define DEF_SVC1_UUID_128             {0x79, 0x41, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E}

#define DEF_CUST1_CP_TX_UUID_128      {0x79, 0x41, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E}
#define DEF_CUST1_CP_RX_UUID_128      {0x79, 0x41, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E}

//length = MTU - 3, change it when increasing MTU or use DLE
#define DEF_CUST1_CP_TX_CHAR_LEN      (247 - 3)
#define DEF_CUST1_CP_RX_CHAR_LEN      (247 - 3)

#define CUST1_CP_TX_USER_DESC     "CP TX Data"
#define CUST1_CP_RX_USER_DESC     "CP RX Data"
/// Custom1 Service Data Base Characteristic enum
enum
{
    // Custom Service 1
    SVC1_IDX_SVC = 0,

    CUST1_IDX_CP_RX_CHAR,
    CUST1_IDX_CP_RX_VAL,
    CUST1_IDX_CP_RX_USER_DESC,
	
	  CUST1_IDX_CP_TX_CHAR,
    CUST1_IDX_CP_TX_VAL,
    CUST1_IDX_CP_TX_NTF_CFG,
    CUST1_IDX_CP_TX_USER_DESC,

    CUSTS1_IDX_NB
};

/// @} USER_CONFIG

#endif // _USER_CUSTS1_DEF_H_
