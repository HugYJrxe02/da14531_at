/**
 ****************************************************************************************
 *
 * @file user_custs1_def.c
 *
 * @brief Custom Server 1 (CUSTS1) profile database definitions.
 *
 * Copyright (C) 2016-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @defgroup USER_CONFIG
 * @ingroup USER
 * @brief Custom server 1 (CUSTS1) profile database definitions.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "co_utils.h"
#include "prf_types.h"
#include "attm_db_128.h"
#include "user_custs1_def.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
//chen 2021-7-7
static const att_svc_desc128_t custs1_svc1                      = DEF_SVC1_UUID_128;

static const uint8_t CUST1_UUID_128[ATT_UUID_128_LEN]       = DEF_CUST1_UUID_128;


static struct att_char128_desc custs1_server_tx_char        = {ATT_CHAR_PROP_NTF,
                                                              {0, 0},
                                                              DEF_CUST1_UUID_128};

// Attribute specifications
static const uint16_t att_decl_svc       = ATT_DECL_PRIMARY_SERVICE;
static const uint16_t att_decl_char      = ATT_DECL_CHARACTERISTIC;
static const uint16_t att_desc_cfg       = ATT_DESC_CLIENT_CHAR_CFG;
static const uint16_t att_desc_user_desc = ATT_DESC_CHAR_USER_DESCRIPTION;

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
//chen 2021-7-7
const uint8_t custs1_services[]  = {SVC1_IDX_SVC,CUSTS1_IDX_NB};
const uint8_t custs1_services_size = ARRAY_LEN(custs1_services) - 1;
const uint16_t custs1_att_max_nb = CUSTS1_IDX_NB;

/// Full CUSTS1 Database Description - Used to add attributes into the database
const struct attm_desc_128 custs1_att_db[CUSTS1_IDX_NB] =
{
    /*************************
     * Service 1 configuration
     *************************
     */
    // chen 2021-7-7
    [SVC1_IDX_SVC]                     = {(uint8_t*)&att_decl_svc, ATT_UUID_128_LEN, PERM(RD, ENABLE),
                                            sizeof(custs1_svc1), sizeof(custs1_svc1), (uint8_t*)&custs1_svc1},
    // Server  Characteristic Declaration
    [CUST1_IDX_CHAR]          = {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            sizeof(custs1_server_tx_char), sizeof(custs1_server_tx_char), (uint8_t*)&custs1_server_tx_char},

    // Characteristic Value 
    [CUST1_IDX_VAL]           = {CUST1_UUID_128, ATT_UUID_128_LEN, PERM(NTF, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE) ,
                                            DEF_CUST1_CHAR_LEN, 0, NULL},

    //  Client Characteristic Configuration Descriptor
    [CUST1_IDX_NTF_CFG]       = {(uint8_t*)&att_desc_cfg, ATT_UUID_16_LEN, PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE) | PERM(WRITE_COMMAND, ENABLE),
                                            sizeof(uint16_t), 0, NULL},

    // Server  Characteristic User Description
    [CUST1_IDX_USER_DESC]     = {(uint8_t*)&att_desc_user_desc, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            sizeof(CUST1_USER_DESC) - 1, sizeof(CUST1_USER_DESC) - 1, CUST1_USER_DESC},

};

/// @} USER_CONFIG
