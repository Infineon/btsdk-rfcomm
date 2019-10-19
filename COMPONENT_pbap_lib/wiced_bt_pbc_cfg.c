/*
 * Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/*****************************************************************************
**
**  Name:           wiced_bt_pbc_cfg.c
**
**  Description:    This file contains compile-time configurable constants
**                  for the BTA Phone Book Access Client.
**
**
*****************************************************************************/

#include "wiced_bt_pbc_int.h"

/* Realm Character Set */
#ifndef WICED_BT_PBC_REALM_CHARSET
#define WICED_BT_PBC_REALM_CHARSET   0       /* ASCII */
#endif

/* Specifies whether or not client's user id is required during obex authentication */
#ifndef WICED_BT_PBC_USERID_REQ
#define WICED_BT_PBC_USERID_REQ      FALSE
#endif

/* This value specifies the amount of time platform is willing to wait for abort
 * response before giving up (Disconnect Obex)
 */
#ifndef WICED_BT_PBC_STOP_ABORT_TIMEOUT
#define WICED_BT_PBC_STOP_ABORT_TIMEOUT     3 //3000    3 seconds
#endif

#ifndef WICED_BT_PBC_PCE_SERVICE_NAME
#define WICED_BT_PBC_PCE_SERVICE_NAME        "PBAP PCE"
#endif

const tWICED_BT_PBC_CFG wiced_bt_pbc_cfg =
{
    WICED_BT_PBC_REALM_CHARSET,
    WICED_BT_PBC_USERID_REQ,
    WICED_BT_PBC_PCE_SERVICE_NAME,   /* Client only */
    WICED_BT_PBC_STOP_ABORT_TIMEOUT  /* Client only */
};

tWICED_BT_PBC_CFG *p_wiced_bt_pbc_cfg = (tWICED_BT_PBC_CFG *)&wiced_bt_pbc_cfg;
