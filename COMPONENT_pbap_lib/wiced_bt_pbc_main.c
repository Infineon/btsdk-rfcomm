/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
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
**  Name:           wiced_bt_pbc_main.c
**
**  Description:    This file contains the phone book access client main functions
**                  and state machine.
**
**
*****************************************************************************/
#include <string.h>

#include "wiced_bt_pbc_int.h"
#include "wiced_bt_obex.h"
#include "wiced_bt_rfcomm.h"
/*****************************************************************************
** Constants and types
*****************************************************************************/

/* state machine states */
enum
{
  WICED_BT_PBC_IDLE_ST = 0,      /* Idle  */
  WICED_BT_PBC_W4_CONN_ST,       /* Waiting for an Obex connect response */
  WICED_BT_PBC_CONN_ST,          /* Connected - PBAP Session is active */
  WICED_BT_PBC_CLOSING_ST        /* Closing is in progress */
};

/* state machine action enumeration list */
enum
{
    WICED_BT_PBC_INIT_OPEN,
    WICED_BT_PBC_START_CLIENT,
    WICED_BT_PBC_STOP_CLIENT,
    WICED_BT_PBC_INIT_GETFILE,
    WICED_BT_PBC_LISTDIR,
    WICED_BT_PBC_CHDIR,
    WICED_BT_PBC_SEND_AUTHRSP,
    WICED_BT_PBC_CI_WRITE,
    WICED_BT_PBC_CI_OPEN,
    WICED_BT_PBC_OBX_CONN_RSP,
    WICED_BT_PBC_CLOSE,
    WICED_BT_PBC_OBX_ABORT_RSP,
    WICED_BT_PBC_OBX_PASSWORD,
    WICED_BT_PBC_OBX_TIMEOUT,
    WICED_BT_PBC_OBX_GET_RSP,
    WICED_BT_PBC_OBX_SETPATH_RSP,
    WICED_BT_PBC_TRANS_CMPL,
    WICED_BT_PBC_FREE_DB,
    WICED_BT_PBC_IGNORE_OBX,
    WICED_BT_PBC_FIND_SERVICE,
    WICED_BT_PBC_INITIALIZE,
    WICED_BT_PBC_CLOSE_COMPLETE,
    WICED_BT_PBC_SET_DISABLE,
    WICED_BT_PBC_ABORT,
    WICED_BT_PBC_RSP_TIMEOUT,
    WICED_BT_PBC_DISABLE_COMPLETE,
    WICED_BT_PBC_IGNORE
};

/* action function list */
const wiced_bt_pbc_action_t wiced_bt_pbc_action[] =
{
    wiced_bt_pbc_init_open,
    wiced_bt_pbc_start_client,
    wiced_bt_pbc_stop_client,
    wiced_bt_pbc_init_getfile,
    wiced_bt_pbc_listdir,
    wiced_bt_pbc_chdir,
    wiced_bt_pbc_send_authrsp,
    wiced_bt_pbc_ci_write,
    wiced_bt_pbc_ci_open,
    wiced_bt_pbc_obx_conn_rsp,
    wiced_bt_pbc_close,
    wiced_bt_pbc_obx_abort_rsp,
    wiced_bt_pbc_obx_password,
    wiced_bt_pbc_obx_timeout,
    wiced_bt_pbc_obx_get_rsp,
    wiced_bt_pbc_obx_setpath_rsp,
    wiced_bt_pbc_trans_cmpl,
    wiced_bt_pbc_free_db,
    wiced_bt_pbc_ignore_obx,
    wiced_bt_pbc_find_service,
    wiced_bt_pbc_initialize,
    wiced_bt_pbc_close_complete,
    wiced_bt_pbc_set_disable,
    wiced_bt_pbc_abort,
    wiced_bt_pbc_rsp_timeout,
    wiced_bt_pbc_disable_complete
};


/* state table information */
#define WICED_BT_PBC_ACTIONS             2       /* number of actions */
#define WICED_BT_PBC_NEXT_STATE          2       /* position of next state */
#define WICED_BT_PBC_NUM_COLS            3       /* number of columns in state tables */

/* state table for idle state */
static const UINT8 wiced_bt_pbc_st_idle[][WICED_BT_PBC_NUM_COLS] =
{
/* Event                            Action 1                    Action 2                Next state */
/* WICED_BT_PBC_API_DISABLE_EVT     */   {WICED_BT_PBC_SET_DISABLE,       WICED_BT_PBC_INITIALIZE,     WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_API_OPEN_EVT        */   {WICED_BT_PBC_INIT_OPEN,         WICED_BT_PBC_FIND_SERVICE,   WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_API_CLOSE_EVT       */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_API_GETFILE_EVT     */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_API_LISTDIR_EVT     */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_API_CHDIR_EVT       */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_API_AUTHRSP_EVT     */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_API_ABORT_EVT       */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_SDP_OK_EVT          */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_SDP_FAIL_EVT        */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_CI_WRITE_EVT        */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_CI_OPEN_EVT         */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_OBX_CONN_RSP_EVT    */   {WICED_BT_PBC_IGNORE_OBX,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_OBX_ABORT_RSP_EVT   */   {WICED_BT_PBC_IGNORE_OBX,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_OBX_TOUT_EVT        */   {WICED_BT_PBC_IGNORE_OBX,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_OBX_PASSWORD_EVT    */   {WICED_BT_PBC_IGNORE_OBX,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_OBX_CLOSE_EVT       */   {WICED_BT_PBC_IGNORE_OBX,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_OBX_GET_RSP_EVT     */   {WICED_BT_PBC_IGNORE_OBX,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_OBX_SETPATH_RSP_EVT */   {WICED_BT_PBC_IGNORE_OBX,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_OBX_CMPL_EVT        */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_CLOSE_CMPL_EVT      */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_DISABLE_CMPL_EVT    */   {WICED_BT_PBC_DISABLE_COMPLETE,  WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_RSP_TOUT_EVT        */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST}
};

/* state table for wait for authentication response state */
static const UINT8 wiced_bt_pbc_st_w4_conn[][WICED_BT_PBC_NUM_COLS] =
{
/* Event                            Action 1                    Action 2                Next state */
/* WICED_BT_PBC_API_DISABLE_EVT     */   {WICED_BT_PBC_SET_DISABLE,       WICED_BT_PBC_STOP_CLIENT,    WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_API_OPEN_EVT        */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_API_CLOSE_EVT       */   {WICED_BT_PBC_STOP_CLIENT,       WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_API_GETFILE_EVT     */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_API_LISTDIR_EVT     */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_API_CHDIR_EVT       */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_API_AUTHRSP_EVT     */   {WICED_BT_PBC_SEND_AUTHRSP,      WICED_BT_PBC_IGNORE,         WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_API_ABORT_EVT       */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_SDP_OK_EVT          */   {WICED_BT_PBC_FREE_DB,           WICED_BT_PBC_START_CLIENT,   WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_SDP_FAIL_EVT        */   {WICED_BT_PBC_FREE_DB,           WICED_BT_PBC_CLOSE,          WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_CI_WRITE_EVT        */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_CI_OPEN_EVT         */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_OBX_CONN_RSP_EVT    */   {WICED_BT_PBC_OBX_CONN_RSP,      WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_OBX_ABORT_RSP_EVT   */   {WICED_BT_PBC_IGNORE_OBX,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_OBX_TOUT_EVT        */   {WICED_BT_PBC_OBX_TIMEOUT,       WICED_BT_PBC_IGNORE,         WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_OBX_PASSWORD_EVT    */   {WICED_BT_PBC_OBX_PASSWORD,      WICED_BT_PBC_IGNORE,         WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_OBX_CLOSE_EVT       */   {WICED_BT_PBC_CLOSE,             WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_OBX_GET_RSP_EVT     */   {WICED_BT_PBC_IGNORE_OBX,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_OBX_SETPATH_RSP_EVT */   {WICED_BT_PBC_IGNORE_OBX,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_OBX_CMPL_EVT        */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_W4_CONN_ST},
/* WICED_BT_PBC_CLOSE_CMPL_EVT      */   {WICED_BT_PBC_CLOSE_COMPLETE,    WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_DISABLE_CMPL_EVT    */   {WICED_BT_PBC_DISABLE_COMPLETE,  WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_RSP_TOUT_EVT        */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST}

};

/* state table for open state */
static const UINT8 wiced_bt_pbc_st_connected[][WICED_BT_PBC_NUM_COLS] =
{
/* Event                            Action 1                    Action 2                Next state */
/* WICED_BT_PBC_API_DISABLE_EVT     */   {WICED_BT_PBC_SET_DISABLE,       WICED_BT_PBC_STOP_CLIENT,    WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_API_OPEN_EVT        */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_API_CLOSE_EVT       */   {WICED_BT_PBC_STOP_CLIENT,       WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_API_GETFILE_EVT     */   {WICED_BT_PBC_INIT_GETFILE,      WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_API_LISTDIR_EVT     */   {WICED_BT_PBC_LISTDIR,           WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_API_CHDIR_EVT       */   {WICED_BT_PBC_CHDIR,             WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_API_AUTHRSP_EVT     */   {WICED_BT_PBC_SEND_AUTHRSP,      WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_API_ABORT_EVT       */   {WICED_BT_PBC_ABORT,             WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_SDP_OK_EVT          */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_SDP_FAIL_EVT        */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_CI_WRITE_EVT        */   {WICED_BT_PBC_CI_WRITE,          WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_CI_OPEN_EVT         */   {WICED_BT_PBC_CI_OPEN,           WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_OBX_CONN_RSP_EVT    */   {WICED_BT_PBC_IGNORE_OBX,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_OBX_ABORT_RSP_EVT   */   {WICED_BT_PBC_OBX_ABORT_RSP,     WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_OBX_TOUT_EVT        */   {WICED_BT_PBC_OBX_TIMEOUT,       WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_OBX_PASSWORD_EVT    */   {WICED_BT_PBC_OBX_PASSWORD,      WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_OBX_CLOSE_EVT       */   {WICED_BT_PBC_CLOSE,             WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_OBX_GET_RSP_EVT     */   {WICED_BT_PBC_OBX_GET_RSP,       WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_OBX_SETPATH_RSP_EVT */   {WICED_BT_PBC_OBX_SETPATH_RSP,   WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_OBX_CMPL_EVT        */   {WICED_BT_PBC_TRANS_CMPL,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_CLOSE_CMPL_EVT      */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST},
/* WICED_BT_PBC_DISABLE_CMPL_EVT    */   {WICED_BT_PBC_DISABLE_COMPLETE,  WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_RSP_TOUT_EVT        */   {WICED_BT_PBC_RSP_TIMEOUT,       WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CONN_ST}
};

/* state table for closing state */
static const UINT8 wiced_bt_pbc_st_closing[][WICED_BT_PBC_NUM_COLS] =
{
/* Event                            Action 1                    Action 2                Next state */
/* WICED_BT_PBC_API_DISABLE_EVT     */   {WICED_BT_PBC_SET_DISABLE,       WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_API_OPEN_EVT        */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_API_CLOSE_EVT       */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_API_GETFILE_EVT     */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_API_LISTDIR_EVT     */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_API_CHDIR_EVT       */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_API_AUTHRSP_EVT     */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_API_ABORT_EVT       */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_SDP_OK_EVT          */   {WICED_BT_PBC_FREE_DB,           WICED_BT_PBC_CLOSE,          WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_SDP_FAIL_EVT        */   {WICED_BT_PBC_FREE_DB,           WICED_BT_PBC_CLOSE,          WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_CI_WRITE_EVT        */   {WICED_BT_PBC_CLOSE_COMPLETE,    WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_CI_OPEN_EVT         */   {WICED_BT_PBC_CLOSE_COMPLETE,    WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_OBX_CONN_RSP_EVT    */   {WICED_BT_PBC_IGNORE_OBX,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_OBX_ABORT_RSP_EVT   */   {WICED_BT_PBC_OBX_ABORT_RSP,     WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_OBX_TOUT_EVT        */   {WICED_BT_PBC_OBX_TIMEOUT,       WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_OBX_PASSWORD_EVT    */   {WICED_BT_PBC_IGNORE_OBX,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_OBX_CLOSE_EVT       */   {WICED_BT_PBC_CLOSE,             WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_OBX_GET_RSP_EVT     */   {WICED_BT_PBC_IGNORE_OBX,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_OBX_SETPATH_RSP_EVT */   {WICED_BT_PBC_IGNORE_OBX,        WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_OBX_CMPL_EVT        */   {WICED_BT_PBC_IGNORE,            WICED_BT_PBC_IGNORE,         WICED_BT_PBC_CLOSING_ST},
/* WICED_BT_PBC_CLOSE_CMPL_EVT      */   {WICED_BT_PBC_CLOSE_COMPLETE,    WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_DISABLE_CMPL_EVT    */   {WICED_BT_PBC_DISABLE_COMPLETE,  WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST},
/* WICED_BT_PBC_RSP_TOUT_EVT        */   {WICED_BT_PBC_RSP_TIMEOUT,       WICED_BT_PBC_IGNORE,         WICED_BT_PBC_IDLE_ST}
};

/* type for state table */
typedef const UINT8 (*tWICED_BT_PBC_ST_TBL)[WICED_BT_PBC_NUM_COLS];

/* state table */
const tWICED_BT_PBC_ST_TBL wiced_bt_pbc_st_tbl[] =
{
    wiced_bt_pbc_st_idle,
    wiced_bt_pbc_st_w4_conn,
    wiced_bt_pbc_st_connected,
    wiced_bt_pbc_st_closing
};

/*****************************************************************************
** Global data
*****************************************************************************/

/* PBC control block */
#if WICED_BT_DYNAMIC_MEMORY == FALSE
wiced_bt_pbc_cb_t  wiced_bt_pbc_cb;
#endif

#if WICED_BT_PBC_DEBUG == TRUE
static char *pbc_evt_code(wiced_bt_pbc_int_evt_t evt_code);
static char *pbc_state_code(wiced_bt_pbc_state_t state_code);
#endif

const wiced_bt_pbc_fs_cfg_t wiced_bt_pbc_fs_cfg =
{
    wiced_bt_pbc_fs_file_len,
    wiced_bt_pbc_fs_path_len,
    wiced_bt_pbc_fs_path_separator
};

wiced_bt_pbc_fs_cfg_t *p_wiced_bt_pbc_fs_cfg = (wiced_bt_pbc_fs_cfg_t *)&wiced_bt_pbc_fs_cfg;

/*******************************************************************************
**
** Function         wiced_bt_pbc_sm_execute
**
** Description      State machine event handling function for PBC
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_sm_execute(wiced_bt_pbc_cb_t *p_cb, UINT16 event, wiced_bt_pbc_data_t *p_data)
{
    tWICED_BT_PBC_ST_TBL     state_table;
    UINT8               action;
    int                 i;

    WICED_BT_TRACE("wiced_bt_pbc_sm_execute: ******ENTERING**** State 0x%02x [%s], Event [%s]", p_cb->state,
                      pbc_state_code(p_cb->state),
                      pbc_evt_code(event));

    /* look up the state table for the current state */
    state_table = wiced_bt_pbc_st_tbl[p_cb->state];

    event &= 0x00FF;

    /* set next state */
    p_cb->state = state_table[event][WICED_BT_PBC_NEXT_STATE];

    /* execute action functions */
    for (i = 0; i < WICED_BT_PBC_ACTIONS; i++)
    {
        if ((action = state_table[event][i]) != WICED_BT_PBC_IGNORE)
        {
            (*wiced_bt_pbc_action[action])(p_cb, p_data);
        }
        else
        {
            break;
        }
    }

    WICED_BT_TRACE("wiced_bt_pbc_sm_execute: ******LEAVING**** ");
}

/*****************************************************************************
**
**  Function:    wiced_bt_pbc_sdp_register()
**
**  Purpose:     Registers the PBC service with SDP
**
**  Parameters:
**
**
**  Returns:     void
**
*****************************************************************************/
static void wiced_bt_pbc_sdp_register (wiced_bt_pbc_cb_t *p_cb)
{
    UINT16              pbap_service = UUID_SERVCLASS_PBAP_PCE;
    UINT16              browse = UUID_SERVCLASS_PUBLIC_BROWSE_GROUP;
    BOOLEAN             status = FALSE;

//    if ((p_cb->sdp_handle = SDP_CreateRecord()) == 0)
//    {
//        WICED_BT_TRACE("PBC SDP: Unable to register PBAP PCE Service");
//        return;
//    }
//
//    /* add service class */
//    if (SDP_AddServiceClassIdList(p_cb->sdp_handle, 1, &pbap_service))
//    {
//        status = TRUE;  /* All mandatory fields were successful */
//
//        /* optional:  if name is not "", add a name entry */
//        if (p_wiced_bt_pbc_cfg->pce_name && p_wiced_bt_pbc_cfg->pce_name[0] != '\0')
//            SDP_AddAttribute(p_cb->sdp_handle,
//                             (UINT16)ATTR_ID_SERVICE_NAME,
//                             (UINT8)TEXT_STR_DESC_TYPE,
//                             (UINT32)(strlen(p_wiced_bt_pbc_cfg->pce_name) + 1),
//                             (UINT8 *)p_wiced_bt_pbc_cfg->pce_name);
//
//        /* Add in the Bluetooth Profile Descriptor List */
//        SDP_AddProfileDescriptorList(p_cb->sdp_handle,
//                                         UUID_SERVCLASS_PHONE_ACCESS,
//                                         WICED_BT_PBC_DEFAULT_VERSION);
//    } /* end of setting mandatory service class */
//
//    /* Make the service browseable */
//    SDP_AddUuidSequence (p_cb->sdp_handle, ATTR_ID_BROWSE_GROUP_LIST, 1, &browse);
//
//    if (!status)
//    {
//        SDP_DeleteRecord(p_cb->sdp_handle);
//        WICED_BT_TRACE("wiced_bt_pbc_sdp_register FAILED");
//        p_cb->sdp_handle = 0;
//    }
//    else
//    {
//        bta_sys_add_uuid(pbap_service); /* UUID_SERVCLASS_PBAP_PCE */
//        WICED_BT_TRACE("PBC:  SDP Registered (handle 0x%08x)", p_cb->sdp_handle);
//    }

    return;
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_api_enable
**
** Description      Handle an api enable event.  This function enables the PBC
**                  Client by opening an Obex/Rfcomm channel with a peer device.
**
**
** Returns          void
**
*******************************************************************************/
static void wiced_bt_pbc_api_enable(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    if (!p_cb->is_enabled)
    {
        /* initialize control block */
        memset(p_cb, 0, sizeof(wiced_bt_pbc_cb_t));

        /* store parameters */
        p_cb->p_cback = p_data->api_enable.p_cback;
        p_cb->p_data_cback = p_data->api_enable.p_data_cback;
        p_cb->app_id = p_data->api_enable.app_id;
        p_cb->fd = WICED_BT_PBC_INVALID_FD;
        p_cb->is_enabled = TRUE;
#if (defined(WICED_BT_PBAP_1_2_SUPPORTED) && WICED_BT_PBAP_1_2_SUPPORTED == TRUE)
        p_cb->local_features = p_data->api_enable.local_features;
#endif
        wiced_bt_pbc_sdp_register(p_cb);
    }

    /* callback with enable event */
    (*p_cb->p_cback)(WICED_BT_PBC_ENABLE_EVT, 0);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_hdl_event
**
** Description      PBC main event handling function.
**
**
** Returns          void
**
*******************************************************************************/
BOOLEAN wiced_bt_pbc_hdl_event(BT_HDR *p_msg)
{
    wiced_bt_pbc_cb_t *p_cb = &wiced_bt_pbc_cb;
#if WICED_BT_PBC_DEBUG == TRUE
    wiced_bt_pbc_state_t in_state = p_cb->state;
    WICED_BT_TRACE("PBC Event Handler: State 0x%02x [%s], Event [%s]", in_state,
                      pbc_state_code(in_state),
                      pbc_evt_code(p_msg->event));
#endif

    switch (p_msg->event)
    {
        case WICED_BT_PBC_API_ENABLE_EVT:
            wiced_bt_pbc_api_enable(p_cb, (wiced_bt_pbc_data_t *) p_msg);
            break;

        default:
            if (p_cb->is_enabled)
            {
                wiced_bt_pbc_sm_execute(p_cb, p_msg->event, (wiced_bt_pbc_data_t *) p_msg);

                if ( p_cb->state == WICED_BT_PBC_CONN_ST )
                {
                    if (( p_cb->pm_state == WICED_BT_PBC_PM_IDLE )
                      &&( p_cb->obx_oper != PBC_OP_NONE ))
                    {
                        /* inform power manager */
                        WICED_BT_TRACE("BTA PBC informs DM/PM busy state");
                       // bta_sys_busy( WICED_BT_ID_PBC, p_cb->app_id, p_cb->bd_addr );

                        // TODO ==> Add call back to the application
                        p_cb->pm_state = WICED_BT_PBC_PM_BUSY;
                    }
                    else if (( p_cb->pm_state == WICED_BT_PBC_PM_BUSY )
                           &&( p_cb->obx_oper == PBC_OP_NONE ))
                    {
                        /* inform power manager */
                        WICED_BT_TRACE("BTA PBC informs DM/PM idle state");
                        //bta_sys_idle( WICED_BT_ID_PBC ,p_cb->app_id, p_cb->bd_addr);

                        // TODO ==> Add call back to the application
                        p_cb->pm_state = WICED_BT_PBC_PM_IDLE;
                    }
                }
                else if ( p_cb->state == WICED_BT_PBC_IDLE_ST )
                {
                    /* initialize power management state */
                    p_cb->pm_state = WICED_BT_PBC_PM_BUSY;
                }

            }
            break;
    }

#if WICED_BT_PBC_DEBUG == TRUE
    if (in_state != p_cb->state)
    {
        WICED_BT_TRACE("PBC State Change: [%s] -> [%s] after Event [%s]",
                      pbc_state_code(in_state),
                      pbc_state_code(p_cb->state),
                      pbc_evt_code(p_msg->event));
    }
#endif

    return (TRUE);
}


/*****************************************************************************
**  Debug Functions
*****************************************************************************/
#if WICED_BT_PBC_DEBUG == TRUE

/*******************************************************************************
**
** Function         pbc_evt_code
**
** Description
**
** Returns          void
**
*******************************************************************************/
static char *pbc_evt_code(wiced_bt_pbc_int_evt_t evt_code)
{
    switch(evt_code)
    {
    case WICED_BT_PBC_API_DISABLE_EVT:
        return "WICED_BT_PBC_API_DISABLE_EVT";
    case WICED_BT_PBC_API_OPEN_EVT:
        return "WICED_BT_PBC_API_OPEN_EVT";
    case WICED_BT_PBC_API_CLOSE_EVT:
        return "WICED_BT_PBC_API_CLOSE_EVT";
    case WICED_BT_PBC_API_GETFILE_EVT:
        return "WICED_BT_PBC_API_GETFILE_EVT";
    case WICED_BT_PBC_API_LISTDIR_EVT:
        return "WICED_BT_PBC_API_LISTDIR_EVT";
    case WICED_BT_PBC_API_CHDIR_EVT:
        return "WICED_BT_PBC_API_CHDIR_EVT";
    case WICED_BT_PBC_API_AUTHRSP_EVT:
        return "WICED_BT_PBC_API_AUTHRSP_EVT";
    case WICED_BT_PBC_API_ABORT_EVT:
        return "WICED_BT_PBC_API_ABORT_EVT";
    case WICED_BT_PBC_SDP_OK_EVT:
        return "WICED_BT_PBC_SDP_OK_EVT";
    case WICED_BT_PBC_SDP_FAIL_EVT:
        return "WICED_BT_PBC_SDP_FAIL_EVT";
    case WICED_BT_PBC_CI_WRITE_EVT:
        return "WICED_BT_PBC_CI_WRITE_EVT";
    case WICED_BT_PBC_CI_OPEN_EVT:
        return "WICED_BT_PBC_CI_OPEN_EVT";
    case WICED_BT_PBC_OBX_CONN_RSP_EVT:
        return "WICED_BT_PBC_OBX_CONN_RSP_EVT";
    case WICED_BT_PBC_OBX_ABORT_RSP_EVT:
        return "WICED_BT_PBC_OBX_ABORT_RSP_EVT";
    case WICED_BT_PBC_OBX_TOUT_EVT:
        return "WICED_BT_PBC_OBX_TOUT_EVT";
    case WICED_BT_PBC_OBX_PASSWORD_EVT:
        return "WICED_BT_PBC_OBX_PASSWORD_EVT";
    case WICED_BT_PBC_OBX_CLOSE_EVT:
        return "WICED_BT_PBC_OBX_CLOSE_EVT";
    case WICED_BT_PBC_OBX_GET_RSP_EVT:
        return "WICED_BT_PBC_OBX_GET_RSP_EVT";
    case WICED_BT_PBC_OBX_SETPATH_RSP_EVT:
        return "WICED_BT_PBC_OBX_SETPATH_RSP_EVT";
    case WICED_BT_PBC_OBX_CMPL_EVT:
        return "WICED_BT_PBC_OBX_CMPL_EVT";
    case WICED_BT_PBC_DISABLE_CMPL_EVT:
        return "WICED_BT_PBC_DISABLE_CMPL_EVT";
    case WICED_BT_PBC_RSP_TOUT_EVT:
        return "WICED_BT_PBC_RSP_TOUT_EVT";
    case WICED_BT_PBC_API_ENABLE_EVT:
        return "WICED_BT_PBC_API_ENABLE_EVT";
    default:
        return "unknown PBC event code";
    }
}

/*******************************************************************************
**
** Function         pbc_state_code
**
** Description
**
** Returns          void
**
*******************************************************************************/
static char *pbc_state_code(wiced_bt_pbc_state_t state_code)
{
    switch(state_code)
    {
    case WICED_BT_PBC_IDLE_ST:
        return "WICED_BT_PBC_IDLE_ST";
    case WICED_BT_PBC_W4_CONN_ST:
        return "WICED_BT_PBC_W4_CONN_ST";
    case WICED_BT_PBC_CONN_ST:
        return "WICED_BT_PBC_CONN_ST";
    case WICED_BT_PBC_CLOSING_ST:
        return "WICED_BT_PBC_CLOSING_ST";
    default:
        return "unknown PBC state code";
    }
}

#endif  /* Debug Functions */
