/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
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
**  Name:           wiced_bt_mce_main.c
**
**  Description:    This file contains the Message Access Client main functions
**                  and state machine.
**
*****************************************************************************/
#include <string.h>

#include "wiced.h"
#include "wiced_bt_mce_int.h"

/*****************************************************************************
** Message Access Client State Table
*****************************************************************************/
/*****************************************************************************
** Constants and types
*****************************************************************************/
/* state machine action enumeration list for MCE */
/* The order of this enumeration must be the same as wiced_mce_ma_act_tbl[] */
enum
{
    WICED_MCE_MA_INIT_SDP,
    WICED_MCE_MA_START_CLIENT,
    WICED_MCE_MA_STOP_CLIENT,
    WICED_MCE_MA_OBX_CONN_RSP,
    WICED_MCE_MA_CLOSE,
    WICED_MCE_MA_ABORT,
    WICED_MCE_MA_PUSH_MSG,
    WICED_MCE_MA_PUT_RSP,
    WICED_MCE_MA_CLOSE_COMPL,
    WICED_MCE_MA_IGNORE_OBX,
    WICED_MCE_MA_FREE_DB,
    WICED_MCE_MA_NOTIF_REG,
    WICED_MCE_MA_RSP_TOUT,
    WICED_MCE_MA_UPD_INBOX,
    WICED_MCE_MA_CHDIR,
    WICED_MCE_MA_LIST,
    WICED_MCE_MA_GET_MSG,
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
    WICED_MCE_MA_GET_MAS_INS_INFO,
#endif
    WICED_MCE_MA_SET_STS,
    WICED_MCE_MA_TRANS_CMPL,
    WICED_MCE_MA_OBX_ABORT_RSP,
    WICED_MCE_MA_OBX_SETPATH_RSP,
    WICED_MCE_MA_GET_RSP,
    WICED_MCE_MA_IGNORE
};

typedef void (*tWICED_MCE_MA_ACTION)(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);

static const tWICED_MCE_MA_ACTION wiced_mce_ma_action[] =
{
    wiced_mce_ma_init_sdp,
    wiced_mce_ma_start_client,
    wiced_mce_ma_stop_client,
    wiced_mce_ma_obx_conn_rsp,
    wiced_mce_ma_close,
    wiced_mce_ma_abort,
    wiced_mce_ma_push_msg,
    wiced_mce_ma_put_rsp,
    wiced_mce_ma_close_compl,
    wiced_mce_ma_ignore_obx,
    wiced_mce_ma_free_db,
    wiced_mce_ma_notif_reg,
    wiced_mce_ma_rsp_tout,
    wiced_mce_ma_upd_inbox,
    wiced_mce_ma_chdir,
    wiced_mce_ma_list,
    wiced_mce_ma_get_msg,
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
    wiced_mce_ma_get_mas_ins_info,
#endif
    wiced_mce_ma_set_sts,
    wiced_mce_ma_trans_cmpl,
    wiced_mce_obx_abort_rsp,
    wiced_mce_ma_obx_setpath_rsp,
    wiced_mce_ma_obx_get_rsp
};


/* state table information */
#define WICED_MCE_MA_ACTIONS             1       /* number of actions */
#define WICED_MCE_MA_ACTION_COL          0       /* position of action */
#define WICED_MCE_MA_NEXT_STATE          1       /* position of next state */
#define WICED_MCE_MA_NUM_COLS            2       /* number of columns in state tables */

/* state table for idle state */
static const UINT8 wiced_mce_ma_st_idle[][WICED_MCE_MA_NUM_COLS] =
{
/* Event                                      Action 1                       Next state */
/* WICED_MCE_MA_API_OPEN       */             {WICED_MCE_MA_INIT_SDP,        WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_MA_API_CLOSE      */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_API_NOTIF_REG_EVT */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_API_UPD_INBOX_EVT */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_API_CHDIR_EVT     */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_API_LIST_EVT      */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_API_GET_MSG_EVT   */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_IDLE_ST},
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
/* WICED_MCE_API_GET_MAS_INS_INFO_EVT */      {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_IDLE_ST},
#endif
/* WICED_MCE_API_SET_STS_EVT   */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_API_PUSH_EVT      */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_API_ABORT_EVT     */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_SDP_FAIL_EVT      */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_SDP_OK_EVT        */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_MA_OBX_CONN_RSP   */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_MA_OBX_PUT_RSP    */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_MA_OBX_GET_RSP    */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_MA_OBX_SETPATH_RSP*/             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_MA_OBX_TOUT_EVT   */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_MA_OBX_CLOSE      */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_MA_OBX_ABORT_RSP  */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_MA_OBX_CMPL_EVT   */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_MA_CLOSE_CMPL_EVT */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_RSP_TOUT_EVT      */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_IDLE_ST}
};

/* state table for wait for authentication state */
static const UINT8 wiced_mce_ma_st_w4_conn[][WICED_MCE_MA_NUM_COLS] =
{
/* Event                                      Action 1                       Next state */
/* WICED_MCE_MA_API_OPEN       */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_MA_API_CLOSE      */             {WICED_MCE_MA_STOP_CLIENT,     WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_API_NOTIF_REG_EVT */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_API_UPD_INBOX_EVT */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_API_CHDIR_EVT     */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_API_LIST_EVT      */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_API_GET_MSG_EVT   */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_W4_CONN_ST},
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
/* WICED_MCE_API_GET_MAS_INS_INFO_EVT */      {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_W4_CONN_ST},
#endif
/* WICED_MCE_API_SET_STS_EVT   */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_API_PUSH_EVT      */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_API_ABORT_EVT     */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_SDP_FAIL_EVT      */             {WICED_MCE_MA_FREE_DB,         WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_SDP_OK_EVT        */             {WICED_MCE_MA_START_CLIENT,    WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_MA_OBX_CONN_RSP   */             {WICED_MCE_MA_OBX_CONN_RSP,    WICED_MCE_MA_CONN_ST},
/* WICED_MCE_MA_OBX_PUT_RSP    */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_MA_OBX_GET_RSP    */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_MA_OBX_SETPATH_RSP*/             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_MA_OBX_TOUT_EVT   */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_MA_OBX_CLOSE      */             {WICED_MCE_MA_CLOSE,           WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_MA_OBX_ABORT_RSP  */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_MA_OBX_CMPL_EVT   */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_MA_CLOSE_CMPL_EVT */             {WICED_MCE_MA_CLOSE_COMPL,     WICED_MCE_MA_W4_CONN_ST},
/* WICED_MCE_RSP_TOUT_EVT      */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_W4_CONN_ST}
};

/* state table for connected state */
static const UINT8 wiced_mce_ma_st_connected[][WICED_MCE_MA_NUM_COLS] =
{
/* Event                                      Action 1                       Next state */
/* WICED_MCE_MA_API_OPEN       */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CONN_ST},
/* WICED_MCE_MA_API_CLOSE      */             {WICED_MCE_MA_STOP_CLIENT,     WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_API_NOTIF_REG_EVT */             {WICED_MCE_MA_NOTIF_REG,       WICED_MCE_MA_CONN_ST},
/* WICED_MCE_API_UPD_INBOX_EVT */             {WICED_MCE_MA_UPD_INBOX,       WICED_MCE_MA_CONN_ST},
/* WICED_MCE_API_CHDIR_EVT     */             {WICED_MCE_MA_CHDIR,           WICED_MCE_MA_CONN_ST},
/* WICED_MCE_API_LIST_EVT      */             {WICED_MCE_MA_LIST,            WICED_MCE_MA_CONN_ST},
/* WICED_MCE_API_GET_MSG_EVT   */             {WICED_MCE_MA_GET_MSG,         WICED_MCE_MA_CONN_ST},
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
/* WICED_MCE_API_GET_MAS_INS_INFO_EVT */      {WICED_MCE_MA_GET_MAS_INS_INFO,WICED_MCE_MA_CONN_ST},
#endif
/* WICED_MCE_API_SET_STS_EVT   */             {WICED_MCE_MA_SET_STS,         WICED_MCE_MA_CONN_ST},
/* WICED_MCE_API_PUSH_EVT      */             {WICED_MCE_MA_PUSH_MSG,        WICED_MCE_MA_CONN_ST},
/* WICED_MCE_API_ABORT_EVT     */             {WICED_MCE_MA_ABORT,           WICED_MCE_MA_CONN_ST},
/* WICED_MCE_SDP_FAIL_EVT      */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CONN_ST},
/* WICED_MCE_SDP_OK_EVT        */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CONN_ST},
/* WICED_MCE_MA_OBX_CONN_RSP   */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_CONN_ST},
/* WICED_MCE_MA_OBX_PUT_RSP    */             {WICED_MCE_MA_PUT_RSP,         WICED_MCE_MA_CONN_ST},
/* WICED_MCE_MA_OBX_GET_RSP    */             {WICED_MCE_MA_GET_RSP,         WICED_MCE_MA_CONN_ST},
/* WICED_MCE_MA_OBX_SETPATH_RSP*/             {WICED_MCE_MA_OBX_SETPATH_RSP, WICED_MCE_MA_CONN_ST},
/* WICED_MCE_MA_OBX_TOUT_EVT   */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_CONN_ST},
/* WICED_MCE_MA_OBX_CLOSE      */             {WICED_MCE_MA_CLOSE,           WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_MA_OBX_ABORT_RSP  */             {WICED_MCE_MA_OBX_ABORT_RSP,   WICED_MCE_MA_CONN_ST},
/* WICED_MCE_MA_OBX_CMPL_EVT   */             {WICED_MCE_MA_TRANS_CMPL,      WICED_MCE_MA_CONN_ST},
/* WICED_MCE_MA_CLOSE_CMPL_EVT */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CONN_ST},
/* WICED_MCE_RSP_TOUT_EVT      */             {WICED_MCE_MA_RSP_TOUT,        WICED_MCE_MA_CLOSING_ST}
};

/* state table for closing state */
static const UINT8 wiced_mce_ma_st_closing[][WICED_MCE_MA_NUM_COLS] =
{
/* Event                                      Action 1                       Next state */
/* WICED_MCE_MA_API_OPEN       */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_MA_API_CLOSE      */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_API_NOTIF_REG_EVT */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_API_UPD_INBOX_EVT */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_API_CHDIR_EVT     */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_API_LIST_EVT      */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_API_GET_MSG_EVT   */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CLOSING_ST},
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
/* WICED_MCE_API_GET_MAS_INS_INFO_EVT */      {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CLOSING_ST},
#endif
/* WICED_MCE_API_SET_STS_EVT   */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_API_PUSH_EVT      */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_API_ABORT_EVT     */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_SDP_FAIL_EVT      */             {WICED_MCE_MA_FREE_DB,         WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_SDP_OK_EVT        */             {WICED_MCE_MA_FREE_DB,         WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_MA_OBX_CONN_RSP   */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_MA_OBX_PUT_RSP    */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_MA_OBX_GET_RSP    */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_MA_OBX_SETPATH_RSP*/             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_MA_OBX_TOUT_EVT   */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_MA_OBX_CLOSE      */             {WICED_MCE_MA_CLOSE,           WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_MA_OBX_ABORT_RSP  */             {WICED_MCE_MA_IGNORE_OBX,      WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_MA_OBX_CMPL_EVT   */             {WICED_MCE_MA_IGNORE,          WICED_MCE_MA_CLOSING_ST},
/* WICED_MCE_MA_CLOSE_CMPL_EVT */             {WICED_MCE_MA_CLOSE_COMPL,     WICED_MCE_MA_IDLE_ST},
/* WICED_MCE_RSP_TOUT_EVT      */             {WICED_MCE_MA_RSP_TOUT,        WICED_MCE_MA_CLOSING_ST}
};

/* type for state table */
typedef const UINT8 (*wiced_mce_ma_st_tbl_t)[WICED_MCE_MA_NUM_COLS];

/* state table */
const wiced_mce_ma_st_tbl_t wiced_mce_ma_st_tbl[] =
{
    wiced_mce_ma_st_idle,
    wiced_mce_ma_st_w4_conn,
    wiced_mce_ma_st_connected,
    wiced_mce_ma_st_closing
};

/*****************************************************************************
** Message Notification Server State Table
*****************************************************************************/
/* state machine action enumeration list for MNS */
enum
{
    WICED_MCE_MN_INT_CLOSE,
    WICED_MCE_MN_OBX_CONNECT,
    WICED_MCE_MN_OBX_DISC,
    WICED_MCE_MN_OBX_CLOSE,
    WICED_MCE_MN_OBX_ABORT,
    WICED_MCE_MN_OBX_PUT,
    WICED_MCE_MN_OBX_ACTION,
    WICED_MCE_MN_CONN_ERR_RSP,
    WICED_MCE_MN_IGNORE_OBX,
    WICED_MCE_MN_IGNORE
};

/* type for action functions */
typedef void (*tWICED_MCE_MN_ACTION)(UINT8 scb_idx, wiced_mce_data_t *p_data);

/* action function list for MNS */
const tWICED_MCE_MN_ACTION wiced_mce_mn_action[] =
{
    wiced_mce_mn_int_close,
    wiced_mce_mn_obx_connect,
    wiced_mce_mn_obx_disc,
    wiced_mce_mn_obx_close,
    wiced_mce_mn_obx_abort,
    wiced_mce_mn_obx_put,
    wiced_mce_mn_obx_action,
    wiced_mce_mn_conn_err_rsp,
    wiced_mce_mn_ignore_obx
};

/* state table information */
#define WICED_MCE_MN_ACTIONS             1       /* number of actions */
#define WICED_MCE_MN_NEXT_STATE          1       /* position of next state */
#define WICED_MCE_MN_NUM_COLS            2       /* number of columns in state tables */

/* state table for MNS idle state */
static const UINT8 wiced_mce_mn_st_idle[][WICED_MCE_MN_NUM_COLS] =
{
/* Event                               Action 1                       Next state */
/* WICED_MCE_MN_INT_CLOSE_EVT     */   {WICED_MCE_MN_IGNORE,          WICED_MCE_MN_IDLE_ST},
/* WICED_MCE_MN_OBX_CONN_REQ_EVT  */   {WICED_MCE_MN_IGNORE_OBX,      WICED_MCE_MN_IDLE_ST},
/* WICED_MCE_MN_OBX_PUT_EVT       */   {WICED_MCE_MN_IGNORE_OBX,      WICED_MCE_MN_IDLE_ST},
/* WICED_MCE_MN_OBX_ACTION_EVT    */   {WICED_MCE_MN_IGNORE_OBX,      WICED_MCE_MN_IDLE_ST},
/* WICED_MCE_MN_OBX_ABORT_EVT     */   {WICED_MCE_MN_IGNORE_OBX,      WICED_MCE_MN_IDLE_ST},
/* WICED_MCE_MN_OBX_DISC_EVT      */   {WICED_MCE_MN_IGNORE_OBX,      WICED_MCE_MN_IDLE_ST},
/* WICED_MCE_MN_OBX_CLOSE_EVT     */   {WICED_MCE_MN_IGNORE_OBX,      WICED_MCE_MN_IDLE_ST}
};

/* state table for obex/rfcomm connection state */
static const UINT8 wiced_mce_mn_st_listen[][WICED_MCE_MN_NUM_COLS] =
{
/* Event                               Action 1                    Next state */
/* WICED_MCE_MN_INT_CLOSE_EVT     */   {WICED_MCE_MN_IGNORE,       WICED_MCE_MN_LISTEN_ST},
/* WICED_MCE_MN_OBX_CONN_REQ_EVT  */   {WICED_MCE_MN_OBX_CONNECT,  WICED_MCE_MN_CONN_ST},
/* WICED_MCE_MN_OBX_PUT_EVT       */   {WICED_MCE_MN_IGNORE_OBX,   WICED_MCE_MN_LISTEN_ST},
/* WICED_MCE_MN_OBX_ACTION_EVT    */   {WICED_MCE_MN_IGNORE_OBX,   WICED_MCE_MN_LISTEN_ST},
/* WICED_MCE_MN_OBX_ABORT_EVT     */   {WICED_MCE_MN_IGNORE_OBX,   WICED_MCE_MN_LISTEN_ST},
/* WICED_MCE_MN_OBX_DISC_EVT      */   {WICED_MCE_MN_IGNORE_OBX,   WICED_MCE_MN_LISTEN_ST},
/* WICED_MCE_MN_OBX_CLOSE_EVT     */   {WICED_MCE_MN_IGNORE_OBX,   WICED_MCE_MN_LISTEN_ST}
};

/* state table for open state */
static const UINT8 wiced_mce_mn_st_connected[][WICED_MCE_MN_NUM_COLS] =
{
/* Event                               Action 1                       Next state */
/* WICED_MCE_MN_INT_CLOSE_EVT     */   {WICED_MCE_MN_INT_CLOSE,       WICED_MCE_MN_CLOSING_ST},
/* WICED_MCE_MN_OBX_CONN_REQ_EVT  */   {WICED_MCE_MN_CONN_ERR_RSP,    WICED_MCE_MN_CONN_ST},
/* WICED_MCE_MN_OBX_PUT_EVT       */   {WICED_MCE_MN_OBX_PUT,         WICED_MCE_MN_CONN_ST},
/* WICED_MCE_MN_OBX_ACTION_EVT    */   {WICED_MCE_MN_OBX_ACTION,      WICED_MCE_MN_CONN_ST},
/* WICED_MCE_MN_OBX_ABORT_EVT     */   {WICED_MCE_MN_OBX_ABORT,       WICED_MCE_MN_CONN_ST},
/* WICED_MCE_MN_OBX_DISC_EVT      */   {WICED_MCE_MN_OBX_DISC,        WICED_MCE_MN_CLOSING_ST},
/* WICED_MCE_MN_OBX_CLOSE_EVT     */   {WICED_MCE_MN_OBX_CLOSE,       WICED_MCE_MN_LISTEN_ST}
};

/* state table for closing state */
static const UINT8 wiced_mce_mn_st_closing[][WICED_MCE_MN_NUM_COLS] =
{
/* Event                               Action 1                       Next state */
/* WICED_MCE_MN_INT_CLOSE_EVT     */   {WICED_MCE_MN_IGNORE,          WICED_MCE_MN_CLOSING_ST},
/* WICED_MCE_MN_OBX_CONN_REQ_EVT  */   {WICED_MCE_MN_CONN_ERR_RSP,    WICED_MCE_MN_CLOSING_ST},
/* WICED_MCE_MN_OBX_PUT_EVT       */   {WICED_MCE_MN_IGNORE_OBX,      WICED_MCE_MN_CLOSING_ST},
/* WICED_MCE_MN_OBX_ACTION_EVT    */   {WICED_MCE_MN_IGNORE_OBX,      WICED_MCE_MN_CLOSING_ST},
/* WICED_MCE_MN_OBX_ABORT_EVT     */   {WICED_MCE_MN_OBX_ABORT,       WICED_MCE_MN_CLOSING_ST},
/* WICED_MCE_MN_OBX_DISC_EVT      */   {WICED_MCE_MN_OBX_CLOSE,       WICED_MCE_MN_CLOSING_ST},
/* WICED_MCE_MN_OBX_CLOSE_EVT     */   {WICED_MCE_MN_OBX_CLOSE,       WICED_MCE_MN_LISTEN_ST}
};

/* type for state table MNS */
typedef const UINT8 (*wiced_mce_mn_st_tbl_t)[WICED_MCE_MN_NUM_COLS];

/* MNS state table */
const wiced_mce_mn_st_tbl_t wiced_mce_mn_st_tbl[] =
{
    wiced_mce_mn_st_idle,
    wiced_mce_mn_st_listen,
    wiced_mce_mn_st_connected,
    wiced_mce_mn_st_closing
};
/*****************************************************************************
** Global data
*****************************************************************************/

/* MCE control block */
wiced_mce_cb_t  wiced_mce_cb =
{
    .is_enabled = FALSE
};

#if (WICED_MCE_DEBUG == TRUE) && (BT_USE_TRACES == TRUE)
static char *mce_evt_code(wiced_mce_int_evt_t evt_code);
static char *mce_state_code(wiced_mce_ma_state_t state_code);
static char *mce_mn_evt_code(wiced_mce_int_evt_t evt_code);
static char *mce_mn_state_code(wiced_mce_mn_state_t state_code);
#endif

static void wiced_mce_discover_cback(UINT16 status);
static void wiced_mce_api_enable(wiced_mce_cb_t *p_cb, wiced_mce_data_t *p_data);
static void wiced_mce_api_disable(wiced_mce_cb_t *p_cb, wiced_mce_data_t *p_data);
static void wiced_mce_api_discover(wiced_mce_cb_t *p_cb, wiced_mce_data_t *p_data);
static wiced_mce_inst_cb_t *wiced_mce_alloc_icb(wiced_bt_ma_inst_id_t inst_id, UINT8 ccb_idx, UINT8 *p_icb_idx);

static void wiced_mce_api_close_all(wiced_mce_cb_t *p_cb, wiced_mce_data_t *p_data);

/*******************************************************************************
**
** Function         wiced_mce_ma_sm_execute
**
** Description      State machine event handling function for MCE
**
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_sm_execute(UINT8 ccb_idx, UINT8 icb_idx, UINT16 event, wiced_mce_data_t *p_data)
{
    wiced_mce_inst_cb_t     *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_mce_ma_st_tbl_t   state_table;
    UINT8                   action;
    int                     i;
#if (WICED_MCE_DEBUG == TRUE) && (BT_USE_TRACES == TRUE)
    wiced_mce_ma_state_t    in_state  = p_icb->ma_state;
    UINT16                  cur_event = event;
    APPL_TRACE_EVENT3("MCE MA Event Handle: State 0x%02x [%s], Event [%s]\n", in_state,
                      mce_state_code(in_state),
                      mce_evt_code(cur_event));
#endif

    /* look up the state table for the current state */
    state_table = wiced_mce_ma_st_tbl[p_icb->ma_state];

    event &= 0x00FF;

    /* set next state */
    p_icb->ma_state = state_table[event][WICED_MCE_MA_NEXT_STATE];

    /* execute action functions */
    for (i = 0; i < WICED_MCE_MA_ACTIONS; i++)
    {
        if ((action = state_table[event][i]) != WICED_MCE_MA_IGNORE)
        {
            (*wiced_mce_ma_action[action])(ccb_idx, icb_idx, p_data);
        }
    }

#if (WICED_MCE_DEBUG == TRUE) && (BT_USE_TRACES == TRUE)
    if (in_state != p_icb->ma_state)
    {
        APPL_TRACE_DEBUG3("MCE State Change: [%s] -> [%s] after Event [%s]\n",
                          mce_state_code(in_state),
                          mce_state_code(p_icb->ma_state),
                          mce_evt_code(cur_event));
    }
#endif

}

/*******************************************************************************
**
** Function         wiced_mce_mn_sm_execute
**
** Description      State machine event handling function for MNC
**
** Parameters       scb_idx       - MN server control block index
**                  event         - MN server event
**                  p_data        - Pointer to MCE msg data
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_mn_sm_execute(UINT8 scb_idx, UINT16 event, wiced_mce_data_t *p_data)
{
    wiced_mce_mn_cb_t   *p_scb = WICED_MCE_GET_MN_CB_PTR(scb_idx);
    wiced_mce_mn_st_tbl_t state_table;
    UINT8               action;
    int                 i;

#if (WICED_MCE_DEBUG == TRUE) && (BT_USE_TRACES == TRUE)
    wiced_mce_ma_state_t in_state = p_scb->mn_state;
    UINT16              cur_event = event;
    APPL_TRACE_EVENT3("MCE MN Event Handler: State 0x%02x [%s], Event [%s]\n", in_state,
                      mce_mn_state_code(in_state),
                      mce_mn_evt_code(cur_event));
#endif

    /* look up the state table for the current state */
    state_table = wiced_mce_mn_st_tbl[p_scb->mn_state];

    event -= WICED_MCE_MN_EVT_MIN;    /* offset by MN_EVT_MIN */

    /* set next state */
    p_scb->mn_state = state_table[event][WICED_MCE_MN_NEXT_STATE];

    /* execute action functions */
    for (i = 0; i < WICED_MCE_MN_ACTIONS; i++)
    {
        if ((action = state_table[event][i]) != WICED_MCE_MN_IGNORE)
        {
            (*wiced_mce_mn_action[action])(scb_idx, p_data);
        }
    }

#if (WICED_MCE_DEBUG == TRUE) && (BT_USE_TRACES == TRUE)
    if (in_state != p_scb->mn_state)
    {
        APPL_TRACE_DEBUG3("MN State Change: [%s] -> [%s] after Event [%s]\n",
                          mce_mn_state_code(in_state),
                          mce_mn_state_code(p_scb->mn_state),
                          mce_mn_evt_code(cur_event));
    }
#endif

}
/*******************************************************************************
**
** Function         wiced_mce_ma_api_enable
**
** Description      Handle an api enable event.  This function enables the MAP
**                  Client.
**
** Parameters       p_cb        - pointer to the client's control block
**                  p_data      - Pointer to MCE msg data
**
** Returns          void
**
*******************************************************************************/
static void wiced_mce_api_enable(wiced_mce_cb_t *p_cb, wiced_mce_data_t *p_data)
{
    wiced_bt_ma_status_t status = WICED_BT_MA_STATUS_FAIL;

    if (!p_cb->is_enabled)
    {
        /* initialize control block */
        memset(p_cb, 0, sizeof(wiced_mce_cb_t));

        /* store parameters */
        p_cb->p_cback = p_data->api_enable.p_cback;
        p_cb->app_id = p_data->api_enable.app_id;
        p_cb->is_enabled = TRUE;
        status = WICED_BT_MA_STATUS_OK;
    }

    /* callback with enable event */
    (*p_cb->p_cback)(WICED_BT_MCE_ENABLE_EVT, (wiced_bt_mce_t *)&status);
}

/*****************************************************************************
** Function         wiced_mce_discover_cback
**
** Description      This is the SDP callback function used by MCE.
**                  This function will be executed by SDP when the service
**                  search is completed.  If the search is successful, it
**                  finds the first record in the database that matches the
**                  UUID of the search.  Then retrieves the scn and psm from the
**                  record.
**
**                  status      - status of the SDP
**
** Returns          Nothing.
**
******************************************************************************/
static void wiced_mce_discover_cback(UINT16 status)
{
    wiced_mce_cb_t      *p_cb = &wiced_mce_cb;
    wiced_bt_sdp_discovery_record_t *p_rec = NULL;
    wiced_bt_sdp_discovery_attribute_t *p_attr;
    wiced_bt_sdp_protocol_elem_t pe;
    UINT8               cnt=0;
    wiced_bt_mce_discover_t dis;

    APPL_TRACE_EVENT1("wiced_mce_discover_cback status:%d\n", status);

    /* Init some fields for callback data */
    memset(&dis, 0, sizeof(wiced_bt_mce_discover_t));
    memcpy(dis.bd_addr, p_cb->api_dis_bd_addr, BD_ADDR_LEN);
    dis.status = status;

    if (status == WICED_BT_SDP_SUCCESS)
    {
        /* loop through all records we found */
        do
        {
            /* Init rest of the fields for callback data */
            dis.rec[cnt].peer_features = WICED_BT_MA_DEFAULT_SUPPORTED_FEATURES;
            dis.rec[cnt].rec_version = WICED_BT_MA_VERSION_1_0;

            /* get next record; if none found, we're done */
            if ((p_rec = wiced_bt_sdp_find_service_in_db(p_cb->p_api_dis_db, UUID_SERVCLASS_MESSAGE_ACCESS,
                                             p_rec)) == NULL)
            {
                APPL_TRACE_EVENT0("wiced_mce_discover_cback no record found\n");
                break;
            }
            else
            {
                APPL_TRACE_EVENT1("wiced_mce_discover_cback found rec cnt=%d\n",cnt);
            }

            if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_INSTANCE_ID))!=NULL)
            {
                dis.rec[cnt].mas_inst_id = p_attr->attr_value.v.u8;
                APPL_TRACE_EVENT1("wiced_mce_discover_cback found inst_id=%d\n", dis.rec[cnt].mas_inst_id);
            }

            if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_SUPPORTED_MSG_TYPE))!=NULL)
            {
                dis.rec[cnt].supported_msg_type = p_attr->attr_value.v.u8;
                APPL_TRACE_EVENT1("wiced_mce_discover_cback found supported msg type=0x%x\n", dis.rec[cnt].supported_msg_type);
            }

            if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_SERVICE_NAME))!=NULL)
            {
                BCM_STRNCPY_S (dis.rec[cnt].name, sizeof(dis.rec[cnt].name), (char *)p_attr->attr_value.v.array,
                        WICED_BT_MCE_MAX_NAME_LEN_PER_LINE);
                APPL_TRACE_EVENT1("wiced_mce_discover_cback found service name=%s\n", dis.rec[cnt].name);
            }


            /* this is a mandatory attribute */
            wiced_bt_sdp_find_profile_version_in_rec (p_rec, UUID_SERVCLASS_MAP_PROFILE, &dis.rec[cnt].rec_version);
            APPL_TRACE_EVENT1("wiced_mce_discover_cback found version=%d\n", dis.rec[cnt].rec_version);

#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
            /* If profile version is 1.2 or greater, look for supported features and L2CAP PSM */
            if (dis.rec[cnt].rec_version >= BTA_MA_VERSION_1_2)
            {

                /* Look for peer supported features */
                if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec,
                                ATTR_ID_SUPPORTED_FEATURES_32)) != NULL)
                {
                    dis.rec[cnt].peer_features = p_attr->attr_value.v.u32;
                    APPL_TRACE_DEBUG1("wiced_mce_discover_cback peer_features=0x%08x\n", dis.rec[cnt].peer_features);
                }
                /* No supported features, use the default */
                else
                {
                    APPL_TRACE_DEBUG0("wiced_mce_discover_cback peer supported features not found use default\n");
                }

                /* Look for L2CAP PSM */
                if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec,
                                ATTR_ID_OBX_OVR_L2CAP_PSM)) != NULL)
                {
                    dis.rec[cnt].psm = p_attr->attr_value.v.u16;
                    if ((SDP_DISC_ATTR_LEN(p_attr->attr_len_type) == 2) && L2C_IS_VALID_PSM(dis.rec[cnt].psm))
                    {
                        /* Both supported features and PSM found, done! */
                        /* Supported feature can be either from peer or default one */
                        APPL_TRACE_DEBUG1("wiced_mce_discover_cback psm=0x%02x (Using MAP 1.2 or later)\n",dis.rec[cnt].psm);
                    }
                    /* Bad PSM, look for next record */
                    else
                    {
                        APPL_TRACE_WARNING1("wiced_mce_discover_cback BAD psm=0x%02x\n",dis.rec[cnt].psm);
                        dis.rec[cnt].psm = 0;    /* Reset PSM */
                    }

                }
                /* No L2CAP PSM, look for next record */
                else
                {
                    APPL_TRACE_WARNING0("wiced_mce_discover_cback PSM for L2CAP not found\n");
                }

            }
#endif /* BTA_MAP_1_2_SUPPORTED */
            /* get scn from proto desc list; if not found, go to next record */
            if (wiced_bt_sdp_find_protocol_list_elem_in_rec(p_rec, UUID_PROTOCOL_RFCOMM, &pe))
            {
                dis.rec[cnt].scn = (UINT8) pe.params[0];
                /* we've got the one, we're done */
                APPL_TRACE_EVENT1("wiced_mce_discover_cback scn=%d\n", dis.rec[cnt].scn);
            }

            cnt++;
            APPL_TRACE_EVENT1("wiced_mce_discover_cback cnt=%d\n",cnt);

        } while (cnt < WICED_BT_MCE_NUM_INST);
    }

    dis.num_mas_srv = cnt;

    if (p_cb->p_api_dis_db)
    {
        utl_freebuf((void **)&p_cb->p_api_dis_db);
    }

    /* callback with API discover event */
    (*p_cb->p_cback)(WICED_BT_MCE_DISCOVER_EVT, (wiced_bt_mce_t *)&dis);

}

/*******************************************************************************
**
** Function         wiced_mce_api_disable
**
** Description      This function disable the MAP Client.
**
** Parameters       p_cb         - Pointer to MCE control block
** Parameters       p_data       - Pointer to MCE msg data
**
** Returns          void
**
*******************************************************************************/
static void wiced_mce_api_disable(wiced_mce_cb_t *p_cb, wiced_mce_data_t *p_data)
{
    UINT8 i,j;

    APPL_TRACE_EVENT0("wiced_mce_api_disable\n");

    if (p_cb->is_enabled)
    {
        wiced_mce_mn_disable(p_cb, p_data);

        if (wiced_mce_ma_get_num_of_act_conn(p_cb) == 0)
        {
            wiced_mce_disable_complete(p_cb);
        }
        else
        {
            p_cb->disabling = TRUE;
            for (i = 0; i < WICED_BT_MCE_NUM_MA; i++)
            {
                for (j = 0; j < WICED_BT_MCE_NUM_INST; j++)
                {
                    if (wiced_mce_cb.ccb[i].icb[j].ma_state == WICED_MCE_MA_CONN_ST)
                    {
                        wiced_mce_ma_sm_execute(i, j, WICED_MCE_API_CLOSE_EVT, p_data);
                    }
                }
            }
        }
    }
    else
    {
        APPL_TRACE_WARNING0("MAP is not enabled\n");
    }
}

/*******************************************************************************
**
** Function         wiced_mce_api_close_all
**
** Description      This function close all MAS session
**
** Parameters       p_cb        - pointer to the client's control block
**                  p_data      - Pointer to MCE msg data
**
** Returns          void
**
*******************************************************************************/
static void wiced_mce_api_close_all(wiced_mce_cb_t *p_cb, wiced_mce_data_t *p_data)
{
    UINT8 i,j;

    APPL_TRACE_EVENT0("wiced_mce_api_close_all\n");

    for ( i = 0; i < WICED_BT_MCE_NUM_MA; i++)
    {
        for ( j = 0; j < WICED_BT_MCE_NUM_INST; j++ )
        {
            if (wiced_mce_cb.ccb[i].icb[j].ma_state == WICED_MCE_MA_W4_CONN_ST)
            {
                wiced_mce_cb.ccb[i].active_count++;
                wiced_bt_mce_close(wiced_mce_cb.ccb[i].icb[j].obx_handle);
            }
            else if (wiced_mce_cb.ccb[i].icb[j].ma_state == WICED_MCE_MA_CONN_ST)
            {
                wiced_bt_mce_close(wiced_mce_cb.ccb[i].icb[j].obx_handle);
            }
        }
    }
}

/*******************************************************************************
**
** Function         wiced_mce_api_open
**
** Description      Process API MA open request.
**
** Parameters       p_cb   - Pointer to MCE control block
**                  p_data - Pointer to MCE event data
**
** Returns          void
**
*******************************************************************************/
static void wiced_mce_api_open(wiced_mce_cb_t *p_cb, wiced_mce_data_t *p_data)
{
    wiced_mce_inst_cb_t     *p_icb = NULL;
    wiced_bt_mce_ma_open_close_t app_evt;
    UINT8                   ccb_idx;
    UINT8                   icb_idx;

    APPL_TRACE_EVENT0("wiced_mce_api_open\n");

    if (wiced_mce_find_available_ma_cb_index(p_data->api_open.bd_addr, &ccb_idx) &&
        ((p_icb = wiced_mce_alloc_icb(p_data->api_open.mas_inst_id, ccb_idx, &icb_idx)) != NULL))
    {
        wiced_mce_ma_sm_execute(ccb_idx, icb_idx, WICED_MCE_API_OPEN_EVT, p_data);
    }
    else
    {
        app_evt.status = WICED_BT_MA_STATUS_NO_RESOURCE;
        app_evt.mas_inst_id = p_data->api_open.mas_inst_id;
        memcpy(app_evt.bd_addr, p_data->api_open.bd_addr, BD_ADDR_LEN);

        (*p_cb->p_cback)(WICED_BT_MCE_MA_OPEN_EVT, (wiced_bt_mce_t *)&app_evt);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_api_close
**
** Description      Process API MA close request.
**
** Parameters       p_cb   - Pointer to MCE control block
**                  p_data - Pointer to MCE event data
**
** Returns          void
**
*******************************************************************************/
static void wiced_mce_api_close(wiced_mce_cb_t *p_cb, wiced_mce_data_t *p_data)
{
    wiced_bt_mce_ma_open_close_t app_evt;
    UINT8                   ccb_idx;
    UINT8                   icb_idx;

    APPL_TRACE_EVENT0("wiced_mce_api_close\n");

    if (wiced_mce_find_obx_handle_match_ma_cb_indexes(p_data->hdr.layer_specific, &ccb_idx, &icb_idx))
    {
        wiced_mce_ma_sm_execute(ccb_idx, icb_idx, WICED_MCE_API_CLOSE_EVT, p_data);
    }
    else
    {
        app_evt.status = WICED_BT_MA_STATUS_FAIL;
        app_evt.session_id = p_data->hdr.layer_specific;

        (*p_cb->p_cback)(WICED_BT_MCE_MA_CLOSE_EVT, (wiced_bt_mce_t *)&app_evt);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_mn_api_close
**
** Description      Process API MN close request.
**
** Parameters       p_cb   - Pointer to MCE control block
**                  p_data - Pointer to MCE event data
**
** Returns          void
**
*******************************************************************************/
static void wiced_mce_mn_api_close(wiced_mce_cb_t *p_cb, wiced_mce_data_t *p_data)
{
    wiced_bt_mce_ma_open_close_t app_evt;
    UINT8                        scb_idx;

    APPL_TRACE_EVENT0("wiced_mce_mn_api_close\n");

    if (wiced_mce_find_obx_handle_match_mn_cb_index(p_data->api_mn_close.session_id, &scb_idx))
    {
        wiced_mce_mn_sm_execute(scb_idx, WICED_MCE_MN_INT_CLOSE_EVT, p_data);
    }
    else
    {
        app_evt.status = WICED_BT_MA_STATUS_FAIL;
        p_cb->p_cback(WICED_BT_MCE_MN_CLOSE_EVT, (wiced_bt_mce_t *)&app_evt);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_api_discover
**
** Description      Handle an api enable event.  This function does the MAP
**                  service discover.
**
** Parameters       p_cb        - Pointer to MCE control block
**                  p_data      - Pointer to MCE msg data
**
** Returns          void
**
*******************************************************************************/
static void wiced_mce_api_discover(wiced_mce_cb_t *p_cb, wiced_mce_data_t *p_data)
{
    wiced_bt_mce_discover_t dis;
    wiced_bt_uuid_t uuid_list;
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
    UINT16          attr_list[8];
    UINT16          num_attrs = 8;
#else
    UINT16          attr_list[7];
    UINT16          num_attrs = 7;
#endif

    APPL_TRACE_EVENT0("wiced_mce_api_discover\n");

    memset(&dis, 0, sizeof(wiced_bt_mce_discover_t));
    memcpy(dis.bd_addr, p_data->api_discover.bd_addr, BD_ADDR_LEN);
    dis.status = WICED_BT_MA_STATUS_FAIL;

    if (!p_cb->p_api_dis_db && /* only one SDP can be active at a time */
        (p_cb->p_api_dis_db = (wiced_bt_sdp_discovery_db_t *) GKI_getbuf(WICED_MCE_DISC_SIZE)) != NULL)
    {
        attr_list[0] = ATTR_ID_SERVICE_CLASS_ID_LIST;
        attr_list[1] = ATTR_ID_PROTOCOL_DESC_LIST;
        attr_list[2] = ATTR_ID_SERVICE_NAME;
        attr_list[3] = ATTR_ID_BT_PROFILE_DESC_LIST;
        attr_list[4] = ATTR_ID_INSTANCE_ID;
        attr_list[5] = ATTR_ID_SUPPORTED_MSG_TYPE;
        /* always search peer features */
        attr_list[6] = ATTR_ID_SUPPORTED_FEATURES_32;
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
        attr_list[7] = ATTR_ID_OBX_OVR_L2CAP_PSM;
#endif

        uuid_list.len = LEN_UUID_16;
        uuid_list.uu.uuid16 = UUID_SERVCLASS_MESSAGE_ACCESS;

        memcpy(p_cb->api_dis_bd_addr, p_data->api_discover.bd_addr, BD_ADDR_LEN);

        wiced_bt_sdp_init_discovery_db(p_cb->p_api_dis_db, WICED_MCE_DISC_SIZE, 1, &uuid_list, num_attrs, attr_list);

        if (!wiced_bt_sdp_service_search_attribute_request(p_data->api_discover.bd_addr, p_cb->p_api_dis_db, wiced_mce_discover_cback))
        {
            /* callback with discover event */
            (*p_cb->p_cback)(WICED_BT_MCE_DISCOVER_EVT, (wiced_bt_mce_t *)&dis);
        }
    }
    else    /* No services available */
    {
        APPL_TRACE_EVENT1("wiced_mce_api_discover failed %x\n", p_cb->p_api_dis_db);
        (*p_cb->p_cback)(WICED_BT_MCE_DISCOVER_EVT, (wiced_bt_mce_t *)&dis);
    }

}

/*******************************************************************************
**
** Function         wiced_mce_rsp_timer_cback
**
** Description      MCE response timer callback.
**
**
** Returns          void
**
*******************************************************************************/
static void wiced_mce_rsp_timer_cback(uint32_t cb_params)
{
    BT_HDR          *p_buf;

    if ((p_buf = (BT_HDR *) GKI_getbuf(sizeof(BT_HDR))) != NULL)
    {
        p_buf->event = WICED_MCE_RSP_TOUT_EVT;
        p_buf->layer_specific = (UINT16)cb_params;
        wiced_mce_send_event((BT_HDR *)p_buf);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_ma_alloc_icb
**
** Description      Allocate a new instance control block
**
** Parameters       inst_id         - MA instance control block ID
**                  ccb_idx         - MA client control block index
**                  p_icb_idx       - (output) pointer to the MA instance control block index
**
** Returns          void
**
*******************************************************************************/
static wiced_mce_inst_cb_t *wiced_mce_alloc_icb(wiced_bt_ma_inst_id_t inst_id, UINT8 ccb_idx, UINT8 *p_icb_idx)
{
    UINT8 i;

    for (i = 0; i < WICED_BT_MCE_NUM_INST; i++)
    {
        if (wiced_mce_cb.ccb[ccb_idx].icb[i].in_use == FALSE)
        {
            *p_icb_idx = i;

            if (wiced_mce_cb.ccb[ccb_idx].in_use == FALSE)
               wiced_mce_cb.ccb[ccb_idx].in_use = TRUE;

            wiced_mce_cb.ccb[ccb_idx].icb[i].in_use = TRUE;
            wiced_init_timer(&wiced_mce_cb.ccb[ccb_idx].icb[i].rsp_timer,
                             wiced_mce_rsp_timer_cback,
                             (UINT32)WICED_MCE_CCB_INST_TO_TPARAM(((UINT16)ccb_idx), inst_id),
                             WICED_SECONDS_TIMER);

            APPL_TRACE_DEBUG1("Find an empty control block idx = %d\n", i);
            return &wiced_mce_cb.ccb[ccb_idx].icb[i];
        }
    }
    APPL_TRACE_ERROR0("Cannot find an empty control\n");
    return NULL;
}

/*******************************************************************************
**
** Function         wiced_mce_hdl_event
**
** Description      MCE main event handling function.
**
**
** Returns          void
**
*******************************************************************************/
BOOLEAN wiced_mce_hdl_event(BT_HDR *p_msg)
{
    wiced_mce_data_t    *p_data = (wiced_mce_data_t *) p_msg;
    UINT8               ccb_idx;
    UINT8               icb_idx;
    UINT8               scb_idx;

    switch (p_msg->event)
    {
        /* Events handled outside state machine */
        case WICED_MCE_API_ENABLE_EVT:
            wiced_mce_api_enable(&wiced_mce_cb, p_data);
            break;
        case WICED_MCE_API_DISCOVER_EVT:
            wiced_mce_api_discover(&wiced_mce_cb, p_data);
            break;
        case WICED_MCE_API_DISABLE_EVT:
            wiced_mce_api_disable(&wiced_mce_cb, p_data);
            break;
        case WICED_MCE_API_CLOSE_ALL_EVT:
            wiced_mce_api_close_all(&wiced_mce_cb, p_data);
            break;
        case WICED_MCE_API_START_EVT:
            wiced_mce_mn_start_service(&wiced_mce_cb, p_data);
            break;
        case WICED_MCE_API_STOP_EVT:
            wiced_mce_mn_disable(&wiced_mce_cb, p_data);
            break;
        case WICED_MCE_API_OPEN_EVT:
            wiced_mce_api_open(&wiced_mce_cb, p_data);
            break;
        case WICED_MCE_API_CLOSE_EVT:
            wiced_mce_api_close(&wiced_mce_cb, p_data);
            break;
        case WICED_MCE_API_MN_CLOSE_EVT:
            wiced_mce_mn_api_close(&wiced_mce_cb, p_data);
            break;
        default:
            if (p_msg->event < WICED_MCE_MN_EVT_MIN)
            {
                if (wiced_mce_find_ma_cb_indexes((wiced_mce_data_t *) p_msg, &ccb_idx, &icb_idx))
                {
                    if (p_msg->event < WICED_MCE_MN_EVT_MIN)
                    {
                        wiced_mce_ma_sm_execute(ccb_idx, icb_idx, p_msg->event, p_data);
                        //wiced_mce_ma_update_pm_state(ccb_idx, icb_idx);
                    }
                }
            }
            /* Events handled by MN state machine */
            else
            {
                if (wiced_mce_find_mn_cb_index((wiced_mce_data_t *) p_msg, &scb_idx))
                {
                    wiced_mce_mn_sm_execute(scb_idx, p_msg->event, p_data);
                    //wiced_mce_mn_update_pm_state(&wiced_mce_cb, scb_idx);
                }
            }
            break;
    }

    return(TRUE);
}

void wiced_mce_send_event(BT_HDR *p_msg)
{
    wiced_mce_hdl_event(p_msg);

    GKI_freebuf(p_msg);
}

/*****************************************************************************
**  Debug Functions
*****************************************************************************/
#if (WICED_MCE_DEBUG == TRUE) && (BT_USE_TRACES == TRUE)

/*******************************************************************************
**
** Function         mce_evt_code
**
** Description      MCE internal event code
**
** Returns          void
**
*******************************************************************************/
static char *mce_evt_code(wiced_mce_int_evt_t evt_code)
{
    switch (evt_code)
    {
        case WICED_MCE_API_ENABLE_EVT:
            return "WICED_MCE_API_ENABLE_EVT";
        case WICED_MCE_API_DISABLE_EVT:
            return "WICED_MCE_API_DISABLE_EVT";
        case WICED_MCE_API_OPEN_EVT:
            return "WICED_MCE_API_OPEN_EVT";
        case WICED_MCE_API_CLOSE_EVT:
            return "WICED_MCE_API_CLOSE_EVT";
        case WICED_MCE_API_NOTIF_REG_EVT:
            return "WICED_MCE_API_NOTIF_REG_EVT";
        case WICED_MCE_API_UPD_INBOX_EVT:
            return "WICED_MCE_API_UPD_INBOX_EVT";
        case WICED_MCE_API_CHDIR_EVT:
            return "WICED_MCE_API_CHDIR_EVT";
        case WICED_MCE_API_LIST_EVT:
            return "WICED_MCE_API_LIST_EVT";
        case WICED_MCE_API_GET_MSG_EVT:
            return "WICED_MCE_API_GET_MSG_EVT";
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
        case WICED_MCE_API_GET_MAS_INS_INFO_EVT:
            return "WICED_MCE_API_GET_MAS_INS_INFO_EVT";
#endif
        case WICED_MCE_API_SET_STS_EVT:
            return "WICED_MCE_API_SET_STS_EVT";
        case WICED_MCE_API_PUSH_EVT:
            return "WICED_MCE_API_PUSH_EVT";
        case WICED_MCE_API_ABORT_EVT:
            return "WICED_MCE_API_ABORT_EVT";
        case WICED_MCE_SDP_FAIL_EVT:
            return "WICED_MCE_SDP_FAIL_EVT";
        case WICED_MCE_SDP_OK_EVT:
            return "WICED_MCE_SDP_OK_EVT";
        case WICED_MCE_MA_OBX_CONN_RSP_EVT:
            return "WICED_MCE_MA_OBX_CONN_RSP_EVT";
        case WICED_MCE_MA_OBX_PUT_RSP_EVT:
            return "WICED_MCE_MA_OBX_PUT_RSP_EVT";
        case WICED_MCE_MA_OBX_GET_RSP_EVT:
            return "WICED_MCE_MA_OBX_GET_RSP_EVT";
        case WICED_MCE_MA_OBX_SETPATH_RSP_EVT:
            return "WICED_MCE_MA_OBX_SETPATH_RSP_EVT";
        case WICED_MCE_MA_OBX_TOUT_EVT:
            return "WICED_MCE_MA_OBX_TOUT_EVT";
        case WICED_MCE_MA_OBX_CLOSE_EVT:
            return "WICED_MCE_MA_OBX_CLOSE_EVT";
        case WICED_MCE_MA_OBX_ABORT_RSP_EVT:
            return "WICED_MCE_MA_OBX_ABORT_RSP_EVT";
        case WICED_MCE_MA_OBX_CMPL_EVT:
            return "WICED_MCE_MA_OBX_CMPL_EVT";
        case WICED_MCE_RSP_TOUT_EVT:
            return "WICED_MCE_RSP_TOUT_EVT";
        case WICED_MCE_MA_CLOSE_CMPL_EVT:
            return "WICED_MCE_MA_CLOSE_CMPL_EVT";

        default:
            return "unknown MCE event code";
    }
}

static char *mce_mn_evt_code(wiced_mce_int_evt_t evt_code)
{
    switch (evt_code)
    {
        case WICED_MCE_MN_INT_CLOSE_EVT:
            return "WICED_MCE_MN_INT_CLOSE_EVT";
        case WICED_MCE_MN_OBX_CONN_REQ_EVT:
            return "WICED_MCE_MN_OBX_CONN_REQ_EVT";
        case WICED_MCE_MN_OBX_PUT_EVT:
            return "WICED_MCE_MN_OBX_PUT_EVT";
        case WICED_MCE_MN_OBX_ACTION_EVT:
            return "WICED_MCE_MN_OBX_ACTION_EVT";
        case WICED_MCE_MN_OBX_ABORT_EVT:
            return "WICED_MCE_MN_OBX_ABORT_EVT";
        case WICED_MCE_MN_OBX_DISC_EVT:
            return "WICED_MCE_MN_OBX_DISC_EVT";
        case WICED_MCE_MN_OBX_CLOSE_EVT:
            return "WICED_MCE_MN_OBX_CLOSE_EVT";
        default:
            return "Unknown Event";
    }
}

/*******************************************************************************
**
** Function         mce_state_code
**
** Description      MCE side state code
**
** Returns          void
**
*******************************************************************************/
static char *mce_state_code(wiced_mce_ma_state_t state_code)
{
    switch (state_code)
    {
        case WICED_MCE_MA_IDLE_ST:
            return "WICED_MCE_MA_IDLE_ST";
        case WICED_MCE_MA_W4_CONN_ST:
            return "WICED_MCE_MA_W4_CONN_ST";
        case WICED_MCE_MA_CONN_ST:
            return "WICED_MCE_MA_CONN_ST";
        case WICED_MCE_MA_CLOSING_ST:
            return "WICED_MCE_MA_CLOSING_ST";
        default:
            return "unknown MCE state code";
    }
}
static char *mce_mn_state_code(wiced_mce_mn_state_t state_code)
{
    switch (state_code)
    {
        case WICED_MCE_MN_IDLE_ST:
            return "WICED_MCE_MN_IDLE_ST";
        case WICED_MCE_MN_LISTEN_ST:
            return "WICED_MCE_MN_LISTEN_ST";
        case WICED_MCE_MN_CONN_ST:
            return "WICED_MCE_MN_CONN_ST";
        case WICED_MCE_MN_CLOSING_ST:
            return "WICED_MCE_MN_CLOSING_ST";
        default:
            return "unknown MNS state code";
    }
}

#endif  /* Debug Functions */
