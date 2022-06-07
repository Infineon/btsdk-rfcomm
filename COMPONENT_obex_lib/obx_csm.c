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
**  Name:         obx_csm.c
**
**  File:         OBEX Client State Machine and Control Block Access Functions
**
**
*****************************************************************************/
#include <string.h>
#include "obx_int.h"

/* OBEX Client Action Functions Enums (must match obx_cl_action [below] */
enum
{
    OBEX_CA_SND_REQ,
    OBEX_CA_NOTIFY,
    OBEX_CA_CONNECT_ERROR,
    OBEX_CA_STATE,
    OBEX_CA_CLOSE_PORT,
    OBEX_CA_CONNECT_FAIL,
    OBEX_CA_DISCNT_REQ,
    OBEX_CA_START_TIMER,
    OBEX_CA_FAIL_RSP,
    OBEX_CA_SND_PART,
    OBEX_CA_CONNECT_OK,
    OBEX_CA_SESSION_OK,
    OBEX_CA_SESSION_CONT,
    OBEX_CA_SESSION_GET,
    OBEX_CA_SESSION_FAIL,
    OBEX_CA_ABORT,
    OBEX_CA_SND_PUT_REQ,
    OBEX_CA_SND_GET_REQ,
    OBEX_CA_SRM_SND_REQ,
    OBEX_CA_SRM_PUT_REQ,
    OBEX_CA_SRM_GET_REQ,
    OBEX_CA_SRM_PUT_NOTIFY,
    OBEX_CA_SRM_GET_NOTIFY,
    OBEX_CA_SAVE_RSP,
    OBEX_CA_SAVE_REQ
};

/* OBEX Client Action Functions */
static const tOBEX_CL_ACT obx_cl_action[] =
{
    obx_ca_snd_req,
    obx_ca_notify,
    obx_ca_connect_error,
    obx_ca_state,
    obx_ca_close_port,
    obx_ca_connect_fail,
    obx_ca_discnt_req,
    obx_ca_start_timer,
    obx_ca_fail_rsp,
    obx_ca_snd_part,
    obx_ca_connect_ok,
#ifdef OBEX_LIB_SESSION_SUPPORTED
    obx_ca_session_ok,
    obx_ca_session_cont,
    obx_ca_session_get,
    obx_ca_session_fail,
#else
    NULL,
    NULL,
    NULL,
    NULL,
#endif
    obx_ca_abort,
    obx_ca_snd_put_req,
    obx_ca_snd_get_req,
    obx_ca_srm_snd_req,
    obx_ca_srm_put_req,
    obx_ca_srm_get_req,
    obx_ca_srm_put_notify,
    obx_ca_srm_get_notify,
    obx_ca_save_rsp,
    obx_ca_save_req
};

/************ OBX Client FSM State/Event Indirection Table **************/
/* obx_csm_event() first looks at obx_csm_entry_map[][] to get an entry of the event of a particular state
 * 0 means the event in the current state is ignored.
 * a number with 0x80 bit set, use obx_cl_all_table[][] as the "state table".
 * other numbers, look up obx_cl_main_state_table[] for the state table of current state.
 *
 * once the state table is determined,
 * look up the "action" column to find the associated action function
 * and the "next state" column to find the "next state" candidate.
 *
 * The actual next state could be either the state in the "next state" column
 * or the state returned from the action function.
 */
static const UINT8 obx_csm_entry_map[][OBEX_CS_MAX-1] =
{
/* state name:  NtCon SesRs ConRs Conn  DscRs StpRs ActRs AbtRs PutRs GetRs Put   Get   PutS  GetS  Part */
/* CONN_R   */{ 1,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0 },
/* SESS_R   */{ 2,    0,    0,    5,    0,    0,    0,    0,    2,    2,    2,    2,    5,    5,    1 },
/* DISCNT_R */{ 0,    0x87, 2,    0x81, 0x87, 0x81, 0x81, 0x81, 0x87, 0x87, 0x87, 0x87, 0x87, 0x87, 0x87 },
/* PUT_R    */{ 0,    0,    0,    1,    0,    0,    0,    0,    0,    0,    1,    0,    2,    0,    0 },
/* GET_R    */{ 0,    3,    0,    2,    0,    0,    0,    0,    0,    0,    0,    1,    0,    2,    0 },
/* SETPATH_R*/{ 0,    0,    0,    3,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0 },
/* ACT_R    */{ 0,    0,    0,    6,    0,    0,    0,    0,    0x82, 0x82, 0x82, 0x82, 3,    3,    1 },
/* ABORT_R  */{ 0,    0,    0,    4,    0,    0,    0,    0,    0x82, 0x82, 0x82, 0x82, 3,    3,    1 },
/* OK_C     */{ 0,    1,    3,    0,    1,    0x83, 0x83, 0x83, 0x83, 0x83, 0,    0,    0x83, 0x83, 3 },
/* CONT_C   */{ 0,    2,    0,    0,    2,    0,    0,    1,    1,    1,    0,    0,    4,    4,    3 },
/* FAIL_C   */{ 0,    4,    1,    0,    0x87, 0x86, 0x86, 2,    0x86, 0x86, 0,    0,    0x86, 0x86, 3 },
/* PORT_CLS */{ 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84 },
/* TX_EMPTY */{ 0x87, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0 },
/* FCS_SET  */{ 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    2 },
/* STATE    */{ 0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85 },
/* TIMEOUT  */{ 3,    2,    0x87, 0,    0x87, 0,    0,    0,    0,    0x87, 0,    0,    0,    0x87, 0 }
};

static const UINT8 obx_cl_all_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* DISCNT_R */{OBEX_CA_SND_REQ,          OBEX_CS_DISCNT_REQ_SENT },
/* ABORT_R  */{OBEX_CA_SND_REQ,          OBEX_CS_ABORT_REQ_SENT },
/* OK_C     */{OBEX_CA_NOTIFY,           OBEX_CS_NULL },
/* PORT_CLS */{OBEX_CA_CONNECT_ERROR,    OBEX_CS_NOT_CONNECTED },
/* STATE    */{OBEX_CA_STATE,            OBEX_CS_NULL },
/* FAIL_C   */{OBEX_CA_FAIL_RSP,         OBEX_CS_NULL },
/* end      */{OBEX_CA_CLOSE_PORT,       OBEX_CS_NOT_CONNECTED }
};

static const UINT8 obx_cl_not_conn_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* CONN_R   */{OBEX_CA_SND_REQ,          OBEX_CS_CONNECT_REQ_SENT },
/* SESS_R   */{OBEX_CA_SND_REQ,          OBEX_CS_SESSION_REQ_SENT },
/* TIMEOUT  */{OBEX_CA_SESSION_FAIL,     OBEX_CS_NOT_CONNECTED }
};

static const UINT8 obx_cl_sess_rs_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* OK_C     */{OBEX_CA_SESSION_OK,       OBEX_CS_NOT_CONNECTED },
/* CONT_C   */{OBEX_CA_SESSION_CONT,     OBEX_CS_SESSION_REQ_SENT },
/* GET_R    */{OBEX_CA_SESSION_GET,      OBEX_CS_SESSION_REQ_SENT },
/* FAIL_C   */{OBEX_CA_SESSION_FAIL,     OBEX_CS_NOT_CONNECTED }
};

static const UINT8 obx_cl_conn_rs_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* FAIL_C   */{OBEX_CA_CONNECT_FAIL,     OBEX_CS_NULL },
/* DISCNT_R */{OBEX_CA_DISCNT_REQ,       OBEX_CS_NULL },
/* OK_C     */{OBEX_CA_CONNECT_OK,       OBEX_CS_NULL }
};

static const UINT8 obx_cl_conn_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* PUT_R    */{OBEX_CA_SND_PUT_REQ,      OBEX_CS_PUT_REQ_SENT },
/* GET_R    */{OBEX_CA_SND_GET_REQ,      OBEX_CS_GET_REQ_SENT },
/* SETPATH_R*/{OBEX_CA_SND_REQ,          OBEX_CS_SETPATH_REQ_SENT },
/* ABORT_R  */{OBEX_CA_ABORT,            OBEX_CS_CONNECTED },
/* SESS_R   */{OBEX_CA_SND_REQ,          OBEX_CS_SESSION_REQ_SENT },
/* ACT_R    */{OBEX_CA_SND_REQ,          OBEX_CS_ACTION_REQ_SENT }
};

static const UINT8 obx_cl_discnt_rs_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* OK_C     */{OBEX_CA_NOTIFY,           OBEX_CS_NOT_CONNECTED },  /* and close port */
/* CONT_C   */{OBEX_CA_START_TIMER,      OBEX_CS_DISCNT_REQ_SENT }
};

/* static const UINT8 obx_cl_setpath_rs_table[][OBEX_SM_NUM_COLS] = { */
/* Event       Action                   Next State */
/* DISCNT_R   {OBEX_CA_SND_REQ,          OBEX_CS_DISCNT_REQ_SENT },*/
/* OK_C       {OBEX_CA_NOTIFY,           OBEX_CS_NULL },*/
/* FAIL_C     {OBEX_CA_FAIL_RSP,         OBEX_CS_NULL },*/
/* PORT_CLS   {OBEX_CA_CONNECT_ERROR,    OBEX_CS_NOT_CONNECTED },*/
/* STATE      {OBEX_CA_STATE,            OBEX_CS_NULL },*/
/* }; */

static const UINT8 obx_cl_abort_rs_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* CONT_C   */{OBEX_CA_START_TIMER,      OBEX_CS_ABORT_REQ_SENT },
/* FAIL_C   */{OBEX_CA_NOTIFY,           OBEX_CS_CONNECTED }
};

static const UINT8 obx_cl_put_rs_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* CONT_C   */{OBEX_CA_NOTIFY,           OBEX_CS_PUT_TRANSACTION },
/* SESS_R   */{OBEX_CA_SND_REQ,          OBEX_CS_SESSION_REQ_SENT }
};

static const UINT8 obx_cl_get_rs_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* CONT_C   */{OBEX_CA_NOTIFY,           OBEX_CS_GET_TRANSACTION },
/* SESS_R   */{OBEX_CA_SND_REQ,          OBEX_CS_SESSION_REQ_SENT }
};

static const UINT8 obx_cl_put_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* PUT_R    */{OBEX_CA_SND_REQ,          OBEX_CS_PUT_REQ_SENT },
/* SESS_R   */{OBEX_CA_SND_REQ,          OBEX_CS_SESSION_REQ_SENT }
};

static const UINT8 obx_cl_get_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* GET_R    */{OBEX_CA_SND_REQ,          OBEX_CS_GET_REQ_SENT },
/* SESS_R   */{OBEX_CA_SND_REQ,          OBEX_CS_SESSION_REQ_SENT }
};

static const UINT8 obx_cl_put_s_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* DISCNT_R */{OBEX_CA_SRM_SND_REQ,      OBEX_CS_DISCNT_REQ_SENT },
/* PUT_R    */{OBEX_CA_SRM_PUT_REQ,      OBEX_CS_PUT_SRM },
/* ABORT_R  */{OBEX_CA_SRM_SND_REQ,      OBEX_CS_ABORT_REQ_SENT },
/* CONT_C   */{OBEX_CA_SRM_PUT_NOTIFY,   OBEX_CS_PUT_SRM },
/* SESS_R   */{OBEX_CA_SND_REQ,          OBEX_CS_SESSION_REQ_SENT }
};

static const UINT8 obx_cl_get_s_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* DISCNT_R */{OBEX_CA_SRM_SND_REQ,      OBEX_CS_DISCNT_REQ_SENT },
/* GET_R    */{OBEX_CA_SRM_GET_REQ,      OBEX_CS_GET_SRM },
/* ABORT_R  */{OBEX_CA_SRM_SND_REQ,      OBEX_CS_ABORT_REQ_SENT },
/* CONT_C   */{OBEX_CA_SRM_GET_NOTIFY,   OBEX_CS_GET_SRM },
/* SESS_R   */{OBEX_CA_SND_REQ,          OBEX_CS_SESSION_REQ_SENT }
};

static const UINT8 obx_cl_part_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* ABORT_R  */{OBEX_CA_SAVE_REQ,         OBEX_CS_PARTIAL_SENT },
/* FCS_SET  */{OBEX_CA_SND_PART,         OBEX_CS_NULL },
/* FAIL_C   */{OBEX_CA_SAVE_RSP,         OBEX_CS_NULL }
};

static const tOBEX_SM_TBL obx_cl_main_state_table[] = {
    obx_cl_not_conn_table,
    obx_cl_sess_rs_table,
    obx_cl_conn_rs_table,
    obx_cl_conn_table,
    obx_cl_discnt_rs_table,
    NULL, /* obx_cl_setpath_rs_table */
    NULL, /* obx_cl_action_rs_table */
    obx_cl_abort_rs_table,
    obx_cl_put_rs_table,
    obx_cl_get_rs_table,
    obx_cl_put_table,
    obx_cl_get_table,
    obx_cl_put_s_table,
    obx_cl_get_s_table,
    obx_cl_part_table
};

/*******************************************************************************
**
** Function     obx_csm_event
**
** Description  Handle events to the client state machine. It looks up the entry
**              in the obx_csm_entry_map array. If it is a valid entry, it gets
**              the state table. Set the next state, if not NULL state.Execute
**              the action function according to the state table. If the state
**              returned by action function is not NULL state, adjust the new
**              state to the returned state.If (api_evt != MAX), call callback
**              function.
**
** Returns      void.
**
*******************************************************************************/
void obx_csm_event(tOBEX_CL_CB *p_cb, tOBEX_CL_EVENT event, BT_HDR *p_msg)
{
    UINT8           curr_state = p_cb->state;
    tOBEX_SM_TBL     state_table = NULL;
    UINT8           action, entry;
    tOBEX_CL_STATE   act_state = OBEX_CS_NULL;
    UINT8           prev_state = OBEX_CS_NULL;

    if( curr_state == OBEX_CS_NULL || curr_state >= OBEX_CS_MAX)
    {
        OBEX_TRACE_WARNING1( "Invalid state: %d\n", curr_state) ;
        if(p_msg)
            GKI_freebuf(p_msg);
        return;
    }
    OBEX_TRACE_DEBUG4( "obx_csm_event: Client Handle 0x%x, State: %s, event: %s srm:0x%x\n",
        p_cb->ll_cb.comm.handle, obx_cl_get_state_name( p_cb->state ), obx_cl_get_event_name(event), p_cb->srm ) ;
    OBEX_TRACE_DEBUG1("obx_csm_event csm offset:%d\n", p_cb->param.sess.obj_offset);

    /* look up the state table for the current state */
    /* lookup entry /w event & curr_state */
    /* If entry is ignore, return.
     * Otherwise, get state table (according to curr_state or all_state) */
    if( (entry = obx_csm_entry_map[event][curr_state-1]) != OBEX_SM_IGNORE )
    {
        if(entry&OBEX_SM_ALL)
        {
            entry &= OBEX_SM_ENTRY_MASK;
            state_table = obx_cl_all_table;
        }
        else
            state_table = obx_cl_main_state_table[curr_state-1];
    }

    if( entry == OBEX_SM_IGNORE || state_table == NULL)
    {
        OBEX_TRACE_WARNING4( "Ignore event %s(%d) in state %s(%d)\n",
            obx_cl_get_event_name(event), event, obx_cl_get_state_name(curr_state), curr_state );
        if(p_msg)
            GKI_freebuf(p_msg);
        return;
    }

    /* Get possible next state from state table. */
    if( state_table[entry-1][OBEX_SME_NEXT_STATE] != OBEX_CS_NULL )
    {
        prev_state = p_cb->state;
        p_cb->state = state_table[entry-1][OBEX_SME_NEXT_STATE];
        if (prev_state != p_cb->state)
        {
            p_cb->prev_state = prev_state;
            OBEX_TRACE_DEBUG1( "saved state1:%s\n", obx_cl_get_state_name(p_cb->prev_state));
        }
    }
    OBEX_TRACE_DEBUG1( "possible new state = %s\n", obx_cl_get_state_name( p_cb->state ) ) ;

    /* If action is not ignore, clear param, exec action and get next state.
     * The action function may set the Param for cback.
     * Depending on param, call cback or free buffer. */
    /* execute action */
    action = state_table[entry-1][OBEX_SME_ACTION];
    if (action != OBEX_SM_NO_ACTION)
    {
        if (obx_cl_action[action] != NULL)
            act_state = (*obx_cl_action[action])(p_cb, p_msg);
    }

    /* adjust next state, if it needs to use the new state returned from action function */
    if( act_state != OBEX_CS_NULL)
    {
        prev_state = p_cb->state;
        p_cb->state = act_state;
        OBEX_TRACE_DEBUG1( "new state = %s (action)\n", obx_cl_get_state_name( p_cb->state )) ;
        if (prev_state != p_cb->state)
        {
            p_cb->prev_state = prev_state;
            OBEX_TRACE_DEBUG1( "saved state2:%s\n", obx_cl_get_state_name(p_cb->prev_state));
        }
    }

    if(p_cb->api_evt)
    {
        (p_cb->p_cback) (p_cb->ll_cb.comm.handle, p_cb->api_evt, p_cb->rsp_code, p_cb->param, (UINT8 *)p_msg);
        p_cb->api_evt   = OBEX_NULL_EVT;
        p_cb->rsp_code  = 0;
        memset(&p_cb->param, 0, sizeof (p_cb->param) );
    }
    else if(action == OBEX_SM_NO_ACTION && p_msg)
            GKI_freebuf(p_msg);
    OBEX_TRACE_DEBUG1("after csm offset:%d\n", p_cb->param.sess.obj_offset);

    OBEX_TRACE_DEBUG2( "result state = %s/%d\n", obx_cl_get_state_name( p_cb->state ), p_cb->state ) ;
}
