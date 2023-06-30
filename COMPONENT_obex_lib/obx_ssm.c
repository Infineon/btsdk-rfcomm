/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
**  Name:         obx_ssm.c
**
**  File:         OBEX Server State Machine and Control Block Access Functions
**
**
*****************************************************************************/
#include <string.h>
#include "obx_int.h"

/* OBEX Server Action Functions Enums (must match obx_sr_action [below] */
enum
{
    OBEX_SA_CLOSE_PORT,
    OBEX_SA_CONNECTION_ERROR,
    OBEX_SA_STATE,
    OBEX_SA_CONNECT_IND,
    OBEX_SA_WC_CONN_IND,
    OBEX_SA_NC_TO,
    OBEX_SA_CONNECT_RSP,
    OBEX_SA_SND_RSP,
    OBEX_SA_SAVE_REQ,
    OBEX_SA_SND_PART,
    OBEX_SA_REJ_REQ,
    OBEX_SA_ABORT_RSP,
    OBEX_SA_OP_RSP,
    OBEX_SA_GET_IND,
    OBEX_SA_GET_REQ,
    OBEX_SA_SESSION_IND,
    OBEX_SA_SESS_CONN_IND,
    OBEX_SA_WC_SESS_IND,
    OBEX_SA_SESSION_RSP,
    OBEX_SA_PUT_IND,
    OBEX_SA_SRM_PUT_REQ,
    OBEX_SA_SRM_PUT_RSP,
    OBEX_SA_SRM_GET_FCS,
    OBEX_SA_SRM_GET_RSP,
    OBEX_SA_SRM_GET_REQ,
    OBEX_SA_CLEAN_PORT
};

/* OBEX Server Action Functions */
static const tOBEX_SR_ACT obx_sr_action[] =
{
    obx_sa_close_port,
    obx_sa_connection_error,
    obx_sa_state,
    obx_sa_connect_ind,
    obx_sa_wc_conn_ind,
    obx_sa_nc_to,
    obx_sa_connect_rsp,
    obx_sa_snd_rsp,
    obx_sa_save_req,
    obx_sa_snd_part,
    obx_sa_rej_req,
    obx_sa_abort_rsp,
    obx_sa_op_rsp,
    obx_sa_get_ind,
    obx_sa_get_req,
#ifdef OBEX_LIB_SESSION_SUPPORTED
    obx_sa_session_ind,
    obx_sa_sess_conn_ind,
    obx_sa_wc_sess_ind,
    obx_sa_session_rsp,
#else
    NULL,
    NULL,
    NULL,
    NULL,
#endif
    obx_sa_put_ind,
    obx_sa_srm_put_req,
    obx_sa_srm_put_rsp,
    obx_sa_srm_get_fcs,
    obx_sa_srm_get_rsp,
    obx_sa_srm_get_req,
    obx_sa_clean_port
};

/************ OBX Server FSM State/Event Indirection Table **************/
/* obx_ssm_event() first looks at obx_ssm_entry_map[][] to get an entry of the event of a particular state
 * 0 means the event in the current state is ignored.
 * a number with 0x80 bit set, use obx_sr_all_table[][] as the "state table".
 * other numbers, look up obx_sr_main_state_table[] for the state table of current state.
 *
 * once the state table is determined,
 * look up the "action" column to find the associated action function
 * and the "next state" column to find the "next state" candidate.
 *
 * The actual next state could be either the state in the "next state" column
 * or the state returned from the action function.
 */
static const UINT8 obx_ssm_entry_map[][OBEX_SS_MAX-1] =
{
/* state name:  NtCon SesIn CntIn Conn  DscIn StpIn ActIn AbtIn PutIn GetIn Put   Get   PutS  GetS  Part WtCls */
/* CONN_R   */{ 1,    0x82, 0x82, 0x86, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82 ,1    },
/* SESS_R   */{ 3,    0x82, 0x82, 0x88, 3,    0x85, 0x85, 0x85, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 1    ,3    },
/* DISCNT_R */{ 0x86, 0x82, 0x82, 0x85, 0x82, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 1    ,0x86 },
/* PUT_R    */{ 0x86, 0x82, 0x82, 1,    0x87, 0x87, 0x87, 0x82, 0x87, 0x87, 1,    0x82, 1,    0x82, 0x82 ,0x86 },
/* GET_R    */{ 0x86, 0x82, 0x82, 2,    0x87, 0x87, 0x87, 0x82, 0x87, 0x87, 0x82, 1,    0x82, 1,    0x82 ,0x86 },
/* SETPATH_R*/{ 0x86, 0x82, 0x82, 3,    0x87, 0x87, 0x87, 0x82, 0x87, 0x87, 0x82, 0x82, 0x82, 0x82, 0x82 ,0x86 },
/* ACTION_R */{ 0x86, 0x82, 0x82, 4,    0x87, 0x87, 0x87, 0x82, 0x87, 0x87, 0x82, 0x82, 0x82, 0x82, 0x82 ,0x86 },
/* ABORT_R  */{ 0x86, 0x82, 0x82, 0x86, 0x82, 0x82, 0x82, 0x82, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 1    ,0x86 },
/* CONN_C   */{ 0,    0,    1,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0    ,0    },
/* SESS_C   */{ 0,    1,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0    ,0    },
/* DISCNT_C */{ 0,    0x82, 0x82, 0x82, 1,    0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82 ,0    },
/* PUT_C    */{ 0,    0,    0,    0,    0,    0,    0,    1,    1,    0,    0,    0,    2,    0,    0    ,0    },
/* GET_C    */{ 0,    0,    0,    0,    0,    0,    0,    1,    0,    1,    0,    0,    0,    2,    0    ,0    },
/* SETPATH_C*/{ 0,    0,    0,    0,    0,    1,    0,    0,    0,    0,    0,    0,    0,    0,    0    ,0    },
/* ACTION_C */{ 0,    0,    0,    0,    0,    0,    1,    0,    0,    0,    0,    0,    0,    0,    0    ,0    },
/* ABORT_C  */{ 0,    0,    0,    0,    0,    0,    0,    2,    0,    0,    0,    0,    0,    0,    0    ,0    },
/* PORT_CLS */{ 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83 ,0x83 },
/* FCS_SET  */{ 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    3,    2    ,0    },
/* STATE    */{ 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84 ,0x84 },
/* TIMEOUT  */{ 2,    0,    0,    0,    2,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0    ,2    },
/* BAD_REQ  */{ 0x86, 0x82, 0x82, 0x86, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82 ,0x86 },
/* TX_EMPTY */{ 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0    ,0    }
};

static const UINT8 obx_sr_all_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* ABORT_R  */{OBEX_SM_NO_ACTION,        OBEX_SS_ABORT_INDICATED },
/* misc     */{OBEX_SA_CLOSE_PORT,       OBEX_SS_NOT_CONNECTED },
/* PORT_CLS */{OBEX_SA_CONNECTION_ERROR, OBEX_SS_NOT_CONNECTED },
/* STATE    */{OBEX_SA_STATE,            OBEX_SS_NULL },
/* DISCNT_R */{OBEX_SM_NO_ACTION,        OBEX_SS_DISCNT_INDICATED },
/* misc     */{OBEX_SA_REJ_REQ,          OBEX_SS_NULL },
/* illegalop*/{OBEX_SA_CLEAN_PORT,       OBEX_SS_NOT_CONNECTED },
/* SESS_R   */{OBEX_SA_SESSION_IND,      OBEX_SS_SESS_INDICATED }
};

static const UINT8 obx_sr_not_conn_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* CONN_R   */{OBEX_SA_CONNECT_IND,      OBEX_SS_CONN_INDICATED },
/* TIMEOUT  */{OBEX_SA_NC_TO,            OBEX_SS_NOT_CONNECTED},
/* SESS_R   */{OBEX_SA_SESS_CONN_IND,    OBEX_SS_SESS_INDICATED }
};

static const UINT8 obx_sr_connect_ind_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* CONN_C   */{OBEX_SA_CONNECT_RSP,      OBEX_SS_CONNECTED }
};

static const UINT8 obx_sr_session_ind_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* SESS_R   */{OBEX_SA_SESSION_RSP,      OBEX_SS_NOT_CONNECTED }
};

static const UINT8 obx_sr_conn_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* PUT_R    */{OBEX_SA_PUT_IND,          OBEX_SS_PUT_INDICATED },
/* GET_R    */{OBEX_SA_GET_IND,          OBEX_SS_GET_INDICATED },
/* SETPATH_R*/{OBEX_SM_NO_ACTION,        OBEX_SS_SETPATH_INDICATED },
/* ACTION_R */{OBEX_SM_NO_ACTION,        OBEX_SS_ACTION_INDICATED }
};

static const UINT8 obx_sr_disconnect_ind_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* DISCNT_C */{OBEX_SA_SND_RSP,          OBEX_SS_WAIT_CLOSE },
/* TIMEOUT  */{OBEX_SA_CLOSE_PORT,       OBEX_SS_NOT_CONNECTED},
/* SESS_R   */{OBEX_SA_SESSION_IND,      OBEX_SS_DISCNT_INDICATED}
};

static const UINT8 obx_sr_setpath_ind_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* SETPATH_C*/{OBEX_SA_SND_RSP,          OBEX_SS_CONNECTED }
};

static const UINT8 obx_sr_abort_ind_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* GET/PUT_C*/{OBEX_SA_OP_RSP,           OBEX_SS_ABORT_INDICATED },
/* ABORT_C  */{OBEX_SA_ABORT_RSP,        OBEX_SS_CONNECTED }
};

static const UINT8 obx_sr_put_ind_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* PUT_C    */{OBEX_SA_SND_RSP,          OBEX_SS_PUT_TRANSACTION }
};

static const UINT8 obx_sr_get_ind_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* GET_C    */{OBEX_SA_SND_RSP,          OBEX_SS_GET_TRANSACTION }
};

static const UINT8 obx_sr_put_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* PUT_R    */{OBEX_SM_NO_ACTION,        OBEX_SS_PUT_INDICATED }
};

static const UINT8 obx_sr_get_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* GET_R    */{OBEX_SA_GET_REQ,          OBEX_SS_GET_INDICATED }
};

static const UINT8 obx_sr_put_s_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* PUT_R    */{OBEX_SA_SRM_PUT_REQ,      OBEX_SS_PUT_SRM },
/* PUT_C    */{OBEX_SA_SRM_PUT_RSP,      OBEX_SS_PUT_SRM }
};

static const UINT8 obx_sr_get_s_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* GET_R    */{OBEX_SA_SRM_GET_REQ,      OBEX_SS_GET_SRM },
/* GET_C    */{OBEX_SA_SRM_GET_RSP,      OBEX_SS_GET_SRM },
/* FCS_SET  */{OBEX_SA_SRM_GET_FCS,      OBEX_SS_GET_SRM }
};

static const UINT8 obx_sr_part_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* ABORT_R  */{OBEX_SA_SAVE_REQ,         OBEX_SS_NULL },/* and DISCNT_R */
/* FCS_SET  */{OBEX_SA_SND_PART,         OBEX_SS_NULL }
};

static const UINT8 obx_sr_wait_close_table[][OBEX_SM_NUM_COLS] = {
/* Event       Action                   Next State */
/* CONN_R   */{OBEX_SA_WC_CONN_IND,      OBEX_SS_CONN_INDICATED },
/* TIMEOUT  */{OBEX_SA_NC_TO,            OBEX_SS_NOT_CONNECTED},
/* SESS_R   */{OBEX_SA_WC_SESS_IND,      OBEX_SS_SESS_INDICATED }
};

static const tOBEX_SM_TBL obx_sr_main_state_table[] = {
    obx_sr_not_conn_table,
    obx_sr_session_ind_table,
    obx_sr_connect_ind_table,
    obx_sr_conn_table,
    obx_sr_disconnect_ind_table,
    obx_sr_setpath_ind_table,
    obx_sr_setpath_ind_table, /* same action table for action_ind */
    obx_sr_abort_ind_table,
    obx_sr_put_ind_table,
    obx_sr_get_ind_table,
    obx_sr_put_table,
    obx_sr_get_table,
    obx_sr_put_s_table,
    obx_sr_get_s_table,
    obx_sr_part_table,
    obx_sr_wait_close_table
};

/*******************************************************************************
**
** Function     obx_ssm_event
**
** Description  Handle events to the server state machine. It looks up the entry
**              in the obx_ssm_entry_map array. If it is a valid entry, it gets
**              the state table.Set the next state, if not NULL state.Execute
**              the action function according to the state table. If the state
**              returned by action function is not NULL state, adjust the new
**              state to the returned state.If (api_evt != MAX), call callback
**              function.
**
** Returns      void.
**
*******************************************************************************/
void obx_ssm_event(tOBEX_SR_SESS_CB *p_scb, tOBEX_SR_EVENT event, BT_HDR *p_msg)
{
    UINT8           curr_state = p_scb->state;
    tOBEX_SM_TBL     state_table;
    UINT8           action, entry;
    tOBEX_SR_STATE   act_state = OBEX_SS_NULL;
    UINT8           *p_data;
    UINT16          len;
    BT_HDR          *p_dummy;
    tOBEX_EVENT      api_evt;
    tOBEX_SR_CB      *p_cb;

    if( curr_state == OBEX_SS_NULL || curr_state >= OBEX_SS_MAX)
    {
        OBEX_TRACE_WARNING1( "Invalid state: %d\n", curr_state) ;
        if(p_msg)
            GKI_freebuf(p_msg);
        return;
    }

    OBEX_TRACE_DEBUG6( "For Server SHandle 0x%x, State: %s, Event: %s/%d srm:0x%x ssn:%d\n",
        p_scb->ll_cb.comm.handle, obx_sr_get_state_name( p_scb->state ),
        obx_sr_get_event_name(event), event, p_scb->srm, p_scb->ssn ) ;

    /* look up the state table for the current state */
    /* lookup entry /w event & curr_state */
    /* If entry is ignore, return.
     * Otherwise, get state table (according to curr_state or all_state) */
    /* coverity [index_parm] */
    if( (entry = obx_ssm_entry_map[event][curr_state-1]) != OBEX_SM_IGNORE )
    {
        if(entry&OBEX_SM_ALL)
        {
            entry &= OBEX_SM_ENTRY_MASK;
            state_table = obx_sr_all_table;
        }
        else
            state_table = obx_sr_main_state_table[curr_state-1];
    }
    else
    {
        OBEX_TRACE_WARNING2( "Ignore event %d in state %d\n", event, curr_state );
        if(p_msg)
            GKI_freebuf(p_msg);
        return;
    }

    /* Get possible next state from state table. */
    if( state_table[entry-1][OBEX_SME_NEXT_STATE] != OBEX_CS_NULL )
        p_scb->state = state_table[entry-1][OBEX_SME_NEXT_STATE];
    p_scb->prev_state = curr_state;
    OBEX_TRACE_DEBUG3( "possible new state = %s/%s/%d\n",
        obx_sr_get_state_name(p_scb->state), obx_sr_get_state_name( p_scb->prev_state ), p_scb->prev_state) ;

    /* If action is not ignore, clear param, exec action and get next state.
     * The action function may set the Param for cback.
     * Depending on param, call cback or free buffer. */
    /* execute action */
    action = state_table[entry-1][OBEX_SME_ACTION];
    if (action != OBEX_SM_NO_ACTION)
    {
        if (obx_sr_action[action] != NULL)
            act_state = (*obx_sr_action[action])(p_scb, p_msg);
    }

    /* adjust next state, if it needs to use the new state returned from action function */
    if( act_state != OBEX_CS_NULL)
    {
        p_scb->state = act_state;
        OBEX_TRACE_DEBUG1( "new state = %s (action)\n", obx_sr_get_state_name( p_scb->state ) ) ;
    }

    if(p_scb->api_evt)
    {
        api_evt         = p_scb->api_evt;
        p_scb->api_evt   = OBEX_NULL_EVT;
        /* we do not want the operation to be challenged by the client */
        if( event <= OBEX_SEVT_MAX_REQ && event != OBEX_CONNECT_REQ_SEVT &&
            OBEX_ReadByteStrHdr(p_msg, OBEX_HI_CHALLENGE, &p_data, &len, 0) == TRUE)
        {
            /* send bad request response */
            p_dummy = obx_build_dummy_rsp(p_scb, OBEX_RSP_BAD_REQUEST);
            event += OBEX_SEVT_DIFF_REQ_CFM;
            obx_ssm_event(p_scb, event, p_dummy);
            GKI_freebuf(p_msg);
        }
        else
        {
            p_cb = &obx_cb.server[p_scb->handle - 1];
            (p_cb->p_cback) (p_scb->ll_cb.comm.handle, api_evt, p_scb->param, (UINT8 *)p_msg);
        }
        memset(&p_scb->param, 0, sizeof (p_scb->param) );
    }
    else if(action == OBEX_SM_NO_ACTION && p_msg)
            GKI_freebuf(p_msg);


    OBEX_TRACE_DEBUG2( "result state = %s ssn:%d\n", obx_sr_get_state_name( p_scb->state ), p_scb->ssn ) ;
}
