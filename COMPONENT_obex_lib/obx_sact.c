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
**  Name:         obx_sact.c
**
**  File:         OBEX Server State Machine Action Functions
**
**
*****************************************************************************/
#include <string.h>

#include "obx_int.h"

/*******************************************************************************
** Function     obx_sa_snd_rsp
** Description  Call p_send_fn() to send the OBEX message to the peer.
**              Start timer. Return NULL state.If data is partially sent, set
**              next_state in port control block. Return PART state.
*******************************************************************************/
tOBEX_SR_STATE obx_sa_snd_rsp(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE   state       = p_scb->state;
    UINT8           rsp_code    = OBEX_RSP_DEFAULT;
    tOBEX_SR_CB      *p_cb;
    BOOLEAN         not_cong = TRUE;

    obx_access_rsp_code(p_pkt, &rsp_code);
    p_scb->cur_op    = OBEX_REQ_ABORT;

#if BT_TRACE_PROTOCOL == TRUE
    DispObxMsg(p_pkt, obx_api_evt_to_disp_type[p_pkt->event]);
#endif

    p_scb->ll_cb.comm.p_txmsg = p_pkt;
    rsp_code &= ~OBEX_FINAL;
    /* Get and Put operation may need to  adjust the state*/
    if (rsp_code != OBEX_RSP_CONTINUE )
    {
        if (p_scb->state == OBEX_SS_GET_TRANSACTION ||
            p_scb->state == OBEX_SS_PUT_SRM         ||
            p_scb->state == OBEX_SS_GET_SRM         ||
            p_scb->state == OBEX_SS_PUT_TRANSACTION)
        {
            state = OBEX_SS_CONNECTED;
            /* the SRM bits can not be cleared here, if aborting */
            if ((p_scb->srm &OBEX_SRM_ABORT) == 0)
                p_scb->srm &= OBEX_SRM_ENABLE;
        }
    }

    OBEX_TRACE_DEBUG2("obx_sa_snd_rsp sess_st:%d, event:%d", p_scb->sess_st, p_pkt->event);
    if ((p_scb->sess_st == OBEX_SESS_ACTIVE) && (p_pkt->event != (OBEX_SESSION_CFM_SEVT + 1)))
    {
        p_scb->ssn++;
    }

    if (p_scb->ll_cb.comm.p_send_fn(&p_scb->ll_cb) == FALSE)
    {
        p_scb->next_state   = state;
        state = OBEX_SS_PARTIAL_SENT;
    }
    else if (p_scb->state == OBEX_SS_GET_SRM)
    {
        if (((p_scb->srmp & OBEX_SRMP_WAIT) == 0) && (rsp_code == OBEX_RSP_CONTINUE))
        {
#ifdef OBEX_LIB_L2CAP_INCLUDED
            if (p_scb->ll_cb.comm.p_send_fn == (tOBEX_SEND_FN *)obx_l2c_snd_msg)
            {
                OBEX_TRACE_DEBUG1("obx_sa_snd_rsp cong:%d", p_scb->ll_cb.l2c.cong);
                if (p_scb->ll_cb.l2c.cong)
                {
                    not_cong = FALSE;
                }
            }
#endif

            /* do not need to wait
             - fake a get request event, so the profile would issue another GET response */
            if (not_cong)
            {
                p_cb = obx_sr_get_cb(p_scb->handle);
                (p_cb->p_cback) (p_scb->ll_cb.comm.handle, OBEX_GET_REQ_EVT, p_scb->param, NULL);
            }
        }
        p_scb->srmp &= ~OBEX_SRMP_WAIT;
    }
    else if (p_scb->sess_st == OBEX_SESS_SUSPENDING)
    {
        p_scb->ssn++;
        p_scb->param.sess.ssn = p_scb->ssn;
        p_scb->param.sess.nssn = p_scb->ssn;
        p_scb->sess_st = OBEX_SESS_SUSPEND;
        p_scb->sess_info[OBEX_SESSION_INFO_ST_IDX] = p_scb->state;
        state = OBEX_SS_SESS_INDICATED;
        p_scb->api_evt   = OBEX_SESSION_REQ_EVT;
    }

    return state;
}

/*******************************************************************************
** Function     obx_sa_snd_part
** Description  Call p_send_fn() to send the left-over OBEX message to the
**              peer. Start timer. If all the data is sent, call obx_ssm_event()
**              with STATE event to next_state in the port control block.
**              If (p_saved), call obx_ssm_event() to process the saved request.
*******************************************************************************/
tOBEX_SR_STATE obx_sa_snd_part(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE state = OBEX_SS_NULL;
    UINT8   rsp_code = OBEX_RSP_DEFAULT;
    tOBEX_SR_CB      *p_cb;

    obx_access_rsp_code(p_scb->ll_cb.comm.p_txmsg , &rsp_code);
    if (p_scb->ll_cb.comm.p_send_fn(&p_scb->ll_cb) == TRUE)
    {
        obx_ssm_event(p_scb, OBEX_STATE_SEVT, NULL);
        if (p_scb->p_next_req)
        {
            p_pkt = p_scb->p_next_req;
            obx_access_rsp_code(p_pkt , &rsp_code);
            p_scb->p_next_req = NULL;
            p_scb->api_evt    = (tOBEX_EVENT)p_pkt->event;
            obx_ssm_event(p_scb, (tOBEX_SR_EVENT)(p_pkt->event-1), p_pkt);
        }

        OBEX_TRACE_DEBUG3("obx_sa_snd_part state:%d, srm:0x%x, rsp_code:0x%x", p_scb->state, p_scb->srm, rsp_code);
        if (p_scb->state == OBEX_SS_GET_SRM)
        {
            rsp_code &= ~OBEX_FINAL;
            if (((p_scb->srm & OBEX_SRM_WAIT) == 0) && (rsp_code == OBEX_RSP_CONTINUE))
            {
                /* do not need to wait
                 - fake a get request event, so the profile would issue another GET response */
                p_cb = obx_sr_get_cb(p_scb->handle);
                (p_cb->p_cback) (p_scb->ll_cb.comm.handle, OBEX_GET_REQ_EVT, p_scb->param, NULL);
            }
        }
    }
    return state;
}

/*******************************************************************************
** Function     obx_sa_abort_rsp
** Description  Send an abort response.
**              If Put/Get response has not been sent yet,
**              send it before the abort response.
*******************************************************************************/
tOBEX_SR_STATE obx_sa_abort_rsp(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE state = OBEX_SS_NULL;
    BT_HDR       *p_dummy;
    UINT8        rsp_code = OBEX_RSP_CONTINUE;

    if (p_scb->cur_op != OBEX_REQ_ABORT && ((p_scb->srm & OBEX_SRM_ENGAGE) == 0))
    {
        /* if we have not respond to an op yet, send a dummy response */
        if (p_scb->cur_op == (rsp_code|OBEX_REQ_PUT) )
            rsp_code = OBEX_RSP_INTRNL_SRVR_ERR;
        p_dummy = obx_build_dummy_rsp(p_scb, rsp_code);
        obx_sa_snd_rsp(p_scb, p_dummy);
    }

    /* clear the SRM bits; leave only the enabled bit */
    p_scb->srm &= OBEX_SRM_ENABLE;
    state = obx_sa_snd_rsp(p_scb, p_pkt);
    return state;
}

/*******************************************************************************
** Function     obx_sa_op_rsp
** Description  Send response for Put/Get when Abort request is already received
*******************************************************************************/
tOBEX_SR_STATE obx_sa_op_rsp(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE state = OBEX_SS_NULL;
    if (p_scb->cur_op != OBEX_REQ_ABORT)
        state = obx_sa_snd_rsp(p_scb, p_pkt);
#if (BT_USE_TRACES == TRUE)
    else
        OBEX_TRACE_WARNING0("OBX is not waiting for a rsp API!!");
#endif
    return state;
}

/*******************************************************************************
** Function     obx_verify_target
** Description  Verify that target header or connection ID is correct.
**              Make sure that they do not both exist
*******************************************************************************/
static UINT8 obx_verify_target(tOBEX_SR_CB *p_cb, tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    UINT16  len = 0;
    UINT8   rsp_code    = OBEX_RSP_SERVICE_UNAVL;
    UINT32  conn_id     = 0;
    UINT8   *p_target   = NULL;

    OBEX_Read4ByteHdr(p_pkt, OBEX_HI_CONN_ID, &conn_id);

    /* If target header does not exist, p_target would remain the default value/NULL
       and len would be set to 0. There's no need to check the return value of
       OBEX_ReadTargetHdr */
    wiced_bt_obex_find_byte_sequence_header((UINT8 *)p_pkt, OBEX_HI_TARGET, &p_target, &len);

    if (p_cb->target.len)
    {
        /* directed connection: make sure the connection ID matches */
        if ( conn_id && conn_id == p_scb->conn_id )
            rsp_code    = OBEX_RSP_OK;

        if (p_cb->target.len == OBEX_DEFAULT_TARGET_LEN)
        {
            /* the user verify target (cases like BIP that has multiple targets) */
            rsp_code = OBEX_RSP_OK;
        }
        else if ( len == p_cb->target.len && p_target &&
            memcmp(p_cb->target.target, p_target, p_cb->target.len) == 0)
        {
            rsp_code = OBEX_RSP_OK;
        }
    }
    else
    {
        /* no target - request to inbox, like OPP */
        if (conn_id == 0 && len == 0)
        {
            if (p_scb->ll_cb.comm.tx_mtu < OBEX_MIN_MTU)
                rsp_code = OBEX_RSP_FORBIDDEN;
            else
                rsp_code = OBEX_RSP_OK;
        }
    }

    /* target header and connection ID are not supposed to exist in the same packet*/
    if (conn_id != 0 && p_target != NULL)
        rsp_code = OBEX_RSP_BAD_REQUEST;

    OBEX_TRACE_DEBUG3("obx_verify_target rsp: %x, id:%x, code:%x",
        rsp_code, conn_id, ((tOBEX_RX_HDR *)(p_pkt + 1))->code);
    if (rsp_code != OBEX_RSP_OK)
        p_pkt->event = OBEX_CONNECT_RSP_EVT;
    return rsp_code;
}

/*******************************************************************************
** Function     obx_conn_rsp
** Description  Called by OBEX_ConnectRsp() and obx_sa_connect_ind() to compose
**              a connect response packet.
*******************************************************************************/
BT_HDR * obx_conn_rsp(tOBEX_SR_CB *p_cb, tOBEX_SR_SESS_CB *p_scb, UINT8 rsp_code, BT_HDR *p_pkt)
{
    UINT8       msg[OBEX_HDR_OFFSET + OBEX_MAX_CONN_HDR_EXTRA];
    UINT8       *p = msg;

    /* response packets always have the final bit set */
    *p++ = (rsp_code | OBEX_FINAL);
    p += OBEX_PKT_LEN_SIZE;

    *p++ = OBEX_VERSION;
    *p++ = OBEX_CONN_FLAGS;
    UINT16_TO_BE_STREAM(p, p_scb->ll_cb.comm.rx_mtu);

    /* add session sequence number, if session is active */
    if (p_scb->sess_st == OBEX_SESS_ACTIVE)
    {
        *p++ = OBEX_HI_SESSION_SN;
        *p++ = (p_scb->ssn+1);
    }

    if (p_scb->conn_id && rsp_code == OBEX_RSP_OK)
    {
        *p++    = OBEX_HI_CONN_ID;
        UINT32_TO_BE_STREAM(p, p_scb->conn_id);
    }

    p_pkt = obx_sr_prepend_msg(p_pkt, msg, (UINT16)(p - msg) );

    /* If the target is registered to server and the WHO headers not in the packet
     * add WHO header here */
    p_pkt->event    = OBEX_CONNECT_RSP_EVT;
    if (p_cb->target.len && p_cb->target.len != OBEX_DEFAULT_TARGET_LEN &&
        OBEX_CheckHdr(p_pkt, OBEX_HI_WHO) == NULL)
    {
        OBEX_AddByteStrHdr(p_pkt, OBEX_HI_WHO, p_cb->target.target, p_cb->target.len);
        /* adjust the packet len */
        obx_adjust_packet_len(p_pkt);
    }
    return p_pkt;
}

/*******************************************************************************
** Function     obx_sa_wc_conn_ind
** Description  Connect Req is received when waiting for the port to close
**              Call callback with OBEX_CLOSE_IND_EVT to clean up the profiles
**              then call obx_sa_connect_ind to process the connect req.
*******************************************************************************/
tOBEX_SR_STATE obx_sa_wc_conn_ind(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_EVT_PARAM  param;
    tOBEX_SR_CB      *p_cb = obx_sr_get_cb(p_scb->handle);

    obx_stop_timer(&p_scb->ll_cb.comm.tle);
    memset(&param, 0, sizeof(tOBEX_EVT_PARAM));
    if (p_cb && p_cb->p_cback)
        (*p_cb->p_cback)(p_scb->ll_cb.comm.handle, OBEX_CLOSE_IND_EVT, param, NULL);
    obx_sa_connect_ind(p_scb, p_pkt);
    return OBEX_SS_NULL;
}

/*******************************************************************************
** Function     obx_sa_connect_ind
** Description  Save peer MTU in the server control block.
*******************************************************************************/
tOBEX_SR_STATE obx_sa_connect_ind(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE   state = OBEX_SS_NULL;
    UINT8           rsp_code = OBEX_RSP_SERVICE_UNAVL;
    tOBEX_SR_CB      *p_cb = obx_sr_get_cb(p_scb->handle);
    UINT8           *p;
    tOBEX_EVT_PARAM  param;              /* The event parameter. */

    /* clear the SRM bits; leave only the enabled bit */
    p_scb->srm &= OBEX_SRM_ENABLE;

    OBEX_TRACE_DEBUG0("obx_sa_connect_ind");
    p_scb->api_evt = OBEX_NULL_EVT;

    /* verify that the connect request is OK */
    rsp_code = obx_verify_target(p_cb, p_scb, p_pkt);
    if (rsp_code == OBEX_RSP_OK)
    {
        if (p_cb->target.len && p_scb->conn_id == 0)
        {
            /* if Connection ID is used for this connection and none assigned yet,
             * - assign one  */
            p_scb->conn_id = obx_sr_get_next_conn_id();
            OBEX_TRACE_DEBUG1(" **** obx_sr_get_next_conn_id (0x%08x)", p_scb->conn_id);
        }

        if (p_scb->sess_st == OBEX_SESS_ACTIVE)
        {
            p = &p_scb->sess_info[OBEX_SESSION_INFO_ID_IDX];
            UINT32_TO_BE_STREAM(p, p_scb->conn_id);
            param.sess.p_sess_info   = p_scb->sess_info;
            param.sess.sess_op       = OBEX_SESS_OP_CREATE;
            param.sess.sess_st       = p_scb->sess_st;
            param.sess.nssn          = p_scb->param.ssn;
            param.sess.ssn           = p_scb->param.ssn;
            param.sess.obj_offset    = 0;
            p = &p_scb->sess_info[OBEX_SESSION_INFO_MTU_IDX];
            UINT16_TO_BE_STREAM(p, p_scb->param.conn.mtu);
            memcpy(param.sess.peer_addr, p_scb->peer_addr, BD_ADDR_LEN);
            (*p_cb->p_cback)(p_scb->ll_cb.comm.handle, OBEX_SESSION_REQ_EVT, param, NULL);
        }
        (*p_cb->p_cback)(p_scb->ll_cb.comm.handle, OBEX_CONNECT_REQ_EVT, p_scb->param, (UINT8 *)p_pkt);
    }
    else
    {
        /* bad target or connection ID - reject the request */
        rsp_code    |= OBEX_FINAL;
        obx_access_rsp_code(p_pkt, &rsp_code);
        state = OBEX_SS_NOT_CONNECTED;
    }

    if (state != OBEX_SS_NULL)
    {
        /* each port has its own credit.
         * It's very unlikely that we can be flow controlled here */
        obx_sa_snd_rsp(p_scb, p_pkt);
    }

    return state;
}

/*******************************************************************************
** Function     obx_sa_connect_rsp
** Description  obx_sa_snd_rsp().If(p_saved), free the OBEX message.If OK
**              response, return NULL state. If fail response, return
**              NOT_CONN state.
*******************************************************************************/
tOBEX_SR_STATE obx_sa_connect_rsp(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    UINT8   rsp_code = OBEX_RSP_DEFAULT;
    tOBEX_SR_STATE   state   = p_scb->state;

    obx_access_rsp_code(p_pkt, &rsp_code);

    if (p_scb->p_saved_msg)
    {
        GKI_freebuf(p_scb->p_saved_msg);
        p_scb->p_saved_msg   = NULL;
    }

    if (rsp_code != (OBEX_RSP_OK | OBEX_FINAL) )
        state = OBEX_SS_NOT_CONNECTED;

    /* each port has its own credit.
     * It's very unlikely that we can be flow controlled here */
    obx_sa_snd_rsp(p_scb, p_pkt);

    return state;
}

/*******************************************************************************
** Function     obx_sa_connection_error
** Description  Stop timer. Reopen transport. Call callback function with
**              OBEX_CLOSE_IND_EVT.
*******************************************************************************/
tOBEX_SR_STATE obx_sa_connection_error(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_EVT_PARAM  param;
    tOBEX_SR_CB      *p_cb = obx_sr_get_cb(p_scb->handle);
    tOBEX_SR_CBACK   *p_cback = NULL;
    tOBEX_SR_STATE   save_state;

    OBEX_TRACE_DEBUG4("obx_sa_connection_error tx_mtu: %d, sess_st:%d state:%d, prev_state:%d",
        p_scb->ll_cb.comm.tx_mtu, p_scb->sess_st, p_scb->state, p_scb->prev_state);

    if (p_cb)
    {
        p_cback = p_cb->p_cback;
        memset(&param, 0, sizeof(tOBEX_EVT_PARAM));
    }

    /* clear buffers from previous connection */
    obx_free_buf (&p_scb->ll_cb);

    if (p_scb->sess_st == OBEX_SESS_ACTIVE)
    {
        /* The transport is interrupted while a reliable session is active:
         * report a suspend event fot application to save the information in NV */
        save_state = p_scb->prev_state;
        p_scb->sess_info[OBEX_SESSION_INFO_SRM_IDX] = p_scb->srm;
        param.sess.p_sess_info   = p_scb->sess_info;
        param.sess.sess_op       = OBEX_SESS_OP_TRANSPORT;
        param.sess.sess_st       = p_scb->sess_st;
        param.sess.nssn          = p_scb->ssn;
        param.sess.ssn           = p_scb->ssn;
        param.sess.obj_offset    = 0;
        param.sess.timeout       = OBEX_SESS_TIMEOUT_VALUE;
        if (save_state == OBEX_SS_PARTIAL_SENT)
            save_state = p_scb->next_state;

        if ((p_scb->srm & OBEX_SRM_ENGAGE) == 0)
        {
            /* SRM is not engaged.
             * When the session is resume, client needs to send the request first,
             * the save ssm state may need to be adjusted */
            if (save_state == OBEX_SS_PUT_INDICATED)
            {
                save_state = OBEX_SS_PUT_TRANSACTION;
            }
            else if (save_state == OBEX_SS_GET_INDICATED)
            {
                save_state = OBEX_SS_GET_TRANSACTION;
            }
        }
        p_scb->sess_info[OBEX_SESSION_INFO_ST_IDX] = save_state;
        OBEX_TRACE_DEBUG2("saved state:0x%x, srm:0x%x", save_state, p_scb->sess_info[OBEX_SESSION_INFO_SRM_IDX]);
        memcpy(param.sess.peer_addr, p_scb->peer_addr, BD_ADDR_LEN);
        p_scb->sess_st = OBEX_SESS_NONE;
        if (p_cback)
            (*p_cback)(p_scb->ll_cb.comm.handle, OBEX_SESSION_INFO_EVT, param, NULL);
    }
    else if (p_scb->sess_st == OBEX_SESS_CLOSE || p_scb->state == OBEX_SS_NOT_CONNECTED)
        p_scb->sess_st = OBEX_SESS_NONE;

    if (p_scb->ll_cb.comm.tx_mtu != 0)
    {
        p_scb->ll_cb.comm.tx_mtu = 0;
        obx_stop_timer(&p_scb->ll_cb.comm.tle);
        p_scb->conn_id    = 0;
        if (p_cback)
            (*p_cback)(p_scb->ll_cb.comm.handle, OBEX_CLOSE_IND_EVT, param, (UINT8 *)p_pkt);
    }

#ifdef OBEX_LIB_L2CAP_INCLUDED
    if (p_scb->ll_cb.comm.p_send_fn == (tOBEX_SEND_FN *)obx_l2c_snd_msg)
    {
        p_scb->ll_cb.comm.id = 0; /* mark this port unused. */
        p_scb->srm &= OBEX_SRM_ENABLE;
        obx_add_port (p_scb->handle);
    }
#endif

    return OBEX_SS_NULL;
}

/*******************************************************************************
** Function     obx_sa_close_port
** Description  Close transport. Start timer.
*******************************************************************************/
tOBEX_SR_STATE obx_sa_close_port(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    if (p_pkt)
        GKI_freebuf(p_pkt);
    p_scb->ll_cb.comm.p_close_fn(p_scb->ll_cb.comm.id);
    obx_sr_free_scb(p_scb);
    obx_start_timer(&p_scb->ll_cb.comm);
    return OBEX_SS_NULL;
}

/*******************************************************************************
** Function     obx_sa_clean_port
** Description  Close transport and clean up api_evt if illegal obex message is
**              received.
*******************************************************************************/
tOBEX_SR_STATE obx_sa_clean_port(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    p_scb->api_evt = OBEX_NULL_EVT;
    return obx_sa_close_port(p_scb, p_pkt);
}
/*******************************************************************************
** Function     obx_sa_state
** Description  change state
*******************************************************************************/
tOBEX_SR_STATE obx_sa_state(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    return p_scb->next_state;
}

/*******************************************************************************
** Function     obx_sa_nc_to
** Description  Timer expires in not_conn state
**              if there is existing connections -> disconnect
*******************************************************************************/
tOBEX_SR_STATE obx_sa_nc_to(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    BD_ADDR bd_addr;

    obx_stop_timer(&p_scb->ll_cb.comm.tle);
    /* if there is existing connections -> disconnect */
    if (wiced_bt_obex_get_peer_addr(p_scb->ll_cb.comm.handle, bd_addr) != 0)
    {
        p_scb->ll_cb.comm.p_close_fn(p_scb->ll_cb.comm.id);
        p_scb->conn_id    = 0;
        /* wait for the conn err event to re-open the port */
    }
    return OBEX_SS_NULL;
}

/*******************************************************************************
** Function     obx_sa_save_req
** Description  When a request received from peer in PART state,
**              save the request for later processing
*******************************************************************************/
tOBEX_SR_STATE obx_sa_save_req(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    if (p_scb->p_next_req)
    {
        GKI_freebuf(p_scb->p_next_req);
    }
    p_scb->p_next_req    = p_pkt;
    p_scb->api_evt       = OBEX_NULL_EVT;
    return OBEX_SS_NULL;
}

/*******************************************************************************
** Function     obx_sa_rej_req
** Description  Send bad request response when request comes in bad state
*******************************************************************************/
tOBEX_SR_STATE obx_sa_rej_req(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    UINT8       msg[OBEX_HDR_OFFSET];
    UINT8       *p = msg;
    tOBEX_RX_HDR *p_rxh;
    UINT32      conn_id     = p_scb->conn_id;
    UINT8       req_code;
    tOBEX_SR_STATE   state = OBEX_SS_NULL;

    p_rxh   = (tOBEX_RX_HDR *)(p_pkt + 1);
    req_code            = p_rxh->code & ~OBEX_FINAL;

    OBEX_TRACE_DEBUG0( "obx_sa_rej_req" ) ;
    if (p_pkt)
        GKI_freebuf(p_pkt);

    /* response packets always have the final bit set */
    /* Always send OK for disconnect response even when bad packet BT IrDA Spec */
    if (req_code == OBEX_REQ_DISCONNECT)
    {
        if (p_scb->conn_id)
        {
            OBEX_Read4ByteHdr (p_pkt, OBEX_HI_CONN_ID, &conn_id);
        }
        *p++    = (OBEX_RSP_OK | OBEX_FINAL);
    }
    else
    {
        *p++    = (OBEX_RSP_SERVICE_UNAVL | OBEX_FINAL);
    }
    p       += OBEX_PKT_LEN_SIZE;

    /* add session sequence number, if session is active */
    if (p_scb->sess_st == OBEX_SESS_ACTIVE)
    {
        *p++ = OBEX_HI_SESSION_SN;
        *p++ = (p_scb->ssn+1);
    }

    /* add connection ID, if needed */
    if (p_scb->conn_id)
    {
        *p++    = OBEX_HI_CONN_ID;
        UINT32_TO_BE_STREAM(p, conn_id);
    }

    p_pkt = obx_sr_prepend_msg(NULL, msg, (UINT16)(p - msg) );
    p_pkt->event    = OBEX_PUT_RSP_EVT; /* any response */
    p_scb->api_evt   = OBEX_NULL_EVT;

    state =  obx_sa_snd_rsp(p_scb, p_pkt);
    if (req_code == OBEX_REQ_DISCONNECT)
        state = OBEX_SS_DISCNT_INDICATED;
    return state;
}

/*******************************************************************************
** Function     obx_sa_get_ind
** Description  received a GET request from the client. Check if SRM is engaged.
*******************************************************************************/
tOBEX_SR_STATE obx_sa_get_ind(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE state = OBEX_SS_NULL;

    if ((p_scb->srm & OBEX_SRM_REQING) && (p_scb->srm & OBEX_SRM_ENABLE))
    {
        state   = OBEX_CS_GET_SRM;
    }

    /* the GET request does not set the final bit.
     * Save this request, send response automatically and do not report the event */
    if ((!p_scb->param.get.final) && ((p_scb->srmp & OBEX_SRMP_NONF_EVT) == 0))
    {
        p_scb->srmp |= OBEX_SRMP_NONF;
        if (p_scb->p_saved_msg)
            GKI_freebuf(p_scb->p_saved_msg);
        p_scb->p_saved_msg = p_pkt;
        p_scb->api_evt   = OBEX_NULL_EVT;
        wiced_bt_obex_send_response(p_scb->ll_cb.comm.handle, OBEX_REQ_GET, OBEX_RSP_CONTINUE, NULL);
    }
    return state;
}

/*******************************************************************************
** Function     obx_merge_get_req
** Description  merge the given 2 GET request packets and return the merged result
*******************************************************************************/
BT_HDR * obx_merge_get_req(BT_HDR *p_pkt1, BT_HDR *p_pkt2)
{
    BT_HDR *p_ret = p_pkt1;
    UINT16  size, need;
    UINT8   *p1, *p2;
    UINT8   *p, pre_size = OBEX_GET_HDRS_OFFSET;

    /* skip the connection ID header */
    p = (UINT8 *)(p_pkt2 + 1) + p_pkt2->offset;
    if (*p == OBEX_HI_CONN_ID)
        pre_size += 5;

    need = p_pkt1->len + p_pkt2->len;
    if ((p_pkt2->len == pre_size) || (need >= OBEX_LRG_DATA_POOL_SIZE))
    {
        GKI_freebuf (p_pkt2);
        return p_pkt1;
    }

    /* get rid of the GET request header - opcode(1) + packet len(2) (and maybe connection ID) before merging */
    p_pkt2->len -= pre_size;
    p_pkt2->offset += pre_size;
    size = GKI_get_buf_size(p_pkt1);

    if (size < need)
    {
        /* the original p_pkt1 is too small.
         * Allocate a bigger GKI buffer, p_ret, and copy p_pkt1 into p_ret */
        if (need < GKI_MAX_BUF_SIZE)
        {
            /* Use the largest general pool to allow challenge tags appendage */
            p_ret = (BT_HDR *)GKI_getbuf(GKI_MAX_BUF_SIZE);
        }
        else
        {
            p_ret = (BT_HDR *) GKI_getpoolbuf(OBEX_LRG_DATA_POOL_ID);
        }
        if (p_ret != NULL)
        {
            memcpy (p_ret, p_pkt1, sizeof (BT_HDR));
            p_ret->offset = 0;
            p1 = (UINT8 *)(p_ret + 1);
            p2 = (UINT8 *)(p_pkt1 + 1) + p_pkt1->offset;
            memcpy (p1, p2, p_pkt1->len);
        }
        GKI_freebuf (p_pkt1);
    }

    if (p_ret != NULL)
    {
        /* adjust the actualy packet length to reflect the combined packet and copy p_pkt2 into p_ret */
        p1 = (UINT8 *)(p_ret + 1) + p_ret->offset + 1;
        size = p_ret->len + p_pkt2->len;
        UINT16_TO_BE_STREAM(p1, size);
        p1 = (UINT8 *)(p_ret + 1) + p_ret->offset + p_ret->len;
        p2 = (UINT8 *)(p_pkt2 + 1) + p_pkt2->offset;
        p_ret->len = size;
        memcpy (p1, p2, p_pkt2->len);
    }
    GKI_freebuf (p_pkt2);

    return p_ret;
}

/*******************************************************************************
** Function     obx_sa_get_req
** Description
*******************************************************************************/
tOBEX_SR_STATE obx_sa_get_req(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE   state = OBEX_SS_NULL;
    tOBEX_SR_CB      *p_cb;

    OBEX_TRACE_DEBUG2("obx_sa_get_req srmp:0x%x final:%d", p_scb->srmp, p_scb->param.get.final);
    if (p_scb->srmp & OBEX_SRMP_NONF)
    {
        /* the GET request does not set the final bit yet.
         * merge this request, send response automatically and do not report the event */
        if (!p_scb->param.get.final)
        {
            p_scb->p_saved_msg  = obx_merge_get_req(p_scb->p_saved_msg, p_pkt);
            p_scb->api_evt      = OBEX_NULL_EVT;
            wiced_bt_obex_send_response(p_scb->ll_cb.comm.handle, OBEX_REQ_GET, OBEX_RSP_CONTINUE, NULL);
        }
        else
        {
            p_scb->srmp        &= ~OBEX_SRMP_NONF;
            p_pkt               = obx_merge_get_req(p_scb->p_saved_msg, p_pkt);
#if BT_TRACE_PROTOCOL == TRUE
            DispObxMsg(p_pkt, (BOOLEAN)(OBEX_GET_REQ_DISP | OBEX_DISP_IS_RECV));
#endif
            p_scb->p_saved_msg  = NULL;
            p_scb->api_evt      = OBEX_NULL_EVT;
            p_cb = &obx_cb.server[p_scb->handle - 1];
            (p_cb->p_cback) (p_scb->ll_cb.comm.handle, OBEX_GET_REQ_EVT, p_scb->param, (UINT8 *)p_pkt);
            memset(&p_scb->param, 0, sizeof (p_scb->param) );
        }
    }

    return state;
}

/*******************************************************************************
** Function     obx_sa_make_sess_id
** Description  compute the session id
*******************************************************************************/
static tOBEX_STATUS obx_sa_make_sess_id (tOBEX_SR_SESS_CB *p_scb, UINT8 *p_sess_info,
                                        tOBEX_TRIPLET *p_triplet, UINT8 num_triplet)
{
    UINT8           data[10];
    UINT8           ind;
    UINT8           *p;

    /* check the device address session parameter */
    ind = obx_read_triplet(p_triplet, num_triplet, OBEX_TAG_SESS_PARAM_ADDR);
    OBEX_TRACE_DEBUG2("addr ind:%d, num:%d", ind, num_triplet);
    if (ind == num_triplet || p_triplet[ind].len != BD_ADDR_LEN)
    {
        OBEX_TRACE_ERROR0("No Device Addr parameter");
        return OBEX_BAD_PARAMS;
    }

    if (memcmp (p_scb->peer_addr, p_triplet[ind].p_array, BD_ADDR_LEN) != 0)
    {
        OBEX_TRACE_ERROR0("Bad Device Addr parameter");
        return OBEX_BAD_PARAMS;
    }

    /* check the nonce session parameter */
    ind = obx_read_triplet(p_triplet, num_triplet, OBEX_TAG_SESS_PARAM_NONCE);
    OBEX_TRACE_DEBUG2("nonce ind:%d, num:%d", ind, num_triplet);
    if (ind == num_triplet || (p_triplet[ind].len < OBEX_MIN_NONCE_SIZE) || (p_triplet[ind].len > OBEX_NONCE_SIZE))
    {
        OBEX_TRACE_ERROR0("No Nonce parameter");
        return OBEX_BAD_PARAMS;
    }
    p = data;
    wiced_bt_dev_read_local_addr (p);

    /* compute the session ID */
    obx_session_id (p_sess_info, p_scb->peer_addr, p_triplet[ind].p_array, p_triplet[ind].len,
                                    data, &p_scb->sess_info[OBEX_SESSION_ID_SIZE], OBEX_LOCAL_NONCE_SIZE);

    return OBEX_SUCCESS;
}

/*******************************************************************************
** Function     obx_sa_session_ind
** Description  process session request from client
**              when the session request is received is not OBEX_SS_NOT_CONNECTED
*******************************************************************************/
tOBEX_SR_STATE obx_sa_session_ind(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_TRIPLET    triplet[OBEX_MAX_SESS_PARAM_TRIP];
    UINT8           num = OBEX_MAX_SESS_PARAM_TRIP, ind;
    UINT8           *p;
    tOBEX_SR_CB      *p_cb = obx_sr_get_cb(p_scb->handle);
    UINT8           rsp_code = OBEX_RSP_FORBIDDEN;
    BOOLEAN         now = FALSE;
#if (BT_USE_TRACES == TRUE)
    tOBEX_SESS_ST    old_sess_st = p_scb->sess_st;
#endif
    tOBEX_EVENT      old_api_evt;
    UINT32          obj_offset = 0;
    UINT8           ind_to;
    UINT32          timeout = OBEX_INFINITE_TIMEOUT;
    tOBEX_SR_STATE   state = OBEX_SS_NULL;

    OBEX_TRACE_DEBUG0("obx_sa_session_ind");
    OBEX_ReadTriplet(p_pkt, OBEX_HI_SESSION_PARAM, triplet, &num);
    if (p_cb->nonce == 0)
    {
        OBEX_TRACE_ERROR0("reliable session is not supported by this server");
        /* do not report the session_req_evt */
        p_scb->api_evt   = OBEX_NULL_EVT;
        obx_prepend_rsp_msg(p_scb->ll_cb.comm.handle, OBEX_SESSION_CFM_SEVT,
                            OBEX_RSP_NOT_IMPLEMENTED, NULL);
        return OBEX_SS_NULL;
    }
    else if (num)
    {
        ind = obx_read_triplet(triplet, num, OBEX_TAG_SESS_PARAM_SESS_OP);
        OBEX_TRACE_DEBUG2("sess_op ind:%d, num:%d", ind, num);
        if ((ind != num) && (triplet[ind].len == OBEX_LEN_SESS_PARAM_SESS_OP))
        {
            p = triplet[ind].p_array;
            p_scb->param.sess.sess_op = *p;
            OBEX_TRACE_DEBUG1("sess_op :%d", *p);
            switch (*p)
            {
            case OBEX_SESS_OP_CREATE:
                /* do not report the API event */
                p_scb->api_evt   = OBEX_NULL_EVT;
                /* the session is already active; reject with Service Unavailable */
                rsp_code = OBEX_RSP_SERVICE_UNAVL;
                break;

            case OBEX_SESS_OP_CLOSE:
                /* verify that the session ID matches an existing one. Otherwise, FORBIDDEN */
                if (p_scb->sess_st != OBEX_SESS_NONE)
                {
                    ind = obx_read_triplet(triplet, num, OBEX_TAG_SESS_PARAM_SESS_ID);
                    if (ind == num || triplet[ind].len != OBEX_SESSION_ID_SIZE)
                        break;
                    if (memcmp (p_scb->sess_info, triplet[ind].p_array, OBEX_SESSION_ID_SIZE) != 0)
                    {
                        /* bad session id */
                        break;
                    }
                    /* must be closing a good session 0 send the response now */
                    now = TRUE;
                    rsp_code = OBEX_RSP_OK;
                    p_scb->sess_st = OBEX_SESS_CLOSE;
                }
                break;

            case OBEX_SESS_OP_SUSPEND:
                /* verify that a session is active. Otherwise, FORBIDDEN */
                if (p_scb->sess_st == OBEX_SESS_ACTIVE)
                {
                    rsp_code = OBEX_RSP_OK;
                    p_scb->sess_st = OBEX_SESS_SUSPEND;
                    p_scb->sess_info[OBEX_SESSION_INFO_ST_IDX] = p_scb->prev_state;
                    p_scb->sess_info[OBEX_SESSION_INFO_SRM_IDX] = p_scb->srm;
                    /* save the session state in a GKI buffer on OBEX_SessionRsp */
                    if (p_scb->prev_state == OBEX_SS_PUT_INDICATED || p_scb->prev_state == OBEX_SS_GET_INDICATED)
                    {
                        /* out of sequence suspend:
                         * report the suspend event when the PutRsp or GetRsp is called
                         * this would allow server to resume the session in the right state */
                        p_scb->api_evt   = OBEX_NULL_EVT;
                        p_scb->sess_st   = OBEX_SESS_SUSPENDING;
                        state = p_scb->prev_state;
                    }
                }
                break;

            case OBEX_SESS_OP_RESUME:
                rsp_code = OBEX_RSP_SERVICE_UNAVL;
                /* do not report the API event */
                p_scb->api_evt   = OBEX_NULL_EVT;
                break;

            case OBEX_SESS_OP_SET_TIME:
                /*TODO respond SET_TIME right away */
                now = TRUE;
                /* TODO need to report the new timeout */
                if (p_scb->sess_st == OBEX_SESS_NONE)
                {
                    rsp_code = OBEX_RSP_FORBIDDEN;
                }
                break;
            }
        }
    }
    OBEX_TRACE_DEBUG6("obx_sa_session_ind tx_mtu: %d, sess_st:%d->%d, rsp_code:0x%x, now:%d pstate:%d",
        p_scb->ll_cb.comm.tx_mtu, old_sess_st, p_scb->sess_st, rsp_code, now, p_scb->prev_state);

    if (rsp_code == OBEX_RSP_OK)
    {
        obx_read_timeout (triplet, num, &timeout, &p_scb->sess_info[OBEX_SESSION_INFO_TO_IDX]);
    }

    if ((rsp_code != OBEX_RSP_OK) || now)
    {
        /* hold the original api_evt temporarily, so it's not reported at this obx_ssm_event */
        old_api_evt     = p_scb->api_evt;
        p_scb->api_evt  = OBEX_NULL_EVT;
        /* send the response now */
        obx_prepend_rsp_msg(p_scb->ll_cb.comm.handle, OBEX_SESSION_CFM_SEVT,
                            rsp_code, NULL);
        /* restore the api event */
        p_scb->api_evt  = old_api_evt;
    }
    else
    {
        ind_to = obx_read_triplet(triplet, num, OBEX_TAG_SESS_PARAM_SESS_OP);
        if ((ind_to != num) && (triplet[ind_to].len == OBEX_TIMEOUT_SIZE))
        {
            p = triplet[ind_to].p_array;
            BE_STREAM_TO_UINT32(timeout, p);
        }
        p_scb->param.sess.p_sess_info   = p_scb->sess_info;
        p_scb->param.sess.sess_st       = p_scb->sess_st;
        p_scb->param.sess.ssn           = p_scb->ssn;
        p_scb->param.sess.obj_offset    = obj_offset;
        p_scb->param.sess.timeout       = timeout;
        memcpy(p_scb->param.sess.peer_addr , p_scb->peer_addr, BD_ADDR_LEN);

    }
    return state;
}

/*******************************************************************************
** Function     obx_sa_sess_conn_ind
** Description  process session request from client
**              when the session request is received in OBEX_SS_NOT_CONNECTED
*******************************************************************************/
tOBEX_SR_STATE obx_sa_sess_conn_ind(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    UINT8           sess_info[OBEX_SESSION_INFO_SIZE]; /* session id + local nonce */
    tOBEX_TRIPLET    triplet[OBEX_MAX_SESS_PARAM_TRIP];
    UINT8           num = OBEX_MAX_SESS_PARAM_TRIP, ind;
    UINT8           *p;
    tOBEX_SR_CB      *p_cb = obx_sr_get_cb(p_scb->handle);
    UINT8           rsp_code = OBEX_RSP_FORBIDDEN;
    BOOLEAN         now = FALSE;
#if (BT_USE_TRACES == TRUE)
    tOBEX_SESS_ST    old_sess_st = p_scb->sess_st;
#endif
    tOBEX_SESS_OP    sess_op = OBEX_SESS_OP_CREATE;
    tOBEX_EVENT      old_api_evt;
    UINT32          obj_offset = 0;/* to report in evt param */
    UINT8           op_ssn;
    UINT8           num_trip = 0;
    BT_HDR          *p_rsp = NULL;
    UINT8           data[12];
    UINT8           ind_to;
    UINT32          timeout = OBEX_INFINITE_TIMEOUT;
    UINT32          offset = 0; /* if non-0, add to triplet on resume */
    tOBEX_SPND_CB    *p_spndcb;
    tOBEX_EVT_PARAM  param;              /* The event parameter. */

    OBEX_TRACE_DEBUG0("obx_sa_sess_conn_ind");
    OBEX_ReadTriplet(p_pkt, OBEX_HI_SESSION_PARAM, triplet, &num);
    if (p_cb->nonce == 0)
    {
        OBEX_TRACE_ERROR0("reliable session is not supported by this server");
        /* do not report the session_req_evt */
        p_scb->api_evt   = OBEX_NULL_EVT;
        obx_prepend_rsp_msg(p_scb->ll_cb.comm.handle, OBEX_SESSION_CFM_SEVT,
                            OBEX_RSP_NOT_IMPLEMENTED, NULL);
        return OBEX_SS_NULL;
    }
    else if (num)
    {
        ind = obx_read_triplet(triplet, num, OBEX_TAG_SESS_PARAM_SESS_OP);
        OBEX_TRACE_DEBUG2("sess_op ind:%d, num:%d", ind, num);
        if ((ind != num) && (triplet[ind].len == OBEX_LEN_SESS_PARAM_SESS_OP))
        {
            p = triplet[ind].p_array;
            p_scb->param.sess.sess_op = sess_op = *p;
            OBEX_TRACE_DEBUG1("sess_op :%d", *p);
            switch (*p)
            {
            case OBEX_SESS_OP_CREATE:
                /* do not report the API event */
                p_scb->api_evt   = OBEX_NULL_EVT;

                if (p_scb->sess_st == OBEX_SESS_ACTIVE)
                {
                    /* the session is already active; reject with OBEX_RSP_FORBIDDEN */
                    break;
                }

                /* check if we still have room for one more session */
                if (obx_find_suspended_session (p_scb, NULL, 0) == NULL)
                {
                    rsp_code = OBEX_RSP_DATABASE_FULL;
                    break;
                }

                p = &p_scb->sess_info[OBEX_SESSION_INFO_NONCE_IDX];
                UINT32_TO_BE_STREAM(p, p_cb->nonce);
                p_cb->nonce++;
                /* make sure it's not 0 (which means reliable session is disabled) */
                if (!p_cb->nonce)
                    p_cb->nonce++;

                if (obx_sa_make_sess_id (p_scb, p_scb->sess_info, triplet, num) == OBEX_SUCCESS)
                {
                    rsp_code = OBEX_RSP_OK;
                    p_scb->ssn = 0;
                    p_scb->sess_st = OBEX_SESS_CREATE;
                }
                break;

            case OBEX_SESS_OP_RESUME:
                /* verify that a previously interrupted session exists with the same session parameters.
                   Otherwise, OBEX_RSP_SERVICE_UNAVL.
                */
                p_spndcb = obx_find_suspended_session (p_scb, triplet, num);
                if (p_spndcb)
                {
                    op_ssn = p_spndcb->ssn;
                    memcpy (p_scb->sess_info, p_spndcb->sess_info, OBEX_SESSION_INFO_SIZE);
                    if (obx_sa_make_sess_id (p_scb, sess_info, triplet, num) == OBEX_SUCCESS &&
                        memcmp (sess_info, p_scb->sess_info, OBEX_SESSION_ID_SIZE) == 0)
                    {
                        /* clear the suspend cb info */
                        p_spndcb->state = OBEX_SS_NULL;
                        if (p_spndcb->stle.param)
                        {
                            btu_stop_timer (&p_spndcb->stle);
                            p_spndcb->stle.param = 0;
                        }
                        rsp_code        = OBEX_RSP_OK;
                        p_scb->sess_st  = OBEX_SESS_RESUME;
                        p_scb->srmp     |= OBEX_SRMP_SESS_FST;
                        ind = obx_read_triplet(triplet, num, OBEX_TAG_SESS_PARAM_NSEQNUM);
                        if (ind != num)
                        {
                            /* ssn exists - must be immediate suspend */
                            p = triplet[ind].p_array;
                            op_ssn = *p;
                            obj_offset  = obx_read_obj_offset(triplet, num);
                        }
                    }
                    obxu_dump_hex (p_scb->sess_info, "sess info", OBEX_SESSION_INFO_SIZE);
                    OBEX_TRACE_DEBUG4("p_spndcb->offset: 0x%x srm:x%x op_ssn %d ssn %d", p_spndcb->offset, p_scb->sess_info[OBEX_SESSION_INFO_SRM_IDX],op_ssn,p_scb->ssn);

                    if (p_scb->sess_info[OBEX_SESSION_INFO_SRM_IDX] & OBEX_SRM_ENGAGE)
                    {
                        /*
                        If offset in the request is smaller which only happens when it is a get operation, then client's offset and ssn matters.
                        So server's offset and ssn should be set to the client's ones.
                        If offset in the request is greater which only happens when it is a put operation, then server's offset and ssn matters.
                        So server keeps its offset(do nothing) and ssn and send it back to client in the resume reponse.
                        If offset are equal, either side's offset & ssn is fine, we choose to use the one in the reqeust
                        */

                        offset = p_spndcb->offset;
                        if (obj_offset && (obj_offset <= offset))
                        {
                            offset      = obj_offset;
                            p_scb->ssn  = op_ssn;
                            /* Adjust ssn in the next continue to be the same as nssn in the resume request */
                            if (p_scb->sess_info[OBEX_SESSION_INFO_ST_IDX] ==OBEX_CS_GET_SRM)
                                p_scb->ssn--;


                        }

                    }
                    /* SRM is not enabled */
                    else
                    {
                        p_scb->ssn  = op_ssn;
                    }

                    OBEX_TRACE_DEBUG4("offset: 0x%x ssn:%d obj_offset:0x%x srm:0x%x", offset, p_scb->ssn, obj_offset, p_scb->sess_info[OBEX_SESSION_INFO_SRM_IDX]);
                }


                if (rsp_code != OBEX_RSP_OK)
                {
                    rsp_code = OBEX_RSP_SERVICE_UNAVL;
                }
                /* do not report the API event for RESUME.
                 * if OBEX_RSP_OK, the event is reported in this function.
                 * p_rsp is used to call obx_ssm_event, and can not be reported to cback */
                p_scb->api_evt   = OBEX_NULL_EVT;
                break;
            }
        }
    }
    else
    {
        /* do not have session parameters - bad req do not report the API event */
        p_scb->api_evt   = OBEX_NULL_EVT;
        rsp_code = OBEX_RSP_BAD_REQUEST;
    }
    OBEX_TRACE_DEBUG5("obx_sa_sess_conn_ind tx_mtu: %d, sess_st:%d->%d, rsp_code:0x%x, now:%d",
        p_scb->ll_cb.comm.tx_mtu, old_sess_st, p_scb->sess_st, rsp_code, now);

    if (rsp_code == OBEX_RSP_OK)
    {
        /* send the session response now.
         * do not report OBEX_SESSION_REQ_EVT until connect indication,
         * so connection id can be reported at the same time in sess_info */
        if ( (p_rsp = (BT_HDR *)wiced_bt_obex_header_init(p_scb->handle, OBEX_MIN_MTU))== NULL)
        {
            rsp_code = OBEX_RSP_INTRNL_SRVR_ERR;
        }
        else
        {
            obx_read_timeout (triplet, num, &timeout, &p_scb->sess_info[OBEX_SESSION_INFO_TO_IDX]);

            p = (UINT8 *) (p_rsp + 1) + p_rsp->offset;
            /* response packet always has the final bit set */
            *p++ = (OBEX_RSP_OK | OBEX_FINAL);
            p_rsp->len = 3;
            p = data;

            /* add address */
                triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_ADDR;
                triplet[num_trip].len = BD_ADDR_LEN;
                triplet[num_trip].p_array = p;
                wiced_bt_dev_read_local_addr (p);
                p += BD_ADDR_LEN;
                num_trip++;

                /* add nonce 4 - 16 bytes */
                triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_NONCE;
                triplet[num_trip].len = OBEX_LOCAL_NONCE_SIZE;
            triplet[num_trip].p_array = &p_scb->sess_info[OBEX_SESSION_INFO_NONCE_IDX];
                num_trip++;

                /* add session id */
                triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_SESS_ID;
                triplet[num_trip].len = OBEX_SESSION_ID_SIZE;
                triplet[num_trip].p_array = p_scb->sess_info;
                num_trip++;

            if (sess_op == OBEX_SESS_OP_RESUME)
            {
                /* add session id */
                triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_NSEQNUM;
                triplet[num_trip].len = 1;
                triplet[num_trip].p_array = p;
                /* Adjust ssn in the resume response to be the same as nssn in the resume request */
                if (p_scb->sess_info[OBEX_SESSION_INFO_ST_IDX] == OBEX_CS_GET_SRM)
                    *p++ = p_scb->ssn+1;
                else
                    *p++ = p_scb->ssn;
                num_trip++;
                if (offset)
                {
                    triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_OBJ_OFF;
                    triplet[num_trip].len = OBEX_LEN_SESS_PARAM_OBJ_OFF;
                    triplet[num_trip].p_array = p;
                    UINT32_TO_BE_STREAM(p, offset);
                    num_trip++;
                    obj_offset = offset;
                }
            }

                /* add timeout */
            if (timeout != OBEX_INFINITE_TIMEOUT && obx_cb.sess_tout_val != OBEX_INFINITE_TIMEOUT && (obx_cb.sess_tout_val > timeout))
            {
                timeout = obx_cb.sess_tout_val;
                triplet[num_trip].p_array = p;
                num_trip += obx_add_timeout (&triplet[num_trip], obx_cb.sess_tout_val, &p_scb->param.sess);
                p = &p_scb->sess_info[OBEX_SESSION_INFO_TO_IDX];
                UINT32_TO_BE_STREAM(p, timeout);
            }
                OBEX_AddTriplet(p_rsp, OBEX_HI_SESSION_PARAM, triplet, num_trip);

                /* adjust the packet len */
                p = (UINT8 *) (p_rsp + 1) + p_rsp->offset + 1;
                UINT16_TO_BE_STREAM(p, p_rsp->len);
                p_rsp->event = OBEX_SESSION_CFM_SEVT + 1;
                p_scb->sess_st = OBEX_SESS_ACTIVE;
            p_scb->param.sess.p_sess_info   = p_scb->sess_info;
            p_scb->param.sess.sess_st       = p_scb->sess_st;
            p_scb->param.sess.ssn           = p_scb->ssn;
            p_scb->param.sess.nssn          = p_scb->ssn;
            p_scb->param.sess.obj_offset    = obj_offset;
            p_scb->param.sess.timeout       = timeout;
            memcpy(p_scb->param.sess.peer_addr , p_scb->peer_addr, BD_ADDR_LEN);
                obx_ssm_event(p_scb, OBEX_SESSION_CFM_SEVT, p_rsp);

            if (sess_op == OBEX_SESS_OP_RESUME)
            {
                (*p_cb->p_cback)(p_scb->ll_cb.comm.handle, OBEX_SESSION_REQ_EVT, p_scb->param, NULL);
                memset(&p_scb->param, 0, sizeof (p_scb->param) );
                param.conn.ssn      = p_scb->ssn;
                memcpy (param.conn.peer_addr, p_scb->peer_addr, BD_ADDR_LEN);
                p = &p_scb->sess_info[OBEX_SESSION_INFO_MTU_IDX];
                BE_STREAM_TO_UINT16(param.conn.mtu, p);
                p_scb->ll_cb.comm.tx_mtu    = param.conn.mtu;
                param.conn.handle = p_scb->ll_cb.comm.handle;
                OBEX_TRACE_DEBUG1("RESUME tx_mtu: %d", p_scb->ll_cb.comm.tx_mtu);
                /* report OBEX_CONNECT_REQ_EVT to let the client know the MTU */
                (*p_cb->p_cback)(p_scb->ll_cb.comm.handle, OBEX_CONNECT_REQ_EVT, param, NULL);
            }
        }
    }

    if ((rsp_code != OBEX_RSP_OK) || now)
    {
        /* hold the original api_evt temporarily, so it's not reported at this obx_ssm_event */
        old_api_evt     = p_scb->api_evt;
        p_scb->api_evt  = OBEX_NULL_EVT;
        /* send the response now */
        obx_prepend_rsp_msg(p_scb->ll_cb.comm.handle, OBEX_DISCNT_CFM_SEVT,
                            rsp_code, NULL);
        /* restore the api event */
        p_scb->api_evt  = old_api_evt;
    }
    else
    {
        ind_to = obx_read_triplet(triplet, num, OBEX_TAG_SESS_PARAM_SESS_OP);
        if ((ind_to != num) && (triplet[ind_to].len == OBEX_TIMEOUT_SIZE))
        {
            p = triplet[ind_to].p_array;
            BE_STREAM_TO_UINT32(timeout, p);
        }
        p_scb->param.sess.p_sess_info   = p_scb->sess_info;
        p_scb->param.sess.sess_st       = p_scb->sess_st;
        p_scb->param.sess.ssn           = p_scb->ssn;
        p_scb->param.sess.nssn          = p_scb->ssn;
        p_scb->param.sess.obj_offset    = obj_offset;
        p_scb->param.sess.timeout       = timeout;
        memcpy(p_scb->param.sess.peer_addr , p_scb->peer_addr, BD_ADDR_LEN);

    }
    return OBEX_SS_NULL;
}

/*******************************************************************************
** Function     obx_sa_wc_sess_ind
** Description  process session request from client
**              when the session request is received in OBEX_SS_WAIT_CLOSE
*******************************************************************************/
tOBEX_SR_STATE obx_sa_wc_sess_ind(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_EVT_PARAM  param;
    tOBEX_SR_CB      *p_cb = obx_sr_get_cb(p_scb->handle);

    OBEX_TRACE_DEBUG1("obx_sa_wc_sess_ind sess_st:%d", p_scb->sess_st);
    if (p_scb->sess_st == OBEX_SESS_ACTIVE)
    {
        /* processing CloseSession */
        obx_sa_session_ind(p_scb, p_pkt);
    }
    else
    {
        /* probably CreateSession */
        obx_sa_sess_conn_ind(p_scb, p_pkt);
        OBEX_TRACE_DEBUG1("obx_sa_wc_sess_ind (after obx_sa_sess_conn_ind) sess_st:%d", p_scb->sess_st);
        if (p_scb->sess_st == OBEX_SESS_ACTIVE)
        {
            /* after session command and still is Active
             * a session must have been created during Wait_close state.
             * need to report a OBEX_CLOSE_IND_EVT to clean up the profiles */
            memset(&param, 0, sizeof(tOBEX_EVT_PARAM));
            if (p_cb && p_cb->p_cback)
                (*p_cb->p_cback)(p_scb->ll_cb.comm.handle, OBEX_CLOSE_IND_EVT, param, NULL);
        }
    }
    return OBEX_SS_NULL;
}

/*******************************************************************************
** Function     obx_sa_session_rsp
** Description  process Session response API.
*******************************************************************************/
tOBEX_SR_STATE obx_sa_session_rsp(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE   new_state = OBEX_SS_NULL;
    UINT8           *p;

    OBEX_TRACE_DEBUG1("obx_sa_session_rsp pstate:%d", p_scb->prev_state);
    new_state = obx_sa_snd_rsp(p_scb, p_pkt);
    OBEX_TRACE_DEBUG2("sess_st: %d op:%d", p_scb->sess_st, p_scb->param.sess.sess_op);
    if (p_scb->sess_st == OBEX_SESS_ACTIVE && p_scb->param.sess.sess_op == OBEX_SESS_OP_RESUME)
    {
        p_scb->srm = p_scb->sess_info[OBEX_SESSION_INFO_SRM_IDX];
        new_state = p_scb->sess_info[OBEX_SESSION_INFO_ST_IDX];
        p = &p_scb->sess_info[OBEX_SESSION_INFO_ID_IDX];
        BE_STREAM_TO_UINT32(p_scb->conn_id, p);
        OBEX_TRACE_DEBUG3("new_state; %d Connection ID: 0x%x, srm:0x%x", new_state, p_scb->conn_id, p_scb->srm);
#ifdef OBEX_LIB_L2CAP_INCLUDED
        if ((p_scb->srm & OBEX_SRM_ENGAGE) && (new_state == OBEX_SS_GET_SRM))
        {
            tOBEX_L2C_CB     *p_lcb;
            tOBEX_L2C_EVT_PARAM  evt_param;

            p_lcb   = &p_scb->ll_cb.l2c;
            evt_param.any = 0;
            obx_l2c_snd_evt (p_lcb, evt_param, OBEX_L2C_EVT_RESUME);
        }
#endif
        /* report OBEX_CONNECT_REQ_EVT in obx_sa_sess_conn_ind()
        if (new_state == OBEX_SS_CONNECTED)
        {
        } */
    }
    else if (p_scb->sess_st == OBEX_SESS_CLOSE)
    {
        new_state = OBEX_SS_WAIT_CLOSE;
    }

    return new_state;
}

/*******************************************************************************
** Function     obx_sa_put_ind
** Description  received a PUT request from the client. Check if SRM is engaged.
*******************************************************************************/
tOBEX_SR_STATE obx_sa_put_ind(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE state = OBEX_SS_NULL;
    if ((p_scb->srm & OBEX_SRM_REQING) && (p_scb->srm & OBEX_SRM_ENABLE))
    {
        state   = OBEX_CS_PUT_SRM;
    }
    return state;
}

/*******************************************************************************
** Function     obx_sa_srm_put_req
** Description  received a PUT request from the client when SRM is engaged.
*******************************************************************************/
tOBEX_SR_STATE obx_sa_srm_put_req(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    if (!p_scb->param.put.final)
        p_scb->srm |= OBEX_SRM_WAIT_UL;
    return OBEX_SS_NULL;
}

/*******************************************************************************
** Function     obx_sa_srm_put_rsp
** Description  process PUT response API function.
**              report PUT request event, if any is queued
*******************************************************************************/
tOBEX_SR_STATE obx_sa_srm_put_rsp(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE   state = OBEX_SS_NULL;
    tOBEX_COMM_CB    *p_comm = &p_scb->ll_cb.comm;
    UINT8   rsp_code = OBEX_RSP_DEFAULT;
    BOOLEAN ret = TRUE;

    obx_access_rsp_code(p_pkt, &rsp_code);
    rsp_code &= ~OBEX_FINAL;
    if (rsp_code != OBEX_RSP_CONTINUE )
    {
        p_scb->srm |= OBEX_SRM_NEXT;
        if (rsp_code != OBEX_RSP_OK)
            p_scb->srm |= OBEX_SRM_ABORT;
    }

    p_scb->srm &= ~OBEX_SRM_WAIT_UL;
    OBEX_TRACE_DEBUG2("obx_sa_srm_put_rsp srm:0x%x rsp_code:0x%x", p_scb->srm, rsp_code);
    if (p_scb->srm & OBEX_SRM_NEXT)
    {
        p_scb->srm &= ~OBEX_SRM_NEXT;
        state = obx_sa_snd_rsp (p_scb, p_pkt);
    }
    else
    {
        if (p_scb->sess_st == OBEX_SESS_ACTIVE)
        {
            p_scb->ssn++;
        }
        obx_start_timer(&p_scb->ll_cb.comm);
        GKI_freebuf(p_pkt);
    }
    OBEX_TRACE_DEBUG1("obx_sa_srm_put_rsp srm:0x%x", p_scb->srm);

    while (ret && (p_pkt = (BT_HDR *)GKI_dequeue (&p_comm->rx_q)) != NULL)
    {
        if (state != OBEX_SS_NULL)
        {
            p_scb->state = state;
            state = OBEX_SS_NULL;
        }
        ret = obx_sr_proc_pkt (p_scb, p_pkt);
        if ((p_scb->srm & OBEX_SRM_ABORT) == 0)
            ret = FALSE;
        obx_flow_control(p_comm);
        OBEX_TRACE_DEBUG3("obx_sa_srm_put_rsp rx_q.count: %d srm:0x%x, ret:%d", p_comm->rx_q.count, p_scb->srm, ret );
    }

    return state;
}

/*******************************************************************************
** Function     obx_sa_srm_get_fcs
** Description  Process L2CAP congestion event
*******************************************************************************/
tOBEX_SR_STATE obx_sa_srm_get_fcs(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
#ifdef OBEX_LIB_L2CAP_INCLUDED
    BOOLEAN         not_cong = TRUE;
    if (p_scb->ll_cb.comm.p_send_fn == (tOBEX_SEND_FN *)obx_l2c_snd_msg)
    {
        OBEX_TRACE_DEBUG1("obx_sa_srm_get_fcs cong:%d", p_scb->ll_cb.l2c.cong);
        if (p_scb->ll_cb.l2c.cong)
            not_cong = FALSE;
    }
    if (not_cong)
        p_scb->api_evt = OBEX_GET_REQ_EVT;
#endif
    return OBEX_SS_NULL;
}

/*******************************************************************************
** Function     obx_sa_srm_get_rsp
** Description  send GET response to client
*******************************************************************************/
tOBEX_SR_STATE obx_sa_srm_get_rsp(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE   state;
    OBEX_TRACE_DEBUG1("obx_sa_srm_get_rsp srm:0x%x", p_scb->srm);
    state = obx_sa_snd_rsp(p_scb, p_pkt);
    return state;
}



/*******************************************************************************
** Function     obx_sa_srm_get_req
** Description  process GET request from client
*******************************************************************************/
tOBEX_SR_STATE obx_sa_srm_get_req(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE   state = OBEX_SS_NULL;
    tOBEX_SR_CB      *p_cb;

    OBEX_TRACE_DEBUG3("obx_sa_srm_get_req srm:0x%x srmp:0x%x final:%d", p_scb->srm, p_scb->srmp, p_scb->param.get.final);
    if (p_scb->srmp & OBEX_SRMP_NONF)
    {
        /* the GET request does not set the final bit yet.
         * merge this request, send response automatically and do not report the event */
        if (!p_scb->param.get.final)
        {
            p_scb->p_saved_msg  = obx_merge_get_req(p_scb->p_saved_msg, p_pkt);
            p_scb->api_evt      = OBEX_NULL_EVT;
        }
        else
        {
            p_scb->srmp        &= ~OBEX_SRMP_NONF;
            p_pkt               = obx_merge_get_req(p_scb->p_saved_msg, p_pkt);
#if BT_TRACE_PROTOCOL == TRUE
            DispObxMsg(p_pkt, (BOOLEAN)(OBEX_GET_REQ_DISP | OBEX_DISP_IS_RECV));
#endif
            p_scb->p_saved_msg  = NULL;
            p_scb->api_evt      = OBEX_NULL_EVT;
            p_cb = &obx_cb.server[p_scb->handle - 1];
            (p_cb->p_cback) (p_scb->ll_cb.comm.handle, OBEX_GET_REQ_EVT, p_scb->param, (UINT8 *)p_pkt);
            memset(&p_scb->param, 0, sizeof (p_scb->param) );
        }
    }

    return state;
}
