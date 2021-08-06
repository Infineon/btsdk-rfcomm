/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
**  Name:         obx_cact.c
**
**  File:         OBEX Client State Machine Action Functions
**
**
*****************************************************************************/
#include <string.h>
#include "obx_int.h"

const tOBEX_EVENT obx_cl_state_2_event_map[] =
{
    OBEX_DISCONNECT_RSP_EVT, /* OBEX_CS_NOT_CONNECTED */
    OBEX_SESSION_RSP_EVT,    /* OBEX_CS_SESSION_REQ_SENT */
    OBEX_CONNECT_RSP_EVT,    /* OBEX_CS_CONNECT_REQ_SENT */
    OBEX_ABORT_RSP_EVT,      /* OBEX_CS_CONNECTED */
    OBEX_DISCONNECT_RSP_EVT, /* OBEX_CS_DISCNT_REQ_SENT */
    OBEX_SETPATH_RSP_EVT,    /* OBEX_CS_SETPATH_REQ_SENT */
    OBEX_ACTION_RSP_EVT,     /* OBEX_CS_ACTION_REQ_SENT */
    OBEX_ABORT_RSP_EVT,      /* OBEX_CS_ABORT_REQ_SENT */
    OBEX_PUT_RSP_EVT,        /* OBEX_CS_PUT_REQ_SENT */
    OBEX_GET_RSP_EVT,        /* OBEX_CS_GET_REQ_SENT */
    OBEX_PUT_RSP_EVT,        /* OBEX_CS_PUT_TRANSACTION */
    OBEX_GET_RSP_EVT,        /* OBEX_CS_GET_TRANSACTION */
    OBEX_PUT_RSP_EVT,        /* OBEX_CS_PUT_SRM */
    OBEX_GET_RSP_EVT         /* OBEX_CS_GET_SRM */
};

/*******************************************************************************
** Function     obx_ca_close_sess_req
** Description  send close session request
*******************************************************************************/
BT_HDR * obx_ca_close_sess_req(tOBEX_CL_CB *p_cb)
{
    BT_HDR      *p_req;
    UINT8       *p;
    UINT8       num_trip = 0;
    tOBEX_TRIPLET triplet[4];
    UINT8       data[2];

    p_req   = (BT_HDR *)wiced_bt_obex_header_init(OBEX_HANDLE_NULL, OBEX_MIN_MTU);
    p = (UINT8 *) (p_req + 1) + p_req->offset;
    /* Session request packet always has the final bit set */
    *p++ = (OBEX_REQ_SESSION | OBEX_FINAL);
    p_req->len = 3;
    p = data;

    /* add session opcode */
    triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_SESS_OP;
    triplet[num_trip].len = OBEX_LEN_SESS_PARAM_SESS_OP;
    triplet[num_trip].p_array = p;
    *p = OBEX_SESS_OP_CLOSE;
    p += OBEX_LEN_SESS_PARAM_SESS_OP;
    num_trip++;

    /* add session id */
    triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_SESS_ID;
    triplet[num_trip].len = OBEX_SESSION_ID_SIZE;
    triplet[num_trip].p_array = p_cb->sess_info;
    num_trip++;

    OBEX_AddTriplet(p_req, OBEX_HI_SESSION_PARAM, triplet, num_trip);
    /* adjust the packet len */
    p = (UINT8 *) (p_req + 1) + p_req->offset + 1;
    UINT16_TO_BE_STREAM(p, p_req->len);
    p_req->event    = OBEX_SESSION_REQ_EVT;
    p_cb->sess_st   = OBEX_SESS_CLOSE;
    OBEX_TRACE_DEBUG2("obx_ca_close_sess_req shandle:0x%x, sess_st:%d\n", p_cb->ll_cb.comm.handle, p_cb->sess_st);
    return p_req;
}

/*******************************************************************************
** Function     obx_ca_connect_req
** Description  send connect request
*******************************************************************************/
void obx_ca_connect_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    UINT8       msg[OBEX_HDR_OFFSET + OBEX_MAX_CONN_HDR_EXTRA];
    UINT8       *p = msg;

    /* Connect request packet always has the final bit set */
    *p++ = (OBEX_REQ_CONNECT | OBEX_FINAL);
    p += OBEX_PKT_LEN_SIZE;

    *p++ = OBEX_VERSION;
    *p++ = OBEX_CONN_FLAGS;
    UINT16_TO_BE_STREAM(p, p_cb->ll_cb.port.rx_mtu);

    /* add session sequence number, if session is active */
    if (p_cb->sess_st == OBEX_SESS_ACTIVE)
    {
        *p++ = OBEX_HI_SESSION_SN;
        *p++ = p_cb->ssn;
    }
    if (p_cb->srm)
    {
        p_cb->srm   = OBEX_SRM_ENABLE;
    }

    /* IrOBEX spec forbids connection ID in Connect Request */
    p_pkt = obx_cl_prepend_msg(p_cb, p_pkt, msg, (UINT16)(p - msg) );

    p_pkt->event    = OBEX_CONNECT_REQ_EVT;
    obx_csm_event(p_cb, OBEX_CONNECT_REQ_CEVT, p_pkt);
}

void obx_ca_connect_req_with_auth_res(wiced_bt_obex_handle_t handle, BT_HDR *p_pkt)
{
    UINT8       msg[OBEX_HDR_OFFSET + OBEX_MAX_CONN_HDR_EXTRA];
    UINT8       *p = msg;
    tOBEX_CL_CB *p_cb = obx_cl_get_cb(handle);


    OBEX_TRACE_DEBUG4("obx_ca_connect_req_with_auth_res");

    if (p_cb == NULL)
    {
        OBEX_TRACE_DEBUG4("obx_ca_connect_req_with_auth_res p_cb is null");
        return;
    }
    else
    {
        OBEX_TRACE_DEBUG4("obx_ca_connect_req_with_auth_res p_cb not null");
    }
    /* Connect request packet always has the final bit set */
    *p++ = (OBEX_REQ_CONNECT | OBEX_FINAL);
    p += OBEX_PKT_LEN_SIZE;

    *p++ = OBEX_VERSION;
    *p++ = OBEX_CONN_FLAGS;
    UINT16_TO_BE_STREAM(p, p_cb->ll_cb.port.rx_mtu);

    /* add session sequence number, if session is active */
    if (p_cb->sess_st == OBEX_SESS_ACTIVE)
    {
        *p++ = OBEX_HI_SESSION_SN;
        *p++ = p_cb->ssn;
    }
    if (p_cb->srm)
    {
        p_cb->srm   = OBEX_SRM_ENABLE;
    }

    /* IrOBEX spec forbids connection ID in Connect Request */
    p_pkt = obx_cl_prepend_msg(p_cb, p_pkt, msg, (UINT16)(p - msg) );

    p_pkt->event    = OBEX_CONNECT_REQ_EVT;
    //obx_csm_event(p_cb, OBEX_CONNECT_REQ_CEVT, p_pkt);
    obx_ca_snd_req(p_cb, p_pkt);
}

/*******************************************************************************
** Function     obx_ca_state
** Description  change state
*******************************************************************************/
tOBEX_CL_STATE obx_ca_state(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    /* p_pkt should be NULL here */
    return p_cb->next_state;
}

/*******************************************************************************
** Function     obx_ca_start_timer
** Description  start timer
*******************************************************************************/
tOBEX_CL_STATE obx_ca_start_timer(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    if (p_pkt)
        GKI_freebuf(p_pkt);
    obx_start_timer(&p_cb->ll_cb.comm);
    return OBEX_CS_NULL;
}

/*******************************************************************************
** Function     obx_ca_connect_ok
** Description  process the connect OK response from server
*******************************************************************************/
tOBEX_CL_STATE obx_ca_connect_ok(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    UINT8   *p;
    tOBEX_EVT_PARAM  param;              /* The event parameter. */

    if (p_cb->sess_st == OBEX_SESS_ACTIVE)
    {
        /* reliable session was established - need to report the session event first */
        p = &p_cb->sess_info[OBEX_SESSION_INFO_ID_IDX];
        UINT32_TO_BE_STREAM(p, p_cb->conn_id);
        param.sess.p_sess_info= p_cb->sess_info;
        param.sess.sess_st    = p_cb->sess_st;
        param.sess.nssn       = p_cb->ssn;
        param.sess.obj_offset = 0;
        p = &p_cb->sess_info[OBEX_SESSION_INFO_MTU_IDX];
        UINT16_TO_BE_STREAM(p, p_cb->param.conn.mtu);
        param.sess.sess_op    = OBEX_SESS_OP_CREATE;
        (*p_cb->p_cback)(p_cb->ll_cb.comm.handle, OBEX_SESSION_RSP_EVT, p_cb->rsp_code, param, NULL);
    }
    return obx_ca_notify(p_cb, p_pkt);
}

/*******************************************************************************
** Function     obx_ca_session_ok
** Description  process the session OK response from server
*******************************************************************************/
tOBEX_CL_STATE obx_ca_session_ok(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_TRIPLET    triplet[OBEX_MAX_SESS_PARAM_TRIP];
    UINT8           num = OBEX_MAX_SESS_PARAM_TRIP;
    UINT8           *p_nonce = NULL, *p_addr = NULL, *p_sess_id = NULL;
    UINT8           *p, *p_cl_nonce;
    UINT8           ind, nonce_len = 0;
    UINT8           ind_sess_id;
    BD_ADDR         cl_addr;
    tOBEX_STATUS     status = OBEX_SUCCESS;
    UINT8           nssn = 0;
#if (BT_USE_TRACES == TRUE)
    tOBEX_SESS_ST    old_sess_st = p_cb->sess_st;
#endif
    tOBEX_SESS_OP    sess_op = OBEX_SESS_OP_SET_TIME;
    tOBEX_CL_STATE   new_state = OBEX_CS_NULL;
    tOBEX_CL_EVENT   sm_evt = OBEX_BAD_SM_EVT;
    UINT32          obj_offset = p_cb->param.sess.obj_offset;
    UINT32          timeout = p_cb->param.sess.timeout;
    tOBEX_EVT_PARAM  param;              /* The event parameter. */
    UINT8           dropped = 0;

    OBEX_TRACE_DEBUG4("obx_ca_session_ok sess_st: %d ssn:%d obj_offset:%d prev_state:%d\n", p_cb->sess_st, p_cb->ssn, obj_offset, p_cb->prev_state);
    OBEX_ReadTriplet(p_pkt, OBEX_HI_SESSION_PARAM, triplet, &num);
    obx_read_timeout (triplet, num, &timeout, &p_cb->sess_info[OBEX_SESSION_INFO_TO_IDX]);
    p_cb->param.sess.timeout = timeout;
    if (p_cb->sess_st == OBEX_SESS_SUSPEND)
    {
        p_cb->sess_st   = OBEX_SESS_SUSPENDED;
        sess_op         = OBEX_SESS_OP_SUSPEND;
        p_cb->sess_info[OBEX_SESSION_INFO_SRM_IDX] = p_cb->srm;
        nssn            = p_cb->ssn;
        /* send a tx_empty event to close port */
        sm_evt  = OBEX_TX_EMPTY_CEVT;
        OBEX_TRACE_DEBUG2("suspend saved st:%d, srm:0x%x\n", p_cb->sess_info[OBEX_SESSION_INFO_ST_IDX], p_cb->sess_info[OBEX_SESSION_INFO_SRM_IDX]);
    }
    else if (p_cb->sess_st == OBEX_SESS_CLOSE)
    {
        sess_op         = OBEX_SESS_OP_CLOSE;
        p_cb->sess_st   = OBEX_SESS_NONE;
        /* send a tx_empty event to close port */
        sm_evt  = OBEX_TX_EMPTY_CEVT;
    }
    else if (num)
    {
        ind_sess_id = obx_read_triplet(triplet, num, OBEX_TAG_SESS_PARAM_SESS_ID);
        if ((ind_sess_id != num) && (triplet[ind_sess_id].len == OBEX_SESSION_ID_SIZE))
        {
            p_sess_id = triplet[ind_sess_id].p_array;
        }
        switch (p_cb->sess_st)
        {
        case OBEX_SESS_CREATE:
            sess_op = OBEX_SESS_OP_CREATE;
            status = OBEX_BAD_PARAMS;
            ind = obx_read_triplet(triplet, num, OBEX_TAG_SESS_PARAM_ADDR);
            if ((ind != num) && (triplet[ind].len == BD_ADDR_LEN))
            {
                p_addr = triplet[ind].p_array;
            }
            ind = obx_read_triplet(triplet, num, OBEX_TAG_SESS_PARAM_NONCE);
            if ((ind != num) && (triplet[ind].len >= OBEX_MIN_NONCE_SIZE) && (triplet[ind].len <= OBEX_NONCE_SIZE))
            {
                p_nonce = triplet[ind].p_array;
                nonce_len = triplet[ind].len;
            }

            if (p_nonce && p_addr && p_sess_id)
            {
                OBEX_TRACE_DEBUG0("verify session id\n");
                wiced_bt_dev_read_local_addr (cl_addr);
                p_cl_nonce = &p_cb->sess_info[OBEX_SESSION_ID_SIZE];
                p = p_cl_nonce;
                UINT32_TO_BE_STREAM(p, p_cb->nonce);
                /* calculate client copy of session id */
                obx_session_id (p_cb->sess_info, cl_addr, p_cl_nonce, OBEX_LOCAL_NONCE_SIZE, p_addr, p_nonce, nonce_len);
                obxu_dump_hex (p_cb->sess_info, "cl sess id", OBEX_SESSION_ID_SIZE);
                obxu_dump_hex (p_sess_id, "sr sess id", OBEX_SESSION_ID_SIZE);
                /* verify that the server copy is the same */
                if (memcmp (p_sess_id, p_cb->sess_info, OBEX_SESSION_ID_SIZE) == 0)
                {
                    p_cb->sess_st   = OBEX_SESS_ACTIVE;
                    p_cb->ssn       = 0;
                    /* do we want a timer here */
                    status = OBEX_SUCCESS;
                    OBEX_TRACE_DEBUG0("freeing received packet\n");
                    GKI_freebuf (p_pkt) ;
                    p_pkt  = p_cb->p_next_req;
                    p_cb->p_next_req = NULL;
                    obx_ca_connect_req (p_cb, p_pkt);
                    /*
                    p_cb->param.sess.p_sess_info= p_cb->sess_info;
                    p_cb->param.sess.sess_st    = p_cb->sess_st;
                    p_cb->param.sess.nssn       = p_cb->ssn;
                    p_cb->param.sess.sess_op    = sess_op;
                    (*p_cb->p_cback)(p_cb->ll_cb.comm.handle, OBEX_SESSION_RSP_EVT, p_cb->rsp_code, p_cb->param, NULL);
                    */
                    return new_state;
                }
            }
            break;

        case OBEX_SESS_RESUME:
            status = OBEX_BAD_PARAMS;
            dropped = p_cb->sess_info[OBEX_SESSION_INFO_ST_IDX] & OBEX_CL_STATE_DROP;
            ind = obx_read_triplet(triplet, num, OBEX_TAG_SESS_PARAM_NSEQNUM);
            if ((ind == num) || (triplet[ind].len != 1))
            {
                OBEX_TRACE_ERROR0("RESUME:do not have valid NSSN tag\n");
                break;
            }

            nssn = *(triplet[ind].p_array);
            /* if SRM is enaged; make sure object offset TAG exists */
            if ((p_cb->sess_info[OBEX_SESSION_INFO_SRM_IDX] & OBEX_SRM_ENGAGE) != 0)
            {
                obj_offset = obx_read_obj_offset(triplet, num);
                OBEX_TRACE_DEBUG2("RESUME:SRM is engaged and object offset:0x%x (0x%x)\n",
                    obj_offset, p_cb->param.sess.obj_offset);

                /* client always takes the offset and ssn from the response since adjustment was done at server side */
                p_cb->param.sess.obj_offset = obj_offset;
                p_cb->ssn = nssn;
                status = OBEX_SUCCESS;
            }
            /* otherwise make sure NSSN from server is OK */
            else if (nssn == p_cb->ssn)
            {
                OBEX_TRACE_DEBUG0("RESUME:nssn matches expected ssn\n");
                status = OBEX_SUCCESS;
            }
            else if (dropped != 0)
            {
                OBEX_TRACE_DEBUG2("RESUME:link drop suspend nssn:%d cb ssn:%d\n", nssn, p_cb->ssn);
                if ((UINT8)(nssn+1) == p_cb->ssn)
                {
                    OBEX_TRACE_DEBUG0("RESUME:nssn matches expected(ssn-1)\n");
                    p_cb->ssn -= 1;
                    status = OBEX_SUCCESS;
                }
                else if (nssn == (UINT8)(p_cb->ssn+1))
                {
                    OBEX_TRACE_DEBUG0("RESUME:nssn matches expected(ssn+1)\n");
                    nssn -= 1;
                    status = OBEX_SUCCESS;
                }
            }
            else
            {
                OBEX_TRACE_ERROR2("RESUME:bad NSSN:%d (%d)\n", nssn, p_cb->ssn);
                break;
            }
            p_cb->sess_st   = OBEX_SESS_ACTIVE;
            sess_op         = OBEX_SESS_OP_RESUME;
            OBEX_TRACE_DEBUG2("RESUME:info new_state:0x%x, srm:0x%x\n", p_cb->sess_info[OBEX_SESSION_INFO_ST_IDX], p_cb->sess_info[OBEX_SESSION_INFO_SRM_IDX]);
            p_cb->srm = p_cb->sess_info[OBEX_SESSION_INFO_SRM_IDX];
            p_cb->sess_info[OBEX_SESSION_INFO_ST_IDX] &= ~OBEX_CL_STATE_DROP;
            if (p_cb->srm & OBEX_SRM_ENGAGE)
            {
                new_state = p_cb->sess_info[OBEX_SESSION_INFO_ST_IDX];
                if (new_state == OBEX_CS_GET_SRM)
                {
                    p_cb->srm |= OBEX_SRM_WAIT_UL;
                    /* Adjust snn in the control block since it is off by one with nssn in resume request */
                    p_cb->ssn--;
                }
            }
            else
            {
                new_state    = OBEX_CS_CONNECTED;
                p_cb->srmp  |= OBEX_SRMP_SESS_FST;
            }
            OBEX_TRACE_DEBUG2("RESUME:new_state:%d, srm:0x%x\n", new_state, p_cb->srm);
            break;

        default:
            status = OBEX_BAD_PARAMS;
        }
    }
    else
    {
        status = OBEX_BAD_PARAMS;
    }
    OBEX_TRACE_DEBUG5("obx_ca_session_ok prev:%d, sess_st:%d->%d obj_offset:%d status:%d\n", p_cb->prev_state, old_sess_st, p_cb->sess_st, obj_offset, status);

    if (sess_op == OBEX_SESS_OP_SET_TIME)
        new_state = p_cb->prev_state;

    if (status != OBEX_SUCCESS)
    {
        if (p_cb->sess_st == OBEX_SESS_CLOSE)
            p_cb->sess_st = OBEX_SESS_NONE;
        obx_csm_event(p_cb, OBEX_TIMEOUT_CEVT, NULL);
        return OBEX_CS_NULL;
    }
    p_cb->param.sess.p_sess_info= p_cb->sess_info;
    p_cb->param.sess.sess_st    = p_cb->sess_st;
    p_cb->param.sess.nssn       = nssn;
    p_cb->param.sess.ssn        = nssn;
    p_cb->param.sess.sess_op    = sess_op;
    p_cb->param.sess.obj_offset = obj_offset;
    p_cb->param.sess.timeout    = timeout;
    (*p_cb->p_cback)(p_cb->ll_cb.comm.handle, OBEX_SESSION_RSP_EVT, p_cb->rsp_code, p_cb->param, NULL);

    if ((sess_op == OBEX_SESS_OP_RESUME) && (p_cb->sess_st == OBEX_SESS_ACTIVE))
    {
        param.conn.ssn      = p_cb->ssn;
        memcpy (param.conn.peer_addr, p_cb->peer_addr, BD_ADDR_LEN);
        p = &p_cb->sess_info[OBEX_SESSION_INFO_MTU_IDX];
        BE_STREAM_TO_UINT16(param.conn.mtu, p);
        p_cb->ll_cb.comm.tx_mtu = param.conn.mtu;
        param.conn.handle = p_cb->ll_cb.comm.handle;
        OBEX_TRACE_DEBUG1("RESUME: tx_mtu: %d\n", p_cb->ll_cb.comm.tx_mtu);
        /* report OBEX_CONNECT_RSP_EVT to let the client know the MTU */
        (*p_cb->p_cback)(p_cb->ll_cb.comm.handle, OBEX_CONNECT_RSP_EVT, OBEX_RSP_OK, param, NULL);
        sm_evt = OBEX_STATE_CEVT;
        p_cb->next_state = OBEX_CS_CONNECTED;
    }

    if (sm_evt != OBEX_BAD_SM_EVT)
    {
        /* send an event to csm */
        obx_csm_event(p_cb, sm_evt, NULL);
    }
    return new_state;
}

/*******************************************************************************
** Function     obx_ca_session_cont
** Description  process the continue response from server for SRM after
**              a suspend session request is sent
*******************************************************************************/
tOBEX_CL_STATE obx_ca_session_cont(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    BOOLEAN free = TRUE;

    OBEX_TRACE_DEBUG3("obx_ca_session_cont sess_st:%d prev_state:%d, srm:0x%x\n", p_cb->sess_st, p_cb->prev_state, p_cb->srm);
    if (p_cb->sess_st == OBEX_SESS_SUSPEND)
    {
        if (p_cb->prev_state == OBEX_CS_GET_SRM)
        {
            if ((p_cb->srm & OBEX_SRM_WAIT_UL) == 0)
            {
                p_cb->srm |= OBEX_SRM_WAIT_UL;
                p_cb->api_evt = OBEX_GET_RSP_EVT;
            }
            else
            {
                GKI_enqueue_head  (&p_cb->ll_cb.comm.rx_q, p_pkt);
                OBEX_TRACE_DEBUG1("obx_ca_session_cont rx_q.count:%d\n", p_cb->ll_cb.comm.rx_q.count);
            }
            free = FALSE;
        }
        else if (p_cb->prev_state == OBEX_CS_GET_REQ_SENT)
        {
            p_cb->api_evt = OBEX_GET_RSP_EVT;
            free = FALSE;
        }

    }
    if (free && p_pkt)
        GKI_freebuf(p_pkt);
    OBEX_TRACE_DEBUG1("obx_ca_session_cont srm: 0x%x(e)\n", p_cb->srm );
    return OBEX_CS_NULL;
}

/*******************************************************************************
** Function     obx_ca_session_get
** Description  process the get req api for SRM after
**              a suspend session request is sent (to clean out the received buffers)
*******************************************************************************/
tOBEX_CL_STATE obx_ca_session_get(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    OBEX_TRACE_DEBUG3("obx_ca_session_get sess_st:%d prev_state: %d, srm:0x%x\n", p_cb->sess_st, p_cb->prev_state, p_cb->srm );
    if (p_cb->sess_st == OBEX_SESS_SUSPEND && p_cb->prev_state == OBEX_CS_GET_SRM)
        return obx_ca_srm_get_req(p_cb, p_pkt);
    return OBEX_CS_NULL;
}

/*******************************************************************************
** Function     obx_ca_session_fail
** Description  process the session failed response from server
*******************************************************************************/
tOBEX_CL_STATE obx_ca_session_fail(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_SESS_ST    old_sess_st = p_cb->sess_st;

    p_cb->sess_st = OBEX_SESS_NONE;
    OBEX_TRACE_DEBUG2("obx_ca_session_fail, sess_st:%d->%d\n", old_sess_st, p_cb->sess_st);
    if (old_sess_st == OBEX_SESS_CREATE && p_cb->rsp_code != OBEX_RSP_OK)
    {
        /* peer device does not support session. Continue with regular session */
        /* report session failure */
        (*p_cb->p_cback)(p_cb->ll_cb.comm.handle, OBEX_SESSION_RSP_EVT, p_cb->rsp_code, p_cb->param, NULL);
        OBEX_TRACE_DEBUG0("freeing received packet\n");
        if (p_pkt)
            GKI_freebuf (p_pkt) ;
        p_pkt  = p_cb->p_next_req;
        p_cb->p_next_req = NULL;
        obx_ca_connect_req (p_cb, p_pkt);
    }
    else
    {
        (*p_cb->p_cback)(p_cb->ll_cb.comm.handle, OBEX_SESSION_RSP_EVT, OBEX_RSP_FAILED, p_cb->param, NULL);
        obx_ca_close_port (p_cb, p_pkt);
    }
    return OBEX_CS_NULL;
}

/*******************************************************************************
** Function     obx_ca_abort
** Description  process the abort request in connected state
*******************************************************************************/
tOBEX_CL_STATE obx_ca_abort(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    OBEX_TRACE_DEBUG2("obx_ca_abort srm:0x%x srmp:0x%x\n", p_cb->srm, p_cb->srmp);

    if ( p_cb->srmp & OBEX_SRMP_SESS_FST)
    {
        /* the first request after a session is resume.
         * We may need to abort the previous request */
        if (p_cb->sess_info[OBEX_SESSION_INFO_ST_IDX] != OBEX_CS_CONNECTED)
        {
            /* set the state here, just in case the result of obx_ca_snd_req is partial_sent */
            p_cb->state = OBEX_CS_ABORT_REQ_SENT;
            obx_ca_snd_req(p_cb, p_pkt);
            return OBEX_CS_NULL;
        }
        p_cb->srmp &= ~OBEX_SRMP_SESS_FST;
    }
    obx_ca_notify (p_cb, p_pkt);
    return OBEX_CS_NULL;
}

/*******************************************************************************
** Function     obx_ca_snd_put_req
** Description  send put request in connected state
*******************************************************************************/
tOBEX_CL_STATE obx_ca_snd_put_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE state = OBEX_CS_NULL;

    OBEX_TRACE_DEBUG1("obx_ca_snd_put_req, srm:0x%x\n", p_cb->srm);
    state = obx_ca_snd_req (p_cb, p_pkt);
    if (p_cb->srm & OBEX_SRM_ENGAGE)
    {
        if (state == OBEX_CS_PARTIAL_SENT)
        {
            p_cb->next_state   = OBEX_CS_PUT_SRM;
        }
        else
        {
            p_cb->state = OBEX_CS_PUT_SRM;
            if ((p_cb->srm & OBEX_SRM_WAIT) == 0)
            {
                p_cb->rsp_code = OBEX_RSP_CONTINUE;
                p_cb->param.put.final = FALSE;
                p_cb->param.put.type = OBEX_PUT_TYPE_PUT;
                p_cb->param.put.ssn = 0;
                state = obx_ca_notify (p_cb, NULL);
            }
        }
    }
    return state;
}

/*******************************************************************************
** Function     obx_ca_snd_get_req
** Description  send get request in connected state
*******************************************************************************/
tOBEX_CL_STATE obx_ca_snd_get_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE state;

    OBEX_TRACE_DEBUG1("obx_ca_snd_get_req srm:0x%x\n", p_cb->srm );
    state = obx_ca_snd_req (p_cb, p_pkt);
    OBEX_TRACE_DEBUG1("srm:0x%x\n", p_cb->srm );
    if (p_cb->srm & OBEX_SRM_ENABLE)
    {
        if (state == OBEX_CS_PARTIAL_SENT)
        {
            p_cb->next_state   = OBEX_CS_GET_SRM;
        }
        else
            state = OBEX_CS_GET_SRM;
        p_cb->srm &= ~OBEX_SRM_WAIT_UL;
    }
    OBEX_TRACE_DEBUG1("srm:0x%x\n", p_cb->srm );
    return state;
}

/*******************************************************************************
** Function     obx_ca_srm_snd_req
** Description  send Abort or Disconnect request
*******************************************************************************/
tOBEX_CL_STATE obx_ca_srm_snd_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE   state;
    tOBEX_COMM_CB    *p_comm = &p_cb->ll_cb.comm;

    OBEX_TRACE_DEBUG2("obx_ca_srm_snd_req rx_q.count: %d, srm:0x%x\n", p_comm->rx_q.count, p_cb->srm );
    p_cb->srm &= ~OBEX_SRM_WAIT_UL;
    state = obx_ca_snd_req (p_cb, p_pkt);
    if ((p_pkt = (BT_HDR *)GKI_dequeue (&p_comm->rx_q)) != NULL)
    {
        GKI_freebuf(p_pkt);
    }
    OBEX_TRACE_DEBUG2("                   rx_q.count: %d, srm:0x%x\n", p_comm->rx_q.count, p_cb->srm );
    return state;
}

/*******************************************************************************
** Function     obx_ca_srm_put_req
** Description  send a PUT request when SRM is engaged
*******************************************************************************/
tOBEX_CL_STATE obx_ca_srm_put_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE   state;

    OBEX_TRACE_DEBUG1("obx_ca_srm_put_req srm:0x%x\n", p_cb->srm );
    state = obx_ca_snd_req (p_cb, p_pkt);
    OBEX_TRACE_DEBUG4("obx_ca_srm_put_req state:%d srm:0x%x, final:%d rsp_code:0x%x\n", state, p_cb->srm, p_cb->final, p_cb->rsp_code );
    if (state != OBEX_CS_PARTIAL_SENT && p_cb->final != TRUE && (p_cb->srm & OBEX_SRM_WAIT) == 0)
    {
        p_cb->rsp_code = OBEX_RSP_CONTINUE;
        state = obx_ca_notify (p_cb, NULL);
    }
    return state;
}

/*******************************************************************************
**
** Function     obx_resend_get_req_msg
**
** Description  This function is called to resend Get request when SRMP used by client
** Returns      void
**
*******************************************************************************/
static void obx_resend_get_req_msg(tOBEX_HANDLE handle)
{
    tOBEX_CL_CB *p_cb = obx_cl_get_cb(handle);
    UINT8       msg[OBEX_HDR_OFFSET];
    UINT8       *p = msg;
    BT_HDR      *p_pkt;

    OBEX_TRACE_DEBUG1("obx_resend_get_req_msg (hdl %x)\n", handle);

    if (p_cb)
    {
        /* Our client Get ALWAYS sends request in single packet (FINAL) */
        *p++ = (UINT8)(OBEX_REQ_GET | OBEX_FINAL);
        p += OBEX_PKT_LEN_SIZE;

        /* add session sequence number, if session is active */
        if (p_cb->sess_st == OBEX_SESS_ACTIVE)
        {
            *p++ = OBEX_HI_SESSION_SN;
            *p++ = p_cb->ssn;
        }

        /* add connection ID, if needed */
        if ((p_cb->conn_id != OBEX_INVALID_CONN_ID) &&
            /* always use connection ID in CONNECTED state or being challenged on operation */
            (p_cb->state == OBEX_CS_CONNECTED))
        {
            *p++    = OBEX_HI_CONN_ID;
            UINT32_TO_BE_STREAM(p, p_cb->conn_id);
        }

        p_pkt = obx_cl_prepend_msg(p_cb, (BT_HDR *)NULL, msg, (UINT16)(p - msg) );
        obx_ca_snd_req(p_cb, p_pkt);
    }
    else
    {
        OBEX_TRACE_ERROR1("obx_resend_get_req_msg (hdl %x): Error getting buffer to resend\n", handle);
    }
}


/*******************************************************************************
** Function     obx_ca_srm_get_req
** Description  send a GET request when SRM is engaged
*******************************************************************************/
tOBEX_CL_STATE obx_ca_srm_get_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_COMM_CB    *p_comm = &p_cb->ll_cb.comm;

    OBEX_TRACE_DEBUG3("obx_ca_srm_get_req sess_st: %d, rx_q.count: %d, srm:0x%x\n", p_cb->sess_st, p_comm->rx_q.count, p_cb->srm );

    if (p_cb->srm == OBEX_SRM_ENABLE)
    {
        OBEX_TRACE_DEBUG3("SRM enabled but disengaged, obx_ca_snd_req");
        return obx_ca_snd_req(p_cb, p_pkt);
    }

    obx_start_timer(&p_cb->ll_cb.comm);
    p_cb->srm &= ~OBEX_SRM_WAIT_UL;
    if (p_cb->sess_st == OBEX_SESS_ACTIVE)
    {
        p_cb->ssn++;
    }

    if (p_pkt)
        GKI_freebuf(p_pkt);

    if ((p_pkt = (BT_HDR *)GKI_dequeue (&p_comm->rx_q)) != NULL)
    {
        obx_cl_proc_pkt (p_cb, p_pkt);
        obx_flow_control(p_comm);
        OBEX_TRACE_DEBUG1("Freeing dequeued and processed pkt 0x%x",p_pkt);
        GKI_freebuf(p_pkt);
        OBEX_TRACE_DEBUG1("obx_ca_srm_get_req rx_q.count: %d\n", p_cb->ll_cb.comm.rx_q.count );
    }
    OBEX_TRACE_DEBUG1("obx_ca_srm_get_req srm:0x%x\n", p_cb->srm );

    return OBEX_CS_NULL;
}

/*******************************************************************************
** Function     obx_ca_srm_put_notify
** Description  Notify the OBX user OBEX_PUT_RSP_EVT (OBX is ready for next req)
*******************************************************************************/
tOBEX_CL_STATE obx_ca_srm_put_notify(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE   state;

    OBEX_TRACE_DEBUG2("obx_ca_srm_put_notify srm: 0x%x, srmp: 0x%x\n", p_cb->srm, p_cb->srmp );

    state = obx_ca_notify (p_cb, p_pkt);
    return state;
}

/*******************************************************************************
** Function     obx_ca_srm_get_notify
** Description  Notify the OBX user OBEX_GET_RSP_EVT (OBX is ready for next req)
*******************************************************************************/
tOBEX_CL_STATE obx_ca_srm_get_notify(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE   state = OBEX_CS_NULL;

    OBEX_TRACE_DEBUG1("obx_ca_srm_get_notify srm: 0x%x\n", p_cb->srm );
    /* do not allow SRMP for now */
    p_cb->srm &= ~OBEX_SRM_WAIT;

    if (p_cb->srm & OBEX_SRM_ENGAGE)
    {
        if ((p_cb->srm & OBEX_SRM_WAIT_UL) == 0)
        {
            /* If get request is in response to a server response with SRMP=1, then send another request */
            if (p_pkt && (p_cb->srmp & OBEX_SRMP_CL_FINAL))
            {
                /* Only process SRMP header if Get response does not include END BODY header */
#ifdef OBEX_LIB_L2CAP_INCLUDED
                if ((OBEX_CheckHdr(p_pkt, OBEX_HI_BODY_END) == NULL))
                {
                    OBEX_TRACE_DEBUG1("obx_ca_srm_get_notify - Resending Get Request (SRMP=1), hdl %x\n", p_cb->ll_cb.l2c.handle);
                    obx_resend_get_req_msg(p_cb->ll_cb.l2c.handle);
                }
#endif
                p_cb->srmp &= ~OBEX_SRMP_CL_FINAL;
            }

            p_cb->srm |= OBEX_SRM_WAIT_UL;
            state = obx_ca_notify (p_cb, p_pkt);
        }
        else
        {
            GKI_enqueue_head  (&p_cb->ll_cb.comm.rx_q, p_pkt);
            OBEX_TRACE_DEBUG1("obx_ca_srm_get_notify rx_q.count:%d\n", p_cb->ll_cb.comm.rx_q.count);
        }
    }
    else
    {
        state = obx_ca_notify (p_cb, p_pkt);
        if (state == OBEX_CS_GET_SRM || state == OBEX_CS_NULL)
            state = OBEX_CS_GET_TRANSACTION;

        if (p_cb->state == OBEX_CS_GET_REQ_SENT && state == OBEX_CS_GET_TRANSACTION)
            state = OBEX_CS_GET_REQ_SENT;
    }
    OBEX_TRACE_DEBUG2("obx_ca_srm_get_notify srm: 0x%x(e) state:%s\n", p_cb->srm, obx_sr_get_state_name(state) );
    return state;
}

/*******************************************************************************
** Function     obx_ca_save_rsp
** Description  save response in control block
*******************************************************************************/
tOBEX_CL_STATE obx_ca_save_rsp(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    GKI_enqueue  (&p_cb->ll_cb.comm.rx_q, p_pkt);
    return OBEX_CS_NULL;
}

/*******************************************************************************
** Function     obx_ca_save_req
** Description  save request in control block
*******************************************************************************/
tOBEX_CL_STATE obx_ca_save_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    if (p_cb->p_next_req)
    {
        /* this probably would not happen */
        /* this action only occurs when we are flow controlled by the peer
         * and the client wants to abort the operation */
        /* Just in case that the user keeps calling abort request.... */
        OBEX_TRACE_WARNING1("free next req: 0x%x\n", p_cb->p_next_req );
        GKI_freebuf(p_cb->p_next_req);
    }

    p_cb->p_next_req = p_pkt;

    return OBEX_CS_NULL;
}


/*******************************************************************************
** Function     obx_ca_snd_req
** Description  If (p_pkt), call p_send_fn() to send the message to the peer.
**              Start timer. Return NULL state.If data is partially sent, return
**              PART state.
*******************************************************************************/
tOBEX_CL_STATE obx_ca_snd_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE state = OBEX_CS_NULL;
    UINT8   rsp_code = OBEX_RSP_DEFAULT;
    tOBEX_COMM_CB    *p_comm = &p_cb->ll_cb.comm;
    BT_HDR          *p_buf;

    obx_access_rsp_code(p_pkt, &rsp_code);
    p_cb->final = (rsp_code&OBEX_FINAL) ? TRUE : FALSE;
    OBEX_TRACE_DEBUG2("obx_ca_snd_req rsp_code: 0x%x final:%d\n", rsp_code, p_cb->final );

    /* save a copy of the request sent to the server */
    if (p_cb->p_saved_req)
        GKI_freebuf(p_cb->p_saved_req);

    p_cb->p_saved_req   = obx_dup_pkt(p_pkt);

    OBEX_TRACE_DEBUG3( "event p_saved_req:%d, pkt:%d, final: %d\n", p_cb->p_saved_req->event, p_pkt->event,p_cb->final);

    /* If Abort req is being sent, need to flush rx_q to prevent congestion */
    if (rsp_code == (OBEX_REQ_ABORT | OBEX_FINAL))
    {
        if (!GKI_queue_is_empty(&p_comm->rx_q))
        {
            OBEX_TRACE_DEBUG0("obx_ca_snd_req abort free rx_q\n");
            while((p_buf = (BT_HDR*)GKI_dequeue (&p_comm->rx_q)) != NULL)
                GKI_freebuf(p_buf);
        }
    }

    p_cb->ll_cb.comm.p_txmsg  = p_pkt;
#if BT_TRACE_PROTOCOL == TRUE
    DispObxMsg(p_pkt, obx_api_evt_to_disp_type[p_pkt->event]);
#endif
    /* debug: obxu_dump_hex ((UINT8 *)(p_pkt + 1) + p_pkt->offset, "conn req", p_pkt->len); */

    p_cb->srmp &= ~OBEX_SRMP_SESS_FST;
    if (p_cb->sess_st == OBEX_SESS_ACTIVE)
    {
        p_cb->ssn++;
    }

    if (p_cb->ll_cb.comm.p_send_fn(&p_cb->ll_cb) == FALSE)
    {
        p_cb->next_state   = p_cb->state;
        state = OBEX_CS_PARTIAL_SENT;
    }

    if (p_cb->srm == OBEX_SRM_ENABLE && p_cb->state == OBEX_CS_GET_SRM)
    {
        OBEX_TRACE_DEBUG3("SRM disengaged, sent get req, change state to OBEX_CS_GET_REQ_SENT\n");
        state = OBEX_CS_GET_REQ_SENT;
    }
    return state;
}

/*******************************************************************************
** Function     obx_ca_close_port
** Description  Close the transport. Return NULL state.
*******************************************************************************/
tOBEX_CL_STATE obx_ca_close_port(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    if (p_pkt)
        GKI_freebuf(p_pkt);
    p_cb->ll_cb.comm.p_close_fn(p_cb->ll_cb.comm.id);
    return OBEX_CS_NULL;
}

/*******************************************************************************
** Function     obx_ca_snd_part
** Description  Call p_send_fn() to send the left-over OBEX message to the peer.
**              Start timer. If all the data is sent, call obx_csm_event() with
**              STATE event to next_state in the port control block.
**              If (p_next_req), call obx_csm_event() to process the saved request.
**              Return NULL state.
*******************************************************************************/
tOBEX_CL_STATE obx_ca_snd_part(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_CL_STATE state = OBEX_CS_NULL;

    OBEX_TRACE_DEBUG1("obx_ca_snd_part sess_st:%d\n", p_cb->sess_st);

    /* p_pkt should be NULL here */
    if (p_cb->ll_cb.comm.p_send_fn(&p_cb->ll_cb) == TRUE)
    {
        /* data is all sent. change state to the appropriate state */
        obx_csm_event(p_cb, OBEX_STATE_CEVT, NULL);
        if (p_cb->p_next_req && (p_cb->sess_st != OBEX_SESS_CREATE))
        {
            /* abort request was issued - send it now */
            p_pkt = p_cb->p_next_req;
            p_cb->p_next_req = NULL;
            obx_csm_event(p_cb, (tOBEX_CL_EVENT)(p_pkt->event-1), p_pkt);
        }

        OBEX_TRACE_DEBUG2("obx_ca_snd_part state:%d, srm:0x%x\n", p_cb->state, p_cb->srm);
        if ((p_pkt = (BT_HDR *)GKI_dequeue (&p_cb->ll_cb.comm.rx_q)) != NULL)
        {
            obx_cl_proc_pkt (p_cb, p_pkt);
        }
        else if (p_cb->state == OBEX_CS_PUT_SRM)
        {
            if (((p_cb->srm & OBEX_SRM_WAIT) == 0) && (p_cb->final != TRUE))
            {
                state = obx_ca_notify (p_cb, NULL);
            }
        }
    }
    return state;
}

/*******************************************************************************
** Function     obx_ca_connect_error
** Description  Call callback function with OBEX_CLOSE_IND_EVT. Free the client
**              control block. Return NULL state.
*******************************************************************************/
tOBEX_CL_STATE obx_ca_connect_error(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_CL_CBACK   *p_cback = p_cb->p_cback;
    tOBEX_HANDLE     handle   = p_cb->ll_cb.comm.handle;
    tOBEX_EVT_PARAM  param    = p_cb->param;
    tOBEX_SR_STATE   save_state;

    if (p_cb->sess_st == OBEX_SESS_ACTIVE)
    {
        /* The transport is interrupted while a reliable session is active:
         * report a suspend event fot application to save the information in NV.
         * The only time this is called is for port close evt /w next state as not_connected
         * we need to use prev_state as the potential state to resume session
         */
        save_state = p_cb->prev_state;
        if (save_state == OBEX_CS_PARTIAL_SENT)
            save_state = p_cb->next_state;
        /* marks link drop suspend only when SRM is not engaged */
        if ((p_cb->srm & OBEX_SRM_ENGAGE) == 0)
            save_state |= OBEX_CL_STATE_DROP;
        else if (save_state == OBEX_CS_GET_SRM)
            p_cb->srm &= ~OBEX_SRM_WAIT_UL;
        p_cb->sess_info[OBEX_SESSION_INFO_ST_IDX] = save_state;
        OBEX_TRACE_DEBUG2("obx_ca_connect_error saved state:0x%x, srm:0x%x\n", save_state, p_cb->srm);
        p_cb->sess_info[OBEX_SESSION_INFO_SRM_IDX] = p_cb->srm;
        param.sess.p_sess_info   = p_cb->sess_info;
        param.sess.sess_op       = OBEX_SESS_OP_TRANSPORT;
        param.sess.sess_st       = p_cb->sess_st;
        param.sess.nssn          = p_cb->ssn;
        param.sess.ssn           = p_cb->ssn;
        param.sess.obj_offset    = 0;
        param.sess.timeout       = OBEX_SESS_TIMEOUT_VALUE;
        memcpy(param.sess.peer_addr, p_cb->peer_addr, BD_ADDR_LEN);
        p_cb->sess_st = OBEX_SESS_NONE;
        (*p_cback)(handle, OBEX_SESSION_INFO_EVT, OBEX_RSP_OK, param, NULL);
    }

    obx_cl_free_cb(p_cb);
    (*p_cback)(handle, OBEX_CLOSE_IND_EVT, OBEX_RSP_DEFAULT, param, (UINT8 *)p_pkt);
    return OBEX_CS_NULL;
}

/*******************************************************************************
** Function     obx_ca_connect_fail
** Description  Connect Request is rejected by server, call
**              obx_csm_event() with OBEX_DISCNT_REQ_CEVT. (We do not need to
**              send disconnect request to the server, since we are not
**              connected yet). Return NULL state.
*******************************************************************************/
tOBEX_CL_STATE obx_ca_connect_fail(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE   state = OBEX_CS_NULL;

    /* Notify the user of the failure and .. */
    (*p_cb->p_cback)(p_cb->ll_cb.comm.handle, OBEX_CONNECT_RSP_EVT, p_cb->rsp_code, p_cb->param, (UINT8 *)p_pkt);
    p_cb->api_evt   = OBEX_NULL_EVT;

    if (p_cb->rsp_code == OBEX_RSP_UNAUTHORIZED)
	return state;
    /* and close the RFCOMM port */
    obx_csm_event(p_cb, OBEX_DISCNT_REQ_CEVT, NULL);

    return state;
}

/*******************************************************************************
** Function     obx_ca_discnt_req
** Description  OBEX_DISCNT_REQ_CEVT event is received in OBEX_CS_CONNECT_REQ_SENT
**              state. Just close port.
*******************************************************************************/
tOBEX_CL_STATE obx_ca_discnt_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_CL_STATE state = OBEX_CS_NOT_CONNECTED;
    UINT8       msg[OBEX_HDR_OFFSET];
    UINT8       *p = msg;

    /* connection is not officially up yet.
     * just close the port */
    obx_ca_close_port(p_cb, p_pkt);

    return state;
}

/*******************************************************************************
** Function     obx_ca_notify
** Description  Use api_evt or look up the event according to the state. Fill
**              the event parameter. Call callback function with the event.
**              Return NULL state.
*******************************************************************************/
tOBEX_CL_STATE obx_ca_notify(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE   state = OBEX_CS_CONNECTED;
    tOBEX_EVENT      event = obx_cl_state_2_event_map[p_cb->state - 1];
    BOOLEAN         notify = FALSE;
    tOBEX_CL_EVENT   sm_evt = OBEX_BAD_SM_EVT;
    BT_HDR          *p_req = NULL;

    OBEX_TRACE_DEBUG6( "obx_ca_notify state: %s, prev_state: %s, rsp:0x%x, sess_st:%d, event:%d srm:0x%x\n",
        obx_cl_get_state_name( p_cb->state ), obx_cl_get_state_name(p_cb->prev_state), p_cb->rsp_code, p_cb->sess_st, event, p_cb->srm);
    OBEX_TRACE_DEBUG2( "ssn:0x%x/0x%x\n", p_cb->ssn, p_cb->param.ssn);

    if ( (p_cb->final == TRUE && p_cb->rsp_code == OBEX_RSP_CONTINUE && p_cb->state == OBEX_CS_PUT_TRANSACTION) ||
        (p_cb->final == FALSE && p_cb->rsp_code != OBEX_RSP_CONTINUE) )
    {
        /* final bit on the request mismatch the responde code --- Error!! */
        OBEX_TRACE_ERROR2( "final:%d on the request mismatch the responde code:0x%x\n",
            p_cb->final, p_cb->rsp_code) ;
        /* change the state to not connected state */
        p_cb->next_state   = OBEX_CS_NOT_CONNECTED;
        obx_csm_event(p_cb, OBEX_STATE_CEVT, NULL);
        notify  = TRUE;
        /* send a tx_empty event to close port */
        sm_evt  = OBEX_TX_EMPTY_CEVT;
    }

    else if (event != OBEX_NULL_EVT)
    {
        switch (p_cb->state)
        {
        case OBEX_CS_PUT_TRANSACTION:
        case OBEX_CS_GET_TRANSACTION:
        case OBEX_CS_PUT_REQ_SENT:
        case OBEX_CS_GET_REQ_SENT:
        case OBEX_CS_PUT_SRM:
        case OBEX_CS_GET_SRM:
            if (p_cb->rsp_code == OBEX_RSP_CONTINUE )
            {
                /* notify the event in this function. the new state stays the same */
                notify = TRUE;

                if (p_cb->srm & OBEX_SRM_ENGAGE)
                {
                    if (p_cb->state == OBEX_CS_PUT_TRANSACTION)
                    {
                        p_cb->state = OBEX_CS_PUT_SRM;
                    }
                    else if (p_cb->state == OBEX_CS_GET_TRANSACTION)
                    {
                        p_cb->state = OBEX_CS_GET_SRM;
                    }
                    else if (p_cb->state == OBEX_CS_PUT_SRM && p_pkt && (p_cb->srm & OBEX_SRM_WAIT) == 0)
                    {
                        OBEX_TRACE_ERROR0 ("unexpected PUT response. disconnect now!!\n");
                        notify = FALSE;
                        event   = OBEX_NULL_EVT;
                        obx_ca_close_port(p_cb, p_pkt);
                    }
                    /* clear the wait bit here to avoid the link being disconnected by accident */
                    p_cb->srm &= ~OBEX_SRM_WAIT;
                    if (p_cb->srmp)
                    {
                        p_cb->srmp = 0;
                        p_cb->srm |= OBEX_SRM_WAIT;
                    }
                }
            }
            /* else let obx_csm_event notify the event. the new state is OBEX_CS_CONNECTED */
            else
            {
                /* dis-engage SRM */
                p_cb->srm &= OBEX_SRM_ENABLE;
                OBEX_TRACE_DEBUG1( "disengage srm:0x%x\n", p_cb->srm);
            }
            break;

        case OBEX_CS_NOT_CONNECTED:
            if (p_cb->sess_st == OBEX_SESS_ACTIVE && p_cb->prev_state == OBEX_CS_DISCNT_REQ_SENT)
            {
                p_req   = obx_ca_close_sess_req (p_cb);
                sm_evt  = OBEX_SESSION_REQ_CEVT;
                state   = OBEX_CS_NULL;
            }
            else
            {
                notify  = TRUE;
                /* send a tx_empty event to close port */
                sm_evt  = OBEX_TX_EMPTY_CEVT;
            }
            break;

        case OBEX_CS_CONNECT_REQ_SENT:
            if (p_cb->rsp_code == OBEX_RSP_FAILED)
            {
                /* client challenged the server and the server does not return a good digest */
                notify  = TRUE;
                /* send a disconnect req event to close port */
                sm_evt  = OBEX_DISCNT_REQ_CEVT;
            }
            break;

        case OBEX_CS_ABORT_REQ_SENT:
            p_cb->srm &= OBEX_SRM_ENABLE;
            OBEX_TRACE_DEBUG1( "(ab) disengage srm:0x%x\n", p_cb->srm);
            break;
        }
    }

    if (notify == TRUE )
    {
        (*p_cb->p_cback)(p_cb->ll_cb.comm.handle, event, p_cb->rsp_code, p_cb->param, (UINT8 *)p_pkt);
        event   = OBEX_NULL_EVT;
    }

    if (sm_evt != OBEX_BAD_SM_EVT)
    {
        /* send an event to csm */
        obx_csm_event(p_cb, sm_evt, p_req);
    }

    p_cb->api_evt   = event;
    if (event == OBEX_NULL_EVT)
    {
        state = OBEX_CS_NULL;
    }
    else
    {
        p_cb->ssn = p_cb->param.ssn;
        OBEX_TRACE_DEBUG1( "ssn:0x%x\n", p_cb->ssn);
    }
    OBEX_TRACE_DEBUG1("obx_ca_notify %d", state);
    return state;
}

/*******************************************************************************
** Function     obx_ca_fail_rsp
** Description  Save the OBEX message in control block, set api_evt according to
**              the event/state table. Return CONN state.
*******************************************************************************/
tOBEX_CL_STATE obx_ca_fail_rsp(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_SR_STATE   state = OBEX_CS_CONNECTED;

    p_cb->srm &= OBEX_SRM_ENABLE;

    p_cb->api_evt   = obx_cl_state_2_event_map[p_cb->state - 1];

    return state;
}
