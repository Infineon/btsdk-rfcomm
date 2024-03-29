/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
**  Name:         obx_utils.c
**
**  File:         OBEX common utility functions
**
**
*****************************************************************************/
#include <string.h>
#include <stdio.h>
#include "wiced_bt_rfcomm.h"

#include "obx_int.h"

const tOBEX_EVENT obx_sm_evt_to_api_evt[OBEX_MAX_EVT_MAP_NUM] =
{
    OBEX_CONNECT_REQ_EVT,
    OBEX_SESSION_REQ_EVT,    /* A Crease Session or Resume Session request is received by the server. Call OBEX_SessionRsp(). */
    OBEX_DISCONNECT_REQ_EVT,
    OBEX_PUT_REQ_EVT,
    OBEX_GET_REQ_EVT,
    OBEX_SETPATH_REQ_EVT,
    OBEX_ACTION_REQ_EVT,     /* An Action request is received by the server. Call OBEX_ActionRsp(). */
    OBEX_ABORT_REQ_EVT
};

#if BT_TRACE_PROTOCOL == TRUE
const UINT8 obx_api_evt_to_disp_type[] =
{
    OBEX_NULL_DISP,
    /* server events */
    OBEX_CONNECT_REQ_DISP,
    OBEX_SESSION_REQ_DISP,
    OBEX_DISCONNECT_REQ_DISP,
    OBEX_PUT_REQ_DISP,
    OBEX_GET_REQ_DISP,
    OBEX_SETPATH_REQ_DISP,
    OBEX_ABORT_REQ_DISP,
    OBEX_ACTION_REQ_DISP,

    /* client events */
    OBEX_CONNECT_RSP_DISP,
    OBEX_SESSION_RSP_DISP,
    OBEX_DISCONNECT_RSP_DISP,
    OBEX_PUT_RSP_DISP,
    OBEX_GET_RSP_DISP,
    OBEX_SETPATH_RSP_DISP,
    OBEX_ACTION_RSP_DISP,
    OBEX_ABORT_RSP_DISP
};
#endif

const UINT8 obx_rsp_code[] =
{
    OBEX_RSP_CONTINUE,            /* 0x10 Continue */
    OBEX_RSP_OK,                  /* 0x20 OK, Success */
    OBEX_RSP_CREATED,             /* 0x21 Created */
    OBEX_RSP_ACCEPTED,            /* 0x22 Accepted */
    OBEX_RSP_NON_AUTH_INFO,       /* 0x23 Non-Authoritative Information */
    OBEX_RSP_NO_CONTENT,          /* 0x24 No Content */
    OBEX_RSP_RESET_CONTENT,       /* 0x25 Reset Content */
    OBEX_RSP_PART_CONTENT,        /* 0x26 Partial Content */
    OBEX_RSP_MULTI_CHOICES,       /* 0x30 Multiple Choices */
    OBEX_RSP_MVD_PERM,            /* 0x31 Moved Permanently */
    OBEX_RSP_MVD_TEMP,            /* 0x32 Moved temporarily */
    OBEX_RSP_SEE_OTHER,           /* 0x33 See Other */
    OBEX_RSP_NOT_MODIFIED,        /* 0x34 Not modified */
    OBEX_RSP_USE_PROXY,           /* 0x35 Use Proxy */
    OBEX_RSP_BAD_REQUEST,         /* 0x40 Bad Request - server couldn't understand request */
    OBEX_RSP_UNAUTHORIZED,        /* 0x41 Unauthorized */
    OBEX_RSP_PAYMENT_REQD,        /* 0x42 Payment required */
    OBEX_RSP_FORBIDDEN,           /* 0x43 Forbidden - operation is understood but refused */
    OBEX_RSP_NOT_FOUND,           /* 0x44 Not Found */
    OBEX_RSP_NOT_ALLOWED,         /* 0x45 Method not allowed */
    OBEX_RSP_NOT_ACCEPTABLE,      /* 0x46 Not Acceptable */
    OBEX_RSP_PROXY_AUTH_REQD,     /* 0x47 Proxy Authentication required */
    OBEX_RSP_REQUEST_TIMEOUT,     /* 0x48 Request Time Out */
    OBEX_RSP_CONFLICT,            /* 0x49 Conflict */
    OBEX_RSP_GONE,                /* 0x4A Gone */
    OBEX_RSP_LENGTH_REQD,         /* 0x4B Length Required */
    OBEX_RSP_PRECONDTN_FAILED,    /* 0x4C Precondition failed */
    OBEX_RSP_REQ_ENT_2_LARGE,     /* 0x4D Requested entity too large */
    OBEX_RSP_REQ_URL_2_LARGE,     /* 0x4E Request URL too large */
    OBEX_RSP_UNSUPTD_TYPE,        /* 0x4F Unsupported media type */
    OBEX_RSP_INTRNL_SRVR_ERR,     /* 0x50 Internal Server Error */
    OBEX_RSP_NOT_IMPLEMENTED,     /* 0x51 Not Implemented */
    OBEX_RSP_BAD_GATEWAY,         /* 0x52 Bad Gateway */
    OBEX_RSP_SERVICE_UNAVL,       /* 0x53 Service Unavailable */
    OBEX_RSP_GATEWAY_TIMEOUT,     /* 0x54 Gateway Timeout */
    OBEX_RSP_HTTP_VER_NOT_SUPTD,  /* 0x55 HTTP version not supported */
    OBEX_RSP_DATABASE_FULL,       /* 0x60 Database Full */
    OBEX_RSP_DATABASE_LOCKED,     /* 0x61 Database Locked */
    OBEX_RSP_DEFAULT
};

static void obx_read_mtu(BT_HDR *p_pkt, tOBEX_HANDLE handle, tOBEX_CONN_EVT  *p_evt);

/*******************************************************************************
** Function     obx_read_srm
** Description  read the SRM and SRM_PARAM headers from the packet received from
**              peer and set the control block data member accordingly
** Return       UINT8
*******************************************************************************/
UINT8 obx_read_srm (tOBEX_SRM *p_srm, BOOLEAN is_client, BT_HDR *p_pkt)
{
    UINT8   srm = 0, srmp = 0, ret_srmp=0;
    UINT8   old_srm = *p_srm;
    BOOLEAN allowed = FALSE, clear = TRUE;

    OBEX_TRACE_DEBUG1("obx_read_srm srm:0x%x\n", *p_srm );
    if (*p_srm)
    {
        /* if the SRM enable request is not granted in the next packet, the request is not valid any more
         * clear the requesting flag */
        *p_srm &= ~OBEX_SRM_REQING;

        if (OBEX_Read1ByteHdr(p_pkt, OBEX_HI_SRM, &srm))
        {
            if (srm == OBEX_HV_SRM_ENABLE)
            {
                if (is_client)
                {
                    if (old_srm & OBEX_SRM_REQING)
                    {
                        *p_srm |= OBEX_SRM_ENGAGE;
                        allowed = TRUE;
                    }
                }
                else /* is server */
                {
                    *p_srm |= OBEX_SRM_REQING;
                    allowed = TRUE;
                }
            }
            OBEX_TRACE_DEBUG3("SRM :0x%x srm:0x%x old_srm:0x%x\n", srm, *p_srm, old_srm );
        }

        if (!allowed)
            allowed = old_srm & OBEX_SRM_PARAM_AL;

        if (OBEX_Read1ByteHdr(p_pkt, OBEX_HI_SRM_PARAM, &srmp))
        {
            if ((srmp == OBEX_HV_SRM_PARAM_WAIT) && allowed)
            {
                ret_srmp = OBEX_SRMP_WAIT;
                *p_srm |= OBEX_SRM_PARAM_AL;

                /* Mark the case where server asks client to wait during a Get request  (SRMP=1) */
                if (is_client)
                    ret_srmp |= OBEX_SRMP_CL_FINAL;

                clear = FALSE;
            }
        }
        OBEX_TRACE_DEBUG4("SRM_PARAM :0x%x srm:0x%x allowed:%d clear:%d\n", srmp, *p_srm, allowed, clear );

        /* once the SRMP header is not used, it should be ignored for the rest of the transaction */
        if (clear)
            *p_srm &= ~(OBEX_SRM_PARAM_AL | OBEX_SRMP_CL_FINAL);
    }

    return ret_srmp;
}

/*******************************************************************************
** Function     obx_add_timeout
** Description  add the timeout triplet
**
** Return       UINT8
*******************************************************************************/
UINT8 obx_add_timeout (tOBEX_TRIPLET *p_trip, UINT32 timeout, tOBEX_SESS_EVT *p_param)
{
    UINT8   *p;
    UINT8   ret = 0;

    if (timeout != OBEX_INFINITE_TIMEOUT)
    {
        p_trip->tag = OBEX_TAG_SESS_PARAM_TOUT;
        p_trip->len = OBEX_TIMEOUT_SIZE;
        p = p_trip->p_array;
        UINT32_TO_BE_STREAM(p, timeout);
        ret = 1;
    }
    p_param->timeout = timeout;
    return ret;
}

/*******************************************************************************
** Function     obx_read_timeout
** Description  add the timeout triplet
**
** Return       void
*******************************************************************************/
void obx_read_timeout (tOBEX_TRIPLET *p_trip, UINT8 num, UINT32 *p_timeout, UINT8 *p_toa)
{
    UINT8   ind;
    UINT8   *p;
    UINT32  tmp;

    p = p_toa;
    BE_STREAM_TO_UINT32(tmp, p);
    OBEX_TRACE_DEBUG2("obx_read_timeout %d/%d\n", *p_timeout, tmp);
    if (*p_timeout == 0)
        *p_timeout = tmp;
    ind = obx_read_triplet(p_trip, num, OBEX_TAG_SESS_PARAM_TOUT);
    if ((ind != num) && (p_trip[ind].len == OBEX_TIMEOUT_SIZE))
    {
        p = p_trip[ind].p_array;
        BE_STREAM_TO_UINT32(tmp, p);
        if (tmp < (*p_timeout))
        {
            (*p_timeout) = tmp;
            OBEX_TRACE_DEBUG1("new timeout %d\n", tmp);
        }
    }
    UINT32_TO_BE_STREAM(p_toa, (*p_timeout));
}

/*******************************************************************************
** Function     obx_verify_response
** Description
** Return       OBEX_BAD_SM_EVT, if bad.
*******************************************************************************/
UINT8 obx_verify_response(UINT8 opcode, tOBEX_RX_HDR *p_rxh)
{
    UINT8   final   = (opcode & OBEX_FINAL) ? TRUE : FALSE;
    int     xx      = 0;
    UINT8   res_code = opcode & ~OBEX_FINAL;

    p_rxh->sm_evt = OBEX_BAD_SM_EVT;

    /* response packet must have the final bit set */
    if (final == TRUE)
    {
        if (res_code == OBEX_RSP_CONTINUE)
        {
            p_rxh->sm_evt   = OBEX_CONT_CFM_CEVT;
        }
        else
        {
            /* figure out the kind of response, Continue, OK, or Error */
            while(obx_rsp_code[xx] != OBEX_RSP_DEFAULT)
            {
                if (obx_rsp_code[xx] == res_code)
                    break;
                xx++;
            }

            if (obx_rsp_code[xx] != OBEX_RSP_DEFAULT)
            {
                if (obx_rsp_code[xx] <= OBEX_MAX_OK_RSP)
                    p_rxh->sm_evt   = OBEX_OK_CFM_CEVT;
                else
                    p_rxh->sm_evt   = OBEX_FAIL_CFM_CEVT;
            }
            /* else bad response code */
        }
        if (p_rxh->sm_evt != OBEX_BAD_SM_EVT)
        {
            p_rxh->code     = opcode;
        }
    }
    /*
    else final bit not set in response packet -> bad response code
     */

    return p_rxh->sm_evt;
}

/*******************************************************************************
**
** Function     obx_cl_proc_pkt
**
** Description  process a packet received from the connected server
**              verify that the response is valid
**              fill in event parameters
**              call csm to process the event
**
** Returns      void
**
*******************************************************************************/
void obx_cl_proc_pkt (tOBEX_CL_CB *p_cb, BT_HDR *p_pkt)
{
    tOBEX_RX_HDR *p_rxh;
    UINT32      conn_id;
    BOOLEAN     pass = FALSE;
    UINT8       xx;
    tOBEX_CL_EVENT   sm_evt;
    UINT8       ssn;

    OBEX_TRACE_DEBUG3("obx_cl_proc_pkt 0x%x srm:0x%x, sess_st:%d\n", p_pkt, p_cb->srm, p_cb->sess_st);
    OBEX_TRACE_DEBUG1("csm offset:%d\n", p_cb->param.sess.obj_offset);

    p_rxh = (tOBEX_RX_HDR *)(p_pkt + 1);
    p_cb->rsp_code      = p_rxh->code & ~OBEX_FINAL;

    p_pkt->event    = OBEX_PUT_RSP_EVT; /* any response */
    /* setup callback event param
    memset(&p_cb->param, 0, sizeof(tOBEX_EVT_PARAM)); */

    if (p_cb->state == OBEX_CS_CONNECT_REQ_SENT )
    {
        /* when a response packet is received in conn_rs state,
         * it must be a connect response packet */
        p_pkt->event            = OBEX_CONNECT_RSP_EVT;
        p_cb->param.conn.handle = p_cb->ll_cb.port.handle;
        obx_read_mtu(p_pkt, p_cb->ll_cb.port.handle, &(p_cb->param.conn));
        p_cb->ll_cb.port.tx_mtu = p_cb->param.conn.mtu;

        /* save Connection ID */
        if (OBEX_Read4ByteHdr(p_pkt, OBEX_HI_CONN_ID, &conn_id) == TRUE)
            p_cb->conn_id   = conn_id;
        OBEX_TRACE_DEBUG1("Connection ID: 0x%x\n", p_cb->conn_id );
    }

    if (p_cb->sess_st == OBEX_SESS_ACTIVE)
    {
        /* verify the session sequence number */
        if (OBEX_Read1ByteHdr (p_pkt, OBEX_HI_SESSION_SN, &ssn))
        {
            OBEX_TRACE_DEBUG1("ssn:%d\n", ssn);
            p_cb->param.ssn = ssn;
        }
    }

#if BT_TRACE_PROTOCOL == TRUE
    DispObxMsg(p_pkt, (BOOLEAN)(obx_api_evt_to_disp_type[p_pkt->event] | OBEX_DISP_IS_RECV));
#endif
    sm_evt = p_rxh->sm_evt;

    p_cb->srmp = obx_read_srm (&p_cb->srm, TRUE, p_pkt);
    obx_csm_event(p_cb, sm_evt, p_pkt);
}

/*******************************************************************************
**
** Function     obx_cl_prepend_msg
**
** Description  This function is used by API functions to add the data in the
**              reserved space in the OBEX packet
**
** Returns      void.
**
*******************************************************************************/
BT_HDR * obx_cl_prepend_msg(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt, UINT8 * p_data, UINT16 data_len)
{
    UINT8   *p;
    UINT16  len;

    if (p_pkt == NULL)
    {
        p_pkt = (BT_HDR *)wiced_bt_obex_header_init(OBEX_HANDLE_NULL, OBEX_MIN_MTU);
        len = data_len;
    }
    else
    {
        len = p_pkt->len + data_len;
    }

    WC_ASSERT(p_pkt->offset >= data_len);
    p = (UINT8 *)(p_pkt + 1) + p_pkt->offset - data_len;
    memcpy(p, p_data, data_len);
    p++;
    /* adjust the packet len */
    UINT16_TO_BE_STREAM(p, len);
    p_pkt->len      += data_len;
    p_pkt->offset   -= data_len;
    p_pkt->layer_specific   -= data_len;

    return p_pkt;
}

/*******************************************************************************
** Function     obx_verify_request
** Description
** Return       OBEX_BAD_SM_EVT, if bad.
*******************************************************************************/
UINT8 obx_verify_request(UINT8 opcode, tOBEX_RX_HDR *p_rxh)
{
    UINT8   final = (opcode & OBEX_FINAL) ? TRUE : FALSE;
    UINT8   req_code = opcode & ~OBEX_FINAL;
    p_rxh->sm_evt = OBEX_BAD_SM_EVT;

    OBEX_TRACE_DEBUG2("obx_verify_request opcode 0x%x: final:%d\n", opcode, final);
    switch (req_code)
    {
    case OBEX_REQ_CONNECT:
        /* this request must have the final bit set */
        if (final == TRUE)
            p_rxh->sm_evt   = OBEX_CONNECT_REQ_SEVT;
        break;

    case OBEX_REQ_SESSION:
        /* this request must have the final bit set */
        if (final == TRUE)
            p_rxh->sm_evt   = OBEX_SESSION_REQ_SEVT;
        break;

    case OBEX_REQ_ACTION:
        /* this request must have the final bit set */
        if (final == TRUE)
            p_rxh->sm_evt   = OBEX_ACTION_REQ_SEVT;
        break;

    case OBEX_REQ_DISCONNECT:
        /* this request must have the final bit set */
        if (final == TRUE)
            p_rxh->sm_evt   = OBEX_DISCNT_REQ_SEVT;
        break;

    case OBEX_REQ_PUT:
        p_rxh->sm_evt       = OBEX_PUT_REQ_SEVT;
        break;

    case OBEX_REQ_GET:
        p_rxh->sm_evt       = OBEX_GET_REQ_SEVT;
        break;

    case OBEX_REQ_SETPATH:
        /* this request must have the final bit set */
        if (final == TRUE)
            p_rxh->sm_evt   = OBEX_SETPATH_REQ_SEVT;
        break;

    case OBEX_REQ_ABORT:
        /* this request must have the final bit set */
        if (final == TRUE)
            p_rxh->sm_evt   = OBEX_ABORT_REQ_SEVT;
        break;
    }

    if (p_rxh->sm_evt != OBEX_BAD_SM_EVT)
    {
        p_rxh->code     = opcode;
    }
    return p_rxh->sm_evt;
}

/*******************************************************************************
**
** Function     obx_is_get_or_put_cont
**
** Returns      TRUE, if it's a GET/PUT continuing continuing
**              TRUE, if it's an ABORT
**
*******************************************************************************/
BOOLEAN obx_is_get_or_put_cont (tOBEX_SR_SESS_CB *p_scb, UINT8 req_code)
{
    BOOLEAN is_cont = FALSE;

    if (req_code == OBEX_REQ_ABORT)
    {
        is_cont = TRUE;
    }
    else if (req_code == OBEX_REQ_PUT)
    {
        if (p_scb->state == OBEX_SS_PUT_TRANSACTION || p_scb->state == OBEX_SS_PUT_SRM)
        {
            is_cont = TRUE;
        }
    }
    else if (req_code == OBEX_REQ_GET)
    {
        if (p_scb->state == OBEX_SS_GET_TRANSACTION || p_scb->state == OBEX_SS_GET_SRM)
        {
            is_cont = TRUE;
        }
    }

    return is_cont;
}

/*******************************************************************************
**
** Function     obx_sr_proc_pkt
**
** Description  process a packet received from the connected client
**              verify that the request is valid
**              fill in event parameters
**              call ssm to process the event
**
** Returns      TRUE, if abort
**
*******************************************************************************/
BOOLEAN obx_sr_proc_pkt (tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt)
{
    tOBEX_RX_HDR *p_rxh;
    UINT8       *p_body;
    UINT16      len;
    BOOLEAN     end;
    UINT32      conn_id     = 0;
    UINT8       req_code;
    BOOLEAN     final;
    UINT8       ssn, tmp_ssn;
    UINT8       num_hdrs, num_body, num_ssn;
    UINT8       rsp_code = OBEX_RSP_OK;
    UINT8       num_id;
    BOOLEAN     chk_add = FALSE;

    p_rxh   = (tOBEX_RX_HDR *)(p_pkt + 1);
    final   = (p_rxh->code & OBEX_FINAL) ? TRUE : FALSE;
    req_code            = p_rxh->code & ~OBEX_FINAL;
    p_scb->api_evt      = (UINT8)p_pkt->event;
    memset(&p_scb->param, 0, sizeof(tOBEX_EVT_PARAM));
    p_scb->param.get.final  = final;

    num_id  = OBEX_Read4ByteHdr (p_pkt, OBEX_HI_CONN_ID, &conn_id);

    OBEX_TRACE_DEBUG6("obx_sr_proc_pkt 0x%x srm:0x%x sess_st:%d req_code:0x%x, sm_evt:%d ssn:%d\n",
        p_pkt, p_scb->srm, p_scb->sess_st, req_code, p_rxh->sm_evt, p_scb->ssn);
    OBEX_TRACE_DEBUG1("srmp:0x%x\n", p_scb->srmp);

    num_ssn = OBEX_Read1ByteHdr (p_pkt, OBEX_HI_SESSION_SN, &ssn);
    if (p_scb->srm & OBEX_SRM_ABORT)
    {
        /* OBEX_SRM_ABORT bit is set by server only when PUT req /w SRM is rejected before transaction actually ends
         * this means we need to ignore follow packets until the next transaction starts */
        if (req_code == OBEX_REQ_PUT)
        {
            /* check if this is the put for the previous transaction */
            num_hdrs = OBEX_ReadNumHdrs(p_pkt, &num_body);
            OBEX_TRACE_DEBUG4("num_hdrs:%d num_body:%d num_id:%d num_ssn:%d\n", num_hdrs, num_body, num_id, num_ssn);
            num_body += num_ssn;
            num_body += num_id;
            if (num_hdrs <= num_body)
            {
                p_scb->ssn = ssn+1;
                /* it is left-over, drop it. */
                GKI_freebuf (p_pkt);
                p_scb->api_evt      = OBEX_NULL_EVT;
                return TRUE;
            }
        }
        /* clear the SRM bits; leave only the enabled bit */
        p_scb->srm &= OBEX_SRM_ENABLE;
    }

    if ((p_scb->sess_st == OBEX_SESS_ACTIVE) && (req_code != OBEX_REQ_SESSION) && (req_code != OBEX_REQ_ABORT))
    {
        rsp_code = OBEX_RSP_BAD_REQUEST;
        /* verify the session sequence number */
        if (num_ssn)
        {
            OBEX_TRACE_DEBUG2("ssn pkt/cb=%d/%d\n", ssn, p_scb->ssn);
            if (ssn == p_scb->ssn)
            {
                p_scb->param.ssn = p_scb->ssn;
                rsp_code = OBEX_RSP_OK;
            }
            else if (p_scb->srmp & OBEX_SRMP_SESS_FST)
            {
                tmp_ssn = ssn+1;
                if (tmp_ssn == p_scb->ssn)
                {
                    p_scb->param.ssn = ssn;
                    p_scb->ssn = ssn;
                    rsp_code = OBEX_RSP_OK;
                }
            }
        }
        p_scb->srmp &= ~OBEX_SRMP_SESS_FST;
    }

    /* fill event parameter */
    if (req_code == OBEX_REQ_CONNECT)
    {
        p_scb->param.conn.handle = p_scb->handle;
        obx_read_mtu(p_pkt, p_scb->ll_cb.comm.handle, &(p_scb->param.conn));

        p_scb->ll_cb.port.tx_mtu    = p_scb->param.conn.mtu;
        /* verify the target header and connection ID
         * in obx_sa_connect_ind() for OBEX_SS_NOT_CONNECTED */
        /* verify the connection ID in obx_sa_auth_ind() for OBEX_SS_WAIT_AUTH */
        chk_add = TRUE;
    }
    else if (req_code == OBEX_REQ_SESSION)
    {
        /* do nothing */
        if (conn_id != 0)
        {
            OBEX_TRACE_ERROR1("Session command should not use Connection ID: 0x%x\n", conn_id);
            p_rxh->sm_evt   = OBEX_BAD_REQ_SEVT;
        }
    }
    else
    {
        OBEX_TRACE_DEBUG3("Connection ID: 0x%x/0x%x state:%d\n", p_scb->conn_id, conn_id, p_scb->state );
        /* verify the connection ID */
        if ( (conn_id == p_scb->conn_id) ||
            (conn_id == 0 && p_scb->conn_id != 0 && obx_is_get_or_put_cont(p_scb, req_code)))
        {
            /* match: both non-exist or both exist and equal */
            /* connection ID header does not exist, but control block uses connection ID,
             * if this is continuation packets for PUT and GET, it's OK */
            if (req_code == OBEX_REQ_PUT)
            {
                p_scb->param.put.final   = final;

                /* determine the PUT type */
                p_scb->param.put.type    = OBEX_PUT_TYPE_PUT;
                if (p_scb->state == OBEX_SS_CONNECTED && final == TRUE)
                {
                    if (OBEX_ReadBodyHdr(p_pkt, &p_body, &len, &end) == FALSE)
                    {
                        /* final is set, no BODY or End-of-Body
                         * -> Delete request */
                        p_scb->param.put.type    = OBEX_PUT_TYPE_DELETE;
                    }
                    else if (end == TRUE && len == 0)
                    {
                        /* an empty End-of-Body header
                         * -> Create-Empty request */
                        p_scb->param.put.type    = OBEX_PUT_TYPE_CREATE;
                    }
                }
                OBEX_TRACE_EVENT1("Put request type: %d\n", p_scb->param.put.type);
            }
            else if (req_code == OBEX_REQ_SETPATH)
            {
                p_scb->param.sp.flag  = *((UINT8 *)(p_pkt + 1) + p_pkt->offset + OBEX_SETPATH_FLAG_OFFSET);
            }
            /* Ignore a Get request when SRM is already engaged (per spec) */
            else if (req_code == OBEX_REQ_GET && (p_scb->srm & OBEX_SRM_ENGAGE) &&
                     (!((p_scb->srmp & OBEX_SRMP_NONF) == OBEX_SRMP_NONF)) )
            {
                OBEX_TRACE_WARNING0("GET Request while SRM already engaged!!! (ignoring...)\n");
                /* Free the memory for this request since we're ignoring */
                GKI_freebuf (p_pkt);
                p_scb->api_evt = OBEX_NULL_EVT;
                return FALSE;
            }

        }
        else
        {
            /* does not have good connection ID */
            p_rxh->sm_evt   = OBEX_BAD_REQ_SEVT;
            p_scb->api_evt  = OBEX_NULL_EVT;
        }
    }

    /* process the SRM header */
    p_scb->srmp |= obx_read_srm (&p_scb->srm, FALSE, p_pkt);
    OBEX_TRACE_DEBUG2("After process SRM header p_scb->srm=0x%x, p_scb->srmp=0x%x\n",
                     p_scb->srm, p_scb->srmp);

    if (p_rxh->sm_evt != OBEX_BAD_REQ_SEVT && req_code != OBEX_REQ_ABORT)
    {
        p_scb->cur_op = req_code;
        if (final)
            p_scb->cur_op |= OBEX_FINAL;
    }

    if (rsp_code != OBEX_RSP_OK)
    {
        p_rxh->sm_evt = OBEX_BAD_REQ_SEVT;
    }

    OBEX_TRACE_DEBUG2("rsp_code:0x%x, sm_evt:%d\n", rsp_code, p_rxh->sm_evt);

    obx_ssm_event(p_scb, p_rxh->sm_evt, p_pkt);
    if (chk_add)
    {
        if (p_scb->ll_cb.comm.p_send_fn == (tOBEX_SEND_FN *)obx_rfc_snd_msg)
            obx_add_port(p_scb->handle);
    }
    return FALSE;
}

/*******************************************************************************
**
** Function     obx_sr_prepend_msg
**
** Description  This function is used by API functions to add the data in the
**              reserved space in the OBEX packet
**
** Returns      void.
**
*******************************************************************************/
BT_HDR * obx_sr_prepend_msg(BT_HDR *p_pkt, UINT8 * p_data, UINT16 data_len)
{
    UINT8   *p;
    UINT16  len;

    if (p_pkt == NULL)
    {
        p_pkt = (BT_HDR *)wiced_bt_obex_header_init(OBEX_HANDLE_NULL, OBEX_MIN_MTU);
        len = data_len;
    }
    else
    {
        len = p_pkt->len + data_len;
    }

    WC_ASSERT(p_pkt->offset >= data_len);
    p = (UINT8 *)(p_pkt + 1) + p_pkt->offset - data_len;
    memcpy(p, p_data, data_len);
    p++;
    /* adjust the packet len */
    UINT16_TO_BE_STREAM(p, len);
    p_pkt->len      += data_len;
    p_pkt->offset   -= data_len;
    p_pkt->layer_specific   -= data_len;

    return p_pkt;
}

/*******************************************************************************
**
** Function     obx_read_mtu
**
** Description  This function is used to access the MTU in CONNECT packets
**
** Returns      void.
**
*******************************************************************************/
void obx_read_mtu(BT_HDR *p_pkt, tOBEX_HANDLE handle, tOBEX_CONN_EVT  *p_evt)
{
    UINT8   *p = (UINT8 *)(p_pkt + 1) + p_pkt->offset + OBEX_CONNECT_MTU_OFFSET;

    BE_STREAM_TO_UINT16(p_evt->mtu, p);
    if (p_evt->mtu > OBEX_MAX_MTU)
        p_evt->mtu = OBEX_MAX_MTU;
    if (p_evt->mtu == 0)
        p_evt->mtu = OBEX_MIN_MTU;

    /* Get the Bd_Addr */
    wiced_bt_obex_get_peer_addr (handle, p_evt->peer_addr);
}

/*******************************************************************************
**
** Function     obx_free_buf
**
** Description  This function is used to free the GKI buffers of the given
**              lower layer control block
**
** Returns      void.
**
*******************************************************************************/
void obx_free_buf(tOBEX_LL_CB *p_ll_cb)
{
    void        *p_pkt;


    if (p_ll_cb->comm.p_send_fn == (tOBEX_SEND_FN *)obx_rfc_snd_msg)
    {
        if (p_ll_cb->port.p_rxmsg)
        {
            OBEX_TRACE_WARNING0("obx_free_buf release p_rxmsg\n");
            GKI_freebuf(p_ll_cb->port.p_rxmsg);
            p_ll_cb->port.p_rxmsg = 0;
        }
    }

    if (!GKI_queue_is_empty(&p_ll_cb->comm.rx_q))
    {
        while((p_pkt = (void *)GKI_dequeue (&p_ll_cb->comm.rx_q)) != NULL)
        {
            OBEX_TRACE_WARNING0("obx_free_buf release rx_q\n");
            GKI_freebuf(p_pkt);
        }
    }

    if (p_ll_cb->comm.p_txmsg)
    {
        OBEX_TRACE_WARNING0("obx_free_buf release p_txmsg\n");
        GKI_freebuf(p_ll_cb->comm.p_txmsg);
        p_ll_cb->comm.p_txmsg = 0;
    }
}

/*******************************************************************************
**
** Function     obx_flow_control
**
** Description  If we had flowed control the peer, enable the data path now
**
** Returns      void.
**
*******************************************************************************/
void obx_flow_control(tOBEX_COMM_CB *p_comm)
{
    OBEX_TRACE_DEBUG1 ("obx_flow_control stopped:%d\n", p_comm->stopped );
    if (p_comm->stopped)
    {
        if (p_comm->p_send_fn == (tOBEX_SEND_FN *)obx_rfc_snd_msg)
        {
            wiced_bt_rfcomm_flow_control(p_comm->id, TRUE);
        }
        else
        {
            L2CA_FlowControl(p_comm->id, TRUE);
        }
        p_comm->stopped = FALSE;
    }
}

/*****************************************************************************
* Function: obx_read_triplet
* Purpose:  Read the application parameters with the given tag
*****************************************************************************/
UINT8 obx_read_triplet(tOBEX_TRIPLET *p_trip, UINT8 num_trip, UINT8 tag)
{
    UINT8   xx = 0;

    for (xx=0; xx<num_trip; xx++, p_trip++)
    {
        if (p_trip->tag == tag)
        {
            break;
        }
    }

    return xx;
}
/*****************************************************************************
* Function: obx_read_obj_offset
* Purpose:  Read the application parameters with the object offset tag
*****************************************************************************/
UINT32 obx_read_obj_offset(tOBEX_TRIPLET *p_trip, UINT8 num_trip)
{
    UINT32  obj_offset = 0;
    UINT8   ind;
    UINT8   *p = NULL, *pe;
    UINT8   extra = 0, xx;

    ind = obx_read_triplet(p_trip, num_trip, OBEX_TAG_SESS_PARAM_OBJ_OFF);
    if (ind != num_trip)
    {
        if (p_trip[ind].len == 4)
        {
            p = p_trip[ind].p_array;
        }
        else if (p_trip[ind].len > 4)
        {
            extra = p_trip[ind].len - 4;
        }

        if (extra)
        {
            /* the TLV is bigger than 4 bytes
             * if the MSBs are all 0, we can still handle it with UINT32 */
            pe = p_trip[ind].p_array;
            for (xx=0; xx<extra; xx++)
            {
                if (*pe == 0)
                    pe++;
                else
                    break;
            }
            if (xx == extra)
                p = pe;
        }

        if (p)
            BE_STREAM_TO_UINT32(obj_offset, p);
    }

    return obj_offset;
}

/*******************************************************************************
**
** Function         obxu_dump_hex
**
** Description      This function dumps hex data
**
*/
#if (BT_USE_TRACES == TRUE)
void obxu_dump_hex (UINT8 *p, char *p_title, UINT16 len)
{
    UINT16  xx, yy;
    char    buff1[100], buff2[20];
#if 0
    if (p_title)
        OBEX_TRACE_DEBUG1 ("%s:\n", p_title);

    memset (buff2, ' ', 16);
    buff2[16] = 0;

    yy = sprintf (buff1, "%04x: ", 0);
    for (xx = 0; xx < len; xx++)
    {
        if ( (xx) && ((xx & 15) == 0) )
        {
            OBEX_TRACE_DEBUG2 ("    %s  %s\n", buff1, buff2);
            yy = sprintf(buff1, "%04x: ", xx);
            memset (buff2, ' ', 16);
        }
        yy += sprintf (&buff1[yy], "%02x ", *p);

        if ((*p >= ' ') && (*p <= 'z'))
            buff2[xx & 15] = *p;
        else
            buff2[xx & 15] = '.';

        p++;
    }

    /* Pad out the remainder */
    for ( ; ; xx++)
    {
        if ((xx & 15) == 0)
	    {
            OBEX_TRACE_DEBUG2 ("    %s  %s\n", buff1, buff2);
	        break;
        }
        yy += sprintf (&buff1[yy], "   ");
    }
#endif
}
#endif

/**
 * Function     wiced_bt_obex_get_peer_addr
 *
 *              This function is called to get the Bluetooth address of the
 *              connected device
 *
 *  @param[in]   handle   : OBEX server handle
 *  @param[out]  bd_addr  : Remote BD address
 *
 *  @return      L2CAP channel ID
 *
 */
uint16_t wiced_bt_obex_get_peer_addr(wiced_bt_obex_handle_t handle, wiced_bt_device_address_t bd_addr)
{
    UINT16      lcid = 0;
    tOBEX_SR_SESS_CB *p_scb;
    tOBEX_CL_CB  *p_cb;

    if (handle & OBEX_CL_HANDLE_MASK)
    {
        p_cb = obx_cl_get_cb(handle);
        if (p_cb)
        {
            if (p_cb->ll_cb.comm.p_send_fn == (tOBEX_SEND_FN *)obx_rfc_snd_msg)
            {
                wiced_bt_rfcomm_check_connection(p_cb->ll_cb.port.port_handle, p_cb->peer_addr, &lcid);
                memcpy (bd_addr, p_cb->peer_addr, BD_ADDR_LEN);
                lcid = p_cb->ll_cb.comm.id;
            }
            else if (p_cb->ll_cb.comm.id)
            {
                /* GetPeerAddr for l2c */
                memcpy (bd_addr, p_cb->peer_addr, BD_ADDR_LEN);
                lcid = p_cb->ll_cb.comm.id;
            }
        }
    }

    if ((handle & OBEX_CL_HANDLE_MASK) == 0)
    {
        p_scb = obx_sr_get_scb(handle);
        if (p_scb)
        {
            if (p_scb->ll_cb.comm.p_send_fn == (tOBEX_SEND_FN *)obx_rfc_snd_msg)
            {
                wiced_bt_rfcomm_check_connection(p_scb->ll_cb.port.port_handle, bd_addr, &lcid);
            }
            else if (p_scb->ll_cb.comm.id)
            {
                /* GetPeerAddr for l2c */
                memcpy (bd_addr, p_scb->peer_addr, BD_ADDR_LEN);
                lcid = p_scb->ll_cb.comm.id;
            }
        }
    }

    return lcid;
}

wiced_bool_t wiced_bt_obex_send_pkt_allowed (wiced_bt_obex_handle_t handle)
{
    tOBEX_CL_CB  *p_cb = obx_cl_get_cb(handle);
    if (p_cb && p_cb->srm)
    {
        if (p_cb->srm & OBEX_SRM_WAIT)
            return WICED_FALSE;
        return WICED_TRUE;
    }
    return WICED_TRUE;
}
