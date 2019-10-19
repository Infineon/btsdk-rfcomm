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
**  Name:         obx_sapi.c
**
**  File:         OBEX Server Application Programming Interface functions
**
**
*****************************************************************************/
#include <string.h>

#include "obx_int.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_rfcomm.h"

/**
 * Function     wiced_bt_obex_start_server
 *
 *              Start OBEX server
 *
 *  @param[in]   p_params : Parameters for starting server
 *  @param[out]  p_handle : Pointer to return OBEX server handle
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_start_server(wiced_bt_obex_start_params_t *p_params, wiced_bt_obex_handle_t *p_handle)
{
    tOBEX_SR_CB *p_cb = NULL;
    wiced_bt_obex_status_t status = OBEX_NO_RESOURCES;
    tOBEX_SR_SESS_CB *p_scb;
    tOBEX_HANDLE obx_handle;
    UINT8       size;

    WC_ASSERT(p_params);
    WC_ASSERT(p_params->p_cback);
    WC_ASSERT(p_handle);

    if (p_params->max_sessions > OBEX_MAX_SR_SESSION)
    {
        OBEX_TRACE_ERROR2("OBEX_StartServer bad max_sessions:%d (1-%d)",
            p_params->max_sessions, OBEX_MAX_SR_SESSION);
        return OBEX_BAD_PARAMS;
    }

    if (p_params->scn == 0 && L2C_INVALID_PSM(p_params->psm))
    {
        OBEX_TRACE_ERROR2("OBEX_StartServer bad scn:%d and psm:0x%x", p_params->scn, p_params->psm);
        return OBEX_BAD_PARAMS;
    }

    if (p_params->max_sessions == 0)
        p_params->max_sessions = 1;

    /* allocate a server control block */
    obx_handle = obx_sr_alloc_cb(p_params);
    if (obx_handle)
        p_cb = &obx_cb.server[obx_handle-1];

    if (p_cb != NULL)
    {
        p_scb = &obx_cb.sr_sess[p_cb->sess[0]-1];
        if (p_cb->scn)
        {
            /* open an RFCOMM port to listen for incoming messages */
            /* allocate the port for the first session now. The others will be allocated when needed */
            status = obx_open_port(&p_scb->ll_cb.port, PB_BT_BD_ANY, p_cb->scn);
        }
        else
        {
            status      = OBEX_SUCCESS;
        }

        if (status == OBEX_SUCCESS)
        {
            /* if everything is OK, save the other parameters for this server */
            memset(p_cb->target.target, 0, OBEX_MAX_TARGET_LEN);
            if (p_params->p_target)
            {
                if (p_params->p_target->len)
                {
                    /* OBX handles who, connection ID headers */
                    p_cb->target.len = p_params->p_target->len;
                    memcpy(p_cb->target.target, p_params->p_target->target, p_params->p_target->len);
                }
                else
                {
                    /* the regular default server */
                    /* the user handles target, who headers.
                     * OBX handles connection ID header */
                    p_cb->target.len = OBEX_DEFAULT_TARGET_LEN;
                }
            }
            else
            {
                /* the one and only default server */
                /* no target, who, connection id headers for this case */
                p_cb->target.len = 0;
            }
            OBEX_TRACE_DEBUG3("OBEX_StartServer target len:%d, authenticate:%d handle:0x%x",
                p_cb->target.len, p_params->authenticate, p_scb->ll_cb.port.handle);
            p_cb->p_cback       = p_params->p_cback;
            p_scb->state         = OBEX_SS_NOT_CONNECTED;

            /* give the handle to application */
            *p_handle = p_scb->ll_cb.port.handle;
        }
        else
        {
            /* otherwise, free the control block */
            obx_sr_free_cb(obx_handle);
        }
    }

    return status;
}


/**
 * Function     wiced_bt_obex_stop_server
 *
 *              Stop OBEX server
 *
 *  @param[in]   handle : OBEX server handle
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_stop_server(wiced_bt_obex_handle_t handle)
{
    wiced_bt_obex_status_t status = OBEX_SUCCESS;
    tOBEX_SR_CB *p_cb = obx_sr_get_cb(handle);
    tOBEX_SR_SESS_CB *p_scb;
    int xx;
    tOBEX_SPND_CB    *p_spndcb;

    if (p_cb)
    {
        /* Process suspended session if necessary */
        if (p_cb->p_suspend)
        {
            for (xx=0, p_spndcb=p_cb->p_suspend; xx<p_cb->max_suspend; xx++, p_spndcb++)
            {
                if (p_spndcb->state)
                {
                    btu_stop_timer (&p_spndcb->stle);
                }
            }
            GKI_freebuf (p_cb->p_suspend);
        }

        for (xx=0; xx < p_cb->num_sess && p_cb->sess[xx]; xx ++)
        {
            p_scb = &obx_cb.sr_sess[p_cb->sess[xx]-1];
            if (p_scb->ll_cb.comm.id)
            {
                wiced_bt_obex_send_response(handle, OBEX_REQ_DISCONNECT, OBEX_RSP_SERVICE_UNAVL, NULL);
                if (p_scb->ll_cb.comm.p_send_fn == (tOBEX_SEND_FN *)obx_rfc_snd_msg)
                    wiced_bt_rfcomm_remove_connection(p_scb->ll_cb.port.port_handle,WICED_FALSE);
            }
        }

#ifdef OBEX_LIB_L2CAP_INCLUDED
        if (p_cb->psm)
            L2CA_DEREGISTER (p_cb->psm);
#endif

        obx_sr_free_cb (handle);
    }
    else
        status = OBEX_BAD_HANDLE;
    return status;
}

/*******************************************************************************
**
** Function     OBEX_AddSuspendedSession
**
** Description  This function is to add the session information for a previously
**				suspended reliable session to the server control block
**
** Returns      OBEX_SUCCESS, if successful.
**              OBEX_BAD_HANDLE, if the handle is not valid.
**
*******************************************************************************/
tOBEX_STATUS OBEX_AddSuspendedSession(tOBEX_HANDLE handle, BD_ADDR peer_addr, UINT8 *p_sess_info,
                                    UINT32 timeout, UINT8 ssn, UINT32 offset)
{
    tOBEX_STATUS status = OBEX_SUCCESS;
    tOBEX_SR_CB  *p_cb = obx_sr_get_cb(handle);
    UINT16      size;
    UINT8       xx;
    tOBEX_SPND_CB    *p_spndcb;
    INT32       ticks = 0x7FFFFFFF, remain_ticks;
    BOOLEAN     added = FALSE;
    UINT8       saved_xx = 0;

    OBEX_TRACE_DEBUG2("OBEX_AddSuspendedSession BDA: %06x%06x",
                    (peer_addr[0]<<16)+(peer_addr[1]<<8)+peer_addr[2],
                    (peer_addr[3]<<16)+(peer_addr[4]<<8)+peer_addr[5]);
    if (p_cb && p_sess_info && p_cb->max_suspend)
    {
        if (p_cb->p_suspend == NULL)
        {
            size = p_cb->max_suspend * sizeof (tOBEX_SPND_CB);
            p_cb->p_suspend = (tOBEX_SPND_CB *)GKI_getbuf( size);
            memset (p_cb->p_suspend, 0, size);
        }

        if (p_cb->p_suspend)
        {
            for (xx=0, p_spndcb=p_cb->p_suspend; xx<p_cb->max_suspend; xx++, p_spndcb++)
            {
                OBEX_TRACE_DEBUG4("[%d] state: %d, ssn:%d BDA: %08x", xx, p_spndcb->state, p_spndcb->ssn,
                   (p_spndcb->peer_addr[2]<<24)+(p_spndcb->peer_addr[3]<<16)+(p_spndcb->peer_addr[4]<<8)+p_spndcb->peer_addr[5]);
                if (p_spndcb->state == OBEX_SS_NULL || memcmp(p_spndcb->peer_addr, peer_addr, BD_ADDR_LEN) == 0)
                {
                    added = TRUE;
                    break;
                }
                else if (p_spndcb->state != OBEX_SS_NULL && ticks)
                {
                    if (p_spndcb->stle.param == 0)
                    {
                        /* this entry has infinite timeout; just use it */
                        ticks = 0;
                        saved_xx = xx;
                        OBEX_TRACE_DEBUG1("[%d] infinite timeout", xx );
                    }
                    /* find the entry the expires in the shortest time  */
                    else
                    {
                        remain_ticks = 0;//btu_remaining_time(&p_spndcb->stle);
                        OBEX_TRACE_DEBUG2("[%d] remain_ticks: %d", xx, remain_ticks );
                        if (remain_ticks < ticks)
                        {
                            ticks = remain_ticks;
                            saved_xx = xx;
                        }
                    }
                }
            }

            if (!added)
            {
                /* if cannot use an empty/or reuse an existing entry, use the one expires soon */
                added = TRUE;
                xx = saved_xx; /* this is for debug trace; don't optimize */
                p_spndcb = &p_cb->p_suspend[xx];
                OBEX_TRACE_DEBUG1("reuse entry [%d]", xx );
            }

            if (added)
            {
		        memcpy (p_spndcb->sess_info, p_sess_info, OBEX_SESSION_INFO_SIZE);
                p_spndcb->state = p_sess_info[OBEX_SESSION_INFO_ST_IDX];
                p_spndcb->ssn = ssn;
                p_spndcb->offset = offset;
                OBEX_TRACE_DEBUG6("[%d] timeout: %d state:%d ssn:%d offset:%d, BDA: %08x",
                    xx, timeout, p_spndcb->state, ssn, offset,
                   (peer_addr[2]<<24)+(peer_addr[3]<<16)+(peer_addr[4]<<8)+peer_addr[5]);
                memcpy(p_spndcb->peer_addr, peer_addr, BD_ADDR_LEN);
                if (timeout != OBEX_INFINITE_TIMEOUT)
                {
                    p_spndcb->stle.param = (TIMER_PARAM_TYPE)p_spndcb;
                    wiced_init_timer(&p_spndcb->stle.wiced_timer, obx_sr_timeout, (uint32_t)&p_spndcb->stle, WICED_SECONDS_TIMER);
                    wiced_start_timer(&p_spndcb->stle.wiced_timer, timeout);
                    OBEX_TRACE_DEBUG2("timeout: %d ticks:%d", timeout, p_spndcb->stle.ticks);
                }
                else
                    p_spndcb->stle.param = 0;
            }
        }
    }
    else
        status = OBEX_BAD_HANDLE;
    return status;
}

/**
 * Function     wiced_bt_obex_send_rsp
 *
 *              Send response to a Request from an OBEX client.
 *
 *  @param[in]   handle   : OBEX server handle
 *  @param[in]   req_code : Request code
 *  @param[in]   rsp_code : Response code
 *  @param[in]   p_pkt    : Response packet
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_send_response(wiced_bt_obex_handle_t handle, wiced_bt_obex_req_code_t req_code, wiced_bt_obex_rsp_code_t rsp_code, uint8_t *p_pkt)
{
    wiced_bt_obex_status_t status = OBEX_SUCCESS;

    switch (req_code)
    {
    case OBEX_REQ_CONNECT:
        {
            tOBEX_SR_SESS_CB *p_scb = obx_sr_get_scb(handle);
            tOBEX_SR_CB      *p_cb = obx_sr_get_cb(handle);

            if (p_scb)
            {
                obx_ssm_event(p_scb, OBEX_CONNECT_CFM_SEVT, obx_conn_rsp(p_cb, p_scb, rsp_code, (BT_HDR *)p_pkt));
            }
            else
            {
                OBEX_TRACE_DEBUG1("OBEX_ConnectRsp Bad Handle: 0x%x", handle);
                status = OBEX_BAD_HANDLE;
            }
        }
        break;

    case OBEX_REQ_DISCONNECT:
        status = obx_prepend_rsp_msg(handle, OBEX_DISCNT_CFM_SEVT, rsp_code, (BT_HDR *)p_pkt);
        break;

    case OBEX_REQ_PUT:
        status = obx_prepend_rsp_msg(handle, OBEX_PUT_CFM_SEVT, rsp_code, (BT_HDR *)p_pkt);
        break;

    case OBEX_REQ_GET:
        status = obx_prepend_rsp_msg(handle, OBEX_GET_CFM_SEVT, rsp_code, (BT_HDR *)p_pkt);
        break;

    case OBEX_REQ_SETPATH:
        status = obx_prepend_rsp_msg(handle, OBEX_SETPATH_CFM_SEVT, rsp_code, (BT_HDR *)p_pkt);
        break;

    case OBEX_REQ_ACTION:
        status = obx_prepend_rsp_msg(handle, OBEX_ACTION_CFM_SEVT, rsp_code, (BT_HDR *)p_pkt);
        break;

    case OBEX_REQ_ABORT:
        status = obx_prepend_rsp_msg(handle, OBEX_ABORT_CFM_SEVT, rsp_code, (BT_HDR *)p_pkt);
        break;

    default:
        status = OBEX_BAD_PARAMS;
        break;
    }

    return status;
}

#ifdef OBEX_LIB_SESSION_SUPPORTED
/**
 * Function     wiced_bt_obex_session_rsp
 *
 *              Respond to a request to create a reliable session.
 *
 *  @param[in]   handle   : OBEX server handle
 *  @param[in]   rsp_code : Response code
 *  @param[in]   ssn      : Session sequence number
 *  @param[in]   offset   : Data offset
 *  @param[in]   p_pkt    : Response packet
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_session_response(wiced_bt_obex_handle_t handle, wiced_bt_obex_rsp_code_t rsp_code, uint8_t ssn, uint32_t offset, uint8_t *p_pkt)
{
    wiced_bt_obex_status_t status = OBEX_SUCCESS;
    tOBEX_SR_SESS_CB *p_scb = obx_sr_get_scb(handle);
    UINT8       *p;
    BT_HDR      *p_rsp = NULL;
    UINT8       data[12];
    UINT32      timeout;
    tOBEX_SESS_ST    old_sess_st;

    OBEX_TRACE_API0("OBEX_SessionRsp");
    if (p_scb)
    {
        old_sess_st = p_scb->sess_st;
        p_rsp   = (BT_HDR *)wiced_bt_obex_header_init(p_scb->handle, OBEX_MIN_MTU);
        if (p_rsp)
        {
            p = (UINT8 *) (p_rsp + 1) + p_rsp->offset;
            /* response packet always has the final bit set */
            *p++ = (rsp_code | OBEX_FINAL);
            p_rsp->len = 3;
            p = data;
            if (rsp_code == OBEX_RSP_OK)
            {
                switch (p_scb->sess_st)
                {
                case OBEX_SESS_CREATE:
                case OBEX_SESS_RESUME:
                    GKI_freebuf (p_rsp);
                    if (p_pkt)
                        GKI_freebuf (p_pkt);
                    OBEX_TRACE_DEBUG0("OBEX_SessionRsp do not need to be called for CREATE and RESUME");
                    return OBEX_SUCCESS;
                case OBEX_SESS_SUSPEND:
                    p_scb->sess_st = OBEX_SESS_SUSPENDED;
                    p = &p_scb->sess_info[OBEX_SESSION_INFO_TO_IDX];
                    BE_STREAM_TO_UINT32(timeout, p);
                    OBEX_AddSuspendedSession(p_scb->handle, p_scb->peer_addr, p_scb->sess_info, timeout, ssn, offset);
                    break;
                case OBEX_SESS_CLOSE:
                    p_scb->sess_st = OBEX_SESS_NONE;
                    break;
                }

                if (p_pkt)
                {
                    p = (UINT8 *) (p_rsp + 1) + p_rsp->offset + p_rsp->len;
                    memcpy (p, ((UINT8 *) (p_pkt + 1) + ((BT_HDR*)p_pkt)->offset), ((BT_HDR*)p_pkt)->len);
                    p_rsp->len += ((BT_HDR*)p_pkt)->len;
                }
            }
            p = (UINT8 *) (p_rsp + 1) + p_rsp->offset + 1;
            UINT16_TO_BE_STREAM(p, p_rsp->len);

            p_rsp->event = OBEX_SESSION_CFM_SEVT + 1;
        }
        OBEX_TRACE_DEBUG3("Rsp sess_st:%d->%d status:%d", old_sess_st, p_scb->sess_st, status);

        obx_ssm_event(p_scb, OBEX_SESSION_CFM_SEVT, p_rsp);
        /* clear the "previous" session state as required by earlier comment */
        p_scb->param.sess.sess_st = 0;
    }
    else
        status = OBEX_BAD_HANDLE;

    if (p_pkt)
    {
        GKI_freebuf (p_pkt);
    }
    return status;
}
#endif

/*******************************************************************************
**
** Function     obx_prepend_rsp_msg
**
** Description  This function is called to add response code and connection ID
**              to the given OBEX message
** Returns      OBEX_SUCCESS, if successful.
**              OBEX_BAD_HANDLE, if the handle is not valid.
**
*******************************************************************************/
tOBEX_STATUS obx_prepend_rsp_msg(tOBEX_HANDLE handle, tOBEX_SR_EVENT event, UINT8 rsp_code, BT_HDR *p_pkt)
{
    tOBEX_STATUS status = OBEX_SUCCESS;
    tOBEX_SR_SESS_CB *p_scb = obx_sr_get_scb(handle);
    UINT8       msg[OBEX_HDR_OFFSET];
    UINT8       *p = msg;
    BOOLEAN     skip_clear = FALSE;

    if (p_scb)
    {
        /* response packets always have the final bit set */
        *p++ = (rsp_code | OBEX_FINAL);
        p += OBEX_PKT_LEN_SIZE;

        /* add session sequence number, if session is active */
        if (p_scb->sess_st == OBEX_SESS_ACTIVE || p_scb->sess_st == OBEX_SESS_SUSPENDING)
        {
            *p++ = OBEX_HI_SESSION_SN;
            *p++ = (p_scb->ssn+1);
        }

        if (event == OBEX_DISCNT_CFM_SEVT)
            p_scb->conn_id = 0;

        if (p_scb->srm & OBEX_SRM_REQING)
        {
            p_scb->srm &= ~OBEX_SRM_REQING;

            if (rsp_code == OBEX_RSP_CONTINUE)
            {
                if (event == OBEX_PUT_CFM_SEVT)
                    p_scb->srm |= OBEX_SRM_NEXT;
            }

            if (rsp_code == OBEX_RSP_CONTINUE)
            {
                /* Set SRM state to engage if no need to wait from peer */
                if (p_scb->srmp && ((p_scb->srmp & OBEX_SRMP_WAIT) == OBEX_SRMP_WAIT))
                {
                    p_scb->srmp |= OBEX_SRMP_WAITING_CLEARED;
                    skip_clear = TRUE;
                }
                else
                    p_scb->srm |= OBEX_SRM_ENGAGE;
            }


            /* Respond with SRM Enable header even if this
             *first packet is the final packet of the transaction
             */
            if (rsp_code == OBEX_RSP_CONTINUE || rsp_code == OBEX_RSP_OK)
            {
                *p++ = OBEX_HI_SRM;
                *p++ = OBEX_HV_SRM_ENABLE;
            }
        }

        /* If waiting condition due to SRMP header has been cleared, engage SRM */
        if (skip_clear == FALSE && (p_scb->srmp & OBEX_SRMP_WAITING_CLEARED) &&
            !((p_scb->srmp & OBEX_SRMP_WAIT) == OBEX_SRMP_WAIT))
        {
            OBEX_TRACE_EVENT0("SRM is engaged after peer cleared wait flag");
            p_scb->srmp &= ~OBEX_SRMP_WAITING_CLEARED;
            p_scb->srm |= OBEX_SRM_ENGAGE;
        }

        p_pkt = obx_sr_prepend_msg(p_pkt, msg, (UINT16)(p - msg) );
        /* this event code needs to be set up properly for flow control reasons */
        p_pkt->event    = event+1;
        obx_ssm_event(p_scb, event, p_pkt);
    }
    else
        status = OBEX_BAD_HANDLE;

    return status;
}
