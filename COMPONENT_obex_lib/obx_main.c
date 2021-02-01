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
**  Name:         obx_main.c
**
**  File:         OBEX common API and interface to other Bluetooth modules
**
**
*****************************************************************************/
#include <string.h>

#include "obx_int.h"
#include "wiced_bt_l2c.h"

#if (defined (BT_USE_TRACES) && BT_USE_TRACES == TRUE)
const char * const obx_cl_state_name [] =
{
    "NULL",
    "NOT_CONN",
    "SESS_RS",
    "CONN_RS",
    "CONN",
    "DISCNT_RS",
    "SETPATH_RS",
    "ACT_RS",
    "ABORT_RS",
    "PUT_RS",
    "GET_RS",
    "PUT",
    "GET",
    "PUT_S",
    "GET_S",
    "PART",
    "Unknown"
};

const char * const obx_cl_event_name [] =
{
    "CONN_R",
    "SESS_R",
    "DISCNT_R",
    "PUT_R",
    "GET_R",
    "SETPATH_R",
    "ACT_R",
    "ABORT_R",
    "OK_C",
    "CONT_C",
    "FAIL_C",
    "PORT_CLS",
    "TX_EMPTY",
    "FCS_SET",
    "STATE",
    "TIMEOUT",
    "Unknown"
};

const char * const obx_sr_state_name [] =
{
    "NULL",
    "NOT_CONN",
    "SESS_I",
    "CONN_I",
    "CONN",
    "DISCNT_I",
    "SETPATH_I",
    "ACT_I",
    "ABORT_I",
    "PUT_I",
    "GET_I",
    "PUT",
    "GET",
    "PUT_S",
    "GET_S",
    "PART",
    "WAIT_CLS",
    "Unknown"
};

const char * const obx_sr_event_name [] =
{
    "CONN_R",
    "SESS_R",
    "DISCNT_R",
    "PUT_R",
    "GET_R",
    "SETPATH_R",
    "ACT_R",
    "ABORT_R",
    "CONN_C",
    "SESS_C",
    "DISCNT_C",
    "PUT_C",
    "GET_C",
    "SETPATH_C",
    "ACT_C",
    "ABORT_C",
    "PORT_CLS",
    "FCS_SET",
    "STATE",
    "TIMEOUT",
    "BAD_REQ",
    "TX_EMPTY",
    "Unknown"
};
#endif

#if OBEX_DYNAMIC_MEMORY == FALSE
tOBEX_CB obx_cb;
#endif

const BD_ADDR PB_BT_BD_ANY = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };


#if (defined (BT_USE_TRACES) && BT_USE_TRACES == TRUE)
/*******************************************************************************
** Function     obx_cl_get_state_name
** Returns      The client state name.
*******************************************************************************/
const char *obx_cl_get_state_name(tOBEX_CL_STATE state)
{
    const char * p_str = obx_cl_state_name[OBEX_CS_MAX];
    if (state < OBEX_CS_MAX)
    {
        p_str = obx_cl_state_name[state];
    }
    return p_str;
}

/*******************************************************************************
** Function     obx_cl_get_event_name
** Returns      The client event name.
*******************************************************************************/
const char *obx_cl_get_event_name(tOBEX_CL_EVENT event)
{
    const char * p_str = obx_cl_event_name[OBEX_MAX_CEVT];
    if (event < OBEX_MAX_CEVT)
    {
        p_str = obx_cl_event_name[event];
    }
    return p_str;
}

/*******************************************************************************
** Function     obx_sr_get_state_name
** Returns      The server state name.
*******************************************************************************/
const char *obx_sr_get_state_name(tOBEX_SR_STATE state)
{
    const char * p_str = obx_sr_state_name[OBEX_SS_MAX];
    if (state < OBEX_SS_MAX)
    {
        p_str = obx_sr_state_name[state];
    }
    return p_str;
}

/*******************************************************************************
** Function     obx_sr_get_event_name
** Returns      The server event name.
*******************************************************************************/
const char *obx_sr_get_event_name(tOBEX_SR_EVENT event)
{
    const char * p_str = obx_sr_event_name[OBEX_MAX_SEVT];
    if (event < OBEX_MAX_SEVT)
    {
        p_str = obx_sr_event_name[event];
    }
    return p_str;
}
#endif

/*******************************************************************************
** Function     obx_start_timer
** Description  start BTU timer.
*******************************************************************************/
void obx_start_timer(tOBEX_COMM_CB *p_pcb)
{
    UINT32 timeout = obx_cb.timeout_val;

    if (timeout)
    {
        if (p_pcb->handle & OBEX_CL_HANDLE_MASK)
        {
            if (p_pcb->p_txmsg && p_pcb->p_txmsg->event == OBEX_DISCONNECT_REQ_EVT)
            {
                timeout = OBEX_DISC_TOUT_VALUE;
            }
            wiced_init_timer(&p_pcb->tle.wiced_timer, obx_cl_timeout, (uint32_t)&p_pcb->tle, WICED_SECONDS_TIMER);
        }
        else
        {
            if (p_pcb->p_txmsg && p_pcb->p_txmsg->event == (OBEX_DISCNT_CFM_SEVT + 1))
            {
                timeout = OBEX_DISC_TOUT_VALUE;
            }
            wiced_init_timer(&p_pcb->tle.wiced_timer, obx_sr_timeout, (uint32_t)&p_pcb->tle, WICED_SECONDS_TIMER);
        }
        wiced_start_timer(&p_pcb->tle.wiced_timer, timeout);
    }
    OBEX_TRACE_DEBUG2("obx_start_timer val:%d, p_tle:0x%x\n", timeout, &p_pcb->tle);
}

/*******************************************************************************
** Function     obx_stop_timer
** Description  Stop BTU timer
*******************************************************************************/
void obx_stop_timer(TIMER_LIST_ENT *p_tle)
{
    wiced_stop_timer(&p_tle->wiced_timer);
    OBEX_TRACE_DEBUG1("obx_stop_timer p_tle:0x%x\n", p_tle);
}

/*******************************************************************************
** Function     obx_cl_timeout
** Description  Get client control block from timer param. Start BTU timer again.
**              Call application callback routine with OBEX_TIMEOUT_EVT event.
*******************************************************************************/
void obx_cl_timeout(uint32_t cb_params)
{
    TIMER_LIST_ENT  *p_tle = (TIMER_LIST_ENT *)cb_params;
    tOBEX_CL_CB      *p_cb = (tOBEX_CL_CB *) p_tle->param;
    tOBEX_EVT_PARAM  evtp;
    tOBEX_HANDLE     handle = p_cb->ll_cb.comm.handle;
    tOBEX_CL_CBACK   *p_cback = p_cb->p_cback;

    OBEX_TRACE_DEBUG0("obx_cl_timeout\n");
    memset(&evtp, 0, sizeof(tOBEX_EVT_PARAM));
    if (obx_cb.timeout_val)
        wiced_start_timer(&p_tle->wiced_timer, obx_cb.timeout_val);
    else
        wiced_stop_timer(&p_tle->wiced_timer);
    obx_csm_event(p_cb, OBEX_TIMEOUT_CEVT, NULL);
    (*p_cback) (handle, OBEX_TIMEOUT_EVT, OBEX_RSP_DEFAULT, evtp, NULL);
}

/*******************************************************************************
**
** Function     obx_cl_alloc_cb
**
** Description  allocate a client control block.
**
*******************************************************************************/
tOBEX_CL_CB *obx_cl_alloc_cb(void)
{
    int         xx, yy;
    tOBEX_CL_CB  *p_cb = NULL;

    /* allocate a client control block */
    for (xx=0, yy=obx_cb.next_ind; xx<obx_cb.num_client; xx++, yy++)
    {
        if (yy >= obx_cb.num_client)
            yy = 0;
        p_cb = &obx_cb.client[yy];

        if (p_cb->ll_cb.comm.handle == OBEX_HANDLE_NULL)
        {
            p_cb->ll_cb.comm.handle   = OBEX_CL_HANDLE_MASK | (yy + 1);
//            obx_cb.next_ind     = yy+1; /* it will be adjusted, so we do not need to check the range now */
            OBEX_TRACE_DEBUG3("obx_cl_alloc_cb obx handle:0x%x, yy:%d, next: %d\n",
                p_cb->ll_cb.comm.handle, yy, obx_cb.next_ind);
            p_cb->ll_cb.comm.tx_mtu = 0;
            p_cb->conn_id     = OBEX_INVALID_CONN_ID;

            p_cb->ll_cb.comm.tle.param = (TIMER_PARAM_TYPE)p_cb;
            p_cb->psm         = 0;
            p_cb->srm         = 0;
            break;
        }
    }

    if (xx == obx_cb.num_client)
        p_cb = NULL;
    return p_cb;
}

/*******************************************************************************
**
** Function     obx_cl_get_cb
**
** Description  Returns the pointer to the client control block with given handle.
**
*******************************************************************************/
tOBEX_CL_CB *obx_cl_get_cb(tOBEX_HANDLE handle)
{
    tOBEX_CL_CB *p_cb = NULL;
    UINT8       ind  = (handle & OBEX_CL_CB_IND_MASK);

    if (handle & OBEX_CL_HANDLE_MASK)
    {
        if (ind <= obx_cb.num_client && ind > 0)
        {
            if (obx_cb.client[--ind].ll_cb.comm.handle == handle)
                p_cb = &obx_cb.client[ind];
        }
    }

    return p_cb;
}

/*******************************************************************************
**
** Function     obx_cl_get_suspended_cb
**
** Description  Returns the pointer to the client control block with given handle.
**
*******************************************************************************/
tOBEX_CL_CB *obx_cl_get_suspended_cb(tOBEX_HANDLE *p_handle, UINT8 *p_session_info)
{
    tOBEX_HANDLE handle = *p_handle;
    tOBEX_CL_CB *p_cb = NULL;
    tOBEX_CL_CB *p_ccb = NULL;
    UINT8       ind  = (handle & OBEX_CL_CB_IND_MASK);

    OBEX_TRACE_DEBUG1("obx_cl_get_suspended_cb handle: 0x%x\n", handle);
    if (handle & OBEX_CL_HANDLE_MASK)
    {
        if (ind <= obx_cb.num_client && ind > 0)
        {
            if (obx_cb.client[--ind].ll_cb.comm.handle == handle)
            {
                p_ccb = &obx_cb.client[ind];
                if (p_ccb->sess_st == OBEX_SESS_SUSPENDED &&
                    ((p_session_info == p_ccb->sess_info) || (memcmp(p_session_info, p_ccb->sess_info, OBEX_SESSION_INFO_SIZE) == 0)))
                {
                    OBEX_TRACE_DEBUG0("found a suspended session\n");
                    p_cb = p_ccb;
                }
            }
        }
    }
    if (p_cb == NULL) /* if the handle is NULL, assume the system was power cycled. attempt to allocate a control block */
    {
        p_cb = obx_cl_alloc_cb();
        if (p_cb)
        {
            *p_handle       = p_cb->ll_cb.comm.handle;
            p_cb->sess_st   = OBEX_SESS_SUSPENDED;
            memcpy(p_cb->sess_info, p_session_info, OBEX_SESSION_INFO_SIZE);
            OBEX_TRACE_DEBUG1("allocated a suspended session handle: 0x%x\n", *p_handle);
        }
    }

    return p_cb;
}

/*******************************************************************************
**
** Function     obx_cl_free_cb
**
** Description  Free the given client control block.
**
** Returns      void
**
*******************************************************************************/
void obx_cl_free_cb(tOBEX_CL_CB * p_cb)
{
    if (p_cb)
    {
        OBEX_TRACE_DEBUG2("obx_cl_free_cb id: %d, sess_st:%d\n", p_cb->ll_cb.comm.id, p_cb->sess_st);

        if (p_cb->ll_cb.comm.id>0)
        {
            if (p_cb->ll_cb.comm.p_send_fn == (tOBEX_SEND_FN *)obx_rfc_snd_msg)
                obx_cb.hdl_map[p_cb->ll_cb.port.port_handle - 1] = 0;
#ifdef OBEX_LIB_L2CAP_INCLUDED
            else
                obx_cb.l2c_map[p_cb->ll_cb.l2c.lcid - L2CAP_BASE_APPL_CID] = 0;
#endif
        }

        /* make sure the GKI buffers are freed */
        if (p_cb->p_next_req)
            GKI_freebuf(p_cb->p_next_req);

        if (p_cb->p_saved_req)
            GKI_freebuf(p_cb->p_saved_req);

        obx_free_buf (&p_cb->ll_cb);

#ifdef OBEX_LIB_L2CAP_INCLUDED
        if (p_cb->psm)
            L2CA_DEREGISTER (p_cb->psm);
#endif

        /* make sure the timer is stopped */
        obx_stop_timer(&p_cb->ll_cb.comm.tle);

        memset(p_cb, 0, sizeof(tOBEX_CL_CB) );
    }
}

/*******************************************************************************
** Function     obx_find_suspended_session
** Description  if p_triplet is NULL,
**                  check if there's still room for a reliable session
**              else check if the given session is still in the suspended list
*******************************************************************************/
tOBEX_SPND_CB *obx_find_suspended_session (tOBEX_SR_SESS_CB *p_scb, tOBEX_TRIPLET *p_triplet, UINT8 num)
{
    UINT8   ind;
    BOOLEAN found = FALSE;
    BOOLEAN infinite = FALSE;
    UINT8   *p, xx;
    tOBEX_SR_CB      *p_cb = obx_sr_get_cb(p_scb->handle);
    tOBEX_SPND_CB    *p_spndcb, *p_ret = NULL;

    OBEX_TRACE_DEBUG0("obx_find_suspended_session\n");
    if (p_triplet == NULL)
    {
        if (p_cb->p_suspend)
        {
            for (xx=0, p_spndcb=p_cb->p_suspend; xx<p_cb->max_suspend; xx++, p_spndcb++)
            {
                if ((p_spndcb->state == OBEX_SS_NULL) || (memcmp(p_spndcb->peer_addr, p_scb->peer_addr, BD_ADDR_LEN)==0))
                {
                    OBEX_TRACE_DEBUG3("[%d] state: %d, BDA: %08x\n", xx, p_spndcb->state,
                   (p_spndcb->peer_addr[2]<<24)+(p_spndcb->peer_addr[3]<<16)+(p_spndcb->peer_addr[4]<<8)+p_spndcb->peer_addr[5]);
                    /* this entry is not used yet or overwriting the entry with the same address */
                    found = TRUE;
                    break;
                }
                else if (p_spndcb->stle.param == 0)
                {
                    infinite = TRUE;
                }
            }
            OBEX_TRACE_DEBUG2("found: %d infinite:%d\n", found, infinite);
            if (found == FALSE)
                found = infinite;
        }
        else if (p_cb->max_suspend > 0)
        {
            found = TRUE;
        }
        p_ret = (tOBEX_SPND_CB *)p_scb;
    }
    else if (p_cb->p_suspend)
    {
        ind = obx_read_triplet(p_triplet, num, OBEX_TAG_SESS_PARAM_SESS_ID);
        if (ind < num && p_triplet[ind].len == OBEX_SESSION_ID_SIZE)
        {
            p = p_triplet[ind].p_array;
            for (xx=0, p_spndcb=p_cb->p_suspend; xx<p_cb->max_suspend; xx++, p_spndcb++)
            {
                OBEX_TRACE_DEBUG5("[%d] state: %d/%d, ssn:%d offset:x%x\n", xx, p_spndcb->state,
                    p_spndcb->sess_info[OBEX_SESSION_INFO_ST_IDX], p_spndcb->ssn, p_spndcb->offset);
                if ((p_spndcb->state != OBEX_SS_NULL) &&
                    (memcmp (p, p_spndcb->sess_info, OBEX_SESSION_ID_SIZE) == 0))
                {
                    obxu_dump_hex (p_spndcb->sess_info, "sess info", OBEX_SESSION_INFO_SIZE);
                    /* prepare p_scb to the proper state for resume */
                    p_ret = p_spndcb;
                    break;
                }
            }
        }
    }
    return p_ret;
}

/*******************************************************************************
** Function     obx_sr_sess_timeout
** Description  Get suspended session control block from timer param.
**              mark the suspended session as NULL
*******************************************************************************/
void obx_sr_sess_timeout(uint32_t cb_params)
{
    TIMER_LIST_ENT  *p_tle = (TIMER_LIST_ENT *)cb_params;
    tOBEX_SPND_CB    *p_spndcb = (tOBEX_SPND_CB *) p_tle->param;

    OBEX_TRACE_DEBUG0("obx_sr_sess_timeout\n");
    p_spndcb->state = OBEX_SS_NULL;
}

/*******************************************************************************
** Function     obx_sr_timeout
** Description  Get server control block from timer param. Start BTU timer again.
**              Call application callback routine with OBEX_TIMEOUT_EVT event.
*******************************************************************************/
void obx_sr_timeout(uint32_t cb_params)
{
    TIMER_LIST_ENT  *p_tle = (TIMER_LIST_ENT *)cb_params;
    tOBEX_SR_SESS_CB *p_scb = (tOBEX_SR_SESS_CB *) p_tle->param;
    tOBEX_EVT_PARAM  evtp;
    tOBEX_HANDLE     handle = p_scb->ll_cb.comm.handle;
    tOBEX_SR_CBACK   *p_cback;
    tOBEX_SR_CB      *p_cb;

    memset(&evtp, 0, sizeof(tOBEX_EVT_PARAM));
    if (obx_cb.timeout_val)
        wiced_start_timer(&p_tle->wiced_timer, obx_cb.timeout_val);
    else
        wiced_stop_timer(&p_tle->wiced_timer);
    p_cb = &obx_cb.server[p_scb->handle - 1];
    p_cback = p_cb->p_cback;
    obx_ssm_event(p_scb, OBEX_TIMEOUT_SEVT, NULL);
    (*p_cback) (handle, OBEX_TIMEOUT_EVT, evtp, NULL);
}

/*******************************************************************************
**
** Function     obx_sr_alloc_cb
**
** Description  allocate a server control block.
**
*******************************************************************************/
tOBEX_HANDLE obx_sr_alloc_cb(tOBEX_StartParams *p_params)
{
    int         xx, yy, zz;
    tOBEX_SR_CB  *p_cb = &obx_cb.server[0];
    tOBEX_SR_SESS_CB *p_scb = &obx_cb.sr_sess[0];
    tOBEX_HANDLE obx_handle = OBEX_HANDLE_NULL;

    OBEX_TRACE_DEBUG1("obx_sr_alloc_cb num sess: %d\n", p_params->max_sessions);
    /* allocate a server control block */
    for (xx=0; xx<obx_cb.num_server; xx++, p_cb++)
    {
        if (p_cb->scn == 0 && p_cb->psm == 0)
        {
            obx_handle  = xx + 1;
            p_cb->scn   = p_params->scn;
            p_cb->psm   = p_params->psm;
            break;
        }
    }

    if (xx != obx_cb.num_server)
    {
        /* allocate session control blocks */
        zz = 0;
        for (yy=0; yy<obx_cb.num_sr_sess && zz < p_params->max_sessions; yy++, p_scb++)
        {
            if (p_scb->ll_cb.comm.handle == OBEX_HANDLE_NULL)
            {
                p_scb->handle   = obx_handle;
                if (p_params->get_nonf)
                    p_scb->srmp = OBEX_SRMP_NONF_EVT;
                p_cb->sess[zz]  = yy + 1;
                p_scb->srm      = p_params->srm;
                p_scb->ll_cb.comm.handle = OBEX_ENC_SESS_HANDLE((obx_handle), zz);
                p_scb->ll_cb.comm.tx_mtu = 0;
                p_scb->ll_cb.comm.tle.param = (TIMER_PARAM_TYPE)p_scb;
                p_scb->ll_cb.comm.rx_mtu   = p_params->mtu;
                OBEX_TRACE_DEBUG2("[%d]: 0x%x\n", zz, p_scb->ll_cb.comm.handle);

                zz++;
            }
        }

#ifdef OBEX_LIB_L2CAP_INCLUDED
        /* If all sessions for server successfully initialized and PSM valid register with L2CAP */
        if (zz == p_params->max_sessions && L2C_IS_VALID_PSM(p_params->psm))
        {
            if (obx_l2c_sr_register(p_cb) != OBEX_SUCCESS)
            {
                OBEX_TRACE_ERROR0("Cannot register to L2CAP\n");
                zz = 0; /* let it fail */
            }
        }
#endif

        if (zz != p_params->max_sessions)
        {
            OBEX_TRACE_ERROR0("not enough resources: release the allocated ones\n");
            p_cb->scn   = 0;
            p_cb->psm   = 0;
            obx_handle  = OBEX_HANDLE_NULL;
            p_scb = &obx_cb.sr_sess[0];
            for (yy=0; yy<obx_cb.num_sr_sess; yy++, p_scb++)
            {
                if (p_scb->handle == obx_handle)
                {
                    p_scb->ll_cb.comm.handle = OBEX_HANDLE_NULL;
                }
            }
        }
        else    /* Server sessions successfully registered */
        {
            p_cb->num_sess  = p_params->max_sessions;
            p_cb->nonce     = p_params->nonce;
            p_cb->max_suspend       = p_params->max_suspend;
            if (p_cb->nonce && (p_cb->max_suspend == 0))
                p_cb->max_suspend = 1;
            if (p_cb->max_suspend > OBEX_MAX_SUSPEND_SESSIONS)
                p_cb->max_suspend = OBEX_MAX_SUSPEND_SESSIONS;
            if ((p_cb->max_suspend * sizeof (tOBEX_SPND_CB)) > GKI_MAX_BUF_SIZE)
            {
                OBEX_TRACE_ERROR1("OBEX_MAX_SUSPEND_SESSIONS:%d is too big\n", OBEX_MAX_SUSPEND_SESSIONS);
            }
        }
    }
    return obx_handle;
}

/*******************************************************************************
**
** Function     obx_sr_get_cb
**
** Description  Returns the pointer to the server control block with given handle.
**
*******************************************************************************/
tOBEX_SR_CB * obx_sr_get_cb(tOBEX_HANDLE handle)
{
    tOBEX_SR_CB *p_cb = NULL;
    tOBEX_HANDLE obx_handle = OBEX_DEC_HANDLE(handle);

    /* check range */
    if (obx_handle <= obx_cb.num_server && obx_handle > 0 && obx_cb.server[obx_handle-1].p_cback)
    {
        p_cb = &obx_cb.server[obx_handle-1];
    }

    return p_cb;
}

/*******************************************************************************
**
** Function     obx_sr_get_scb
**
** Description  Returns the pointer to the server control block with given handle.
**
*******************************************************************************/
tOBEX_SR_SESS_CB * obx_sr_get_scb(tOBEX_HANDLE handle)
{
    tOBEX_SR_CB      *p_cb;
    tOBEX_SR_SESS_CB *p_scb = NULL;
    tOBEX_HANDLE obx_handle;
    UINT16      sess_ind;

    /* check range */
    obx_handle = OBEX_DEC_HANDLE(handle) - 1;
    if (obx_handle < obx_cb.num_server)
    {
        /* make sure the handle is a valid one */
        p_cb = &obx_cb.server[obx_handle];
        sess_ind = OBEX_DEC_SESS_IND(handle);
        if ((sess_ind < p_cb->num_sess))
        {
            if ((obx_cb.sr_sess[p_cb->sess[sess_ind]-1].ll_cb.comm.handle) == handle)
                p_scb = &obx_cb.sr_sess[p_cb->sess[sess_ind]-1];
        }
    }

    return p_scb;
}

/*******************************************************************************
**
** Function     obx_sr_free_cb
**
** Description  Free the given server control block.
**
** Returns      void
**
*******************************************************************************/
void obx_sr_free_cb(tOBEX_HANDLE handle)
{
    tOBEX_SR_CB * p_cb = obx_sr_get_cb(handle);
    tOBEX_SR_SESS_CB *p_scb;
    int yy;

    OBEX_TRACE_DEBUG1("obx_sr_free_cb handle:0x%x\n", handle);
    /* check range */
    if (p_cb)
    {
        p_scb = &obx_cb.sr_sess[0];
        for (yy=0; yy<obx_cb.num_sr_sess; yy++, p_scb++)
        {
            if (OBEX_DEC_HANDLE(p_scb->handle) == OBEX_DEC_HANDLE(handle))
            {
                obx_sr_free_scb(p_scb);
                if (p_scb->ll_cb.comm.id>0)
                {
                    if (p_scb->ll_cb.comm.p_send_fn == (tOBEX_SEND_FN *)obx_rfc_snd_msg)
                        obx_cb.hdl_map[p_scb->ll_cb.port.port_handle - 1] = 0;
#ifdef OBEX_LIB_L2CAP_INCLUDED
                    else
                        obx_cb.l2c_map[p_scb->ll_cb.l2c.lcid - L2CAP_BASE_APPL_CID] = 0;
#endif
                }
                memset(p_scb, 0, sizeof(tOBEX_SR_SESS_CB) );
            }
        }

        memset(p_cb, 0, sizeof(tOBEX_SR_CB) );
    }
}

/*******************************************************************************
**
** Function     obx_sr_free_scb
**
** Description  Free the given server session control block.
**
** Returns      void
**
*******************************************************************************/
void obx_sr_free_scb(tOBEX_SR_SESS_CB *p_scb)
{
    OBEX_TRACE_DEBUG2("obx_sr_free_scb shandle:0x%x, sess_st:%d\n", p_scb->ll_cb.comm.handle, p_scb->sess_st);

    /* make sure the GKI buffers are freed */
    if (p_scb->p_saved_msg)
    {
        GKI_freebuf(p_scb->p_saved_msg);
        p_scb->p_saved_msg = 0;
    }

    if (p_scb->p_next_req)
    {
        GKI_freebuf(p_scb->p_next_req);
        p_scb->p_next_req = 0;
    }
    obx_free_buf (&p_scb->ll_cb);

    /* make sure the timer is stopped */
    btu_stop_timer(&p_scb->ll_cb.comm.tle);
}

/*******************************************************************************
**
** Function     obx_sr_get_next_conn_id
**
** Description  assigns the next available connection id.  It will avoid using
**              active conn id instances as well so that we can work with the
**              IVT stack bugs.
**
*******************************************************************************/
UINT32 obx_sr_get_next_conn_id(void)
{
    tOBEX_CL_CB  *p_ccb;
    int          xx;
    BOOLEAN      done;

    /* Make sure no client instances are using the value */
    do
    {
        obx_cb.next_cid++;

        /* Increment the value and make sure it is legal */
        if (obx_cb.next_cid == OBEX_INVALID_CONN_ID)
            obx_cb.next_cid = OBEX_INITIAL_CONN_ID;

        done = TRUE;
        for (xx=0, p_ccb = &obx_cb.client[0]; xx < obx_cb.num_client; xx++, p_ccb++)
        {
            /* If the handle is in use and same as proposed conn_id, increment and restart */
            if (p_ccb->ll_cb.comm.handle != OBEX_HANDLE_NULL && p_ccb->conn_id == obx_cb.next_cid)
            {
                OBEX_TRACE_WARNING1(" **** OBX CONN_ID Collision (0x%08x)  trying another...\n", obx_cb.next_cid);
                done = FALSE;
                break;
            }
        }
    } while (!done);

    return obx_cb.next_cid;
}


/*******************************************************************************
**
** Function     obx_port_handle_2cb
**
** Description  Given a port handle, return the associated client or server
**              control block.
**
** Returns
**
*******************************************************************************/
tOBEX_PORT_CB * obx_port_handle_2cb(UINT16 port_handle)
{
    tOBEX_PORT_CB    *p_pcb = NULL;
    tOBEX_HANDLE     obx_handle = 0;
    tOBEX_SR_CB      *p_cb;

    /* this function is called by obx_rfc_cback() only.
     * assume that port_handle is within range */
    obx_handle  = obx_cb.hdl_map[port_handle-1];
    OBEX_TRACE_DEBUG2("obx_port_handle_2cb port_handle:%d obx_handle:0x%x\n", port_handle, obx_handle);

    if (obx_handle > 0)
    {
        if (obx_handle & OBEX_CL_HANDLE_MASK)
        {
            obx_handle &= OBEX_CL_CB_IND_MASK;
            p_pcb = &obx_cb.client[obx_handle - 1].ll_cb.port;
        }
        else
        {
            p_cb = &obx_cb.server[OBEX_DEC_HANDLE(obx_handle) - 1];
            p_pcb = &obx_cb.sr_sess[p_cb->sess[OBEX_DEC_SESS_IND(obx_handle)]-1].ll_cb.port;
            OBEX_TRACE_DEBUG1("p_pcb port_handle:%d\n", p_pcb->port_handle);
        }
    }

    return p_pcb;
}


/**
 * Function     wiced_bt_obex_init
 *
 *              Initialize the OBEX library
 *              This function must be called before accessing any other of OBEX APIs
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_init(void)
{
    memset(&obx_cb, 0, sizeof(tOBEX_CB));

    obx_cb.next_ind     = 0;
    obx_cb.num_client   = OBEX_NUM_CLIENTS;

    obx_cb.num_server   = OBEX_NUM_SERVERS;
    obx_cb.num_sr_sess  = OBEX_NUM_SR_SESSIONS;
    obx_cb.next_cid     = OBEX_INITIAL_CONN_ID;
    obx_cb.timeout_val  = OBEX_TIMEOUT_VALUE;
    obx_cb.sess_tout_val= OBEX_SESS_TIMEOUT_VALUE;
    obx_cb.max_rx_qcount= OBEX_MAX_RX_QUEUE_COUNT;

    return OBEX_SUCCESS;
}

/*******************************************************************************
** Function     OBEX_HandleToMtu
**
** Description  Given an OBEX handle, return the associated peer MTU.
**
** Returns      MTU.
**
*******************************************************************************/
UINT16 OBEX_HandleToMtu(tOBEX_HANDLE handle)
{
    UINT16  mtu = OBEX_MIN_MTU;
    BOOLEAN rx  = (handle & OBEX_HANDLE_RX_MTU_MASK);
    int     xx;
    tOBEX_SR_SESS_CB *p_scb;

    handle &= ~OBEX_HANDLE_RX_MTU_MASK;

    if (handle != OBEX_HANDLE_NULL)
    {
        if (handle & OBEX_CL_HANDLE_MASK)
        {
            /* a client handle */
            for (xx=0; xx<obx_cb.num_client; xx++)
            {
                if (handle == obx_cb.client[xx].ll_cb.comm.handle)
                {
                    mtu = (rx) ? obx_cb.client[xx].ll_cb.comm.rx_mtu : obx_cb.client[xx].ll_cb.comm.tx_mtu;
                    break;
                }
            }
        }
        else
        {
            /* a server handle */
            p_scb = obx_sr_get_scb(handle);
            /* make sure the handle is a valid one */
            if (p_scb && p_scb->ll_cb.comm.handle != OBEX_HANDLE_NULL)
            {
                mtu = (rx) ? p_scb->ll_cb.comm.rx_mtu : p_scb->ll_cb.comm.tx_mtu;
            }
        }
    }

    if (mtu < OBEX_MIN_MTU)
        mtu = OBEX_MIN_MTU;
    OBEX_TRACE_DEBUG3("OBEX_HandleToMtu handle: 0x%x, rx:%x, mtu:%d\n", handle, rx, mtu);
    return mtu;
}
