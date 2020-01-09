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
**  Name:           obx_l2c.c
**
**  Description:    This OBX module interfaces to L2CAP
**
**
*****************************************************************************/

#include <string.h>
#include "data_types.h"
#include "wiced_bt_obex.h"
#include "obx_int.h"
#include "wiced_bt_l2c.h"

#ifdef OBEX_LIB_L2CAP_INCLUDED

/* Configuration flags. */
#define OBEX_L2C_CFG_IND_DONE    0x01
#define OBEX_L2C_CFG_CFM_DONE    0x02
#define OBEX_L2C_SECURITY_DONE   0x04
#define OBEX_L2C_CONN_RQS_DONE   0x07

/* "states" used for L2CAP channel */
#define OBEX_CH_IDLE    0       /* No connection */
#define OBEX_CH_CONN    1       /* Waiting for connection confirm */
#define OBEX_CH_CFG     2       /* Waiting for configuration complete */
#define OBEX_CH_OPEN    3       /* Channel opened */

/* callback function declarations */
static void obx_l2c_connect_ind_cback(BD_ADDR bd_addr, UINT16 lcid, UINT16 psm, UINT8 id);

static void obx_l2c_connect_cfm_cback(UINT16 lcid, UINT16 result);

static void obx_l2c_config_cfm_cback(UINT16 lcid, wiced_bt_l2cap_cfg_information_t *p_cfg);
static void obx_l2c_config_ind_cback(UINT16 lcid, wiced_bt_l2cap_cfg_information_t *p_cfg);
static void obx_l2c_disconnect_ind_cback(UINT16 lcid, BOOLEAN ack_needed);
static void obx_l2c_disconnect_cfm_cback(UINT16 lcid, UINT16 result);
static void obx_l2c_data_ind_cback(UINT16 lcid, BT_HDR *p_buf);
static void obx_l2c_congestion_ind_cback(UINT16 lcid, BOOLEAN is_congested);
static tOBEX_SR_SESS_CB *obx_lcb_2_sr_sess_cb(tOBEX_L2C_CB *p_lcb);

/* L2CAP callback function structure for server */
const wiced_bt_l2cap_appl_information_t obx_l2c_sr_appl =
{
    obx_l2c_connect_ind_cback,      /* tL2CA_CONNECT_IND_CB       */
    NULL,                           /* tL2CA_CONNECT_CFM_CB       */
    NULL,                           /* tL2CA_CONNECT_PND_CB       */
    obx_l2c_config_ind_cback,       /* tL2CA_CONFIG_IND_CB        */
    obx_l2c_config_cfm_cback,       /* tL2CA_CONFIG_CFM_CB        */
    obx_l2c_disconnect_ind_cback,   /* tL2CA_DISCONNECT_IND_CB    */
    obx_l2c_disconnect_cfm_cback,   /* tL2CA_DISCONNECT_CFM_CB    */
    NULL,                           /* tL2CA_QOS_VIOLATION_IND_CB */
    obx_l2c_data_ind_cback,         /* tL2CA_DATA_IND_CB          */
    obx_l2c_congestion_ind_cback,   /* tL2CA_CONGESTION_STATUS_CB */
    NULL                            /* tL2CA_TX_COMPLETE_CB */
};

/* L2CAP callback function structure for client */
const wiced_bt_l2cap_appl_information_t obx_l2c_cl_appl =
{
    NULL,                           /* tL2CA_CONNECT_IND_CB       */
    obx_l2c_connect_cfm_cback,      /* tL2CA_CONNECT_CFM_CB       */
    NULL,                           /* tL2CA_CONNECT_PND_CB       */
    obx_l2c_config_ind_cback,       /* tL2CA_CONFIG_IND_CB        */
    obx_l2c_config_cfm_cback,       /* tL2CA_CONFIG_CFM_CB        */
    obx_l2c_disconnect_ind_cback,   /* tL2CA_DISCONNECT_IND_CB    */
    obx_l2c_disconnect_cfm_cback,   /* tL2CA_DISCONNECT_CFM_CB    */
    NULL,                           /* tL2CA_QOS_VIOLATION_IND_CB */
    obx_l2c_data_ind_cback,         /* tL2CA_DATA_IND_CB          */
    obx_l2c_congestion_ind_cback,   /* tL2CA_CONGESTION_STATUS_CB */
    NULL                            /* tL2CA_TX_COMPLETE_CB */
};

/* OBX eL2CAP default options */
const wiced_bt_l2cap_fcr_options_t obx_l2c_fcr_opts_def =
{
    L2CAP_FCR_ERTM_MODE,            /* Mandatory for Obex over L2CAP */
    OBEX_FCR_OPT_TX_WINDOW_SIZE_BR_EDR,  /* Tx window size over Bluetooth */
    OBEX_FCR_OPT_MAX_TX_B4_DISCNT,   /* Maximum transmissions before disconnecting */
    OBEX_FCR_OPT_RETX_TOUT,          /* Retransmission timeout (2 secs) */
    OBEX_FCR_OPT_MONITOR_TOUT,       /* Monitor timeout (12 secs) */
    L2CAP_DEFAULT_ERM_MPS           /* MPS segment size */
};

/*******************************************************************************
** Function     obx_l2c_snd_evt
** Description  Sends an L2CAP event to OBX through the BTU task.
*******************************************************************************/
void obx_l2c_snd_evt (tOBEX_L2C_CB *p_l2cb, tOBEX_L2C_EVT_PARAM  param, tOBEX_L2C_EVT l2c_evt)
{
    tOBEX_L2C_EVT_MSG   evt;

    if (!p_l2cb)
        return;

    evt.l2c_evt = l2c_evt;
    evt.p_l2cb  = p_l2cb;
    evt.param  = param;

    if (p_l2cb->handle & OBEX_CL_HANDLE_MASK)
        obx_cl_proc_l2c_evt(&evt);
    else
        obx_sr_proc_l2c_evt(&evt);
}

/*******************************************************************************
**
** Function         obx_sr_cb_by_psm
**
** Description      Find the server control block for L2CAP PSM.
**
**
** Returns          void
**
*******************************************************************************/
tOBEX_SR_CB *obx_sr_cb_by_psm (UINT16 psm)
{
    UINT32  xx;

    for (xx=0; xx < obx_cb.num_server; xx++)
    {
        if (obx_cb.server[xx].psm == psm)
        {
            OBEX_TRACE_ERROR1 ("obx_sr_cb_by_psm: Found server with index %d", xx);
            return (&obx_cb.server[xx]);
        }
    }

    OBEX_TRACE_ERROR0 ("obx_sr_cb_by_psm: Server NOT FOUND!");
    return (NULL);
}

/*******************************************************************************
**
** Function         obx_sr_scb_by_psm
**
** Description      Find the server session control block for L2CAP.
**
**
** Returns          void
**
*******************************************************************************/
tOBEX_SR_SESS_CB * obx_sr_scb_by_psm (UINT16 psm)
{
    UINT32  xx, yy;
    tOBEX_SR_CB      *p_cb;
    tOBEX_SR_SESS_CB *p_scb = NULL, *p_scbt;
    UINT16          port_handle;

    for (xx=0; xx<obx_cb.num_server; xx++)
    {
        if (obx_cb.server[xx].psm == psm)
        {
            p_cb = &obx_cb.server[xx];
            /* find one that has not allocated a RFCOMM port */
            for (yy=0; yy<p_cb->num_sess; yy++)
            {
                p_scbt = &obx_cb.sr_sess[p_cb->sess[yy]-1];
                if (p_scbt->ll_cb.comm.id == 0)
                {
                    p_scb = p_scbt;
                    p_scb->ll_cb.l2c.p_close_fn = obx_close_l2c;
                    p_scb->ll_cb.l2c.p_send_fn = (tOBEX_SEND_FN *)obx_l2c_snd_msg;
                    break;
                }
            }

            if (p_scb == NULL)
            {
                /* check if an RFCOMM port can be freed */
                for (yy=0; yy<p_cb->num_sess; yy++)
                {
                    p_scbt = &obx_cb.sr_sess[p_cb->sess[yy]-1];
                    if (p_scbt->ll_cb.comm.p_send_fn == (tOBEX_SEND_FN *)obx_rfc_snd_msg && p_scbt->state == OBEX_SS_NOT_CONNECTED)
                    {
                        port_handle = p_scbt->ll_cb.port.port_handle;
                        p_scbt->ll_cb.port.port_handle = 0;
                        p_scb = p_scbt;
                        p_scb->ll_cb.l2c.p_close_fn = obx_close_l2c;
                        p_scb->ll_cb.l2c.p_send_fn = (tOBEX_SEND_FN *)obx_l2c_snd_msg;
                        obx_close_port(port_handle);
                        RFCOMM_RemoveServer(port_handle);
                        obx_sr_free_scb(p_scbt);
                        break;
                    }
                }
            }
            break;
        }
    }

    return p_scb;
}

/*******************************************************************************
**
** Function     obx_sr_proc_l2c_evt
**
** Description  This is called to process BT_EVT_TO_OBEX_SR_L2C_MSG
**              Process server events from L2CAP. Get the associated server control
**              block. If this is a request packet, stop timer. Find the
**              associated API event and save it in server control block
**              (api_evt). Fill the event parameter (param).
**              Call obx_ssm_event() with the associated events.If the associated
**              control block is not found (maybe the target header does not
**              match) or busy, compose a service unavailable response and call
**              obx_l2c_snd_msg().
** Returns      void
**
*******************************************************************************/
void obx_sr_proc_l2c_evt (tOBEX_L2C_EVT_MSG *p_msg)
{
    tOBEX_SR_SESS_CB *p_scb = NULL;
    tOBEX_L2C_CB *p_lcb;
    BT_HDR  *p_pkt=NULL;
    tOBEX_RX_HDR *p_rxh;
    UINT8 opcode;
    tOBEX_L2C_IND    *p_ind;
    wiced_bt_l2cap_cfg_information_t cfg;
    UINT16          result = L2CAP_CONN_NO_RESOURCES;
#if (BT_USE_TRACES == TRUE)
    UINT16          len;
#endif
    tL2CAP_ERTM_INFO ertm_info;
    tOBEX_EVT_PARAM  param;
    tOBEX_SR_CB      *p_cb;

    if (p_msg == NULL || p_msg->p_l2cb == NULL)
        return;

    if (p_msg->l2c_evt == OBEX_L2C_EVT_CONN_IND)
    {
        memset(&cfg, 0, sizeof(wiced_bt_l2cap_cfg_information_t));
        cfg.mtu_present = TRUE;

        p_ind = &p_msg->param.conn_ind;
        if (((p_scb = obx_sr_scb_by_psm(p_ind->psm)) != NULL)
            && (obx_sr_cb_by_psm(p_ind->psm) != NULL))
        {
            p_scb->state = OBEX_SS_NOT_CONNECTED;
            memcpy(p_scb->param.conn.peer_addr, p_ind->bd_addr, BD_ADDR_LEN);
            memcpy(p_scb->peer_addr, p_ind->bd_addr, BD_ADDR_LEN);
            result  = L2CAP_CONN_OK;
            p_lcb   = &p_scb->ll_cb.l2c;
            cfg.mtu = p_lcb->rx_mtu;

            cfg.fcr_present = TRUE;
            cfg.fcr         = obx_l2c_fcr_opts_def;
        }
        /* else no control channel yet, reject */

        /* Set the FCR options: */
        ertm_info.preferred_mode  = obx_l2c_fcr_opts_def.mode;
        ertm_info.allowed_modes = L2CAP_FCR_CHAN_OPT_ERTM;
        ertm_info.user_rx_pool_id = OBEX_USER_RX_POOL_ID;
        ertm_info.user_tx_pool_id = OBEX_USER_TX_POOL_ID;
        ertm_info.fcr_rx_pool_id =  OBEX_FCR_RX_POOL_ID;
        ertm_info.fcr_tx_pool_id =  OBEX_FCR_TX_POOL_ID;

        /* Send L2CAP connect rsp */
        L2CA_CONNECT_RSP(p_ind->bd_addr, p_ind->id, p_ind->lcid, result, 0, &ertm_info);

        /* if result ok, proceed with connection */
        if (result == L2CAP_CONN_OK)
        {
            /* store LCID */
            p_lcb->lcid = p_ind->lcid;
            obx_cb.l2c_map[p_ind->lcid - L2CAP_BASE_APPL_CID] = p_lcb->handle;
            OBEX_TRACE_DEBUG2("l2c_map[%d]=0x%x",p_ind->lcid - L2CAP_BASE_APPL_CID, p_lcb->handle );

            /* transition to configuration state */
            p_lcb->ch_state = OBEX_CH_CFG;
            p_lcb->ch_flags = OBEX_L2C_SECURITY_DONE;

            /* Send L2CAP config req */
            L2CA_CONFIG_REQ(p_ind->lcid, &cfg);
        }
        return;
    }

    p_lcb = p_msg->p_l2cb;
    p_scb = obx_lcb_2_sr_sess_cb(p_lcb);
    if (p_scb == NULL)
        return;

    switch (p_msg->l2c_evt)
    {
    case OBEX_L2C_EVT_RESUME:
        p_cb = &obx_cb.server[p_scb->handle - 1];
        param.ssn = p_scb->ssn;
        (p_cb->p_cback) (p_scb->ll_cb.comm.handle, OBEX_GET_REQ_EVT, param, p_pkt);
        break;

    case OBEX_L2C_EVT_CONG:
        p_lcb->cong = p_msg->param.is_cong;
        obx_ssm_event (p_scb, OBEX_FCS_SET_SEVT, NULL);
        break;

    case OBEX_L2C_EVT_CLOSE:
        obx_ssm_event (p_scb, OBEX_PORT_CLOSE_SEVT, NULL);
        break;

    case OBEX_L2C_EVT_DATA_IND:
        p_pkt = p_msg->param.p_pkt;
        OBEX_TRACE_DEBUG2("obx_sr_proc_l2c_evt len:%d, offset:%d", p_pkt->len, p_pkt->offset );
#if (BT_USE_TRACES == TRUE)
        len = p_pkt->len;
        if (len > 0x20)
            len = 0x20;
        obxu_dump_hex ((UINT8 *)(p_pkt + 1) + p_pkt->offset, "rsp evt", len);
#endif
        p_rxh   = (tOBEX_RX_HDR *)(p_pkt + 1);
        opcode  = *((UINT8 *)(p_pkt + 1) + p_pkt->offset);
        memset(p_rxh, 0, sizeof(tOBEX_RX_HDR));
        if (obx_verify_request (opcode, p_rxh) == OBEX_BAD_SM_EVT)
        {
            OBEX_TRACE_ERROR1("bad opcode:0x%x disconnect now", opcode );
            GKI_freebuf(p_pkt);
            /* coverity [overrun-call] */
            obx_ssm_event(p_scb, OBEX_TX_EMPTY_SEVT, NULL);
            return;
        }
        p_pkt->event    = obx_sm_evt_to_api_evt[p_rxh->sm_evt];
        p_pkt->layer_specific    = GKI_get_buf_size(p_pkt) - BT_HDR_SIZE - p_pkt->offset - p_pkt->len;
        OBEX_TRACE_DEBUG3("opcode:0x%x event:%d sm_evt:%d", opcode, p_pkt->event, p_rxh->sm_evt );
#if BT_TRACE_PROTOCOL == TRUE
        DispObxMsg(p_pkt, (BOOLEAN)(obx_api_evt_to_disp_type[p_pkt->event] | OBEX_DISP_IS_RECV));
#endif
        if (p_pkt->event != OBEX_BAD_SM_EVT)
        {
            if (GKI_queue_is_empty(&p_lcb->rx_q) && (p_scb->srm & OBEX_SRM_WAIT_UL) == 0)
            {
                obx_sr_proc_pkt (p_scb, p_pkt);
            }
            else
            {
                GKI_enqueue (&p_lcb->rx_q, p_pkt);
                if (p_lcb->rx_q.count > obx_cb.max_rx_qcount)
                {
                    p_lcb->stopped = TRUE;
                    L2CA_FlowControl(p_lcb->lcid, FALSE);
                }
                OBEX_TRACE_DEBUG4 ("obx_sr_proc_l2c_evt stopped:%d state:%d rx_q.count:%d, srm:0x%x",
                    p_lcb->stopped, p_scb->state, p_lcb->rx_q.count, p_scb->srm );
            }
        }
        else
        {
            OBEX_TRACE_ERROR0("bad SM event" );
        }
        break;
    }
}

/*******************************************************************************
**
** Function         obx_l2c_sr_register
**
** Description      register the PSM to L2CAP.
**
**
** Returns          void
**
*******************************************************************************/
tOBEX_STATUS obx_l2c_sr_register (tOBEX_SR_CB  *p_cb)
{
    tOBEX_STATUS     status = OBEX_NO_RESOURCES;

    if (L2CA_REGISTER (p_cb->psm, &obx_l2c_sr_appl))
    {
        status = OBEX_SUCCESS;
    }
    return status;
}


/*******************************************************************************
**
** Function         obx_lcb_2_sr_sess_cb
**
** Description      Find the client control block for the given l2cap session.
**
**
** Returns          void
**
*******************************************************************************/
tOBEX_SR_SESS_CB * obx_lcb_2_sr_sess_cb(tOBEX_L2C_CB *p_lcb)
{
    UINT32  xx, yy;
    tOBEX_SR_CB      *p_cb;
    tOBEX_SR_SESS_CB *p_scb = NULL;

    for (xx=0; xx<obx_cb.num_server; xx++)
    {
        if (obx_cb.server[xx].num_sess)
        {
            p_cb = &obx_cb.server[xx];
            for (yy=0; yy<p_cb->num_sess; yy++)
            {
                if (&(obx_cb.sr_sess[p_cb->sess[yy]-1].ll_cb.l2c) == p_lcb)
                {
                    p_scb = &(obx_cb.sr_sess[p_cb->sess[yy]-1]);
                    break;
                }
            }

            if (p_scb)
                break;
        }
    }
    return p_scb;
}

/*******************************************************************************
**
** Function         obx_lcb_2_clcb
**
** Description      Find the client control block for the given l2cap session.
**
**
** Returns          void
**
*******************************************************************************/
tOBEX_CL_CB * obx_lcb_2_clcb(tOBEX_L2C_CB *p_lcb)
{
    UINT32  xx;
    tOBEX_CL_CB  *p_cl_cb = NULL;

    for (xx=0; xx<obx_cb.num_client; xx++)
    {
        if (&obx_cb.client[xx].ll_cb.l2c == p_lcb)
        {
            p_cl_cb = &obx_cb.client[xx];
            break;
        }
    }
    return p_cl_cb;
}

/*******************************************************************************
**
** Function     obx_lcid_2lcb
**
** Description  Given a lcid, return the associated client or server
**              control block.
**
** Returns
**
*******************************************************************************/
tOBEX_L2C_CB * obx_lcid_2lcb(UINT16 lcid)
{
    tOBEX_L2C_CB     *p_lcb = NULL;
    tOBEX_HANDLE     obx_handle = 0;
    tOBEX_SR_CB      *p_cb;
    tOBEX_HANDLE     obx_mskd_handle;

    /* this function is called by obx_rfc_cback() only.
     * assume that port_handle is within range */
    obx_handle  = obx_cb.l2c_map[lcid-L2CAP_BASE_APPL_CID];
    obx_mskd_handle = obx_handle&OBEX_HANDLE_MASK;
    OBEX_TRACE_DEBUG3("obx_lcid_2lcb lcid:0x%x obx_handle:0x%x obx_mskd_handle:0x%x",
        lcid, obx_handle, obx_mskd_handle);

    if (obx_handle > 0)
    {
        if (obx_mskd_handle & OBEX_CL_HANDLE_MASK)
        {
            obx_mskd_handle &= ~OBEX_CL_HANDLE_MASK;
            p_lcb = &obx_cb.client[obx_mskd_handle - 1].ll_cb.l2c;
        }
        else if (obx_mskd_handle < OBEX_NUM_SERVERS)
        {
            p_cb = &obx_cb.server[obx_mskd_handle - 1];
            p_lcb = &obx_cb.sr_sess[p_cb->sess[OBEX_DEC_SESS_IND(obx_handle)]-1].ll_cb.l2c;
            OBEX_TRACE_DEBUG3("p_lcb lcid:0x%x sess_ind:%d, sr_sess[%d]",
                p_lcb->lcid, OBEX_DEC_SESS_IND(obx_handle), p_cb->sess[OBEX_DEC_SESS_IND(obx_handle)]-1);
        }
    }

    return p_lcb;
}

/*******************************************************************************
**
** Function         obx_l2c_checks_ch_flags
**
** Description      This function processes the L2CAP configuration indication
**                  event.
**
** Returns          void
**
*******************************************************************************/
static void obx_l2c_checks_ch_flags (tOBEX_L2C_CB     *p_lcb)
{
    tOBEX_L2C_EVT_PARAM  evt_param;

    OBEX_TRACE_DEBUG1 ("obx_l2c_checks_ch_flags ch_flags:0x%x ", p_lcb->ch_flags);
    /* if all the required ch_flags are set, report the OPEN event now */
    if ((p_lcb->ch_flags & OBEX_L2C_CONN_RQS_DONE) == OBEX_L2C_CONN_RQS_DONE)
    {
        p_lcb->ch_state = OBEX_CH_OPEN;
        obx_start_timer((tOBEX_COMM_CB *)p_lcb);
        evt_param.is_cong = FALSE;
        obx_l2c_snd_evt (p_lcb, evt_param, OBEX_L2C_EVT_CONG);
    }
}

/*******************************************************************************
**
** Function         obx_l2c_connect_ind_cback
**
** Description      This is the L2CAP connect indication callback function.
**
**
** Returns          void
**
*******************************************************************************/
static void obx_l2c_connect_ind_cback(BD_ADDR bd_addr, UINT16 lcid, UINT16 psm, UINT8 id)
{
    tOBEX_L2C_EVT_PARAM  evt_param;

    obx_cb.sr_l2cb.handle = 0; /* to mark as server event */
    memcpy( evt_param.conn_ind.bd_addr, bd_addr, BD_ADDR_LEN);
    evt_param.conn_ind.lcid = lcid;
    evt_param.conn_ind.psm  = psm;
    evt_param.conn_ind.id   = id;
    obx_l2c_snd_evt (&obx_cb.sr_l2cb, evt_param, OBEX_L2C_EVT_CONN_IND);
}

/*******************************************************************************
**
** Function     obx_cl_proc_l2c_evt
**
** Description  This is called to process BT_EVT_TO_OBEX_CL_L2C_MSG
**              Process client events from L2CAP. Get the associated client control
**              block. If this is a response packet, stop timer. Call
**              obx_csm_event() with event OK_CFM, FAIL_CFM or CONT_CFM.
** Returns      void
**
*******************************************************************************/
void obx_cl_proc_l2c_evt (tOBEX_L2C_EVT_MSG *p_msg)
{
    tOBEX_CL_CB *p_cl_cb = NULL;
    tOBEX_L2C_CB *p_l2cb;
    BT_HDR  *p_pkt;
    tOBEX_RX_HDR *p_rxh;
    UINT8 opcode;

    if (p_msg == NULL || p_msg->p_l2cb == NULL)
        return;

    p_l2cb = p_msg->p_l2cb;
    p_cl_cb = obx_lcb_2_clcb(p_l2cb);
    if (p_cl_cb == NULL)
        return;

    switch (p_msg->l2c_evt)
    {
    case OBEX_L2C_EVT_CONG:
        p_msg->p_l2cb->cong = p_msg->param.is_cong;
        obx_csm_event (p_cl_cb, OBEX_FCS_SET_CEVT, NULL);
        break;

    case OBEX_L2C_EVT_CLOSE:
        obx_csm_event (p_cl_cb, OBEX_PORT_CLOSE_CEVT, NULL);
        break;

    case OBEX_L2C_EVT_DATA_IND:
        p_pkt = p_msg->param.p_pkt;
        p_rxh   = (tOBEX_RX_HDR *)(p_pkt + 1);
        opcode  = *((UINT8 *)(p_pkt + 1) + p_pkt->offset);
        memset(p_rxh, 0, sizeof(tOBEX_RX_HDR));
        obx_verify_response (opcode, p_rxh);

        OBEX_TRACE_DEBUG4 ("obx_cl_proc_l2c_evt event:0x%x/0x%x state:%d srm:0x%x", p_pkt->event, p_rxh->sm_evt, p_cl_cb->state, p_cl_cb->srm );
        if (p_rxh->sm_evt != OBEX_BAD_SM_EVT)
        {
            if (GKI_queue_is_empty(&p_l2cb->rx_q) && (p_cl_cb->srm & OBEX_SRM_WAIT_UL) == 0)
            {
                obx_cl_proc_pkt (p_cl_cb, p_pkt);
            }
            else
            {
                GKI_enqueue (&p_l2cb->rx_q, p_pkt);
                if (p_l2cb->rx_q.count > obx_cb.max_rx_qcount)
                {
                    p_l2cb->stopped = TRUE;
                    L2CA_FlowControl(p_l2cb->lcid, FALSE);
                }
                OBEX_TRACE_DEBUG3 ("obx_cl_proc_l2c_evt rx_q.count:%d, stopped:%d state:%d", p_l2cb->rx_q.count, p_l2cb->stopped, p_cl_cb->state );
            }
        }
        else
        {
            OBEX_TRACE_ERROR0("bad SM event" );
        }
        break;
    }
}

/*******************************************************************************
**
** Function         obx_l2c_sec_check_complete
**
** Description      The function called when Security Manager finishes
**                  verification of the service side connection
**
** Returns          void
**
*******************************************************************************/
static void obx_l2c_sec_check_complete (BD_ADDR bd_addr, BOOLEAN transport, void *p_ref_data, UINT8 res)
{
    tOBEX_L2C_CB     *p_lcb = (tOBEX_L2C_CB *)p_ref_data;

    OBEX_TRACE_DEBUG3 ("obx_l2c_sec_check_complete ch_state:%d, ch_flags:0x%x, status:%d",
        p_lcb->ch_state, p_lcb->ch_flags, res);
    if (p_lcb->ch_state == OBEX_CH_IDLE)
        return;

    if (res == BTM_SUCCESS)
    {
        p_lcb->ch_flags |= OBEX_L2C_SECURITY_DONE;
        obx_l2c_checks_ch_flags (p_lcb);
    }
    else
    {
        /* security failed - disconnect the channel */
        L2CA_DISCONNECT_REQ (p_lcb->lcid);
    }
}

/*******************************************************************************
**
** Function         obx_l2c_connect_cfm_cback
**
** Description      This is the L2CAP connect confirm callback function.
**
**
** Returns          void
**
*******************************************************************************/
static void obx_l2c_connect_cfm_cback(UINT16 lcid, UINT16 result)
{
    tOBEX_L2C_CB     *p_lcb;
    wiced_bt_l2cap_cfg_information_t cfg;
    tOBEX_L2C_EVT_PARAM  evt_param;
    tOBEX_CL_CB *p_cl_cb = NULL;

    /* look up lcb for this channel */
    if ((p_lcb = obx_lcid_2lcb(lcid)) != NULL)
    {
        OBEX_TRACE_DEBUG1("obx_l2c_connect_cfm_cback ch_state:%d", p_lcb->ch_state);
        /* if in correct state */
        if (p_lcb->ch_state == OBEX_CH_CONN)
        {
            /* if result successful */
            if (result == L2CAP_CONN_OK)
            {
                /* set channel state */
                p_lcb->ch_state = OBEX_CH_CFG;

                p_cl_cb = obx_lcb_2_clcb(p_lcb);
                btm_sec_mx_access_request (p_cl_cb->peer_addr, p_cl_cb->psm, TRUE,
                                           0, 0, &obx_l2c_sec_check_complete, p_lcb);

                /* Send L2CAP config req */
                memset(&cfg, 0, sizeof(wiced_bt_l2cap_cfg_information_t));

                cfg.mtu_present = TRUE;
                cfg.mtu = p_lcb->rx_mtu;

                cfg.fcr_present  = TRUE;
                cfg.fcr          = obx_l2c_fcr_opts_def;

                L2CA_CONFIG_REQ(lcid, &cfg);
            }
            /* else failure */
            else
            {
                evt_param.any = 0;
                obx_l2c_snd_evt (p_lcb, evt_param, OBEX_L2C_EVT_CLOSE);
            }
        }
    }
}


/*******************************************************************************
**
** Function         obx_l2c_config_cfm_cback
**
** Description      This is the L2CAP config confirm callback function.
**
**
** Returns          void
**
*******************************************************************************/
static void obx_l2c_config_cfm_cback(UINT16 lcid, wiced_bt_l2cap_cfg_information_t *p_cfg)
{
    tOBEX_L2C_CB     *p_lcb;

    /* look up lcb for this channel */
    if ((p_lcb = obx_lcid_2lcb(lcid)) != NULL)
    {
        /* if in correct state */
        if (p_lcb->ch_state == OBEX_CH_CFG)
        {
            /* if result successful */
            if (p_cfg->result == L2CAP_CFG_OK)
            {
                /* update flags */
                p_lcb->ch_flags |= OBEX_L2C_CFG_CFM_DONE;

                /* if configuration complete */
                obx_l2c_checks_ch_flags(p_lcb);
            }
            /* else failure */
            else
            {
                /* store result value
                p_lcb->ch_result = p_cfg->result; */

                /* Send L2CAP disconnect req */
                L2CA_DISCONNECT_REQ(lcid);
            }
        }
    }
}

/*******************************************************************************
**
** Function         obx_l2c_config_ind_cback
**
** Description      This is the L2CAP config indication callback function.
**
**
** Returns          void
**
*******************************************************************************/
static void obx_l2c_config_ind_cback(UINT16 lcid, wiced_bt_l2cap_cfg_information_t *p_cfg)
{
    tOBEX_L2C_CB     *p_lcb;
    UINT16          max_mtu = OBEX_MAX_MTU;

    /* Don't include QoS nor flush timeout in the response since we
       currently always accept these values.  Note: fcr_present is left
       untouched since l2cap negotiates this internally
    */
    p_cfg->flush_to_present = FALSE;
    p_cfg->qos_present = FALSE;

    /* look up lcb for this channel */
    if ((p_lcb = obx_lcid_2lcb(lcid)) != NULL)
    {
        /* store the mtu in tbl */
        if (p_cfg->mtu_present)
        {
            p_lcb->tx_mtu = p_cfg->mtu;
        }
        else
        {
            p_lcb->tx_mtu = L2CAP_DEFAULT_MTU;
        }

        if (p_lcb->tx_mtu > max_mtu)
        {
            p_lcb->tx_mtu = p_cfg->mtu = max_mtu;

            /* Must tell the peer what the adjusted value is */
            p_cfg->mtu_present = TRUE;
        }
        else    /* Don't include in the response */
            p_cfg->mtu_present = FALSE;
        OBEX_TRACE_DEBUG2 ("obx_l2c_config_ind_cback tx_mtu:%d use:%d", p_lcb->tx_mtu, max_mtu);

        p_cfg->result = L2CAP_CFG_OK;

        /* send L2CAP configure response */
        L2CA_CONFIG_RSP(lcid, p_cfg);

        if (p_cfg->result != L2CAP_CFG_OK)
        {
            return;
        }

        /* if first config ind */
        if ((p_lcb->ch_flags & OBEX_L2C_CFG_IND_DONE) == 0)
        {
            /* update flags */
            p_lcb->ch_flags |= OBEX_L2C_CFG_IND_DONE;

            /* if configuration complete */
            obx_l2c_checks_ch_flags(p_lcb);
        }
    }
}

/*******************************************************************************
**
** Function         obx_l2c_disconnect_ind_cback
**
** Description      This is the L2CAP disconnect indication callback function.
**
**
** Returns          void
**
*******************************************************************************/
static void obx_l2c_disconnect_ind_cback(UINT16 lcid, BOOLEAN ack_needed)
{
    tOBEX_L2C_CB     *p_lcb;
    tOBEX_L2C_EVT_PARAM  evt_param;

    /* look up lcb for this channel */
    if ((p_lcb = obx_lcid_2lcb(lcid)) != NULL)
    {
        if (ack_needed)
        {
            /* send L2CAP disconnect response */
            L2CA_DISCONNECT_RSP(lcid);
        }

        evt_param.any = 0;
        obx_l2c_snd_evt (p_lcb, evt_param, OBEX_L2C_EVT_CLOSE);
    }
}

/*******************************************************************************
**
** Function         obx_l2c_disconnect_cfm_cback
**
** Description      This is the L2CAP disconnect confirm callback function.
**
**
** Returns          void
**
*******************************************************************************/
static void obx_l2c_disconnect_cfm_cback(UINT16 lcid, UINT16 result)
{
    tOBEX_L2C_CB     *p_lcb;
    tOBEX_L2C_EVT_PARAM  evt_param;

    /* look up lcb for this channel */
    if ((p_lcb = obx_lcid_2lcb(lcid)) != NULL)
    {
        evt_param.any = 0;
        obx_l2c_snd_evt (p_lcb, evt_param, OBEX_L2C_EVT_CLOSE);
    }
}

/*******************************************************************************
**
** Function         obx_l2c_data_ind_cback
**
** Description      This is the L2CAP data indication callback function.
**
**
** Returns          void
**
*******************************************************************************/
static void obx_l2c_data_ind_cback(UINT16 lcid, BT_HDR *p_buf)
{
    tOBEX_L2C_CB     *p_lcb;
    tOBEX_L2C_EVT_PARAM  evt_param;
#if (BT_USE_TRACES == TRUE)
    UINT16          len;
#endif

    /* look up lcb for this channel */
    if ((p_lcb = obx_lcid_2lcb(lcid)) != NULL)
    {
        evt_param.p_pkt = p_buf;
        OBEX_TRACE_DEBUG3("obx_l2c_data_ind_cback 0x%x, len:%d, offset:%d", p_buf, p_buf->len, p_buf->offset );
#if (BT_USE_TRACES == TRUE)
        len = p_buf->len;
        if (len > 0x20)
            len = 0x20;
        obxu_dump_hex ((UINT8 *)(p_buf + 1) + p_buf->offset, "rsp cback", len);
#endif
        obx_l2c_snd_evt (p_lcb, evt_param, OBEX_L2C_EVT_DATA_IND);
    }
    else /* prevent buffer leak */
        GKI_freebuf(p_buf);
}


/*******************************************************************************
**
** Function         obx_l2c_congestion_ind_cback
**
** Description      This is the L2CAP congestion indication callback function.
**
**
** Returns          void
**
*******************************************************************************/
static void obx_l2c_congestion_ind_cback(UINT16 lcid, BOOLEAN is_congested)
{
    tOBEX_L2C_CB     *p_lcb;
    tOBEX_L2C_EVT_PARAM  evt_param;

    OBEX_TRACE_DEBUG2("obx_l2c_congestion_ind_cback lcid:%d, is_congested:%d",lcid, is_congested );
    /* look up lcb for this channel */
    if ((p_lcb = obx_lcid_2lcb(lcid)) != NULL)
    {
        evt_param.is_cong = is_congested;
        obx_l2c_snd_evt (p_lcb, evt_param, OBEX_L2C_EVT_CONG);
    }
}

/*******************************************************************************
** Function     obx_register_l2c
** Description  Call L2CA_Register() to get virtual psm.
** Returns
*******************************************************************************/
void obx_register_l2c(tOBEX_CL_CB *p_cl_cb, UINT16 psm)
{
    p_cl_cb->psm = L2CA_REGISTER (psm, &obx_l2c_cl_appl);
}

/*******************************************************************************
** Function     obx_open_l2c
** Description  Call L2CA_Register() & L2CA_ConnectReq() to get lcid.
** Returns      port handle
*******************************************************************************/
tOBEX_STATUS obx_open_l2c(tOBEX_CL_CB *p_cl_cb, const BD_ADDR bd_addr)
{
    tOBEX_L2C_CB *p_l2cb = &p_cl_cb->ll_cb.l2c;
    tOBEX_STATUS status  = OBEX_NO_RESOURCES; /* successful */
    UINT16      max_mtu = OBEX_MAX_MTU;
    wiced_bt_l2cap_cfg_information_t cfg;
    tL2CAP_ERTM_INFO ertm_info;

    OBEX_TRACE_DEBUG2("obx_open_l2c rxmtu:%d, cbmtu:%d", p_l2cb->rx_mtu, max_mtu );

    /* clear buffers from previous connection */
    obx_free_buf(&p_cl_cb->ll_cb);

    /* make sure the MTU is in registered range */
    if (p_l2cb->rx_mtu > max_mtu)
        p_l2cb->rx_mtu   = max_mtu;
    if (p_l2cb->rx_mtu < OBEX_MIN_MTU)
        p_l2cb->rx_mtu   = OBEX_MIN_MTU;

    if (p_cl_cb->psm)
    {
        memcpy (p_cl_cb->peer_addr, bd_addr, BD_ADDR_LEN);

        /* Set the FCR options: */
        ertm_info.preferred_mode  = obx_l2c_fcr_opts_def.mode;
        ertm_info.allowed_modes = L2CAP_FCR_CHAN_OPT_ERTM;
        ertm_info.user_rx_pool_id = OBEX_USER_RX_POOL_ID;
        ertm_info.user_tx_pool_id = OBEX_USER_TX_POOL_ID;
        ertm_info.fcr_rx_pool_id =  OBEX_FCR_RX_POOL_ID;
        ertm_info.fcr_tx_pool_id =  OBEX_FCR_TX_POOL_ID;
        p_l2cb->lcid = L2CA_CONNECT_REQ (p_cl_cb->psm, (BD_ADDR_PTR)bd_addr, &ertm_info);

        if (p_l2cb->lcid)
        {
            p_l2cb->ch_state = OBEX_CH_CONN;
            p_l2cb->ch_flags = 0;
            p_l2cb->cong    = TRUE;
            memset(&cfg, 0, sizeof(wiced_bt_l2cap_cfg_information_t));
            cfg.fcr_present = TRUE;
            cfg.fcr         = obx_l2c_fcr_opts_def;
            status = OBEX_SUCCESS;
        }
    }

    OBEX_TRACE_DEBUG3("obx_open_l2c rxmtu:%d, lcid:%d, l2c.handle:0x%x",
        p_l2cb->rx_mtu, p_l2cb->lcid, p_l2cb->handle );

    if (status == OBEX_SUCCESS)
    {
        obx_cb.l2c_map[p_l2cb->lcid - L2CAP_BASE_APPL_CID] = p_l2cb->handle;
        p_l2cb->p_send_fn = (tOBEX_SEND_FN *)obx_l2c_snd_msg;
        p_l2cb->p_close_fn = obx_close_l2c;
    }
    else
    {
        status = OBEX_NO_RESOURCES;
    }

    return status;
}

/*******************************************************************************
**
** Function     obx_close_l2c
** Description  Clear the port event mask and callback. Close the port.
** Returns      void
*******************************************************************************/
void obx_close_l2c(UINT16 lcid)
{
    L2CA_DISCONNECT_REQ (lcid);
}

/*******************************************************************************
** Function     obx_l2c_snd_msg
** Description  Call PORT_WriteData() to send an OBEX message to peer. If
**              all data is sent, free the GKI buffer that holds
**              the OBEX message.  If  only portion of data is
**              sent, adjust the BT_HDR for PART state.
** Returns      TRUE if all data is sent
*******************************************************************************/
BOOLEAN obx_l2c_snd_msg(tOBEX_L2C_CB *p_l2cb)
{
    BOOLEAN sent = FALSE;

    if (!p_l2cb->cong)
    {
        OBEX_TRACE_DEBUG2("obx_l2c_snd_msg len:%d, offset:0x%x", p_l2cb->p_txmsg->len, p_l2cb->p_txmsg->offset);

        obx_stop_timer(&p_l2cb->tle);
        if (L2CA_DATA_WRITE (p_l2cb->lcid, p_l2cb->p_txmsg) == L2CAP_DW_CONGESTED)
        {
            OBEX_TRACE_DEBUG0("obx_l2c_snd_msg congested");
            p_l2cb->cong = TRUE;
        }
        obx_start_timer ((tOBEX_COMM_CB *)p_l2cb);
        p_l2cb->p_txmsg = NULL;
        sent = TRUE;
    }

    return sent;
}

#endif  /* OBEX_LIB_L2CAP_INCLUDED */
