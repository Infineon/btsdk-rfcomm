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
**  Name:           wiced_bt_mce_act.c
**
**  Description:    This file contains the message access client action
**                  functions.
**
*****************************************************************************/

#include <string.h>

#include "obx_int.h"
#include "wiced.h"
#include "wiced_bt_sdp_defs.h"
#include "wiced_bt_mce_int.h"
#include "wiced_bt_l2c.h"

/*****************************************************************************
**  Constants
*****************************************************************************/
#define WICED_MCE_HANDLE_STR_SIZE       25
#define WICED_MCE_FILE_PATH_SIZE       305

static void wiced_mce_sdp_cback0(UINT16 status);
static void wiced_mce_sdp_cback1(UINT16 status);
static void wiced_mce_sdp_cback2(UINT16 status);
static void wiced_mce_sdp_cback3(UINT16 status);
static void wiced_mce_sdp_cback4(UINT16 status);
static void wiced_mce_sdp_cback5(UINT16 status);
static void wiced_mce_sdp_cback6(UINT16 status);
static void wiced_mce_sdp_cback7(UINT16 status);

/* SDP callback function table */
typedef wiced_bt_sdp_discovery_complete_cback_t *wiced_bt_mce_sdp_cback_t;
static const wiced_bt_mce_sdp_cback_t wiced_mce_sdp_cback_arr[] = {
    wiced_mce_sdp_cback0,
    wiced_mce_sdp_cback1,
    wiced_mce_sdp_cback2,
    wiced_mce_sdp_cback3,
    wiced_mce_sdp_cback4,
    wiced_mce_sdp_cback5,
    wiced_mce_sdp_cback6,
    wiced_mce_sdp_cback7
};

static void wiced_mce_search_sdp_db(UINT8 ccb_idx, wiced_bt_ma_inst_id_t inst_id);
static void wiced_mce_sdp_cback(UINT8 ccb_idx, UINT16 status);
void wiced_mce_ma_obx_cback (wiced_bt_obex_handle_t handle, wiced_bt_obex_event_t obx_event, UINT8 rsp_code, wiced_bt_obex_evt_param_t param, uint8_t *p_pkt);
void wiced_mce_mn_obx_cback (wiced_bt_obex_handle_t handle, wiced_bt_obex_event_t obx_event, wiced_bt_obex_evt_param_t param, uint8_t *p_pkt);
#if (WICED_MCE_DEBUG == TRUE) && (BT_USE_TRACES == TRUE)
static char *mce_ma_obx_evt_code(UINT16 evt_code);
static char *mce_mn_obx_evt_code(UINT16 evt_code);
#endif

/*******************************************************************************
** Message Access Client (MCE_MA) Action functions
********************************************************************************/

/*******************************************************************************
**
** Function         wiced_mce_ma_init_sdp
**
** Description      Action routine for init SDP event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_init_sdp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_bt_uuid_t     uuid_list;
    wiced_mce_ma_cb_t   *p_ccb = WICED_MCE_GET_MA_CB_PTR(ccb_idx);
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
    UINT16              attr_list[8];
    UINT16              num_attrs = 8;
#else
    UINT16              attr_list[7];
    UINT16              num_attrs = 7;
#endif

    APPL_TRACE_EVENT0("wiced_mce_ma_init_sdp\n");

    /* If there is no active MAP connection, start SDP */
    if (p_ccb->active_count == 0)
    {
        p_ccb->in_use = TRUE;
        p_ccb->sec_mask = p_data->api_open.sec_mask;
        p_ccb->sdp_cback = wiced_mce_sdp_cback_arr[ccb_idx];
        memcpy(p_ccb->bd_addr, p_data->api_open.bd_addr, BD_ADDR_LEN);
        p_icb->obx_oper = WICED_MCE_OP_NONE;
        p_icb->inst_id = p_data->api_open.mas_inst_id;
        p_icb->is_opening = TRUE;

        /* only one SDP can be active at a time */
        if (!p_ccb->sdp_pending)
        {
            p_ccb->sdp_inst_id = p_icb->inst_id;
            if ((p_ccb->p_db = (wiced_bt_sdp_discovery_db_t *) GKI_getbuf(WICED_MCE_DISC_SIZE)) != NULL)
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

                wiced_bt_sdp_init_discovery_db(p_ccb->p_db, WICED_MCE_DISC_SIZE, 1, &uuid_list, num_attrs, attr_list);

                if (!wiced_bt_sdp_service_search_attribute_request(p_ccb->bd_addr, p_ccb->p_db, p_ccb->sdp_cback))
                {
                    wiced_mce_ma_sm_execute(ccb_idx, icb_idx, WICED_MCE_SDP_FAIL_EVT, p_data);
                }
                else
                {
                    p_ccb->sdp_pending = TRUE;
                }
            }
        }
        /* There is a SDP going on, set the pending flag to TRUE */
        /* It will be handled when SDP finishes in  wiced_mce_sdp_cback */
        else
        {
            p_icb->sdp_pending = TRUE;
        }

    }
    /* search the existing SDP database to open the connection */
    else
    {
        p_icb->inst_id = p_data->api_open.mas_inst_id;
        p_icb->is_opening = TRUE;
        wiced_mce_search_sdp_db(ccb_idx, p_icb->inst_id);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_ma_free_db
**
** Description      Action routine for free SDP event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_free_db(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_ma_cb_t *p_ccb = WICED_MCE_GET_MA_CB_PTR(ccb_idx);

    APPL_TRACE_EVENT0("wiced_mce_ma_free_db\n");
    p_ccb->sdp_pending = FALSE;

    utl_freebuf((void **)&p_ccb->p_db);

    wiced_mce_ma_close(ccb_idx, icb_idx, p_data);
}

/*******************************************************************************
**
** Function         wiced_mce_ma_start_client
**
** Description      Action routine for start client (MAS session) event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_start_client(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_ma_cb_t   *p_ccb = WICED_MCE_GET_MA_CB_PTR(ccb_idx);
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_mce_obx_pkt_t *p_obx = WICED_MCE_GET_MA_OBX_PKT_PTR(ccb_idx, icb_idx);
    BOOLEAN             send_close_evt = TRUE;
    wiced_mce_obx_evt_t obx_evt;

#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
    tOBEX_STATUS         status;
    BOOLEAN             use_srm = TRUE; /* Always allow if OBEX/L2CAP */

    APPL_TRACE_EVENT0("wiced_mce_ma_start_client l2cap\n");

    /* save peer supported features */
    p_icb->mce_peer_features = p_data->sdp_result.peer_features;
    APPL_TRACE_EVENT1("Peer Supported Features 0x%08x\n",p_icb->mce_peer_features);
    p_ccb->sdp_pending = FALSE;

    /* Allocate an OBX packet */
    if ((p_obx->p_pkt = (BT_HDR *)(BT_HDR  *)wiced_bt_obex_header_init(OBEX_HANDLE_NULL, OBEX_MAX_MTU)) != NULL)
    {
        status = wiced_bt_obex_alloc_session (NULL, p_data->sdp_result.scn, &p_data->sdp_result.psm,
                              wiced_mce_ma_obx_cback, &p_icb->obx_handle);

        /* set security level */
        if (p_data->sdp_result.scn)
        {
            use_srm = FALSE;
        }

        if (status == OBEX_SUCCESS)
        {
            wiced_bt_obex_add_header((uint8_t *)p_obx->p_pkt, OBEX_HI_TARGET, (UINT8 *)WICED_MAS_MESSAGE_ACCESS_TARGET_UUID,
                             WICED_MAS_UUID_LENGTH);

            if ((wiced_bt_obex_create_session (p_ccb->bd_addr, OBEX_MAX_MTU, use_srm, 0,
                               p_icb->obx_handle, (uint8_t *)p_obx->p_pkt)) == OBEX_SUCCESS)
            {
                p_obx->p_pkt = NULL;    /* OBX will free the memory */
                send_close_evt = FALSE;
            }
        }
    }
#else

    APPL_TRACE_EVENT0("wiced_mce_ma_start_client\n");

    p_ccb->sdp_pending = FALSE;

    /* Allocate an OBX packet */
    if ((p_obx->p_pkt = (BT_HDR *)(BT_HDR  *)wiced_bt_obex_header_init(OBEX_HANDLE_NULL, OBEX_CMD_POOL_SIZE)) != NULL)
    {
        wiced_bt_obex_add_header((uint8_t *)p_obx->p_pkt, OBEX_HI_TARGET, (UINT8 *)WICED_MAS_MESSAGE_ACCESS_TARGET_UUID,
                         WICED_MAS_UUID_LENGTH);

        if (wiced_bt_obex_connect(p_ccb->bd_addr, p_data->sdp_result.scn, OBEX_MAX_MTU,
                           wiced_mce_ma_obx_cback, &p_icb->obx_handle, (uint8_t *)p_obx->p_pkt) == OBEX_SUCCESS)
        {
            p_obx->p_pkt = NULL;    /* OBX will free the memory */
            send_close_evt = FALSE;
        }
    }
#endif  /* BTA_MAP_1_2_SUPPORTED */

    if (send_close_evt)
    {
        utl_freebuf((void**)&p_obx->p_pkt);
        memset(&obx_evt, 0, sizeof(wiced_mce_obx_evt_t));
        obx_evt.rsp_code = OBEX_RSP_FORBIDDEN;
        wiced_mce_ma_sm_execute(ccb_idx, icb_idx, WICED_MCE_MA_OBX_CLOSE_EVT, (wiced_mce_data_t *) &obx_evt);
        wiced_mce_send_ma_open_evt(ccb_idx, icb_idx, WICED_BT_MA_STATUS_FAIL);
    }

}

/*******************************************************************************
**
** Function         wiced_mce_ma_stop_client
**
** Description      Action routine for stop client (MAS session) event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_stop_client(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_ma_cb_t   *p_ccb = WICED_MCE_GET_MA_CB_PTR(ccb_idx);
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);

    APPL_TRACE_EVENT0("wiced_mce_ma_stop_client\n");

    if (!p_ccb->sdp_pending)
    {
        /* Start stop response timer */
        wiced_start_timer(&p_icb->rsp_timer, 2);

        p_icb->timer_oper = WICED_MCE_TIMER_OP_STOP;
        wiced_bt_obex_disconnect(p_icb->obx_handle, NULL);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_ma_close
**
** Description      Action routine for Obex close event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_close(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_ma_cb_t   *p_ccb = WICED_MCE_GET_MA_CB_PTR(ccb_idx);
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);

    APPL_TRACE_EVENT0("wiced_mce_ma_close\n");

    /* finished if not waiting for SDP */
    if (!p_ccb->sdp_pending)
    {
        wiced_mce_ma_sm_execute(ccb_idx, icb_idx, WICED_MCE_MA_CLOSE_CMPL_EVT, p_data);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_ma_obx_conn_rsp
**
** Description      Action routine for connection response event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_obx_conn_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_ma_cb_t   *p_ccb = WICED_MCE_GET_MA_CB_PTR(ccb_idx);
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);

    APPL_TRACE_EVENT0("wiced_mce_ma_obx_conn_rsp\n");

    p_ccb->peer_mtu = p_data->obx_evt.param.conn.mtu;

    /* Finished opening process, set the flag to FALSE */
    p_icb->is_opening = FALSE;

    /* inform role manager */
    //wiced_mce_pm_conn_open(p_ccb->bd_addr);

    /* Done with Obex packet */
    utl_freebuf((void**)&p_data->obx_evt.p_pkt);

    if (p_data->obx_evt.rsp_code == OBEX_RSP_FORBIDDEN)
        wiced_mce_send_ma_open_evt(ccb_idx, icb_idx, WICED_BT_MA_STATUS_EACCES);
    else
        wiced_mce_send_ma_open_evt(ccb_idx, icb_idx, WICED_BT_MA_STATUS_OK);
}

/*******************************************************************************
**
** Function         wiced_mce_ma_abort
**
** Description      Action routine for API abort event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_abort(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t      *p_cb = &wiced_mce_cb;
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_bt_mce_abort_t app_evt;

    APPL_TRACE_EVENT0("wiced_mce_ma_abort\n");

    /* Abort an active request */
    if (p_icb->obx_oper != WICED_MCE_OP_NONE)
    {
        p_icb->aborting = WICED_MCE_ABORT_REQ_NOT_SENT;

        /* Issue the abort request only if no request pending.
         * some devices do not like out of sequence aborts even though
         * the spec allows it */
        if ( !p_icb->req_pending || p_icb->first_req_pkt ) /* allow request for the first packet */
        {
            wiced_mce_send_abort_req(p_icb);
        }
    }
    else
    {
        app_evt.status = WICED_BT_MA_STATUS_FAIL;
        app_evt.session_id = p_icb->obx_handle;
        app_evt.obx_rsp_code = OBEX_RSP_OK;
        p_cb->p_cback(WICED_BT_MCE_ABORT_EVT, (wiced_bt_mce_t *)&app_evt);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_ma_notif_reg
**
** Description      Action routine for API notification registration event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_notif_reg(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t          *p_cb = &wiced_mce_cb;
    wiced_mce_inst_cb_t     *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_mce_obx_pkt_t     *p_obx = WICED_MCE_GET_MA_OBX_PKT_PTR(ccb_idx, icb_idx);
    wiced_bt_ma_status_t    status = WICED_BT_MA_STATUS_FAIL;
    wiced_bool_t            final = WICED_TRUE;
    wiced_bt_mce_notif_reg_t app_evt;
    UINT8                   *p, *p_start;
    UINT16                  len = 0;

    APPL_TRACE_EVENT0("wiced_mce_ma_notif_reg\n");
    if (p_icb->obx_oper == WICED_MCE_OP_NONE)
    {
        if ((p_obx->p_pkt = (BT_HDR  *)wiced_bt_obex_header_init(p_icb->obx_handle, OBX_CMD_POOL_SIZE)) != NULL)
        {
            p_obx->offset = 0;

            /* add type header */
            p = (UINT8 *)WICED_BT_MA_HDR_TYPE_NOTIF_REG;
            wiced_bt_obex_add_header((uint8_t *)p_obx->p_pkt, OBEX_HI_TYPE, p, strlen((char *)p) + 1);

            /* add app params for Notification Status */
            p_start = wiced_bt_obex_add_byte_sequence_start((uint8_t *)p_obx->p_pkt, &len);
            p = p_start;

            *p++    = WICED_BT_MA_APH_NOTIF_STATUS;
            *p++    = 1;
            *p++    = p_data->api_notif_reg.status;

            /* here param header is added */
            wiced_bt_obex_add_byte_sequence_end((uint8_t *)p_obx->p_pkt, OBEX_HI_APP_PARMS, (UINT16)(p - p_start));

            /* add filler byte */
            wiced_mce_add_filler_byte(p_obx);

            /* Send out the data */
            if (wiced_bt_obex_send_request(p_icb->obx_handle, OBEX_REQ_PUT, (wiced_bt_obex_req_param_t*)&final, (uint8_t *)p_obx->p_pkt) == OBEX_SUCCESS)
            {
                p_icb->obx_oper = WICED_MCE_OP_NOTIF_REG;
                p_icb->req_pending = TRUE;
                status = WICED_BT_MA_STATUS_OK;
            }
            p_obx->p_pkt = NULL;
        }
    }

    if (status != WICED_BT_MA_STATUS_OK)
    {
        app_evt.status = status;
        app_evt.session_id = p_icb->obx_handle;
        app_evt.obx_rsp_code = OBEX_RSP_OK;
        p_cb->p_cback(WICED_BT_MCE_NOTIF_REG_EVT, (wiced_bt_mce_t *)&app_evt);
    }
}


/*******************************************************************************
**
** Function         wiced_mce_ma_push_msg
**
** Description      Action routine for API PUSH Message event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_push_msg(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t              *p_cb = &wiced_mce_cb;
    wiced_mce_inst_cb_t         *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_mce_obx_pkt_t         *p_obx = WICED_MCE_GET_MA_OBX_PKT_PTR(ccb_idx, icb_idx);
    wiced_bt_ma_status_t        status = WICED_BT_MA_STATUS_FAIL;
    wiced_bt_ma_push_msg_param_t *p_param = &p_data->api_push_msg.param;
    wiced_bool_t                final = p_param->is_final;
    wiced_bt_mce_push_msg_t     app_evt;
    UINT8                       *p, *p_start;
    UINT16                      len = 0;
    wiced_bool_t                add_header = (p_icb->first_push_msg || final);

    APPL_TRACE_EVENT0("wiced_mce_ma_push_msg final %d\n", final);

    if (p_icb->obx_oper == WICED_MCE_OP_NONE)
    {
        if ((p_obx->p_pkt = (BT_HDR  *)wiced_bt_obex_header_init(p_icb->obx_handle, OBX_LRG_DATA_POOL_SIZE)) != NULL)
        {
            p_obx->offset = 0;

            if (add_header)
            {
                /* add type header */
                p = (UINT8 *)WICED_BT_MA_HDR_TYPE_MSG;
                wiced_bt_obex_add_header((uint8_t *)p_obx->p_pkt, OBEX_HI_TYPE, p, strlen((char *)p) + 1);
            }

            /* add name header */
            if ( add_header && !wiced_bt_obex_add_header_utf8((UINT8 *)p_obx->p_pkt, OBEX_HI_NAME, (UINT8 *)p_param->p_folder))
            {
                utl_freebuf((void**)&p_obx->p_pkt);
            }
            else
            {
                /* add application header */
                if (add_header)
                {
                    p_start = wiced_bt_obex_add_byte_sequence_start((uint8_t *)p_obx->p_pkt, &len);
                    p = p_start;

                    *p ++ = WICED_BT_MA_APH_CHARSET;
                    *p ++ = 1;
                    *p ++ = p_param->charset ;

                    if (p_param->transparent != WICED_BT_MA_TRANSP_UNKNOWN)
                    {
                        *p ++ = WICED_BT_MA_APH_TRANSPARENT;
                        *p ++ = 1;
                        *p ++ = p_param->transparent;
                    }

                    if (p_param->retry != WICED_BT_MA_RETRY_UNKNOWN)
                    {
                        *p ++ = WICED_BT_MA_APH_RETRY;
                        *p ++ = 1;
                        *p ++ = p_param->retry;
                    }

                    wiced_bt_obex_add_byte_sequence_end((uint8_t *)p_obx->p_pkt, OBEX_HI_APP_PARMS, (UINT16)(p - p_start));
                }
                /* Add the body header to the packet */
                wiced_bt_obex_add_header((uint8_t *)p_obx->p_pkt,
                        p_param->is_final ? OBEX_HI_BODY_END : OBEX_HI_BODY,
                        p_param->p_msg, p_param->len);

                APPL_TRACE_EVENT1("OBEX Send out %d bytes message data\n", p_param->len);
                if (wiced_bt_obex_send_request(p_icb->obx_handle, OBEX_REQ_PUT, (wiced_bt_obex_req_param_t*)&final, (uint8_t *)p_obx->p_pkt) == OBEX_SUCCESS)
                {
                    p_icb->first_push_msg = WICED_FALSE;
                    p_icb->obx_oper = WICED_MCE_OP_PUSH_MSG;
                    p_icb->req_pending = TRUE;
                    status = WICED_BT_MA_STATUS_OK;
                }
                p_obx->p_pkt = NULL;

                if (p_param->is_final)
                {
                    p_icb->first_push_msg = WICED_TRUE;
                }
            }
        }
    }

    // Send request for next package assuming SRM is enabled.
    if ( (status == WICED_BT_MA_STATUS_OK) && wiced_bt_obex_send_pkt_allowed(p_icb->obx_handle)
        && (p_param->is_final == WICED_FALSE))
    {
        wiced_bt_mce_push_msg_t evt_data;

        evt_data.status = WICED_BT_MA_STATUS_OK;
        evt_data.session_id = p_icb->obx_handle;

        memcpy(evt_data.msg_handle, p_icb->msg_hdl_str, WICED_BT_MA_HANDLE_SIZE);

        p_cb->p_cback(WICED_BT_MCE_PUSH_MSG_EVT, (wiced_bt_mce_t *)&evt_data);
        p_icb->obx_oper = WICED_MCE_OP_NONE;
    }

    if (status != WICED_BT_MA_STATUS_OK)
    {
        app_evt.obx_rsp_code = OBEX_RSP_OK;
        app_evt.status = status;
        app_evt.session_id = p_icb->obx_handle;
        p_icb->first_push_msg = WICED_TRUE;
        p_cb->p_cback(WICED_BT_MCE_PUSH_MSG_EVT, (wiced_bt_mce_t *)&app_evt);
    }
}
/*******************************************************************************
**
** Function         wiced_mce_ma_upd_inbox
**
** Description      Action routine for API update inbox event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_upd_inbox(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t          *p_cb = &wiced_mce_cb;
    wiced_mce_inst_cb_t     *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_mce_obx_pkt_t     *p_obx = WICED_MCE_GET_MA_OBX_PKT_PTR(ccb_idx, icb_idx);
    wiced_bool_t            final = WICED_TRUE;
    wiced_bt_mce_update_inbox_t app_evt;
    wiced_bt_ma_status_t    status = WICED_BT_MA_STATUS_FAIL;
    UINT8                   *p;

    APPL_TRACE_EVENT0("wiced_mce_ma_upd_inbox\n");

    if (p_icb->obx_oper == WICED_MCE_OP_NONE)
    {
        if ((p_obx->p_pkt = (BT_HDR  *)wiced_bt_obex_header_init(p_icb->obx_handle, OBX_CMD_POOL_SIZE)) != NULL)
        {
            /* add type header */
            p = (UINT8 *)WICED_BT_MA_HDR_TYPE_MSG_UPDATE;
            wiced_bt_obex_add_header((uint8_t *)p_obx->p_pkt, OBEX_HI_TYPE, p, strlen((char *)p) + 1);

            /* add filler byte */
            wiced_mce_add_filler_byte(p_obx);

            /* Send out the data */
            if (wiced_bt_obex_send_request(p_icb->obx_handle, OBEX_REQ_PUT, (wiced_bt_obex_req_param_t*)&final, (uint8_t *)p_obx->p_pkt) == OBEX_SUCCESS)
            {
                p_icb->obx_oper = WICED_MCE_OP_UPD_INBOX;
                p_icb->req_pending = TRUE;
                status = WICED_BT_MA_STATUS_OK;
            }
            p_obx->p_pkt = NULL;
        }
    }

    if (status != WICED_BT_MA_STATUS_OK)
    {
        app_evt.status = status;
        app_evt.obx_rsp_code = OBEX_RSP_OK;
        app_evt.session_id = p_icb->obx_handle;
        p_cb->p_cback(WICED_BT_MCE_UPDATE_INBOX_EVT, (wiced_bt_mce_t *)&app_evt);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_ma_chdir
**
** Description      Action routine for API change directory event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_chdir(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t              *p_cb = &wiced_mce_cb;
    wiced_mce_inst_cb_t         *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_mce_obx_pkt_t         *p_obx = WICED_MCE_GET_MA_OBX_PKT_PTR(ccb_idx, icb_idx);
    wiced_mce_api_setfolder_t   *p_chdir = &p_data->api_setfolder;
    wiced_bt_ma_status_t        status = WICED_BT_MA_STATUS_FAIL;
    wiced_bt_obex_setpath_flag_t obx_flags = OBEX_SPF_NO_CREATE;
    wiced_bt_mce_set_folder_t   app_evt;

    APPL_TRACE_EVENT0("wiced_mce_ma_chdir %d\n", p_icb->obx_oper);

    if (p_icb->obx_oper == WICED_MCE_OP_NONE)
    {
        if ((p_obx->p_pkt = (BT_HDR  *)wiced_bt_obex_header_init(p_icb->obx_handle, OBX_CMD_POOL_SIZE)) != NULL)
        {
            status = WICED_BT_MA_STATUS_OK;

            /* Add the name header if not backing up */
            if (p_chdir->flag == WICED_BT_MA_DIR_NAV_UP_ONE_LVL)
                obx_flags |= OBEX_SPF_BACKUP;

            if (!wiced_bt_obex_add_header_utf8((UINT8 *)p_obx->p_pkt, OBEX_HI_NAME, (UINT8 *)p_chdir->p_dir))
            {
                utl_freebuf((void**)&p_obx->p_pkt);
                status = WICED_BT_MA_STATUS_FAIL;
            }
        }
    }
    if (status == WICED_BT_MA_STATUS_OK)
    {
        if (wiced_bt_obex_send_request(p_icb->obx_handle, OBEX_REQ_SETPATH, (wiced_bt_obex_req_param_t*)&obx_flags, (UINT8 *)p_obx->p_pkt))
        {
            status = WICED_BT_MA_STATUS_FAIL;
        }
        else
        {
            p_icb->req_pending = TRUE;
            p_icb->obx_oper = WICED_MCE_OP_CHDIR;
            p_obx->p_pkt = NULL;    /* OBX will free the packet */
        }
    }

    if (status != WICED_BT_MA_STATUS_OK)
    {
        app_evt.status = status;
        app_evt.session_id = p_icb->obx_handle;
        app_evt.obx_rsp_code = OBEX_RSP_OK;
        p_cb->p_cback(WICED_BT_MCE_SET_FOLDER_EVT, (wiced_bt_mce_t *)&app_evt);
    }
}
/*******************************************************************************
**
** Function         wiced_mce_ma_list
**
** Description      Action routine for API get folder list event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_list(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);

    APPL_TRACE_EVENT0("wiced_mce_ma_list\n");

    /* Only process if no other OBX operation is active and connected to MAS */
    if (p_icb->obx_oper == WICED_MCE_OP_NONE)
        wiced_mce_get_listing(p_icb, &p_data->api_list);
    else
        wiced_mce_listing_err(p_icb, &p_icb->obx.p_pkt, WICED_BT_MA_STATUS_FAIL, OBEX_RSP_OK);
}
/*******************************************************************************
**
** Function         wiced_mce_ma_get_msg
**
** Description      Action routine for API get message event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_get_msg(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t          *p_cb = &wiced_mce_cb;
    wiced_mce_inst_cb_t     *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_mce_obx_pkt_t     *p_obx = WICED_MCE_GET_MA_OBX_PKT_PTR(ccb_idx, icb_idx);
    wiced_mce_api_get_msg_t *p_getmsg = &p_data->api_getmsg;
    wiced_bt_ma_status_t    status = WICED_BT_MA_STATUS_FAIL;
    wiced_bt_mce_get_msg_t  app_evt;
    UINT8                   *p, *p_start;
    UINT16                  len = 0;
    char                    handle_str[WICED_MCE_HANDLE_STR_SIZE];
    char                    file_path[WICED_MCE_FILE_PATH_SIZE];

    APPL_TRACE_EVENT0("wiced_mce_ma_get_msg\n");

    if (p_icb->obx_oper == WICED_MCE_OP_NONE)
    {
        p_icb->first_req_pkt = TRUE;
        p_icb->frac_deliver  = WICED_BT_MA_FRAC_DELIVER_NO;
        p_obx->offset = 0;

        memcpy(handle_str, p_getmsg->param.handle, WICED_BT_MA_HANDLE_SIZE);

        if ((p_obx->p_pkt = (BT_HDR  *)wiced_bt_obex_header_init(p_icb->obx_handle, OBX_CMD_POOL_SIZE)) != NULL)
        {
            status = WICED_BT_MA_STATUS_OK;

            memcpy(p_icb->msg_handle, p_getmsg->param.handle, sizeof(wiced_bt_ma_msg_handle_t));

            /* Add the name header if not backing up */
            if (!wiced_bt_obex_add_header_utf8((UINT8 *)p_obx->p_pkt, OBEX_HI_NAME, (UINT8 *)handle_str))
            {
                utl_freebuf((void**)&p_obx->p_pkt);
                status = WICED_BT_MA_STATUS_FAIL;
            }
            else
            {
                /* add type header */
                p = (UINT8*)WICED_BT_MA_HDR_TYPE_MSG;
                wiced_bt_obex_add_header((uint8_t *)p_obx->p_pkt, OBEX_HI_TYPE, p, strlen((char *)p) + 1);

                /* add application headers */
                p_start = wiced_bt_obex_add_byte_sequence_start((uint8_t *)p_obx->p_pkt, &len);
                p= p_start;

                *p ++ = WICED_BT_MA_APH_ATTACH;
                *p ++ = 1;
                *p ++ = p_getmsg->param.attachment ? WICED_BT_MA_ATTACH_ON : WICED_BT_MA_ATTACH_OFF;

                *p ++ = WICED_BT_MA_APH_CHARSET;
                *p ++ = 1;
                *p ++ = p_getmsg->param.charset;

                if (p_getmsg->param.fraction_request < WICED_BT_MA_FRAC_REQ_NO)
                {
                    *p ++ = WICED_BT_MA_APH_FRAC_REQ;
                    *p ++ = 1;
                    *p ++ = p_icb->frac_req = p_getmsg->param.fraction_request;
                }
                wiced_bt_obex_add_byte_sequence_end((uint8_t *)p_obx->p_pkt, OBEX_HI_APP_PARMS, (UINT16)(p - p_start));
            }
        }

        if (status == WICED_BT_MA_STATUS_OK)
        {
            p_icb->obx_oper = WICED_MCE_OP_GET_MSG;
            wiced_mce_send_get_req(p_icb);
        }
    }

    if (status != WICED_BT_MA_STATUS_OK)
    {
        utl_freebuf((void**)&p_obx->p_pkt);
        app_evt.status = status;
        app_evt.session_id = p_icb->obx_handle;
        app_evt.obx_rsp_code = OBEX_RSP_OK;
        p_cb->p_cback(WICED_BT_MCE_GET_MSG_EVT, (wiced_bt_mce_t *)&app_evt);
    }
}

#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
/*******************************************************************************
**
** Function         wiced_mce_ma_get_mas_ins_info
**
** Description      Action routine for API get MAS instance information event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_get_mas_ins_info(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t                  *p_cb = &wiced_mce_cb;
    wiced_mce_ma_cb_t               *p_ccb = WICED_MCE_GET_MA_CB_PTR(ccb_idx);
    wiced_mce_inst_cb_t             *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_mce_obx_pkt_t             *p_obx = WICED_MCE_GET_MA_OBX_PKT_PTR(ccb_idx, icb_idx);
    wiced_mce_api_get_mas_ins_info_t *p_get_mas_ins_info = &p_data->api_get_mas_ins_info;
    wiced_bt_ma_status_t            status = WICED_BT_MA_STATUS_FAIL;
    wiced_bool_t                    final = WICED_TRUE;
    wiced_bt_mce_get_mas_ins_info_t app_evt;
    tOBEX_TRIPLET                    app_param;
    UINT8                           *p;

    if ( p_icb->mce_peer_features & WICED_BT_MA_SUP_FEA_INST_INFO)
    {
        APPL_TRACE_EVENT0("wiced_mce_ma_get_mas_ins_info \n");
        /* Save the instance ID */
        p_ccb->get_mas_inst_id = p_get_mas_ins_info->mas_instance_id;
        if (p_icb->obx_oper == WICED_MCE_OP_NONE)
        {
            if ((p_obx->p_pkt = (BT_HDR  *)wiced_bt_obex_header_init(p_icb->obx_handle, OBX_CMD_POOL_SIZE)) != NULL)
            {
                /* add type header */
                p = (UINT8 *)WICED_BT_MA_HDR_TYPE_MAS_INS_INFO;
                wiced_bt_obex_add_header((uint8_t *)p_obx->p_pkt, OBEX_HI_TYPE, (uint8_t *)p, strlen((char *)p) + 1);

                /* add app params */
                app_param.tag = WICED_BT_MA_APH_MAS_INST_ID;
                app_param.len = 1;
                app_param.p_array = &(p_get_mas_ins_info->mas_instance_id);
                wiced_bt_obex_add_header((uint8_t *)p_obx->p_pkt, OBEX_HI_APP_PARMS, (uint8_t *)&app_param, sizeof(app_param));
            }

            /* Send out get request */
            if (wiced_bt_obex_send_request(p_icb->obx_handle, OBEX_REQ_GET, (wiced_bt_obex_req_param_t*)&final, (uint8_t *)p_obx->p_pkt) == OBEX_SUCCESS)
            {
                p_icb->obx_oper = WICED_MCE_OP_GET_MAS_INS_INFO;

                p_icb->req_pending = TRUE;
                status = WICED_BT_MA_STATUS_OK;
                p_obx->p_pkt = NULL;
            }  /* getreq */

        }
    }
    else
    {
        APPL_TRACE_WARNING0("wiced_mce_ma_get_mas_ins_info not supported by the peer\n");
        status = WICED_BT_MA_STATUS_UNSUP_FEATURES;
    }

    if (status != WICED_BT_MA_STATUS_OK)
    {
        utl_freebuf((void**)&p_obx->p_pkt);
        app_evt.status = status;
        app_evt.session_id = p_icb->obx_handle;
        app_evt.mas_instance_id = p_ccb->get_mas_inst_id;
        /* Set descriptor to NULL */
        app_evt.mas_ins_info[0] = '\0';
        app_evt.obx_rsp_code = OBEX_RSP_OK;
        p_cb->p_cback(WICED_BT_MCE_GET_MAS_INS_INFO, (wiced_bt_mce_t *)&app_evt);
    }
}
#endif /* #if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE) */

/*******************************************************************************
**
** Function         wiced_mce_ma_set_sts
**
** Description       Action routine for API set message status event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_set_sts(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t          *p_cb = &wiced_mce_cb;
    wiced_mce_inst_cb_t     *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_mce_obx_pkt_t     *p_obx = WICED_MCE_GET_MA_OBX_PKT_PTR(ccb_idx, icb_idx);
    wiced_bt_ma_status_t    status = WICED_BT_MA_STATUS_FAIL;
    wiced_bool_t            final = WICED_TRUE;
    UINT8                   *p, *p_start;
    UINT16                  len = 0;
    wiced_bt_mce_set_msg_status_t app_evt;
    char                    handle_str[WICED_MCE_HANDLE_STR_SIZE];

    APPL_TRACE_EVENT0("wiced_mce_ma_set_sts\n");

    if (p_icb->obx_oper == WICED_MCE_OP_NONE)
    {
        if ((p_obx->p_pkt = (BT_HDR  *)wiced_bt_obex_header_init(p_icb->obx_handle, OBX_CMD_POOL_SIZE)) != NULL)
        {
            memcpy(handle_str, p_data->api_setstatus.msg_handle, WICED_BT_MA_HANDLE_SIZE);

            /* Add the Name Header to the request */
            if (wiced_bt_obex_add_header_utf8((UINT8 *)p_obx->p_pkt, OBEX_HI_NAME, (UINT8 *)handle_str))
            {
                /* add type header */
                p = (UINT8 *)WICED_BT_MA_HDR_TYPE_MSG_STATUS;
                wiced_bt_obex_add_header((uint8_t *)p_obx->p_pkt, OBEX_HI_TYPE, p, strlen((char *)p) + 1);

                /* add app params for Notification Status */
                p_start = wiced_bt_obex_add_byte_sequence_start((uint8_t *)p_obx->p_pkt, &len);
                p = p_start;

                /* add status indicator header */
                *p++    = WICED_BT_MA_APH_STS_INDCTR;
                *p++    = 1;
                *p++    = p_data->api_setstatus.indicator;

                /* add status value indicator header */
                *p++    = WICED_BT_MA_APH_STS_VALUE;
                *p++    = 1;
                *p++    = p_data->api_setstatus.value;

                /* here param header is added */
                wiced_bt_obex_add_byte_sequence_end((uint8_t *)p_obx->p_pkt, OBEX_HI_APP_PARMS, (UINT16)(p - p_start));

                /* add filler byte */
                wiced_mce_add_filler_byte(p_obx);

                /* Send out the data */
                if (wiced_bt_obex_send_request(p_icb->obx_handle, OBEX_REQ_PUT, (wiced_bt_obex_req_param_t*)&final, (uint8_t *)p_obx->p_pkt) == OBEX_SUCCESS)
                {
                    p_icb->obx_oper = WICED_MCE_OP_SET_MSG_STS;

                    p_icb->req_pending = TRUE;
                    status = WICED_BT_MA_STATUS_OK;
                    p_obx->p_pkt = NULL;
                } /* putreq */
            }/* add name header */
            else
                utl_freebuf((void**)&p_obx->p_pkt);
        }/* init OBX package */
    }

    if (status != WICED_BT_MA_STATUS_OK)
    {
        app_evt.status = status;
        app_evt.session_id = p_icb->obx_handle;
        app_evt.obx_rsp_code = OBEX_RSP_OK;
        p_cb->p_cback(WICED_BT_MCE_SET_MSG_STATUS_EVT, (wiced_bt_mce_t *)&app_evt);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_proc_put_rsp
**
** Description      Process the PUT response for single package PUT transaction.
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_proc_put_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t      *p_cb = &wiced_mce_cb;
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_bt_mce_t      evt_data;
    wiced_bt_ma_status_t status = WICED_BT_MA_STATUS_FAIL;
    UINT8               evt = WICED_BT_MCE_INVALID_EVT;

    APPL_TRACE_EVENT0("wiced_mce_proc_put_rsp\n");

    if (p_data->obx_evt.rsp_code == OBEX_RSP_OK)
    {
        status = WICED_BT_MA_STATUS_OK;
    }

    switch ( p_icb->obx_oper )
    {
        case WICED_MCE_OP_UPD_INBOX:
            evt = WICED_BT_MCE_UPDATE_INBOX_EVT;
            evt_data.upd_ibx.session_id = p_icb->obx_handle;
            evt_data.upd_ibx.status = status;
            evt_data.upd_ibx.obx_rsp_code = p_data->obx_evt.rsp_code;
            break;

        case WICED_MCE_OP_NOTIF_REG:
            evt = WICED_BT_MCE_NOTIF_REG_EVT;
            evt_data.notif_reg.session_id = p_icb->obx_handle;
            evt_data.notif_reg.status = status;
            evt_data.notif_reg.obx_rsp_code = p_data->obx_evt.rsp_code;
            break;

        case WICED_MCE_OP_SET_MSG_STS:
            evt = WICED_BT_MCE_SET_MSG_STATUS_EVT;
            evt_data.set_msg_sts.session_id = p_icb->obx_handle;
            evt_data.set_msg_sts.status = status;
            evt_data.set_msg_sts.obx_rsp_code = p_data->obx_evt.rsp_code;
            break;
        default:
            APPL_TRACE_EVENT1("Unknown operation : %d\n", p_icb->obx_oper);
            break;
    }

    if (evt != WICED_BT_MCE_INVALID_EVT)
    {
        p_cb->p_cback(evt, (wiced_bt_mce_t *)&evt_data);
        p_icb->obx_oper = WICED_MCE_OP_NONE;
    }
}

/*******************************************************************************
**
** Function         wiced_mce_proc_push_msg_rsp
**
** Description      Process the PUT response for PUSH message transaction.
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_proc_push_msg_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t      *p_cb = &wiced_mce_cb;
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_mce_obx_evt_t *p_evt = &p_data->obx_evt;
    wiced_bt_mce_push_msg_t evt_data;

    APPL_TRACE_EVENT0("wiced_mce_proc_push_msg_rsp\n");

    memset(&evt_data, 0, sizeof(wiced_bt_mce_push_msg_t));

    if (p_evt->rsp_code == OBEX_RSP_OK || p_evt->rsp_code == OBEX_RSP_CONTINUE)
        evt_data.status = WICED_BT_MA_STATUS_OK;
    else
        evt_data.status = WICED_BT_MA_STATUS_FAIL;
    evt_data.session_id = p_icb->obx_handle;

    if (wiced_bt_obex_read_header_utf8((uint8_t *)p_evt->p_pkt, OBEX_HI_NAME, (UINT8 *)p_icb->msg_hdl_str,
                            WICED_BT_MA_64BIT_HEX_STR_SIZE) == OBEX_SUCCESS)
    {
        memcpy(evt_data.msg_handle, p_icb->msg_hdl_str, WICED_BT_MA_HANDLE_SIZE);
    }

    p_cb->p_cback(WICED_BT_MCE_PUSH_MSG_EVT, (wiced_bt_mce_t *)&evt_data);
    p_icb->obx_oper = WICED_MCE_OP_NONE;
}
/*******************************************************************************
**
** Function         wiced_mce_ma_put_rsp
**
** Description      Action routine for Obex put response event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_put_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);

    APPL_TRACE_EVENT0("wiced_mce_ma_put_rsp\n");

    p_icb->req_pending = FALSE;
    p_icb->first_req_pkt = FALSE;

    switch (p_icb->obx_oper)
    {
        case WICED_MCE_OP_NOTIF_REG:
        case WICED_MCE_OP_SET_MSG_STS:
        case WICED_MCE_OP_UPD_INBOX:
            wiced_mce_proc_put_rsp(ccb_idx, icb_idx, p_data);
            break;
        case WICED_MCE_OP_PUSH_MSG:
            wiced_mce_proc_push_msg_rsp(ccb_idx, icb_idx, p_data);
            break;
        default:
            APPL_TRACE_EVENT1("Unknown operation : %d\n", p_icb->obx_oper);
            break;
    }

    /* Done with Obex packet */
    utl_freebuf((void**)&p_data->obx_evt.p_pkt);
}

/*******************************************************************************
**
** Function         wiced_mce_ma_obx_get_rsp
**
** Description      Action routine for Obex get response event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_obx_get_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);

    APPL_TRACE_EVENT0("wiced_mce_ma_obx_get_rsp\n");

    if (p_icb->obx_oper == WICED_MCE_OP_FOLDER_LIST ||
        p_icb->obx_oper == WICED_MCE_OP_MSG_LIST)
    {
        wiced_mce_proc_get_list_rsp(ccb_idx, icb_idx, &p_data->obx_evt);
    }
    else if (p_icb->obx_oper == WICED_MCE_OP_GET_MSG)
    {
        wiced_mce_proc_get_msg_rsp(ccb_idx, icb_idx, p_data);
    }
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
    else if (p_icb->obx_oper == WICED_MCE_OP_GET_MAS_INS_INFO)
    {
        wiced_mce_proc_get_mas_ins_info_rsp(ccb_idx, icb_idx, p_data);
    }
#endif
    else
        /* Release the unexpected OBX response packet */
        utl_freebuf((void**)&p_data->obx_evt.p_pkt);
}
/*******************************************************************************
**
** Function         wiced_mce_ma_obx_setpath_rsp
**
** Description      Action routine for Obex setpath response event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_obx_setpath_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t      *p_cb = &wiced_mce_cb;
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_mce_obx_evt_t *p_evt = &p_data->obx_evt;
    wiced_bt_mce_t      evt_data;

    APPL_TRACE_EVENT0("wiced_mce_ma_obx_setpath_rsp \n");

    p_icb->req_pending = FALSE;

    evt_data.set_folder.status = WICED_BT_MA_STATUS_FAIL;
    if (p_evt->rsp_code == OBEX_RSP_OK)
        evt_data.set_folder.status = WICED_BT_MA_STATUS_OK;

    evt_data.set_folder.obx_rsp_code = p_evt->rsp_code;
    evt_data.set_folder.session_id = p_icb->obx_handle;

    p_icb->obx_oper = WICED_MCE_OP_NONE;

    /* Done with Obex packet */
    utl_freebuf((void**)&p_evt->p_pkt);
    p_cb->p_cback(WICED_BT_MCE_SET_FOLDER_EVT, (wiced_bt_mce_t *)&evt_data);
}

/*******************************************************************************
**
** Function         wiced_mce_ma_close_compl
**
** Description     Action routine for close complete event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_close_compl(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t          *p_cb = &wiced_mce_cb;
    wiced_mce_ma_cb_t       *p_ccb = WICED_MCE_GET_MA_CB_PTR(ccb_idx);
    wiced_mce_inst_cb_t     *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_bt_mce_ma_open_close_t param;

    APPL_TRACE_EVENT1("wiced_mce_ma_close_compl is_opening=%d\n", p_icb->is_opening);
    if (p_ccb->active_count > 0)
        p_ccb->active_count--;

    APPL_TRACE_EVENT1("wiced_mce_ma_close_compl active_count %d\n", p_ccb->active_count);
    /* inform role manager */
    if (p_ccb->active_count == 0)
    {
        p_ccb->in_use = FALSE;
        //wiced_mce_pm_conn_close(p_ccb->bd_addr);
        if (p_ccb->p_db)
        {
            APPL_TRACE_EVENT0("wiced_mce_ma_close_compl clean database\n");
            utl_freebuf((void **)&p_ccb->p_db);
        }
    }

    param.session_id = p_icb->obx_handle;
    param.status = p_icb->status;

    memcpy(param.bd_addr, p_ccb->bd_addr, BD_ADDR_LEN);
    if (p_ccb->p_devname != NULL)
        BCM_STRNCPY_S((char *)param.dev_name, sizeof(param.dev_name), p_ccb->p_devname, BD_NAME_LEN);

    /* If Obex is closed due to open failure, send open event with fail status to the upper layer*/
    if (p_icb->is_opening == TRUE)
    {
        wiced_mce_send_ma_open_evt(ccb_idx, icb_idx, WICED_BT_MA_STATUS_FAIL);
        p_icb->is_opening = FALSE;
    }
    else
    /* otherwise send close event to the upper layer */
    {
        p_cb->p_cback(WICED_BT_MCE_MA_CLOSE_EVT, (wiced_bt_mce_t *)&param);
    }
    wiced_mce_initialize(ccb_idx, icb_idx, p_data);
}
/*******************************************************************************
**
** Function         wiced_mce_ma_rsp_tout    .
**
** Description      Action routine for response timer timeout event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_rsp_tout(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_ma_cb_t   *p_ccb = WICED_MCE_GET_MA_CB_PTR(ccb_idx);
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);

    APPL_TRACE_EVENT0("wiced_mce_ma_rsp_tout\n");

    if (p_icb->timer_oper == WICED_MCE_TIMER_OP_ABORT)
    {
        /* Start stop response timer */
        p_icb->timer_oper = WICED_MCE_TIMER_OP_STOP;
        p_ccb->status = WICED_BT_MA_STATUS_ABORTED;
        wiced_start_timer(&p_icb->rsp_timer, 2);

        wiced_bt_obex_disconnect(p_icb->obx_handle, NULL);
    }
    else    /* Timeout waiting for disconnect response */
    {
        wiced_mce_ma_sm_execute(ccb_idx, icb_idx, WICED_MCE_MA_CLOSE_CMPL_EVT, p_data);
    }
}
/*******************************************************************************
**
** Function         wiced_mce_obx_abort_rsp
**
** Description      Action routine for OBX abort event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_obx_abort_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t      *p_cb = &wiced_mce_cb;
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_bt_mce_t      param;

    APPL_TRACE_EVENT0("wiced_mce_obx_abort_rsp\n");

    wiced_stop_timer(&p_icb->rsp_timer);

    /* Done with Obex packet */
    utl_freebuf((void**)&p_data->obx_evt.p_pkt);

    param.abort.status = WICED_BT_MA_STATUS_OK;
    param.abort.session_id = p_icb->obx_handle;
    param.abort.obx_rsp_code = p_data->obx_evt.rsp_code;
    p_cb->p_cback(WICED_BT_MCE_ABORT_EVT, (wiced_bt_mce_t *)&param);

    if (p_icb->obx_oper != WICED_MCE_OP_NONE)
    {
        /* Mark the fact we have already received the response from the peer */
        p_icb->aborting |= WICED_MCE_ABORT_RSP_RCVD;
        /* OBX_RSP_GONE indicates aborted */
        p_data->obx_evt.rsp_code = (!p_icb->int_abort) ? OBEX_RSP_GONE : OBEX_RSP_INTRNL_SRVR_ERR;
        wiced_mce_ma_sm_execute(ccb_idx, icb_idx, WICED_MCE_MA_OBX_CMPL_EVT, p_data);
    }
}
/*******************************************************************************
**
** Function         wiced_mce_ma_trans_cmpl
**
** Description      Action routine for OBX transaction complete event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_trans_cmpl(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t              *p_cb = &wiced_mce_cb;
    wiced_mce_inst_cb_t         *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_bt_mce_get_msg_t      gm_param;
    wiced_bt_mce_push_msg_t     pm_param;
    wiced_bt_ma_status_t        status;
    wiced_bt_ma_sess_handle_t   session_id;

    APPL_TRACE_EVENT0("wiced_mce_ma_trans_cmpl\n");

    status = wiced_mce_convert_obx_to_status(p_data->obx_evt.rsp_code);
    session_id = p_icb->obx_handle;

    switch (p_icb->obx_oper)
    {
        case WICED_MCE_OP_GET_MSG:
            gm_param.status = status;
            gm_param.session_id = session_id;
            gm_param.frac_deliver = p_icb->frac_deliver;
            if (gm_param.status != WICED_BT_MA_STATUS_OK)
            {
                /* ---- fpushmsg
                TODO: discard received data ---- */
                APPL_TRACE_WARNING2("MCE: GetMsg Operation Aborted or Error [%s], status 0x%02x\n",
                                    p_icb->msg_handle, gm_param.status);
            }
            gm_param.obx_rsp_code = p_data->obx_evt.rsp_code;
            p_cb->p_cback(WICED_BT_MCE_GET_MSG_EVT, (wiced_bt_mce_t *)&gm_param);
            break;

        case WICED_MCE_OP_PUSH_MSG:
            pm_param.status = status;
            pm_param.session_id = session_id;
            memcpy( pm_param.msg_handle, p_icb->msg_handle, WICED_BT_MA_HANDLE_SIZE);
            pm_param.obx_rsp_code = p_data->obx_evt.rsp_code;
            p_cb->p_cback(WICED_BT_MCE_PUSH_MSG_EVT, (wiced_bt_mce_t *)&pm_param);
            break;

        default:
            break;
    }
    /* Clean up control block */
    wiced_mce_reset_icb(p_icb);
}

/*******************************************************************************
**
** Function         wiced_mce_initialize
**
** Description      Initialize the control block.
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_initialize(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t      *p_cb = &wiced_mce_cb;
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);

    APPL_TRACE_EVENT0("wiced_mce_initialize\n");

    wiced_stop_timer(&p_icb->rsp_timer);

    /* reset instance control block */
    wiced_mce_reset_icb(p_icb);
    p_icb->in_use = FALSE;

    if (p_cb->disabling && !wiced_mce_ma_get_num_of_act_conn(p_cb))
    {
        wiced_mce_disable_complete(p_cb);
        p_cb->disabling = FALSE;
    }
}

/*****************************************************************************
** Function         wiced_mce_search_sdp_db
**
** Description      This function search the existing SDP database to look for
**                  a instance ID and related information to open a MAP
**                  connection
**
** Returns          Nothing.
**
******************************************************************************/
static void wiced_mce_search_sdp_db(UINT8 ccb_idx, wiced_bt_ma_inst_id_t mas_instance_id)
{
    wiced_mce_sdp_evt_t *p_buf;
    wiced_bt_sdp_discovery_record_t *p_rec = NULL;
    wiced_bt_sdp_discovery_attribute_t *p_attr;
    wiced_bt_sdp_protocol_elem_t pe;
    wiced_mce_ma_cb_t   *p_ccb = WICED_MCE_GET_MA_CB_PTR(ccb_idx);
    UINT16              version = WICED_BT_MA_VERSION_1_0;
    UINT16              psm = 0;
    UINT8               scn = 0, inst_id = 0;
    BOOLEAN             found = FALSE;
    UINT8               cnt=0;
    UINT32              peer_features = WICED_BT_MA_DEFAULT_SUPPORTED_FEATURES;

    APPL_TRACE_EVENT1("wiced_mce_search_sdp_db inst_id:%d\n", inst_id);

    /* loop through all records we found */
    do
    {
        /* get next record; if none found, we're done */
        if ((p_rec = wiced_bt_sdp_find_service_in_db(p_ccb->p_db, UUID_SERVCLASS_MESSAGE_ACCESS, p_rec)) == NULL)
        {
            APPL_TRACE_EVENT0("wiced_mce_search_sdp_db no record found --failed\n");
            break;
        }
        else
        {
            APPL_TRACE_EVENT1("wiced_mce_search_sdp_db found rec cnt=%d\n",cnt);
        }

        inst_id = 0;
        if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_INSTANCE_ID)) != NULL)
        {
            inst_id = p_attr->attr_value.v.u8;
            APPL_TRACE_EVENT1("wiced_mce_search_sdp_db found inst_id=%d\n",inst_id);
        }

        if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_SUPPORTED_MSG_TYPE)) != NULL)
        {
            APPL_TRACE_EVENT1("wiced_mce_search_sdp_db found supported msg type=%d\n",p_attr->attr_value.v.u8);
        }

        if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_SERVICE_NAME)) != NULL)
        {
            APPL_TRACE_EVENT1("wiced_mce_search_sdp_db found service name=%s\n",p_attr->attr_value.v.array);
        }

        APPL_TRACE_EVENT1("wiced_mce_search_sdp_db desired inst_id=%d\n", mas_instance_id);
        if (inst_id == mas_instance_id)
        {
            /* this is a mandatory attribute */
            wiced_bt_sdp_find_profile_version_in_rec (p_rec, UUID_SERVCLASS_MAP_PROFILE, &version);

#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
            /* If profile version is 1.2 or greater, look for supported features and L2CAP PSM */
            if (version >= WICED_BT_MA_VERSION_1_2)
            {

                /* Look for peer supported features */
                if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_SUPPORTED_FEATURES_32)) != NULL)
                {
                    peer_features = p_attr->attr_value.v.u32;
                    APPL_TRACE_DEBUG1("wiced_mce_search_sdp_db peer_features=0x%08x\n", peer_features);
                }
                /* No supported features, use the default */
                else
                {
                    APPL_TRACE_DEBUG0("wiced_mce_search_sdp_db peer supported features not found use default\n");
                }

                /* Look for L2CAP PSM */
                if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_OBX_OVR_L2CAP_PSM)) != NULL)
                {
                    psm = p_attr->attr_value.v.u16;
                    if ((SDP_DISC_ATTR_LEN(p_attr->attr_len_type) == 2) && L2C_IS_VALID_PSM(psm))
                    {
                        /* Both supported features and PSM found, done! */
                        /* Supported feature can be either from peer or default one */
                        found = TRUE;
                        APPL_TRACE_DEBUG1("wiced_mce_search_sdp_db psm=0x%02x (Using MAP 1.2 or later)\n",psm);
                        break;
                    }
                    /* Bad PSM, look for next record */
                    else
                    {
                        APPL_TRACE_WARNING1("wiced_mce_search_sdp_db BAD psm=0x%02x\n",psm);
                        psm = 0;    /* Reset PSM */
                        continue;
                    }
                }
                /* No L2CAP PSM, look for next record */
                else
                {
                    APPL_TRACE_WARNING0("wiced_mce_search_sdp_db PSM for L2CAP not found\n");
                    continue;
                }

            }
#endif /* BTA_MAP_1_2_SUPPORTED */
            /* get scn from proto desc list; if not found, go to next record */
            if (!found && wiced_bt_sdp_find_protocol_list_elem_in_rec(p_rec, UUID_PROTOCOL_RFCOMM, &pe))
            {
                found = TRUE;
                scn = (UINT8) pe.params[0];
                /* we've got the one, we're done */
                APPL_TRACE_EVENT1("wiced_mce_search_sdp_db scn=%d\n",scn);
                break;
            }
            else
            {
                continue;
            }
        }
        cnt++;
        APPL_TRACE_EVENT1("wiced_mce_search_sdp_db cnt=%d\n",cnt);
    } while (TRUE);

    /* send result in event back */
    if ((p_buf = (wiced_mce_sdp_evt_t *) GKI_getbuf(sizeof(wiced_mce_sdp_evt_t))) != NULL)
    {
        p_buf->hdr.event = WICED_MCE_SDP_FAIL_EVT;
        p_buf->ccb_idx = ccb_idx;
        p_buf->mas_instance_id = mas_instance_id;

        if (found)
        {
            p_buf->hdr.event = WICED_MCE_SDP_OK_EVT;
            p_buf->scn = scn;
            p_buf->psm = psm;
            p_buf->peer_features = peer_features;
        }

        wiced_mce_send_event((BT_HDR *)p_buf);
    }
}

/*****************************************************************************
** Function         wiced_mce_sdp_cback
**
** Description      This is the SDP callback function used by MCE.
**                  This function will be executed by SDP when the service
**                  search is completed.  If the search is successful, it
**                  finds the first record in the database that matches the
**                  UUID of the search.  Then retrieves the scn and psm from the
**                  record.
**
** Returns          Nothing.
**
******************************************************************************/
static void wiced_mce_sdp_cback(UINT8 ccb_idx, UINT16 status)
{
    wiced_mce_ma_cb_t   *p_ccb = NULL;
    wiced_mce_sdp_evt_t *p_buf;
    wiced_mce_inst_cb_t *p_icb = NULL;
    wiced_bt_sdp_discovery_record_t *p_rec = NULL;
    wiced_bt_sdp_discovery_attribute_t *p_attr;
    wiced_bt_sdp_protocol_elem_t pe;
    UINT16              version = WICED_BT_MA_VERSION_1_1;  /* if missing, we'll assume 1.1 */
    UINT16              psm = 0;
    UINT8               scn = 0, inst_id = 0;
    BOOLEAN             found = FALSE;
    UINT8               cnt = 0;
    UINT32              peer_features = WICED_BT_MA_DEFAULT_SUPPORTED_FEATURES;
    UINT8               i;

    if (ccb_idx >= WICED_BT_MCE_NUM_MA)
    {
        APPL_TRACE_WARNING2("[%s] CCB_IDX(%d) out of range ",__FUNCTION__,ccb_idx);
        return;
    }
    p_ccb = WICED_MCE_GET_MA_CB_PTR(ccb_idx);
    APPL_TRACE_EVENT1("wiced_mce_sdp_cback status:%d\n", status);

    if ((p_icb = wiced_mce_get_mce_inst_cb_using_inst_id(ccb_idx, p_ccb->sdp_inst_id)) != NULL)
    {
        if (status == WICED_BT_SDP_SUCCESS)
        {
            /* loop through all records we found */
            do
            {
                /* get next record; if none found, we're done */
                if ((p_rec = wiced_bt_sdp_find_service_in_db(p_ccb->p_db, UUID_SERVCLASS_MESSAGE_ACCESS, p_rec)) == NULL)
                {
                    APPL_TRACE_EVENT0("wiced_mce_sdp_cback no record found --failed\n");
                    break;
                }
                else
                {
                    APPL_TRACE_EVENT1("wiced_mce_sdp_cback found rec cnt=%d\n",cnt);
                }

                inst_id = 0;
                if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_INSTANCE_ID)) != NULL)
                {
                    inst_id = p_attr->attr_value.v.u8;
                    APPL_TRACE_EVENT1("wiced_mce_sdp_cback found inst_id=%d\n",inst_id);
                }

                if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_SUPPORTED_MSG_TYPE))!= NULL)
                {
                    APPL_TRACE_EVENT1("wiced_mce_sdp_cback found supported msg type=%d\n",p_attr->attr_value.v.u8);
                }

                if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_SERVICE_NAME))!= NULL)
                {
                    APPL_TRACE_EVENT1("wiced_mce_sdp_cback found service name=%s\n",p_attr->attr_value.v.array);
                }

                APPL_TRACE_EVENT1("wiced_mce_sdp_cback desired inst_id=%d\n",p_icb->inst_id);
                if (inst_id == p_icb->inst_id)
                {
                    /* this is a mandatory attribute */
                    wiced_bt_sdp_find_profile_version_in_rec (p_rec, UUID_SERVCLASS_MAP_PROFILE, &version);
                    APPL_TRACE_DEBUG1("wiced_mce_sdp_cback() MAP peer version = %x\n", version);
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
                    /* If profile version is 1.2 or greater, look for supported features and L2CAP PSM */
                    if (version >= WICED_BT_MA_VERSION_1_2)
                    {
                        /* Look for peer supported features */
                        if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_SUPPORTED_FEATURES_32)) != NULL)
                        {
                            peer_features = p_attr->attr_value.v.u32;
                            APPL_TRACE_DEBUG1("wiced_mce_sdp_cback peer_features=0x%08x\n", peer_features);

                            /* BSA_SPECIFIC */
                            if (peer_features & WICED_BT_MA_SUP_FEA_UPLOADING)
                            {
                                APPL_TRACE_DEBUG0("wiced_mce_sdp_cback() Message Uploading supported\n");
                            }
                            else
                            {
                                APPL_TRACE_DEBUG0("wiced_mce_sdp_cback() Message Uploading Not supported\n");
                            }
                        }
                        /* No supported features, use the default */
                        else
                        {
                            APPL_TRACE_DEBUG0("wiced_mce_sdp_cback peer supported features not found use default\n");
                        }

                        /* Look for L2CAP PSM */
                        if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec, ATTR_ID_OBX_OVR_L2CAP_PSM)) != NULL)
                        {
                            psm = p_attr->attr_value.v.u16;
                            if ((SDP_DISC_ATTR_LEN(p_attr->attr_len_type) == 2) && L2C_IS_VALID_PSM(psm))
                            {
                                /* Both supported features and PSM found, done! */
                                /* Supported feature can be either from peer or default one */
                                found = TRUE;
                                APPL_TRACE_DEBUG1("wiced_mce_sdp_cback psm=0x%02x (Using MAP 1.2 or later)\n",psm);
                                break;
                            }
                            /* Bad PSM, look for next record */
                            else
                            {
                                APPL_TRACE_WARNING1("wiced_mce_sdp_cback BAD psm=0x%02x\n",psm);
                                psm = 0;    /* Reset PSM */
                                continue;
                            }
                        }
                        /* No L2CAP PSM, look for next record */
                        else
                        {
                            APPL_TRACE_WARNING0("wiced_mce_sdp_cback PSM for L2CAP not found\n");
                            continue;
                        }
                    }
#endif /* BTA_MAP_1_2_SUPPORTED */
                    /* get scn from proto desc list; if not found, go to next record */
                    if (!found && wiced_bt_sdp_find_protocol_list_elem_in_rec(p_rec, UUID_PROTOCOL_RFCOMM, &pe))
                    {
                        found = TRUE;
                        scn = (UINT8) pe.params[0];
                        /* we've got the one, we're done */
                        APPL_TRACE_EVENT1("wiced_mce_sdp_cback scn=%d\n",scn);
                        break;
                    }
                    else
                    {
                        continue;
                    }
                }
                cnt++;
                APPL_TRACE_EVENT1("wiced_mce_sdp_cback cnt=%d\n",cnt);

            } while (TRUE);
        }

        /* send result back to MCE MA state machine */
        if ((p_buf = (wiced_mce_sdp_evt_t *) GKI_getbuf(sizeof(wiced_mce_sdp_evt_t))) != NULL)
        {
            p_buf->hdr.event = WICED_MCE_SDP_FAIL_EVT;
            p_buf->ccb_idx = ccb_idx;
            p_buf->mas_instance_id = p_icb->inst_id;

            if (status == WICED_BT_SDP_SUCCESS)
            {
                if (found)
                {
                    p_buf->hdr.event = WICED_MCE_SDP_OK_EVT;
                    p_buf->scn = scn;
                    p_buf->psm = psm;
                    p_buf->peer_features = peer_features;
                }
            }

            wiced_mce_send_event((BT_HDR *)p_buf);
        }
    }

    /* Looking for pending SDP instance control block and making connection */
    for (i = 0; i < WICED_BT_MCE_NUM_INST; i++)
    {
        if (p_ccb->icb[i].sdp_pending == TRUE && p_ccb->p_db)
        {
            wiced_mce_search_sdp_db(ccb_idx, p_ccb->icb[i].inst_id);
            p_ccb->icb[i].sdp_pending = FALSE;
        }
    }
}

/******************************************************************************
**
** Function         wiced_mce_ma_sdp_cback0
**
** Description      This is the SDP callback function used by MA indx = 0
**
** Parameters       status  - status of the SDP callabck
**
** Returns          void.
**
******************************************************************************/
static void wiced_mce_sdp_cback0(UINT16 status)
{
    wiced_mce_sdp_cback(0, status);
}

/******************************************************************************
**
** Function         wiced_mce_ma_sdp_cback1
**
** Description      This is the SDP callback function used by MA indx = 1
**
** Parameters       status  - status of the SDP callabck
**
** Returns          void.
**
******************************************************************************/
static void wiced_mce_sdp_cback1(UINT16 status)
{
    wiced_mce_sdp_cback(1, status);
}

/******************************************************************************
**
** Function         wiced_mce_ma_sdp_cback2
**
** Description      This is the SDP callback function used by MA indx = 2
**
** Parameters       status  - status of the SDP callabck
**
** Returns          void.
**
******************************************************************************/
static void wiced_mce_sdp_cback2(UINT16 status)
{
    wiced_mce_sdp_cback(2, status);
}

/******************************************************************************
**
** Function         wiced_mce_ma_sdp_cback3
**
** Description      This is the SDP callback function used by MA indx = 3
**
** Parameters       status  - status of the SDP callabck
**
** Returns          void.
**
******************************************************************************/
static void wiced_mce_sdp_cback3(UINT16 status)
{
    wiced_mce_sdp_cback(3, status);
}

/******************************************************************************
**
** Function         wiced_mce_ma_sdp_cback4
**
** Description      This is the SDP callback function used by MA indx = 4
**
** Parameters       status  - status of the SDP callabck
**
** Returns          void.
**
******************************************************************************/
static void wiced_mce_sdp_cback4(UINT16 status)
{
    wiced_mce_sdp_cback(4, status);
}

/******************************************************************************
**
** Function         wiced_mce_ma_sdp_cback5
**
** Description      This is the SDP callback function used by MA indx = 5
**
** Parameters       status  - status of the SDP callabck
**
** Returns          void.
**
******************************************************************************/
static void wiced_mce_sdp_cback5(UINT16 status)
{
    wiced_mce_sdp_cback(5, status);
}

/******************************************************************************
**
** Function         wiced_mce_ma_sdp_cback6
**
** Description      This is the SDP callback function used by MA indx = 6
**
** Parameters       status  - status of the SDP callabck
**
** Returns          void.
**
******************************************************************************/
static void wiced_mce_sdp_cback6(UINT16 status)
{
    wiced_mce_sdp_cback(6, status);
}

/******************************************************************************
**
** Function         wiced_mce_ma_sdp_cback7
**
** Description      This is the SDP callback function used by MA indx = 7
**
** Parameters       status  - status of the SDP callabck
**
** Returns          void.
**
******************************************************************************/
static void wiced_mce_sdp_cback7(UINT16 status)
{
    wiced_mce_sdp_cback(7, status);
}

/*******************************************************************************
**
** Function         wiced_mce_ma_obx_cback
**
** Description      OBX callback function for MCE MAS client.
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_obx_cback (wiced_bt_obex_handle_t handle, wiced_bt_obex_event_t obx_event, UINT8 rsp_code,
                             wiced_bt_obex_evt_param_t param, uint8_t *p_pkt)
{
    wiced_mce_obx_evt_t *p_obx_msg;
    UINT16              event = 0;
    wiced_bt_mce_obex_rsp_t app_evt;
    wiced_mce_inst_cb_t *p_icb = NULL;

    APPL_TRACE_EVENT1("wiced_mce_ma_obx_cback [%d]\n", obx_event);

    switch (obx_event)
    {
        /* MA Client */
        case OBEX_CONNECT_RSP_EVT:
            if (rsp_code == OBEX_RSP_OK)
            {
                event = WICED_MCE_MA_OBX_CONN_RSP_EVT;
            }
            else    /* Obex will disconnect */
            {
                APPL_TRACE_WARNING1("MCE_CBACK: Bad connect response 0x%02x\n", rsp_code);
                if (p_pkt)
                    GKI_freebuf(p_pkt);
                event = WICED_MCE_MA_OBX_CONN_RSP_EVT;
                //return;
            }
            break;
        case OBEX_PUT_RSP_EVT:
            event = WICED_MCE_MA_OBX_PUT_RSP_EVT;
            break;
        case OBEX_GET_RSP_EVT:
            event = WICED_MCE_MA_OBX_GET_RSP_EVT;
            break;
        case OBEX_SETPATH_RSP_EVT:
            event = WICED_MCE_MA_OBX_SETPATH_RSP_EVT;
            break;
        case OBEX_ABORT_RSP_EVT:
            event = WICED_MCE_MA_OBX_ABORT_RSP_EVT;
            break;
        case OBEX_TIMEOUT_EVT:
            event = WICED_MCE_MA_OBX_TOUT_EVT;
            break;
        case OBEX_CLOSE_IND_EVT:
            event = WICED_MCE_MA_OBX_CLOSE_EVT;
            break;

        default:
/*  case OBX_DISCONNECT_RSP_EVT: Handled when OBX_CLOSE_IND_EVT arrives */
            if (p_pkt)
                GKI_freebuf(p_pkt);
            return;
    }

    if (event && ((p_icb = wiced_mce_get_mce_inst_cb_using_obx_handle(handle)) != NULL))
    {
        app_evt.session_id      = (wiced_bt_ma_sess_handle_t) handle;
        app_evt.mas_instance_id = p_icb->inst_id;
        app_evt.rsp_code        = rsp_code;
        switch (event)
        {
            case WICED_MCE_MA_OBX_PUT_RSP_EVT:
                wiced_mce_cb.p_cback(WICED_BT_MCE_OBEX_PUT_RSP_EVT, (wiced_bt_mce_t *)&app_evt);
                break;
            case WICED_MCE_MA_OBX_GET_RSP_EVT:
                wiced_mce_cb.p_cback(WICED_BT_MCE_OBEX_GET_RSP_EVT, (wiced_bt_mce_t *)&app_evt);
                break;
            default:
                break;
        }
    }

    /* send event up, if any */
    if (event && (p_obx_msg =
                  (wiced_mce_obx_evt_t *) GKI_getbuf(sizeof(wiced_mce_obx_evt_t))) != NULL)
    {
#if (WICED_MCE_DEBUG == TRUE) && (BT_USE_TRACES == TRUE)
        APPL_TRACE_EVENT1("OBX Event Callback: ma_obx_event [%s]\n",
                          mce_ma_obx_evt_code(event));
#endif
        p_obx_msg->hdr.event = event;
        p_obx_msg->obx_event = obx_event;
        p_obx_msg->handle = handle;
        p_obx_msg->rsp_code = rsp_code;
        p_obx_msg->param = param;
        p_obx_msg->p_pkt = (BT_HDR *)p_pkt;

        wiced_mce_send_event((BT_HDR *)p_obx_msg);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_ma_ignore_obx
**
** Description      Action routine for ignored OBX event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_ignore_obx(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    APPL_TRACE_EVENT0("wiced_mce_ma_ignore_obx\n");
    utl_freebuf((void**)&p_data->obx_evt.p_pkt);
}

/*******************************************************************************
** Message Notification Server (MCE_MN) Action functions
********************************************************************************/

/*******************************************************************************
**
** Function         wiced_mce_mn_ignore_obx
**
** Description      Action routine for ignored OBX event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_mn_ignore_obx(UINT8 scb_idx, wiced_mce_data_t *p_data)
{
    APPL_TRACE_EVENT0("wiced_mce_mn_ignore_obx\n");
    utl_freebuf((void**)&p_data->obx_evt.p_pkt);
}

/*******************************************************************************
**
** Function         wiced_mce_mn_start_service
**
** Description      MN state machine action routine for API start server event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_mn_start_service(wiced_mce_cb_t *p_cb, wiced_mce_data_t *p_data)
{
    wiced_mce_api_mn_start_t        *p_api = &p_data->api_start;
    wiced_bt_mce_mn_start_stop_t    start_evt;
    wiced_bt_obex_target_t          target;
    wiced_bt_obex_start_params_t    start_msg;
    wiced_bt_ma_status_t    status = WICED_BT_MA_STATUS_FAIL;
    UINT8                   i;

    APPL_TRACE_EVENT0("wiced_mce_mn_start_service\n");

    if (!p_cb->mn_started)
    {
        memcpy(target.target, WICED_MAS_MESSAGE_NOTIFICATION_TARGET_UUID, WICED_MAS_UUID_LENGTH);
        target.len = WICED_MAS_UUID_LENGTH;

        /* store parameters */
        p_cb->mn_scn = p_api->scn; // WICED_BT_MNS_RFCOMM_SCN;     //BTM_AllocateSCN();


#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
        p_cb->mn_psm = p_api->psm; // WICED_BT_MNS_L2CAP_PSM;      //L2CA_AllocatePSM();
        p_cb->mce_local_features = p_api->mce_local_features;
#endif

        /* Start up the MAS service */
        memset(&start_msg, 0, sizeof(wiced_bt_obex_start_params_t));
        start_msg.p_target = &target;

        /* Make the MTU fit into one RFC frame */
        start_msg.mtu = OBEX_MAX_MTU;
        start_msg.scn = p_cb->mn_scn;
        start_msg.p_cback = wiced_mce_mn_obx_cback;

#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
        start_msg.psm = p_cb->mn_psm;
        start_msg.srm = TRUE;
#endif

        start_msg.max_sessions = WICED_BT_MCE_MN_NUM_SESSION;

        if (wiced_bt_obex_start_server (&start_msg, &p_cb->mn_obx_handle) == OBEX_SUCCESS)
        {
            status = WICED_BT_MA_STATUS_OK;

            for (i = 0; i < WICED_BT_MCE_MN_NUM_SESSION; i++)
            {
                p_cb->scb[i].mn_state = WICED_MCE_MN_LISTEN_ST;
            }

            /* Set up the SDP record for Message Notification Server */
            //wiced_mce_mn_sdp_register(p_cb, p_api->servicename);
            p_cb->mn_started = TRUE;
        }
        else
        {
            APPL_TRACE_ERROR0("OBX_StartServer returns error\n");
        }

        start_evt.status = status;
        start_evt.session_id = (wiced_bt_ma_sess_handle_t)p_cb->mn_obx_handle;

        p_cb->p_cback(WICED_BT_MCE_START_EVT, (wiced_bt_mce_t *) &start_evt);
    }
    else
    {
        start_evt.status = WICED_BT_MA_STATUS_FAIL;
        p_cb->p_cback(WICED_BT_MCE_START_EVT, (wiced_bt_mce_t *) &start_evt);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_mn_int_close
**
** Description      Processes the internal MNS session close event
**
** Parameters       scb_idx     - Index to the MNS control block
**                  p_data      - Pointer to the event data
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_mn_int_close(UINT8 scb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_mn_cb_t           *p_scb = WICED_MCE_GET_MN_CB_PTR(scb_idx);
    wiced_bt_device_address_t   bd_addr;

    APPL_TRACE_EVENT0("wiced_mce_mn_int_close\n");

    if (wiced_bt_obex_get_peer_addr(p_scb->obx_handle, bd_addr) != 0)
    {
        APPL_TRACE_EVENT1("Send Obx Discon rsp obx session id=%d\n", p_scb->obx_handle);
        /* resources will be freed at WICED_MCE_MN_OBX_CLOSE_EVT */
        wiced_bt_obex_send_response(p_scb->obx_handle, OBEX_REQ_DISCONNECT, OBEX_RSP_SERVICE_UNAVL, NULL);
    }
    else
    {
        /* OBX close already */
        wiced_mce_mn_sm_execute(scb_idx, WICED_MCE_MN_OBX_CLOSE_EVT, p_data);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_mn_obx_connect
**
** Description      MN state machine action routine for Obex connect event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_mn_obx_connect(UINT8 scb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t          *p_cb = &wiced_mce_cb;
    wiced_mce_ma_cb_t       *p_ccb = NULL;
    wiced_mce_mn_cb_t       *p_scb = WICED_MCE_GET_MN_CB_PTR(scb_idx);
    wiced_mce_obx_evt_t     *p_evt = &p_data->obx_evt;
    wiced_bt_mce_mn_open_close_t open_evt;

    APPL_TRACE_EVENT0("wiced_mce_mn_obx_connect\n");

    if ((p_ccb = wiced_mce_get_ma_cb_using_bd_addr(p_evt->param.conn.peer_addr)) != NULL)
    {
        memcpy(p_scb->bd_addr, p_evt->param.conn.peer_addr, BD_ADDR_LEN);
        p_scb->mn_peer_mtu = p_evt->param.conn.mtu;
        p_scb->obx_handle = p_evt->handle;
        p_scb->in_use = TRUE;
        APPL_TRACE_EVENT1("MCE MN Connect: peer mtu 0x%04x\n", p_ccb->peer_mtu);

        wiced_bt_obex_send_response(p_evt->handle, OBEX_REQ_CONNECT, OBEX_RSP_OK, NULL);

        /* inform role manager */
        //wiced_mce_pm_conn_open(p_scb->bd_addr);

        memcpy(open_evt.bd_addr, p_evt->param.conn.peer_addr, BD_ADDR_LEN);

        open_evt.status = WICED_BT_MA_STATUS_OK;
        open_evt.obx_rsp_code = OBEX_RSP_OK;
        p_scb->p_devname = NULL;
        if (p_ccb->p_devname != NULL)
        {
            p_scb->p_devname = p_ccb->p_devname;
            BCM_STRNCPY_S((char *)open_evt.dev_name, sizeof(open_evt.dev_name), p_ccb->p_devname, BTM_MAX_REM_BD_NAME_LEN);
        }
        open_evt.session_id = (wiced_bt_ma_sess_handle_t) p_evt->handle;
    }
    else
    {
        /* connection request coming from a different device, reject it */
        open_evt.status = WICED_BT_MA_STATUS_FAIL;
        open_evt.obx_rsp_code = OBEX_RSP_FORBIDDEN;
        wiced_bt_obex_send_response(p_evt->handle, OBEX_REQ_CONNECT, OBEX_RSP_FORBIDDEN, NULL);
    }

    /* Done with Obex packet */
    utl_freebuf((void**)&p_evt->p_pkt);
    p_cb->p_cback(WICED_BT_MCE_MN_OPEN_EVT, (wiced_bt_mce_t *) &open_evt);
}

/*******************************************************************************
**
** Function         wiced_mce_mn_obx_disc
**
** Description      MN state machine action routine for Obex disconnect event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_mn_obx_disc(UINT8 scb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_obx_evt_t *p_evt = &p_data->obx_evt;
    UINT8               rsp_code;

    APPL_TRACE_EVENT0("wiced_mce_mn_obx_disc\n");

    rsp_code = (p_evt->obx_event == OBEX_DISCONNECT_REQ_EVT) ? OBEX_RSP_OK
               : OBEX_RSP_BAD_REQUEST;
    wiced_bt_obex_send_response(p_evt->handle, OBEX_REQ_DISCONNECT, rsp_code, NULL);

    /* Done with Obex packet */
    utl_freebuf((void**)&p_evt->p_pkt);
}

/*******************************************************************************
**
** Function         wiced_mce_mn_obx_put
**
** Description      MN state machine action routine for Obex put event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_mn_obx_put(UINT8 scb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t      *p_cb = &wiced_mce_cb;
    wiced_mce_mn_cb_t   *p_scb = WICED_MCE_GET_MN_CB_PTR(scb_idx);
    wiced_bt_obex_rsp_code_t rsp_code = OBEX_RSP_BAD_REQUEST;
    wiced_bt_mce_notif_t app_evt;
    wiced_mce_obx_evt_t *p_evt = &p_data->obx_evt;
    UINT8               *p_type, *p_param = NULL;
    UINT16              len = 0;

    APPL_TRACE_EVENT0("wiced_mce_mn_obx_put\n");

    /* See if this is an event report */
    if (wiced_bt_obex_find_byte_sequence_header((uint8_t *)p_evt->p_pkt, OBEX_HI_TYPE, &p_type, &len))
    {
        if (!memcmp(p_type, WICED_BT_MA_HDR_TYPE_EVENT_RPT, len))
        {
            p_scb->mn_oper = WICED_MCE_MN_OPER_MAP_EVT_RPT;
            p_scb->mn_rcv_inst_id = FALSE;
            p_param = wiced_mce_read_app_params(p_evt->p_pkt, WICED_BT_MA_APH_MAS_INST_ID, &len);
            if (p_param)
            {
                p_scb->mn_rcv_inst_id = TRUE;
                p_scb->mn_inst_id =  *p_param;
            }
        }
    }

    if (p_scb->mn_oper == WICED_MCE_MN_OPER_MAP_EVT_RPT)
    {
        /* read event report */
        if (!wiced_bt_obex_find_body_header((uint8_t *)p_evt->p_pkt, &app_evt.p_object, &app_evt.len, &app_evt.final))
        {
            app_evt.p_object = NULL;
            app_evt.len = 0;
        }

        if (p_scb->mn_rcv_inst_id)
        {
            if (p_evt->param.put.final)
                rsp_code = OBEX_RSP_OK;
            else
                rsp_code = OBEX_RSP_CONTINUE;
        }

        if (rsp_code == OBEX_RSP_BAD_REQUEST)
        {
            app_evt.status = WICED_BT_MA_STATUS_FAIL;
            p_scb->mn_oper = WICED_MCE_MN_OPER_NONE;
        }
        else
            app_evt.status = WICED_BT_MA_STATUS_OK;

        app_evt.obx_rsp_code = rsp_code;

        wiced_bt_obex_send_response(p_evt->handle, OBEX_REQ_PUT, rsp_code, NULL);
        app_evt.session_id = p_evt->handle;
        app_evt.inst_id = p_scb->mn_inst_id;
        app_evt.final = p_evt->param.put.final;
        p_cb->p_cback(WICED_BT_MCE_NOTIF_EVT, (wiced_bt_mce_t *)&app_evt);
    }

    /* Done with Obex packet */
    utl_freebuf((void**)&p_evt->p_pkt);
}

/*******************************************************************************
**
** Function         wiced_mce_mn_obx_action
**
** Description      MN state machine action routine for Obex action event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_mn_obx_action(UINT8 scb_idx, wiced_mce_data_t *p_data)
{
    APPL_TRACE_EVENT0("wiced_mce_mn_obx_action\n");

    /* Action operation is not supported in MAP, send reject rsp and free data */
    wiced_bt_obex_send_response(p_data->obx_evt.handle, OBEX_REQ_ACTION, OBEX_RSP_NOT_IMPLEMENTED, NULL);

    /* Done with Obex packet */
    utl_freebuf((void**)&(p_data->obx_evt.p_pkt));
}

/*******************************************************************************
**
** Function         wiced_mce_mn_obx_abort
**
** Description      MN state machine action routine for Obex abort event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_mn_obx_abort(UINT8 scb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t *p_cb = &wiced_mce_cb;

    APPL_TRACE_EVENT0("wiced_mce_mn_obx_abort \n");

    wiced_bt_obex_send_response(p_cb->mn_obx_handle, OBEX_REQ_ABORT, OBEX_RSP_OK, NULL);

    /* Done with Obex packet */
    utl_freebuf((void**)&p_data->obx_evt.p_pkt);
}

/*******************************************************************************
**
** Function        wiced_mce_mn_obx_close
**
** Description     MN state machine action routine for Obex close event.
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_mn_obx_close(UINT8 scb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t          *p_cb = &wiced_mce_cb;
    wiced_mce_mn_cb_t       *p_scb = WICED_MCE_GET_MN_CB_PTR(scb_idx);
    wiced_bt_mce_mn_open_close_t app_evt;

    APPL_TRACE_EVENT0("wiced_mce_mn_obx_close \n");
    memcpy(app_evt.bd_addr, p_scb->bd_addr, BD_ADDR_LEN);

    if (p_scb->p_devname != NULL)
        BCM_STRNCPY_S((char *)app_evt.dev_name, sizeof(app_evt.dev_name), p_scb->p_devname, BD_NAME_LEN);

    app_evt.status = WICED_BT_MA_STATUS_OK;
    app_evt.session_id = (wiced_bt_ma_sess_handle_t) p_data->obx_evt.handle;

    p_scb->in_use = FALSE;
    /* inform role manager */
    //wiced_mce_pm_conn_close(p_scb->bd_addr);

    /* Notify the MMI that a connection has been closed */
    p_cb->p_cback(WICED_BT_MCE_MN_CLOSE_EVT, (wiced_bt_mce_t *)&app_evt);

    /* Done with Obex packet */
    utl_freebuf((void**)&p_data->obx_evt.p_pkt);
}

/*******************************************************************************
**
** Function         wiced_mce_mn_disable
**
** Description      MN state machine action routine for API stop event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_mn_disable(wiced_mce_cb_t *p_cb, wiced_mce_data_t *p_data)
{
    wiced_bt_mce_mn_start_stop_t stop_evt;

    APPL_TRACE_EVENT1("wiced_mce_mn_disable handle %d\n",p_cb->mn_obx_handle);

    if (p_cb->mn_started)
    {
        /* Stop the OBEX server */
        wiced_bt_obex_stop_server(p_cb->mn_obx_handle);

        /* Remove the Message Notification service from the SDP database */
        if (p_cb->mn_sdp_handle)
            p_cb->mn_sdp_handle = 0;

        /* Free the allocated server channel number */
        //BTM_FreeSCN(p_cb->mn_scn);

        stop_evt.status = WICED_BT_MA_STATUS_OK;
        stop_evt.session_id = (wiced_bt_ma_sess_handle_t) p_cb->mn_obx_handle;
        p_cb->mn_obx_handle = 0 ;
        p_cb->mn_started = FALSE;
        p_cb->p_cback(WICED_BT_MCE_STOP_EVT, (wiced_bt_mce_t *)&stop_evt);
    }
}

/*******************************************************************************
**
** Function       wiced_mce_mn_conn_err_rsp
**
** Description    MN state machine action routine for Obex connect request in
**                wrong state event
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_mn_conn_err_rsp(UINT8 scb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t  *p_cb = &wiced_mce_cb;

    APPL_TRACE_EVENT0("wiced_mce_mn_conn_err_rsp \n");

    wiced_bt_obex_send_response(p_cb->mn_obx_handle, OBEX_REQ_CONNECT, OBEX_RSP_BAD_REQUEST, NULL);
}

/*******************************************************************************
**
** Function         wiced_mce_mn_obx_cback
**
** Description      OBX callback function for MCE MNS server.
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_mn_obx_cback (wiced_bt_obex_handle_t handle, wiced_bt_obex_event_t obx_event,
                             wiced_bt_obex_evt_param_t param, uint8_t *p_pkt)
{
    wiced_mce_obx_evt_t *p_obx_msg;
    UINT16              event = 0;

    APPL_TRACE_EVENT1("wiced_mce_mn_obx_cback [%d]\n", obx_event);

    switch (obx_event)
    {
        /* MN server */
        case OBEX_CONNECT_REQ_EVT:
            event = WICED_MCE_MN_OBX_CONN_REQ_EVT;
            break;
        case OBEX_DISCONNECT_REQ_EVT:
            event = WICED_MCE_MN_OBX_DISC_EVT;
            break;
        case OBEX_PUT_REQ_EVT:
            event = WICED_MCE_MN_OBX_PUT_EVT;
            break;
        case OBEX_ACTION_REQ_EVT:
            event = WICED_MCE_MN_OBX_ACTION_EVT;
            break;
        case OBEX_ABORT_REQ_EVT:
            event = WICED_MCE_MN_OBX_ABORT_EVT;
            break;
        case OBEX_CLOSE_IND_EVT:
            event = WICED_MCE_MN_OBX_CLOSE_EVT;
            break;
        default:
            if (p_pkt)
                GKI_freebuf(p_pkt);
    }
    /* send event up, if any */
    if (event && (p_obx_msg =
                  (wiced_mce_obx_evt_t *) GKI_getbuf(sizeof(wiced_mce_obx_evt_t))) != NULL)
    {
#if WICED_MCE_DEBUG == TRUE
        APPL_TRACE_EVENT1("OBX Event Callback: mn_obx_event [%s]\n",
                          mce_mn_obx_evt_code(event));
#endif
        p_obx_msg->hdr.event = event;
        p_obx_msg->obx_event = obx_event;
        p_obx_msg->handle = handle;
        p_obx_msg->param = param;
        p_obx_msg->p_pkt = (BT_HDR *)p_pkt;

        wiced_mce_send_event((BT_HDR *)p_obx_msg);
    }
    return;
}

/*****************************************************************************
**  Debug Functions
*****************************************************************************/
#if (WICED_MCE_DEBUG == TRUE) && (BT_USE_TRACES == TRUE)

/*******************************************************************************
**
** Function         mce_ma_obx_evt_code
**
** Description      MCE MA obex event code
**
** Returns          void
**
*******************************************************************************/
static char *mce_ma_obx_evt_code(UINT16 evt_code)
{
    switch (evt_code)
    {
        case WICED_MCE_MA_OBX_CONN_RSP_EVT:
            return "WICED_MCE_MA_OBX_CONN_RSP_EVT";
        case WICED_MCE_MA_OBX_PUT_RSP_EVT:
            return "WICED_MCE_MA_OBX_PUT_RSP_EVT";
        case WICED_MCE_MA_OBX_GET_RSP_EVT:
            return "WICED_MCE_MA_OBX_GET_RSP_EVT";
        case WICED_MCE_MA_OBX_SETPATH_RSP_EVT:
            return "WICED_MCE_MA_OBX_SETPATH_RSP_EVT";
        case WICED_MCE_MA_OBX_ABORT_RSP_EVT:
            return "WICED_MCE_MA_OBX_ABORT_RSP_EVT";
        case WICED_MCE_MA_OBX_TOUT_EVT:
            return "WICED_MCE_MA_OBX_TOUT_EVT";
        case WICED_MCE_MA_OBX_CLOSE_EVT:
            return "WICED_MCE_MA_OBX_CLOSE_EVT";
        default:
            return "unknown OBX event code";
    }
}

/*******************************************************************************
**
** Function         mce_mn_obx_evt_code
**
** Description      MCE MN obex event code
**
** Returns          void
**
*******************************************************************************/
static char *mce_mn_obx_evt_code(UINT16 evt_code)
{
    switch (evt_code)
    {
        case WICED_MCE_MN_OBX_CONN_REQ_EVT:
            return "WICED_MCE_MN_OBX_CONN_REQ_EVT";
        case WICED_MCE_MN_OBX_DISC_EVT:
            return "WICED_MCE_MN_OBX_DISC_EVT";
        case WICED_MCE_MN_OBX_PUT_EVT:
            return "WICED_MCE_MN_OBX_PUT_EVT";
        case WICED_MCE_MN_OBX_ACTION_EVT:
            return "WICED_MCE_MN_OBX_ACTION_EVT";
        case WICED_MCE_MN_OBX_ABORT_EVT:
            return "WICED_MCE_MN_OBX_ABORT_EVT";
        case WICED_MCE_MN_OBX_CLOSE_EVT:
            return "WICED_MCE_MN_OBX_CLOSE_EVT";
        default:
            return "unknown OBX event code";
    }
}
#endif  /* WICED_MCE_DEBUG */
