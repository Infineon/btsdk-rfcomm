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
**  Name:           wiced_bt_mce_utils.c
**
**  Description:    This file implements the utility functions for the
**                  Message Access client.
**
*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "wiced.h"
#include "wiced_bt_types.h"

#include "wiced_bt_mce_int.h"
#include "wiced_bt_utils.h"
#include "wiced_bt_mce_api.h"

/*******************************************************************************
**
** Function         wiced_mce_reset_icb
**
** Description      reset MCE instance control block.
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_reset_icb (wiced_mce_inst_cb_t *p_icb)
{
    utl_freebuf((void**)&p_icb->obx.p_pkt);
    utl_freebuf((void**)&p_icb->p_folder);

    p_icb->sdp_pending   = FALSE;
    p_icb->obx_oper      = WICED_MCE_OP_NONE;
    p_icb->aborting      = FALSE;
    p_icb->req_pending   = FALSE;
    p_icb->first_req_pkt = FALSE;
    p_icb->int_abort     = FALSE;
    p_icb->first_push_msg = TRUE;
}

/*******************************************************************************
**
** Function         wiced_mce_add_filler_byte
**
** Description      Add Filler byte into the body header of the OBX package
**
** Parameters
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_add_filler_byte(wiced_mce_obx_pkt_t *p_obx)
{
    UINT8 filler = WICED_BT_MA_BODY_FILLER_BYTE;

    wiced_bt_obex_add_header((uint8_t *)p_obx->p_pkt, OBEX_HI_BODY_END, &filler, 1);

    return;
}

/*******************************************************************************
**
** Function         wiced_mce_read_app_params
**
** Description      Read the application parameters from the given OBX packet
**
** Parameters
**
** Returns          void
**
*******************************************************************************/
UINT8 *wiced_mce_read_app_params(BT_HDR *p_pkt, UINT8 tag, UINT16 *param_len)
{
    UINT8   *p_data = NULL, *p = NULL;
    UINT16  data_len = 0;
    int     left, len;

    if (wiced_bt_obex_find_byte_sequence_header((uint8_t *)p_pkt, OBEX_HI_APP_PARMS, &p_data, &data_len))
    {
        left = data_len;
        while (left > 0)
        {
            len = *(p_data + 1);
            if (*p_data == tag)
            {
                p_data += 2;
                p = p_data;
                *param_len = (UINT16) len;
                break;
            }
            p_data     += (len+2);
            left       -= (len+2);
        }
    }
    return p;
}

/*******************************************************************************
**
** Function         wiced_mce_send_abort_req
**
** Description      Send a Abort request.
**
** Parameters
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_send_abort_req(wiced_mce_inst_cb_t *p_icb)
{
    if (WICED_MCE_ABORT_REQ_NOT_SENT == p_icb->aborting)
    {
        p_icb->aborting = WICED_MCE_ABORT_REQ_SENT;
        wiced_bt_obex_send_request(p_icb->obx_handle, OBEX_REQ_ABORT, NULL, NULL);

        /* Start abort response timer */
        p_icb->timer_oper = WICED_MCE_TIMER_OP_ABORT;
        wiced_start_timer(&p_icb->rsp_timer, 2);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_send_get_req
**
** Description      Processes a GetMsg Operation.
**
** Parameters       p_cb      - Pointer to the FTC control block
**
** Returns          (UINT8) OBX response code
**
*******************************************************************************/
UINT8 wiced_mce_send_get_req(wiced_mce_inst_cb_t *p_icb)
{
    wiced_mce_obx_pkt_t *p_obx = &p_icb->obx;
    wiced_bool_t final = WICED_TRUE;
    UINT8 rsp_code = OBEX_RSP_FAILED;

    /* Do not start another request if currently aborting */
    if (!p_icb->aborting)
    {
        /* OBX header are added in bta_ftc_init_getfile */
        if (wiced_bt_obex_send_request(p_icb->obx_handle, OBEX_REQ_GET, (wiced_bt_obex_req_param_t*)&final, (uint8_t *)p_obx->p_pkt) == OBEX_SUCCESS)
        {
            p_icb->req_pending = TRUE;
            rsp_code = OBEX_RSP_OK;
            p_obx->p_pkt = NULL;
        }
    }
    else
    {
        wiced_mce_send_abort_req(p_icb);
    }

    return(rsp_code);
}

/*******************************************************************************
**
** Function         wiced_mce_convert_obx_to_status
**
** Description      Convert OBX status into MAP status code.
**
**
** Returns          MAP status code.
**
*******************************************************************************/
wiced_bt_ma_status_t wiced_mce_convert_obx_to_status(wiced_bt_obex_rsp_code_t status)
{
    if (status == OBEX_RSP_OK || status == OBEX_RSP_CONTINUE)
    {
        return WICED_BT_MA_STATUS_OK;
    }
    else
    {
        return WICED_BT_MA_STATUS_FAIL;
    }
}

/*******************************************************************************
**
** Function         wiced_mce_add_list_hdr
**
** Description      Add type/name/app header for listing request.
**
** Returns          MAP status code.
**
*******************************************************************************/
wiced_bt_ma_status_t wiced_mce_add_list_hdr(wiced_mce_obx_pkt_t *p_obx, wiced_mce_api_list_t *p_list)
{
    UINT8   *p = (p_list->list_type == WICED_MCE_LIST_TYPE_DIR) ?
                 (UINT8 *) WICED_BT_MA_HDR_TYPE_FOLDER_LIST :
                 (UINT8 *) WICED_BT_MA_HDR_TYPE_MSG_LIST;
    UINT8 *p2, *p_start;
    wiced_bt_ma_status_t  status = WICED_BT_MA_STATUS_OK;
    BOOLEAN is_ok = TRUE;
    UINT16 len = 0;
    UINT8 str_len = 0;

    if (p_list->list_type == WICED_MCE_LIST_TYPE_MSG)
    {
        is_ok = wiced_bt_obex_add_header_utf8((UINT8 *)p_obx->p_pkt, OBEX_HI_NAME, (UINT8 *)p_list->p_folder);
    }
    if (is_ok)
    {
        /* add type header */
        is_ok = wiced_bt_obex_add_header((uint8_t *)p_obx->p_pkt, OBEX_HI_TYPE, p, strlen((char *)p) + 1);

        if (p_list->is_srmp_pause)
        {
            uint8_t srmp = 1;
            wiced_bt_obex_add_header((uint8_t *)p_obx->p_pkt, OBEX_HI_SRM_PARAM, &srmp, sizeof(uint8_t));
        }

        if (is_ok)
        {
            /* add app params for Notification Status */
            p_start = wiced_bt_obex_add_byte_sequence_start((uint8_t *)p_obx->p_pkt, &len);
            p = p_start;

            if (p_list->max_list_count != 0xFFFF)
            {
                *p++    = WICED_BT_MA_APH_MAX_LIST_COUNT;
                *p++    = 2;
                UINT16_TO_BE_STREAM(p, p_list->max_list_count);
            }
            if (p_list->list_start_offset)
            {
                *p++    = WICED_BT_MA_APH_START_STOFF;
                *p++    = 2;
                UINT16_TO_BE_STREAM(p, p_list->list_start_offset);
            }

            /* add more app headers */
            if (p_list->list_type == WICED_MCE_LIST_TYPE_MSG && p_list->p_param)
            {
                /* subject length */
                if (p_list->p_param->subject_length)
                {
                    *p++    = WICED_BT_MA_APH_SUBJ_LEN;
                    *p++    = 1;
                    *p++    =p_list->p_param->subject_length;
                }
                /* Param mask */
                if (p_list->p_param->parameter_mask)
                {
                    *p++    = WICED_BT_MA_APH_PARAM_MASK;
                    *p++    = 4;
                    UINT32_TO_BE_STREAM(p, p_list->p_param->parameter_mask);
                }
                /*filter msg type */
                if (p_list->p_param->msg_mask)
                {
                    *p++    = WICED_BT_MA_APH_FILTER_MSG_TYPE;
                    *p++    = 1;
                    *p++    = p_list->p_param->msg_mask;
                }
                /* Filter Period Begin */
                if (strlen(p_list->p_param->period_begin))
                {
                    *p++    = WICED_BT_MA_APH_FILTER_PRD_BEGIN;
                    *p      = strlen(p_list->p_param->period_begin)+1;
                    BCM_STRNCPY_S((char *) (p+1), strlen(p_list->p_param->period_begin)+1, p_list->p_param->period_begin, *p);
                    p2 = p + 1 + *p;
                    p = p2;
                }
                /* Filter Period End */
                if (strlen(p_list->p_param->period_end))
                {
                    *p++    = WICED_BT_MA_APH_FILTER_PRD_END;
                    *p      = strlen(p_list->p_param->period_end)+1;
                    BCM_STRNCPY_S((char *) (p+1), strlen(p_list->p_param->period_end)+1, p_list->p_param->period_end, *p);
                    p2 = p + 1 + *p;
                    p = p2;
                }
                /* read status */
                *p++    = WICED_BT_MA_APH_FILTER_READ_STS;
                *p++    = 1;
                *p++    = p_list->p_param->read_status;

                /* filter recipient */
                if ((str_len = strlen(p_list->p_param->recipient)) != 0)
                {
                    *p++    = WICED_BT_MA_APH_FILTER_RECEIP;
                    *p++    = str_len;
                    BCM_STRNCPY_S((char *)p, str_len+1, (const char *)p_list->p_param->recipient, str_len);
                    p += (str_len);

                }
                /* filter originator */
                if ((str_len = strlen(p_list->p_param->originator)) != 0)
                {
                    *p++    = WICED_BT_MA_APH_FILTER_ORIGIN;
                    *p++    = str_len;
                    BCM_STRNCPY_S((char *)p, str_len+1, (const char *)p_list->p_param->originator, str_len);
                    p += (str_len);

                }
                /* Filter Priority */
                if (p_list->p_param->pri_status)
                {
                    *p++    = WICED_BT_MA_APH_FILTER_PRIORITY;
                    *p++    = 1;
                    *p++    = p_list->p_param->pri_status;
                }
            }

            if (p != p_start)
            {
                wiced_bt_obex_add_byte_sequence_end((uint8_t *)p_obx->p_pkt, OBEX_HI_APP_PARMS, (UINT16)(p - p_start));
            }
        }
        else
            status = WICED_BT_MA_STATUS_FAIL;
    }
    else
        status = WICED_BT_MA_STATUS_FAIL;

    return status;
}

/*******************************************************************************
**
** Function         wiced_mce_get_listing
**
** Description      Initiates or Continues a GET Listing operation
**                  on the server's current directory.
**
** Parameters       p_cb - pointer to the client's control block.
**                  first_pkt - TRUE if initial GET request to server.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_get_listing(wiced_mce_inst_cb_t *p_icb, wiced_mce_api_list_t *p_data)
{
    wiced_mce_obx_pkt_t     *p_obx = &p_icb->obx;
    wiced_bt_ma_status_t    status = WICED_BT_MA_STATUS_FAIL;
    wiced_bool_t            final = WICED_TRUE;

    if ((p_obx->p_pkt = (BT_HDR *)wiced_bt_obex_header_init(p_icb->obx_handle, OBX_CMD_POOL_SIZE)) != NULL)
    {
        status = WICED_BT_MA_STATUS_OK;
        /* add type/name/app headers if first packet */
        if (p_data)
        {
            status = wiced_mce_add_list_hdr(p_obx, p_data);
        }
    }

    if (status == WICED_BT_MA_STATUS_OK)
    {
        if (wiced_bt_obex_send_request(p_icb->obx_handle, OBEX_REQ_GET, (wiced_bt_obex_req_param_t*)&final, (uint8_t *)p_obx->p_pkt) == OBEX_SUCCESS)
        {
            p_obx->p_pkt = NULL;
            if (p_data)
            {
                p_icb->obx_oper = WICED_MCE_OP_FOLDER_LIST + p_data->list_type;
                p_icb->req_pending = TRUE;
                p_icb->first_req_pkt = TRUE;
            }
        }
        else
        {
            status = WICED_BT_MA_STATUS_FAIL;
        }
    }
    if (status != WICED_BT_MA_STATUS_OK)
    {
        wiced_mce_listing_err(p_icb, &p_obx->p_pkt, status, OBEX_RSP_OK);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_listing_err
**
** Description      Send a directory listing error event to the application
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_listing_err(wiced_mce_inst_cb_t *p_icb, BT_HDR **p_pkt, wiced_bt_ma_status_t status, wiced_bt_obex_rsp_code_t obx_rsp)
{
    wiced_mce_cb_t      *p_cb = &wiced_mce_cb;
    wiced_bt_mce_list_data_t list_data;
    UINT8               evt = WICED_BT_MCE_FOLDER_LIST_EVT + p_icb->obx_oper - WICED_MCE_OP_FOLDER_LIST;

    p_icb->obx_oper = WICED_MCE_OP_NONE;
    utl_freebuf((void**)p_pkt);

    memset(&list_data, 0, sizeof(wiced_bt_mce_list_data_t));

    list_data.status = status;
    list_data.session_id = p_icb->obx_handle;
    list_data.is_final = TRUE;
    list_data.obx_rsp_code = obx_rsp;

    p_cb->p_cback(evt, (wiced_bt_mce_t *)&list_data);
}

/*******************************************************************************
**
** Function         wiced_mce_proc_list_param
**
** Description      read folder/msg listing app parameter header.
**
** Returns
**
*******************************************************************************/
void wiced_mce_proc_list_param(wiced_bt_mce_list_app_param_t * p_param, BT_HDR *p_pkt)
{
    UINT8   *p_data = NULL, aph;
    UINT16  data_len = 0;
    int     left, len;

    if (wiced_bt_obex_find_byte_sequence_header((uint8_t *)p_pkt, OBEX_HI_APP_PARMS, &p_data, &data_len))
    {
        memset(p_param, 0, sizeof(wiced_bt_mce_list_app_param_t));
        left    = data_len;

        while (left > 0)
        {
            aph = *p_data++;
            len = *p_data++;
            left -= len;
            left -= 2;
            switch (aph)
            {
                case WICED_BT_MA_APH_FOLDER_LST_SIZE:
                    BE_STREAM_TO_UINT16(p_param->fld_list_size, p_data);
                    return;

                case WICED_BT_MA_APH_MSG_LST_SIZE:
                    BE_STREAM_TO_UINT16(p_param->msg_list_param.msg_list_size, p_data);
                    break;

                case WICED_BT_MA_APH_NEW_MSG:
                    p_param->msg_list_param.new_msg = *p_data++;
                    break;
                default:
                    p_data += len;
            }
        }
    }
    else
        p_param = NULL;
}

/*******************************************************************************
**
** Function         wiced_mce_proc_get_list_rsp
**
** Description      Processes responses to directory listing requests
**                  Loops through returned data generating application
**                  listing data events.  If needed, issues a new request
**                  to the server.
**
** Parameters       ccb_idx       - MA client control block index
**                  icb_idx       - MA instance control block index
**                  p_evt         - Pointer to MCE OBEX event data
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_proc_get_list_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_obx_evt_t *p_evt)
{
    wiced_mce_cb_t             *p_cb = &wiced_mce_cb;
    wiced_mce_inst_cb_t        *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_bt_mce_list_data_t   app_evt;
    wiced_bt_mce_list_app_param_t app_param;
    BOOLEAN                 is_ok = FALSE;
    BOOLEAN                 first_req_pkt = p_icb->first_req_pkt;
    UINT8                   evt = WICED_BT_MCE_FOLDER_LIST_EVT + p_icb->obx_oper - WICED_MCE_OP_FOLDER_LIST;

    app_evt.status = wiced_mce_convert_obx_to_status(p_evt->rsp_code);
    app_evt.obx_rsp_code = p_evt->rsp_code;

    if (p_evt->rsp_code == OBEX_RSP_OK || p_evt->rsp_code == OBEX_RSP_CONTINUE)
    {
        app_evt.p_param = NULL;
        app_evt.session_id = p_icb->obx_handle;

        if (p_icb->first_req_pkt == TRUE)
        {
            app_evt.p_param = &app_param;
            p_icb->first_req_pkt = FALSE;
            /* process app headers */
            wiced_mce_proc_list_param(app_evt.p_param, p_evt->p_pkt);
        }

        if (wiced_bt_obex_find_body_header((uint8_t *)p_evt->p_pkt, &app_evt.p_data, &app_evt.len, &app_evt.is_final))
        {
            is_ok = TRUE;
        }
        else if (first_req_pkt &&
                 (p_evt->rsp_code == OBEX_RSP_CONTINUE || p_evt->rsp_code == OBEX_RSP_OK))
        {
            /* no body header is OK, if this is the first packet and is continue response
               or request only for folder listing size */
            is_ok = TRUE;
        }

        if (is_ok)
        {
            /* len > 0 or is final packet */
            if (app_evt.len || app_evt.is_final || app_evt.p_param)
            {
                p_cb->p_cback(evt, (wiced_bt_mce_t *)&app_evt);
            }
            utl_freebuf((void**)&p_evt->p_pkt);
            /* Initiate another request if not finished */
            if (p_evt->rsp_code == OBEX_RSP_CONTINUE)
            {
                if (p_icb->aborting)
                    wiced_mce_send_abort_req(p_icb);
                else
                    wiced_mce_get_listing(p_icb, NULL);
            }
            else
                p_icb->obx_oper = WICED_MCE_OP_NONE;  /* Finished with directory listing */
        }
        else
        {
            /* Missing body header & not the first packet */
            wiced_mce_listing_err(p_icb, &p_evt->p_pkt, WICED_BT_MA_STATUS_FAIL, OBEX_RSP_NO_CONTENT);
        }
    }
    else    /* Issue an error list entry */
        wiced_mce_listing_err(p_icb, &p_evt->p_pkt, app_evt.status, p_evt->rsp_code);
}

/*******************************************************************************
**
** Function         wiced_mce_proc_get_msg_rsp
**
** Description      Process a GET response for GetMsg operation.
**
** Parameters       ccb_idx       - MA client control block index
**                  icb_idx       - MA instance control block index
**                  p_data        - Pointer to MCE msg data
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_proc_get_msg_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t          *p_cb = &wiced_mce_cb;
    wiced_mce_inst_cb_t     *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_mce_obx_pkt_t     *p_obx = WICED_MCE_GET_MA_OBX_PKT_PTR(ccb_idx, icb_idx);
    wiced_mce_obx_evt_t     *p_evt = &p_data->obx_evt;
    wiced_bt_mce_get_msg_t  app_evt;
    static wiced_bt_ma_frac_deliver_t frac_delv = WICED_BT_MA_FRAC_DELIVER_NO;
    UINT8                   *p_val;
    UINT16                  len;

    app_evt.status = wiced_mce_convert_obx_to_status(p_evt->rsp_code);
    app_evt.obx_rsp_code = p_evt->rsp_code;

    if (p_evt->rsp_code == OBEX_RSP_OK || p_evt->rsp_code == OBEX_RSP_CONTINUE)
    {
        app_evt.session_id = p_icb->obx_handle;

        /* Only continue if not aborting */
        if (p_icb->aborting)
        {
            /* Aborting: done with current packet */
            utl_freebuf((void**)&p_evt->p_pkt);

            /* If aborting, and this response is not last packet send abort */
            if (p_evt->rsp_code == OBEX_RSP_CONTINUE)
            {
                wiced_mce_send_abort_req(p_icb);
                return;
            }
            else /* Last packet - abort and remove file (p_evt->rsp_code == OBEX_RSP_OK) */
            {
                p_evt->rsp_code = (!p_icb->int_abort) ? OBEX_RSP_GONE : OBEX_RSP_INTRNL_SRVR_ERR;
                wiced_mce_ma_sm_execute(ccb_idx, icb_idx, WICED_MCE_MA_OBX_CMPL_EVT, p_data);
                return;
            }
        }

        /* If length header exists, save the file length */
        if (p_icb->first_req_pkt == TRUE)
        {
            p_icb->first_req_pkt = FALSE;

            /* read 1st application header */
            if ((p_val = wiced_mce_read_app_params(p_evt->p_pkt, WICED_BT_MA_APH_FRAC_DELVR, &len)) != NULL)
            {
                frac_delv = *p_val;
            }
            else if (p_icb->frac_req)
            {
                p_evt->rsp_code = OBEX_RSP_BAD_REQUEST;
            }
        }

        if (wiced_bt_obex_find_body_header((uint8_t *)p_evt->p_pkt, &app_evt.p_data, &app_evt.len, &app_evt.is_final))
        {
            app_evt.frac_deliver = frac_delv;
            p_cb->p_cback(WICED_BT_MCE_GET_MSG_EVT, (wiced_bt_mce_t *)&app_evt);
        }

        utl_freebuf((void**)&p_evt->p_pkt);

        if (p_evt->rsp_code == OBEX_RSP_CONTINUE)
        {
            /* Send a new get request */
            wiced_mce_send_get_req(p_icb);
        }
        else
            p_icb->obx_oper = WICED_MCE_OP_NONE;  /* Finished with directory listing */
    }
    else
    {
        wiced_mce_ma_sm_execute(ccb_idx, icb_idx, WICED_MCE_MA_OBX_CMPL_EVT, p_data);
        utl_freebuf((void**)&p_evt->p_pkt);
    }
}

/*******************************************************************************
**
** Function         wiced_mce_proc_get_mas_ins_info_rsp
**
** Description      Process a GET response for GetMASInstanceInfo operation.
**
** Parameters       ccb_idx       - MA client control block index
**                  icb_idx       - MA instance control block index
**                  p_data        - Pointer to MCE msg data
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_proc_get_mas_ins_info_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data)
{
    wiced_mce_cb_t              *p_cb = &wiced_mce_cb;
    wiced_mce_ma_cb_t            *p_ccb = WICED_MCE_GET_MA_CB_PTR(ccb_idx);
    wiced_mce_inst_cb_t         *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_mce_obx_evt_t         *p_evt = &p_data->obx_evt;
    wiced_bt_mce_get_mas_ins_info_t app_evt;
    UINT16                      len;
    wiced_bt_ma_status_t        status = WICED_BT_MA_STATUS_FAIL;
    BOOLEAN                     final;
    UINT8                       *p_byte;

    if (p_evt->rsp_code == OBEX_RSP_OK)
    {
        memset (p_icb->mas_ins_info, 0, WICED_BT_MA_INS_INFO_MAX_LEN);
        if (wiced_bt_obex_find_body_header((uint8_t *)p_evt->p_pkt, &p_byte, &len, &final) )
        {
            status = WICED_BT_MA_STATUS_OK;
            memcpy(p_icb->mas_ins_info, p_byte, len);
#if (WICED_MCE_DEBUG == TRUE) && (BT_USE_TRACES == TRUE)
            APPL_TRACE_DEBUG1("MAS Instance Response [%s]\n", p_icb->mas_ins_info);
#endif
        }
        else
        {
            APPL_TRACE_WARNING0("MAS Instance Response [FAILED]\n");
        }
    }

    app_evt.status = status;
    app_evt.session_id = p_icb->obx_handle;
    app_evt.mas_instance_id = p_ccb->get_mas_inst_id;
    app_evt.obx_rsp_code = p_evt->rsp_code;
    len = strlen(p_icb->mas_ins_info);

    BCM_STRNCPY_S(app_evt.mas_ins_info, WICED_BT_MA_INS_INFO_MAX_LEN, p_icb->mas_ins_info, len);
    app_evt.mas_ins_info[len] = '\0';

    /* Clean up control block */
    wiced_mce_reset_icb(p_icb);
    /* Free the message */
    utl_freebuf((void**)&p_data->obx_evt.p_pkt);
    p_cb->p_cback(WICED_BT_MCE_GET_MAS_INS_INFO, (wiced_bt_mce_t *)&app_evt);
}

#if 0
/*******************************************************************************
**
** Function         wiced_mce_cont_put_file
**
** Description      Continues a Push Msg Operation.
**                  Builds a new OBX packet, and initiates a read operation.
**
** Parameters       p_icb - pointer to the instance control block.
**                  p_param - Pointer to MA push msg data
**
** Returns          UINT8 OBX response code
**
*******************************************************************************/
UINT8 wiced_mce_cont_put_file(wiced_mce_inst_cb_t *p_icb, wiced_bt_ma_push_msg_param_t *p_param)
{
    wiced_mce_obx_pkt_t *p_obx = &p_icb->obx;
    UINT8 rsp_code;
    UINT8 *p, *p_start;
    UINT16 len = 0;

    /* Do not start another request if currently aborting */
    if (p_icb->aborting)
    {
        wiced_mce_send_abort_req(p_icb);
        return(OBEX_RSP_OK);
    }

    rsp_code = OBEX_RSP_FAILED;

    if ((p_obx->p_pkt = wiced_bt_obex_header_init(p_icb->obx_handle, OBX_LRG_DATA_POOL_SIZE)) != NULL)
    {
        p_obx->offset = 0;

        /* Add length header if it exists */
        if (p_param)
        {
            /* add type header */
            p = (UINT8 *)WICED_BT_MA_HDR_TYPE_MSG;
            wiced_bt_obex_add_header(p_obx->p_pkt, OBEX_HI_TYPE, (char *)p, strlen((char *)p) + 1);

            /* add name header */
            wiced_bt_obex_add_header_utf8((UINT8 *)p_obx->p_pkt, OBEX_HI_NAME, (UINT8 *)p_param->p_folder);

            /* add application header */
            p_start = wiced_bt_obex_add_byte_sequence_start(p_obx->p_pkt, &len);
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

            wiced_bt_obex_add_byte_sequence_end(p_obx->p_pkt, OBEX_HI_APP_PARMS, (UINT16)(p - p_start));
        }

        /* Add the start of the Body Header */
        p_obx->bytes_left = 0;
        p_obx->p_start = OBX_AddBodyStart(p_obx->p_pkt, &p_obx->bytes_left);

        /* Read in the first packet's worth of data */
        p_icb->cout_active = TRUE;
        wiced_mce_co_read_upload_msg(p_icb->fd, p_obx->bytes_left, &p_obx->p_start[p_obx->offset],
                                   WICED_MCE_CI_READ_EVT, p_icb->obx_handle);
        rsp_code = OBEX_RSP_OK;
    }

    return(rsp_code);
}

/*******************************************************************************
**
** Function         wiced_mce_ma_update_pm_state
**
** Description      Inform power manager to update the power state due to MA
**                  state change
**
** Parameters       ccb_idx       - MA client control block index
**                  icb_idx       - MA instance control block index
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_ma_update_pm_state(UINT8 ccb_idx, UINT8 icb_idx)
{
    wiced_mce_inst_cb_t *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_mce_ma_cb_t   *p_ccb = WICED_MCE_GET_MA_CB_PTR(ccb_idx);
    wiced_mce_pm_cb_t   *p_pcb = wiced_mce_get_pm_cb_using_bd_addr(p_ccb->bd_addr);

    if (p_pcb == NULL)
        return;

    if ( p_icb->ma_state == WICED_MCE_MA_CONN_ST )
    {
        if (( p_pcb->pm_state == WICED_MCE_PM_IDLE )
          &&( p_icb->obx_oper != WICED_MCE_OP_NONE ))
        {
            /* inform power manager */
            APPL_TRACE_DEBUG0("BTA MCE informs DM/PM busy state\n");
            bta_sys_busy( BTA_ID_MCE, wiced_mce_cb.app_id, p_pcb->bd_addr );
            p_pcb->pm_state = WICED_MCE_PM_BUSY;
        }
        else if (( p_pcb->pm_state == WICED_MCE_PM_BUSY )
               &&( p_icb->obx_oper == WICED_MCE_OP_NONE ))
        {
            /* inform power manager */
            APPL_TRACE_DEBUG0("BTA MCE informs DM/PM idle state\n");
            bta_sys_idle( BTA_ID_MCE, wiced_mce_cb.app_id, p_pcb->bd_addr);
            p_pcb->pm_state = WICED_MCE_PM_IDLE;
        }
    }
    else if ( p_icb->ma_state == WICED_MCE_MA_IDLE_ST )
    {
        /* initialize power management state */
        p_pcb->pm_state = WICED_MCE_PM_BUSY;
    }
}

/*******************************************************************************
**
** Function         wiced_mce_mn_update_pm_state
**
** Description      Inform power manager to update the power state due to MN
**                  state change
**
** Parameters       p_cb - pointer to the client's control block.
**                  scb_idx - index of the mn server control block
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_mn_update_pm_state(wiced_mce_cb_t *p_cb, UINT8 scb_idx)
{
    wiced_mce_mn_cb_t *p_scb = WICED_MCE_GET_MN_CB_PTR(scb_idx);
    wiced_mce_pm_cb_t *p_pcb = wiced_mce_get_pm_cb_using_bd_addr(p_scb->bd_addr);

    if (p_pcb == NULL)
        return;

    if ( p_scb->mn_state == WICED_MCE_MN_CONN_ST )
    {
        if (( p_pcb->pm_state == WICED_MCE_PM_IDLE )
          &&( p_scb->mn_oper != WICED_MCE_MN_OPER_NONE ))
        {
            /* inform power manager */
            APPL_TRACE_DEBUG0("BTA MCE informs DM/PM busy state\n");
            bta_sys_busy( BTA_ID_MCE, p_cb->app_id, p_pcb->bd_addr );
            p_pcb->pm_state = WICED_MCE_PM_BUSY;
        }
        else if (( p_pcb->pm_state == WICED_MCE_PM_BUSY )
               &&( p_scb->mn_oper == WICED_MCE_MN_OPER_NONE ))
        {
            /* inform power manager */
            APPL_TRACE_DEBUG0("BTA MCE informs DM/PM idle state\n");
            bta_sys_idle( BTA_ID_MCE ,p_cb->app_id, p_pcb->bd_addr);
            p_pcb->pm_state = WICED_MCE_PM_IDLE;
        }
    }
    else if ( p_scb->mn_state == WICED_MCE_MN_LISTEN_ST )
    {
        /* initialize power management state */
        p_pcb->pm_state = WICED_MCE_PM_BUSY;
    }
}
#endif

/*******************************************************************************
**
** Function         wiced_mce_send_ma_open_evt
**
** Description      Send a MA Open event
**
** Parameters       ccb_idx       - MA client control block index
**                  icb_idx       - MA instance control block index
**                  status        - Status for the open event
**                                  TRUE - connection opened
**                                  FALSE - unable to open connection
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_send_ma_open_evt(UINT8 ccb_idx, UINT8 icb_idx, wiced_bt_ma_status_t status)
{
    wiced_mce_cb_t             *p_cb = &wiced_mce_cb;
    wiced_mce_ma_cb_t          *p_ccb = WICED_MCE_GET_MA_CB_PTR(ccb_idx);
    wiced_mce_inst_cb_t        *p_icb = WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, icb_idx);
    wiced_bt_mce_ma_open_close_t param;

    param.session_id  = p_icb->obx_handle;
    param.mas_inst_id = p_icb->inst_id;
    param.status = status;

    memcpy(param.bd_addr, p_ccb->bd_addr, BD_ADDR_LEN);
    BCM_STRNCPY_S((char *)param.dev_name, sizeof(param.dev_name), p_ccb->p_devname, BD_NAME_LEN);

    if (status == WICED_BT_MA_STATUS_OK)
    {
        p_ccb->active_count++;
    }
    p_cb->p_cback(WICED_BT_MCE_MA_OPEN_EVT, (wiced_bt_mce_t *)&param);
}

/*******************************************************************************
**
** Function         wiced_mce_disable_complete
**
** Description      This function returns does the clean up for the disable routine
**                  and returns event to the upper layer
** Parameters       p_cb - pointer to the client's control block.
**
** Returns          void
**
*******************************************************************************/
void wiced_mce_disable_complete(wiced_mce_cb_t *p_cb)
{
    wiced_bt_ma_status_t  status = WICED_BT_MA_STATUS_OK;

    p_cb->is_enabled = FALSE;
    (*p_cb->p_cback)(WICED_BT_MCE_DISABLE_EVT, (wiced_bt_mce_t *)&status);
    memset(p_cb, 0, sizeof(wiced_mce_cb_t));
}

/*******************************************************************************
**
** Function      wiced_mce_ma_get_num_of_act_conn
**
** Description   Get the number of active MA connections
**
** Parameters    p_cb - pointer to the client's control block.
**
** Returns       UINT8 - Number of active MA connections
**
*******************************************************************************/
UINT8 wiced_mce_ma_get_num_of_act_conn(wiced_mce_cb_t *p_cb)
{
    UINT8 i,cnt = 0;

    for (i = 0; i < WICED_BT_MCE_NUM_MA; i++)
    {
        if (p_cb->ccb[i].active_count)
            cnt++;
    }

    APPL_TRACE_DEBUG1("wiced_mce_ma_get_num_of_act_conn, cnt=%d\n", cnt);
    return cnt;
}

/*******************************************************************************
**
** Function       wiced_mce_find_ma_cb_indexes
**
** Description    Finds the MA client and instance control block indexes
**                based on the received internal event
**
** Parameters     p_msg        - Pointer to MCE msg data
**                p_ccb_idx    - (output) pointer to the MA client control block index
**                p_icb_idx    - (output) pointer to the MA instance control block index
**
** Returns        BOOLEAN - TRUE found
**                          FALSE not found
**
*******************************************************************************/
BOOLEAN wiced_mce_find_ma_cb_indexes(wiced_mce_data_t *p_msg, UINT8 *p_ccb_idx, UINT8 *p_icb_idx)
{
    BOOLEAN found = FALSE;

    APPL_TRACE_DEBUG0("wiced_mce_find_ma_cb_indexes\n");

    switch (p_msg->hdr.event)
    {
        case WICED_MCE_SDP_OK_EVT:
        case WICED_MCE_SDP_FAIL_EVT:
            if (wiced_mce_find_inst_id_match_ma_icb_cb_index(p_msg->sdp_result.ccb_idx, p_msg->sdp_result.mas_instance_id, p_icb_idx))
            {
                found = TRUE;
                *p_ccb_idx = p_msg->sdp_result.ccb_idx;
            }
            break;
        case WICED_MCE_API_GET_MAS_INS_INFO_EVT:
            if (wiced_mce_find_obx_handle_match_ma_cb_indexes(p_msg->api_get_mas_ins_info.hdr.layer_specific, p_ccb_idx, p_icb_idx))
                found = TRUE;
            break;
        case WICED_MCE_MA_OBX_ABORT_RSP_EVT:
        case WICED_MCE_MA_OBX_CONN_RSP_EVT:
        case WICED_MCE_MA_OBX_PUT_RSP_EVT:
        case WICED_MCE_MA_OBX_GET_RSP_EVT:
        case WICED_MCE_MA_OBX_SETPATH_RSP_EVT:
        case WICED_MCE_MA_OBX_TOUT_EVT:
        case WICED_MCE_MA_OBX_CLOSE_EVT:
            if (wiced_mce_find_obx_handle_match_ma_cb_indexes(p_msg->obx_evt.handle, p_ccb_idx, p_icb_idx))
                found = TRUE;
            break;
        case WICED_MCE_API_UPD_INBOX_EVT:
        case WICED_MCE_API_CHDIR_EVT:
        case WICED_MCE_API_LIST_EVT:
        case WICED_MCE_API_NOTIF_REG_EVT:
        case WICED_MCE_API_GET_MSG_EVT:
        case WICED_MCE_API_SET_STS_EVT:
        case WICED_MCE_API_PUSH_EVT:
            if (wiced_mce_find_obx_handle_match_ma_cb_indexes(p_msg->hdr.layer_specific, p_ccb_idx, p_icb_idx))
                found = TRUE;
            break;
        case WICED_MCE_API_ABORT_EVT:
            if (wiced_mce_find_bd_addr_match_ma_cb_index(p_msg->api_open.bd_addr, p_ccb_idx))
            {
                if (wiced_mce_find_inst_id_match_ma_icb_cb_index(*p_ccb_idx, p_msg->api_abort.mas_inst_id, p_icb_idx))
                    found = TRUE;
            }
            break;
        case WICED_MCE_RSP_TOUT_EVT:
            if (wiced_mce_find_inst_id_match_ma_icb_cb_index(WICED_MCE_TPARAM_TO_CCB_IDX(p_msg->hdr.layer_specific),
                WICED_MCE_TPARAM_TO_INST_ID(p_msg->hdr.layer_specific), p_icb_idx))
            {
                found = TRUE;
                *p_ccb_idx = WICED_MCE_TPARAM_TO_CCB_IDX(p_msg->hdr.layer_specific);
            }
            break;
        default:
            break;
    }

    return found;
}

/*******************************************************************************
**
** Function       wiced_mce_find_mn_cb_index
**
** Description    Finds the MN server control block index based on the received internal event
**
** Parameters     p_msg        - Pointer to MCE msg data
**                p_scb_idx    - (output) pointer to the MN server control block index
**
** Returns        BOOLEAN - TRUE found
**                          FALSE not found
**
*******************************************************************************/
BOOLEAN wiced_mce_find_mn_cb_index(wiced_mce_data_t *p_msg, UINT8 *p_scb_idx)
{
    BOOLEAN found = FALSE;

    APPL_TRACE_DEBUG0("wiced_mce_find_mn_cb_index\n");

    switch(p_msg->hdr.event)
    {
        case WICED_MCE_MN_OBX_CONN_REQ_EVT:
            if (wiced_mce_find_available_mn_cb_index(p_msg->obx_evt.param.conn.peer_addr, p_scb_idx))
            {
                found = TRUE;
            }
            break;
        case WICED_MCE_MN_OBX_PUT_EVT:
        case WICED_MCE_MN_OBX_ACTION_EVT:
        case WICED_MCE_MN_OBX_ABORT_EVT:
        case WICED_MCE_MN_OBX_DISC_EVT:
        case WICED_MCE_MN_OBX_CLOSE_EVT:
            if (wiced_mce_find_obx_handle_match_mn_cb_index(p_msg->obx_evt.handle, p_scb_idx))
            {
                found = TRUE;
            }
            break;
        default:
            break;
    }

    return found;
}

/*******************************************************************************
**
** Function         wiced_mce_find_available_ma_cb_index
**
** Description      Finds an available MA client control block index
**
** Parameters       p_bd_addr - peer bd addr to copy on the connection control block.
**                  p_ccb_idx - (output) pointer to the MA client control block index
**
** Returns          BOOLEAN - TRUE found
**                            FALSE not found
**
*******************************************************************************/
BOOLEAN wiced_mce_find_available_ma_cb_index(BD_ADDR p_bd_addr, UINT8 *p_ccb_idx)
{
    BOOLEAN found = FALSE;
    UINT8 i;

    for (i = 0; i < WICED_BT_MCE_NUM_MA; i++)
    {
        if ((wiced_mce_cb.ccb[i].in_use == TRUE) &&
            (memcmp(wiced_mce_cb.ccb[i].bd_addr, p_bd_addr, BD_ADDR_LEN) == 0))
        {
            found = TRUE;
            *p_ccb_idx = i;
        }
    }

    if (!found)
    {
        for (i = 0; i < WICED_BT_MCE_NUM_MA; i++)
        {
            if (wiced_mce_cb.ccb[i].in_use == FALSE)
            {
                found = TRUE;
                *p_ccb_idx = i;
            }
        }
    }

    return found;
}

/*******************************************************************************
**
** Function       wiced_mce_find_inst_id_match_ma_icb_cb_index
**
** Description    Finds the MA instance control block index based on the specified
**                MA instance ID
**
** Parameters     ccb_idx       - MA client control block index
**                inst_id       - MA instance ID
**                p_icb_idx     - (output) pointer to the MA instance control block index
**
** Returns        BOOLEAN - TRUE found
**                          FALSE not found
*******************************************************************************/
BOOLEAN wiced_mce_find_inst_id_match_ma_icb_cb_index(UINT8 ccb_idx, wiced_bt_ma_inst_id_t inst_id, UINT8 *p_icb_idx)
{
    BOOLEAN found = FALSE;
    UINT8 i;

    if(ccb_idx >= WICED_BT_MCE_NUM_MA)
        return found;

    for (i = 0; i < WICED_BT_MCE_NUM_INST; i++)
    {
        if ((wiced_mce_cb.ccb[ccb_idx].icb[i].in_use == TRUE) &&
            (wiced_mce_cb.ccb[ccb_idx].icb[i].inst_id == inst_id))
        {
            found = TRUE;
            *p_icb_idx = i;
        }
    }

    return found;
}

/*******************************************************************************
**
** Function      wiced_mce_find_obx_handle_match_ma_cb_indexes
**
** Description   Find the MA control block indexes based on the specified obx handle
**
** Parameters    obx_handle  - Obex session handle
**               p_ccb_idx   - (output) pointer to the MA client control block index
**               p_icb_idx   - (output) pointer to the MA instance control block index
**
** Returns       BOOLEAN - TRUE found
**                         FALSE not found
**
*******************************************************************************/
BOOLEAN wiced_mce_find_obx_handle_match_ma_cb_indexes(wiced_bt_obex_handle_t obx_handle, UINT8 *p_ccb_idx, UINT8 *p_icb_idx)
{
    BOOLEAN found = FALSE;
    UINT8 i,j;

    for (i = 0; i < WICED_BT_MCE_NUM_MA; i++)
    {
        for (j = 0; j < WICED_BT_MCE_NUM_INST; j++)
        {
            if ((wiced_mce_cb.ccb[i].in_use == TRUE) &&
                (wiced_mce_cb.ccb[i].icb[j].obx_handle == obx_handle))
            {
                found = TRUE;
                *p_ccb_idx = i;
                *p_icb_idx = j;
            }
        }
    }

    return found;
}

/*******************************************************************************
**
** Function      wiced_mce_find_bd_addr_match_ma_cb_index
**
** Description   Find the MA control block index based on the specified BD address
**
** Parameters   p_bd_addr   - Pointer to the BD address
**              p_ccb_idx   - (output) pointer to the MA client control block index
**
** Returns      BOOLEAN - TRUE found
**                        FALSE not found
**
*******************************************************************************/
BOOLEAN wiced_mce_find_bd_addr_match_ma_cb_index(BD_ADDR p_bd_addr, UINT8 *p_ccb_idx)
{
    BOOLEAN found = FALSE;
    UINT8 i;

    for (i = 0; i < WICED_BT_MCE_NUM_MA; i++)
    {
        if ((wiced_mce_cb.ccb[i].in_use == TRUE) &&
            (memcmp(wiced_mce_cb.ccb[i].bd_addr, p_bd_addr, BD_ADDR_LEN) == 0))
        {
            found = TRUE;
            *p_ccb_idx = i;
            break;
        }
    }

    APPL_TRACE_DEBUG2("wiced_mce_find_bd_addr_match_ma_cb_index found=%d index=%d\n", found, i);
    return found;
}

/*******************************************************************************
**
** Function         wiced_mce_get_mce_inst_cb_using_inst_id
**
** Description      This function returns MCE instance control block associated with a
**                  specific instance_id.
**
** Parameters       ccb_idx       - MA client control block index
**                  inst_id       - MA instance control ID
**
** Returns          void
**
*******************************************************************************/
wiced_mce_inst_cb_t *wiced_mce_get_mce_inst_cb_using_inst_id(UINT8 ccb_idx, wiced_bt_ma_inst_id_t inst_id)
{
    UINT8 i;

    if(ccb_idx >= WICED_BT_MCE_NUM_MA)
        return NULL;

    for (i = 0; i < WICED_BT_MCE_NUM_INST; i++)
    {
        if ((wiced_mce_cb.ccb[ccb_idx].icb[i].in_use == TRUE) &&
            (wiced_mce_cb.ccb[ccb_idx].icb[i].inst_id == inst_id))
        {
            return &wiced_mce_cb.ccb[ccb_idx].icb[i];
        }
    }

    APPL_TRACE_ERROR1("Cannot find control block using instance id inst_id = %d\n", inst_id);
    return NULL;
}

/*******************************************************************************
**
** Function         wiced_mce_get_mce_inst_cb_using_obx_handle
**
** Description      This function returns MCE instance control block associated with a
**                  specific obex handle. Note obx_handle from obex level is the same as
**                  session id from the upper level.
**
** Parameters       obx_handle    - Obex session handle
**
** Returns          void
**
*******************************************************************************/
wiced_mce_inst_cb_t *wiced_mce_get_mce_inst_cb_using_obx_handle(wiced_bt_obex_handle_t obx_handle)
{
    UINT8 i,j;

    for (i = 0; i < WICED_BT_MCE_NUM_MA; i++)
    {
        for (j = 0; j < WICED_BT_MCE_NUM_INST; j++)
        {
            if ((wiced_mce_cb.ccb[i].icb[j].in_use == TRUE) &&
                (wiced_mce_cb.ccb[i].icb[j].obx_handle == obx_handle))
            {
                return &wiced_mce_cb.ccb[i].icb[j];
            }
        }
    }

    APPL_TRACE_ERROR1("Cannot find the control block using obex handle obx_handle = %x\n", obx_handle);
    return NULL;
}

/*******************************************************************************
**
** Function         wiced_mce_get_ma_cb_using_bd_addr
**
** Description      This function returns MA client control block associated
**                  with specified BD address.
**
** Parameters       p_bd_addr   - BD addr to find
**
** Returns          void
**
*******************************************************************************/
wiced_mce_ma_cb_t *wiced_mce_get_ma_cb_using_bd_addr(BD_ADDR p_bd_addr)
{
    UINT8 i;

    for (i = 0; i < WICED_BT_MCE_NUM_MA; i++)
    {
        if ((wiced_mce_cb.ccb[i].in_use == TRUE) && (memcmp(wiced_mce_cb.ccb[i].bd_addr, p_bd_addr, BD_ADDR_LEN) == 0))
        {
            return &wiced_mce_cb.ccb[i];
        }
    }

    return NULL;
}

/*******************************************************************************
**
** Function         wiced_mce_find_available_mn_cb_index
**
** Description      Finds an available MN server control block index
**
** Parameters       p_bd_addr   - Pointer to the BD address
**                  p_scb_idx   - (output) pointer to the MN server control block index
**
** Returns          void
**
*******************************************************************************/
BOOLEAN wiced_mce_find_available_mn_cb_index(BD_ADDR p_bd_addr, UINT8 *p_scb_idx)
{
    BOOLEAN found = FALSE;
    UINT8 i;

    /* Check whether there is already a connection */
    if (wiced_mce_find_bd_addr_match_mn_cb_index(p_bd_addr, p_scb_idx))
        return found;

    for (i = 0; i < WICED_BT_MCE_NUM_MN; i++)
    {
        if ((wiced_mce_cb.scb[i].in_use == FALSE) &&
            (wiced_mce_cb.scb[i].mn_state == WICED_MCE_MN_LISTEN_ST))
        {
            found = TRUE;
            *p_scb_idx = i;
            break;
        }
    }

    APPL_TRACE_DEBUG2("wiced_mce_find_available_mn_cb_index found=%d index=%d\n", found, i);
    return found;
}

/*******************************************************************************
**
** Function      wiced_mce_find_bd_addr_match_mn_cb_index
**
** Description   Find the MN control block index based on the specified BD address
**
** Parameters   p_bd_addr   - Pointer to the BD address
**              p_scb_idx   - (output) pointer to the MN server control block index
**
** Returns      BOOLEAN - TRUE found
**                        FALSE not found
**
*******************************************************************************/
BOOLEAN wiced_mce_find_bd_addr_match_mn_cb_index(BD_ADDR p_bd_addr, UINT8 *p_scb_idx)
{
    BOOLEAN found = FALSE;
    UINT8 i;

    for (i = 0; i < WICED_BT_MCE_NUM_MN; i++)
    {
        if ((wiced_mce_cb.scb[i].in_use == TRUE) &&
            (memcmp(wiced_mce_cb.scb[i].bd_addr, p_bd_addr, BD_ADDR_LEN) == 0))
        {
            found = TRUE;
            *p_scb_idx = i;
            break;
        }
    }

    APPL_TRACE_DEBUG2("wiced_mce_find_bd_addr_match_mn_cb_index found=%d index=%d\n", found, i);
    return found;
}

/*******************************************************************************
**
** Function      wiced_mce_find_obx_handle_match_mn_cb_index
**
** Description   Find the MN server control block index based on the specified obx handle
**
** Parameters    obx_handle  - Obex session handle
**               p_scb_idx   - (output) pointer to the MN server control block index
**
** Returns       BOOLEAN - TRUE found
**                         FALSE not found
**
*******************************************************************************/
BOOLEAN wiced_mce_find_obx_handle_match_mn_cb_index(wiced_bt_obex_handle_t obx_handle, UINT8 *p_scb_idx)
{
    BOOLEAN found = FALSE;
    UINT8 i;

    for (i = 0; i < WICED_BT_MCE_NUM_MN; i++)
    {
        if ((wiced_mce_cb.scb[i].in_use == TRUE) &&
            (wiced_mce_cb.scb[i].obx_handle == obx_handle))
        {
            found = TRUE;
            *p_scb_idx = i;
            break;
        }
    }

    APPL_TRACE_DEBUG2("wiced_mce_find_obx_handle_match_mn_cb_index found=%d index=%d\n", found, i);
    return found;
}

#if 0
/*******************************************************************************
**
** Function         wiced_mce_get_pm_cb_using_bd_addr
**
** Description      This function returns PM control block associated
**                  with specified BD address.
**
** Parameters       p_bd_addr   - Pointer to the BD address
**
** Returns          void
**
*******************************************************************************/
wiced_mce_pm_cb_t *wiced_mce_get_pm_cb_using_bd_addr(BD_ADDR p_bd_addr)
{
    UINT8 i;

    for (i = 0; i < WICED_BT_MCE_NUM_PM; i++)
    {
        if ((wiced_mce_cb.pcb[i].in_use == TRUE) &&
            (memcmp(wiced_mce_cb.pcb[i].bd_addr, p_bd_addr, BD_ADDR_LEN) == 0))
        {
            return &wiced_mce_cb.pcb[i];
        }
    }

    return NULL;
}

/*******************************************************************************
**
** Function      wiced_mce_find_pm_cb_index
**
** Description   Finds the PM control block index
**               based on the specified BD address
**
** Parameters    p_bd_addr  - BD address
**               p_idx      - (output) pointer to the MN control
**                              block index
**
** Returns       BOOLEAN    - TRUE found
**                            FALSE not found
**
*******************************************************************************/
BOOLEAN wiced_mce_find_pm_cb_index(BD_ADDR p_bd_addr, UINT8 *p_idx)
{
    BOOLEAN found=FALSE;
    UINT8 i;

    for (i = 0; i < WICED_BT_MCE_NUM_PM; i++)
    {
        if ((wiced_mce_cb.pcb[i].in_use) &&
            (memcmp(wiced_mce_cb.pcb[i].bd_addr, p_bd_addr, BD_ADDR_LEN) == 0))
        {
            found = TRUE;
            *p_idx = i;
            break;
        }
    }
    return found;
}

/*******************************************************************************
**
** Function      wiced_mce_find_avail_pm_cb_idx
**
** Description   Finds a not in use PM control block index
**
** Parameters    p_idx      - (output) pointer to the PM control block index
**
** Returns       BOOLEAN    - TRUE found
**                            FALSE not found
**
*******************************************************************************/
BOOLEAN wiced_mce_find_avail_pm_cb_idx(UINT8 *p_idx)
{
    BOOLEAN found = FALSE;
    UINT8 i;

    for (i = 0; i < WICED_BT_MCE_NUM_PM; i++)
    {
        if (!wiced_mce_cb.pcb[i].in_use)
        {
            found = TRUE;
            *p_idx = i;
            break;
        }
    }

    return found;
}

/*******************************************************************************
**
** Function      wiced_mce_pm_conn_open
**
** Description   Determine whether or not bta_sys_conn_open should be called
**
** Parameters    bd_addr - peer BD address
**
** Returns       None
**
*******************************************************************************/
void wiced_mce_pm_conn_open(BD_ADDR bd_addr)
{
    wiced_mce_pm_cb_t *p_pcb;
    UINT8 pm_idx;

    APPL_TRACE_DEBUG0("bta_mse_pm_conn_open\n");

    if (wiced_mce_get_pm_cb_using_bd_addr(bd_addr) == NULL)
    {
        if (wiced_mce_find_avail_pm_cb_idx(&pm_idx))
        {
            p_pcb = WICED_MCE_GET_PM_CB_PTR(pm_idx);

            p_pcb->in_use = TRUE;
            p_pcb->opened = TRUE;
            utl_bdcpy(p_pcb->bd_addr, bd_addr);
            bta_sys_conn_open(BTA_ID_MCE , wiced_mce_cb.app_id, bd_addr);
        }
    }
}

/*******************************************************************************
**
** Function      wiced_mce_pm_conn_close
**
** Description   Determine whether or not bta_sys_conn_close should be called
**
** Parameters    bd_addr - peer BD address
**
** Returns       None
*******************************************************************************/
void wiced_mce_pm_conn_close(BD_ADDR bd_addr)
{
    wiced_mce_pm_cb_t *p_pcb;
    UINT8  pm_idx, mn_idx, ma_idx;
    BOOLEAN found_bd_addr = FALSE;

    APPL_TRACE_DEBUG0("wiced_mce_pm_conn_close\n");

    if (wiced_mce_find_pm_cb_index(bd_addr, &pm_idx))
    {
        p_pcb = WICED_MCE_GET_PM_CB_PTR(pm_idx);
        if (p_pcb->opened)
        {
            if (wiced_mce_find_bd_addr_match_mn_cb_index(bd_addr, &mn_idx))
            {
                found_bd_addr = TRUE;
            }

            if (!found_bd_addr)
            {
                if ( wiced_mce_find_bd_addr_match_ma_cb_index(bd_addr, &ma_idx))
                {
                    found_bd_addr = TRUE;
                }
            }

            if (!found_bd_addr)
            {
                memset(p_pcb, 0, sizeof(wiced_mce_pm_cb_t));
                bta_sys_conn_close(BTA_ID_MCE, wiced_mce_cb.app_id, bd_addr);
            }
        }
    }
}
#endif
