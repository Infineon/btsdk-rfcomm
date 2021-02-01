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
**  Name:           wiced_bt_pbc_utils.c
**
**  Description:    This file implements object store functions for the
**                  phone book access client.
**
**
*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "wiced_bt_obex.h"
#include "wiced_bt_pbc_int.h"

#include "wiced_bt_pbc_co.h"


/*******************************************************************************
**  Constants
*******************************************************************************/

/*******************************************************************************
**  Local Function Prototypes
*******************************************************************************/

/*******************************************************************************
*  Macros for PBC
*******************************************************************************/

/*******************************************************************************
**  Local Function
*******************************************************************************/
/*******************************************************************************
**
** Function         wiced_bt_pbc_proc_pbap_param
**
** Description      read PBAP app parameter header.
**
** Parameters
**
** Returns
**
*******************************************************************************/
wiced_bt_pbc_pb_param_t * wiced_bt_pbc_proc_pbap_param(wiced_bt_pbc_pb_param_t *p_param, BT_HDR *p_pkt)
{
    UINT8   *p_data = NULL, aph;
    UINT16  data_len = 0;
    int     left, len;
    if(wiced_bt_obex_find_byte_sequence_header((UINT8 *)p_pkt, OBEX_HI_APP_PARMS, &p_data, &data_len))
    {
        memset(p_param, 0, sizeof(wiced_bt_pbc_pb_param_t));
        left    = data_len;
        while(left > 0)
        {
            aph = *p_data++;
            len = *p_data++;
            left -= len;
            left -= 2;
            switch(aph)
            {
            case WICED_BT_PBC_APH_PB_SIZE:
                BE_STREAM_TO_UINT16(p_param->phone_book_size, p_data);
                p_param->pbs_exist = TRUE;
                break;
            case WICED_BT_PBC_APH_NEW_MISSED_CALL:
                p_param->new_missed_calls = *p_data++;
                p_param->nmc_exist = TRUE;
                break;
            default:
                p_data += len;
            }
        }
    }
    else
        p_param = NULL;
    return p_param;
}

/*******************************************************************************
*  Exported Functions
*******************************************************************************/
/*******************************************************************************
**
** Function         wiced_bt_pbc_send_get_req
**
** Description      Processes a Get File Operation.
**
** Parameters       p_cb      - Pointer to the PBC control block
**
** Returns          (UINT8) OBX response code
**
*******************************************************************************/
UINT8 wiced_bt_pbc_send_get_req(wiced_bt_pbc_cb_t *p_cb)
{
    wiced_bt_pbc_obx_pkt_t *p_obx = &p_cb->obx;
    UINT8 rsp_code = OBEX_RSP_FAILED;

    /* Do not start another request if currently aborting */
    if (!p_cb->aborting)
    {
        wiced_bt_obex_req_param_t param;
        param.final = WICED_TRUE;

        /* OBX header are added in wiced_bt_pbc_init_getfile */
        if ((wiced_bt_obex_send_request(p_cb->obx_handle, OBEX_REQ_GET, &param, (UINT8 *)p_obx->p_pkt)) == OBEX_SUCCESS)
        {
            p_cb->req_pending = TRUE;
            rsp_code = OBEX_RSP_OK;
            p_obx->p_pkt = NULL;
        }
    }
    /* else wiced_bt_obx_abort_req must have been issued */

    return (rsp_code);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_proc_get_rsp
**
** Description      Processes a Get File response packet.
**                  Initiates a file write if no errors.
**
** Parameters
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_proc_get_rsp(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    wiced_bt_pbc_obx_evt_t *p_evt = &p_data->obx_evt;
    wiced_bt_pbc_obx_pkt_t *p_obx = &p_cb->obx;
    UINT16            body_size, read_size;
    BOOLEAN           final;
    BOOLEAN           done = TRUE;
    BOOLEAN           send_request = TRUE;
    UINT8             num_hdrs;
    wiced_bt_pbc_t          data;

    do
    {
        if (p_evt->rsp_code == OBEX_RSP_OK || p_evt->rsp_code == OBEX_RSP_CONTINUE)
        {
            /* Only continue if not aborting */
            if (p_cb->aborting)
            {
                /* Done with current packet */
                utl_freebuf((void**)&p_evt->p_pkt);
                if(p_evt->rsp_code == OBEX_RSP_CONTINUE)
                {
                    wiced_bt_obex_send_request(p_cb->obx_handle, OBEX_REQ_ABORT, NULL, NULL);
                    return;
                }
            }

            p_obx->final_pkt = (p_evt->rsp_code == OBEX_RSP_OK) ? TRUE : FALSE;

            /* If length header exists, save the file length */
            if (p_cb->first_get_pkt == TRUE)
            {
                p_cb->first_get_pkt = FALSE;

                /* if the obj_type is  */
                if(p_cb->obj_type == WICED_BT_PBC_GET_PB)
                {
                    wiced_bt_pbc_proc_pbap_param(&data.pb, p_evt->p_pkt);
                    wiced_bt_pbc_cb.p_cback(WICED_BT_PBC_PHONEBOOK_EVT, &data);
                    if(p_cb->fd == WICED_BT_PBC_INVALID_FD)
                    {
                        /* if the obj_type is get pb && the file id not open,
                         * must be getting the phonebook size only
                         * - end of transaction */
                        break;
                    }
                }
                read_size = sizeof(p_cb->file_size);
                if (wiced_bt_obex_read_header((UINT8 *)p_evt->p_pkt, OBEX_HI_LENGTH, (UINT8 *)&p_cb->file_size, &read_size) != OBEX_SUCCESS)
                    p_cb->file_size = WICED_BT_PBC_LEN_UNKNOWN;

            }

            /* Read the body header from the obx packet */
            num_hdrs = wiced_bt_obex_find_body_header((UINT8 *)p_evt->p_pkt, &p_obx->p_start, &body_size,
                                       &final);
            /* process body header */
            if (num_hdrs == 1)
            {
                if (body_size)
                {
                    /* Data to be written */
                    p_obx->p_pkt = p_evt->p_pkt;
                    p_obx->offset = body_size;  /* Save write size for comparison */
                    p_cb->cout_active = TRUE;

                    // Send data on the data_cback to the application.
                    wiced_bt_pbc_cb.p_data_cback(p_obx->p_start, body_size);

                    wiced_bt_pbc_co_write(p_cb->fd, p_obx->p_start, body_size, WICED_BT_PBC_CI_WRITE_EVT, 0, p_cb->app_id);

                    done = FALSE;
                    send_request = FALSE;
                }
            }
            else if (num_hdrs > 1)  /* Cannot handle more than a single body header */
            {
                p_evt->rsp_code = OBEX_RSP_BAD_REQUEST;
            }
            /* Empty Body; send next request or finished */
            else if (!p_obx->final_pkt)
                done = FALSE;
        }
    } while (0);

    if (done)
    {
        wiced_bt_pbc_sm_execute(p_cb, WICED_BT_PBC_OBX_CMPL_EVT, p_data);
        utl_freebuf((void**)&p_evt->p_pkt);
    }
    else if (send_request)
    {
        /* Free current packet and send a new request */
        utl_freebuf((void**)&p_evt->p_pkt);
        wiced_bt_pbc_send_get_req(p_cb);
    }
}



/*******************************************************************************
**
** Function         wiced_bt_pbc_proc_list_data
**
** Description      Processes responses to directory listing requests
**                  Loops through returned data generating application
**                  listing data events.  If needed, issues a new request
**                  to the server.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_proc_list_data(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_obx_evt_t *p_evt)
{
    wiced_bt_pbc_t app_evt;
    wiced_bt_pbc_pb_param_t param;

    app_evt.list.status = wiced_bt_pbc_convert_obx_to_pbc_status(p_evt->rsp_code);

    if (p_evt->rsp_code == OBEX_RSP_OK || p_evt->rsp_code == OBEX_RSP_CONTINUE)
    {
        app_evt.list.p_param = NULL;
        if (p_cb->first_get_pkt == TRUE)
        {
            p_cb->first_get_pkt = FALSE;
            app_evt.list.p_param = wiced_bt_pbc_proc_pbap_param(&param, p_evt->p_pkt);
        }

        /* Return ignored. void cast for coverity checker */
        (void) wiced_bt_obex_find_body_header((UINT8 *)p_evt->p_pkt, &app_evt.list.data, &app_evt.list.len,
                        &app_evt.list.final);

        /* len > 0 or is final packet */
        if(app_evt.list.len || app_evt.list.final)

        {
            wiced_bt_pbc_cb.p_cback(WICED_BT_PBC_LIST_EVT, &app_evt);
        }
        utl_freebuf((void**)&p_evt->p_pkt);

        /* Initiate another request if not finished */
        if (p_evt->rsp_code == OBEX_RSP_CONTINUE)
        {
            if (p_cb->aborting)
                wiced_bt_obex_send_request(p_cb->obx_handle, OBEX_REQ_ABORT, NULL, NULL);
            else
                wiced_bt_pbc_get_listing(p_cb, NULL, NULL);
        }
        else
            p_cb->obx_oper = PBC_OP_NONE;  /* Finished with directory listing */
    }
    else    /* Issue an error list entry */
        wiced_bt_pbc_listing_err(&p_evt->p_pkt, app_evt.list.status);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_get_listing
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
void wiced_bt_pbc_get_listing(wiced_bt_pbc_cb_t *p_cb, char *p_name, wiced_bt_pbc_list_param_t *p_param)
{
    wiced_bt_pbc_obx_pkt_t *p_obx = &p_cb->obx;
    wiced_bt_pbc_status_t   status = WICED_BT_PBC_FAIL;
    BOOLEAN           is_ok = TRUE;
    UINT8        *p, *p2, *p_start;
    UINT16       len    = 0;

    if ((p_obx->p_pkt = (BT_HDR *)wiced_bt_obex_header_init(p_cb->obx_handle, OBX_CMD_POOL_SIZE)) != NULL)
    {

        /* first packet */
        /* Add the Type Header */
        p = (UINT8 *) WICED_BT_PBC_FOLDER_LISTING_TYPE;
        if (p_cb->sdp_service == UUID_SERVCLASS_PBAP_PSE)
            p = (UINT8 *) WICED_BT_PBC_PULL_VCARD_LISTING_TYPE;

        is_ok = wiced_bt_obex_add_header((UINT8 *)p_obx->p_pkt, OBEX_HI_TYPE, p, strlen((char *)p) + 1);

        if (p_name)
        {
            if(strcmp(p_name, ".") == 0 || p_name[0] == 0)
            {
                p_name = NULL;
            }
        }

        is_ok = wiced_bt_obex_add_header_utf8((UINT8 *)p_obx->p_pkt, OBEX_HI_NAME, (UINT8 *) p_name);

        if (p_param && p_cb->sdp_service == UUID_SERVCLASS_PBAP_PSE)
        {
            /* add app params for PCE */
            p_start = (UINT8 *)wiced_bt_obex_add_byte_sequence_start((UINT8 *)p_obx->p_pkt, &len);
            p = p_start;
            if (p_param->order < WICED_BT_PBC_ORDER_MAX)
            {
                *p++    = WICED_BT_PBC_APH_ORDER;
                *p++    = 1;
                *p++    = p_param->order;
            }
            if (p_param->p_value)
            {
                *p++    = WICED_BT_PBC_APH_SEARCH_VALUE;
                *p      = strlen(p_param->p_value);
                BCM_STRNCPY_S((char *) (p+1), strlen(p_param->p_value)+1, p_param->p_value, *p);
                p2 = p + 1 + *p;
                p = p2;
            }
            if (p_param->attribute < WICED_BT_PBC_ATTR_MAX)
            {
                *p++    = WICED_BT_PBC_APH_SEARCH_ATTR;
                *p++    = 1;
                *p++    = p_param->attribute;
            }
            if (p_param->max_list_count != 0xFFFF)
            {
                *p++    = WICED_BT_PBC_APH_MAX_LIST_COUNT;
                *p++    = 2;
                UINT16_TO_BE_STREAM(p, p_param->max_list_count);
            }
            if (p_param->list_start_offset)
            {
                *p++    = WICED_BT_PBC_APH_LIST_STOFF;
                *p++    = 2;
                UINT16_TO_BE_STREAM(p, p_param->list_start_offset);
            }

#if (defined(WICED_BT_PBAP_1_2_SUPPORTED) && WICED_BT_PBAP_1_2_SUPPORTED == TRUE)
            /* Add APH resetnewmissedcalls with value 1 to reset NMC */
            if ((p_param->is_reset_miss_calls == TRUE) && (p_cb->peer_features & WICED_BT_PBC_SUP_FEA_ENH_MISSED_CALLS)
                && (p_cb->local_features & WICED_BT_PBC_SUP_FEA_ENH_MISSED_CALLS))
            {
                *p++    = WICED_BT_PBC_APH_RESET_NMC;
                *p++    = 1;
                *p++    = 1;
            }
            /* Add APH VcardSelector and VcardSelectorOperator */
            if ((p_cb->peer_features & WICED_BT_PBC_SUP_FEA_VCARD_SELECTING) &&
                (p_cb->local_features & WICED_BT_PBC_SUP_FEA_VCARD_SELECTING))
            {
                if (p_param->selector)
                {
                    *p++    = WICED_BT_PBC_APH_VCARD_SELE;
                    *p++    = 8;
                    UINT64_TO_BE_STREAM(p, p_param->selector);

                    *p++    = WICED_BT_PBC_APH_VCARD_SELE_OP;
                    *p++    = 1;
                    *p++    = p_param->selector_op;
                }

            }
#endif

            /* If any of the app param header is added */
            if (p != p_start)
            {
                wiced_bt_obex_add_byte_sequence_end((UINT8 *)p_obx->p_pkt, OBEX_HI_APP_PARMS, (UINT16)(p - p_start));
            }
            p_cb->first_get_pkt = TRUE;
        } /* if p_param: PBAP PCE need to keep AppParam in the first Get response */

        if (is_ok)
        {
            wiced_bt_obex_req_param_t param;
            param.final = WICED_TRUE;

            if ((wiced_bt_obex_send_request(p_cb->obx_handle, OBEX_REQ_GET, &param, (UINT8 *)p_obx->p_pkt)) == OBEX_SUCCESS)
            {
                p_cb->req_pending = TRUE;
                p_obx->p_pkt = NULL;    /* OBX will free the memory */
                p_cb->obx_oper = PBC_OP_GET_LIST;
                status = WICED_BT_PBC_OK;
            }
        }
    }

    if (status != WICED_BT_PBC_OK)   /* Send an error response to the application */
        wiced_bt_pbc_listing_err(&p_obx->p_pkt, status);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_listing_err
**
** Description      Send a directory listing error event to the application
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_listing_err(BT_HDR **p_pkt, wiced_bt_pbc_status_t status)
{
    wiced_bt_pbc_t err_rsp;

    if (wiced_bt_pbc_cb.obx_oper == PBC_OP_GET_LIST)
        wiced_bt_pbc_cb.obx_oper = PBC_OP_NONE;
    utl_freebuf((void**)p_pkt);

    err_rsp.list.len = 0;
    err_rsp.list.status = status;
    err_rsp.list.final = TRUE;
    err_rsp.list.data = NULL;
    wiced_bt_pbc_cb.p_cback(WICED_BT_PBC_LIST_EVT, &err_rsp);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_convert_obx_to_pbc_status
**
** Description      Convert OBX response code into BTA PBC status code.
**
** Returns          void
**
*******************************************************************************/
wiced_bt_pbc_status_t wiced_bt_pbc_convert_obx_to_pbc_status(wiced_bt_obex_status_t obex_status)
{
    wiced_bt_pbc_status_t status;

    switch (obex_status)
    {
    case OBEX_RSP_OK:
    case OBEX_RSP_CONTINUE:
        status = WICED_BT_PBC_OK;
        break;
    case OBEX_RSP_UNAUTHORIZED:
        status = WICED_BT_PBC_NO_PERMISSION;
        break;
    case OBEX_RSP_NOT_FOUND:
        status = WICED_BT_PBC_NOT_FOUND;
        break;
    case OBEX_RSP_REQ_ENT_2_LARGE:
    case OBEX_RSP_DATABASE_FULL:
        status = WICED_BT_PBC_FULL;
        break;
    case OBEX_RSP_GONE:
        status = WICED_BT_PBC_ABORTED;
        break;
    case OBEX_RSP_PRECONDTN_FAILED:
        status = WICED_BT_PBC_PRECONDITION_FAILED;
        break;
    default:
        status = WICED_BT_PBC_FAIL;
    }

    return (status);
}
