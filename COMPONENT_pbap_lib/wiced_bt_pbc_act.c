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
**  Name:           wiced_bt_pbc_act.c
**
**  Description:    This file contains the phone book access client action
**                  functions for the state machine.
**
**
*****************************************************************************/

#include <string.h>
#include "stdio.h"

#include "bt_types.h"
#include "wiced.h"
#include "wiced_bt_types.h"

#include "wiced_bt_sdp.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_bt_trace.h"

#include "wiced_bt_pbc_api.h"
#include "wiced_bt_pbc_int.h"

#include "wiced_bt_obex.h"


/*****************************************************************************
**  Constants
*****************************************************************************/
/* sdp discovery database size */
#define WICED_BT_PBC_DISC_SIZE       500

#define WICED_BT_PBC_UID_LEN         32
#define WICED_BT_PBC_NMC_RESET       1
#define WICED_BT_PBC_UID_VCF_SUFFIX  ".vcf"


/*****************************************************************************
**  Local Function prototypes
*****************************************************************************/
#if WICED_BT_PBC_DEBUG == TRUE
static char *pbc_obx_evt_code(wiced_bt_obex_event_t evt_code);
#endif

static void pbc_reset_cb (wiced_bt_pbc_cb_t *p_cb);
static void wiced_bt_pbc_sdp_cback(UINT16 status);

/*****************************************************************************
**  Action Functions
*****************************************************************************/

/*******************************************************************************
**
** Function         wiced_bt_pbc_init_open
**
** Description      Initiate a connection with a peer device's service.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_init_open(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    p_cb->obx_oper = PBC_OP_NONE;
    p_cb->sec_mask = p_data->api_open.sec_mask;
    p_cb->services = WICED_BT_PBAP_SERVICE_MASK;
    bdcpy(p_cb->bd_addr, p_data->api_open.bd_addr);
    p_cb->sdp_pending = TRUE;
}


/*******************************************************************************
** Function     wiced_bt_pbc_rsp_timeout_cback
** Description  Get client control block from timer param. Start BTU timer again.
**              Call application callback routine with OBX_TIMEOUT_EVT event.
*******************************************************************************/
void wiced_bt_pbc_rsp_timeout_cback(uint32_t cb_params)
{
    BT_HDR  *p_buf;

    WICED_BT_TRACE("wiced_bt_pbc_rsp_timeout_cback");

    if ((p_buf = (BT_HDR *)GKI_getbuf(sizeof(BT_HDR))) != NULL)
    {
        p_buf->event = WICED_BT_PBC_RSP_TOUT_EVT;
        wiced_bt_pbc_hdl_event((BT_HDR *)p_buf);
        GKI_freebuf(p_buf);
    }
}


/*******************************************************************************
**
** Function         wiced_bt_pbc_abort
**
** Description      Abort an active Get operation
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_abort(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    WICED_BT_TRACE("wiced_bt_pbc_abort");

    /* Abort an active request */
    if (p_cb->obx_oper != PBC_OP_NONE)
    {
        p_cb->aborting = TRUE;

        /* Start abort response timer */
        p_cb->timer_oper = PBC_TIMER_OP_ABORT;

        wiced_stop_timer(&p_cb->rsp_timer.wiced_timer);
        wiced_deinit_timer(&p_cb->rsp_timer.wiced_timer);

        wiced_init_timer(&p_cb->rsp_timer.wiced_timer, wiced_bt_pbc_rsp_timeout_cback, (uint32_t)&p_cb->rsp_timer, WICED_SECONDS_TIMER);
        wiced_start_timer(&p_cb->rsp_timer.wiced_timer, p_wiced_bt_pbc_cfg->stopabort_tout);

        /* Issue the abort request only if no request pending.
         * some devices do not like out of sequence aborts even though
         * the spec allows it */
        if (!p_cb->req_pending)
            wiced_bt_obex_send_request(p_cb->obx_handle, OBEX_REQ_ABORT, NULL, NULL);
    }
}


/*******************************************************************************
**
** Function         wiced_bt_pbc_init_getfile
**
** Description      Pull a file off the server.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_init_getfile(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    wiced_bt_pbc_api_get_t *p_get = &p_data->api_get;
    wiced_bt_pbc_get_param_t *p_param = p_get->p_param;
    wiced_bt_pbc_obx_pkt_t *p_obx = &p_cb->obx;
    wiced_bt_pbc_status_t   status = WICED_BT_PBC_FAIL;
    wiced_bt_pbc_t          data;
    UINT16            buflen;
    UINT16            rem_name_len;
    BOOLEAN is_ok;
    UINT8   *p = (UINT8 *)WICED_BT_PBC_PULL_VCARD_ENTRY_TYPE;
    UINT8   *p_start;
    UINT16  len    = 0;
    char    temp_buf[100] = "X-BT-UID:";

    /* Check whether peer device supports the requested repository type */
    if ((!strcmp(p_get->p_rem_name, WICED_BT_PBC_PULL_PB_SPD_NAME) && !(p_cb->peer_repositories & WICED_BT_PBC_REPOSIT_SPEED_DIAL)) ||
        (!strcmp(p_get->p_rem_name, WICED_BT_PBC_PULL_PB_FAV_NAME) && !(p_cb->peer_repositories & WICED_BT_PBC_REPOSIT_FAVORITES))
        )
    {
        WICED_BT_TRACE("wiced_bt_pbc_init_getfile peer device does not support %s", p_get->p_rem_name);
        data.status = status;
        p_cb->p_cback(WICED_BT_PBC_GETFILE_EVT, &data);
        return;
    }

    /* Available for PBC only */
    if (p_cb->obx_oper == PBC_OP_NONE &&
        p_cb->sdp_service == UUID_SERVCLASS_PBAP_PSE)
    {
        p_cb->first_get_pkt = TRUE;
        p_obx->offset = 0;
        if ((p_obx->p_pkt = (BT_HDR *)wiced_bt_obex_header_init(p_cb->obx_handle, OBX_CMD_POOL_SIZE/*p_cb->peer_mtu*/)) != NULL)
        {

            /* add UID behind "X-BT-UID:" for get entry using UID */
            if (strlen(p_get->p_rem_name) == WICED_BT_PBC_UID_LEN)
            {
                BCM_STRCAT_S(temp_buf, sizeof(temp_buf), p_get->p_rem_name);
                is_ok = wiced_bt_obex_add_header_utf8((UINT8 *)p_obx->p_pkt, OBEX_HI_NAME, (UINT8 *)temp_buf);
            }
            else
            {
                /* Add the Name Header to the request */
                is_ok = wiced_bt_obex_add_header_utf8((UINT8 *)p_obx->p_pkt, OBEX_HI_NAME, (UINT8 *)p_get->p_rem_name);
            }

            if (p_cb->sdp_service == UUID_SERVCLASS_PBAP_PSE && is_ok)
            {
                /* add type header */
                if (p_get->obj_type == WICED_BT_PBC_GET_PB)
                    p = (UINT8 *)WICED_BT_PBC_PULL_PB_TYPE;
                is_ok = wiced_bt_obex_add_header((UINT8 *)p_obx->p_pkt, OBEX_HI_TYPE, (UINT8 *)p, strlen((char *)p) + 1);

                /* add app params for PCE */
                p_start = (UINT8 *)wiced_bt_obex_add_byte_sequence_start((UINT8 *)p_obx->p_pkt, &len);
                p = p_start;
                if (p_get->obj_type != WICED_BT_PBC_GET_PB || p_param->max_list_count != 0)
                {
                    if (p_param->filter)
                    {
                        *p++    = WICED_BT_PBC_APH_PROP_SELECTOR;
                        *p++    = 8;    /* Length field 8 */
#if (defined(WICED_BT_PBAP_1_2_SUPPORTED) && WICED_BT_PBAP_1_2_SUPPORTED == TRUE)
                        if ((p_cb->local_features & WICED_BT_PBC_SUP_FEA_UCI_VCARD_FIELD) &&
                            (p_cb->peer_features & WICED_BT_PBC_SUP_FEA_UCI_VCARD_FIELD))
                        {
                            p_param->filter |= WICED_BT_PBC_FILTER_X_BT_UCI;
                        }
                        if ((p_cb->local_features & WICED_BT_PBC_SUP_FEA_UID_VCARD_FIELD) &&
                            (p_cb->peer_features & WICED_BT_PBC_SUP_FEA_UID_VCARD_FIELD) &&
                            (p_cb->local_features & WICED_BT_PBC_SUP_FEA_CONTACT_REF) &&
                            (p_cb->peer_features & WICED_BT_PBC_SUP_FEA_CONTACT_REF))
                        {
                            p_param->filter |= WICED_BT_PBC_FILTER_X_BT_UID;
                        }
                        if ((p_cb->local_features & WICED_BT_PBC_SUP_FEA_DEF_CONTACT_IMAGE_FORMAT) &&
                            (p_cb->peer_features & WICED_BT_PBC_SUP_FEA_DEF_CONTACT_IMAGE_FORMAT))
                        {
                            p_param->filter |= WICED_BT_PBC_FILTER_PHOTO;
                        }
#endif
                        UINT64_TO_BE_STREAM(p, p_param->filter);
                    }
                    if (p_param->format < WICED_BT_PBC_FORMAT_MAX)
                    {
                        *p++    = WICED_BT_PBC_APH_FORMAT;
                        *p++    = 1;    /* Length field 1 */
                        *p++    = p_param->format;
                    }
                }

                if (p_get->obj_type == WICED_BT_PBC_GET_PB)
                {
                    /* this is mandatory */
                    *p++    = WICED_BT_PBC_APH_MAX_LIST_COUNT;
                    *p++    = 2;    /* Length field 2 */
                    UINT16_TO_BE_STREAM(p, p_param->max_list_count);
#if (defined(WICED_BT_PBAP_1_2_SUPPORTED) && WICED_BT_PBAP_1_2_SUPPORTED == TRUE)
                    /* Add APH resetnewmissedcalls with value 1 to reset NMC */
                    if ((p_param->is_reset_miss_calls == TRUE) && (p_cb->peer_features & WICED_BT_PBC_SUP_FEA_ENH_MISSED_CALLS)
                        && (p_cb->local_features & WICED_BT_PBC_SUP_FEA_ENH_MISSED_CALLS))
                    {
                        *p++    = WICED_BT_PBC_APH_RESET_NMC;
                        *p++    = 1;    /* Length field 1 */
                        *p++    = WICED_BT_PBC_NMC_RESET;    /* reset new missed calls */
                    }
                    /* Add APH VcardSelector and VcardSelectorOperator */
                    if ((p_cb->peer_features & WICED_BT_PBC_SUP_FEA_VCARD_SELECTING) &&
                        (p_cb->local_features & WICED_BT_PBC_SUP_FEA_VCARD_SELECTING))
                    {
                        if (p_param->selector)
                        {
                            *p++    = WICED_BT_PBC_APH_VCARD_SELE;
                            *p++    = 8;    /* Length field 8 */
                            UINT64_TO_BE_STREAM(p, p_param->selector);

                            *p++    = WICED_BT_PBC_APH_VCARD_SELE_OP;
                            *p++    = 1;    /* Length field 1 */
                            *p++    = p_param->selector_op;
                        }

                    }

#endif
                    if (p_param->list_start_offset)
                    {
                        *p++    = WICED_BT_PBC_APH_LIST_STOFF;
                        *p++    = 2;    /* Length field 2 */
                        UINT16_TO_BE_STREAM(p, p_param->list_start_offset);
                    }
                }

                /* If any of the app param header is added */
                if (p != p_start)
                {
                    wiced_bt_obex_add_byte_sequence_end((UINT8 *)p_obx->p_pkt, OBEX_HI_APP_PARMS, (UINT16)(p - p_start));
                }
            }
        }
        /* add .vcf for UID */
        if (strlen(p_get->p_rem_name) == WICED_BT_PBC_UID_LEN)
        {
            strcat(p_get->p_name, WICED_BT_PBC_UID_VCF_SUFFIX);
        }

        rem_name_len = (UINT16)(strlen(p_get->p_rem_name) + 1);
        /* Need a buffer that can hold the local path and UTF-8 filename */
        buflen = p_wiced_bt_pbc_fs_cfg->max_path_len + 2 + rem_name_len;

        /* Save the UNICODE name of the remote file, and local filename and path */
        if ((p_cb->p_name = (char *)GKI_getbuf(buflen)) != NULL)
        {
            memset(p_cb->p_name, 0, buflen);
            //BCM_STRCPY_S(p_cb->p_name, buflen, p_get->p_name);

            utl_strcpy(p_cb->p_name,  p_get->p_name);

            p_cb->obj_type = p_get->obj_type;
            p_cb->obx_oper = PBC_OP_GET_FILE;
            if(p_cb->obj_type == WICED_BT_PBC_GET_PB && p_param->max_list_count == 0)
            {
                /* getting the phone book size only. do not need to open a file */
                wiced_bt_pbc_send_get_req(p_cb);
            }
            else
            {
                p_cb->cout_active = TRUE;
                wiced_bt_pbc_co_open(WICED_BT_PBC_CI_OPEN_EVT, p_cb->app_id);
            }
            status = WICED_BT_PBC_OK;
        }
    }

    if (status != WICED_BT_PBC_OK)
    {
        data.status = status;
        p_cb->p_cback(WICED_BT_PBC_GETFILE_EVT, &data);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_chdir
**
** Description      Change Directory and get a listing of it.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_chdir(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    wiced_bt_pbc_api_chdir_t  *p_chdir = &p_data->api_chdir;
    wiced_bt_pbc_obx_pkt_t    *p_obx = &p_cb->obx;
    wiced_bt_obex_setpath_flag_t obx_flags = OBEX_SPF_NO_CREATE;
    wiced_bt_pbc_status_t      status = WICED_BT_PBC_FAIL;
    UINT16               len = OBX_CMD_POOL_SIZE;    /* Don't need special OBX buffer pool */

    /* Only process if no other OBX operation is active and connected to PBC */
    if (p_cb->obx_oper == PBC_OP_NONE)
    {
        if(p_cb->sdp_service == UUID_SERVCLASS_PBAP_PSE)
        {
            /* Allocate an OBX packet */
            if ((p_obx->p_pkt = (BT_HDR *)wiced_bt_obex_header_init(p_cb->obx_handle, len)) != NULL)
            {
                status = WICED_BT_PBC_OK;

                /* Add the name header if not backing up */
                if (p_chdir->flag != WICED_BT_PBC_FLAG_BACKUP)
                {
                    if (!wiced_bt_obex_add_header_utf8((UINT8 *)p_obx->p_pkt, OBEX_HI_NAME, (UINT8 *)p_chdir->p_dir))
                        status = WICED_BT_PBC_FAIL;
                }
                else
                    obx_flags |= OBEX_SPF_BACKUP;
            }
        }
    }

    if (status == WICED_BT_PBC_OK)
    {
        if (wiced_bt_obex_send_request(p_cb->obx_handle, OBEX_REQ_SETPATH, (wiced_bt_obex_req_param_t*)&obx_flags, (UINT8 *)p_obx->p_pkt))
        {
            status = WICED_BT_PBC_FAIL;
        }
        else
        {
            p_cb->req_pending = TRUE;
            p_cb->obx_oper = PBC_OP_CHDIR;
            p_obx->p_pkt = NULL;    /* OBX will free the packet */
        }
    }

    if (status != WICED_BT_PBC_OK)   /* Send an error response to the application */
    {
        utl_freebuf((void**)&p_obx->p_pkt);
        p_cb->p_cback(WICED_BT_PBC_CHDIR_EVT, (wiced_bt_pbc_t *)&status);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_list_dir
**
** Description      List the specified directory contents.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_listdir(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    wiced_bt_pbc_t err_rsp;

    /* Check whether peer device supports the requested repository type */
    if ((!strcmp(p_data->api_list.p_dir, WICED_BT_PBC_PULL_LIST_SPD_NAME) && !(p_cb->peer_repositories & WICED_BT_PBC_REPOSIT_SPEED_DIAL)) ||
        (!strcmp(p_data->api_list.p_dir, WICED_BT_PBC_PULL_LIST_FAV_NAME) && !(p_cb->peer_repositories & WICED_BT_PBC_REPOSIT_FAVORITES))
        )
    {
        WICED_BT_TRACE("wiced_bt_pbc_listdir peer device does not support %s", p_data->api_list.p_dir);

        err_rsp.list.len = 0;
        err_rsp.list.status = WICED_BT_PBC_FAIL;
        err_rsp.list.final = TRUE;
        err_rsp.list.data = NULL;
        p_cb->p_cback(WICED_BT_PBC_LIST_EVT, &err_rsp);
        return;
    }

    /* Only process if no other OBX operation is active and connected to PBC */
    if (p_cb->obx_oper == PBC_OP_NONE)
        wiced_bt_pbc_get_listing(p_cb, p_data->api_list.p_dir, p_data->api_list.p_param);
    else
        wiced_bt_pbc_listing_err(&p_cb->obx.p_pkt, WICED_BT_PBC_FAIL);
}


/*******************************************************************************
**
** Function         wiced_bt_pbc_send_auth_rsp
**
** Description      Pass the response to an authentication request back to the
**                  client.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_send_authrsp(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
#if 0
    //NOT IMPLEMENTED
    UINT8 *p_pwd = NULL;
    UINT8 *p_userid = NULL;

    if (p_data->auth_rsp.key_len > 0)
        p_pwd = (UINT8 *)p_data->auth_rsp.key;
    if (p_data->auth_rsp.userid_len > 0)
        p_userid = (UINT8 *)p_data->auth_rsp.userid;

     wiced_bt_obx_auth_response(p_cb->obx_handle, p_pwd, p_data->auth_rsp.key_len,
                         p_userid, p_data->auth_rsp.userid_len, TRUE);
#endif
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_trans_cmpl
**
** Description      pull complete
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_trans_cmpl(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    wiced_bt_pbc_t param;

    /* Close the opened file */
    if (p_cb->fd >= 0)
    {
        p_cb->fd = WICED_BT_PBC_INVALID_FD;
    }

    param.status = wiced_bt_pbc_convert_obx_to_pbc_status(p_data->obx_evt.rsp_code);

    if (p_cb->obx_oper == PBC_OP_GET_FILE ||
        p_cb->obx_oper == PBC_OP_GET_LIST)
    {
        if (param.status != WICED_BT_PBC_OK)
        {
            WICED_BT_TRACE("PBC: Get File Operation Aborted or Error [%s], status 0x%02x",
                                 p_cb->p_name, param.status);
        }

        p_cb->p_cback(WICED_BT_PBC_GETFILE_EVT, &param);
    }

    /* Clean up control block */
    pbc_reset_cb(p_cb);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_ci_write
**
** Description      Continue with the current write operation
**                  (Get File processing)
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_ci_write(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    wiced_bt_pbc_obx_pkt_t *p_obx = &p_cb->obx;
    wiced_bt_pbc_t          param;
    UINT8             rsp_code = OBEX_RSP_FAILED;

    WICED_BT_TRACE("wiced_bt_pbc_ci_write");

    p_cb->cout_active = FALSE;

    /* Process write call-in event if operation is still active */
    if (p_cb->obx_oper != PBC_OP_GET_FILE)
    {
        WICED_BT_TRACE("wiced_bt_pbc_ci_write p_cb->obx_oper != PBC_OP_GET_FILE *** ERROR LEAK ****");
        return;
    }

    /* Free current packet */
    utl_freebuf((void**)&p_obx->p_pkt);

    if (p_data->write_evt.status == WICED_BT_PBC_CO_OK)
    {
        param.prog.bytes = p_obx->offset;
        param.prog.file_size = p_cb->file_size;
        p_cb->p_cback(WICED_BT_PBC_PROGRESS_EVT, &param);

        /* Send another Get request if not finished */
        if (!p_obx->final_pkt)
        {
            /* send a new request */
            wiced_bt_pbc_send_get_req(p_cb);
            rsp_code = OBEX_RSP_CONTINUE;
        }
        else
            rsp_code = OBEX_RSP_OK;
    }

    if (rsp_code != OBEX_RSP_CONTINUE)
    {
        p_data->obx_evt.rsp_code = rsp_code;
        wiced_bt_pbc_sm_execute(p_cb, WICED_BT_PBC_OBX_CMPL_EVT, p_data);
    }
}


/*******************************************************************************
**
** Function         wiced_bt_pbc_ci_open
**
** Description      Continue with the current file open operation
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_ci_open(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    wiced_bt_pbc_ci_open_evt_t *p_open = &p_data->open_evt;
    UINT8 rsp_code = OBEX_RSP_FAILED;

    WICED_BT_TRACE("wiced_bt_pbc_ci_open");

    p_cb->cout_active = FALSE;

    /* if file is accessible read/write the first buffer of data */
    if (p_open->status == WICED_BT_PBC_CO_OK)
    {
        p_cb->fd = p_open->fd;

        if (p_cb->obx_oper == PBC_OP_GET_FILE)
        {
            /* Initiate the first OBX GET request */
            rsp_code = wiced_bt_pbc_send_get_req(p_cb);
        }
    }
    else
    {
        if (p_open->status == WICED_BT_PBC_CO_EACCES)
            rsp_code = OBEX_RSP_UNAUTHORIZED;
        else    /* File could not be found */
            rsp_code = OBEX_RSP_NOT_FOUND;
    }

    if (rsp_code != OBEX_RSP_OK)
    {
        p_data->obx_evt.rsp_code = rsp_code;
        wiced_bt_pbc_sm_execute(p_cb, WICED_BT_PBC_OBX_CMPL_EVT, p_data);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_obx_conn_rsp
**
** Description      Process the OBX connect event.
**
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_obx_conn_rsp(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    wiced_bt_pbc_obx_evt_t    *p_evt = &p_data->obx_evt;
    wiced_bt_pbc_t            param;

    p_cb->peer_mtu = p_data->obx_evt.param.conn.mtu;

    if (p_cb->sdp_service == UUID_SERVCLASS_PBAP_PSE)
    {
      //  param.open.service = WICED_BT_PBAP_SERVICE_ID;
        param.open.peer_features = p_cb->peer_features;
        param.open.peer_repositories = p_cb->peer_repositories;
        WICED_BT_TRACE("[BTA PBC]PBAP Service Id ");
    }

    p_cb->p_cback(WICED_BT_PBC_OPEN_EVT, &param);

    /* Done with Obex packet */
    utl_freebuf((void**)&p_evt->p_pkt);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_obx_abort_rsp
**
** Description      Process the OBX abort event
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_obx_abort_rsp(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
   // bta_sys_stop_timer(&p_cb->rsp_timer);
    WICED_BT_TRACE("wiced_bt_pbc_obx_abort_rsp");

    wiced_stop_timer(&p_cb->rsp_timer.wiced_timer);
    wiced_deinit_timer(&p_cb->rsp_timer.wiced_timer);


    /* Done with Obex packet */
    utl_freebuf((void**)&p_data->obx_evt.p_pkt);

    if (p_cb->obx_oper != PBC_OP_NONE)
    {
        if (!p_cb->cout_active)
        {
            p_data->obx_evt.rsp_code = OBEX_RSP_GONE; /* indicates aborted; used internally */
            wiced_bt_pbc_sm_execute(p_cb, WICED_BT_PBC_OBX_CMPL_EVT, p_data);
        }
    }
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_obx_password
**
** Description      Process the OBX password request
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_obx_password(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    wiced_bt_pbc_obx_evt_t    *p_evt = &p_data->obx_evt;
    wiced_bt_pbc_auth_t        parms;
    //tOBX_AUTH_OPT        options;

    memset(&parms, 0, sizeof(wiced_bt_pbc_auth_t));

// TEMPORARILY LEFT IN

//    /* Extract user id from packet (if available) */
//    if (wiced_bt_obx_read_challenge(p_evt->p_pkt, &parms.realm_charset,
//                          &parms.p_realm,
//                          &parms.realm_len, &options))
//    {
//        if (options & OBX_AO_USR_ID)
//            parms.userid_required = TRUE;
//    }
//
    /* Don't need OBX packet any longer */
    utl_freebuf((void**)&p_evt->p_pkt);

    /* Notify application */
    p_cb->p_cback(WICED_BT_PBC_AUTH_EVT, (wiced_bt_pbc_t *)&parms);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_obx_timeout
**
** Description      Process the OBX timeout event
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_obx_timeout(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    WICED_BT_TRACE("wiced_bt_pbc_obx_timeout");

    utl_freebuf((void**)&p_data->obx_evt.p_pkt);

    /* If currently processing OBX request, peer is unresponsive so we're done */
    if (p_cb->aborting || p_cb->obx_oper != PBC_OP_NONE)
    {
        p_data->obx_evt.rsp_code = OBEX_RSP_GONE;

        /* Start stop response timer */
        p_cb->timer_oper = PBC_TIMER_OP_STOP;

        wiced_stop_timer(&p_cb->rsp_timer.wiced_timer);
        wiced_deinit_timer(&p_cb->rsp_timer.wiced_timer);

        wiced_init_timer(&p_cb->rsp_timer.wiced_timer, wiced_bt_pbc_rsp_timeout_cback, (uint32_t)&p_cb->rsp_timer, WICED_SECONDS_TIMER);
        wiced_start_timer(&p_cb->rsp_timer.wiced_timer, p_wiced_bt_pbc_cfg->stopabort_tout);

        wiced_bt_obex_disconnect(p_cb->obx_handle, NULL);
        wiced_bt_pbc_sm_execute(p_cb, WICED_BT_PBC_OBX_CLOSE_EVT, p_data);
    }
}


/*******************************************************************************
**
** Function         wiced_bt_pbc_obx_get_rsp
**
** Description      Process the OBX file get and folder listing events
**                  If the type header is not folder listing, then pulling a file.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_obx_get_rsp(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    wiced_bt_pbc_obx_evt_t  *p_evt = &p_data->obx_evt;

    p_cb->req_pending = FALSE;

    if (p_cb->obx_oper == PBC_OP_GET_FILE)
        wiced_bt_pbc_proc_get_rsp(p_cb, p_data);
    else if (p_cb->obx_oper == PBC_OP_GET_LIST)
        wiced_bt_pbc_proc_list_data(p_cb, p_evt);
    else    /* Release the unexpected OBX response packet */
        utl_freebuf((void**)&p_evt->p_pkt);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_obx_set_path_rsp
**
** Description      Process the response to a change directory requests
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_obx_setpath_rsp(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    wiced_bt_pbc_obx_evt_t  *p_evt = &p_data->obx_evt;
    wiced_bt_pbc_status_t     status = WICED_BT_PBC_FAIL;
    wiced_bt_pbc_evt_t        event = WICED_BT_PBC_CHDIR_EVT;

    p_cb->req_pending = FALSE;

    if (p_evt->rsp_code == OBEX_RSP_OK)
        status = WICED_BT_PBC_OK;

    p_cb->obx_oper = PBC_OP_NONE;

    utl_freebuf((void**)&p_evt->p_pkt); /* Done with Obex packet */
    p_cb->p_cback(event, (wiced_bt_pbc_t *)&status);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_rsp_timeout
**
** Description      Process the OBX response timeout event
**                  stop and abort
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_rsp_timeout(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    WICED_BT_TRACE("wiced_bt_pbc_rsp_timeout");

    if (p_cb->timer_oper == PBC_TIMER_OP_ABORT)
    {
        /* Start stop response timer */
        p_cb->timer_oper = PBC_TIMER_OP_STOP;

        wiced_stop_timer(&p_cb->rsp_timer.wiced_timer);
        wiced_deinit_timer(&p_cb->rsp_timer.wiced_timer);

        wiced_init_timer(&p_cb->rsp_timer.wiced_timer, wiced_bt_pbc_rsp_timeout_cback, (uint32_t)&p_cb->rsp_timer, WICED_SECONDS_TIMER);
        wiced_start_timer(&p_cb->rsp_timer.wiced_timer, p_wiced_bt_pbc_cfg->stopabort_tout);

        wiced_bt_obex_disconnect(p_cb->obx_handle, NULL);
    }
    else    /* Timeout waiting for disconnect response */
    {
        p_cb->cout_active = FALSE;
        wiced_bt_pbc_initialize(p_cb, p_data);

        p_cb->p_cback(WICED_BT_PBC_CLOSE_EVT, (wiced_bt_pbc_t *)NULL);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_initialize
**
** Description      Initialize the control block.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_initialize(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    wiced_bt_pbc_co_status_t   status = 0;

    WICED_BT_TRACE("wiced_bt_pbc_initialize");

    wiced_stop_timer(&p_cb->rsp_timer.wiced_timer);
    wiced_deinit_timer(&p_cb->rsp_timer.wiced_timer);

    /* Close any open files */
    if (p_cb->fd >= 0)
    {
        wiced_bt_pbc_co_close(p_cb->fd, p_cb->app_id);
        p_cb->fd = WICED_BT_PBC_INVALID_FD;

        /* Delete an aborted unfinished get file operation */
        if (p_cb->obx_oper == PBC_OP_GET_FILE)
        {
            WICED_BT_TRACE("PBC: Remove ABORTED Get File Operation [%s], status 0x%02x",
                                 p_cb->p_name, status);
        }
    }

    /* Clean up control block */
    pbc_reset_cb(p_cb);
    p_cb->sdp_service = 0;
    p_cb->sdp_pending = FALSE;

    if (p_cb->disabling)
    {
        if (p_cb->sdp_handle)
        {
            p_cb->sdp_handle = 0;
        }
        p_cb->is_enabled = FALSE;
        p_cb->disabling = FALSE;
        wiced_bt_pbc_sm_execute(p_cb, WICED_BT_PBC_DISABLE_CMPL_EVT, NULL);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_stop_client
**
** Description      Stop OBX client.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_stop_client(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    WICED_BT_TRACE("wiced_bt_pbc_stop_client");

    if (!p_cb->sdp_pending)
    {
        /* Start stop response timer */
        wiced_stop_timer(&p_cb->rsp_timer.wiced_timer);
        wiced_deinit_timer(&p_cb->rsp_timer.wiced_timer);

        wiced_init_timer(&p_cb->rsp_timer.wiced_timer, wiced_bt_pbc_rsp_timeout_cback, (uint32_t)&p_cb->rsp_timer, WICED_SECONDS_TIMER);
        wiced_start_timer(&p_cb->rsp_timer.wiced_timer, p_wiced_bt_pbc_cfg->stopabort_tout);

        p_cb->timer_oper = PBC_TIMER_OP_STOP;
        wiced_bt_obex_disconnect(p_cb->obx_handle, NULL);
    }

#if WICED_BT_PBC_DEBUG == TRUE
    else
    {
        WICED_BT_TRACE("wiced_bt_pbc_stop_client: Waiting for SDP to complete");
    }
#endif
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_close
**
** Description      If not waiting for a call-in function, complete the closing
**                  of the channel.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_close(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    /* finished if not waiting on a call-in function */
    if (!p_cb->sdp_pending && !p_cb->cout_active)
        wiced_bt_pbc_sm_execute(p_cb, WICED_BT_PBC_CLOSE_CMPL_EVT, p_data);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_start_client
**
** Description      Start an PBC client.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_start_client(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    wiced_bt_pbc_obx_pkt_t *p_obx = &p_cb->obx;
    wiced_bt_obex_status_t      status = OBEX_NO_RESOURCES;

#if (defined(WICED_BT_PBAP_1_2_SUPPORTED) && WICED_BT_PBAP_1_2_SUPPORTED == TRUE)
    BOOLEAN         use_srm = FALSE;
    wiced_bt_obex_triplet_t app_param;
    UINT8           buf[4], *p;
#endif

    /* save peer supported features */
    WICED_BT_TRACE("wiced_bt_pbc_start_client peer_features = 0x%08x peer_repositories = 0x%02x",
                       p_data->sdp_ok.peer_features, p_data->sdp_ok.peer_repositories);
    p_cb->peer_features = p_data->sdp_ok.peer_features;
    p_cb->peer_repositories = p_data->sdp_ok.peer_repositories;

    /* Allocate an OBX packet */
    if ((p_obx->p_pkt = (BT_HDR *)wiced_bt_obex_header_init(OBEX_HANDLE_NULL, OBEX_MAX_MTU)) != NULL)
    {
        if (p_cb->sdp_service == UUID_SERVCLASS_PBAP_PSE)
        {
            wiced_bt_obex_add_header((UINT8 *)p_obx->p_pkt, OBEX_HI_TARGET, (UINT8 *)WICED_BT_PBC_PB_ACCESS_TARGET_UUID,
                             WICED_BT_PBC_UUID_LENGTH);
#if (defined(WICED_BT_PBAP_1_2_SUPPORTED) && WICED_BT_PBAP_1_2_SUPPORTED == TRUE)
            /* Add supported features only if peer SDP returns support features */
            if (p_data->sdp_ok.is_peer_features_present)
            {
                app_param.tag = WICED_BT_PBC_APH_SUP_FEA;
                app_param.len = 4;
                p = buf;
                UINT32_TO_BE_STREAM(p, p_cb->local_features);
                app_param.p_array = (UINT8*) buf;
                wiced_bt_obex_add_triplet_header((UINT8 *)p_obx->p_pkt, OBEX_HI_APP_PARMS, &app_param, 1);
            }
#endif
        }

#if (defined(WICED_BT_PBAP_1_2_SUPPORTED) && WICED_BT_PBAP_1_2_SUPPORTED == TRUE)
        if ((status = wiced_bt_obex_alloc_session (NULL, p_data->sdp_ok.scn,
                                        &p_data->sdp_ok.psm,
                                        wiced_bt_pbc_obx_cback,
                                        &p_cb->obx_handle)) == OBEX_SUCCESS)
        {
            /* set security level */
            if (p_data->sdp_ok.scn)     /* if SCN is not '0', then we are in legacy mode */
            {
                BTM_SetSecurityLevel (TRUE, "WICED_BT_PBC", BTM_SEC_SERVICE_PBAP,
                                      p_cb->sec_mask, BT_PSM_RFCOMM,
                                      BTM_SEC_PROTO_RFCOMM, p_data->sdp_ok.scn);
            }
            else    /* Using Obex over L2CAP */
            {
                BTM_SetSecurityLevel (TRUE, "WICED_BT_PBC", BTM_SEC_SERVICE_PBAP,
                                      p_cb->sec_mask, p_data->sdp_ok.psm, 0, 0);
                use_srm = TRUE;
            }

            if ((wiced_bt_obex_create_session (p_cb->bd_addr, OBEX_MAX_MTU, use_srm, 0,
                               p_cb->obx_handle, p_obx->p_pkt)) == OBEX_SUCCESS)
            {
                p_obx->p_pkt = NULL;    /* OBX will free the memory */
            }
        }

#else   /* Version 1.1 or earlier so we are good to go */
        /* set security level */

        //TODO Need security level setting

        wiced_bt_obex_connect(p_cb->bd_addr, p_data->sdp_ok.scn, OBEX_MAX_MTU,
                       wiced_bt_pbc_obx_cback, &p_cb->obx_handle, (UINT8 *)p_obx->p_pkt);
        p_obx->p_pkt = NULL;    /* OBX will free the memory */

        status = OBEX_SUCCESS;
#endif
    }

    /* Something failed along the way */
    if (status != OBEX_SUCCESS)
    {
        utl_freebuf((void**)&p_obx->p_pkt);
        p_data->obx_evt.rsp_code = OBEX_RSP_FAILED;
        wiced_bt_pbc_sm_execute(p_cb, WICED_BT_PBC_OBX_CLOSE_EVT, p_data);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_free_db
**
** Description      Free buffer used for service discovery database.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_free_db(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    utl_freebuf((void**)&p_cb->p_db);
    p_cb->sdp_pending = FALSE;
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_ignore_obx
**
** Description      Free OBX packet for ignored OBX events.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_ignore_obx(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    utl_freebuf((void**)&p_data->obx_evt.p_pkt);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_find_service
**
** Description      Perform service discovery to find OPP and/or PBC services on
**                  peer device. If the service ID is both PBC and OPP then PBC
**                  is tried first.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_find_service(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    //tSDP_UUID   uuid_list;
    wiced_bt_uuid_t             uuid_list;

#if (defined(WICED_BT_PBAP_1_2_SUPPORTED) && WICED_BT_PBAP_1_2_SUPPORTED == TRUE)
    UINT16      attr_list[6];
    UINT16      num_attrs = 6;
#else
    UINT16      attr_list[5];
    UINT16      num_attrs = 5;
#endif
    tWICED_BT_SERVICE_MASK   pbc_services = (WICED_BT_PBAP_SERVICE_MASK);

    /* Make sure at least one service was specified */
    if ((p_cb->services & pbc_services) && p_cb->sdp_pending)
    {
        if ((p_cb->p_db = (wiced_bt_sdp_discovery_db_t *) GKI_getbuf(WICED_BT_PBC_DISC_SIZE)) != NULL)
        {
            attr_list[0] = ATTR_ID_SERVICE_CLASS_ID_LIST;
            attr_list[1] = ATTR_ID_PROTOCOL_DESC_LIST;
            attr_list[2] = ATTR_ID_BT_PROFILE_DESC_LIST;
            /* always search peer features */
            attr_list[3] = ATTR_ID_SUPPORTED_FEATURES_32;

#if (defined(WICED_BT_PBAP_1_2_SUPPORTED) && WICED_BT_PBAP_1_2_SUPPORTED == TRUE)
            attr_list[4] = ATTR_ID_OBX_OVR_L2CAP_PSM;
            attr_list[5] = ATTR_ID_SUPPORTED_REPOSITORIES;

#endif
            uuid_list.len = LEN_UUID_16;

            if (p_cb->services & WICED_BT_PBAP_SERVICE_MASK)
            {
                p_cb->services &= ~WICED_BT_PBAP_SERVICE_MASK;
                p_cb->sdp_service = UUID_SERVCLASS_PBAP_PSE;
                uuid_list.uu.uuid16 = UUID_SERVCLASS_PBAP_PSE;
            }

            wiced_bt_sdp_init_discovery_db(p_cb->p_db, WICED_BT_PBC_DISC_SIZE, 1, &uuid_list, num_attrs, attr_list);
            if(!wiced_bt_sdp_service_search_attribute_request(p_cb->bd_addr, p_cb->p_db, wiced_bt_pbc_sdp_cback))
            {
                wiced_bt_pbc_sm_execute(p_cb, WICED_BT_PBC_SDP_FAIL_EVT, p_data);

            }
        }
    }
    else    /* No services available */
        wiced_bt_pbc_sm_execute(p_cb, WICED_BT_PBC_SDP_FAIL_EVT, p_data);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_close_complete
**
** Description      Finishes the memory cleanup after a channel is closed.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_close_complete(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    p_cb->cout_active = FALSE;
    wiced_bt_pbc_initialize(p_cb, p_data);

    p_cb->p_cback(WICED_BT_PBC_CLOSE_EVT, (wiced_bt_pbc_t *)NULL);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_disable_complete
**
** Description      Sends a disable event to the client
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_disable_complete(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    /*Send disable complete event up to the caller*/
    if(p_cb->p_cback)
        p_cb->p_cback(WICED_BT_PBC_DISABLE_EVT, (wiced_bt_pbc_t *)NULL);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_set_disable
**
** Description      Finishes the memory cleanup after a channel is closed.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_set_disable(wiced_bt_pbc_cb_t *p_cb, wiced_bt_pbc_data_t *p_data)
{
    p_cb->disabling = TRUE;
}



/*****************************************************************************
**  Callback Functions
*****************************************************************************/
/*******************************************************************************
**
** Function         wiced_bt_pbc_obx_cback
**
** Description      OBX callback function.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_obx_cback (wiced_bt_obex_handle_t handle, wiced_bt_obex_event_t obx_event, UINT8 rsp_code,
        wiced_bt_obex_evt_param_t param, UINT8 *p_pkt)
{
    wiced_bt_pbc_obx_evt_t *p_obx_msg;
    UINT16              event = 0;

#if WICED_BT_PBC_DEBUG == TRUE
    WICED_BT_TRACE("OBX Event Callback: obx_event [%s]", pbc_obx_evt_code(obx_event));
#endif

    switch(obx_event)
    {
    case OBEX_CONNECT_RSP_EVT:
        if (rsp_code == OBEX_RSP_OK)
        {
            event = WICED_BT_PBC_OBX_CONN_RSP_EVT;
        }
        else    /* Obex will disconnect underneath BTA */
        {
            WICED_BT_TRACE("PBC_CBACK: Bad connect response 0x%02x", rsp_code);
            if (p_pkt)
                GKI_freebuf(p_pkt);
            return;
        }
        break;
    case OBEX_GET_RSP_EVT:
        event = WICED_BT_PBC_OBX_GET_RSP_EVT;
        break;
    case OBEX_SETPATH_RSP_EVT:
        event = WICED_BT_PBC_OBX_SETPATH_RSP_EVT;
        break;
    case OBEX_ABORT_RSP_EVT:
        event = WICED_BT_PBC_OBX_ABORT_RSP_EVT;
        break;
    case OBEX_CLOSE_IND_EVT:
        event = WICED_BT_PBC_OBX_CLOSE_EVT;
        break;
    case OBEX_TIMEOUT_EVT:
        event = WICED_BT_PBC_OBX_TOUT_EVT;
        break;
    case OBEX_PASSWORD_EVT:
        event = WICED_BT_PBC_OBX_PASSWORD_EVT;
        break;

    default:
/*  case OBX_DISCONNECT_RSP_EVT: Handled when OBX_CLOSE_IND_EVT arrives */
        if (p_pkt)
            GKI_freebuf(p_pkt);
        return;
    }

    /* send event to BTA, if any */
    if ((p_obx_msg = (wiced_bt_pbc_obx_evt_t *) GKI_getbuf(sizeof(wiced_bt_pbc_obx_evt_t))) != NULL)
    {
        p_obx_msg->hdr.event = event;
        p_obx_msg->obx_event = obx_event;
        p_obx_msg->handle = handle;
        p_obx_msg->rsp_code = rsp_code;
        p_obx_msg->param = param;
        p_obx_msg->p_pkt = (BT_HDR *)p_pkt;

        wiced_bt_pbc_hdl_event((BT_HDR *) p_obx_msg);

        GKI_freebuf(p_obx_msg);
    }
}

/******************************************************************************
**
** Function         wiced_bt_pbc_sdp_cback
**
** Description      This is the SDP callback function used by PBC.
**                  This function will be executed by SDP when the service
**                  search is completed.  If the search is successful, it
**                  finds the first record in the database that matches the
**                  UUID of the search.  Then retrieves the scn from the
**                  record.
**
** Returns          Nothing.
**
******************************************************************************/
static void wiced_bt_pbc_sdp_cback(UINT16 status)
{
    wiced_bt_pbc_cb_t         *p_cb = &wiced_bt_pbc_cb;
    tWICED_BT_PBC_SDP_OK_EVT *p_buf;
    wiced_bt_sdp_discovery_record_t       *p_rec = NULL;
    wiced_bt_sdp_protocol_elem_t   pe;
    UINT16               psm = 0;
    UINT16               version = WICED_BT_PBC_VERSION_1_1;
    UINT8                scn = 0;
    BOOLEAN              found = FALSE;
    //tSDP_DISC_ATTR      *p_attr;
    wiced_bt_sdp_discovery_attribute_t *p_attr = NULL;
    UINT32               peer_features = WICED_BT_PBC_DEFAULT_SUPPORTED_FEATURES;
    UINT8                peer_repositories = WICED_BT_PBC_DEFAULT_REPOSITORIES;
    BOOLEAN              is_peer_features_present = FALSE;

    WICED_BT_TRACE("wiced_bt_pbc_sdp_cback status:%d", status);

    if (status == WICED_BT_SDP_SUCCESS)
    {
        /* loop through all records we found */
        do
        {
            /* get next record; if none found, we're done */
            if ((p_rec = wiced_bt_sdp_find_service_in_db(p_cb->p_db, p_cb->sdp_service,
                                             p_rec)) == NULL)
                break;

            if (wiced_bt_sdp_find_profile_version_in_rec (p_rec, UUID_SERVCLASS_PHONE_ACCESS, &version))
            {
                WICED_BT_TRACE("Peer Version %04x", version);
            }

            /* Look for supported repositories */
            if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec,
                            ATTR_ID_SUPPORTED_REPOSITORIES)) != NULL)
            {
                peer_repositories = p_attr->attr_value.v.u8;
                WICED_BT_TRACE("wiced_bt_pbc_sdp_cback peer_repositories=0x%02x", peer_repositories);

            }
            /* No supported repositories, use the default */
            else
            {
                WICED_BT_TRACE("wiced_bt_pbc_sdp_cback peer supported repositories not found use default");
            }
#if (defined(WICED_BT_PBAP_1_2_SUPPORTED) && WICED_BT_PBAP_1_2_SUPPORTED == TRUE)
            /* If profile version is 1.2 or greater, look for supported features and L2CAP PSM */
            if (version >= WICED_BT_PBC_VERSION_1_2)
            {
                /* Look for peer suppored features */
                if ((p_attr = wiced_bt_sdp_find_attribute_in_rec(p_rec,
                                ATTR_ID_SUPPORTED_FEATURES_32)) != NULL)
                {
                    peer_features = p_attr->attr_value.v.u32;
                    is_peer_features_present = TRUE;
                    WICED_BT_TRACE("wiced_bt_pbc_sdp_cback peer_features=0x%08x", peer_features);
                }
                /* No supported features, use the default */
                else
                {
                    WICED_BT_TRACE("wiced_bt_pbc_sdp_cback peer supported features not found use default");
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
                        WICED_BT_TRACE("wiced_bt_pbc_sdp_cback psm=0x%02x (Using PBAP 1.2)",psm);
                        break;
                    }
                    /* Bad PSM, look for next record */
                    else
                    {
                        WICED_BT_TRACE("wiced_bt_pbc_sdp_cback BAD psm=0x%02x",psm);
                        psm = 0;    /* Reset PSM */
                        continue;
                    }
                }
                /* No L2CAP PSM, ignore this record since 1.2 and later requires this!  */
                else
                {
                    WICED_BT_TRACE("bta_mce_sdp_cback PSM for L2CAP not found");
                    continue;

                }
            }

#endif /* WICED_BT_PBAP_1_2_SUPPORTED */

            /* get scn from proto desc list; if not found, go to next record */
            if (!found && wiced_bt_sdp_find_protocol_list_elem_in_rec(p_rec, UUID_PROTOCOL_RFCOMM, &pe))
            {

                found = TRUE;
                scn = (UINT8) pe.params[0];

                /* we've got everything, we're done */
                break;
            }
            else
            {
                continue;
            }
        } while (TRUE);
    }

    /* send result in event back to BTA */
    if ((p_buf = (tWICED_BT_PBC_SDP_OK_EVT *) GKI_getbuf(sizeof(tWICED_BT_PBC_SDP_OK_EVT))) != NULL)
    {
        p_buf->hdr.event = WICED_BT_PBC_SDP_FAIL_EVT;

        if (status == WICED_BT_SDP_SUCCESS)
        {
            if (found == TRUE)
            {
                p_buf->hdr.event = WICED_BT_PBC_SDP_OK_EVT;
                p_buf->scn = scn;
                p_buf->psm = psm;
                p_buf->is_peer_features_present = is_peer_features_present;
                p_buf->peer_features = peer_features;
                p_buf->peer_repositories = peer_repositories;
            }
        }

        wiced_bt_pbc_hdl_event((BT_HDR*)p_buf);  // TODO SDP Callback?!
        GKI_freebuf(p_buf);
    }
}

/*****************************************************************************
**  Local PBC Event Processing Functions
*****************************************************************************/
static void pbc_reset_cb (wiced_bt_pbc_cb_t *p_cb)
{
    /* Clean up control block */
    utl_freebuf((void**)&p_cb->obx.p_pkt);
    utl_freebuf((void**)(BT_HDR **)&p_cb->p_name);
    p_cb->obx_oper = PBC_OP_NONE;
    p_cb->aborting = FALSE;
    p_cb->req_pending = FALSE;
}


/*****************************************************************************
**  Debug Functions
*****************************************************************************/
#if WICED_BT_PBC_DEBUG == TRUE

/*******************************************************************************
**
** Function         pbc_obx_evt_code
**
** Description
**
** Returns          void
**
*******************************************************************************/
static char *pbc_obx_evt_code(wiced_bt_obex_event_t evt_code)
{
    switch(evt_code)
    {
    case OBEX_CONNECT_RSP_EVT:
        return "OBEX_CONNECT_RSP_EVT";
    case OBEX_DISCONNECT_RSP_EVT:
        return "OBEX_DISCONNECT_RSP_EVT";
    case OBEX_GET_RSP_EVT:
        return "OBEX_GET_RSP_EVT";
    case OBEX_SETPATH_RSP_EVT:
        return "OBEX_SETPATH_RSP_EVT";
    case OBEX_ABORT_RSP_EVT:
        return "OBEX_ABORT_RSP_EVT";
    case OBEX_CLOSE_IND_EVT:
        return "OBEX_CLOSE_IND_EVT";
    case OBEX_TIMEOUT_EVT:
        return "OBEX_TIMEOUT_EVT";
    case OBEX_PASSWORD_EVT:
        return "OBEX_PASSWORD_EVT";
    default:
        return "unknown OBX event code";
    }
}
#endif  /* Debug Functions */
