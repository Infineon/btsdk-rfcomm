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

/** @file
 *
 * WICED BT OPP server action functions
 *
 */

#include <string.h>
#include "stdio.h"

#include "wiced.h"
#include "wiced_bt_types.h"

#include "wiced_bt_sdp.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_bt_trace.h"

#include "wiced_bt_ops_api.h"
#include "wiced_bt_ops_int.h"

#include "wiced_bt_obex.h"
#include "wiced_bt_ops_co.h"


/*****************************************************************************
**  Constants
*****************************************************************************/

#define WICED_BT_OPS_NUM_FMTS        7
#define WICED_BT_OPS_PROTOCOL_COUNT  3

/* object format lookup table */
const wiced_bt_op_fmt_t wiced_bt_ops_obj_fmt[] =
{
    WICED_BT_OP_VCARD21_FMT,
    WICED_BT_OP_VCARD30_FMT,
    WICED_BT_OP_VCAL_FMT,
    WICED_BT_OP_ICAL_FMT,
    WICED_BT_OP_VNOTE_FMT,
    WICED_BT_OP_VMSG_FMT,
    WICED_BT_OP_OTHER_FMT
};

/*****************************************************************************
**  Local Function prototypes
*****************************************************************************/
#if WICED_BT_OPS_DEBUG == TRUE
static char *ops_obx_evt_code(wiced_bt_obex_event_t evt_code);
#endif

const wiced_bt_ops_fs_cfg_t wiced_bt_ops_fs_cfg =
{
    wiced_bt_ops_fs_file_len,
    wiced_bt_ops_fs_path_len,
    wiced_bt_ops_fs_path_separator
};

wiced_bt_ops_fs_cfg_t *p_wiced_bt_ops_fs_cfg = (wiced_bt_ops_fs_cfg_t *)&wiced_bt_ops_fs_cfg;

char *utl_strrchr(char *s, int c);
/*****************************************************************************
**  Action Functions
*****************************************************************************/
/*******************************************************************************
**
** Function         wiced_bt_ops_enable
**
** Description      Perform necessary operations to enable object push.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_enable(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_api_enable_t *p_data)
{
    wiced_bt_obex_start_params_t    start_params;
    UINT16      servclass = UUID_SERVCLASS_OBEX_OBJECT_PUSH;
    int         i, j;
    wiced_bt_obex_status_t status;
    UINT8       desc_type[WICED_BT_OPS_NUM_FMTS];
    UINT8       type_len[WICED_BT_OPS_NUM_FMTS];
    UINT8       *type_value[WICED_BT_OPS_NUM_FMTS];
    UINT16      mtu = OBEX_MAX_MTU;
    UINT8       temp[4], *p;
    UINT16      version = GOEP_ENHANCED_VERSION;

    /* allocate scn for opp */
    p_cb->scn = p_data->scn;
    p_cb->app_id = p_data->app_id;
    p_cb->psm = L2CA_AllocatePSM();

    memset (&start_params, 0, sizeof(wiced_bt_obex_start_params_t));
    start_params.p_target = NULL;
    start_params.p_cback = &wiced_bt_ops_obx_cback;
    start_params.mtu = mtu;
    start_params.scn = p_cb->scn;
    start_params.psm = p_cb->psm;
    start_params.srm = p_cb->srm;
    start_params.nonce = 0;
    start_params.authenticate = FALSE;
    start_params.auth_option = 0x0;     /* OBX_AO_NONE */
    start_params.realm_charset = 0x0;   /* OBX_RCS_ASCII */
    start_params.p_realm = NULL;
    start_params.realm_len = 0;

    if ((status = wiced_bt_obex_start_server (&start_params, &p_cb->obx_handle)) == OBEX_SUCCESS)
    {
        /* store formats value */
        p_cb->formats = p_data->formats;
    }

    p_cb->p_cback(WICED_BT_OPS_ENABLE_EVT, NULL);
}

/*******************************************************************************
**
** Function         wiced_bt_ops_api_disable
**
** Description      Perform necessary operations to disable object push.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_api_disable(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    /* Free any outstanding headers and control block memory */
    wiced_bt_ops_clean_getput(p_cb, TRUE);

    /* Stop the OBEX server */
    wiced_bt_obex_stop_server(p_cb->obx_handle);

    if (p_cb->p_cback)
    {
        /* Notify the application */
        p_cb->p_cback(WICED_BT_OPS_DISABLE_EVT, 0);
        p_cb->p_cback = NULL;
    }

}

/*******************************************************************************
**
** Function         wiced_bt_ops_api_accessrsp
**
** Description      Process the access API event.
**                  If permission had been granted, continue the push or pull
**                  operation.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_api_accessrsp(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    UINT8 rsp_code;

    /* Process the currently active access response */
    switch (p_cb->acc_active)
    {
    case WICED_BT_OP_OPER_PUSH:
        if (p_data->api_access.oper == WICED_BT_OP_OPER_PUSH)
        {
            if (p_data->api_access.flag == WICED_BT_OP_ACCESS_ALLOW)
            {
                p_cb->cout_active = TRUE;
                wiced_bt_ops_co_open(WICED_BT_OPS_CI_OPEN_EVT, p_cb->app_id);
            }
            else    /* Access denied or Unsupported */
            {
                rsp_code = (p_data->api_access.flag == WICED_BT_OP_ACCESS_NONSUP)
                         ? OBEX_RSP_UNSUPTD_TYPE : OBEX_RSP_UNAUTHORIZED;
                wiced_bt_ops_clean_getput(p_cb, TRUE);
                wiced_bt_obex_send_response(p_cb->obx_handle, OBEX_REQ_PUT, rsp_code, NULL);
            }
            p_cb->acc_active = 0;
        }
        break;

    case WICED_BT_OP_OPER_PULL:
        if (p_data->api_access.oper == WICED_BT_OP_OPER_PULL)
        {
            if (p_data->api_access.flag == WICED_BT_OP_ACCESS_ALLOW)
            {
                p_cb->cout_active = TRUE;
                wiced_bt_ops_co_open(WICED_BT_OPS_CI_OPEN_EVT, p_cb->app_id);
            }
            else    /* Denied */
                wiced_bt_ops_get_obj_rsp(OBEX_RSP_UNAUTHORIZED, 0);

            p_cb->acc_active = 0;
        }
        break;

    default:
        WICED_BT_TRACE("OPS ACCRSP: Unknown wiced_bt_op_oper_t value (%d)\n",
                            p_cb->acc_active);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_ops_api_close
**
** Description      Handle an api close event.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_api_close(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    /* resources will be freed at WICED_BT_OPS_OBX_CLOSE_EVT */
    wiced_bt_obex_send_response(p_cb->obx_handle, OBEX_REQ_DISCONNECT, OBEX_RSP_SERVICE_UNAVL, NULL);
}

/*******************************************************************************
**
** Function         wiced_bt_ops_ci_write
**
** Description      Continue with the current write operation
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_ci_write(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    UINT8 rsp_code = OBEX_RSP_INTRNL_SRVR_ERR;

    p_cb->cout_active = FALSE;

    if (!p_cb->aborting)
    {
        /* Process write call-in event if operation is still active */
        if (p_cb->obx_oper == OPS_OP_PUSH_OBJ)
        {
            if (p_data->write_evt.status == WICED_BT_OPS_CO_OK)
                rsp_code = OBEX_RSP_OK;
            else
            {
                if (p_data->write_evt.status == WICED_BT_OPS_CO_ENOSPACE)
                    rsp_code = OBEX_RSP_DATABASE_FULL;
                wiced_bt_ops_clean_getput(p_cb, TRUE);
            }

            /* Process response to OBX client */
            wiced_bt_ops_put_obj_rsp(rsp_code);
        }
    }
    else    /* Finish aborting */
    {
        wiced_bt_ops_clean_getput(p_cb, TRUE);
        wiced_bt_obex_send_response(p_cb->obx_handle, OBEX_REQ_ABORT, OBEX_RSP_OK, NULL);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_ops_ci_read
**
** Description      Handles the response to a read call-out request.
**                  This is called within the OBX get file request.  If the
**                  operation has completed, the OBX response is sent out;
**                  otherwise a read for additional data is made.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_ci_read(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    wiced_bt_ops_ci_read_evt_t *p_revt = &p_data->read_evt;
    UINT8 rsp_code = OBEX_RSP_INTRNL_SRVR_ERR;

    p_cb->cout_active = FALSE;

    if (!p_cb->aborting)
    {
        /* Process read call-in event if operation is still active */
        if (p_cb->obx_oper == OPS_OP_PULL_OBJ && p_revt->fd == p_cb->fd)
        {
            /* Read was successful, not finished yet */
            if (p_revt->status == WICED_BT_OPS_CO_OK)
                rsp_code = OBEX_RSP_CONTINUE;

            /* Read was successful, end of file has been detected */
            else if (p_revt->status == WICED_BT_OPS_CO_EOF)
                rsp_code = OBEX_RSP_OK;

            /* Process response to OBX client */
            wiced_bt_ops_get_obj_rsp(rsp_code, p_revt->num_read);
        }
    }
    else    /* Finish aborting */
    {
        wiced_bt_ops_clean_getput(p_cb, TRUE);
        wiced_bt_obex_send_response(p_cb->obx_handle, OBEX_REQ_ABORT, OBEX_RSP_OK, NULL);
        WICED_BT_TRACE("OPS PUSH OBJ: Finished ABORTING!!!\n");
    }
}

/*******************************************************************************
**
** Function         wiced_bt_ops_ci_open
**
** Description      Continue with the current file open operation
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_ci_open(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    wiced_bt_ops_obx_pkt_t    *p_obx = &p_cb->obx;
    wiced_bt_ops_ci_open_evt_t *p_open = &p_data->open_evt;
    UINT8                rsp_code = OBEX_RSP_OK;
    UINT8                num_hdrs;
    BOOLEAN              endpkt;
    char                *p_name;
    uint8_t              ret;

    p_cb->cout_active = FALSE;

    if (p_cb->aborting)
    {
        wiced_bt_ops_clean_getput(p_cb, TRUE);
        wiced_bt_obex_send_response(p_cb->obx_handle, OBEX_REQ_ABORT, OBEX_RSP_OK, NULL);
        return;
    }

    /* Only process file get or put operations */
    if (p_cb->obx_oper == OPS_OP_PULL_OBJ)
    {
        /* if file is accessible read/write the first buffer of data */
        if (p_open->status == WICED_BT_OPS_CO_OK)
        {
            p_cb->fd = p_open->fd;
            p_cb->file_length = p_open->file_size;

            /* Add the name and length headers */
            p_name = (char*)utl_strrchr(p_cb->p_path, (int)p_wiced_bt_ops_fs_cfg->path_separator);
            if (p_name == NULL)
                p_name = p_cb->p_path;
            else
                p_name++;   /* increment past the file separator */

            ret = wiced_bt_obex_add_header_utf8((UINT8 *)p_obx->p_pkt, OBEX_HI_NAME, (UINT8 *) p_name);
            if(ret != OBEX_SUCCESS)
            {
               WICED_BT_TRACE("wiced_bt_obex_add_header_utf8 return %d", ret);
            }


            if (p_cb->file_length != WICED_BT_OPS_LEN_UNKNOWN)
            {
                wiced_bt_obex_add_header((UINT8 *)p_obx->p_pkt, OBEX_HI_LENGTH,(UINT8 *)&p_cb->file_length, sizeof(uint32_t));

                if (p_cb->file_length > 0)
                {
                    rsp_code = OBEX_RSP_CONTINUE;
                }
            }

            /* Send continuation response with the length of the file and no body */
            wiced_bt_obex_send_response(p_cb->obx_handle, OBEX_REQ_GET, rsp_code, (UINT8 *)p_obx->p_pkt);
            p_obx->p_pkt = NULL;    /* Do not deallocate buffer; OBX will */
        }
        else
        {
            if (p_open->status == WICED_BT_OPS_CO_EACCES)
                rsp_code = OBEX_RSP_UNAUTHORIZED;
            else    /* File could not be found */
                rsp_code = OBEX_RSP_NOT_FOUND;

            /* Send OBX response if an error occurred */
            wiced_bt_ops_get_obj_rsp(rsp_code, 0);
        }
    }
    else if (p_cb->obx_oper == OPS_OP_PUSH_OBJ)
    {
        /* if file is accessible read/write the first buffer of data */
        if (p_open->status == WICED_BT_OPS_CO_OK)
        {
            p_cb->fd = p_open->fd;

            /* Read in start of body if there is a body header */
            num_hdrs = wiced_bt_obex_find_body_header((UINT8 *)p_obx->p_pkt, &p_obx->p_start,
                                                      &p_obx->bytes_left, &endpkt);

            if (num_hdrs == 1)
            {
                rsp_code = OBEX_RSP_PART_CONTENT;   /* Do not send OBX response yet */

                /* Initiate the writing out of the data */
                p_cb->cout_active = TRUE;

                wiced_bt_ops_cb.p_data_cback(&p_obx->p_start[p_obx->offset], p_obx->bytes_left);

                wiced_bt_ops_co_write(p_cb->fd, &p_obx->p_start[p_obx->offset],
                                p_obx->bytes_left, WICED_BT_OPS_CI_WRITE_EVT,
                                0, p_cb->app_id);
            }
            else if (num_hdrs > 1)  /* Too many body headers to handle */
            {
                rsp_code = OBEX_RSP_BAD_REQUEST;
                wiced_bt_ops_clean_getput(p_cb, TRUE);
            }
            else    /* No body: respond with an OK so client can start sending the data */
                p_obx->bytes_left = 0;
        }
        else
        {
            if (p_open->status == WICED_BT_OPS_CO_EACCES)
                rsp_code = OBEX_RSP_UNAUTHORIZED;
            else if (p_open->status == WICED_BT_OPS_CO_ENOSPACE)
                rsp_code = OBEX_RSP_DATABASE_FULL;
            else
                rsp_code = OBEX_RSP_INTRNL_SRVR_ERR;
        }

        /* Send OBX response now if an error occurred or no body */
        if (rsp_code != OBEX_RSP_PART_CONTENT)
            wiced_bt_ops_put_obj_rsp(rsp_code);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_ops_obx_connect
**
** Description      Process the OBX connect event
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_obx_connect(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    wiced_bt_ops_obx_evt_t  *p_evt = &p_data->obx_evt;

    p_cb->peer_mtu = p_evt->param.conn.mtu;
    memcpy(p_cb->bd_addr, p_evt->param.conn.peer_addr, BD_ADDR_LEN);
    WICED_BT_TRACE("OPS Connect: peer mtu 0x%04x\n", p_cb->peer_mtu);

    /* done with obex packet */
    utl_freebuf((void**)&p_evt->p_pkt);

    wiced_bt_obex_send_response(p_evt->handle, OBEX_REQ_CONNECT, OBEX_RSP_OK, NULL);

    /* Notify the MMI that a connection has been opened */
    p_cb->p_cback(WICED_BT_OPS_OPEN_EVT, (wiced_bt_ops_t*)wiced_bt_ops_cb.bd_addr);
}

/*******************************************************************************
**
** Function         wiced_bt_ops_obx_disc
**
** Description      Process the OBX disconnect event
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_obx_disc(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    wiced_bt_ops_obx_evt_t  *p_evt = &p_data->obx_evt;
    UINT8                rsp_code;

    rsp_code = (p_evt->obx_event == OBEX_DISCONNECT_REQ_EVT) ? OBEX_RSP_OK
                                                            : OBEX_RSP_BAD_REQUEST;

    wiced_bt_obex_send_response(p_cb->obx_handle, OBEX_REQ_DISCONNECT, rsp_code, NULL);

    /* Done with Obex packet */
    utl_freebuf((void**)&p_evt->p_pkt);
}

/*******************************************************************************
**
** Function         wiced_bt_ops_obx_close
**
** Description      Process the OBX link lost event
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_obx_close(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    /* finished if not waiting on a call-in function */
    if (!p_cb->cout_active)
        wiced_bt_ops_sm_execute(p_cb, WICED_BT_OPS_CLOSE_CMPL_EVT, p_data);
}

/*******************************************************************************
**
** Function         wiced_bt_ops_obx_abort
**
** Description      Process the OBX abort event
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_obx_abort(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    wiced_bt_ops_obx_evt_t  *p_evt = &p_data->obx_evt;
    UINT8                rsp_code = OBEX_RSP_OK;

    utl_freebuf((void**)&p_evt->p_pkt);

    if (!p_cb->cout_active)
    {
        wiced_bt_ops_clean_getput(p_cb, TRUE);
        wiced_bt_obex_send_response(p_cb->obx_handle, OBEX_REQ_ABORT, rsp_code, NULL);
    }
    else    /* Delay the response if a call-out function is active */
        p_cb->aborting = TRUE;
}

/*******************************************************************************
**
** Function         wiced_bt_ops_obx_put
**
** Description      Process the OBX push object put event
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_obx_put(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    wiced_bt_ops_obx_evt_t  *p_evt = &p_data->obx_evt;
    UINT8    rsp_code = OBEX_RSP_CONTINUE;
    UINT16   read_size;

    p_cb->obx.final_pkt = p_evt->param.put.final;

    /* If currently processing a push, use the current name */
    if (wiced_bt_ops_cb.obx_oper == OPS_OP_PUSH_OBJ)
    {
        wiced_bt_ops_proc_put_obj(p_evt->p_pkt);
    }

    /* This is a new request; allocate enough memory to hold the path (including file name) */
    else if ((p_cb->p_path = (char *)GKI_getbuf((UINT16)(p_wiced_bt_ops_fs_cfg->max_path_len
                                        + p_wiced_bt_ops_fs_cfg->max_file_len + 2))) != NULL)
    {
        p_cb->p_name = (p_cb->p_path + p_wiced_bt_ops_fs_cfg->max_path_len + 1);
        p_cb->p_name[p_wiced_bt_ops_fs_cfg->max_file_len] = '\0';
        p_cb->p_path[p_wiced_bt_ops_fs_cfg->max_path_len] = '\0';

        /* read the name header if it exists and is valid */
        read_size = wiced_bt_ops_fs_file_len;
        if ( (wiced_bt_obex_read_header((UINT8 *)p_evt->p_pkt, OBEX_HI_NAME, (UINT8 *)p_cb->p_name, &read_size) == OBEX_SUCCESS) &&
            (utl_check_utf8(p_cb->p_name, read_size + 1 /*(UINT16)(p_wiced_bt_ops_fs_cfg->max_file_len + 1)*/)) )
        {
            /* get file type from file name; check if supported */
            if ((p_cb->obj_fmt = wiced_bt_ops_fmt_supported(p_cb->p_name,
                                                       p_cb->formats)) != 0)
            {
                read_size = sizeof(p_cb->file_length);
                if (wiced_bt_obex_read_header((UINT8 *)p_evt->p_pkt, OBEX_HI_LENGTH, (UINT8 *)&p_cb->file_length, &read_size) != OBEX_SUCCESS)
                    p_cb->file_length = WICED_BT_OPS_LEN_UNKNOWN;

                p_cb->obx.p_pkt = p_evt->p_pkt; /* save the packet for later use */
                p_cb->obx.offset = 0;  /* Initial offset into OBX data */
                p_cb->obx_oper = OPS_OP_PUSH_OBJ;

                /* request access from the app */
                wiced_bt_ops_req_app_access (WICED_BT_OP_OPER_PUSH, p_cb);
            }
            else
                rsp_code = OBEX_RSP_UNSUPTD_TYPE;
        }
        else
            rsp_code = OBEX_RSP_BAD_REQUEST;
    }
    else
        rsp_code = OBEX_RSP_INTRNL_SRVR_ERR;

    /* Error has been detected; respond with error code */
    if (rsp_code != OBEX_RSP_CONTINUE)
    {
        utl_freebuf((void**)&p_evt->p_pkt); /* done with obex packet */
        utl_freebuf((void**)&p_cb->p_path);
        p_cb->p_name = NULL;
        wiced_bt_obex_send_response(p_evt->handle, OBEX_REQ_PUT, rsp_code, NULL);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_ops_obx_get
**
** Description      Process the OBX pull vCard object.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_obx_get(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    /* this is a new request; validate it */
    if (wiced_bt_ops_cb.obx_oper != OPS_OP_PULL_OBJ)
        wiced_bt_ops_init_get_obj(p_cb, p_data);
    else    /* this is a continuation request */
        wiced_bt_ops_proc_get_obj(p_cb);

    /* done with Obex packet */
    utl_freebuf((void**)&p_data->obx_evt.p_pkt);
}

/*******************************************************************************
**
** Function         wiced_bt_ops_obx_action
**
** Description      Process the OBX action request.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_obx_action(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    /* Action operation is not supported in OPP, send reject rsp and free data */
    wiced_bt_obex_send_response(p_cb->obx_handle, OBEX_REQ_ACTION, OBEX_RSP_NOT_IMPLEMENTED, NULL);

    /* done with Obex packet */
    utl_freebuf((void**)&p_data->obx_evt.p_pkt);
}
/*******************************************************************************
**
** Function         wiced_bt_ops_close_complete
**
** Description      Finishes the memory cleanup after a channel is closed.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_close_complete(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    p_cb->cout_active = FALSE;

    wiced_bt_ops_clean_getput(p_cb, TRUE);

    /* Notify the MMI that a connection has been closed */
    p_cb->p_cback(WICED_BT_OPS_CLOSE_EVT, (wiced_bt_ops_t*)p_cb->bd_addr);
    memset(p_cb->bd_addr, 0, BD_ADDR_LEN);

    if (p_data->obx_evt.p_pkt)
        WICED_BT_TRACE("OPS: OBX CLOSE CALLED WITH non-NULL Packet!!!\n");
}

/*****************************************************************************
**  Callback Functions
*****************************************************************************/
/*******************************************************************************
**
** Function         wiced_bt_ops_obx_cback
**
** Description      OBX callback function.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_obx_cback (wiced_bt_obex_handle_t handle, wiced_bt_obex_event_t obx_event,
                        wiced_bt_obex_evt_param_t param, UINT8 *p_pkt)
{
    wiced_bt_ops_obx_evt_t *p_obx_msg;
    UINT16              event = 0;

#if WICED_BT_OPS_DEBUG == TRUE
    WICED_BT_TRACE("OBX Event Callback: ops_obx_event[%s]\n", ops_obx_evt_code(obx_event));
#endif

    switch(obx_event)
    {
    case OBEX_CONNECT_REQ_EVT:
        event = WICED_BT_OPS_OBX_CONN_EVT;
        break;
    case OBEX_DISCONNECT_REQ_EVT:
        event = WICED_BT_OPS_OBX_DISC_EVT;
        break;
    case OBEX_PUT_REQ_EVT:
        event = WICED_BT_OPS_OBX_PUT_EVT;
        break;
    case OBEX_GET_REQ_EVT:
        event = WICED_BT_OPS_OBX_GET_EVT;
        break;
    case OBEX_ABORT_REQ_EVT:
        event = WICED_BT_OPS_OBX_ABORT_EVT;
        break;
    case OBEX_CLOSE_IND_EVT:
        event = WICED_BT_OPS_OBX_CLOSE_EVT;
        break;
    case OBEX_TIMEOUT_EVT:
        break;
    case OBEX_ACTION_REQ_EVT:
        event = WICED_BT_OPS_OBX_ACTION_EVT;
        break;
    default:
        /* Unrecognized packet; disconnect the session */
        if (p_pkt)
            event = WICED_BT_OPS_OBX_DISC_EVT;
    }

    if (event && (p_obx_msg =
        (wiced_bt_ops_obx_evt_t *) GKI_getbuf(sizeof(wiced_bt_ops_obx_evt_t))) != NULL)
    {
        p_obx_msg->hdr.event = event;
        p_obx_msg->obx_event = obx_event;
        p_obx_msg->handle = handle;
        p_obx_msg->param = param;
        p_obx_msg->p_pkt = (BT_HDR *)p_pkt;

        wiced_bt_ops_hdl_event((BT_HDR *) p_obx_msg);

        GKI_freebuf(p_obx_msg);

    }
}

/*****************************************************************************
**  Debug Functions
*****************************************************************************/
#if WICED_BT_OPS_DEBUG == TRUE

/*******************************************************************************
**
** Function         ops_obx_evt_code
**
** Description
**
** Returns          void
**
*******************************************************************************/
static char *ops_obx_evt_code(wiced_bt_obex_event_t evt_code)
{
    switch(evt_code)
    {
    case OBEX_CONNECT_REQ_EVT:
        return "OBEX_CONNECT_REQ_EVT";
    case OBEX_DISCONNECT_REQ_EVT:
        return "OBEX_DISCONNECT_REQ_EVT";
    case OBEX_PUT_REQ_EVT:
        return "OBEX_PUT_REQ_EVT";
    case OBEX_GET_REQ_EVT:
        return "OBEX_GET_REQ_EVT";
    case OBEX_SETPATH_REQ_EVT:
        return "OBEX_SETPATH_REQ_EVT";
    case OBEX_ABORT_REQ_EVT:
        return "OBEX_ABORT_REQ_EVT";
    case OBEX_CLOSE_IND_EVT:
        return "OBEX_CLOSE_IND_EVT";
    case OBEX_TIMEOUT_EVT:
        return "OBEX_TIMEOUT_EVT";
    case OBEX_PASSWORD_EVT:
        return "OBEX_PASSWORD_EVT";
    case OBEX_SESSION_REQ_EVT:
        return "OBEX_SESSION_REQ_EVT";
    case OBEX_ACTION_REQ_EVT:
        return "OBEX_ACTION_REQ_EVT";
    default:
        return "unknown OBX event code";
    }
}
#endif  /* Debug Functions */
