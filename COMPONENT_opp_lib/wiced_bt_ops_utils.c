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
 * WICED BT OPP server utils
 *
 */

#include <stdio.h>
#include <string.h>
#include "wiced_bt_obex.h"
#include "wiced_bt_ops_int.h"
#include "wiced_bt_ops_co.h"
#include "wiced_bt_utils.h"

/*******************************************************************************
**  Constants
*******************************************************************************/

/*******************************************************************************
**  Local Function Prototypes
*******************************************************************************/
static int wiced_bt_ops_stricmp (const char *p_str1, const char *p_str2);

/*******************************************************************************
*  Exported Functions
*******************************************************************************/
uint16_t wiced_bt_obex_read_header_len(uint8_t *p_pkt, wiced_bt_obex_header_identifier_t id);
uint8_t * wiced_bt_obex_add_body_start(uint8_t *p_pkt, uint16_t *p_len);
void wiced_bt_obex_add_body_end(uint8_t *p_pkt, uint8_t *p_body, uint16_t len, wiced_bool_t end);
/*******************************************************************************
**
** Function         wiced_bt_ops_init_get_obj
**
** Description      Processes a begin object pull request.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_init_get_obj(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    wiced_bt_ops_obx_evt_t  *p_evt = &p_data->obx_evt;
    UINT8                rsp_code = OBEX_RSP_FORBIDDEN;
    UINT16               len;
    UINT8               *p_type;
    UINT16               name_len;

    /* Type Hdr must be present */
    if (wiced_bt_obex_find_byte_sequence_header((UINT8 *)p_evt->p_pkt, OBEX_HI_TYPE, &p_type, &len))
    {
        /* Type Hdr must be correct */
        if (!wiced_bt_ops_stricmp((const char *)p_type, "text/x-vcard"))
        {
            /* Erratum 385 - original OPP spec says "Name Header must not be used."
             *               errara says "Name Header must be empty"
             * Be a forgiving OPP server, allow either way */
            name_len = wiced_bt_obex_read_header_len((UINT8 *)p_evt->p_pkt, OBEX_HI_NAME);

            if((name_len == WICED_BT_OPS_INVALID_HDR_LEN) ||/* no name header */
               (name_len == 3) /* name header is empty */ )
            {
                p_cb->obx.p_pkt = (BT_HDR *)wiced_bt_obex_header_init(p_cb->obx_handle,
                                        /*p_cb->peer_mtu*/OBX_CMD_POOL_SIZE);
                p_cb->p_path = (char *)GKI_getbuf((UINT16)(p_wiced_bt_ops_fs_cfg->max_path_len + 1));

                if (p_cb->p_path && p_cb->obx.p_pkt)
                {
                    p_cb->p_path[0] = '\0';
                    p_cb->p_name = p_cb->p_path;
                    p_cb->obx_oper = OPS_OP_PULL_OBJ;
                    p_cb->obj_fmt = WICED_BT_OP_VCARD21_FMT;
                    p_cb->file_length = WICED_BT_OPS_LEN_UNKNOWN;

                    /* Notify the appl that a pull default card has been requested */
                    wiced_bt_ops_req_app_access (WICED_BT_OP_OPER_PULL, p_cb);
                    rsp_code = OBEX_RSP_CONTINUE;
                }
                else
                    rsp_code = OBEX_RSP_INTRNL_SRVR_ERR;
            }
        }
    }

    /* Error has been detected; respond with error code */
    if (rsp_code != OBEX_RSP_CONTINUE)
    {
        utl_freebuf((void**)&p_cb->obx.p_pkt);
        utl_freebuf((void**)&p_cb->p_path);
        p_cb->p_name = NULL;
        wiced_bt_obex_send_response(p_evt->handle, OBEX_REQ_GET, rsp_code, NULL);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_ops_proc_get_obj
**
** Description      Processes a continuation packet for pulling a vcard.
**                  Builds a response packet and initiates a read into it.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_proc_get_obj(wiced_bt_ops_cb_t *p_cb)
{
    wiced_bt_ops_obx_pkt_t *p_obx = &p_cb->obx;

    /* Allocate a new OBX packet */
    if ((p_obx->p_pkt = (BT_HDR *)wiced_bt_obex_header_init(p_cb->obx_handle,
                /*p_cb->peer_mtu*/OBEX_LRG_DATA_POOL_SIZE)) != NULL)
    {
        /* Add the start of the Body Header */
        p_obx->offset = 0;
        p_obx->bytes_left = 0;
        p_obx->p_start = (uint8_t *)wiced_bt_obex_add_body_start((UINT8 *)p_obx->p_pkt, &p_obx->bytes_left);

        p_cb->cout_active = TRUE;
        wiced_bt_ops_co_read(p_cb->fd, &p_obx->p_start[p_obx->offset],
            p_obx->bytes_left, WICED_BT_OPS_CI_READ_EVT, 0, p_cb->app_id);
    }
    else
        wiced_bt_ops_get_obj_rsp(OBEX_RSP_INTRNL_SRVR_ERR, 0);
}

/*******************************************************************************
**
** Function         wiced_bt_ops_proc_put_obj
**
** Description      Processes a Push Object Operation.
**                  Initiates the opening of an object for writing, or continues
**                  with a new Obx packet of data (continuation).
**
** Parameters       p_pkt - Pointer to the OBX Put request
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_proc_put_obj(BT_HDR *p_pkt)
{
    wiced_bt_ops_cb_t      *p_cb = &wiced_bt_ops_cb;
    wiced_bt_ops_obx_pkt_t *p_obx = &p_cb->obx;
    UINT8             num_hdrs;
    BOOLEAN           endpkt;

    p_obx->p_pkt = p_pkt;

    p_obx->offset = 0;  /* Initial offset into OBX data */
    /* Read in start of body if there is a body header */
    num_hdrs = wiced_bt_obex_find_body_header((UINT8 *)p_obx->p_pkt, &p_obx->p_start,
                               &p_obx->bytes_left, &endpkt);

    if (num_hdrs == 1)
    {
        p_cb->cout_active = TRUE;

        wiced_bt_ops_cb.p_data_cback(&p_obx->p_start[p_obx->offset], p_obx->bytes_left);

        wiced_bt_ops_co_write(p_cb->fd, &p_obx->p_start[p_obx->offset],
            p_obx->bytes_left, WICED_BT_OPS_CI_WRITE_EVT, 0, p_cb->app_id);
    }
    else
    {
        wiced_bt_ops_clean_getput(p_cb, TRUE);
        wiced_bt_ops_put_obj_rsp(OBEX_RSP_BAD_REQUEST);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_ops_get_obj_rsp
**
** Description      Finishes up the end body of the object get, and sends out the
**                  OBX response
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_get_obj_rsp(UINT8 rsp_code, UINT16 num_read)
{
    wiced_bt_ops_cb_t       *p_cb = &wiced_bt_ops_cb;
    wiced_bt_ops_obx_pkt_t  *p_obx = &p_cb->obx;
    wiced_bt_ops_progress_t  param;
    BOOLEAN            done = TRUE;

    /* Send the response packet if successful operation */
    if (rsp_code == OBEX_RSP_OK || rsp_code == OBEX_RSP_CONTINUE)
    {
        p_obx->offset += num_read;

        /* More to be sent */
        if (rsp_code == OBEX_RSP_CONTINUE)
        {
            if (p_obx->bytes_left != num_read)
                WICED_BT_TRACE("OPS Read: Requested (0x%04x), Read In (0x%04x)\n",
                                     p_obx->bytes_left, num_read);
            done = FALSE;
        }

        wiced_bt_obex_add_body_end((UINT8 *)p_obx->p_pkt, p_obx->p_start, p_obx->offset, done);

        /* Notify application with progress */
        if (num_read)
        {
            param.bytes = num_read;
            param.obj_size = p_cb->file_length;
            param.operation = WICED_BT_OP_OPER_PULL;
            p_cb->p_cback(WICED_BT_OPS_PROGRESS_EVT, (wiced_bt_ops_t *)&param);
        }
    }
    else
        p_cb->obx_oper = OPS_OP_NONE;

    wiced_bt_obex_send_response(p_cb->obx_handle, OBEX_REQ_GET, rsp_code, (UINT8 *)p_obx->p_pkt);
    p_obx->p_pkt = NULL;    /* Do not deallocate buffer; OBX will */

    /* Final response packet sent out */
    if (done)
        wiced_bt_ops_clean_getput(p_cb, FALSE);
}

/*******************************************************************************
**
** Function         wiced_bt_ops_put_obj_rsp
**
** Description      Responds to a put request, and closes the object if finished
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_put_obj_rsp(UINT8 rsp_code)
{
    wiced_bt_ops_cb_t       *p_cb = &wiced_bt_ops_cb;
    wiced_bt_ops_obx_pkt_t  *p_obx = &p_cb->obx;
    wiced_bt_ops_progress_t  param;
    wiced_bt_ops_object_t    object;

    /* Finished with input packet */
    utl_freebuf((void**)&p_obx->p_pkt);

    if (rsp_code == OBEX_RSP_OK)
    {
        /* Update application if object data was transferred */
        if (p_obx->bytes_left)
        {
            param.bytes = p_obx->bytes_left;
            param.obj_size = p_cb->file_length;
            param.operation = WICED_BT_OP_OPER_PUSH;
            p_cb->p_cback(WICED_BT_OPS_PROGRESS_EVT, (wiced_bt_ops_t *)&param);
        }

        /* If not end of object put, set the continue response */
        if (!p_obx->final_pkt)
            rsp_code = OBEX_RSP_CONTINUE;
        else    /* Done - free the allocated memory */
        {
            /* callback to app with object */
            object.format = p_cb->obj_fmt;
            object.p_name = p_cb->p_name;

            wiced_bt_ops_clean_getput(p_cb, FALSE);
            p_cb->p_cback(WICED_BT_OPS_OBJECT_EVT, (wiced_bt_ops_t *) &object);

        }
    }
    else
        p_cb->obx_oper = OPS_OP_NONE;

    wiced_bt_obex_send_response(p_cb->obx_handle, OBEX_REQ_PUT, rsp_code, NULL);
}

/*******************************************************************************
**
** Function         wiced_bt_ops_req_app_access
**
** Description      Sends an access request event to the application.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_req_app_access (wiced_bt_op_oper_t oper, wiced_bt_ops_cb_t *p_cb)
{
    wiced_bt_ops_access_t *p_acc_evt;
    char            *p_devname;
    UINT16           len;

    /* Notify the application that a put or get file has been requested */
    if ((p_acc_evt = (wiced_bt_ops_access_t *)GKI_getbuf(sizeof(wiced_bt_ops_access_t))) != NULL)
    {
        memset(p_acc_evt, 0, sizeof(wiced_bt_ops_access_t));
        p_acc_evt->p_name = p_cb->p_name;
        p_acc_evt->size = p_cb->file_length;
        p_acc_evt->oper = p_cb->acc_active = oper;
        p_acc_evt->format = p_cb->obj_fmt;
        utl_bdcpy(p_acc_evt->bd_addr, p_cb->bd_addr);

        /* Only pass the object type if Push operation */
        if (oper == WICED_BT_OP_OPER_PUSH)
        {
            if (!wiced_bt_obex_find_byte_sequence_header((UINT8 *)p_cb->obx.p_pkt, OBEX_HI_TYPE, (UINT8 **)&p_acc_evt->p_type, &len))
                p_acc_evt->p_type = NULL;
        }

        if (p_acc_evt->p_type)
        {
            WICED_BT_TRACE("OPS Access Request...Name [%s], Oper [%d], Type [%s]\n",
                p_cb->p_name, oper, p_acc_evt->p_type);
        }
        else
        {
            WICED_BT_TRACE("OPS Access Request...Name [%s], Oper [%d]\n",
                p_cb->p_name, oper);
        }

        p_cb->p_cback(WICED_BT_OPS_ACCESS_EVT, (wiced_bt_ops_t *)p_acc_evt);
        GKI_freebuf(p_acc_evt);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_ops_clean_getput
**
** Description      Cleans up the get/put resources and control block
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_clean_getput(wiced_bt_ops_cb_t *p_cb, BOOLEAN is_aborted)
{
    wiced_bt_ops_co_status_t status;

    /* Clean up control block */
    utl_freebuf((void**)&p_cb->obx.p_pkt);

    /* Close any open files */
    if (p_cb->fd >= 0)
    {
        wiced_bt_ops_co_close(p_cb->fd, p_cb->app_id);
        p_cb->fd = WICED_BT_OPS_INVALID_FD;

        /* Delete an aborted unfinished push file operation */
        if (is_aborted && p_cb->obx_oper == OPS_OP_PUSH_OBJ)
        {
            status = wiced_bt_ops_co_unlink(p_cb->p_path, p_cb->app_id);
            WICED_BT_TRACE("OPS: Remove ABORTED Push File Operation [%s], status 0x%02x\n", p_cb->p_path, status);
        }
    }

    utl_freebuf((void**)&p_cb->p_path);
    p_cb->p_name = NULL;

    p_cb->obx_oper = OPS_OP_NONE;
    p_cb->obx.bytes_left = 0;
    p_cb->file_length = WICED_BT_OPS_LEN_UNKNOWN;
    p_cb->acc_active = 0;
    p_cb->aborting = FALSE;
}

/*******************************************************************************
**
** Function         wiced_bt_ops_fmt_supported
**
** Description      This function determines the file type from a file name
**                  and checks if the file type is supported.
**
**
** Returns          Format type value or zero if format unsupported.
**
*******************************************************************************/
wiced_bt_op_fmt_t wiced_bt_ops_fmt_supported(char *p, wiced_bt_op_fmt_mask_t fmt_mask)
{
    char        *p_suffix;
    wiced_bt_op_fmt_t fmt = WICED_BT_OP_OTHER_FMT;

    /* scan to find file suffix */
    if ((p_suffix = (char*)utl_strrchr(p, '.')) != NULL)
    {
        p_suffix++;
        if (wiced_bt_ops_stricmp (p_suffix, "vcf") == 0)
        {
            fmt = WICED_BT_OP_VCARD21_FMT;
        }
        else if (wiced_bt_ops_stricmp (p_suffix, "vcd") == 0)
        {
            fmt = WICED_BT_OP_VCARD30_FMT;
        }
        else if (wiced_bt_ops_stricmp (p_suffix, "vcs") == 0)
        {
            fmt = WICED_BT_OP_VCAL_FMT;
        }
        else if (wiced_bt_ops_stricmp (p_suffix, "ics") == 0)
        {
            fmt = WICED_BT_OP_ICAL_FMT;
        }
        else if (wiced_bt_ops_stricmp (p_suffix, "vmg") == 0)
        {
            fmt = WICED_BT_OP_VMSG_FMT;
        }
        else if (wiced_bt_ops_stricmp (p_suffix, "vnt") == 0)
        {
            fmt = WICED_BT_OP_VNOTE_FMT;
        }
    }

    /* see if supported */
    if (fmt == WICED_BT_OP_OTHER_FMT)
    {
        if (!(fmt_mask & WICED_BT_OP_ANY_MASK))
            fmt = 0;    /* Other types not supported */
    }
    else
    {
        if (!((1 << (fmt - 1)) & fmt_mask))
            fmt = 0;
    }

    return fmt;
}

/*******************************************************************************
**
** Function         wiced_bt_ops_discard_data
**
** Description      frees the data
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_discard_data(UINT16 event, wiced_bt_ops_data_t *p_data)
{
    switch(event)
    {
    case WICED_BT_OPS_OBX_CONN_EVT:
    case WICED_BT_OPS_OBX_DISC_EVT:
    case WICED_BT_OPS_OBX_ABORT_EVT:
    case WICED_BT_OPS_OBX_CLOSE_EVT:
    case WICED_BT_OPS_OBX_PUT_EVT:
    case WICED_BT_OPS_OBX_GET_EVT:
    case WICED_BT_OPS_OBX_ACTION_EVT:
        utl_freebuf((void**)&p_data->obx_evt.p_pkt);
        break;

    default:
        /*Nothing to free*/
        break;
    }
}

/*******************************************************************************
*  Static Functions
*******************************************************************************/
/*******************************************************************************
**
** Function         wiced_bt_ops_stricmp
**
** Description      Used to compare type header
**
**
** Returns          void
**
*******************************************************************************/
static int wiced_bt_ops_stricmp (const char *p_str1, const char *p_str2)
{
    UINT16 i;
    UINT16 cmplen;

    if (!p_str1 || !p_str2)
        return (1);

    cmplen = strlen(p_str1);
    if (cmplen != strlen(p_str2))
        return (1);

    for (i = 0; i < cmplen; i++)
    {
        if (utl_toupper(p_str1[i]) != utl_toupper(p_str2[i]))
            return (i+1);
    }

    return 0;
}
