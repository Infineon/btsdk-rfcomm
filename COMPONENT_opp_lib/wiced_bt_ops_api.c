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
 * WICED BT OPP server APIs
 *
 */

#include <string.h>

#include "wiced_bt_dev.h"
#include "wiced_bt_ops_int.h"

/*****************************************************************************
**  Constants
*****************************************************************************/

/*******************************************************************************
**
** Function         wiced_bt_ops_op_enable
**
** Description      Enable the object push server.  This function must be
**                  called before any other functions in the OPS API are called.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_op_enable(UINT8 scn, wiced_bt_op_fmt_mask_t formats,
                   wiced_bt_ops_cback_t *p_cback, wiced_bt_ops_data_cback_t *p_data_cback)
{
    wiced_bt_ops_api_enable_t     *p_buf;

    wiced_bt_obex_init();

    if ((p_buf = (wiced_bt_ops_api_enable_t *) GKI_getbuf(sizeof(wiced_bt_ops_api_enable_t))) != NULL)
    {
        p_buf->hdr.event = WICED_BT_OPS_API_ENABLE_EVT;
        p_buf->p_cback = p_cback;
        p_buf->p_data_cback = p_data_cback;
        p_buf->app_id = 0;
        p_buf->formats = formats;
        p_buf->scn = scn;
        p_buf->srm = FALSE;

        wiced_bt_ops_hdl_event((BT_HDR *)p_buf);
        GKI_freebuf(p_buf);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_ops_op_disable
**
** Description      Disable the object push server.  If the server is currently
**                  connected to a peer device the connection will be closed.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_op_disable(void)
{
    BT_HDR  *p_buf;

    if ((p_buf = (BT_HDR *) GKI_getbuf(sizeof(BT_HDR))) != NULL)
    {
        p_buf->event = WICED_BT_OPS_API_DISABLE_EVT;
        wiced_bt_ops_hdl_event((BT_HDR *)p_buf);
        GKI_freebuf(p_buf);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_ops_op_close
**
** Description      Close the current connection.  This function is called if
**                  the phone wishes to close the connection before the object
**                  push is completed.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_op_close(void)
{
    BT_HDR  *p_buf;

    if ((p_buf = (BT_HDR *) GKI_getbuf(sizeof(BT_HDR))) != NULL)
    {
        p_buf->event = WICED_BT_OPS_API_CLOSE_EVT;
        wiced_bt_ops_hdl_event((BT_HDR *)p_buf);
        GKI_freebuf(p_buf);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_ops_op_access_rsp
**
** Description      Sends a reply to an access request event (WICED_BT_OPS_ACCESS_EVT).
**                  This call MUST be made whenever the event occurs.
**
** Parameters       oper    - operation being accessed.
**                  access  - WICED_BT_OP_ACCESS_ALLOW, WICED_BT_OP_ACCESS_FORBID, or
**                            WICED_BT_OP_ACCESS_NONSUP.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_op_access_rsp(wiced_bt_op_oper_t oper, wiced_bt_op_access_t access)
{
    wiced_bt_ops_api_access_rsp_t *p_buf;

    if ((p_buf = (wiced_bt_ops_api_access_rsp_t *)GKI_getbuf((UINT16)(sizeof(wiced_bt_ops_api_access_rsp_t)
                                       + p_wiced_bt_ops_fs_cfg->max_path_len + 1))) != NULL)
    {
        p_buf->flag = access;
        p_buf->oper = oper;
        p_buf->hdr.event = WICED_BT_OPS_API_ACCESSRSP_EVT;
        wiced_bt_ops_hdl_event((BT_HDR *)p_buf);
        GKI_freebuf(p_buf);
    }
}
