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
**  Name:           wiced_bt_pbc_api.c
**
**  Description:    This is the implementation of the API for the phone
**                  book access client subsystem of BTA, Widcomm's Bluetooth
**                  application layer for mobile phones.
**
**
*****************************************************************************/

#include <string.h>

#include "wiced_bt_dev.h"
#include "wiced_bt_pbc_int.h"


/*******************************************************************************
**
** Function         wiced_bt_pbc_op_enable
**
** Description      Enable the phone book access client.  This function must be
**                  called before any other functions in the PBC API are called.
**                  When the enable operation is complete the callback function
**                  will be called with an WICED_BT_PBC_ENABLE_EVT event.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_op_enable(wiced_bt_pbc_cback_t *p_cback, wiced_bt_pbc_data_cback_t *p_data_cback, UINT8 app_id, wiced_bt_pbc_sup_fea_mask_t local_features)
{
    wiced_bt_pbc_api_enable_t *p_buf;

    // Initialize OBEX
    wiced_bt_obex_init();

    if ((p_buf = (wiced_bt_pbc_api_enable_t *)GKI_getbuf(sizeof(wiced_bt_pbc_api_enable_t))) != NULL)
    {
        memset(p_buf, 0, sizeof(wiced_bt_pbc_api_enable_t));

        p_buf->hdr.event = WICED_BT_PBC_API_ENABLE_EVT;
        p_buf->p_cback = p_cback;
        p_buf->p_data_cback = p_data_cback;
        p_buf->app_id = app_id;

        p_buf->local_features = local_features;

        wiced_bt_pbc_hdl_event((BT_HDR*)p_buf);
        GKI_freebuf(p_buf);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_op_disable
**
** Description      Disable the phone book access client.  If the client is currently
**                  connected to a peer device the connection will be closed.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_op_disable(void)
{
    BT_HDR  *p_buf;

    if ((p_buf = (BT_HDR *)GKI_getbuf(sizeof(BT_HDR))) != NULL)
    {
        p_buf->event = WICED_BT_PBC_API_DISABLE_EVT;
        wiced_bt_pbc_hdl_event((BT_HDR *)p_buf);
        GKI_freebuf(p_buf);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_op_open
**
** Description      Open a connection to an PBAP server.
**
**                  When the connection is open the callback function
**                  will be called with a WICED_BT_PBC_OPEN_EVT.  If the connection
**                  fails or otherwise is closed the callback function will be
**                  called with a WICED_BT_PBC_CLOSE_EVT.
**
**                  Note: Pbc always enable (WICED_BT_SEC_AUTHENTICATE | WICED_BT_SEC_ENCRYPT)
**
**
**
** Returns          void
**
*******************************************************************************/

void wiced_bt_pbc_op_open(wiced_bt_device_address_t bd_addr, uint8_t sec_mask)
{
    wiced_bt_pbc_api_open_t *p_buf;

    if ((p_buf = (wiced_bt_pbc_api_open_t *)GKI_getbuf(sizeof(wiced_bt_pbc_api_open_t))) != NULL)
    {
        memset(p_buf, 0, sizeof(wiced_bt_pbc_api_open_t));

        p_buf->hdr.event = WICED_BT_PBC_API_OPEN_EVT;

        p_buf->sec_mask = (sec_mask | /*WICED_BT_SEC_AUTHENTICATE*/ BTM_SEC_IN_AUTHENTICATE | BTM_SEC_OUT_AUTHENTICATE | BTM_SEC_ENCRYPT);
        memcpy(p_buf->bd_addr, bd_addr, BD_ADDR_LEN);

        wiced_bt_pbc_hdl_event((BT_HDR *)p_buf);
        GKI_freebuf(p_buf);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_op_close
**
** Description      Close the current connection to the server.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_op_close(void)
{
    BT_HDR  *p_buf;

    if ((p_buf = (BT_HDR *)GKI_getbuf(sizeof(BT_HDR))) != NULL)
    {
        p_buf->event = WICED_BT_PBC_API_CLOSE_EVT;
        wiced_bt_pbc_hdl_event((BT_HDR *)p_buf);
        GKI_freebuf(p_buf);
    }
}


/*******************************************************************************
**
** Function         wiced_bt_pbc_op_getphonebook
**
** Description      Retrieve a PhoneBook from the peer device and copy it to the
**                  local file system.
**
**                  This function can only be used when the client is connected
**                  in PBAP mode.
**
** Note:            local file name is specified with a fully qualified path.
**                  Remote file name is absolute path in UTF-8 format.
**                  (telecom/pb.vcf or SIM1/telecom/pb.vcf).
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_op_getphonebook(char *p_local_name, char *p_remote_name,
                         wiced_bt_pbc_filter_mask_t filter, wiced_bt_pbc_format_t format,
                         UINT16 max_list_count, UINT16 list_start_offset,
                         BOOLEAN is_reset_miss_calls, wiced_bt_pbc_filter_mask_t selector,
                         UINT8 selector_op)
{
    wiced_bt_pbc_data_t      *p_msg;
    wiced_bt_pbc_api_get_t   *p_get;
    wiced_bt_pbc_get_param_t *p_getp;
    UINT16              remote_name_length = (p_remote_name) ? strlen(p_remote_name) : 0;

    if ((p_msg = (wiced_bt_pbc_data_t *)GKI_getbuf((UINT16)(sizeof(wiced_bt_pbc_data_t) +
        (p_wiced_bt_pbc_fs_cfg->max_path_len + 1) + remote_name_length + 1))) != NULL)
    {
        p_get = &p_msg->api_get;
        p_get->p_param = (wiced_bt_pbc_get_param_t *)(p_msg + 1);
        p_getp = p_get->p_param;
        p_get->p_rem_name = (char *)(p_getp + 1);
        p_get->p_name = (char *)(p_get->p_rem_name + remote_name_length + 1);
        p_get->obj_type = WICED_BT_PBC_GET_PB;

        /* copy the local name */
        if (p_local_name)
        {
            BCM_STRNCPY_S(p_get->p_name, p_wiced_bt_pbc_fs_cfg->max_path_len+1,
                          p_local_name, p_wiced_bt_pbc_fs_cfg->max_path_len);
            p_get->p_name[p_wiced_bt_pbc_fs_cfg->max_path_len] = '\0';
        }
        else
            p_get->p_name[0] = '\0';

        /* copy remote name */
        if( p_remote_name)
        {
            BCM_STRNCPY_S(p_get->p_rem_name, remote_name_length+1, p_remote_name, remote_name_length);
            p_get->p_rem_name[remote_name_length] = '\0';
        }
        else
            p_get->p_rem_name[0] = '\0';

        p_getp->filter = filter;
        p_getp->format = format;
        p_getp->list_start_offset = list_start_offset;
        p_getp->max_list_count = max_list_count;

        p_getp->is_reset_miss_calls = is_reset_miss_calls;
        p_getp->selector = selector;
        p_getp->selector_op = selector_op;

        p_get->hdr.event = WICED_BT_PBC_API_GETFILE_EVT;
        wiced_bt_pbc_hdl_event((BT_HDR*)p_msg);
        GKI_freebuf(p_msg);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_op_getcard
**
** Description      Retrieve a vCard from the peer device and copy it to the
**                  local file system.
**
**                  This function can only be used when the client is connected
**                  in PBAP mode.
**
** Note:            local file name is specified with a fully qualified path.
**                  Remote file name is relative path in UTF-8 format.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_op_getcard(char *p_local_name, char *p_remote_name,
                    wiced_bt_pbc_filter_mask_t filter, wiced_bt_pbc_format_t format)
{
    wiced_bt_pbc_data_t      *p_msg;
    wiced_bt_pbc_api_get_t   *p_get;
    wiced_bt_pbc_get_param_t *p_getp;
    UINT16            remote_name_length = (p_remote_name) ? strlen(p_remote_name) : 0;

    if ((p_msg = (wiced_bt_pbc_data_t *)GKI_getbuf((UINT16)(sizeof(wiced_bt_pbc_data_t) +
        (p_wiced_bt_pbc_fs_cfg->max_path_len + 1) + remote_name_length + 1))) != NULL)
    {
        p_get = &p_msg->api_get;
        p_get->p_param = (wiced_bt_pbc_get_param_t *)(p_msg + 1);
        p_getp = p_get->p_param;
        p_get->p_rem_name = (char *)(p_getp + 1);
        p_get->p_name = (char *)(p_get->p_rem_name + remote_name_length + 1);
        p_get->obj_type = WICED_BT_PBC_GET_CARD;

        /* copy the local name */
        if (p_local_name)
        {
            BCM_STRNCPY_S(p_get->p_name, p_wiced_bt_pbc_fs_cfg->max_path_len+1,
                          p_local_name, p_wiced_bt_pbc_fs_cfg->max_path_len);
            p_get->p_name[p_wiced_bt_pbc_fs_cfg->max_path_len] = '\0';
        }
        else
            p_get->p_name[0] = '\0';

        /* copy remote name */
        if( p_remote_name)
        {
            BCM_STRNCPY_S(p_get->p_rem_name, remote_name_length+1,
                          p_remote_name, remote_name_length);
            p_get->p_rem_name[remote_name_length] = '\0';
        }
        else
            p_get->p_rem_name[0] = '\0';

        p_getp->filter = filter;
        p_getp->format = format;

        p_get->hdr.event = WICED_BT_PBC_API_GETFILE_EVT;
        wiced_bt_pbc_hdl_event((BT_HDR *)p_msg);
        GKI_freebuf(p_msg);
    }
}


/*******************************************************************************
**
** Function         wiced_bt_pbc_op_chdir
**
** Description      Change PB path on the peer device.
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_op_chdir(char *p_dir, wiced_bt_pbc_flag_t flag)
{
    wiced_bt_pbc_data_t      *p_msg;
    wiced_bt_pbc_api_chdir_t *p_chdir;
    UINT16              dir_len;

    /* If directory is specified set the length */
    dir_len = (p_dir && *p_dir != '\0') ? (UINT16)(strlen(p_dir) + 1): 0;

    if ((p_msg = (wiced_bt_pbc_data_t *)GKI_getbuf((UINT16)(sizeof(wiced_bt_pbc_data_t) +
                                             dir_len))) != NULL)
    {
        p_chdir = &p_msg->api_chdir;
        p_chdir->flag = flag;
        if (dir_len)
        {
            p_chdir->p_dir = (char *)(p_msg + 1);
            BCM_STRNCPY_S(p_chdir->p_dir, dir_len, p_dir, dir_len-1);
            p_chdir->p_dir[dir_len-1] = '\0';
        }
        else    /* Setting to root directory OR backing up a directory */
            p_chdir->p_dir = NULL;

        p_chdir->hdr.event = WICED_BT_PBC_API_CHDIR_EVT;
        wiced_bt_pbc_hdl_event((BT_HDR *)p_msg);
        GKI_freebuf(p_msg);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_op_authrsp
**
** Description      Sends a response to an OBEX authentication challenge to the
**                  connected OBEX server. Called in response to an WICED_BT_PBC_AUTH_EVT
**                  event.
**
** Note:            If the "userid_required" is TRUE in the WICED_BT_PBC_AUTH_EVT event,
**                  then p_userid is required, otherwise it is optional.
**
**                  p_password  must be less than WICED_BT_PBC_MAX_AUTH_KEY_SIZE (16 bytes)
**                  p_userid    must be less than OBX_MAX_REALM_LEN (defined in target.h)
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_op_authrsp (char *p_password, char *p_userid)
{
    wiced_bt_pbc_api_auth_rsp_t *p_auth_rsp;
    WICED_BT_TRACE("wiced_bt_pbc_obx_password");

    if ((p_auth_rsp = (wiced_bt_pbc_api_auth_rsp_t *)GKI_getbuf(sizeof(wiced_bt_pbc_api_auth_rsp_t))) != NULL)
    {
        memset(p_auth_rsp, 0, sizeof(wiced_bt_pbc_api_auth_rsp_t));

        p_auth_rsp->hdr.event = WICED_BT_PBC_API_AUTHRSP_EVT;

        if (p_password)
        {
            p_auth_rsp->key_len = strlen(p_password);
            if (p_auth_rsp->key_len > WICED_BT_PBC_MAX_AUTH_KEY_SIZE)
                p_auth_rsp->key_len = WICED_BT_PBC_MAX_AUTH_KEY_SIZE;
            memcpy(p_auth_rsp->key, p_password, p_auth_rsp->key_len);
        }

        if (p_userid)
        {
            p_auth_rsp->userid_len = strlen(p_userid);
            if (p_auth_rsp->userid_len > OBX_MAX_REALM_LEN)
                p_auth_rsp->userid_len = OBX_MAX_REALM_LEN;
            memcpy(p_auth_rsp->userid, p_userid, p_auth_rsp->userid_len);
        }

        wiced_bt_pbc_hdl_event((BT_HDR *)p_auth_rsp);
        GKI_freebuf(p_auth_rsp);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_op_listcards
**
** Description      Retrieve a directory listing from the peer device.
**                  When the operation is complete the callback function will
**                  be called with one or more WICED_BT_PBC_LIST_EVT events
**                  containing directory list information formatted as described
**                  in the PBAP Spec, Version 0.9, section 3.1.6.
**                  This function can only be used when the client is connected
**                  to a peer device.
**
**
** Parameters       p_dir - Name of directory to retrieve listing of.
**
** Returns          void
**
*******************************************************************************/

void wiced_bt_pbc_op_listcards(char *p_dir, wiced_bt_pbc_order_t order, char *p_value,
                      wiced_bt_pbc_attr_t attribute, UINT16 max_list_count,
                      UINT16 list_start_offset, BOOLEAN is_reset_miss_calls,
                      wiced_bt_pbc_filter_mask_t selector, UINT8 selector_op)
{
    wiced_bt_pbc_data_t      *p_msg;
    wiced_bt_pbc_api_list_t  *p_list;
    wiced_bt_pbc_list_param_t *p_param;
    UINT16              dir_len = (p_dir == NULL) ? 0 : strlen(p_dir) ;
    UINT16              value_len = (p_value == NULL) ? 0 : strlen(p_value) ;

    if ((p_msg = (wiced_bt_pbc_data_t *)GKI_getbuf((UINT16)(sizeof(wiced_bt_pbc_data_t) +
                                             sizeof(wiced_bt_pbc_list_param_t) +
                                             dir_len + value_len + 2))) != NULL)
    {
        p_list = &p_msg->api_list;
        p_list->p_param       = (wiced_bt_pbc_list_param_t *)(p_msg + 1);
        p_param = p_list->p_param;
        p_list->p_dir = (char *)(p_param + 1);
        p_param->order               = order;
        p_param->attribute           = attribute;
        p_param->max_list_count      = max_list_count;
        p_param->list_start_offset   = list_start_offset;

        p_param->is_reset_miss_calls = is_reset_miss_calls;
        p_param->selector = selector;
        p_param->selector_op = selector_op;

        if (dir_len)
            BCM_STRNCPY_S(p_list->p_dir, dir_len+1, p_dir, dir_len);
        p_list->p_dir[dir_len] = 0;

        if (value_len)
        {
            p_param->p_value = (char *)(p_list->p_dir + dir_len + 1);
            BCM_STRNCPY_S(p_param->p_value, value_len+1, p_value, value_len);
            p_param->p_value[value_len] = 0;
        }
        else
            p_param->p_value = NULL;

        p_list->hdr.event = WICED_BT_PBC_API_LISTDIR_EVT;
        wiced_bt_pbc_hdl_event((BT_HDR *)p_msg);
        GKI_freebuf(p_msg);
    }
}



/*******************************************************************************
**
** Function         wiced_bt_pbc_op_abort
**
** Description      Aborts any active Get Card operation.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_pbc_op_abort(void)
{
    BT_HDR  *p_buf;

    if ((p_buf = (BT_HDR *)GKI_getbuf(sizeof(BT_HDR))) != NULL)
    {
        p_buf->event = WICED_BT_PBC_API_ABORT_EVT;
        wiced_bt_pbc_hdl_event((BT_HDR *)p_buf);
        GKI_freebuf(p_buf);
    }
}
