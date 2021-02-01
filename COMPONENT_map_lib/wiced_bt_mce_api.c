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
**  Name:           wiced_bt_mce_api.c
**
**  Description:    This is the implementation of the API for the Message Access
**                  client subsystem.
**
*****************************************************************************/

#include <string.h>
#include "wiced_bt_mce_int.h"

/*****************************************************************************
**  Constants
*****************************************************************************/

/*******************************************************************************
**
** Function         wiced_bt_mce_enable
**
** Description      Enable the MCE subsystem.  This function must be
**                  called before any other functions in the MCE API are called.
**                  When the enable operation is complete the callback function
**                  will be called with an BTA_MCE_ENABLE_EVT event.
**
** Parameter        p_cback: call function registered to receive call back events.
**                  app_id: application ID.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_enable(wiced_bt_mce_cback_t *p_cback, UINT8 app_id)
{
    wiced_mce_api_enable_t *p_msg;

    // Initialize OBEX
    wiced_bt_obex_init();

    if ((p_msg = (wiced_mce_api_enable_t *)GKI_getbuf(sizeof(wiced_mce_api_enable_t))) != NULL)
    {
        APPL_TRACE_API0("wiced_bt_mce_enable\n");
        p_msg->hdr.event = WICED_MCE_API_ENABLE_EVT,
        p_msg->app_id = app_id;
        p_msg->p_cback = p_cback;

        wiced_mce_send_event((BT_HDR *)p_msg);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_mce_disable
**
** Description      Disable the MCE subsystem.  If the client is currently
**                  connected to a peer device the connection will be closed.
**
** Parameter        None
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_disable(void)
{
    BT_HDR *p_msg;

    if ((p_msg = (BT_HDR *)GKI_getbuf(sizeof(BT_HDR))) != NULL)
    {
        APPL_TRACE_API0("wiced_bt_mce_disable\n");
        p_msg->event = WICED_MCE_API_DISABLE_EVT,

        wiced_mce_send_event((BT_HDR *)p_msg);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_mce_mn_start
**
** Description      Start the Message Notification service server.
**                  When the Start operation is complete the callback function
**                  will be called with an BTA_MCE_START_EVT event.
**                  Note: MCE always enables (BTA_SEC_AUTHENTICATE | BTA_SEC_ENCRYPT)
**
**  Parameters      sec_mask - The security setting for the message access server.
**                  p_service_name - The name of the Message Notification service, in SDP.
**                                   Maximum length is 35 bytes.
**                  features - Local supported features
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_mn_start(UINT8 sec_mask, const char *p_service_name, wiced_bt_ma_supported_features_t features)
{
    wiced_mce_api_mn_start_t *p_msg;

    if ((p_msg = (wiced_mce_api_mn_start_t *)GKI_getbuf(sizeof(wiced_mce_api_mn_start_t))) != NULL)
    {
        APPL_TRACE_API0("wiced_bt_mce_mn_start\n");
        memset (p_msg, 0, sizeof(wiced_mce_api_mn_start_t));

        p_msg->hdr.event = WICED_MCE_API_START_EVT;
        p_msg->sec_mask = sec_mask;
        p_msg->servicename[0] = '\0';
        if (p_service_name)
            BCM_STRNCPY_S(p_msg->servicename, sizeof(p_msg->servicename), p_service_name, BD_NAME_LEN);

#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
        p_msg->mce_local_features = features;
#endif

        wiced_mce_send_event((BT_HDR *)p_msg);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_mce_mn_stop
**
** Description      Stop the Message Access service server.  If the server is currently
**                  connected to a peer device the connection will be closed.
**
** Parameter        None
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_mn_stop(void)
{
    BT_HDR *p_msg;

    if ((p_msg = (BT_HDR *)GKI_getbuf(sizeof(BT_HDR))) != NULL)
    {
        APPL_TRACE_API0("wiced_bt_mce_mn_stop\n");
        p_msg->event = WICED_MCE_API_STOP_EVT;

        wiced_mce_send_event((BT_HDR *)p_msg);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_mce_discover
**
** Description      Start service discover of MAP on the peer device
**
**                  When SDP is finished, the callback function will be called
**                  with BTA_MCE_DISCOVER_EVT with status.
**
** Parameter        bd_addr: MAS server bd address.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_discover(wiced_bt_device_address_t bd_addr)
{
    wiced_mce_api_discover_t *p_msg;

    if ((p_msg = (wiced_mce_api_discover_t *)GKI_getbuf(sizeof(wiced_mce_api_discover_t))) != NULL)
    {
        APPL_TRACE_API1("wiced_bt_mce_discover %B\n", bd_addr);

        memset(p_msg, 0, sizeof(wiced_mce_api_discover_t));
        p_msg->hdr.event = WICED_MCE_API_DISCOVER_EVT;

        memcpy(p_msg->bd_addr, bd_addr, BD_ADDR_LEN);

        wiced_mce_send_event((BT_HDR *)p_msg);
    }

}

/*******************************************************************************
**
** Function         wiced_bt_mce_open
**
** Description      Open a connection to an Message Access service server
**                  based on specified mas_instance_id
**
**                  When the connection is open the callback function
**                  will be called with a BTA_MCE_MA_OPEN_EVT.  If the connection
**                  fails or otherwise is closed the callback function will be
**                  called with a BTA_MCE_MA_CLOSE_EVT.
**
**                  Note: MCE always enables (BTA_SEC_AUTHENTICATE | BTA_SEC_ENCRYPT)
**
** Parameter        bd_addr: MAS server bd address.
**                  mas_instance_id - MAS instance ID on server device.
**
**                  sec_mask: security mask used for this connection.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_open(wiced_bt_device_address_t bd_addr, wiced_bt_ma_inst_id_t mas_instance_id, UINT8 sec_mask)
{
    wiced_mce_api_open_t *p_msg;

    if ((p_msg = (wiced_mce_api_open_t *)GKI_getbuf(sizeof(wiced_mce_api_open_t))) != NULL)
    {
        APPL_TRACE_API2("wiced_bt_mce_open %B inst_id=%d\n", bd_addr, mas_instance_id);

        memset(p_msg, 0, sizeof(wiced_mce_api_open_t));
        p_msg->hdr.event = WICED_MCE_API_OPEN_EVT;
        p_msg->mas_inst_id = mas_instance_id;
        p_msg->sec_mask = sec_mask;

        memcpy(p_msg->bd_addr, bd_addr, BD_ADDR_LEN);
        wiced_mce_send_event((BT_HDR *)p_msg);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_mce_close
**
** Description      Close the specified MAS session to the server.
**
** Parameter        session_id - MAS session ID
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_close(wiced_bt_ma_sess_handle_t session_id)
{
    BT_HDR  *p_buf;

    if ((p_buf = (BT_HDR *)GKI_getbuf(sizeof(BT_HDR))) != NULL)
    {
        APPL_TRACE_API1("wiced_bt_mce_close sess_id=%d\n",session_id);
        p_buf->event = WICED_MCE_API_CLOSE_EVT;
        p_buf->layer_specific = session_id;

        wiced_mce_send_event((BT_HDR *)p_buf);
    }

}

/*******************************************************************************
**
** Function         wiced_bt_mce_mn_close
**
** Description      Close a MNS session to the client.
**
** Parameters       session_id - MNS session ID
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_mn_close(wiced_bt_ma_sess_handle_t session_id)
{
    wiced_mce_api_mn_close_t *p_msg;

    if ((p_msg = (wiced_mce_api_mn_close_t *)GKI_getbuf(sizeof(wiced_mce_api_mn_close_t))) != NULL)
    {
        memset(p_msg, 0, sizeof(wiced_mce_api_mn_close_t));
        p_msg->hdr.event = WICED_MCE_API_MN_CLOSE_EVT;
        p_msg->session_id = session_id;

        wiced_mce_send_event((BT_HDR *)p_msg);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_mce_close_all
**
** Description      Close all MAS session to the server.
**
** Parameter        None
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_close_all(void)
{
    BT_HDR  *p_buf;

    if ((p_buf = (BT_HDR *)GKI_getbuf(sizeof(BT_HDR))) != NULL)
    {
        APPL_TRACE_API0("wiced_bt_mce_close_all\n");
        p_buf->event = WICED_MCE_API_CLOSE_ALL_EVT;

        wiced_mce_send_event((BT_HDR *)p_buf);
    }

}

/*******************************************************************************
**
** Function         wiced_bt_mce_notif_reg
**
** Description      Set the Message Notification status to On or OFF on the MSE.
**                  When notification is registered, message notification service
**                  must be enabled by calling API wiced_bt_mce_MnStart().
**
** Parameter        status - BTA_MA_NOTIF_ON if notification required
**                           BTA_MA_NOTIF_OFF if no notification
**                  session_id - MAS session ID
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_notif_reg(wiced_bt_ma_sess_handle_t session_id, wiced_bt_ma_notif_status_t status)
{
    wiced_mce_api_notif_reg_t  *p_buf;

    if ((p_buf = (wiced_mce_api_notif_reg_t *)GKI_getbuf(sizeof(wiced_mce_api_notif_reg_t))) != NULL)
    {
        APPL_TRACE_API1("wiced_bt_mce_notif_reg sess_id=%d\n",session_id);
        p_buf->hdr.event = WICED_MCE_API_NOTIF_REG_EVT;
        p_buf->hdr.layer_specific = session_id;
        p_buf->status = status;

        wiced_mce_send_event((BT_HDR *)p_buf);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_mce_update_inbox
**
** Description      This function is used to update the inbox for the
**                  specified MAS session.
**
** Parameter        session_id - MAS session ID
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_update_inbox(wiced_bt_ma_sess_handle_t session_id)
{
    BT_HDR  *p_buf;

    if ((p_buf = (BT_HDR *)GKI_getbuf(sizeof(BT_HDR))) != NULL)
    {
        APPL_TRACE_API1("wiced_bt_mce_update_inbox sess_id=%d\n",session_id);
        p_buf->event = WICED_MCE_API_UPD_INBOX_EVT;
        p_buf->layer_specific = session_id;

        wiced_mce_send_event((BT_HDR *)p_buf);
    }

}

/*******************************************************************************
**
** Function         wiced_bt_mce_set_folder
**
** Description      This function is used to navigate the folders of the MSE for
**                  the specified MAS instance
**
** Parameter        a combination of flag and p_folder specify how to nagivate the
**                  folders on the MSE
**                  case 1 flag = 2 folder = empty - reset to the default directory "telecom"
**                  case 2 flag = 2 folder = name of child folder - go down 1 level into
**                  this directory name
**                  case 3 flag = 3 folder = name of child folder - go up 1 level into
**                  this directory name (same as cd ../name)
**                  case 4 flag = 3 folder = empty - go up 1 level to the parent directory
**                  (same as cd ..)
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_set_folder(wiced_bt_ma_sess_handle_t session_id, wiced_bt_ma_dir_nav_t flag, char *p_folder)
{
    wiced_mce_data_t          *p_msg;
    wiced_mce_api_setfolder_t *p_setfolder;
    UINT16                    dir_len;

    /* If directory is specified set the length */
    dir_len = (p_folder && *p_folder != '\0') ? (UINT16)(strlen(p_folder) + 1): 0;

    if ((p_msg = (wiced_mce_data_t *)GKI_getbuf((UINT16)(sizeof(wiced_mce_data_t) + dir_len))) != NULL)
    {
        APPL_TRACE_API2("wiced_bt_mce_set_folder flag=%d folder=%s\n", flag, p_folder);
        p_setfolder = &p_msg->api_setfolder;

        p_setfolder->hdr.event = WICED_MCE_API_CHDIR_EVT;
        p_setfolder->hdr.layer_specific = session_id;

        p_setfolder->flag = flag;
        if (dir_len)
        {
            p_setfolder->p_dir = (char *)(p_msg + 1);
            BCM_STRNCPY_S(p_setfolder->p_dir, dir_len+1, p_folder, dir_len);
        }
        else    /* Setting to root directory OR backing up a directory */
            p_setfolder->p_dir = NULL;

        wiced_mce_send_event((BT_HDR *)p_msg);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_mce_get_folder_list
**
** Description      This function is used to retrieve the folder list object from
**                  the current folder
**
** Parameter        session_id - MAS session ID
**                  max_list_count - maximum number of folder-list objects allowed
**                            The maximum allowed value for this filed is 1024
**                  start_offset - offset of the from the first entry of the folder-list
**                                   object
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_get_folder_list(wiced_bt_ma_sess_handle_t session_id,
                                  UINT16 max_list_count, UINT16 start_offset)
{
    wiced_mce_api_list_t  *p_msg;

    if ((p_msg = (wiced_mce_api_list_t *)GKI_getbuf((UINT16)sizeof(wiced_mce_api_list_t))) != NULL)
    {
        APPL_TRACE_API1("wiced_bt_mce_get_folder_list sess_id=%d\n", session_id);
        memset(p_msg, 0, sizeof(wiced_mce_api_list_t));

        p_msg->hdr.event = WICED_MCE_API_LIST_EVT;
        p_msg->hdr.layer_specific = session_id;

        p_msg->list_type = WICED_MCE_LIST_TYPE_DIR;
        p_msg->max_list_count = max_list_count;
        p_msg->list_start_offset = start_offset;

        wiced_mce_send_event((BT_HDR *)p_msg);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_mce_get_msg_list
**
** Description      This function is used to retrieve the folder list object from
**                  the current folder of the MSE
**
** Parameter        session_id -  session handle
**                  p_folder       - folder name
**                  p_filter_param - Message listing option parameters
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_get_msg_list(wiced_bt_ma_sess_handle_t session_id,
                               const char *p_folder,
                               wiced_bt_ma_msg_list_filter_param_t *p_filter_param)
{
    wiced_mce_data_t     *p_msg;
    wiced_mce_api_list_t *p_list;

    UINT16  dir_len = (p_folder == NULL) ? 0 : strlen(p_folder) + 1;
    UINT16  len = sizeof(wiced_mce_data_t) + dir_len + sizeof(wiced_bt_ma_msg_list_filter_param_t);


    if ((p_msg = (wiced_mce_data_t *)GKI_getbuf(len)) != NULL)
    {
        APPL_TRACE_API1("wiced_bt_mce_get_msg_list sess_id=%d\n",session_id);
        memset(p_msg, 0, len);

        p_list = &p_msg->api_list;
        p_list->p_folder = (char *)(p_msg + 1);

        if (dir_len)
            BCM_STRNCPY_S(p_list->p_folder, dir_len+1, p_folder, dir_len);

        p_list->p_folder[dir_len] = 0;

        p_list->hdr.event = WICED_MCE_API_LIST_EVT;
        p_list->hdr.layer_specific = session_id;

        p_list->list_type = WICED_MCE_LIST_TYPE_MSG;

        /* set default value of max list count and starting offset */
        p_list->max_list_count = 0xffff;
        p_list->list_start_offset =0;

        if (p_filter_param != NULL)
        {
            p_list->max_list_count = p_filter_param->max_list_cnt;
            p_list->list_start_offset = p_filter_param->list_start_offset;

            p_list->p_param = (wiced_bt_ma_msg_list_filter_param_t *)(p_list->p_folder + dir_len + 1);
            memcpy(p_list->p_param, p_filter_param, sizeof(wiced_bt_ma_msg_list_filter_param_t));
        }
        else
            p_list->p_param = NULL;

        wiced_mce_send_event((BT_HDR *)p_msg);
    }

}

/*******************************************************************************
**
** Function         wiced_bt_mce_get_msg
**
** Description      This function is used to get bMessage or bBody of the
**                  specified message handle from MSE
**
** Parameter        session_id - session ID
**                  p_param - get message parameters, it shall not be NULL.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_get_msg(wiced_bt_ma_sess_handle_t session_id, wiced_bt_ma_get_msg_param_t *p_param)
{
    wiced_mce_api_get_msg_t *p_msg;

    if ((p_msg = (wiced_mce_api_get_msg_t *)GKI_getbuf(sizeof(wiced_mce_api_get_msg_t))) != NULL)
    {
        APPL_TRACE_API0("wiced_bt_mce_get_msg\n");
        memset(p_msg, 0, sizeof(wiced_mce_api_get_msg_t));

        p_msg->hdr.event = WICED_MCE_API_GET_MSG_EVT;
        p_msg->hdr.layer_specific = session_id;

        if (p_param != NULL)
        {
            memcpy(&p_msg->param, p_param, sizeof(wiced_bt_ma_get_msg_param_t));
        }

        wiced_mce_send_event((BT_HDR *)p_msg);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_mce_get_mas_instance_info
**
** Description      This function enables the MCE to get the MAS instance information
**                  from the MSE
**
** Parameters       session_id - session ID
**                  mas_instance_id - MAS Instance ID
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_get_mas_instance_info(wiced_bt_ma_sess_handle_t session_id, wiced_bt_ma_inst_id_t mas_instance_id)
{
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
    wiced_mce_api_get_mas_ins_info_t   *p_msg;

    if ((p_msg = (wiced_mce_api_get_mas_ins_info_t *)GKI_getbuf(sizeof(wiced_mce_api_get_mas_ins_info_t))) != NULL)
    {
        APPL_TRACE_API0("wiced_bt_mce_get_mas_instance_info\n");
        memset(p_msg, 0, sizeof(wiced_mce_api_get_mas_ins_info_t));

        p_msg->hdr.event = WICED_MCE_API_GET_MAS_INS_INFO_EVT;
        p_msg->hdr.layer_specific = session_id;

        p_msg->mas_instance_id = mas_instance_id;

        wiced_mce_send_event((BT_HDR *)p_msg);
    }
#endif
}

/*******************************************************************************
**
** Function         wiced_bt_mce_set_msg_status
**
** Description      This function is used to set the message status of the
**                  specified message handle
**
** Parameter        session_id - MAS session ID
**                  status_indicator : read/delete message
**                  status_value : on/off
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_set_msg_status(wiced_bt_ma_sess_handle_t session_id,
                                 wiced_bt_ma_msg_handle_t msg_handle,
                                 wiced_bt_ma_sts_indctr_t status_indicator,
                                 wiced_bt_ma_sts_value_t status_value)
{
    wiced_mce_api_set_sts_t *p_msg;

    if ((p_msg = (wiced_mce_api_set_sts_t *)GKI_getbuf(sizeof(wiced_mce_api_set_sts_t))) != NULL)
    {
        APPL_TRACE_API1("wiced_bt_mce_set_msg_status sess_id=%d\n",session_id);
        memset(p_msg, 0, sizeof(wiced_mce_api_set_sts_t) );

        p_msg->hdr.event    = WICED_MCE_API_SET_STS_EVT;
        p_msg->hdr.layer_specific = session_id;

        p_msg->indicator    = status_indicator;
        p_msg->value        = status_value;
        memcpy(p_msg->msg_handle, msg_handle, sizeof(wiced_bt_ma_msg_handle_t));

        wiced_mce_send_event((BT_HDR *)p_msg);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_mce_push_msg
**
** Description      This function is used to upload a message
**                  to the specified folder in MSE
**
** Parameter        session_id - MAS session ID
**                  p_param - push message parameters, it shall not be NULL.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_mce_push_msg(wiced_bt_ma_sess_handle_t session_id, wiced_bt_ma_push_msg_param_t *p_param)
{
    wiced_mce_api_push_msg_t   *p_msg;

    UINT16  folder_len = (p_param && p_param->p_folder) ? strlen(p_param->p_folder) + 1 : 0;
    UINT16  len = sizeof(wiced_mce_api_push_msg_t) + folder_len;

    if ((p_msg = (wiced_mce_api_push_msg_t *)GKI_getbuf(len)) != NULL)
    {
        APPL_TRACE_API1("wiced_bt_mce_push_msg sess_id=%d\n",session_id);
        memset(p_msg, 0, len);

        p_msg->hdr.event = WICED_MCE_API_PUSH_EVT;
        p_msg->hdr.layer_specific = session_id;

        if (p_param)
        {
            memcpy(&p_msg->param, p_param, sizeof(wiced_bt_ma_push_msg_param_t));

            if (p_param->p_folder)
            {
                p_msg->param.p_folder = (char *) (((char *) p_msg) + sizeof(wiced_mce_api_push_msg_t));
                memcpy(p_msg->param.p_folder, p_param->p_folder, strlen(p_param->p_folder));
            }
            else
                p_msg->param.p_folder = NULL;

        }
        wiced_mce_send_event((BT_HDR *)p_msg);
    }
}

/*******************************************************************************
**
** Function         wiced_bt_mce_abort
**
** Description      This function is used to abort the current OBEX multi-packt
**                  operation
**
** Parameter        bd_addr: MAS server bd address.
**                  mas_instance_id - MAS instance ID on server device.
**
** Returns          void
**
** Note: bd_addr is for future MCE enhancement which MCE can connect to one
**       or more MSEs
*******************************************************************************/
void wiced_bt_mce_abort(wiced_bt_device_address_t bd_addr, wiced_bt_ma_inst_id_t mas_instance_id)
{
    wiced_mce_api_abort_t *p_msg;

    if ((p_msg = (wiced_mce_api_abort_t *)GKI_getbuf(sizeof(wiced_mce_api_abort_t))) != NULL)
    {
        APPL_TRACE_API1("wiced_bt_mce_abort inst_id=%d\n",mas_instance_id);

        p_msg->hdr.event = WICED_MCE_API_ABORT_EVT;
        p_msg->mas_inst_id = mas_instance_id;
        memcpy(p_msg->bd_addr, bd_addr, BD_ADDR_LEN);

        wiced_mce_send_event((BT_HDR *)p_msg);
    }
}
