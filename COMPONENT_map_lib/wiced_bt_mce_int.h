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
**  Name:           wiced_bt_mce_int.h
**
**  Description:    This is the private file for the message access
**                  client equipment(MCE).
**
*****************************************************************************/
#ifndef WICED_BT_MCE_INT_H
#define WICED_BT_MCE_INT_H

#include "bt_types.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_obex.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include "wiced_bt_ma_def.h"
#include "wiced_bt_mce_api.h"
#include "wiced_bt_utils.h"

/*****************************************************************************
**  Constants and data types
*****************************************************************************/
#define WICED_MAS_MESSAGE_ACCESS_TARGET_UUID       "\xBB\x58\x2B\x40\x42\x0C\x11\xDB\xB0\xDE\x08\x00\x20\x0C\x9A\x66"
#define WICED_MAS_MESSAGE_NOTIFICATION_TARGET_UUID "\xBB\x58\x2B\x41\x42\x0C\x11\xDB\xB0\xDE\x08\x00\x20\x0C\x9A\x66"
#define WICED_MAS_UUID_LENGTH                     16

#undef  OBX_CMD_POOL_SIZE
#define OBX_CMD_POOL_SIZE           256

#undef  OBEX_LRG_DATA_POOL_SIZE
#define OBEX_LRG_DATA_POOL_SIZE     1024
#ifndef OBX_LRG_DATA_POOL_SIZE
#define OBX_LRG_DATA_POOL_SIZE      1024
#endif

typedef wiced_bt_obex_status_t (tBTA_MA_OBX_RSP) (wiced_bt_obex_handle_t handle, UINT8 rsp_code, BT_HDR *p_pkt);

/* sdp discovery database size */
#define WICED_MCE_DISC_SIZE       1600 /* 400*4 */

/* Message Access state machine states */
enum
{
    WICED_MCE_MA_IDLE_ST = 0,      /* Idle  */
    WICED_MCE_MA_W4_CONN_ST,       /* Waiting for an Obex connect response */
    WICED_MCE_MA_CONN_ST,          /* Connected - MAS Client Session is active */
    WICED_MCE_MA_CLOSING_ST        /* Closing is in progress */
};

/* Message Notification state machine states */
enum
{
    WICED_MCE_MN_IDLE_ST = 0,      /* Idle  */
    WICED_MCE_MN_LISTEN_ST,        /* wait for connection state */
    WICED_MCE_MN_CONN_ST,          /* Connected - MNS Session is active */
    WICED_MCE_MN_CLOSING_ST        /* Closing is in progress */
};

/* state machine events */
enum
{
    /* these events are handled by the state machine */
    /* Make sure the following event order is consistent with MN min and max event(below)*/
    WICED_MCE_API_OPEN_EVT = 0,           /* Open MAS connection to peer device */
    WICED_MCE_API_CLOSE_EVT,              /* Close MAS connection to peer device */
    WICED_MCE_API_NOTIF_REG_EVT,          /* Notification registration request */
    WICED_MCE_API_UPD_INBOX_EVT,          /* Update inbox event */
    WICED_MCE_API_CHDIR_EVT,              /* Set folder */
    WICED_MCE_API_LIST_EVT,               /* Get msg/folder listing request */
    WICED_MCE_API_GET_MSG_EVT,            /* Get message request */
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
    WICED_MCE_API_GET_MAS_INS_INFO_EVT,   /* Get MAS instance info request */
#endif
    WICED_MCE_API_SET_STS_EVT,            /* API set message status request */
    WICED_MCE_API_PUSH_EVT,               /* API push message request */

    WICED_MCE_API_ABORT_EVT,              /* API request to abort current operation */
    WICED_MCE_SDP_FAIL_EVT,               /* SDP failed */
    WICED_MCE_SDP_OK_EVT,                 /* SDP finished OK */
    WICED_MCE_MA_OBX_CONN_RSP_EVT,        /* OBX Channel Connect Request */
    WICED_MCE_MA_OBX_PUT_RSP_EVT,         /* OBX Put response */
    WICED_MCE_MA_OBX_GET_RSP_EVT,         /* OBX Get response */
    WICED_MCE_MA_OBX_SETPATH_RSP_EVT,     /* Make or Change Directory */

    WICED_MCE_MA_OBX_TOUT_EVT,            /* OBX Channel timeout */
    WICED_MCE_MA_OBX_CLOSE_EVT,           /* OBX Channel Disconnected (Link Lost) */
    WICED_MCE_MA_OBX_ABORT_RSP_EVT,       /* OBX_operation aborted */
    WICED_MCE_MA_OBX_CMPL_EVT,            /* OBX operation completed */
    WICED_MCE_MA_CLOSE_CMPL_EVT,          /* Close complete event */
    WICED_MCE_RSP_TOUT_EVT,               /* response timer timeout event */

    WICED_MCE_MN_INT_CLOSE_EVT,           /* Close MNS connection internally */
    WICED_MCE_MN_OBX_CONN_REQ_EVT,        /* Connection request for MNS */
    WICED_MCE_MN_OBX_PUT_EVT,             /* Handle OBX Put request */
    WICED_MCE_MN_OBX_ACTION_EVT,          /* Handle OBX Action request */
    WICED_MCE_MN_OBX_ABORT_EVT,           /* Handle OBX abort request */
    WICED_MCE_MN_OBX_DISC_EVT,            /* Handle OBX disconnect request */
    WICED_MCE_MN_OBX_CLOSE_EVT,           /* Handle OBX close request */

    /* these events are handled outside the state machine */
    WICED_MCE_API_ENABLE_EVT,             /* API enable event */
    WICED_MCE_API_DISABLE_EVT,            /* API disable event */
    WICED_MCE_API_START_EVT,              /* Start MN server */
    WICED_MCE_API_STOP_EVT,               /* Stop MN server */
    WICED_MCE_API_DISCOVER_EVT,           /* API discover event */
    WICED_MCE_API_MN_CLOSE_EVT,           /* API close MNS connection */
    WICED_MCE_API_CLOSE_ALL_EVT           /* API close all event */ /* BSA_SPECIFIC */
};

typedef UINT16 wiced_mce_int_evt_t;

#define WICED_MCE_MN_EVT_MIN      WICED_MCE_MN_INT_CLOSE_EVT
#define WICED_MCE_MN_EVT_MAX      WICED_MCE_MN_OBX_CLOSE_EVT

#define WICED_MCE_CCB_IDX_MASK        0x0000000f
#define WICED_MCE_INST_ID_MASK        0x0000000f
#define WICED_MCE_TPARAM_TO_CCB_IDX(timer_param)              ((timer_param >> 8) & WICED_MCE_CCB_IDX_MASK)
#define WICED_MCE_TPARAM_TO_INST_ID(timer_param)              (timer_param & WICED_MCE_INST_ID_MASK)
#define WICED_MCE_CCB_INST_TO_TPARAM(ccb_idx, inst_id)        (((ccb_idx & WICED_MCE_CCB_IDX_MASK) << 8) | (inst_id & WICED_MCE_INST_ID_MASK))

/* active mas obex operation (Valid in connected state) */
#define WICED_MCE_OP_NONE         0
#define WICED_MCE_OP_GET_MSG      1
#define WICED_MCE_OP_PUSH_MSG     2
#define WICED_MCE_OP_FOLDER_LIST  3
#define WICED_MCE_OP_MSG_LIST     4
#define WICED_MCE_OP_CHDIR        5
#define WICED_MCE_OP_SEND_EVT     6
#define WICED_MCE_OP_NOTIF_REG    7
#define WICED_MCE_OP_SET_MSG_STS  8
#define WICED_MCE_OP_UPD_INBOX    9
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
#define WICED_MCE_OP_GET_MAS_INS_INFO     10
#endif

/* Response Timer Operations */
#define WICED_MCE_TIMER_OP_STOP   0
#define WICED_MCE_TIMER_OP_ABORT  1

/* File abort mask states */
/* Abort must receive cout and response before abort completed */
#define WICED_MCE_ABORT_REQ_NOT_SENT  0x1
#define WICED_MCE_ABORT_REQ_SENT      0x2
#define WICED_MCE_ABORT_RSP_RCVD      0x4
#define WICED_MCE_ABORT_COUT_DONE     0x8

#define WICED_MCE_ABORT_COMPLETED     (WICED_MCE_ABORT_REQ_SENT | WICED_MCE_ABORT_RSP_RCVD | WICED_MCE_ABORT_COUT_DONE)

/* MCE MN operation type */
#define WICED_MCE_MN_OPER_NONE            0
#define WICED_MCE_MN_OPER_MAP_EVT_RPT     1

#ifdef WICED_BT_TRACE_ENABLE
#define APPL_TRACE_API0         WICED_BT_TRACE
#define APPL_TRACE_API1         WICED_BT_TRACE
#define APPL_TRACE_API2         WICED_BT_TRACE
#define APPL_TRACE_API3         WICED_BT_TRACE
#define APPL_TRACE_EVENT0       WICED_BT_TRACE
#define APPL_TRACE_EVENT1       WICED_BT_TRACE
#define APPL_TRACE_EVENT2       WICED_BT_TRACE
#define APPL_TRACE_EVENT3       WICED_BT_TRACE
#define APPL_TRACE_DEBUG0       WICED_BT_TRACE
#define APPL_TRACE_DEBUG1       WICED_BT_TRACE
#define APPL_TRACE_DEBUG2       WICED_BT_TRACE
#define APPL_TRACE_DEBUG3       WICED_BT_TRACE
#define APPL_TRACE_WARNING0     WICED_BT_TRACE
#define APPL_TRACE_WARNING1     WICED_BT_TRACE
#define APPL_TRACE_WARNING2     WICED_BT_TRACE
#define APPL_TRACE_WARNING3     WICED_BT_TRACE
#define APPL_TRACE_ERROR0       WICED_BT_TRACE
#define APPL_TRACE_ERROR1       WICED_BT_TRACE
#define APPL_TRACE_ERROR2       WICED_BT_TRACE
#define APPL_TRACE_ERROR3       WICED_BT_TRACE
#else
#define APPL_TRACE_API0
#define APPL_TRACE_API1
#define APPL_TRACE_API2
#define APPL_TRACE_API3
#define APPL_TRACE_EVENT0
#define APPL_TRACE_EVENT1
#define APPL_TRACE_EVENT2
#define APPL_TRACE_EVENT3
#define APPL_TRACE_DEBUG0
#define APPL_TRACE_DEBUG1
#define APPL_TRACE_DEBUG2
#define APPL_TRACE_DEBUG3
#define APPL_TRACE_WARNING0
#define APPL_TRACE_WARNING1
#define APPL_TRACE_WARNING2
#define APPL_TRACE_WARNING3
#define APPL_TRACE_ERROR0
#define APPL_TRACE_ERROR1
#define APPL_TRACE_ERROR2
#define APPL_TRACE_ERROR3
#endif

/* Power management state for MCE */
#define WICED_MCE_PM_BUSY     0
#define WICED_MCE_PM_IDLE     1

typedef UINT8 wiced_mce_pm_state_t;

/* data type for WICED_MCE_API_ENABLE_EVT */
typedef struct
{
    BT_HDR              hdr;
    wiced_bt_mce_cback_t *p_cback;        /* pointer to application callback function */
    UINT8               app_id;

} wiced_mce_api_enable_t;

/* data type for all obex events
    hdr.event contains the MCE event
*/
typedef struct
{
    BT_HDR              hdr;
    wiced_bt_obex_handle_t        handle;
    wiced_bt_obex_evt_param_t     param;
    BT_HDR              *p_pkt;
    wiced_bt_obex_event_t         obx_event;
    UINT8               rsp_code;
} wiced_mce_obx_evt_t;

/* data type for WICED_MCE_API_ENABLE_EVT */
typedef struct
{
    BT_HDR              hdr;
    char                servicename[BD_NAME_LEN + 1];
    UINT8               sec_mask;
    BOOLEAN             use_srm;
    wiced_bt_ma_supported_features_t  mce_local_features;
} wiced_mce_api_mn_start_t;

/* data structure for WICED_MCE_API_DISCOVER_EVT */
typedef struct
{
    BT_HDR              hdr;
    wiced_bt_device_address_t             bd_addr;
}wiced_mce_api_discover_t;

/* data structure for WICED_MCE_API_OPEN_EVT */
typedef struct
{
    BT_HDR                      hdr;
    wiced_bt_device_address_t   bd_addr;
    wiced_bt_ma_inst_id_t       mas_inst_id;
    UINT8                       sec_mask;
}wiced_mce_api_open_t;

/* data type for WICED_MCE_API_MN_CLOSE_EVT */
typedef struct
{
    BT_HDR                      hdr;
    wiced_bt_ma_sess_handle_t   session_id;
} wiced_mce_api_mn_close_t;

/* data structure for WICED_MCE_API_ABORT_EVT */
typedef struct
{
    BT_HDR                      hdr;
    wiced_bt_device_address_t   bd_addr;
    wiced_bt_ma_inst_id_t       mas_inst_id;
} wiced_mce_api_abort_t;

/* data structure for WICED_MCE_API_NOTIF_REG_EVT */
typedef struct
{
    BT_HDR                      hdr;
    wiced_bt_ma_notif_status_t  status;
} wiced_mce_api_notif_reg_t;

/* data type for WICED_MCE_API_CHDIR_EVT */
typedef struct
{
    BT_HDR                  hdr;
    char                    *p_dir;    /* UTF-8 name from listing */
    wiced_bt_ma_dir_nav_t   flag;
} wiced_mce_api_setfolder_t;

#define     WICED_MCE_LIST_TYPE_DIR    0
#define     WICED_MCE_LIST_TYPE_MSG    1
typedef UINT8 wiced_mce_list_type_t;

/* data type for BTA_FTC_API_LISTDIR_EVT */
typedef struct
{
    BT_HDR                          hdr;
    UINT16                          max_list_count;
    UINT16                          list_start_offset;
    wiced_mce_list_type_t           list_type;
    wiced_bt_ma_msg_list_filter_param_t *p_param;
    char                            *p_folder;
} wiced_mce_api_list_t;

/* data structure for get message API call */
typedef struct
{
    BT_HDR                      hdr;
    wiced_bt_ma_get_msg_param_t param;
} wiced_mce_api_get_msg_t;

#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
/* data structure for get MAS instance information API call */
typedef struct
{
    BT_HDR                  hdr;
    wiced_bt_ma_inst_id_t   mas_instance_id;
}wiced_mce_api_get_mas_ins_info_t;
#endif

/* Set message status data structure */
typedef struct
{
    BT_HDR                      hdr;
    wiced_bt_ma_msg_handle_t    msg_handle;
    wiced_bt_ma_sts_indctr_t    indicator;
    wiced_bt_ma_sts_value_t     value;

}wiced_mce_api_set_sts_t;

/* data structure for Push Message */
typedef struct
{
    BT_HDR                          hdr;
    wiced_bt_ma_push_msg_param_t    param;
}wiced_mce_api_push_msg_t;

/* data structure for SDP result event */
typedef struct
{
    BT_HDR              hdr;
    UINT16              psm;
    UINT8               ccb_idx;
    UINT8               scn;
    UINT32              peer_features;
    wiced_bt_ma_inst_id_t mas_instance_id;
}wiced_mce_sdp_evt_t;

/* union of all event data types */
typedef union
{
    BT_HDR                     hdr;        /* API_DISABLE EVT */
    wiced_mce_api_enable_t     api_enable;
    wiced_mce_api_discover_t   api_discover;
    wiced_mce_api_open_t       api_open;
    wiced_mce_api_mn_close_t   api_mn_close;
    wiced_mce_api_abort_t      api_abort;
    wiced_mce_obx_evt_t        obx_evt;
    wiced_mce_api_mn_start_t   api_start;
    wiced_mce_api_notif_reg_t  api_notif_reg;
    wiced_mce_api_setfolder_t  api_setfolder;
    wiced_mce_api_list_t       api_list;
    wiced_mce_api_get_msg_t    api_getmsg;
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
    wiced_mce_api_get_mas_ins_info_t   api_get_mas_ins_info;
#endif
    wiced_mce_api_set_sts_t    api_setstatus;
    wiced_mce_sdp_evt_t        sdp_result;
    wiced_mce_api_push_msg_t   api_push_msg;
} wiced_mce_data_t;

typedef UINT8 wiced_mce_ma_state_t;
typedef UINT8 wiced_mce_mn_state_t;

/* OBX Packet Structure - Holds current response packet info */
typedef struct
{
    BT_HDR  *p_pkt;             /* (Get/Put) Holds the current OBX header for Put or Get */
    UINT8   *p_start;           /* (Get/Put) Start of the Body of the packet */
    UINT16   offset;            /* (Get/Put) Contains the current offset into the Body (p_start) */
    UINT16   bytes_left;        /* (Get/Put) Holds bytes available left in Obx packet */
    BOOLEAN  final_pkt;         /* (Put)     Holds the final bit of the Put packet */
} wiced_mce_obx_pkt_t;

typedef struct
{
    wiced_bt_ma_status_t    status;
    char                    *p_folder;
    wiced_bt_ma_msg_handle_t msg_handle;
    char                    msg_hdl_str[17];
    wiced_bt_obex_handle_t  obx_handle;     /* use MAC connection handle as MCE session ID */
    wiced_mce_obx_pkt_t     obx;            /* Holds the current OBX packet information */
    wiced_timer_t           rsp_timer;      /* response timer */
    wiced_mce_ma_state_t    ma_state;       /* state machine state for Message Aceess Client */
    wiced_bt_ma_push_msg_param_t push_param;
    UINT8                   obx_oper;       /* current active OBX operation PUT FILE or GET FILE */
    UINT8                   timer_oper;     /* current active response timer action (abort or close) */
    BOOLEAN                 first_req_pkt;  /* TRUE if retrieving the first packet of GET/PUT file */
    wiced_bt_ma_frac_deliver_t frac_deliver;
    UINT8                   aborting;       /* Non-zero if client is in process of aborting */
    BOOLEAN                 int_abort;
    UINT32                  obj_size;
    wiced_bt_ma_frac_req_t  frac_req;
    BOOLEAN                 req_pending;    /* TRUE when waiting for an obex response */
    wiced_bt_ma_inst_id_t   inst_id;
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
    tBTA_MA_MAS_INS_INFO    mas_ins_info;   /* MAS instance description*/
#endif
    BOOLEAN                 in_use;
    wiced_bt_ma_supported_features_t mce_peer_features; /*  peer MA feature, doesn't need to be part of MAP_1_2 */
    BOOLEAN                 sdp_pending;
    BOOLEAN                 is_opening;     /* True if in the openning process */
} wiced_mce_inst_cb_t;

typedef struct
{
    wiced_bt_ma_status_t    status;
    wiced_bt_sdp_discovery_db_t       *p_db;                  /* pointer to discovery database, all the instance share one sdp database */
    wiced_bt_sdp_discovery_complete_cback_t       *sdp_cback;
    wiced_bt_ma_inst_id_t   sdp_inst_id;            /* Instance ID of the first time SDP */
    UINT8                   sec_mask;
    UINT16                  peer_mtu;
    BOOLEAN                 sdp_pending;
    wiced_bt_device_address_t bd_addr;
    BOOLEAN                 in_use;
    char                    *p_devname;
    wiced_mce_inst_cb_t     icb[WICED_BT_MCE_NUM_INST]; /* An array of instance control block */
    UINT8                   active_count;           /* Number of active instance control block */
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
    wiced_bt_ma_inst_id_t         get_mas_inst_id;        /* Instance ID of the get mas instance info */
#endif
} wiced_mce_ma_cb_t;

typedef struct
{
    wiced_mce_mn_state_t    mn_state;               /* state machine state for Message Notification Service */
    UINT16                  mn_peer_mtu;
    UINT8                   mn_oper;
    BOOLEAN                 mn_rcv_inst_id;
    UINT8                   mn_inst_id;             /* instance id received from the Msg evt report*/
    char                    *p_devname;
    wiced_bt_obex_handle_t  obx_handle;
    wiced_bt_device_address_t bd_addr;
    BOOLEAN                 in_use;
} wiced_mce_mn_cb_t;

typedef struct
{
    wiced_mce_pm_state_t    pm_state;
    BOOLEAN                 in_use;
    wiced_bt_device_address_t bd_addr;
    BOOLEAN                 opened;
    BOOLEAN                 busy;
} wiced_mce_pm_cb_t;

typedef struct
{
    wiced_mce_ma_cb_t       ccb[WICED_BT_MCE_NUM_MA];    /* MA Client Control Blocks */
    wiced_mce_mn_cb_t       scb[WICED_BT_MCE_NUM_MN];    /* MN Server Control Blocks */
    wiced_mce_pm_cb_t       pcb[WICED_BT_MCE_NUM_PM];

    wiced_bt_mce_cback_t    *p_cback;               /* pointer to application callback function */
    wiced_bt_sdp_discovery_db_t       *p_api_dis_db;          /* pointer to discovery database for the API discover */
    wiced_bt_device_address_t api_dis_bd_addr;        /* Peer bd_addr associated with API discover */

    UINT8                   app_id;                 /* Application ID */
    BOOLEAN                 is_enabled;             /* MCE is enabled or not */
    BOOLEAN                 disabling;              /* TRUE if client is in process of disabling */

    /* MNS Client */
    BOOLEAN                 mn_started;
    wiced_bt_obex_handle_t  mn_obx_handle;
    UINT32                  mn_sdp_handle;          /* SDP record handle for MNS */
    UINT8                   mn_scn;                 /* SCN for MNS */
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
    UINT16                  mn_psm;                 /* PSM for Obex Over L2CAP [MAP 1.2] */
    wiced_bt_ma_supported_features_t mce_local_features; /* local MN supported features */
#endif
} wiced_mce_cb_t;


/******************************************************************************
**  Configuration Definitions
*******************************************************************************/
/*****************************************************************************
**  Global data
*****************************************************************************/

/* MAS control block */
extern wiced_mce_cb_t  wiced_mce_cb;

#define WICED_MCE_GET_MA_CB_PTR(ccb_idx) &(wiced_mce_cb.ccb[(ccb_idx)])
#define WICED_MCE_GET_MN_CB_PTR(scb_idx) &(wiced_mce_cb.scb[(scb_idx)])
#define WICED_MCE_GET_PM_CB_PTR(pcb_idx) &(wiced_mce_cb.pcb[(pcb_idx)])
#define WICED_MCE_GET_MA_INST_CB_PTR(ccb_idx, inst_idx) &(wiced_mce_cb.ccb[(ccb_idx)].icb[(inst_idx)])
#define WICED_MCE_GET_MA_OBX_PKT_PTR(ccb_idx, inst_idx) &(wiced_mce_cb.ccb[(ccb_idx)].icb[(inst_idx)].obx)

/*****************************************************************************
**  Function prototypes
*****************************************************************************/

extern void wiced_mce_send_event(BT_HDR *p_msg);
extern BOOLEAN wiced_mce_hdl_event(BT_HDR *p_msg);
extern void wiced_mce_ma_sm_execute(UINT8 ccb_idx, UINT8 icb_idx, UINT16 event, wiced_mce_data_t *p_data);
extern void wiced_mce_mn_sm_execute(UINT8 scb_idx, UINT16 event, wiced_mce_data_t *p_data);
extern void wiced_mce_mn_start_service(wiced_mce_cb_t *p_cb, wiced_mce_data_t *p_data);
extern void wiced_mce_mn_disable(wiced_mce_cb_t *p_cb, wiced_mce_data_t *p_data);
extern void wiced_mce_mn_sdp_register(wiced_mce_cb_t *p_cb, char *p_service_name);

/* action functions */
extern void wiced_mce_ma_init_sdp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_start_client(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_stop_client(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_close(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_push_msg(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_free_db(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_obx_conn_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_put_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_obx_get_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_obx_setpath_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_close_compl(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_ignore_obx(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_notif_reg(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_abort(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_rsp_tout(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_obx_abort_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_upd_inbox(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_chdir(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_list(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_get_msg(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
extern void wiced_mce_ma_get_mas_ins_info(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
#endif
extern void wiced_mce_ma_set_sts(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_trans_cmpl(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_ci_write(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_ci_read(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_ma_ci_open(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_initialize(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);

/* utility functions */
extern void wiced_mce_reset_icb (wiced_mce_inst_cb_t *p_icb);
extern void wiced_mce_get_listing(wiced_mce_inst_cb_t *p_icb, wiced_mce_api_list_t *p_data);
extern void wiced_mce_add_filler_byte(wiced_mce_obx_pkt_t *p_obx);
extern UINT8 *wiced_mce_read_app_params(BT_HDR *p_pkt, UINT8 tag, UINT16 *param_len);
extern wiced_bt_ma_status_t wiced_mce_convert_obx_to_status(wiced_bt_obex_status_t status);
extern UINT8 wiced_mce_send_get_req(wiced_mce_inst_cb_t *p_icb);
extern void wiced_mce_send_abort_req(wiced_mce_inst_cb_t *p_icb);
extern UINT8 wiced_mce_cont_put_file(wiced_mce_inst_cb_t *p_icb, wiced_bt_ma_push_msg_param_t *p_param);
extern void wiced_mce_send_ma_open_evt(UINT8 ccb_idx, UINT8 icb_idx, wiced_bt_ma_status_t status);
extern void wiced_mce_listing_err(wiced_mce_inst_cb_t *p_icb, BT_HDR **p_pkt, wiced_bt_ma_status_t status, wiced_bt_obex_rsp_code_t obx_rsp);
extern void wiced_mce_proc_put_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_proc_push_msg_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_proc_get_list_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_obx_evt_t *p_evt);
extern void wiced_mce_proc_get_msg_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
#if (defined(BTA_MAP_1_2_SUPPORTED) && BTA_MAP_1_2_SUPPORTED == TRUE)
extern void wiced_mce_proc_get_mas_ins_info_rsp(UINT8 ccb_idx, UINT8 icb_idx, wiced_mce_data_t *p_data);
#endif
extern void wiced_mce_ma_update_pm_state(UINT8 ccb_idx, UINT8 icb_idx);
extern void wiced_mce_mn_update_pm_state(wiced_mce_cb_t *p_cb, UINT8 scb_idx);
extern void wiced_mce_disable_complete(wiced_mce_cb_t *p_cb);
extern UINT8 wiced_mce_ma_get_num_of_act_conn(wiced_mce_cb_t *p_cb);
extern BOOLEAN wiced_mce_find_ma_cb_indexes(wiced_mce_data_t *p_msg, UINT8 *p_ccb_idx, UINT8 *p_icb_idx);
extern BOOLEAN wiced_mce_find_mn_cb_index(wiced_mce_data_t *p_msg, UINT8 *p_scb_idx);
extern BOOLEAN wiced_mce_find_available_ma_cb_index(wiced_bt_device_address_t p_bd_addr, UINT8 *p_ccb_idx);
extern BOOLEAN wiced_mce_find_inst_id_match_ma_icb_cb_index(UINT8 ccb_idx, wiced_bt_ma_inst_id_t inst_id, UINT8 *p_icb_idx);
extern BOOLEAN wiced_mce_find_obx_handle_match_ma_cb_indexes(wiced_bt_obex_handle_t obx_handle, UINT8 *p_ccb_idx, UINT8 *p_icb_idx);
extern BOOLEAN wiced_mce_find_bd_addr_match_ma_cb_index(wiced_bt_device_address_t p_bd_addr, UINT8 *p_idx);
extern wiced_mce_inst_cb_t *wiced_mce_get_mce_inst_cb_using_inst_id(UINT8 ccb_idx, wiced_bt_ma_inst_id_t inst_id);
extern wiced_mce_inst_cb_t *wiced_mce_get_mce_inst_cb_using_obx_handle(wiced_bt_obex_handle_t obx_handle);
extern wiced_mce_ma_cb_t *wiced_mce_get_ma_cb_using_bd_addr(wiced_bt_device_address_t p_bd_addr);
extern BOOLEAN wiced_mce_find_available_mn_cb_index(wiced_bt_device_address_t p_bd_addr, UINT8 *p_scb_idx);
extern BOOLEAN wiced_mce_find_bd_addr_match_mn_cb_index(wiced_bt_device_address_t p_bd_addr, UINT8 *p_scb_idx);
extern BOOLEAN wiced_mce_find_obx_handle_match_mn_cb_index(wiced_bt_obex_handle_t obx_handle, UINT8 *p_scb_idx);
extern void GKI_freebuf(void *memPtr);

/* action function for MNS */
extern void wiced_mce_mn_int_close(UINT8 scb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_mn_obx_connect(UINT8 scb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_mn_obx_disc(UINT8 scb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_mn_obx_put(UINT8 scb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_mn_obx_action(UINT8 scb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_mn_obx_abort(UINT8 scb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_mn_obx_close(UINT8 scb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_mn_conn_err_rsp(UINT8 scb_idx, wiced_mce_data_t *p_data);
extern void wiced_mce_mn_ignore_obx(UINT8 scb_idx, wiced_mce_data_t *p_data);

/* miscellaneous functions */
extern wiced_mce_pm_cb_t *wiced_mce_get_pm_cb_using_bd_addr(wiced_bt_device_address_t p_bd_addr);
extern BOOLEAN wiced_mce_find_avail_pm_cb_idx(UINT8 *p_idx);
extern void wiced_mce_pm_conn_open(wiced_bt_device_address_t bd_addr);
extern void wiced_mce_pm_conn_close(wiced_bt_device_address_t bd_addr);

#endif /* WICED_BT_MCE_INT_H */
