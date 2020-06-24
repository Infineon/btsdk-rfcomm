/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
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
 * WICED BT OPP server interface
 *
 */

#ifndef WICED_BT_OPS_INT_H
#define WICED_BT_OPS_INT_H

#include "wiced_bt_obex.h"
#include "wiced_bt_ops_api.h"
#include "wiced_bt_ops_co.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"

/*****************************************************************************
**  Constants and data types
*****************************************************************************/


/* OPS active obex operation (Valid in connected state) */
#define OPS_OP_NONE         0
#define OPS_OP_PULL_OBJ     1
#define OPS_OP_PUSH_OBJ     2

#define GOEP_ENHANCED_VERSION       0x0102

#define WICED_BT_OPS_INVALID_FD     (-1)
#define WICED_BT_OPS_LEN_UNKNOWN   0xFFFFFFFF
#define WICED_BT_OPS_INVALID_HDR_LEN  0xFFFF

//TODO REMOVE
#define BTM_SEC_SERVICE_OBEX        6
#define BTM_SEC_PROTO_RFCOMM        3

#undef  OBX_CMD_POOL_SIZE
#define OBX_CMD_POOL_SIZE           256

#undef  OBEX_LRG_DATA_POOL_SIZE
#define OBEX_LRG_DATA_POOL_SIZE     GKI_MAX_BUF_SIZE

/* character used as path separator */
#ifndef wiced_bt_ops_fs_path_separator
#define wiced_bt_ops_fs_path_separator   ((char) 0x2f)   /* 0x2f ('/'), or 0x5c ('\') */
#endif

/* maximum path length supported */
#ifndef wiced_bt_ops_fs_path_len
#define wiced_bt_ops_fs_path_len         294
#endif

#ifndef wiced_bt_ops_fs_file_len
#define wiced_bt_ops_fs_file_len         256
#endif

typedef struct
{
    UINT16  max_file_len;           /* Maximum size file name */
    UINT16  max_path_len;           /* Maximum path length (includes appended file name) */
    char    path_separator;         /* 0x2f ('/'), or 0x5c ('\') */
} wiced_bt_ops_fs_cfg_t;

extern wiced_bt_ops_fs_cfg_t * p_wiced_bt_ops_fs_cfg;




/* Open Complete Event */
typedef struct
{
    BT_HDR                      hdr;
    wiced_bt_ops_co_status_t    status;
    UINT32                      file_size;
    int                         fd;
    const char                  *p_file;
} wiced_bt_ops_ci_open_evt_t;

/* Read Ready Event */
typedef struct
{
    BT_HDR                      hdr;
    wiced_bt_ops_co_status_t    status;
    int                         fd;
    UINT16                      num_read;
} wiced_bt_ops_ci_read_evt_t;

/* Write Ready Event */
typedef struct
{
    BT_HDR                      hdr;
    wiced_bt_ops_co_status_t    status;
    int                         fd;
} wiced_bt_ops_ci_write_evt_t;

/* state machine events */
enum
{
    /* these events are handled by the state machine */
    WICED_BT_OPS_API_DISABLE_EVT,

    WICED_BT_OPS_API_ACCESSRSP_EVT,      /* Response to an access request */
    WICED_BT_OPS_API_CLOSE_EVT,          /* Close API */
    WICED_BT_OPS_CI_OPEN_EVT,            /* Response to File Open request */
    WICED_BT_OPS_CI_WRITE_EVT,           /* Response to Write request */
    WICED_BT_OPS_CI_READ_EVT,            /* Response to Read request */
    WICED_BT_OPS_OBX_CONN_EVT,           /* OBX Channel Connect Request */
    WICED_BT_OPS_OBX_DISC_EVT,           /* OBX Channel Disconnect */
    WICED_BT_OPS_OBX_ABORT_EVT,          /* OBX_operation aborted */
    WICED_BT_OPS_OBX_CLOSE_EVT,          /* OBX Channel Disconnected (Link Lost) */
    WICED_BT_OPS_OBX_PUT_EVT,            /* Write file data or delete */
    WICED_BT_OPS_OBX_GET_EVT,            /* Read file data or folder listing */
    WICED_BT_OPS_OBX_ACTION_EVT,         /* OBX Action Request */
    WICED_BT_OPS_CLOSE_CMPL_EVT,         /* Finished closing channel */

    /* these events are handled outside the state machine */
    WICED_BT_OPS_API_ENABLE_EVT
};

typedef UINT16 wiced_bt_ops_int_evt_t;

typedef UINT8 wiced_bt_ops_state_t;

/* data type for WICED_BT_OPS_API_ENABLE_EVT */
typedef struct
{
    BT_HDR              hdr;
    wiced_bt_ops_cback_t        *p_cback;
    wiced_bt_ops_data_cback_t   *p_data_cback;
    wiced_bt_op_fmt_mask_t      formats;
    UINT8                       sec_mask;
    UINT8                       scn;
    BOOLEAN                     srm;
    UINT8                       app_id;
} wiced_bt_ops_api_enable_t;

/* data type for WICED_BT_OPS_API_ACCESSRSP_EVT */
typedef struct
{
    BT_HDR          hdr;
    char            *p_name;
    wiced_bt_op_oper_t    oper;
    wiced_bt_op_access_t  flag;
} wiced_bt_ops_api_access_rsp_t;

/* data type for all obex events
    hdr.event contains the OPS event
*/
typedef struct
{
    BT_HDR              hdr;
    wiced_bt_obex_handle_t         handle;
    wiced_bt_obex_evt_param_t      param;
    BT_HDR             *p_pkt;
    wiced_bt_obex_event_t          obx_event;
    UINT8               rsp_code;
} wiced_bt_ops_obx_evt_t;

/* union of all event data types */
typedef union
{
    BT_HDR                  hdr;
    wiced_bt_ops_api_enable_t     api_enable;
    wiced_bt_ops_api_access_rsp_t  api_access;
    wiced_bt_ops_obx_evt_t      obx_evt;
    wiced_bt_ops_ci_open_evt_t     open_evt;
    wiced_bt_ops_ci_read_evt_t     read_evt;
    wiced_bt_ops_ci_write_evt_t    write_evt;
} wiced_bt_ops_data_t;

/* OBX Response Packet Structure - Holds current response packet info */
typedef struct
{
    BT_HDR      *p_pkt;         /* (Pull/Push) Holds the current OBX hdr for Push or Pull */
    UINT8       *p_start;       /* (Pull/Push) Start of the Body of the packet */
    UINT16       offset;        /* (Pull/Push) Contains the current offset into the Body (p_start) */
    UINT16       bytes_left;    /* (Pull/Push) Holds bytes available left in Obx packet */
    BOOLEAN      final_pkt;     /* (Push)      Holds the final bit of the Push packet */
} wiced_bt_ops_obx_pkt_t;

/* Power management state for OPS */
#define WICED_BT_OPS_PM_BUSY     0
#define WICED_BT_OPS_PM_IDLE     1

/* OPS control block */
typedef struct
{
    wiced_bt_ops_cback_t        *p_cback;         /* pointer to application callback function */
    wiced_bt_ops_data_cback_t   *p_data_cback;    /*pointer to data callback function */
    char            *p_name;        /* Holds name of current operation */
    char            *p_path;        /* Holds path of current operation */
    wiced_bt_ops_obx_pkt_t obx;           /* Holds the current OBX packet information */
    UINT32           sdp_handle;    /* SDP record handle */
    UINT32           file_length;   /* length of file being Push/Pull */
    int              fd;            /* File Descriptor of opened file */
    BD_ADDR          bd_addr;       /* Device currently connected to */
    wiced_bt_obex_handle_t      obx_handle;
    UINT16           peer_mtu;
    UINT16           psm;           /* PSM for Obex Over L2CAP */
    BOOLEAN          srm;           /* TRUE, to use SIngle Response Mode */
    UINT8            scn;           /* SCN of the OPP server */
    wiced_bt_op_fmt_mask_t formats;       /* supported object formats */
    wiced_bt_ops_state_t   state;         /* state machine state */
    wiced_bt_op_fmt_t      obj_fmt;       /* file format of received object */
    UINT8            obx_oper;      /* current active OBX operation PUSH OBJ, or PULL OBJ */
    UINT8            app_id;
    wiced_bt_op_oper_t     acc_active;    /* op code when waiting for an access rsp (API) (0 not active) */
    BOOLEAN          cout_active;   /* TRUE when waiting for a call-in function */
    BOOLEAN          aborting;      /* TRUE when waiting for a call-in function */
    UINT8            pm_state;
} wiced_bt_ops_cb_t;

/*****************************************************************************
**  Global data
*****************************************************************************/

/* OPS control block */
#if WICED_BT_DYNAMIC_MEMORY == FALSE
extern wiced_bt_ops_cb_t  wiced_bt_ops_cb;
#else
extern wiced_bt_ops_cb_t *wiced_bt_ops_cb_ptr;
#define wiced_bt_ops_cb (*wiced_bt_ops_cb_ptr)
#endif


/*****************************************************************************
**  Function prototypes
*****************************************************************************/

extern BOOLEAN  wiced_bt_ops_hdl_event(BT_HDR *p_msg);
extern void     wiced_bt_ops_sm_execute(wiced_bt_ops_cb_t *p_cb, UINT16 event, wiced_bt_ops_data_t *p_data);
extern void     wiced_bt_ops_obx_cback (wiced_bt_obex_handle_t handle, wiced_bt_obex_event_t event,
                                   wiced_bt_obex_evt_param_t param, UINT8 *p_pkt);

/* action functions */
extern void wiced_bt_ops_api_disable(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_api_accessrsp(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_api_close(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_ci_write(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_ci_read(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_ci_open(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_obx_connect(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_obx_disc(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_obx_close(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_obx_abort(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_obx_put(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_obx_get(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_obx_action(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_gasp_err_rsp(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_close_complete(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);

/* object store */
extern void wiced_bt_ops_init_get_obj(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_proc_get_obj(wiced_bt_ops_cb_t *p_cb);
extern void wiced_bt_ops_proc_put_obj(BT_HDR *p_pkt);

/* miscellaneous functions */
extern void wiced_bt_ops_enable(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_api_enable_t *p_data);
extern void wiced_bt_ops_get_obj_rsp(UINT8 rsp_code, UINT16 num_read);
extern void wiced_bt_ops_put_obj_rsp(UINT8 rsp_code);
extern void wiced_bt_ops_clean_getput(wiced_bt_ops_cb_t *p_cb, BOOLEAN is_aborted);
extern void wiced_bt_ops_discard_data(UINT16 event, wiced_bt_ops_data_t *p_data);
extern void wiced_bt_ops_req_app_access (wiced_bt_op_oper_t oper, wiced_bt_ops_cb_t *p_cb);
extern wiced_bt_op_fmt_t wiced_bt_ops_fmt_supported(char *p, wiced_bt_op_fmt_mask_t fmt_mask);

#endif /* WICED_BT_OPS_INT_H */
