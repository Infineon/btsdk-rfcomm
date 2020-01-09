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

/*****************************************************************************
**
**  Name:         obx_int.h
**
**  File:         OBEX Internal header file
**
**
*****************************************************************************/
#ifndef OBX_INT_H
#define OBX_INT_H

#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_gki.h"
#include "wiced_timer.h"
#include "wiced_bt_obex.h"


#define RFCOMM_MIN_OFFSET           5       /* from rfc_defs.h */
#define PORT_SUCCESS                0       /* from port_api.h */

#define OBEX_CMD_POOL_SIZE          256
#define OBEX_CMD_POOL_ID            2
#define OBEX_LRG_DATA_POOL_SIZE     GKI_MAX_BUF_SIZE
#define OBEX_LRG_DATA_POOL_ID       4

#undef  MAX_RFC_PORTS
#define MAX_RFC_PORTS               5

#undef  MAX_L2CAP_CHANNELS
#define MAX_L2CAP_CHANNELS          10

#undef  BT_USE_TRACES
#define BT_USE_TRACES               TRUE

#define OBEX_SESS_TIMEOUT_VALUE     600
#define OBEX_TIMEOUT_VALUE          60
#define OBEX_DISC_TOUT_VALUE        5

#define OBEX_NUM_SERVERS            4
#define OBEX_NUM_SR_SESSIONS        4
#define OBEX_MAX_SR_SESSION         4
#define OBEX_MAX_SUSPEND_SESSIONS   4

#define OBEX_NUM_CLIENTS            4

#define OBEX_MAX_REALM_LEN          30

#define OBEX_MAX_RX_QUEUE_COUNT     3

#define OBEX_CONN_ID_SIZE           4
#define OBEX_PKT_LEN_SIZE           2

#define OBEX_CONN_HDRS_OFFSET       7
#define OBEX_SESS_HDRS_OFFSET       3
#define OBEX_DISCON_HDRS_OFFSET     3
#define OBEX_PUT_HDRS_OFFSET        3
#define OBEX_GET_HDRS_OFFSET        3
#define OBEX_SETPATH_REQ_HDRS_OFFSET 5
#define OBEX_ABORT_HDRS_OFFSET      3
#define OBEX_ACTION_HDRS_OFFSET     3
#define OBEX_RESPONSE_HDRS_OFFSET   3

#define OBEX_TRACE_API0             WICED_BT_TRACE
#define OBEX_TRACE_API1             WICED_BT_TRACE
#define OBEX_TRACE_API2             WICED_BT_TRACE
#define OBEX_TRACE_API3             WICED_BT_TRACE
#define OBEX_TRACE_API4             WICED_BT_TRACE
#define OBEX_TRACE_API5             WICED_BT_TRACE
#define OBEX_TRACE_EVENT0           WICED_BT_TRACE
#define OBEX_TRACE_EVENT1           WICED_BT_TRACE
#define OBEX_TRACE_EVENT2           WICED_BT_TRACE
#define OBEX_TRACE_EVENT3           WICED_BT_TRACE
#define OBEX_TRACE_EVENT4           WICED_BT_TRACE
#define OBEX_TRACE_EVENT5           WICED_BT_TRACE
#define OBEX_TRACE_DEBUG0           WICED_BT_TRACE
#define OBEX_TRACE_DEBUG1           WICED_BT_TRACE
#define OBEX_TRACE_DEBUG2           WICED_BT_TRACE
#define OBEX_TRACE_DEBUG3           WICED_BT_TRACE
#define OBEX_TRACE_DEBUG4           WICED_BT_TRACE
#define OBEX_TRACE_DEBUG5           WICED_BT_TRACE
#define OBEX_TRACE_DEBUG6           WICED_BT_TRACE
#define OBEX_TRACE_WARNING0         WICED_BT_TRACE
#define OBEX_TRACE_WARNING1         WICED_BT_TRACE
#define OBEX_TRACE_WARNING2         WICED_BT_TRACE
#define OBEX_TRACE_WARNING3         WICED_BT_TRACE
#define OBEX_TRACE_WARNING4         WICED_BT_TRACE
#define OBEX_TRACE_WARNING5         WICED_BT_TRACE
#define OBEX_TRACE_ERROR0           WICED_BT_TRACE
#define OBEX_TRACE_ERROR1           WICED_BT_TRACE
#define OBEX_TRACE_ERROR2           WICED_BT_TRACE
#define OBEX_TRACE_ERROR3           WICED_BT_TRACE
#define OBEX_TRACE_ERROR4           WICED_BT_TRACE
#define OBEX_TRACE_ERROR5           WICED_BT_TRACE

#define WC_ASSERT(_x) if ( !(_x) )  WICED_BT_TRACE("WC_ASSERT at %s line %d\n", __FILE__, __LINE__);

typedef wiced_bt_obex_status_t          tOBEX_STATUS;
typedef wiced_bt_obex_handle_t          tOBEX_HANDLE;
typedef wiced_bt_obex_event_t           tOBEX_EVENT;
typedef wiced_bt_obex_evt_param_t       tOBEX_EVT_PARAM;
typedef wiced_bt_obex_client_cback_t    tOBEX_CL_CBACK;
typedef wiced_bt_obex_server_cback_t    tOBEX_SR_CBACK;
typedef wiced_bt_obex_sess_state_t      tOBEX_SESS_ST;
typedef wiced_bt_obex_target_t          tOBEX_TARGET;
typedef wiced_bt_obex_triplet_t         tOBEX_TRIPLET;
typedef wiced_bt_obex_start_params_t    tOBEX_StartParams;
typedef wiced_bt_obex_sess_evt_t        tOBEX_SESS_EVT;
typedef wiced_bt_obex_conn_evt_t        tOBEX_CONN_EVT;

/* 18 is 7/OBEX_CONN_HDRS_OFFSET + 5/conn id, 2/ssn, 2/srm 2/srm_param */
#define OBEX_HDR_OFFSET              (18 + L2CAP_MIN_OFFSET)
/* this is not needed if OBEX_HDR_OFFSET is 18+ */
#define OBEX_MAX_CONN_HDR_EXTRA      8 /* 5/conn id, 2/ssn, 2/srm 2/srm_param - (5/setpath + 5/conn_id - 7/conn) */

#define OBEX_DEFAULT_TARGET_LEN      0xFF
#define OBEX_INITIAL_CONN_ID         0x1

#define OBEX_MAX_EVT                OBEX_PASSWORD_EVT
#define OBEX_MAX_OFFSET_IND         OBEX_ABORT_RSP_EVT /* This is used to access obx_hdr_start_offset */

#define OBEX_MAX_OK_RSP             OBEX_RSP_PART_CONTENT

/*
** Define Miscellaneous Constants
*/
#define OBEX_VERSION                     0x10    /* Version 1.0 */
#define OBEX_CONN_FLAGS                  0       /* Connect flags per IrOBEX spec */
#define OBEX_SETPATH_CONST               0       /* SetPath Request constants per IrOBEX spec */
#define OBEX_INVALID_CONN_ID             0xFFFFFFFF  /* invalid connection ID per IrOBEX spec */
#define OBEX_INFINITE_TIMEOUT            0xFFFFFFFF

/* Header Identifier Data Type Constants */
#define OBEX_HI_TYPE_MASK                0xC0    /* This mask get the encoding (data type) of the header ID. */
#define OBEX_HI_ID_MASK                  0x3F    /* This mask gets the meaning of the header ID. */
#define OBEX_HI_TYPE_UNIC                0x00    /* Null terminated Unicode text */
#define OBEX_HI_TYPE_ARRAY               0x40    /* Unstructured octet array (byte sequence) */
#define OBEX_HI_TYPE_BYTE                0x80    /* 8-bit integer */
#define OBEX_HI_TYPE_INT                 0xC0    /* 32-bit integer */

/* the rules for OBX handles: (some of the definitions are in obx_api.h)
 *
 * tOBEX_HANDLE is UINT16
 * It was UINT8, the support for multiple clients on the same SCN for MAP is required
 *
 * LSB (0x00FF) is the same as the old definition.
 * The 0x80 bit (OBEX_CL_HANDLE_MASK) is set for client connections.
 * The 0x40 bit (OBEX_HANDLE_RX_MTU_MASK) is used internally for RFCOMM to allocate a buffer to receive data
 *
 * The MSB (0xFF00) is used for enhancements add for BTE release 3.15
 * This byte is the session index; used for server only.
 */

#define OBEX_CL_HANDLE_MASK          0x80
#define OBEX_CL_CB_IND_MASK          0x007F
#define OBEX_HANDLE_RX_MTU_MASK      0x40
#define OBEX_LOCAL_NONCE_SIZE        OBEX_MIN_NONCE_SIZE  /* 4 - 16 per IrOBEX spec. local nonce is always 4 */

#define OBEX_MAX_EVT_MAP_NUM     (OBEX_ABORT_REQ_SEVT+1)

#define OBEX_PORT_EVENT_MASK  (PORT_EV_RXCHAR | PORT_EV_TXEMPTY | \
                              PORT_EV_FC | PORT_EV_FCS | PORT_EV_CONNECT_ERR)

#define OBEX_BAD_SM_EVT              0xFF

enum
{
    OBEX_CS_NULL,                /* 0    0 */
    OBEX_CS_NOT_CONNECTED,       /* 1    1 */
    OBEX_CS_SESSION_REQ_SENT,    /* 2      */
    OBEX_CS_CONNECT_REQ_SENT,    /* 3    2 */
    OBEX_CS_CONNECTED,           /* 4    3 */
    OBEX_CS_DISCNT_REQ_SENT,     /* 5    4 */
    OBEX_CS_SETPATH_REQ_SENT,    /* 6    5 */
    OBEX_CS_ACTION_REQ_SENT,     /* 7      */
    OBEX_CS_ABORT_REQ_SENT,      /* 8    6 */
    OBEX_CS_PUT_REQ_SENT,        /* 9    7 */
    OBEX_CS_GET_REQ_SENT,        /*10    8 */
    OBEX_CS_PUT_TRANSACTION,     /*11    9 */
    OBEX_CS_GET_TRANSACTION,     /*12   10 */
    OBEX_CS_PUT_SRM,             /*13      */
    OBEX_CS_GET_SRM,             /*14      */
    OBEX_CS_PARTIAL_SENT,        /*15   11 */
    OBEX_CS_MAX
};

typedef UINT8 tOBEX_CL_STATE;
#define OBEX_CL_STATE_DROP   0x80 /* this is used only in session_info[] to mark link drop suspend*/

enum
{
    OBEX_CONNECT_REQ_CEVT,   /* API call to send a CONNECT request. */
    OBEX_SESSION_REQ_CEVT,   /* API call to send a SESSION request. */
    OBEX_DISCNT_REQ_CEVT,    /* API call to send a DISCONNECT request. */
    OBEX_PUT_REQ_CEVT,       /* API call to send a PUT request. */
    OBEX_GET_REQ_CEVT,       /* API call to send a GET request.*/
    OBEX_SETPATH_REQ_CEVT,   /* API call to send a SETPATH request. */
    OBEX_ACTION_REQ_CEVT,    /* API call to send an ACTION request. */
    OBEX_ABORT_REQ_CEVT,     /* API call to send an ABORT request. */
    OBEX_OK_CFM_CEVT,        /* Received success response from server. */
    OBEX_CONT_CFM_CEVT,      /* Received continue response from server.  */
    OBEX_FAIL_CFM_CEVT,      /* Received failure response from server. */
    OBEX_PORT_CLOSE_CEVT,    /* Transport is closed. */
    OBEX_TX_EMPTY_CEVT,      /* Transmit Queue Empty */
    OBEX_FCS_SET_CEVT,       /* Data flow enable */
    OBEX_STATE_CEVT,         /* Change state. */
    OBEX_TIMEOUT_CEVT,       /* Timeout occurred. */
    OBEX_MAX_CEVT
};
typedef UINT8 tOBEX_CL_EVENT;

#define OBEX_MAX_API_CEVT    OBEX_ABORT_REQ_CEVT

enum
{
    OBEX_CONNECT_REQ_SEVT,   /* 1  1 Received CONNECT request from client. */
    OBEX_SESSION_REQ_SEVT,   /* 2    Received SESSION request from client. */
    OBEX_DISCNT_REQ_SEVT,    /* 3  2 Received DISCONNECT request from client. */
    OBEX_PUT_REQ_SEVT,       /* 4  3 Received PUT request from client. */
    OBEX_GET_REQ_SEVT,       /* 5  4 Received GET request from client. */
    OBEX_SETPATH_REQ_SEVT,   /* 6  5 Received SETPATH request from client. */
    OBEX_ACTION_REQ_SEVT,    /* 7    Received ACTION request  from client. */
    OBEX_ABORT_REQ_SEVT,     /* 8  6 Received ABORT request from client. */
    OBEX_CONNECT_CFM_SEVT,   /* 9  7 API call to send a CONNECT response. */
    OBEX_SESSION_CFM_SEVT,   /*10    API call to send a SESSION response. */
    OBEX_DISCNT_CFM_SEVT,    /*11  8 API call to send a DISCONNECT response or close the connection to the client. */
    OBEX_PUT_CFM_SEVT,       /*12  9 API call to send a PUT response. */
    OBEX_GET_CFM_SEVT,       /*13 10 API call to send a GET response. */
    OBEX_SETPATH_CFM_SEVT,   /*14 11 API call to send a SETPATH response. */
    OBEX_ACTION_CFM_SEVT,    /*15    API call to send an ACTION response. */
    OBEX_ABORT_CFM_SEVT,     /*16 12 API call to send an ABORT response. */
    OBEX_PORT_CLOSE_SEVT,    /*17 13 Transport is closed. */
    OBEX_FCS_SET_SEVT,       /*18 14 Data flow enable */
    OBEX_STATE_SEVT,         /*19 15 Change state. */
    OBEX_TIMEOUT_SEVT,       /*20 16 Timeout has occurred. */
    OBEX_BAD_REQ_SEVT,       /*21 17 Received a bad request from client. */
    OBEX_TX_EMPTY_SEVT,      /*22 18 Transmit Queue Empty */
    OBEX_MAX_SEVT
};
typedef UINT8 tOBEX_SR_EVENT;

#define OBEX_SEVT_DIFF_REQ_CFM       (OBEX_CONNECT_CFM_SEVT - OBEX_CONNECT_REQ_SEVT) /* the index difference between *REQ_SEVT and *CFM_SEVT */
#define OBEX_SEVT_MAX_REQ            OBEX_ACTION_REQ_SEVT /* last *REQ_SEVT */

enum
{
    OBEX_SS_NULL,                /* 0   0 */
    OBEX_SS_NOT_CONNECTED,       /* 1   1 */
    OBEX_SS_SESS_INDICATED,      /* 2     */
    OBEX_SS_CONN_INDICATED,      /* 3   2 */
    OBEX_SS_CONNECTED,           /* 4   3 */
    OBEX_SS_DISCNT_INDICATED,    /* 5   4 */
    OBEX_SS_SETPATH_INDICATED,   /* 6   5 */
    OBEX_SS_ACTION_INDICATED,    /* 7    */
    OBEX_SS_ABORT_INDICATED,     /* 8   6 */
    OBEX_SS_PUT_INDICATED,       /* 9   7 */
    OBEX_SS_GET_INDICATED,       /*10   8 */
    OBEX_SS_PUT_TRANSACTION,     /*11   9 */
    OBEX_SS_GET_TRANSACTION,     /*12  10 */
    OBEX_SS_PUT_SRM,             /*13    */
    OBEX_SS_GET_SRM,             /*14    */
    OBEX_SS_PARTIAL_SENT,        /*15  11 */
    OBEX_SS_WAIT_CLOSE,          /*16  12 */
    OBEX_SS_MAX
};
typedef UINT8 tOBEX_SR_STATE;

typedef UINT8 tOBEX_STATE;   /* this must be the same type as tOBEX_SR_STATE and tOBEX_CL_STATE */

typedef struct
{
    UINT16  pkt_len;/* the packet length */
    UINT8   code;   /* the response/request code with the final bit */
    UINT8   sm_evt; /* The state machine event*/
} tOBEX_RX_HDR;

typedef void (tOBEX_CLOSE_FN) (UINT16);
typedef BOOLEAN (tOBEX_SEND_FN) (void *p_cb);
typedef UINT8 (*tOBEX_VERIFY_OPCODE)(UINT8 opcode, tOBEX_RX_HDR *p_rxh);

#define OBEX_SRM_NO          0x00    /* SRM is not enabled and not engaged */
#define OBEX_SRM_ENABLE      0x01    /* SRM is enabled. */
#define OBEX_SRM_PARAM_AL    0x02    /* SRMP is allowed only in the beginning of a transaction */
#define OBEX_SRM_REQING      0x04    /* requesting/requested to enable SRM. */
#define OBEX_SRM_ABORT       0x08    /* (server) abort/reject at SRM GET/PUT rsp API*/
#define OBEX_SRM_ENGAGE      0x10    /* SRM is engaged. */
#define OBEX_SRM_NEXT        0x20    /* peer is ready for next packet. */
#define OBEX_SRM_WAIT        0x40    /* wait for peer. */
#define OBEX_SRM_WAIT_UL     0x80    /* wait for upper layer. */
typedef UINT8 tOBEX_SRM;

#define OBEX_SRMP_WAIT       0x40    /* wait for peer  */
#define OBEX_SRMP_NONF       0x80    /* handle GET non-final (used by server only) */
#define OBEX_SRMP_NONF_EVT   0x20    /* report GET non-final req event (used by server only) */
#define OBEX_SRMP_SESS_FST   0x01    /* mark the session resume. The SSN on first req might not match */
#define OBEX_SRMP_CL_FINAL   0x02    /* handle GET Response final with SRMP=1 (used by client only) */
#define OBEX_SRMP_WAITING_CLEARED 0x04 /* Receiving SRMP=1 while SRM is enabled, wait peer to clear SRMP */
typedef UINT8 tOBEX_SRMP;

/* Obex Header Values for the SRM header */
#define OBEX_HV_SRM_DISABLE  0x00    /* SRM header value - disable */
#define OBEX_HV_SRM_ENABLE   0x01    /* SRM header value - enable */
#define OBEX_HV_SRM_IND      0x02    /* SRM header value - indicate support */

/* Obex Header Values for the SRM Parameter header */
#define OBEX_HV_SRM_PARAM_MORE   0x00    /* SRM Param header value - request additional packet */
#define OBEX_HV_SRM_PARAM_WAIT   0x01    /* SRM Param header value - wait for next req/rsp */
#define OBEX_HV_SRM_PARAM_COMBO  0x02    /* SRM Param header value - next and wait */

/* Session Opcode Definitions: */
#define OBEX_SESS_OP_CREATE              0x00 /* Create Session */
#define OBEX_SESS_OP_CLOSE               0x01 /* Close Session */
#define OBEX_SESS_OP_SUSPEND             0x02 /* Suspend Session */
#define OBEX_SESS_OP_RESUME              0x03 /* Resume Session */
#define OBEX_SESS_OP_SET_TIME            0x04 /* Set Timeout */
#define OBEX_SESS_OP_TRANSPORT           0xFF /* transport dropped */
typedef UINT8   tOBEX_SESS_OP;

/* offset for header functions to access fields */
#define OBEX_CONNECT_MTU_OFFSET      5
#define OBEX_SETPATH_FLAG_OFFSET     3

#define OBEX_UNICODE_SIZE            2 /* sizeof(UINT16) */

#define OBEX_INVALID_HDR_LEN         0xFFFF

#define OBEX_MIN_NONCE_SIZE          4   /* fixed size per IrOBEX spec */
#define OBEX_NONCE_SIZE              16  /* fixed size per IrOBEX spec */
#define OBEX_DIGEST_SIZE             16  /* fixed size per IrOBEX spec */
#define OBEX_SESSION_ID_SIZE         16  /* fixed size per IrOBEX spec */
#define OBEX_SESSION_INFO_SIZE       32  /* OBEX_SESSION_ID_SIZE + 4(local nonce) + 4 (connection id) + 4 (timeout) + 2(mtu) + 1(state) + 1(srm)  */
#define OBEX_SESSION_INFO_NONCE_IDX  16  /* The index to the (local nonce) in session info */
#define OBEX_SESSION_INFO_ID_IDX     20  /* The index to the (connection id) in session info */
#define OBEX_SESSION_INFO_TO_IDX     24  /* The index to the (timeout) in session info */
#define OBEX_SESSION_INFO_MTU_IDX    28  /* The index to peer MTU in session info */
#define OBEX_SESSION_INFO_ST_IDX     30  /* The index to sr/cl state in session info */
#define OBEX_SESSION_INFO_SRM_IDX    31  /* The index to srm in session info */
#define OBEX_TIMEOUT_SIZE            4

/* handle related definitions */
#define OBEX_SESS_SHIFT              8
#define OBEX_ENC_SESS_HANDLE(oh, os) (((os)<<OBEX_SESS_SHIFT)|(oh))
#define OBEX_HANDLE_MASK             0xFF
#define OBEX_SESS_MASK               0x7F00
#define OBEX_DEC_HANDLE(os)          ((os) & OBEX_HANDLE_MASK)
#define OBEX_DEC_SESS_IND(os)        ((os & OBEX_SESS_MASK)>>OBEX_SESS_SHIFT)

#define OBEX_TAG_SESS_PARAM_ADDR         0x00
#define OBEX_TAG_SESS_PARAM_NONCE        0x01
#define OBEX_TAG_SESS_PARAM_SESS_ID      0x02
#define OBEX_TAG_SESS_PARAM_NSEQNUM      0x03
#define OBEX_TAG_SESS_PARAM_TOUT         0x04
#define OBEX_TAG_SESS_PARAM_SESS_OP      0x05
#define OBEX_TAG_SESS_PARAM_OBJ_OFF      0x06
#define OBEX_MAX_SESS_PARAM_TRIP         7   /* max number of TLV for session operations */

#define OBEX_LEN_SESS_PARAM_SESS_OP      1
#define OBEX_LEN_SESS_PARAM_OBJ_OFF      4   /* this value varies, so it needs to be verified on the receiving side */

#define OBEX_FINAL                       0x80

/* continue to define tOBEX_SESS_ST for internal use */
enum
{
    OBEX_SESS_NONE,                       /* 0x00 session is not engaged/closed */
    OBEX_SESS_ACTIVE,                     /* 0x01 session is active. */
    OBEX_SESS_SUSPENDED,                  /* 0x02 session is suspended. */
    OBEX_SESS_TIMEOUT,                    /* 0x03 session is requested/set timeout. */
    OBEX_SESS_CREATE,                     /* 0x04 session is requested/create. */
    OBEX_SESS_SUSPEND,                    /* 0x05 session is requested/suspend. */
    OBEX_SESS_RESUME,                     /* 0x06 session is requested/resume. */
    OBEX_SESS_CLOSE,                      /* 0x07 session is requested/close. */
    OBEX_SESS_SUSPENDING                  /* 0x08 session is requested/suspend: server has not reported the suspend event */
};
#define OBEX_SESS_DROP       (0x80|OBEX_SESS_SUSPENDED)   /* the session as suspended by link drop */

typedef void (TIMER_CBACK)(void *p_tle);

typedef struct _tle
{
    wiced_timer_t       wiced_timer;
    TIMER_CBACK        *p_cback;
    INT32               ticks;
    TIMER_PARAM_TYPE    param;
    UINT16              event;
    UINT8               in_use;
} TIMER_LIST_ENT;

typedef struct
{
    void    *p_first;
    void    *p_last;
    UINT16   count;
} BUFFER_Q;

/* port control block */
typedef struct
{
    tOBEX_HANDLE    handle;             /* The handle of the client or session handle for server */
    UINT16          port_handle;        /* Port handle of connection       */
    UINT16          tx_mtu;             /* The MTU of the connected peer */
    UINT16          rx_mtu;             /* The MTU of this instance */
    TIMER_LIST_ENT  tle;                /* This session's Timer List Entry */
    tOBEX_CLOSE_FN  *p_close_fn;        /* the close connection function */
    tOBEX_SEND_FN   *p_send_fn;         /* the send message function */
    BT_HDR          *p_txmsg;           /* The message to send to peer */
    BUFFER_Q        rx_q;               /* received data buffer queue       */
    BOOLEAN         stopped;            /* TRUE, if flow control the peer (stop peer from sending more data). */
    BT_HDR          *p_rxmsg;           /* The message received from peer */
} tOBEX_PORT_CB;

typedef struct
{
    tOBEX_PORT_CB   *p_pcb;              /* the port control block */
    UINT32          code;               /* the event code from RFCOMM */
} tOBEX_PORT_EVT;

/* l2cap control block */
typedef struct
{
    tOBEX_HANDLE    handle;             /* The handle of the client or session handle for server */
    UINT16          lcid;               /* l2cap lcid of connection       */
    UINT16          tx_mtu;             /* The MTU of the connected peer */
    UINT16          rx_mtu;             /* The MTU of this instance */
    TIMER_LIST_ENT  tle;                /* This session's Timer List Entry */
    tOBEX_CLOSE_FN  *p_close_fn;        /* the close connection function */
    tOBEX_SEND_FN   *p_send_fn;         /* the send message function */
    BT_HDR          *p_txmsg;           /* The message to send to peer */
    BUFFER_Q        rx_q;               /* received data buffer queue       */
    BOOLEAN         stopped;            /* TRUE, if flow control the peer (stop peer from sending more data). */
    UINT8           ch_state;           /* L2CAP channel state */
    UINT8           ch_flags;           /* L2CAP configuration flags */
    BOOLEAN         cong;               /* TRUE, if L2CAP is congested. */
} tOBEX_L2C_CB;

enum
{
    OBEX_L2C_EVT_CONG,
    OBEX_L2C_EVT_CLOSE,
    OBEX_L2C_EVT_CONN_IND,
    OBEX_L2C_EVT_DATA_IND,
    OBEX_L2C_EVT_RESUME
};
typedef UINT8 tOBEX_L2C_EVT;

typedef struct
{
    BD_ADDR         bd_addr;
    UINT16          lcid;
    UINT16          psm;
    UINT8           id;
} tOBEX_L2C_IND;

typedef union
{
    tOBEX_L2C_IND   conn_ind;
    BOOLEAN         is_cong;
    BT_HDR          *p_pkt;
    UINT8           any;
    BD_ADDR         remote_addr;
} tOBEX_L2C_EVT_PARAM;

typedef struct
{
    tOBEX_L2C_CB         *p_l2cb;        /* the L2CAP control block */
    tOBEX_L2C_EVT_PARAM  param;
    tOBEX_L2C_EVT        l2c_evt;        /* the event code from L2CAP */
} tOBEX_L2C_EVT_MSG;

/* common of port & l2cap control block */
typedef struct
{
    tOBEX_HANDLE    handle;             /* The handle of the client or session handle for server */
    UINT16          id;                 /* l2cap lcid or port handle     */
    UINT16          tx_mtu;             /* The MTU of the connected peer */
    UINT16          rx_mtu;             /* The MTU of this instance */
    TIMER_LIST_ENT  tle;                /* This session's Timer List Entry */
    tOBEX_CLOSE_FN  *p_close_fn;        /* the close connection function */
    tOBEX_SEND_FN   *p_send_fn;         /* the send message function */
    BT_HDR          *p_txmsg;           /* The message to send to peer */
    BUFFER_Q        rx_q;               /* received data buffer queue       */
    BOOLEAN         stopped;            /* TRUE, if flow control the peer (stop peer from sending more data). */
} tOBEX_COMM_CB;

/* lower layer control block */
typedef union
{
    tOBEX_PORT_CB    port;
    tOBEX_L2C_CB     l2c;
    tOBEX_COMM_CB    comm;
} tOBEX_LL_CB;

/* client control block */
typedef struct
{
    tOBEX_CL_CBACK  *p_cback;           /* Application callback function to receive events */
    BT_HDR          *p_next_req;        /* This is used when the session is flow controlled by peer
                                         * and a DISCONNECT or ABORT request is sent.*/
    BT_HDR          *p_saved_req;       /* Client saves a copy of the request sent to the server
                                         * Just in case the operation is challenged by the server */
    tOBEX_LL_CB     ll_cb;              /* lower layer control block for this client */
    UINT32          conn_id;            /* Connection ID for this connection */
    UINT32          nonce;              /* This is converted to UINT8[16] internally before adding to the OBEX header. This value is copied to the server control block and is increased after each use. 0, if only legacy OBEX (unreliable) session is desired. */
    BD_ADDR         peer_addr;          /* peer address */
    tOBEX_EVT_PARAM param;              /* The event parameter. */
    UINT8           sess_info[OBEX_SESSION_INFO_SIZE]; /* session id + local nonce */
    tOBEX_EVENT     api_evt;            /* Set the API event, if need to notify user outside of action function. */
    tOBEX_CL_STATE  state;              /* The current state */
    tOBEX_STATE     next_state;         /* Use by PART state to return to regular states */
    tOBEX_CL_STATE  prev_state;         /* The previous state */
    UINT16          psm;                /* L2CAP virtual psm */
    tOBEX_SRM       srm;                /* Single Response Mode  */
    tOBEX_SRMP      srmp;               /* Single Response Mode Parameter */
    UINT8           ssn;                /* Session Sequence number */
    tOBEX_SESS_ST   sess_st;            /* Session state */
    UINT8           rsp_code;           /* The response code of the response packet */
    BOOLEAN         final;              /* The final bit status of last request */
} tOBEX_CL_CB;

/* suspend control block */
typedef struct
{
    TIMER_LIST_ENT  stle;               /* The timer for suspended session timeout */
    BD_ADDR         peer_addr;          /* peer address */
    UINT8           sess_info[OBEX_SESSION_INFO_SIZE]; /* session id + local nonce + conn id + state + srm */
    UINT32          offset;             /* the file offset */
    tOBEX_SR_STATE  state;              /* The current state */
    UINT8           ssn;                /* the last ssn */
} tOBEX_SPND_CB;

/* server control block */
typedef struct
{
    tOBEX_SR_CBACK  *p_cback;           /* Application callback function to receive events */
    tOBEX_TARGET    target;             /* target header of this server */
    UINT8           sess[OBEX_MAX_SR_SESSION]; /* index + 1 of sr_sess[]. 0, if not used. */
    tOBEX_SPND_CB   *p_suspend;         /* the information for suspended sessions (GKI buffer) */
    UINT32          nonce;              /* This is converted to UINT8[16] internally before adding to the OBEX header. This value is copied to the server control block and is increased after each use. 0, if only legacy OBEX (unreliable) session is desired. */
    UINT16          psm;                /* PSM for this server         */
    UINT8           max_suspend;        /* the max number of tOBEX_SPD_CB[] in p_suspend. must be less than OBEX_MAX_SUSPEND_SESSIONS */
    UINT8           scn;                /* SCN for this server         */
    UINT8           num_sess;           /* Max number of session control blocks used by this server*/
} tOBEX_SR_CB;

/* server session control block */
typedef struct
{
    BT_HDR          *p_saved_msg;       /* This is a message saved for GetReq-non-final */
    BT_HDR          *p_next_req;        /* This is a message saved for flow control reasons */
    tOBEX_LL_CB     ll_cb;              /* lower layer control block for this session */
    UINT32          conn_id;            /* Connection ID for this connection */
    BD_ADDR         peer_addr;          /* peer address */
    UINT8           sess_info[OBEX_SESSION_INFO_SIZE]; /* session id + local nonce */
    tOBEX_SR_STATE  state;              /* The current state */
    tOBEX_SR_STATE  prev_state;         /* The previous state */
    tOBEX_STATE     next_state;         /* Use by PART state to return to regular states */
    tOBEX_HANDLE    handle;             /* The obx handle for server */
    tOBEX_EVENT     api_evt;            /* The event to notify the API user. */
    tOBEX_EVT_PARAM param;              /* The event parameter. */
    tOBEX_SRMP      srmp;               /* Single Response Mode Parameter */
    tOBEX_SRM       srm;                /* Single Response Mode  */
    tOBEX_SESS_ST   sess_st;            /* Session state */
    UINT8           ssn;                /* Next Session Sequence number */
    UINT8           cur_op;             /* The op code for the current transaction (keep this for Abort reasons) */
                                        /* set to the current OP (non-abort) in rfc.c, set it to abort when a response is sent */
} tOBEX_SR_SESS_CB;

typedef struct
{
    tOBEX_SR_CB     server[OBEX_NUM_SERVERS];/* The server control blocks */
    UINT32          next_cid;               /* Next OBEX connection ID for server */
    tOBEX_SR_SESS_CB sr_sess[OBEX_NUM_SR_SESSIONS];
    tOBEX_L2C_CB     sr_l2cb;                /* for obx_l2c_connect_ind_cback */

    tOBEX_CL_CB     client[OBEX_NUM_CLIENTS];/* The client control blocks */
    tOBEX_PORT_CB   *p_temp_pcb;         /* Valid only during client RFCOMM_CreateConnection call */
    UINT8           next_ind;           /* The index to the next client control block */

    tOBEX_HANDLE    hdl_map[MAX_RFC_PORTS]; /* index of this array is the port_handle,
                                         * the value is the OBX handle */
    UINT16          l2c_map[MAX_L2CAP_CHANNELS]; /* index of this array is (lcid - L2CAP_BASE_APPL_CID) */
    UINT32          timeout_val;        /* The timeout value to wait for activity from peer */
    UINT32          sess_tout_val;      /* The timeout value for reliable sessions to remain in suspend */
    UINT8           num_client;         /* Number of client control blocks */
    UINT8           num_server;         /* Number of server control blocks */
    UINT8           num_sr_sess;        /* Number of server session control blocks */
    UINT8           max_rx_qcount;      /* Max Number of rx_q count */
} tOBEX_CB;

/*****************************************************************************
**  Definition for State Machine
*****************************************************************************/

/* Client Action functions are of this type */
typedef tOBEX_CL_STATE (*tOBEX_CL_ACT)(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);

/* Server Action functions are of this type */
typedef tOBEX_SR_STATE (*tOBEX_SR_ACT)(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);

#define OBEX_SM_IGNORE           0
#define OBEX_SM_NO_ACTION        0xFF
#define OBEX_SM_ALL              0x80 /* for events like close confirm, abort request, close request where event handling is the same for most of the states */
#define OBEX_SM_ENTRY_MASK       0x7F /* state machine entry mask */

#define OBEX_SME_ACTION          0
#define OBEX_SME_NEXT_STATE      1
#define OBEX_SM_NUM_COLS         2

typedef const UINT8 (*tOBEX_SM_TBL)[OBEX_SM_NUM_COLS];

/*****************************************************************************
**  External global data
*****************************************************************************/
#if OBEX_DYNAMIC_MEMORY == FALSE
extern tOBEX_CB  obx_cb;
#else
OBEX_API extern tOBEX_CB *obx_cb_ptr;
#define obx_cb (*obx_cb_ptr)
#endif
extern const tOBEX_EVENT obx_sm_evt_to_api_evt[];
extern const UINT8 obx_hdr_start_offset[];
extern const BD_ADDR PB_BT_BD_ANY;
#if BT_TRACE_PROTOCOL == TRUE
extern const UINT8 obx_api_evt_to_disp_type[];
#endif
/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
/* from obx_main.c */

#if (defined (BT_USE_TRACES) && BT_USE_TRACES == TRUE)
extern const char * obx_cl_get_state_name(tOBEX_CL_STATE state);
extern const char * obx_cl_get_event_name(tOBEX_CL_EVENT event);
#else
#define obx_cl_get_state_name(state_num)    ""
#define obx_cl_get_event_name(event_num)    ""
#endif

#if (defined (BT_USE_TRACES) && BT_USE_TRACES == TRUE)
extern const char * obx_sr_get_state_name(tOBEX_SR_STATE state);
extern const char * obx_sr_get_event_name(tOBEX_SR_EVENT event);
#else
#define obx_sr_get_state_name(state_num)    ""
#define obx_sr_get_event_name(event_num)    ""
#endif
/* client functions in obx_main.c */
extern void obx_cl_timeout(uint32_t cb_params);
extern tOBEX_CL_CB * obx_cl_alloc_cb(void);
extern tOBEX_CL_CB * obx_cl_get_cb(tOBEX_HANDLE handle);
extern tOBEX_CL_CB * obx_cl_get_suspended_cb(tOBEX_HANDLE *p_handle, UINT8 *p_session_info);
extern void obx_cl_free_cb(tOBEX_CL_CB * p_cb);

/* server functions in obx_main.c */
extern tOBEX_SPND_CB * obx_find_suspended_session (tOBEX_SR_SESS_CB *p_scb, tOBEX_TRIPLET *p_triplet, UINT8 num);
extern void obx_sr_timeout(uint32_t cb_params);
extern void obx_sr_sess_timeout(uint32_t cb_params);
extern tOBEX_HANDLE obx_sr_alloc_cb(tOBEX_StartParams *p_params);
extern tOBEX_SR_CB * obx_sr_get_cb(tOBEX_HANDLE handle);
extern tOBEX_SR_SESS_CB * obx_sr_get_scb(tOBEX_HANDLE handle);
extern void obx_sr_free_cb(tOBEX_HANDLE handle);
extern void obx_sr_free_scb(tOBEX_SR_SESS_CB *p_scb);
extern UINT32 obx_sr_get_next_conn_id(void);
/* common functions in obx_main.c */
extern tOBEX_PORT_CB * obx_port_handle_2cb(UINT16 port_handle);
extern void obx_start_timer(tOBEX_COMM_CB *p_pcb);
extern void obx_stop_timer(TIMER_LIST_ENT *p_tle);

/* from obx_rfc.c */
extern void obx_cl_proc_evt(tOBEX_PORT_EVT *p_evt);
extern BT_HDR * obx_build_dummy_rsp(tOBEX_SR_SESS_CB *p_scb, UINT8 rsp_code);
extern void obx_sr_proc_evt(tOBEX_PORT_EVT *p_evt);
extern BOOLEAN obx_rfc_snd_msg(tOBEX_PORT_CB *p_pcb);
extern tOBEX_STATUS obx_open_port(tOBEX_PORT_CB *p_pcb, const BD_ADDR bd_addr, UINT8 scn);
extern void obx_add_port(tOBEX_HANDLE obx_handle);
extern void obx_close_port(UINT16 port_handle);

/* from obx_capi.c */
extern tOBEX_STATUS obx_prepend_req_msg(tOBEX_HANDLE handle, tOBEX_CL_EVENT event, UINT8 req_code, BT_HDR *p_pkt);

/* from obx_csm.c */
extern void obx_csm_event(tOBEX_CL_CB *p_cb, tOBEX_EVENT event, BT_HDR *p_msg);

/* from obx_cact.c */
extern void obx_ca_connect_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_snd_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_close_port(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_snd_part(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_connect_error(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_connect_fail(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_discnt_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_notify(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_fail_rsp(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_state(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_start_timer(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_connect_ok(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_session_ok(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_session_cont(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_session_get(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_session_fail(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_abort(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_snd_put_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_snd_get_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_srm_snd_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_srm_put_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_srm_get_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_srm_put_notify(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_srm_get_notify(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_save_rsp(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern tOBEX_CL_STATE obx_ca_save_req(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);

/* from obx_ssm.c */
extern void obx_ssm_event(tOBEX_SR_SESS_CB *p_scb, tOBEX_SR_EVENT event, BT_HDR *p_msg);

/* from obx_sapi.c */
extern tOBEX_STATUS obx_prepend_rsp_msg(tOBEX_HANDLE handle, tOBEX_SR_EVENT event, UINT8 rsp_code, BT_HDR *p_pkt);

/* from obx_sact.c */
extern tOBEX_SR_STATE obx_sa_snd_rsp(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_snd_part(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_abort_rsp(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_op_rsp(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern BT_HDR * obx_conn_rsp(tOBEX_SR_CB *p_cb, tOBEX_SR_SESS_CB *p_scb, UINT8 rsp_code, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_connect_ind(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_wc_conn_ind(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_connect_rsp(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_connection_error(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_close_port(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_clean_port(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_state(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_nc_to(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_save_req(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_rej_req(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_session_ind(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_sess_conn_ind(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_wc_sess_ind(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_session_rsp(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_put_ind(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_get_ind(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_get_req(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_srm_put_req(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_srm_put_rsp(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_srm_get_fcs(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_srm_get_rsp(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern tOBEX_SR_STATE obx_sa_srm_get_req(tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);

/* from obx_gen.c */
extern void obx_access_rsp_code(BT_HDR *p_pkt, UINT8 *p_rsp_code);
extern void obx_adjust_packet_len(BT_HDR *p_pkt);
extern UINT16 obx_read_header_len(UINT8 *ph);
extern BT_HDR * obx_dup_pkt(BT_HDR *p_pkt);

/* from obx_md5 */
extern void obx_session_id(UINT8 *p_sess_id, UINT8 *p_cl_addr, UINT8 * p_cl_nonce, int cl_nonce_len,
                    UINT8 *p_sr_addr, UINT8 * p_sr_nonce, int sr_nonce_len);

/* from obx_l2c.c */
extern void obx_register_l2c(tOBEX_CL_CB *p_cl_cb, UINT16 psm);
extern tOBEX_STATUS obx_open_l2c(tOBEX_CL_CB *p_cl_cb, const BD_ADDR bd_addr);
extern tOBEX_STATUS obx_l2c_sr_register (tOBEX_SR_CB  *p_cb);
extern void obx_close_l2c(UINT16 lcid);
extern BOOLEAN obx_l2c_snd_msg(tOBEX_L2C_CB *p_l2cb);
extern void obx_l2c_snd_evt (tOBEX_L2C_CB *p_l2cb, tOBEX_L2C_EVT_PARAM  param, tOBEX_L2C_EVT l2c_evt);
extern void obx_sr_proc_l2c_evt (tOBEX_L2C_EVT_MSG *p_msg);
extern void obx_cl_proc_l2c_evt (tOBEX_L2C_EVT_MSG *p_msg);

/* from obx_utils.c */
extern UINT8 obx_verify_response(UINT8 opcode, tOBEX_RX_HDR *p_rxh);
extern UINT8 obx_verify_request(UINT8 opcode, tOBEX_RX_HDR *p_rxh);
extern BOOLEAN obx_sr_proc_pkt (tOBEX_SR_SESS_CB *p_scb, BT_HDR *p_pkt);
extern void obx_cl_proc_pkt (tOBEX_CL_CB *p_cb, BT_HDR *p_pkt);
extern void obx_free_buf(tOBEX_LL_CB *p_ll_cb);
extern UINT8 obx_add_timeout (tOBEX_TRIPLET *p_trip, UINT32 timeout, tOBEX_SESS_EVT *p_param);
extern void obx_read_timeout (tOBEX_TRIPLET *p_trip, UINT8 num, UINT32 *p_timeout, UINT8 *p_toa);
#if (BT_USE_TRACES == TRUE)
extern void obxu_dump_hex (UINT8 *p, char *p_title, UINT16 len);
#else
#define obxu_dump_hex(p, p_title, len)
#endif
extern BT_HDR * obx_sr_prepend_msg(BT_HDR *p_pkt, UINT8 * p_data, UINT16 data_len);
extern BT_HDR * obx_cl_prepend_msg(tOBEX_CL_CB *p_cb, BT_HDR *p_pkt, UINT8 * p_data, UINT16 data_len);
extern void obx_flow_control(tOBEX_COMM_CB *p_comm);
extern UINT8 obx_read_triplet(tOBEX_TRIPLET *p_trip, UINT8 num_trip, UINT8 tag);
extern UINT32 obx_read_obj_offset(tOBEX_TRIPLET *p_trip, UINT8 num_trip);

/* from obx_api.h */
extern BOOLEAN OBEX_Add1ByteHdr(BT_HDR *p_pkt, UINT8 id, UINT8 data);
extern BOOLEAN OBEX_Add4ByteHdr(BT_HDR *p_pkt, UINT8 id, UINT32 data);
extern BOOLEAN OBEX_AddByteStrHdr(BT_HDR *p_pkt, UINT8 id, UINT8 *p_data, UINT16 len);
extern BOOLEAN OBEX_AddUnicodeHdr(BT_HDR *p_pkt, UINT8 id, UINT16 *p_data, UINT16 len);
extern BOOLEAN OBEX_AddTriplet(BT_HDR *p_pkt, UINT8 id, tOBEX_TRIPLET *p_triplet, UINT8 num);
extern UINT8 * OBEX_CheckHdr(BT_HDR *p_pkt, UINT8 id);
extern BOOLEAN OBEX_Read1ByteHdr(BT_HDR *p_pkt, UINT8 id, UINT8 *p_data);
extern BOOLEAN OBEX_Read4ByteHdr(BT_HDR *p_pkt, UINT8 id, UINT32 *p_data);
extern BOOLEAN OBEX_ReadByteStrHdr(BT_HDR *p_pkt, UINT8 id, UINT8 **p_data, UINT16 *p_len, UINT8 next);
extern BOOLEAN OBEX_ReadUnicodeHdr(BT_HDR *p_pkt, UINT8 id, UINT16 *p_data, UINT16 *p_len);
extern BOOLEAN OBEX_ReadTriplet(BT_HDR *p_pkt, UINT8 id, tOBEX_TRIPLET *p_triplet, UINT8 *p_num);
extern UINT8 OBEX_ReadBodyHdr(BT_HDR *p_pkt, UINT8 **p_body, UINT16 *p_len, BOOLEAN *p_end);
extern UINT16 OBEX_CharToWchar (UINT16 *w_str, char* a_str, UINT16 w_size);
extern void OBEX_WcharToChar (char *a_str, UINT16* w_str, UINT16 a_size) ;
extern UINT16 utfc_16_to_8(UINT8 *p_utf8, UINT16 utf8_len, UINT16 *p_utf16, UINT16 utf16_len);
extern UINT16 utfc_8_to_16(UINT16 *p_utf16, UINT16 utf16_len, UINT8 *p_utf8);
extern UINT8 *OBEX_AddBodyStart(BT_HDR *p_pkt, UINT16 *p_len);
extern void OBEX_AddBodyEnd(BT_HDR *p_pkt, UINT8 *p, UINT16 len, BOOLEAN end);
extern UINT16 OBEX_ReadHdrLen(BT_HDR *p_pkt, UINT8 id);

/* from obx_hdrs.c file */
extern UINT8 OBEX_ReadNumHdrs(BT_HDR *p_pkt, UINT8 *p_num_body);
extern UINT16 OBEX_HandleToMtu(tOBEX_HANDLE handle);

/* Other Necessary External Definitions */
extern int PORT_Write (uint16_t handle, BT_HDR *p_buf);
extern int PORT_Read (uint16_t handle, BT_HDR **pp_buf);
extern int PORT_ReadData (UINT16 handle, char *p_data, UINT16 max_len, UINT16 *p_len);
extern BOOLEAN L2CA_FlowControl (UINT16 cid, BOOLEAN data_enabled);
extern void *GKI_getpoolbuf (uint8_t pool_id);
extern void *GKI_getbuf (uint16_t);
extern void GKI_freebuf (void *memPtr);
extern uint16_t GKI_get_pool_bufsize (UINT8 pool_id);
extern uint16_t GKI_get_buf_size (void *p_buf);
extern uint16_t GKI_poolcount (uint8_t pool_id);
extern uint16_t GKI_poolfreecount (uint16_t pool_id);
extern void GKI_enqueue (BUFFER_Q *p_q, void *p_buf);
extern void GKI_enqueue_head (BUFFER_Q *p_q, void *p_buf);
extern BOOLEAN GKI_queue_is_empty(BUFFER_Q *p_q);
extern void *GKI_dequeue (BUFFER_Q *p_q);
extern void btu_stop_timer (TIMER_LIST_ENT *p_tle);

#endif /* OBX_INT_H */
