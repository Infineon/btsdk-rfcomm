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
 * WICED BT OPP server APIs
 *
 */

#ifndef WICED_BT_OP_API_H
#define WICED_BT_OP_API_H

#include "wiced_bt_types.h"
#include "wiced_result.h"
#include "bt_types.h"

/*****************************************************************************
**  Constants and data types
*****************************************************************************/
/* Extra Debug Code */
#ifndef WICED_BT_OPS_DEBUG
#define WICED_BT_OPS_DEBUG           FALSE
#endif

#ifndef WICED_BT_OPC_DEBUG
#define WICED_BT_OPC_DEBUG           FALSE
#endif


/* Object format */
#define WICED_BT_OP_VCARD21_FMT          1       /* vCard 2.1 */
#define WICED_BT_OP_VCARD30_FMT          2       /* vCard 3.0 */
#define WICED_BT_OP_VCAL_FMT             3       /* vCal 1.0 */
#define WICED_BT_OP_ICAL_FMT             4       /* iCal 2.0 */
#define WICED_BT_OP_VNOTE_FMT            5       /* vNote */
#define WICED_BT_OP_VMSG_FMT             6       /* vMessage */
#define WICED_BT_OP_OTHER_FMT            0xFF    /* other format */

typedef UINT8 wiced_bt_op_fmt_t;

/* Object format mask */
#define WICED_BT_OP_VCARD21_MASK         0x01    /* vCard 2.1 */
#define WICED_BT_OP_VCARD30_MASK         0x02    /* vCard 3.0 */
#define WICED_BT_OP_VCAL_MASK            0x04    /* vCal 1.0 */
#define WICED_BT_OP_ICAL_MASK            0x08    /* iCal 2.0 */
#define WICED_BT_OP_VNOTE_MASK           0x10    /* vNote */
#define WICED_BT_OP_VMSG_MASK            0x20    /* vMessage */
#define WICED_BT_OP_ANY_MASK             0x40    /* Any type of object. */

typedef UINT8 wiced_bt_op_fmt_mask_t;

/* Status */
#define WICED_BT_OP_OK                   0       /* Operation successful. */
#define WICED_BT_OP_FAIL                 1       /* Operation failed. */
#define WICED_BT_OP_MEM                  2       /* Not enough memory to complete operation. */

typedef UINT8 wiced_bt_op_status_t;

/* This structure describes an object property, or individual item, inside an object. */
typedef struct
{
    UINT8       *p_data;            /* Pointer to property data. */
    UINT32      parameters;         /* Property parameters. */
    UINT16      name;               /* Property name. */
    UINT16      len;                /* Length of data. */
    UINT8       *p_param;           /* Pointer to the Parameters */
    UINT16      param_len;          /* Param Len */
} wiced_bt_op_prop_t;


/* Access response types */
#define WICED_BT_OP_ACCESS_ALLOW     0   /* Allow the requested operation */
#define WICED_BT_OP_ACCESS_FORBID    1   /* Disallow the requested operation */
#define WICED_BT_OP_ACCESS_NONSUP    2   /* Requested operation is not supported */

typedef UINT8 wiced_bt_op_access_t;

/* Access event operation types */
#define WICED_BT_OP_OPER_PUSH        1
#define WICED_BT_OP_OPER_PULL        2

typedef UINT8 wiced_bt_op_oper_t;


/* Client callback function event */
#define WICED_BT_OPC_ENABLE_EVT          0   /* Object push client is enabled. */
#define WICED_BT_OPC_OPEN_EVT            1   /* Connection to peer is open. */
#define WICED_BT_OPC_PROGRESS_EVT        2   /* push/pull in progres */
#define WICED_BT_OPC_OBJECT_EVT          3   /* Object Pulled */
#define WICED_BT_OPC_OBJECT_PSHD_EVT     4   /* Object pushed */
#define WICED_BT_OPC_CLOSE_EVT           5   /* Connection to peer closed. */

typedef UINT8 wiced_bt_opc_evt_t;

/* Client callback function result */
#define WICED_BT_OPC_OK             0   /* Object push succeeded. */
#define WICED_BT_OPC_FAIL           1   /* Object push failed. */
#define WICED_BT_OPC_NOT_FOUND      2   /* Object not found. */
#define WICED_BT_OPC_NO_PERMISSION  3   /* Operation not authorized. */
#define WICED_BT_OPC_SRV_UNAVAIL    4   /* Service unavaliable */
#define WICED_BT_OPC_RSP_FORBIDDEN  5   /* Operation forbidden */
#define WICED_BT_OPC_RSP_NOT_ACCEPTABLE  6  /* Operation not acceptable */

typedef UINT8 wiced_bt_opc_status_t;

/* Structure associated with WICED_BT_OPC_OBJECT_EVT */
typedef struct
{
    char                *p_name;        /* Object name. */
    wiced_bt_opc_status_t    status;
} wiced_bt_opc_object_t;

typedef struct
{
    UINT32          obj_size;   /* Total size of object (WICED_BT_OPS_LEN_UNKNOWN if unknown) */
    UINT16          bytes;      /* Number of bytes read or written since last progress event */
    wiced_bt_op_oper_t    operation;  /* Is progress for Push or Pull */
} wiced_bt_opc_progress_t;

/* Union of all client callback structures */
typedef union
{
    wiced_bt_opc_object_t   object;
    wiced_bt_opc_progress_t prog;
    wiced_bt_opc_status_t   status;
} wiced_bt_opc_t;

/* Client callback function */
typedef void (wiced_bt_opc_cback_t)(wiced_bt_opc_evt_t event, wiced_bt_opc_t *p_data);

/* Server callback function event */
#define WICED_BT_OPS_ENABLE_EVT          0   /* Object push server is enabled. */
#define WICED_BT_OPS_OPEN_EVT            1   /* Connection to peer is open. */
#define WICED_BT_OPS_PROGRESS_EVT        2   /* Object being sent or received. */
#define WICED_BT_OPS_OBJECT_EVT          3   /* Object has been received. */
#define WICED_BT_OPS_CLOSE_EVT           4   /* Connection to peer closed. */
#define WICED_BT_OPS_ACCESS_EVT          5   /* Request for access to push or pull object */
#define WICED_BT_OPS_DISABLE_EVT         6   /* Object push server is disabled.*/

typedef UINT8 wiced_bt_ops_evt_t;

/* Structure associated with WICED_BT_OPS_OBJECT_EVT */
typedef struct
{
    char                *p_name;        /* Object name. */
    wiced_bt_op_fmt_t         format;         /* Object format. */
} wiced_bt_ops_object_t;

typedef struct
{
    UINT32              obj_size;       /* Total size of object (WICED_BT_OPS_LEN_UNKNOWN if unknown) */
    UINT16              bytes;          /* Number of bytes read or written since last progress event */
    wiced_bt_op_oper_t        operation;      /* Is progress for Push or Pull */
} wiced_bt_ops_progress_t;

typedef struct
{
    char                *p_name;        /* Object name */
    char                *p_type;        /* Object type (NULL if not specified) */
    UINT32              size;           /* Object size */
    BD_ADDR             bd_addr;        /* Address of device */
    wiced_bt_op_oper_t        oper;           /* Operation (push or pull) */
    wiced_bt_op_fmt_t         format;         /* Object format */
} wiced_bt_ops_access_t;

/* Union of all server callback structures */
typedef union
{
    wiced_bt_ops_object_t     object;
    wiced_bt_ops_progress_t   prog;
    wiced_bt_ops_access_t     access;
    BD_ADDR             bd_addr;
} wiced_bt_ops_t;

/* Server event callback function */
typedef void (wiced_bt_ops_cback_t)(wiced_bt_ops_evt_t event, wiced_bt_ops_t *p_data);

/* Server data callback function */
typedef void (wiced_bt_ops_data_cback_t)(const UINT8 *p_buf, UINT16 nbytes);


/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

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
extern void wiced_bt_ops_op_enable(UINT8 scn, wiced_bt_op_fmt_mask_t formats,
                                  wiced_bt_ops_cback_t *p_cback,
                                  wiced_bt_ops_data_cback_t *p_data_cback);

/*******************************************************************************
**
** Function         wiced_bt_pbc_op_disable
**
** Description      Disable the object push server.  If the server is currently
**                  connected to a peer device the connection will be closed.
**
** Returns          void
**
*******************************************************************************/
extern void wiced_bt_ops_op_disable(void);

/*******************************************************************************
**
** Function         wiced_bt_pbc_op_close
**
** Description      Close the current connection.  This function is called if
**                  the phone wishes to close the connection before the object
**                  push is completed.
**
**
** Returns          void
**
*******************************************************************************/
extern void wiced_bt_ops_op_close(void);

/*******************************************************************************
**
** Function         wiced_bt_ops_op_access_rsp
**
** Description      Sends a reply to an access request event (WICED_BT_OPS_ACCESS_EVT).
**                  This call MUST be made whenever the event occurs.
**
** Returns          void
**
*******************************************************************************/
extern void wiced_bt_ops_op_access_rsp(wiced_bt_op_oper_t oper, wiced_bt_op_access_t access);

#ifdef __cplusplus
}
#endif

#endif /* WICED_BT_OP_API_H */
