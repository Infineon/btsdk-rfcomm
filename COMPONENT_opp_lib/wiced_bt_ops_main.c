/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

/** @file
 *
 * WICED BT OPP server State Machine
 *
 */

#include <string.h>
#include "wiced_bt_ops_int.h"
#include "wiced_bt_obex.h"
#include "wiced_bt_rfcomm.h"

/*****************************************************************************
** Constants and types
*****************************************************************************/

/* state machine states */
enum
{
    WICED_BT_OPS_IDLE_ST = 0,      /* Idle  */
    WICED_BT_OPS_LISTEN_ST,        /* Listen - waiting for OBX/RFC connection */
    WICED_BT_OPS_CONN_ST,          /* Connected - OPP Session is active */
    WICED_BT_OPS_CLOSING_ST        /* Closing is in progress */
};

/* state machine action enumeration list */
enum
{
    WICED_BT_OPS_API_DISABLE,
    WICED_BT_OPS_API_ACCESSRSP,
    WICED_BT_OPS_API_CLOSE,
    WICED_BT_OPS_CI_WRITE,
    WICED_BT_OPS_CI_READ,
    WICED_BT_OPS_CI_OPEN,
    WICED_BT_OPS_OBX_CONNECT,
    WICED_BT_OPS_OBX_DISC,
    WICED_BT_OPS_OBX_CLOSE,
    WICED_BT_OPS_OBX_ABORT,
    WICED_BT_OPS_OBX_PUT,
    WICED_BT_OPS_OBX_GET,
    WICED_BT_OPS_OBX_ACTION,
    WICED_BT_OPS_CLOSE_COMPLETE,
    WICED_BT_OPS_IGNORE
};

/* type for action functions */
typedef void (*wiced_bt_ops_action_t)(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data);

/* action function list */
const wiced_bt_ops_action_t wiced_bt_ops_action[] =
{
    wiced_bt_ops_api_disable,
    wiced_bt_ops_api_accessrsp,
    wiced_bt_ops_api_close,
    wiced_bt_ops_ci_write,
    wiced_bt_ops_ci_read,
    wiced_bt_ops_ci_open,
    wiced_bt_ops_obx_connect,
    wiced_bt_ops_obx_disc,
    wiced_bt_ops_obx_close,
    wiced_bt_ops_obx_abort,
    wiced_bt_ops_obx_put,
    wiced_bt_ops_obx_get,
    wiced_bt_ops_obx_action,
    wiced_bt_ops_close_complete
};


/* state table information */
#define WICED_BT_OPS_ACTIONS             1       /* number of actions */
#define WICED_BT_OPS_NEXT_STATE          1       /* position of next state */
#define WICED_BT_OPS_NUM_COLS            2       /* number of columns in state tables */

/* state table for idle state */
static const UINT8 wiced_bt_ops_st_idle[][WICED_BT_OPS_NUM_COLS] =
{
/* Event                          Action 1               Next state */
/* WICED_BT_OPS_API_DISABLE_EVT   */   {WICED_BT_OPS_IGNORE,       WICED_BT_OPS_IDLE_ST},
/* WICED_BT_OPS_API_ACCESSRSP_EVT */   {WICED_BT_OPS_IGNORE,       WICED_BT_OPS_IDLE_ST},
/* WICED_BT_OPS_API_CLOSE_EVT     */   {WICED_BT_OPS_IGNORE,       WICED_BT_OPS_IDLE_ST},
};

/* state table for obex/rfcomm connection state */
static const UINT8 wiced_bt_ops_st_listen[][WICED_BT_OPS_NUM_COLS] =
{
/* Event                          Action 1               Next state */
/* WICED_BT_OPS_API_DISABLE_EVT   */   {WICED_BT_OPS_API_DISABLE,  WICED_BT_OPS_IDLE_ST},
/* WICED_BT_OPS_API_ACCESSRSP_EVT */   {WICED_BT_OPS_IGNORE,       WICED_BT_OPS_LISTEN_ST},
/* WICED_BT_OPS_API_CLOSE_EVT     */   {WICED_BT_OPS_IGNORE,       WICED_BT_OPS_LISTEN_ST},
/* WICED_BT_OPS_CI_OPEN_EVT       */   {WICED_BT_OPS_IGNORE,       WICED_BT_OPS_LISTEN_ST},
/* WICED_BT_OPS_CI_WRITE_EVT      */   {WICED_BT_OPS_IGNORE,       WICED_BT_OPS_LISTEN_ST},
/* WICED_BT_OPS_CI_READ_EVT       */   {WICED_BT_OPS_IGNORE,       WICED_BT_OPS_LISTEN_ST},
/* WICED_BT_OPS_OBX_CONN_EVT      */   {WICED_BT_OPS_OBX_CONNECT,  WICED_BT_OPS_CONN_ST},
/* WICED_BT_OPS_OBX_DISC_EVT      */   {WICED_BT_OPS_IGNORE,       WICED_BT_OPS_LISTEN_ST},
/* WICED_BT_OPS_OBX_ABORT_EVT     */   {WICED_BT_OPS_IGNORE,       WICED_BT_OPS_LISTEN_ST},
/* WICED_BT_OPS_OBX_CLOSE_EVT     */   {WICED_BT_OPS_IGNORE,       WICED_BT_OPS_LISTEN_ST},
/* WICED_BT_OPS_OBX_PUT_EVT       */   {WICED_BT_OPS_IGNORE,       WICED_BT_OPS_LISTEN_ST},
/* WICED_BT_OPS_OBX_GET_EVT       */   {WICED_BT_OPS_IGNORE,       WICED_BT_OPS_LISTEN_ST},
/* WICED_BT_OPS_OBX_ACTION_EVT    */   {WICED_BT_OPS_IGNORE,       WICED_BT_OPS_LISTEN_ST},
/* WICED_BT_OPS_CLOSE_CMPL_EVT    */   {WICED_BT_OPS_IGNORE,       WICED_BT_OPS_LISTEN_ST}
};

/* state table for open state */
static const UINT8 wiced_bt_ops_st_connected[][WICED_BT_OPS_NUM_COLS] =
{
/* Event                          Action 1                  Next state */
/* WICED_BT_OPS_API_DISABLE_EVT   */   {WICED_BT_OPS_API_DISABLE,     WICED_BT_OPS_IDLE_ST},
/* WICED_BT_OPS_API_ACCESSRSP_EVT */   {WICED_BT_OPS_API_ACCESSRSP,   WICED_BT_OPS_CONN_ST},
/* WICED_BT_OPS_API_CLOSE_EVT     */   {WICED_BT_OPS_API_CLOSE,       WICED_BT_OPS_CLOSING_ST},
/* WICED_BT_OPS_CI_OPEN_EVT       */   {WICED_BT_OPS_CI_OPEN,         WICED_BT_OPS_CONN_ST},
/* WICED_BT_OPS_CI_WRITE_EVT      */   {WICED_BT_OPS_CI_WRITE,        WICED_BT_OPS_CONN_ST},
/* WICED_BT_OPS_CI_READ_EVT       */   {WICED_BT_OPS_CI_READ,         WICED_BT_OPS_CONN_ST},
/* WICED_BT_OPS_OBX_CONN_EVT      */   {WICED_BT_OPS_IGNORE,          WICED_BT_OPS_CONN_ST},
/* WICED_BT_OPS_OBX_DISC_EVT      */   {WICED_BT_OPS_OBX_DISC,        WICED_BT_OPS_CLOSING_ST},
/* WICED_BT_OPS_OBX_ABORT_EVT     */   {WICED_BT_OPS_OBX_ABORT,       WICED_BT_OPS_CONN_ST},
/* WICED_BT_OPS_OBX_CLOSE_EVT     */   {WICED_BT_OPS_OBX_CLOSE,       WICED_BT_OPS_CLOSING_ST},
/* WICED_BT_OPS_OBX_PUT_EVT       */   {WICED_BT_OPS_OBX_PUT,         WICED_BT_OPS_CONN_ST},
/* WICED_BT_OPS_OBX_GET_EVT       */   {WICED_BT_OPS_OBX_GET,         WICED_BT_OPS_CONN_ST},
/* WICED_BT_OPS_OBX_ACTION_EVT    */   {WICED_BT_OPS_OBX_ACTION,      WICED_BT_OPS_CONN_ST},
/* WICED_BT_OPS_CLOSE_CMPL_EVT    */   {WICED_BT_OPS_IGNORE,          WICED_BT_OPS_CONN_ST}
};

/* state table for closing state */
static const UINT8 wiced_bt_ops_st_closing[][WICED_BT_OPS_NUM_COLS] =
{
/* Event                          Action 1                  Next state */
/* WICED_BT_OPS_API_DISABLE_EVT   */   {WICED_BT_OPS_API_DISABLE,     WICED_BT_OPS_IDLE_ST},
/* WICED_BT_OPS_API_ACCESSRSP_EVT */   {WICED_BT_OPS_IGNORE,          WICED_BT_OPS_CLOSING_ST},
/* WICED_BT_OPS_API_CLOSE_EVT     */   {WICED_BT_OPS_IGNORE,          WICED_BT_OPS_CLOSING_ST},
/* WICED_BT_OPS_CI_OPEN_EVT       */   {WICED_BT_OPS_CLOSE_COMPLETE,  WICED_BT_OPS_LISTEN_ST},
/* WICED_BT_OPS_CI_WRITE_EVT      */   {WICED_BT_OPS_CLOSE_COMPLETE,  WICED_BT_OPS_LISTEN_ST},
/* WICED_BT_OPS_CI_READ_EVT       */   {WICED_BT_OPS_CLOSE_COMPLETE,  WICED_BT_OPS_LISTEN_ST},
/* WICED_BT_OPS_OBX_CONN_EVT      */   {WICED_BT_OPS_IGNORE,          WICED_BT_OPS_CLOSING_ST},
/* WICED_BT_OPS_OBX_DISC_EVT      */   {WICED_BT_OPS_IGNORE,          WICED_BT_OPS_CLOSING_ST},
/* WICED_BT_OPS_OBX_ABORT_EVT     */   {WICED_BT_OPS_OBX_ABORT,       WICED_BT_OPS_CLOSING_ST},
/* WICED_BT_OPS_OBX_CLOSE_EVT     */   {WICED_BT_OPS_OBX_CLOSE,       WICED_BT_OPS_CLOSING_ST},
/* WICED_BT_OPS_OBX_PUT_EVT       */   {WICED_BT_OPS_IGNORE,          WICED_BT_OPS_CLOSING_ST},
/* WICED_BT_OPS_OBX_GET_EVT       */   {WICED_BT_OPS_IGNORE,          WICED_BT_OPS_CLOSING_ST},
/* WICED_BT_OPS_OBX_ACTION_EVT    */   {WICED_BT_OPS_IGNORE,          WICED_BT_OPS_CLOSING_ST},
/* WICED_BT_OPS_CLOSE_CMPL_EVT    */   {WICED_BT_OPS_CLOSE_COMPLETE,  WICED_BT_OPS_LISTEN_ST}
};

/* type for state table */
typedef const UINT8 (*tWICED_BT_OPS_ST_TBL)[WICED_BT_OPS_NUM_COLS];

/* state table */
const tWICED_BT_OPS_ST_TBL wiced_bt_ops_st_tbl[] =
{
    wiced_bt_ops_st_idle,
    wiced_bt_ops_st_listen,
    wiced_bt_ops_st_connected,
    wiced_bt_ops_st_closing
};

/*****************************************************************************
** Global data
*****************************************************************************/

/* OPS control block */
#if WICED_BT_DYNAMIC_MEMORY == FALSE
wiced_bt_ops_cb_t  wiced_bt_ops_cb;
#endif

#if WICED_BT_OPS_DEBUG == TRUE
static char *ops_evt_code(wiced_bt_ops_int_evt_t evt_code);
static char *ops_state_code(wiced_bt_ops_state_t state_code);
#endif

/*******************************************************************************
**
** Function         wiced_bt_ops_sm_execute
**
** Description      State machine event handling function for OPS
**
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_ops_sm_execute(wiced_bt_ops_cb_t *p_cb, UINT16 event, wiced_bt_ops_data_t *p_data)
{
    tWICED_BT_OPS_ST_TBL     state_table;
    UINT8               action;
    int                 i;
#if WICED_BT_OPS_DEBUG == TRUE
    wiced_bt_ops_state_t in_state = wiced_bt_ops_cb.state;
    UINT16         in_event = event;
    WICED_BT_TRACE("OPS Event Handler: State 0x%02x [%s], Event [%s]\n", in_state,
                      ops_state_code(in_state),
                      ops_evt_code(event));
#endif

    /* look up the state table for the current state */
    state_table = wiced_bt_ops_st_tbl[p_cb->state];

    event &= 0x00FF;

    /* set next state */
    p_cb->state = state_table[event][WICED_BT_OPS_NEXT_STATE];

    /* execute action functions */
    for (i = 0; i < WICED_BT_OPS_ACTIONS; i++)
    {
        if ((action = state_table[event][i]) != WICED_BT_OPS_IGNORE)
        {
            (*wiced_bt_ops_action[action])(p_cb, p_data);
        }
        else
        {
            /* discard ops data */
            wiced_bt_ops_discard_data(p_data->hdr.event, p_data);
            break;
        }
    }

#if WICED_BT_OPS_DEBUG == TRUE
    if (in_state != wiced_bt_ops_cb.state)
    {
        WICED_BT_TRACE("OPS State Change: [%s] -> [%s] after Event [%s]\n",
                      ops_state_code(in_state),
                      ops_state_code(wiced_bt_ops_cb.state),
                      ops_evt_code(in_event));
    }
#endif
}

/*******************************************************************************
**
** Function         wiced_bt_ops_api_enable
**
** Description      Handle an api enable event.  This function enables the OP
**                  Server by opening an Obex/Rfcomm channel and placing it into
**                  listen mode.
**
**
** Returns          void
**
*******************************************************************************/
static void wiced_bt_ops_api_enable(wiced_bt_ops_cb_t *p_cb, wiced_bt_ops_data_t *p_data)
{
    wiced_bt_ops_api_enable_t *p_enable = &p_data->api_enable;

    /* initialize control block */
    memset(p_cb, 0, sizeof(wiced_bt_ops_cb_t));
    p_cb->p_cback = p_enable->p_cback;
    p_cb->p_data_cback = p_enable->p_data_cback;
    p_cb->app_id = p_enable->app_id;
    p_cb->srm = p_enable->srm;
    p_cb->fd = WICED_BT_OPS_INVALID_FD;

    wiced_bt_ops_cb.state = WICED_BT_OPS_LISTEN_ST;

    /* call enable action function */
    wiced_bt_ops_enable(p_cb, p_enable);
}

/*******************************************************************************
**
** Function         wiced_bt_ops_hdl_event
**
** Description      File transfer server main event handling function.
**
**
** Returns          void
**
*******************************************************************************/
BOOLEAN wiced_bt_ops_hdl_event(BT_HDR *p_msg)
{
    wiced_bt_ops_obx_evt_t* obx_et;
#if WICED_BT_OPS_DEBUG == TRUE
    wiced_bt_ops_state_t in_state = wiced_bt_ops_cb.state;
#endif

    switch (p_msg->event)
    {
        case WICED_BT_OPS_API_ENABLE_EVT:
#if WICED_BT_OPS_DEBUG == TRUE
            WICED_BT_TRACE("OPS Event Handler: State 0x%02x [%s], Event [%s]\n", in_state,
                              ops_state_code(in_state),
                              ops_evt_code(p_msg->event));
#endif
            wiced_bt_ops_api_enable(&wiced_bt_ops_cb, (wiced_bt_ops_data_t *) p_msg);

#if WICED_BT_OPS_DEBUG == TRUE
            if (in_state != wiced_bt_ops_cb.state)
            {
                WICED_BT_TRACE("OPS State Change: [%s] -> [%s] after Event [%s]\n",
                              ops_state_code(in_state),
                              ops_state_code(wiced_bt_ops_cb.state),
                              ops_evt_code(p_msg->event));
            }
#endif
            break;

        default:

            if (p_msg->event == WICED_BT_OPS_OBX_CONN_EVT ||
                p_msg->event == WICED_BT_OPS_OBX_DISC_EVT)
            {
                obx_et = (wiced_bt_ops_obx_evt_t*)p_msg;
                wiced_bt_ops_cb.handle = obx_et->handle;
                WICED_BT_TRACE("Storing handle as %d", wiced_bt_ops_cb.handle);
            }

            wiced_bt_ops_sm_execute(&wiced_bt_ops_cb, p_msg->event, (wiced_bt_ops_data_t *) p_msg);

            if ( wiced_bt_ops_cb.state == WICED_BT_OPS_CONN_ST )
            {
                if (( wiced_bt_ops_cb.pm_state == WICED_BT_OPS_PM_IDLE )
                  &&( wiced_bt_ops_cb.obx_oper != OPS_OP_NONE ))
                {
                    wiced_bt_ops_cb.pm_state = WICED_BT_OPS_PM_BUSY;
                }
                else if (( wiced_bt_ops_cb.pm_state == WICED_BT_OPS_PM_BUSY )
                       &&( wiced_bt_ops_cb.obx_oper == OPS_OP_NONE ))
                {
                    wiced_bt_ops_cb.pm_state = WICED_BT_OPS_PM_IDLE;
                }
            }
            else if ( wiced_bt_ops_cb.state == WICED_BT_OPS_LISTEN_ST )
            {
                /* initialize power management state */
                wiced_bt_ops_cb.pm_state = WICED_BT_OPS_PM_BUSY;
            }

            break;
    }

    return (TRUE);
}

/*****************************************************************************
**  Debug Functions
*****************************************************************************/
#if WICED_BT_OPS_DEBUG == TRUE

/*******************************************************************************
**
** Function         ops_evt_code
**
** Description
**
** Returns          void
**
*******************************************************************************/
static char *ops_evt_code(wiced_bt_ops_int_evt_t evt_code)
{
    switch(evt_code)
    {
    case WICED_BT_OPS_API_DISABLE_EVT:
        return "WICED_BT_OPS_API_DISABLE_EVT";
    case WICED_BT_OPS_API_ACCESSRSP_EVT:
        return "WICED_BT_OPS_API_ACCESSRSP_EVT";
    case WICED_BT_OPS_API_CLOSE_EVT:
        return "WICED_BT_OPS_API_CLOSE_EVT";
    case WICED_BT_OPS_CI_OPEN_EVT:
        return "WICED_BT_OPS_CI_OPEN_EVT";
    case WICED_BT_OPS_CI_WRITE_EVT:
        return "WICED_BT_OPS_CI_WRITE_EVT";
    case WICED_BT_OPS_CI_READ_EVT:
        return "WICED_BT_OPS_CI_READ_EVT";
    case WICED_BT_OPS_OBX_CONN_EVT:
        return "WICED_BT_OPS_OBX_CONN_EVT";
    case WICED_BT_OPS_OBX_DISC_EVT:
        return "WICED_BT_OPS_OBX_DISC_EVT";
    case WICED_BT_OPS_OBX_ABORT_EVT:
        return "WICED_BT_OPS_OBX_ABORT_EVT";
    case WICED_BT_OPS_OBX_CLOSE_EVT:
        return "WICED_BT_OPS_OBX_CLOSE_EVT";
    case WICED_BT_OPS_OBX_PUT_EVT:
        return "WICED_BT_OPS_OBX_PUT_EVT";
    case WICED_BT_OPS_OBX_GET_EVT:
        return "WICED_BT_OPS_OBX_GET_EVT";
    case WICED_BT_OPS_OBX_ACTION_EVT:
        return "WICED_BT_OPS_OBX_ACTION_EVT";
    case WICED_BT_OPS_CLOSE_CMPL_EVT:
        return "WICED_BT_OPS_CLOSE_CMPL_EVT";
    case WICED_BT_OPS_API_ENABLE_EVT:
        return "WICED_BT_OPS_API_ENABLE_EVT";
    default:
        return "unknown OPS event code";
    }
}

/*******************************************************************************
**
** Function         ops_evt_code
**
** Description
**
** Returns          void
**
*******************************************************************************/
static char *ops_state_code(wiced_bt_ops_state_t state_code)
{
    switch(state_code)
    {
    case WICED_BT_OPS_IDLE_ST:
        return "WICED_BT_OPS_IDLE_ST";
    case WICED_BT_OPS_LISTEN_ST:
        return "WICED_BT_OPS_LISTEN_ST";
    case WICED_BT_OPS_CONN_ST:
        return "WICED_BT_OPS_CONN_ST";
    case WICED_BT_OPS_CLOSING_ST:
        return "WICED_BT_OPS_CLOSING_ST";
    default:
        return "unknown OPS state code";
    }
}

#endif  /* Debug Functions */
