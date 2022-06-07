/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
**  Name:           wiced_bt_pbc_co.c
**
**  Description:    This file contains the file system call-out
**                  function implementation for WICED PBAP.
**
**
*****************************************************************************/


#include "wiced_bt_pbc_api.h"
#include "wiced_bt_pbc_co.h"
#include "wiced_bt_pbc_int.h"
#include "string.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "wiced_bt_event.h"

#define CO_NONE     0
#define CO_OPEN     1
#define CO_WRITE    2

UINT8           co_oper;    /* current active response action (abort or close) */
UINT16          co_evt;
int             co_fd;


/*******************************************************************************
** Function     wiced_bt_pbc_co_serialize
** Description
*******************************************************************************/
int wiced_bt_pbc_co_serialize()
{
    wiced_bt_pbc_co_status_t status = WICED_BT_PBC_CO_OK;
    UINT32  file_size = 0;
    int fd = 1;
    wiced_bt_pbc_ci_open_evt_t *p_msg  = NULL;
    wiced_bt_pbc_ci_write_evt_t *p_msg_write  = NULL;


    WICED_BT_TRACE("[CO] wiced_bt_pbc_co_serialize: co_evt:%d, co_oper :%d", co_evt, co_oper);

    if(co_oper == CO_OPEN)
    {
        if ((p_msg = (wiced_bt_pbc_ci_open_evt_t *)GKI_getbuf(sizeof(wiced_bt_pbc_ci_open_evt_t))) != NULL)
        {
            memset(p_msg, 0, sizeof(wiced_bt_pbc_ci_open_evt_t));

            p_msg->hdr.event = co_evt;
            p_msg->fd = co_fd;
            p_msg->status = status;

            wiced_bt_pbc_hdl_event((BT_HDR *)p_msg);
            GKI_freebuf(p_msg);
        }
    }
    else if(co_oper == CO_WRITE)
    {
        if ((p_msg_write = (wiced_bt_pbc_ci_write_evt_t *)GKI_getbuf(sizeof(wiced_bt_pbc_ci_write_evt_t))) != NULL)
        {
            memset(p_msg_write, 0, sizeof(wiced_bt_pbc_ci_write_evt_t));

            p_msg_write->hdr.event = co_evt;
            p_msg_write->fd = co_fd;
            p_msg_write->status = status;

            wiced_bt_pbc_hdl_event((BT_HDR *)p_msg_write);
            GKI_freebuf(p_msg_write);
        }
    }
    return 0;
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_co_open
**
** Description      This function is executed by BTA when a file is opened.
**                  The phone uses this function to open
**                  a file for reading or writing.
**
** Parameters       p_path  - Fully qualified path and file name.
**                  oflags  - permissions and mode (see constants above)
**                  size    - size of file to put (0 if unavailable or not applicable)
**                  evt     - event that must be passed into the call-in function.
**                  app_id  - application ID specified in the enable functions.
**                            It can be used to identify which profile is the caller
**                            of the call-out function.
**
** Returns          void
**
**                  Note: Upon completion of the request, a file descriptor (int),
**                        if successful, and an error code (wiced_bt_pbc_co_status_t)
**                        are returned in the call-in function, wiced_bt_pbc_ci_open().
**
*******************************************************************************/
void wiced_bt_pbc_co_open(UINT16 evt, UINT8 app_id)
{
    wiced_bt_pbc_co_status_t status = WICED_BT_PBC_CO_OK;
    UINT32  file_size = 0;
    int fd = 1;
    WICED_BT_TRACE("[CO] wiced_bt_pbc_co_open: evt:%d, app id:%d", evt, app_id);

    co_fd = fd;
    co_evt = evt;
    co_oper = CO_OPEN;
    wiced_app_event_serialize( wiced_bt_pbc_co_serialize, NULL);
}

/*******************************************************************************
**
** Function         wiced_bt_pbc_co_close
**
** Description      This function is called by BTA when a connection to a
**                  client is closed.
**
** Parameters       fd      - file descriptor of file to close.
**                  app_id  - application ID specified in the enable functions.
**                            It can be used to identify which profile is the caller
**                            of the call-out function.
**
** Returns          (wiced_bt_pbc_co_status_t) status of the call.
**                      [WICED_BT_PBC_CO_OK if successful],
**                      [WICED_BT_PBC_CO_FAIL if failed  ]
**
*******************************************************************************/
wiced_bt_pbc_co_status_t wiced_bt_pbc_co_close(int fd, UINT8 app_id)
{
    wiced_bt_pbc_co_status_t status = WICED_BT_PBC_CO_OK;
    WICED_BT_TRACE("[CO] wiced_bt_pbc_co_close: handle:%d, app id:%d", fd, app_id);

    return (status);
}


void dump_bytes(const uint8_t *p, uint32_t len)
{
    uint32_t i, j;
    char     buff1[100];

    while (len != 0)
    {
        memset(buff1, 0, sizeof(buff1));
        for (i = 0; i < len && i < 100; i++)
        {
            if( *p == '\n' || *p == '\r')
                buff1[i] = ' ';
            else
                buff1[i] = *p;

            p++;
        }
        len -= i;
        if (len != 0)
            WICED_BT_TRACE("%s\n", buff1);
    }
    WICED_BT_TRACE("%s\n", buff1);
}


/*******************************************************************************
**
** Function         wiced_bt_pbc_co_write
**
** Description      This function is called by io to send file data to the
**                  phone.
**
** Parameters       fd      - file descriptor of file to write to.
**                  p_buf   - buffer to read the data from.
**                  nbytes  - number of bytes to write out to the file.
**                  evt     - event that must be passed into the call-in function.
**                  app_id  - application ID specified in the enable functions.
**                            It can be used to identify which profile is the caller
**                            of the call-out function.
**
** Returns          void
**
**                  Note: Upon completion of the request, wiced_bt_pbc_ci_write() is
**                        called with the file descriptor and the status.  The
**                        call-in function should only be called when ALL requested
**                        bytes have been written, or an error has been detected,
**
*******************************************************************************/
void wiced_bt_pbc_co_write(int fd, const UINT8 *p_buf, UINT16 nbytes, UINT16 evt, UINT8 ssn, UINT8 app_id)
{
    wiced_bt_pbc_co_status_t  status = WICED_BT_PBC_CO_OK;

    WICED_BT_TRACE("wiced_bt_pbc_co_write: status=%d, nbytes:%d", status, nbytes);

    dump_bytes( p_buf, nbytes);

    co_fd = fd;
    co_evt = evt;
    co_oper = CO_WRITE;
    wiced_app_event_serialize( wiced_bt_pbc_co_serialize, NULL);
}
