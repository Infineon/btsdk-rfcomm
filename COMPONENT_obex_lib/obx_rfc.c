/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
**  Name:         obx_rfc.c
**
**  File:         OBEX  interface to the RFCOMM module
**
**
*****************************************************************************/
#include <string.h>

#include "obx_int.h"

#include "wiced_bt_sdp_defs.h"
#include "wiced_bt_rfcomm.h"

/*******************************************************************************
** Function     obx_rfc_snd_evt
** Description  Sends an rfcomm event to OBX through the BTU task.
*******************************************************************************/
static void obx_rfc_snd_evt (tOBEX_PORT_CB *p_pcb, UINT32 code)
{
    tOBEX_PORT_EVT evt;

    if (!p_pcb)
        return;

    evt.code = code;
    evt.p_pcb  = p_pcb;

    if (p_pcb->handle & OBEX_CL_HANDLE_MASK)
        obx_cl_proc_evt(&evt);
    else
        obx_sr_proc_evt(&evt);
}

/*******************************************************************************
** Function     obx_rfc_cback
** Description  find the port control block and post an event to BTU task.
**              NOTE: This callback does not handle connect up/down events.
**                  obx_rfc_mgmt_cback is used for these events.
*******************************************************************************/
static void obx_rfc_cback (wiced_bt_rfcomm_port_event_t code, UINT16 port_handle)
{
    tOBEX_PORT_CB    *p_pcb = obx_port_handle_2cb(port_handle);

    OBEX_TRACE_DEBUG2("obx_rfc_cback (code: 0x%x, port_handle: %d)\n", code, port_handle);

    if (p_pcb)
    {
        obx_rfc_snd_evt (p_pcb, code);
    }
    else
    {
        OBEX_TRACE_WARNING0("Can not find control block\n");
    }
}

/*******************************************************************************
** Function     obx_rfc_mgmt_cback
** Callback registered with the PORT entity's Management Callback so that OBX
** can be notified when the connection has come up or gone down.
********************************************************************************/
void obx_rfc_mgmt_cback(wiced_bt_rfcomm_result_t port_status, UINT16 port_handle)
{
    tOBEX_PORT_CB    *p_pcb = obx_port_handle_2cb(port_handle);
    UINT32           code;

    OBEX_TRACE_DEBUG2("obx_rfc_mgmt_cback(port_status: %d, port_handle: %d)\n", port_status, port_handle);

    if (!p_pcb && port_status != WICED_BT_RFCOMM_SUCCESS)
    {
        /* See if error called within RFCOMM_CreateConnection */
        if (obx_cb.p_temp_pcb)
        {
            p_pcb = obx_cb.p_temp_pcb;
            obx_cb.p_temp_pcb = NULL;
        }
    }

    if (p_pcb)
    {
        code = (port_status == WICED_BT_RFCOMM_SUCCESS) ? PORT_EV_CONNECTED : PORT_EV_CONNECT_ERR;
        obx_rfc_snd_evt (p_pcb, code);

        p_pcb->b_connected = (code == PORT_EV_CONNECTED) ? TRUE : FALSE;

        if (p_pcb->b_connected && p_pcb->p_txmsg != NULL)
            obx_rfc_snd_msg(p_pcb);
    }
    else
    {
        OBEX_TRACE_WARNING0("mgmt cback: Can not find control block\n");
    }
}


/*******************************************************************************
** Function     obx_read_data
** Description  This functions reads data from FRCOMM. Return a message if the
**              whole packet is read.
**              The following defines how BT_HDR is used in this function
**              event:          response or request event code
**              len:            the length read so far.
**              offset:         offset to the beginning of the actual data.
**              layer_specific: left
*******************************************************************************/
static BT_HDR * obx_read_data (tOBEX_PORT_CB *p_pcb, tOBEX_VERIFY_OPCODE p_verify_opcode)
{
    BT_HDR  *p_ret = NULL;
    UINT8   *p;
    UINT16  ask_len;
    UINT16  got_len;
    int     rc;
    tOBEX_RX_HDR *p_rxh;
    tOBEX_SR_SESS_CB  *p_scb;
    UINT8       opcode;
    UINT16      pkt_len;
    BOOLEAN     failed = FALSE;

    OBEX_TRACE_DEBUG1("obx_read_data port_handle:%d\n", p_pcb->port_handle );
    for (;;)
    {
        if (p_pcb->p_rxmsg == NULL)
        {
            p_pcb->p_rxmsg = (BT_HDR *)wiced_bt_obex_header_init((tOBEX_HANDLE)(p_pcb->handle|OBEX_HANDLE_RX_MTU_MASK),
                                         OBEX_LRG_DATA_POOL_SIZE);
            if (p_pcb->p_rxmsg == NULL)
            {
                failed = TRUE;
                break;
            }
            memset((p_pcb->p_rxmsg + 1), 0, sizeof(tOBEX_RX_HDR));
        }
        /* we use this header to keep the status of this packet (instead of in control block) */
        p_rxh   = (tOBEX_RX_HDR *)(p_pcb->p_rxmsg + 1);

        ask_len = 0;
        if (p_rxh->code == 0)
        {
            if (p_pcb->p_rxmsg->len == 0) /* we need this if statement in case of "throw away" */
                ask_len = 1;
        }
        else if (p_pcb->p_rxmsg->len < (OBEX_PKT_LEN_SIZE + 1) )
        {
            /* if we do not know the packet len yet, read from port */
            ask_len = OBEX_PKT_LEN_SIZE + 1 - p_pcb->p_rxmsg->len;
        }
        else
        {
            /* we already know the packet len.
             * determine how many more bytes we need for this packet */
            ask_len = p_rxh->pkt_len - p_pcb->p_rxmsg->len;
        }

        /* the position of next byte to read */
        p = (UINT8 *)(p_pcb->p_rxmsg + 1) + p_pcb->p_rxmsg->offset + p_pcb->p_rxmsg->len;

        if (ask_len)
        {
            rc = PORT_ReadData( p_pcb->port_handle, (char*)p, ask_len, &got_len);
            if (rc != PORT_SUCCESS)
            {
                OBEX_TRACE_WARNING2("Error %d returned from PORT_Read_Data, len:%d\n", rc, got_len);
            }

            OBEX_TRACE_DEBUG2("ask_len: %d, got_len:%d\n", ask_len, got_len );
            if (got_len == 0)
            {
                /* If we tried to read but did not get anything, */
                /* there is nothing more to read at this time */
                break;
            }
            p_pcb->p_rxmsg->len             += got_len;
            p_pcb->p_rxmsg->layer_specific  -= got_len;
        }

        /* process the response/opcode, if not yet */
        if (p_rxh->code == 0 && p_pcb->p_rxmsg->len)
        {
            opcode  = *((UINT8 *)(p_pcb->p_rxmsg + 1) + p_pcb->p_rxmsg->offset);
            if ( (p_verify_opcode)(opcode, p_rxh) == OBEX_BAD_SM_EVT)
            {
                OBEX_TRACE_WARNING1("bad opcode:0x%x - Disconnecting\n", opcode );
                /* received data with bad length. */

                /*bad length disconnect */
                failed = TRUE;
                break;
            }
            continue;
        }

        /* process the packet len */
        if (p_rxh->pkt_len == 0 && p_pcb->p_rxmsg->len >= (OBEX_PKT_LEN_SIZE + 1) )
        {
            p = (UINT8 *)(p_pcb->p_rxmsg + 1) + p_pcb->p_rxmsg->offset + 1;
            BE_STREAM_TO_UINT16(pkt_len, p);

            if ( (pkt_len > p_pcb->rx_mtu) ||
                (pkt_len < 3) ||
                (pkt_len == 4) )
            {
                /* received data with bad length. */
                OBEX_TRACE_WARNING2("Received bad packet len -Disconnecting: %d RX MTU: %x\n",
                    pkt_len, p_pcb->rx_mtu);
                /*bad length disconnect */
                failed = TRUE;
                break;
            }
            else
            {
                /* keep the packet len in the header */
                p_rxh->pkt_len  = pkt_len;
            }
            continue;
        }

        if (p_pcb->p_rxmsg->len == p_rxh->pkt_len)
        {
            /* received a whole packet */
            OBEX_TRACE_DEBUG1("got a packet. opcode:0x%x\n", p_rxh->code );
            p_ret = p_pcb->p_rxmsg;
            p_pcb->p_rxmsg = NULL;
            break;
        }

    }

    if (failed)
    {
        if (p_pcb->handle & OBEX_CL_HANDLE_MASK)
        {
            obx_close_port(p_pcb->port_handle);
        }
        else
        {
            if ((p_scb = obx_sr_get_scb(p_pcb->handle)) != NULL)
                obx_ssm_event(p_scb, OBEX_PORT_CLOSE_SEVT, NULL);

        }
        p_ret = NULL;
    }

    if (p_pcb->p_rxmsg)
    {
       if (p_pcb->p_rxmsg->len == 0)
       {
           GKI_freebuf(p_pcb->p_rxmsg);
           p_pcb->p_rxmsg = NULL;
       }
    }

    return p_ret;
}


/*******************************************************************************
** Function     obx_cl_proc_evt
** Description  This is called to process BT_EVT_TO_OBEX_CL_MSG
**              Process events from RFCOMM. Get the associated client control
**              block. If this is a response packet, stop timer. Call
**              obx_csm_event() with event OK_CFM, FAIL_CFM or CONT_CFM.
*******************************************************************************/
void obx_cl_proc_evt(tOBEX_PORT_EVT *p_evt)
{
    tOBEX_PORT_CB *p_pcb = p_evt->p_pcb;
    tOBEX_CL_CB  *p_cb = obx_cl_get_cb(p_pcb->handle);
    BT_HDR      *p_pkt;

    if (p_cb == NULL)
    {
        /* probably already close the port and deregistered from OBX */
        OBEX_TRACE_ERROR1("Could not find control block for handle: 0x%x\n", p_pcb->handle);
        return;
    }

    if (p_evt->code & PORT_EV_CONNECT_ERR)
    {
        obx_csm_event(p_cb, OBEX_PORT_CLOSE_CEVT, NULL);
        return;
    } /* PORT_EV_CONNECT_ERR */

    if (p_evt->code & PORT_EV_TXEMPTY)
    {
        obx_csm_event(p_cb, OBEX_TX_EMPTY_CEVT, NULL);
    } /* PORT_EV_TXEMPTY */

    if (p_evt->code & PORT_EV_RXCHAR)
    {
        while( (p_pkt = obx_read_data(p_pcb, obx_verify_response)) != NULL )
        {
            if (GKI_queue_is_empty(&p_pcb->rx_q))
            {
                if (p_pkt->event != OBEX_BAD_SM_EVT)
                {
                    obx_cl_proc_pkt (p_cb, p_pkt);
                }
                else
                {
                    OBEX_TRACE_ERROR0("bad SM event\n");
                }
            }
            else if (p_pkt->event != OBEX_BAD_SM_EVT)
            {
                GKI_enqueue (&p_pcb->rx_q, p_pkt);
                if (p_pcb->rx_q.count > obx_cb.max_rx_qcount)
                {
                    p_pcb->stopped = TRUE;
                    wiced_bt_rfcomm_flow_control(p_pcb->port_handle, FALSE);
                }
            }
        } /* while received a packet */
    } /* PORT_EV_RXCHAR */

    if (p_evt->code & PORT_EV_FC)
    {
        if (p_evt->code & PORT_EV_FCS)
        {
            OBEX_TRACE_EVENT0("cl flow control event - FCS SET ----\n");
            obx_csm_event(p_cb, OBEX_FCS_SET_CEVT, NULL);
        }
    } /* PORT_EV_FC */
}

/*******************************************************************************
** Function     obx_build_dummy_rsp
** Description  make up a dummy response if the app does not call response API
**              yet and AbortRsp is called
*******************************************************************************/
BT_HDR * obx_build_dummy_rsp(tOBEX_SR_SESS_CB *p_scb, UINT8 rsp_code)
{
    BT_HDR  *p_pkt;
    UINT8   *p;
    UINT16  size = 3;

    if ((p_pkt = (BT_HDR *)wiced_bt_obex_header_init(p_scb->ll_cb.comm.handle, OBEX_CMD_POOL_SIZE)) != NULL)
    {
        p = (UINT8 *)(p_pkt+1)+p_pkt->offset+p_pkt->len;
        *p++    = (rsp_code|OBEX_FINAL);
        if (p_scb->conn_id)
        {
            size += 5;
            UINT16_TO_BE_STREAM(p, size);
            *p++    = OBEX_HI_CONN_ID;
            UINT32_TO_BE_STREAM(p, p_scb->conn_id);
        }
        else
        {
            UINT16_TO_BE_STREAM(p, size);
        }
        p_pkt->len   = size;
        p_pkt->event = OBEX_PUT_RSP_EVT; /* or OBEX_GET_RSP_EVT: for tracing purposes */
    }
    return p_pkt;
}

/*******************************************************************************
** Function     obx_add_port
** Description  check if this server has aother un-used port to open
**
**
** Returns      void
*******************************************************************************/
void obx_add_port(tOBEX_HANDLE obx_handle)
{
    tOBEX_SR_CB * p_cb = obx_sr_get_cb(obx_handle);
    tOBEX_SR_SESS_CB *p_scb, *p_scb0;
    int xx;
    tOBEX_STATUS status = OBEX_NO_RESOURCES;
    BOOLEAN found;

    OBEX_TRACE_DEBUG1("obx_add_port handle:0x%x\n", obx_handle );
    if (p_cb && p_cb->scn)
    {
        OBEX_TRACE_DEBUG2("num_sess:%d scn:%d\n", p_cb->num_sess, p_cb->scn );
        p_scb0 = &obx_cb.sr_sess[p_cb->sess[0]-1];
        found = FALSE;
        /* find an RFCOMM port that is not connected yet */
        for (xx=0; xx < p_cb->num_sess && p_cb->sess[xx]; xx++)
        {
            p_scb = &obx_cb.sr_sess[p_cb->sess[xx]-1];
            OBEX_TRACE_DEBUG3("[%d] id:0x%x, state:%d\n", xx, p_scb->ll_cb.comm.id, p_scb->state );

            if (p_scb->ll_cb.comm.p_send_fn == (tOBEX_SEND_FN *)obx_rfc_snd_msg
                && p_scb->state == OBEX_SS_NOT_CONNECTED)
            {
                found = TRUE;
                break;
            }
        }

        if (!found)
        {
            for (xx=0; xx < p_cb->num_sess && p_cb->sess[xx]; xx++)
            {
                p_scb = &obx_cb.sr_sess[p_cb->sess[xx]-1];
                OBEX_TRACE_DEBUG2("[%d] port_handle:%d\n", xx, p_scb->ll_cb.port.port_handle );
                if (!p_scb->ll_cb.comm.id)
                {
                    status = obx_open_port(&p_scb->ll_cb.port, PB_BT_BD_ANY, p_cb->scn);
                    if (status == OBEX_SUCCESS)
                    {
                        p_scb->ll_cb.port.rx_mtu= p_scb0->ll_cb.port.rx_mtu;
                        p_scb->state            = OBEX_SS_NOT_CONNECTED;
                    }
                    break;
                }
            }
        }
    }
}

/*******************************************************************************
** Function     obx_sr_proc_evt
** Description  This is called to process BT_EVT_TO_OBEX_SR_MSG
**              Process events from RFCOMM. Get the associated server control
**              block. If this is a request packet, stop timer. Find the
**              associated API event and save it in server control block
**              (api_evt). Fill the event parameter (param).
**              Call obx_ssm_event() with the associated events.If the associated
**              control block is not found (maybe the target header does not
**              match) or busy, compose a service unavailable response and call
**              obx_rfc_snd_msg().
*******************************************************************************/
void obx_sr_proc_evt(tOBEX_PORT_EVT *p_evt)
{
    tOBEX_SR_SESS_CB *p_scb;
    BT_HDR      *p_pkt;
    tOBEX_RX_HDR *p_rxh;
    tOBEX_PORT_CB *p_pcb = p_evt->p_pcb;


    OBEX_TRACE_DEBUG2("obx_sr_proc_evt handle: 0x%x, port_handle:%d\n", p_evt->p_pcb->handle, p_evt->p_pcb->port_handle);
    if (p_pcb->handle == 0 || p_pcb->p_send_fn != (tOBEX_SEND_FN*)obx_rfc_snd_msg)
        return;

    if ((p_scb = obx_sr_get_scb(p_pcb->handle)) == NULL)
    {
        /* probably already close the port and deregistered from OBX */
        OBEX_TRACE_ERROR1("Could not find control block for handle: 0x%x\n", p_pcb->handle);
        return;
    }

    if (p_evt->code & PORT_EV_CONNECTED)
    {
        p_scb->ll_cb.port.tx_mtu = OBEX_MIN_MTU;
        obx_start_timer(&p_scb->ll_cb.comm);
        /* Get the Bd_Addr */
        wiced_bt_rfcomm_check_connection (p_scb->ll_cb.port.port_handle,
                              p_scb->param.conn.peer_addr,
                              NULL);
         memcpy(p_scb->peer_addr, p_scb->param.conn.peer_addr, BD_ADDR_LEN);
   }

    if (p_evt->code & PORT_EV_CONNECT_ERR)
    {
        obx_ssm_event(p_scb, OBEX_PORT_CLOSE_SEVT, NULL);
        return;
    } /* PORT_EV_CONNECT_ERR */

    if (p_evt->code & PORT_EV_RXCHAR)
    {
        while( (p_pkt = obx_read_data(p_pcb, obx_verify_request)) != NULL)
        {
            p_rxh = (tOBEX_RX_HDR *)(p_pkt + 1);
            p_pkt->event = obx_sm_evt_to_api_evt[p_rxh->sm_evt];
#if BT_TRACE_PROTOCOL == TRUE
            DispObxMsg(p_pkt, (BOOLEAN)(obx_api_evt_to_disp_type[p_pkt->event] | OBEX_DISP_IS_RECV));
#endif
            if (GKI_queue_is_empty(&p_pcb->rx_q))
            {
                if (p_pkt->event != OBEX_BAD_SM_EVT)
                {
                    obx_sr_proc_pkt (p_scb, p_pkt);
                }
                else
                {
                    OBEX_TRACE_ERROR0("bad SM event\n");
                }
            }
            else
            {
                GKI_enqueue (&p_pcb->rx_q, p_pkt);
                if (p_pcb->rx_q.count > obx_cb.max_rx_qcount)
                {
                    p_pcb->stopped = TRUE;
                    wiced_bt_rfcomm_flow_control(p_pcb->port_handle, WICED_FALSE);
                }
            }
        } /* while a packet */
    } /* PORT_EV_RXCHAR */

    /* The server does not need to handle this event
    */
    if (p_evt->code & PORT_EV_TXEMPTY)
    {
        obx_ssm_event(p_scb, OBEX_TX_EMPTY_SEVT, NULL);
    }

    if (p_evt->code & PORT_EV_FC)
    {
        if (p_evt->code & PORT_EV_FCS)
        {
            OBEX_TRACE_EVENT0("sr flow control event - FCS SET ----\n");
            obx_ssm_event(p_scb, OBEX_FCS_SET_SEVT, NULL);
        }
    } /* PORT_EV_FC */
}

/*******************************************************************************
** Function     obx_open_port
** Description  Call RFCOMM_CreateConnection() to get port_handle.
**              Call PORT_SetEventCallback() with given callback.
**              Call PORT_SetEventMask() with given event mask. Return port handle.
** Returns      port handle
*******************************************************************************/
tOBEX_STATUS obx_open_port(tOBEX_PORT_CB *p_pcb, const BD_ADDR bd_addr, UINT8 scn)
{
    tOBEX_STATUS status = OBEX_SUCCESS; /* successful */
    wiced_bt_rfcomm_result_t result;
    BOOLEAN     is_server = (p_pcb->handle & OBEX_CL_HANDLE_MASK)?FALSE:TRUE;
    UINT16      max_mtu = OBEX_MAX_MTU;

    OBEX_TRACE_DEBUG2("obx_open_port rxmtu:%d, cbmtu:%d\n", p_pcb->rx_mtu, max_mtu );

    /* clear buffers from previous connection */
    obx_free_buf ((tOBEX_LL_CB*)p_pcb);

    /* make sure the MTU is in registered range */
    if (p_pcb->rx_mtu > max_mtu)
        p_pcb->rx_mtu   = max_mtu;
    if (p_pcb->rx_mtu < OBEX_MIN_MTU)
        p_pcb->rx_mtu   = OBEX_MIN_MTU;

    /* There's a remote chance that an error can occur in L2CAP before the handle
     * before the handle can be assigned (server side only).  We will save the
     * client control block while the handle is not known */
    if (!is_server)
    {
        obx_cb.p_temp_pcb = p_pcb;
    }

    result = wiced_bt_rfcomm_create_connection ( UUID_PROTOCOL_OBEX, scn,
                                        is_server, (UINT16)(p_pcb->rx_mtu+1), (BD_ADDR_PTR)bd_addr,
                                        &p_pcb->port_handle, obx_rfc_mgmt_cback);

    OBEX_TRACE_DEBUG3("obx_open_port rxmtu:%d, port_handle:%d, port.handle:0x%x\n",
        p_pcb->rx_mtu, p_pcb->port_handle, p_pcb->handle );

    if (!is_server)
    {
        obx_cb.p_temp_pcb = NULL;
    }

    if (result == WICED_BT_RFCOMM_SUCCESS)
    {
        obx_cb.hdl_map[p_pcb->port_handle - 1] = p_pcb->handle;
        wiced_bt_rfcomm_set_event_callback (p_pcb->port_handle, obx_rfc_cback);
        wiced_bt_rfcomm_set_event_mask (p_pcb->port_handle, OBEX_PORT_EVENT_MASK);
        p_pcb->p_send_fn = (tOBEX_SEND_FN *)obx_rfc_snd_msg;
        p_pcb->p_close_fn = obx_close_port;
    }
    else
    {
        status = OBEX_NO_RESOURCES;
    }

    return status;
}


/*******************************************************************************
** Function     obx_close_port
** Description  Clear the port event mask and callback. Close the port.
** Returns      void
*******************************************************************************/
void obx_close_port(UINT16 port_handle)
{
    wiced_bt_rfcomm_remove_connection(port_handle, WICED_FALSE);
}

/*******************************************************************************
** Function     obx_rfc_snd_msg
** Description  Call wiced_bt_rfcomm_write_data() to send an OBEX message to peer. If
**              all data is sent, free the GKI buffer that holds
**              the OBEX message.  If  only portion of data is
**              sent, adjust the BT_HDR for PART state.
** Returns      TRUE if all data is sent
*******************************************************************************/
BOOLEAN obx_rfc_snd_msg(tOBEX_PORT_CB *p_pcb)
{
    BOOLEAN status = FALSE;
    UINT16  bytes_written = 0;
    wiced_result_t result;

    if(p_pcb->b_connected == FALSE)
        return TRUE;

    obx_stop_timer(&p_pcb->tle);

    result = wiced_bt_rfcomm_write_data(p_pcb->port_handle, ((char*)(p_pcb->p_txmsg + 1)) + p_pcb->p_txmsg->offset, p_pcb->p_txmsg->len, &bytes_written);

    OBEX_TRACE_DEBUG4("obx_rfc_snd_msg port_handle:%d, port.handle:0x%x result:%d written:%d\n", p_pcb->port_handle, p_pcb->handle, result, bytes_written);

    obx_start_timer ((tOBEX_COMM_CB *)p_pcb);

    if (bytes_written == p_pcb->p_txmsg->len)
    {
        GKI_freebuf(p_pcb->p_txmsg);
        p_pcb->p_txmsg = NULL;
        status = TRUE;
    }
    else
    {
        /* packet not completely written to RFCOMM */
        p_pcb->p_txmsg->offset  += bytes_written;
        p_pcb->p_txmsg->len     -= bytes_written;
    }

    return status;
}
