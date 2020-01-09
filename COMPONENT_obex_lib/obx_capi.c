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
**  Name:         obx_capi.c
**
**  File:         OBEX Client Application Programming Interface functions
**
**
*****************************************************************************/
#include "obx_int.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_bt_l2c.h"

tOBEX_STATUS obx_prepend_req_msg(tOBEX_HANDLE handle, tOBEX_CL_EVENT event, UINT8 req_code, BT_HDR *p_pkt);
tOBEX_STATUS obx_session_req (tOBEX_HANDLE handle, tOBEX_SESS_OP opcode, UINT32 timeout);

/**
 * Function     wiced_bt_obex_connect
 *
 *              This function registers a client entity to OBEX and sends a
 *              CONNECT request to the server
 *
 *  @param[in]   bd_addr  : Remote server BD address
 *  @param[in]   scn      : Remote server SCN
 *  @param[in]   mtu      : MTU
 *  @param[in]   p_cback  : Pointer to client event callback function
 *  @param[out]  p_handle : Pointer to return client connection handle
 *  @param[in]   p_pkt    : Connect request packet
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_connect(wiced_bt_device_address_t bd_addr, uint8_t scn, uint16_t mtu,
        wiced_bt_obex_client_cback_t *p_cback, wiced_bt_obex_handle_t *p_handle, uint8_t *p_pkt)
{
    wiced_bt_obex_status_t status = OBEX_NO_RESOURCES;
    UINT8       msg[OBEX_HDR_OFFSET + OBEX_MAX_CONN_HDR_EXTRA];
    UINT8       *p = msg;
    tOBEX_CL_CB  *p_cb;
    BT_HDR      *p_bt_hdr;

    WC_ASSERT(p_handle);

    OBEX_TRACE_DEBUG0("wiced_bt_obex_connect");

    p_cb = obx_cl_get_cb(*p_handle);
    if (p_cb == NULL)
        p_cb = obx_cl_alloc_cb();

    if (p_cb)
    {
        if (p_cb->ll_cb.port.port_handle == 0)
        {
            WC_ASSERT(p_cback);
            /* port is not open yet- open one
             * this is the first CONNECT request */
            p_cb->ll_cb.comm.rx_mtu = mtu;
            p_cb->p_cback   = p_cback;
            p_cb->state     = OBEX_CS_NOT_CONNECTED;
            status          = obx_open_port(&p_cb->ll_cb.port, bd_addr, scn);
            *p_handle       = p_cb->ll_cb.port.handle;
        }
        else
        {
            /* when called by other OBX functions */
            status  = OBEX_SUCCESS;
        }

        if (status == OBEX_SUCCESS)
        {
            /* Connect request packet always has the final bit set */
            *p++ = (OBEX_REQ_CONNECT | OBEX_FINAL);
            p += OBEX_PKT_LEN_SIZE;

            *p++ = OBEX_VERSION;
            *p++ = OBEX_CONN_FLAGS;
            UINT16_TO_BE_STREAM(p, p_cb->ll_cb.port.rx_mtu);
            /* IrOBEX spec forbids connection ID in Connect Request */
            p_bt_hdr = obx_cl_prepend_msg(p_cb, (BT_HDR *)p_pkt, msg, (UINT16)(p - msg) );

            p_bt_hdr->event    = OBEX_CONNECT_REQ_EVT;
            obx_csm_event(p_cb, OBEX_CONNECT_REQ_CEVT, p_bt_hdr);
        }
        else
        {
            OBEX_TRACE_ERROR1("Error opening port for scn: %d", scn);
            obx_cl_free_cb(p_cb);
        }
    }
    return status;
}

/**
 * Function     wiced_bt_obex_disconnect
 *
 *              This function disconnects the server and unregister the
 *              client entity.
 *
 *  @param[in]   handle   : Client connection handle
 *  @param[in]   p_pkt    : Disconnect request packet
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_disconnect(wiced_bt_obex_handle_t handle, uint8_t *p_pkt)
{
    /* Disconnect request always has the final bit set */
    UINT8 req_code = (OBEX_REQ_DISCONNECT|OBEX_FINAL);
    return obx_prepend_req_msg(handle, OBEX_DISCNT_REQ_CEVT, req_code, (BT_HDR *)p_pkt);
}

/**
 * Function     wiced_bt_obex_send_request
 *
 *              This function sends a request to the connected server.
 *
 *  @param[in]   handle   : Client connection handle
 *  @param[in]   req_code : Request code
 *  @param[in]   p_param  : Request parameters
 *  @param[in]   p_pkt    : Request packet
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_send_request(wiced_bt_obex_handle_t handle, wiced_bt_obex_req_code_t req_code,
        wiced_bt_obex_req_param_t *p_param, uint8_t *p_pkt)
{
    wiced_bt_obex_status_t status = OBEX_SUCCESS;
    BT_HDR *p_bt_hdr;

    switch (req_code)
    {
    case OBEX_REQ_PUT:
        if (p_param->final)
            req_code |= OBEX_FINAL;
        status = obx_prepend_req_msg(handle, OBEX_PUT_REQ_CEVT, req_code, (BT_HDR *)p_pkt);
        break;

    case OBEX_REQ_GET:
        if (p_param->final)
            req_code |= OBEX_FINAL;
        status = obx_prepend_req_msg(handle, OBEX_GET_REQ_CEVT, req_code, (BT_HDR *)p_pkt);
        break;

    case OBEX_REQ_SETPATH:
        {
            tOBEX_CL_CB  *p_cb = obx_cl_get_cb(handle);
            UINT8       msg[OBEX_HDR_OFFSET];
            UINT8       *p = msg;
            UINT8       good_flags = (OBEX_SPF_BACKUP | OBEX_SPF_NO_CREATE);

            if (p_cb)
            {
                /* SetPath request packet always has the final bit set */
                *p++    = (req_code | OBEX_FINAL);
                p       += OBEX_PKT_LEN_SIZE;
                *p++    = (p_param->sp_flags & good_flags); /* send only good flags */
                *p++    = OBEX_SETPATH_CONST;

                /* add session sequence number, if session is active */
                if (p_cb->sess_st == OBEX_SESS_ACTIVE)
                {
                    *p++ = OBEX_HI_SESSION_SN;
                    *p++ = p_cb->ssn;
                }

                /* add connection ID, if needed */
                if (p_cb->conn_id != OBEX_INVALID_CONN_ID)
                {
                    *p++    = OBEX_HI_CONN_ID;
                    UINT32_TO_BE_STREAM(p, p_cb->conn_id);
                }
                p_bt_hdr   = obx_cl_prepend_msg(p_cb, (BT_HDR *)p_pkt, msg, (UINT16)(p - msg) );
                p_bt_hdr->event    = OBEX_SETPATH_REQ_EVT;
                obx_csm_event(p_cb, OBEX_SETPATH_REQ_CEVT, p_bt_hdr);
            }
            else
            {
                status = OBEX_BAD_HANDLE;
            }
        }
        break;

    case OBEX_REQ_ACTION:
        {
            UINT8       *p;

            if (p_pkt == NULL)
            {
                OBEX_TRACE_ERROR0("OBEX_ActionReq must include Name & DestName Header for Copy/Move action" );
                OBEX_TRACE_ERROR0("OBEX_ActionReq must include Name & Permission Header for Set Object Permission action" );
                return OBEX_BAD_PARAMS;
            }

            p_bt_hdr = (BT_HDR *)p_pkt;
            req_code |= OBEX_FINAL;

            /* add the Action ID header before the other headers */
            p_bt_hdr->offset   -= 2;
            p_bt_hdr->len      += 2;
            p       = (UINT8 *)(p_bt_hdr + 1) + p_bt_hdr->offset;
            *p++    = OBEX_HI_ACTION_ID;
            *p++    = p_param->action;

            status = obx_prepend_req_msg(handle, OBEX_ACTION_REQ_CEVT, req_code, p_bt_hdr);
        }
        break;

    case OBEX_REQ_ABORT:
        req_code |= OBEX_FINAL;
        status = obx_prepend_req_msg(handle, OBEX_ABORT_REQ_CEVT, req_code, (BT_HDR *)p_pkt);
        break;

    default:
        status = OBEX_BAD_PARAMS;
        break;
    }

    return status;
}


#ifdef OBEX_LIB_SESSION_SUPPORTED
/**
 * Function     wiced_bt_obex_alloc_session
 *
 *              This function tries to find the suspended session with session
 *              info.  If p_session_info is NULL then it uses handle to find a
 *              control block.  If it is not found then allocate a new session.
 *
 *  @param[in]      p_session_info : Session info used to search for suspended session
 *  @param[in]      scn            : RFCOMM SCN
 *  @param[in]      p_psm          : Virtual PSM for L2CAP only, set to 0 if use RFCOMM
 *  @param[in]      p_cback        : Client callback
 *  @param[in/out]  p_handle       : Return allocated handle
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_alloc_session(uint8_t *p_session_info, uint8_t scn, uint16_t *p_psm,
        wiced_bt_obex_client_cback_t *p_cback, wiced_bt_obex_handle_t *p_handle)
{
    wiced_bt_obex_status_t status = OBEX_NO_RESOURCES;
    tOBEX_CL_CB  *p_cb;

    WC_ASSERT(p_handle);
    WC_ASSERT(p_cback);

    OBEX_TRACE_API2("OBEX_AllocSession scn: %d, psm:0x%x", scn, *p_psm);

    if (p_session_info)
    {
        p_cb = obx_cl_get_suspended_cb(p_handle, p_session_info);
        if (p_cb)
        {
            p_cb->srm = p_cb->sess_info[OBEX_SESSION_INFO_SRM_IDX];
            status  = OBEX_SUCCESS;
        }
    }
    else
    {
        p_cb = obx_cl_get_cb(*p_handle);
        if (p_cb == NULL)
            p_cb = obx_cl_alloc_cb();


    }

    if (p_cb)
    {
        p_cb->rsp_code  = 0;
        p_cb->psm       = 0;

        if (p_cb->sess_st != OBEX_SESS_SUSPENDED)
            p_cb->sess_st = OBEX_SESS_NONE;

#ifdef OBEX_LIB_L2CAP_INCLUDED
        if (p_psm && L2C_IS_VALID_PSM(*p_psm))
        {
            obx_register_l2c(p_cb, *p_psm);
            /* obx_register_l2c puts the virtual psm in p_cb->psm */
            if (p_cb->psm)
            {
                *p_psm  = p_cb->psm;
                status  = OBEX_SUCCESS;
            }
        }
#endif

        /* check SCN only when a virtual PSM is not allocated */
        if (!p_cb->psm)
        {
            if (scn)
            {
                /* borrow this data member temporarily */
                p_cb->rsp_code  = scn;
                status  = OBEX_SUCCESS;
            }
        }
    }

    if (status != OBEX_SUCCESS)
    {
        obx_cl_free_cb(p_cb);
        p_cb = NULL;
    }

    if (p_cb)
    {
        *p_handle       = p_cb->ll_cb.comm.handle;
        p_cb->p_cback   = p_cback;
        p_cb->state     = OBEX_CS_NOT_CONNECTED;
    }
    return status;
}

/**
 * Function     wiced_bt_obex_create_session
 *
 *              This function registers a client entity to OBEX and create
 *              a reliable session with server
 *
 *  @param[in]  bd_addr : Server device address
 *  @param[in]  mtu     : MTU
 *  @param[in]  srm     : TRUE - enable SRM, FALSE - no SRM
 *  @param[in]  nonce   : Create reliable session if this parameter is set
 *  @param[in]  handle  : Client handle
 *  @param[in]  p_pkt   : Request packet
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_create_session(wiced_bt_device_address_t bd_addr, uint16_t mtu,
        wiced_bool_t srm, uint32_t nonce, wiced_bt_obex_handle_t handle, uint8_t *p_pkt)
{
    wiced_bt_obex_status_t status = OBEX_NO_RESOURCES;
    tOBEX_CL_CB  *p_cb;
    UINT8       *p;
    UINT8       *pn;
    BT_HDR      *p_req;
    UINT8       data[20];
    tOBEX_TRIPLET triplet[4];
    UINT8       num_trip = 0;

    OBEX_TRACE_API1("OBEX_CreateSession handle: 0x%x", handle);
    p_cb = obx_cl_get_cb(handle);

    if (p_cb)
    {
        if (p_cb->state != OBEX_CS_NOT_CONNECTED || p_cb->sess_st != OBEX_SESS_NONE)
        {
            OBEX_TRACE_ERROR2("bad state: %d, or sess_st:%d", p_cb->state, p_cb->sess_st);
            return status;
        }

        if (p_cb->ll_cb.comm.id == 0)
        {
            p_cb->ll_cb.comm.rx_mtu = mtu;

            OBEX_TRACE_DEBUG2("scn: %d, psm:0x%x", p_cb->rsp_code, p_cb->psm);

#ifdef OBEX_LIB_L2CAP_INCLUDED
            if (p_cb->psm)
            {
                /* L2CAP channel is not open yet- open one
                 * this is the first CONNECT request */
                status = obx_open_l2c(p_cb, bd_addr);
            }
            else
#endif
            if (p_cb->rsp_code) /* p_cb->rsp_code is used as the scn */
            {
                /* port is not open yet- open one
                 * this is the first CONNECT request */
                status = obx_open_port(&p_cb->ll_cb.port, bd_addr, p_cb->rsp_code);
            }
        }
        else
        {
            /* when called by other OBX functions */
            status  = OBEX_SUCCESS;
        }

        if (status == OBEX_SUCCESS)
        {
            /* OBEX 1.5 */
            p_cb->srm       = OBEX_SRM_NO;
            if (srm)
                p_cb->srm   = OBEX_SRM_ENABLE;
            p_cb->nonce     = nonce;
            p_cb->sess_st   = OBEX_SESS_NONE;
            if (nonce)
            {
                if ( (p_req = (BT_HDR *)wiced_bt_obex_header_init(handle, OBEX_MIN_MTU)) != NULL)
                {
                    p = (UINT8 *) (p_req + 1) + p_req->offset;
                    /* Session request packet always has the final bit set */
                    *p++ = (OBEX_REQ_SESSION | OBEX_FINAL);
                    p_req->len = 3;
                    p = data;
                    /* add session opcode */
                    triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_SESS_OP;
                    triplet[num_trip].len = OBEX_LEN_SESS_PARAM_SESS_OP;
                    triplet[num_trip].p_array = p;
                    *p = OBEX_SESS_OP_CREATE;
                    p += OBEX_LEN_SESS_PARAM_SESS_OP;
                    num_trip++;

                    /* add device addr */
                    triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_ADDR;
                    triplet[num_trip].len = BD_ADDR_LEN;
                    triplet[num_trip].p_array = p;
                    wiced_bt_dev_read_local_addr (p);
                    p += BD_ADDR_LEN;
                    num_trip++;

                    /* add nonce 4 - 16 bytes */
                    triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_NONCE;
                    triplet[num_trip].len = OBEX_LOCAL_NONCE_SIZE;
                    pn = &p_cb->sess_info[OBEX_SESSION_ID_SIZE];
                    triplet[num_trip].p_array = pn;
                    UINT32_TO_BE_STREAM(pn, nonce);
                    num_trip++;

                    /* add timeout */
                    triplet[num_trip].p_array = p;
                    if (obx_add_timeout (&triplet[num_trip], obx_cb.sess_tout_val, &p_cb->param.sess))
                    {
                        num_trip ++;
                        p = &p_cb->sess_info[OBEX_SESSION_INFO_TO_IDX];
                        UINT32_TO_BE_STREAM(p, obx_cb.sess_tout_val);
                        p_cb->param.sess.timeout = obx_cb.sess_tout_val;
                    }

                    OBEX_AddTriplet(p_req, OBEX_HI_SESSION_PARAM, triplet, num_trip);
                    if (p_pkt)
                    {
                        /* assume that these headers are to be added to the connect req */
                        p_cb->p_next_req = (BT_HDR *)p_pkt;
                    }
                    /* adjust the packet len */
                    p = (UINT8 *) (p_req + 1) + p_req->offset + 1;
                    UINT16_TO_BE_STREAM(p, p_req->len);
                    p_req->event    = OBEX_SESSION_REQ_EVT;
                    p_cb->sess_st   = OBEX_SESS_CREATE;
                    obx_csm_event(p_cb, OBEX_SESSION_REQ_CEVT, p_req);
                }
                else
                    status = OBEX_NO_RESOURCES;
            }
            else /* legacy */
            {
                obx_ca_connect_req (p_cb, (BT_HDR *)p_pkt);
            }
        }
        if (status != OBEX_SUCCESS)
        {
            obx_cl_free_cb(p_cb);
        }
    }
    return status;
}

/**
 * Function     wiced_bt_obex_close_session
 *
 *              This function closes a reliable session
 *
 *  @param[in]  handle  : Client handle
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_close_session(wiced_bt_obex_handle_t handle)
{
    return obx_session_req(handle, OBEX_SESS_OP_CLOSE, 0);
}

/**
 * Function     wiced_bt_obex_suspend_session
 *
 *              This function suspends a reliable session
 *
 *  @param[in]  handle  : Client handle
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_suspend_session(wiced_bt_obex_handle_t handle)
{
    return obx_session_req(handle, OBEX_SESS_OP_SUSPEND, 0);
}

/**
 * Function     wiced_bt_obex_resume_session
 *
 *              This function registers a client entity to OBEX and resumes
 *              a previously interrupted reliable session
 *
 *  @param[in]  bd_addr : Server device address
 *  @param[in]  ssn     : Session sequence number
 *  @param[in]  offset  : Data offset
 *  @param[in]  handle  : Client handle
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_resume_session(wiced_bt_device_address_t bd_addr, uint8_t ssn,
        uint32_t offset, wiced_bt_obex_handle_t handle)
{
    wiced_bt_obex_status_t status = OBEX_NO_RESOURCES;
    UINT8       *p;
    tOBEX_CL_CB  *p_cb;
    BT_HDR      *p_req;
    tOBEX_TRIPLET triplet[6];
    UINT8       data[13];
    UINT8       num_trip = 0;
    UINT8       *pn;

    OBEX_TRACE_API3("OBEX_ResumeSession handle: 0x%x ssn:%d offset:%d", handle, ssn, offset);
    p_cb = obx_cl_get_cb(handle);

    if (p_cb)
    {
        OBEX_TRACE_DEBUG3("OBEX_ResumeSession, sess_st:%d srm:0x%x, saved state:0x%x", p_cb->sess_st,
            p_cb->sess_info[OBEX_SESSION_INFO_SRM_IDX], p_cb->sess_info[OBEX_SESSION_INFO_SRM_IDX]);
        if (p_cb->sess_st == OBEX_SESS_SUSPENDED)
        {
            if ((p_req = (BT_HDR *)wiced_bt_obex_header_init(OBEX_HANDLE_NULL, OBEX_MIN_MTU)) != NULL)
            {
                p = (UINT8 *) (p_req + 1) + p_req->offset;
                /* Session request packet always has the final bit set */
                *p++ = (OBEX_REQ_SESSION | OBEX_FINAL);
                p_req->len = 3;

                /* add session opcode */
                p = data;
                triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_SESS_OP;
                triplet[num_trip].len = OBEX_LEN_SESS_PARAM_SESS_OP;
                triplet[num_trip].p_array = p;
                *p = OBEX_SESS_OP_RESUME;
                p += OBEX_LEN_SESS_PARAM_SESS_OP;
                num_trip++;

                /* add device addr */
                triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_ADDR;
                triplet[num_trip].len = BD_ADDR_LEN;
                triplet[num_trip].p_array = p;
                wiced_bt_dev_read_local_addr (p);
                p += BD_ADDR_LEN;
                num_trip++;

                /* add nonce 4 - 16 bytes */
                triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_NONCE;
                triplet[num_trip].len = OBEX_LOCAL_NONCE_SIZE;
                pn = &p_cb->sess_info[OBEX_SESSION_ID_SIZE];
                triplet[num_trip].p_array = pn;
                num_trip++;

                /* add session id */
                triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_SESS_ID;
                triplet[num_trip].len = OBEX_SESSION_ID_SIZE;
                triplet[num_trip].p_array = p_cb->sess_info;
                num_trip++;

                if (p_cb->sess_info[OBEX_SESSION_INFO_SRM_IDX] & OBEX_SRM_ENGAGE)
                {
                    /* add ssn */
                    triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_NSEQNUM;
                    triplet[num_trip].len = 1;
                    triplet[num_trip].p_array = p;
                    *p++ = ssn;
                    num_trip++;

                    if (offset)
                    {
                    /* add object offset */
                    p_cb->param.sess.obj_offset = offset;
                    triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_OBJ_OFF;
                    triplet[num_trip].len = OBEX_LEN_SESS_PARAM_OBJ_OFF;
                    triplet[num_trip].p_array = p;
                    UINT32_TO_BE_STREAM(p, offset);
                    num_trip++;
                }
                }

                p_cb->sess_st   = OBEX_SESS_RESUME;
                OBEX_AddTriplet(p_req, OBEX_HI_SESSION_PARAM, triplet, num_trip);
                /* adjust the packet len */
                p = (UINT8 *) (p_req + 1) + p_req->offset + 1;
                UINT16_TO_BE_STREAM(p, p_req->len);
                p_req->event    = OBEX_SESSION_REQ_EVT;
                status = OBEX_SUCCESS;

                if (p_cb->ll_cb.comm.id == 0 || p_cb->ll_cb.comm.p_send_fn == 0)
                {
                    /* the transport is closed. open it again */
                    OBEX_TRACE_DEBUG2("scn: %d, psm:0x%x", p_cb->rsp_code, p_cb->psm);
                    p_cb->ll_cb.comm.rx_mtu = OBEX_MAX_MTU;

#ifdef OBEX_LIB_L2CAP_INCLUDED
                    if (p_cb->psm)
                    {
                        /* L2CAP channel is not open yet- open one
                         * this is the first CONNECT request */
                        status      = obx_open_l2c(p_cb, bd_addr);
                    }
                    else
#endif
                    if (p_cb->rsp_code) /* p_cb->rsp_code is used as the scn */
                    {
                        /* port is not open yet- open one
                         * this is the first CONNECT request */
                        status      = obx_open_port(&p_cb->ll_cb.port, bd_addr, p_cb->rsp_code);
                    }
                }

                if (status == OBEX_SUCCESS)
                {
                    pn = &p_cb->sess_info[OBEX_SESSION_INFO_ID_IDX];
                    BE_STREAM_TO_UINT32(p_cb->conn_id, pn);
                    p_cb->ssn = ssn;
                    p_cb->param.sess.ssn = ssn;
                    obx_csm_event(p_cb, OBEX_SESSION_REQ_CEVT, p_req);
                }
            }
        }
        else
        {
            OBEX_TRACE_ERROR1("Handle is not in a right state: %d for RESUME", p_cb->sess_st);
            status = OBEX_BAD_HANDLE;
        }
    }
    return status;
}

/**
 * Function     wiced_bt_obex_set_session_timeout
 *
 *              This function sets timeout for a reliable session
 *
 *  @param[in]  handle   : Client handle
 *  @param[in]  timeout  : Session timeout
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_set_session_timeout(wiced_bt_obex_handle_t handle, uint32_t timeout)
{
    return obx_session_req(handle, OBEX_SESS_OP_SET_TIME, timeout);
}
#endif

/*******************************************************************************
**
** Function     OBEX_GetPortHandle
**
** Description  This function is called to get RFCOMM port handle for the obex connection.
**
** Returns      OBEX_SUCCESS, if successful.
**              OBEX_NO_RESOURCES, if no existing connection.
**              OBEX_BAD_HANDLE, if the handle is not valid.
**
*******************************************************************************/
tOBEX_STATUS OBEX_GetPortHandle(tOBEX_HANDLE handle, UINT16 *port_handle)
{
    BD_ADDR bd_addr;
    UINT16      lcid;
    tOBEX_STATUS status = OBEX_SUCCESS;
    tOBEX_CL_CB  *p_cb = obx_cl_get_cb(handle);

    if (p_cb)
    {
        if (wiced_bt_rfcomm_check_connection(p_cb->ll_cb.port.port_handle, bd_addr, &lcid) != PORT_SUCCESS)
        {
            status = OBEX_NO_RESOURCES;
        }
        else
        {
            *port_handle = p_cb->ll_cb.port.port_handle;
        }
    }
    else
        status = OBEX_BAD_HANDLE;

    return status;
}

/*******************************************************************************
**
** Function     obx_prepend_req_msg
**
** Description  This function is called to add request code and connection ID
**              to the given OBEX message
** Returns      void
**
*******************************************************************************/
tOBEX_STATUS obx_prepend_req_msg(tOBEX_HANDLE handle, tOBEX_CL_EVENT event, UINT8 req_code, BT_HDR *p_pkt)
{
    tOBEX_STATUS status = OBEX_SUCCESS;
    tOBEX_CL_CB *p_cb = obx_cl_get_cb(handle);
    UINT8       msg[OBEX_HDR_OFFSET];
    UINT8       *p = msg;
    UINT8       srm = 0;
    UINT8       num_hdrs, num_body;

    if (p_cb)
    {
        *p++ = req_code;
        p += OBEX_PKT_LEN_SIZE;

        /* add session sequence number, if session is active */
        if (p_cb->sess_st == OBEX_SESS_ACTIVE)
        {
            *p++ = OBEX_HI_SESSION_SN;
            *p++ = p_cb->ssn;
        }

        req_code &= ~OBEX_FINAL;
/* set OBEX_USE_CONN_ID_ALL_PKTS to TRUE for workaround at UPF */
#define OBEX_USE_CONN_ID_ALL_PKTS        FALSE
#if (OBEX_USE_CONN_ID_ALL_PKTS == FALSE)
        /* add connection ID, if needed */
        if ((p_cb->conn_id != OBEX_INVALID_CONN_ID) &&
            /* always use connection ID in CONNECTED state */
            ((p_cb->state == OBEX_CS_CONNECTED) ||
            /* always use connection ID for abort and disconnect. they may be out of sequence */
            (req_code == OBEX_REQ_ABORT) || (req_code == OBEX_REQ_DISCONNECT)))
        {
            *p++    = OBEX_HI_CONN_ID;
            UINT32_TO_BE_STREAM(p, p_cb->conn_id);
        }
#else
        /* add connection ID, if needed */
        if(p_cb->conn_id != OBEX_INVALID_CONN_ID)
        {
            *p++    = OBEX_HI_CONN_ID;
            UINT32_TO_BE_STREAM(p, p_cb->conn_id);
        }
#endif

        /* add SRM header, if SRM is enabled */
        if (p_cb->srm & OBEX_SRM_ENABLE)
        {
            if (p_cb->state == OBEX_CS_CONNECTED)
            {
                if(event == OBEX_PUT_REQ_CEVT)
                {
                    num_hdrs = OBEX_ReadNumHdrs(p_pkt, &num_body);
                    OBEX_TRACE_DEBUG2("num_hdrs:%d num_body:%d", num_hdrs, num_body);
                    if (num_hdrs == num_body)
                    {
                        OBEX_TRACE_DEBUG0("it is left-over, drop it");
                        if (p_pkt)
                            GKI_freebuf (p_pkt);
                        return OBEX_BAD_PARAMS;
                    }
                    srm = OBEX_SRM_REQING | OBEX_SRM_WAIT;
                }
                else if (event == OBEX_GET_REQ_CEVT)
                {
                    srm = OBEX_SRM_REQING;
                }

                OBEX_TRACE_DEBUG2("cb srm: 0x%x/0x%x", p_cb->srm, srm );
                if (srm)
                {
                    p_cb->srm |= srm;
                    *p++ = OBEX_HI_SRM;
                    *p++ = OBEX_HV_SRM_ENABLE;
                }
            }
        }

        p_pkt = obx_cl_prepend_msg(p_cb, p_pkt, msg, (UINT16)(p - msg) );
        p_pkt->event    = obx_sm_evt_to_api_evt[event];
        obx_csm_event(p_cb, event, p_pkt);
    }
    else
        status = OBEX_BAD_HANDLE;

    return status;
}

/*******************************************************************************
**
** Function     OBEX_SessionReq
**
** Description  This function is used to Suspend/Close a session or update the
**              timeout value of the session.
**              If timeout is 0, OBEX_SESS_TIMEOUT_VALUE is the value used
**              THe timeout value is not added for the CloseSession request
**
** Returns      OBEX_SUCCESS, if successful.
**              OBEX_NO_RESOURCES, if OBX does not resources
**
*******************************************************************************/
tOBEX_STATUS obx_session_req (tOBEX_HANDLE handle, tOBEX_SESS_OP opcode, UINT32 timeout)
{
    tOBEX_STATUS status = OBEX_NO_RESOURCES;
    UINT8       *p;
    tOBEX_CL_CB  *p_cb;
    BT_HDR      *p_req;
    tOBEX_TRIPLET triplet[3];
    UINT8       data[12];
    UINT8       num_trip = 0;
    tOBEX_SESS_ST    old_sess_st;

    p_cb = obx_cl_get_cb(handle);

    if (p_cb)
    {
        OBEX_TRACE_API2("OBEX_SessionReq st:%d opcode:%d", p_cb->sess_st, opcode);
        old_sess_st = p_cb->sess_st;
        if ((p_req = (BT_HDR *)wiced_bt_obex_header_init(OBEX_HANDLE_NULL, OBEX_MIN_MTU)) != NULL)
        {
            status = OBEX_SUCCESS;
            p = (UINT8 *) (p_req + 1) + p_req->offset;
            /* Session request packet always has the final bit set */
            *p++ = (OBEX_REQ_SESSION | OBEX_FINAL);
            p_req->len = 3;

            /* add session opcode */
            p = data;
            triplet[num_trip].tag = OBEX_TAG_SESS_PARAM_SESS_OP;
            triplet[num_trip].len = OBEX_LEN_SESS_PARAM_SESS_OP;
            triplet[num_trip].p_array = p;
            *p = opcode;
            p += OBEX_LEN_SESS_PARAM_SESS_OP;
            num_trip++;
            if (timeout == 0)
            {
                timeout = obx_cb.sess_tout_val;
                if (p_cb->srm & OBEX_SRM_ENABLE)
                    timeout += obx_cb.sess_tout_val;
            }
            triplet[num_trip].p_array = p;
            switch (opcode)
            {
            case OBEX_SESS_OP_CLOSE:
                /* do not need any other session parameters */
                if (p_cb->sess_st != OBEX_SESS_NONE)
                    p_cb->sess_st  = OBEX_SESS_CLOSE;
                else
                {
                    OBEX_TRACE_ERROR1("Handle is not in a right state: %d for CLOSE", p_cb->sess_st);
                    status = OBEX_BAD_HANDLE;
                }
                break;

            case OBEX_SESS_OP_SUSPEND:
                /* do not need any other session parameters */
                if (p_cb->sess_st == OBEX_SESS_ACTIVE)
                {
                    /* add timeout value */
                    num_trip += obx_add_timeout (&triplet[num_trip], timeout, &p_cb->param.sess);
                    p_cb->sess_st  = OBEX_SESS_SUSPEND;
                }
                else
                {
                    OBEX_TRACE_ERROR1("Handle is not in a right state: %d for SUSPEND", p_cb->sess_st);
                    status = OBEX_BAD_HANDLE;
                }
                break;

            case OBEX_SESS_OP_SET_TIME:
                if (p_cb->sess_st == OBEX_SESS_ACTIVE)
                {
                    /* add timeout value */
                    num_trip += obx_add_timeout (&triplet[num_trip], timeout, &p_cb->param.sess);
                    p_cb->sess_st   = OBEX_SESS_TIMEOUT;
                }
                else
                {
                    OBEX_TRACE_ERROR1("Handle is not in a right state: %d for SET_TIME", p_cb->sess_st);
                    status = OBEX_BAD_HANDLE;
                }
                break;
            default:
                OBEX_TRACE_ERROR1("bad session opcode :%d", opcode);
                status = OBEX_BAD_PARAMS;
            }

            OBEX_TRACE_DEBUG4("OBEX_SessionReq, sess_st:%d->%d opcode:%d status:%d", old_sess_st, p_cb->sess_st, opcode, status);
            if (status != OBEX_SUCCESS)
                GKI_freebuf (p_req);
            else
            {
                OBEX_AddTriplet(p_req, OBEX_HI_SESSION_PARAM, triplet, num_trip);
                if (p_cb->sess_st == OBEX_SESS_SUSPEND)
                {
                    p_cb->sess_info[OBEX_SESSION_INFO_ST_IDX] = p_cb->state;
                    if (p_cb->state == OBEX_CS_PARTIAL_SENT)
                        p_cb->sess_info[OBEX_SESSION_INFO_ST_IDX] = p_cb->prev_state;
                    p_cb->sess_info[OBEX_SESSION_INFO_SRM_IDX] = p_cb->srm;
                    OBEX_TRACE_DEBUG2("suspend saved st:%d, srm:0x%x", p_cb->sess_info[OBEX_SESSION_INFO_ST_IDX], p_cb->sess_info[OBEX_SESSION_INFO_SRM_IDX]);
                }
                /* adjust the packet len */
                p = (UINT8 *) (p_req + 1) + p_req->offset + 1;
                UINT16_TO_BE_STREAM(p, p_req->len);
                p_req->event    = OBEX_SESSION_REQ_EVT;
                obx_csm_event(p_cb, OBEX_SESSION_REQ_CEVT, p_req);
            }
        }
    }
    return status;
}
