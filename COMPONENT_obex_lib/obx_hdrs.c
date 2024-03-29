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
**  Name:         obx_hdrs.c
**
**  File:         OBEX Client Application Programming Interface functions
**
**
*****************************************************************************/
#include <string.h>

#include "obx_int.h"
#include "wiced_gki.h"


/**
 * Function     wiced_bt_obex_header_init
 *
 *              This function is called to initialize an OBEX packet. This
 *              function takes a GKI buffer and sets the offset in BT_HDR as
 *              OBEX_HDR_OFFSET, the len as 0. The layer_specific is set to the
 *              length still available. This function compares the given
 *              (pkt_size - sizeof(uint8_t)) with the peer MTU to get the lesser
 *              of the two and set the layer_specific to
 *              (lesser_size - OBEX_HDR_OFFSET).
 *              If composing a header for the CONNECT request (there is no
 *              client handle yet), use OBEX_HANDLE_NULL as the handle.
 *
 *  @param[in]  handle    : OBEX handle
 *  @param[in]  pkt_size  : Required packet size
 *
 *  @return     A pointer to allocated packet
 *
 */
uint8_t * wiced_bt_obex_header_init(wiced_bt_obex_handle_t handle, uint16_t pkt_size)
{
    uint16_t  mtu    = OBEX_HandleToMtu(handle);
    BT_HDR    *p_pkt = NULL;
    uint16_t  buf_size;
#if (BT_USE_TRACES == TRUE)
    uint16_t  req_size = pkt_size;
#endif

    pkt_size -= BT_HDR_SIZE;
    if(pkt_size > mtu )
        pkt_size = mtu;
    pkt_size    += (BT_HDR_SIZE + OBEX_HDR_OFFSET);

    OBEX_TRACE_DEBUG4( "OBEX_HdrInit: checking req_size %d, pkt_size:%d, max:%d, offset:%d\n",
        req_size, pkt_size, GKI_MAX_BUF_SIZE, OBEX_HDR_OFFSET);
    p_pkt = (BT_HDR *) GKI_getbuf(pkt_size);

    if(p_pkt)
    {
        buf_size    = GKI_get_buf_size(p_pkt);
        buf_size    -= BT_HDR_SIZE;
        if(buf_size > mtu)
            buf_size = mtu;

        OBEX_TRACE_DEBUG4( "OBEX_HdrInit: req_size %d, pkt_size = %d, gki_size %d, buf_size %d\n",
                            req_size, pkt_size, GKI_get_buf_size(p_pkt), buf_size);

        p_pkt->offset   = OBEX_HDR_OFFSET;
        p_pkt->len      = 0;
        p_pkt->event    = 0;

        /* layer specific contains remaining space in packet */
        p_pkt->layer_specific = buf_size - OBEX_HDR_OFFSET ;
        p_pkt->layer_specific -= 2;

        OBEX_TRACE_DEBUG2( "buf size: %d, ls:%d\n", buf_size, p_pkt->layer_specific);
    }
    else
    {
        OBEX_TRACE_ERROR1("OBEX_HdrInit: No buffers for size (%d)\n", pkt_size);
    }

    return (uint8_t *)p_pkt;
}

/**
 * Function     wiced_bt_obex_add_header
 *
 *              This function adds a header to the OBEX packet.
 *
 *              For Unicode text header, parameter p_data should point to an Ascii string
 *              and len should contain the string length.  This function will convert the
 *              input to Unicode string.  If the input string is UTF-8, call
 *              @link wiced_bt_obex_add_header_utf8 @endlink  instead.
 *              For byte sequence header, parameter p_data should point to the byte sequence
 *              and len should contain the byte sequence length.
 *              For 1 byte header, parameter p_data should point to a byte and len
 *              should equal to 1.
 *              For 4 byte header, parameter p_data should point to a uint32_t and len
 *              should equal to 4.
 *
 *  @param[in]  p_pkt   : OBEX packet
 *  @param[in]  id      : OBEX header identifier
 *  @param[in]  p_data  : Pointer to data
 *  @param[in]  len     : Data length
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_add_header(uint8_t *p_pkt, wiced_bt_obex_header_identifier_t id,
        uint8_t *p_data, uint16_t len)
{
    wiced_bt_obex_status_t status = OBEX_BAD_PARAMS;
    uint32_t data;

    switch (id & OBEX_HI_TYPE_MASK)
    {
    case OBEX_HI_TYPE_UNIC:
        {
            uint16_t len_data = 0;
            uint16_t *p_unicode = NULL;

            if(p_data)
            {
                len_data = strlen((char *)p_data) + 1;
                p_unicode = (uint16_t *)GKI_getbuf((uint16_t)(len_data*2));
            }

            if(p_unicode)
                len_data = OBEX_CharToWchar(p_unicode, (char *)p_data, len_data);
            else
                len_data = 0;

            status = OBEX_AddUnicodeHdr((BT_HDR *)p_pkt, OBEX_HI_NAME, p_unicode, len_data);

            if(p_unicode)
                GKI_freebuf(p_unicode);
        }
        break;

    case OBEX_HI_TYPE_ARRAY:
        status = OBEX_AddByteStrHdr((BT_HDR *)p_pkt, id, p_data, len);
        break;

    case OBEX_HI_TYPE_BYTE:
        status = OBEX_Add1ByteHdr((BT_HDR *)p_pkt, id, *p_data);
        break;

    case OBEX_HI_TYPE_INT:
        if (len == sizeof(uint32_t))
        {
            data = *((uint32_t *)p_data);
        }
        else if (len == sizeof(uint16_t))
        {
            data = (uint32_t)*((uint16_t *)p_data);
        }
        else if (len == sizeof(uint8_t))
        {
            data = (uint32_t)*p_data;
        }
        else
        {
            return OBEX_BAD_PARAMS;
        }

        status = OBEX_Add4ByteHdr((BT_HDR *)p_pkt, id, data);
        break;
    }

    return status;
}

/**
 * Function     wiced_bt_obex_add_header_utf8
 *
 *              This function adds a Unicode string header with UTF-8 string as input
 *
 *  @param[in]  p_pkt  : OBEX packet
 *  @param[in]  id     : OBEX header identifier
 *  @param[in]  p_str  : Pointer to UTF-8 string
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_add_header_utf8(uint8_t *p_pkt, wiced_bt_obex_header_identifier_t id, uint8_t *p_str)
{
    wiced_bt_obex_status_t status;
    uint16_t *p_utf16 = NULL;
    uint16_t utf16_len = 0;

    /* this function is for Unicode string header only */
    if ((id & OBEX_HI_TYPE_MASK) != OBEX_HI_TYPE_UNIC)
        return OBEX_BAD_PARAMS;

    if (p_str)
    {
        /* Check if the string is empty */
        if (((char*)p_str)[0] != '\0')
        {
            utf16_len = strlen((char *)p_str) * 2 + 2;
            p_utf16 = (UINT16 *)GKI_getbuf(utf16_len);
        }
    }

    if (p_utf16)
        utf16_len = utfc_8_to_16(p_utf16, utf16_len, p_str);
    else
        utf16_len = 0;

    status = OBEX_AddUnicodeHdr((BT_HDR *)p_pkt, OBEX_HI_NAME, p_utf16, utf16_len);

    if (p_utf16)
        GKI_freebuf(p_utf16);

    return status;
}

/**
 * Function     wiced_bt_obex_add_triplet_header
 *
 *              This function adds a header with data in triplet format
 *
 *  @param[in]  p_pkt      : OBEX packet
 *  @param[in]  id         : OBEX header identifier
 *  @param[in]  p_triplet  : Pointer to triplets
 *  @param[in]  num        : Number of triplets
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_add_triplet_header(uint8_t *p_pkt, wiced_bt_obex_header_identifier_t id,
        wiced_bt_obex_triplet_t *p_triplet, uint8_t num)
{
    return OBEX_AddTriplet((BT_HDR *)p_pkt, id, p_triplet, num);
}

/**
 * Function     wiced_bt_obex_add_byte_sequence_start
 *
 *              This is an alternative way to add a byte sequence header.  Call this
 *              function to get the starting point of the byte sequence header buffer,
 *              fill in byte sequence data, then call wiced_bt_obex_add_byte_sequence_end
 *              to finish.
 *
 *  @param[in]   p_pkt  : OBEX packet
 *  @param[out]  p_len  : Pointer to return available buffer length
 *
 *  @return     Starting point to the OBEX packet byte sequence header buffer
 *
 */
uint8_t * wiced_bt_obex_add_byte_sequence_start(uint8_t *p_pkt, uint16_t *p_len)
{
    BT_HDR  *p_bt_hdr = (BT_HDR *)p_pkt;
    uint8_t *p = (uint8_t *)(p_bt_hdr + 1) + p_bt_hdr->offset + p_bt_hdr->len + 3;

    if(*p_len > (p_bt_hdr->layer_specific - 3) || *p_len == 0)
        *p_len = p_bt_hdr->layer_specific - 3;
    return p;
}

/**
 * Function     wiced_bt_obex_add_byte_sequence_end
 *
 *              This function is called to finish adding byte sequence header by adding
 *              header identifier and length.  It is assumed that the actual value of
 *              the byte sequence has been copied into the OBEX packet.
 *
 *  @param[in]   p_pkt  : OBEX packet
 *  @param[in]   id     : OBEX header identifier
 *  @param[in]   len    : Length of data
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_add_byte_sequence_end(uint8_t *p_pkt, wiced_bt_obex_header_identifier_t id, uint16_t len)
{
    return OBEX_AddByteStrHdr((BT_HDR *)p_pkt, id, NULL, len);
}

/**
 * Function     wiced_bt_obex_read_header
 *
 *              This function reads a header from the OBEX packet.
 *
 *              For Unicode text header, this function will convert Unicode to Ascii
 *              string.  To convert Unicode to UTF-8 string, call
 *              @link wiced_bt_obex_read_header_utf8 @endlink instead.
 *              For byte sequence header, it reads byte sequence to buffer pointed to by
 *              parameter p_data.  To avoid the data copying, call
 *              @link wiced_bt_obex_find_byte_sequence_header @endlink instead.
 *              For 1 byte and 4 byte header, it reads data to buffer pointed to by p_data.
 *
 *  @param[in]      p_pkt   : OBEX packet
 *  @param[in]      id      : OBEX header identifier
 *  @param[out]     p_data  : Pointer to buffer that receives data
 *  @param[in/out]  p_len   : Input: output buffer size, Output: read data length
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_read_header(uint8_t *p_pkt, wiced_bt_obex_header_identifier_t id,
        uint8_t *p_data, uint16_t *p_len)
{
    wiced_bt_obex_status_t status = OBEX_BAD_PARAMS;
    uint16_t len;
    uint16_t *p_unicode;
    uint8_t *p_uint8;
    uint32_t data;

    switch (id & OBEX_HI_TYPE_MASK)
    {
    case OBEX_HI_TYPE_UNIC:
        {
            len = *p_len;
            p_unicode = (uint16_t *)GKI_getbuf(len * 2);
            if(p_unicode)
            {
                if (OBEX_ReadUnicodeHdr((BT_HDR *)p_pkt, id, p_unicode, &len))
                {
                    utfc_16_to_8(p_data, *p_len, p_unicode, len);
                    *p_len = len;
                    status = OBEX_SUCCESS;
                }
                GKI_freebuf(p_unicode);
            }
        }
        break;

    case OBEX_HI_TYPE_ARRAY:
        status = wiced_bt_obex_find_byte_sequence_header(p_pkt, id, &p_uint8, &len);
        if (status == OBEX_SUCCESS)
        {
            if (*p_len > len)
                *p_len = len;

            memcpy(p_data, p_uint8, *p_len);
        }
        break;

    case OBEX_HI_TYPE_BYTE:
        if(OBEX_Read1ByteHdr((BT_HDR *)p_pkt, id, p_data))
        {
            status = OBEX_SUCCESS;
        }
        break;

    case OBEX_HI_TYPE_INT:
        if (OBEX_Read4ByteHdr((BT_HDR *)p_pkt, id, (UINT32 *)&data))
        {
            if (*p_len > sizeof(uint32_t))
                *p_len = sizeof(uint32_t);

            memcpy(p_data, &data, *p_len);
            status = OBEX_SUCCESS;
        }
        break;
    }

    return status;
}

/**
 * Function     wiced_bt_obex_read_header_utf8
 *
 *              This function reads a Unicode string header with UTF-8 string as output
 *
 *  @param[in]   p_pkt    : OBEX packet
 *  @param[in]   id       : OBEX header identifier
 *  @param[out]  p_data   : Pointer to buffer that receives UTF-8 string
 *  @param[in]   max_len  : Max output size
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_read_header_utf8(uint8_t *p_pkt, wiced_bt_obex_header_identifier_t id,
        uint8_t *p_data, uint16_t max_len)
{
    wiced_bt_obex_status_t status = OBEX_BAD_PARAMS;
    uint16_t *p_unicode = (uint16_t *)GKI_getbuf((uint16_t)((max_len + 1) * 2));
    uint16_t len = max_len;

    /* this function is for Unicode string header only */
    if ((id & OBEX_HI_TYPE_MASK) != OBEX_HI_TYPE_UNIC)
        return OBEX_BAD_PARAMS;

    if (p_unicode)
    {
        status = OBEX_ReadUnicodeHdr((BT_HDR *)p_pkt, id, p_unicode, &len);
        if (status == OBEX_SUCCESS)
            utfc_16_to_8(p_data, max_len, p_unicode, len);
        GKI_freebuf(p_unicode);
    }
    return status;
}

/**
 * Function     wiced_bt_obex_find_byte_sequence_header
 *
 *              This function finds a specific byte sequence header, return its data pointer and length
 *
 *  @param[in]   p_pkt   : OBEX packet
 *  @param[in]   id      : OBEX header identifier
 *  @param[out]  p_data  : A pointer to return header data pointer
 *  @param[out]  p_len   : Return header length
 *
 *  @return @link wiced_bt_obex_status_e wiced_bt_obex_status_t @endlink
 *
 */
wiced_bt_obex_status_t wiced_bt_obex_find_byte_sequence_header(uint8_t *p_pkt, wiced_bt_obex_header_identifier_t id,
        uint8_t **p_data, uint16_t *p_len)
{
    return OBEX_ReadByteStrHdr((BT_HDR *)p_pkt, id, p_data, p_len, 0);
}

/**
 * Function     wiced_bt_obex_find_body_header
 *
 *              This function finds OBEX_HI_BODY and OBEX_HI_BODY_END headers in an
 *              OBEX packet
 *
 *  @param[in]   p_pkt   : OBEX packet
 *  @param[out]  p_body  : A pointer to return body header pointer
 *  @param[out]  p_len   : Return body header length
 *  @param[out]  p_end   : TRUE  : found a OBEX_HI_BODY_END header
 *                       : FALSE : found a OBEX_HI_BODY header or no body header was found
 *
 *  @return      0 : No body header was found
 *               1 : Found a OBEX_HI_BODY or OBEX_HI_BODY_END header
 *               2 : Found both OBEX_HI_BODY and OBEX_HI_BODY_END headers
 *
 */
uint8_t wiced_bt_obex_find_body_header(uint8_t *p_pkt, uint8_t **p_body, uint16_t *p_len, wiced_bool_t *p_end)
{
    return OBEX_ReadBodyHdr((BT_HDR *)p_pkt, p_body, p_len, p_end);
}

/**
 * Function     wiced_bt_obex_add_body_start
 *
 *              This function is called to get the address to the beginning of
 *              the byte sequence for an OBEX body header in an OBEX packet.
 *
 *  @param[in]      p_pkt   : OBEX packet
 *  @param[in/out]  p_len   : Input: location to receive the length, Output: length available to hold the body header
 *
 *  @return      The address to add body content
 *
 */
uint8_t * wiced_bt_obex_add_body_start(uint8_t *p_pkt, uint16_t *p_len)
{
    return OBEX_AddBodyStart((BT_HDR *)p_pkt, p_len);
}

/**
 * Function     wiced_bt_obex_add_body_end
 *
 *              This function is called to add the HI and the length of HV of an
 *              OBEX body header to an OBEX packet. If end is TRUE, HI is
 *              OBEX_HI_BODY_END. If FALSE, HI is OBEX_HI_BODY. It is assumed that
 *              the actual value of the body has been copied into the OBEX packet.
 *
 *  @param[in]   p_pkt   : OBEX packet
 *  @param[out]  p_body  : A pointer to return body header pointer
 *  @param[out]  p_len   : Return body header length
 *  @param[out]  p_end   : TRUE  : found a OBEX_HI_BODY_END header
 *                       : FALSE : found a OBEX_HI_BODY header or no body header was found
 *
 *  @return      void
 */
void wiced_bt_obex_add_body_end(uint8_t *p_pkt, uint8_t *p_body, uint16_t len, wiced_bool_t end)
{
    OBEX_AddBodyEnd((BT_HDR *)p_pkt, p_body, len, end);
}

/**
 * Function     wiced_bt_obex_read_header_len
 *
 *              This function is called to check the length of the specified
 *              header in the given OBEX packet
 *
 *  @param[in]  p_pkt   : OBEX packet
 *  @param[in]  id      : OBEX header identifier
 *
 *  @return     OBEX_INVALID_HDR_LEN, if the header is not in the OBEX packet.
 *              Otherwise the actual length of the header.
 */
uint16_t wiced_bt_obex_read_header_len(uint8_t *p_pkt, wiced_bt_obex_header_identifier_t id)
{
    return OBEX_ReadHdrLen((BT_HDR *)p_pkt, id);
}

const UINT8 obx_hdr_start_offset[] =
{
    OBEX_CONN_HDRS_OFFSET,       /* OBEX_CONNECT_REQ_EVT */
    OBEX_SESS_HDRS_OFFSET,       /* OBEX_SESSION_REQ_EVT */
    OBEX_DISCON_HDRS_OFFSET,     /* OBEX_DISCONNECT_REQ_EVT */
    OBEX_PUT_HDRS_OFFSET,        /* OBEX_PUT_REQ_EVT */
    OBEX_GET_HDRS_OFFSET,        /* OBEX_GET_REQ_EVT */
    OBEX_SETPATH_REQ_HDRS_OFFSET,/* OBEX_SETPATH_REQ_EVT */
    OBEX_ABORT_HDRS_OFFSET,      /* OBEX_ABORT_REQ_EVT */
    OBEX_ACTION_HDRS_OFFSET,     /* OBEX_ACTION_REQ_EVT */
    OBEX_CONN_HDRS_OFFSET,       /* OBEX_CONNECT_RSP_EVT */
    OBEX_RESPONSE_HDRS_OFFSET,   /* OBEX_SESSION_RSP_EVT */
    OBEX_RESPONSE_HDRS_OFFSET,   /* OBEX_DISCONNECT_RSP_EVT */
    OBEX_RESPONSE_HDRS_OFFSET,   /* OBEX_PUT_RSP_EVT */
    OBEX_RESPONSE_HDRS_OFFSET,   /* OBEX_GET_RSP_EVT */
    OBEX_RESPONSE_HDRS_OFFSET,   /* OBEX_SETPATH_RSP_EVT */
    OBEX_RESPONSE_HDRS_OFFSET,   /* OBEX_ABORT_RSP_EVT */
    OBEX_RESPONSE_HDRS_OFFSET    /* OBEX_ACTION_RSP_EVT */
};

/*******************************************************************************
**
** Function     obx_access_rsp_code
**
** Description  This function is used to read/change response code
**
** Returns      void.
**
*******************************************************************************/
void obx_access_rsp_code(BT_HDR *p_pkt, UINT8 *p_rsp_code)
{
    UINT8   *p = (UINT8 *)(p_pkt + 1) + p_pkt->offset;
    if(*p_rsp_code == OBEX_RSP_DEFAULT)
        *p_rsp_code = *p;
    else
        *p = *p_rsp_code;
}

/*******************************************************************************
**
** Function     obx_adjust_packet_len
**
** Description  Adjust the packet length in the OBEX packet
**
** Returns      void.
**
*******************************************************************************/
void obx_adjust_packet_len(BT_HDR *p_pkt)
{
    UINT8   *p = (UINT8 *)(p_pkt + 1) + p_pkt->offset + 1;
    UINT16_TO_BE_STREAM(p, p_pkt->len);
}

/*******************************************************************************
**
** Function     obx_read_header_len
**
** Description  ph is the beginning of an OBEX header
**
** Returns      total length of the header
**
*******************************************************************************/
UINT16 obx_read_header_len(UINT8 *ph)
{
    UINT16  len = 0;

    /*
    OBEX_TRACE_DEBUG1( "obx_read_header_len: 0x%x\n", *ph);
    */
    switch(*ph&OBEX_HI_TYPE_MASK)
    {
    case OBEX_HI_TYPE_BYTE:
        len = 2;
        break;
    case OBEX_HI_TYPE_INT:
        len = 5;
        break;
    case OBEX_HI_TYPE_ARRAY:
    case OBEX_HI_TYPE_UNIC:
        ph++;
        BE_STREAM_TO_UINT16(len, ph);
        break;
    }
    /*
    OBEX_TRACE_DEBUG1( "len:%d\n", len);
    */
    return len;
}

/*******************************************************************************
**
** Function     obx_dup_pkt
**
** Description  This function duplicate the OBEX message
**
** Returns      BT_HDR *.
**
*******************************************************************************/
BT_HDR * obx_dup_pkt(BT_HDR *p_pkt)
{
    BT_HDR *p_new;
    UINT16 size = p_pkt->len + p_pkt->offset + BT_HDR_SIZE;

    if (size < GKI_MAX_BUF_SIZE)
    {
        /* Use the largest general pool to allow challenge tags appendage */
        p_new = (BT_HDR *)GKI_getbuf(GKI_MAX_BUF_SIZE);
    }
    else
    {
        p_new = (BT_HDR *) GKI_getpoolbuf(OBEX_LRG_DATA_POOL_ID);
    }

    if (p_new)
        memcpy(p_new, p_pkt, size );

    return p_new;
}


/*******************************************************************************
**
** Function     OBEX_Add1ByteHdr
**
** Description  This function is called to add a header with type as UINT8
**              to an OBEX packet.
**
** Returns      TRUE, if the header is added successfully.
**              FALSE, if the operation failed. p_pkt is not altered.
**
*******************************************************************************/
BOOLEAN OBEX_Add1ByteHdr(BT_HDR *p_pkt, UINT8 id, UINT8 data)
{
    UINT8   *p;
    BOOLEAN status = FALSE;
    UINT16  size = 2;    /* total length added by this header - 1/hi & 1/hv */

    if(p_pkt)
    {
        p = (UINT8 *)(p_pkt+1)+p_pkt->offset+p_pkt->len;
        /* verify that the HI is of correct type and the remaining length in the packet is good */
        if( ((id & OBEX_HI_TYPE_MASK) == OBEX_HI_TYPE_BYTE) && (p_pkt->layer_specific >= size) )
        {
            *p++        = id;
            *p++        = data;

            p_pkt->len  += size;
            p_pkt->layer_specific   -= size;
            status = TRUE;
        }
    }

    return status;
}

/*******************************************************************************
**
** Function     OBEX_Add4ByteHdr
**
** Description  This function is called to add a header with type as UINT32
**              to an OBEX packet.
**
** Returns      TRUE, if the header is added successfully.
**              FALSE, if the operation failed. p_pkt is not altered.
**
*******************************************************************************/
BOOLEAN OBEX_Add4ByteHdr(BT_HDR *p_pkt, UINT8 id, UINT32 data)
{
    UINT8   *p;
    BOOLEAN status = FALSE;
    UINT16  size = 5;    /* total length added by this header - 1/hi & 4/hv */

    if(p_pkt)
    {
        p = (UINT8 *)(p_pkt+1)+p_pkt->offset+p_pkt->len;
        /* verify that the HI is of correct type and the remaining length in the packet is good */
        if( ((id & OBEX_HI_TYPE_MASK) == OBEX_HI_TYPE_INT) && (p_pkt->layer_specific >= size) )
        {
            *p++        = id;
            UINT32_TO_BE_STREAM(p, data);

            p_pkt->len  += size;
            p_pkt->layer_specific   -= size;
            status = TRUE;
        }
    }

    return status;
}

/*******************************************************************************
**
** Function     OBEX_AddByteStrStart
**
** Description  This function is called to get the address to the beginning of
**              the byte sequence for an OBEX header in an OBEX packet.
**
** Returns      The address to add the byte sequence.
**
*******************************************************************************/
UINT8 *OBEX_AddByteStrStart(BT_HDR *p_pkt, UINT16 *p_len)
{
    UINT8 *p = (UINT8 *)(p_pkt + 1) + p_pkt->offset + p_pkt->len + 3;

    WC_ASSERT(p_len);

    if(*p_len > (p_pkt->layer_specific - 3) || *p_len == 0)
        *p_len = p_pkt->layer_specific - 3;
    return p;
}

/*******************************************************************************
**
** Function     OBEX_AddByteStrHdr
**
** Description  This function is called to add a header with type as byte sequence
**              to an OBEX packet.
**
** Returns      TRUE, if the header is added successfully.
**              FALSE, if the operation failed. p_pkt is not altered.
**
*******************************************************************************/
BOOLEAN OBEX_AddByteStrHdr(BT_HDR *p_pkt, UINT8 id, UINT8 *p_data, UINT16 len)
{
    UINT8   *p;
    BOOLEAN status = FALSE;
    UINT16  size = (len+3); /* total length added by this header - 1/hi & len+2/hv */

    if(p_pkt)
    {
        p = (UINT8 *)(p_pkt+1)+p_pkt->offset+p_pkt->len;
        /* verify that the HI is of correct type and the remaining length in the packet is good */
        if( ((id & OBEX_HI_TYPE_MASK) == OBEX_HI_TYPE_ARRAY) && (p_pkt->layer_specific >= size) )
        {
            *p++        = id;
            UINT16_TO_BE_STREAM(p, size);
            if(p_data)
                memcpy(p, p_data, len);

            p_pkt->len  += size;
            p_pkt->layer_specific   -= size;
            status = TRUE;
        }
    }

    return status;
}

/*******************************************************************************
**
** Function     OBEX_AddUnicodeHdr
**
** Description  This function is called to add a header with type as Unicode string
**              to an OBEX packet.
**
** Returns      TRUE, if the header is added successfully.
**              FALSE, if the operation failed. p_pkt is not altered.
**
*******************************************************************************/
BOOLEAN OBEX_AddUnicodeHdr(BT_HDR *p_pkt, UINT8 id, UINT16 *p_data, UINT16 len)
{
    UINT8   *p;
    BOOLEAN status = FALSE;
    UINT16  size, xx;

    if(p_pkt)
    {
        p = (UINT8 *)(p_pkt+1)+p_pkt->offset+p_pkt->len;
        size = (len*OBEX_UNICODE_SIZE + 3); /* total length added by this header - 1/hi & len*OBEX_UNICODE_SIZE+2/hv */
        OBEX_TRACE_DEBUG4( "OBEX_AddUnicodeHdr len: %d, size: %d, left: %d, id: 0x%x\n",
            len, size, p_pkt->layer_specific, id );

        /* verify that the HI is of correct type and the remaining length in the packet is good */
        if( ((id & OBEX_HI_TYPE_MASK) == OBEX_HI_TYPE_UNIC) && (p_pkt->layer_specific >= size) )
        {
            *p++        = id;
            UINT16_TO_BE_STREAM(p, size);
            for(xx=0; xx<len; xx++)
            {
                UINT16_TO_BE_STREAM(p, *p_data);
                p_data++;
            }

            p_pkt->len  += size;
            p_pkt->layer_specific   -= size;
            status = TRUE;
        }
    }

    return status;
}

/* Alternate Body header functions: for non-blocking scenario */
/*******************************************************************************
**
** Function     OBEX_AddBodyStart
**
** Description  This function is called to get the address to the beginning of
**              the byte sequence for an OBEX body header in an OBEX packet.
**
** Returns      The address to add body content.
**
*******************************************************************************/
UINT8 *OBEX_AddBodyStart(BT_HDR *p_pkt, UINT16 *p_len)
{
    UINT8 *p = (UINT8 *)(p_pkt + 1) + p_pkt->offset + p_pkt->len + 3;

    WC_ASSERT(p_len);

    if(*p_len > (p_pkt->layer_specific - 3) || *p_len == 0)
        *p_len = p_pkt->layer_specific - 3;
    return p;
}

/*******************************************************************************
**
** Function     OBEX_AddBodyEnd
**
** Description  This function is called to add the HI and the length of HV of an
**              OBEX body header to an OBEX packet. If end is TRUE, HI is
**              OBEX_HI_BODY_END. If FALSE, HI is OBEX_HI_BODY. It is assumed that
**              the actual value of the body has been copied into the OBEX packet.
**
** Returns      void
**
*******************************************************************************/
void OBEX_AddBodyEnd(BT_HDR *p_pkt, UINT8 *p, UINT16 len, BOOLEAN end)
{
    UINT8 id = (end)?OBEX_HI_BODY_END:OBEX_HI_BODY;
    UINT8 *pb = (UINT8 *)(p_pkt + 1) + p_pkt->offset + p_pkt->len;
    *pb++   = id;
    len     += 3; /* 1/hi, 2/hv_len */
    UINT16_TO_BE_STREAM(pb, len);
    p_pkt->layer_specific   -= len;
    p_pkt->len              += len;
}

/*******************************************************************************
**
** Function     OBEX_AddTriplet
**
** Description  This function is called to add a header with type as byte sequence
**              to an OBEX packet.
**
** Note:        The byte sequence uses a Tag-Length-Value encoding scheme
**              These headers include: Application Parameters header
**                                     Authenticate Challenge header
**                                     Authenticate Response header
**
** Returns      TRUE, if the header is added successfully.
**              FALSE, if the operation failed. p_pkt is not altered.
**
*******************************************************************************/
BOOLEAN OBEX_AddTriplet(BT_HDR *p_pkt, UINT8 id, tOBEX_TRIPLET *p_triplet, UINT8 num)
{
    UINT8   *p = (UINT8 *)(p_pkt+1)+p_pkt->offset+p_pkt->len;
    BOOLEAN status = FALSE;
    UINT16  size = 3;/* 1/hi & len+2/hv */
    UINT8   xx;

    /* calculate the total length added by this header */
    for(xx=0; xx< num; xx++)
        size += (p_triplet[xx].len + 2);

    /* verify that the HI is of correct type and the remaining length in the packet is good */
    if( ((id & OBEX_HI_TYPE_MASK) == OBEX_HI_TYPE_ARRAY) && (p_pkt->layer_specific >= size) )
    {
        *p++    = id;
        UINT16_TO_BE_STREAM(p, size);
        for(xx=0; xx< num; xx++)
        {
            *p++    = p_triplet[xx].tag;
            *p++    = p_triplet[xx].len;
            memcpy(p, p_triplet[xx].p_array, p_triplet[xx].len);
            p       += p_triplet[xx].len;
        }
        p_pkt->len  += size;
        p_pkt->layer_specific   -= size;
        status = TRUE;
    }

    return status;
}


/*******************************************************************************
**
** Function     OBEX_CheckNext
**
** Description  This function is called to check if the given OBEX packet
**              contains the specified header.
**
** Returns      NULL, if the header is not in the OBEX packet.
**              The pointer to the specified header beginning from HI.
**
*******************************************************************************/
UINT8 * OBEX_CheckNext(BT_HDR *p_pkt, UINT8 *p_start, UINT8 id)
{
    UINT8   *p;
    UINT8   *p_res = NULL;
    UINT16  len, start, skip;
    int     remain=0;

    if(p_pkt != NULL && p_start != NULL)
    {
        p = (UINT8 *)(p_pkt+1)+p_pkt->offset;
        if(p_pkt->event <= OBEX_MAX_OFFSET_IND)
        {
            start = obx_hdr_start_offset[p_pkt->event-1];
            p++;
            BE_STREAM_TO_UINT16(len, p);
            remain  = len - start;
            p   = p - 3 + start;

            while(remain >0)
            {
                if(*p != id || p < p_start)
                {
                    skip = obx_read_header_len(p);
                    p       += skip;
                    /* Just in case this is a bad packet, make sure that remain is >= 0 */
                    if(skip && (remain > skip))
                        remain  -= skip;
                    else
                        remain = 0;
                }
                else
                {
                    p_res = p;
                    break;
                }
            }
        }
    }

    if (p_pkt)
    {
        OBEX_TRACE_DEBUG2( "OBEX_CheckNext: remain: %d len:%d\n", remain, p_pkt->len);
    }

    return p_res;
}


/*******************************************************************************
**
** Function     OBEX_CheckHdr
**
** Description  This function is called to check if the given OBEX packet
**              contains the specified header.
**
** Returns      NULL, if the header is not in the OBEX packet.
**              The pointer to the specified header beginning from HI.
**
*******************************************************************************/
UINT8 * OBEX_CheckHdr(BT_HDR *p_pkt, UINT8 id)
{
    UINT8   *p;
    UINT8   *p_res = NULL;
    UINT16  len, start, skip;
    int     remain;

    if(p_pkt != NULL)
    {
        p = (UINT8 *)(p_pkt+1)+p_pkt->offset;
        if(p_pkt->event <= OBEX_MAX_OFFSET_IND)
        {
            start = obx_hdr_start_offset[p_pkt->event-1];
            p++;
            BE_STREAM_TO_UINT16(len, p);
            remain  = len - start;
            p   = p - 3 + start;

            while(remain >0)
            {
                if(*p != id)
                {
                    skip = obx_read_header_len(p);
                    p       += skip;
                    /* Just in case this is a bad packet, make sure that remain is >= 0 */
                    if(skip && (remain > skip))
                        remain  -= skip;
                    else
                        remain = 0;
                }
                else
                {
                    p_res = p;
                    break;
                }
            }
        }
    }

    return p_res;
}

/*******************************************************************************
**
** Function     OBEX_ReadNumHdrs
**
** Description  This function is called to check the number of headers in the
**              given OBEX packet
**
** Returns      number of headers.
**
*******************************************************************************/
UINT8 OBEX_ReadNumHdrs(BT_HDR *p_pkt, UINT8 *p_num_body)
{
    UINT8   num_hdrs = 0, num_body = 0;
    UINT8   *p;
    UINT16  len, start, skip;
    int     remain = 0;

    if(p_pkt != NULL)
    {
        p = (UINT8 *)(p_pkt+1)+p_pkt->offset;
        if(p_pkt->event == 0)
        {
            /* GKI buffer just went through OBEX_HdrInit; not processed by the state machine yet */
            remain = len = p_pkt->len;
        }
        else if(p_pkt->event <= OBEX_MAX_OFFSET_IND)
        {
            start = obx_hdr_start_offset[p_pkt->event-1];
            p++;
            BE_STREAM_TO_UINT16(len, p);
            remain  = len - start;
            p   = p - 3 + start;
        }

        while(remain >0)
        {
            num_hdrs++;
            if(*p == OBEX_HI_BODY || *p == OBEX_HI_BODY_END)
                num_body++;

            skip = obx_read_header_len(p);
            p       += skip;
            /* Just in case this is a bad packet, make sure that remain is >= 0 */
            if(skip && (remain > skip))
                remain  -= skip;
            else
                remain = 0;

        }
    }
    if (p_num_body)
        *p_num_body = num_body;
    return num_hdrs;
}

/*******************************************************************************
**
** Function     OBEX_ReadHdrLen
**
** Description  This function is called to check the length of the specified
**              header in the given OBEX packet.
**
** Returns      OBEX_INVALID_HDR_LEN, if the header is not in the OBEX packet.
**              Otherwise the actual length of the header.
**
*******************************************************************************/
UINT16 OBEX_ReadHdrLen(BT_HDR *p_pkt, UINT8 id)
{
    UINT8   *p;
    UINT16  len = OBEX_INVALID_HDR_LEN;

    if( (p = OBEX_CheckHdr(p_pkt, id)) != NULL)
        len = obx_read_header_len(p);

    return len;
}

/*******************************************************************************
**
** Function     OBEX_Read1ByteHdr
**
** Description  This function is called to get the UINT8 HV of the given HI
**              in the given OBEX packet.
**
** Returns      TRUE, if the header is in the OBEX packet.
**              FALSE, otherwise.
**
*******************************************************************************/
BOOLEAN OBEX_Read1ByteHdr(BT_HDR *p_pkt, UINT8 id, UINT8 *p_data)
{
    BOOLEAN status = FALSE;
    UINT8   *p_start = OBEX_CheckHdr(p_pkt, id);

    if(p_start)
    {
        *p_data = *(++p_start);
        status = TRUE;
    }
    return status;
}

/*******************************************************************************
**
** Function     OBEX_Read4ByteHdr
**
** Description  This function is called to get the UINT32 HV of the given HI
**              in the given OBEX packet.
**
** Returns      TRUE, if the header is in the OBEX packet.
**              FALSE, otherwise.
**
*******************************************************************************/
BOOLEAN OBEX_Read4ByteHdr(BT_HDR *p_pkt, UINT8 id, UINT32 *p_data)
{
    BOOLEAN status = FALSE;
    UINT8   *p_start = OBEX_CheckHdr(p_pkt, id);

    if(p_start)
    {
        p_start++;
        BE_STREAM_TO_UINT32(*p_data, p_start);
        status = TRUE;
    }
    return status;
}

/*******************************************************************************
**
** Function     OBEX_ReadByteStrHdr
**
** Description  This function is called to get the byte sequence HV of the given
**              HI in the given OBEX packet.
**
** Returns      TRUE, if the header is in the OBEX packet.
**              FALSE, otherwise.
**
*******************************************************************************/
BOOLEAN OBEX_ReadByteStrHdr(BT_HDR *p_pkt, UINT8 id, UINT8 **p_data, UINT16 *p_len, UINT8 next)
{
    BOOLEAN status = FALSE;
    UINT8   *p_start = OBEX_CheckHdr(p_pkt, id);

    if(p_start)
    {
        next += 1;
        while(next && (id == *p_start++))
        {
            next--;
            BE_STREAM_TO_UINT16(*p_len, p_start);
            if(next == 0)
            {
                status = TRUE;
                *p_len -= 3;    /* get rid of hi and hv_len */
                *p_data = p_start;
            }
            else
                p_start = p_start + *p_len - 3;
        }
    }
    else
    {
        *p_len = 0;
    }
    return status;
}

/*******************************************************************************
**
** Function     OBEX_ReadUnicodeHdr
**
** Description  This function is called to get the Unicode HV of the given
**              HI in the given OBEX packet.
**
** Returns      TRUE, if the header is in the OBEX packet.
**              FALSE, otherwise.
**
*******************************************************************************/
BOOLEAN OBEX_ReadUnicodeHdr(BT_HDR *p_pkt, UINT8 id, UINT16 *p_data, UINT16 *p_len)
{
    BOOLEAN status = FALSE;
    UINT16  len, xx, max_len;
    UINT8   *p_start = OBEX_CheckHdr(p_pkt, id);

    if(p_start)
    {
        max_len = *p_len;
        p_start++; /* 1/hi*/
        BE_STREAM_TO_UINT16(len, p_start);
        len -= 3; /* 1/hi, 2/hv_len */
        len /= OBEX_UNICODE_SIZE; /* size in UINT16 */
        /* only convert the provided size */
        if( len > max_len)
            len = max_len;
        for(xx=0; xx<len; xx++)
        {
            BE_STREAM_TO_UINT16(*p_data, p_start);
            p_data++;
        }
        *p_len = len;
        status = TRUE;
        max_len -= len;
        while ( (p_start = OBEX_CheckNext(p_pkt, p_start, id)) != NULL && (max_len > 0))
        {
            p_start++; /* 1/hi*/
            BE_STREAM_TO_UINT16(len, p_start);
            len -= 3; /* 1/hi, 2/hv_len */
            len /= OBEX_UNICODE_SIZE; /* size in UINT16 */
            /* only conver the provided size */
            if( len > max_len)
                len = max_len;
            for(xx=0; xx<len; xx++)
            {
                BE_STREAM_TO_UINT16(*p_data, p_start);
                p_data++;
            }
            *p_len += len;
            max_len -= len;
        }
    }
    else
    {
        *p_len = 0;
    }
    return status;
}

/*******************************************************************************
**
** Function     OBEX_ReadTriplet
**
** Description  This function is called to get the Triplet HV of the given
**              HI in the given OBEX packet.
**
** Returns      TRUE, if the header is in the OBEX packet.
**              FALSE, otherwise.
**
*******************************************************************************/
BOOLEAN OBEX_ReadTriplet(BT_HDR *p_pkt, UINT8 id, tOBEX_TRIPLET *p_triplet, UINT8 *p_num)
{
    BOOLEAN status = FALSE;
    UINT8   *p_start = OBEX_CheckHdr(p_pkt, id);
    UINT16  len;
    UINT8   count = 0;

    if(p_start)
    {
        p_start++; /* 1/hi*/
        BE_STREAM_TO_UINT16(len, p_start);
        len -= 3;   /* 1/hi, 2/hv_len */
        while(len && *p_num > count)
        {
            p_triplet[count].tag = *p_start++;
            p_triplet[count].len = *p_start++;
            OBEX_TRACE_DEBUG3( "OBEX_ReadTriplet: count: %d, tag: %x, len: %d\n",
                count, p_triplet[count].tag, p_triplet[count].len);
            p_triplet[count].p_array = p_start;
            p_start += p_triplet[count].len;
            if(len > (p_triplet[count].len + 2) )
                len -= (p_triplet[count].len + 2);
            else
                len = 0;
            count++;
        }
        status = TRUE;
    }
    *p_num = count;
    return status;
}

/*******************************************************************************
**
** Function     OBEX_ReadActionIdHdr
**
** Description  This function is called to get the HV of the Action ID header
**              in the given OBEX packet.
**
** Returns      TRUE, if the header is in the OBEX packet.
**              FALSE, otherwise.
**
*******************************************************************************/
BOOLEAN OBEX_ReadActionIdHdr(BT_HDR *p_pkt, UINT8 *p_data)
{
    BOOLEAN status = FALSE;
    UINT8   *p_start = OBEX_CheckHdr(p_pkt, OBEX_HI_ACTION_ID);

    if(p_start)
    {
        p_start++;
        /* check for valid values: 0-2 */
        /* do not allow 0x80 - 0xFF (vendor extention) for now. */
        if (*p_start <= OBEX_ACTION_PERMISSION)
        {
            *p_data = *(p_start);
            status = TRUE;
        }
    }
    return status;
}

/*******************************************************************************
**
** Function     OBEX_ReadBodyHdr
**
** Description  This function is called to get the Body Header in the
**              given OBEX packet.
**
** Returns      1, if a single header is in the OBEX packet.
**              2, if a end of body header is in the OBEX packet.
**              0, (FALSE) otherwise.
**
*******************************************************************************/
UINT8 OBEX_ReadBodyHdr(BT_HDR *p_pkt, UINT8 **p_body, UINT16 *p_len, BOOLEAN *p_end)
{
    BOOLEAN status;
    UINT8   *p_body2;
    UINT16  len2;
    UINT8   num;

    *p_end  = FALSE;
    num     = OBEX_ReadByteStrHdr(p_pkt, OBEX_HI_BODY, p_body, p_len, 0);
    status  = OBEX_ReadByteStrHdr(p_pkt, OBEX_HI_BODY_END, &p_body2, &len2, 0);
    if(num == FALSE && status == TRUE)
    {
        *p_body = p_body2;
        *p_len  = len2;
        *p_end  = TRUE;
    }
    num += status;

    return num;
}

/*******************************************************************************
**
** Function     OBEX_CharToWchar
**
** Description  This function is called to convert ASCII to Unicode (UINT16).
**
** Returns      the length.
**
*******************************************************************************/
UINT16 OBEX_CharToWchar (UINT16 *w_str, char* a_str, UINT16 w_size)
{
    UINT16  result = 0;
    int     size = w_size;

    if (a_str == NULL || w_str == NULL)
        return 0;

    while (size > 0 && *a_str != '\0')
    {
        w_str[result++] = (UINT16) *a_str;
        a_str++;
        size--;
    }

    if (size > 0)
    {
        w_str[result] = 0;
    }

    return (result+1);
}

/*******************************************************************************
**
** Function     OBEX_WcharToChar
**
** Description  This function is called to convert Unicode (UINT16) to ASCII.
**
** Returns      void.
**
*******************************************************************************/
void OBEX_WcharToChar (char *a_str, UINT16* w_str, UINT16 a_size)
{
    UINT16  result = 0;
    int     size = a_size;

    if (w_str == NULL || a_str == NULL)
        return;

    while (size > 0 && *w_str != 0)
    {
        if ((*w_str & ~0xff) != 0)
        {
            result = 0;
            break;
        }

        a_str[result++] = (char) *w_str;
        ++(w_str);
        --size;
    }

    if(size)
        a_str[result] = 0;

    return;


}
