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

/*****************************************************************************
**
**  Name:           utfc.c
**
**  Description:    UTF conversion utilities.
**
**
*****************************************************************************/

#include <string.h>
#include "wiced_bt_types.h"
#include "utfc.h"

/* Based on code from Unicode, Inc:
 *
 * Copyright 2001-2003 Unicode, Inc.
 *
 * Limitations on Rights to Redistribute This Code
 *
 * Unicode, Inc. hereby grants the right to freely use the information
 * supplied in this file in the creation of products supporting the
 * Unicode Standard, and to make copies of this file in any form
 * for internal or external distribution as long as this notice
 * remains attached.
 */

/*******************************************************************************
** Constants
*******************************************************************************/

#define UTFC_8_MASK             0xBF
#define UTFC_8_MARK             0x80

#define UTFC_SUR_HIGH_START     0xD800
#define UTFC_SUR_HIGH_END       0xDBFF
#define UTFC_SUR_LOW_START      0xDC00
#define UTFC_SUR_LOW_END        0xDFFF

/* Lookup table for length of UTF-8 byte sequence based on upper four bits
** of first byte. Illegal values give length zero.
*/
static const UINT8 utfc_8_seq_len[] = {1, 1, 1, 1, 1, 1, 1, 1,
                                       0, 0, 0, 0, 2, 2, 3, 4};

/* Magic values subtracted from a buffer value during UTF-8 conversion.
** This table contains as many values as there might be trailing bytes
** in a UTF-8 sequence.
**/
static const UINT32 utfc_8_offset[] = {0x00000000, 0x00003080,
                                       0x000E2080, 0x03C82080};

static const UINT8 utfc_8_first_byte[] = {0x00, 0x00, 0xC0, 0xE0, 0xF0};

/*******************************************************************************
**
** Function         utfc_16_to_8
**
** Description      Convert a UTF-16 array to a null-terminated UTF-8 string.
**                  Illegal characters are skipped.
**
** Returns          Length of UTF-8 string in bytes.
**
*******************************************************************************/
UINT16 utfc_16_to_8(UINT8 *p_utf8, UINT16 utf8_len, UINT16 *p_utf16, UINT16 utf16_len)
{
    UINT32  ch, ch2;
    UINT16  len = 0;
    UINT8   seq_len;

    /* sanity check destination buffer len */
    if (utf8_len == 0)
    {
        /* set null */
        *p_utf8 = 0;
        return len;
    }

    /* save space for null */
    utf8_len--;

    while (utf16_len-- > 0)
    {
        ch = (UINT32) *p_utf16++;

        /* if we have a surrogate pair, convert to UTF-32 first */
        if (ch >= UTFC_SUR_HIGH_START && ch <= UTFC_SUR_HIGH_END)
        {
            /* if not enough characters we're done */
            if (utf16_len == 0)
            {
                break;
            }

            /* get next character */
            ch2 = *p_utf16++;
            utf16_len--;

            /* if it's a low surrogate, convert to UTF-32 */
            if (ch2 >= UTFC_SUR_LOW_START && ch2 <= UTFC_SUR_LOW_END)
            {
                ch = ((ch - UTFC_SUR_HIGH_START) << 10) +
                      (ch2 - UTFC_SUR_LOW_START) + 0x00010000;
            }
            else
            {
                /* illegal UTF-16 sequence, skip it */
                continue;
            }
        }

        /* Figure out how many bytes the result will require */
        if (ch < 0x00000080)
            seq_len = 1;
        else if (ch < 0x00000800)
            seq_len = 2;
        else if (ch < 0x00010000)
            seq_len = 3;
        else
            seq_len = 4;

        /* if sequence doesn't fit we're done */
        if (utf8_len < len + seq_len)
        {
            break;
        }

        /* build UTF-8 sequence */
        switch (seq_len)
        {   /* note: everything falls through. */
            case 4: p_utf8[3] = (UINT8) ((ch | UTFC_8_MARK) & UTFC_8_MASK); ch >>= 6;
            case 3: p_utf8[2] = (UINT8) ((ch | UTFC_8_MARK) & UTFC_8_MASK); ch >>= 6;
            case 2: p_utf8[1] = (UINT8) ((ch | UTFC_8_MARK) & UTFC_8_MASK); ch >>= 6;
            case 1: p_utf8[0] = (UINT8)  (ch | utfc_8_first_byte[seq_len]);
        }

        /* converted value is a null we're done */
        if (*p_utf8 == 0)
        {
            break;
        }

        p_utf8 += seq_len;
        len += seq_len;
    }

    /* set null */
    *p_utf8 = 0;

    return len;
}

/*******************************************************************************
**
** Function         utfc_8_to_16
**
** Description      Convert a null-terminated UTF-8 string to a UTF-16 array.
**                  Illegal characters are skipped.  The UTF-16 array is
**                  appended with a zero (null) character.
**
** Returns          Length of UTF-16 array including null character.
**
*******************************************************************************/
UINT16 utfc_8_to_16(UINT16 *p_utf16, UINT16 utf16_len, UINT8 *p_utf8)
{
    UINT32  ch;
    UINT8   *p_end;
    UINT16  *p;
    UINT8   seq_len;

    /* sanity check destination buffer len */
    if (utf16_len == 0)
    {
        *p_utf16 = 0;
        return 0;
    }

    /* save space for null */
    utf16_len--;

    p = p_utf16;
    p_end = (UINT8 *) p_utf8 + strlen((char *) p_utf8);

    while (*p_utf8)
    {
        /* get sequence length; skip if illegal */
        if ((seq_len = utfc_8_seq_len[*p_utf8 >> 4]) == 0)
        {
            p_utf8++;
            continue;
        }

        /* make sure sequence doesn't extend past end of UTF-8 buffer */
        if (p_utf8 + seq_len > p_end)
        {
            break;
        }

        /* construct UTF-32 character from sequence */
        ch = 0;
        switch (seq_len)
        {   /* note: everything falls through. */
            case 4: ch += *p_utf8++; ch <<= 6;
            case 3: ch += *p_utf8++; ch <<= 6;
            case 2: ch += *p_utf8++; ch <<= 6;
            case 1: ch += *p_utf8++;
        }
        ch -= utfc_8_offset[seq_len - 1];

        if (ch <= 0x0000FFFF)
        {
            /* UTF-16 surrogate values are illegal in UTF-32 */
            if (ch >= UTFC_SUR_HIGH_START && ch <= UTFC_SUR_LOW_END)
            {
                continue;
            }

            /* make sure fits */
            if (p - p_utf16 == utf16_len)
            {
                break;
            }

            *p++ = (UINT16) ch;
        }
        else if (ch < 0x0010FFFF)
        {
            /* make sure fits */
            if ((p - p_utf16) == (utf16_len - 1))
            {
                break;
            }

            ch -= 0x00010000;
            *p++ = (UINT16) ((ch >> 10) + UTFC_SUR_HIGH_START);
            *p++ = (UINT16) ((ch & 0x000003FF) + UTFC_SUR_LOW_START);
        }
    }

    /* set null */
   *p++ = 0;

    return (UINT16) (p - p_utf16);
}
