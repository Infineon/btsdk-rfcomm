/*
 * Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
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

#include "wiced_bt_types.h"

typedef wiced_bt_device_address_t BD_ADDR;
extern void GKI_freebuf (void *memPtr);

void bdcpy(BD_ADDR a, const BD_ADDR b)
{
    int i;

    for (i = BD_ADDR_LEN; i != 0; i--)
    {
        *a++ = *b++;
    }
}

void utl_freebuf(void **p)
{
    if (*p != NULL)
    {
        GKI_freebuf(*p);
        *p = NULL;
    }
}

char *utl_strrchr(char *s, int c)
{
    char* ret=0;
    do {
        if( *s == (char)c )
            ret=s;
    } while(*s++);
    return ret;
}

char utls_toupper(char c)
{
    if( c>='a' && c<='z')
        return (c = c +'A' - 'a');
    else
        return c;
}

BOOLEAN utl_check_utf8 (char *string, uint16_t max_len)
{
    unsigned int i;
    char c, d;

    i = 0;
    c = string[i];

    while (c)
    {
        /* If b7 of 1'st byte set, this is a multi-byte sequences */
        if (c & 0x80)
        {
            /* In multi-byte sequences b7 and b6 must be set */
            if (!(c & 0x40))
            {
                string[i] = 0;
                return FALSE;
            }

            if ((c & 0xf0) == 0xf0)
            {   /* modified utf-8 cannot accept 1111xxxx */
                string[i] = 0;
                return FALSE;
            }

            if (++i >= max_len)
            {
                string[--i] = 0;
                return FALSE;
            }

            /* 2'nd byte */
            d = string[i];

            if ((d & 0xC0) != 0x80)
            {   /* utf-8 accepts only 10xxxxxx as 2'nd byte */
                string[--i] = 0;
                return FALSE;
            }

            /* If a 3'rd byte is present (b5 set) */
            if (c & 0x20)
            {
                if (++i >= max_len)
                {
                    i -= 2;
                    string[i] = 0;
                    return FALSE;
                }

                /* 3'rd byte */
                d = string[i];
                if ((d & 0xC0) != 0x80)
                {   /* utf-8 accepts only 10xxxxxx as 3'rd byte */
                    i -= 2;
                    string[i] = 0;
                    return FALSE;
                }

                /* If a fourth byte is present (b4 set) */
                if (c & 0x10)
                {
                    if (++i >= max_len)
                    {
                        i -= 3;
                        string[i] = 0;
                        return FALSE;
                    }

                    /* 4'th byte */
                    d = string[i];
                    if (((d & 0xC0) != 0x80) || (c & 0x8))
                    {   /* utf-8 accepts only 10xxxxxx as 4'th byte. 5 bytes (or more) not supported */
                        i -= 3;
                        string[i] = 0;
                        return FALSE;
                    }

                }
            }
        }

        if (++i >= max_len)
            return TRUE;

        c = string[i];
    }

    return TRUE;
}
