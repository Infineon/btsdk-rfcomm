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

/*****************************************************************************
**
**  Name:         obx_md5.c
**
**  File:         OBEX Authentication related functions
**
**
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "obx_int.h"


#define BEFORE_SIZE 64

/*
* This code implements the MD5 message-digest algorithm.
* The algorithm is due to Ron Rivest. This code was
* written by Colin Plumb in 1993, no copyright is claimed.
* This code is in the public domain; do with it what you wish.
*
* Equivalent code is available from RSA Data Security, Inc.
* This code has been tested against that, and is equivalent,
* except that you don't need to include two pages of legalese
* with every copy.
*
* To compute the message digest of a chunk of bytes, declare an
* MD5Context structure, pass it to MD5Init, call MD5Update as
* needed on buffers full of bytes, and then call MD5Final, which
* will fill a supplied 16-byte array with the digest.
*/
typedef UINT32 word32;
typedef UINT8  byte;
struct xMD5Context {
    word32 buf[4];
    word32 bytes[2];
    word32 in[16];
};

static void obx_md5_transform(word32 buf[4], word32 const in[16]);

/*
* Shuffle the bytes into little-endian order within words, as per the
* MD5 spec. Note: this code works regardless of the byte order.
*/
static void obx_byte_swap(word32 *buf, unsigned words)
{
    byte *p = (byte *)buf;
    do {
        *buf++ = (word32)((unsigned)p[3] << 8 | p[2]) << 16
               | (word32)((unsigned)p[1] << 8 | p[0]);
        p += 4;
    } while (--words);
}

/*
* Start MD5 accumulation. Set bit count to 0 and buffer to mysterious
* initialization constants.
*/
static void obx_md5_init(struct xMD5Context *ctx)
{
    ctx->buf[0] = 0x67452301;
    ctx->buf[1] = 0xefcdab89;
    ctx->buf[2] = 0x98badcfe;
    ctx->buf[3] = 0x10325476;
    ctx->bytes[0] = 0;
    ctx->bytes[1] = 0;
}

/*
* Update context to reflect the concatenation of another buffer full
* of bytes.
*/
static void obx_md5_update(struct xMD5Context *ctx, byte const *buf, int len)
{
    word32 t;
    /* Update byte count */
    t = ctx->bytes[0];
    if ((ctx->bytes[0] = t + len) < t)
        ctx->bytes[1]++; /* Carry from low to high */
    t = 64 - (t & 0x3f); /* Space avail in ctx->in (at least 1) */
    if ((unsigned)t > (unsigned)len)
    {
        memcpy ((byte *)ctx->in + 64 - (unsigned)t, buf, len);
        return;
    }
    /* First chunk is an odd size */
    memcpy (ctx->in + (64 - (unsigned)t), buf, (unsigned)t);
    obx_byte_swap(ctx->in, 16);
    obx_md5_transform(ctx->buf, ctx->in);
    buf += (unsigned)t;
    len -= (unsigned)t;
    /* Process data in 64-byte chunks */
    while (len >= 64)
    {
        /* coverity[access_dbuff_const] */
        memcpy (ctx->in, buf, 64);
        obx_byte_swap(ctx->in, 16);
        obx_md5_transform(ctx->buf, ctx->in);
        buf += 64;
        len -= 64;
    }
    /* Handle any remaining bytes of data. */
    memcpy (ctx->in, buf, len);
}

/*
* Final wrapup - pad to 64-byte boundary with the bit pattern
* 1 0* (64-bit count of bits processed, MSB-first)
*/
static void obx_md5_final(byte digest[16], struct xMD5Context *ctx)
{
    int count = (int)(ctx->bytes[0] & 0x3f); /* Bytes in ctx->in */
    byte *p = (byte *)ctx->in + count; /* First unused byte */
    /* Set the first char of padding to 0x80. There is always room.*/
    *p++ = 0x80;
    /* Bytes of padding needed to make 56 bytes (-8..55) */
    count = 56 - 1 - count;
    if (count < 0) /* Padding forces an extra block */
    {
        memset (p, 0, count+8);
        obx_byte_swap(ctx->in, 16);
        obx_md5_transform(ctx->buf, ctx->in);
        p = (byte *)ctx->in;
        count = 56;
    }
    memset (p, 0, count+8);
    obx_byte_swap(ctx->in, 14);
    /* Append length in bits and transform */
    ctx->in[14] = ctx->bytes[0] << 3;
    ctx->in[15] = ctx->bytes[1] << 3 | ctx->bytes[0] >> 29;
    obx_md5_transform(ctx->buf, ctx->in);
    obx_byte_swap(ctx->buf, 4);
    memcpy (digest, ctx->buf, 16);
    memset (ctx, 0, sizeof(*ctx));
}

/* The four core functions - F1 is optimized somewhat */
/* #define F1(x, y, z) (x & y | ~x & z) */
#define F1(x, y, z) (z ^ (x & (y ^ z)))
#define F2(x, y, z) F1(z, x, y)
#define F3(x, y, z) (x ^ y ^ z)
#define F4(x, y, z) (y ^ (x | ~z))

/* This is the central step in the MD5 algorithm. */
#define MD5STEP(f,w,x,y,z,in,s) \
(w += f(x,y,z) + in, w = (w<<s | w>>(32-s)) + x)

/*
* The core of the MD5 algorithm, this alters an existing MD5 hash to
* reflect the addition of 16 longwords of new data. MD5Update blocks
* the data and converts bytes into longwords for this routine.
*/
static const word32 obx_md5_f1 [] =
{
    0xd76aa478,
    0xe8c7b756,
    0x242070db,
    0xc1bdceee,

    0xf57c0faf,
    0x4787c62a,
    0xa8304613,
    0xfd469501,

    0x698098d8,
    0x8b44f7af,
    0xffff5bb1,
    0x895cd7be,

    0x6b901122,
    0xfd987193,
    0xa679438e,
    0x49b40821
};

static const word32 obx_md5_f2 [] =
{
    0xf61e2562,
    0xc040b340,
    0x265e5a51,
    0xe9b6c7aa,

    0xd62f105d,
    0x02441453,
    0xd8a1e681,
    0xe7d3fbc8,

    0x21e1cde6,
    0xc33707d6,
    0xf4d50d87,
    0x455a14ed,

    0xa9e3e905,
    0xfcefa3f8,
    0x676f02d9,
    0x8d2a4c8a
};

static const word32 obx_md5_f3 [] =
{
    0xfffa3942,
    0x8771f681,
    0x6d9d6122,
    0xfde5380c,

    0xa4beea44,
    0x4bdecfa9,
    0xf6bb4b60,
    0xbebfbc70,

    0x289b7ec6,
    0xeaa127fa,
    0xd4ef3085,
    0x04881d05,

    0xd9d4d039,
    0xe6db99e5,
    0x1fa27cf8,
    0xc4ac5665
};

static const word32 obx_md5_f4 [] =
{
    0xf4292244,
    0x432aff97,
    0xab9423a7,
    0xfc93a039,

    0x655b59c3,
    0x8f0ccc92,
    0xffeff47d,
    0x85845dd1,

    0x6fa87e4f,
    0xfe2ce6e0,
    0xa3014314,
    0x4e0811a1,

    0xf7537e82,
    0xbd3af235,
    0x2ad7d2bb,
    0xeb86d391
};

static const UINT8 obx_md5_a [] =
{
    1,
    2,
    3,
    0,
    1,
    2,
    3
};

static const word32 obx_md5_var1 [] =
{
    7,
    12,
    17,
    22
};

static const word32 obx_md5_var2 [] =
{
    5,
    9,
    14,
    20
};

static const word32 obx_md5_var3 [] =
{
    4,
    11,
    16,
    23
};

static const word32 obx_md5_var4 [] =
{
    6,
    10,
    15,
    21
};

static void obx_md5_transform(word32 buf[4], word32 const in[16])
{
    int     xx, yy, zz, i, j, k;
    word32  a[4];

    a[0] = buf[0];
    a[1] = buf[1];
    a[2] = buf[2];
    a[3] = buf[3];

    yy = 0;
    for(xx=0; xx<4; xx++)
    {
        j  = 3;
        for(i=0; i < 4; i++)
        {
            k = j--;
            /*
            OBEX_TRACE_DEBUG4( "f1 a: %d, yy: %d, inc: 0x%x, var: %d",
                obx_md5_a[k], yy, obx_md5_f1[yy], obx_md5_var1[i]);
             */
            MD5STEP(F1, a[obx_md5_a[k]], a[obx_md5_a[k+1]], a[obx_md5_a[k+2]], a[obx_md5_a[k+3]],
                    in[yy] + obx_md5_f1[yy], obx_md5_var1[i]);
            yy++;
        }
    }

    yy = 1;
    zz = 0;
    for(xx=0; xx<4; xx++)
    {
        j  = 3;
        for(i=0; i < 4; i++)
        {
            k = j--;
            /*
            OBEX_TRACE_DEBUG4( "f2 a: %d, yy: %d, inc: 0x%x, var: %d",
                obx_md5_a[k], yy, obx_md5_f2[zz], obx_md5_var2[i]);
             */
            MD5STEP(F2, a[obx_md5_a[k]], a[obx_md5_a[k+1]], a[obx_md5_a[k+2]], a[obx_md5_a[k+3]],
                    in[yy] + obx_md5_f2[zz++], obx_md5_var2[i]);
            yy += 5;
            yy %= 16;
        }
    }

    yy = 5;
    zz = 0;
    for(xx=0; xx<4; xx++)
    {
        j  = 3;
        for(i=0; i < 4; i++)
        {
            k = j--;
            /*
            OBEX_TRACE_DEBUG4( "f3 a: %d, yy: %d, inc: 0x%x, var: %d",
                obx_md5_a[k], yy, obx_md5_f3[zz], obx_md5_var3[i]);
             */
            MD5STEP(F3, a[obx_md5_a[k]], a[obx_md5_a[k+1]], a[obx_md5_a[k+2]], a[obx_md5_a[k+3]],
                    in[yy] + obx_md5_f3[zz++], obx_md5_var3[i]);
            yy += 3;
            yy %= 16;
        }
    }


    yy = 0;
    zz = 0;
    for(xx=0; xx<4; xx++)
    {
        j  = 3;
        for(i=0; i < 4; i++)
        {
            k = j--;
            /*
            OBEX_TRACE_DEBUG4( "f4 a: %d, yy: %d, inc: 0x%x, var: %d",
                obx_md5_a[k], yy, obx_md5_f4[zz], obx_md5_var4[i]);
             */
            MD5STEP(F4, a[obx_md5_a[k]], a[obx_md5_a[k+1]], a[obx_md5_a[k+2]], a[obx_md5_a[k+3]],
                    in[yy] + obx_md5_f4[zz++], obx_md5_var4[i]);
            yy += 7;
            yy %= 16;
        }
    }

    buf[0] += a[0];
    buf[1] += a[1];
    buf[2] += a[2];
    buf[3] += a[3];
}

/*******************************************************************************
**
** Function     OBEX_MD5
**
** Description  This function is called to run the MD5 algorithm.
**
** Returns      void
**
*******************************************************************************/
static void OBEX_MD5(void *digest, UINT8 *nonce, UINT8 * password, int password_len)
{
    struct xMD5Context context;
    UINT8  before[BEFORE_SIZE];

    memcpy(before, nonce, OBEX_NONCE_SIZE);
    before[OBEX_NONCE_SIZE] = ':';
    memcpy(before + OBEX_NONCE_SIZE + 1, password, password_len);
    /*
    scru_dump_hex (before, "before", OBEX_NONCE_SIZE + 1 + password_len, TRACE_LAYER_OBEX, TRACE_TYPE_GENERIC);
    */

    obx_md5_init(&context);
    obx_md5_update(&context, (byte const *)before, OBEX_NONCE_SIZE + 1 + password_len);
    obx_md5_final((byte *)digest, &context);
    /*
    scru_dump_hex (digest, "after", 16, TRACE_LAYER_OBEX, TRACE_TYPE_GENERIC);
    */
}

/*******************************************************************************
**
** Function     obx_session_id
**
** Description  This function is called to run the MD5 algorithm to create a
**              session id.
**
** Returns      void
**
*******************************************************************************/
void obx_session_id(UINT8 *p_sess_id, UINT8 *p_cl_addr, UINT8 * p_cl_nonce, int cl_nonce_len,
                    UINT8 *p_sr_addr, UINT8 * p_sr_nonce, int sr_nonce_len)
{
    struct xMD5Context context;
    UINT8  before[BEFORE_SIZE];
    UINT8  *p = before;
    UINT8  len;

    memcpy(p, p_cl_addr, BD_ADDR_LEN);
    p += BD_ADDR_LEN;
    memcpy(p, p_cl_nonce, cl_nonce_len);
    p += cl_nonce_len;
    memcpy(p, p_sr_addr, BD_ADDR_LEN);
    p += BD_ADDR_LEN;
    memcpy(p, p_sr_nonce, sr_nonce_len);
    p += sr_nonce_len;
    /*
    scru_dump_hex (before, "before", OBEX_NONCE_SIZE + 1 + password_len, TRACE_LAYER_OBEX, TRACE_TYPE_GENERIC);
    */

    len = BD_ADDR_LEN + cl_nonce_len + BD_ADDR_LEN + sr_nonce_len;
    obx_md5_init(&context);
    obx_md5_update(&context, (byte const *)before, len);
    obx_md5_final((byte *)p_sess_id, &context);
    /*
    scru_dump_hex (p_sess_id, "after", 16, TRACE_LAYER_OBEX, TRACE_TYPE_GENERIC);
    */
}
