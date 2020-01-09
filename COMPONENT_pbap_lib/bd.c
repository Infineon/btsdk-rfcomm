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
**  Name:           bd.c
**
**  Description:    BD address and String services.
**
**
*****************************************************************************/

#include "wiced_bt_types.h"
#include "data_types.h"

#ifdef WICED_X
#ifndef __LONG_MAX__
#define __LONG_MAX__ 2147483647
#endif
#endif

#undef LONG_MAX
#define LONG_MAX __LONG_MAX__
/* Maximum value an `unsigned long int' can hold.  (Minimum is 0).  */
#undef ULONG_MAX
#define ULONG_MAX (LONG_MAX * 2UL + 1UL)


/*****************************************************************************
**  Constants
*****************************************************************************/

typedef wiced_bt_device_address_t BD_ADDR;

/* global constant for "any" bd addr */
const BD_ADDR bd_addr_any = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const BD_ADDR bd_addr_null= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

extern void GKI_freebuf (void *memPtr);
extern char *utl_strcpy( char *p_dst, char *p_src );
extern int utl_strlen( char *p_str );
/*****************************************************************************
**  Functions
*****************************************************************************/

void utl_freebuf(void **p)
{
    if (*p != NULL)
    {
        GKI_freebuf(*p);
        *p = NULL;
    }
}

int strcmp(const char *s1, const char *s2)
{
    for ( ; *s1 == *s2; s1++, s2++)
    if (*s1 == '\0')
        return 0;
    return ((*(unsigned char *)s1 < *(unsigned char *)s2) ? -1 : +1);
}


int utl_strncmp(const char *s1, const char *s2, int n)
{
    for ( ; n > 0; s1++, s2++, --n)
    if (*s1 != *s2)
        return ((*(unsigned char *)s1 < *(unsigned char *)s2) ? -1 : +1);
    else if (*s1 == '\0')
        return 0;
    return 0;
}

char *strchr(const char *s, int c)
{
    const char ch = c;

    for ( ; *s != ch; s++)
        if (*s == '\0')
            return 0;
    return (char *)s;
}

char * strcat(char *s1, const char *s2)
{
    utl_strcpy(&s1[utl_strlen(s1)], (char *)s2);
    return s1;
}

/*******************************************************************************
**
** Function         bdcpy
**
** Description      Copy bd addr b to a.
**
**
** Returns          void
**
*******************************************************************************/
void bdcpy(BD_ADDR a, const BD_ADDR b)
{
    int i;

    for (i = BD_ADDR_LEN; i != 0; i--)
    {
        *a++ = *b++;
    }
}

/*******************************************************************************
**
** Function         bdcmp
**
** Description      Compare bd addr b to a.
**
**
** Returns          Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
int bdcmp(const BD_ADDR a, const BD_ADDR b)
{
    int i;

    for (i = BD_ADDR_LEN; i != 0; i--)
    {
        if (*a++ != *b++)
        {
            return -1;
        }
    }
    return 0;
}

/*******************************************************************************
**
** Function         bdcmpany
**
** Description      Compare bd addr to "any" bd addr.
**
**
** Returns          Zero if a equals bd_addr_any.
**
*******************************************************************************/
int bdcmpany(const BD_ADDR a)
{
    return bdcmp(a, bd_addr_any);
}

/*******************************************************************************
**
** Function         bdsetany
**
** Description      Set bd addr to "any" bd addr.
**
**
** Returns          void
**
*******************************************************************************/
void bdsetany(BD_ADDR a)
{
    bdcpy(a, bd_addr_any);
}











/*
 * The table below is used to convert from ASCII digits to a
 * numerical equivalent.  It maps from '0' through 'z' to integers
 * (100 for non-digit characters).
 */

static char cvtIn[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,        /* '0' - '9' */
    100, 100, 100, 100, 100, 100, 100,        /* punctuation */
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,    /* 'A' - 'Z' */
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35,
    100, 100, 100, 100, 100, 100,        /* punctuation */
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,    /* 'a' - 'z' */
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35};


//unsigned long int strtoul(string, endPtr, base)
//CONST char *string;        /* String of ASCII digits, possibly
//                 * preceded by white space.  For bases
//                 * greater than 10, either lower- or
//                 * upper-case digits may be used.
//                 */
//    char **endPtr;        /* Where to store address of terminating
//                 * character, or NULL. */
//    int base;            /* Base for conversion.  Must be less
//                 * than 37.  If 0, then the base is chosen
//                 * from the leading characters of string:
//                 * "0x" means hex, "0" means octal, anything
//                 * else means decimal.
//                 */
//

//static inline int isspace(char c)
//{
//    return (c == ' ' || c == '\t' || c == '\n' || c == '\12');
//}

static inline int
isupper(char c)
{
    return (c >= 'A' && c <= 'Z');
}

static inline int
isalpha(char c)
{
    return ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'));
}


static inline int
isspace(char c)
{
    return (c == ' ' || c == '\t' || c == '\n' || c == '\12');
}

static inline int
isdigit(char c)
{
    return (c >= '0' && c <= '9');
}


unsigned long utl_strtoul(const char *nptr, char **endptr, int base)
{
     register const char *s;
     register unsigned long acc, cutoff;
     register int c;
     register int neg, any, cutlim;

     /*
      * See strtol for comments as to the logic used.
      */
     s = nptr;
     do {
         c = (unsigned char) *s++;
     } while (isspace(c));
     if (c == '-') {
         neg = 1;
         c = *s++;
     } else {
         neg = 0;
         if (c == '+')
             c = *s++;
     }
     if ((base == 0 || base == 16) && c == '0' && (*s == 'x' || *s == 'X')) {
         c = s[1];
         s += 2;
         base = 16;
     }
     if (base == 0)
         base = c == '0' ? 8 : 10;
     cutoff = ULONG_MAX / (unsigned long) base;
     cutlim = ULONG_MAX % (unsigned long) base;
     for (acc = 0, any = 0;; c = (unsigned char) *s++) {
         if (isdigit(c))
             c -= '0';
         else if (isalpha(c))
             c -= isupper(c) ? 'A' - 10 : 'a' - 10;
         else
             break;
         if (c >= base)
             break;
         if (any < 0)
             continue;
         if ((acc > cutoff || acc == cutoff) && c > cutlim) {
             any = -1;
             acc = ULONG_MAX;
         } else {
             any = 1;
             acc *= (unsigned long) base;
             acc += c;
         }
     }
     if (neg && any > 0)
         acc = -acc;
     if (endptr != 0)
         *endptr = (char *) (any ? s - 1 : nptr);
     return (acc);
 }



//unsigned long strtoul(nptr, endptr, base)
//    const char *nptr;
//    char **endptr;
//    register int base;
//{
//    register const char *s = nptr;
//    register unsigned long acc;
//    register int c;
//    register unsigned long cutoff;
//    register int neg = 0, any, cutlim;
//
//    /*
//     * See strtol for comments as to the logic used.
//     */
//    do {
//        c = *s++;
//    } while (isspace(c));
//    if (c == '-') {
//        neg = 1;
//        c = *s++;
//    } else if (c == '+')
//        c = *s++;
//    if ((base == 0 || base == 16) &&
//        c == '0' && (*s == 'x' || *s == 'X')) {
//        c = s[1];
//        s += 2;
//        base = 16;
//    } else if ((base == 0 || base == 2) &&
//        c == '0' && (*s == 'b' || *s == 'B')) {
//        c = s[1];
//        s += 2;
//        base = 2;
//    }
//    if (base == 0)
//        base = c == '0' ? 8 : 10;
//    cutoff = (unsigned long)ULONG_MAX / (unsigned long)base;
//    cutlim = (unsigned long)ULONG_MAX % (unsigned long)base;
//    for (acc = 0, any = 0;; c = *s++) {
//        if (isdigit(c))
//            c -= '0';
//        else if (isalpha(c))
//            c -= isupper(c) ? 'A' - 10 : 'a' - 10;
//        else
//            break;
//        if (c >= base)
//            break;
//        if (any < 0 || acc > cutoff || acc == cutoff && c > cutlim)
//            any = -1;
//        else {
//            any = 1;
//            acc *= base;
//            acc += c;
//        }
//    }
//    if (any < 0) {
//        acc = ULONG_MAX;
////        errno = ERANGE;
//    } else if (neg)
//        acc = -acc;
//    if (endptr != 0)
//        *endptr = (char *)(any ? s - 1 : nptr);
//    return (acc);
//}
//



//
//unsigned long strtoul(const char *string,  char **endPtr, int base)
//{
//    register const char *p;
//    register unsigned long int result = 0;
//    register unsigned digit;
//    int anyDigits = 0;
//    int negative=0;
//    int overflow=0;
//
//    /*
//     * Skip any leading blanks.
//     */
//
//    p = string;
//    while (isspace((UINT8)(*p))) {
//    p += 1;
//    }
//    if (*p == '-') {
//        negative = 1;
//        p += 1;
//    } else {
//        if (*p == '+') {
//            p += 1;
//        }
//    }
//
//    /*
//     * If no base was provided, pick one from the leading characters
//     * of the string.
//     */
//
//    if (base == 0)
//    {
//    if (*p == '0') {
//        p += 1;
//        if ((*p == 'x') || (*p == 'X')) {
//        p += 1;
//        base = 16;
//        } else {
//
//        /*
//         * Must set anyDigits here, otherwise "0" produces a
//         * "no digits" error.
//         */
//
//        anyDigits = 1;
//        base = 8;
//        }
//    }
//    else base = 10;
//    } else if (base == 16) {
//
//    /*
//     * Skip a leading "0x" from hex numbers.
//     */
//
//    if ((p[0] == '0') && ((p[1] == 'x') || (p[1] == 'X'))) {
//        p += 2;
//    }
//    }
//
//    /*
//     * Sorry this code is so messy, but speed seems important.  Do
//     * different things for base 8, 10, 16, and other.
//     */
//
//    if (base == 8) {
//    unsigned long maxres = ULONG_MAX >> 3;
//    for ( ; ; p += 1) {
//        digit = *p - '0';
//        if (digit > 7) {
//        break;
//        }
//        if (result > maxres) { overflow = 1; }
//        result = (result << 3);
//        if (digit > (ULONG_MAX - result)) { overflow = 1; }
//        result += digit;
//        anyDigits = 1;
//    }
//    } else if (base == 10) {
//    unsigned long maxres = ULONG_MAX / 10;
//    for ( ; ; p += 1) {
//        digit = *p - '0';
//        if (digit > 9) {
//        break;
//        }
//        if (result > maxres) { overflow = 1; }
//        result *= 10;
//        if (digit > (ULONG_MAX - result)) { overflow = 1; }
//        result += digit;
//        anyDigits = 1;
//    }
//    } else if (base == 16) {
//    unsigned long maxres = ULONG_MAX >> 4;
//    for ( ; ; p += 1) {
//        digit = *p - '0';
//        if (digit > ('z' - '0')) {
//        break;
//        }
//        digit = cvtIn[digit];
//        if (digit > 15) {
//        break;
//        }
//        if (result > maxres) { overflow = 1; }
//        result = (result << 4);
//        if (digit > (ULONG_MAX - result)) { overflow = 1; }
//        result += digit;
//        anyDigits = 1;
//    }
//    } else if ( base >= 2 && base <= 36 ) {
//    unsigned long maxres = ULONG_MAX / base;
//    for ( ; ; p += 1) {
//        digit = *p - '0';
//        if (digit > ('z' - '0')) {
//        break;
//        }
//        digit = cvtIn[digit];
//        if (digit >= ( (unsigned) base )) {
//        break;
//        }
//        if (result > maxres) { overflow = 1; }
//        result *= base;
//        if (digit > (ULONG_MAX - result)) { overflow = 1; }
//        result += digit;
//        anyDigits = 1;
//    }
//    }
//
//    /*
//     * See if there were any digits at all.
//     */
//
//    if (!anyDigits) {
//    p = string;
//    }
//
//    if (endPtr != 0) {
//    /* unsafe, but required by the strtoul prototype */
//    *endPtr = (char *) p;
//    }
//
//    if (overflow) {
//    //errno = ERANGE;
//    return ULONG_MAX;
//    }
//    if (negative) {
//    return -result;
//    }
//    return result;
//}
//
