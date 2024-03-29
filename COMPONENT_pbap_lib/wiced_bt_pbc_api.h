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
**  Name:           wiced_bt_pbc_api.h
**
**  Description:    This is the public interface file for the Phone Book Access Client
**                  subsystem of BTA, Widcomm's
**                  Bluetooth application layer for mobile phones.
**
**
*****************************************************************************/
#ifndef WICED_BT_PBC_API_H
#define WICED_BT_PBC_API_H

#include "wiced_bt_types.h"
#include "wiced_result.h"


/*****************************************************************************
**  Constants and data types
*****************************************************************************/

/**************************
**  Client Definitions
***************************/
/* Extra Debug Code */
#ifndef WICED_BT_PBC_DEBUG
#define WICED_BT_PBC_DEBUG           TRUE
#endif

#define WICED_BT_PBC_FLAG_NONE       0
#define WICED_BT_PBC_FLAG_BACKUP     1

typedef UINT8 wiced_bt_pbc_flag_t;


#define WICED_BT_PBC_PASSWORD       "0000"
#define WICED_BT_PBC_AUTH_DIGEST_SIZE 16
#define WICED_BT_PBC_AUTH_FIELD_SIZE 27



/* Client supported feature bits */
#define WICED_BT_PBC_SUP_FEA_DOWNLOADING                         0x00000001      /* Downloading */
#define WICED_BT_PBC_SUP_FEA_BROWSING                            0x00000002      /* Browsing */
#define WICED_BT_PBC_SUP_FEA_DATABASE_ID                         0x00000004      /* Database identifier */
#define WICED_BT_PBC_SUP_FEA_FOLDER_VER_COUNTER                  0x00000008      /* Folder version counter */
#define WICED_BT_PBC_SUP_FEA_VCARD_SELECTING                     0x00000010      /* Vcard selecting */
#define WICED_BT_PBC_SUP_FEA_ENH_MISSED_CALLS                    0x00000020      /* Enhanced missed calls */
#define WICED_BT_PBC_SUP_FEA_UCI_VCARD_FIELD                     0x00000040      /* UCI Vcard field */
#define WICED_BT_PBC_SUP_FEA_UID_VCARD_FIELD                     0x00000080      /* UID Vcard field */
#define WICED_BT_PBC_SUP_FEA_CONTACT_REF                         0x00000100      /* Contact Referencing */
#define WICED_BT_PBC_SUP_FEA_DEF_CONTACT_IMAGE_FORMAT            0x00000200      /* Default contact image format */

typedef UINT32 wiced_bt_pbc_sup_fea_mask_t;

#define WICED_BT_PBC_FILTER_VERSION              ((UINT64)1<<0)  /* Version */
#define WICED_BT_PBC_FILTER_FN                   ((UINT64)1<<1)  /* Formatted Name */
#define WICED_BT_PBC_FILTER_N                    ((UINT64)1<<2)  /* Structured Presentation of Name */
#define WICED_BT_PBC_FILTER_PHOTO                ((UINT64)1<<3)  /* Associated Image or Photo */
#define WICED_BT_PBC_FILTER_BDAY                 ((UINT64)1<<4)  /* Birthday */
#define WICED_BT_PBC_FILTER_ADR                  ((UINT64)1<<5)  /* Delivery Address */
#define WICED_BT_PBC_FILTER_LABEL                ((UINT64)1<<6)  /* Delivery */
#define WICED_BT_PBC_FILTER_TEL                  ((UINT64)1<<7)  /* Telephone Number */
#define WICED_BT_PBC_FILTER_EMAIL                ((UINT64)1<<8)  /* Electronic Mail Address */
#define WICED_BT_PBC_FILTER_MAILER               ((UINT64)1<<9)  /* Electronic Mail */
#define WICED_BT_PBC_FILTER_TZ                   ((UINT64)1<<10)  /* Time Zone */
#define WICED_BT_PBC_FILTER_GEO                  ((UINT64)1<<11) /* Geographic Position */
#define WICED_BT_PBC_FILTER_TITLE                ((UINT64)1<<12) /* Job */
#define WICED_BT_PBC_FILTER_ROLE                 ((UINT64)1<<13) /* Role within the Organization */
#define WICED_BT_PBC_FILTER_LOGO                 ((UINT64)1<<14) /* Organization Logo */
#define WICED_BT_PBC_FILTER_AGENT                ((UINT64)1<<15) /* vCard of Person Representing */
#define WICED_BT_PBC_FILTER_ORG                  ((UINT64)1<<16) /* Name of Organization */
#define WICED_BT_PBC_FILTER_NOTE                 ((UINT64)1<<17) /* Comments */
#define WICED_BT_PBC_FILTER_REV                  ((UINT64)1<<18) /* Revision */
#define WICED_BT_PBC_FILTER_SOUND                ((UINT64)1<<19) /* Pronunciation of Name */
#define WICED_BT_PBC_FILTER_URL                  ((UINT64)1<<20) /* Uniform Resource Locator */
#define WICED_BT_PBC_FILTER_UID                  ((UINT64)1<<21) /* Unique ID */
#define WICED_BT_PBC_FILTER_KEY                  ((UINT64)1<<22) /* Public Encryption Key */
#define WICED_BT_PBC_FILTER_NICKNAME             ((UINT64)1<<23) /* Nickname */
#define WICED_BT_PBC_FILTER_CATEGORIES           ((UINT64)1<<24) /* Categories */
#define WICED_BT_PBC_FILTER_PROID                ((UINT64)1<<25) /* Product ID */
#define WICED_BT_PBC_FILTER_CLASS                ((UINT64)1<<26) /* Class information */
#define WICED_BT_PBC_FILTER_SORT_STRING          ((UINT64)1<<27) /* String used for sorting operation */
#define WICED_BT_PBC_FILTER_CALL_DATETIME        ((UINT64)1<<28) /* Time stamp */
#define WICED_BT_PBC_FILTER_X_BT_SPEEDDIALKEY    ((UINT64)1<<29) /* Speed-dial shortcut */
#define WICED_BT_PBC_FILTER_X_BT_UCI             ((UINT64)1<<30) /* Uniform Caller Identifier field */
#define WICED_BT_PBC_FILTER_X_BT_UID             ((UINT64)1<<31) /* Bluetooth Contact Unique Identifier */
#define WICED_BT_PBC_FILTER_ALL      (0)
typedef UINT64 wiced_bt_pbc_filter_mask_t;

/* Profile supported repositories */
#define WICED_BT_PBC_REPOSIT_LOCAL      0x01    /* Local PhoneBook */
#define WICED_BT_PBC_REPOSIT_SIM        0x02    /* SIM card PhoneBook */
#define WICED_BT_PBC_REPOSIT_SPEED_DIAL 0x04    /* Speed Dial */
#define WICED_BT_PBC_REPOSIT_FAVORITES  0x08    /* Favorites */

typedef UINT8 wiced_bt_pbc_sup_reposit_mask_t;

enum
{
    WICED_BT_PBC_FORMAT_CARD_21, /* vCard format 2.1 */
    WICED_BT_PBC_FORMAT_CARD_30, /* vCard format 3.0 */
    WICED_BT_PBC_FORMAT_MAX
};
typedef UINT8 wiced_bt_pbc_format_t;

enum
{
    WICED_BT_PBC_ORDER_INDEXED = 0,  /* indexed */
    WICED_BT_PBC_ORDER_ALPHANUM,     /* alphanumeric */
    WICED_BT_PBC_ORDER_PHONETIC,      /* phonetic */
    WICED_BT_PBC_ORDER_MAX
};
typedef UINT8 wiced_bt_pbc_order_t;

enum
{
    WICED_BT_PBC_ATTR_NAME = 0,      /* name */
    WICED_BT_PBC_ATTR_NUMBER,        /* number */
    WICED_BT_PBC_ATTR_SOUND,         /* sound */
    WICED_BT_PBC_ATTR_MAX
};
typedef UINT8 wiced_bt_pbc_attr_t;


/* Client callback function events */
#define WICED_BT_PBC_ENABLE_EVT      0   /* Phone Book Access client is enabled. */
#define WICED_BT_PBC_OPEN_EVT        1   /* Connection to peer is open. */
#define WICED_BT_PBC_CLOSE_EVT       2   /* Connection to peer closed. */
#define WICED_BT_PBC_AUTH_EVT        3   /* Request for Authentication key and user id */
#define WICED_BT_PBC_LIST_EVT        4   /* Event contains a directory entry (wiced_bt_PBC_LIST) */
#define WICED_BT_PBC_PROGRESS_EVT    5   /* Number of bytes read or written so far */
#define WICED_BT_PBC_GETFILE_EVT     6   /* Get complete */
#define WICED_BT_PBC_CHDIR_EVT       7   /* Change Directory complete */
#define WICED_BT_PBC_PHONEBOOK_EVT   8   /* Report the Application Parameters for wiced_bt_pbcGetPhoneBook response */
#define WICED_BT_PBC_DISABLE_EVT     9   /* Phone Book Access client is disabled. */

typedef UINT8 wiced_bt_pbc_evt_t;

/* Client callback function event data */

#define WICED_BT_PBC_OK              0
#define WICED_BT_PBC_FAIL            1
#define WICED_BT_PBC_NO_PERMISSION   2
#define WICED_BT_PBC_NOT_FOUND       3
#define WICED_BT_PBC_FULL            4
#define WICED_BT_PBC_BUSY            5
#define WICED_BT_PBC_ABORTED         6
#define WICED_BT_PBC_PRECONDITION_FAILED     7

typedef UINT8 wiced_bt_pbc_status_t;

typedef UINT8 wiced_bt_service_id_t;

typedef struct
{
    wiced_bt_service_id_t service;    /* Connection is open with PBAP */
    wiced_bt_pbc_sup_fea_mask_t   peer_features;      /* Peer supported features */
    wiced_bt_pbc_sup_reposit_mask_t   peer_repositories;  /* Peer supported repositories */
} wiced_bt_pbc_open_t;

typedef struct
{
    UINT16          phone_book_size;
    BOOLEAN         pbs_exist;          /* phone_book_size is present in the response */
    UINT8           new_missed_calls;
    BOOLEAN         nmc_exist;          /* new_missed_calls is present in the response */
} wiced_bt_pbc_pb_param_t;

typedef struct
{
    wiced_bt_pbc_pb_param_t *p_param;
    UINT8           *data;
    UINT16           len;
    BOOLEAN          final;     /* If TRUE, entry is last of the series */
    wiced_bt_pbc_status_t  status;    /* Fields are valid when status is WICED_BT_PBC_OK */
} wiced_bt_pbc_list_t;

typedef struct
{
    UINT32 file_size;   /* Total size of file (WICED_BT_PBC_LEN_UNKNOWN if unknown) */
    UINT16 bytes;       /* Number of bytes read or written since last progress event */
} wiced_bt_pbc_progress_t;

typedef struct
{
    UINT8  *p_realm;
    UINT8   realm_len;
    UINT8   realm_charset;
    BOOLEAN userid_required;    /* If TRUE, a user ID must be sent */
} wiced_bt_pbc_auth_t;


typedef union
{
    wiced_bt_pbc_status_t     status;
    wiced_bt_pbc_open_t       open;
    wiced_bt_pbc_list_t       list;
    wiced_bt_pbc_progress_t   prog;
    wiced_bt_pbc_auth_t       auth;
    wiced_bt_pbc_pb_param_t   pb;
} wiced_bt_pbc_t;

/* Client callback function */
typedef void wiced_bt_pbc_cback_t(wiced_bt_pbc_evt_t event, wiced_bt_pbc_t *p_data);

/* Client callback function */
typedef void wiced_bt_pbc_data_cback_t(const UINT8 *p_buf, UINT16 nbytes);

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/**************************
**  Client Functions
***************************/

/*******************************************************************************
**
** Function         wiced_bt_pbc_op_enable
**
** Description      Enable the phone book access client.  This function must be
**                  called before any other functions in the PBC API are called.
**                  When the enable operation is complete the callback function
**                  will be called with an WICED_BT_PBC_ENABLE_EVT event.
**
**
** Returns          void
**
*******************************************************************************/

extern void wiced_bt_pbc_op_enable(wiced_bt_pbc_cback_t *p_cback,
                                    wiced_bt_pbc_data_cback_t *p_data_cback,
                                    UINT8 app_id,
                                    wiced_bt_pbc_sup_fea_mask_t local_features);
/*******************************************************************************
**
** Function         wiced_bt_pbc_disable
**
** Description      Disable the phone book access client.  If the client is currently
**                  connected to a peer device the connection will be closed.
**
** Returns          void
**
*******************************************************************************/
extern void wiced_bt_pbc_op_disable(void);

/*******************************************************************************
**
** Function         wiced_bt_pbc_op_open
**
** Description      Open a connection to an PBAP server.
**
**                  When the connection is open the callback function
**                  will be called with a WICED_BT_PBC_OPEN_EVT.  If the connection
**                  fails or otherwise is closed the callback function will be
**                  called with a WICED_BT_PBC_CLOSE_EVT.
**
**                  Note: Pbc always enable (BTA_SEC_AUTHENTICATE | BTA_SEC_ENCRYPT)
**
** Returns          void
**
*******************************************************************************/
extern void wiced_bt_pbc_op_open(wiced_bt_device_address_t bd_addr, UINT8 sec_mask);

/*******************************************************************************
**
** Function         wiced_bt_pbc_op_close
**
** Description      Close the current connection to the server. Aborts any
**                  active PBAP transfer.
**
** Returns          void
**
*******************************************************************************/
extern void wiced_bt_pbc_op_close(void);


/*******************************************************************************
**
** Function         wiced_bt_pbc_op_getphonebook
**
** Description      Retrieve a PhoneBook from the peer device and copy it to the
**                  local file system.
**
**                  This function can only be used when the client is connected
**                  in PBAP mode.
**
** Note:            local file name is specified with a fully qualified path.
**                  Remote file name is absolute path in UTF-8 format
**                  (telecom/pb.vcf or SIM1/telecom/pb.vcf).
**
** Returns          void
**
*******************************************************************************/
extern void wiced_bt_pbc_op_getphonebook(char *p_local_name, char *p_remote_name,
                         wiced_bt_pbc_filter_mask_t filter, wiced_bt_pbc_format_t format,
                         UINT16 max_list_count, UINT16 list_start_offset,
                         BOOLEAN is_reset_miss_calls, wiced_bt_pbc_filter_mask_t selector,
                         UINT8 selector_op);

/*******************************************************************************
**
** Function         wiced_bt_pbc_op_getcard
**
** Description      Retrieve a vCard from the peer device and copy it to the
**                  local file system.
**
**                  This function can only be used when the client is connected
**                  in PBAP mode.
**
** Note:            local file name is specified with a fully qualified path.
**                  Remote file name is relative path in UTF-8 format.
**
** Returns          void
**
*******************************************************************************/
extern void wiced_bt_pbc_op_getcard(char *p_local_name, char *p_remote_name,
                    wiced_bt_pbc_filter_mask_t filter, wiced_bt_pbc_format_t format);


/*******************************************************************************
**
** Function         wiced_bt_pbc_op_chdir
**
** Description      Change PB path on the peer device.
**
**                  This function can only be used when the client is connected
**                  in PBAP mode.
**
** Returns          void
**
*******************************************************************************/
extern void wiced_bt_pbc_op_chdir(char *p_dir, wiced_bt_pbc_flag_t flag);

/*******************************************************************************
**
** Function         wiced_bt_pbc_op_authrsp
**
** Description      Sends a response to an OBEX authentication challenge to the
**                  connected OBEX server. Called in response to an WICED_BT_PBC_AUTH_EVT
**                  event.
**
** Note:            If the "userid_required" is TRUE in the WICED_BT_PBC_AUTH_EVT event,
**                  then p_userid is required, otherwise it is optional.
**
**                  p_password  must be less than WICED_BT_PBC_MAX_AUTH_KEY_SIZE (16 bytes)
**                  p_userid    must be less thanWICED_BT_OBX_MAX_REALM_LEN (defined in target.h)
**
** Returns          void
**
*******************************************************************************/
extern void wiced_bt_pbc_op_authrsp (char *p_password, char *p_userid);

/*******************************************************************************
**
** Function         wiced_bt_pbc_op_list_cards
**
** Description      Retrieve a directory listing from the peer device.
**                  When the operation is complete the callback function will
**                  be called with one or more WICED_BT_PBC_LIST_EVT events
**                  containing directory list information formatted as described
**                  in the PBAP Spec, Version 0.9, section 3.1.6.
**                  This function can only be used when the client is connected
**                  to a peer device.
**
**                  This function can only be used when the client is connected
**                  in PBAP mode.
**
** Parameters       p_dir - Name of directory to retrieve listing of.
**
** Returns          void
**
*******************************************************************************/

extern void wiced_bt_pbc_op_listcards(char *p_dir, wiced_bt_pbc_order_t order, char *p_value,
                      wiced_bt_pbc_attr_t attribute, UINT16 max_list_count,
                      UINT16 list_start_offset, BOOLEAN is_reset_miss_calls,
                      wiced_bt_pbc_filter_mask_t selector, UINT8 selector_op);


/*******************************************************************************
**
** Function         wiced_bt_pbc_op_abort
**
** Description      Aborts any active PBC operation.
**
** Returns          void
**
*******************************************************************************/
extern void wiced_bt_pbc_op_abort(void);


#ifdef __cplusplus
}
#endif

#endif /* WICED_BT_PBC_API_H */
