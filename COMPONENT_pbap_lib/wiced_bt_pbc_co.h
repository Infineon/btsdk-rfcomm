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
**  Name:           wiced_bt_pbc_co.h
**
**  Description:    This is the interface file for the PBAP client call-out functions.
**
**
*****************************************************************************/
#ifndef WICED_BT_PBC_CO_H
#define WICED_BT_PBC_CO_H

/*****************************************************************************
**  Constants and Data Types
*****************************************************************************/

/**************************
**  Common Definitions
***************************/

/* Status codes returned by call-out functions, or in call-in functions as status */
#define WICED_BT_PBC_CO_OK            0
#define WICED_BT_PBC_CO_FAIL          1   /* Used to pass all other errors */
#define WICED_BT_PBC_CO_EACCES        2

typedef UINT16 wiced_bt_pbc_co_status_t;

/***************************************************************
**  Function Declarations
*****************************************************************************/


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
void wiced_bt_pbc_co_open(UINT16 evt, UINT8 app_id);

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
extern wiced_bt_pbc_co_status_t wiced_bt_pbc_co_close(int fd, UINT8 app_id);

/*******************************************************************************
**
** Function         wiced_bt_pbc_co_read
**
** Description      This function is called by BTA to read in data from the
**                  previously opened file on the phone.
**
** Parameters       fd      - file descriptor of file to read from.
**                  p_buf   - buffer to read the data into.
**                  nbytes  - number of bytes to read into the buffer.
**                  evt     - event that must be passed into the call-in function.
**                  ssn     - session sequence number. Ignored, if wiced_bt_pbc_co_open
**                            was not called with WICED_BT_PBC_CO_RELIABLE.
**                  app_id  - application ID specified in the enable functions.
**                            It can be used to identify which profile is the caller
**                            of the call-out function.
**
** Returns          void
**
**                  Note: Upon completion of the request, wiced_bt_pbc_ci_read() is
**                        called with the buffer of data, along with the number
**                        of bytes read into the buffer, and a status.  The
**                        call-in function should only be called when ALL requested
**                        bytes have been read, the end of file has been detected,
**                        or an error has occurred.
**
*******************************************************************************/
extern void wiced_bt_pbc_co_read(int fd, UINT8 *p_buf, UINT16 nbytes, UINT16 evt,
                           UINT8 ssn, UINT8 app_id);

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
**                  ssn     - session sequence number. Ignored, if wiced_bt_pbc_co_open
**                            was not called with WICED_BT_PBC_CO_RELIABLE.
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
extern void wiced_bt_pbc_co_write(int fd, const UINT8 *p_buf, UINT16 nbytes, UINT16 evt,
                            UINT8 ssn, UINT8 app_id);

#endif /* WICED_BT_PBC_CO_H */
