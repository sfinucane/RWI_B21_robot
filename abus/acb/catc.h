
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Welcome!
 *****
 ***** This file is part of the robot control software provided
 ***** by Real World Interface Inc.
 *****
 ***** All copyrights are by Real World Interface Inc., Carnegie
 ***** Mellon University, and the University of Bonn, Germany.
 ***** Removing the copyright information is illegal. Please read
 ***** and make sure you understand the disclaimer below.
 *****
 ***** Contact tyson@rwii.com if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                      Tyson D. Sawyer and Sebastian Thrun
 *****
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED
 ***** BY APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING
 ***** THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM
 ***** "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR
 ***** IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 ***** OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE
 ***** ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM
 ***** IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME
 ***** THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO
 ***** LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES
 ***** SUSTAINED BY YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM
 ***** TO OPERATE WITH ANY OTHER PROGRAMS OR FAILURE TO CONTROL A
 ***** PHYSICAL DEVICE OF ANY TYPE), EVEN IF SUCH HOLDER OR OTHER
 ***** PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Source file:     $Source: /home/rhino/rhino/CVS/bee/src/abus/acb/catc.h,v $
 *****
 ***** Created by:      $Author: tyson $
 *****
 ***** Revision #:      $Revision: 1.2 $
 *****
 ***** Date of revision $Date: 1997/02/25 18:12:36 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: catc.h,v $
 * Revision 1.2  1997/02/25 18:12:36  tyson
 * client lib for PANTILT and lots of little stuff
 *
 * Revision 1.1.1.1  1996/09/22 16:46:03  rhino
 * General reorganization of the directories/repository, fusion with the
 * RWI software.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


/*
 * catc.h  ACCESS.bus manager/kernel module for CATC ab125i
 *
 * Tyson D. Sawyer
 * tyson@rwii.com
 * Real World Interface, Inc.
 *
 * The creation and continued develoment of this software
 * is sponsored and directed by RWI in the interest of
 * providing the mobile robotics and AI research communities
 * with a well designed and robust Robot Applications
 * Interface (RAI) for the complete line of RWI mobile robots. 
 */

/*
 * Copyright 1995, Real World Interface, Inc.
 *
 * Portions of this code were created by and are
 * copyrighted by Computer Access Technology Corp.
 *
 * Permission to use this software is herby granted by
 * Real World Interface, Inc. (RWI) for non-commercial uses without
 * fee, provided, however, that the above copyright notice appear in
 * all copies, that both the copyright notice and this permission
 * notice appear in supporting documentation.  This permission may be
 * extended to commercial use of this software with specific written
 * permision from RWI.  RWI makes no representations about the
 * suitability of this software for any purpose.  It is provided
 * "as is" without express or implied warranty.  RWI requests
 * notification of any modifications to this software or its
 * documentation.
 *
 *   ==Contact  support@rwii.com  for further information==
 */

/********************************************************************/
/*                             catc.h                               */
/********************************************************************/

#ifndef _LINUX_CATC_H_
#define _LINUX_CATC_H_

#include "acb/global.h"

#ifdef __KERNEL__
#define AB_MAJOR	28
#define AB_MODULE_NAME  "abus"
#endif

/********************************************************************
* Return status values
*********************************************************************/
#define     ABOK        0
#define     ABERR       -1

/********************************************************************
* Ports, Irq and interrupts
*********************************************************************/
#define     MC_PORT1     0x250
#define     MC_PORT2     0x260
#define     MC_PORT3     0x350
#define     MC_DEFPORT   MC_PORT1   /* Controller's default port */

#define     MC_IRQ1      10
#define     MC_IRQ2      11
#define     MC_IRQ3      12
#define     MC_DEFIRQ    MC_IRQ2   /* controller's default irq */

#define     MGR_DEFMAX   24       /* Manager's default max devices */

/********************************************************************
* Manager to controller commands
*********************************************************************/
#define     M2C_START              0x00
#define     M2C_RESET              0x00
#define     M2C_GET_TABLE          0x01
#define     M2C_GET_LONG_ID        0x02
#define     M2C_GET_ID             0x03
#define     M2C_GET_TYPE           0x04
#define     M2C_GET_SELF_TEST      0x05
#define     M2C_GET_STATUS         0x06
#define     M2C_AB_EMULATION       0x07
#define     M2C_KBD_EMULATION      0x08
#define     M2C_KBD_TABLE          0x09
#define     M2C_KBD_LOOPBACK       0x0A
#define     M2C_ENABLE             0x0B
#define     M2C_MSG_TO_DEVICE      0x0C
#define     M2C_END                0x0C

/********************************************************************
* Controller to manager commands
*********************************************************************/
#define     C2M_START              0x80
#define     C2M_LONG_ID            0x80
#define     C2M_ID                 0x81
#define     C2M_TYPE               0x82
#define     C2M_SELF_TEST          0x83
#define     C2M_STATUS             0x84
#define     C2M_END_TABLE          0x85
#define     C2M_DISCONNECT         0x86
#define     C2M_AB_EMULATION       0x87
#define     C2M_MSG_TO_DRIVER      0x88
#define     C2M_MSG_TO_MONITOR     0x89
#define     C2M_END                0x89

/********************************************************************
* Driver to manager commands
*********************************************************************/
/*
* Internal interface - used only by CATC
*/
#define     D2M_INT_START          0xC0
#define     D2M_AB_EMULATION       0xC0
#define     D2M_KBD_EMULATION      0xC1
#define     D2M_KBD_TABLE          0xC2
#define     D2M_KBD_LOOPBACK       0xC3
#define     D2M_MONITOR            0xC4
#define     D2M_GET_PDT            0xC5
#define     D2M_DISABLE_ALL        0xC6
#define     D2M_DISCONNECT_ALL     0xC7
#define     D2M_CONNECT_ALL        0xC8
#define     D2M_INT_END            0xC8

/********************************************************************
* Kernel to daemon commands - These are rwi implementation specific
*********************************************************************/

#define     K2D_CLIENT_CONNECT     0xCA	/* minor 1 = connect */

/******************************************************************
* Daemon and Monitor to Kernel ioctl()
******************************************************************/
#define     ABIO_I_AM_MANAGER                 0x10
#define     ABIO_CONNECT_TO_CONTROLLER        0x11
#define     ABIO_DISCONNECT_FROM_CONTROLLER   0x12
#define     ABIO_ASSGN_MONITOR                0x13
#define     ABIO_ASSGN_DEV_TO_CLIENT          0x14
#define     ABIO_SET_DEV_STATUS               0x15

#define     ABIO_ENABLE_IRQ                   0x20
#define     ABIO_DISABLE_IRQ                  0x21
#define     ABIO_MSG_BUSY                     0x22
#define     ABIO_MSG_IDLE                     0x23

#define     ABIO_SEND_TIME                    0x30

/******************************************************************
* Kernel to Manager messages
******************************************************************/
typedef struct ABKMSG {
   ABMSG     msg;
   int       clientNum;
} ABKMSG;

/********************************************************************
* Error codes
*********************************************************************/
#define     ERR_START            0x100   /* our errors */
#define     ERR_UNKNOWN_OPCODE   0x101   /* unknown opcode */
#define     ERR_UNKNOWN_DEVID    0x102   /* unknown device id */
#define     ERR_RESEND           0x103   /* resend message */
#define     ERR_BUFFER_FULL      0x104   /* AB buffer full */
#define     ERR_LEN_NOT_MATCH    0x105   /* message length not match */
#define     ERR_BUSY             0x10E   /* device busy */
#define     ERR_TIMEOUT          0x10F   /* timeout */
#define     ERR_INVALID_PORT     0x110   /* invalid port */
#define     ERR_INVALID_IRQ      0x111   /* invalid irq */
#define     ERR_AB_NOT_READY     0x112   /* MC not ready */
#define     ERR_NOMEM            0x113   /* no memory */
#define     ERR_NO_DRIVER        0x116   /* no driver handler */
#define     ERR_TOO_MANY_DEVICES 0x117   /* too many devices */
#define     ERR_DEVICE_EXISTS    0x118   /* device already exists */
#define     ERR_NOT_IMPLEMENTED  0x119   /* not implemented yet */
#define     ERR_DRIVER_EXISTS    0x120   /* driver exists */
#define     ERR_NO_AB_MANAGER    0x121   /* ab manager not installed */
#define     ERR_INTR_USED        0x122   /* ab interrupt used */
#define     ERR_SELF_TEST        0x123   /* ab self test error */
#define     ERR_END              0x123   /* end of errors */

/*
* protocol definitions
*/

typedef enum {
	ACK_LIMIT=0xD,
	ACK_ERR,
	ACK_IDLE
} MsgAck;


/* units of jiffies, 1/100 sec */
/* #define     TIMEOUT_LIMIT    3	*/
/* Playing with this -BR */
#define     TIMEOUT_LIMIT    5	/* units of jiffies, 1/100 sec */

#define     AB_RETRIES         200
#define     AB_UDELAY          50
#define     ABMSG_ACK(status)  (status & 0xF)
#define     ABMSG_ERR(status)  ((status >> 4) & 0xF)
#define     NEXTCNT(x)       x=(x == ACK_LIMIT) ? 1 : x+1


/*
* IN/OUT ports offsets and access
*/
#define     INPDATA_OFS     0 	/* 0-1 --> In data*/
#define     INPSTAT_OFS     2	/* 2-3 --> In status */
#define     OUTDATA_OFS     4	/* 4-5 --> Out data */
#define     OUTSTATI_OFS    6	/* 6-7 --> Out status interrupted */
#define     OUTSTAT_OFS     8	/* 8-9 --> Out status */
#define     INPSTATI_OFS    10	/* 10-11 --> In status interrupted */
#define     INPDATA()       inw_p(port+INPDATA_OFS)
#define     INPSTAT()       (inw_p(port+INPSTAT_OFS) & 0xFF)
#define     OUTDATA(x)      outw_p(x, port+OUTDATA_OFS)
#define     OUTSTATI(x)     outw_p(x, port+OUTSTATI_OFS)
#define     OUTSTAT(x)      outw_p(x, port+OUTSTAT_OFS)
#define     CLRIRQ()        inw_p(port+INPSTATI_OFS)

/*******************************************************************
* MsgStatus
*	1 byte :	LS 4 bits - acknowledge counter
*				MS 4 bits - error code
********************************************************************/

typedef enum {
	MSG_OK               =0x0,
	MSG_UNKNOWN_OPCODE   =0x1,
	MSG_UNKNOWN_DEVID    =0x2,
	MSG_RESEND           =0x3,
	MSG_BUFFER_FULL      =0x4,
	MSG_LEN_NOT_MATCH    =0x5,
	MSG_BUSY             =0xE,
	MSG_TIMEOUT          =0xF
} MSGERR;


#endif /* _LINUX_CATC_H_ */
