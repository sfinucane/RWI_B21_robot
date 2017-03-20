
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/sys/mspMessages.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:33:41 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: mspMessages.h,v $
 * Revision 1.1  2002/09/14 15:33:41  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:10  rhino
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


/********************************************************************/
/*                    msp/include/messages.h                        */
/********************************************************************/

#ifndef _MSP_MESSAGES_H_
#define _MSP_MESSAGES_H_

#include <acb/global.h>		/* standard A.b defines and typedefs */
#include <acb/rwiab.h>		/* standard RWI defines and typedefs */

/* 0x00 - 0x7F are reserved for vendor usage */
/* 0xC0 - 0xC8 are reserved for vendor usage */

/* MSP Control/Status opcodes */
/*   Protocol bit == 1 */

#define MSP_AD_REQ         0x11
#define MSP_AD_RPL         0x12
#define MSP_AD_PARMS       0x13	/* not yet supported */
#define MSP_AD_PARMS_RPL   0x14	/* not yet supported */

#define MSP_BMP_REQ        0x21
#define MSP_BMP_RPL        0x22
#define MSP_BMP_PARMS      0x23	/* not yet supported */
#define MSP_BMP_PARMS_RPL  0x24	/* not yet supported */

#define MSP_IR_REQ         0x31
#define MSP_IR_RPL         0x32
#define MSP_IR_PARMS       0x33
#define MSP_IR_PARMS_RPL   0x34

#define MSP_SON_REQ        0x41
#define MSP_SON_RPL        0x42
#define MSP_SON_PARMS      0x43
#define MSP_SON_PARMS_RPL  0x44
#define MSP_SON_TABLE      0x45
#define MSP_SON_TABLE_RPL  0x46
#define MSP_SON_START      0x47

/* MSP needs space in its BusMsg's to record Rx time... */
/* ... so we have yet an other typdef...               */

typedef struct {
  BusMsgHdr hdr;
  unsigned char data[MSG_DATA_LEN];
  unsigned short time;
} MSP_Msg;

/* AD converter messages */

#define MSP_AD_NUM 8

typedef RWI_ReqBody MSP_AdReqBody;

typedef struct {
  RWI_BodyHdr   rwihdr;
  unsigned char AdVals[MSP_AD_NUM];
} MSP_AdRplBody;


/* Bump messags */

#define MSP_BMP_NUM 1

typedef RWI_ReqBody MSP_BmpReqBody;

typedef struct {
  RWI_BodyHdr   rwihdr;
  unsigned char BmpVals[MSP_BMP_NUM];
} MSP_BmpRplBody;

/* IR messages */

#define MSP_IR_NUM 8

typedef RWI_ReqBody MSP_IrReqBody;

typedef struct {
  RWI_BodyHdr    rwihdr;
  unsigned char IrVals[MSP_IR_NUM];
} MSP_IrRplBody;

typedef struct {
  RWI_BodyHdr   rwihdr;
  unsigned short parms[RWI_DATA_LEN/sizeof(short)];
} MSP_IrParmsBody;

typedef struct {
  RWI_BodyHdr   rwihdr;
  unsigned short irInterval;
} MSP_IrParmsRplBody;

#define MSP_IR_PARM_INTERVAL 1


/* Sonar messages */

typedef struct {
  RWI_BodyHdr    rwihdr;
  unsigned short sonAddr[RWI_DATA_LEN/sizeof(short)];
  /* MSB of sonAddr is MSP number (i.e. dipswitches) */
  /* MSNibble of LSB is chain numer */
  /* LSNibble of LSB is transducer number */
} MSP_SonReqBody;

typedef struct {
  RWI_BodyHdr    rwihdr;
  unsigned short echoNum;
  unsigned short data[RWI_DATA_LEN/sizeof(short)];
} MSP_SonRplBody;

typedef struct {
  RWI_BodyHdr    rwihdr;
  unsigned short parms[RWI_DATA_LEN/sizeof(short)];
} MSP_SonParmsBody;

typedef struct {
  RWI_BodyHdr   rwihdr;
  unsigned short fireInterval;
  unsigned short echoCount;
  unsigned short echoTimeout;
  unsigned short initBlank;
  unsigned short echoBlank;
  unsigned short startDelay;
  unsigned short fireDelay;
} MSP_SonParmsRplBody;

#define MSP_SON_PARM_FIRE_INTERVAL  1
#define MSP_SON_PARM_ECHO_NUMBER    2
#define MSP_SON_PARM_ECHO_TIMEOUT   3
#define MSP_SON_PARM_INIT_BLNK_TIME 4
#define MSP_SON_PARM_ECHO_BLNK_TIME 5
#define MSP_SON_PARM_START_DELAY    6
#define MSP_SON_PARM_FIRE_DELAY     7

typedef struct {
  RWI_BodyHdr    rwihdr;
  unsigned short table[RWI_DATA_LEN/sizeof(short)];
} MSP_SonTableBody;

typedef struct {
  RWI_BodyHdr    rwihdr;
  unsigned short table[RWI_DATA_LEN/sizeof(short)];
} MSP_SonTableRplBody;

typedef struct {
  RWI_BodyHdr    rwihdr;
  unsigned char  op;
} MSP_SonStartBody;

#define MSP_SON_START_START 1
#define MSP_SON_START_STOP  2

/*

MSP_SON_RPL
  short   data
    0      sonar address
    1      first return value
    2      second return value
    N      Nth return value
    ...
 echoNum-1 return value
 echoNum   second sonar address
 echoNum+1 first return value of second sonar
    ...
      
MSP_SON_TABLE
  short   data
    0     first sonar address
    1     second sonar address
    ...
    0000  indicating end of first sonar list
    ...
    0000  end of second list
    ...
      
*/
      
#endif _MSP_MESSAGES_H_
