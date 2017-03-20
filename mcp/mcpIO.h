
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/mcp/mcpIO.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:29:36 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: mcpIO.h,v $
 * Revision 1.1  2002/09/14 15:29:36  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1997/01/24 21:42:28  tyson
 * fixed serious libmcp bug, added bUtils for parameter files, logging and daemonizing plus misc.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:08  rhino
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
 * Copyright 1994, Brown University, Providence, RI
 *
 * Permission to use and modify this software and its documentation for
 * any purpose other than its incorporation into a commercial product is
 * hereby granted without fee.  Permission to copy and distribute this
 * software and its documentation only for non-commercial use is also
 * granted without fee, provided, however, that the above copyright notice
 * appear in all copies, that both that copyright notice and this permission
 * notice appear in supporting documentation, that the name of Brown
 * University not be used in advertising or publicity pertaining to
 * distribution of the software without specific, written prior permission,
 * and that the person doing the distribution notify Brown University of
 * such distributions outside of his or her organization. Brown University
 * makes no representations about the suitability of this software for
 * any purpose.  It is provided "as is" without express or implied warranty.
 * Brown University requests notification of any modifications to this
 * software or its documentation.
 *
 * Send the following redistribution information:
 *
 *	Name:
 *	Organization:
 *	Address (postal and/or electronic):
 *
 * To:
 *	Software Librarian
 *	Computer Science Department, Box 1910
 *	Brown University
 *	Providence, RI 02912
 *
 *		or
 *
 *	brusd@cs.brown.edu
 *
 * We will acknowledge all electronic notifications.
 */



#ifndef _MCP_IO_H
#define  _MCP_IO_H

#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <utils.h>
#include <rai.h>

#ifdef __cplusplus
extern "C" {
#endif



typedef struct
{
  int fd;
  int state;
  char* lockname;
  char* return_string;
  int start_in_stream;
  int pos;
  char buff[512];
  int  buffLen;
  int  buffPos;
}  mcpIOState;

void mcpSendCmd(mcpIOState*, unsigned char opcode, unsigned long param);
char* mcpSelect(mcpIOState*);
mcpIOState* openMcp(const char* devicename, char* lockname,speed_t baudRate);
void resetMcpComm(mcpIOState* state);
void  closeMcp(mcpIOState*);




/***** Binary mode opcodes which are common to all MCP usages.   ****/
/***** The semantics of other MCP opcodes depend on what the MCP ****/
/*****  is hooked up to  */

#define OP_GET_VERSION_REPORT 	0x01
#define OP_MCP_KILL 		0x02
#define OP_MCP_DELAY 		0x05
#define OP_GET_USER_MESSAGE     0x08
#define OP_LEAVE_DIRECT_MODE 	0x09
#define OP_ERROR_DELAY 		0x0A
#define OP_ERROR_ACKNOWLEDGE 	0x0B
#define OP_FULL_DUPLEX 		0x0C
#define OP_HALF_DUPLEX 		0x0D
#define OP_JOYSTICK_DISABLE	0x0E
#define OP_RADIO_DISABLE	0x0F
#define OP_GET_ONE_REPORT       0x10
#define OP_SET_STATUS_DATA      0x11
#define OP_SET_STATUS_PERIOD    0x12
#define OP_WATCH_DOG_TIMER	0x16
#define OP_ADJUST_HEADING	0x17   /* delta is sent as signed short */

#define OP_GET_MCP_CLOCK	0x63
#define OP_INDEX_REPORT         0x65

#ifdef __cplusplus
}
#endif

#endif
