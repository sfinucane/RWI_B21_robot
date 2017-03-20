
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/baseServer/baseOpcodes.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 16:03:25 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: baseOpcodes.h,v $
 * Revision 1.1  2002/09/14 16:03:25  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:09  rhino
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
#ifndef _BASE_OPCODES_H
#define _BASE_OPCODES_H

#include <mcpIO.h>  /*  opcodes common to all MCP applications*/

#define OP_GET_BATTERY_CURRENT 	0x61
#define OP_GET_BATTERY_VOLTAGE 	0x60
#define OP_FIND_ROT_INDEX 	0x13
#define OP_LOAD_POSITION	0x14
#define OP_LOAD_HEADING		0x15


#define OP_BUMPS_ENABLE		0x64

#define OP_ROTATE_TO_POS	0x5B
#define OP_ROTATE_HALT		0x42
#define OP_ROTATE_LIMP		0x43
#define OP_ROTATE_REL_POS	0x44
#define OP_ROTATE_REL_NEG	0x45
#define OP_ROTATE_VEL_POS	0x46
#define OP_ROTATE_VEL_NEG	0x47
#define OP_ROTATE_TRQ_NEG	0x48
#define OP_ROTATE_TRQ_POS	0x49
#define OP_ROTATE_PWR_NEG	0x4B
#define OP_ROTATE_PWR_POS	0x4A

#define OP_GET_ROTATE_CURRENT	0x5A
#define OP_GET_ROTATE_WHERE	0x5C

#define OP_SET_ROTATE_VEL	0x40
#define OP_SET_ROTATE_ACCEL	0x41
#define OP_SET_ROTATE_ZERO	0x50
#define OP_SET_ROTATE_TORQUE	0x4D
#define OP_SET_ROTATE_SLOPE	0x4E
#define OP_SET_ROTATE_FRICTION	0x4F

#define OP_TRANS_LIMP		0x23
#define OP_TRANS_HALT		0x22

#define OP_GET_TRANS_WHERE	0x3C
#define OP_GET_TRANS_CURRENT	0x3A

#define OP_SET_TRANS_VEL	0x20
#define OP_SET_TRANS_ACCEL	0x21
#define OP_SET_TRANS_TORQUE 	0x2D
#define OP_SET_TRANS_SLOPE	0x2E
#define OP_SET_TRANS_ZERO	0x30

#define OP_TRANS_REL_POS	0x24
#define OP_TRANS_REL_NEG	0x25
#define OP_TRANS_VEL_POS	0x26
#define OP_TRANS_VEL_NEG	0x27
#define OP_TRANS_TRQ_NEG	0x28
#define OP_TRANS_TRQ_POS	0x29
#define OP_TRANS_PWR_NEG	0x2B
#define OP_TRANS_PWR_POS	0x2A
#define OP_TRANS_TO_POS		0x3B


#endif
