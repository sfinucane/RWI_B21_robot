
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/mcp/mcpStatus.h,v $
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
 * $Log: mcpStatus.h,v $
 * Revision 1.1  2002/09/14 15:29:36  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1999/12/06 13:21:22  fox
 * Added time stamps to base report.
 *
 * Revision 1.2  1997/04/17 16:10:33  tyson
 * Added support for Ramona plus fixed a couple of minor bugs
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

#ifndef MCPSTATUS_HEADER
#define MCPSTATUS_HEADER


/* Status reports for Base MCP */
typedef struct  {
  unsigned long Request;
  unsigned long Clock;
  unsigned long GeneralStatus;
  unsigned long Xpos;
  unsigned long Ypos;  
  unsigned long Heading;
  unsigned long BaseRelativeHeading;
  unsigned long TranslateError;
  unsigned long TranslateVelocity;
  unsigned long TranslateStatus;
  unsigned long RotateError;
  unsigned long RotateVelocity;
  unsigned long RotateStatus;
  unsigned long TimeStampSec;
  unsigned long TimeStampUSec;
} statusReportType;


/* what bits of the TranslateStatus is the phase number, the most */ 
/* interesting part of the Translate and Rotate Status fields     */
/* so if statusPhase(report->RotateStatus) == PHASE_ACCEL, the    */
/* motor on the rotate axis is accelerating */

#define PHASE_MASK 0xE000
#define statusPhase(status)  ( (status & PHASE_MASK) >> 13)
#define statusDirectionPositive(status)  ( !((status >> 11) & 1))
#define phaseMotionless(x) ((x==PHASE_STOP)||(x==PHASE_HALT)||(x==PHASE_LIMP))

#define PHASE_STOP  0
#define PHASE_ACCEL 1
#define PHASE_VELOC 2
#define PHASE_DECEL 3
#define PHASE_HALT  4
#define PHASE_LIMP  5
#define PHASE_POWER  6
#define PHASE_TORQUE  7


/*******************************************************************/
/*
    Support for specifying what a status report should include
*/ 
/*******************************************************************/

/* These are the bits you send to statusData or statusReport in  */
/* order to have the corresponding piece of data sent back. That */
/* is, to get Xpos and Ypos back, you send (REPORT_X | REPORT_Y) */

#define REPORT_STATUS_DATA	  	(1<<0)  /* allways set automatically */
#define REPORT_BASE_CLOCK		(1<<1) 
#define REPORT_GENERAL_STATUS		(1<<2) 
#define REPORT_X  			(1<<4)
#define REPORT_Y  			(1<<5)
#define REPORT_HEADING  		(1<<6)
#define REPORT_BASE_RELATIVE_HEADING	(1<<7)
#define REPORT_TRANSLATE_ERROR  	(1<<8)
#define REPORT_TRANSLATE_VELOCITY  	(1<<9)
#define REPORT_TRANSLATE_STATUS  	(1<<11)
#define REPORT_ROTATE_ERROR	  	(1<<16)
#define REPORT_ROTATE_VELOCITY  	(1<<17)
#define REPORT_ROTATE_STATUS  	        (1<<19)

/* half the stuff the status report could send is not really of interest */
/* so to get everything useful use REPORT_EVERYTHING  */ 

#define REPORT_EVERYTHING ( \
 REPORT_STATUS_DATA 		| \
 REPORT_BASE_CLOCK 		| \
 REPORT_GENERAL_STATUS 		| \
 REPORT_X  			| \
 REPORT_Y  			| \
 REPORT_HEADING  		| \
 REPORT_BASE_RELATIVE_HEADING 	| \
 REPORT_TRANSLATE_ERROR  	| \
 REPORT_TRANSLATE_VELOCITY 	| \
 REPORT_TRANSLATE_STATUS  	| \
 REPORT_ROTATE_ERROR		| \
 REPORT_ROTATE_VELOCITY		| \
 REPORT_ROTATE_STATUS  	\
)




/* The status report can also report this stuff, which seemed fairly */
/* obsolete now that the programming interface is so different       */
/* 3 = base bump switches, not used */ 
/* 10-15 A/D and motor stuff */
/* 18-31 More A/D, motor, radio stuff */

#endif


