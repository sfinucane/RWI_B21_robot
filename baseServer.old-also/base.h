
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
 ***** Source file:     $Source: /home/rhino/rhino/CVS/bee/src/baseServer/base.h,v $
 *****
 ***** Created by:      $Author: tyson $
 *****
 ***** Revision #:      $Revision: 1.4 $
 *****
 ***** Date of revision $Date: 1997/03/25 21:44:37 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: base.h,v $
 * Revision 1.4  1997/03/25 21:44:37  tyson
 * Many bug fixes.
 *
 * Revision 1.3  1997/01/24 21:42:25  tyson
 * fixed serious libmcp bug, added bUtils for parameter files, logging and daemonizing plus misc.
 *
 * Revision 1.2  1996/11/22 20:50:55  tyson
 * minor clean-ups
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

/****************************************************************/
/*    FILE: base.h                                              */
/****************************************************************/

#ifndef _BASE_H
#define _BASE_H

#include <termios.h>

#include <baseMessages.h>
#include <bUtils.h>

#define BASE_CONFIG_VARIABLE "BASEHOST"
#define BASE_DEVICE "/dev/RWI_base"


/* How long in seconds the base controller can go without hearing from */
/* the base module. Any longer and it will limp all motors */
#define BASE_WATCHDOG_INTERVAL_IN_SEC 2 

#ifdef  __cplusplus
extern "C" {
#endif

  void BaseInit(const char *deviceName, speed_t baudRate);
  void BaseShutdown(void);

  void leaveDirectMode(void);
  void batteryCurrent(void);
  void batteryVoltage(void);
  void baseClock(void);
  void errorAcknowledge(unsigned long err_ack);
  void errorDelay(unsigned long delay);
  void baseDelay(unsigned long delay);
  void findRotIndex(void);
  void baseKill(void);
  void loadHeading(unsigned long heading);
  void loadPosition(unsigned long heading);

  /* rotation */
  void rotateHalt(void);
  void rotateLimp(void);
  void rotateVelocityPos(void);
  void rotateVelocityNeg(void);
  void rotateToPosition(unsigned long position);
  void rotateRelativePos(unsigned long relative);
  void rotateRelativeNeg(unsigned long relative);
  void rotateTorquePos(unsigned long relative);
  void rotateTorqueNeg(unsigned long relative);
  void rotatePowerPos(unsigned long relative);
  void rotatePowerNeg(unsigned long relative);

  void setRotateVelocity(int velocity);
  void setRotateFriction(unsigned long friction);
  void setRotateSlope(unsigned long slope);
  void setRotateTorque(unsigned long tork);
  void setRotateZero(unsigned long zero);
  void setRotateAcceleration(unsigned long accel);

  void rotateCurrent(void);
  void rotateWhere(void);

  void statusReportData(unsigned long data);
  void statusReportPeriod(unsigned long period);
  void statusReportRequest(unsigned long report);
  void registerStatusCallback(statusHandler fcn);
  void registerWatchdogCallback(watchdogHandler fcn);
  void assumeWatchdog();

  void getStatusReport(void);
  void joystickDisable(unsigned long disable);
  void radioDisable(unsigned long disable);
  void bumpSStopEnable(unsigned long);  
  void watchdogTimer(unsigned long);
  void registerBaseCallback(baseEventHandler);


  void setTranslateAcceleration(unsigned long);
  void setTranslateTorque(unsigned long);
  void setTranslateZero(unsigned long);
  void setTranslateSlope(unsigned long);
  void setTranslateVelocity(unsigned long);

  void translateCurrent(void);
  void translateWhere(void);

  void translateVelocityPos(void);
  void translateVelocityNeg(void);
  void translateToPosition(unsigned long);
  void translateLimp(void);
  void translateHalt(void);
  void translateRelativePos(unsigned long);
  void translateRelativeNeg(unsigned long);
  void translateTorquePos(unsigned long);
  void translateTorqueNeg(unsigned long);
  void translatePowerPos(unsigned long);
  void translatePowerNeg(unsigned long);


#ifdef  __cplusplus
}
#endif


/****************************************************************************/
/*
     Other admisitrivia
*/
/****************************************************************************/
#define BASE_TIME_UNITS_PER_SECOND 256


/* The types of errors the base can generate */
#define TERR 0x01
#define TC   0x02
#define RERR 0x04
#define RC   0x08
#define BHI  0x10
#define BLO  0x20



/* Obviously, programs will not be running if the base is at 0 volts. */
/* So, 0 volts, or a very high voltage reading, is probably a problem with*/
/* the base (like the base computer has been turned off) so we should not */ 
/* believe it. */

#define MIN_BELIEVABLE_VOLTAGE 1
#define MAX_BELIEVABLE_VOLTAGE 100

#endif
