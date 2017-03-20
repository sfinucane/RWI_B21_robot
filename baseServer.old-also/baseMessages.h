
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
 ***** Source file:     $Source: /home/rhino/rhino/CVS/bee/src/baseServer/baseMessages.h,v $
 *****
 ***** Created by:      $Author: thrun $
 *****
 ***** Revision #:      $Revision: 1.6 $
 *****
 ***** Date of revision $Date: 1997/07/30 21:02:00 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: baseMessages.h,v $
 * Revision 1.6  1997/07/30 21:02:00  thrun
 * baseServer now sends confirmations of all motion commands. Is used
 * for logging in the module "learn"
 *
 * Revision 1.5  1997/07/17 17:31:41  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.4  1997/06/03 11:49:10  fox
 * Museum version.
 *
 * Revision 1.3  1997/02/25 18:12:37  tyson
 * client lib for PANTILT and lots of little stuff
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


#ifndef _BASE_MESSAGES_H
#define _BASE_MESSAGES_H

#include <mcpStatus.h>


/* Message format for the confirmation of a baseServer command 
 * put in by Sebastian, July 1997 */

typedef struct  {
  unsigned long operation;
  unsigned long param;
} confirmCommandDataType;




typedef void (*baseEventHandler)(unsigned long, unsigned long); 
typedef void (*watchdogHandler)();
typedef void (*statusHandler)(statusReportType*);
typedef void (*commandConfirmationHandler)(confirmCommandDataType*);


typedef enum {

  /* USED BY BASE SERVER ONLY */
  BASE_subscribe,
  
  /* USED BY BASE MODULE & SERVER TO COMMUNICATE WITH USER */
  
  /* error conditions */
  BASE_translateError,
  BASE_rotateError,
  BASE_batteryHigh,
  BASE_batteryLow,
  BASE_watchdogTimeout,     
  
  /* commands */
  BASE_baseKill,
  BASE_loadHeading,
  BASE_loadPosition,
  BASE_statusReportData,
  BASE_statusReportPeriod,
  BASE_statusReport,
  BASE_assumeWatchdog,
  BASE_watchdogTimer,     
  
  
  BASE_findRotIndex,
  BASE_rotateLimp,
  BASE_rotateHalt,
  BASE_rotateVelocityPos,
  BASE_rotateVelocityNeg,
  BASE_rotateRelativePos,
  BASE_rotateRelativeNeg,
  BASE_rotateTorquePos,
  BASE_rotateTorqueNeg,
  BASE_rotatePowerPos,
  BASE_rotatePowerNeg,
  BASE_rotateToPosition,
  
  BASE_setRotateSlope,
  BASE_setRotateTorque,
  BASE_setRotateZero,
  BASE_setRotateAcceleration,
  BASE_setRotateFriction,
  BASE_setRotateVelocity,


  BASE_translateLimp,
  BASE_translateHalt,
  BASE_translateVelocityPos,
  BASE_translateVelocityNeg,
  BASE_translateRelativePos,
  BASE_translateRelativeNeg,
  BASE_translateTorquePos,
  BASE_translateTorqueNeg,
  BASE_translatePowerPos,
  BASE_translatePowerNeg,
  BASE_translateToPosition,
  
  BASE_setTranslateSlope,
  BASE_setTranslateTorque,
  BASE_setTranslateZero,
  BASE_setTranslateAcceleration,
  BASE_setTranslateVelocity,
  
  /* commands that return values, message is used when returning value too */
  BASE_batteryCurrent,
  BASE_batteryVoltage,
  BASE_rotateCurrent,
  BASE_rotateWhere,
  BASE_translateCurrent,
  BASE_translateWhere,
  
  BASE_sonarStart,
  BASE_sonarStop,
  
  BASE_indexReport,
  
  /* New odometry commands */
  BASE_odometryChangeX,
  BASE_odometryChangeY,
  BASE_odometryChangeH,
  
  BASE_requestOdometryLock,
  BASE_releaseOdometryLock,
  BASE_odometryLockNotify,

  /* New message to report back a base command */
  BASE_confirmCommand
  
}  BaseMessages;

#endif
