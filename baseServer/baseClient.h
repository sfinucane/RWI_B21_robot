
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
 ***** Source file:     $Source: /home/rhino/rhino/CVS/bee/src/baseServer/baseClient.h,v $
 *****
 ***** Created by:      $Author: swa $
 *****
 ***** Revision #:      $Revision: 1.15 $
 *****
 ***** Date of revision $Date: 1998/01/15 16:30:33 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: baseClient.h,v $
 * Revision 1.15  1998/01/15 16:30:33  swa
 * Minor modifications towards enabling client-reconnects. Not yet fully
 * implemented.
 *
 * Revision 1.14  1997/09/09 16:11:56  tyson
 * Minor cleanup and debug for 1.1 release.
 *
 * Revision 1.13  1997/07/30 21:02:00  thrun
 * baseServer now sends confirmations of all motion commands. Is used
 * for logging in the module "learn"
 *
 * Revision 1.12  1997/07/24 15:27:16  tyson
 * added bNetwork.h, combined examples into on directory.  See Changes for other details
 *
 * Revision 1.11  1997/07/17 17:31:41  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.10  1997/06/21 22:36:07  thrun
 * Blocking is now an option when connecting to base server
 *
 * Revision 1.9  1997/03/24 20:03:08  thrun
 * Fixed a typo in the declarations. (swa)
 *
 * Revision 1.8  1997/03/11 17:03:18  tyson
 * added IR simulation and other work
 *
 * Revision 1.7  1997/02/27 15:59:56  tyson
 * upgrades to wander and bXxxxAngle() functions
 *
 * Revision 1.6  1997/02/25 18:12:37  tyson
 * client lib for PANTILT and lots of little stuff
 *
 * Revision 1.5  1996/12/31 20:23:19  tyson
 * B14 fixes and other minor work
 *
 * Revision 1.4  1996/12/12 02:17:58  tyson
 * simulator2<->baseServer work
 *
 * Revision 1.3  1996/11/22 20:50:55  tyson
 * minor clean-ups
 *
 * Revision 1.2  1996/11/18 04:34:55  tyson
 * More baseServer<->simulator2 work. Still not done.
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


#ifndef _BASE_CLIENT_H
#define _BASE_CLIENT_H

#include <math.h>
#include <raiClient.h>
/* for definition of sonar struct and export of sonars[] */
#include <sonarClient.h>
#include <irClient.h>
#include <tactileClient.h>
#include <baseMessages.h>
#include <odoClient.h>


/* we supply a callback to fill this in when new reports come, but */
/* client can read it directly if they wish */ 
extern statusReportType activeStatusReport; 

/**********************************************************************
 *
 * Vars for functions with consistent units and coordinate systems
 *
 *********************************************************************/

extern float bOriginX;
extern float bOriginY;
extern float bOriginHeading;

extern TCX_MODULE_PTR baseServer;

#ifdef __cplusplus
extern "C" {
#endif

void registerBaseClient(void);
int  baseConnect(int blocking);
void registerBaseCallback(baseEventHandler);
void registerStatusCallback(statusHandler);
void registerWatchdogCallback(watchdogHandler);
void registerCommandConfirmationCallback(commandConfirmationHandler fcn);

/* General */

void baseKill();
void loadHeading(unsigned long heading);
void loadPosition(unsigned long);
void statusReportData(unsigned long);
void statusReportPeriod(unsigned long);
void batteryVoltage(void);
void batteryVoltage(void);
/* void batteryCurrent(void); */
void assumeWatchdog(void); 
void watchdogTimer(unsigned long);
void joystickDisable(unsigned long);
void findRotIndex(void);

/* Rotation Commands*/

void rotateLimp();
void rotateHalt(void);
void rotateVelocityPos(void);
void rotateVelocityNeg(void);
void rotateToPosition(unsigned long position);
void rotateRelativePos(unsigned long relative);
void rotateRelativeNeg(unsigned long relative);
void rotateTorquePos(unsigned long relative);
void rotateTorqueNeg(unsigned long relative);
void rotatePowerPos(unsigned long relative);
void rotatePowerNeg(unsigned long relative);

void setRotateVelocity(unsigned long velocity);
void setRotateAcceleration(unsigned long accel);
void setRotateTorque(unsigned long tork);

/* Rotation Queries*/
void rotateWhere(void);
void rotateCurrent(void);

/* Translation Commands*/

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

void setTranslateVelocity(unsigned long);
void setTranslateAcceleration(unsigned long);
void setTranslateTorque(unsigned long);

void translateCurrent(void);
void translateWhere(void);

/**********************************************************************
 *
 * Functions with consistent units and coordinate systems
 *
 *********************************************************************/


float bNormalizeAngle(float radians);
float bWorldAngle(float robotAngle, float dt);
float bRobotAngle(float worldAngle, float dt);

float bSonarAngle(int sonarRow, int sonarCol);
float bIrAngle(int irRow, int irCol);
float bTactileAngle(int tactileRow, int tactileCol);

void  bSetVel(float rotVel, float transVel);
float bGetRotVel(void);
float bGetTransVel(void);

float bOdoToRobotX(float X, float Y);
float bOdoToRobotY(float X, float Y);
float bRobotToOdoX(float X, float Y);
float bRobotToOdoY(float X, float Y);

int   bSetPosition(float X, float Y, float heading);
void  bUpdatePosition(float dX, float dY, float dHeading);

void  bRegisterOdometryLockCallback(odoCallbackType fcn);
void  bRequestOdometryLock(unsigned short priority);
void  bReleaseOdometryLock();

float bRobotHeading(float dt);
float bRobotX(float dt);
float bRobotY(float dt);

/*
 * Obsolete functions
 */

int  findBaseServer(void); /* replaced by baseConnect */

#ifdef __cplusplus
}
#endif

#endif
