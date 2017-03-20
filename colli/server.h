
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/server.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:25:05 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: server.h,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.4  1998/04/12 15:54:31  wolfram
 * Added option -robot for multi robot support
 *
 * Revision 1.3  1996/11/22 20:50:54  tyson
 * minor clean-ups
 *
 * Revision 1.2  1996/11/22 06:32:37  tyson
 * Minor cleanups
 *
 * Revision 1.1.1.1  1996/09/22 16:46:05  rhino
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



#include <baseMessages.h>
#include <sonarClient.h>
#include <irClient.h>
#include <tactileClient.h>
#include <sensorMessages.h>


#define TCX_SERVER_MODULE_NAME "baseTCXServer"

/***** TCX messages and handlers for the base server ****/

#define SERVER_messages \
  {"baseFixed", "{int, long}"}, \
  {"baseVar", "{int,int,<char: 2>}"}


#define LENGTH_OF_MODULE_NAMES 128
#define NUMBER_OF_TCX_MODULES 8

#define BASE_MODULE           0
#define SERVER_MODULE         1
#define LASER_SERVER_MODULE   2
#define SIMULATOR_MODULE      3
#define BASE_ROUTED_MODULE    4
#define ARM_MODULE            5
#define SUNVIS_MODULE         6
#define SOUND_MODULE          7





#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR SERVER;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR SERVER;	/* otherwise: reference */

#endif



void baseSendServerFixed(int operation,unsigned long arg);
void init_server_connection();



/* Each member of the API on the client side consists of a function which */
/* just sends the corresponding message (fcn name with BASE_ prepended    */
/* and perhaps argument, to the base server.  So I wrote macros to form   */ 
/* the functions */

/* I have long suspected trained monkeys could write C :-) */  

#define HserverCmd(name) void name(void);
#define HserverArgCmd(name) void name(unsigned long arg);



/*** GENERAL ****/
HserverCmd(baseKill)
HserverArgCmd(loadHeading)
HserverArgCmd(loadPosition)
HserverArgCmd(statusReportData)
HserverArgCmd(statusReportPeriod)

HserverCmd(batteryVoltage)
HserverCmd(batteryCurrent)

/*** ROTATION ****/
HserverCmd(rotateLimp)
HserverCmd(rotateHalt)
HserverCmd(rotateVelocityPos)
HserverCmd(rotateVelocityNeg)
HserverArgCmd(rotateRelativePos)
HserverArgCmd(rotateRelativeNeg)
HserverArgCmd(rotateTorquePos)
HserverArgCmd(rotateTorqueNeg)
HserverArgCmd(rotatePowerPos)
HserverArgCmd(rotatePowerNeg)
HserverArgCmd(rotateToPosition) 
HserverCmd(findRotIndex)

HserverArgCmd(setRotateFriction)
HserverArgCmd(setRotateVelocity)
HserverArgCmd(setRotateAcceleration)
HserverArgCmd(setRotateSlope)
HserverArgCmd(setRotateTorque)
HserverArgCmd(setRotateZero)

HserverCmd(rotateCurrent)
HserverCmd(rotateWhere)


/*** TRANSLATION ****/

HserverCmd(translateLimp)
HserverCmd(translateHalt)
HserverCmd(translateVelocityPos)
HserverCmd(translateVelocityNeg)
HserverArgCmd(translateRelativePos)
HserverArgCmd(translateRelativeNeg)
HserverArgCmd(translateTorquePos)
HserverArgCmd(translateTorqueNeg)
HserverArgCmd(translatePowerPos)
HserverArgCmd(translatePowerNeg)
HserverArgCmd(translateToPosition) 

HserverArgCmd(setTranslateVelocity)
HserverArgCmd(setTranslateAcceleration)
HserverArgCmd(setTranslateSlope)
HserverArgCmd(setTranslateTorque)
HserverArgCmd(setTranslateZero)

HserverCmd(translateCurrent)
HserverCmd(translateWhere)



#ifdef define_base_message_names


char *baseMessageNamesXXX[] ={
   "BASE_subscribe",
   "BASE_translateError",
   "BASE_rotateError",
   "BASE_batteryHigh",
   "BASE_batteryLow",
   "BASE_watchdogTimeout", 
   "BASE_baseKill",
   "BASE_loadHeading",
   "BASE_loadPosition",
   "BASE_statusReportData",
   "BASE_statusReportPeriod",
   "BASE_statusReport",
   "BASE_assumeWatchdog",
   "BASE_watchdogTimer",
   "BASE_findRotIndex",
   "BASE_rotateLimp",
   "BASE_rotateHalt",
   "BASE_rotateVelocityPos",
   "BASE_rotateVelocityNeg",
   "BASE_rotateRelativePos",
   "BASE_rotateRelativeNeg",
   "BASE_rotateTorquePos",
   "BASE_rotateTorqueNeg",
   "BASE_rotatePowerPos",
   "BASE_rotatePowerNeg",
   "BASE_rotateToPosition",
   "BASE_setRotateSlope",
   "BASE_setRotateTorque",
   "BASE_setRotateZero",
   "BASE_setRotateAcceleration",
   "BASE_setRotateFriction",
   "BASE_setRotateVelocity",
   "BASE_translateLimp",
   "BASE_translateHalt",
   "BASE_translateVelocityPos",
   "BASE_translateVelocityNeg",
   "BASE_translateRelativePos",
   "BASE_translateRelativeNeg",
   "BASE_translateTorquePos",
   "BASE_translateTorqueNeg",
   "BASE_translatePowerPos",
   "BASE_translatePowerNeg",
   "BASE_translateToPosition",
   "BASE_setTranslateSlope",
   "BASE_setTranslateTorque",
   "BASE_setTranslateZero",
   "BASE_setTranslateAcceleration",
   "BASE_setTranslateVelocity",
   "BASE_batteryCurrent",
   "BASE_batteryVoltage",
   "BASE_rotateCurrent",
   "BASE_rotateWhere",
   "BASE_translateCurrent",
   "BASE_translateWhere"};


#else
  extern char *baseMessageNamesXXX[];
#endif
