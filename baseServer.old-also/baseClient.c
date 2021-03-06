
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
 ***** Source file:     $Source: /home/rhino/rhino/CVS/bee/src/baseServer/baseClient.c,v $
 *****
 ***** Created by:      $Author: swa $
 *****
 ***** Revision #:      $Revision: 1.23 $
 *****
 ***** Date of revision $Date: 1998/01/15 16:30:32 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: baseClient.c,v $
 * Revision 1.23  1998/01/15 16:30:32  swa
 * Minor modifications towards enabling client-reconnects. Not yet fully
 * implemented.
 *
 * Revision 1.22  1997/10/21 21:14:32  tyson
 * minor fixes
 *
 * Revision 1.21  1997/09/09 16:11:55  tyson
 * Minor cleanup and debug for 1.1 release.
 *
 * Revision 1.20  1997/07/30 21:01:59  thrun
 * baseServer now sends confirmations of all motion commands. Is used
 * for logging in the module "learn"
 *
 * Revision 1.19  1997/07/24 15:27:16  tyson
 * added bNetwork.h, combined examples into on directory.  See Changes for other details
 *
 * Revision 1.18  1997/07/17 17:31:40  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.17  1997/06/21 22:36:07  thrun
 * Blocking is now an option when connecting to base server
 *
 * Revision 1.16  1997/04/11 18:56:51  tyson
 * minor fixes and chasing TCX segv
 *
 * Revision 1.15  1997/03/26 23:49:03  thrun
 * changed TCX_RECV_NONE to TCX_RECV_ALL
 *
 * Revision 1.14  1997/03/25 21:44:37  tyson
 * Many bug fixes.
 *
 * Revision 1.13  1997/03/25 01:23:45  thrun
 * .
 *
 * Revision 1.12  1997/03/25 01:13:55  thrun
 * new variable initialization.
 *
 * Revision 1.11  1997/03/24 20:03:08  thrun
 * Fixed a typo in the declarations. (swa)
 *
 * Revision 1.10  1997/03/11 17:03:17  tyson
 * added IR simulation and other work
 *
 * Revision 1.9  1997/02/27 15:59:56  tyson
 * upgrades to wander and bXxxxAngle() functions
 *
 * Revision 1.8  1997/02/25 18:12:37  tyson
 * client lib for PANTILT and lots of little stuff
 *
 * Revision 1.7  1997/02/02 22:32:26  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.6  1997/01/23 18:02:28  thrun
 * Fixed a few minor things that crashed the gcc on a SUN. Should not change
 * the functionality of the software.
 *
 * Revision 1.5  1997/01/15 06:22:51  tyson
 * Unified Robot.h, fixed buglettes in baseServer and wander
 *
 * Revision 1.4  1996/12/31 20:23:19  tyson
 * B14 fixes and other minor work
 *
 * Revision 1.3  1996/12/01 05:39:31  tyson
 * More simulator <-> baseServer work
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


 
#ifndef lint
static char rcsid[] =
"$Id: baseClient.c,v 1.23 1998/01/15 16:30:32 swa Exp $";
static void *const use_rcsid = ((void)&use_rcsid, (void)&rcsid, 0);
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include <tcx.h>
#include <rai.h>
#include <baseServer.h>
#include <baseClient.h>
#include <bUtils.h>


extern void tcxRegisterCloseHnd(void (*closeHnd)());

#undef VERBOSE 

#define BASE_CLIENT_NAME "baseclient"
#define BASE_CLIENT_HANDLERS  {   \
  {BASE_FIXED_MESSAGE,"BaseFxd",handleBaseClientFixed, TCX_RECV_ALL,NULL}, \
  {BASE_VARIABLE_MESSAGE,"BaseVar",handleBaseClientVariable, TCX_RECV_ALL,NULL}}

TCX_MODULE_PTR  baseServer  = NULL;

baseEventHandler baseClientFcn=NULL;
sonarCallbackType userSonarFcn=NULL;
irCallbackType userIrFcn=NULL; 
tactileCallbackType userTactileFcn=NULL; 
odoCallbackType userOdoFcn = NULL;


statusHandler statusClientFcn=NULL;
watchdogHandler userWatchdogFcn=NULL;
commandConfirmationHandler commandConfirmationClientFcn = NULL;

/*
 *  Globals which are exported by the generic sensor API. We
 *  must replicate them here, and then get the actual info
 *  from the base server and fill them in
 */

statusReportType  activeStatusReport;

sonarType sonars[B_MAX_SENSOR_COLS];

irType irs[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];

tactileType tactiles[B_MAX_SENSOR_ROWS][B_MAX_SENSOR_COLS];


/* Pointer references to above for calling usr callbacks */
irType  * irPtr[B_MAX_SENSOR_ROWS];
tactileType  * tactilePtr[B_MAX_SENSOR_ROWS];

/**********************************************************************
 *
 * Vars for functions with consistent units and coordinate systems
 *
 *********************************************************************/

float bOriginX = 0.0;
float bOriginY = 0.0;
float bOriginHeading = 0.0;

int bOdometryLock = B_ODOMETRY_OTHER_HAS_LOCK;

void odometryLockNotify(unsigned long param);


/****************** FUNCTIONS FOR SENDING TO THE SERVER *************/

void baseSendServerFixed(int operation,unsigned long arg)
{
  RAI_FixedMsgType command;
  command.operation = operation;
  command.parameter = arg;
  tcxSendMsg(baseServer,BASE_FIXED_MESSAGE,&command);
}


void baseSendServerVariable(int operation, unsigned char *data, int size) {
  RAI_VariableMsgType command;

  command.operation = operation;
  command.bufsize = size;
  command.buffer = data;

  tcxSendMsg(baseServer, BASE_VARIABLE_MESSAGE, &command);
}

/*** General ****/


void registerBaseCallback(baseEventHandler fcn)
{
  baseClientFcn=fcn;
}

void registerStatusCallback(statusHandler fcn)
{
  statusClientFcn=fcn;
}

void sonarInit(void) {
}

void registerSonarCallback(sonarCallbackType fcn)
{
 userSonarFcn=fcn;
}


void irInit(void) {
  int i, j;

  for (i = 0; i < bRobot.ir_rows; i++) {
    for (j = 0; j < bRobot.ir_cols[i]; j++){
      irs[i][j].value      = 0;
      irs[i][j].mostRecent = 0;
      irs[i][j].time.tv_sec = 0;
      irs[i][j].time.tv_usec = 0;
    }
  }
}

void registerIrCallback(irCallbackType fcn)
{
  userIrFcn=fcn;
}

void irStart(void) {
}

void irStop(void) {
}

void registerWatchdogCallback(watchdogHandler fcn)
{
 userWatchdogFcn=fcn;
}

void registerCommandConfirmationCallback(commandConfirmationHandler fcn)
{
  commandConfirmationClientFcn=fcn;
}

void tactileInit(void) 
{
  int i, j;
  
  for (i = 0; i < bRobot.tactile_rows; i++) {
    for (j = 0; j < bRobot.tactile_cols[i]; j++){
      tactiles[i][j].value         = (char) 0;
      tactiles[i][j].mostRecent    = FALSE;
      tactiles[i][j].time.tv_sec   = 0;
      tactiles[i][j].time.tv_usec  = 0;
    }
  }
}

void registerTactileCallback(tactileCallbackType fcn)
{
 userTactileFcn=fcn;
}


/*
 * Each member of the API on the client side consists of a function which
 * just sends the corresponding message (fcn name with BASE_ prepended
 * and perhaps argument, to the base server.  So I wrote macros to form
 * the functions
 */

#define serverCmd(name) void name(void) {baseSendServerFixed(BASE_ ## name,0);}
#define serverArgCmd(name) void name(unsigned long arg) \
 {baseSendServerFixed(BASE_ ## name,arg);}


/*** GENERAL ****/
serverCmd(baseKill)
serverArgCmd(loadHeading)
serverArgCmd(loadPosition)
serverArgCmd(statusReportData)
serverArgCmd(statusReportPeriod)
serverArgCmd(watchdogTimer);
serverCmd(assumeWatchdog);

serverCmd(batteryVoltage);
serverCmd(batteryCurrent);

/*** ROTATION ****/
serverCmd(rotateLimp)
serverCmd(rotateHalt)
serverCmd(rotateVelocityPos)
serverCmd(rotateVelocityNeg)
serverArgCmd(rotateRelativePos)
serverArgCmd(rotateRelativeNeg)
serverArgCmd(rotateTorquePos)
serverArgCmd(rotateTorqueNeg)
serverArgCmd(rotatePowerPos)
serverArgCmd(rotatePowerNeg)
serverArgCmd(rotateToPosition) 
serverCmd(findRotIndex)

serverArgCmd(setRotateFriction)
serverArgCmd(setRotateVelocity)
serverArgCmd(setRotateAcceleration)
serverArgCmd(setRotateSlope)
serverArgCmd(setRotateTorque)
serverArgCmd(setRotateZero)

serverCmd(rotateCurrent)
serverCmd(rotateWhere)


/*** TRANSLATION ****/

serverCmd(translateLimp)
serverCmd(translateHalt)
serverCmd(translateVelocityPos)
serverCmd(translateVelocityNeg)
serverArgCmd(translateRelativePos)
serverArgCmd(translateRelativeNeg)
serverArgCmd(translateTorquePos)
serverArgCmd(translateTorqueNeg)
serverArgCmd(translatePowerPos)
serverArgCmd(translatePowerNeg)
serverArgCmd(translateToPosition) 

serverArgCmd(setTranslateVelocity)
serverArgCmd(setTranslateAcceleration)
serverArgCmd(setTranslateSlope)
serverArgCmd(setTranslateTorque)
serverArgCmd(setTranslateZero)

serverCmd(translateCurrent)
serverCmd(translateWhere)

/*** SONARS ***/
serverCmd(sonarStart)
serverCmd(sonarStop)


/*------------------------------------------------------------*/

/****************** SUPPORT FUNCTIONS FOR SERVER RESPONSES *************/

/* sonarLongs alternate between the ping transducer index and ping value */
void
incorporateSonars(long sonarLongs[])
{
  int longNum=0;
  long index;
  struct timeval time;

  gettimeofday(&time, NULL);

  for (index = 0; index < bRobot.sonar_cols[0]; index++) {
    sonars[index].mostRecent = FALSE;
  }

  for(index=ntohl(sonarLongs[longNum++]);
      index >= 0;
      index=ntohl(sonarLongs[longNum++]))
    {
      sonars[index].value = ntohl(sonarLongs[longNum++]);
      sonars[index].mostRecent = TRUE;
      sonars[index].time.tv_sec   = time.tv_sec;
      sonars[index].time.tv_usec  = time.tv_usec;
    }

  if (userSonarFcn != NULL) {
    userSonarFcn(sonars);
  }
}

/*------------------------------------------------------------*/

/* irLongs are <row,col,value> .... */
void
incorporateIrs(unsigned long irLongs[])
{
  int longNum=0;
  int row;
  int col;
  int i, j;
  struct timeval time;

  gettimeofday(&time, NULL);

  for (i = 0; i < bRobot.ir_rows; i++) {
    for (j = 0; j < bRobot.ir_cols[i]; j++){
      irs[i][j].mostRecent = FALSE;
    }
  }

  /*
   * the server will pass a row number < 0 as a flag
   * after all real data
   */

  for(row = ntohl(irLongs[longNum++]); row >= 0;
      row= ntohl(irLongs[longNum++])) {
    col = ntohl(irLongs[longNum++]);
    irs[row][col].value = ntohl(irLongs[longNum++]);
    irs[row][col].mostRecent = TRUE;
    irs[row][col].time.tv_sec   = time.tv_sec;
    irs[row][col].time.tv_usec  = time.tv_usec;
  }
  
  if (userIrFcn != NULL) {
    userIrFcn(irPtr);
  }
}

/*------------------------------------------------------------*/

/* tactileLongs are <row,col,value> .... */
void
incorporateTactiles(long tLongs[])
{
  int longNum=0;
  int row;
  int i, j;
  int col;
  struct timeval time;

  gettimeofday(&time, NULL);

  for (i = 0; i < bRobot.tactile_rows; i++) {
    for (j = 0; j < bRobot.tactile_cols[i]; j++){
      tactiles[i][j].mostRecent    = FALSE;
    }
  }

  /*
   * the server will pass a row number < 0 as a flag
   * after all real data
   */

  for(row = ntohl(tLongs[longNum++]);
      row >= 0;
      row = ntohl(tLongs[longNum++]))
    {
      col = ntohl(tLongs[longNum++]);
      tactiles[row][col].value = ntohl(tLongs[longNum++]);
      tactiles[row][col].mostRecent    = TRUE;
      tactiles[row][col].time.tv_sec   = time.tv_sec;
      tactiles[row][col].time.tv_usec  = time.tv_usec;
    }

  if (userTactileFcn != NULL) {
    userTactileFcn(tactilePtr);
  }
}

/*------------------------------------------------------------*/

void
incorporateStatusReport(statusReportType* newReport)
{
  /*
   * NOTE!
   *
   * This only works because the entire struct
   * is longs and not a mix of types.  If the struct becomes
   * other than all longs this will likely fail do to different
   * architectures packing structures differently.  -tds
   */

  activeStatusReport.Request            = ntohl(newReport->Request);
  activeStatusReport.Clock              = ntohl(newReport->Clock);
  activeStatusReport.GeneralStatus      = ntohl(newReport->GeneralStatus);
  activeStatusReport.Xpos               = ntohl(newReport->Xpos);
  activeStatusReport.Ypos               = ntohl(newReport->Ypos);
  activeStatusReport.Heading            = ntohl(newReport->Heading);
  activeStatusReport.BaseRelativeHeading
                                   = ntohl(newReport->BaseRelativeHeading);
  activeStatusReport.TranslateError     = ntohl(newReport->TranslateError);
  activeStatusReport.TranslateVelocity  = ntohl(newReport->TranslateVelocity);
  activeStatusReport.TranslateStatus    = ntohl(newReport->TranslateStatus);
  activeStatusReport.RotateError        = ntohl(newReport->RotateError);
  activeStatusReport.RotateVelocity     = ntohl(newReport->RotateVelocity);
  activeStatusReport.RotateStatus       = ntohl(newReport->RotateStatus);

  if (statusClientFcn != NULL) {
    statusClientFcn(&activeStatusReport);
  }
}

/*------------------------------------------------------------*/

void
incorporateCommandConfirmation(confirmCommandDataType* newConfirmData)
{
  confirmCommandDataType confirmData;

  confirmData.operation = ntohl(newConfirmData->operation);
  confirmData.param     = ntohl(newConfirmData->param);

  /* fprintf(stderr, " [%d:%d] ", (int) confirmData.operation, 
     (int) confirmData.param); */

  if (commandConfirmationClientFcn != NULL)
    commandConfirmationClientFcn(&confirmData);
}

/*------------------------------------------------------------*/

/****************** HANDLING RESPONSES FROM THE SERVER *************/

void
baseDispatchFixedMessage(int operation, unsigned long param)
{
  switch(operation)
    {
    case BASE_watchdogTimeout:
      if(userWatchdogFcn!= NULL)
	userWatchdogFcn();
      else
	fprintf(stderr,"BaseClient: Missed watchdog timeout\n");
      break;
      
    case BASE_odometryLockNotify:
      odometryLockNotify(param);
      break;
      
    case BASE_odometryChangeX:
      ntohf(param, bOriginX);
      break;
      
    case BASE_odometryChangeY:
      ntohf(param, bOriginY);
      break;
      
    case BASE_odometryChangeH:
      ntohf(param, bOriginHeading);
      break;

    default:
      if(baseClientFcn!= NULL)
	baseClientFcn(operation,param);
      else
	fprintf(stderr,"Base client received %d,%lu from server\n",
		operation,param);
      break;
    }
}

/*------------------------------------------------------------*/

void
baseDispatchVariableMessage(int operation, int bufsize, char buffer[])
{

  switch(operation)
    {
    case TACTILE_newValues:
      {
	incorporateTactiles( (long*) buffer);
	break;
      }

    case SONAR_newValues:
      {
	incorporateSonars( (unsigned long*) buffer);
	break;
      }

    case IR_newValues:
      {
	incorporateIrs( (unsigned long*) buffer);
	break;
      }

    case BASE_statusReport:
      {
	incorporateStatusReport( (statusReportType*) buffer);
	break;
      }

    /*
     * The following message echoes all commands sent to the baseServer,
     * to every baseClient. It can be used for logging commands.
     * Implemented by Sebastian, 7-97
     */
    case BASE_confirmCommand:
      {
	incorporateCommandConfirmation( (confirmCommandDataType*) buffer);
	break;
      }
      

    default:
      fprintf(stderr, "BaseClient: Variable opn %d,%d not yet implemented.\n",
              operation, bufsize);
      break;
    }
}

/*------------------------------------------------------------*/

void
handleBaseClientFixed(TCX_REF_PTR message,RAI_FixedMsgType *msg_ptr)
{
  int operation;
  unsigned long param;

  operation = msg_ptr->operation;
  param = msg_ptr->parameter;

#ifdef VERBOSE
    fprintf(stderr,"baseClient fixed msg %d %lu\n",operation,param); 
#endif
  baseDispatchFixedMessage(operation,param);
  tcxFreeByRef(message,msg_ptr);
}

/*------------------------------------------------------------*/

void
handleBaseClientVariable(TCX_REF_PTR message,RAI_VariableMsgType *msg_ptr)
{
  int operation;
  int bufsize;
  unsigned char* buffer;
  operation = msg_ptr->operation;
  bufsize = msg_ptr->bufsize;
  buffer = msg_ptr->buffer;

#ifdef VERBOSE
    fprintf(stderr,"baseClient received var msg %d %d\n",operation,bufsize); 
#endif
  baseDispatchVariableMessage(operation,bufsize,buffer);
  tcxFreeByRef(message,msg_ptr);
}

/*============================================================*/

/******************* SETUP ***************************/

void
baseInit(void *disconnectCallback)
{
  int numberOfMessages;
  int numberOfHandlers;
  int i;

  TCX_REG_MSG_TYPE messages[] = { BASE_MESSAGES }; 
  TCX_REG_HND_TYPE handlers[] = BASE_CLIENT_HANDLERS;
  
  numberOfMessages = sizeof(messages)/sizeof(TCX_REG_MSG_TYPE); 
  numberOfHandlers = sizeof(handlers)/sizeof(TCX_REG_HND_TYPE); 

  registerInterface(BASE_SERVER_NAME,numberOfMessages,messages,
		    numberOfHandlers,handlers);


  /* so we can pass an  irType** to user functions */
  for(i = 0; i < bRobot.ir_rows; i++)
    irPtr[i]=irs[i];

  for(i = 0; i < bRobot.tactile_rows; i++)
    tactilePtr[i]=tactiles[i];
}

/*------------------------------------------------------------*/

int
baseConnect(int blocking)
{
  if (blocking) {
    fprintf(stderr, "BaseClient: Connecting to baseServer...\n");
    baseServer = tcxConnectModule(BASE_SERVER_NAME);
  }
  else {
    fprintf(stderr, "BaseClient: Connecting to baseServer...\n");
    baseServer = tcxConnectOptional(BASE_SERVER_NAME);
  }

  if (baseServer) {
    baseSendServerFixed(BASE_subscribe, 0);  
    fprintf(stderr, "BaseClient: Connected.\n");
    return(1);
  }
  else {
    return(0);
  }
}

/*============================================================*/

void
registerBaseClient()
{
  baseInit(NULL);
  sonarInit();
  irInit();
  tactileInit();
}

/*------------------------------------------------------------*/

int
findBaseServer(void)
{
  return(baseConnect(1));
}

/*============================================================*/

/**********************************************************************
 *
 * Functions with consistent units and coordinate systems
 *
 *********************************************************************/

float
bOdoToRobotX(float X, float Y)
{
  float X1, Y1;

  X1 = X - bOriginX;
  Y1 = Y - bOriginY;

  X = X1 * cos(bOriginHeading) + Y1 * sin(bOriginHeading);

  return(X);
}

/*------------------------------------------------------------*/

float
bOdoToRobotY(float X, float Y)
{
  float X1, Y1;

  X1 = X - bOriginX;
  Y1 = Y - bOriginY;

  Y = X1 * sin(-bOriginHeading) + Y1 * cos(bOriginHeading);

  return(Y);
}

/*------------------------------------------------------------*/

float
bRobotToOdoX(float X, float Y)
{
  float X1;

  X1 = X*cos(bOriginHeading) + Y*sin(-bOriginHeading);
  X = X1 + bOriginX;
  return(X);
}

/*------------------------------------------------------------*/

float
bRobotToOdoY(float X, float Y)
{
  float Y1;

  Y1 = X*sin(bOriginHeading) + Y*cos(bOriginHeading);
  Y = Y1 + bOriginY;
  return(Y);
}

/*------------------------------------------------------------*/

float
bNormalizeAngle(float radians)
{
  while(radians>M_PI) {
    radians -= 2.0*M_PI;
  }
  
  while(radians<-M_PI) {
    radians += 2.0*M_PI;
  }

  return(radians);
}

/*------------------------------------------------------------*/

void
bSetVel(float rotVel, float transVel)
{
  setRotateVelocity(abs((int)(rotVel*512.0/M_PI)));

  if (rotVel == 0.0) {
    rotateHalt();
  }
  else if (rotVel < 0.0) {
    rotateVelocityPos();
  }
  else {
    rotateVelocityNeg();
  }
  
  setTranslateVelocity(abs((int)(transVel*10.0)));

  if (transVel == 0.0) {
    translateHalt();
  }
  else if (transVel > 0.0) {
    translateVelocityPos();
  }
  else {
    translateVelocityNeg();
  }

  return;
}

/*------------------------------------------------------------*/

int bSetPosition(float X, float Y, float heading)
{
  float test1, test2, test3;
  float newX, newY, newH;
  double X1, Y1;

  if (bOdometryLock == B_ODOMETRY_I_HAVE_LOCK) {
    newH = bNormalizeAngle((M_PI/2.0) - 
                        (float)activeStatusReport.Heading*M_PI/512.0
		    - heading);

    X1 = X*cos(newH) + Y*sin(-newH);
    Y1 = X*sin(newH) + Y*cos(newH);

    newX = (float)activeStatusReport.Xpos/10.0 - X1;
    newY = (float)activeStatusReport.Ypos/10.0 - Y1;

    /* Now have to send these new values to the base server */
    htonf(newX, test1);
    htonf(newY, test2);
    htonf(newH, test3);
    baseSendServerFixed(BASE_odometryChangeX, test1);
    baseSendServerFixed(BASE_odometryChangeY, test2);
    baseSendServerFixed(BASE_odometryChangeH, test3);

    return 1;
  } else {
    return 0;
  }
}

/*------------------------------------------------------------*/

void
bUpdatePosition(float dX, float dY, float dHeading)
{
  float X, Y, TH;

  X = bRobotX(0) + dX;
  Y = bRobotY(0) + dY;
  TH = bNormalizeAngle(bRobotHeading(0) + dHeading);

  bSetPosition(X, Y, TH);

  return;
}

/*------------------------------------------------------------*/

float
bGetRotVel(void)
{
  if (activeStatusReport.RotateStatus & 0x800) {
    return((float)activeStatusReport.RotateVelocity*M_PI/512.0);
  }
  else {
    return(-(float)activeStatusReport.RotateVelocity*M_PI/512.0);
  }
}

/*------------------------------------------------------------*/

float
bGetTransVel(void)
{
  if (activeStatusReport.TranslateStatus & 0x800) {
    return(-(float)activeStatusReport.TranslateVelocity/10.0);
  }
  else {
    return((float)activeStatusReport.TranslateVelocity/10.0);
  }
}

/*------------------------------------------------------------*/

float
bRobotHeading(float dt)
{
  float heading;

  heading= bNormalizeAngle((M_PI/2.0) - ((float)activeStatusReport.Heading)
			   * M_PI / 512.0 - bOriginHeading + 
			   bGetRotVel() * dt);

  return(heading);
}

/*------------------------------------------------------------*/

float
bRobotX(float dt)
{
  float X1, Y1;
  float X;

  X1 = (float)((signed long)activeStatusReport.Xpos);
  X1 = X1/10.0 - bOriginX;

  Y1 = (float)((signed long)activeStatusReport.Ypos);
  Y1 = Y1/10.0 - bOriginY;

  X = X1 * cos(bOriginHeading) + Y1 * sin(bOriginHeading);

  X = X + bGetTransVel() * dt * cos(bRobotHeading(dt/2.0));

  return(X);
}

/*------------------------------------------------------------*/

float
bRobotY(float dt)
{
  float X1, Y1;
  float Y;

  X1 = (float)((signed long)activeStatusReport.Xpos);
  X1 = X1/10.0 - bOriginX;

  Y1 = (float)((signed long)activeStatusReport.Ypos);
  Y1 = Y1/10.0 - bOriginY;

  Y = X1 * sin(-bOriginHeading) + Y1 * cos(bOriginHeading);

  Y = Y + bGetTransVel() * dt * sin(bRobotHeading(dt/2.0));

  return(Y);
}

/*------------------------------------------------------------*/

float
bWorldAngle(float robotAngle, float dt)
{
  return(robotAngle+bRobotHeading(dt));
}

/*------------------------------------------------------------*/

float
bRobotAngle(float worldAngle, float dt)
{
  return(worldAngle-bRobotHeading(dt));
}

/*------------------------------------------------------------*/

/*
 * NOTE!
 * 
 * There are assumptions here about the sensor geometry and
 * the robot type.
 *
 */

float
bSonarAngle(int sonarRow, int sonarCol)
{
  float robotAngle;

  if (sonarRow>0) {
    return(0.0);
  }

  if (sonarCol >= bRobot.sonar_cols[0]) {
    return(0.0);
  }

  robotAngle = (((float)sonarCol+.5)*
		(1024.0/(float)bRobot.sonar_cols[0]) - 512.0);
  robotAngle = -robotAngle*M_PI/512.0;

  return(robotAngle);
}

/*------------------------------------------------------------*/

/*
 * NOTE!
 * 
 * There are assumptions here about the sensor geometry and
 * the robot type.
 *
 */

float
bIrAngle(int irRow, int irCol)
{
  float robotAngle = 0.0;

  if (irRow >= bRobot.ir_rows) {
    return(0.0);
  }

  if (irCol >= bRobot.ir_cols[irRow]) {
    return(0.0);
  }

  switch (irRow) {
  case 0:
    robotAngle = (((float)irCol+.5) *
		  (1024.0/(float)bRobot.ir_cols[irRow]) - 512.0);
    robotAngle = -robotAngle*M_PI/512.0;
    break;

  case 1:
    robotAngle = (((float)irCol+0.7) * (1024.0/(float)bRobot.ir_cols[irRow])
		  + activeStatusReport.BaseRelativeHeading 
		  - activeStatusReport.Heading);
    robotAngle = M_PI - robotAngle*M_PI/512.0;
    break;

  case 2:
    robotAngle = (((float)irCol+0.05) * (1024.0/(float)bRobot.ir_cols[irRow])
		  + activeStatusReport.BaseRelativeHeading 
		  - activeStatusReport.Heading);
    robotAngle = M_PI - robotAngle*M_PI/512.0;
    break;
  }

  return(robotAngle);
}

/*------------------------------------------------------------*/

/*
 * NOTE!
 * 
 * There are assumptions here about the sensor geometry and
 * the robot type.
 *
 */

float
bTactileAngle(int tactileRow, int tactileCol)
{
  float robotAngle = 0.0;

  if (tactileRow >= bRobot.tactile_rows) {
    return(0.0);
  }

  if (tactileCol >= bRobot.tactile_cols[tactileRow]) {
    return(0.0);
  }

  switch (tactileRow) {
  case 0:
  case 1:
    robotAngle = (((float)tactileCol+.5) *
		  (1024.0/(float)bRobot.tactile_cols[tactileRow]) - 512.0);
    robotAngle = -robotAngle*M_PI/512.0;
    break;

  case 2:
  case 3:
    robotAngle = ((float)tactileCol+0.5) * 
		   (1024.0/(float)bRobot.tactile_cols[tactileRow]);

    robotAngle += activeStatusReport.BaseRelativeHeading;
    robotAngle -= activeStatusReport.Heading;

    robotAngle = M_PI - robotAngle*M_PI/512.0;

    break;
  }

  return(robotAngle);
}

/*------------------------------------------------------------*/

void bRegisterOdometryLockCallback(odoCallbackType fcn)
{
  userOdoFcn = fcn;
}

/*------------------------------------------------------------*/

/*
 * This is a short to make a smoother transition when we ditch tcx
 */

void bRequestOdometryLock(unsigned short priority) 
{
  baseSendServerFixed(BASE_requestOdometryLock, (unsigned long)priority);
}

/*------------------------------------------------------------*/

void bReleaseOdometryLock() 
{
  baseSendServerFixed(BASE_releaseOdometryLock, 0);
}

/*------------------------------------------------------------*/

void odometryLockNotify(unsigned long param)
{
  int new_lock = (int)param;

  if (new_lock != bOdometryLock) {
    bOdometryLock = new_lock;

    if (userOdoFcn) {
      userOdoFcn(new_lock);
    }
  }
}
