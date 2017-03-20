
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/communication.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 17:06:29 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: communication.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.67  2000/03/06 20:00:42  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.66  2000/01/26 22:56:42  fox
 * Might work. If anything weird happens, contact me.
 *
 * Revision 1.65  1999/11/02 18:12:32  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.64  1999/10/21 17:30:42  fox
 * Fixed some bugs in real time scripts.
 *
 * Revision 1.63  1999/09/29 16:06:09  fox
 * Should work.
 *
 * Revision 1.62  1999/09/26 21:20:56  fox
 * Nothing special.
 *
 * Revision 1.61  1999/09/09 15:27:59  fox
 * Nothing special.
 *
 * Revision 1.60  1999/09/09 02:48:36  fox
 * Final version before germany.
 *
 * Revision 1.59  1999/09/03 22:22:38  fox
 * Changed hadnling of real time script. This version contains both.
 *
 * Revision 1.58  1999/09/03 13:43:35  fox
 * Nothing special.
 *
 * Revision 1.57  1999/09/01 00:02:56  fox
 * Getting closer. But still not close enough.
 *
 * Revision 1.56  1999/08/30 05:48:41  fox
 * Doesn't work!!
 *
 * Revision 1.55  1999/08/27 22:22:31  fox
 * Better communication for multi robot localization.
 *
 * Revision 1.54  1999/06/29 21:36:21  fox
 * Changes for new script reader.
 *
 * Revision 1.53  1999/06/24 00:21:49  fox
 * Some changes for the urbies.
 *
 * Revision 1.52  1999/05/21 14:48:46  fox
 * Added SEND_REPORT keyword.
 *
 * Revision 1.51  1999/05/18 15:15:18  fox
 * Added keywords to determine whether correction parameters should be sent to map and plan or not.
 *
 * Revision 1.50  1999/04/29 13:35:19  fox
 * Further adaptation to make multi localize run.
 *
 * Revision 1.49  1999/04/26 18:55:38  fox
 * Communication with sampling seems to work (no more stuck situations).
 *
 * Revision 1.48  1999/03/12 00:41:48  fox
 * Minor changes.
 *
 * Revision 1.47  1999/03/08 16:47:39  wolfram
 * Fixed problems with map, caused by the fact that it no longer accpets
 * correction parameters from LOCALIZE.  Improved the handling of sonars
 * with Pioneers.  Currently sonars do not work properly on a B21.
 *
 * Revision 1.46  1999/02/17 19:42:22  fox
 * Enhanced gif utilities.
 *
 * Revision 1.45  1999/02/05 23:02:42  fox
 * Minor changes for samples.
 *
 * Revision 1.44  1999/02/01 21:52:21  fox
 * Added support for dumping gif files.
 *
 * Revision 1.43  1999/01/22 18:10:38  fox
 * Removed some hacks done for denver and the sampling paper.
 *
 * Revision 1.42  1999/01/22 17:48:00  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.41  1999/01/11 19:47:46  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.40  1998/11/19 03:14:23  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.39  1998/10/29 03:44:58  fox
 * Nothing special.
 *
 * Revision 1.38  1998/09/18 15:44:24  fox
 * First attempt to get a running system after museum.
 *
 * Revision 1.37  1998/08/26 15:34:02  wolfram
 * Finished integration of vision
 *
 * Revision 1.36  1998/08/23 00:00:58  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.35  1998/08/20 00:22:56  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.34  1998/08/19 16:33:53  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.33  1998/04/08 21:59:09  wolfram
 * Completed multi-robot support. Added neat stuff for nick.
 *
 * Revision 1.32  1998/04/08 16:35:59  wolfram
 * Changed the concept of multi-robot support. TCX now contains a procedure to
 * generate module names.
 *
 * Revision 1.31  1998/04/07 18:31:49  wolfram
 * Removed Multi calls from tcx. Version runs. Major rework has to be
 * done in order to support multiple robots.
 *
 * Revision 1.30  1998/04/06 19:44:11  wolfram
 * Added -robot option for multi-robot support
 *
 * Revision 1.29  1998/03/04 14:46:42  fox
 * This version should run.
 *
 * Revision 1.28  1998/02/12 15:47:19  derr
 * compiler warnings (unused variable ... ) removed
 *
 * Revision 1.27  1998/01/22 13:06:11  fox
 * First version after selection-submission.
 *
 * Revision 1.26  1998/01/05 10:37:09  fox
 * Several changes especially in selection parts.
 *
 * Revision 1.25  1997/12/09 12:03:22  wolfram
 * Added support for Marker in scripts
 *
 * Revision 1.24  1997/12/02 15:20:35  fox
 * Nothing remarkable.
 *
 * Revision 1.23  1997/11/20 12:58:08  fox
 * Version with good sensor selection.
 *
 * Revision 1.22  1997/11/07 12:39:37  fox
 * Added some graphic features.
 *
 * Revision 1.21  1997/09/29 10:45:22  wolfram
 * Added default values for all values in .ini file
 *
 * Revision 1.20  1997/09/26 17:02:08  wolfram
 * Intermediate version, do not use
 *
 * Revision 1.19  1997/08/22 04:16:37  fox
 * Final version before IJCAI.
 *
 * Revision 1.18  1997/08/16 22:59:49  fox
 * Last version before I change selsection.
 *
 * Revision 1.17  1997/07/04 17:29:12  fox
 * Final version before holiday!!!
 *
 * Revision 1.16  1997/06/25 14:16:38  fox
 * Changed laser incorporation.
 *
 * Revision 1.15  1997/06/20 07:36:08  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.14  1997/05/27 07:42:30  fox
 * Nothing special.
 *
 * Revision 1.13  1997/05/09 16:28:17  fox
 * Nothing special.
 *
 * Revision 1.12  1997/04/27 15:48:20  wolfram
 * Changes in script.c
 *
 * Revision 1.11  1997/03/13 17:36:20  fox
 * Temporary version. Don't use!
 *
 * Revision 1.10  1997/01/30 13:34:11  fox
 * Minor changes.
 *
 * Revision 1.9  1997/01/29 12:23:02  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.8  1997/01/16 19:43:21  fox
 * And another bug ...
 *
 * Revision 1.7  1997/01/03 10:09:42  fox
 * First version with exploration.
 *
 * Revision 1.6  1996/12/20 15:29:35  fox
 * Added four parameters.
 *
 * Revision 1.5  1996/12/13 13:55:36  fox
 * Implemented connection to PLAN.
 *
 * Revision 1.4  1996/12/02 10:32:01  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.3  1996/10/24 12:07:08  fox
 * Fixed a bug.
 *
 * Revision 1.2  1996/10/24 09:56:52  fox
 * LOCALIZE also works in a write protected directory.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:31  rhino
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

#include <stdio.h>
#include <string.h>
#include <unistd.h>


#include "general.h"
#include "script.h"
#include "localTcx.h"
#include "planTcx.h"
#include "file.h"
#include "vision.h"
#include "movement.h"
#include "abstract.h"
#include "communication.h"

int robotType = B21_TWO_LASERS_ROBOT;


/* This struct contains all relevant information for communication
 * with the base or a script process. */
informationsFor_COMMUNICATION communicationInfo;

#define USE_TCX_TOKEN                            0
#define CONNECT_TO_BASE_TOKEN                    1
#define SUBSCRIBE_PROXIMITY_REPORT_TOKEN         2
#define SUBSCRIBE_BASE_REPORT_TOKEN              3
#define SCRIPT_TOKEN                             4
#define TIME_FACTOR_TOKEN                        5
#define LOG_FILE_TOKEN                           6
#define ODOMETRY_CORRECTION_TOKEN                7
#define ROBOT_NAME_TOKEN                         8
#define SUBSCRIBE_CAMERA_TOKEN                   9
#define REAL_TIME_SCRIPT_TOKEN                  10
#define SEND_CORRECTION_TO_PLAN_TOKEN           11
#define SEND_CORRECTION_TO_MAP_TOKEN            12
#define SEND_REPORTS_TOKEN                      13
#define ROBOT_TYPE_TOKEN                        14
#define TIME_TO_BE_SKIPPED_TOKEN                15
#define RUN_TIME_TOKEN                          16


#define TCX_USER_MODULE_NAME "LOCALIZE"

/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/

/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/
void
initialize_COMMUNICATION(  char* fileName,
			   int argc,
			   char **argv,
			   actionInformation* actionInfo,
			   sensingActionMask* actionMask)
{
  char scriptFile[MAX_STRING_LENGTH], directory[MAX_STRING_LENGTH],
    logFileName[MAX_STRING_LENGTH], buf[MAX_STRING_LENGTH];
  int i = 1;
  
  /*-------------------------------------------------------------------------
   * Get the parameters from the file.
   *------------------------------------------------------------------------*/
  token tok[NUMBER_OF_COMMUNICATION_PARAMETERS];

  /* disable warning */
  actionMask=actionMask;

  /*-------------------------------------------------------------------------
   * Initialization.
   *------------------------------------------------------------------------*/

  actionInfo->robotName = NULL;
  while (i < argc){
    if (strcmp(argv[i], "-robot") == 0){
      i++;
      if (i < argc && argv[i][0] != '-')
	actionInfo->robotName = argv[i];
      else{
	fprintf(stderr, "Error: wrong or missing robot name!");
	exit(0);
      }
    }
    i++;
  }
  sprintf(logFileName, "%s", DEFAULT_LOG_FILE);
  communicationInfo.useTcx = 1;
  communicationInfo.subscribeBaseReport = 2;
  communicationInfo.subscribeProximityReport = 2;
  communicationInfo.subscribeCameraReport = 0;
  communicationInfo.scr = (script*) malloc( sizeof( script));
  communicationInfo.connectBase = 1;
  communicationInfo.sendReports = FALSE;
  communicationInfo.sendCorrectionParametersToMap = actionInfo->onlineMapping;
  communicationInfo.sendCorrectionParametersToPlan = FALSE;

  getDirectory(directory);
  sprintf(scriptFile, "%s.script", directory);
  communicationInfo.scr->timeFactor = 1.0;
  communicationInfo.scr->odometryCorrection = FALSE;
  communicationInfo.scr->realTime = FALSE;
  communicationInfo.scr->timeToBeSkipped = 0.0;
  communicationInfo.scr->runTime = 0.0;

  setTokensInitialized(tok, NUMBER_OF_COMMUNICATION_PARAMETERS);

  tok[USE_TCX_TOKEN].format   = INT_FORMAT;
  tok[USE_TCX_TOKEN].variable = &( communicationInfo.useTcx);
  tok[USE_TCX_TOKEN].keyWord  = USE_TCX_KEYWORD;

  tok[CONNECT_TO_BASE_TOKEN].format   = INT_FORMAT;
  tok[CONNECT_TO_BASE_TOKEN].variable = &( communicationInfo.connectBase);
  tok[CONNECT_TO_BASE_TOKEN].keyWord  = CONNECT_TO_BASE_KEYWORD;

  tok[SUBSCRIBE_BASE_REPORT_TOKEN].format   = INT_FORMAT;
  tok[SUBSCRIBE_BASE_REPORT_TOKEN].variable = 
&( communicationInfo.subscribeBaseReport);
  tok[SUBSCRIBE_BASE_REPORT_TOKEN].keyWord  = SUBSCRIBE_BASE_REPORT_KEYWORD;

  tok[SUBSCRIBE_PROXIMITY_REPORT_TOKEN].format   = INT_FORMAT;
  tok[SUBSCRIBE_PROXIMITY_REPORT_TOKEN].variable = 
&( communicationInfo.subscribeProximityReport);
  tok[SUBSCRIBE_PROXIMITY_REPORT_TOKEN].keyWord  = SUBSCRIBE_PROXIMITY_REPORT_KEYWORD;

  tok[SCRIPT_TOKEN].format   = STRING_FORMAT;
  tok[SCRIPT_TOKEN].variable = scriptFile;
  tok[SCRIPT_TOKEN].keyWord  = SCRIPT_KEYWORD;

  tok[TIME_FACTOR_TOKEN].format   = FLOAT_FORMAT;
  tok[TIME_FACTOR_TOKEN].variable = &( communicationInfo.scr->timeFactor);
  tok[TIME_FACTOR_TOKEN].keyWord  = TIME_FACTOR_KEYWORD;

  tok[LOG_FILE_TOKEN].format   = STRING_FORMAT;
  tok[LOG_FILE_TOKEN].variable = logFileName;
  tok[LOG_FILE_TOKEN].keyWord  = LOG_FILE_KEYWORD;

  tok[ODOMETRY_CORRECTION_TOKEN].format   = INT_FORMAT;
  tok[ODOMETRY_CORRECTION_TOKEN].variable =
    &( communicationInfo.scr->odometryCorrection);
  tok[ODOMETRY_CORRECTION_TOKEN].keyWord  = ODOMETRY_CORRECTION_KEYWORD;

  tok[ROBOT_NAME_TOKEN].format   = STRING_FORMAT;
  tok[ROBOT_NAME_TOKEN].variable = buf;
  tok[ROBOT_NAME_TOKEN].keyWord  = ROBOT_NAME_KEYWORD;

  tok[SUBSCRIBE_CAMERA_TOKEN].format   = INT_FORMAT;
  tok[SUBSCRIBE_CAMERA_TOKEN].variable =
    &( communicationInfo.subscribeCameraReport);
  tok[SUBSCRIBE_CAMERA_TOKEN].keyWord  = SUBSCRIBE_CAMERA_KEYWORD;

  tok[REAL_TIME_SCRIPT_TOKEN].format   = INT_FORMAT;
  tok[REAL_TIME_SCRIPT_TOKEN].variable = &( communicationInfo.scr->realTime);
  tok[REAL_TIME_SCRIPT_TOKEN].keyWord  = REAL_TIME_SCRIPT_KEYWORD;

  tok[SEND_REPORTS_TOKEN].format   = INT_FORMAT;
  tok[SEND_REPORTS_TOKEN].variable = &( communicationInfo.sendReports);
  tok[SEND_REPORTS_TOKEN].keyWord  = SEND_REPORTS_KEYWORD;

  tok[SEND_CORRECTION_TO_PLAN_TOKEN].format   = INT_FORMAT;
  tok[SEND_CORRECTION_TO_PLAN_TOKEN].variable = &( communicationInfo.sendCorrectionParametersToPlan);
  tok[SEND_CORRECTION_TO_PLAN_TOKEN].keyWord  = SEND_CORRECTION_TO_PLAN_KEYWORD;

  tok[SEND_CORRECTION_TO_MAP_TOKEN].format   = INT_FORMAT;
  tok[SEND_CORRECTION_TO_MAP_TOKEN].variable = &( communicationInfo.sendCorrectionParametersToMap);
  tok[SEND_CORRECTION_TO_MAP_TOKEN].keyWord  = SEND_CORRECTION_TO_MAP_KEYWORD;

  tok[ROBOT_TYPE_TOKEN].format               = INT_FORMAT;
  tok[ROBOT_TYPE_TOKEN].variable             = &(robotType);
  tok[ROBOT_TYPE_TOKEN].keyWord              = ROBOT_TYPE_KEYWORD;
  
  tok[TIME_TO_BE_SKIPPED_TOKEN].format               = FLOAT_FORMAT;
  tok[TIME_TO_BE_SKIPPED_TOKEN].variable             = &(communicationInfo.scr->timeToBeSkipped);
  tok[TIME_TO_BE_SKIPPED_TOKEN].keyWord              = TIME_TO_BE_SKIPPED_KEYWORD;
  
  tok[RUN_TIME_TOKEN].format               = FLOAT_FORMAT;
  tok[RUN_TIME_TOKEN].variable             = &(communicationInfo.scr->runTime);
  tok[RUN_TIME_TOKEN].keyWord              = RUN_TIME_KEYWORD;
  
  readTokens( fileName, tok, NUMBER_OF_COMMUNICATION_PARAMETERS, FALSE);
  {
    int i;
    for ( i = 1; i < argc; i++) {
      if ((strcmp(argv[i], "-skip")==0)) {
	if ( i < argc - 1) {
	  communicationInfo.scr->timeToBeSkipped = atof(argv[++i]);
	  fprintf( stderr, "#Skip %f seconds.\n", communicationInfo.scr->timeToBeSkipped);
	  writeLog( "#Skip %f seconds.\n", communicationInfo.scr->timeToBeSkipped);
	}
	else {
	  fprintf( stderr, "#ERROR: time must follow keyword -skip.\n");
	  writeLog( "#ERROR: time must follow keyword -skip.\n");
	  exit(0);
	}
      }
    }
    
    if ( argc == 7) {
      sprintf( logFileName, "logs/%d-%d-%d-%d-%.2f",
	       atoi(argv[2]), 
	       atoi(argv[3]), 
	       atoi(argv[4]), 
	       atoi(argv[5]), 
	       atof(argv[6]));
      fprintf(stderr, "log %s\n", logFileName);
    }
  }
  
  /* Open the log. */
  openLogFile( logFileName);

  /* Check wether the right combination of values is given. */
  if ( ! tok[SCRIPT_TOKEN].initialized
       && ( ! tok[CONNECT_TO_BASE_TOKEN].initialized
	    || communicationInfo.connectBase == FALSE)) {
    fprintf( stderr, "Error: no script given.\n");
    closeLogAndExit(0);
  }
  
  /*-------------------------------------------------------------------------
   * done.
   *------------------------------------------------------------------------*/
  
  /* Set the file name in the global stucture. */
  actionInfo->logFileName = (char*) malloc( (strlen(logFileName) + 1) * sizeof(char));
  strcpy( actionInfo->logFileName, logFileName);
  
  initializeModuleNames(actionInfo->robotName);
  
  /* Base initializer not given or base connection not desired.
   * Use script. */
  if ( ! tok[CONNECT_TO_BASE_TOKEN].initialized
       || communicationInfo.connectBase == FALSE) {
    communicationInfo.connectBase = FALSE;
    actionInfo->measuredRobotPosition.x = 0.0;
    actionInfo->measuredRobotPosition.y = 0.0;
    actionInfo->measuredRobotPosition.rot = 0.0;
    if ( communicationInfo.scr->realTime && communicationInfo.scr->timeFactor <= 0.0)
      communicationInfo.scr->timeFactor = 1.0;
    fprintf( stderr, "# Reading data from %s (realtime: %d time factor %.2f).\n",
	     scriptFile, communicationInfo.scr->realTime,
	     communicationInfo.scr->timeFactor);
    writeLog( "# Reading data from %s (realtime: %dtime factor %.2f).\n",
	      scriptFile, communicationInfo.scr->realTime,
	      communicationInfo.scr->timeFactor);
    if ( ! openScript( scriptFile, communicationInfo.scr))
      closeLogAndExit(1);
#ifdef DO_NOT_USE_NEW_SCRIPT_LIBRARY
    communicationInfo.scr->actionInfo = actionInfo;
#endif
  }
  /* Base connection desired. */
  else {
    communicationInfo.useTcx = TRUE;
    communicationInfo.connectBase = TRUE;
    fprintf( stderr, "# Getting seonsor data from the robot.\n");
    writeLog( "# Getting seonsor data from the robot.\n");
  }
  
}

/********************************************************************
 ********************************************************************
 * Global functions.
 ********************************************************************
 ********************************************************************/
bool
updateSensorInformation( actionInformation* info,
			 rawSensings* actualSensings,
			 abstractSensorVector* abstractSensors)
{
  bool returnValue;

  /* Initialize tcx */
  if ( communicationInfo.useTcx && !tcx_initialized) {
    init_tcx( info,
	      communicationInfo.connectBase,
	      communicationInfo.subscribeBaseReport,
	      communicationInfo.subscribeProximityReport,
	      communicationInfo.subscribeCameraReport);
    /* Set the start time of the script. */
    gettimeofday(& (communicationInfo.scr->startTimeVal), NULL);
  }
					    
  
  if ( ! communicationInfo.connectBase) {

    if ( readScript( communicationInfo.scr, actualSensings)) {
      
      struct timeval TCX_waiting_time = {0, 0};

      if ( actualSensings->delta.isNew)
	sendBasePositionToUpdateModules( actualSensings->basePosition);
      /* DDD Don't send laser reports. */
      if ( 0 && actualSensings->frontLaser.isNew)
	sendLaserToUpdateModules( actualSensings->frontLaser,
				  actualSensings->rearLaser);

      if ( communicationInfo.scr->bumpOccured) {
	writeLog( "%f %f %f %f %f %f %f 1000.0 #BUMP\n",
		  elapsedScriptTime,
		  communicationInfo.scr->bumpForward,
		  communicationInfo.scr->bumpSideward,
		  communicationInfo.scr->bumpRot,
		  info->estimatedRobot.pos.x,
		  info->estimatedRobot.pos.y,
		  info->estimatedRobot.pos.rot);
	communicationInfo.scr->bumpOccured = FALSE;
      }
      
      if ( communicationInfo.scr->newMapPosition) {
	info->measuredRobotPosition = communicationInfo.scr->mapPosition;
	communicationInfo.scr->newMapPosition = FALSE;
      }
      
      /* Check wether we got some tcx messages to deal with. */
      if ( tcx_initialized) {
	tcxRecvLoop((void *) &TCX_waiting_time);
      }

      returnValue = TRUE;
    }
    else {
      closeScript( communicationInfo.scr);
      returnValue = FALSE;
    }
    
    /* Update the time when the last shift has been observed */
    if ( actualSensings->delta.isNew) {
      if ( ! info->useProbGrid) {
	scriptTime t = communicationInfo.scr->timeOfScript;
	info->samples.timeOfLastShift.tv_sec =
	  (long) t.hour * 3600 + t.minute * 60 + floor(t.second);
	info->samples.timeOfLastShift.tv_usec = (t.second - floor(t.second)) *
	  1000000;

      }
    }
    if ( communicationInfo.scr->runTime > 0.0) {
      if ( elapsedScriptTime > communicationInfo.scr->runTime) {
      /*  if ( actualSensings->distanceTraveled > 1000.0) { */
      /*      fprintf(stderr, "Travelled %f meters.\n", actualSensings->distanceTraveled); */
      /*      writeLog( "Travelled %f meters.\n", actualSensings->distanceTraveled); */
	fprintf(stderr, "Read %f seconds from the script.\n", elapsedScriptTime);
	writeLog( "Read %f seconds from the script.\n", elapsedScriptTime);
	closeLogAndExit(1);
      }
    }
  }
  else {
    /* If we perform online mapping wait until the map is initialized. */
    do {
      returnValue = getSensing( actualSensings);
    }
    while( ! info->onlineMap.initialized);

    /* Update the time when the last shift has been observed */
    if ( actualSensings->delta.isNew) {
      if ( ! info->useProbGrid)
	gettimeofday( &(info->samples.timeOfLastShift), NULL);
    }
  }
  
  /*-----------------------------------------------------------------
   * Set the corresponding values in the abstract sensors.
   *-----------------------------------------------------------------*/
  updateAbstractSensors( actualSensings, abstractSensors);

  /* Distance traveled since beginning in meters. */
  if ( actualSensings->delta.isNew) {
    actualSensings->distanceTraveled += 0.01 *
      sqrt( fSqr( actualSensings->delta.forward) +
	    fSqr( actualSensings->delta.sideward));
  }


  /* #define ROTATE_ROBOT */
#ifdef ROTATE_ROBOT
  if ( actualSensings->delta.isNew) {

    FILE* fp;

    if ( (fp = fopen( "_eg", "r")) != NULL) {
      float degree = DEG_90 / 90.0 * -135.0;
      fclose(fp);
      
      fprintf(stderr, "Add rotation of %f degree.\n", rad2Deg(degree));
      writeLog( "Add rotation of %f degree.\n", rad2Deg(degree));
      actualSensings->delta.rotation += degree;
      actualSensings->delta.forward += 500;
      actualSensings->delta.sideward += 300;
      system( "rm _eg");
    }
  }
#endif

  return returnValue;
}



