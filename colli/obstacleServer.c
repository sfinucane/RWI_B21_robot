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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/obstacleServer.c,v $
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
 * $Log: obstacleServer.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.16  1999/09/26 21:30:44  fox
 * Replaced localize position by correction parameters.
 *
 * Revision 1.15  1999/07/23 19:46:37  fox
 * Added ability to read grid maps for the obstacle server.
 *
 * Revision 1.14  1998/08/26 23:23:45  fox
 * Final version before I changed exception handling.
 *
 * Revision 1.13  1998/05/13 07:19:43  fox
 * Minor changes.
 *
 * Revision 1.12  1997/11/12 17:07:31  fox
 * Removed some old arm stuff.
 *
 * Revision 1.11  1997/06/17 09:39:31  fox
 * Changed rotate away.
 *
 * Revision 1.10  1997/06/03 11:49:11  fox
 * Museum version.
 *
 * Revision 1.9  1997/05/26 09:06:12  fox
 * Minor changes.
 *
 * Revision 1.8  1997/05/16 08:30:36  fox
 * Added connection to BASE_ROUTED.
 *
 * Revision 1.7  1997/04/17 09:19:38  fox
 * Minor changes.
 *
 * Revision 1.6  1997/04/17 09:16:20  fox
 * Added timeout for laser devices --> colliServer only needs 30% cpu.
 *
 * Revision 1.5  1997/04/10 12:32:31  fox
 * Support for bumpers.
 *
 * Revision 1.4  1997/04/09 12:57:50  fox
 * Minor changes.
 *
 * Revision 1.3  1997/03/28 03:48:29  tyson
 * finding .ini files and minor stuff
 *
 * Revision 1.2  1997/03/27 15:20:07  fox
 * Improved obstacle server.
 *
 * Revision 1.1  1997/03/26 18:42:05  fox
 * Incorporated external obstacle information into the collision avoidance.
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>

#include "tcx.h"
#include "tcxP.h"
#include "fakeSensors.hh" 
#include "localize.h"
#include "beeSoftVersion.h"


#define TCX_define_variables /* this makes sure variables are installed */
#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */
#include "BASE-messages.h"
#include "LOCALIZE-messages.h"
#include "obstacleServer.h"

#define TCX_USER_MODULE_NAME "OBSTACLE_SERVER"

BASE_register_auto_update_type baseUpdate;
LOCALIZE_register_auto_update_type localizeUpdate;

/************************************************************************
 * Global variables according to robot position.
 ************************************************************************/
realPosition mapPosition;
realPosition robotPosition;
correctionParameter correction;
int correctionParametersKnown = FALSE;
int robotPositionKnown = FALSE;

#define MOVEMENT_THRESHOLD 40.0
#define MAX_NUMBER_OF_OBSTACLE_POINTS 90

/************************************************************************
 * Checks wether the robot has moved enough since the last update and
 * sends the obstacles around the robot to the collision avoidance.
 ************************************************************************/
static void
deleteObstaclePoints()
{
  BASE_obstacle_points_type obstaclePoints;

  if ( BASE == NULL)
    return;
    
  obstaclePoints.points = NULL;
  obstaclePoints.no_of_points = 0;
  
  tcxSendMsg( BASE, "BASE_obstacle_points", &obstaclePoints);
}

/************************************************************************
 * Checks wether the robot has moved enough since the last update and
 * sends the obstacles around the robot to the collision avoidance.
 ************************************************************************/
static void
updateObstaclePoints()
{  
  static int movement = MOVEMENT_THRESHOLD;
  static realPosition previousPos;
  static int firstTime = TRUE;

  if ( BASE == NULL)
    return;
  
  /* Update the summed movement. */
  if ( firstTime) {
    firstTime = FALSE;
  }
  else
    movement += sqrt( fSqr( previousPos.x - robotPosition.x)
		      + fSqr( previousPos.y - robotPosition.y)
		      + fSqr( previousPos.rot - robotPosition.rot));

  previousPos = robotPosition;
  
  if ( 1 || movement >= MOVEMENT_THRESHOLD) {

    BASE_obstacle_points_type obstaclePoints;
    obstaclePoint points[MAX_NUMBER_OF_OBSTACLE_POINTS];
    int i, numberOfPoints = 0;

    realPosition robPosWithCorrectRotation = robotPosition;
    robPosWithCorrectRotation.rot = DEG_90 - DegToRad( robPosWithCorrectRotation.rot);
    
#ifdef USE_SIM_MAP
    for ( i = 0; i < MAX_NUMBER_OF_OBSTACLE_POINTS; i++) {

      float relativeAngle = DegToRad( (i - MAX_NUMBER_OF_OBSTACLE_POINTS / 2)
				      * 360 / MAX_NUMBER_OF_OBSTACLE_POINTS);

      if ( obstacleInDirection( mapPosition, robPosWithCorrectRotation,
				relativeAngle,
				&(points[numberOfPoints]),
				&correction))
	numberOfPoints++;
    }
#else
    numberOfPoints = createScanInGrid( &gridMap, mapPosition, robPosWithCorrectRotation,
				       points, MAX_NUMBER_OF_OBSTACLE_POINTS);
#endif
    obstaclePoints.points = points;
    obstaclePoints.no_of_points = numberOfPoints;
    
    tcxSendMsg( BASE, "BASE_obstacle_points", &obstaclePoints);
    movement = 0.0;
  }
}

/************************************************************************
 *
 *   Name:         OBSTACLE_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
OBSTACLE_close_handler(char *name, TCX_MODULE_PTR module)
{

  fprintf( stderr, "OBSTACLE_SERVER: closed connection detected: %s\n", name);
  
  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    exit(0);
  }
  else if ( module == BASE) {
    fprintf( stderr, "BASE disconnected.\n");
    robotPositionKnown = FALSE;
    BASE = NULL;
  }
  else if ( module == LOCALIZE) {
    fprintf( stderr, "LOCALIZE disconnected.\n");
    correctionParametersKnown = FALSE;
    deleteObstaclePoints();
    LOCALIZE = NULL;
  }
}

/**********************************************************************
 **********************************************************************
 *
 *  BASE handlers
 *
 **********************************************************************
 **********************************************************************/
void
BASE_update_status_reply_handler( TCX_REF_PTR ref,
				  BASE_update_status_reply_ptr status)
{
#ifdef TCX_debug
  fprintf( stderr, "TCX: Received a BASE_update_status_reply message.\n");
  fprintf( stderr, "robot: %g %g %g\n", 
	   status->pos_x, status->pos_y, status->orientation);
#endif

  robotPositionKnown = TRUE;
  
  /* Set the new robot position. */
  robotPosition.x = status->pos_x;
  robotPosition.y = status->pos_y;
  robotPosition.rot = status->orientation;

  if ( correctionParametersKnown) {

#define USE_CORRECTION
    
#ifdef USE_CORRECTION
    robotCoordinates2MapCoordinates( robotPosition.x,
				     robotPosition.y,
				     robotPosition.rot,
				     correction.x,
				     correction.y,
				     correction.rot,
				     correction.type,
				     &(mapPosition.x),
				     &(mapPosition.y),
				     &(mapPosition.rot));  
    
#endif
#ifdef TCX_debug
    fprintf( stderr, "New map position: %f %f %f\n",
	     mapPosition.x, mapPosition.y, RadToDeg(mapPosition.rot));
#endif
/*      fprintf( stderr, "New robot position: %f %f %f\n", */
/*  	     mapPosition.x, mapPosition.y, RadToDeg(mapPosition.rot)); */

    updateObstaclePoints();
  }
  
  tcxFree("BASE_update_status_reply", status); /* don't remove this! */
}

void
BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				  BASE_robot_position_reply_ptr pos)
{;}


void
BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
				   BASE_action_executed_reply_ptr data)
{;}


/**********************************************************************
 **********************************************************************
 *
 *  LOCALIZE handlers
 *
 **********************************************************************
 **********************************************************************/
void
LOCALIZE_map_reply_handler( TCX_REF_PTR                 ref,
				 LOCALIZE_map_reply_ptr map)
{}

void
LOCALIZE_samples_reply_handler( TCX_REF_PTR                 ref,
			    LOCALIZE_samples_reply_ptr samples)
{}


void
LOCALIZE_update_status_reply_handler( TCX_REF_PTR                 ref,
				      LOCALIZE_update_status_reply_ptr status)
{
#ifdef TCX_debug
  fprintf( stderr, "TCX: Received a LOCALIZE_update_status_reply message.\n");
  fprintf( stderr, "local: %d %f\n",
	   status->numberOfLocalMaxima, status->probOfGlobalMaximum);
#endif
  fprintf( stderr, "TCX: Received a LOCALIZE_update_status_reply message.\n");
  fprintf( stderr, "local: %d %f: %f %f %f\n",
	   status->numberOfLocalMaxima, status->probOfGlobalMaximum,
	   status->robotX, status->robotY, status->robotRot);
  
  /* Set the new correction parameters. */
  correction.x = status->corrX;
  correction.y = status->corrY;
  correction.rot = status->corrRot;
  correction.type = status->corrType;
  
  if (status->numberOfLocalMaxima < 3) {  
    correctionParametersKnown = TRUE;

    if ( robotPositionKnown) {

#ifdef USE_CORRECTION
      /* Get the new corrected position of the robot. */
      robotCoordinates2MapCoordinates( robotPosition.x,
				       robotPosition.y,
				       robotPosition.rot,
				       correction.x,
				       correction.y,
				       correction.rot,
				       correction.type,
				       &mapPosition.x, &mapPosition.y, &mapPosition.rot);
#else
      mapPosition.x = status->robotX;
      mapPosition.y = status->robotY;
      mapPosition.rot = status->robotRot;
#endif
      
/*        fprintf( stderr, "New localize position: %f %f %f\n", */
/*  	       mapPosition.x, mapPosition.y, RadToDeg(mapPosition.rot)); */
    }
  }
  else
    correctionParametersKnown = FALSE;
  
  tcxFree("LOCALIZE_update_status_reply", status); /* don't remove this! */
}


void
initTcx()
{
  char *tcxMachine = NULL;
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    BASE_messages,
    LOCALIZE_messages
  };

  baseUpdate.subscribe_status_report = 1;
  baseUpdate.subscribe_sonar_report = 0;
  baseUpdate.subscribe_laser_report = 0;
  baseUpdate.subscribe_ir_report = 0;
  baseUpdate.subscribe_colli_report = 0;
  
  localizeUpdate.subscribe = 2;

  fprintf(stderr, "Connecting to TCX...");
  tcxMachine = getenv("TCXHOST");   

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize(TCX_USER_MODULE_NAME, (void *) tcxMachine);
  fprintf(stderr, "done.\n");

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));
  

  tcxRegisterHandlers(BASE_reply_handler_array,
		      sizeof(BASE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(LOCALIZE_reply_handler_array,
		      sizeof(LOCALIZE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterCloseHnd(OBSTACLE_close_handler); 
}


int
main( int argc, char** argv)
{
  float xrot, yrot;
  float distance;
  int i;
  int useBaseRouted = FALSE;
  struct timeval TCX_waiting_time = {0, 0};
  char* mapFile = NULL;
  
  FILE *mapfd = NULL;

  if ( argc > 1) {
    for (i=1; i<argc; i++) {
      if ((strcmp(argv[i],"-router")==0))
	useBaseRouted = TRUE;
      else {
	fprintf( stderr, "Get map from %s\n", argv[i]);
#ifdef USE_SIM_MAP
	mapfd = fopen(argv[i], "r");
#else
	mapFile = argv[i];
#endif
      }
    }    
  }

#ifdef USE_SIM_MAP
  if ( mapfd == NULL) {
    fprintf( stderr, "Get map from floor.sim\n");
    mapfd = fopen("floor.sim", "r");
  }

  installSimMap(mapfd);
#else
  if ( mapFile == NULL) {
    fprintf(stderr, "No map provided.\n");
    exit(0);
  }

  readProbabilityMap( mapFile, &gridMap);
#endif

  if (0) {
    realPosition mapPosition = {530,1370,90};
    realPosition robPosWithCorrectRotation;
    obstaclePoint points[MAX_NUMBER_OF_OBSTACLE_POINTS];
    
    createScanInGrid( &gridMap, mapPosition, robPosWithCorrectRotation,
		      points, MAX_NUMBER_OF_OBSTACLE_POINTS);
    exit(0);
  }
  initTcx();
  
  for (;;) {
    if ( LOCALIZE == NULL || BASE == NULL) {
      if ( LOCALIZE == NULL) {
	fprintf(stderr, "Connecting to %s...", TCX_LOCALIZE_MODULE_NAME);
	LOCALIZE = tcxConnectModule(TCX_LOCALIZE_MODULE_NAME);
	tcxSendMsg( LOCALIZE, "LOCALIZE_register_auto_update", &localizeUpdate);
	fprintf(stderr, "done.\n");
      }
      if ( BASE == NULL){
	fprintf(stderr, "Connecting to %s...", TCX_BASE_MODULE_NAME);
	if ( useBaseRouted)
	  BASE = tcxConnectModule("BASE_ROUTED");
	else
	  BASE = tcxConnectModule(TCX_BASE_MODULE_NAME);
	fprintf(stderr, "ok\n");
	tcxSendMsg(BASE, "BASE_register_auto_update", &baseUpdate);
	fprintf(stderr, "done.\n");
      }
    }
    else
      block_wait(NULL, 1, 0);
    
    TCX_waiting_time.tv_sec = 0;
    TCX_waiting_time.tv_usec = 0;
    tcxRecvLoop((void *) &TCX_waiting_time);
  }
  
  exit(0);			/* should never reach here! */
}

