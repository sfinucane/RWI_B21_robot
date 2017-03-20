
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
 ***** IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFnmapECTIVE, YOU ASSUME
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection-old/localTcx.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:01 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: localTcx.c,v $
 * Revision 1.1  2002/09/14 20:45:01  rstone
 * *** empty log message ***
 *
 * Revision 1.13  1999/01/27 16:33:43  fox
 * Nothing special.
 *
 * Revision 1.12  1998/09/05 00:25:28  fox
 * Works fine to detect people in the museum.
 *
 * Revision 1.11  1998/08/29 21:44:44  fox
 * Changed the detection report and improved stuck detection.
 *
 * Revision 1.10  1998/08/23 22:57:41  fox
 * First version of building maps of humans.
 *
 * Revision 1.9  1997/10/08 15:46:53  haehnel
 * change some sound-reply stuff
 *
 * Revision 1.8  1997/10/08 14:15:16  fox
 * Nothing special.
 *
 * Revision 1.7  1997/06/03 11:49:15  fox
 * Museum version.
 *
 * Revision 1.6  1997/05/25 10:40:53  fox
 * Nothing special.
 *
 * Revision 1.5  1997/05/09 16:28:40  fox
 * Works quiet fine.
 *
 * Revision 1.4  1997/05/06 14:22:59  fox
 * Nothing special.
 *
 * Revision 1.3  1997/05/06 08:19:53  fox
 * Added several messages.
 *
 * Revision 1.2  1997/05/05 16:54:08  fox
 * Incorporated graphics from people.c.
 *
 * Revision 1.1.1.1  1997/04/30 16:17:09  fox
 * Module to detect unexpected obstacles.
 *
 * Revision 1.40  1997/04/27 10:40:36  fox
 * Changed status report.
 *
 * Revision 1.39  1997/04/03 13:17:50  fox
 * Some minor changes.
 *
 * Revision 1.38  1997/04/02 08:57:33  fox
 * Display the robot right after sensor selection.
 *
 * Revision 1.37  1997/03/26 09:42:57  fox
 * Updated correction parameter message.
 *
 * Revision 1.36  1997/03/19 17:52:42  fox
 * New laser parameters.
 *
 * Revision 1.35  1997/03/14 17:58:20  fox
 * This version should run quite stable now.
 *
 * Revision 1.34  1997/03/13 17:36:21  fox
 * Temporary version. Don't use!
 *
 * Revision 1.33  1997/02/28 12:57:30  fox
 * Minor changes.
 *
 * Revision 1.32  1997/02/27 16:02:35  fox
 * Added command line argument to SetRobot.
 *
 * Revision 1.31  1997/02/22 05:16:41  thrun
 * Version numbers are now also checked for libraries.
 *
 * Revision 1.30  1997/02/22 00:59:05  thrun
 * Introduced version number support
 *
 * Revision 1.29  1997/02/12 15:08:37  fox
 * Integrated laser support.
 *
 * Revision 1.28  1997/02/11 10:11:17  fox
 * Nothing special.
 *
 * Revision 1.27  1997/02/11 10:09:32  fox
 * No comment.
 *
 * Revision 1.26  1997/02/06 09:05:30  fox
 * Added parameter to send map.
 *
 * Revision 1.25  1997/02/05 14:43:02  wolfram
 * Completed laser update
 *
 * Revision 1.24  1997/01/31 17:11:02  fox
 * Integrated laser reply.
 *
 * Revision 1.23  1997/01/31 16:19:17  wolfram
 * Added start position to each reading of a scan
 *
 * Revision 1.22  1997/01/30 17:17:25  fox
 * New version with integrated laser.
 *
 * Revision 1.21  1997/01/30 13:34:12  fox
 * Minor changes.
 *
 * Revision 1.20  1997/01/29 12:23:09  fox
 * First version of restructured DETECTION.
 *
 * Revision 1.19  1997/01/20 13:09:38  fox
 * Unbelievable, that this positino estimation ever worked ..........
 *
 * Revision 1.18  1997/01/19 19:31:17  fox
 * yeah
 *
 * Revision 1.17  1997/01/18 18:19:22  wolfram
 * *** empty log message ***
 *
 * Revision 1.16  1997/01/14 16:53:23  fox
 * Removed a very very nasty bug for sensor selection.
 *
 * Revision 1.15  1997/01/08 15:52:56  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.14  1997/01/07 09:38:06  fox
 * Changed the loading of the ini file such that all parameters have to be
 * given only if the corresponding feature has to be used.
 *
 * Revision 1.13  1997/01/06 17:38:40  fox
 * Improved version.
 *
 * Revision 1.12  1997/01/03 18:07:47  fox
 * Successfully localized the robot and moved it into several rooms without
 * loosing the position again.
 *
 * Revision 1.11  1997/01/03 10:09:46  fox
 * First version with exploration.
 *
 * Revision 1.10  1996/12/31 09:19:23  fox
 * Last version before I switched exploration mode in PLAN to random
 * mode in COLLI.
 *
 * Revision 1.9  1996/12/20 15:29:39  fox
 * Added four parameters.
 *
 * Revision 1.8  1996/12/13 13:55:37  fox
 * Implemented connection to PLAN.
 *
 * Revision 1.7  1996/12/09 10:12:00  wolfram
 * Fixed a bug in angle computation
 *
 * Revision 1.6  1996/12/04 10:25:22  wolfram
 * Removed two warnings
 *
 * Revision 1.5  1996/12/03 05:35:27  thrun
 * If TCXHOST is not se, the software will now assume "localhost" and
 * won't terminate.
 *
 * Revision 1.4  1996/12/02 10:32:08  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.3  1996/11/18 09:58:30  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.2  1996/10/24 12:07:11  fox
 * Fixed a bug.
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



/* #define TCX_debug */

#include <sys/time.h>
#include <unistd.h>

#include "tcx.h"
#include "tcxP.h"
#include "general.h"
#include "localTcx.h"
#include "localize.h"
#include "function.h"
#include "laser.h"
#include "o-graphics.h"
#include "graphics.h"
#include "beeSoftVersion.h"
#include "pantilt.h"
#include "detection.h"
#include "allocate.h"

#define TCX_define_variables /* this makes sure variables are installed */

#include "DETECTION-messages.h"

#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */
#include "BASE-messages.h"
#include "LASER-messages.h"
#include "LOCALIZE-messages.h"
#include "SOUND-messages.h"
#include "MOUTH-messages.h"
#include "PANTILT-messages.h"

#define TCX_USER_MODULE_NAME "DETECTION"

#define LASER_SUBSCRIBE 1

bool connectBase = TRUE;
bool connectPantilt = TRUE;
bool connectSound = TRUE;
bool connectMouth = TRUE;


float correctionX, correctionY, correctionRot;
int correctionType;

realPosition robotPosition;
float uncorrectedRobotRot;

bool robotInMotion = FALSE;
bool correctionParametersKnown = FALSE;
bool robotPositionKnown = FALSE;
bool mapPositionKnown = FALSE;
bool laserPositionKnown = FALSE;
bool targetPointGiven = FALSE;

float rotationalSpeed = 0.0;

/* #define TCX_debug  */

extern void
tcxRegisterCloseHnd(void (*closeHnd)());

bool newPosition = FALSE;
bool tcx_initialized = FALSE;

/**********************************************************************
 **********************************************************************
 *               LOCAL Functions
 **********************************************************************
 **********************************************************************/

static void connect_to_SOUND(PROGRAM_STATE_PTR program_state);

static void connect_to_MOUTH(PROGRAM_STATE_PTR program_state);

/**********************************************************************
 **********************************************************************
 ******* auto-reply stuff, data definitions ***************
 **********************************************************************
 **********************************************************************/



#define MAX_N_AUTO_UPDATE_MODULES 100


int n_auto_update_modules = 0; /* number of processes to whom
				* position should be forwarded
				* automatically upon change */


typedef struct{
  TCX_MODULE_PTR module;	/* pointer to TCX module */
  int        subscribe;
} auto_update_type;


auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];
                                /* collection of TCX-module ptrs */



/************************************************************************
 *
 *   NAME:         count_auto_update_modules()
 *                 
 *   FUNCTION:     Counts, how many modules of the different types require
 *                 auto-updates
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/

void
count_auto_update_modules()
{
  int i;
  int registeredModules = n_auto_update_modules;
  n_auto_update_modules = 0;
  
  for (i = 0; i < registeredModules; i++){
    if (auto_update_modules[i].subscribe) n_auto_update_modules++;
  }
}


/************************************************************************
 *
 *   NAME:         add_auto_update_module()
 *                 
 *   FUNCTION:     Adds a module to the list of modules that
 *                 get automatical map updates
 *                 
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *                 int map                     1, if subscribe to map
 *                 int correction              1, if subscribe to correction
 *                 
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/

static int
add_auto_update_module( TCX_MODULE_PTR module, int subscribe)
{
  int i;
  
  if ( subscribe < 0)
    subscribe = 1;
  
  if (n_auto_update_modules >= MAX_N_AUTO_UPDATE_MODULES){
    fprintf(stderr, "ERROR: auto-module memory overflow. Request rejected.\n");
    return 0;
  }
  else
    for (i = 0; i < n_auto_update_modules; i++)
      if (auto_update_modules[i].module == module){
	fprintf(stderr, 
		"Module %s already known. Subscription modified: %d\n",
		tcxModuleName(module), subscribe);
	auto_update_modules[i].subscribe = subscribe; /* subsrc? */
	count_auto_update_modules();
	return 1;
      }
  fprintf(stderr, "Add %s to auto-reply list: %d.\n",
	  tcxModuleName(module), subscribe);
  auto_update_modules[n_auto_update_modules].module         = module;         /* pointer*/
  auto_update_modules[n_auto_update_modules].subscribe      = subscribe;      /* subsrc? */
  
  n_auto_update_modules++;
  count_auto_update_modules();
  
  return 1;
}



/************************************************************************
 *
 *   NAME:         remove_auto_update_module()
 *                 
 *   FUNCTION:     Attempts to remove a module from the list of 
 *                 modules that get automatical map updates 
 *                 
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *                 
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/

static int
remove_auto_update_module(TCX_MODULE_PTR module)
{     
  int i, j, found = 0;;
  for (i = 0; i < n_auto_update_modules; i++) /* search for module */
    if (auto_update_modules[i].module == module){ /* if module found */
      fprintf(stderr, "Remove %s from auto-reply list.\n",
	      tcxModuleName(module));
      found++;
      n_auto_update_modules--;	/* remove that entry, one less now */

      for (j = i; j < n_auto_update_modules; j++){
	auto_update_modules[j].module = 
	  auto_update_modules[j+1].module; /* shift back */
	auto_update_modules[j].subscribe = 
	  auto_update_modules[j+1].subscribe; /* shift back */
      }
    }
  if ( ! found) {
    fprintf(stderr, "(%s: no auto-replies removed)\n",
	    tcxModuleName(module));
  }
  
  count_auto_update_modules();

  return found;
}
  

/**********************************************************************
 **********************************************************************
 *
 *  DETECTION handlers
 *
 **********************************************************************
 **********************************************************************/


/************************************************************************
 *
 *   NAME: DETECTION_register_auto_update_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
void
DETECTION_register_auto_update_handler( TCX_REF_PTR  ref,
					DETECTION_register_auto_update_ptr data)
{
  DETECTION_update_status_reply_type status;
  
  add_auto_update_module(ref->module, 
			 data->subscribe);

  /* Send initial status update to the new module. */

  if (data != NULL){
    tcxFree("DETECTION_register_auto_update", data);
    data = NULL;
  }
}

void
DETECTION_look_for_motion_handler( TCX_REF_PTR  ref,
				   void* data)
{
  fprintf( stderr, "Start looking for motion.\n");

  detectionMode = LOOK_FOR_MOTION;
  
  if (data != NULL){
    tcxFree("DETECTION_look_for_motion", data);
    data = NULL;
  }
}


void
DETECTION_look_for_unexpected_handler( TCX_REF_PTR  ref,
				       void* data)
{
  fprintf( stderr, "Start looking for unexpected.\n");

  detectionMode = LOOK_FOR_UNEXPECTED;
  
  if (data != NULL){
    tcxFree("DETECTION_look_for_unexpected", data);
    data = NULL;
  }
}

void
DETECTION_stop_handler( TCX_REF_PTR  ref,
			void* data)
{
  fprintf( stderr, "Stop detection.\n");

  detectionMode = STOP_DETECTION;
  
  if (data != NULL){
    tcxFree("DETECTION_stop", data);
    data = NULL;
  }
}


/************************************************************************
 *
 *   Name:         DETECTION_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
DETECTION_close_handler(char *name, TCX_MODULE_PTR module)
{

  
  remove_auto_update_module(module);
  
  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    exit(0);
  }
  else if ( module == LOCALIZE) {
    fprintf( stderr, "LOCALIZE disconnected.\n");
    correctionParametersKnown = FALSE;
    LOCALIZE = NULL;
    program_state->localize_connected = 0;
    if (program_state->graphics_initialized)
      G_display_switch(LOCALIZE_CONNECTED_BUTTON, 0);
  }
  else if ( module == BASE) {
    fprintf( stderr, "BASE disconnected.\n");
    robotPositionKnown = FALSE;
    BASE = NULL;
    program_state->base_connected = 0;
    if (program_state->graphics_initialized)
      G_display_switch(BASE_CONNECTED_BUTTON, 0);
  }
  else if (!strcmp(name, "SOUND")){ /* SOUND shut down */
    program_state->sound_connected = 0;
    if (program_state->graphics_initialized)
      G_display_switch(SOUND_CONNECTED_BUTTON, 0);
  }
  else if (!strcmp(name, "MOUTH")){ /* MOUTH shut down */
    program_state->mouth_connected = 0;
    if (program_state->graphics_initialized)
      G_display_switch(MOUTH_CONNECTED_BUTTON, 0);
  }
  else if (!strcmp(name, "PANTILT")){ /* PANTILT shut down */
    program_state->pantilt_connected = 0;
    if (program_state->graphics_initialized)
      G_display_switch(PANTILT_CONNECTED_BUTTON, 0);
  }
}




/************************************************************************
 *
 *   NAME:         send_automatic_localize_update
 *                 
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


static int auto_update_counter = 0;

void
broadcastStatusReport( detectionStruct detection)
{
  int tcxStructInitialized = FALSE;

  if ( tcx_initialized && n_auto_update_modules > 0) {
        
    int i;
    static DETECTION_update_status_reply_type tcxStruct;
   
    for (i = 0; i < n_auto_update_modules; i++) {
      
      if ( auto_update_modules[i].subscribe > 0)
	if (auto_update_counter % auto_update_modules[i].subscribe == 0){
	  fprintf(stderr, ".");
	  /* Initialize the structure only if necessary. */
	  if ( ! tcxStructInitialized) {
	    
	    int obst, obstacleCnt = 0;
	    
	    fprintf(stderr, "|");
	    for ( obst = 0; obst < detection.numberOfAngles; obst++)
	      if ( detection.distances[obst] > 0.0)
		obstacleCnt++;
	    fprintf(stderr, "cnt %d\n", obstacleCnt);
	    tcxStruct.mode = detection.detectionMode;
	    tcxStruct.numberOfObstacles = obstacleCnt;
	    
	    /* Alllocate memory if necessary. */
	    if ( obstacleCnt > 0) {
	      tcxStruct.angles = (float*) allocate1D( obstacleCnt, FLOAT);
	      tcxStruct.distances = (float*) allocate1D( obstacleCnt, FLOAT);
	    }
	    
	    obstacleCnt = 0;
	    for ( obst = 0; obst < detection.numberOfAngles; obst++)
	      if ( detection.distances[obst] > 0.0) {
		tcxStruct.angles[obstacleCnt] = detection.angles[obst];
		tcxStruct.distances[obstacleCnt] = detection.distances[obst];
		obstacleCnt++;
	      }
	    
	    tcxStructInitialized = TRUE;
	    
	    fprintf(stderr, "mode: %d num %d\n",   tcxStruct.mode,  tcxStruct.numberOfObstacles);
	  }
	  
	  fprintf(stderr, "(");
	  tcxSendMsg( auto_update_modules[i].module, "DETECTION_update_status_reply",
		      &tcxStruct);
	  fprintf(stderr, ")");
	}
      auto_update_counter++;
    }
    if ( tcxStructInitialized && detection.foundSomething) {
      fprintf(stderr, "[");
      free1D( tcxStruct.angles, FLOAT);
      free1D( tcxStruct.distances, FLOAT);
      fprintf(stderr, "]");
    }
  }
}  


/**********************************************************************
 **********************************************************************
 *
 *  LOCALIZE handlers
 *
 **********************************************************************
 **********************************************************************/
static void
connectToLocalize()
{
  LOCALIZE_register_auto_update_type data;

  data.subscribe = 1;

  fprintf(stderr, "Connecting to %s...", TCX_LOCALIZE_MODULE_NAME);
  LOCALIZE = tcxConnectModule(TCX_LOCALIZE_MODULE_NAME); 
  tcxSendMsg(LOCALIZE, "LOCALIZE_register_auto_update", &data);
  fprintf(stderr, "done.\n");

  if (program_state->graphics_initialized)
    G_display_switch(LOCALIZE_CONNECTED_BUTTON, 1);
  program_state->localize_connected = TRUE;
}


void
LOCALIZE_update_status_reply_handler( TCX_REF_PTR                 ref,
				      LOCALIZE_update_status_reply_ptr status)
{
#ifdef TCX_debug
  fprintf( stderr, "TCX: Received a LOCALIZE_update_status_reply message.\n");
  fprintf( stderr, "local: %d: %f %f %f %d\n",
	   status->numberOfLocalMaxima, status->probOfGlobalMaximum,
	   status->corrX,status->corrY, status->corrRot, status->corrType); 
#endif
  
  /* Set the new correction parameters. */
  correctionX = status->corrX;
  correctionY = status->corrY;
  correctionRot = status->corrRot;
  correctionType = status->corrType;

  if ( status->numberOfLocalMaxima == 1) {
    
    correctionParametersKnown = TRUE;
    
    if (1)
      fprintf( stderr, "Received a LOCALIZE_update_status_reply.\n");
    
    if ( robotPositionKnown)
      mapPositionKnown = TRUE;
  }

  else
    correctionParametersKnown = FALSE;

  tcxFree("LOCALIZE_update_status_reply", status); /* don't remove this! */
}

void
LOCALIZE_map_reply_handler( TCX_REF_PTR                 ref,
			    LOCALIZE_map_reply_ptr map)
{}


/**********************************************************************
 **********************************************************************
 *
 *  Mouth handlers
 *
 **********************************************************************
 **********************************************************************/

void
setMood( int mood)
{
  static int mouth[NUMBER_OF_MOODS] = {-55,-20,15,45}; 
  static int eyes[NUMBER_OF_MOODS] = {50,25,0,-40};

  if ( program_state->mouth_connected) {
    
    MOUTH_set_face_type face;
    
    face.mouth = mouth[mood];
    face.eyes =  eyes[mood];
    
    tcxSendMsg( MOUTH, "MOUTH_set_face", &face);
  }
}

/**********************************************************************
 **********************************************************************
 *
 *  BASE handlers
 *
 **********************************************************************
 **********************************************************************/

static void
connectToBase()
{
  if ( connectBase) {
    BASE_register_auto_update_type data;
    
    data.subscribe_status_report = 1;
    data.subscribe_sonar_report  = 0;
    data.subscribe_laser_report  = LASER_SUBSCRIBE;
    data.subscribe_colli_report  = 0;
    
    fprintf(stderr, "Connecting to %s...", TCX_BASE_MODULE_NAME);
    BASE = tcxConnectModule(TCX_BASE_MODULE_NAME); 
    tcxSendMsg(BASE, "BASE_register_auto_update", &data);
    fprintf(stderr, "done.\n");
    
    if (program_state->graphics_initialized)
      G_display_switch(BASE_CONNECTED_BUTTON, 1);
    program_state->base_connected = TRUE;
  }
}

void
BASE_update_status_reply_handler( TCX_REF_PTR ref,
				  BASE_update_status_reply_ptr status)
{
  static realPosition prevPos = { 0.0, 0.0, 0.0};

  if (0) {
    int i;
    for ( i = 0; i < obstacles.numberOfAngles; i++)
      fprintf(stderr, "%d %f %f\n", i , obstacles.distances[i], obstacles.measuredDistances[i]);
  }

  
#ifdef TCX_debug
  fprintf( stderr, "TCX: Received a BASE_update_status_reply message.\n");
  fprintf( stderr, "robot: %g %g %g\n", 
	   status->pos_x, status->pos_y, status->orientation);
#endif


  rotationalSpeed = status->rot_current_speed;

  targetPointGiven = status->targetDefined;
  
  /* Set the new robot position. */
  uncorrectedRobotRot = status->orientation;
  robotPosition.x = status->pos_x;
  robotPosition.y = status->pos_y;
  robotPosition.rot = DEG_90 - Deg2Rad( status->orientation);
  
  if ( ! robotPositionKnown) {
    prevPos = robotPosition;
    robotPositionKnown = TRUE;
  }

  /* Check whether the robot has moved since the last status report. */
  robotInMotion = prevPos.x != robotPosition.x || prevPos.y != robotPosition.y ||
    prevPos.rot != robotPosition.rot;
  prevPos = robotPosition;

  if ( correctionParametersKnown) {
    mapPositionKnown = TRUE;

#ifdef TCX_debug
    fprintf( stderr, "New map position: %f %f %f\n",
	     robotPosition.x, robotPosition.y, Rad2Deg(robotPosition.rot));
#endif

  }
  newPosition = TRUE;
  tcxFree("BASE_update_status_reply", status); /* don't remove this! */

  emergencyHandling( robotPosition, status->emergency);

}


void
BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				  BASE_robot_position_reply_ptr pos)
{;}


void
BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
				   BASE_action_executed_reply_ptr data)
{;}


void
setBaseMode( int modeNumber)
{
  BASE_setmode_type mode;
  mode.modeNumber = modeNumber;
  mode.useSonar = -1;
  mode.useLaser = -1;
  
  tcxSendMsg(BASE, "BASE_setmode", &mode);
}

/**********************************************************************
 **********************************************************************
 *
 *  LASER handlers
 *
 **********************************************************************
 **********************************************************************/
void
LASER_laser_reply_handler(TCX_REF_PTR           ref,
			  LASER_laser_reply_ptr laser)
{
  int i;
  
#ifdef TCX_debug
  fprintf( stderr, "TCX: Received a LASER_laser_reply message.\n");
#endif

  /* Get the front readings. */
  for (i = 0; i < laser->f_numberOfReadings; i++) {

    int scanIndex = FRONT_START_INDEX + i;
    if ( scanIndex > 359)
      scanIndex -= 360;
    
    if (laser->f_reading[i] < 0.0)
      laserScan.reading[scanIndex].dist = 0.0;
    else
      laserScan.reading[scanIndex].dist = fMin( laserMaxRange, laser->f_reading[i]);

    laserScan.frontRealRob.x = laser->xPos;
    laserScan.frontRealRob.y = laser->yPos;
    laserScan.frontRealRob.rot = laser->rotPos;
  }
  
  /* Get the rear readings. */
  for (i = 0; i < laser->r_numberOfReadings; i++) {
    
    int scanIndex = REAR_START_INDEX + i;

    if (laser->r_reading[i] < 0.0)
      laserScan.reading[scanIndex].dist = 0.0;
    else
      laserScan.reading[scanIndex].dist = 
	fMin( laserMaxRange, laser->r_reading[i]);

    laserScan.rearRealRob.x = laser->xPos;
    laserScan.rearRealRob.y = laser->yPos;
    laserScan.rearRealRob.rot = laser->rotPos;
  }

  laserScan.numberOfReadings = laser->f_numberOfReadings + laser->r_numberOfReadings;

  if ( mapPositionKnown) {
    updateLaserMapPositions();
    laserPositionKnown = TRUE;
  }

  updateLaserFeatures();
  
  laserScan.isNew = TRUE;

  tcxFree("LASER_laser_reply", laser);
}




bool
getSensing()
{
  struct timeval TCX_waiting_time = {0, 0};

  int block_wait(struct timeval *timeout, int tcx_initialized,
		 int X_initialized);

  newPosition = FALSE;
  laserScan.isNew = FALSE;

  /* Try to connect to PANTILT and SOUND. */
  if (1) {
    if (!program_state->sound_connected)
      connect_to_SOUND(program_state);
    if (!program_state->pantilt_connected)
      connect_to_PANTILT(program_state);
    if (!program_state->mouth_connected)
      connect_to_MOUTH(program_state);
  }
  
  while (! (laserScan.isNew)) {
    
    TCX_waiting_time.tv_sec = 0;
    TCX_waiting_time.tv_usec = 100000;
    
    tcxRecvLoop((void *) &TCX_waiting_time);
    
    if ( BASE == NULL) 
      connectToBase();
    
    if ( LOCALIZE == NULL && 
	 (detectionMode == LOOK_FOR_UNEXPECTED || detectionMode == BUILD_HUMAN_MAP)) 
      connectToLocalize();
  } 
  return(TRUE);
}




/**********************************************************************
 **********************************************************************
 *
 *  SOUND handlers
 *
 **********************************************************************
 **********************************************************************/

void
sayHello()
{ 
  if ( SOUND != NULL) {
    
    char text[80];
    char *text_ptr;
    
    if ( timeExpired( SOUND_TIMER) > 3.0) {
      
      /*     strcpy(text, "Hello"); */
      /*     text_ptr = (char *) &text; */
      /*     resetTimer( SOUND_TIMER); */
      /*     tcxSendMsg(SOUND, "SOUND_talk_query", &(text_ptr)); */
      strcpy(text, "hi wolli");
      /*     strcpy(text, "pfeife"); */
      text_ptr = (char *) &text;
      resetTimer( SOUND_TIMER);
      tcxSendMsg(SOUND, "SOUND_talk_query", &(text_ptr));
      /*     tcxSendMsg(SOUND, "SOUND_play_query", &(text_ptr)); */
    }
  }
}

#define MAX_STUCK_TIME 5.0
#define MAX_STUCK_CNT 3
#define MAX_ROAR_TIME 45.0
#define NO_SOUND_AFTER_ROAR 10.0

void
triggerSound( char* text)
{  
  if ( SOUND != NULL) {
    
    tcxSendMsg(SOUND, "SOUND_play_query", &(text));
  }
  fprintf(stderr, "%s\n", text);
}

/************************************************************************
 *
 *   NAME:         connect_to_SOUND
 *                 
 *   FUNCTION:     checks, and connects to SOUND, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

static struct timeval last_attempt_connect_SOUND = {0, 0};


static void connect_to_SOUND(PROGRAM_STATE_PTR program_state)
{
  struct timeval current_time;

  if (connectSound) {
    
    if(!program_state->sound_connected){
      
      gettimeofday(&current_time, NULL);
      if (current_time.tv_sec < last_attempt_connect_SOUND.tv_sec + 3
	  || (current_time.tv_sec == last_attempt_connect_SOUND.tv_sec + 3 &&
	      current_time.tv_usec < last_attempt_connect_SOUND.tv_usec))
	return;
      
      
      if (program_state->graphics_initialized)
	G_display_switch(SOUND_CONNECTED_BUTTON, 2);
      
      last_attempt_connect_SOUND.tv_sec  = current_time.tv_sec;
      last_attempt_connect_SOUND.tv_usec = current_time.tv_usec;
      
      
      SOUND = tcxConnectOptional(TCX_SOUND_MODULE_NAME); /* checks, but does 
							  * not wait */
      
      if (SOUND != NULL){
	
	if (program_state->graphics_initialized)
	  G_display_switch(SOUND_CONNECTED_BUTTON, 1);
	
	program_state->sound_connected = 1;
	
      }
      else if (program_state->graphics_initialized){
	/*usleep(200000);*/
	G_display_switch(SOUND_CONNECTED_BUTTON, 0);
      }
    }
  }
}


void 
SOUND_play_reply_handler(TCX_REF_PTR ref, void *data)
{
  if (0)
    fprintf(stderr, "TCX: Received a SOUND_play_reply message.\n");
}

void SOUND_playing_reply_handler(TCX_REF_PTR ref,
				 SOUND_playing_reply_type *data)
{
  tcxFree( "SOUND_playing_reply", data);
}




/************************************************************************
 *
 *   NAME:         connect_to_SOUND
 *                 
 *   FUNCTION:     checks, and connects to SOUND, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

static struct timeval last_attempt_connect_MOUTH = {0, 0};


static void connect_to_MOUTH(PROGRAM_STATE_PTR program_state)
{

  struct timeval current_time;

  if (connectMouth) {
    
    if(!program_state->mouth_connected){
      
      gettimeofday(&current_time, NULL);
      if (current_time.tv_sec < last_attempt_connect_MOUTH.tv_sec + 3
	  || (current_time.tv_sec == last_attempt_connect_MOUTH.tv_sec + 3 &&
	      current_time.tv_usec < last_attempt_connect_MOUTH.tv_usec))
	return;
      
      
      if (program_state->graphics_initialized)
	G_display_switch(MOUTH_CONNECTED_BUTTON, 2);
      
      last_attempt_connect_MOUTH.tv_sec  = current_time.tv_sec;
      last_attempt_connect_MOUTH.tv_usec = current_time.tv_usec;
      
      
      MOUTH = tcxConnectOptional(TCX_MOUTH_MODULE_NAME); /* checks, but does 
							  * not wait */
      
      if (MOUTH != NULL){
	
	if (program_state->graphics_initialized)
	  G_display_switch(MOUTH_CONNECTED_BUTTON, 1);
	
	program_state->mouth_connected = 1;
	
      }
      else if (program_state->graphics_initialized){
	/*usleep(200000);*/
	G_display_switch(MOUTH_CONNECTED_BUTTON, 0);
      }
    }
  }
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



TCX_REG_HND_TYPE DETECTION_handler_array[] = {

  {"DETECTION_register_auto_update", "DETECTION_register_auto_update_handler",
     DETECTION_register_auto_update_handler, TCX_RECV_ALL, NULL},
  {"DETECTION_look_for_unexpected", "DETECTION_look_for_unexpected_handler",
     DETECTION_look_for_unexpected_handler, TCX_RECV_ALL, NULL},
  {"DETECTION_look_for_motion", "DETECTION_look_for_motion_handler",
     DETECTION_look_for_motion_handler, TCX_RECV_ALL, NULL},
  {"DETECTION_stop", "DETECTION_stop_handler",
     DETECTION_stop_handler, TCX_RECV_ALL, NULL},
};


void
init_tcx()
{
  char *tcxMachine = NULL;
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    BASE_messages,
    LASER_messages,
    LOCALIZE_messages,
    DETECTION_messages,
    SOUND_messages,
    MOUTH_messages,
    PANTILT_messages
  };

  fprintf(stderr, "Connecting to TCX...");
  tcxMachine = getenv("TCXHOST");   

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize(TCX_USER_MODULE_NAME, (void *) tcxMachine);
  check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
		       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
		       NULL, 1);


  fprintf(stderr, "done.\n");

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));
  

  tcxRegisterHandlers(BASE_reply_handler_array,
		      sizeof(BASE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(LASER_reply_handler_array,
		      sizeof(LASER_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(LOCALIZE_reply_handler_array,
		      sizeof(LOCALIZE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
  
  tcxRegisterHandlers(PANTILT_reply_handler_array,
		      sizeof(PANTILT_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(SOUND_reply_handler_array,
		      sizeof(SOUND_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(MOUTH_reply_handler_array,
		      sizeof(MOUTH_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(DETECTION_handler_array,
		      sizeof(DETECTION_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterCloseHnd(DETECTION_close_handler); 

  connectToBase();

  if ( detectionMode == LOOK_FOR_UNEXPECTED)
    connectToLocalize();

  newPosition = FALSE;

  tcx_initialized = TRUE;  
}





