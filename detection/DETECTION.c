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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection/DETECTION.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:44:56 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: DETECTION.c,v $
 * Revision 1.1  2002/09/14 20:44:56  rstone
 * *** empty log message ***
 *
 * Revision 1.16  2000/12/04 20:25:49  thrun
 * Problem with compilation.
 *
 * Revision 1.15  1999/01/27 16:33:42  fox
 * Nothing special.
 *
 * Revision 1.14  1998/09/05 00:25:25  fox
 * Works fine to detect people in the museum.
 *
 * Revision 1.13  1998/08/29 21:44:41  fox
 * Changed the detection report and improved stuck detection.
 *
 * Revision 1.12  1998/08/23 22:57:38  fox
 * First version of building maps of humans.
 *
 * Revision 1.11  1997/06/03 11:49:13  fox
 * Museum version.
 *
 * Revision 1.10  1997/05/28 14:04:09  fox
 * Fixed a bug.
 *
 * Revision 1.9  1997/05/28 11:11:33  wolfram
 * parameter tuning
 *
 * Revision 1.8  1997/05/28 09:06:19  wolfram
 * fixed the mouse button
 *
 * Revision 1.7  1997/05/28 09:01:28  wolfram
 * added motion-only mode
 *
 * Revision 1.6  1997/05/25 10:40:49  fox
 * Nothing special.
 *
 * Revision 1.5  1997/05/09 16:28:38  fox
 * Works quiet fine.
 *
 * Revision 1.4  1997/05/06 14:22:56  fox
 * Nothing special.
 *
 * Revision 1.3  1997/05/06 08:19:51  fox
 * Added several messages.
 *
 * Revision 1.2  1997/05/05 16:54:04  fox
 * Incorporated graphics from people.c.
 *
 * Revision 1.1.1.1  1997/04/30 16:17:09  fox
 * Module to detect unexpected obstacles.
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/



#include <signal.h>

#include "general.h"
#include "localTcx.h"
#include "pantilt.h"
#include "detection.h"
#include "laser.h"
#include "function.h"
#include "bUtils.h"
#include "o-graphics.h"
#include "graphics.h"
#include "human.h"
#include "beeSoftVersion.h"

#include "DETECTION-messages.h"

detectionStruct obstacles = {0,NULL,NULL};
float mapResolution = 20.0;
int performStuckDetection = FALSE;
int useGraphics = TRUE;

#define USAGE_STRING "Usage: DETECTION <exp_dist_file> [-base] [-nopan] [-nosound] [-d] [-s] [-m].\n"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


struct bParamList * bParamList = NULL;
const char *bRobotType;

PROGRAM_STATE            program_state_data;
ROBOT_STATE              robot_state_data;
ROBOT_SPECIFICATIONS     robot_specifications_data;
PROGRAM_STATE_PTR        program_state        = &program_state_data;
ROBOT_STATE_PTR          robot_state          = &robot_state_data;
ROBOT_SPECIFICATIONS_PTR robot_specifications = &robot_specifications_data;


/************************************************************************
 *
 *   NAME:         initStructs
 *                 
 *   FUNCTION:     Initializes the structure "program_state" and "robot_state"
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_STATE_PTR  robot_state      Pointer  to general
 *                                                   robot state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *                 NEURAL_NETWORK_PTR neural_network   pointer to network
 *                 ALL_PTR            structure to all variables
 *                 
 *                 
 *                 NOTE: "NULL" disables initialization
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void initStructs()
{
  int i;

  bParamList = bParametersAddEntry(bParamList, "robot", "name", "B21");
  bParamList = bParametersAddEntry(bParamList, "", "TCXHOST", "localhost");
  bParamList = bParametersAddFile (bParamList, "etc/beeSoft.ini");
  bRobotType = bParametersGetParam(bParamList, "robot", "name");

  if (program_state != NULL){
    program_state->tcx_initialized                         = 0;
    program_state->graphics_initialized                    = 0;
    program_state->base_connected                          = 0;
    program_state->localize_connected                     = 0;
    program_state->sound_connected                         = 0;
    program_state->mouth_connected                         = 0;
    program_state->pantilt_connected                       = 0;
    program_state->quit                                    = 0; 
  }


  if (robot_state != NULL){
    robot_state->x                                         = 0.0;
    robot_state->y                                         = 0.0;
    robot_state->orientation                               = 0.0;
    robot_state->translational_speed                       = 0.0;
    robot_state->rotational_speed                          = 0.0;
    robot_state->known                                     = 0;
    robot_state->sensor_values                             = NULL;
    robot_state->detection_values                          = NULL;
    robot_state->person_found                              = 0;
    robot_state->local_person_x                            = 0.0;
    robot_state->local_person_y                            = 0.0;
    robot_state->global_person_x                           = 0.0;
    robot_state->global_person_y                           = 0.0;
  }

  if (robot_specifications != NULL){
    robot_specifications->robot_size                       = 30.0;

    robot_specifications->max_detection_range              = MAX_DETECTION_RANGE;
    robot_specifications->min_detection_range              = 0.0;
    robot_specifications->num_detections                   = 
      NUMBER_OF_DETECTION_ANGLES;
    robot_specifications->first_detection_angle            = 0.0;
    robot_specifications->detection_angles                 = NULL;
    
  }


  /* ALLOCATE NEW MEMORY FOR THE DETECTION */
  
  if (robot_specifications->detection_angles != NULL)
    free(robot_specifications->detection_angles);
  if (robot_state->detection_values != NULL)
    free(robot_state->detection_values);
  
  robot_specifications->detection_angles = 
    (float *) (malloc(sizeof(float) * 
		      robot_specifications->num_detections));
  robot_state->detection_values = 
    (float *) (malloc(sizeof(float) * 
		      robot_specifications->num_detections));
  if (robot_specifications->detection_angles == NULL ||
      robot_state->detection_values == NULL){
    printf("ABORT: out of memory!\n");
    exit(1);
  }
  for (i = 0; i < robot_specifications->num_detections; i++){
    robot_specifications->detection_angles[i] = 
      robot_specifications->first_detection_angle 
	+ (360.0 * ((float) i)
	   / ((float)  robot_specifications->num_detections));
    if (program_state->use_tcx)
      robot_state->detection_values[i] = 0.0;
    else{
      robot_state->detection_values[i] = 
	robot_specifications->max_detection_range
	  * 0.5 * (sin(((float) i) * 0.2) + 1.0); 
      if (robot_state->detection_values[i] < 
	  robot_specifications->max_detection_range * 0.5)
	robot_state->detection_values[i] = 0.0;
    }
  }
}


void
initializeAll( int argc, char** argv)
{
  int distIndex=1, i;
  int readDistances = TRUE;
  int readHumanProbs = FALSE;
  char* robotName = NULL;
  
  if ( argc < 2) {
    fprintf(stderr, USAGE_STRING);
    exit(0);
  }

  for (i=2; i<argc; i++) {
    if ((strcmp(argv[i],"-base")==0))
      connectBase = FALSE;
    else if ((strcmp(argv[i],"-nopan")==0))
      connectPantilt = FALSE;
    else if ((strcmp(argv[i],"-nosound")==0))
      connectSound = FALSE;
    else if ((strcmp(argv[i],"-nomouth")==0))
      connectMouth = FALSE;
    else if ((strcmp(argv[i],"-s") == 0)) {
      detectionMode = LOOK_FOR_UNEXPECTED;
      performStuckDetection = TRUE;
    }
    else if ((strcmp(argv[i],"-d")==0))
      useGraphics = FALSE;
    else if ((strcmp(argv[i],"-m") == 0)) {
      detectionMode = LOOK_FOR_MOTION;
      readDistances = FALSE;
    }
    else if ((strcmp(argv[i],"-h") == 0)) {
      detectionMode = BUILD_HUMAN_MAP;
      readDistances = TRUE;
      readHumanProbs = TRUE;
    }
    else if ((strcmp(argv[i],"-res")==0)) {
      if ( argc < i + 2)
	fprintf ( stderr, "Need a resolution following keyword \"res\"\n");
      else
	mapResolution = atof( argv[++i]);
    }
    else if ((strcmp(argv[i],"-robot")==0)) {
      if ( argc < i + 2)
	fprintf ( stderr, "Need a robot name following keyword \"-robot\"\n");
      else
	robotName = argv[++i];
    }
    else{
      fprintf(stderr, USAGE_STRING);
      exit(0);
    }
  }

  if ( robotName != NULL) 
    tcxSetModuleNameExtension( robotName);
  
  initStructs();
  
  initializeLaser( argv[distIndex], readDistances);
  
  if ( detectionMode == BUILD_HUMAN_MAP) {
    useGraphics = FALSE;
    initializeHumanMapping();
  }

  initFastLog();
  
  program_state->use_graphics = useGraphics;
  initGraphics( robot_state, program_state, robot_specifications);

  init_tcx();
}

/*****************************************************************************
 *****************************************************************************/
int
main( int argc, char** argv)
{
  int previousDetectionMode = -1;

  initializeAll( argc, argv);  

  /* --------------------------------------------------------------------- */
  /* Everything is initialized. Now start to integrate the sensors. */
   /* --------------------------------------------------------------------- */

  do {
    if ( previousDetectionMode != detectionMode) {
      if (useGraphics)
	G_display_switch(MODE_BUTTON, detectionMode);
      previousDetectionMode = detectionMode;
    }
    
    getSensing();
    
    if ( detectionMode == BUILD_HUMAN_MAP && laserScan.isNew)
      integrateLaserScan( &laserScan, &humanProbs, 
			  &humanMap);
    else {
      
      /* Set the detection mode. */
      obstacles.detectionMode = detectionMode;
      
      if ( obstacles.detectionMode != STOP_DETECTION) {
	
	/* Look for unexpected objects. 
	 * This only makes sense if the map position is known. */
	if ( obstacles.detectionMode == LOOK_FOR_UNEXPECTED) {

	  if ( mapPositionKnown)
	    detectUnexpectedObstacles( &obstacles);
	  else
	    obstacles.foundSomething = FALSE;
	  
	  if ( performStuckDetection) {
	    stuckDetection( &obstacles);
	  }
	}
	/* Look for motion. */
	else
	  detectMotion( &obstacles);
	
	/* Move the pantilt to the object. */
	movePanTilt( obstacles);
	
	broadcastStatusReport( obstacles);
      }
      else {
	obstacles.detectionMode = STOP_DETECTION;
	obstacles.foundSomething = FALSE;
	broadcastStatusReport( obstacles);
      }
      
      if (program_state->graphics_initialized)
	mouse_test_loop(robot_state, program_state, robot_specifications);
    }
  }
  while (!program_state->quit);
  
  exit(1);
}



