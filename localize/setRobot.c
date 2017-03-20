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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/setRobot.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:40 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: setRobot.c,v $
 * Revision 1.1  2002/09/14 20:45:40  rstone
 * *** empty log message ***
 *
 * Revision 1.22  2000/03/06 20:00:47  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.21  1999/08/27 22:22:34  fox
 * Better communication for multi robot localization.
 *
 * Revision 1.20  1999/06/24 00:21:53  fox
 * Some changes for the urbies.
 *
 * Revision 1.19  1999/03/12 00:41:51  fox
 * Minor changes.
 *
 * Revision 1.18  1999/03/10 15:30:36  schulz
 * Added message for querying samples
 *
 * Revision 1.17  1999/01/07 01:07:11  wolfram
 * Changes to integrate scan matching
 *
 * Revision 1.15  1998/11/23 21:19:27  fox
 * Fixed some minor bugs.
 *
 * Revision 1.14  1998/11/17 23:26:29  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.13  1998/08/20 00:23:03  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.12  1998/02/12 15:47:25  derr
 * compiler warnings (unused variable ... ) removed
 *
 * Revision 1.11  1997/08/02 16:51:08  wolfram
 * 1. Changed the order of indexes of the grid (and only of the grid):
 * The order now is grid->prob[rot][x][y].  This results in a significant
 * speed-up for different operations such as normalization and
 * integration of new sensory data.  Reimplemented the ConvolveThirdDim
 * procedure for convolving over rot.
 *
 * 2. Changed the algorithm to detect linear alignments of readings.  Now
 * we use the approach of Lu.
 *
 * 3. Linear alignments of readings is also checked for laser readings.
 *
 * 4. Expected distances are now computed given the simulator map if
 * available.  For that purpose the library libGetDistance is included.
 *
 * 5. Graphic output now displays the simulator map (if available). This
 * concernes the map overlay as well as the robot window.
 *
 * 6. Fixed some minor bugs.
 *
 * 7. Added different parameters to the ini-file (see example.ini).
 *
 * Revision 1.10  1997/05/19 21:42:15  wolfram
 * Distance Probabilty Function is now read from a file
 *
 * Revision 1.9  1997/05/06 15:55:14  wolfram
 * Robot position can now also be set in a graphic window
 *
 * Revision 1.8  1997/05/02 09:10:25  wolfram
 * Removed detection handler
 *
 * Revision 1.7  1997/04/30 12:25:43  fox
 * Some minor changes.
 *
 * Revision 1.6  1997/02/27 16:02:36  fox
 * Added command line argument to SetRobot.
 *
 * Revision 1.5  1997/02/22 05:16:41  thrun
 * Version numbers are now also checked for libraries.
 *
 * Revision 1.4  1997/02/22 00:59:05  thrun
 * Introduced version number support
 *
 * Revision 1.3  1997/01/30 13:34:13  fox
 * Minor changes.
 *
 * Revision 1.2  1997/01/30 10:50:03  fox
 * Minor changes.
 *
 * Revision 1.1  1997/01/29 13:23:54  fox
 * Tool to set the robot position in LOCALIZE and SIMULATOR.
 *
 * Revision 1.1  1997/01/29 12:23:02  fox
 * First version of restructured LOCALIZE.
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


/* #define USER_debug */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#include "tcx.h"
#include "tcxP.h"
#include "localize.h"
#define TCX_define_variables /* this makes sure variables are installed */
#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */
#include "general.h"
#include "graphic.h"
#include <EZX11.h>

#include "LOCALIZE-messages.h"
#include "SIMULATOR-messages.h"



#include "beeSoftVersion.h"

#define TCX_USER_MODULE_NAME "SET_ROBOT"


extern bool
readGridMap(char *mapname, char *extension, probabilityGrid *m);

extern bool
readSimulatorMapFile( char *fileName, simulatorMap *simMap);

extern gridWindow *
createMapWindow(probabilityGrid *m, char* text, int x, int y, int scale);

extern void
destroyMapWindow(gridWindow *win);

extern void
displayMapWindow(probabilityGrid *m, gridWindow *mapwin);

extern realPosition
positionRobotinWindow(int mode, gridWindow mapwindow, probabilityGrid *map,
		      simulatorMap *simMap,
		      robot rob, int x, int y, int destroyPositionTool);


int
block_wait( struct timeval *timeout, int tcx_initialized,
	    int X_initialized);

extern float
deg2Rad(float x);


extern float
rad2Deg(float x);

/************************************************************************
 * DUMMY HANDLER
 ************************************************************************/
void
LOCALIZE_map_reply_handler( TCX_REF_PTR                 ref,
				 LOCALIZE_map_reply_ptr map)
{ref=ref;map=map;}

void
LOCALIZE_samples_reply_handler( TCX_REF_PTR                 ref,
			    LOCALIZE_samples_reply_ptr samples)
{
  ref = ref; samples = samples;
}

void
LOCALIZE_update_status_reply_handler( TCX_REF_PTR                 ref,
				      LOCALIZE_update_status_reply_ptr status)
{ref=ref;status=status;}


void
SIMULATOR_message_to_base_handler(TCX_REF_PTR   ref,
				       char **message)
{ref=ref;message=message;}

void
SIMULATOR_message_to_baseServer_handler(TCX_REF_PTR   ref,
					     char *message)
{ref=ref;message=message;}



void SIMULATOR_status_update_handler( TCX_REF_PTR ref,
				      SIMULATOR_status_update_ptr status)
{ref=ref;status=status;}
 
void
SIMULATOR_message_to_sonar_handler(TCX_REF_PTR   ref,
					char **message)
{ref=ref;message=message;}

void
SIMULATOR_message_to_laser_handler(TCX_REF_PTR   ref,
					SIMULATOR_message_to_laser_ptr	data)
{ref=ref;data=data;}


void
set_robot_close_handler(char *name, TCX_MODULE_PTR module)
{
  if (!strcmp(name, TCX_LOCALIZE_MODULE_NAME)){
    LOCALIZE = NULL;
  }
  else if (!strcmp(name, TCX_SIMULATOR_MODULE_NAME)){
    SIMULATOR = NULL;
  }
  else if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    exit(0);
  }
  module=module;
}


static void
initTcx()
{
  char *tcxMachine = NULL;

  TCX_REG_MSG_TYPE TCX_message_array[] = {
    SIMULATOR_messages,
    LOCALIZE_messages
  };

  fprintf(stderr, "Connecting to TCX...");
  tcxMachine = getenv("TCXHOST");

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize( TCX_USER_MODULE_NAME, (void *) tcxMachine);
  check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
		       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
		       NULL, 1);


  fprintf(stderr, "done.\n");

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));

  tcxRegisterHandlers( LOCALIZE_reply_handler_array,
		       sizeof(LOCALIZE_reply_handler_array)
		       / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers( SIMULATOR_reply_handler_array,
		       sizeof(SIMULATOR_reply_handler_array)
		       / sizeof(TCX_REG_HND_TYPE));
  {
    extern void
      tcxRegisterCloseHnd(void (*closeHnd)());
    tcxRegisterCloseHnd(set_robot_close_handler);
  }
}


/******************************************************************************
 ******************************************************************************
 * Main control for LOCALIZE.
 ******************************************************************************
 ******************************************************************************/
int
main( int argc, char** argv)
{
  struct timeval TCX_waiting_time = {0, 0};
  realPosition pos = {0.0,0.0,0.0};
  int useGridMap=0;
  int useSimMap=0;
  int useGraphics = 0;
  char *mapFileName="";
  char* robotName = NULL;
  probabilityGrid gridMap;
  robot rob;
  simulatorMap simMap;
  int minScale = 1;
  
  int i, useSimulator = TRUE, useLocalize = TRUE;
  simMap.initialized = FALSE;

  rob.pos.x = rob.pos.y = rob.pos.rot = 0.0;
  rob.radius = ROB_RADIUS;

  if ( argc < 2) {
    fprintf( stderr, "Usage: setRobot [-pos x y rot]  [-gmap grid mapfile] [-smap sim mapfile] [-sim] [-loc] [-scale s] [-robot name].\n");
    exit(-1);
  }

  /* Maybe SIMULATOR of LOCALIZE is not desired. */
  for ( i = 1; i < argc; i++) {
    if ((strcmp(argv[i],"-sim")==0)) {
      useSimulator = FALSE; }
    else if ((strcmp(argv[i],"-loc")==0)) {
      useLocalize = FALSE; }
    else if ((strcmp(argv[i],"-pos")==0)) {
      if ( i < argc - 3) {
	pos.x = atof( argv[++i]);
	pos.y = atof( argv[++i]);
	pos.rot = deg2Rad( atof( argv[++i]));
	useGraphics = 0;
	fprintf( stderr, "Set robot to %f %f %f.\n", pos.x, pos.y, rad2Deg(pos.rot));
      }
      else {
	fprintf( stderr, "x-y-rot coordinates must follow keyword -pos.\n");
	exit(0);
      }
    }
    else if ((strcmp(argv[i],"-gmap")==0)) {
      if ( i < argc - 1) {
	mapFileName = argv[++i];
	useGridMap = 1;
	useGraphics = 1;
      }
      else {
	fprintf( stderr, "grid map file name must follow keyword -gmap.\n");
	exit(0);
      }
    }
    else if ((strcmp(argv[i],"-robot")==0)) {
      if ( i < argc - 1) {
	robotName = argv[++i];
      }
      else {
	fprintf( stderr, "name must follow keyword -robot.\n");
	exit(0);
      }
    }
    else if ((strcmp(argv[i],"-smap")==0)) {
      if ( i < argc - 1) {
	mapFileName = argv[++i];
	useSimMap = 1;
      }
      else {
	fprintf( stderr, "simulator map file name must follow keyword -smap.\n");
	exit(0);
      }
    }
    else if ((strcmp(argv[i],"-scale")==0)) {
      if ( i < argc - 1) {
	minScale = atoi(argv[++i]);
      }
      else {
	fprintf( stderr, "integer must follow keyword -scale.\n");
	exit(0);
      }
    }
    else
      fprintf(stderr, "usage: setRobot x y rot [-sim] [-loc].\n");

  }

  if ( robotName != NULL) {
    fprintf(stderr, "Connect to robot %s.\n", robotName);
    tcxSetModuleNameExtension( robotName);
  }
    
  
  initTcx();

  if (useGraphics) {
    if (useGridMap){
      if (!readGridMap( mapFileName, "", &gridMap)){
	fprintf(stderr, "error: could not read %s!\n", mapFileName);
	exit(-1);
      }
    }
    if (useSimMap){
      if (!readSimulatorMapFile( mapFileName, &simMap)){
	fprintf(stderr, "error: could not read %s!\n", mapFileName);
	exit(-1);
      }
    }
  }


  for (;;) {

    /* Reconnect. */
    if ((useLocalize && LOCALIZE == NULL) ||
	(useSimulator && SIMULATOR == NULL)) {
      
      if ( useLocalize && LOCALIZE == NULL){
	fprintf(stderr, "Connecting to %s...", TCX_LOCALIZE_MODULE_NAME);
	LOCALIZE = tcxConnectModule(TCX_LOCALIZE_MODULE_NAME);
	fprintf(stderr, "done.\n");
      }
      if ( useSimulator && SIMULATOR == NULL){
	fprintf(stderr, "Connecting to %s...", TCX_SIMULATOR_MODULE_NAME);
	SIMULATOR = tcxConnectModule(TCX_SIMULATOR_MODULE_NAME);
	fprintf(stderr, "done.\n");
      }

      /* If we don't use a map, then the position is set whenever a module
       * reconnects. */
      setRobotPosition(pos.x, pos.y, pos.rot);
    }
    
    if ( useGraphics) {
      static gridWindow *mapWindow = NULL;
      
      if ( mapWindow == NULL) {
	if ( robotName !=  NULL)
	  mapWindow = createMapWindow(&gridMap, robotName, 0, 0, minScale);
	else
	  mapWindow = createMapWindow(&gridMap, "Map", 0, 0, minScale);
	displayMapWindow(&gridMap, mapWindow);
	putchar(7);
      }

      pos = positionRobotinWindow( 4, /* SET_ROBOT_POSITION_TOKEN */
				   *mapWindow,
				   &gridMap,
				   &simMap,
				   rob,
				   mapWindow->sizeX + 2,
				   0, 0);
      
      setRobotPosition(pos.x, pos.y, pos.rot);
    }
    
    sleep(1);

    tcxRecvLoop((void *) &TCX_waiting_time);

  }
  
  exit(0);			/* should never reach here! */
}










