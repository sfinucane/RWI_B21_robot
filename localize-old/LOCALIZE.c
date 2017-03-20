
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/LOCALIZE.c,v $
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
 * $Log: LOCALIZE.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.68  2000/03/06 20:00:41  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.67  2000/01/26 22:56:42  fox
 * Might work. If anything weird happens, contact me.
 *
 * Revision 1.66  2000/01/02 15:33:14  fox
 * Should work.
 *
 * Revision 1.65  1999/11/02 18:12:32  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.64  1999/10/21 17:30:42  fox
 * Fixed some bugs in real time scripts.
 *
 * Revision 1.63  1999/08/30 05:48:41  fox
 * Doesn't work!!
 *
 * Revision 1.62  1999/06/24 00:21:48  fox
 * Some changes for the urbies.
 *
 * Revision 1.61  1999/04/29 00:58:27  fox
 * Some minor changes for multi localize.
 *
 * Revision 1.60  1999/01/22 17:47:59  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.59  1998/11/27 09:12:02  wolfram
 * Some changes for Frank
 *
 * Revision 1.58  1998/11/24 23:05:24  fox
 * Implemented furhter routines for condensation and vision.
 *
 * Revision 1.57  1998/11/23 19:45:05  fox
 * Latest version.
 *
 * Revision 1.56  1998/11/19 03:14:23  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.55  1998/11/17 23:26:14  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.54  1998/10/29 03:44:56  fox
 * Nothing special.
 *
 * Revision 1.53  1998/10/02 15:16:35  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.52  1998/09/25 04:02:52  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.51  1998/09/18 17:24:40  fox
 * Added skeleton files for condensation.
 *
 * Revision 1.50  1998/09/18 15:44:23  fox
 * First attempt to get a running system after museum.
 *
 * Revision 1.49  1998/08/23 00:00:57  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.48  1998/08/20 00:22:55  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.47  1998/08/19 16:33:52  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.46  1998/06/12 10:16:19  fox
 * Implemented virutal sensor.
 *
 * Revision 1.45  1998/04/07 18:31:48  wolfram
 * Removed Multi calls from tcx. Version runs. Major rework has to be
 * done in order to support multiple robots.
 *
 * Revision 1.44  1998/04/06 19:44:11  wolfram
 * Added -robot option for multi-robot support
 *
 * Revision 1.43  1998/03/02 06:35:23  wolfram
 * Added output of sensor errror values. Small change in graphic.c
 *
 * Revision 1.42  1998/01/22 13:06:09  fox
 * First version after selection-submission.
 *
 * Revision 1.41  1997/11/20 12:58:05  fox
 * Version with good sensor selection.
 *
 * Revision 1.40  1997/10/31 13:11:38  fox
 * Version for active sensing.
 *
 * Revision 1.39  1997/09/09 19:45:09  fox
 * Fixed a bug in finding local maxima and did several changes to active
 * localization.
 *
 * Revision 1.38  1997/08/16 22:59:48  fox
 * Last version before I change selsection.
 *
 * Revision 1.37  1997/07/04 17:29:11  fox
 * Final version before holiday!!!
 *
 * Revision 1.36  1997/06/27 16:26:24  fox
 * New model of the proximity sensors.
 *
 * Revision 1.35  1997/06/20 07:36:05  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.34  1997/05/27 07:42:27  fox
 * Nothing special.
 *
 * Revision 1.33  1997/05/26 08:47:38  fox
 * Last version before major changes.
 *
 * Revision 1.32  1997/04/24 21:25:42  wolfram
 * Ooops, fourth parameter of CUBE's is radius and not diameter!
 *
 * Revision 1.31  1997/04/03 13:17:49  fox
 * Some minor changes.
 *
 * Revision 1.30  1997/04/02 08:57:31  fox
 * Display the robot right after sensor selection.
 *
 * Revision 1.29  1997/03/18 18:45:27  fox
 * First executable version with a global scan mask for different sensor types.
 *
 * Revision 1.28  1997/03/17 18:41:12  fox
 * First version with sensor selection in cluttered environments.
 *
 * Revision 1.27  1997/03/13 17:36:18  fox
 * Temporary version. Don't use!
 *
 * Revision 1.26  1997/02/11 11:04:08  wolfram
 * Faster Computation of expected distances, height of sensors is considered
 *
 * Revision 1.25  1997/02/11 10:09:31  fox
 * No comment.
 *
 * Revision 1.24  1997/01/29 12:22:58  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.23  1997/01/17 08:11:47  fox
 * Added BATCH_MODE flag.
 *
 * Revision 1.22  1997/01/16 19:43:20  fox
 * And another bug ...
 *
 * Revision 1.21  1997/01/03 10:09:42  fox
 * First version with exploration.
 *
 * Revision 1.20  1996/12/31 09:19:21  fox
 * Last version before I switched exploration mode in PLAN to random
 * mode in COLLI.
 *
 * Revision 1.19  1996/12/20 15:29:34  fox
 * Added four parameters.
 *
 * Revision 1.18  1996/12/13 13:55:36  fox
 * Implemented connection to PLAN.
 *
 * Revision 1.17  1996/12/09 10:11:58  wolfram
 * Fixed a bug in angle computation
 *
 * Revision 1.16  1996/12/03 12:27:38  fox
 * Seems to work with the thin corridor.
 *
 * Revision 1.15  1996/12/02 18:46:24  fox
 * First version with the new expected distances.
 *
 * Revision 1.14  1996/12/02 10:31:58  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.13  1996/11/29 15:33:36  fox
 * ok
 *
 * Revision 1.12  1996/11/28 12:40:28  wolfram
 * Wolframs version.
 *
 * Revision 1.11  1996/11/28 09:05:48  fox
 * nicks spezielles.
 *
 * Revision 1.10  1996/11/27 12:18:16  fox
 * Nothing special.
 *
 * Revision 1.9  1996/11/26 16:08:24  fox
 * Nothing special.
 *
 * Revision 1.8  1996/11/25 19:35:38  fox
 * Test version for decisions of movements.
 *
 * Revision 1.7  1996/11/22 16:33:45  fox
 * First version.
 *
 * Revision 1.6  1996/11/22 10:30:23  wolfram
 * Added graphic for expected errors of movements
 *
 * Revision 1.5  1996/10/24 12:07:07  fox
 * Fixed a bug.
 *
 * Revision 1.4  1996/10/24 09:56:50  fox
 * LOCALIZE also works in a write protected directory.
 *
 * Revision 1.3  1996/10/23 14:37:24  fox
 * Example file for initialization.
 *
 * Revision 1.2  1996/09/24 16:29:36  wolfram
 * Fixed a problem with the new Makefile
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



#include <signal.h>

#include "general.h"
#include "sensings.h"
#include "condensation.h"
#include "file.h"
#include "planTcx.h"
#include "colliTcx.h"
#include "graphic.h"
#include "communication.h"
#include "proximityTools.h"

void 
signalHandler(int sig)
{
  if (1){
    fprintf( stderr, "Caught a signal %d.\n", sig);
    writeLog( "Caught a signal %d.\n", sig);
    swallowStatusReports(DONT_WAIT);
    planStopAutonomous();
    stopColli();
    closeLogAndExit(1);
  }
}

#define BATCH_MODE 1
void installSignalHandler()
{
  if ( ! BATCH_MODE)
    signal( SIGTERM, signalHandler); 
  signal( SIGINT, signalHandler); 
}


void
initializeAll( int argc, char** argv,
	       actionInformation**       actionInfo,
	       sensingActionMask**       actionMask,
	       informationsFor_GRAPHIC** graphicInfo)
{
  char* parameterFile;
  int i=1;
  
  if ( argc > 1)
    parameterFile = argv[1];
  else
    parameterFile = DEFAULT_PARAMETER_FILE;

  installSignalHandler();
  initFastLog();
  
  while (i < argc){
    if (strcmp(argv[i], "-seed") == 0){
      i++;
      if (i < argc && argv[i][0] != '-') {
	int seed = atoi( argv[i]);
	srand( seed);
	fprintf( stderr, "# Set seed to %d.\n", seed);
	writeLog( "# Set seed to %d.\n", seed);
      }
      else{
	fprintf(stderr, "Error: wrong or missing seed!");
	exit(0);
      }
    }
    i++;
  }
      
  
  /* Allocate memory. */
  *actionInfo     = (actionInformation*)       malloc( sizeof( actionInformation));
  *actionMask     = (sensingActionMask*)       malloc( sizeof( sensingActionMask));
  *graphicInfo    = (informationsFor_GRAPHIC*) malloc( sizeof( informationsFor_GRAPHIC));
  
  /* Set the struct to not initialized. */
  (*actionInfo)->map.initialized                  = FALSE;
  (*actionInfo)->laserMap.initialized             = FALSE;
  (*actionInfo)->sonarMap.initialized             = FALSE;
  (*actionInfo)->simMap.initialized               = FALSE;
  (*actionInfo)->initialPositionProbs.initialized = FALSE; 
  (*actionInfo)->positionProbs.initialized        = FALSE;
  (*actionInfo)->localMaxima.numberOfCells        = 0;
  (*actionInfo)->onlineMapping                    = FALSE;

  /* Now initialize the main structures. */
  initialize_COMMUNICATION( parameterFile, argc, argv,
			    *actionInfo, *actionMask);

  initialize_MAP( parameterFile, *actionInfo);

  initialize_PROBGRID( parameterFile, *actionInfo);

  initialize_SENSINGS( parameterFile, *actionInfo, *actionMask);

  initialize_CONDENSATION( parameterFile, *actionInfo, *actionMask, argc, argv);

  initialize_GRAPHICS( parameterFile, *actionInfo, *actionMask, *graphicInfo);
  /* ddd */
  checkWhichActionsToPerform_CONDENSATION( *actionInfo, *actionMask);
    
  initialize_ACTIVE( parameterFile, *actionInfo, *actionMask);
}

/*****************************************************************************
 *****************************************************************************/
int
main( int argc, char** argv)
{
  actionInformation* actionInfo;
  sensingActionMask* actionMask;
  informationsFor_GRAPHIC* graphicInfo;

  initializeAll( argc, argv,
		 &actionInfo,
		 &actionMask,
		 &graphicInfo);

  /* --------------------------------------------------------------------- */
  /* Everything is initialized. Now start to integrate the sensors. */
  /* --------------------------------------------------------------------- */

  while( updateSensorInformation( actionInfo,
				  &(actionInfo->actualSensings),
				  actionInfo->abstractSensors)) {

    /* Accumulate the movements seperately in each plane.
     * This has to be done even when no shifting is desired!! */
    accumulateMovements( actionInfo, actionMask);
    
    /* Accumulate the movements for the graphical output. */
    accumulateMovements_GRAPHICS( actionInfo->actualSensings.delta, graphicInfo);
    
    /* Check what kind of sensings have to be considered. */
    checkWhichSensingsToConsider( actionInfo, actionMask);
    
    /* First check what has to be done with the grid. */
    checkWhichActionsToPerform( actionInfo, actionMask);
    
    /* Integrate the sensings. */
    performActions( actionInfo, actionMask);
    
    /* Display the new information. */
    update_GRAPHICS( actionInfo, actionMask, graphicInfo);

/* #define PROXIMITY_ERROR_OUTPUT      */
#ifdef PROXIMITY_ERROR_OUTPUT
    dumpProximityErrors( actionInfo, actionMask);
#endif
/* #define PROXIMITY_ENDPOINT_OUTPUT       */
#ifdef PROXIMITY_ENDPOINT_OUTPUT
    dumpProximityEndPoints( actionInfo, actionMask);
#endif
  }
  
  if (! BATCH_MODE)
    getchar();
  closeLogAndExit(0);
  return 0;
}




