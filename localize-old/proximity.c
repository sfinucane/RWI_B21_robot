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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/proximity.c,v $
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
 * $Log: proximity.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.111  2000/08/18 15:37:59  wolfram
 * Nothing special
 *
 * Revision 1.110  2000/08/09 23:40:46  wolfram
 * Fixed the pain problem with MIN_WINDOW_SCALE
 *
 * Revision 1.109  2000/03/06 20:00:46  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.108  1999/12/20 08:45:55  fox
 * Implementation of precomputation of p(x|o).
 *
 * Revision 1.107  1999/11/02 18:12:36  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.106  1999/09/01 00:02:57  fox
 * Getting closer. But still not close enough.
 *
 * Revision 1.105  1999/07/13 23:08:02  fox
 * Some changes.
 *
 * Revision 1.104  1999/07/12 15:24:53  fox
 * Minor changes in Texas.
 *
 * Revision 1.103  1999/06/25 19:48:13  fox
 * Minor changs for the urbie.
 *
 * Revision 1.102  1999/06/24 00:21:53  fox
 * Some changes for the urbies.
 *
 * Revision 1.101  1999/06/23 16:22:03  fox
 * Added robot type urban.
 *
 * Revision 1.100  1999/03/19 16:04:41  wolfram
 * Added REAL_TIME simulation in script.c
 *
 * Revision 1.99  1999/03/13 17:50:22  fox
 * Updated localTcx.c
 *
 * Revision 1.98  1999/03/12 00:41:50  fox
 * Minor changes.
 *
 * Revision 1.97  1999/03/08 16:47:45  wolfram
 * Fixed problems with map, caused by the fact that it no longer accpets
 * correction parameters from LOCALIZE.  Improved the handling of sonars
 * with Pioneers.  Currently sonars do not work properly on a B21.
 *
 * Revision 1.96  1999/02/17 19:42:24  fox
 * Enhanced gif utilities.
 *
 * Revision 1.95  1999/02/05 23:02:43  fox
 * Minor changes for samples.
 *
 * Revision 1.94  1999/02/01 21:52:23  fox
 * Added support for dumping gif files.
 *
 * Revision 1.93  1999/01/22 18:10:41  fox
 * Removed some hacks done for denver and the sampling paper.
 *
 * Revision 1.92  1999/01/22 17:48:09  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.91  1999/01/14 23:39:32  wolfram
 * Added sensorMaxRange and standardDeviaton to the sensor model for lasers
 *
 * Revision 1.90  1999/01/14 00:33:03  wolfram
 * Changes for vision
 *
 * Revision 1.89  1999/01/11 19:47:55  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.87  1999/01/08 22:28:47  wolfram
 * Better integration of scanAlignment
 *
 * Revision 1.86  1999/01/07 01:07:10  wolfram
 * Changes to integrate scan matching
 *
 * Revision 1.85  1998/11/19 03:14:29  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.84  1998/11/17 23:26:25  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.83  1998/11/03 21:02:21  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.82  1998/10/29 03:45:03  fox
 * Nothing special.
 *
 * Revision 1.81  1998/10/23 20:52:51  fox
 * Nothing specatacular.
 *
 * Revision 1.80  1998/10/19 18:29:56  fox
 * *** empty log message ***
 *
 * Revision 1.79  1998/10/02 15:16:41  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.78  1998/09/25 17:53:31  fox
 * Improved version of condensation.
 *
 * Revision 1.77  1998/09/25 04:02:58  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.76  1998/09/18 15:44:28  fox
 * First attempt to get a running system after museum.
 *
 * Revision 1.75  1998/08/23 00:01:03  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.74  1998/08/19 16:33:57  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.73  1998/08/16 18:21:25  wolfram
 * Fixed ScanMask is used when no selection is applied
 *
 * Revision 1.72  1998/06/12 10:16:38  fox
 * Implemented virutal sensor.
 *
 * Revision 1.71  1998/03/09 10:07:47  wolfram
 * slight changes
 *
 * Revision 1.70  1998/01/22 13:06:19  fox
 * First version after selection-submission.
 *
 * Revision 1.69  1997/12/19 11:30:13  fox
 * FIXED a bug I added.
 *
 * Revision 1.68  1997/11/25 17:12:58  fox
 * Should work.
 *
 * Revision 1.67  1997/10/31 13:11:44  fox
 * Version for active sensing.
 *
 * Revision 1.66  1997/08/22 04:16:40  fox
 * Final version before IJCAI.
 *
 * Revision 1.65  1997/08/16 02:48:06  wolfram
 * DistTables for Laser and sonar are now computed from the sonarMap and
 * laserMap resp.
 *
 * Revision 1.64  1997/08/02 16:51:06  wolfram
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
 * Revision 1.63  1997/07/04 17:29:16  fox
 * Final version before holiday!!!
 *
 * Revision 1.62  1997/06/27 16:26:29  fox
 * New model of the proximity sensors.
 *
 * Revision 1.61  1997/06/26 11:23:17  fox
 * Fixed a bug in normalize.
 *
 * Revision 1.60  1997/06/25 14:16:41  fox
 * Changed laser incorporation.
 *
 * Revision 1.59  1997/06/20 07:36:13  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.58  1997/06/03 11:49:23  fox
 * Museum version.
 *
 * Revision 1.57  1997/05/27 07:42:36  fox
 * Nothing special.
 *
 * Revision 1.56  1997/05/26 08:47:55  fox
 * Last version before major changes.
 *
 * Revision 1.55  1997/04/10 13:01:04  fox
 * Fixed a bug.
 *
 * Revision 1.54  1997/04/08 14:56:25  fox
 * Changed the reset function for new planes and the prob function.
 *
 * Revision 1.53  1997/04/03 13:17:51  fox
 * Some minor changes.
 *
 * Revision 1.52  1997/04/02 08:57:34  fox
 * Display the robot right after sensor selection.
 *
 * Revision 1.51  1997/03/24 21:51:44  wolfram
 * Minor changes
 *
 * Revision 1.50  1997/03/19 17:52:44  fox
 * New laser parameters.
 *
 * Revision 1.49  1997/03/18 18:45:30  fox
 * First executable version with a global scan mask for different sensor types.
 *
 * Revision 1.48  1997/03/17 18:41:15  fox
 * First version with sensor selection in cluttered environments.
 *
 * Revision 1.47  1997/03/14 17:58:22  fox
 * This version should run quite stable now.
 *
 * Revision 1.46  1997/03/13 17:36:23  fox
 * Temporary version. Don't use!
 *
 * Revision 1.45  1997/02/11 11:04:10  wolfram
 * Faster Computation of expected distances, height of sensors is considered
 *
 * Revision 1.44  1997/01/31 16:19:17  wolfram
 * Added start position to each reading of a scan
 *
 * Revision 1.43  1997/01/30 17:17:25  fox
 * New version with integrated laser.
 *
 * Revision 1.42  1997/01/29 12:23:12  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.41  1997/01/23 14:07:23  fox
 * Version of ijcai-submission.
 *
 * Revision 1.40  1997/01/20 09:37:31  wolfram
 * Stopped huge output to log file
 *
 * Revision 1.39  1997/01/19 23:35:15  fox
 * Fixed a bug in convolve.
 *
 * Revision 1.38  1997/01/19 19:31:18  fox
 * yeah
 *
 * Revision 1.37  1997/01/19 18:56:57  wolfram
 * Again a bug ...
 *
 * Revision 1.36  1997/01/19 14:05:27  wolfram
 * Added counter for number of integrated readings
 *
 * Revision 1.35  1997/01/18 19:41:03  fox
 * Improved action selection.
 *
 * Revision 1.34  1997/01/18 18:19:23  wolfram
 * *** empty log message ***
 *
 * Revision 1.33  1997/01/18 17:24:44  wolfram
 * Fixed two bugs
 *
 * Revision 1.32  1997/01/18 14:07:56  fox
 * Test version.
 *
 * Revision 1.31  1997/01/16 19:43:24  fox
 * And another bug ...
 *
 * Revision 1.30  1997/01/16 12:42:51  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.29  1997/01/14 18:26:16  fox
 * Nothing special.
 *
 * Revision 1.28  1997/01/14 16:53:24  fox
 * Removed a very very nasty bug for sensor selection.
 *
 * Revision 1.27  1997/01/14 10:38:20  wolfram
 * Test Version
 *
 * Revision 1.26  1997/01/10 15:19:24  fox
 * Improved several methods.
 *
 * Revision 1.25  1997/01/08 18:28:20  fox
 * Fixed a bug in setCellList.
 *
 * Revision 1.24  1997/01/08 15:56:13  wolfram
 * Nothing special
 *
 * Revision 1.23  1997/01/08 15:52:58  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.22  1997/01/07 12:31:27  wolfram
 * FindLocalMaxima additionally computes a list of localMaxima
 *
 * Revision 1.21  1996/12/31 09:19:25  fox
 * Last version before I switched exploration mode in PLAN to random
 * mode in COLLI.
 *
 * Revision 1.20  1996/12/09 10:12:00  wolfram
 * Fixed a bug in angle computation
 *
 * Revision 1.19  1996/12/05 16:18:33  wolfram
 * Fixed a bug in the random sensor selection
 *
 * Revision 1.18  1996/12/03 17:47:12  wolfram
 * added normalized prob function table to improve the sensor selection
 *
 * Revision 1.17  1996/12/02 18:46:26  fox
 * First version with the new expected distances.
 *
 * Revision 1.16  1996/12/02 14:18:25  fox
 * Fixed a bug in proximityDecision.c
 *
 * Revision 1.15  1996/12/02 10:32:11  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.14  1996/11/29 15:33:39  fox
 * ok
 *
 * Revision 1.13  1996/11/26 11:08:13  fox
 * Improved version.
 *
 * Revision 1.12  1996/11/25 19:35:41  fox
 * Test version for decisions of movements.
 *
 * Revision 1.11  1996/11/25 12:10:20  wolfram
 * Added procedure to initialize grid with a gridCellList
 *
 * Revision 1.10  1996/11/25 09:47:25  wolfram
 * Added graphic support for movment errors
 * Improved procedure for computing the entropy of a grid
 *
 * Revision 1.9  1996/11/22 10:30:24  wolfram
 * Added graphic for expected errors of movements
 *
 * Revision 1.8  1996/11/21 14:37:14  wolfram
 * Movement Support for Exploration
 *
 * Revision 1.7  1996/11/21 14:08:06  wolfram
 * First steps toward action selection
 *
 * Revision 1.6  1996/11/21 13:41:56  fox
 * First version to show the main structure to find the best movement of the
 * robot.
 *
 * Revision 1.5  1996/11/18 09:58:31  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.4  1996/11/15 17:44:07  ws
 * *** empty log message ***
 *
 * Revision 1.3  1996/10/24 12:07:12  fox
 * Fixed a bug.
 *
 * Revision 1.2  1996/10/24 09:56:55  fox
 * LOCALIZE also works in a write protected directory.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:32  rhino
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


#include "general.h"
#include "probGrid.h"
#include "allocate.h"
#include "function.h"
#include "sonar.h"
#include "proximity.h"
#include "proximityTools.h"
#include "movement.h"
#include "file.h"
#include "graphic.h"
#include "probGrid.h"
#include "activeLocalize.h"
#include "selection.h"
#include "graphic.h"
#include "probGridTools.h"
#include "abstract.h"
#include "localTcx.h"

static void
integrateAbstractReadingsIntoGrid( abstractSensorVector* readings);

static void
integrateAbstractReadingsIntoSamples( abstractSensorVector* readings);

static void
integrateAbstractReadingsIntoSamplesOnline( abstractSensorVector* readings);

static void
integrateTwoScansIntoSamplesOnline( abstractSensorVector* scan1,
				    abstractSensorVector* scan2);

/********************************************************************
 ********************************************************************
 * Global functions.
 ********************************************************************
 ********************************************************************/



/*****************************************************************************
 *  Sets the tables needed to compute the probabilities of distance measurements.
 *****************************************************************************/
void
initializeDistProbTables( informationsFor_PROXIMITY* info,
			  actionInformation *actionInfo,
			  int numberOfAngles,
			  sensorParameters params,
			  probabilityGrid *map,
			  char* distFile,
			  bool computeExpectedDist)
{
  /* If the table is not given in a file we compute the table
   * which includes the expected distances measured by the
   * sensors at each position and orientation in the grid. */
  simulatorMap *simMap = &(actionInfo->simMap);
  
  if ( ! computeExpectedDist) 
    info->expectedDistances = readDistTab( distFile,
					   params,
					   info->onlineMapping);

  if ( computeExpectedDist || info->expectedDistances.dist == NULL) {
    info->expectedDistances =
      preprocessedExpectedDistances( map,
				     simMap,
				     params,
				     numberOfAngles,
				     distFile,
				     info->onlineMapping);
  }

  if ( ! info->onlineMapping) {
    if ( info->expectedDistances.sizeX != map->sizeX
	 || info->expectedDistances.sizeY != map->sizeY) {
      
      fprintf( stderr, "Sorry. Sizes of expected distance table (%d, %d)",
	       info->expectedDistances.sizeX, info->expectedDistances.sizeY);
      fprintf( stderr, " don't fit map size (%d %d).\n",
	       map->sizeX, map->sizeY);
      writeLog( "Sorry. Sizes of expected distance table (%d, %d)",
		info->expectedDistances.sizeX, info->expectedDistances.sizeY);
      writeLog( " don't fit map size (%d %d).\n",
		map->sizeX, map->sizeY);
      fprintf(stderr, "Remove %s and restart\n", distFile);
      closeLogAndExit(-1);
    }
    
    info->expectedDistances.gridCellSize = map->resolution;
  }
  /* Computes the table of the probabilities of distance readings. */
  info->distProbFunctionTableDividedByPOfFeature =
    probFunctionTable( params, 
 		       info->expectedDistances); 
  info->distProbFunctionTableNormed =
    normedProbFunctionTable( params, 
			     info->expectedDistances); 
}


/***********************************************************************
 * Integrates the distance readings given by info->scan.
 ***********************************************************************/

void
integrateDistScan( informationsFor_PROXIMITY* info)
{
  /*DDD */
  if (0) { 
    int beam;
    for ( beam = 0; beam < info->abstractScan->numberOfSensors; beam++) {
      fprintf( stderr, "	expectedScan.reading[%d].dist = distIndex * %f;\n", 
	       beam, 
	       info->scan->reading[beam].dist);
    }
  }

  if ( (! info->abstractScan->chooseOptimal)
       || ( *info->useProbGrid && info->localMaxima->numberOfCells <= 0)) {
    if (0)
      setRandomScanMask(info->abstractScan);
    else 
      setFixedScanMask( info->abstractScan, 0);
  }
  
  if ( *(info->useProbGrid)) {
    integrateAbstractReadingsIntoGrid( info->abstractScan);
  }
  else 
    {
      /* If we use static maps, then we can apply preprocessed distances. */
      if ( ! info->onlineMapping)
	integrateAbstractReadingsIntoSamples( info->abstractScan);
      else 
	integrateAbstractReadingsIntoSamplesOnline( info->abstractScan);
      
    }
  info->numberOfIntegratedReadings += info->abstractScan->numberOfSensorsToBeUsed;
}

void
integrateTwoDistScans( informationsFor_PROXIMITY* info1,
		       informationsFor_PROXIMITY* info2)
{
  if ( *(info1->useProbGrid)) {
    fprintf(stderr, "Two scan option not available for prob grid.\n");
    integrateDistScan( info1);
    integrateDistScan( info2);
  }
  else {

    /* If we use static maps, then we can apply preprocessed distances. */
    if (! info1->onlineMapping) {
      integrateDistScan( info1);
      integrateDistScan( info2);
    }
    else {
      
      setFixedScanMask( info1->abstractScan, 0);
      setFixedScanMask( info2->abstractScan, 0);
      integrateTwoScansIntoSamplesOnline( info1->abstractScan,
					  info2->abstractScan);
    }
  }
  info1->numberOfIntegratedReadings += info1->abstractScan->numberOfSensorsToBeUsed;
  info2->numberOfIntegratedReadings += info2->abstractScan->numberOfSensorsToBeUsed;
}


#define NUMBER_OF_SWALLOWS_PER_INTEGRATION 10

/***********************************************************************
 * Integrates the distance readings of abstract sensors.
 ***********************************************************************/
static void

integrateAbstractReadingsIntoGrid( abstractSensorVector* scan)
{  

  if ( scan->numberOfSensorsToBeUsed > 0) {

    int x, y, plane;
    int readCnt, cnt = 0;
    
    /* For fast integration we assume that the general information to integrate
     * the distances is the same for all sensors of the scan (i.e. all sensors
     * are the same. We use the information provided in the first sensor. */
    infoForProximityFeature* info =
      (infoForProximityFeature*) scan->sensor[0].infoForFeatures;

    /* Pointer for faster access */
    probability** distProb = info->distProbFunctionTableDividedByPOfFeature->prob;
    probability minProb = info->grid->minimumProbability;
    abstractSensorType* sensor = scan->sensor;
    expectedDistance ***dist = info->distTab->dist;
    int numberOfSensorsToBeUsed = scan->numberOfSensorsToBeUsed;

    int* planeOfPlaneInDistTab;
    register probability **gridPlane;
    
    /* This is just a lookup table for the given distance readings. */
    int** planeInDistTab =
      preprocessedPlaneInDistTable( scan);

    int swallowCnt = info->grid->sizeX / NUMBER_OF_SWALLOWS_PER_INTEGRATION;
    
    if (scan->sensorOffsetType != BEAM_OFFSET){
      
      movement scanOffset = scan->sensor[0].integrationInfo.sensorOffset;
      bool scanHasOffset = scan->sensorOffsetType == SCAN_OFFSET;
      
      
      /* Step through each plane. */
      for ( plane = 0; plane < info->grid->sizeZ; plane++) {
	
	if ( info->grid->updatePlane[plane]) {
	  
	/* The center of the scan has to be converted into an offset in grid
	 * cells according to the rotation of the plane. */
	  int offsetX = 0;
	  int offsetY = 0;
	  
	  int startX = 0, startY = 0;
	  int lastX = info->grid->sizeX, lastY = info->grid->sizeY;
	  int shiftedX;
	  
	  gridPlane = info->grid->prob[plane];
	  planeOfPlaneInDistTab = planeInDistTab[plane];
	  
	  
	  if ( scanHasOffset) {
	
	    /* The center position of the scan is not necessarily the center of the
	   * robot (e.g. laser). When updating the grid we must consider this
	   * center position. */
	    
	    realPosition offset =
	      endPoint( info->grid->summedMovementOfPlane[plane], scanOffset);
	  
	    offsetX = round( offset.x / info->grid->positionResolution);
	    offsetY = round( offset.y / info->grid->positionResolution);
	    
	    /* Now adjust the limits of the counters. */
	    if ( offsetX > 0) 
	      lastX = info->grid->sizeX - offsetX;
	    else if ( offsetX < 0) 
	      startX = -offsetX;
	    if ( offsetY > 0) 
	      lastY = info->grid->sizeY - offsetY;
	    else if ( offsetY < 0) 
	      startY = -offsetY;
	  }
	  
	  /* Now do the real thing. */
	  shiftedX =  startX + offsetX;
	  for ( x = startX; x < lastX; x++) {
	    int shiftedY = startY + offsetY;
	    register probability *cell = &gridPlane[x][startY];
	    
	    /* Just to avoid crash of the system. */
	    
	    if ( x % swallowCnt == 0)
	      swallowStatusReports(DONT_WAIT);
	    
	    for ( y = startY; y < lastY; y++) {
	      
	      if ( *cell > minProb) {
		int *mask = scan->mask;		
		expectedDistance* distTabXY = dist[shiftedX][shiftedY];
		
		cnt++;
		
	      for ( readCnt = 0;
		    readCnt < numberOfSensorsToBeUsed && *cell > minProb; readCnt++) {
		
		/* The probability of the cell is given by the probability of the
		 * measured feature at the shifted position. */
		*cell *= distProb[distTabXY[planeOfPlaneInDistTab[readCnt]]]
		  
		  [sensor[*(mask++)].measuredFeature];
	      }
	      }
	      cell++;
	      shiftedY++;
	    }
	    shiftedX++;
	  }
	}
      }
    }
    else{
      /* BEAM_OFFSET */
      for ( readCnt = 0; readCnt < numberOfSensorsToBeUsed; readCnt++) {
	movement beamOffset = scan->sensor[readCnt].integrationInfo.sensorOffset;
	int measuredFeature = sensor[scan->mask[readCnt]].measuredFeature;

	/* Step through each plane. */
	for ( plane = 0; plane < info->grid->sizeZ; plane++) {
	
	  if ( info->grid->updatePlane[plane]) {
	  
	    /* The center of the scan has to be converted into an offset in grid
	     * cells according to the rotation of the plane. */
	    int offsetX = 0;
	    int offsetY = 0;
	  
	    int startX = 0, startY = 0;
	    int lastX = info->grid->sizeX, lastY = info->grid->sizeY;
	    int shiftedX;
	    int distTabPlane;
	    
	    /* The starting position of the beam is not necessarily the center of the
	     * robot (e.g. laser). When updating the grid we must consider this
	     * center position. */
	    
	    realPosition offset =
	      endPoint( info->grid->summedMovementOfPlane[plane], beamOffset);

	    gridPlane = info->grid->prob[plane];
	    distTabPlane = planeInDistTab[plane][readCnt];
	    
	    offsetX = round( offset.x / info->grid->positionResolution);
	    offsetY = round( offset.y / info->grid->positionResolution);
	    
	    /* Now adjust the limits of the counters. */
	    if ( offsetX > 0) 
	      lastX = info->grid->sizeX - offsetX;
	    else if ( offsetX < 0) 
	      startX = -offsetX;
	    if ( offsetY > 0) 
	      lastY = info->grid->sizeY - offsetY;
	    else if ( offsetY < 0) 
	      startY = -offsetY;
	  


	    /* Now do the real thing. */
	    shiftedX =  startX + offsetX;
	    for ( x = startX; x < lastX; x++) {
	      int shiftedY = startY + offsetY;
	      register probability *cell = &gridPlane[x][startY];
	      
	    /* Just to avoid crash of the system. */
	      
	      if ( x % swallowCnt == 0)
		swallowStatusReports(DONT_WAIT);
	    
	      for ( y = startY; y < lastY; y++) {
	      
		if ( *cell > minProb) {
		  cnt++;
		
		  /* The probability of the cell is given by the probability of the
		   * measured feature at the shifted position. */
		  *cell *= distProb[dist[shiftedX][shiftedY][distTabPlane]][measuredFeature];
		}
		cell++;
		shiftedY++;
	      }
	      shiftedX++;
	    }
	  }
	}
      }
    }
      
    
    info->grid->quotaOfValuesToBeUpdated = (float) cnt /
      (float) (info->grid->sizeX * info->grid->sizeY * info->grid->sizeZ );
    
#ifdef KURT
    { 
      double t = (double) timeExpired(0);
      double c = (double) cnt / 1000000;
      fprintf(stderr, "%f %d %f --> %f %f\n", t, cnt, c, 
	      t / (float) (cnt), t / c);
    }
#endif
  }
}


/***********************************************************************
 * Integrates the distance readings of abstract sensors.
 ***********************************************************************/
static void
integrateAbstractReadingsIntoSamplesOnline( abstractSensorVector* scan)
{
  if ( scan->numberOfSensorsToBeUsed > 0) {
    
    int readCnt, cnt = 0, s;
    double sumOfIntegratedWeight = 0.0;
    
    /* For fast integration we assume that the general information to integrate
     * the distances is the same for all sensors of the scan (i.e. all sensors
     * are the same. We use the information provided in the first sensor. */
    infoForProximityFeature* info =
      (infoForProximityFeature*) scan->sensor[0].infoForFeatures;

    /* The samples. */
    sampleSet* samples = info->samples;
    int stopIntegration = samples->variableSampleSize;
      /* If only for laser: && (scan->sensor[0].type == FRONT_LASER_TYPE); */
    float stopThreshold = samples->integrateThreshold;
    float minNumberOfSamples = samples->minNumberOfSamples - 1;
    
    /* Pointer for faster access */
    abstractSensorType* sensor = scan->sensor;
    int numberOfSensorsToBeUsed = scan->numberOfSensorsToBeUsed;
    int *mask = scan->mask;

    distProbTable* distProbTable = info->distProbFunctionTableDividedByPOfFeature;
    probability** distProb = info->distProbFunctionTableDividedByPOfFeature->prob;

    /* Get resolution information from the dist prob table. */
    float distanceResolution = distProbTable->distanceResolution;
    int numberOfStdDevs = distProbTable->numberOfStdDevs;

    int swallowCnt = samples->numberOfSamples / NUMBER_OF_SWALLOWS_PER_INTEGRATION;

#define ACCURACY
#ifdef ACCURACY
    simulatorMap *simMap = info->simMap;
    expectedDistTable* distanceTable = info->distTab;
#endif
    
    if (scan->sensorOffsetType != BEAM_OFFSET){
      
      /* The offset of the scan relative to the samples. */
      movement scanOffset = scan->sensor[0].integrationInfo.sensorOffset;
      bool scanHasOffset = scan->sensorOffsetType == SCAN_OFFSET;
      
      /* We use the grid map to compute the expected distances. */
      probabilityGrid* map = info->gridMap;
      
      
      /* First get the relative angles of the beams in the scan. This has to
       * be done only once. */    
      float beamAngle[MAX_SIZE_OF_SCAN];
      for ( readCnt = 0; readCnt < numberOfSensorsToBeUsed; readCnt++) {
	infoForProximityFeature* info =
	  (infoForProximityFeature*) scan->sensor[scan->mask[readCnt]].infoForFeatures;
	beamAngle[readCnt] = info->sensorRot;
      }
      
      /*--------------------------------------------------------
       * Step through all the samples. 
       *--------------------------------------------------------*/
      for ( s = 0; s < samples->numberOfSamples; s++) {
	
	double* sampleProb = &(samples->sample[s].weight);
	
	int mapX, mapY;
	
	/* Center of this scan. */
	realPosition scanCenter = samples->sample[s].pos;
	if ( scanHasOffset)
	  scanCenter = endPoint( scanCenter, scanOffset);
	
	/* Map coordinate. */
	mapX = (scanCenter.x - map->offsetX) / map->resolution;
	mapY = (scanCenter.y - map->offsetY) / map->resolution;
	
	cnt++;

	if ( s % swallowCnt == 0) {
	  swallowStatusReports(DONT_WAIT);
	  if ( samples->replacedViaTcx) {
	    fprintf(stderr, "#Sample set replaced. Interrupt integration.\n");
	    writeLog( "#Sample set replaced. Interrupt integration.\n");
	    samples->alreadySampled = TRUE;
	    samples->alreadyNormalized = TRUE;
	    return;
	  }
	}
	
	/* Integrate the single beams. */
	for ( readCnt = 0;
	      readCnt < numberOfSensorsToBeUsed && *sampleProb > MINIMUM_PROBABILITY;
	      readCnt++) {
	  int distIndex, beamRot, tabValue;
	  
	  float absoluteAngle = scanCenter.rot + beamAngle[readCnt];

	  
#ifdef ACCURACY
	  if (simMap->initialized){
	    static int count = 0;
	    float distance = simulatorObjectDistance(simMap, scanCenter.x,
						     scanCenter.y,
						     35,
						     cos(absoluteAngle),
						     sin(absoluteAngle),
						     distanceTable->sensorMaxRange);
	    if (0 && count >= 10) {
	      writeLog("%f %f # expected measured %s\n",
		       distance, (float) 
		       sensor[mask[readCnt]].measuredFeature * distanceTable->distanceResolution, scan->sensor->type);
	      count = 0;
	    }
	    else
	      count ++;
	    
	    distIndex = (int) (distance / distanceResolution );
	    if (distIndex >= distProbTable->numberOfExpectedDistances)
	      distIndex = distProbTable->numberOfExpectedDistances - 1;
	  }
	  else {
	    
#endif	  
	    beamRot = rad2Deg( absoluteAngle);
	    
	    distIndex = expectedFeatureGridMap( map,
						mapX, mapY, beamRot % 360,
						distanceResolution,
						distProbTable->numberOfExpectedDistances);
#ifdef ACCURACY
	    }
#endif
	  
	  /* Don't integrate max range or unexpored areas. */
	  tabValue = distIndex * numberOfStdDevs;
	    
	  /* The probability of the cell is given by the probability of the
	   * measured feature at the shifted position. */
	  *sampleProb *= distProb[tabValue][sensor[mask[readCnt]].measuredFeature];
	}
	
	if ( stopIntegration) {
	  sumOfIntegratedWeight += *sampleProb;
	  if ( sumOfIntegratedWeight > stopThreshold && s >= minNumberOfSamples) {
	    samples->numberOfSamples = s + 1;
	  }
	}
      }
    }
    
    else
      
      {
	/* We use the grid map to compute the expected distances. */
	probabilityGrid* map = info->gridMap;
	
	/* Integrate the single beams. */
	for ( readCnt = 0; readCnt < numberOfSensorsToBeUsed; readCnt++) {

	  if ( sensor[mask[readCnt]].measuredFeature < distProbTable->numberOfExpectedDistances - 1) {
	    
	    movement beamOffset = scan->sensor[readCnt].integrationInfo.sensorOffset;
	    

	    
/* 	  fprintf(stderr, "%d: %f %f %f\n", readCnt, beamOffset.forward, beamOffset.sideward, beamOffset.rotation);	   */
	  /*--------------------------------------------------------
	   * Step through all the samples. 
	   *--------------------------------------------------------*/
	  for ( s = 0; s < samples->numberOfSamples; s++) {
	  
	    double* sampleProb = &(samples->sample[s].weight);
	    
	    int mapX, mapY;

	    int distIndex, beamRot, tabValue;
	  
	    /* Center of this scan. */
	    realPosition beamStart = endPoint(samples->sample[s].pos, beamOffset);
 	    float absoluteAngle = beamStart.rot;

	    /* Map coordinate. */
	    mapX = (beamStart.x - map->offsetX) / map->resolution;
	    mapY = (beamStart.y - map->offsetY) / map->resolution;
	    
	    cnt++;
	
	    
#ifdef ACCURACY
	    if (simMap->initialized){
	      distIndex = (int) (
				 simulatorObjectDistance(simMap,
							 beamStart.x,
							 beamStart.y,
							 35,
							 cos(absoluteAngle),
							 sin(absoluteAngle),
							 distanceTable->sensorMaxRange)
				 / distanceResolution );
	      if (distIndex >= distProbTable->numberOfExpectedDistances)
		distIndex = distProbTable->numberOfExpectedDistances - 1;
	    }
	    else {
	      
#endif	  
	      beamRot = rad2Deg( absoluteAngle);
	      
	      distIndex = expectedFeatureGridMap( map,
						  mapX, mapY, beamRot % 360,
						  distanceResolution,
						  distProbTable->numberOfExpectedDistances);
#ifdef ACCURACY
	    }
#endif
	    
	    /* Don't integrate max range or unexpored areas. */
	    tabValue = distIndex * numberOfStdDevs + 0;
	    
	    /* The probability of the cell is given by the probability of the
	     * measured feature at the shifted position. */
	    *sampleProb *= distProb[tabValue][sensor[mask[readCnt]].measuredFeature];
	    if ( stopIntegration) {
	      sumOfIntegratedWeight += *sampleProb;
	      if ( sumOfIntegratedWeight > stopThreshold && s >= minNumberOfSamples) {
		samples->numberOfSamples = s + 1;
	      }
	    }
	  }
	  }
	}
	
      }
        
    samples->alreadySampled = FALSE;
    samples->alreadyNormalized = FALSE;
  }
}


/***********************************************************************
 * Integrates the distance readings of abstract sensors.
 ***********************************************************************/
static void
integrateTwoScansIntoSamplesOnline( abstractSensorVector* scan1,
				    abstractSensorVector* scan2)
{

 if ( scan1->numberOfSensorsToBeUsed > 0) {
    
    int readCnt, cnt = 0, s;
    double sumOfIntegratedWeight = 0.0;
    
    /* For fast integration we assume that the general information to integrate
     * the distances is the same for all sensors of the scan (i.e. all sensors
     * are the same. We use the information provided in the first sensor. */
    infoForProximityFeature* info =
      (infoForProximityFeature*) scan1->sensor[0].infoForFeatures;
    
    /* The samples. */
    sampleSet* samples = info->samples;
    int stopIntegration = samples->variableSampleSize;
    float stopThreshold = samples->integrateThreshold;
    float minNumberOfSamples = samples->minNumberOfSamples - 1;    
    /* Pointer for faster access */
    abstractSensorType* sensor1 = scan1->sensor;
    abstractSensorType* sensor2 = scan2->sensor;
    int numberOfSensorsToBeUsed = scan1->numberOfSensorsToBeUsed;
    int *mask1 = scan1->mask;
    int *mask2 = scan2->mask;
    
    
    /* Sionce both tables are identical, we only need one dist prob table. */
    distProbTable* distProbTable = info->distProbFunctionTableDividedByPOfFeature;
    probability** distProb = info->distProbFunctionTableDividedByPOfFeature->prob;
    
    /* Get resolution information from the dist prob table. */
    float distanceResolution = distProbTable->distanceResolution;
    int numberOfStdDevs = distProbTable->numberOfStdDevs;


    if (scan1->sensorOffsetType == BEAM_OFFSET ||
	scan2->sensorOffsetType == BEAM_OFFSET){
      fprintf(stderr, "Sorry, BEAM_OFFSET not yet implemented\n");
    }
    else
      {
	/* The offset of the scan relative to the samples. */
	movement scan1Offset = scan1->sensor[0].integrationInfo.sensorOffset;
	movement scan2Offset = scan2->sensor[0].integrationInfo.sensorOffset;
	bool scan1HasOffset = scan1->sensorOffsetType == SCAN_OFFSET;
	bool scan2HasOffset = scan2->sensorOffsetType == SCAN_OFFSET;
	
	/* We use the grid map to compute the expected distances. */
	probabilityGrid* map = info->gridMap;
	
	/* First get the relative angles of the beams in the scan. This has to
	 * be done only once. */    
	float beamAngle1[MAX_SIZE_OF_SCAN];
	float beamAngle2[MAX_SIZE_OF_SCAN];
	
	for ( readCnt = 0; readCnt < numberOfSensorsToBeUsed; readCnt++) {
	  infoForProximityFeature* info =
	    (infoForProximityFeature*) scan1->sensor[scan1->mask[readCnt]].infoForFeatures;
	  beamAngle1[readCnt] = info->sensorRot;
	  
	  info = (infoForProximityFeature*) scan2->sensor[scan2->mask[readCnt]].infoForFeatures;
	  beamAngle2[readCnt] = info->sensorRot;
	}
	
	/*--------------------------------------------------------
	 * Step through all the samples. 
	 *--------------------------------------------------------*/
	for ( s = 0; s < samples->numberOfSamples; s++) {
	  
	  double* sampleProb = &(samples->sample[s].weight);
	  
	  int map1X, map1Y;
	  int map2X, map2Y;
	  
	  realPosition robotPos = samples->sample[s].pos;
	  realPosition scan1Center = robotPos;
	  realPosition scan2Center = robotPos;
	  
	  if ( scan1HasOffset)
	    scan1Center = endPoint( robotPos, scan1Offset);
	  
	  if ( scan2HasOffset)
	    scan2Center = endPoint( robotPos, scan2Offset);
	  
	  /* Map coordinate. */
	  map1X = (scan1Center.x - map->offsetX) / map->resolution;
	  map1Y = (scan1Center.y - map->offsetY) / map->resolution;
	  
	  map2X = (scan2Center.x - map->offsetX) / map->resolution;
	  map2Y = (scan2Center.y - map->offsetY) / map->resolution;
	  
	  cnt++;
	  
	  /* Integrate the single beams. */
	  for ( readCnt = 0;
		readCnt < numberOfSensorsToBeUsed && *sampleProb > MINIMUM_PROBABILITY;
		readCnt++) {
	    
#ifdef DONT_USE_MAX_RANGE
	    if ( sensor1[mask1[readCnt]].measuredFeature
		 < distProbTable->numberOfExpectedDistances-1) {
#endif
	      
	      int distIndex, beamRot, tabValue;
	      float absoluteAngle;
	      
	      /* First scan. */
	      absoluteAngle = scan1Center.rot + beamAngle1[readCnt];
	      beamRot = rad2Deg( absoluteAngle);
	      
	      distIndex = expectedFeatureGridMap( map,
						  map1X, map1Y, beamRot % 360,
						  distanceResolution,
						  distProbTable->numberOfExpectedDistances);
	      
	      tabValue = distIndex * numberOfStdDevs;
	      
	      /* The probability of the cell is given by the probability of the
	       * measured feature at the shifted position. */
	      *sampleProb *= distProb[tabValue][sensor1[mask1[readCnt]].measuredFeature];
	      
	      /* Second scan. */
	      absoluteAngle = scan2Center.rot + beamAngle2[readCnt];
	      beamRot = rad2Deg( absoluteAngle);
	      
	      distIndex = expectedFeatureGridMap( map,
						  map2X, map2Y, beamRot % 360,
						  distanceResolution,
						  distProbTable->numberOfExpectedDistances);
	      
	      tabValue = distIndex * numberOfStdDevs;
	      
	      /* The probability of the cell is given by the probability of the
	       * measured feature at the shifted position. */
	      *sampleProb *= distProb[tabValue][sensor2[mask2[readCnt]].measuredFeature];
	      
#ifdef DONT_USE_MAX_RANGE
 	    }
	    else
	      fprintf(stderr, "maX %d\n", sensor1[mask1[readCnt]].measuredFeature);
#endif
	  }
	  if ( stopIntegration) {
	    sumOfIntegratedWeight += *sampleProb;
	    if ( sumOfIntegratedWeight > stopThreshold && s >= minNumberOfSamples) {
	      samples->numberOfSamples = s + 1;
	    }
	  }
	}
      }
    samples->alreadySampled = FALSE;
    samples->alreadyNormalized = FALSE;
  }
}



static void
integrateAbstractReadingsIntoSamples( abstractSensorVector* scan)
{  

  if ( scan->numberOfSensorsToBeUsed > 0) {

    double sumOfIntegratedWeight = 0.0;
    int readCnt, cnt = 0, s;

    int gridX, gridY, planeOfScanCenter;
    expectedDistance* distTabXY;

    /* For fast integration we assume that the general information to integrate
     * the distances is the same for all sensors of the scan (i.e. all sensors
     * are the same. We use the information provided in the first sensor. */
    infoForProximityFeature* info =
      (infoForProximityFeature*) scan->sensor[0].infoForFeatures;

    sampleSet* samples = info->samples;
    int stopIntegration = samples->variableSampleSize;
    float stopThreshold = samples->integrateThreshold;
    float minNumberOfSamples = samples->minNumberOfSamples - 1;
            
    /* Pointer for faster access */
    abstractSensorType* sensor = scan->sensor;
    int numberOfSensorsToBeUsed = scan->numberOfSensorsToBeUsed;
    expectedDistTable* distanceTable = info->distTab;
    expectedDistance ***dist = info->distTab->dist;
    probability** distProb = info->distProbFunctionTableDividedByPOfFeature->prob;     
    /* probability** distProb = info->distProbFunctionTableNormed->prob; */
    int *mask = scan->mask;

    /* To convert real positions into grid positions. */
    float distanceResolution  = distanceTable->gridCellSize;

    /* The hardest part is to get the right angle plane in the distance table. */
    int relPlaneOfBeam[MAX_SIZE_OF_SCAN];
    int numberOfPlanes = distanceTable->sizeZ;


    if (scan->sensorOffsetType != BEAM_OFFSET) {
      
    /* The offset of the scan relative to the samples. */
      movement scanOffset = scan->sensor[0].integrationInfo.sensorOffset;
      bool scanHasOffset = scan->sensorOffsetType == SCAN_OFFSET;
      
      /* First get the relative angles of the beams in the scan. This has to
       * be done only once. */    
      for ( readCnt = 0; readCnt < numberOfSensorsToBeUsed; readCnt++) {
	infoForProximityFeature* info =
	  (infoForProximityFeature*) scan->sensor[scan->mask[readCnt]].infoForFeatures;
	relPlaneOfBeam[readCnt] = distTabPlaneOfRotation( info->sensorRot, distanceTable);
      }
      
      /*--------------------------------------------------------
       * Step through all the samples. 
       *--------------------------------------------------------*/
      for ( s = 0; s < samples->numberOfSamples; s++) {
	
	double* sampleProb = &(samples->sample[s].weight);
	
	/* Center of this scan. */
	realPosition scanCenter = samples->sample[s].pos;
	if ( scanHasOffset)
	  scanCenter = endPoint( scanCenter, scanOffset);
	
	
	/* Get the position in the distance table. Independent of the beam. */
	gridX = scanCenter.x / distanceResolution;
	gridY = scanCenter.y / distanceResolution;
	
	if ( gridX > 0 && gridX < distanceTable->sizeX &&
	     gridY > 0 && gridY < distanceTable->sizeY) {
	  
	  distTabXY = dist[gridX][gridY];
	  
	  planeOfScanCenter = distTabPlaneOfRotation( scanCenter.rot, distanceTable);
	  
	  cnt++;
	  for ( readCnt = 0;
		readCnt < numberOfSensorsToBeUsed && *sampleProb > MINIMUM_PROBABILITY;
		readCnt++) {
	    
	    int planeOfBeam =
	      (planeOfScanCenter + relPlaneOfBeam[readCnt]) % numberOfPlanes;
	    
	    /* The probability of the cell is given by the probability of the
	     * measured feature at the shifted position. */
	    *sampleProb *= distProb[distTabXY[planeOfBeam]]
	      [sensor[mask[readCnt]].measuredFeature];
	  }
	}
	else
	  *sampleProb = 0.0;
	
	if ( stopIntegration) {
	  sumOfIntegratedWeight += *sampleProb;
	  if ( sumOfIntegratedWeight > stopThreshold && s >= minNumberOfSamples) {
	    samples->numberOfSamples = s + 1;
	  }
	}
      }
    }
    else {
      /* First get the relative angles of the beams in the scan. This has to
       * be done only once. */    
      for ( readCnt = 0; readCnt < numberOfSensorsToBeUsed; readCnt++) {
	infoForProximityFeature* info =
	  (infoForProximityFeature*) scan->sensor[scan->mask[readCnt]].infoForFeatures;
	movement beamOffset =
	  scan->sensor[readCnt].integrationInfo.sensorOffset;

	/*--------------------------------------------------------
	 * Step through all the samples. 
	 *--------------------------------------------------------*/
	for ( s = 0; s < samples->numberOfSamples; s++) {
	  
	  double* sampleProb = &(samples->sample[s].weight);
	  
	  /* Center of this scan. */
	  realPosition beamStart = endPoint(samples->sample[s].pos, beamOffset);
	  
	  int planeOfBeam = distTabPlaneOfRotation( samples->sample[s].pos.rot +
						    scan->sensor[readCnt].integrationInfo.sensorOffset.rotation,
						    distanceTable);
	  /* Get the position in the distance table. Independent of the beam. */
	  gridX = beamStart.x / distanceResolution;
	  gridY = beamStart.y / distanceResolution;
	  
	  if ( gridX > 0 && gridX < distanceTable->sizeX &&
	       gridY > 0 && gridY < distanceTable->sizeY) {
	    
	    distTabXY = dist[gridX][gridY];
	    
	    cnt++;
	    
	    /* The probability of the cell is given by the probability of the
	     * measured feature at the shifted position. */
	    *sampleProb *= distProb[distTabXY[planeOfBeam]]
	      [sensor[mask[readCnt]].measuredFeature];
	  }
	  else
	    *sampleProb = 0.0;
	  
	  if ( stopIntegration) {
	    sumOfIntegratedWeight += *sampleProb;
	    if ( sumOfIntegratedWeight > stopThreshold && s >= minNumberOfSamples) {
	      samples->numberOfSamples = s + 1;
	    }
	  }
	}
      }
      fprintf(stderr, "Sorry, BEAM_OFFSET not yet implemented\n");
    }

   samples->alreadySampled = FALSE;
    samples->alreadyNormalized = FALSE;
   if ( stopIntegration)
     fprintf(stderr, "stop %d samples %d %f %f\n",
	     stopIntegration, samples->numberOfSamples, stopThreshold,
	     sumOfIntegratedWeight);
  }
}












