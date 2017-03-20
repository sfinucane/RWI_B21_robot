
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/proximity.h,v $
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
 * $Log: proximity.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.37  1999/01/22 17:48:09  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.36  1999/01/14 23:39:33  wolfram
 * Added sensorMaxRange and standardDeviaton to the sensor model for lasers
 *
 * Revision 1.35  1999/01/11 19:47:55  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.34  1998/11/17 23:26:26  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.33  1998/10/23 20:52:51  fox
 * Nothing specatacular.
 *
 * Revision 1.32  1998/10/02 15:16:42  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.31  1998/09/25 17:53:32  fox
 * Improved version of condensation.
 *
 * Revision 1.30  1998/09/25 04:02:58  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.29  1998/08/20 00:23:01  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.28  1997/08/22 04:16:40  fox
 * Final version before IJCAI.
 *
 * Revision 1.27  1997/08/16 02:48:06  wolfram
 * DistTables for Laser and sonar are now computed from the sonarMap and
 * laserMap resp.
 *
 * Revision 1.26  1997/08/02 16:51:07  wolfram
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
 * Revision 1.25  1997/07/04 17:29:16  fox
 * Final version before holiday!!!
 *
 * Revision 1.24  1997/06/27 16:26:30  fox
 * New model of the proximity sensors.
 *
 * Revision 1.23  1997/06/25 14:16:41  fox
 * Changed laser incorporation.
 *
 * Revision 1.22  1997/06/03 11:49:24  fox
 * Museum version.
 *
 * Revision 1.21  1997/05/19 21:42:15  wolfram
 * Distance Probabilty Function is now read from a file
 *
 * Revision 1.20  1997/04/07 11:03:23  fox
 * Should be ok.
 *
 * Revision 1.19  1997/04/03 13:17:52  fox
 * Some minor changes.
 *
 * Revision 1.18  1997/03/18 18:45:31  fox
 * First executable version with a global scan mask for different sensor types.
 *
 * Revision 1.17  1997/03/17 18:41:15  fox
 * First version with sensor selection in cluttered environments.
 *
 * Revision 1.16  1997/03/13 17:36:38  fox
 * Temporary version. Don't use!
 *
 * Revision 1.15  1997/01/30 17:17:26  fox
 * New version with integrated laser.
 *
 * Revision 1.14  1997/01/29 12:23:13  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.13  1997/01/19 14:05:27  wolfram
 * Added counter for number of integrated readings
 *
 * Revision 1.12  1997/01/16 12:42:52  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.11  1997/01/13 16:54:13  fox
 * Nothing special.
 *
 * Revision 1.10  1997/01/10 15:19:24  fox
 * Improved several methods.
 *
 * Revision 1.9  1996/12/03 23:06:01  wolfram
 * added a normed distProbFunctionTab to the distProbTable struct to improve
 * selection of optimal sensor
 *
 * Revision 1.8  1996/12/02 10:32:12  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.7  1996/11/29 15:33:40  fox
 * ok
 *
 * Revision 1.6  1996/11/25 19:35:42  fox
 * Test version for decisions of movements.
 *
 * Revision 1.5  1996/11/25 12:10:20  wolfram
 * Added procedure to initialize grid with a gridCellList
 *
 * Revision 1.4  1996/11/21 14:37:15  wolfram
 * Movement Support for Exploration
 *
 * Revision 1.3  1996/11/18 09:58:32  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.2  1996/11/15 17:44:08  ws
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:33  rhino
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


#ifndef PROXIMITY_INCLUDE
#define PROXIMITY_INCLUDE

#include "general.h"
#include "sensings.h"
#include "probGrid.h"
#include "function.h"
#include "probGridTools.h"

#define MAX_SIZE_OF_SCAN 400

/*********************************************************
 *********************************************************
 * Structures.
 *********************************************************
 *********************************************************/

typedef unsigned char expectedDistance;


/* This struct is used to get the probability of a measured distance
 * given the expected distance.
 */

typedef struct {
  distance distanceResolution;
  int numberOfMeasuredDistances;
  int numberOfExpectedDistances;
  int numberOfStdDevs;
  probability** prob;
} distProbTable;

/* This struct is used to get the probability of a measured distance
 * given the expected distance.
 */
typedef struct {
  int sizeX;
  int sizeY;
  int sizeZ;
  float angleResolution;
  distance distanceResolution;
  int numberOfExpectedDistances;
  int numberOfStdDevs;
  float sensorMaxRange;
  float gridCellSize;
  expectedDistance*** dist;
} expectedDistTable;

/* This struct stores information needed to integrate the distance information. */
typedef struct {
  int*                    useProbGrid;
  positionProbabilityGrid* grid;
  sampleSet*               samples;
  probabilityGrid*         map;
  probabilityGrid*         initialPositionProbs;
  simulatorMap*            simMap;
  sensing_PROXIMITY*       scan;
  abstractSensorVector*    abstractScan; 
  distProbTable            distProbFunctionTableDividedByPOfFeature;
  distProbTable            distProbFunctionTableNormed;
  expectedDistTable        expectedDistances;
  realCellList*            localMaxima;
  int                      numberOfIntegratedReadings;
  int                      computeExpectedDistanceOnline;
  int                      onlineMapping;
} informationsFor_PROXIMITY;


typedef struct {
  int numberOfStdDevs;
  int numberOfExpectedDistances;
  int numberOfMeasuredDistances;
  float sensorMaxRange;
  float openingAngle;
  float sensorHeight;
  float maxFactor;
  float *stdDevThreshold;
  char *probFunctionFileName;
} sensorParameters;

  
/* This struct is used to store the information necessary to compute the
 * probability of a single sensor measurement in a certain direction. */
typedef struct {
  positionProbabilityGrid *grid;
  sampleSet* samples;
  expectedDistTable *distTab;
  distProbTable* distProbFunctionTableDividedByPOfFeature;
  distProbTable* distProbFunctionTableNormed;
  distProbTable* distProbFunctionTable;
  float* uninformedNormed;
  float* uninformedDividedByPOfFeature;
  float sensorRot;

  /* Used for online computation of the expected distances. */
  probabilityGrid*           gridMap;
  simulatorMap*              simMap;
} infoForProximityFeature;

/* A function to compute the expected distance of a sensor given the map,
 * position in the map, the orientation of the sensor, and MAX_RANGE of
 * the sensor.
 */
typedef distance (obstacleDistFunction) ( probabilityGrid map,
					  int xPos, int yPos,
					  float rot,
					  distance maxRange); 

  
/***********************************************************************
 * Returns the probability of a reading given the grid position
 * and the probability function for the distances.
 ***********************************************************************/
void
integrateDistScan( informationsFor_PROXIMITY* info);

void
integrateTwoDistScans( informationsFor_PROXIMITY* info1,
		       informationsFor_PROXIMITY* info2);

void
initializeDistProbTables( informationsFor_PROXIMITY* info,
			  actionInformation *actionInfo,
			  int numberOfAngles,
			  sensorParameters params,
			  probabilityGrid *map,
			  char* distFile,
			  bool computeExpectedDist);

#endif






