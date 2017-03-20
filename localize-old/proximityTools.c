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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/proximityTools.c,v $
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
 * $Log: proximityTools.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.103  1999/12/16 16:14:00  fox
 * Several preparation changes for angles.
 *
 * Revision 1.102  1999/12/10 15:20:17  wolfram
 * Improved version for on-line computation of expected distances
 *
 * Revision 1.101  1999/11/02 18:12:37  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.100  1999/07/12 15:24:54  fox
 * Minor changes in Texas.
 *
 * Revision 1.99  1999/04/18 19:00:10  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.98  1999/02/17 19:42:25  fox
 * Enhanced gif utilities.
 *
 * Revision 1.97  1999/01/22 17:48:10  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.96  1999/01/22 17:37:03  wolfram
 * For Dieter
 *
 * Revision 1.95  1999/01/14 23:39:33  wolfram
 * Added sensorMaxRange and standardDeviaton to the sensor model for lasers
 *
 * Revision 1.94  1999/01/14 00:33:04  wolfram
 * Changes for vision
 *
 * Revision 1.93  1999/01/11 19:47:56  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.91  1998/12/16 14:59:04  wolfram
 * First version without libGetDistance. Use with caution.
 *
 * Revision 1.90  1998/12/16 08:51:50  wolfram
 * This version does not need the libGetDistance to compute the expected Distances
 *
 * Revision 1.89  1998/11/19 03:14:30  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.88  1998/11/17 23:26:26  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.87  1998/10/29 03:45:04  fox
 * Nothing special.
 *
 * Revision 1.86  1998/10/23 20:52:52  fox
 * Nothing specatacular.
 *
 * Revision 1.85  1998/10/19 18:29:57  fox
 * *** empty log message ***
 *
 * Revision 1.84  1998/10/02 15:16:42  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.83  1998/09/25 17:53:32  fox
 * Improved version of condensation.
 *
 * Revision 1.82  1998/08/31 22:29:23  wolfram
 * Several changes
 *
 * Revision 1.81  1998/08/20 00:23:02  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.80  1998/08/11 23:05:41  wolfram
 * Changes for the Smithsonian museum.
 *
 * WARNING: LOCALIZE now uses only one stddev for Lasers. Laser beams have 25 m
 * Maxrange now.
 * Please use ~wolfram/bee/data/localize/params/laser-0.1stdev with this version
 * of LOCALIZE
 *
 * Revision 1.79  1998/06/12 10:16:39  fox
 * Implemented virutal sensor.
 *
 * Revision 1.78  1998/04/08 21:59:10  wolfram
 * Completed multi-robot support. Added neat stuff for nick.
 *
 * Revision 1.77  1998/04/06 19:44:14  wolfram
 * Added -robot option for multi-robot support
 *
 * Revision 1.76  1998/03/02 06:35:24  wolfram
 * Added output of sensor errror values. Small change in graphic.c
 *
 * Revision 1.75  1998/02/13 14:12:24  fox
 * Minor changes.
 *
 * Revision 1.74  1998/02/12 15:47:23  derr
 * compiler warnings (unused variable ... ) removed
 *
 * Revision 1.73  1998/01/29 16:46:52  fox
 * Removed some hacks.
 *
 * Revision 1.72  1998/01/22 13:06:20  fox
 * First version after selection-submission.
 *
 * Revision 1.71  1998/01/08 12:19:47  wolfram
 * Minor changes for Sebastians new maps
 *
 * Revision 1.70  1998/01/05 10:37:15  fox
 * Several changes especially in selection parts.
 *
 * Revision 1.69  1997/12/15 14:03:12  wolfram
 * Faster reading of expected distance tables
 *
 * Revision 1.68  1997/12/02 15:20:41  fox
 * Nothing remarkable.
 *
 * Revision 1.67  1997/11/27 18:11:21  fox
 * Several changes to make angles work better.
 *
 * Revision 1.66  1997/11/26 15:47:44  fox
 * Added some structures for questions.
 *
 * Revision 1.65  1997/11/25 17:13:00  fox
 * Should work.
 *
 * Revision 1.64  1997/11/20 12:58:14  fox
 * Version with good sensor selection.
 *
 * Revision 1.63  1997/11/07 12:39:42  fox
 * Added some graphic features.
 *
 * Revision 1.62  1997/10/31 13:11:44  fox
 * Version for active sensing.
 *
 * Revision 1.61  1997/10/29 09:11:41  wolfram
 * Simulator map is now installed only once
 *
 * Revision 1.60  1997/10/01 11:30:00  fox
 * Minor changes.
 *
 * Revision 1.59  1997/09/25 16:37:18  wolfram
 * Fixed a bug in setMaxFactor
 *
 * Revision 1.58  1997/09/09 19:45:13  fox
 * Fixed a bug in finding local maxima and did several changes to active
 * localization.
 *
 * Revision 1.57  1997/08/22 04:16:40  fox
 * Final version before IJCAI.
 *
 * Revision 1.56  1997/08/22 02:14:55  wolfram
 * If a beam reaches an unknown field we assume that maxRange is measured
 *
 * Revision 1.55  1997/08/18 19:34:20  wolfram
 * Small changes in graphic.c and proximityTools.c
 *
 * Revision 1.54  1997/08/16 22:59:52  fox
 * Last version before I change selsection.
 *
 * Revision 1.53  1997/08/02 16:51:07  wolfram
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
 * Revision 1.52  1997/07/04 17:29:17  fox
 * Final version before holiday!!!
 *
 * Revision 1.51  1997/06/27 20:52:59  wolfram
 * Intermediate version reading initial feature probs (sonar only)
 *
 * Revision 1.50  1997/06/27 16:26:30  fox
 * New model of the proximity sensors.
 *
 * Revision 1.49  1997/06/26 11:23:18  fox
 * Fixed a bug in normalize.
 *
 * Revision 1.48  1997/06/25 14:16:42  fox
 * Changed laser incorporation.
 *
 * Revision 1.47  1997/06/20 07:36:14  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.46  1997/06/03 11:49:24  fox
 * Museum version.
 *
 * Revision 1.45  1997/05/26 10:49:44  fox
 * Changed proximity dump.
 *
 * Revision 1.44  1997/05/26 10:32:03  fox
 * Added dump of rear laser and sonar data.
 *
 * Revision 1.43  1997/05/26 08:47:57  fox
 * Last version before major changes.
 *
 * Revision 1.42  1997/05/19 21:42:15  wolfram
 * Distance Probabilty Function is now read from a file
 *
 * Revision 1.41  1997/05/11 19:08:08  wolfram
 * Faster normalization, slight changes in graphic.c
 *
 * Revision 1.40  1997/04/30 12:25:42  fox
 * Some minor changes.
 *
 * Revision 1.39  1997/04/25 15:52:54  wolfram
 * ProbFunctionTable is now saved in nicer gnuplot format
 *
 * Revision 1.38  1997/04/25 07:42:02  wolfram
 * Added function to read a distProbTable from a file
 *
 * Revision 1.37  1997/04/10 13:01:04  fox
 * Fixed a bug.
 *
 * Revision 1.36  1997/04/08 14:56:26  fox
 * Changed the reset function for new planes and the prob function.
 *
 * Revision 1.35  1997/04/07 11:03:23  fox
 * Should be ok.
 *
 * Revision 1.34  1997/04/06 22:38:33  wolfram
 * Dumping expected and measured features in log file now
 *
 * Revision 1.33  1997/03/24 06:55:29  wolfram
 * Cleaned up graphic.c and added a graphic window as a global variable
 * in graphic.c
 *
 * Revision 1.32  1997/03/20 21:52:42  wolfram
 * Fixed bug in obstacleDistanceReal
 *
 * Revision 1.31  1997/03/20 14:26:49  fox
 * Should be a good version.
 *
 * Revision 1.30  1997/03/19 17:52:44  fox
 * New laser parameters.
 *
 * Revision 1.29  1997/03/18 18:45:31  fox
 * First executable version with a global scan mask for different sensor types.
 *
 * Revision 1.28  1997/03/14 17:58:23  fox
 * This version should run quite stable now.
 *
 * Revision 1.27  1997/03/13 17:56:26  wolfram
 * Small change
 *
 * Revision 1.26  1997/03/13 17:36:24  fox
 * Temporary version. Don't use!
 *
 * Revision 1.25  1997/02/11 11:04:11  wolfram
 * Faster Computation of expected distances, height of sensors is considered
 *
 * Revision 1.24  1997/01/31 16:19:17  wolfram
 * Added start position to each reading of a scan
 *
 * Revision 1.23  1997/01/30 17:17:26  fox
 * New version with integrated laser.
 *
 * Revision 1.22  1997/01/29 12:23:14  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.21  1997/01/19 23:35:16  fox
 * Fixed a bug in convolve.
 *
 * Revision 1.20  1997/01/19 18:56:57  wolfram
 * Again a bug ...
 *
 * Revision 1.19  1997/01/14 16:53:25  fox
 * Removed a very very nasty bug for sensor selection.
 *
 * Revision 1.18  1997/01/14 10:38:21  wolfram
 * Test Version
 *
 * Revision 1.17  1997/01/13 16:54:14  fox
 * Nothing special.
 *
 * Revision 1.16  1997/01/08 15:52:59  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.15  1997/01/03 18:07:47  fox
 * Successfully localized the robot and moved it into several rooms without
 * loosing the position again.
 *
 * Revision 1.14  1997/01/03 13:00:13  wolfram
 * New parameters for the distprobfunction
 *
 * Revision 1.13  1996/12/31 09:19:25  fox
 * Last version before I switched exploration mode in PLAN to random
 * mode in COLLI.
 *
 * Revision 1.12  1996/12/19 14:33:29  wolfram
 * More sensitive computation of expected distance tables
 *
 * Revision 1.11  1996/12/09 10:12:01  wolfram
 * Fixed a bug in angle computation
 *
 * Revision 1.10  1996/12/03 17:47:11  wolfram
 * added normalized prob function table to improve the sensor selection
 *
 * Revision 1.9  1996/12/02 18:46:27  fox
 * First version with the new expected distances.
 *
 * Revision 1.8  1996/12/02 13:27:10  wolfram
 * better dist prob functions
 *
 * Revision 1.7  1996/12/02 10:32:12  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.6  1996/11/29 15:33:40  fox
 * ok
 *
 * Revision 1.5  1996/11/29 09:31:39  fox
 * ok
 *
 * Revision 1.4  1996/11/15 17:44:08  ws
 * *** empty log message ***
 *
 * Revision 1.3  1996/10/24 12:07:13  fox
 * Fixed a bug.
 *
 * Revision 1.2  1996/10/24 09:56:56  fox
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

#include <values.h>

#include "general.h"
#include "file.h"
#include "probGrid.h"
#include "allocate.h"
#include "function.h"
#include "proximity.h"
#include "proximityTools.h"
#include "laser.h"
#include "sonar.h"
#include "movement.h"
#include "abstract.h"
#include "script.h"
#include "colliTcx.h"


#define ANGLE_RESOLUTION 360
/* degree */


#define OBSTACLE_THRESHOLD 0.9
#define START_OFFSET 0.5      

#define PARAMPATH "../params/"

/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/


static int
stdDevIndex(float stdDev, sensorParameters params);

static bool
inDistTab( gridPosition gridPos, expectedDistTable *distTab);

static void
setMaxFactor( distProbTable* table,
	      float maxFactor);

static void
convolveWithAverage( distProbTable* table,
		     float maxFactor);


/********************************************************************
 ********************************************************************
 * Global functions.
 ********************************************************************
 ********************************************************************/

void
setNormedDistTab( void* infoForFeature,
		  float** uninformed)
{
  infoForProximityFeature* info = (infoForProximityFeature*) infoForFeature;

  info->distProbFunctionTable = info->distProbFunctionTableNormed;
  *uninformed = info->uninformedNormed;
}

void
setUnNormedDistTab( void* infoForFeature,
		    float** uninformed)
{
  infoForProximityFeature* info = (infoForProximityFeature*) infoForFeature;

  info->distProbFunctionTable = info->distProbFunctionTableDividedByPOfFeature;
  *uninformed = info->uninformedDividedByPOfFeature;
}


/*****************************************************************************
 *  Returns the rotation of the given plane in the grid.
 *****************************************************************************/
float
rotationOfDistTabPlane( int plane, expectedDistTable* tab)
{
  return normalizedAngle( plane * tab->angleResolution);
}


/*****************************************************************************
 *  Returns the plane closest to the rotation.
 *****************************************************************************/
int
distTabPlaneOfRotation( float rot, expectedDistTable* tab)
{
  return round( rot / tab->angleResolution) % tab->sizeZ;
}


/***********************************************************************
 * Returns the probability of a reading given the reference position
 * in the table of the expected distances and the probability function
 * for the distances.
 ***********************************************************************/
probability
distProb_GivenPositionAndPlane( int x,
				int y,
				int plane,
				int distIndex,
				expectedDistTable* expectedDists,
				distProbTable* distProbs)
{
  return distProbs->prob[expectedDists->dist[x][y][plane]][distIndex];
}

/***********************************************************************
 * Returns the probability of a reading given the grid position
 * and the probability function for the distances.
 ***********************************************************************/
probability
distProb_GivenPositionAndRot( int distIndex,
			      gridPosition pos,
			      float sensorRotation,
			      positionProbabilityGrid* grid,
			      expectedDistTable* expectedDists,
			      distProbTable* distProbFunction)
{
  int expectedDist, distTabPlane;
  float planeAngle;

  planeAngle = rotationOfPlane( pos.rot, grid);

  distTabPlane = distTabPlaneOfRotation( planeAngle
					 + sensorRotation,
					 expectedDists);

  expectedDist = expectedDists->dist[pos.x][pos.y][distTabPlane];

  return distProbFunction->prob[expectedDist][distIndex];
}

/***********************************************************************
 * Returns the probability of a feature to be measured at a given position.
 **********************************************************************/
float
probabilityOfProximityFeatureGivenPosition( int feature,
					    gridPosition position,
					    void* infoForFeature)
{
  infoForProximityFeature* info = (infoForProximityFeature*) infoForFeature;

  return distProb_GivenPositionAndRot( feature,
				       position,
				       info->sensorRot,
				       info->grid,
				       info->distTab,
				       info->distProbFunctionTable);
}


float
probabilityOfProximityFeatureGivenExpected( int feature,
					    featureStruct* expectedFeature,
					    void* infoForFeature)
{
  infoForProximityFeature* info = (infoForProximityFeature*) infoForFeature;

  return info->distProbFunctionTable->prob[expectedFeature->integrateNumber][feature];
}


void
setQuality( featureStruct* fStruct, int quality)
{

  fStruct->integrateNumber = fStruct->feature * fStruct->numberOfQualities
    + quality;
  fStruct->quality = quality;
}


featureStruct
expectedProximityFeature( gridPosition position,
			  void* infoForFeature)
{
  infoForProximityFeature* info = (infoForProximityFeature*) infoForFeature;
  expectedDistTable *distTab = info->distTab;
  featureStruct expectedFeature;

  float planeAngle = rotationOfPlane( position.rot, info->grid);

  position.rot = distTabPlaneOfRotation( planeAngle
					 + info->sensorRot,
					 distTab);

  if ( inDistTab( position, distTab)) {
    expectedFeature.integrateNumber = distTab->dist[position.x][position.y][position.rot];
    expectedFeature.feature = expectedFeature.integrateNumber / distTab->numberOfStdDevs;
    expectedFeature.quality = expectedFeature.integrateNumber % distTab->numberOfStdDevs;
    expectedFeature.numberOfQualities = distTab->numberOfStdDevs;
  }
  else {
    expectedFeature.integrateNumber = 0;
    expectedFeature.feature = 0;
    expectedFeature.quality = 0;
    expectedFeature.numberOfQualities = distTab->numberOfStdDevs;
  }

  return expectedFeature;
}


/* Returns the distance to the next obstacle in the map for a given
 * position and orientation. */
/* To extract the expected distance to an obstacle in a given map. */



int
signum(float x)
{
  int sig;

  if (x > 0)
    sig = 1;
  else if (x < 0)
    sig = -1;
  else
    sig = 0;
  return sig;
}


int
nextField(float x, float deltaX)
{
  if (deltaX > 0)
    return (int) round(x + 0.5);
  else if (deltaX == 0.0)
    return (int) round( x - 0.5);
  else
    return (int) (--x);
}



static distance
obstacleDistanceRealStep( probabilityGrid m, realPosition realPos,
			  distance maxRange,
			  probability threshold)
{
  register float deltaX = cos( realPos.rot);
  register float deltaY = sin( realPos.rot);
  
  float occProb;
  
  /* add something to get maxRange in every case */
  float maxDistance = maxRange / m.resolution + 3;
  
  float startX = floatMapCoordinateOfRealCoordinate( realPos.x,
						     m.offsetX,
						     m.resolution);
  
  float startY = floatMapCoordinateOfRealCoordinate( realPos.y,
						     m.offsetY,
						     m.resolution);
  
  int nextX, nextY;
  float stepsNextX, stepsNextY;
  
  float x = startX;
  float y = startY;
  
  float sum = 0.0;
  float dist = 0.0;
  
  while (dist <= maxDistance && sum < threshold){
    occProb = occupancyProbabilityMapUnknown((int) x, (int) y, m);
    if (occProb == m.unknown)
      return maxRange;
    else 
      sum = occProb;
    
    /* the threshold is used to avoid roundoff errors */
    if (fabs(fabs(deltaX) - fabs(deltaY)) >= 1e-4){
      nextX = nextField(x, deltaX);
      nextY = nextField(y, deltaY);
      
      if (deltaX == 0.0)
	stepsNextX = MAXFLOAT;
      else
	stepsNextX = (nextX - x) / deltaX;
      
      if (deltaY == 0.0)
	stepsNextY = MAXFLOAT;
      else
	stepsNextY = (nextY - y) / deltaY;
      
      if (stepsNextX < stepsNextY){
	x = nextX;
	y += stepsNextX * deltaY;
      }
      else{
	y = nextY;
	x += stepsNextY * deltaX;
      }
    }
    else{
      if (deltaX > 0)
	x++;
      else if (deltaX < 0)
	x--;
      if (deltaY > 0)
	y++;
      else if (deltaY < 0)
	y--;
    }
    
    dist = sqrt( fSqr(x - startX) + fSqr( y - startY));
  }
  
  return fMax(0.0, fMin(maxRange, (dist - 1) * m.resolution));
}




#define NUMBER_OF_OFFSET_ANGLES 720
#define ANGLE_TO_INDEX 2

int
expectedFeatureGridMap( probabilityGrid* map,
			int centerX, int centerY, int rot,
			distance distanceResolution,
			int maxFeature)
{
  static int firstTime = TRUE;
  int angle = rot * ANGLE_TO_INDEX;
  int step;

  /* We precompute most of the indices so that searching for the next occupied cell
   * in the beam direction will be done by table lookups. */
  
  /* Offset in the real map used to get the i-th index along the z-th beam. */
  static int** xOffset = NULL;
  static int** yOffset = NULL;

  /* Number of different offsets along the z-th beam. */
  static int* numberOfOffsets = NULL;

  /* Feature to the i-th index along the z-th beam. */
  static int** offsetFeature = NULL;

  /* Precompute the lookup tables. */
  if ( firstTime) {

    int z;
    float angleStep = DEG_360 / NUMBER_OF_OFFSET_ANGLES;
    int maxNumberOfDifferentSteps = 2 * maxFeature * distanceResolution / map->resolution + 1;
    firstTime = FALSE;

#define VERBOSE 0

    if (VERBOSE) fprintf(stderr, "%f %f #dist/map-resolution\n", (float) distanceResolution, (float) map->resolution );
    xOffset = (int**) allocate2D( NUMBER_OF_OFFSET_ANGLES,
				  maxNumberOfDifferentSteps, INT);
    yOffset = (int**) allocate2D( NUMBER_OF_OFFSET_ANGLES,
				  maxNumberOfDifferentSteps, INT);
    numberOfOffsets = (int*) allocate1D( NUMBER_OF_OFFSET_ANGLES, INT);
    offsetFeature = (int**) allocate2D( NUMBER_OF_OFFSET_ANGLES,
					maxNumberOfDifferentSteps, INT);
    
    /* For each precompute the distances along the beam. We only increment the step
     * if a new grid cell in the map is reached. Thus, for each angle and step we
     * also preprocess the resulting distance. */
    for ( z = 0; z < NUMBER_OF_OFFSET_ANGLES; z++) {
      double angle = z * angleStep;
      double deltaX = cos(angle);
      double deltaY = sin(angle);
      double x , y;
      int nextX, nextY;
      double stepsNextX, stepsNextY;

      int n, feature;

      if (VERBOSE) fprintf(stderr, "\n%d %f #ange\n", z, (float) angle);

      n= 0;
      x = y = START_OFFSET;
      feature = 0;

      if (VERBOSE) fprintf(stderr, "%d #maxNumberOfDifferentSteps\n%d #maxFeature\n", maxNumberOfDifferentSteps, maxFeature);
      
      while (n < maxNumberOfDifferentSteps && feature < maxFeature){
	/* Store the indices in the map if going along the beam. */
	if (VERBOSE) fprintf(stderr, "%d %d %d %d\n", n, (int) x, (int) y, feature);
	xOffset[z][n] = x;
	yOffset[z][n] = y;
	offsetFeature[z][n] = feature;
	n++;

	/* compute the next values for x and y */
	if (fabs(fabs(deltaX) - fabs(deltaY)) >= 1e-4){
	  nextX = nextField(x, deltaX);
	  nextY = nextField(y, deltaY);
	  
	  if (deltaX == 0.0)
	    stepsNextX = MAXFLOAT;
	  else
	    stepsNextX = (nextX - x) / deltaX;
	  
	  if (deltaY == 0.0)
	  stepsNextY = MAXFLOAT;
	  else
	    stepsNextY = (nextY - y) / deltaY;
	
	  if (stepsNextX < stepsNextY){
	    x = nextX;
	  y += stepsNextX * deltaY;
	  }
	  else{
	    y = nextY;
	    x += stepsNextY * deltaX;
	  }
	}
	else{
	  if (deltaX > 0)
	    x++;
	  else if (deltaX < 0)
	    x--;
	  if (deltaY > 0)
	    y++;
	  else if (deltaY < 0)
	    y--;
	}

      /* compute the feature number of the next field */

      feature = round(sqrt(fSqr(x - START_OFFSET) + fSqr(y - START_OFFSET)) / distanceResolution * map->resolution);

      }
      numberOfOffsets[z] = n;
      if (VERBOSE) fprintf(stderr, "%f %d #steps\n", rad2Deg(angle), numberOfOffsets[z]);
    }
  }
  
#define OCCUPIED_THRESHOLD 0.8
  
  /* Now step through the offsets and search for the first occupied cell. */
  for ( step = 0; step < numberOfOffsets[angle]; step++) {

    int mapX = xOffset[angle][step] + centerX;
    int mapY = yOffset[angle][step] + centerY;
    
    if ( mapX >= 0 && mapX < map->sizeX && mapY >= 0 && mapY < map->sizeY) {
      if ( map->prob[mapX][mapY] > OCCUPIED_THRESHOLD)
	return offsetFeature[angle][step];
    }
    else
      return maxFeature - 1;
  }
  return maxFeature - 1;
}



distance
obstacleDistanceReal( probabilityGrid m, realPosition realPos,
		      distance maxRange)
{
    return obstacleDistanceRealStep( m, realPos, maxRange,
				     OBSTACLE_THRESHOLD);
}



distance
expectedDistanceGridMap( probabilityGrid* map,
			 int centerX, int centerY, int rot,
			 distance distanceResolution,
			 int maxFeature)
{
  int feature = expectedFeatureGridMap( map, centerX, centerY, rot,
					distanceResolution, maxFeature);

  
  return feature * distanceResolution;
}




/*****************************************************************************
 * Reads the distance table from <fileName>.
 *****************************************************************************/
expectedDistTable
readDistTab( char* fileName, sensorParameters params, int onlineMapping)
{
  FILE* file;
  char extendedFileName[80];
  int x = 0, y;
  expectedDistance dummy;
  expectedDistTable tab;
  char *usedFileName = extendedFileName;
  
  sprintf(extendedFileName, "%s.%d.%d", fileName, params.numberOfStdDevs,
	   (int) round(params.sensorMaxRange));
  
  if ( (file = fopen( extendedFileName, "r")) == NULL) {
    if ( ! onlineMapping)
      fprintf( stderr, "# Error: cannot open %s.\n", extendedFileName);
    if ( (file = fopen( fileName, "r")) == NULL) {
      if ( ! onlineMapping)
	fprintf( stderr, "# Error: cannot open %s.\n", extendedFileName);
      tab.dist = NULL;
      return tab;
    }
    else
      usedFileName = fileName;
  }
  
  fscanf( file, "%d %d %d", &(tab.sizeX), &(tab.sizeY), &(tab.sizeZ));
  
  if ( ! onlineMapping) {
    
    writeLog( "# Reading expected dists from %s (size is %d X %d X %d) ... ",
	      usedFileName, tab.sizeX, tab.sizeY, tab.sizeZ);
    fprintf( stderr,
	     "# Reading %s (size is %d X %d X %d) (0%%) ... ",
	     usedFileName, tab.sizeX, tab.sizeY, tab.sizeZ);
    
    if (fscanf( file, "%f %f %d %d%c",
		&(tab.angleResolution),
		&(tab.distanceResolution),
		&(tab.numberOfExpectedDistances),
		&(tab.numberOfStdDevs),
		&dummy) != 5){
      fprintf(stderr, "# Error reading %s\n", extendedFileName);
      closeLogAndExit( -1);
    }
    tab.sensorMaxRange = params.sensorMaxRange;
    
    /* Allocate memory and get the data. */
    tab.dist = (expectedDistance***)
      allocate3D( tab.sizeX, tab.sizeY, tab.sizeZ, EXPECTED_DISTANCE);
    
    for ( x = 0; x < tab.sizeX; x++){
      for ( y = 0; y < tab.sizeY; y++)
	fread( tab.dist[x][y], 1,  tab.sizeZ, file);
      fprintf( stderr,
	       "\r# Reading %s (size is %d X %d X %d) (%2.0f%%) ... ",
	       usedFileName, tab.sizeX, tab.sizeY, tab.sizeZ,
	       100.0 * ((float) x)/tab.sizeX);
    }

    writeLog( "done\n");
    fprintf( stderr, "\r# Reading %s (size is %d X %d X %d) (100%%) ... done\n",
	   usedFileName, tab.sizeX, tab.sizeY, tab.sizeZ);
    
  }
  /* Just extract some information from the file. */
  else {
    
    writeLog( "# Online mapping: (size is %d X %d X %d) ... ",
	      tab.sizeX, tab.sizeY, tab.sizeZ);
    fprintf( stderr, "# Online mapping: (size is %d X %d X %d) ... ",
	      tab.sizeX, tab.sizeY, tab.sizeZ);
    
    if (fscanf( file, "%f %f %d %d%c",
		&(tab.angleResolution),
		&(tab.distanceResolution),
		&(tab.numberOfExpectedDistances),
		&(tab.numberOfStdDevs),
		&dummy) != 5){
      fprintf(stderr, "# Error reading %s\n", usedFileName);
      closeLogAndExit( -1);
    }
    
    tab.dist = NULL;
  }
  
  fclose( file);
    
  return tab;
}


/*****************************************************************************
 * Writes the values in bytes into <fileName>.
 *****************************************************************************/
void
dumpDistTab( expectedDistTable* tab, char* fileName)
{
  FILE* file;
  int x, y, z;

  char extendedFileName[80];

  sprintf(extendedFileName, "%s.%d.%d", fileName, tab->numberOfStdDevs,
	  (int)
	  round(tab->sensorMaxRange));

  if ( (file = fopen( extendedFileName, "w")) == NULL) {
    fprintf( stderr, "Error: cannot open %s.\n", extendedFileName);
    return;
  }

  fprintf( stderr, "# Dumping expected distance table into %s (%d K) ...",
	   extendedFileName, (tab->sizeX * tab->sizeY * tab->sizeZ) / 1024);

  fprintf( file, "%d %d %d ", tab->sizeX, tab->sizeY, tab->sizeZ);
  fprintf( file, "%f %f %d %d\n",
	  tab->angleResolution,
	  tab->distanceResolution,
	  tab->numberOfExpectedDistances,
	  tab->numberOfStdDevs);

  for ( x = 0; x < tab->sizeX; x++)
    for ( y = 0; y < tab->sizeY; y++)
      for ( z = 0; z < tab->sizeZ; z++)
	fputc( (unsigned char) tab->dist[x][y][z], file);

  fclose( file);

  fprintf( stderr, "done\n");

}

#define USE_LOOKUP_TABLE_FOR_DISTANCES 
#define NO_LIB_GET_DISTANCE 1

void
computeDistTabValuesThirdDim( int x, int y, probabilityGrid* map,
			      simulatorMap* simMap,
			      bool useSimulatorMap,
			      sensorParameters params,
			      expectedDistTable *tab)
{
  int z, count, angle, angleOffset, endCount, step;
  distance expDist[ANGLE_RESOLUTION];
  distance averageDistance[ANGLE_RESOLUTION];
  distance distanceStdDev[ANGLE_RESOLUTION];

  step = ANGLE_RESOLUTION / tab->sizeZ;

  if (tab->sizeZ * step != ANGLE_RESOLUTION){
    char *error = "# Error: 360 is not a multiple of the number of angles in the expeced distance table\n";
    fprintf(stderr, error);
    writeLog(error);
    closeLogAndExit(1);
  }

  if (useSimulatorMap){
    float realPosX = realCoordinateOfMapCoordinate(x, map->offsetX,
						   map->resolution);
    float realPosY = realCoordinateOfMapCoordinate(y, map->offsetY,
						   map->resolution);
    for (z = 0; z < ANGLE_RESOLUTION; z++){
      expDist[z] = simulatorObjectDistance( simMap,
					    realPosX, realPosY,
					    params.sensorHeight,
					    cos(deg2Rad((float) z)),
					    sin(deg2Rad((float) z)),
					    params.sensorMaxRange);
      }
    }
  else
    for (z = 0; z < ANGLE_RESOLUTION; z++){
      
#ifndef USE_LOOKUP_TABLE_FOR_DISTANCES
      expDist[z] = obstacleDistance( map, x, y,
				     deg2Rad((float) z),
				     params.sensorMaxRange);
#else
      expDist[z] = expectedFeatureGridMap( map,
					   x, y, z,
					   1.0,
					   params.sensorMaxRange);
#endif
    }
  

  /* determine the average expected distances within angle resolution
     of the sensor */
  angleOffset = round(rad2Deg(params.openingAngle)) / 2;

  for (z = 0; z < ANGLE_RESOLUTION; z++){
    averageDistance[z] = 0.0;

    if (z < angleOffset)
      angle = ANGLE_RESOLUTION + z - angleOffset;
    else
      angle = z - angleOffset;

    endCount = z + angleOffset + 1;  /* symmetry */
    for (count = z - angleOffset; count < endCount; count++){
      if (angle >=ANGLE_RESOLUTION)
	angle = 0;
      averageDistance[z] += expDist[angle];
      angle++;
    }
    averageDistance[z] /= 2*angleOffset + 1;
  }

  /* now determine the stdDev for each distance within the angle resolution
     of the sensor */
  for (z = 0; z < ANGLE_RESOLUTION; z++){
    distanceStdDev[z] = 0.0;

    if (z < angleOffset)
      angle = ANGLE_RESOLUTION + z - angleOffset;
    else
      angle = z - angleOffset;

    endCount = z + angleOffset + 1;  /* symmetry */
    for (count = z - angleOffset; count < endCount; count++){
      if (angle >=ANGLE_RESOLUTION)
	angle = 0;
      distanceStdDev[z] += fSqr(expDist[angle]-
				averageDistance[z]);
      angle++;
    }
    distanceStdDev[z] = sqrt(distanceStdDev[z]);
  }


  /* In the case of three beams we search for edges and use the
   * minimal distance instead of average. */
  if ( angleOffset == 1) {

    for (z = 0; z < ANGLE_RESOLUTION; z++){

      if (stdDevIndex(distanceStdDev[z], params) > 0) {

	int middle = z;
	int left = z > 0 ? z -1 : ANGLE_RESOLUTION-1;
	int right = z < ANGLE_RESOLUTION-1 ? z + 1 : 0;

	float leftDeviation = fabs(expDist[middle] - expDist[left]);
	float rightDeviation = fabs(expDist[middle] - expDist[right]);

#define DEVIATION_THRESHOLD 50.0

	if ( fabs( leftDeviation - rightDeviation) > DEVIATION_THRESHOLD) {

	  float minVal = expDist[middle];
	  int min = middle;

	  if ( expDist[left] < minVal)
	    min = left;
	  if ( expDist[right] < minVal)
	    min = right;

	  if (0) fprintf(stderr, "%f %f %f ...", leftDeviation, rightDeviation, expDist[min]);
	  if ( leftDeviation > rightDeviation) {
	    if ( min == left)
	      averageDistance[z] = expDist[left];
	    else
	      averageDistance[z] = 0.5 * (expDist[middle] + expDist[right]);
	  }
	  else {
	    if ( min == right)
	      averageDistance[z] = expDist[right];
	      else
		averageDistance[z] = 0.5 * (expDist[middle] + expDist[left]);
	  }

	  if (0) fprintf( stderr, "%f %f %f  -> %f\n",
			  expDist[left], expDist[middle], expDist[right], averageDistance[z]);

	}
      }
    }
  }

  /*  now set the tab values */
  for (z = 0; z < tab->sizeZ; z++){
    int distIndex = distanceIndex( averageDistance[z], tab->distanceResolution,
				   tab->numberOfExpectedDistances);
    int tabValue = distIndex * tab->numberOfStdDevs
      + stdDevIndex(distanceStdDev[z], params);
    tab->dist[x][y][z] = (expectedDistance) tabValue;
  }
}



static bool
inDistTab( gridPosition gridPos, expectedDistTable *distTab)
{
  return ( gridPos.x >= 0 && gridPos.x < distTab->sizeX
	   && gridPos.y >= 0 && gridPos.y < distTab->sizeY
	   && gridPos.rot >= 0 && gridPos.rot < distTab->sizeZ);
}





/* returns the index of the expected distance in the expected distance table */
int
expectedDistIndex(gridPosition gridPos, expectedDistTable *distTab)
{
  int index;

  if ( inDistTab( gridPos, distTab))
    index = (int) (distTab->dist[gridPos.x][gridPos.y][gridPos.rot]
		    / distTab->numberOfStdDevs);
  else
    index = 0;

  return index;
}


/* returns the expected distance in the expected distance table */
distance
expectedDist(gridPosition gridPos, expectedDistTable *distTab)
{
  return expectedDistIndex( gridPos, distTab) * distTab->distanceResolution;
}


int
numberOfExpectedStdDev(gridPosition gridPos, expectedDistTable *distTab)
{
  int stdDev;

  if ( inDistTab( gridPos, distTab))
    stdDev = distTab->dist[gridPos.x][gridPos.y][gridPos.rot]
      % distTab->numberOfStdDevs;
  else
    stdDev = 0;

  return stdDev;
}



int
distanceIndex( float distance, float distanceResolution, int numberOfDistances)
{
  int index = (int) round(distance / distanceResolution);

  if (index >= numberOfDistances)
    index = numberOfDistances - 1;
  else if (index < 0)
    index = 0;

  return index;
}



static int
stdDevIndex(float stdDev, sensorParameters params)
{
  int i, index = 0;
  bool stop = FALSE;

  for (i=0; i < params.numberOfStdDevs && !stop; i++){
    if ((stop = (stdDev < params.stdDevThreshold[i])))
      index = i;
  }
  return index;
}




/*****************************************************************************
 * Computes the distance table.

 *****************************************************************************/
expectedDistTable
preprocessedExpectedDistances( probabilityGrid* map,
			       simulatorMap* simMap,
			       sensorParameters params,
			       int numberOfOrientations,
			       char* file,
			       bool onlineMapping)
{
  expectedDistTable tab;
  
  if ( ! onlineMapping) {
    
    int x, y;    
    bool useSimulatorMap = FALSE;
    
    setTimer(0);
    if (rad2Deg(params.openingAngle) < 1){
      char *error =
	"# Error: angle resolution of sensor less than 1 deg, fix that first\n";
      fprintf(stderr, error);
      writeLog(error);
      closeLogAndExit(1);
    }

    useSimulatorMap = simMap->initialized;
    
    tab.sizeX = map->sizeX;
    tab.sizeY = map->sizeY;
    tab.sizeZ = numberOfOrientations;
    
    /* Allocate memory. */
    tab.dist = (expectedDistance***)
      allocate3D( tab.sizeX, tab.sizeY, tab.sizeZ, EXPECTED_DISTANCE);
    
    if (useSimulatorMap)
      writeLog( "# Computing expected distances from simulator map ... \n");
    else
      writeLog( "# Computing expected distances from grid map ... \n");
    
    tab.angleResolution = DEG_360 / numberOfOrientations;
    tab.distanceResolution = params.sensorMaxRange /
      (params.numberOfExpectedDistances - 1);
    tab.numberOfExpectedDistances = params.numberOfExpectedDistances;
    tab.numberOfStdDevs = params.numberOfStdDevs;
    tab.sensorMaxRange = params.sensorMaxRange;
    
    /* Fill in the expected distances. */
    for ( x = 0; x < map->sizeX; x++) {
      if ( x % 5 == 0)
	fprintf( stderr, "# Computing expected distances %.0f %%\r",
		 100.0 * (float) x / (float) map->sizeX);
      for ( y = 0; y < map->sizeY; y++) {
	computeDistTabValuesThirdDim(x, y, map, simMap,
				     useSimulatorMap, params, &tab);
      }
    }
    fprintf( stderr, "# Computed expected distances in %f secs.\n", timeExpired(0));
    if ( file != NULL) {
      writeLog( " done in %f secs.\nDump into %s.\n", timeExpired(0), file);
      dumpDistTab( &tab, file);
    }
  }

  /* For online mapping we don't precompute the expected distances.
   * But some default values need still to be set. */
  else {

    tab.sizeX = map->sizeX;
    tab.sizeY = map->sizeY;
    tab.sizeZ = numberOfOrientations;
    
    tab.dist = NULL;
  
    writeLog( "# Don't compute expected distances.\n");
    fprintf( stderr, "# Set values of parameters.\n");

    tab.angleResolution = DEG_360 / numberOfOrientations;
    tab.distanceResolution = params.sensorMaxRange /
      (params.numberOfExpectedDistances - 1);
    tab.numberOfExpectedDistances = params.numberOfExpectedDistances;
    tab.numberOfStdDevs = params.numberOfStdDevs;
    tab.sensorMaxRange = params.sensorMaxRange;
  }
  
  return tab;
}


/**************************************************************************
 * Computes the expected distance of a single linear beam
 **************************************************************************/

distance
obstacleDistance( probabilityGrid* map, int x, int y,
		  float rot, distance maxRange)
{
  realPosition pos;
  pos.x = realCoordinateOfMapCoordinate(x, map->offsetX, map->resolution);
  pos.y = realCoordinateOfMapCoordinate(y, map->offsetY, map->resolution);
  pos.rot = rot;
  return obstacleDistanceReal( *map, pos, maxRange);
}



void
averageProbabilities( distProbTable* table, float* probs)
{
  int stdDevCnt, measuredCnt, expectedCnt, expectedIndex;

  for (stdDevCnt = 0; stdDevCnt < table->numberOfStdDevs; stdDevCnt++) {
    probs[stdDevCnt] = 0.0;
    for (measuredCnt = 0;
	 measuredCnt < table->numberOfMeasuredDistances; measuredCnt++) {
      for (expectedCnt = 0;
	   expectedCnt < table->numberOfExpectedDistances; expectedCnt++){
	expectedIndex = expectedCnt*table->numberOfStdDevs + stdDevCnt;
	probs[stdDevCnt] += table->prob[expectedIndex][measuredCnt];
      }
    }
    probs[stdDevCnt] /=
      table->numberOfMeasuredDistances * table->numberOfExpectedDistances;
  }
}

/*****************************************************************************
 * reads the distance probability function tables
 *****************************************************************************/
distProbTable
readProbFunctionTable( char *fileName,
		       int numberOfStdDevs,
		       expectedDistTable expDist)
{
  int measuredCnt, expectedCnt, expectedIndex, stdDevIndex;
  char line[MAX_STRING_LENGTH], extendedFileName[MAX_STRING_LENGTH];
  distProbTable table;
  bool error = FALSE;
  float tmp;

  FILE *fp;

  table.distanceResolution = expDist.distanceResolution;
  table.numberOfExpectedDistances =
    table.numberOfMeasuredDistances = expDist.numberOfExpectedDistances;
  table.numberOfStdDevs = expDist.numberOfStdDevs;

  /***********************************************************************
   * Allocate memory.
   ***********************************************************************/
  table.prob = (probability**)
    allocate2D( table.numberOfExpectedDistances * table.numberOfStdDevs,
		table.numberOfMeasuredDistances,
		PROBABILITY);

  /* open files */
  for (stdDevIndex = 0; stdDevIndex < numberOfStdDevs; stdDevIndex++){
    sprintf(extendedFileName, "%s-%d.%d.%d", fileName, stdDevIndex,
	    numberOfStdDevs, (int) round(expDist.sensorMaxRange));
    if ((fp = fopen(extendedFileName, "r")) == NULL){
      sprintf(line, "%s%s", PARAMPATH, extendedFileName);
      if ((fp = fopen(line, "r")) == NULL){
	fprintf(stderr, "# Could not find: %s\n", extendedFileName);
	sprintf(extendedFileName, "%s-%d", fileName, stdDevIndex);
	if ((fp = fopen(extendedFileName, "r")) == NULL){
	  sprintf(line, "%s%s", PARAMPATH, extendedFileName);
	  if ((fp = fopen(line, "r")) == NULL){
	    fprintf(stderr, "# Could not find: %s\n", extendedFileName);
	    free2D( (void**) table.prob,
		    table.numberOfExpectedDistances * table.numberOfStdDevs,
		    PROBABILITY);
	    table.prob = NULL;
	    return table;
	  }
	}
      }
    }

    writeLog( "# Reading %s ...", extendedFileName);
    fprintf(stderr, "# Reading %s\n", extendedFileName);
    /* read the first comment line */
    error = (fgets(line,MAX_STRING_LENGTH,fp) == NULL);
    if (!error && sscanf(&line[1], "%d %d", &expectedCnt, &measuredCnt) == 2){
      if (expectedCnt != table.numberOfExpectedDistances ||
	  measuredCnt != table.numberOfMeasuredDistances){
	fprintf(stderr, "# Wrong dimension (table %d %d) (expected file %d %d) in  %s\n",
		table.numberOfExpectedDistances, table.numberOfMeasuredDistances,
		expectedCnt, measuredCnt, extendedFileName);
	fclose(fp);
	free2D( (void**) table.prob,
		table.numberOfExpectedDistances * table.numberOfStdDevs,
		PROBABILITY);
	table.prob = NULL;
	return table;
      }
    }
    else {
      fprintf(stderr, "# Could not read %s\n", extendedFileName);
      fclose(fp);
      free2D( (void**) table.prob,
	      table.numberOfExpectedDistances * table.numberOfStdDevs,
	      PROBABILITY);
	table.prob = NULL;
	return table;
    }

    /***********************************************************************
     * Read table.
     ***********************************************************************/
    for ( measuredCnt = 0;
	  measuredCnt < table.numberOfMeasuredDistances; measuredCnt++){
      for ( expectedCnt = 0;
	    expectedCnt < table.numberOfExpectedDistances; expectedCnt++) {
	expectedIndex = expectedCnt*table.numberOfStdDevs + stdDevIndex;
	if (fgets(line,MAX_STRING_LENGTH,fp) != NULL &&
	    sscanf(line, "%e", &tmp) == 1) {
	  table.prob[expectedIndex][measuredCnt] = tmp;
	}
	else{
	  fprintf(stderr, "Error: reading %s\n", extendedFileName);
	  free2D( (void**) table.prob,
		  table.numberOfExpectedDistances * table.numberOfStdDevs,
		  PROBABILITY);
	  table.prob = NULL;
	  fclose(fp);
	  return table;
	}
      }
      if (measuredCnt < table.numberOfMeasuredDistances - 1)
	fgets(line,MAX_STRING_LENGTH,fp);
    }

    fclose(fp);
    writeLog(" done\n");
  }

  return table;
}


#define NORMED_STARTSTRING "normed"
#define INTEGRATE_STARTSTRING "dist"
void
dumpDistTable( distProbTable table, char* startString) {
  char fileName[80];
  static int normedCount = 0;
  static int distCount = 0;
  FILE *fp;
  int stdDevCnt, measuredCnt, expectedCnt, expectedIndex;

  for (stdDevCnt = 0; stdDevCnt < table.numberOfStdDevs; stdDevCnt++) {
     if ( startString == NORMED_STARTSTRING)
	sprintf(fileName, "%s%d-%d.gnu", startString, normedCount, stdDevCnt);
     else
	sprintf(fileName, "%s%d-%d.gnu", startString, distCount, stdDevCnt);
     if ((fp = fopen(fileName, "wt")) != NULL){
	fprintf(fp, "# %d %d\n", table.numberOfExpectedDistances,
		table.numberOfMeasuredDistances);
	for (measuredCnt = 0;
	     measuredCnt < table.numberOfMeasuredDistances; measuredCnt++) {
	  for (expectedCnt = 0;
	       expectedCnt < table.numberOfExpectedDistances; expectedCnt++){
	    expectedIndex = expectedCnt*table.numberOfStdDevs + stdDevCnt;
	    fprintf(fp, "%.3e\n", table.prob[expectedIndex][measuredCnt]);
	  }
	  fprintf(fp,"\n");
	}
	fclose(fp);
     }
  }
  if ( startString == NORMED_STARTSTRING)
     normedCount++;
  else
     distCount++;
}


/*****************************************************************************
 * Computes the probabilities of sensor measurements for given expected
 * measurements.
 *****************************************************************************/
distProbTable
normedProbFunctionTable( sensorParameters params,
			 expectedDistTable expDist)
{
   distProbTable table;

   table.prob = NULL;

   /* check whether table can be read from a file */
   table = readProbFunctionTable( params.probFunctionFileName,
				  expDist.numberOfStdDevs,
				  expDist);
   if (table.prob != NULL){
     plotTable( table, "norm");
/* #define DUMP_DIST_TABLE */
#ifdef DUMP_DIST_TABLE
     dumpDistTable( table, NORMED_STARTSTRING);
#endif
     return table;
   }
   else {
     fprintf( stderr, "Error when reading dist table (you need the directory params).\n");
     fprintf( stderr, "You need to checkout bee/data/localize/params,\n");
     fprintf( stderr, "if you start LOCALIZE in bee/data/localize/floor,\n");
     exit(0);
   }
}


/*****************************************************************************
 * Computes the probabilities of sensor measurements for given expected
 * measurements.
 *****************************************************************************/
distProbTable
probFunctionTable( sensorParameters params,
		   expectedDistTable expDist)
{
  distProbTable table;

  table.prob = NULL;

  /* check whether table can be read from a file */
  table = readProbFunctionTable( params.probFunctionFileName,
				 expDist.numberOfStdDevs,
				 expDist);

  if (table.prob != NULL){
    if ( params.maxFactor > 1.0) {
      setMaxFactor( &table, params.maxFactor);
      plotTable( table, "maxFactor");
    }
    else {
      convolveWithAverage( &table, params.maxFactor);
      plotTable( table, "convolved");
    }

#ifdef DUMP_DIST_TABLE
    dumpDistTable( table, INTEGRATE_STARTSTRING);
#endif
    return table;
  }
  else {
    fprintf( stderr, "Error when reading dist table.\n");
    exit(0);
  }
}



/* Convolve the prob function with an uninformed function given by average. */
static void
convolveWithAverage( distProbTable* table,
		     float tableWeight)
{

	
  int stdDevCnt, expectedCnt, measuredCnt, expectedIndex;

  writeLog( "# Convolve probability table with weight %f.\n", tableWeight);
  
  for (stdDevCnt = 0; stdDevCnt < table->numberOfStdDevs; stdDevCnt++) {

    probability pOfMaxRange=0.0, pOfNotMaxRange=0.0;
    
    for ( expectedCnt = 0;
	  expectedCnt < table->numberOfExpectedDistances; expectedCnt++){

      probability avg = 0.0; 
      int expectedIndex = expectedCnt*table->numberOfStdDevs + stdDevCnt;
      
      for ( measuredCnt = 0;
	    measuredCnt < table->numberOfMeasuredDistances; measuredCnt++) {
	
	avg += table->prob[expectedIndex][measuredCnt];
      }
      avg /= table->numberOfMeasuredDistances;
      
      for ( measuredCnt = 0;
	    measuredCnt < table->numberOfMeasuredDistances; measuredCnt++) {	

	/* Convolve with average. */
	table->prob[expectedIndex][measuredCnt] =
	  tableWeight * table->prob[expectedIndex][measuredCnt]
	  + (1.0 - tableWeight) * avg;

      	/* Multiply with average to get values around one in order to
	 * avoid problems with floating resolution. */
	table->prob[expectedIndex][measuredCnt] /= avg;
      }

      if (0) {
	probability minProb = 1e10, maxProb = 0.0, tmp;
	
	for ( measuredCnt = 0;
	      measuredCnt < table->numberOfMeasuredDistances - 1; measuredCnt++) {
	  tmp = table->prob[expectedIndex][measuredCnt];

	  if ( tmp < minProb)
	    minProb = tmp;
	  if ( tmp > maxProb)
	    maxProb = tmp;
	}
      }
    }
  }
}


static void
setMaxFactor( distProbTable* table,
	      float maxFactor)
{
  probability minProb = 1e10, maxProb = 0.0;
  /*  probability probFactor; */
  probability tmp;
  int stdDevCnt, expectedCnt, measuredCnt, expectedIndex;

  for (stdDevCnt = 0; stdDevCnt < table->numberOfStdDevs; stdDevCnt++) {

    probability sum = 0.0;
    probability pOfMaxRange=0.0, pOfNotMaxRange=0.0;

    /* Compute probability of measuring no max range. */
    for ( measuredCnt = 0;
	  measuredCnt < table->numberOfMeasuredDistances - 1; measuredCnt++) {

      for ( expectedCnt = 0;
	    expectedCnt < table->numberOfExpectedDistances; expectedCnt++){
	expectedIndex = expectedCnt*table->numberOfStdDevs + stdDevCnt;

	pOfNotMaxRange += table->prob[expectedIndex][measuredCnt];
      }
    }
    pOfNotMaxRange /= table->numberOfExpectedDistances *
      (table->numberOfMeasuredDistances-1);

    /* Compute probability of measuring max range. */
    measuredCnt = table->numberOfMeasuredDistances - 1;
    for ( expectedCnt = 0;
	  expectedCnt < table->numberOfExpectedDistances; expectedCnt++){
      expectedIndex = expectedCnt*table->numberOfStdDevs + stdDevCnt;

      pOfMaxRange += table->prob[expectedIndex][measuredCnt];
    }
    pOfMaxRange /= table->numberOfExpectedDistances;

    for ( expectedCnt = 0;
	  expectedCnt < table->numberOfExpectedDistances; expectedCnt++){
      expectedIndex = expectedCnt*table->numberOfStdDevs + stdDevCnt;

      for ( measuredCnt = 0;
	    measuredCnt < table->numberOfMeasuredDistances; measuredCnt++) {
	if ( measuredCnt != table->numberOfMeasuredDistances - 1)
	  table->prob[expectedIndex][measuredCnt] /= pOfNotMaxRange;
	else
	  table->prob[expectedIndex][measuredCnt] /= pOfMaxRange;
	tmp = table->prob[expectedIndex][measuredCnt];
	sum += tmp;
	if ( tmp < minProb)
	  minProb = tmp;
	if ( tmp > maxProb)
	  maxProb = tmp;
      }
    }
    
    
    if (1) {
      /*      probability minFactor = 1.0 / minProb; */
      
      probability newMaxProb = maxFactor;
      
      probability expo = log( 1.0 / maxProb) / log( minProb);
      probability newMinProb = minProb * pow( maxProb / newMaxProb, 1.0 / expo);
      
      writeLog( "# Set max factor (%f) min: %f (%f) max: %f --> %f %f\n",
		maxFactor, minProb, 1.0 / minProb, maxProb, newMinProb, newMaxProb);
      
      /* Now crunch the values such that the factor between the minimal probability
       * and the maximal probability becomes MAX_FACTOR. */
      for ( expectedCnt = 0;
	    expectedCnt < table->numberOfExpectedDistances; expectedCnt++){
	expectedIndex = expectedCnt*table->numberOfStdDevs + stdDevCnt;
	for ( measuredCnt = 0;
	      measuredCnt < table->numberOfMeasuredDistances; measuredCnt++) {
	  tmp = table->prob[expectedIndex][measuredCnt];
	  if ( tmp < 1.0)
	    table->prob[expectedIndex][measuredCnt] = fNorm( tmp, minProb, 1.0, newMinProb, 1.0);
	  else
	    table->prob[expectedIndex][measuredCnt] = fNorm( tmp, 1.0, maxProb, 1.0, newMaxProb);
	  
	}
      }
    }
  }

/* #define SET_SHORT_TO_ONE 1  */
#ifdef SET_SHORT_TO_ONE
  {
    /* set Factor 1 for too short values */
    for (stdDevCnt = 0; stdDevCnt < table->numberOfStdDevs; stdDevCnt++) {
      for ( expectedCnt = 0;
	    expectedCnt < table->numberOfExpectedDistances; expectedCnt++){
	expectedIndex = expectedCnt*table->numberOfStdDevs + stdDevCnt;
	for ( measuredCnt = 0;
	      measuredCnt < expectedCnt ; measuredCnt++) {
	  table->prob[expectedIndex][measuredCnt] =
	    fMax(table->prob[expectedIndex][measuredCnt], 1.0);
	}
      }
    }
    dumpDistTable(*table, "xxxx");
  }
#endif
  
#ifdef SET_MAXRANGE_TO_NORMAL

  for (stdDevCnt = 0; stdDevCnt < table->numberOfStdDevs; stdDevCnt++) {
    for ( expectedCnt = 0;
	  expectedCnt < table->numberOfExpectedDistances; expectedCnt++){
      expectedIndex = expectedCnt*table->numberOfStdDevs + stdDevCnt;
      measuredCnt = table->numberOfMeasuredDistances-1;
      table->prob[expectedIndex][measuredCnt] =
	table->prob[expectedIndex][measuredCnt-1];
    }
  }
  
  dumpDistTable(*table, "yyyy");
#endif


}



/***********************************************************************
 * Precomputes for each plane and each reading the number of the
 * corresponding plane in the expected distance table.
 ***********************************************************************/
int**
preprocessedPlaneInDistTable( abstractSensorVector* scan)
{
  static int** tab = NULL;
  static int numberOfPlanes=0;

  int plane, readCnt;
  float planeRot;

  /* In this case all sensors in the scan must be of the same type.
   * We use the information of the first sensor for the whole scan. */
  infoForProximityFeature* scanInfo =
    (infoForProximityFeature*) scan->sensor[0].infoForFeatures;

  positionProbabilityGrid* grid = scanInfo->grid;
  expectedDistTable* expectedDists = scanInfo->distTab;

  /* Check wether the needed size changed. */
  if ( scan->numberOfSensors > MAX_SIZE_OF_SCAN) {
    fprintf( stderr, "Too many sensors (%d) in scan. Resize MAX_SIZE_OF_SCAN.\n",
	     scan->numberOfSensors);
    closeLogAndExit( -1);
  }

  if ( numberOfPlanes < grid->sizeZ) {

    /* Free old memory. */
    if ( numberOfPlanes > 0) {
      for ( plane=0; plane < numberOfPlanes; plane++)
	free( tab[plane]);
      free( tab);
    }

    numberOfPlanes   = grid->sizeZ;

    /* Allocate new memory. */
    tab = (int**) allocate2D( numberOfPlanes, MAX_SIZE_OF_SCAN, INT);
  }

  for ( plane=0; plane < numberOfPlanes; plane++) {

    planeRot = rotationOfPlane( plane, grid);

    for ( readCnt=0; readCnt < scan->numberOfSensorsToBeUsed; readCnt++) {

      infoForProximityFeature* info =
	(infoForProximityFeature*) scan->sensor[scan->mask[readCnt]].infoForFeatures;

      tab[plane][readCnt] =
	distTabPlaneOfRotation( normalizedAngle( planeRot + info->sensorRot),
				expectedDists);
    }
  }
  return tab;
}


/**************************************************************************
 **************************************************************************
 * Functions for the probabilities of distance readings.
 **************************************************************************
 **************************************************************************/

/* Sets the mask to the readings with the highest / lowest evaluation. */
void
setMaskAccordingToEvaluations( int* mask, int sizeOfMask,
			       double* values, int numberOfValuesToBeSet,
			       int order)
{
  int sensor, num;
  static bool* alreadyInMask = NULL;
  static int sizeOfOwnMask = 0;

  if ( sizeOfOwnMask < sizeOfMask) {
    if ( alreadyInMask != NULL)
      free1D( alreadyInMask, BOOL);
    alreadyInMask = (bool*) allocate1D( sizeOfMask, BOOL);
    sizeOfOwnMask = sizeOfMask;
  }

  /* Initialize the check mask. */
  for ( num = 0; num < sizeOfMask; num++)
    alreadyInMask[num] = FALSE;

  /* Collect extreme values until enough. */
  for ( sensor = 0; sensor < numberOfValuesToBeSet; sensor++){

    /* Search the extreme value. If order ==  ASCENDING_ORDER we look for the
     * minimal value. */
    int extremeIndex = -1;
    double extremeValue = (order == DESCENDING_ORDER) ? -1.0e10 : 1.0e10;

    for (num = 0; num < sizeOfMask; num++){
      if ( ! alreadyInMask[num]) {
	bool isExtreme =
	  (order == DESCENDING_ORDER) ? values[num] > extremeValue : values[num] < extremeValue;

	if ( isExtreme) {
	  extremeValue = values[num];
	  extremeIndex = num;
	}
      }
    }

    /* Search this sensor in the remaining sensor mask and put
     * to the current position. */
    if ( extremeIndex >= 0) {
      alreadyInMask[extremeIndex] = TRUE;
      mask[sensor] = extremeIndex;
    }
  }
}

int
extractPartialScanMask( int* globalMask, int sizeOfGlobalMask,
			int* mask,
			int start, int end)
{
  int i, numberOfFound = 0;

  for ( i = 0; i < sizeOfGlobalMask; i++)
    if ( globalMask[i] >= start && globalMask[i] < end)
      mask[numberOfFound++] = globalMask[i] - start;

  return numberOfFound;
}




/* Dumps expected and measured distances at the estimated position. */
void
dumpProximityValues( actionInformation* info,
		     sensingActionMask* mask,
		     realPosition estimatedPos)
{
  int num, distIndex, stdDevIndex;

  if ( mask->use[LASER] && mask->perform[LASER][INTEGRATE_LASER]) {

    /* First front laser. */
    informationsFor_LASER* laserInfo =
      (informationsFor_LASER*) info->info[LASER];
    expectedDistTable* laserExpDist =
      &(laserInfo->general.expectedDistances);
    abstractSensorVector *scan = &(info->abstractSensors[ABSTRACT_FRONT_LASER]);

    realPosition scanCenter = endPoint( estimatedPos,
					scan->sensor[0].integrationInfo.sensorOffset);

    gridPosition gridPos = gridPositionOfRealPosition( scanCenter,
						       &(info->positionProbs));

    for ( num = 0; num < scan->numberOfSensors; num++) {

      float rot = estimatedPos.rot
	+ ((infoForProximityFeature*) scan->sensor[num].infoForFeatures)->sensorRot;


      gridPos.rot = distTabPlaneOfRotation( rot, laserExpDist);
      /* compute end point for expected distance */

      distIndex = expectedDistIndex( gridPos, laserExpDist);
      stdDevIndex = numberOfExpectedStdDev( gridPos, laserExpDist);

      writeLog("%f %d %d StdDev%dLasF\n", elapsedScriptTime, (int) distIndex,
	       scan->sensor[num].measuredFeature,
	       (int) stdDevIndex);
    }

    /* Rear laser. */
    scan = &(info->abstractSensors[ABSTRACT_REAR_LASER]);

    scanCenter = endPoint( estimatedPos,
			   scan->sensor[0].integrationInfo.sensorOffset);

    gridPos = gridPositionOfRealPosition( scanCenter,
					  &(info->positionProbs));

    for ( num = 0; num < scan->numberOfSensors; num++) {

      float rot = estimatedPos.rot
	+ ((infoForProximityFeature*) scan->sensor[num].infoForFeatures)->sensorRot;


      gridPos.rot = distTabPlaneOfRotation( rot, laserExpDist);
      /* compute end point for expected distance */

      distIndex = expectedDistIndex( gridPos, laserExpDist);
      stdDevIndex = numberOfExpectedStdDev( gridPos, laserExpDist);

      writeLog("%f %d %d StdDev%dLasR\n", elapsedScriptTime, (int) distIndex,
	       scan->sensor[num].measuredFeature,
	       (int) stdDevIndex);
    }
  }

  if ( mask->use[SONAR] && mask->perform[SONAR][INTEGRATE_SONAR]) {

    informationsFor_SONAR* sonarInfo =
      (informationsFor_SONAR*) info->info[SONAR];
    expectedDistTable* sonarExpDist =
      &(sonarInfo->general.expectedDistances);
    abstractSensorVector *sonarScan = &(info->abstractSensors[ABSTRACT_SONAR]);

    gridPosition gridPos = gridPositionOfRealPosition( estimatedPos,
						       &(info->positionProbs));

    for ( num = 0; num < sonarScan->numberOfSensors; num++) {

      float rot = estimatedPos.rot
	+ ((infoForProximityFeature*) sonarScan->sensor[num].infoForFeatures)->sensorRot;


      gridPos.rot = distTabPlaneOfRotation( rot, sonarExpDist);


      distIndex = expectedDistIndex( gridPos, sonarExpDist);
      stdDevIndex = numberOfExpectedStdDev( gridPos, sonarExpDist);

      writeLog("%f %d %d StdDev%dSon\n", elapsedScriptTime, (int) distIndex,
	       sonarScan->sensor[num].measuredFeature,
	       (int) stdDevIndex);
    }
  }
}


static void
dumpScanEndPoints( actionInformation* info,
		   abstractSensorVector* abstractScan,
		   distanceScan* rawScan,
		   realPosition robotPos,
		   probability averageProbOfReading,
		   char* measuredNoMax, char* measuredMax,
		   char* expectedNoMax, char* expectedMax)
{
  int num;
  movement beam = {0.0, 0.0};
  realPosition scannedPoint;
  char* filtered;
  bool filterMask[MAX_SIZE_OF_SCAN];
  
  float cmPerFeature = ((infoForProximityFeature*)
			abstractScan->sensor[0].infoForFeatures)->
    distProbFunctionTable->distanceResolution;
  
  realPosition scanCenter = endPoint( robotPos,
				      rawScan->offset);

  gridPosition realCenter = gridPositionOfRealPosition( scanCenter,
							&(info->positionProbs));


  /* Find all those sensor readings that are filtered out. */
  for ( num = 0; num < abstractScan->numberOfSensors; num++) 
    filterMask[num] = TRUE;

  for ( num = 0; num < abstractScan->numberOfSensorsToBeUsed; num++) 
    filterMask[abstractScan->mask[num]] = FALSE;
  
  
  for ( num = 0; num < abstractScan->numberOfSensors; num++) {

    featureStruct expFeature;
    realPosition startPosOfBeam = scanCenter;
    startPosOfBeam.rot = robotPos.rot + rawScan->reading[num].rot;

    if ( filterMask[num] == TRUE)
      filtered = "FILT";
    else
      filtered = "USED";
    
    /* Set the endpoint of the measured distance. */
    beam.forward = rawScan->reading[num].dist;
    scannedPoint = endPoint( startPosOfBeam, beam);

    if ( beam.forward < rawScan->maxDistance)
      writeLog("%f %f %f %f %s %s\n", elapsedScriptTime, scannedPoint.x, scannedPoint.y,
	       averageProbOfReading, measuredNoMax, filtered);
    else
      writeLog("%f %f %f %f %s %s\n", elapsedScriptTime, scannedPoint.x, scannedPoint.y,
	       averageProbOfReading, measuredMax, filtered);

    /* Set the endpoint of the expected distance. */
    expFeature = abstractScan->sensor[num].
      expectedFeature( realCenter,
		       abstractScan->sensor[num].infoForFeatures) ;

    beam.forward = expFeature.feature * cmPerFeature;

    scannedPoint = endPoint( startPosOfBeam, beam);

    if ( expFeature.feature < abstractScan->sensor[num].numberOfFeatures -1)
      writeLog("%f %f %f %f %s %s\n", elapsedScriptTime, scannedPoint.x, scannedPoint.y,
	       averageProbOfReading, expectedNoMax, filtered);
    else
      writeLog("%f %f %f %f %s %s\n", elapsedScriptTime, scannedPoint.x, scannedPoint.y,
	       averageProbOfReading, expectedMax, filtered);
  }
}


static void
dumpScan( actionInformation* info,
	  abstractSensorVector* frontAbstract,
	  abstractSensorVector* rearAbstract,
	  distanceScan* rawFrontScan,
	  distanceScan* rawRearScan,
	  realPosition robotPos)
{
  int num;
  bool frontMask[MAX_SIZE_OF_LASER_SCAN];
  bool rearMask[MAX_SIZE_OF_LASER_SCAN];
  
  info = info;
  /* Find all those sensor readings that are filtered out. */
  for ( num = 0; num < MAX_SIZE_OF_LASER_SCAN; num++) {
    frontMask[num] = TRUE;
    rearMask[num] = TRUE;
  }

  for ( num = 0; num < frontAbstract->numberOfSensorsToBeUsed; num++) 
    frontMask[frontAbstract->mask[num]] = FALSE;

  for ( num = 0; num < rearAbstract->numberOfSensorsToBeUsed; num++) 
    rearMask[rearAbstract->mask[num]] = FALSE;

  
  writeLog( "#SCAN %f %f %f:", robotPos.x, robotPos.y, robotPos.rot);

  /* Dump front scan. */
  for ( num = 0; num < MAX_SIZE_OF_LASER_SCAN; num++) {
    
    if ( frontMask[num] == TRUE)
      writeLog(" %f", - rawFrontScan->reading[num].dist);
    else
      writeLog(" %f", rawFrontScan->reading[num].dist);
  }
  
  /* Dump front scan. */
  for ( num = 0; num < MAX_SIZE_OF_LASER_SCAN; num++) {
    
    if ( rearMask[num] == TRUE)
      writeLog(" %f", - rawRearScan->reading[num].dist);
    else
      writeLog(" %f", rawRearScan->reading[num].dist);
  }
  
  writeLog("\n");
}


static void
dumpMeasurementErrors( actionInformation* info,
		       abstractSensorVector* abstractScan,
		       distanceScan* rawScan,
		       realPosition robotPos,
		       float sensorHeight,
		       char* text)
{
  int num;
  movement beam = {0.0, 0.0};
  realPosition maxEndPosOfBeam;
  float expectedDistance = 0.0;
  realPosition scanCenter = endPoint( robotPos,
				      rawScan->offset);
  float maxDist;
  
  info = info;

  if (sensorHeight < 70)
    maxDist = 500; /* laser */
  else
    maxDist = 3248.00; /* sonar */

  
  for ( num = 0; num < abstractScan->numberOfSensorsToBeUsed; num++) {
    
    int index = abstractScan->mask[num];
    realPosition startPosOfBeam = scanCenter;
    startPosOfBeam.rot = robotPos.rot + rawScan->reading[index].rot;
    
    /* Set the maximum endpoint of the expected distance. */
    beam.forward = maxDist;
    maxEndPosOfBeam = endPoint( startPosOfBeam, beam);

    expectedDistance =
      simulatorObjectDistance( &(info->simMap),
			       startPosOfBeam.x,
			       startPosOfBeam.y,
			       sensorHeight,
			       cos(robotPos.rot + rawScan->reading[index].rot),
			       sin(robotPos.rot + rawScan->reading[index].rot),
			       maxDist);
    
    writeLog("%f %f %f %s\n", rawScan->reading[index].dist, expectedDistance,
	     (rawScan->reading[index].dist - expectedDistance), text);
  }
}


void
dumpProximityErrors( actionInformation* info,
		     sensingActionMask* mask)
{
  /* Try to get the reference position. */
  realPosition estimatedPos = info->estimatedRobot.pos;
  
  info = info;
  
  /*---------------------------------------------------------------------------
   * FRONT LASER
   *---------------------------------------------------------------------------*/
  if ( mask->use[LASER] && mask->perform[LASER][INTEGRATE_LASER])
    dumpMeasurementErrors( info,
			   &(info->abstractSensors[ABSTRACT_FRONT_LASER]),
			   &(info->actualSensings.frontLaser), 
			   estimatedPos, LASER_HEIGHT, "#fLaser");

  /*---------------------------------------------------------------------------
   * REAR LASER
   *---------------------------------------------------------------------------*/
  if ( mask->use[LASER] && mask->perform[LASER][INTEGRATE_LASER])
    dumpMeasurementErrors( info,
			   &(info->abstractSensors[ABSTRACT_REAR_LASER]),
			   &(info->actualSensings.rearLaser),
			   estimatedPos, LASER_HEIGHT, "#rLaser");

  /*---------------------------------------------------------------------------
   * SONAR
   *---------------------------------------------------------------------------*/
  if ( mask->use[SONAR] && mask->perform[SONAR][INTEGRATE_SONAR])
    dumpMeasurementErrors( info,
			   &(info->abstractSensors[ABSTRACT_SONAR]),
			   &(info->actualSensings.sonar), 
			   estimatedPos, SONAR_HEIGHT, "#Sonar");

  nonRelevantTime += timeExpired( 0);
}



void
dumpProximityEndPoints( actionInformation* info,
			sensingActionMask* mask)
{
  /* Try to get the reference position. */
  bool success;
  realPosition measuredPos = measuredRobotPosition( elapsedScriptTime, &success);
  probability averageProbOfReading = 0.0;
  
  setTimer( 0);

  if ( ! success) {
    if ( info->localMaxima.numberOfCells == 0)
      return;
    else
      measuredPos = info->estimatedRobot.pos;
  }

#define TEST_DIRK
#ifdef TEST_DIRK
  if ( mask->use[LASER] && mask->perform[LASER][INTEGRATE_LASER])
    dumpScan( info,
	      &(info->abstractSensors[ABSTRACT_FRONT_LASER]),
	      &(info->abstractSensors[ABSTRACT_REAR_LASER]),
	      &(info->actualSensings.frontLaser),
	      &(info->actualSensings.rearLaser),
	      measuredPos);

/*   nonRelevantTime += timeExpired( 0); */
/*   return; */
#endif

  /* Compute an approximation of the average probability of the integrated readings. */
  {
    int numReadings = 0;
    
    if ( mask->use[LASER] && mask->perform[LASER][INTEGRATE_LASER])
      numReadings =
	info->abstractSensors[ABSTRACT_FRONT_LASER].numberOfSensorsToBeUsed +
	info->abstractSensors[ABSTRACT_REAR_LASER].numberOfSensorsToBeUsed;
    
    if ( mask->use[SONAR] && mask->perform[SONAR][INTEGRATE_SONAR])
      numReadings += info->abstractSensors[ABSTRACT_SONAR].numberOfSensorsToBeUsed;

    averageProbOfReading = pow( (double) 1.0 / info->positionProbs.normalizeFactor,
				1.0 / (double) numReadings);
  }
  
  /*---------------------------------------------------------------------------
   * FRONT LASER
   *---------------------------------------------------------------------------*/
  if ( mask->use[LASER] && mask->perform[LASER][INTEGRATE_LASER])
    dumpScanEndPoints( info,
		       &(info->abstractSensors[ABSTRACT_FRONT_LASER]),
		       &(info->actualSensings.frontLaser),
		       measuredPos, averageProbOfReading,
		       "#measuredLasNoMaxF", "#measuredLasMaxF",
		       "#expectedLasNoMaxF", "#expectedLasMaxF");

  /*---------------------------------------------------------------------------
   * REAR LASER
   *---------------------------------------------------------------------------*/
  if ( mask->use[LASER] && mask->perform[LASER][INTEGRATE_LASER])
    dumpScanEndPoints( info,
		       &(info->abstractSensors[ABSTRACT_REAR_LASER]),
		       &(info->actualSensings.rearLaser),
		       measuredPos, averageProbOfReading,
		       "#measuredLasNoMaxR", "#measuredLasMaxR",
		       "#expectedLasNoMaxR", "#expectedLasMaxR");

  /*---------------------------------------------------------------------------
   * SONAR
   *---------------------------------------------------------------------------*/
  if ( mask->use[SONAR] && mask->perform[SONAR][INTEGRATE_SONAR])
    dumpScanEndPoints( info,
		       &(info->abstractSensors[ABSTRACT_SONAR]),
		       &(info->actualSensings.sonar),
		       measuredPos, averageProbOfReading,
		       "#measuredSonNoMax", "#measuredSonMax",
		       "#expectedSonNoMax", "#expectedSonMax");

  nonRelevantTime += timeExpired( 0);
}


/* Dumps the probabilities of expected distances given position probabilities. */
#define NUMBER_OF_VIRTUAL_READINGS 90
#define MAX_NUM_CELLS 10000000

void
generateVirtualSensor( actionInformation* info)
{
  int num, x, y, plane;
  static int cnt = 0;
  probability featureProb[MAX_NUMBER_OF_FEATURES], sumOfProbs = 0.0;
  probability minProb = 0.0;
  positionProbabilityGrid* grid = &(info->positionProbs);
  abstractSensorType* sensor = &(info->abstractSensors[ABSTRACT_FRONT_LASER].
				 sensor[90]);
  float virtualReadings[NUMBER_OF_VIRTUAL_READINGS];  
  int virtualSensor = 0;
  
  infoForProximityFeature* sensorInfo;
  expectedDistTable* distTab;
  float cmPerFeature = ((infoForProximityFeature*)
			sensor->infoForFeatures)->distProbFunctionTable->distanceResolution;

  int sensCnt, useCnt;
  probability sum = 0.0;
  int cellCnt = 0, numberOfActiveCells = 0;
  int feat;
  
  static int firstTime = TRUE;
  static gridPosition* activeCells;
  static float* rotations;
  static float* activeProbs;

  if ( firstTime) {

    firstTime = FALSE;
    
    activeCells = (gridPosition*)
      malloc( MAX_NUM_CELLS * sizeof(realPosition));
    
    rotations = (float*) allocate1D( MAX_NUM_CELLS, FLOAT);
    activeProbs = (float*) allocate1D( MAX_NUM_CELLS, FLOAT);
  }
  
  if ( info->positionProbs.quotaOfValuesToBeUpdated < 0.8) {
    
    minProb = 0.00001;
    
    setTimer(7);
    
    /* Initialize the list of active cells. */

    /* Step through each plane. */
    for ( plane = 0; plane < grid->sizeZ; plane++) {

      float planeRot = rotationOfPlane( plane, grid);
      
      if ( grid->updatePlane[plane]) {
	
	for ( x = 0; x < grid->sizeX; x++) {
	  
	  for ( y = 0; y < grid->sizeY; y++) {
	    
	    if ( grid->prob[plane][x][y] > minProb) {
	      
	      activeCells[cellCnt].x = x;
	      activeCells[cellCnt].y = y;
	      activeProbs[cellCnt] = grid->prob[plane][x][y];
	      rotations[cellCnt] = planeRot;
	      sum += activeProbs[cellCnt];
	      cellCnt++;
	    }
	  }
	}
      }
    }
    numberOfActiveCells = cellCnt;
        
    /* Normalize */
    for ( cellCnt = 0; cellCnt < numberOfActiveCells; cellCnt++) 
      activeProbs[cellCnt] /= sum;
    
    
    /* Now compute for each direction the measurement. */
    for (sensCnt = 0; sensCnt < 360; sensCnt+=4) {
      
      if ( sensCnt < 180) {
	useCnt = sensCnt;
	sensor = &(info->abstractSensors[ABSTRACT_FRONT_LASER].
		   sensor[useCnt]);
      }
      else {
	useCnt = sensCnt - 180;
	sensor = &(info->abstractSensors[ABSTRACT_REAR_LASER].
		   sensor[useCnt]);
      }

      for ( num = 0; num < sensor->numberOfFeatures; num++)
	featureProb[num] = 0;
      
      sensorInfo = (infoForProximityFeature*) sensor->infoForFeatures;
      distTab = sensorInfo->distTab;
	      
      /* Step through the list of active cells and compute the virtual reading. */
      for ( cellCnt = 0; cellCnt < numberOfActiveCells; cellCnt++) {
	
	int rotationInDistTab =
	  distTabPlaneOfRotation( rotations[cellCnt] + sensorInfo->sensorRot,
				  distTab);

	feat =
	  distTab->dist[activeCells[cellCnt].x][activeCells[cellCnt].y][rotationInDistTab]
	  / distTab->numberOfStdDevs;
	
	featureProb[feat] += activeProbs[cellCnt];
      }

      if (0) for ( num = 0; num < sensor->numberOfFeatures; num++) {
	sumOfProbs += featureProb[num];
	writeLog( "%f %f #env%d %d#\n", num * cmPerFeature, featureProb[num],
		  cnt, useCnt);
	writeLog( "\nsum: %f\n", sumOfProbs);
      }
      
      /* Smooth the measurement probabilities. */
      if (1) {
	kernel kern;
	float element[2] = {0.6 , 0.2};
	kern.size = 2;
	kern.element = element;
	convolve1D( &(featureProb[1]),
		    sensor->numberOfFeatures - 2,
		    kern);
	convolve1D( featureProb,
		sensor->numberOfFeatures - 2,
		    kern);

	for ( num = 0; num < sensor->numberOfFeatures; num++) {
	  sumOfProbs += featureProb[num];
	  if (0) writeLog( "%f %f #envsmooth%d %d#\n", num * cmPerFeature,
			   featureProb[num], cnt, useCnt);
	}
	if (0) {
	  writeLog( "%f %f %f %d %f #timeexp\n", timeExpired(7),
		    info->positionProbs.quotaOfValuesToBeUpdated,
		    sum, cellCnt,
		    info->localMaxima.cell[0].prob * info->localMaxima.originalSumOfProbs);
	}
      }

      /* Now generate the reading which with high probability is shorter than this one. */
#define SHORTER_TRHESHOLD 0.01
      {
	float probSum = 0.0;
	num = 0;
	while ( num < sensor->numberOfFeatures && probSum < SHORTER_TRHESHOLD)
	  probSum += featureProb[num++];
	virtualReadings[virtualSensor] = num * cmPerFeature;
	if (0) fprintf( stderr, "%d %f\n", virtualSensor, virtualReadings[virtualSensor]);
	virtualSensor++;
      }
    }
    writeLog( "%f %f %f %d %f #timeexp\n", elapsedScriptTime,
	      timeExpired(7), sum, numberOfActiveCells,
	      info->localMaxima.cell[0].prob * info->localMaxima.originalSumOfProbs);

    fprintf( stderr, "%f %f %f %d %f #timeexp\n", elapsedScriptTime,
	      timeExpired(7), sum, numberOfActiveCells,
	      info->localMaxima.cell[0].prob * info->localMaxima.originalSumOfProbs);

  }
  else {
#define DEFAULT_DISTANCE 100.0

    virtualSensor = 0;
    
    /* Set a default distance. */
    for (sensCnt = 0; sensCnt < 360; sensCnt+=4) 
      virtualReadings[virtualSensor++] = DEFAULT_DISTANCE;
    
    writeLog("#timeexp %f\n", info->positionProbs.quotaOfValuesToBeUpdated);
  }
  
  sendVirtualReadingsToColli( info,
			      NUMBER_OF_VIRTUAL_READINGS,
			      virtualReadings);
  
  cnt++;
  nonRelevantTime += timeExpired( 0);
}


void
plotTable( distProbTable table, char* fName)
{
/* #define PLOT_TABLES */
#ifdef PLOT_TABLES
  int measuredCnt, expectedCnt;

  FILE* fp = fopen(fName, "w");


  for ( measuredCnt = 0;
	measuredCnt < table.numberOfMeasuredDistances; measuredCnt++){

    fprintf( fp, "%f ", measuredCnt * table.distanceResolution);

    for ( expectedCnt = 50;
	      expectedCnt < 60; expectedCnt++) {

      int expectedIndex = expectedCnt*table.numberOfStdDevs;

      fprintf( fp, "%f ", table.prob[expectedIndex][measuredCnt]);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
#else
  table=table;
  fName=fName;
#endif
}


