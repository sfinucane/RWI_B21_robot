
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/proximityTools.h,v $
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
 * $Log: proximityTools.h,v $
 * Revision 1.1  2002/09/14 20:45:40  rstone
 * *** empty log message ***
 *
 * Revision 1.39  1999/12/16 16:14:00  fox
 * Several preparation changes for angles.
 *
 * Revision 1.38  1999/02/17 19:42:25  fox
 * Enhanced gif utilities.
 *
 * Revision 1.37  1999/01/14 23:39:34  wolfram
 * Added sensorMaxRange and standardDeviaton to the sensor model for lasers
 *
 * Revision 1.36  1998/11/17 23:26:27  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.35  1998/10/23 20:52:53  fox
 * Nothing specatacular.
 *
 * Revision 1.34  1998/06/12 10:16:40  fox
 * Implemented virutal sensor.
 *
 * Revision 1.33  1998/03/02 06:35:24  wolfram
 * Added output of sensor errror values. Small change in graphic.c
 *
 * Revision 1.32  1998/02/12 15:47:23  derr
 * compiler warnings (unused variable ... ) removed
 *
 * Revision 1.31  1997/11/25 17:13:02  fox
 * Should work.
 *
 * Revision 1.30  1997/10/29 09:11:42  wolfram
 * Simulator map is now installed only once
 *
 * Revision 1.29  1997/10/01 11:30:01  fox
 * Minor changes.
 *
 * Revision 1.28  1997/09/09 19:45:14  fox
 * Fixed a bug in finding local maxima and did several changes to active
 * localization.
 *
 * Revision 1.27  1997/08/22 04:16:41  fox
 * Final version before IJCAI.
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
 * Revision 1.25  1997/07/04 17:29:17  fox
 * Final version before holiday!!!
 *
 * Revision 1.24  1997/06/27 20:52:59  wolfram
 * Intermediate version reading initial feature probs (sonar only)
 *
 * Revision 1.23  1997/06/27 16:26:31  fox
 * New model of the proximity sensors.
 *
 * Revision 1.22  1997/06/25 14:16:42  fox
 * Changed laser incorporation.
 *
 * Revision 1.21  1997/06/20 07:36:15  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.20  1997/06/03 11:49:24  fox
 * Museum version.
 *
 * Revision 1.19  1997/05/26 10:32:04  fox
 * Added dump of rear laser and sonar data.
 *
 * Revision 1.18  1997/04/30 12:25:42  fox
 * Some minor changes.
 *
 * Revision 1.17  1997/04/06 22:38:34  wolfram
 * Dumping expected and measured features in log file now
 *
 * Revision 1.16  1997/03/24 06:55:29  wolfram
 * Cleaned up graphic.c and added a graphic window as a global variable
 * in graphic.c
 *
 * Revision 1.15  1997/03/18 18:45:31  fox
 * First executable version with a global scan mask for different sensor types.
 *
 * Revision 1.14  1997/03/14 17:58:23  fox
 * This version should run quite stable now.
 *
 * Revision 1.13  1997/03/13 17:36:38  fox
 * Temporary version. Don't use!
 *
 * Revision 1.12  1997/01/30 17:17:26  fox
 * New version with integrated laser.
 *
 * Revision 1.11  1997/01/29 12:23:14  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.10  1997/01/19 19:31:18  fox
 * yeah
 *
 * Revision 1.9  1997/01/19 18:56:58  wolfram
 * Again a bug ...
 *
 * Revision 1.8  1997/01/08 15:53:00  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.7  1996/12/31 09:19:26  fox
 * Last version before I switched exploration mode in PLAN to random
 * mode in COLLI.
 *
 * Revision 1.6  1996/12/19 14:33:29  wolfram
 * More sensitive computation of expected distance tables
 *
 * Revision 1.5  1996/12/09 10:12:01  wolfram
 * Fixed a bug in angle computation
 *
 * Revision 1.4  1996/12/03 17:47:12  wolfram
 * added normalized prob function table to improve the sensor selection
 *
 * Revision 1.3  1996/12/02 10:32:13  wolfram
 * Expected distance file now includes distances + variances
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



#ifndef PROXIMITY_TOOLS_INCLUDE
#define PROXIMITY_TOOLS_INCLUDE

#include "general.h"
#include "sensings.h"
#include "probGrid.h"
#include "function.h"
#include "proximity.h"


/* latter one must be a power of 2 and the product must of the latter
 two must not exceed 256 */

/*********************************************************
 *********************************************************
 * Structures.
 *********************************************************
 *********************************************************/

float
rotationOfDistTabPlane( int plane, expectedDistTable* tab);
int
distTabPlaneOfRotation( float rot, expectedDistTable* tab);

void
setNormedDistTab( void* infoForFeature,
		  float** uninformed);

void
setUnNormedDistTab( void* infoForFeature,
		    float** uninformed);

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
				distProbTable* distProbFunction);

/***********************************************************************
 * Returns the probability of a reading given the grid position
 * and the probability function for the distances.
 ***********************************************************************/
probability
distProb_GivenPositionAndRot( int distIndex,
			      gridPosition pos,
			      float sensorRotation,
			      positionProbabilityGrid *grid,
			      expectedDistTable *expectedDists,
			      distProbTable *distProbFunction);


/***********************************************************************
 * Computes the probability of a feature.
 ***********************************************************************/
float
probabilityOfProximityFeatureGivenPosition( int feature,
					    gridPosition position,
					    void* infoForFeature);

float
probabilityOfProximityFeatureGivenExpected( int feature,
					    featureStruct* expectedFeature,
					    void* infoForFeature);

void
setQuality( featureStruct* fStruct, int quality);

featureStruct
expectedProximityFeature( gridPosition position,
			  void* infoForFeature);

void
averageProbabilities( distProbTable* table, float* probs);

/* Returns the distance to the next obstacle in the map for a given
 * position and orientation. */
distance
obstacleDistanceReal( probabilityGrid m, realPosition realPos,
		     distance maxRange);

int**
preprocessedPlaneInDistTable( abstractSensorVector* scan);

/* Computes the expected distances. */
expectedDistTable
preprocessedExpectedDistances( probabilityGrid* map,
			       simulatorMap* simMap,
			       sensorParameters params,
			       int numberOfOrientations,
			       char* file,
			       bool onlineMapping);


/* Generates a table including the distance probability function. */
distProbTable
probFunctionTable( sensorParameters params,
		   expectedDistTable expDist);

distProbTable
normedProbFunctionTable( sensorParameters params,
			 expectedDistTable expDist);

distProbTable
readProbFunctionTable( char *fileName,
		       int numberOfStdDevs,
		       expectedDistTable expDist);

probability
distProbFunction( sensorParameters params,
		  int expectedDistIndex,
		  int stdDevCnt,
		  int measuredDistIndex);

/* Read the table of the expected distances from a file. */
expectedDistTable
readDistTab( char* fileName, sensorParameters params, int onlineMapping);

void
dumpDistTable( distProbTable table, char* startString);


distance
obstacleDistance( probabilityGrid *m, int x, int y,
		  float rot, distance maxRange);


int
distanceIndex( float distance, float distanceResolution, int numberOfDistances);

#define ASCENDING_ORDER 0
#define DESCENDING_ORDER 1
void
setMaskAccordingToEvaluations( int* mask, int sizeOfMask,
			       double* values, int numberOfValuesToBeSet,
			       int order);

int
extractPartialScanMask( int* globalMask, int sizeOfGlobalMask,
			int* mask,
			int start, int end);


int
expectedDistIndex(gridPosition gridPos, expectedDistTable *distTab);

distance
expectedDist(gridPosition gridPos, expectedDistTable *distTab);

int
numberOfExpectedStdDev(gridPosition gridPos, expectedDistTable *distTab);

/* Dumps expected and measured distance indices at the estimated position. */
void
dumpProximityValues( actionInformation* info,
		     sensingActionMask* mask,
		     realPosition estimatedPos);

/* Dumps the probabilities of expected distances given position probabilities. */
void
generateVirtualSensor( actionInformation* info);


/* Dumps expected and measured distances at the estimated position. */
void
dumpProximityErrors( actionInformation* info,
		     sensingActionMask* mask);

/* Dumps endpoints of selected sensors. */
void
dumpProximityEndPoints( actionInformation* info,
			sensingActionMask* mask);

distance
obstacleDistanceReal( probabilityGrid m, realPosition realPos,
		      distance maxRange);

int
expectedFeatureGridMap( probabilityGrid* map,
			int centerX,  int centerY, int rot,
			distance distanceResolution,
			int maxFeature);

distance
expectedDistanceGridMap( probabilityGrid* map,
			 int centerX, int centerY, int rot,
			 distance distanceResolution,
			 int maxFeature);

bool
simulatorMapInstalled(char *fileName);

void
plotTable( distProbTable table, char* fName);


#endif




