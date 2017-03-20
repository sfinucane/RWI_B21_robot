
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/probGrid.h,v $
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
 * $Log: probGrid.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.25  1999/12/15 16:16:40  fox
 * First attempt to extract p(l | o).
 *
 * Revision 1.24  1998/08/23 00:01:02  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.23  1998/08/19 16:33:57  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.22  1998/08/11 23:05:40  wolfram
 * Changes for the Smithsonian museum.
 *
 * WARNING: LOCALIZE now uses only one stddev for Lasers. Laser beams have 25 m
 * Maxrange now.
 * Please use ~wolfram/bee/data/localize/params/laser-0.1stdev with this version
 * of LOCALIZE
 *
 * Revision 1.21  1998/06/12 10:16:36  fox
 * Implemented virutal sensor.
 *
 * Revision 1.20  1997/12/17 16:48:31  wolfram
 * Added NEIGHBOR_PLANES_TO_ADD keyword for normalize
 *
 * Revision 1.19  1997/12/03 09:09:09  fox
 * Renamde USE_MOVEMENT to USE_POSITION.
 *
 * Revision 1.18  1997/11/20 12:58:13  fox
 * Version with good sensor selection.
 *
 * Revision 1.17  1997/10/01 11:30:00  fox
 * Minor changes.
 *
 * Revision 1.16  1997/08/19 18:24:06  wolfram
 * Fixed a bug in computeWeightedPositionAndSum
 *
 * Revision 1.15  1997/08/16 21:52:03  wolfram
 * Grid maps can now be cad maps.  Obstacles are enlarged accordingly
 *
 * Revision 1.14  1997/08/02 16:51:05  wolfram
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
 * Revision 1.13  1997/06/27 16:26:28  fox
 * New model of the proximity sensors.
 *
 * Revision 1.12  1997/06/20 07:36:12  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.11  1997/04/24 21:25:43  wolfram
 * Ooops, fourth parameter of CUBE's is radius and not diameter!
 *
 * Revision 1.10  1997/03/14 17:58:22  fox
 * This version should run quite stable now.
 *
 * Revision 1.9  1997/03/13 17:36:38  fox
 * Temporary version. Don't use!
 *
 * Revision 1.8  1997/01/29 12:23:12  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.7  1997/01/17 13:21:07  fox
 * Added timer for non relevant functions.
 *
 * Revision 1.6  1997/01/10 15:19:24  fox
 * Improved several methods.
 *
 * Revision 1.5  1997/01/07 09:27:39  wolfram
 * Added procedure to compute weighted position and sum of probs
 *
 * Revision 1.4  1996/12/02 10:32:11  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.3  1996/11/18 09:58:31  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.2  1996/11/15 17:44:07  ws
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


#ifndef PROBGRID_INCLUDE
#define PROBGRID_INCLUDE

#include "general.h"
#include "sensings.h"

#define XY_CUBE 30
#define Z_CUBE  deg2Rad( 3.0)

/* Minimum normalization factor for normalizeFactorOfPlane */
#define MIN_NORM_FACTOR 1e-9
  

/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/

/**************************************************************************
 **************************************************************************
 * Structures.
 **************************************************************************
 **************************************************************************/

typedef struct {
  int numberOfAngles;
  float maxQuotaOfPlanesForNormalization;
  float minShiftForReset;
  int numberOfShiftsForNormalize;
  int numberOfSonarsForNormalize;
  int numberOfAnglesForNormalize;
  int numberOfLasersForNormalize;
  int numberOfVisionsForNormalize;
  int computeStatistics;
  int deltaPosForLocalMax;
  int deltaRotForLocalMax;
  int neighborPlanesToAdd;
  int generateVirtualSensor;
} probGridParameters;



/**************************************************************************
 **************************************************************************
 * Other functions.
 **************************************************************************
 **************************************************************************/

/*  Returns the rotation of the given plane in the grid. */
float
rotationOfPlane( int plane, positionProbabilityGrid* grid);

/*  Returns the plane closest to the rotation. */
int
planeOfRotation( float rot, positionProbabilityGrid* grid);

/*  Returns the real position given a floating point gridPosition. */
realPosition
realPositionOfFloatGridPosition( realPosition pos,
				positionProbabilityGrid* grid);

/*  Returns the real position of the robot in the given grid. */
realPosition
realPositionOfGridPosition( gridPosition pos, positionProbabilityGrid* grid);


/*  Returns the grid position closest to the real position. */
gridPosition
gridPositionOfRealPosition( realPosition pos, positionProbabilityGrid* grid);

/* Is the grid coordinate in the grid? */
bool
coordinateInGrid( int x, int y, positionProbabilityGrid *grid);

/* Is the real position in the grid? */
bool
realPositionInGrid( float x, float y, positionProbabilityGrid *grid);

/*  Returns the probability of the grid cell closest to the position. */
probability
probabilityOfRealPosition( realPosition pos, positionProbabilityGrid* grid);

void
shiftAndMultiplyProbabilities( positionProbabilityGrid* posGrid,
			       probabilityGrid* mapPositionProbs,
			       int plane,
			       int sizeX, int sizeY,
			       int deltaX, int deltaY,
			       int usePosition);

void
checkWhichActionsToPerform_PROBGRID( actionInformation* info, sensingActionMask* mask);

void
performActions_PROBGRID( actionInformation* info, sensingActionMask* mask);

/* normalizes a grid such that the sum of all cells is 1 */
gridPosition
normalizePositionProbabilityGrid( positionProbabilityGrid* posGrid,
				  probabilityGrid* aPrioriProbs);

void
dumpXYPlane( positionProbabilityGrid* grid, int plane, char* fileName);

void
dumpXZPlane( positionProbabilityGrid* grid, int plane, char* fileName);

void
dumpYZPlane( positionProbabilityGrid* grid, int plane, char* fileName);

probability
summedProbabilities( positionProbabilityGrid* posGrid, int plane);

int
normalizeFactorHasChangedUpdatePlanes( positionProbabilityGrid* grid);

void
setPosition( realPosition center, positionProbabilityGrid* grid,
	     float deviation);

void
setPositionUniform( realPosition center, positionProbabilityGrid* grid,
	     float deviation);

void
removeCellsNotInGrid( realCellList* cellList,
		      positionProbabilityGrid* grid);

int
torusPosition(int position, int size);

int
cmToCells( float cm, positionProbabilityGrid *posGrid);

int
radToPlanes( float rad, positionProbabilityGrid *posGrid);

distance
gridPositionAndAngleDistance(gridPosition pos1, gridPosition pos2,
			     positionProbabilityGrid *grid);

realPosition
weightedPosition( gridPosition max,
		  positionProbabilityGrid *posGrid);

void
computeWeightedPositionAndSum( gridPosition max,
			       positionProbabilityGrid *posGrid,
			       realPosition *realPos,
			       probability *probSum,
			       probability *stdDeviation,
			       int deltaPos,
			       int deltaRot);
void
setCellList( positionProbabilityGrid *grid, gridCellList *cells);


probabilityGrid
preprocessedPositionProbabilities( probabilityGrid* map, float additionalSize);

void
resetGrid( positionProbabilityGrid* grid,
	   probabilityGrid* aPrioriProbs);

void
writeCurrentPosition( realCellList* localMaxima);

bool
readStartPosition( realPosition* startPos);

#endif






