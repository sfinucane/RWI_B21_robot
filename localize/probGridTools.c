
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/probGridTools.c,v $
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
 * $Log: probGridTools.c,v $
 * Revision 1.1  2002/09/14 20:45:40  rstone
 * *** empty log message ***
 *
 * Revision 1.23  1999/01/11 19:47:54  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.21  1998/01/22 13:06:18  fox
 * First version after selection-submission.
 *
 * Revision 1.20  1998/01/05 10:37:14  fox
 * Several changes especially in selection parts.
 *
 * Revision 1.19  1997/11/21 15:36:06  fox
 * Modifications in graphic
 *
 * Revision 1.18  1997/10/01 11:30:00  fox
 * Minor changes.
 *
 * Revision 1.17  1997/09/29 22:08:39  wolfram
 * threshold for computing local maxima now depends on the size of the grid.
 * angles are now approximated by a gaussian.
 *
 * Revision 1.16  1997/09/29 16:29:42  wolfram
 * Angles now handle the situation where no sensor is given
 *
 * Revision 1.15  1997/08/22 04:16:39  fox
 * Final version before IJCAI.
 *
 * Revision 1.14  1997/08/19 18:24:06  wolfram
 * Fixed a bug in computeWeightedPositionAndSum
 *
 * Revision 1.13  1997/08/16 22:59:52  fox
 * Last version before I change selsection.
 *
 * Revision 1.12  1997/08/02 16:51:05  wolfram
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
 * Revision 1.11  1997/07/04 17:29:16  fox
 * Final version before holiday!!!
 *
 * Revision 1.10  1997/06/27 16:26:29  fox
 * New model of the proximity sensors.
 *
 * Revision 1.9  1997/06/25 14:16:40  fox
 * Changed laser incorporation.
 *
 * Revision 1.8  1997/06/20 07:36:13  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.7  1997/04/07 11:03:22  fox
 * Should be ok.
 *
 * Revision 1.6  1997/03/26 09:42:58  fox
 * Updated correction parameter message.
 *
 * Revision 1.5  1997/03/19 17:52:43  fox
 * New laser parameters.
 *
 * Revision 1.4  1997/03/18 18:45:30  fox
 * First executable version with a global scan mask for different sensor types.
 *
 * Revision 1.3  1997/03/13 17:36:23  fox
 * Temporary version. Don't use!
 *
 * Revision 1.2  1997/02/11 10:09:32  fox
 * No comment.
 *
 * Revision 1.1  1997/01/29 12:31:50  fox
 * New from exploreTools.
 *
 * Revision 1.25  1997/01/23 14:07:21  fox
 * Version of ijcai-submission.
 *
 * Revision 1.24  1997/01/19 18:56:57  wolfram
 * Again a bug ...
 *
 * Revision 1.23  1997/01/19 14:05:26  wolfram
 * Added counter for number of integrated readings
 *
 * Revision 1.22  1997/01/18 17:24:43  wolfram
 * Fixed two bugs
 *
 * Revision 1.21  1997/01/18 14:07:54  fox
 * Test version.
 *
 * Revision 1.20  1997/01/17 13:21:05  fox
 * Added timer for non relevant functions.
 *
 * Revision 1.19  1997/01/16 19:43:22  fox
 * And another bug ...
 *
 * Revision 1.18  1997/01/16 12:42:47  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.17  1997/01/14 16:53:22  fox
 * Removed a very very nasty bug for sensor selection.
 *
 * Revision 1.16  1997/01/14 10:38:19  wolfram
 * Test Version
 *
 * Revision 1.15  1997/01/08 15:59:10  wolfram
 * Resolved conflicts during merge
 *
 * Revision 1.14  1997/01/08 15:52:55  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.13  1997/01/07 12:31:26  wolfram
 * FindLocalMaxima additionally computes a list of localMaxima
 *
 * Revision 1.12  1997/01/07 09:46:20  wolfram
 * Added function to compute a list of real Positions containing the local maxima
 *
 * Revision 1.11  1997/01/07 08:08:20  wolfram
 * Fixed a bug in findLocalMaxima
 *
 * Revision 1.10  1997/01/03 10:09:45  fox
 * First version with exploration.
 *
 * Revision 1.9  1996/12/31 11:43:55  fox
 * First version using RANDOM_MODE of COLLI.
 *
 * Revision 1.8  1996/12/19 14:33:28  wolfram
 * More sensitive computation of expected distance tables
 *
 * Revision 1.7  1996/12/02 10:32:04  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.6  1996/11/26 11:08:11  fox
 * Improved version.
 *
 * Revision 1.5  1996/11/25 09:47:23  wolfram
 * Added graphic support for movment errors
 * Improved procedure for computing the entropy of a grid
 *
 * Revision 1.4  1996/11/22 10:30:24  wolfram
 * Added graphic for expected errors of movements
 *
 * Revision 1.3  1996/11/18 09:58:29  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.2  1996/11/15 17:44:05  ws
 * *** empty log message ***
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



#include <math.h>

#include "general.h"
#include "probGridTools.h"
#include "probGrid.h"
#include "function.h"
#include "sonar.h"
#include "allocate.h"
#include "file.h"


/*************************************************************
 *************************************************************
 * Some forward declarations.
 *************************************************************
 ***********************************************************/

static bool
localMaximum( positionProbabilityGrid *posGrid, gridPosition gridPos,
	      int deltaPos, int deltaRot);

static void
copyCellValues( gridCell *source, gridCell *dest);

static void
copyRealCellValues( realCell *source, realCell *dest);

/*************************************************************
  returns the normed distance between two angles
  both angles must be betwee 0 and 2*MP_I
  ***********************************************************/

distance
angleDistance(float rot1, float rot2)
{
  rot1 = fabs(rot1 - rot2);
  if (rot1 > DEG_180)
    rot1 = DEG_360 - rot1;
  return rot1;
}


static distance
normedAngleDistance(float rot1, float rot2)
{
  return angleDistance(rot1, rot2) / DEG_180;
}


/*************************************************************
  returns the normed distance between two real positions
  of a grid of sizeX and sizeY
  ***********************************************************/
distance
realPositionDistance(realPosition pos1, realPosition pos2)
{
  return (distance) sqrt(fSqr(pos1.x - pos2.x) + fSqr(pos1.y - pos2.y));
}

static distance
realPositionDistanceSquare(realPosition pos1, realPosition pos2)
{
  return (distance) fSqr(pos1.x - pos2.x) + fSqr(pos1.y - pos2.y);
}


static void
computeDistanceSquareMap(float **distSquareMap, realPosition pos1,
			 positionProbabilityGrid *grid)
{
  register gridPosition gridPos;
  realPosition pos2;

  float mapDiagonalSquare;

  mapDiagonalSquare = (fSqr(grid->sizeX) + fSqr(grid->sizeY)) *
    fSqr(grid->positionResolution);

  mapDiagonalSquare = 1/mapDiagonalSquare;
  
  gridPos.rot = 0;

  for (gridPos.x = 0; gridPos.x < grid->sizeX; gridPos.x++)
    for (gridPos.y = 0; gridPos.y < grid->sizeY; gridPos.y++){
      pos2 = realPositionOfGridPosition(gridPos, grid);
      distSquareMap[gridPos.x][gridPos.y] =
	realPositionDistanceSquare(pos1, pos2) * mapDiagonalSquare;
    }
}


probability
bayesEstimationError( positionProbabilityGrid *posGrid, realPosition reference)
{
  gridPosition pos;
  realPosition realPos;

  probability error = 0.0;
  distance angleDistSquare;
  probability ***grid = posGrid->prob;
  
  static probability **distSquareMap;
  static bool firstTimeErrorComputation = TRUE;

  if (firstTimeErrorComputation){
    distSquareMap = (float **)
      allocate2D( posGrid->sizeX, posGrid->sizeY, FLOAT);
    firstTimeErrorComputation = FALSE;
  }
  computeDistanceSquareMap(distSquareMap, reference, posGrid);
  
  for (pos.rot = 0; pos.rot < posGrid->sizeZ; pos.rot++){
    if (posGrid->updatePlane[pos.rot]){
      pos.x = pos.y = 0;
      realPos = realPositionOfGridPosition( pos, posGrid);
      angleDistSquare = fSqr(normedAngleDistance(reference.rot, realPos.rot));
      for (pos.x = 0; pos.x < posGrid->sizeX; pos.x++)
	for (pos.y = 0; pos.y < posGrid->sizeY; pos.y++)
	  error += sqrt(distSquareMap[pos.x][pos.y] + angleDistSquare)
	    * grid[pos.rot][pos.x][pos.y];
    }
  }

  /* multiply by sqrt(0.5) to get a maximum error of 1 */
  
  return (probability) error * sqrt(0.5);
}

/*****************************************************************************
 * Computes the entropy of a position probability grid;
 *****************************************************************************/


probability
posGridEntropy( positionProbabilityGrid* posGrid)
{
  register int x,y,z;
  probability entropy = 0.0;
  probability ***prob = posGrid->prob;
  probability planeEntropy;
  int cellsInPlane = posGrid->sizeX*posGrid->sizeY;

  for ( z=0; z < posGrid->sizeZ; z++) {
    if (posGrid->updatePlane[z]) {
      planeEntropy = 0.0;
      for (x = 0; x < posGrid->sizeX; x++)
	for (y = 0; y < posGrid->sizeY; y++){
	  if (prob[z][x][y] > 0.0)
	    planeEntropy -= prob[z][x][y] * fastLog(prob[z][x][y]);
	}
    }
    else {
      probability probOfPlane = posGrid->sumOfPlane[z] * posGrid->normalizeFactorOfPlane[z];
      planeEntropy = - probOfPlane * log ( probOfPlane / cellsInPlane);
    }
    
    entropy += planeEntropy;
  }
  return entropy;
}
    

/*************************************************************
  checks whether a given grid position is a local maximum
  ***********************************************************/
static bool
localMaximum( positionProbabilityGrid *posGrid, gridPosition gridPos,
	      int deltaPos, int deltaRot)
{

  int startX, startY, startZ, endX, endY, endZ;
  int x, y, z, zCount;
  probability ***prob = posGrid->prob;
  probability maxProb;

  maxProb =  prob[gridPos.rot][gridPos.x][gridPos.y];
  startX = iMax(gridPos.x - deltaPos, 0);
  startY = iMax(gridPos.y - deltaPos, 0);
  
  endX = iMin(gridPos.x + deltaPos + 1, posGrid->sizeX);
  endY = iMin(gridPos.y + deltaPos + 1, posGrid->sizeY);

  startZ = gridPos.rot - deltaRot;
  endZ = gridPos.rot + deltaRot + 1;

  for (zCount = startZ; zCount < endZ; zCount++){
    z = torusPosition(zCount, posGrid->sizeZ);
    if (posGrid->updatePlane[z])
      for (x = startX; x < endX; x++)
	for (y = startY; y < endY; y++)
	  if (prob[z][x][y] > maxProb)
	    return FALSE;
  }
  
  return TRUE;
} /* localMaximum */





static void
copyCellValues( gridCell *source, gridCell *dest){
  dest->pos.x = source->pos.x;
  dest->pos.y = source->pos.y;
  dest->pos.rot = source->pos.rot;
  dest->prob = source->prob;
}


static void
copyRealCellValues( realCell *source, realCell *dest){
  dest->pos.x = source->pos.x;
  dest->pos.y = source->pos.y;
  dest->pos.rot = source->pos.rot;
  dest->prob = source->prob;
}


void
insertLocalMaximum(gridCell nextCell, gridCellList *list)
{
  int i = list->numberOfCells;

  while (i > 0){
    if (list->cell[i-1].prob < nextCell.prob){
      if (i < MAX_NUMBER_OF_LOCAL_MAX) {
	(void) copyCellValues(&(list->cell[i-1]), &(list->cell[i]));
	list->inMap[i] = list->inMap[i-1];
      }
      i--;
    }
    else
      break;
  }
  
  if (list->numberOfCells < MAX_NUMBER_OF_LOCAL_MAX){
    list->numberOfCells ++;
    copyCellValues(&nextCell, &(list->cell[i]));
    list->inMap[i] = TRUE;
  }
  else
    if (i < list->numberOfCells) {
      copyCellValues(&nextCell, &(list->cell[i]));
      list->inMap[i] = TRUE;
    }
}


void
insertRealLocalMaximum( realCell nextCell, realCellList *list,
			int maxNumberOfLocalMax)
{
  int i = list->numberOfCells;

  while (i > 0){
    if (list->cell[i-1].prob < nextCell.prob){
      if (i < maxNumberOfLocalMax)
	(void) copyRealCellValues(&(list->cell[i-1]), &(list->cell[i]));
      list->inMap[i] = list->inMap[i-1];
      i--;
    }
    else
      break;
  }
  
  if (list->numberOfCells < maxNumberOfLocalMax){
    list->numberOfCells ++;
    list->inMap[i] = TRUE;
    copyRealCellValues(&nextCell, &(list->cell[i]));
  }
  else
    if (i < list->numberOfCells) {
      copyRealCellValues(&nextCell, &(list->cell[i]));
      list->inMap[i] = TRUE;
    }
}


void
normalizeRealCellList( realCellList *localMaxima)
{
  int i;
  probability sum = 0.0;
  
  for ( i = 0; i < localMaxima->numberOfCells; i++)
    sum += localMaxima->cell[i].prob;

  if (sum != 0.0){
    sum = 1 / sum;
    for ( i = 0; i < localMaxima->numberOfCells; i++)
      localMaxima->cell[i].prob *= sum;
  }
}


#define MINLOCALMAXFACTOR 100.0
void
findLocalMaxima( positionProbabilityGrid *posGrid,
		 realCellList *localMaxima,
		 int deltaPos, int deltaRot)
{
  int i;
  gridPosition gridPos;
  realCell realLocalMaxCell;
  float threshold;
  probability ***prob = posGrid->prob;
  int cnt = 0;
  
  threshold = fMin(0.1,  MINLOCALMAXFACTOR /
		   ((float) posGrid->sizeX * (float) posGrid->sizeY *
		    (float) posGrid->sizeZ));

  localMaxima->numberOfCells = 0;
  
  for (gridPos.rot = 0; gridPos.rot < posGrid->sizeZ; gridPos.rot++)
    if (posGrid->updatePlane[gridPos.rot]) {
      cnt++;
      for (gridPos.x = 0; gridPos.x < posGrid->sizeX; gridPos.x++)
	for (gridPos.y = 0; gridPos.y < posGrid->sizeY; gridPos.y++)
	  if (prob[gridPos.rot][gridPos.x][gridPos.y]
	      >
	      threshold)
	    if ( localMaximum( posGrid, gridPos, deltaPos, deltaRot)) {
	      computeWeightedPositionAndSum( gridPos,
					     posGrid,
					     &realLocalMaxCell.pos,
					     &realLocalMaxCell.prob,
					     &realLocalMaxCell.stdDev,
					     deltaPos, deltaRot);
	      insertRealLocalMaximum(realLocalMaxCell,
				     localMaxima,
				     MAX_NUMBER_OF_LOCAL_MAX);
	    }
    }
  localMaxima->originalSumOfProbs = 0.0;
  
  for (i = 0; i < localMaxima->numberOfCells; i++) {
    localMaxima->inMap[i] = TRUE;    
    localMaxima->originalSumOfProbs += localMaxima->cell[i].prob;
  }
  
  localMaxima->numberOfCellsInMap = localMaxima->numberOfCells;
  
  /* the probabilities of the local maxima must sum up to 1.0. */
  normalizeRealCellList( localMaxima);
  localMaxima->originalSumOfProbs = fMin( 1.0, localMaxima->originalSumOfProbs);
} /* find Local Maxima */
  


realPosition
measuredRobotPosition(float expiredTime, bool* success)
{
  static bool firstTimeCalled = TRUE;
  static float *positionTime;
  static realPosition *measuredPos;
  static int numberOfPositions = 0;
  char line[1024];
  realPosition tmpPos;
  float tmpTime;
  int i;
  bool notFound = TRUE;
  
  *success = TRUE;
  
  if (firstTimeCalled){
    char fileName[MAX_STRING_LENGTH];

    FILE *fpMeasuredPos;
    firstTimeCalled = FALSE;
    sprintf(fileName, "%s","position.log");
    if ((fpMeasuredPos = fopen(fileName,"rt")) == NULL) {
      fprintf(stderr,"ERROR: Could not open file '%s'!\n",fileName);
      tmpPos.x = tmpPos.y = tmpPos.rot = 0.0;
      *success = FALSE;
      return tmpPos;
    }

    /* count number of Positions */
    while (fgets(line,MAX_STRING_LENGTH,fpMeasuredPos) != NULL){
      if (sscanf( line, "%f %f %f %f", &tmpTime, &tmpPos.x, &tmpPos.y,
		  &tmpPos.rot) == 4)
	numberOfPositions++;
    }
    
    /* allocate memory */
    if ( (measuredPos = (realPosition *)
	  malloc( numberOfPositions * sizeof(realPosition))) == NULL){
      fprintf( stderr,
	       "Error! Not enough memory for allocating positions\n");
      numberOfPositions = 0;
    }

    if ( (positionTime = (float *)
	malloc( numberOfPositions * sizeof(float))) == NULL){
      fprintf( stderr,
	       "Error! Not enough memory for allocating positions\n");
      numberOfPositions = 0;
    }
    
    rewind(fpMeasuredPos);
    
    /* now read the values */
    i = 0;
    while (i < numberOfPositions){
      fgets(line,MAX_STRING_LENGTH,fpMeasuredPos);
      if (sscanf( line, "%f %f %f %f", &tmpTime, &tmpPos.x, &tmpPos.y,
		  &tmpPos.rot) == 4){
	measuredPos[i].x = tmpPos.x;
	measuredPos[i].y = tmpPos.y;
	measuredPos[i].rot = deg2Rad( tmpPos.rot);
	positionTime[i] = tmpTime;
	i++;
      }
    }
    
    fclose(fpMeasuredPos);
  }

  tmpPos.x = tmpPos.y = tmpPos.rot = 0.0;

  if (numberOfPositions == 0) {
    *success = FALSE;
    return tmpPos;
  }
  
  i = 0;
  notFound = TRUE;
  while (i < numberOfPositions && notFound)
    if (positionTime[i] > expiredTime)
      notFound = FALSE;
    else i++;
  
  if (!notFound)
    if (i == 0)
      tmpPos = measuredPos[0];
    else if (i >= numberOfPositions)
      tmpPos = measuredPos[numberOfPositions-1];
    else {
      tmpTime = positionTime[i] - positionTime[i-1];
      
      if (tmpTime > 0){
	float angleDiff;
	tmpPos.x = measuredPos[i-1].x + (measuredPos[i].x - measuredPos[i-1].x)
	  * ((expiredTime - positionTime[i-1]) / tmpTime);
	tmpPos.y = measuredPos[i-1].y + (measuredPos[i].y - measuredPos[i-1].y)
	  * ((expiredTime - positionTime[i-1]) / tmpTime);
	angleDiff = measuredPos[i].rot - measuredPos[i-1].rot;
	if (angleDiff > M_PI)
	  angleDiff = DEG_360 - angleDiff;
	else if (angleDiff < -M_PI)
	  angleDiff += DEG_360;
	tmpPos.rot = measuredPos[i-1].rot + angleDiff
	  * ((expiredTime - positionTime[i-1]) / tmpTime);
	tmpPos.rot = normalizedAngle(tmpPos.rot);
      }
      else {
	tmpPos.x = measuredPos[i-1].x;
	tmpPos.y = measuredPos[i-1].y;
	tmpPos.rot = measuredPos[i-1].rot;
      }
    }
  
  return tmpPos;
}
    

double
realCellListEntropy( realCellList* cellList)
{
  int i;
  double entropy = 0.0;

  for ( i = 0; i < cellList->numberOfCells; i++){
     if (cellList->cell[i].prob > 0)
       entropy -= cellList->cell[i].prob * log(cellList->cell[i].prob);
  }
  return entropy;
}


double
gridCellListEntropy( gridCellList* cellList)
{
  int i;
  double entropy = 0.0;
  
  for ( i = 0; i < cellList->numberOfCells; i++){
     if (cellList->cell[i].prob > 0)
      entropy -= cellList->cell[i].prob * log(cellList->cell[i].prob);
  }

  return entropy;
}









