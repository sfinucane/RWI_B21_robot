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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/dynamic.c,v $
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
 * $Log: dynamic.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.21  1997/11/25 17:12:46  fox
 * Should work.
 *
 * Revision 1.20  1997/11/20 12:58:09  fox
 * Version with good sensor selection.
 *
 * Revision 1.19  1997/07/04 17:29:13  fox
 * Final version before holiday!!!
 *
 * Revision 1.18  1997/06/20 07:36:08  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.17  1997/05/26 08:47:44  fox
 * Last version before major changes.
 *
 * Revision 1.16  1997/01/29 12:23:04  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.15  1997/01/18 19:41:03  fox
 * Improved action selection.
 *
 * Revision 1.14  1997/01/10 15:19:22  fox
 * Improved several methods.
 *
 * Revision 1.13  1997/01/03 18:07:46  fox
 * Successfully localized the robot and moved it into several rooms without
 * loosing the position again.
 *
 * Revision 1.12  1996/12/31 09:19:22  fox
 * Last version before I switched exploration mode in PLAN to random
 * mode in COLLI.
 *
 * Revision 1.11  1996/12/20 15:29:37  fox
 * Added four parameters.
 *
 * Revision 1.10  1996/12/19 11:13:25  wolfram
 * *** empty log message ***
 *
 * Revision 1.9  1996/12/05 17:18:08  fox
 * First attempt to plan a path to interesting places.
 *
 * Revision 1.8  1996/12/04 14:29:59  fox
 * ok
 *
 * Revision 1.7  1996/12/03 15:40:24  fox
 * ok
 *
 * Revision 1.6  1996/12/03 12:27:40  fox
 * Seems to work with the thin corridor.
 *
 * Revision 1.5  1996/12/02 10:32:03  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.4  1996/11/28 17:56:21  fox
 * *** empty log message ***
 *
 * Revision 1.3  1996/11/28 12:40:29  wolfram
 * Wolframs version.
 *
 * Revision 1.2  1996/11/28 09:05:49  fox
 * nicks spezielles.
 *
 * Revision 1.1  1996/11/27 16:13:37  fox
 * ok
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include "general.h"
#include "selection.h"
#include "dynamic.h"
#include "allocate.h"
#include "function.h"
#include "file.h"
#include "graphic.h"

#define MIN_SUM_OF_UTILITIES_CHANGE 1.001
#define MIN_NUMBER_OF_SWEEPS 20


static void
resizeBorders( int* minX, int* maxX, int maxAllowedX,
	       int* minY, int* maxY, int maxAllowedY)
{
  if ( *minX > 0)
    (*minX)--;
  if ( *maxX < maxAllowedX)
    (*maxX)++;

  if ( *minY > 0)
    (*minY)--;
  if ( *maxY < maxAllowedY)
    (*maxY)++;
}

/*************************************************************************
 ************************************************************************
 * Function to compute the cost factor of a given value. 
 *************************************************************************
 *************************************************************************/

#ifdef JUST_FOR_PRESENTATION
#define MAX_REDUCTION_THRESHOLD 0.8
#define MAX_REDUCTION 0.0001
#define MIN_REDUCTION_THRESHOLD 0.2
#define MIN_REDUCTION 0.99
#define COSTS_EXPONENT 0.4
#else
#define MAX_REDUCTION_THRESHOLD 0.8
#define MAX_REDUCTION 0.0001
#define MIN_REDUCTION_THRESHOLD 0.2
#define MIN_REDUCTION 0.9999 
#define COSTS_EXPONENT 0.4
#endif

probability
costsFunction( probability value)
{
  if ( value > MAX_REDUCTION_THRESHOLD)
    return MAX_REDUCTION;
  else
    if ( value < MIN_REDUCTION_THRESHOLD)
      return pow( MIN_REDUCTION, COSTS_EXPONENT);
    else
      return pow( 1.0 - value, COSTS_EXPONENT);
}


void
spreadUtilities( probabilityGrid* costs,
		 probabilityGrid* utilities,
		 int numberOfSweeps)
{
  int sweep, x, y;
  
  static probability** tmpUtilities = NULL;
  static int tmpSizeX, tmpSizeY;
  static probability** tmpPointer;

  probability diagonalDistFactor = 1.2 / sqrt( 2.0);
  probability previosSumOfUtilities = 0.0;

  int minUpdatedX = utilities->sizeX, maxUpdatedX = 0;
  int minUpdatedY = utilities->sizeY, maxUpdatedY = 0;

  if ( numberOfSweeps <= 0)
    numberOfSweeps = costs->sizeX * costs->sizeY;
  
  /* Check wether the sizes have changed. */
  if ( tmpUtilities == NULL || tmpSizeX != utilities->sizeX ||
       tmpSizeY != utilities->sizeY) {
    
    /* Free the old space. */
    if ( tmpUtilities != NULL) 
      free2D( (void**) tmpUtilities, tmpSizeX, MAP_PROBABILITY);
    
    tmpUtilities = (mapProbability**)
      allocate2D( utilities->sizeX, utilities->sizeY, MAP_PROBABILITY);

    tmpSizeX = utilities->sizeX;
    tmpSizeY = utilities->sizeY;
  }
    
  fprintf(stderr, "# Perform dynamic programming ... ");
  

  /* Initialize the tmpUtitlities with the start utilities. */
  for ( x = 0; x < costs->sizeX; x++)
    for ( y = 0; y < costs->sizeY; y++) {
      tmpUtilities[x][y] = utilities->prob[x][y];
      if ( tmpUtilities[x][y] != utilities->unknown) {
	previosSumOfUtilities += tmpUtilities[x][y];
	if ( x < minUpdatedX)
	  minUpdatedX = x;
	if ( x > maxUpdatedX)
	  maxUpdatedX = x;
	if ( y < minUpdatedY)
	  minUpdatedY = y;
	if ( y > maxUpdatedY)
	  maxUpdatedY = y;
      }
    }
  
  resizeBorders( &minUpdatedX, &maxUpdatedX, costs->sizeX-1,
		 &minUpdatedY, &maxUpdatedY, costs->sizeY-1);
  
  /* Do the real thing. */
  for ( sweep = 0; sweep < numberOfSweeps; sweep++) {
    
    probability sumOfUtilities = 0.0;
    
    /* Just swap the pointers of the probabilities. */
    tmpPointer = tmpUtilities;
    tmpUtilities = utilities->prob;
    utilities->prob = tmpPointer;
    
    for ( x = minUpdatedX; x < maxUpdatedX; x++)
      for ( y = minUpdatedY; y < maxUpdatedY; y++) {

	probability maxStraight = MIN_UTILITY;
	probability maxDiagonal = MIN_UTILITY;
	probability maxNeighborUtility;
	probability utilityReductionFactor;

	/* compute the cost for the step onto thiis cell. */
	if ( costs->prob[x][y] != costs->unknown) 
	  utilityReductionFactor = costsFunction( costs->prob[x][y]);
	else
	  utilityReductionFactor = MAX_REDUCTION;
	  
	/* LEFT */
	if ( x > 0) {
	  if ( tmpUtilities[x-1][y] > maxStraight)
	    maxStraight = tmpUtilities[x-1][y];
	  if ( y > 0) 
	    if ( tmpUtilities[x-1][y-1] > maxDiagonal)
	      maxDiagonal = tmpUtilities[x-1][y-1];
	  if ( y < costs->sizeY - 1) 
	    if ( tmpUtilities[x-1][y+1] > maxDiagonal)
	      maxDiagonal = tmpUtilities[x-1][y+1];
	}
	/* RIGHT */
	if ( x < costs->sizeX - 1) {
	  if ( tmpUtilities[x+1][y] > maxStraight)
	    maxStraight = tmpUtilities[x+1][y];
	  if ( y > 0) 
	    if ( tmpUtilities[x+1][y-1] > maxDiagonal)
	      maxDiagonal = tmpUtilities[x+1][y-1];
	  if ( y < costs->sizeY - 1) 
	    if ( tmpUtilities[x+1][y+1] > maxDiagonal)
	      maxDiagonal = tmpUtilities[x+1][y+1];
	}
	
	/* UPPER */
	if ( y > 0) 
	  if ( tmpUtilities[x][y-1] > maxStraight)
	    maxStraight = tmpUtilities[x][y-1];
	
	/* LOWER */
	if ( y < costs->sizeY - 1) 
	  if ( tmpUtilities[x][y+1] > maxStraight)
	    maxStraight = tmpUtilities[x][y+1];
	
	/* The distance in the diagonal direction is bigger. */
	maxDiagonal *= diagonalDistFactor;
	
	maxNeighborUtility = utilityReductionFactor * fMax( maxStraight, maxDiagonal);
	
	if ( utilities->prob[x][y] < maxNeighborUtility) 
	  utilities->prob[x][y] = maxNeighborUtility;
	
	sumOfUtilities += utilities->prob[x][y];
      }
    
    /* Let's stop this process as soon as possible. */
    if ( sweep >= MIN_NUMBER_OF_SWEEPS
	 && (sumOfUtilities / previosSumOfUtilities) < MIN_SUM_OF_UTILITIES_CHANGE)
      break;
    else {
      previosSumOfUtilities = sumOfUtilities;
      resizeBorders( &minUpdatedX, &maxUpdatedX, costs->sizeX-1,
		     &minUpdatedY, &maxUpdatedY, costs->sizeY-1);
    }
  }
  
  /* Set the utilities. */
  normalizeMap( utilities);
  fprintf( stderr, "done.\n");
  return;
}
  
  
/* This is done by simply inverting the values of the costs,
 * performing spreadUtilities, and finally reinverting the utilities. */
void
spreadCosts( probabilityGrid* costs,
	     probabilityGrid* costsToBeSpread,
	     int numberOfSweeps)
{
  int x, y;
  
  /* Invert to get utilities. */
  for ( x = 0; x < costs->sizeX; x++)
    for ( y = 0; y < costs->sizeY; y++)
      costsToBeSpread->prob[x][y] = 1.0 - costsToBeSpread->prob[x][y];

  costsToBeSpread->unknown = 1.0 - costsToBeSpread->unknown;
  
  /* Spread the utilities. */
  spreadUtilities( costs, costsToBeSpread, numberOfSweeps);

  /* Reinvert to get costs. */
  for ( x = 0; x < costs->sizeX; x++)
    for ( y = 0; y < costs->sizeY; y++)
      costsToBeSpread->prob[x][y] = 1.0 - costsToBeSpread->prob[x][y];

  costsToBeSpread->unknown = 1.0 - costsToBeSpread->unknown;
}


void
shrinkMaxima( probabilityGrid* utilities,
	      int influence)
{
  if ( influence == 0)
    return;
  else {
    
    int x, y;
    int x2, y2;
    probability unknown = utilities->unknown;
    
    
    static probability** tmpUtilities = NULL;
    static int tmpSizeX, tmpSizeY;
    
    /* Check wether the sizes have changed. */
    if ( tmpUtilities == NULL || tmpSizeX != utilities->sizeX ||
	 tmpSizeY != utilities->sizeY) {
      
      /* Free the old space. */
      if ( tmpUtilities != NULL) 
	free2D( (void**) tmpUtilities, tmpSizeX, MAP_PROBABILITY);
      
      tmpUtilities = (mapProbability**)
	allocate2D( utilities->sizeX, utilities->sizeY, MAP_PROBABILITY);
      
      tmpSizeX = utilities->sizeX;
      tmpSizeY = utilities->sizeY;
    }
    
    /* Initialize the tmpUtitlities with the start utilities. */
    for ( x = 0; x < utilities->sizeX; x++)
      for ( y = 0; y < utilities->sizeY; y++) 
	tmpUtilities[x][y] = utilities->prob[x][y];
  
    
    for ( x = 0; x < utilities->sizeX; x++)
      for ( y = 0; y < utilities->sizeY; y++) {
	
	if ( utilities->prob[x][y] != unknown) {
	  
	  probability min = utilities->prob[x][y];
	  
	  for ( x2 = iMax( 0, x - influence);
		x2 <= iMin( utilities->sizeX - 1, x + influence);
		x2++)
	    
	    for ( y2 = iMax( 0, y - influence);
		  y2 <= iMin( utilities->sizeY - 1, y + influence);
		  y2++)
	      
	      if ( tmpUtilities[x2][y2] != unknown && tmpUtilities[x2][y2] < min)
		min = tmpUtilities[x2][y2];
	  
	  /* Now set the value. */
	  utilities->prob[x][y] = min;
	}
      }
  }
}

  
int
extractGradientDownPath( probabilityGrid* map, gridPosition start,
			 gridPosition* path, int maxPathLength)
{
  int i;
  probability min;
  probability** utilities = map->prob;
  
  path[0] = start;

  for ( i = 1; i < maxPathLength; i++) {

    bool foundBetter = FALSE;
    int minX=0, minY=0;
    int currentX = path[i-1].x;
    int currentY = path[i-1].y;
    
    /* Look for the best neighbor. */
    min = utilities[currentX][currentY];
    
    /* LEFT */
    if ( currentX > 0) {
      if ( utilities[currentX-1][currentY] < min) {
	foundBetter = TRUE;
	minX = currentX-1;
	minY = currentY;
	min = utilities[currentX-1][currentY];
      }
      if ( currentY > 0) {
	if ( utilities[currentX-1][currentY-1] < min) {
	  foundBetter = TRUE;
	  minX = currentX-1;
	  minY = currentY-1;
	  min = utilities[currentX-1][currentY-1];
	}
      }
      if ( currentY < map->sizeY - 1) {
	if ( utilities[currentX-1][currentY+1] < min) {
	  foundBetter = TRUE;
	  minX = currentX-1;
	  minY = currentY+1;
	  min = utilities[currentX-1][currentY+1];
	}
      }
    }
    
    
    /* RIGHT */
    if ( currentX < map->sizeX - 1) {
      if ( utilities[currentX+1][currentY] < min) {
	foundBetter = TRUE;
	minX = currentX+1;
	minY = currentY;
	min = utilities[currentX+1][currentY];
      }
      if ( currentY > 0) {
	if ( utilities[currentX+1][currentY-1] < min) {
	  foundBetter = TRUE;
	  minX = currentX+1;
	  minY = currentY-1;
	  min = utilities[currentX+1][currentY-1];
	}
      }
      if ( currentY < map->sizeY - 1) {
	if ( utilities[currentX+1][currentY+1] < min) {
	  foundBetter = TRUE;
	  minX = currentX+1;
	  minY = currentY+1;
	  min = utilities[currentX+1][currentY+1];
	}
      }
    }
      

    /* UPPER */
    if ( currentY > 0) 
      if ( utilities[currentX][currentY-1] < min) {
	foundBetter = TRUE;
	minX = currentX;
	minY = currentY-1;
	min = utilities[currentX][currentY-1];
      }
    
    /* LOWER */
    if ( currentY < map->sizeY - 1) 
      if ( utilities[currentX][currentY+1] < min) {
	foundBetter = TRUE;
	minX = currentX;
	minY = currentY+1;
	min = utilities[currentX][currentY+1];
      }

    if ( foundBetter) {
      path[i].x = minX;
      path[i].y = minY;
    }
    else
      return i;
  }
  return i;
}



int
extractGradientUpPath( probabilityGrid* map, gridPosition start,
		       gridPosition* path, int maxPathLength)
{
  int i;
  probability max;
  probability** utilities = map->prob;
  
  path[0] = start;

  for ( i = 1; i < maxPathLength; i++) {

    bool foundBetter = FALSE;
    int maxX=0, maxY=0;
    int currentX = path[i-1].x;
    int currentY = path[i-1].y;
    
    /* Look for the best neighbor. */
    max = utilities[currentX][currentY];
    
    /* LEFT */
    if ( currentX > 0) {
      if ( utilities[currentX-1][currentY] > max) {
	foundBetter = TRUE;
	maxX = currentX-1;
	maxY = currentY;
	max = utilities[currentX-1][currentY];
      }
      if ( currentY > 0) {
	if ( utilities[currentX-1][currentY-1] > max) {
	  foundBetter = TRUE;
	  maxX = currentX-1;
	  maxY = currentY-1;
	  max = utilities[currentX-1][currentY-1];
	}
      }
      if ( currentY < map->sizeY - 1) {
	if ( utilities[currentX-1][currentY+1] > max) {
	  foundBetter = TRUE;
	  maxX = currentX-1;
	  maxY = currentY+1;
	  max = utilities[currentX-1][currentY+1];
	}
      }
    }
    
    
    /* RIGHT */
    if ( currentX < map->sizeX - 1) {
      if ( utilities[currentX+1][currentY] > max) {
	foundBetter = TRUE;
	maxX = currentX+1;
	maxY = currentY;
	max = utilities[currentX+1][currentY];
      }
      if ( currentY > 0) {
	if ( utilities[currentX+1][currentY-1] > max) {
	  foundBetter = TRUE;
	  maxX = currentX+1;
	  maxY = currentY-1;
	  max = utilities[currentX+1][currentY-1];
	}
      }
      if ( currentY < map->sizeY - 1) {
	if ( utilities[currentX+1][currentY+1] > max) {
	  foundBetter = TRUE;
	  maxX = currentX+1;
	  maxY = currentY+1;
	  max = utilities[currentX+1][currentY+1];
	}
      }
    }
      

    /* UPPER */
    if ( currentY > 0) 
      if ( utilities[currentX][currentY-1] > max) {
	foundBetter = TRUE;
	maxX = currentX;
	maxY = currentY-1;
	max = utilities[currentX][currentY-1];
      }
    
    /* LOWER */
    if ( currentY < map->sizeY - 1) 
      if ( utilities[currentX][currentY+1] > max) {
	foundBetter = TRUE;
	maxX = currentX;
	maxY = currentY+1;
	max = utilities[currentX][currentY+1];
      }

    if ( foundBetter) {
      path[i].x = maxX;
      path[i].y = maxY;
    }
    else
      return i;
  }
  return i;
}



