
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection-old2/human.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:04 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: human.c,v $
 * Revision 1.1  2002/09/14 20:45:04  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1998/09/05 00:25:27  fox
 * Works fine to detect people in the museum.
 *
 * Revision 1.2  1998/08/29 21:44:43  fox
 * Changed the detection report and improved stuck detection.
 *
 * Revision 1.1  1998/08/23 22:57:40  fox
 * First version of building maps of humans.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include "general.h"
#include "PANTILT-messages.h"
#include "detection.h"
#include "allocate.h"
#include "function.h"
#include "laser.h"
#include "localTcx.h"
#include "graphics.h"
#include "o-graphics.h"
#include "pantilt.h"
#include "human.h"

#include "DETECTION-messages.h"

humanProbTable humanProbs;
probabilityGrid humanMap;

#define MAP_RESOLUTION 10.0

/* ----------------------------------------------------------------------
 * ----------------------------------------------------------------------
 * Local functions.
 * ----------------------------------------------------------------------
 * ---------------------------------------------------------------------- */

static int
readHumanProbFunction( char* fileName, humanProbTable* humanProbs);

static void
initializeHumanMap( probabilityGrid* humanMap, int sizeX, int sizeY);

static void
initializeBeamLookupTable( beamLookupTable* beamLookup,
			   probabilityGrid* map,
			   distanceScan* scan);

static float 
bayesianUpdate( float prior, float prob);

static int
nextField(float x, float deltaX);

static void
integrateMap( probabilityGrid* global, probabilityGrid* currentProbs);

static void
shiftPositions( float x, float y, 
		float dx, float dy, 
		float cosRot, float sinRot,
		float resolution, int* shiftedX, int* shiftedY);


/* ----------------------------------------------------------------------
 * ----------------------------------------------------------------------
 * Global functions.
 * ----------------------------------------------------------------------
 * ---------------------------------------------------------------------- */

#define SCAN_MAP_SIZE 100
#define GLOBAL_MAP_SIZE 200

void
initializeHumanMapping()
{
  if ( ! readHumanProbFunction("laser-human-detection", &humanProbs))
    exit(-1);

  initializeHumanMap( &humanMap, GLOBAL_MAP_SIZE, GLOBAL_MAP_SIZE);

  fprintf(stderr, "done");

  if (0) displayHumanProbFunction( humanProbs);

  return;
}



void
integrateLaserScan( distanceScan* laserScan, humanProbTable* humanProbs,
		    probabilityGrid* map)
{
  if ( laserPositionKnown) {
    
    int x, y, beamIndex, beam;
    static probabilityGrid *currentMap;
    static beamLookupTable beamLookup;
    static int* measuredDist;
    static int* obstacleDist;
    static int firstTime = TRUE;

    static gridWindow *globalWin, *currentWin;
    realPosition rpos = laserScan->frontRealMap;

    float numberOfBeamDiscretizationsPerGrid = 
      map->resolution / laserScan->distanceResolution;
    
    setTimer(0);
    
    if ( firstTime) {
      firstTime = FALSE;

      map->refPos.x = laserScan->frontRealMap.x;
      map->refPos.y = laserScan->frontRealMap.y;
      
      currentMap = (probabilityGrid*) malloc( sizeof(probabilityGrid));;
      initializeHumanMap( currentMap, SCAN_MAP_SIZE, SCAN_MAP_SIZE);
      
      initializeBeamLookupTable( &beamLookup, currentMap, laserScan);
      
      measuredDist = (int*) allocate1D( laserScan->numberOfReadings, INT);
      obstacleDist = (int*) allocate1D( laserScan->numberOfReadings, INT);
      
#define DISPLAY
#ifdef DISPLAY
      fprintf(stderr, "done\n");
      if (0) getchar();
      globalWin = createMapWindow( "global", map->sizeX, map->sizeY, 2);
      currentWin = createMapWindow( "current", currentMap->sizeX, currentMap->sizeY, 4);
#endif
    }
    
    /* Set the position of the robot. */
    currentMap->refPos.x = laserScan->frontRealMap.x;
    currentMap->refPos.y = laserScan->frontRealMap.y;
    
    /* Initialize some values for fast lookup. */
    for ( beam = 0; beam < laserScan->numberOfReadings; beam++) {
      
      gridPosition centerInGlobalMap = 
	indexIsOfFrontLaser(beam) ? frontGrid : rearGrid;
      
      measuredDist[beam] = laserScan->reading[beam].feature;
      obstacleDist[beam] = expectedFeature( centerInGlobalMap, 
					    laserScan->reading[beam].rot);
    }
    
    for ( x = 0; x < currentMap->sizeX; x++)
      for ( y = 0; y < currentMap->sizeY; y++) {
	
	float accumulatedProb = 0.0;
	int beam = beamLookup.beam[x][y];
	
	for ( beamIndex = beamLookup.minIndex[x][y];
	      beamIndex <= beamLookup.maxIndex[x][y]; 
	      beamIndex++) 
	  
	  accumulatedProb += 
	    humanProbs->probs[beamIndex][measuredDist[beam]][obstacleDist[beam]];
	
	accumulatedProb = accumulatedProb / beamLookup.numberOfIndices[x][y] 
	  * numberOfBeamDiscretizationsPerGrid;
	
	currentMap->probs[x][y] = accumulatedProb; 
      }

#ifdef DISPLAY
    displayMapWindow( currentMap, currentWin, NULL);
    if (0) getchar();
#endif

    
    /* Now we must integrate both the current map into the long term map. */
    fprintf( stderr, "time: %f ", timeExpired(0));
    integrateMap( currentMap, map);
    
    /* The result of integration is now stored in currentMap. Swap 
     * both maps. */

#ifdef DISPLAY
    displayMapWindow( map, globalWin, &rpos);
    if (0) displayMapWindow( currentMap, currentWin, NULL);
    if (0) getchar();
#endif
  }
  fprintf( stderr, " %f\n", timeExpired(0));
}

#define HUMAN_PRIOR 0.001
#define PRIOR_WEIGHT 0.0
#define PROBABILITY_WEIGHT 0.9


static float 
bayesianUpdate( float prior, float prob)
{
  float posterior, likelihood, odds;

  /*
   * First, weight the likelihood by DECAY_NEW;
   */
  float weightedProb = (prob - 0.5) * PROBABILITY_WEIGHT + 0.5;
    
  /* 
   * Decay MAP likelihood value by WEIGHT_OLD
   */
  
  likelihood = (prior - 0.5) * PRIOR_WEIGHT + 0.5;

  /*
   * compute new odds using Bayes' rule 
   */
    
  odds = likelihood / (1.0 - likelihood) * weightedProb / (1.0 - weightedProb);
    
  /*
   * And reextract the posterior likelihood 
   */
  
  posterior = odds / (odds + 1.0);
  if (0) fprintf( stderr, "%f %f %f %f %f --> %f\n", prior, prob, weightedProb, likelihood, odds, posterior);
  return posterior;
}



/* To integrate the two maps we must consider the motion of the robot between
 * the two maps. This is done by shifting the center point of the maps and
 * then rotating the other cells around this center. */
static void
integrateMap( probabilityGrid* currentProbs, probabilityGrid* global)
{
  int x, y;
  float cosRot, sinRot;
  float xOffset, yOffset;
  int shiftedX, shiftedY;

  float centerX = currentProbs->refPos.x;
  float centerY = currentProbs->refPos.y;

  cosRot = cos( currentProbs->refPos.rot);
  sinRot = sin( currentProbs->refPos.rot);

  fprintf(stderr, "rot %f\n", Rad2Deg(currentProbs->refPos.rot));

  for ( x = 0, xOffset = - currentProbs->resolution * ((float) currentProbs->sizeX * 0.5);
	x < currentProbs->sizeX; 
	x++, xOffset += currentProbs->resolution) {
    for ( y = 0, yOffset = - currentProbs->resolution * ((float) currentProbs->sizeY * 0.5);
	  y < currentProbs->sizeY; 
	  y++, yOffset += currentProbs->resolution) {
      
      shiftPositions( centerX, centerY, xOffset, yOffset, sinRot, cosRot, 
		      global->resolution, &shiftedX, &shiftedY);
      
      if (0) if ( x == 0 && y == 0 || x == 99 && y == 0 || x == 0 && y == 99 || x == 99 && y == 99 || x == 50 && y == 50) {
	currentProbs->probs[x][y] = 0.99;
	if (1) fprintf(stderr, "%d %d   ---> %d %d\n", x, y, shiftedX, shiftedY);
	if (1) fprintf(stderr, "%f %f   ---> %f %f\n", xOffset, yOffset, centerX, centerY);
      }
      if ( shiftedX > 0 && shiftedX < global->sizeX 
	   && shiftedX > 0 && shiftedY < global->sizeY) {
	global->probs[shiftedX][shiftedY] = 
	  bayesianUpdate( global->probs[shiftedX][shiftedY], currentProbs->probs[x][y]);
      }
    }
  }
}


static int
readHumanProbFunction( char* fileName, humanProbTable* humanProbs)
{
  FILE* fp;
  int human, distance, obstacle;
  float tmp;
  float minProb = 1e5;  
  float maxProb = 0.0;

  if ((fp = fopen(fileName, "r")) == NULL) {
    fprintf(stderr, "Cannot open %s.\n", fileName);
    return FALSE;
  }
  else
    fprintf(stderr, "Read  human probabilities from %s ", fileName);
  
  fscanf(fp, "pOfDhGivenSO %d %f\n", 
	 &(humanProbs->numberOfDistances), &(humanProbs->deltaDist));
  fprintf(stderr, "(%d %f) .... ", 
	  humanProbs->numberOfDistances, humanProbs->deltaDist);

  humanProbs->probs = (float***) allocate3D( humanProbs->numberOfDistances, 
					     humanProbs->numberOfDistances, 
					     humanProbs->numberOfDistances, FLOAT);

  for ( human = 0; human < humanProbs->numberOfDistances; human++)
    for ( distance = 0; distance < humanProbs->numberOfDistances; distance ++) 
      for ( obstacle = 0; obstacle < humanProbs->numberOfDistances; obstacle++) {
	fscanf(fp, "%f ", &tmp);
	humanProbs->probs[human][distance][obstacle] = tmp;
	if ( tmp < minProb)
	  minProb = tmp;
	if ( tmp > maxProb)
	  maxProb = tmp;
      }

#define MIN_PROB 0.001
#define MAX_PROB 0.55

  /* Nowe norm the values. */
  for ( human = 0; human < humanProbs->numberOfDistances; human++)
    for ( distance = 0; distance < humanProbs->numberOfDistances; distance ++) 
      for ( obstacle = 0; obstacle < humanProbs->numberOfDistances; obstacle++) {
	humanProbs->probs[human][distance][obstacle] = 
	  fNorm( humanProbs->probs[human][distance][obstacle],
		 minProb, maxProb, 
		 MIN_PROB, MAX_PROB);
      }
  
  fclose(fp);
  fprintf( stderr, "done.\n");

  return TRUE;
}



static void
initializeHumanMap( probabilityGrid* map, int sizeX, int sizeY)
{
  int x, y;
  
  map->sizeX = sizeX;
  map->sizeY = sizeY;
  map->resolution = MAP_RESOLUTION;
  
  map->probs = (float**) allocate2D( map->sizeX, map->sizeY, FLOAT);

  for ( x = 0; x < map->sizeX; x++)
    for ( y = 0; y < map->sizeY; y++)
      map->probs[x][y] = HUMAN_PRIOR;
}


static point
realPositionOfMapPosition( int x, int y, float resolution)
{
  point pt;

  pt.x = x * resolution + 0.5 * resolution;
  pt.y = y * resolution + 0.5 * resolution;

  return pt;
}

static float
angleBetweenPositions( point pt1, point pt2)
{
  pt2.x -= pt1.x;
  pt2.y -= pt1.y;
  
  if (fabs( pt2.x) < 0.001 && fabs( pt2.y) < 0.001)
    return(0.0);
  else 
    return normedAngle((float) atan2((double) pt2.y, (double) pt2.x));
}


static float
distanceBetweenPositions( point pt1, point pt2)
{
  return sqrt( Sqr( pt1.x - pt2.x) + Sqr( pt1.y - pt2.y));
}


static int 
closestBeam( float angle, distanceScan* scan)
{
  int beam,  closestBeam = 0;
  float minDist = DEG_360, dist;

  for ( beam = 0; beam < scan->numberOfReadings; beam++) {
    if ( (dist = fabs( scan->reading[beam].rot - angle)) < minDist) {
      minDist = dist;
      closestBeam = beam;
    }
  }
  return closestBeam;
}

static void
determineIndices( int* min, int* max, int* size,
		  int addSize, int maxIndex,
		  float dist, 
		  float distanceResolution)
{
  int hit = iMin( maxIndex, dist / distanceResolution + 0.5);

  *min = iMax( hit - addSize, 0);
  *max = iMin( hit + addSize, maxIndex);
  *size = *max - *min + 1;
}


static void
initializeBeamLookupTable( beamLookupTable* beamLookup, 
			   probabilityGrid* map,
			   distanceScan* scan)
{
  int x, y;
  int sizeX = map->sizeX;
  int sizeY = map->sizeY;
  
  int intervalSize = map->resolution / scan->distanceResolution;
  point center = realPositionOfMapPosition( sizeX / 2, sizeY / 2, map->resolution);

  fprintf(stderr, "Preprocess beam lookup table ...");

  beamLookup->sizeX = sizeX;
  beamLookup->sizeY = sizeY;

  beamLookup->beam = (int**) allocate2D( sizeX, sizeY, INT);
  beamLookup->minIndex = (int**) allocate2D( sizeX, sizeY, INT);
  beamLookup->maxIndex = (int**) allocate2D( sizeX, sizeY, INT);
  beamLookup->numberOfIndices = (int**) allocate2D( sizeX, sizeY, INT);

  for ( x = 0; x < sizeX; x++) {
    for ( y = 0; y < sizeY; y++) {
      point pos = realPositionOfMapPosition( x, y, map->resolution);
      float angle = angleBetweenPositions( center, pos);
      float dist = distanceBetweenPositions( center, pos);

      beamLookup->beam[x][y] = closestBeam( angle, scan);
      
      determineIndices( &(beamLookup->minIndex[x][y]),
			&(beamLookup->maxIndex[x][y]), 
			&(beamLookup->numberOfIndices[x][y]), 
			intervalSize,
			scan->numberOfFeatures-1,
			dist, scan->distanceResolution);      
    }
  }
  fprintf( stderr, "done.\n");
}

static int
nextField(float x, float deltaX)
{
  if (deltaX > 0)
    return (int) round(x + 0.5);
  else if (deltaX == 0.0)
    return (int) round( x - 0.5);
  else
    return (int) (--x);
}


static void
shiftPositions( float x, float y, 
		float dx, float dy, 
		float sinRot, float cosRot,
		float resolution, int* shiftedX, int* shiftedY)
{
  if (0) fprintf( stderr, "%f %f  --> %f %f #s\n", x + dx, y + dy,
	   x + dx * cosRot - dy * sinRot,
	   y + dx * sinRot + dy * cosRot);
  *shiftedX = (x + dx * cosRot - dy * sinRot) / resolution;
  *shiftedY = (y + dx * sinRot + dy * cosRot) / resolution;
}


