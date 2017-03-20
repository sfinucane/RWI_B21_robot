
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/obstacleServerTools.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:25:05 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: obstacleServerTools.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.7  1999/07/23 19:46:37  fox
 * Added ability to read grid maps for the obstacle server.
 *
 * Revision 1.6  1998/08/26 23:23:45  fox
 * Final version before I changed exception handling.
 *
 * Revision 1.5  1997/06/03 11:49:11  fox
 * Museum version.
 *
 * Revision 1.4  1997/04/17 09:19:39  fox
 * Minor changes.
 *
 * Revision 1.3  1997/04/17 09:16:20  fox
 * Added timeout for laser devices --> colliServer only needs 30% cpu.
 *
 * Revision 1.2  1997/03/28 03:48:30  tyson
 * finding .ini files and minor stuff
 *
 * Revision 1.1  1997/03/26 18:42:06  fox
 * Incorporated external obstacle information into the collision avoidance.
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>

#include "obstacleServer.h"
#include "localize.h"
#include "fakeSensors.hh"

#define BEAM_LENGTH 500.0
#define MAX_STRING_LENGTH 255

typedef struct{
  int   maxDistanceIndex;
  float distanceResolution;
  int   initialized;
  int** xOffset;
  int** yOffset;
  int*  numberOfOffsets;
  int** offsetDistIndex;
  float angleToIndex;
  int   numberOfAngles;
} distanceLookup;

probabilityGrid gridMap;

#define RAD_TO_DEG_CONV 57.29578

static int
expectedDistanceGridMap( probabilityGrid* map,
			 int centerX, int centerY, int rot,
			 float distanceResolution,
			 int maxDistance);

/***************************************************************************
 * Some auxiliary functions. 
 ***************************************************************************/
float
DegToRad(float x)
{
  return x * 0.017453293;
}


float
RadToDeg(float x)
{
  return x * 57.29578;
}

float
fSqr( float x)
{
  return x * x;
}

float 
normalizedAngle(float angle)
{
  while (angle < 0.0)
    (angle) += DEG_360;
  
  while (angle >= DEG_360)
    (angle) -= DEG_360;

  return(angle);
}


/***************************************************************************
 * Checks wether an obstacle is in the given direction. If so the
 * collision point with the obstacle is stored in the point. *
 ***************************************************************************/
int
obstacleInDirection( realPosition mapPos,
		     realPosition robPos,
		     float relativeAngle,
		     obstaclePoint* point,
		     correctionParameter* corr)
{
  
#define MAX_RANGE 600.0
  
  float absoluteAngle = mapPos.rot + relativeAngle;
  float distance;
  
#ifdef USE_SIM_MAP
#define OLD
#ifdef OLD
  /* Compute the end point of the 'beam'. */
  
  float endX = mapPos.x + BEAM_LENGTH * cos( absoluteAngle);
  float endY = mapPos.y + BEAM_LENGTH * sin( absoluteAngle);
  
  if ( getDistance( mapPos.x, mapPos.y, 35.0, endX, endY, &distance)) { 
#else
    if ( getDistance( mapPos.x, mapPos.y, 35.0, 
		      cos( absoluteAngle), sin( absoluteAngle), 0, 
		      MAX_RANGE, &distance)) { 
      
#endif
      /* Set the corresponding point relative to the robot's current position. */
      point->x = robPos.x + distance * cos( robPos.rot + relativeAngle);
      point->y = robPos.y + distance * sin( robPos.rot + relativeAngle);
      
#ifdef SERVER_debug
      fprintf( stderr, "dist %f %f %f  --> %f ( %f %f %f)  --> %d %d\n",
	       mapPos.x, mapPos.y, RadToDeg(mapPos.rot),
	     distance,
	       robPos.x, robPos.y, robPos.rot,
	       point->x, point->y);
      
#endif
      return TRUE;
    }
    else
      return FALSE;
#endif    
}
  
  
  
int
createScanInGrid( probabilityGrid* map,
		  realPosition mapPos,
		  realPosition robPos,
		  obstaclePoint* point,
		  int sizeOfScan)
{

#define MAX_DISTANCE_INDEX 100
#define DISTANCE_RESOLUTION 5.0
  
  int beam;
  float radPerBeam = DegToRad(360 / sizeOfScan);
  int numberOfPoints = 0;

  for ( beam = 0; beam < sizeOfScan; beam++) {
    
    float relativeAngle = beam * radPerBeam;
    int centerX = (mapPos.x  - map->offsetX) / map->resolution;
    int centerY = (mapPos.y  - map->offsetY) / map->resolution;
    int rot     = (int) RadToDeg( normalizedAngle( mapPos.rot + relativeAngle)) % 360;

    int distanceIndex = 
      expectedDistanceGridMap( map,
			       centerX, centerY, rot,
			       DISTANCE_RESOLUTION,
			       MAX_DISTANCE_INDEX);

    if ( distanceIndex < (MAX_DISTANCE_INDEX - 1)) {
      
      float distance = distanceIndex * DISTANCE_RESOLUTION;
      
#ifdef TEST
      fprintf(stderr, "%d: %d %d %d   --> %f %f\n", beam,
	      centerX, centerY, rot,
	      distance, RadToDeg( relativeAngle));
#endif
      
      /* Set the corresponding point relative to the robot's current position. */
      point[numberOfPoints].x = robPos.x + distance * cos( robPos.rot + relativeAngle);
      point[numberOfPoints].y = robPos.y + distance * sin( robPos.rot + relativeAngle);
      numberOfPoints++;
    }
  }
  return numberOfPoints;
  
}
     
static int
shrinkMap(probabilityGrid *m)
{
  register int x,y;
  register int empty = TRUE;

  int min_x,max_x,min_y,max_y;

  int oldsizeX;

  for (min_x = 0; min_x < m->sizeX && empty; min_x++)
    for (y=0; y < m->sizeY; y++)
      empty = empty && (m->prob[min_x][y] == m->unknown);
  min_x--;

  empty = TRUE;
  for (max_x = m->sizeX-1; max_x >=0 && empty; max_x--)
    for (y=0; y < m->sizeY; y++)
      empty = empty && (m->prob[max_x][y] == m->unknown);
  max_x++;
  
  empty = TRUE;
  for (min_y = 0; min_y < m->sizeY && empty; min_y++)
    for (x=0; x < m->sizeX; x++)
      empty = empty && (m->prob[x][min_y] == m->unknown);
  min_y--;
    
  empty = TRUE;
  for (max_y = m->sizeY-1; max_y >=0 && empty; max_y--)
    for (x=0; x < m->sizeX; x++)
      empty = empty && (m->prob[x][max_y] == m->unknown);
  max_y++;
  
  oldsizeX = m->sizeX;
  
  m->sizeX = max_x - min_x + 1;
  m->sizeY = max_y - min_y + 1;

  m->shiftedX = min_x;
  m->shiftedY = min_y;
  
  fprintf(stderr,
	  "# Map shrinked: origin %d %d, size %d %d, resolution %d\n",
	  min_x, min_y,
	  m->sizeX, m->sizeY,
	  m->resolution);

  for(x=0; x<m->sizeX; x++)
    for(y=0; y<m->sizeY; y++)
      m->prob[x][y] = m->prob[x+min_x][y+min_y];

  
  /*  m->offsetX += m->resolution * min_x;
  m->offsetY += m->resolution * min_y; */
  m->offsetX = m->offsetY = 0;
  m->maxRealX = m->offsetX + m->sizeX * m->resolution;
  m->maxRealY = m->offsetY + m->sizeY * m->resolution;
  
  


  return(1);
  
}


int
readProbabilityMap(char *fileName, probabilityGrid *m)
  {
   int x,y;
   float temp;
   char line[MAX_STRING_LENGTH];
   FILE *fp;

   if ((fp = fopen(fileName, "rt")) == NULL) {
     fprintf(stderr,"# Could not open file %s\n", fileName);
     return FALSE;
   }

   fprintf(stderr, "# Reading grid map from %s\n", fileName);
   
   /* seek header for map data in rhino-Type file,
      analyze robot specifications */
   while ((fgets(line,MAX_STRING_LENGTH,fp) != NULL)
	  && (strncmp("global_map[0]", line , 13) != 0)){
     if (strncmp(line,"robot_specifications->resolution",32) == 0) {
	 if (sscanf(&line[32],"%d",&(m->resolution)) != 0 ) {
	 }
     }
     if (strncmp(line,"robot_specifications->autoshifted_x",35) == 0) {
       if (sscanf(&line[35],"%g",&(m->offsetX)) != 0 ) {
	 m->offsetX = m->offsetX;
       }
     }
     if (strncmp(line,"robot_specifications->autoshifted_y",35) == 0) {
       if (sscanf(&line[35],"%g",&(m->offsetY)) != 0 ) {
	 m->offsetY = m->offsetY;
       }
     }
   }
   if (sscanf (line,"global_map[0]: %d %d",&m->sizeY, &m->sizeX)
       == EOF) {
     fprintf(stderr,"ERROR: corrupted file %s\n",fileName);
     fclose(fp);
     return FALSE;
   }
   
   
   m->origsizeX = m->sizeX;
   m->origsizeY = m->sizeY;
   m->offsetX = m->offsetY = 0.0;
   m->unknown = -1.0;

   
   m->prob = (float**) malloc(m->sizeX * sizeof(float*));
   for (x = 0; x < m->sizeX; x++)
     m->prob[x] = (float*) malloc(m->sizeY * sizeof(float));

   
   if (m->prob == (float**) NULL){
      fprintf(stderr,
              "ERROR: Not enough memory for loading map %s\n", fileName);
      fclose(fp);
      return FALSE;
   }
   
   for (x=0;x<m->sizeX; x++)
     for (y=0;y<m->sizeY; y++) {
	fscanf(fp,"%e",&temp);
	if (temp >=  0.0 && temp <= 1.0)
	  m->prob[x][y] = 1.0 - (float) (temp);
	else
	  m->prob[x][y] = m->unknown;
     }

   shrinkMap(m);
   fprintf(stderr, "# done\n");

   return TRUE;
}



#define NUMBER_OF_OFFSET_ANGLES 360
#define ANGLE_TO_INDEX 1

static int
expectedDistanceGridMap( probabilityGrid* map,
			 int centerX, int centerY, int rot,
			 float distanceResolution,
			 int maxDistance)
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

    int x;
    int z;
    float angleStep = (M_PI + M_PI) / NUMBER_OF_OFFSET_ANGLES;
    int maxNumberOfDifferentSteps = 2 * maxDistance * distanceResolution / map->resolution + 1;
    firstTime = FALSE;

    xOffset = (int**) malloc( NUMBER_OF_OFFSET_ANGLES * sizeof(int));
    yOffset = (int**) malloc( NUMBER_OF_OFFSET_ANGLES * sizeof(int));
    offsetFeature = (int**) malloc( NUMBER_OF_OFFSET_ANGLES * sizeof(float));
    numberOfOffsets = (int*) malloc( NUMBER_OF_OFFSET_ANGLES * sizeof(int));
    
    for ( x = 0; x < NUMBER_OF_OFFSET_ANGLES; x++) {
      xOffset[x] = (int*) malloc( maxNumberOfDifferentSteps * sizeof(int));
      yOffset[x] = (int*) malloc( maxNumberOfDifferentSteps * sizeof(int));
      offsetFeature[x] = (int*) malloc( maxNumberOfDifferentSteps * sizeof(int));
    }
    
    /* For each precompute the distances along the beam. We only increment the step
     * if a new grid cell in the map is reached. Thus, for each angle and step we
     * also preprocess the resulting distance. */
    for ( z = 0; z < NUMBER_OF_OFFSET_ANGLES; z++) {
      
      float xStep = cos( (float) z * angleStep) * distanceResolution;
      float yStep = sin( (float) z * angleStep) * distanceResolution;
      int step, offsetStep;
      
      xOffset[z][0] = 0;
      yOffset[z][0] = 0;
      offsetFeature[z][0] = 0;

      /* Store the indices in the map if going along the beam. */
      for ( step = 0, offsetStep = 0; step < maxDistance; step++) {
	
	int nextX =
	  (float) (0.5 + step * xStep) / map->resolution;
	int nextY =
	  (float) (0.5 + step * yStep) / map->resolution;

#define VERBOSE 0
	if (VERBOSE) fprintf(stderr, "%d %d %d\n", step, nextX, nextY);

	/* Only increment the offset if a different cell of the map is hit! */
	if ( (nextX != xOffset[z][offsetStep]) || (nextY != yOffset[z][offsetStep])) {
	  if (VERBOSE) fprintf(stderr, "%d #next\n", offsetStep);
	  offsetStep++;
	  xOffset[z][offsetStep] = nextX;
	  yOffset[z][offsetStep] = nextY;
	  offsetFeature[z][offsetStep] = step - 1;
	}
      }
      numberOfOffsets[z] = offsetStep + 1;
      if (VERBOSE) fprintf(stderr, "%d %d #steps\n", z / 2, numberOfOffsets[z]);
    }
  }
  
#define OCCUPIED_THRESHOLD 0.8
  
  /* Now step through the offsets and search for the first occupied cell. */
  for ( step = 0; step < numberOfOffsets[angle]; step++) {
    
    int mapX = xOffset[angle][step] + centerX;
    int mapY = yOffset[angle][step] + centerY;
    if (0) fprintf(stderr, "%d: %d %d %f\n", step, mapX, mapY, map->prob[mapX][mapY]);
    if ( mapX >= 0 && mapX < map->sizeX && mapY >= 0 && mapY < map->sizeY) {
      if ( map->prob[mapX][mapY] > OCCUPIED_THRESHOLD)
	return offsetFeature[angle][step];
    }
    else if (0)
      fprintf(stderr, "ok: %d  : %d\n", angle, step);
    else
      return maxDistance - 1;
  }
  return maxDistance - 1;
}

