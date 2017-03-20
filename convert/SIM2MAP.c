
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/convert/SIM2MAP.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:34:28 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: SIM2MAP.c,v $
 * Revision 1.1  2002/09/14 15:34:28  rstone
 * *** empty log message ***
 *
 * Revision 1.15  1997/03/31 21:19:16  wolfram
 * Small improvements, which make it a little bit faster ;-)
 *
 * Revision 1.14  1997/03/21 15:16:04  wolfram
 * SIM2MAP now supports cylinders
 *
 * Revision 1.13  1997/03/11 09:10:54  wolfram
 * SIM2MAP now handles cubes and can dump maps for different vertical regions
 *
 * Revision 1.12  1997/03/11 08:55:33  wolfram
 * Small changes
 *
 * Revision 1.11  1997/01/27 08:31:50  wolfram
 * Maps now have the correct border
 *
 * Revision 1.10  1997/01/23 20:55:23  wolfram
 * Fixed a bug
 *
 * Revision 1.9  1997/01/23 19:43:54  wolfram
 * Cells are now interpolated
 *
 * Revision 1.8  1997/01/07 12:25:40  haehnel
 * I hope this is the last modification
 *
 * Revision 1.7  1997/01/02 15:50:37  haehnel
 * i think it's ok now
 *
 * Revision 1.6  1996/12/30 17:26:25  haehnel
 * a small bug
 *
 * Revision 1.5  1996/12/30 17:14:30  haehnel
 * I hope it's ok, now ... (corr-parameters)
 *
 * Revision 1.4  1996/12/30 13:34:30  fox
 * First attempt to set the correction parameters.
 *
 * Revision 1.3  1996/12/30 13:06:03  haehnel
 * Search for the ROBOT ... line in the simulator map
 *
 * Revision 1.2  1996/12/11 17:33:25  wolfram
 * Added correction parameter support
 *
 * Revision 1.1  1996/12/11 16:49:16  wolfram
 * This converter converts simulator maps to map format, first release
 *
 * Revision 1.1.1.1  1996/09/22 16:47:00  rhino
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
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <malloc.h>
#include <values.h>

#include "tcx.h"
#include "rwibase_interface.h"
#include "../localize/general.h"

#define MAX_NUMBER_OF_OBJECTS 2000
#define CUBE 0
#define CYLINDER 1
#define SIMULATOR_MAP_MARK "MAP"
#define CUBE_MARK "CUBE"
#define RECTANGLE_MARK "RECTANGLE"
#define CYLINDER_MARK "CYLINDER"
#define DOOR_MARK "DOOR"
#define SIMULATOR_ROBOT_MARK "ROBOT"
#define MINIMUM_MAP_SIZE 300
#define MAP_SIZE_STEP 50
#define NUMBER_OF_CORNERS 4
#define BUFFLEN 80



typedef struct{
  float centerX;
  float centerY;
  float centerZ;
  float width;
  float depth;
  float height;
  float rot;
} rectangle;


float
fAbs( float x)
{
  return (x >= 0.0 ? x : -x);
}

int
iAbs( int x)
{
  return (x >= 0.0 ? x : -x);
}

float
fMax( float x, float y)
{
 return ( x > y ? x : y);
}

int
iMax( int x, int y)
{
 return ( x > y ? x : y);
}


float
fMin( float x, float y)
{
 return ( x > y ? y : x);
}

int
iMin( int x, int y)
{
 return ( x > y ? y : x);
}


float
fSqr( float x)
{
  return x * x;
}

int
iSqr( int x)
{
  return x * x;
}




void
rotpnt(float angle, float *x, float *y) 
{ 
    float t = *x; 
    *x = *x * cos(angle) - *y * sin(angle); 
    *y = t  * sin(angle) + *y * cos(angle); 
}


void
absolute_points(rectangle *rect, float* inside_x, float* inside_y)
{
  int i;
  inside_x[0] = rect->width/2;
  inside_y[0] = -rect->depth/2;
  rotpnt(rect->rot,&inside_x[0],&inside_y[0]);
  inside_x[1] = rect->width/2;
  inside_y[1] = rect->depth/2;
  rotpnt(rect->rot,&inside_x[1],&inside_y[1]);
  inside_x[2] = -rect->width/2;
  inside_y[2] = rect->depth/2;
  rotpnt(rect->rot,&inside_x[2],&inside_y[2]);
  inside_x[3] = -rect->width/2;
  inside_y[3] = -rect->depth/2;
  rotpnt(rect->rot,&inside_x[3],&inside_y[3]);
  for(i = 0; i < NUMBER_OF_CORNERS; i++) {
    inside_x[i] += rect->centerX;
    inside_y[i] += rect->centerY;
  }
}

int
inhalfplane(float xp, float yp, float x1, float y1, float x2, float y2)
{
    float m,c;
  if(x1 == x2) {
    if(y1 <= y2)
      return (xp <= x1);
    else
      return (xp >= x1);
  }
  else {
    if( x1 < x2 ) {
      m  = (y2 - y1) / (x2 - x1);
      c = y1 - m*x1;
      return (yp >= m*xp+c);
    }
    else {
      m  = (y1 - y2) / (x1 - x2);
      c = y1 - m*x1;
      return (yp <= m*xp+c);
    }
  }
}

int
inside(float xi, float yi, rectangle* rect, float* inside_x, float* inside_y)
{
    int i;
    for(i = 0; i < NUMBER_OF_CORNERS; i++) {
	if(!inhalfplane(xi,yi,
			inside_x[i],inside_y[i],
			inside_x[(i+1)%NUMBER_OF_CORNERS],
			inside_y[(i+1)%NUMBER_OF_CORNERS]))
	    return FALSE;
    }
    return TRUE;
}

void
insertRectangle(probabilityGrid *map, rectangle rect)
{
  register int x, y;
  int minX, maxX, minY, maxY;

  float inside_x[NUMBER_OF_CORNERS];
  float inside_y[NUMBER_OF_CORNERS];
  
  absolute_points( &rect, inside_x, inside_y);

  minX = maxX = inside_x[0];
  minY = maxY = inside_y[0];
  
  for (x = 1; x < NUMBER_OF_CORNERS; x++){
    if (inside_x[x] < minX)
      minX = inside_x[x];
    if (inside_x[x] > maxX)
      maxX = inside_x[x];
    if (inside_y[x] < minY)
      minY = inside_y[x];
    if (inside_y[x] > maxY)
      maxY = inside_y[x];
  }

  minX = iMax(0, minX);
  minX = iMin(minX, map->sizeX - 1);
  maxX = iMax(0, maxX);
  maxX = iMin(maxX, map->sizeX - 1);
  minY = iMax(0, minY);
  minY = iMin(minY, map->sizeY - 1);
  maxY = iMax(0, maxY);
  maxY = iMin(maxY, map->sizeY - 1);

  for (x = minX; x <= maxX; x++)
    for (y = minY; y <= maxY; y++)
      if (inside( (float) x, (float) y, &rect, inside_x, inside_y))
      {
	 map->prob[x][y] = MAXIMUM_PROBABILITY;
      }
}



void
insertCylinder(probabilityGrid *map, rectangle rect)
{
  register int x, y;
  int minX, maxX, minY, maxY;

  minX = rect.centerX - rect.width;
  minY = rect.centerY - rect.width;
  maxX = rect.centerX + rect.width;
  maxY = rect.centerY + rect.width;
  
  minX = iMax(0, minX);
  minX = iMin(minX, map->sizeX - 1);
  maxX = iMax(0, maxX);
  maxX = iMin(maxX, map->sizeX - 1);
  minY = iMax(0, minY);
  minY = iMin(minY, map->sizeY - 1);
  maxY = iMax(0, maxY);
  maxY = iMin(maxY, map->sizeY - 1);

  for (x = minX; x <= maxX; x++)
    {
      float sqrX = fSqr(rect.centerX - x);
      for (y = minY; y <= maxY; y++)
	if (sqrt(sqrX + fSqr(rect.centerY - y)) < rect.width)
	  {
	    map->prob[x][y] = MAXIMUM_PROBABILITY;
	  }
    }
}


static float
pointDistance(float x1, float y1, float x2, float y2)
{
  return sqrt(fSqr(x1 - x2) + fSqr(y1 - y2));
}


static bool
intersection(float from1, float to1, float from2, float to2){
  return from2 <= to1 && from1 <= to2;
}


float
integratedOccupancyProbability(int x, int y, int delta, probabilityGrid *map)
{
  register int i, j;
  int startI, startJ, endI, endJ;

  float prob = 0.0, dist, weightSum = 0.0;
  
  startI = iMax(0, x - delta);
  endI = iMin(map->sizeX, x + delta + 1);

  startJ = iMax(0, y - delta);
  endJ = iMin(map->sizeY, y + delta + 1);


  for (i = startI; i < endI; i++)
    for (j = startJ; j < endJ; j++){
      if (map->prob[i][j] >= 0){
	weightSum += (dist = pointDistance(i,j,x,y));
	prob += dist * map->prob[i][j];
      }
    }
  if (weightSum == 0.0){
     prob = map->prob[x][y];
     fprintf(stderr, "Null: %d %d\n", x, y);
  }
  else
     prob /= weightSum;

  if (prob != -1) {
      if (prob < MINIMUM_PROBABILITY)
	  prob = MINIMUM_PROBABILITY;
      else if (prob > MAXIMUM_PROBABILITY)
	  prob = MAXIMUM_PROBABILITY;
  }
  
  return prob;
}



bool
readSimulatorMap( FILE *fp, 
		  char *mapname,
		  int mapResolution,
		  float additionalObjectSize,
		  float fromHeight,
		  float toHeight,
		  probabilityGrid *m,
		  realPosition *robotPos){
   char line[100];
   int markLength, cnt;
   int found;
   register int x,y;
   float mapX1, mapX2, mapY1, mapY2;
   int delta;
   probabilityGrid tmpMap;
   rectangle rect;
   realPosition pos;
   int type;
   
   
   found = FALSE;
   while ((fgets(line,BUFFLEN,fp) != NULL) && !feof(fp)){
     if (strncmp(line,SIMULATOR_MAP_MARK,strlen(SIMULATOR_MAP_MARK)) == 0){
       markLength = strlen(SIMULATOR_MAP_MARK);
       if (sscanf(&line[markLength],"%f %f %f %f", &mapX1, &mapY1,
		  &mapX2, &mapY2) == 4){
	 fprintf(stderr, "# Map size: (%fx%f) -> (%fx%f)\n", mapX1, mapY1,
		 mapX2, mapY2);
	 found = TRUE;
       }
     }
     if (strncmp(line,SIMULATOR_ROBOT_MARK,
		 strlen(SIMULATOR_ROBOT_MARK)) == 0){
       markLength = strlen(SIMULATOR_ROBOT_MARK);
       if (sscanf(&line[markLength],"%f %f %f", &pos.x, &pos.y,
		  &pos.rot) == 3){
	 fprintf(stderr, "# Simulator robot position: X=%f Y=%f O=%f\n", 
		 pos.x, pos.y, pos.y);
	 *robotPos = pos;
       }
       
     }
   }
       
   if (!found){
      fprintf(stderr, "Error: Map size not found!\n");
      return(FALSE);
   }
   
   tmpMap.sizeX = (int) (mapX2 + 1 - mapX1);
   tmpMap.sizeY = (int) (mapY2 + 1 - mapY1);
   
   tmpMap.prob = (float **)
     malloc(tmpMap.sizeX * sizeof(float *));
   for (x = 0; x < tmpMap.sizeX; x++)
     tmpMap.prob[x] = (float *) malloc(tmpMap.sizeY * sizeof(float));
   
   /* filling everything white */
   
   for (x = 0; x < tmpMap.sizeX; x++)
      for (y = 0; y < tmpMap.sizeY; y++)
	 tmpMap.prob[x][y] = MINIMUM_PROBABILITY;
   
   rewind(fp);
   
   /* scanning for objects */
   
   cnt = 0;

   fprintf(stderr, "# Processing objects ");

   while ((fgets(line,BUFFLEN,fp) != NULL)){
     if (strncmp(line,RECTANGLE_MARK,
		 markLength = strlen(RECTANGLE_MARK)) == 0){
       type = CUBE;
       if (sscanf(&line[markLength],"%f %f %f %f %f",
		  &rect.centerX,
		  &rect.centerY,
		  &rect.width,
		  &rect.depth,
		  &rect.rot) == 5){
	  rect.centerZ = 0.0;
	  rect.height = 300.0;
	  found = TRUE;
       }
       else if (sscanf(&line[markLength],"%f %f %f %f",
		       &rect.centerX,
		       &rect.centerY,
		       &rect.width,
		       &rect.depth) == 4){
	 rect.centerZ = rect.rot = 0.0;
	 rect.height = 300;
	 found = TRUE;
       }
     }
     else if (strncmp(line,CUBE_MARK,
		      markLength = strlen(CUBE_MARK)) == 0){
       type = CUBE;
       if (sscanf(&line[markLength],"%f %f %f %f %f %f %f",
		  &rect.centerX,
		  &rect.centerY,
		  &rect.centerZ,
		  &rect.width,
		  &rect.depth,
		  &rect.height,
		  &rect.rot) == 7){
	 found = TRUE;
       }
       else if (sscanf(&line[markLength],"%f %f %f %f %f %f",
		       &rect.centerX,
		       &rect.centerY,
		       &rect.centerZ,
		       &rect.width,
		       &rect.depth,
		       &rect.height) == 6){
	 rect.rot = 0.0;
	 found = TRUE;
       }
     }
     else if (strncmp(line,CYLINDER_MARK,
		      markLength = strlen(CYLINDER_MARK)) == 0){
       type = CYLINDER;
       if (sscanf(&line[markLength],"%f %f %f %f %f",
		  &rect.centerX,
		  &rect.centerY,
		  &rect.centerZ,
		  &rect.width,
		  &rect.height) == 5){
	 rect.depth = rect.width;
	 rect.rot = 0.0;
	 found = TRUE;

       }
     }

     if (found && intersection(fromHeight, toHeight,
			       rect.centerZ - rect.height * 0.5,
			       rect.centerZ + rect.height * 0.5)){
       fprintf(stderr, ".");
       cnt++;
       rect.centerX -= mapX1;
       rect.centerY -= mapY1;
       rect.width += additionalObjectSize;
       rect.depth += additionalObjectSize;
       switch (type){
       case CYLINDER:
	 insertCylinder(&tmpMap, rect);
	 break;
       case CUBE:
	 insertRectangle(&tmpMap, rect);
	 break;
       }
       found = FALSE;
     }
   }

   fprintf(stderr, "\n");
   fprintf(stderr, "# Found %d objects\n", cnt);
   fprintf(stderr, "# Computing grid map ");
   
   m->sizeX = tmpMap.sizeX / mapResolution;
   m->sizeY = tmpMap.sizeY / mapResolution;
   m->resolution = mapResolution;
   
   m->unknown = -1;
   m->origsizeX = m->sizeX;
   m->origsizeY = m->sizeY;
   
   m->prob = (float **)
     malloc(m->sizeX * sizeof(float *));
   for (x = 0; x < m->sizeX; x++)
     m->prob[x] = (float *) malloc(m->sizeY * sizeof(float));
   
   
   
   if ((mapResolution % 2) == 0)
      delta = mapResolution+1;
   else
      delta = mapResolution;
   delta = (delta - 1) / 2;

   for (x = 0; x < m->sizeX; x++){
     int posX, posY;
     posX = x*mapResolution + delta;
     fprintf(stderr, ".");
     for (y = 0; y < m->sizeY; y++){
       posY = y*mapResolution + delta;
       if ((posX < tmpMap.sizeX) && (posY < tmpMap.sizeY))
	 m->prob[x][y] =
	   integratedOccupancyProbability(posX, posY, delta, &tmpMap);
       else
	 m->prob[x][y] = m->unknown;
     }
   }

   fprintf(stderr, " done\n");

   return(TRUE);
}


void
printUnknown( FILE *fp, int n)
{
   while (n-- > 0)
      fprintf( fp, "-1 ");
}


int
main( int argc, char *argv[] )
{
  FILE *ifp, *ofp;
   
  int argCount;
  int mapResolution;
  float fromHeight, toHeight;
  float additionalObjectBorder;
  probabilityGrid map;
  realPosition robotPos;

  char* inFileName;
  char* outFileName;
   
  argCount = 0;
  if (argc != 7)
    {
      fprintf(stderr,
	      "usage: %s resolution addSize fromHeight toHeight infile outfile\n",
	      argv[0]);
      exit(0);
    }      
  
  argCount++;
  mapResolution = atoi(argv[argCount]);
  if (mapResolution <= 0)
    {
      fprintf(stderr, "Wrong resolution: %s\n", argv[argCount]);
      exit(0);
    }
  else
    fprintf(stderr, "Map resolution: %d\n", mapResolution);

  argCount++;
  additionalObjectBorder = atof(argv[argCount]);
  if (additionalObjectBorder < 0)
    {
      fprintf(stderr, "Wrong border: %s\n", argv[argCount]);
      exit(0);
    }
  else
    fprintf(stderr, "Additional Object Border: %fcm\n",
	    additionalObjectBorder);


  argCount++;
  fromHeight = atof(argv[argCount]);
  if (fromHeight < 0)
    {
      fprintf(stderr, "Wrong fromHeight: %s\n", argv[argCount]);
      exit(0);
    }
  else
    fprintf(stderr, "fromHeight: %f\n",
	    fromHeight);

  argCount++;
  toHeight = atof(argv[argCount]);
  if (toHeight < 0)
    {
      fprintf(stderr, "Wrong toHeight: %s\n", argv[argCount]);
      exit(0);
    }
  else
    fprintf(stderr, "toHeight: %f\n",
	    toHeight);

  argCount++;
  inFileName = argv[argCount];
  if ((ifp = fopen(inFileName,"rt")) == NULL) {
    fprintf(stderr,"ERROR: Could not open simulator file '%s'!\n",
	    inFileName);
    exit(0);
  }
  
  argCount++;
  outFileName = argv[argCount];
  if ((ofp = fopen(outFileName,"wt")) == NULL) {
    fprintf(stderr,"ERROR: Could not open map file '%s'!\n",outFileName);
    exit(0);
  }

  robotPos.x = robotPos.y = robotPos.rot = MAXFLOAT;
    
  if (!readSimulatorMap( ifp, inFileName, mapResolution,
			 additionalObjectBorder, fromHeight, toHeight,
			 &map, &robotPos)){
    fprintf(stderr, "ERROR: Could not read simulator file '%s'!\n",
	    inFileName);
    exit(0);
  }
  else  {
    int globalSizeX, globalSizeY;
    int extendedSizeX, extendedSizeY; 
    int top, bottom, left, right;
    int x, y;

    if (map.sizeX < MINIMUM_MAP_SIZE)
      extendedSizeX = MINIMUM_MAP_SIZE;
    else
      extendedSizeX = ((map.sizeX / 50) + 1) * 50;
	 
    if (map.sizeY < MINIMUM_MAP_SIZE)
      extendedSizeY = MINIMUM_MAP_SIZE;
    else
      extendedSizeY = ((map.sizeY / 50) + 1) * 50;

    top = (extendedSizeY - map.sizeY) / 2;
    bottom = extendedSizeY - top - map.sizeY;
    left = (extendedSizeX - map.sizeX) / 2;
    right = extendedSizeX - left - map.sizeX;
	    
    globalSizeX = extendedSizeX * mapResolution;
    globalSizeY = extendedSizeY * mapResolution;


    /* If there is a robot Position, then I can compute the correction
       parameter ! */
    if( robotPos.rot != MAXFLOAT ) {
      float corr_x, corr_y, rcorr_x, rcorr_y;
      
      corr_x = ((globalSizeX/2.0)+robotPos.x-
		(((extendedSizeX/2)-left)*mapResolution)-ROBOT_INIT_POS_X);
      corr_y = ((globalSizeY/2.0)+robotPos.y-
		(((extendedSizeY/2)-bottom)*mapResolution)-ROBOT_INIT_POS_Y);
      
      rcorr_x = (corr_x+2*ROBOT_INIT_POS_X)/2.0;
      rcorr_y = (corr_y+2*ROBOT_INIT_POS_Y)/2.0;
      
      fprintf( stderr, "*****************************\n" );
      fprintf( stderr, "Map Size: %dx%d\n", map.sizeX, map.sizeY );
      fprintf( stderr, "Extended Map Size: %dx%d\n", extendedSizeX, extendedSizeY );
      fprintf( stderr, "Left: %d\n", left );
      fprintf( stderr, "Right: %d\n", right );
      fprintf( stderr, "Top: %d\n", top );
      fprintf( stderr, "Bottom: %d\n", bottom );
      fprintf( stderr, "Corr - X: %f\n", rcorr_x );
      fprintf( stderr, "Corr - Y: %f\n", rcorr_y );
      fprintf( stderr, "Corr - O: %f\n", 180.0 );
      fprintf( stderr, "*****************************\n" );
      
      fprintf( ofp, "robot_specifications->reposition_robot_initially %d\n", 0);
      fprintf( ofp, "robot_state->correction_type %d\n", 1 );
      fprintf( ofp, "robot_state->correction_parameter_x %f\n", rcorr_x );
      fprintf( ofp, "robot_state->correction_parameter_y %f\n", rcorr_y );
      fprintf( ofp, "robot_state->correction_parameter_angle %f\n", 180.0 );
    }

    fprintf( ofp, "robot_specifications->global_mapsize_x  %d\n", globalSizeY);
    fprintf( ofp, "robot_specifications->global_mapsize_y  %d\n", globalSizeX);
    fprintf( ofp, "robot_specifications->resolution %d\n", mapResolution);
    fprintf( ofp, "global_map[0]: %d %d\n", extendedSizeY, extendedSizeX);
   
    for (x = 0; x < left; x++){
      printUnknown(ofp, extendedSizeY);
      fprintf( ofp, "\n");
    }
   
    for (x = 0; x < map.sizeX; x++){
      printUnknown( ofp, top);
      for (y = 0; y < map.sizeY; y++)
	if (map.prob[x][y] == -1.0)
	  fprintf(ofp, "-1.0 ");
	else
	  fprintf(ofp, "%.3f ", 1-map.prob[x][y]);
      printUnknown( ofp, bottom);
      fprintf(ofp, "\n");
    }
   
    for (x = 0; x < right; x++){
      printUnknown( ofp, extendedSizeY);
      fprintf( ofp, "\n");
    }
  }   
  fclose(ofp);
  exit(0);
}

