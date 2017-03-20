
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/convert/SIM2PNM.c,v $
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
 * $Log: SIM2PNM.c,v $
 * Revision 1.1  2002/09/14 15:34:28  rstone
 * *** empty log message ***
 *
 * Revision 1.4  1997/04/24 21:24:53  wolfram
 * Ooops, fourth parameter of CUBE's is radius and not diameter!
 *
 * Revision 1.3  1997/04/14 08:20:01  wolfram
 * Fixed minor bugs
 *
 * Revision 1.2  1997/04/13 12:19:53  wolfram
 * Improved display of the robot, the program should be faster now
 *
 * Revision 1.1  1997/04/12 20:28:46  wolfram
 * Creates a pnm file out of a simulator file
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
#include "../ezx/EZX11.h"

#define ROBRADIUS 26.5
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
#define EXPONATE_MARK "EXPONAT"
#define EXPONATE_RADIUS 9
#define MAX_NUMBER_OF_EXPONATES 100
#define FONTWIDTH 6
#define FONTHEIGHT 9

typedef struct
{
  int x;
  int y;
} windowPosition;



static int numberDefinition[10][FONTHEIGHT][FONTWIDTH] = 
{
  { /* 0 */
    {0, 0, 1, 1, 0, 0},
    {0, 1, 0, 0, 1, 0},
    {1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1},
    {0, 1, 0, 0, 1, 0},
    {0, 0, 1, 1, 0, 0}
  },
  { /* 1 */
    {0, 0, 0, 1, 0, 0},
    {0, 0, 1, 1, 0, 0},
    {0, 1, 0, 1, 0, 0},
    {0, 0, 0, 1, 0, 0},
    {0, 0, 0, 1, 0, 0},
    {0, 0, 0, 1, 0, 0},
    {0, 0, 0, 1, 0, 0},
    {0, 0, 0, 1, 0, 0},
    {0, 1, 1, 1, 1, 1}
  },
  { /* 2 */
    {0, 1, 1, 1, 1, 0},
    {1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1},
    {0, 0, 0, 0, 0, 1},
    {0, 0, 0, 0, 1, 0},
    {0, 0, 1, 1, 0, 0},
    {0, 1, 0, 0, 0, 0},
    {1, 0, 0, 0, 0, 0},
    {1, 1, 1, 1, 1, 1}
  },
  { /* 3 */
    {1, 1, 1, 1, 1, 1},
    {0, 0, 0, 0, 0, 1},
    {0, 0, 0, 0, 1, 0},
    {0, 0, 0, 1, 0, 0},
    {0, 0, 1, 1, 1, 0},
    {0, 0, 0, 0, 0, 1},
    {0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1},
    {0, 1, 1, 1, 1, 0}
  },
  { /* 4 */
    {0, 0, 0, 0, 1, 0},
    {0, 0, 0, 1, 1, 0},
    {0, 0, 1, 0, 1, 0},
    {0, 1, 0, 0, 1, 0},
    {1, 0, 0, 0, 1, 0},
    {1, 0, 0, 0, 1, 0},
    {1, 1, 1, 1, 1, 1},
    {0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 1, 0}
  },
  { /* 5 */
    {1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 0},
    {1, 0, 0, 0, 0, 0},
    {1, 0, 1, 1, 1, 0},
    {1, 1, 0, 0, 0, 1},
    {0, 0, 0, 0, 0, 1},
    {0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1},
    {0, 1, 1, 1, 1, 0}
  },
  { /* 6 */
    {0, 0, 1, 1, 1, 0},
    {0, 1, 0, 0, 0, 0},
    {1, 0, 0, 0, 0, 0},
    {1, 0, 0, 0, 0, 0},
    {1, 0, 1, 1, 1, 0},
    {1, 1, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1},
    {0, 1, 1, 1, 1, 0}
  },
  { /* 7 */
    {1, 1, 1, 1, 1, 1},
    {0, 0, 0, 0, 0, 1},
    {0, 0, 0, 0, 1, 0},
    {0, 0, 0, 1, 0, 0},
    {0, 0, 0, 1, 0, 0},
    {0, 0, 1, 0, 0, 0},
    {0, 0, 1, 0, 0, 0},
    {0, 1, 0, 0, 0, 0},
    {0, 1, 0, 0, 0, 0}
  },
  { /* 8 */
    {0, 1, 1, 1, 1, 0},
    {1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1},
    {0, 1, 1, 1, 1, 0},
    {1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1},
    {0, 1, 1, 1, 1, 0}
  },
  { /* 9 */
    {0, 1, 1, 1, 1, 0},
    {1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 1, 1},
    {0, 1, 1, 1, 0, 1},
    {0, 0, 0, 0, 0, 1},
    {0, 0, 0, 0, 0, 1},
    {0, 0, 0, 0, 1, 0},
    {0, 1, 1, 1, 0, 0}
  },
};


typedef struct {
  int initialized;
  int sizeX;
  int sizeY;
  int origsizeX;
  int origsizeY;
  int resolution;
  float offsetX;
  float offsetY;  
  float maxRealX;
  float maxRealY;
  int** prob;
} picture;



typedef struct{
  float centerX;
  float centerY;
  float centerZ;
  float width;
  float depth;
  float height;
  float rot;
} rectangle;



typedef struct{
  int number;
  realPosition pos;
} exponateType;


typedef struct{
  int numberOfExponates;
  exponateType exponate[MAX_NUMBER_OF_EXPONATES];
} exponateList;



int
round(float x)
{
  return (int) (x + 0.5);
}

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
    if (angle != 0.0){
      float angleSin = sin(angle);
      float angleCos = cos(angle);
      float t = *x;
      *x = *x * angleCos - *y * angleSin; 
      *y = t  * angleSin + *y * angleCos;
    }
}

void
drawNumber(int n, int x, int y, int color, picture *map)
{
  register int i, j;
  if (n > 9){
    drawNumber(n / 10, x - FONTWIDTH * 0.5, y, color, map);
    drawNumber(n % 10, x + FONTWIDTH * 0.5, y, color, map);
  }
  else {
    int startX = x - FONTWIDTH * 0.5 + 0.5;
    int startY = y + FONTHEIGHT * 0.5 + 0.5;
    
    for (i = 0; i < FONTHEIGHT; i++){
      for (j = 0; j < FONTWIDTH; j++){
	if (numberDefinition[n][i][j]){
	  map->prob[startX+j+1][startY-i] = color;
	}
      }
    }
  }
}


void
absolute_points(rectangle *rect, float* inside_x, float* inside_y)
{
  int i;
  float width = rect->width * 0.5;
  float depth = rect->depth * 0.5;
  
  inside_x[0] = inside_x[1] = width;
  inside_y[0] = inside_y[3] = -depth;
  inside_y[1] = inside_y[2] = depth;
  inside_x[2] = inside_x[3] = -width;
  rotpnt(rect->rot,&inside_x[0],&inside_y[0]);
  rotpnt(rect->rot,&inside_x[1],&inside_y[1]);
  rotpnt(rect->rot,&inside_x[2],&inside_y[2]);
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
    int i,j;
    for(i = 0; i < NUMBER_OF_CORNERS; i++) {
      j=(i+1)%NUMBER_OF_CORNERS;
      if(!inhalfplane(xi, yi,
		      inside_x[i],inside_y[i],
		      inside_x[j],inside_y[j]))
	return FALSE;
    }
    return TRUE;
}

void
insertRectangle(picture *map, rectangle rect, int color)
{
  register int *p, x, y;
  int minX, maxX, minY, maxY;
  int **prob = map->prob;

  if (rect.rot == 0.0){
    float width = rect.width*0.5;
    float depth = rect.depth*0.5;
    minX = iMax(0, rect.centerX - width);
    maxX = iMin(map->sizeX - 1, rect.centerX + width);
    minY = iMax(0, rect.centerY - depth);
    maxY = iMin(map->sizeY - 1, rect.centerY + depth);

    for (x = minX; x <= maxX; x++){
      p = &prob[x][minY];
      for (y = minY; y <= maxY; y++)
	*p++ = color;
    }
  }
  else{
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
    
    for (x = minX; x <= maxX; x++){
      p = &prob[x][minY];
      for (y = minY; y <= maxY; y++){
	if (inside( (float) x, (float) y, &rect, inside_x, inside_y))
	      *p = color;
	p++;
      }
    }
  }
}


void
insertCylinder(picture *map, rectangle rect, int color)
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
	    map->prob[x][y] = color;
	  }
    }
}



void
insertExponate(picture *map, exponateType expo, int color, int resolution)
{
  rectangle rect;
  rect.centerX = expo.pos.x;
  rect.centerY = expo.pos.y;
  rect.width = rect.depth = EXPONATE_RADIUS;
  rect.rot = 0.0;
  insertCylinder(map, rect, color);
  drawNumber(expo.number, rect.centerX, rect.centerY, C_YELLOW, map); 
}

void
insertRobot(picture *map, realPosition pos, int color, int resolution)
{
  realPosition end;
  rectangle rect;
  float radius = ROBRADIUS / resolution;
  
  end.x = pos.x + radius * cos(pos.rot);
  end.y = pos.y + radius * sin(pos.rot);
  end.rot = pos.rot;
  rect.centerX = round(pos.x);
  rect.centerY = round(pos.y);
  rect.width = rect.depth = radius;
  rect.rot = pos.rot;
  insertCylinder(map, rect, color);
  rect.centerX = round((pos.x+end.x) * 0.5);
  rect.centerY = round((pos.y+end.y) * 0.5);
  rect.width = radius;
  rect.depth = 1;
  rect.rot = pos.rot;
  if (fabs(rect.rot) < 0.01){
    rect.centerY += 0.1;
    rect.depth = 0.01;
  }
  insertRectangle(map, rect, C_BLACK);
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


bool
readSimulatorMap( FILE *fp, 
		  char *mapname,
		  int mapResolution,
		  float additionalObjectSize,
		  float fromHeight,
		  float toHeight, bool displayRobot,
		  realPosition robotPos,
		  int resolution,
		  picture *m){
   char line[100];
   int markLength, cnt;
   int found, sizeFound, robotFound;
   register int x,y;
   float mapX1, mapX2, mapY1, mapY2;
   int delta;
   picture tmpMap;
   rectangle rect;
   realPosition pos;
   int type;
   exponateList expoList;

   mapX1 = mapX2 = mapY1 = mapY2 = 0.0;
   
   expoList.numberOfExponates = 0;
   sizeFound = robotFound = FALSE;
   while ((fgets(line,BUFFLEN,fp) != NULL) && !feof(fp)
	  && !sizeFound && !robotFound){
     if (strncmp(line,SIMULATOR_MAP_MARK,strlen(SIMULATOR_MAP_MARK)) == 0){
       markLength = strlen(SIMULATOR_MAP_MARK);
       if (sscanf(&line[markLength],"%f %f %f %f", &mapX1, &mapY1,
		  &mapX2, &mapY2) == 4){
	 fprintf(stderr, "# Map size: (%fx%f) -> (%fx%f)\n", mapX1, mapY1,
		 mapX2, mapY2);
	 sizeFound = TRUE;
       }
     }
     if (strncmp(line,SIMULATOR_ROBOT_MARK,
		 strlen(SIMULATOR_ROBOT_MARK)) == 0){
       markLength = strlen(SIMULATOR_ROBOT_MARK);
       if (sscanf(&line[markLength],"%f %f %f", &pos.x, &pos.y,
		  &pos.rot) == 3){
	 fprintf(stderr, "# Simulator robot position: X=%f Y=%f O=%f\n", 
		 pos.x, pos.y, pos.y);
	 robotFound = TRUE;
	 if (!displayRobot)
	   robotPos = pos;
	 robotPos.x -= mapX1;
	 robotPos.y -= mapY1;
	 robotPos.x /= resolution;
	 robotPos.y /= resolution;
       }
       
     }
   }
       
   if (!sizeFound){
      fprintf(stderr, "Error: Map size not found!\n");
      return(FALSE);
   }

   if (!robotFound && !displayRobot){
     fprintf(stderr, "Error: No robot position found!\n");
     return(FALSE);
   }
   
   m->sizeX = (int) ((mapX2 + 1 - mapX1) / resolution);
   m->sizeY = (int) ((mapY2 + 1 - mapY1) / resolution);
   
   m->prob = (int **)
     malloc(m->sizeX * sizeof(int *));
   for (x = 0; x < m->sizeX; x++)
     m->prob[x] = (int *) malloc(m->sizeY * sizeof(int));
   
   /* filling everything white */
   
   for (x = 0; x < m->sizeX; x++)
      for (y = 0; y < m->sizeY; y++)
	 m->prob[x][y] = C_WHITE;
   
   rewind(fp);
   
   /* scanning for objects */
   
   cnt = 0;
   found = FALSE;
   type = CUBE;
   
   fprintf(stderr, "# Processing objects ...");

   while ((fgets(line,BUFFLEN,fp) != NULL)){
     if (strncmp(line,CUBE_MARK,
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
     else if (strncmp(line, EXPONATE_MARK,
		      markLength = strlen(EXPONATE_MARK)) == 0){
       int n = expoList.numberOfExponates;
       if ( n < MAX_NUMBER_OF_EXPONATES
	    && sscanf(&line[markLength],"%d %f %f",
		      &expoList.exponate[n].number,
		      &expoList.exponate[n].pos.x,
		      &expoList.exponate[n].pos.y) == 3){
	 expoList.exponate[n].pos.rot = 0;
	 expoList.exponate[n].pos.x /= resolution;
	 expoList.exponate[n].pos.y /= resolution;
	 expoList.numberOfExponates++;
	 expoList.exponate[n].number = expoList.numberOfExponates;
       }
     }
     else if (strncmp(line,RECTANGLE_MARK,
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

     if (found && intersection(fromHeight, toHeight,
			       rect.centerZ - rect.height * 0.5,
			       rect.centerZ + rect.height * 0.5)){
       cnt++;
       rect.centerX -= mapX1;
       rect.centerY -= mapY1;
       rect.centerX /= resolution;
       rect.centerY /= resolution;
       rect.width /= resolution;
       rect.depth /= resolution;
       switch (type){
       case CYLINDER:
	 rect.width += additionalObjectSize;
	 rect.depth += additionalObjectSize;
         insertCylinder(m, rect, C_BLACK); 
	 break;
       case CUBE:
	 rect.width += 2*additionalObjectSize;
	 rect.depth += 2*additionalObjectSize;
	 insertRectangle(m, rect, C_BLACK);
	 break;
       }
       found = FALSE;
     }
   }

   for (x = 0; x < expoList.numberOfExponates; x++){
     insertExponate(m, expoList.exponate[x], C_RED, resolution);
   }

   insertRobot(m, robotPos, C_YELLOW, resolution);

   
   fprintf(stderr, " found %d objects and %d exponates\n",
	   cnt, expoList.numberOfExponates);

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
  FILE *ifp;
   
  int argCount;
  int resolution;
  float fromHeight, toHeight;
  float additionalObjectBorder;
  picture map;
  realPosition robotPos;
  bool displayRobot;

  char* inFileName;
  char* outFileName;

  
  argCount = 0;
  if (argc != 9 && argc != 6)
    {
      fprintf(stderr,
	      "usage: %s resolution addSize fromHeight toHeight <robPosX robPosY robPosRot> inFile\n",
	      argv[0]);
      exit(0);
    }      

  displayRobot = (argc == 9);
  
  argCount++;
  resolution = atoi(argv[argCount]);
  if (resolution <= 0)
    {
      fprintf(stderr, "Wrong resolution: %s\n", argv[argCount]);
      exit(0);
    }
  else
    fprintf(stderr, "Picture resolution: %d\n", resolution);

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


  if (displayRobot){
    argCount++;
    robotPos.x = atof(argv[argCount]);
    if (robotPos.x < 0)
    {
      fprintf(stderr, "Wrong x coordinate: %s\n", argv[argCount]);
      exit(0);
    }
    
    argCount++;
    robotPos.y = atof(argv[argCount]);
    if (robotPos.y < 0)
      {
	fprintf(stderr, "Wrong y coordinate: %s\n", argv[argCount]);
	exit(0);
      }
    argCount++;
    robotPos.rot = atof(argv[argCount]);
    if (robotPos.rot < 0)
      {
	fprintf(stderr, "Wrong rotation: %s\n", argv[argCount]);
	exit(0);
      }
    
    fprintf(stderr, "Robot position: %f %f %f\n",
	    robotPos.x, robotPos.y, robotPos.rot);
  }    
  
  argCount++;
  inFileName = argv[argCount];
  if ((ifp = fopen(inFileName,"rt")) == NULL) {
    fprintf(stderr,"ERROR: Could not open simulator file '%s'!\n",
	    inFileName);
    exit(0);
  }
  

  if (!readSimulatorMap( ifp, inFileName, resolution,
			 additionalObjectBorder, fromHeight, toHeight,
			 displayRobot, robotPos,resolution, 
			 &map)){
    fprintf(stderr, "ERROR: Could not read simulator file '%s'!\n",
	    inFileName);
    fclose(ifp);
    exit(0);
  }
  else  {
    int x, y;
    fclose(ifp);
    printf( "P6\n");
    printf("# %s\n", inFileName);
    printf("%d %d\n", map.sizeX, map.sizeY);
    printf("%d\n", 255);
    for (y = map.sizeY - 1; y >= 0; y--){
      for (x = 0; x < map.sizeX; x++)

	switch (map.prob[x][y]){
	case C_BLACK:
	  putchar(0);putchar(0);putchar(0);
	  break;
	case C_WHITE:
	  putchar(255);putchar(255);putchar(255);
	  break;
	case C_YELLOW:
	  putchar(255);putchar(255);putchar(0);
	  break;
	case C_RED:
	  putchar(255);putchar(0);putchar(0);
	  break;
	default:
	  putchar(255);putchar(255);putchar(255);
	  break;
	}
      
    }
  }
  exit(0);
}


