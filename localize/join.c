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
 ***** Source file:     $Source: 
 *****
 ***** Created by:      $Author: 
 *****
 ***** Revision #:      $Revision: 
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:39 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#define ANGLESTEP 6

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>

#include "EZX11.h"
#include "fakeSensors.hh" 
#ifdef UNIBONN
#include "localize.h"
#include "map.h"
#endif
#include "beeSoftVersion.h"


/************************************************************************
 * Global variables according to robot position.
 ************************************************************************/


/************************************************************************
 * Forward declarations
 ************************************************************************/



/************************************************************************
 * Utility functions
 ************************************************************************/


long
round(double x) {
  return (long) floor(x + 0.5);
}

double
rad2Deg(double x)
{
  return x * 57.29578;
}



bool
intersection(float from1, float to1, float from2, float to2){
  return from2 <= to1 && from1 <= to2;
}


/************************************************************************
 * Graphic stuff
 ************************************************************************/


typedef struct {
  EZXW_p window;
  int startX;
  int startY;
  int sizeX;
  int sizeY;
  int gridSizeX;
  int gridSizeY;
  int gridOffsetX;
  int gridOffsetY;
  int gridResolution;
  int scale;
  int planeType;
  bool firstTimePosition;
  realPosition pos;
}  gridWindow;

typedef struct{
  int x;
  int y;
} windowPosition;


static int
cm2Pixel(gridWindow* win, float cm){
  return round(cm / win->gridResolution * win->scale);
}

void
clearMapWindow(gridWindow *win)
{
  EZX_SetColor(C_WHITE);
  EZX_FillRectangle(win->window, 0, 0, win->sizeX, win->sizeY);
}


static windowPosition
windowPositionOfRealPosition(gridWindow* win, realPosition pos) {

  windowPosition winPos;

  pos.x -= win->gridResolution * win->gridOffsetX;
  pos.y -= win->gridResolution * win->gridOffsetY;
  
  winPos.x = win->startX
    +  round((pos.x / win->gridResolution) * win->scale);
  winPos.y = win->startY
    +  round((win->gridSizeY - (pos.y / win->gridResolution))
		  * win->scale);
  return winPos;
}

static int
gridWindowScale(int sizeX, int sizeY, int minScale) {
    int scaleX = (int) (400 / sizeX);
    int scaleY = (int) (300 / sizeY);

    if (scaleX < minScale)
	scaleX = minScale;
    if (scaleY < minScale)
        scaleY = minScale;

    if (scaleY < scaleX)
	return(scaleY);
    else
	return(scaleX);
}

void
displayRobotAtWindowPositions(gridWindow *win, windowPosition pos1,
			      windowPosition pos2, float robRadius,
			      int col, bool fillRobot){
  
    if (fillRobot) {
      EZX_SetColor(col);
      EZX_FillCircle(win->window, pos1.x, pos1.y,
		     (float) robRadius / (float) win->gridResolution *
		     (float) win->scale);
      EZX_SetColor(C_BLACK);
    }
    else {
      EZX_SetColor(col);
      EZX_DrawCircle(win->window, pos1.x, pos1.y,
		     (float) robRadius / (float) win->gridResolution *
		     (float) win->scale);
    }

    EZX_DrawLine(win->window, pos1.x, pos1.y, pos2.x, pos2.y);

    EZX_Flush();
}

void
displayRobot(gridWindow *win, robot rob, char *name, int col, bool fillRobot) {

    windowPosition pos1, pos2;
    realPosition pos;

    pos1 = windowPositionOfRealPosition( win, rob.pos);
    pos.x = rob.pos.x + (rob.radius-3) * cos(rob.pos.rot);
    pos.y = rob.pos.y + (rob.radius-3) * sin(rob.pos.rot);
    pos.rot = rob.pos.rot;
    
    pos2 = windowPositionOfRealPosition( win, pos);
    displayRobotAtWindowPositions(win, pos1, pos2, rob.radius, col, fillRobot);
    pos.y += ROB_RADIUS + 10;

    pos2 = windowPositionOfRealPosition( win, pos);
    EZX_SetColor(C_RED);
    EZX_SetBackgroundColor(C_WHITE);
    if (pos2.x > win->sizeX*0.8)
      EZX_DrawTextAt(win->window, pos2.x, pos2.y, name, 'r');
    else
      EZX_DrawTextAt(win->window, pos2.x, pos2.y, name, 'l');

    EZX_Flush();
}

gridWindow *
createMapWindow(probabilityGrid *m, char* text, int x, int y, int minScale) {
  /* creates a window under EZX depending on map_size_x, map_size_y and
   scale (size of a pixel) */

    char corner[80];
    gridWindow *mapwin;
    
    mapwin = (gridWindow *) malloc(sizeof(gridWindow));
    mapwin->scale = gridWindowScale(m->sizeX, m->sizeY, minScale);

    mapwin->gridSizeX = m->sizeX;
    mapwin->gridSizeY = m->sizeY;
    mapwin->gridResolution = m->resolution;
    mapwin->gridOffsetX = round(m->offsetX / m->resolution);
    mapwin->gridOffsetY = round(m->offsetY / m->resolution);
    
    mapwin->sizeX = m->sizeX * mapwin->scale;
    mapwin->sizeY = m->sizeY * mapwin->scale;

    mapwin->firstTimePosition = TRUE;

    sprintf(corner,"+%d+%d",x,y);
    
    EZX_NoMotionEvents();
    mapwin->window = EZX_MakeWindow(text,mapwin->sizeX,mapwin->sizeY,corner);
    EZX_SetWindowBackground (mapwin->window,C_BLACK);
    EZX_Flush();

    mapwin->startX = 0;
    mapwin->startY = 0;
    return(mapwin);
}

void
drawSimulatorMapZRange( gridWindow *win,
			simulatorMap *simMap,
			int color,
			int deltaX,
			int deltaY,
			float fromZ,
			float toZ){

  int i;
  windowPosition pos1, pos2;
  realPosition realPos;
  simulatorObject *object = simMap->object;
  
  realPos.rot = 0;
  EZX_SetColor(color);
  for (i = 0; i < simMap->numberOfObjects; i++, object++){
    if (intersection( object->posZ-object->height * 0.5,
		      object->posZ+object->height * 0.5,
		      fromZ,
		      toZ))
      switch (object->type){
      case CYLINDER:
	realPos.x = object->posX;
	realPos.y = object->posY;
	pos1 = windowPositionOfRealPosition( win, realPos);
	realPos.x += object->width;
	pos2 = windowPositionOfRealPosition( win, realPos);
	EZX_FillCircle(win->window, pos1.x + deltaX,
		       pos1.y + deltaY, pos2.x - pos1.x);
	break;
      case CUBE:
	if (object->rot == 0.0){
	  realPos.x = object->posX - object->width * 0.5;
	  realPos.y = object->posY + object->depth * 0.5;
	  pos1 = windowPositionOfRealPosition( win, realPos);
	  realPos.x += object->width;
	  realPos.y -= object->depth;
	  pos2 = windowPositionOfRealPosition( win, realPos);
	  pos2.x -= pos1.x;
	  if (pos2.x <= 0) pos2.x = 1;
	  pos2.y -= pos1.y;
	  if (pos2.y <= 0) pos2.y = 1;
	  EZX_FillRectangle(win->window, pos1.x+deltaX, pos1.y+deltaY,
			    pos2.x, pos2.y);
	}
	else {
	  float cosRot = cos(object->rot); 
	  float sinRot = sin(object->rot); 
	  XPoint point[4];
	  float dx1 = object->width * cosRot;
	  float dx2 = object->depth * sinRot;
	  float dy1 = object->width * sinRot;
	  float dy2 = object->depth * cosRot;

	  /* center */
	  realPos.x = object->posX + deltaX;
	  realPos.y = object->posY + deltaY;
	  /* first point */
	  realPos.x = object->posX + 0.5 * (dx1 + dx2);
	  realPos.y = object->posY + 0.5 * (dy1 - dy2);
	  pos1 = windowPositionOfRealPosition( win, realPos);
	  point[0].x = (short) round(pos1.x + deltaX);
	  point[0].y = (short) round(pos1.y + deltaY);
	  /* third point */
	  point[2].x = - (short) cm2Pixel(win, dx1);
	  point[2].y = (short) cm2Pixel(win, dy1);
	  /* forth point */
	  point[3].x = (short) cm2Pixel(win, dx2);
	  point[3].y = (short) cm2Pixel(win, dy2);
	  /* second point */
	  point[1].x = - point[3].x;
	  point[1].y = - point[3].y;
	  
	  EZX_FillPolygon(win->window, 4, point);
	}
	break;
      }	  
  }
  EZX_Flush();
}


int
coordinateInMap(int x, int y, probabilityGrid m) {
    return (x >= 0 && x < m.sizeX && y >= 0 && y < m.sizeY); 
}

mapProbability
occupancyProbabilityMapUnknown(int x, int y, probabilityGrid m) {
  if (coordinateInMap(x, y, m))
    return m.prob[x][y];
  else
    return m.unknown;
}



mapProbability
occupancyProbabilityMap(int x, int y, probabilityGrid m) {
  mapProbability prob = occupancyProbabilityMapUnknown(x, y, m);
  if (prob == m.unknown)
    prob = m.average;

  return prob;
}


#define COL_UNKNOWN     C_LAWNGREEN           /* color of unknown area      */
#define COL_BLACK       54
#define COL_WHITE       154


int
fieldColor(mapProbability f, mapProbability max, mapProbability min){

  if (max <= min)
    return(COL_BLACK + (COL_WHITE-COL_BLACK)*0.5);
  
  if (f <= min){
    if (f < 0.0)
      return COL_UNKNOWN;
    else
      return COL_WHITE;
  }
  else if (f >= max)
    return COL_BLACK;
  else
    return COL_WHITE - (int) round((double) (f - min) / (max - min)
				   *(COL_WHITE-COL_BLACK));
}



void
displayMapWindow(probabilityGrid *m, gridWindow *mapwin) {
/* plots map in window */

    int x,y;

    probability max, min;
    int winX, winY;
    int endX = mapwin->startX + m->sizeX * mapwin->scale;
    int endY = mapwin->startY + m->sizeY * mapwin->scale;
    max = 0.0;
    min = 1.0;
    for (x = 0; x < m->sizeX; x++)
      for (y = 0; y < m->sizeY; y++){
	if (m->prob[x][y] > max)
	  max = m->prob[x][y];
	if (m->prob[x][y] < min)
	  min = m->prob[x][y];
      }
	
    
/*     EZX_SetColor(COL_UNKNOWN); */
/*     EZX_FillRectangle(mapwin->window,mapwin->startX,mapwin->startY,endX,endY); */
    
    y=m->sizeY;
    for (winY=mapwin->startY; winY <= endY; winY += mapwin->scale){
	y--;
	x=0;
	for (winX=mapwin->startX; winX < endX; winX += mapwin->scale){
	  EZX_SetColor(fieldColor( occupancyProbabilityMap(x,y,*m), max, min));
	  EZX_FillRectangle(mapwin->window,winX,winY,
			    mapwin->scale,mapwin->scale);
	  x++;
	}
      }

    EZX_Flush();

}



#define SIMULATOR_MAP_MARK "MAP"
#define SIMULATOR_ROBOT_MARK "ROBOT"
#define RECTANGLE_MARK "RECTANGLE"
#define CUBE_MARK "CUBE"
#define CYLINDER_MARK "CYLINDER"

bool
readSimulatorMapFile( char *fileName, simulatorMap *simMap)
{

  FILE *fp;
  char line[MAX_STRING_LENGTH];
  int markLength;
  int found;
  realPosition pos;
  simulatorObject simObject;
  bool stop;

  if ((fp = fopen(fileName,"r")) == NULL) {
    fprintf(stderr,"# Could not open file %s\n", fileName);
    return FALSE;
  }

  strcpy(simMap->fileName, fileName);

  fprintf(stderr, "# Reading Simulator map from %s\n", simMap->fileName);
  stop = FALSE;
  while ((fgets(line,MAX_STRING_LENGTH,fp) != NULL) && !stop){
    if(strncmp(SIMULATOR_MAP_MARK, line, strlen(SIMULATOR_MAP_MARK)) == 0){
      stop = TRUE;
    }
  }
  
  if (!stop){
    fprintf(stderr,
	    "# File %s has not simulator map format\n", simMap->fileName);
    fclose(fp);
    return FALSE;
  }
  
  rewind(fp);
  
  found = FALSE;
  while ((fgets(line,MAX_STRING_LENGTH,fp) != NULL) && !feof(fp)){
    if (strncmp(line,SIMULATOR_MAP_MARK,strlen(SIMULATOR_MAP_MARK)) == 0){
      markLength = strlen(SIMULATOR_MAP_MARK);
      if (sscanf(&line[markLength],"%f %f %f %f",
		 &(simMap->fromX), &(simMap->fromY),
		 &(simMap->toX), &(simMap->toY)) == 4){
	fprintf(stderr, "# Map size: (%.2fx%.2f) -> (%.2fx%.2f)\n",
		simMap->fromX, simMap->fromY,
		simMap->toX,  simMap->toY);
	found = TRUE;
      }
    }
    if (strncmp(line,SIMULATOR_ROBOT_MARK,
		strlen(SIMULATOR_ROBOT_MARK)) == 0){
      markLength = strlen(SIMULATOR_ROBOT_MARK);
      if (sscanf(&line[markLength],"%f %f %f", &pos.x, &pos.y,
		 &pos.rot) == 3){
	fprintf(stderr, "# Simulator robot position: X=%.2f Y=%.2f O=%.2f\n", 
		pos.x, pos.y, pos.y);
      }
      
    }
  }
  
  if (!found){
    fprintf(stderr, "Error: Map size not found!\n");
    fclose(fp);
    return(FALSE);
  }

  rewind(fp);
  

  simMap->sizeX = simMap->toX - simMap->fromX;
  simMap->sizeY = simMap->toY - simMap->fromY;
  simMap->sizeZ = 0.0;
  simMap->numberOfObjects = 0;
  
  while ((fgets(line,MAX_STRING_LENGTH,fp) != NULL)){
    found = FALSE;
    if (strncmp(line,RECTANGLE_MARK,
		markLength = strlen(RECTANGLE_MARK)) == 0){
      simObject.type = CUBE;
      if (sscanf(&line[markLength],"%f %f %f %f %f", &simObject.posX,
		 &simObject.posY, &simObject.width, &simObject.depth,
		 &simObject.rot) == 5){
	simObject.height = MAXFLOAT;
	simObject.posZ = 0.0;
	found = TRUE;
      }
      else if (sscanf(&line[markLength],"%f %f %f %f", &simObject.posX,
		      &simObject.posY,
		      &simObject.width,
		      &simObject.depth) == 4){
	simObject.height = MAXFLOAT;
	simObject.posZ = 0.0;
	simObject.rot = 0.0;
	found = TRUE;
      }
    }
    
    else if (strncmp(line,CUBE_MARK,
		     markLength = strlen(CUBE_MARK)) == 0){
      simObject.type = CUBE;
      if (sscanf(&line[markLength],"%f %f %f %f %f %f %f",
		 &simObject.posX,
		 &simObject.posY,
		 &simObject.posZ,
		 &simObject.width,
		 &simObject.depth,
		 &simObject.height,
		 &simObject.rot) == 7){
	found = TRUE;
      }
      else if (sscanf(&line[markLength],"%f %f %f %f %f %f",
		      &simObject.posX,
		      &simObject.posY,
		      &simObject.posZ,
		      &simObject.width,
		      &simObject.depth,
		      &simObject.height) == 6){
	simObject.rot = 0.0;
	found = TRUE;
      }
    }
    else if (strncmp(line,CYLINDER_MARK,
		     markLength = strlen(CYLINDER_MARK)) == 0){
      simObject.type = CYLINDER;
      if (sscanf(&line[markLength],"%f %f %f %f %f",
		 &simObject.posX,
		 &simObject.posY,
		 &simObject.posZ,
		 &simObject.width,
		 &simObject.height) == 5){
	simObject.depth = simObject.width;
	simObject.rot = 0.0;
	found = TRUE;
	
      }
    }
    
    if (found){
      simObject.posX -= simMap->fromX;
      simObject.posY -= simMap->fromY;
      simMap->object[simMap->numberOfObjects] = simObject;
      simMap->numberOfObjects++;
    }
  }
     
  fprintf(stderr, "# Found %d objects\n", simMap->numberOfObjects);
  fclose(fp);
  simMap->initialized = TRUE;
  return TRUE;
}

void
convolveProbX(probability **prob, int sizeX, int sizeY)
{
  int x, y;
  probability *tmp = malloc(sizeX * sizeof(probability));

  for (y = 0; y < sizeY; y++){
    for (x = 1; x < sizeX - 1; x++)
      tmp[x] = (prob[x-1][y] + prob[x+1][y]) * 0.1
	+ prob[x][y] * 0.8;
    for (x = 1; x < sizeX - 1; x++)
      prob[x][y] = tmp[x];
  }
  free(tmp);
}




void
convolveProbY(probability **prob, int sizeX, int sizeY)
{
  int x, y;
  probability *tmp = malloc(sizeY * sizeof(probability));
  
  for (x = 0; x < sizeX; x++){
    for (y = 1; y < sizeY-1; y++)
      tmp[y] = (prob[x][y-1] + prob[x][y+1]) * 0.1
	+ prob[x][y] * 0.8;
    for (y = 1; y < sizeY-1; y++)
      prob[x][y] = tmp[y];
  }
  free(tmp);
}



void
convolveMap(probabilityGrid map)
{
  int i;
  for (i = 0; i < 3; i++) {
    convolveProbX(map.prob, map.sizeX, map.sizeY);
    convolveProbY(map.prob, map.sizeX, map.sizeY);
  }
}

void
normalizeMap(probabilityGrid *map)
{
  double sum = 0;
  register int x, y; 
  for(x = 0; x < map->sizeX; x++)
    for (y = 0; y < map->sizeY; y++)
      sum += map->prob[x][y];

  if (sum > 0.0)
    for(x = 0; x < map->sizeX; x++)
      for (y = 0; y < map->sizeY; y++)
	map->prob[x][y] /= sum;
}


void
convolveGrid(positionProbabilityGrid grid)
{
  int x,y,z;
  probability *tmp = (probability *) malloc(grid.sizeZ * sizeof(probability));
  for (z = 0; z < grid.sizeZ; z++){
    convolveProbX(grid.prob[z], grid.sizeX, grid.sizeY);
    convolveProbY(grid.prob[z], grid.sizeX, grid.sizeY);
  }
  for (x = 0; x < grid.sizeX; x++)
    for (y = 0; y < grid.sizeY; y++){
      for (z = 0; z < grid.sizeZ; z++)
	tmp[z] = grid.prob[z][x][y];
      for (z = 0; z < grid.sizeZ; z++){
	int z1, z2;
	z1 = z - 1;
	if (z1 < 0) z1 += grid.sizeZ;
	z2 = z + 1;
	if (z2 >= grid.sizeZ) z2 -= grid.sizeZ;
	grid.prob[z][x][y] = (tmp[z1]+tmp[z2]) * 0.1 + tmp[z] * 0.8;
      }
    }
  free(tmp);
}



float
fSqr(float x){
  return x*x;
}
#define SQRT2PI 2.5066283

float
gauss( float x, float sigma, float mean)
{
  float tmp;
    
  if (sigma != 0.0) {
    tmp = exp( -0.5 * fSqr((mean-x) / sigma));
    return( tmp / (SQRT2PI * sigma));
  }
  else {
    if (x == mean)
      return 1.0;
    else
      return 0.0;
  }
    
}






bool
simulatorMapInstalled(char *fileName){
  FILE *simFp = NULL;
  static bool simMapInstalled = FALSE;

  if (!simMapInstalled){
    if ((simFp = fopen(fileName, "r")) == NULL){
      char *error = "# Warning: could not open simulator map %s\n";
      fprintf(stderr, error, fileName);
    }
    else{
      fprintf(stderr, "# Installing simulator map %s ...", fileName);
      installSimMapNoBorder(simFp);
      fprintf(stderr, "done\n");
      simMapInstalled = TRUE;
    }
  }
  return simMapInstalled;
}

float
realCoordinateOfMapCoordinate( int x, float offset, int resolution){
  return (x + 0.5) * resolution + offset;
}

bool
visible(float dist, int x, int y, double cosAlpha,
	double sinAlpha, probabilityGrid *map)
{
  float tmpDist, endX, endY;
  float realPosX = realCoordinateOfMapCoordinate(x, map->offsetX,
						 map->resolution);
  float realPosY = realCoordinateOfMapCoordinate(y, map->offsetY,
						 map->resolution);
  endX = realPosX + dist * cosAlpha;
  endY = realPosY + dist * sinAlpha;

  if (!getDistance( realPosX, realPosY, 130, endX, endY, &tmpDist))
    return TRUE;
  else
    return FALSE;
}



gridPosition
displayXYMax( positionProbabilityGrid *m,
	      gridWindow *mapwin)
{
  int x,y,z;
  probability recmax, max=0.0, min = 1.0;
  gridPosition bestPos;
  register probability *xyMaxP,*p;
  static probability **xyMaxMap;
  static bool firstTime = TRUE;
  
  int color = 0;
  
  int winX=0, winY=0;
  int endX, endY;
  
  int maxCnt = 0;

  if (firstTime){
    xyMaxMap = (probability **) malloc(m->sizeX * sizeof(probability *));
    for (x = 0; x < m->sizeX; x++)
      xyMaxMap[x] =  (probability *) malloc(m->sizeY * sizeof(probability));
    firstTime = FALSE;
  }

  for (x = 0; x<m->sizeX;x++){
    xyMaxP = xyMaxMap[x];
    for (y = 0; y<m->sizeY;y++)
      *xyMaxP++ = 0.0;
  }
  
  for(z=0;z<m->sizeZ;z++)
    if ( m->updatePlane[z])
      for(x=0;x<m->sizeX;x++){
	xyMaxP = xyMaxMap[x];
	p = m->prob[z][x];
	for(y=0;y<m->sizeY;y++,p++,xyMaxP++){
	  if (*p > *xyMaxP){
	    *xyMaxP = *p;
	    if (*p >= max) {
	      if (*p == max) 
		maxCnt++;
	      else {
		maxCnt = 1;
		max=*p;
		bestPos.x = x;
		bestPos.y = y;
		bestPos.rot = z;
	      }
	    }
	  }
	  if (*p < min) min = *p;
	}
      }
  
  endX = mapwin->startX + m->sizeX * mapwin->scale;
  endY = mapwin->startY + m->sizeY * mapwin->scale;


  if (max == 0.0)
    recmax = 1.0;
  else
    recmax = 1/max;
/*   min = max * 1e-10; */
  for (winX=mapwin->startX,x=0; winX < endX; winX += mapwin->scale, x++){
    xyMaxP = &(xyMaxMap[x][m->sizeY-1]);
    for (winY=mapwin->startY; winY < endY; winY += mapwin->scale){
      if (1) color = fieldColor((*xyMaxP--)*recmax,max, min);
      else
	color = fieldColor((*xyMaxP--)*recmax,max, max*1e-5);
      EZX_SetColor(color);
      EZX_FillRectangle(mapwin->window,winX,winY,mapwin->scale,mapwin->scale);
    }
  }
  
  EZX_Flush();
  
  return bestPos;
}


void
allocateGrid(positionProbabilityGrid *grid)
{
  int z, x;

  grid->prob = (probability ***) malloc(grid->sizeZ * sizeof(probability **));
  for (z = 0; z < grid->sizeZ; z++){
    grid->prob[z] = (probability **)
      malloc(grid->sizeX * sizeof(probability *));
    for (x = 0; x < grid->sizeX; x++)
      grid->prob[z][x] =
	(probability *) malloc(grid->sizeY * sizeof(probability ));
  }
  grid->updatePlane = (bool *) malloc(grid->sizeZ * sizeof(bool));
}

void
initializeGrid(positionProbabilityGrid *grid)
{
  int z, x, y;

/*   probability prob = 1.0 / */
/*     ((probability) grid->sizeX * grid->sizeY * grid->sizeZ); */
  probability prob = 1.0e-20;
  for (z = 0; z < grid->sizeZ; z++){
    grid->updatePlane[z] = 1;
    for (x = 0; x < grid->sizeX; x++)
      for (y = 0; y < grid->sizeY; y++)
    grid->prob[z][x][y] = prob;
  }
}

void
normalizeGrid(positionProbabilityGrid *grid)
{
  int z, x;

  register probability tmp=0.0,  *p;
  register int y;
  probability sum = 0.0;
  probability max = 0.0, min = 1.0;

  for (z = 0; z < grid->sizeZ; z++){
    if (grid->updatePlane[z])
      for (x = 0; x < grid->sizeX; x++){
	p = grid->prob[z][x];
	tmp = 0;
	for (y = 0; y < grid->sizeY; y++)
	  tmp += *p++;
	sum += tmp;
      }
  }

  if (sum > 0){
    sum = 1/sum;
    for (z = 0; z < grid->sizeZ; z++){
      if (grid->updatePlane[z])
	for (x = 0; x < grid->sizeX; x++){
	  p = grid->prob[z][x];
	  for (y = 0; y < grid->sizeY; y++)
	  *p++ *= sum;
      }
    }
  }
  for (z = 0; z < grid->sizeZ; z++){
    if (grid->updatePlane[z])
      for (x = 0; x < grid->sizeX; x++){
	p = grid->prob[z][x];
	for (y = 0; y < grid->sizeY; y++){
	  if (*p > max)
	    max = *p;
	  else if (*p < min)
	    min = *p;
	  p++;
	}
      }
    }
  fprintf(stderr, "\nMax: %e, Min: %e\n", max, min);
}

int
main( int argc, char** argv)
{
  int i, j, x, y, z;

  int alpha;
  char *mapFileName = NULL;
  simulatorMap simMap;
  probabilityGrid map, map2, resultMap;
  positionProbabilityGrid grid1, grid2, resultGrid;
  gridWindow *window, *window2, *resultWin;

  i = 1;
  while (((int) i < argc) && (strcmp(argv[i], "-map") != 0))
    i++;
  i = i+1;
  if ((int)i < argc && (argv[i][0] != '-'))
    mapFileName = argv[i];
  else {
    fprintf(stderr, "Error: missing map file name\n");
    exit(0);
  }
  
  if (mapFileName != NULL){
    fprintf(stderr, "Map file name: %s\n", mapFileName);
  }

  if (!readSimulatorMapFile(mapFileName, &simMap)){
    fprintf(stderr, "Error: could not read %s!\n", mapFileName);
    exit(1);
  }

  if (!simulatorMapInstalled(mapFileName))
    exit(1);
  
  /* setting up graphic */
  
  grid1.positionResolution = map.resolution = 15;
  grid1.sizeZ = round(360.0 / ANGLESTEP);
  grid1.angleResolution = 2.0  * M_PI / grid1.sizeZ;
  grid1.sizeX = map.sizeX = (simMap.sizeX + 1) / (float) map.resolution + 1;
  grid1.sizeY = map.sizeY = (simMap.sizeY + 1) / (float) map.resolution + 1;
  grid1.offsetX = grid1.offsetY = map.offsetX = map.offsetY = 0;
  map2 = resultMap = map;

  grid2 = resultGrid = grid1;
  allocateGrid(&grid1);
  initializeGrid(&grid1);

  grid1.prob[0][grid1.sizeX / 4+3][grid1.sizeY / 4 + 3] = 0.1;
  grid1.prob[grid1.sizeZ/2][(grid1.sizeX * 3) / 4]
    [(grid1.sizeY * 3) / 4 - 8 ] = 0.01;

  for (i = grid1.sizeX / 2 - 10; i < grid1.sizeX /2 + 10; i++)
    grid1.prob[grid1.sizeZ/4][i][grid1.sizeY / 2] = 0.001;
  
  for (i = 0; i < 4; i++) convolveGrid(grid1); 
  normalizeGrid(&grid1);
  
  map.prob = (probability **) malloc(map.sizeX * sizeof(probability *));
  for (i = 0; i < map.sizeX; i++)
    map.prob[i] = (probability *) malloc(map.sizeY * sizeof(probability));

  for (i = 0; i < map.sizeX; i++)
    for (j = 0; j < map.sizeY; j++)
      map.prob[i][j] = 0.0;
  for (z = 0; z < grid1.sizeZ; z++)
    for (i = 0; i < map.sizeX; i++)
      for (j = 0; j < map.sizeY; j++)
	map.prob[i][j] += grid1.prob[z][i][j];


  window = createMapWindow(&map, "ROBOT 1", 100, 100, 2);
  clearMapWindow(window);
  if (0)
    displayMapWindow(&map, window);
  else
    displayXYMax(&grid1, window);
  drawSimulatorMapZRange(window, &simMap, C_BLUE, 0, 0, 0, 1000); 


  allocateGrid(&grid2);
  initializeGrid(&grid2);
  normalizeGrid(&grid2);
  allocateGrid(&resultGrid);
  initializeGrid(&resultGrid);
  normalizeGrid(&resultGrid);



  map2.prob = (probability **) malloc(map2.sizeX * sizeof(probability *));
  for (i = 0; i < map2.sizeX; i++)
    map2.prob[i] = (probability *) malloc(map2.sizeY * sizeof(probability));
  
  for (i = 0; i < map2.sizeX; i++)
    for (j = 0; j < map2.sizeY; j++)
      map2.prob[i][j] = 1.0 / map.sizeX / map.sizeY;
  
  window2 = createMapWindow(&map2, "ROBOT 2", 120, 120, 2);
  clearMapWindow(window2);
  displayXYMax(&grid2, window2);
  drawSimulatorMapZRange(window2, &simMap, C_BLUE, 0, 0, 0, 1000); 
  

  resultMap.prob = (probability **) malloc(resultMap.sizeX
					   * sizeof(probability *));
  for (i = 0; i < resultMap.sizeX; i++)
    resultMap.prob[i] = (probability *) malloc(resultMap.sizeY
					       * sizeof(probability));

  for (alpha = 0; alpha < grid1.sizeZ; alpha++)
    for (x = 0; x < resultGrid.sizeX; x++)
      for (y = 0; y < resultGrid.sizeY; y++)
	resultGrid.prob[alpha][x][y] = 0.0;
  
  for (alpha = 0; alpha < grid1.sizeZ; alpha++){
    int deltaX, deltaY;
    float factor;
    double radAlpha, cosAlpha, sinAlpha;
    int dist = 15;
    float distance = dist * grid1.positionResolution;
    int offsetX, offsetY;
    int targetX, targetY;
    radAlpha = alpha * grid1.angleResolution;
    
    cosAlpha = cos(radAlpha);
    sinAlpha = sin(radAlpha);
    offsetX = round(dist * cosAlpha);
    offsetY = round(dist * sinAlpha);
    
    fprintf(stderr, "%2.1f%%\r", (float) (100.0 * alpha / grid1.sizeZ));
    
    for (deltaX = -1; deltaX < 2; deltaX ++)
      for (deltaY = -1; deltaY < 2; deltaY++){
	if (deltaX == 0 && deltaY == 0)
	  factor = 0.92;
	else
	  factor = 0.01;
	
	for (x = 0; x < resultGrid.sizeX; x++){
	  targetX = x + offsetX + deltaX;
	  for (y = 0; y < resultGrid.sizeY; y++){
	    targetY = y + offsetY + deltaX;
	    if (visible(distance, x, y, cosAlpha, sinAlpha, &map)
		&& coordinateInMap(targetX, targetY, map))
	      resultGrid.prob[alpha][x][y] += grid2.prob[alpha][x][y] * 
		map.prob[targetX][targetY] * factor;
	    else
	      resultGrid.prob[alpha][x][y] += grid2.prob[alpha][x][y] * 1e-30 * factor;
	  }
	}
      }
  }
  normalizeGrid(&resultGrid);
  resultWin = createMapWindow(&map, "Result ROBOT 2", 140, 140, 2);
  clearMapWindow(resultWin);
  displayXYMax(&resultGrid, resultWin);
  drawSimulatorMapZRange(resultWin, &simMap, C_BLUE, 0, 0, 0, 1000); 
  
  getchar();
  
  exit(0);
}







