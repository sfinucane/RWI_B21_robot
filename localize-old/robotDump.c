
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/robotDump.c,v $
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
 * $Log: robotDump.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.17  1998/10/27 23:18:39  wolfram
 * Added bw display to robotDump as well as numbering of images
 *
 * Revision 1.16  1998/09/22 16:05:05  wolfram
 * Copied functions from function.c so that it can be linked again
 *
 * Revision 1.15  1998/06/30 13:55:10  fox
 * Updated question sensor.
 *
 * Revision 1.14  1998/06/12 10:16:42  fox
 * Implemented virutal sensor.
 *
 * Revision 1.13  1998/01/22 13:06:21  fox
 * First version after selection-submission.
 *
 * Revision 1.12  1997/12/19 11:30:13  fox
 * FIXED a bug I added.
 *
 * Revision 1.11  1997/12/18 12:57:27  wolfram
 * Fixed exceptions in shiftAndMultiplyPlane
 *
 * Revision 1.10  1997/12/11 17:06:32  fox
 * Added some parameters.
 *
 * Revision 1.9  1997/12/02 15:20:42  fox
 * Nothing remarkable.
 *
 * Revision 1.8  1997/11/25 17:13:03  fox
 * Should work.
 *
 * Revision 1.7  1997/11/23 16:31:11  wolfram
 * minor change
 *
 * Revision 1.6  1997/11/23 15:50:19  wolfram
 * Changes because of robotDump
 *
 * Revision 1.5  1997/11/21 15:36:08  fox
 * Modifications in graphic
 *
 * Revision 1.4  1997/11/20 10:03:58  wolfram
 * Final version before adding EZX
 *
 * Revision 1.3  1997/11/19 18:51:45  wolfram
 * Slight changes for robotDump
 *
 * Revision 1.2  1997/11/18 16:23:54  wolfram
 * Changes because of robotDump
 *
 * Revision 1.1  1997/11/18 08:40:04  wolfram
 * Creates XFig files showing robot and sensors
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "general.h"
#include "graphic.h"
#include "probGrid.h"
#include "map.h"
#include "script.h"
#include "proximityTools.h"
#include "file.h"
#include "allocate.h"
#include "fileNames.h"
#include "EZX11.h"


#define MAX_NUMBER_OF_OBJECTS 2000
#define MAP_MARK "MAP"
#define RECTANGLE_MARK "RECTANGLE"
#define DOOR_MARK "DOOR"
#define ROBOT_MARK "ROBOT"
#define CUBE_MARK "CUBE"
#define CYLINDER_MARK "CYLINDER"
#define SKIP 5

#define A4CENTERX 4770
#define A4CENTERY 6600

#define WINDOWWIDTH 500
#define SCALEFACTOR ((A4CENTERX - 100) * 0.002)
#define YOFFSET (A4CENTERY - WINDOWWIDTH * SCALEFACTOR)

#define TRUE 1
#define FALSE 0
#define CUBE 0
#define CYLINDER 1

#define SIMULATOR_MAP_MARK "MAP"
#define SIMULATOR_ROBOT_MARK "ROBOT"
#define RECTANGLE_MARK "RECTANGLE"
#define CUBE_MARK "CUBE"
#define CYLINDER_MARK "CYLINDER"

#define XFIG_RED 4
#define XFIG_BLUE 1
#define XFIG_GREEN 2
#define XFIG_YELLOW 6
#define XFIG_BLACK 0
#define XFIG_WHITE 7

/* #define XFIG_RED 0 */
/* #define XFIG_GREEN 0 */

#define NUMBER_OF_GREYSCALES 3
#define LIGHT_GREY 32
#define DARK_GREY 33
#define BLACK_GREY 34

char* greyScales[NUMBER_OF_GREYSCALES] = {"0 32 #dddddd", "0 33 #888888", "0 34 #000000"}; 

typedef struct{
  float centerX;
  float centerY;
  float centerZ;
  float width;
  float depth;
  float height;
  float rot;
  int type;
} rectangle;


typedef struct{
  int x;
  int y;
} windowPosition;

static float inside_x[4];
static float inside_y[4];



long
round(double x) {
  return (long) floor(x + 0.5);
}



bool
intersection(float from1, float to1, float from2, float to2){
  return from2 <= to1 && from1 <= to2;
}


static int
cm2Pixel(gridWindow* win, float cm){
  return round(cm / win->gridResolution * win->scale);
}


void
rotpnt(float angle, float *x, float *y) 
{ 
    float t = *x; 
    *x = *x * cos(angle) - *y * sin(angle); 
    *y = t  * sin(angle) + *y * cos(angle); 
}


void
absolute_points(rectangle *rect)
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
  for(i = 0; i <4; i++) {
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
inside(float xi, float yi, rectangle* rect)
{
    int i;
    rect = rect;
    for(i = 0; i < 4; i++) {
	if(!inhalfplane(xi,yi,
			inside_x[i],inside_y[i],
			inside_x[(i+1)%4],inside_y[(i+1)%4]))
	    return FALSE;
    }
    return TRUE;
}


int
xCoord(float x, float mapSizeX, float robX, float scale){
  mapSizeX = mapSizeX;
  return round((x - robX) * scale) + A4CENTERX;
}


int
yCoord(float y, float mapSizeY, float robY, float scale){
  mapSizeY = mapSizeY;
  return round((robY -y ) * scale)+A4CENTERY;
}


void
writeRectangle(FILE *fp,
	       rectangle rect, float mapSizeX, float mapSizeY,
	       float robX, float robY,
	       float scaleFactor, int color)
{
  int i;
  absolute_points( &rect);

  if (fabs(rect.rot) <= 0.001)
    fprintf(fp, "2 2 0 1 %d %d 2 0 20 0.000 0 0 -1 0 0 5\n",
	    color, color);
  else
    fprintf(fp, "2 1 0 1 %d %d 2 0 20 0.000 0 0 -1 0 0 5\n",
	    color, color);
  
  for (i = 0; i < 5; i++)
    fprintf(fp, "%d %d ",
	   xCoord(inside_x[i % 4], mapSizeX, robX, scaleFactor),
	   yCoord(inside_y[i % 4], mapSizeY, robY, scaleFactor));

  fprintf(fp, "\n");
}




void
writeCircle(FILE *fp,
	    rectangle rect, float mapSizeX, float mapSizeY,
	    float robX, float robY, float scaleFactor,
	    int color)
{
  int x = xCoord(rect.centerX, mapSizeX, robX, scaleFactor),
     y = yCoord(rect.centerY, mapSizeY, robY, scaleFactor),
    radius =  round(rect.width * scaleFactor);

  fprintf(fp, "1 3 0 1 %d %d 2 0 20 0.000 1 0.000 ", color, color);
  
  fprintf(fp, "%d %d %d %d %d %d %d %d\n",
	 x, y, radius, radius, x, y, x+radius, y+radius);

}

void
writeSimObject(FILE *fp, simulatorObject object, float mapSizeX, 
	       float mapSizeY, float robX, float robY, float scaleFactor,
	       int color)
{
  rectangle rect;
  rect.centerX = object.posX;
  rect.centerY = object.posY;
  rect.centerZ = object.posZ;
  rect.width = object.width;
  rect.depth = object.depth;
  rect.height = object.height;
  rect.rot = object.rot;
  rect.type = object.type;
  
  if (object.type == CUBE)
    writeRectangle( fp, rect, mapSizeX, mapSizeY, robX, robY, scaleFactor,color);
  else if  (object.type == CYLINDER)
    writeCircle(fp, rect, mapSizeX, mapSizeY, robX, robY,
		scaleFactor, color);
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



bool
readSimulatorMap( char *mapName, simulatorMap *simMap)
{

  FILE *fp;
  char line[MAX_STRING_LENGTH];
  int markLength;
  int found;
  realPosition pos;
  simulatorObject simObject;
  bool stop;
    
  sprintf(simMap->fileName, "%s", mapName);
  
  if ((fp = fopen(simMap->fileName,"rt")) == NULL) {
    fprintf(stderr,"# Could not open file %s\n", simMap->fileName);
    return FALSE;
  }


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
  while ((fgets(line,BUFFLEN,fp) != NULL) && !feof(fp)){
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
  
  while ((fgets(line,BUFFLEN,fp) != NULL)){
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


gridWindow *
createRobotWindow( probabilityGrid *map, char* text,
		   int size, int x, int y, int scale) {
  /* creates a window under EZX for the robot and sensor information */

  char corner[80];
  gridWindow *robwin;
  
  robwin = (gridWindow *) malloc(sizeof(gridWindow));

  robwin->gridSizeX = size / map->resolution;
  if (robwin->gridSizeX % 2 == 0)
    robwin->gridSizeX ++;

  robwin->gridSizeY = robwin->gridSizeX;
  robwin->gridOffsetX = robwin->gridOffsetY = 0;
  robwin->scale = scale;
  
  robwin->sizeX = robwin->gridSizeX * robwin->scale;
  robwin->sizeY = robwin->gridSizeY * robwin->scale;
  robwin->gridResolution = map->resolution;

  
  robwin->startX = 0;
  robwin->startY = 0;
  robwin->planeType = XYPLANE;

  sprintf(corner,"+%d+%d",x,y);
    
  EZX_NoMotionEvents();
  robwin->window = EZX_MakeWindow(text,robwin->sizeX,robwin->sizeY,corner);
  EZX_SetWindowBackground (robwin->window,C_BLACK);
  EZX_Flush();

  return(robwin);
}

void
clearMapWindow(gridWindow *win)
{
  EZX_SetColor(C_WHITE);
  EZX_FillRectangle(win->window, 0, 0, win->sizeX, win->sizeY);
}

windowPosition
robotWindowPosition(gridWindow* robwin, windowPosition center,
		    realPosition pos) {
    windowPosition winPos;
    winPos.x = 
      robwin->startX
	+ (int) round(((pos.x -
			((center.x - robwin->gridSizeX/2) )
			* robwin->gridResolution)/ robwin->gridResolution
		       )
		      * robwin->scale);
    winPos.y =
      robwin->startY
	+ (int) round(((robwin->gridSizeY -
		       (pos.y - ((center.y - robwin->gridSizeY/2))
			* robwin->gridResolution)/ robwin->gridResolution
			- 1))
		      * robwin->scale);
    return winPos;
}


void
displayRobotAtWindowPositions(gridWindow *win, windowPosition pos1,
			      windowPosition pos2, float robRadius, int col, bool fillRobot){
  
    if (fillRobot) {
      EZX_SetColor(col);
      EZX_FillCircle(win->window, pos1.x, pos1.y,
		     (float) robRadius / (float) win->gridResolution * (float) win->scale);
      EZX_SetColor(C_BLACK);
    }
    else {
      EZX_SetColor(col);
      EZX_DrawCircle(win->window, pos1.x, pos1.y,
		     (float) robRadius / (float) win->gridResolution * (float) win->scale);
    }

    EZX_DrawLine(win->window, pos1.x, pos1.y, pos2.x, pos2.y);

    EZX_Flush();
}


void
displayRobotInRobotWindow(gridWindow *robwin, robot rob, windowPosition center,
			  int col, bool fill){
  windowPosition pos1, pos2;
  realPosition pos;
  pos1 = robotWindowPosition( robwin, center, rob.pos);
  pos.x = rob.pos.x + (rob.radius-3) * cos(rob.pos.rot);
  pos.y = rob.pos.y + (rob.radius-3) * sin(rob.pos.rot);
  pos.rot = rob.pos.rot;
  
  pos2 = robotWindowPosition( robwin, center, pos);

  displayRobotAtWindowPositions(robwin, pos1, pos2, rob.radius, col, fill);
}



#define MAX_POSITION_NUMBER 100000
#define MAX_POSITION_NUMBER 100000

int
main( int argc, char *argv[] )
{
  FILE *robfp, *outfp = NULL;
   
  char line[BUFFLEN];
  float time;
  int selected, maxRange;
  simulatorMap simMap;
  gridWindow *robWin;
  probabilityGrid map;
  int argCnt, showMaxRange = TRUE;
  float scaleFactor, winScaleFactor;
  int dumpXfig = FALSE;
  int startFrame = 0, currentFrame = 0;
  int colored = FALSE, clipFigure = FALSE;
  int maxX, maxY;
  int minX, minY;
  int step = TRUE, allBeamsEqual = FALSE;
  int numbered = FALSE;
  
  map.resolution = 15;
  
  if (argc < 4)
    {
      fprintf( stderr, "usage: %s infile scale robwinlog\n", argv[0]);
      fprintf( stderr, "[-nomax] [-xfig] [-start startFrame] [-colored] [-clip] [-jump] [-equal] [-numbered]\n");
      exit(0);
    }      
   
  /*----------------------------------------------------------------------------
   * Handle arguments.
  *----------------------------------------------------------------------------*/

  if (! readSimulatorMap( argv[1], &simMap)) {
    exit(0);
  }

  if ((robfp = fopen(argv[3],"r")) == NULL) {
    fprintf(stderr,"ERROR: Could not open robwinlog file '%s'!\n",argv[3]);
    exit(0);
  }
  
  winScaleFactor = (float) atof(argv[2]);
  scaleFactor = SCALEFACTOR;
  if (scaleFactor <= 0)
    {
      fprintf(stderr, "Wrong scaling factor: %f\n", scaleFactor);
      exit(0);
    }
  else
    fprintf(stderr, "Scale factor: %f\n", scaleFactor);
 
  for ( argCnt = 4; argCnt < argc; argCnt++) {
    if ((strcmp(argv[argCnt],"-nomax")==0))
      showMaxRange = FALSE;
    else if ((strcmp(argv[argCnt],"-xfig")==0))
      dumpXfig = TRUE;
    else if ((strcmp(argv[argCnt],"-clip")==0))
      clipFigure = TRUE;
    else if ((strcmp(argv[argCnt],"-colored")==0))
      colored = TRUE;
    else if ((strcmp(argv[argCnt],"-jump")==0))
      step = FALSE;
    else if ((strcmp(argv[argCnt],"-equal")==0))
      allBeamsEqual = TRUE;
    else if ((strcmp(argv[argCnt],"-numbered")==0)){
      numbered = TRUE;
      fprintf(stderr, "lkjlkjlj %d\n", numbered);
    }
    else if ((strcmp(argv[argCnt],"-start")==0)) {
      if ( argCnt < argc - 1)
	startFrame = atoi( argv[++argCnt]);
      else {
	fprintf( stderr, "ERROR: number must follow keyword -start.\n");
	exit;
      }
    }
    else
      fprintf(stderr, "Unknown keyword.\n");
  }
  
  robWin =
    createRobotWindow(&map, "Robot window", 1000, 100, 0, winScaleFactor);

  while (!feof(robfp)){
    int eof;
    int sensorTag;
    realPosition start, end;
    windowPosition center;
    robot rob;
    start.rot = end.rot = 0.0;

    rob.radius = ROB_RADIUS;
    
    eof = (fgets(line,BUFFLEN, robfp) == NULL);
    eof = (eof || sscanf(line, "%f %f %f %f", &rob.pos.x, &rob.pos.y, &rob.pos.rot, &time) != 4);

    if (!eof){
      char fileName[BUFFLEN];
      int i;
      realPosition realOrigin;
      windowPosition origin;
      
      center.x = rob.pos.x / map.resolution;
      center.y = rob.pos.y / map.resolution;
      realOrigin.x = realOrigin.y = realOrigin.rot = 0;
      origin = robotWindowPosition(robWin, center, realOrigin);
      origin.y = robWin->sizeY - origin.y - 1;
      
      /*----------------------------------------------------------------------------
       * Display the next frame. 
       *----------------------------------------------------------------------------*/
      if ( currentFrame >= startFrame) {

	clearMapWindow(robWin);
	drawSimulatorMapZRange( robWin, &simMap, C_BLACK, origin.x, -origin.y,
				0.0, 1000.0);
	
	if ( dumpXfig) {
	  int mapColor;

	  if ( colored)
	    mapColor = XFIG_BLUE;
	  else
	    mapColor = LIGHT_GREY;
	  
	  sprintf(fileName, "robot%04d.fig", currentFrame);
	  
	  if ((outfp = fopen(fileName,"w")) == NULL) {
	    fprintf(stderr, "Error: could not open %s\n", fileName);
	    exit(1);
	  }
	  
	  fprintf(stderr, "dumping robot%04d.fig\r", currentFrame);
	  
	  fprintf(outfp, "#FIG 3.1\n");
	  fprintf(outfp, "Portrait\n");
	  fprintf(outfp, "Center\n");
	  fprintf(outfp, "Metric\n");
	  fprintf(outfp, "1200 2\n");
	  fprintf(stderr, "%f %f xy\n", simMap.sizeX, simMap.sizeY);
	  /* Write own greyscale colors. */
	  if ( ! colored) 
	    for (i = 0; i < NUMBER_OF_GREYSCALES; i++)
	      fprintf(outfp, "%s\n", greyScales[i]);
	  
	  
	  if (0) for (i = 0; i < simMap.numberOfObjects; i++)
	    writeSimObject(outfp, simMap.object[i], simMap.sizeX,
			   simMap.sizeY,
			   rob.pos.x, rob.pos.y, scaleFactor, mapColor);
	}
      }
      
      /* Read all the values even if startframe not reached. */
      while (fgets(line, BUFFLEN, robfp)
	     && sscanf(line, "%f %f %f %f %d %d %d",
		       &start.x, &start.y, &end.x, &end.y,
		       &sensorTag, &selected, &maxRange) == 7){
	  
	  /* Plot and dump the lines. */
	  if ( currentFrame >= startFrame) {
	    
	    int xfigColor = 0,xcolor = 0;
	    int width = 1;
	    windowPosition pos1, pos2;
	    int dontDraw = FALSE;
	    
	    switch (sensorTag){
	    case LASER_TAG:
	      if ( maxRange && ! showMaxRange) {
		dontDraw = TRUE;
		break;
	      }
	      if ( selected){
		if (colored)
		  xfigColor = XFIG_RED;
		else if ( ! allBeamsEqual) {
		  width = 2;
		  xfigColor = BLACK_GREY;
		}
		else
		  xfigColor = BLACK_GREY;
		xcolor = C_RED;
	      }
	      else {
		if ( ! maxRange){
		  if (colored)
		    xfigColor = XFIG_GREEN;
		  else if ( ! allBeamsEqual) {
		    width = 1;
		    xfigColor = DARK_GREY;
		  }
		  else
		    xfigColor = BLACK_GREY;
		  xcolor = C_LAWNGREEN;
		}
		else{
		  if ( showMaxRange) {
		    xfigColor = XFIG_BLACK;
		    xcolor = C_BLACK;
		  }
		  else
		    dontDraw = TRUE;
		}
	    }
	      break;
	    case SONAR_TAG:
	      if (colored){
		xfigColor = 0;
		xcolor = C_BLUE;
	      }
	      else{
		xfigColor = XFIG_BLACK;
		xcolor = C_BLACK;
	      }
	      break;
	    default:
	      xfigColor = 0; /* black */
	      xcolor = C_BLACK;
	    }
	    
	    if ( ! dontDraw) {
	      if ( dumpXfig) {
		fprintf(outfp, "2 1 0 %d %d %d 1 0 -1 0.000 0 0 -1 0 0 %d\n",
			width, xfigColor, xfigColor, 2);
		
		fprintf(outfp, "%d %d %d %d\n",
			xCoord(start.x, simMap.sizeX, rob.pos.x, scaleFactor),
			yCoord(start.y, simMap.sizeY, rob.pos.y, scaleFactor),
			xCoord(end.x, simMap.sizeX, rob.pos.x, scaleFactor),
			yCoord(end.y, simMap.sizeY, rob.pos.y, scaleFactor));
	      }
	      
	      /* compute window positions for reading points */
	      pos1 = robotWindowPosition(robWin, center, start);
	      pos2 = robotWindowPosition(robWin, center, end);
	      
	      EZX_SetColor(xcolor);
	      EZX_DrawLine(robWin->window, pos1.x, pos1.y, pos2.x, pos2.y);
	    }
	}
      }

      if ( currentFrame >= startFrame) {

	displayRobotInRobotWindow(robWin, rob, center, C_YELLOW, TRUE);
	
	/* Print robot */ 
	if ( dumpXfig) {
	  
	  int intPosX = A4CENTERX;
	  int intPosY = A4CENTERY;
	  int radius = round( ROB_RADIUS * scaleFactor);
	  int width = WINDOWWIDTH;
	  int upper, lower, left, right;

	  	    
	  upper = A4CENTERY-round(width*scaleFactor);
	  lower = A4CENTERY+round(width*scaleFactor);
	  left  = A4CENTERX - round(width*scaleFactor);
	  right = A4CENTERX + round(width*scaleFactor);
	  
	  fprintf( outfp, "1 3 0 3 0 %d 0 0 20 0.000 1 0.0000 %d %d %d %d %d %d %d %d\n",
		   colored ? XFIG_YELLOW : XFIG_WHITE, 
		   intPosX, intPosY, radius, radius, intPosX, intPosY,
		   intPosX + radius, intPosY + radius);
	  
	  fprintf(outfp, "%s %d\n      ",
		  "2 1 0 3 0 7 0 0 -1 0.000 0 0 -1 0 0",
		  2);
	  
	  fprintf(outfp, "%d %d %d %d\n",
		  intPosX, intPosY,
		  xCoord(rob.pos.x + ROB_RADIUS * cos(rob.pos.rot),
			 simMap.sizeX, rob.pos.x, scaleFactor),
		  yCoord(rob.pos.y + ROB_RADIUS * sin(rob.pos.rot),
			 simMap.sizeY, rob.pos.y, scaleFactor));
	  
	  /* print surrounding white boxes */
	  if (clipFigure) {

	    maxX = (int) A4CENTERX + 10 * simMap.sizeX * (1.0 + scaleFactor);
	    maxY = (int) A4CENTERY + 10 * simMap.sizeY * (1.0 + scaleFactor);
	    
	    minX = (int) A4CENTERX - 10 * simMap.sizeX * (1.0 + scaleFactor);
	    minY = (int) A4CENTERY - 10 * simMap.sizeY * (1.0 + scaleFactor);
	    
	    fprintf(outfp, "2 2 0 0 -1 7 0 0 20 0.000 0 0 -1 0 0 5 \n");
	    fprintf(outfp, "       %d %d %d %d %d %d %d %d %d %d\n",
		    minX, minY, maxX, minY, maxX, upper, minX, upper, minX, minY);
	    
	    fprintf(outfp, "2 2 0 0 -1 7 0 0 20 0.000 0 0 -1 0 0 5 \n");
	    fprintf(outfp, "       %d %d %d %d %d %d %d %d %d %d\n",
		    right, minY, maxX, minY, maxX, maxY, right, maxY, right, minY);
	    
	    fprintf(outfp, "2 2 0 0 -1 7 0 0 20 0.000 0 0 -1 0 0 5 \n");
	    fprintf(outfp, "       %d %d %d %d %d %d %d %d %d %d\n",
		    minX, lower, maxX, lower, maxX, maxY, minX, maxY, minX, lower);
	    
	    fprintf(outfp, "2 2 0 0 -1 7 0 0 20 0.000 0 0 -1 0 0 5 \n");
	    fprintf(outfp, "       %d %d %d %d %d %d %d %d %d %d\n",
		    minX, minY, left, minY, left, maxY, minX, maxY, minX, minY);
	    
	    fprintf(outfp, "2 2 0 1 0 7 0 0 -1 0.000 0 0 -1 0 0 5 \n");
	    fprintf(outfp, "       %d %d %d %d %d %d %d %d %d %d\n",
		    left, upper, right, upper, right, lower, left, lower, left, upper);

	    }
	  if (numbered){
	    fprintf(outfp, "4 0 0 0 0 0 72 0.0000 4 195 135 %d %d %d\\001",
		    left+500, upper+900, currentFrame+1);
	    
	  }
	  fclose(outfp);
	}
	
	fprintf( stderr, "Frame: %d\n", currentFrame);
	EZX_Flush();
	
	if ( ! step) {
	  int next;
	  scanf( "%d", &next);
	  if ( next > currentFrame)
	    startFrame = next;
	  else if ( next < 0)
	    break;
	  fprintf(stderr, "next %d\n", next);
	}
	else
	  getchar();
      }
      currentFrame++;
    }
  }
  fclose(robfp);
      
  if ( dumpXfig) {
    fprintf(stderr, "\nUse the following command to create eps-File:\n");
    fprintf(stderr, "fig2dev -L ps -x -280 robotxxx.fig > _tmp.ps ; ps2epsi _tmp.ps robotxxx.ps\n");
  }
  exit(0);
}





