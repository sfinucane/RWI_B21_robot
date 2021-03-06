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
 ***** Date of revision $Date: 2002/09/14 17:06:29 $
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


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>

#include "EZX11.h"
#include "tcx.h"
#include "tcxP.h"

#include "devUtils.h"

#define TCX_define_variables 1
/* this makes sure variables are installed */
#define DEFINE_REPLY_HANDLERS 1

/* this makes sure we'll get the handler array */

#include "inline.h"
#include "BASE-messages.h"
#include "LOCALIZE-messages.h"
#include "LASER-messages.h"
#include "PLAN-messages.h"

#include "localize.h" 
#include "function.h"
#include "map.h"
#include "beeSoftVersion.h"


#define TCX_USER_MODULE_NAME "DISPLAY_LOCALIZE"
#define NUMBER_OF_ROBOTS 20
#define MODULE_NAME_LENGTH 80

/* Dieter: These target points are used so that you can send the robot
 * to symbolic target locations using stdin and a definition file. */
extern int  listen_for_tcx_events;	/* in devUtils.c */
int numberOfTargetPositions = 0;
#define MAX_NUMBER_OF_TARGET_POSITIONS 20
char* targetPosName[MAX_NUMBER_OF_TARGET_POSITIONS];
float targetPosX[MAX_NUMBER_OF_TARGET_POSITIONS];
float targetPosY[MAX_NUMBER_OF_TARGET_POSITIONS];

#define PLAN_UPDATE_TIME_INTERVAL 1.0
#define PLAN_CHECK_PROGRESS_INTERVAL 30.0
#define MIN_PROGRESS_BETWEEN_PLAN_CHECKS 100.0
#define PLAN_UPDATE_TIMER 0
#define PLAN_CHECK_PROGRESS_TIMER 1


#define COL_ROBOT      C_YELLOW
#define COL_ROBODIR    C_BLACK
#define COL_BUTTONBACK C_RED
#define COL_BUTTONTEXT C_YELLOW 
#define COL_TEXT       C_YELLOW
#define COL_RECTANGLE  C_RED
#define MYWINDOW_X     88
#define MYWINDOW_Y     136
#define COL_BACKGROUND C_BLUE

int numberOfRobots = 0;

/* module pointers */
TCX_MODULE_PTR base[NUMBER_OF_ROBOTS];
TCX_MODULE_PTR localize[NUMBER_OF_ROBOTS];
TCX_MODULE_PTR plan[NUMBER_OF_ROBOTS];


/* module names */
char robotName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];
char baseName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];
char localizeName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];
char planName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];

/* Only update if a robot has moved enough. */
float updateDistance = 100.0;
float motionSinceLastUpdate[NUMBER_OF_ROBOTS];

/* update structures */
BASE_register_auto_update_type baseUpdate[NUMBER_OF_ROBOTS];
LOCALIZE_register_auto_update_type localizeUpdate[NUMBER_OF_ROBOTS];

/************************************************************************
 * Global variables according to robot position.
 ************************************************************************/

realPosition targetPosition[NUMBER_OF_ROBOTS];
bool targetPositionKnown[NUMBER_OF_ROBOTS];
realPosition localizePosition[NUMBER_OF_ROBOTS];
correctionParameter correction[NUMBER_OF_ROBOTS];
bool correctionParametersKnown[NUMBER_OF_ROBOTS];
realPosition basePosition[NUMBER_OF_ROBOTS];
bool basePositionKnown[NUMBER_OF_ROBOTS];

bool tcx_initialized = FALSE;

realPosition mapPosition[NUMBER_OF_ROBOTS];
unsigned long numberOfUpdates[NUMBER_OF_ROBOTS];
char update = FALSE;

/* For display of the lasers. */
realPosition laserPosition[NUMBER_OF_ROBOTS];
bool gotLaserScan[NUMBER_OF_ROBOTS];
distanceScan frontLaserScan[NUMBER_OF_ROBOTS];
distanceScan rearLaserScan[NUMBER_OF_ROBOTS];
int beamColor[NUMBER_OF_ROBOTS];


#define BUTTONHEIGHT 40
#define BUTTONWIDTH 200



/* For actions */

#define NUMBER_OF_ACTIONS 4

#define CANCEL_ACTION 0
#define SET_ROBOT_ACTION 1
#define SET_TARGET_ACTION 2
#define STOP_ROBOT_ACTION 3

#define  CANCEL_ACTION_NAME "ESCAPE"
#define  SET_ROBOT_ACTION_NAME "Set robot position"
#define  SET_TARGET_ACTION_NAME "Set target point"
#define  STOP_ROBOT_ACTION_NAME "Stop robot"


char* actionName[NUMBER_OF_ACTIONS] = {
  CANCEL_ACTION_NAME,
  SET_ROBOT_ACTION_NAME,
  SET_TARGET_ACTION_NAME,
  STOP_ROBOT_ACTION_NAME
};		   

int actionButtonColor[NUMBER_OF_ACTIONS] = {C_RED, C_YELLOW2,
					    C_LAWNGREEN, C_BLUE};


/************************************************************************
 * Forward declarations and some tools.
 ************************************************************************/



int
block_wait( struct timeval *timeout, int tcx_initialized,
            int X_initialized);


static void
computeNewMapPosition(unsigned int rob);

int
writeLog( char* fmt, ...)
{
  return 0;
}

static bool
planStopAutonomous(int module);

static bool
removeAllGoalPointsInPlan(int module);

/************************************************************************
 * Utility functions
 ************************************************************************/

#define DEG_90  M_PI_2
#define DEG_180 M_PI
#define DEG_270 (M_PI + M_PI_2)
#define DEG_360 (M_PI + M_PI)


static float
distanceBetweenPoints( realPosition p1, realPosition p2)
{
  return sqrt( fSqr( p1.x - p2.x) + fSqr( p1.y - p2.y));
}


static realPosition
endPoint( realPosition startPos, movement move)
{
  realPosition endPos;
  float cosRot;
  float sinRot;

  cosRot =  cos(startPos.rot);
  sinRot =  sin(startPos.rot);

  if ( move.forward == 0.0 && move.sideward == 0.0 && move.rotation == 0.0)
    return startPos;
  
  /* Replaced cos( r - 90) by sin( r) and sin( r - 90) by -cos( r) */
  endPos.x = startPos.x + cosRot * move.forward + sinRot * move.sideward;
  endPos.y = startPos.y + sinRot * move.forward - cosRot * move.sideward;
  endPos.rot = normalizedAngle( startPos.rot + move.rotation);

  return endPos;
}

int
moduleNumber(TCX_REF_PTR ref,
	     TCX_MODULE_PTR *module,
	     unsigned int numberOfRobots){
  /* determine the module */
  unsigned int i = 0;
  char found = FALSE;
  while (i < numberOfRobots && !found)
    if (ref->module != module[i])
      i++;
    else
      found = TRUE;

  if (!found){
    fprintf(stderr, "Error: module not found\n");
    exit(0);
  }
  return i;
}



void
swallowStatusReports()
{
  if ( tcx_initialized) {
    struct timeval TCX_waiting_time = {0, 0};
    tcxRecvLoop((void *) &TCX_waiting_time);
  }
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
  float scale;
  int planeType;
  bool firstTimePosition;
  realPosition pos;
}  gridWindow;

gridWindow *window;
EZXW_p robotListWindow;


typedef struct{
  int x;
  int y;
} windowPosition;


static void
displaySensings( gridWindow *robwin,
		 distanceScan sensings,
		 realPosition robPos,
		 int beamColor);


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

static float
gridWindowScale(int sizeX, int sizeY, float minScale) {
    float scaleX = (400.0 / sizeX);
    float scaleY =  (300.0 / sizeY);

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
			      int col, bool fillRobot)
{
  
  if (fillRobot) {
    int radius = (float) robRadius / (float) win->gridResolution *
		   (float) win->scale;

    if (radius < 4) radius = 4;
    
    EZX_SetColor(col);
    EZX_FillCircle(win->window, pos1.x, pos1.y, radius);
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
displayRobot(gridWindow *win, robot rob, int col, bool fillRobot)
{
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

  EZX_SetColor(C_YELLOW2);
  EZX_SetBackgroundColor(C_WHITE);

}


void
displayRobots(gridWindow *win)
{
  int i;
  robot rob;
  rob.radius = ROB_RADIUS;
  
  for (i = 0; i < numberOfRobots; i++) {
    if (basePositionKnown[i]){
      motionSinceLastUpdate[i] = 0.0;

      /* If the laser scan and the robot position are not too far off, we
       * display the scan(s) as well. */
#define MAX_DIST_BETWEEN_SCAN_AND_POS 30.0
      if ( distanceBetweenPoints( mapPosition[i], laserPosition[i])
	   < MAX_DIST_BETWEEN_SCAN_AND_POS) {
	displaySensings( win, frontLaserScan[i], laserPosition[i], beamColor[i]);
	rob.pos = laserPosition[i];
	displayRobot(win, rob, C_YELLOW2, TRUE);
      }
      else {
	if (0) fprintf(stderr, "dist %f\n", distanceBetweenPoints( mapPosition[i], laserPosition[i]));
	rob.pos = mapPosition[i];
	displayRobot(win, rob, beamColor[i], TRUE);
      }
#define TEXT_X_OFFSET 10
#define TEXT_Y_OFFSET 20
      EZX_SetColor( beamColor[i]);
      EZX_DrawTextAt(win->window, TEXT_X_OFFSET, TEXT_Y_OFFSET + i * TEXT_Y_OFFSET, robotName[i], 'l');
    }
    else if (correctionParametersKnown[i]){
      rob.pos.x = localizePosition[i].x;
      rob.pos.y = localizePosition[i].y;
      rob.pos.rot = localizePosition[i].rot;
      fprintf(stderr, "Xavier is at: %f %f %f\n", rob.pos.x, rob.pos.y, rob.pos.rot);
      displayRobot(win, rob, beamColor[i], TRUE);
      EZX_SetColor( beamColor[i]);
      EZX_DrawTextAt(win->window, TEXT_X_OFFSET, TEXT_Y_OFFSET + i * TEXT_Y_OFFSET, robotName[i], 'l');
    }
  }
  EZX_Flush();
}


gridWindow *
createMapWindow(probabilityGrid *m, char* text, int x, int y, float minScale) {
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


static int
chosenAction(int x, int y){
  int i,action = 0;
  struct timeval block_waiting_time = { 999,0 };
  EZX_EventType theEvent;
  EZXW_p actionWindow;
  char corner[80];

  sprintf(corner,"+%d+%d",x,y);
  actionWindow = EZX_MakeWindow("ACTION", BUTTONWIDTH,
				       (NUMBER_OF_ACTIONS)*BUTTONHEIGHT,corner);

  for (i = 0; i < NUMBER_OF_ACTIONS; i++){
    EZX_SetColor(actionButtonColor[i]);
    EZX_FillRectangle(actionWindow, 0, i*BUTTONHEIGHT,
		      BUTTONWIDTH, (i+1)*BUTTONHEIGHT);
    EZX_SetColor(C_BLACK);
    EZX_SetBackgroundColor(actionButtonColor[i]);
    EZX_DrawTextAt(actionWindow, 10 , (i+0.5) * BUTTONHEIGHT+6,
		   actionName[i], 'l');
  }

  EZX_Flush();
  
  do {
    EZX_GetEvent (&theEvent); 
    if (theEvent.type == EZX_Event_was_Nothing)
      EZX_block_and_wait ( &block_waiting_time );
  }
  while (!(theEvent.type == EZX_Event_was_Button_Press &&
	 EZX_TestCursor(actionWindow)));

  action = theEvent.PointerY / BUTTONHEIGHT;
  EZX_EndWindow(actionWindow);

  return action;
}



static void
createRobotListWindow(EZXW_p* robotListWindowP, int numberOfRobots,
		      int x, int y){
  char corner[80];
  sprintf(corner,"+%d+%d",x,y);
  
  *robotListWindowP = EZX_MakeWindow("ROBOTNAME", BUTTONWIDTH,
				       (numberOfRobots)*BUTTONHEIGHT,corner);
}


static void
drawRobotList(EZXW_p robotListWindow, int numberOfRobots){
  int i;
  char *name;
  for (i = 0; i < numberOfRobots; i++){
    EZX_SetColor(beamColor[i]);
    EZX_FillRectangle(robotListWindow, 0, i*BUTTONHEIGHT, BUTTONWIDTH, (i+1)*BUTTONHEIGHT);
    EZX_SetColor(C_BLACK);
    if (strlen(robotName[i]) > 0)
      name = robotName[i];
    else
      name = "robot";
    fprintf(stderr, "%s\n", name);
    EZX_SetBackgroundColor(beamColor[i]);
    EZX_DrawTextAt(robotListWindow, 10 , (i+0.5) * BUTTONHEIGHT+6, name, 'l');
  }
  EZX_Flush();

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

#define COL_BLACK      C_GREY0
#define COL_WHITE      C_GREY100
#define COL_UNKNOWN     C_LAWNGREEN           /* color of unknown area      */

int
fieldColor(mapProbability f){
  if (f < 0.00001){
    if (f < 0.0)
      return COL_UNKNOWN;
    else
      return COL_WHITE;
  }
  else if (f > 0.9999)
    return COL_BLACK;
  else
    return COL_WHITE - (int) round((double) f*(COL_WHITE-COL_BLACK));
}


void
displayMapWindowFast(probabilityGrid *map, gridWindow *mapwin) {
  static XImage *ximage = NULL;
  static char   *ximage_buf = NULL;
  
  if ( ximage_buf == NULL) {
    
#define CLIP_THRESHOLD 0.1
      
      int     x,y,i;
      int     color;
      int     depthFactor = 1;
      int     tlookup;

      float resolution;
      int mapSizeX, mapSizeY;

      resolution = map->resolution;
      mapSizeX = map->sizeX;
      mapSizeY = map->sizeY;
      
      if(theDepth > 8) {
	if(theDepth == 24)
	  depthFactor = 4;
	else
	  depthFactor = theDepth / 8;
      }
      
      ximage_buf = (char *) malloc( sizeof(char) * depthFactor *
				    mapwin->sizeX * mapwin->sizeY);
      
      if (ximage_buf == NULL) {
	fprintf(stderr, 
		"ERROR: Out of memory (1)\n");
	exit(1);
      }
      
      
      XSetForeground(theDisplay, theGC, theBlackPixel);
      XSetBackground(theDisplay, theGC, theWhitePixel);

      for ( y = 0; y < mapwin->sizeY; y++) {
	
	int mapY = y / mapwin->scale;
	
	tlookup = (mapwin->sizeY - y - 1)  * mapwin->sizeX * depthFactor;
	
	for ( x = 0; x < mapwin->sizeX; x++) {
	  
	  int mapX = x / mapwin->scale;

	  if ( mapX < 0 || mapY < 0 || mapX >= mapSizeX || mapY >= mapSizeY) {
	    color = C_LAWNGREEN;
	  }
	  else {
	    color = fieldColor(map->prob[mapX][mapY]);
	  }
	  
	  for(i=0;i<depthFactor;i++)
	    ximage_buf[tlookup+(x*depthFactor+i)] = 
	      thePixels[color] >> (i*8);
	}
      }
      
      ximage = XCreateImage(theDisplay, theVisual,
			    theDepth, ZPixmap,
			    0, 
			    ximage_buf,
			    mapwin->sizeX, 
			    mapwin->sizeY,
			    8*depthFactor, 
			    mapwin->sizeX*depthFactor);
      
      if (ximage == NULL){
	fprintf(stderr, 
		"ERROR: Out of memory (2)\n");
	exit(1);
      }
      
      ximage->byte_order = XImageByteOrder(theDisplay);
  }
  
  XPutImage( theDisplay, mapwin->window->w, theGC, ximage, 
	     0, 0, 
	     0, 0,
	     mapwin->sizeX, mapwin->sizeY);
  
  EZX_Flush();
  
  
}

void
displayMapWindow(probabilityGrid *m, gridWindow *mapwin) {
/* plots map in window */

  static int firstTime = TRUE;

  static XImage *ximage = NULL;
  static char   *ximage_buf = NULL;

  clearMapWindow( mapwin);
  
  if ( firstTime) {
    
    int     x,y,i;
    int     color;
    int     depthFactor = 1;
    
    int     tlookup;
      
    if(theDepth >8) {
      if(theDepth == 24)
	depthFactor = 4;
      else
	depthFactor = theDepth / 8;
    }
    
    ximage_buf = (char *) malloc( sizeof(char) * depthFactor *
				  mapwin->sizeX * mapwin->sizeY);
    
    if (ximage_buf == NULL) {
      fprintf(stderr, 
	      "ERROR: Out of memory in G_display_partial_matrix()\n");
      exit(1);
    }
    
    XSetForeground(theDisplay, theGC, theBlackPixel);
    XSetBackground(theDisplay, theGC, theWhitePixel);
    
    for ( y = 0; y < mapwin->sizeY; y++) {
      
      tlookup = (mapwin->sizeY - y)  * mapwin->sizeX * depthFactor;
      
      for ( x = 0; x < mapwin->sizeX; x++) {
	
	color = fieldColor( m->prob[round(x/mapwin->scale)][round(y/mapwin->scale)]);
	
	for(i=0;i<depthFactor;i++)
	  ximage_buf[tlookup+(x*depthFactor+i)] = 
	    thePixels[color] >> (i*8);
      }
    }
    
    ximage = XCreateImage(theDisplay, theVisual,
			  theDepth, ZPixmap,
			  0, 
			  ximage_buf,
			  mapwin->sizeX, 
			  mapwin->sizeY,
			  8*depthFactor, 
			  mapwin->sizeX*depthFactor);
    
    if (ximage == NULL){
      fprintf(stderr, 
	      "ERROR: Out of memory in G_display_partial_matrix()\n");
      exit(1);
    }
    
    ximage->byte_order = XImageByteOrder(theDisplay);
    firstTime = FALSE;
  }
  
  XPutImage( theDisplay, mapwin->window->w, theGC, ximage, 
	     0, 0, 
	     0, 0,
	     mapwin->sizeX, mapwin->sizeY);


  EZX_Flush();
}





static void
displayMap( bool useGridMap, probabilityGrid* gMap, simulatorMap* simMap,
	    gridWindow *window)
{
  if ( useGridMap) 
    displayMapWindowFast( gMap, window);
  else
    drawSimulatorMapZRange(window, simMap, C_BLACK, 0, 0, 0, 1000); 
}


static void
displaySensings( gridWindow *robwin,
		 distanceScan sensings,
		 realPosition robPos,
		 int beamColor)
{
  int i;
  
#define BEAM_SKIP 6
  
  realPosition sensingStart, sensingEnd;

  windowPosition start, end;

  EZX_SetLineStyle(FillSolid);
  EZX_SetLineWidth(1);

  /* compute start point of reading */
  sensingStart = endPoint( robPos, sensings.offset);
  start = windowPositionOfRealPosition( robwin, sensingStart);
  
  for (i = 0; i < sensings.numberOfReadings; i++) {

    if ( i % BEAM_SKIP == 0) {
      
      movement beam;
      
      /* compute end point for reading */
      sensingStart.rot = normalizedAngle(robPos.rot + sensings.reading[i].rot);
      beam.forward = sensings.reading[i].dist;
      beam.sideward = beam.rotation = 0.0;
      sensingEnd = endPoint( sensingStart, beam);
      
      end = windowPositionOfRealPosition( robwin, sensingEnd);
      
      if (sensings.reading[i].dist < sensings.maxDistance) {
	EZX_SetColor(beamColor);
	EZX_DrawLine(robwin->window, start.x, start.y, end.x, end.y);
      }
      else {
	EZX_SetColor(beamColor);
	EZX_DrawLine(robwin->window, start.x, start.y, end.x, end.y);
      }
    }
  }
  EZX_Flush();
}


/************************************************************************
 * Map handling
 ************************************************************************/


int
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


void
setStatistics( probabilityGrid *m)
{
  int y, x;
  
  int knownFields = 0;
  probability sum = 0.0;

  for (x=0; x<m->sizeX; x++)
    for (y=0; y<m->sizeY; y++)
      if ( m->prob[x][y] != m->unknown) {
	knownFields++;
	sum += m->prob[x][y];
      }
  
  if ( knownFields > 0)
    m->average = sum / knownFields;
  else
    m->average = m->unknown;
}



bool
readGridMap(char *mapName, char *extension, probabilityGrid *m){
   int x,y;
   float temp;
   char line[MAX_STRING_LENGTH], fileName[MAX_STRING_LENGTH];
   FILE *fp;

   sprintf(fileName, "%s%s", mapName, extension);

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
	   fprintf(stderr, "# Map resolution: %d cm\n",m->resolution);
	 }
     }
     if (strncmp(line,"robot_specifications->autoshifted_x",35) == 0) {
       if (sscanf(&line[35],"%g",&(m->offsetX)) != 0 ) {
	 m->offsetX = m->offsetX;
	 fprintf(stderr, "# Map offsetX: %g cm\n",m->offsetX);
       }
     }
     if (strncmp(line,"robot_specifications->autoshifted_y",35) == 0) {
       if (sscanf(&line[35],"%g",&(m->offsetY)) != 0 ) {
	 m->offsetY = m->offsetY;
	 fprintf(stderr, "# Map offsetY: %g cm\n",m->offsetY);
       }
     }
   }
   if (sscanf (line,"global_map[0]: %d %d",&m->sizeY, &m->sizeX)
       != 2 ) {
     fprintf(stderr,"ERROR: corrupted file %s\n",fileName);
     fclose(fp);
     return FALSE;
   }
   
   fprintf(stderr, "# Map size: %d %d\n",m->sizeX,m->sizeY);
   
   m->unknown = -1;
   m->offsetX = m->offsetY = 0.0;
   m->origsizeX = m->sizeX;
   m->origsizeY = m->sizeY;
   m->shiftedX = 0;
   m->shiftedY = 0;
   m->maxRealX = m->offsetX + m->sizeX * m->resolution;
   m->maxRealY = m->offsetY + m->sizeY * m->resolution;
   
   m->prob = (float**) malloc(m->sizeX * sizeof(float *));
   for (x = 0; x < m->sizeX; x++)
     m->prob[x] = (float*) malloc(m->sizeY * sizeof(float));

   
   if (m->prob == (mapProbability**) NULL){
      fprintf(stderr,
              "ERROR: Not enough memory for loading map %s\n", fileName);
      fclose(fp);
      return FALSE;
   }
   
   for (x=0;x<m->sizeX; x++)
      for (y=0;y<m->sizeY; y++) {
	 fscanf(fp,"%e",&temp);
	 if (temp < 0.0 || temp > 1.0)
	   m->prob[x][y] = -1;
	 else {
	   m->prob[x][y] = 1-temp;	   
	   if (m->prob[x][y] < MINIMUM_MAPPROBABILITY)
	     m->prob[x][y] = MINIMUM_MAPPROBABILITY;
	   else if (m->prob[x][y] > MAXIMUM_MAPPROBABILITY)
	     m->prob[x][y] = MAXIMUM_MAPPROBABILITY;
	 }
      }

   shrinkMap(m);
   setStatistics(m);
   fprintf(stderr, "# done\n");
   return TRUE;
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



/***********************************************************************
 * Handles a PLAN action.
 ***********************************************************************/
void
PLAN_action_reply_handler( TCX_REF_PTR              ref,
			   PLAN_action_reply_ptr    action)
{
  tcxFree("PLAN_action_reply", action);
  ref=ref;
}


/***********************************************************************
 * Handles a PLAN status.
 ***********************************************************************/
void
PLAN_status_reply_handler( TCX_REF_PTR              ref,
			   PLAN_status_reply_ptr    status)
{
  static bool goalWasDefined = FALSE;
  static float previousDist = 0.0;
  static bool goalPointWasDeleted = FALSE;
  
  if ( status->num_goals > 0) {

    /* Set the timer. */
    if ( ! goalWasDefined) {
      setTimer( PLAN_CHECK_PROGRESS_TIMER);
      previousDist = status->goal_distance[0];
      fprintf( stderr, "New goal at dist: %f\n", status->goal_distance[0]);
    }
    /* Check for progress. */
    else {
      
      fprintf(stderr, "Progress: %f %f\n", timeExpired( PLAN_CHECK_PROGRESS_TIMER), previousDist - status->goal_distance[0]);
      
      if ( timeExpired( PLAN_CHECK_PROGRESS_TIMER) > PLAN_CHECK_PROGRESS_INTERVAL) {

	if ( previousDist - status->goal_distance[0] < MIN_PROGRESS_BETWEEN_PLAN_CHECKS) {
	  fprintf( stderr, "Not enough progress (%f < %f).\n",
		   previousDist - status->goal_distance[0],
		   MIN_PROGRESS_BETWEEN_PLAN_CHECKS);
	  planStopAutonomous(0);
	  removeAllGoalPointsInPlan(0);
	  goalPointWasDeleted = TRUE;
	}
	/* Robot made enough progress. */
	else {
	  fprintf( stderr, "Enough progress: %f\n", previousDist - status->goal_distance[0]);
	  previousDist = status->goal_distance[0];
	}
	resetTimer( PLAN_CHECK_PROGRESS_TIMER);
      }
    }
    goalWasDefined = TRUE;
  }
  /* Just reached the goal point. */
  else {
    if ( goalWasDefined && ! goalPointWasDeleted) {
      fprintf(stderr, "Reached goal point.\n");    
    }
    goalPointWasDeleted = FALSE;
    goalWasDefined = FALSE;
  }
  
  tcxFree("PLAN_status_reply", status);
  ref=ref;
}



/************************************************************************
 *
 *   Name:         DISPLAY_LOCALIZE_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

  void
DISPLAY_LOCALIZE_close_handler(char *name, TCX_MODULE_PTR module)
{
  int i;

  fprintf( stderr, "%s: closed connection detected: %s\n",
	   TCX_USER_MODULE_NAME, name);
  
  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    exit(0);
  }
  else {
    for (i = 0; i < numberOfRobots; i++){
      if ( module == base[i]) {
	fprintf( stderr, "%s disconnected.\n", baseName[i]);
	base[i] = NULL;
      }
      if ( module == localize[i]) {
	fprintf( stderr, "%s disconnected.\n", localizeName[i]);
	localize[i] = NULL;
      }
    }
  }
}

/**********************************************************************
 **********************************************************************
 *
 *  BASE handlers
 *
 **********************************************************************
 **********************************************************************/
void
BASE_update_status_reply_handler( TCX_REF_PTR ref,
				  BASE_update_status_reply_ptr status)
{

  int mod;
  static realPosition prevPosition[NUMBER_OF_ROBOTS];
 
  
#ifdef TCX_debug
  fprintf( stderr, "TCX: Received a BASE_update_status_reply message.\n");
  fprintf( stderr, "robot: %g %g %g\n", 
	   status->pos_x, status->pos_y, status->orientation);
#endif

  mod = moduleNumber(ref, base, numberOfRobots);

  basePosition[mod].x = status->pos_x;
  basePosition[mod].y = status->pos_y;
  basePosition[mod].rot = status->orientation;

  if ( basePositionKnown[mod])
    motionSinceLastUpdate[mod] += distanceBetweenPoints( basePosition[mod],
							 prevPosition[mod]);

  prevPosition[mod] = basePosition[mod];
  update = ( motionSinceLastUpdate[mod] > updateDistance);

  /* Set the new robot position. */  
  basePositionKnown[mod] = TRUE;

  computeNewMapPosition(mod);

  tcxFree("BASE_update_status_reply", status); /* don't remove this! */
}


void
BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				  BASE_robot_position_reply_ptr pos)
{ref=ref; pos=pos;}


void
BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
				   BASE_action_executed_reply_ptr data)
{ref=ref;data=data;}



/**********************************************************************
 **********************************************************************
 *
 *  LOCALIZE handlers
 *
 **********************************************************************
 **********************************************************************/

void
LOCALIZE_map_reply_handler( TCX_REF_PTR                 ref,
			    LOCALIZE_map_reply_ptr map)
{ref=ref; map=map;}

void
LOCALIZE_samples_reply_handler( TCX_REF_PTR                 ref,
			    LOCALIZE_samples_reply_ptr samples)
{
  ref = ref; samples = samples;
}


void
LOCALIZE_update_status_reply_handler( TCX_REF_PTR                 ref,
				      LOCALIZE_update_status_reply_ptr status)
{
  unsigned int rob;
  
#ifdef TCX_debug
  fprintf( stderr, "TCX: Received a LOCALIZE_update_status_reply message.\n");
  fprintf( stderr, "local: %d %f\n",
	   status->numberOfLocalMaxima, status->probOfGlobalMaximum);
#endif
  
  /* determine the module */
  rob = moduleNumber(ref, localize, numberOfRobots);

#ifdef TCX_debug
  fprintf( stderr, "position of %s: %f %f %f\n",
	   robotName[rob],
	   status->robotX, status->robotY, rad2Deg(status->robotRot));
#endif
  
  /* Set the new correction parameters. */
  correction[rob].x = status->corrX;
  correction[rob].y = status->corrY;
  correction[rob].rot = status->corrRot;
  correction[rob].type = status->corrType;
  localizePosition[rob].x = status->robotX;
  localizePosition[rob].y = status->robotY;
  localizePosition[rob].rot = status->robotRot;
  
  if (status->numberOfLocalMaxima < 3){
    correctionParametersKnown[rob] = TRUE;
    computeNewMapPosition(rob);
  }
  else
    correctionParametersKnown[rob] = FALSE;

  tcxFree("LOCALIZE_update_status_reply", status); /* don't remove this! */
}


/**********************************************************************
 **********************************************************************
 *
 *  LASER handlers
 *
 **********************************************************************
 **********************************************************************/
void
LASER_laser_reply_handler(TCX_REF_PTR           ref,
			  LASER_laser_reply_ptr laser)
{
  int i;
  int rob;
  
  /* determine the module */
  rob = moduleNumber(ref, base, numberOfRobots);

  if ( ! gotLaserScan[rob]) {

    frontLaserScan[rob].numberOfReadings = laser->f_numberOfReadings;
    frontLaserScan[rob].reading          = (distanceReading*)
      malloc ( laser->f_numberOfReadings * sizeof(distanceReading));
    for ( i = 0; i < frontLaserScan[rob].numberOfReadings; i++)
      frontLaserScan[rob].reading[i].rot = normalizedAngle( laser->f_startAngle
						   + i * laser->f_angleResolution);
    
    rearLaserScan[rob].numberOfReadings = laser->r_numberOfReadings;
    rearLaserScan[rob].reading          = (distanceReading*)
      malloc ( laser->r_numberOfReadings * sizeof(distanceReading));
    for ( i = 0; i < rearLaserScan[rob].numberOfReadings; i++)
      rearLaserScan[rob].reading[i].rot = normalizedAngle( laser->r_startAngle
						  + i * laser->r_angleResolution);

#ifdef TCX_debug
    fprintf(stderr, "frontLaser: %d %f %f\n", frontLaserScan[rob].numberOfReadings,  
	    rad2Deg(frontLaserScan[rob].reading[0].rot), 
	    rad2Deg(frontLaserScan[rob].reading[frontLaserScan[rob].numberOfReadings-1].rot));
    
    if (  rearLaserScan[rob].numberOfReadings == 0) 
      fprintf(stderr, "no rear Laser\n");
    else
      fprintf(stderr, "rearLaser: %d %f %f\n", rearLaserScan[rob].numberOfReadings,  
	      rad2Deg(rearLaserScan[rob].reading[0].rot), 
	      rad2Deg(rearLaserScan[rob].reading[rearLaserScan[rob].numberOfReadings-1].rot));
#endif

    gotLaserScan[rob] = TRUE;
  }
  
  /* Get the front readings. */
  for (i = 0; i < laser->f_numberOfReadings; i++) {
    if (laser->f_reading[i] < 0.0)
      frontLaserScan[rob].reading[i].dist = 0.0;
    else {
      frontLaserScan[rob].reading[i].dist = laser->f_reading[i];
    }
  }
  
  /* Get the rear readings. */
  for (i = 0; i < laser->r_numberOfReadings; i++) {
    if (laser->r_reading[i] < 0.0)
      rearLaserScan[rob].reading[i].dist = 0.0;
    else {
      rearLaserScan[rob].reading[i].dist = laser->r_reading[i];
    }
  }


  /* Get the new corrected position of the robot. */
  robotCoordinates2MapCoordinates( laser->xPos,
				   laser->yPos,
				   90.0 - rad2Deg(laser->rotPos),
				   correction[rob].x,
				   correction[rob].y,
				   correction[rob].rot,
				   correction[rob].type,
				   &laserPosition[rob].x,
				   &laserPosition[rob].y,
				   &laserPosition[rob].rot);  
		  
  tcxFree("LASER_laser_reply", laser);
  ref=ref;
}


void
initTcx(char *moduleName)
{
  char *tcxMachine = NULL;
  int i;
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    BASE_messages,
    LOCALIZE_messages,
    LASER_messages,
    PLAN_messages
  };

  for (i=0; i < NUMBER_OF_ROBOTS; i++){
    base[i] = localize[i] = NULL;
    
    numberOfUpdates[i] = 0;
    baseUpdate[i].subscribe_status_report = 4;
    baseUpdate[i].subscribe_sonar_report = 0;
    baseUpdate[i].subscribe_laser_report = 4;
    baseUpdate[i].subscribe_ir_report = 0;
    baseUpdate[i].subscribe_colli_report = 0;

    localizeUpdate[i].subscribe = 1;

    correctionParametersKnown[i] = FALSE;
    basePositionKnown[i] = FALSE;
    motionSinceLastUpdate[i] = 0.0;
    gotLaserScan[i] = FALSE;
    targetPositionKnown[i] = FALSE;
  }

  fprintf(stderr, "Connecting to TCX...");
  tcxMachine = getenv("TCXHOST");   
  
  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize(moduleName, (void *) tcxMachine);
  fprintf(stderr, "done.\n");

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));
  

  tcxRegisterHandlers(BASE_reply_handler_array,
		      sizeof(BASE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(LOCALIZE_reply_handler_array,
		      sizeof(LOCALIZE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(LASER_reply_handler_array,
		      sizeof(LASER_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
  
  tcxRegisterHandlers(PLAN_reply_handler_array,
		      sizeof(PLAN_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
  
  tcxRegisterCloseHnd(DISPLAY_LOCALIZE_close_handler);
  tcx_initialized = TRUE;
}


static realPosition
realPositionOfWindowPosition(gridWindow* win, windowPosition winPos){
  realPosition pos;
  pos.x= (((winPos.x - win->startX) / (float) win->scale) + win->gridOffsetX)
    * win->gridResolution;
  pos.y = ((win->gridSizeY - ((winPos.y - win->startY)
			     / (float) win->scale)) + win->gridOffsetY)
    * win->gridResolution;
  pos.rot = 0.0;
  
  return pos;
}



realPosition
positionRobotinWindow(int mode, gridWindow mapwindow,
		      probabilityGrid *map,
		      simulatorMap *simMap,
		      robot rob, int x, int y,
		      int destroyPositionTool) {
  /* Parameters:
   * mode=SET_ROBOT_POSITION: set robot to single location and get orientation
   *   else: select two coordinates as the area of interest and get orientation
   * gridWindow: structure of window to be used to modify position in
   */

  realPosition initialPos = rob.pos;
  int cancel = 0;
  int xr, yr, function = 0; 
  int x1 = 0, y1 = 0, rot = 0;
  char corner[80];
  char data[10];

  EZX_EventType theEvent;
  struct timeval block_waiting_time = { 999,0 };
  EZXW_p mywindow = NULL;
  windowPosition winPos;
  
  mode = mode;

  if (simMap->initialized){
    clearMapWindow( &mapwindow);
    drawSimulatorMapZRange(&mapwindow, simMap, C_BLACK, 0, 0, 0, 1000); 
  }
  else{
    displayMapWindowFast( map, &mapwindow);
  }
      
  sprintf(corner,"+%d+%d",x,y);
  
  /* Open own window to rotate robot */
  if ( mywindow == NULL) {
    mywindow = EZX_MakeWindow("Place Robot", MYWINDOW_X, MYWINDOW_Y, corner);
    EZX_SetColor(COL_BACKGROUND);
    EZX_FillRectangle(mywindow, 0,0, MYWINDOW_X, MYWINDOW_Y);
    /* plot window, gadgets */
    EZX_SetColor(COL_BUTTONBACK);
    EZX_FillRectangle(mywindow, 5, MYWINDOW_Y-24, MYWINDOW_X-10, 19);
    EZX_FillRectangle(mywindow, 5, MYWINDOW_Y-48, 19, 19);
    EZX_FillRectangle(mywindow, MYWINDOW_X-24, MYWINDOW_Y-48, 19, 19);
    EZX_SetColor(COL_BUTTONTEXT);
    EZX_SetBackgroundColor(COL_BUTTONBACK);
    EZX_DrawText(mywindow, 13, MYWINDOW_Y-9,"D O N E");
    EZX_DrawText(mywindow, 9, MYWINDOW_Y-33,"<");
    EZX_DrawText(mywindow, MYWINDOW_X-18, MYWINDOW_Y-33,">");
  }
  
  winPos.x = 0;
  winPos.y = 0;
  
  do {
    /* plot BIG robot */
    EZX_SetColor(COL_ROBOT);
    EZX_FillCircle(mywindow, MYWINDOW_X/2, MYWINDOW_X/2, MYWINDOW_X/2-5);
    EZX_SetColor(COL_ROBODIR);
    xr = MYWINDOW_X/2 + cos(rot * M_PI/180.0) * (MYWINDOW_X/2-5.5) - 0.5;
    yr = MYWINDOW_X/2 - sin(rot * M_PI/180.0) * (MYWINDOW_X/2-5.5) - 0.5;
    EZX_DrawLine(mywindow, MYWINDOW_X/2, MYWINDOW_X/2, xr, yr);
    sprintf(data,"%3i�",rot);
    EZX_SetColor(COL_TEXT);
    EZX_SetBackgroundColor(COL_BACKGROUND);
    EZX_DrawText(mywindow, 26, MYWINDOW_Y-33, data);
    
    if (1) {
      /* plot small robot */
      rob.pos = realPositionOfWindowPosition(&mapwindow, winPos);
      rob.pos.rot = deg2Rad((float) rot);
      displayRobot(&mapwindow, rob, COL_ROBOT, 1);
      /*      fprintf(stderr,"# Location: x=%.2f y=%.2f rot=%d. Grid position x=%d y=%d\n", 
	      rob.pos.x,
	      rob.pos.y,
	      rot,
	      mapCoordinateOfRealCoordinate(rob.pos.x, 
					    map->offsetX, 
					    map->resolution),
	      mapCoordinateOfRealCoordinate(rob.pos.y, 
					    map->offsetY, 
					    map->resolution)
	      );
      */
    }
    else {
      /* draw frame */
    }
    
    /* wait for mouse activity */
    
    /* wait for a mouse action, sleep if none present */
    EZX_Flush();
    EZX_GetEvent (&theEvent); 
    while (theEvent.type != EZX_Event_was_Button_Press) {
      if (theEvent.type == EZX_Event_was_Nothing)
	EZX_block_and_wait ( &block_waiting_time ); 
      EZX_GetEvent (&theEvent); 
    }
    
    /* analyze mouse action */
    if (EZX_TestCursor(mywindow)) {
      /* mouse was in controlling window */
      if ((theEvent.PointerX >= 5) 
	  && (theEvent.PointerX < (MYWINDOW_X-5))
	  && (theEvent.PointerY < (MYWINDOW_Y-5))
	  && (theEvent.PointerY >= (MYWINDOW_Y-48))
	  && ((theEvent.PointerY < (MYWINDOW_Y-29))
	      || (theEvent.PointerY >= (MYWINDOW_Y-24)))
	  ) {

	if (theEvent.PointerY <= 24) {
	  cancel = 1;
	  function = 1;
	}
	else if (theEvent.PointerY >= (MYWINDOW_Y-24)){
	  /* was in button area... */
	  function = 1;
	  if (theEvent.Button != 1) 
	    cancel = 1;
	}
	else {
	  switch(theEvent.Button) {
	  case 1: function = 5;
	    break;
	  case 2: function = 1;
	    break;
	  default: function = 45;
	    break;
	  }
	  if (theEvent.PointerX < 24) {
	    rot += function;
	    if (rot > 359) rot -= 360;
	  }
	  else if (theEvent.PointerX >= MYWINDOW_X-24) {
	    rot -= function;
	    if (rot < 0) rot += 360;
	  }
	  function = 0;
	}
      }
      
    }
    else if( EZX_TestCursor(mapwindow.window)) {
     
      /* mouse was on map window */
      switch(theEvent.Button) {
      case 1: /* left button */
	winPos.x = x1 = theEvent.PointerX;
	winPos.y = y1 = theEvent.PointerY;
	break;
      case 3: /* right button */
	if (simMap->initialized){
	  clearMapWindow( &mapwindow);
	  drawSimulatorMapZRange(&mapwindow, simMap, C_BLACK, 0, 0, 0, 1000); 
	}
	else{
	  displayMapWindowFast( map, &mapwindow);
	}
	break;
      default: break;
      }
    }
    if (function == 1) {
      /* clear map if done or if new position chosen */
    }

    swallowStatusReports();
    
    
  } while (function != 1);
  
  
  if ( destroyPositionTool)
    EZX_EndWindow(mywindow);
  
  if (cancel == 0)
    return rob.pos;
  else
    return initialPos;

}



static void
computeNewMapPosition(unsigned int rob)
{
  if ( basePositionKnown[rob] && correctionParametersKnown[rob]){
    
    /* Get the new corrected position of the robot. */
    robotCoordinates2MapCoordinates( basePosition[rob].x,
				     basePosition[rob].y,
				     basePosition[rob].rot,
				     correction[rob].x,
				     correction[rob].y,
				     correction[rob].rot,
				     correction[rob].type,
				     &mapPosition[rob].x,
				     &mapPosition[rob].y,
				     &mapPosition[rob].rot);  
    if (0) fprintf( stderr, "New map position for %s: %f %f %f\n",
		    robotName[rob],
		    mapPosition[rob].x,
		    mapPosition[rob].y,
		    rad2Deg(mapPosition[rob].rot));
  }
}


/*****************************************************************************
 * Checks wether the connection to the module is established. If not the function
 * tries to establish the connection.
 *****************************************************************************/
bool
connectionEstablished( TCX_MODULE_PTR* module, char* name)
{
  if (!tcx_initialized)
    return FALSE;
  if ( *module == NULL) {
    *module = tcxConnectOptional( name);
    if (*module != NULL) {
      fprintf(stderr, "Connected to %s.\n", name);
      return TRUE;
    }
    else
      return FALSE;
  }
  else
    return TRUE;
}


static bool
planStopAutonomous(int module)
{
  if ( connectionEstablished( &plan[module], planName[module])) {

    int autonomousMode = 1;

    fprintf(stderr,  "TCX message to %s: stop autonomous.\n",
	    planName[module]);

    tcxSendMsg ( plan[module], "PLAN_stop_autonomous_message",
		 &autonomousMode);
    return TRUE;
  }
  else
    return FALSE;
}

static bool
removeAllGoalPointsInPlan(int module)
{
  if ( connectionEstablished( &plan[module], planName[module])) {

    fprintf(stderr, "TCX message to %s: remove all goal points.\n",
	    planName[module]);

    tcxSendMsg ( plan[module], "PLAN_remove_all_goals", NULL);
    return TRUE;
  }
  else {
    fprintf(stderr, "Could not connect to %s. Try again later\n",
	    planName[module]);
    return FALSE;
  }
}

/*****************************************************************************
 * Send the map to the planning module.
 *****************************************************************************/
bool
sendGoalPointToPlan( int module, realPosition goal)
{
  if ( connectionEstablished( &plan[module], planName[module])) {

    PLAN_goal_message_type goal_message;

    goal_message.x = goal.x;
    goal_message.y = goal.y;
    goal_message.max_radius = 0.0;        /* dummy */
    goal_message.reward     = 0.0;        /* dummy */
    goal_message.name       = 0;          /* dummy */
    goal_message.add        = 1;          /* dummy */

    fprintf(stderr,  "TCX message to %s: goal point.\n", planName[module]);

    tcxSendMsg ( plan[module], "PLAN_goal_message", &goal_message );
    return TRUE;
  }
  else
    return FALSE;
}


/*****************************************************************************
 * Send the planning module the command to start to move to the goal points.
 *****************************************************************************/
bool
planStartAutonomous(int module)
{
  if ( connectionEstablished( &plan[module], planName[module])) {

    int autonomousMode = 1;


    fprintf(stderr,  "TCX message to %s: start autonomous.\n",
	      planName[module]);

    tcxSendMsg ( plan[module], "PLAN_start_autonomous_message",
		 &autonomousMode);
    return TRUE;
  }
  else
    return FALSE;
}


void
updatePlanStatus( int module)
{
  static bool firstTime = TRUE;

  if ( firstTime) {
    setTimer( PLAN_UPDATE_TIMER);
    firstTime = FALSE;
  }
  
  if ( connectionEstablished( &plan[module], planName[module])) {
    if ( timeExpired( PLAN_UPDATE_TIMER) > PLAN_UPDATE_TIME_INTERVAL) {
      tcxSendMsg ( plan[module], "PLAN_status_query", NULL);
      resetTimer( PLAN_UPDATE_TIMER);
    }
  }
}

/* Reads target positions.
 * FORMAT:
 * TARGET_NAME X_COORD Y_COORD
 */
void
getTargetPositions( char* fileName)
{
  if ( fileName != NULL) {
    
    FILE* fp;
    char line[MAX_STRING_LENGTH];

    if ((fp = fopen(fileName,"r")) == NULL) {
      fprintf(stderr,"# Could not open position file %s\n", fileName);
      return;
    }

    fprintf( stderr, "# Get target positions from %s.\n", fileName);
    
    while ((fgets( line, MAX_STRING_LENGTH, fp) != NULL) && !feof(fp)) {
      
      if ( numberOfTargetPositions >= MAX_NUMBER_OF_TARGET_POSITIONS) {
	fprintf( stderr, "Error: too many target positions (%d). Resize \"MAX_NUMER_OF_TARGET_POSITIONS\".", numberOfTargetPositions);
	exit(0);
      }
      
      targetPosName[numberOfTargetPositions] = (char*) malloc( MAX_STRING_LENGTH * sizeof(char));
    
      if ( sscanf( line, "%s %f %f\n",
		   (targetPosName[numberOfTargetPositions]),
		   &(targetPosX[numberOfTargetPositions]),
		   &(targetPosY[numberOfTargetPositions])) < 3) {
	fprintf( stderr, "Error scanning position file: %s\n", line);
	exit(0);
      }
      numberOfTargetPositions++;
    }
    fprintf(stderr, "# Successfully read %d target positions.\n", numberOfTargetPositions);
  }
}

void ProcessCommand(char *buffer)
{
  int target;

  for ( target = 0; target < numberOfTargetPositions; target++) {
    if ( strncmp( buffer, targetPosName[target],
		  strlen(targetPosName[target])) == 0) {

      realPosition goal;

      goal.x   = targetPosX[target];
      goal.y   = targetPosY[target];
      goal.rot = 0.0;
      
      fprintf( stderr, "Found keyword. Send %f %f to plan.\n",
	       targetPosX[target], targetPosY[target]);
      
      if ( sendGoalPointToPlan( 0, goal)) {
	(void) planStartAutonomous(0);
	fprintf( stderr, "Done.\n");
      }
	   
      
    }
    else {
      fprintf(stderr, "No fit: %s %s %d %d\n", buffer, targetPosName[target],
	      strlen(buffer), strlen(targetPosName[target]) );
    }
  }
}

void stdin_inputHnd(int fd, long chars_available)
{
  static char buffer[MAX_STRING_LENGTH+1];
  static char *startPos = buffer; /* position to start parsing from */
  static char *endPos = buffer; /* position to add more characters */
  char *lineEnd;
  int numRead = 0;
  /* should handle characters output by the base */
  
  /* never expect more than DEFAULT_LINE_LENGTH characters on a line.
   * read the first DEFAULT_LINE_LENGTH and let the function get called 
   * again for any remaining characters.  This can be changed.
   */
  
/*    printf ("Entering in stdin_inputHnd\n"); */
  if (startPos == endPos)
    { 
      startPos = endPos = buffer;
      bzero(buffer, MAX_STRING_LENGTH+1);
    }
  
  /* read in the command. */
  numRead = readN(&stdin_device, endPos, 
		  MIN(chars_available,(MAX_STRING_LENGTH - (endPos - startPos))));
  endPos += numRead;
  if (numRead == 0)
    { /* handle error here. The port is already closed. */
    }
  else {
    /* see if we have a \n */
    lineEnd = (char *) strpbrk(startPos,"\n");
    while (lineEnd != NULL)
      {/* found a string, pass it to the parsing routines. */
	*lineEnd = '\0';
	ProcessCommand(startPos);
	startPos = lineEnd+1;
	lineEnd = (char *) strpbrk(startPos,"\n");
      }
    /* Fix up the buffer. Throw out any consumed lines.*/
    if (startPos >= endPos) 
      { /* parsed the whole thing, just clean it all up */
	bzero(buffer, MAX_STRING_LENGTH+1);
	startPos = endPos = buffer;
      }
    else if (startPos != buffer)
      { /* slide it back and wait for more characters */
	bcopy(startPos, buffer, (endPos - startPos));
	endPos = buffer + (endPos - startPos);
	startPos = buffer;
      }
  }
  /*printf ("Exiting from stdin_inputHnd\n");*/
}


static void
wrongUsage(char* progName)
{
  fprintf(stderr,  "Usage: %s -gmap <gridMapFile> -smap <simulatorMapFile>", progName);
  fprintf(stderr,  " -module <moduleName> -robots <r1 ... rn> -update <dist>\n");
  exit(0);
}






int
main( int argc, char** argv)
{
  int i;
  bool wait;
  bool useGridMap = TRUE, mapDefined = FALSE;
  struct timeval TCX_waiting_time = {0, 0};
  float minScale = 1;
  char *moduleName = TCX_USER_MODULE_NAME;
  char *mapFileName = NULL;
  char *positionFileName = NULL;
  bool usePositionFile = FALSE;
  simulatorMap simMap;
  probabilityGrid gMap;

  listen_for_tcx_events = TRUE;
  
  /* Handle argc. */
  for ( i = 1; i < argc; i++) {
    if ((strcmp(argv[i],"-gmap")==0)) {
      if ( i < argc - 1) {
	mapFileName = argv[++i];
	if (! readGridMap( mapFileName, "", &gMap)) {
	  fprintf(stderr, "Error: could not read grid map from %s!\n", mapFileName);
	  wrongUsage( argv[0]);
	}
	else {
	  fprintf(stderr, "# Successfully read grid map from %s.\n", mapFileName);
	  mapDefined = TRUE;
	}

      }
      else {
	fprintf( stderr, "grid map file name must follow keyword -gmap.\n");
	wrongUsage( argv[0]);
      }
    }
    else if ((strcmp(argv[i],"-smap")==0)) {
      if ( i < argc - 1) {
	mapFileName = argv[++i];
	if (! readSimulatorMapFile( mapFileName, &simMap)) {
	  fprintf(stderr, "Error: could not read sim map from %s!\n", mapFileName);
	  wrongUsage( argv[0]);
	}
	else {
	  fprintf(stderr, "# Successfully read simulator map from %s.\n", mapFileName);
	  mapDefined = TRUE;
	}
	useGridMap = FALSE;
      }
      else {
	fprintf( stderr, "simulator map file name must follow keyword -smap.\n");
	wrongUsage( argv[0]);
      }
    }
    else if ((strcmp(argv[i],"-module")==0)) {
      if ( i < argc - 1) {
	if (argv[i+1][0] != '-')
	  moduleName = argv[++i];
	else {
	  fprintf( stderr, "Module name must follow keyword -module.\n");
	  wrongUsage( argv[0]);
	}
      }
      else {
	fprintf( stderr, "Module name must follow keyword -module.\n");
	wrongUsage( argv[0]);
      }
    }
    else if ((strcmp(argv[i],"-posFile")==0)) {
      if ( i < argc - 1) {
	if (argv[i+1][0] != '-') {
	  positionFileName = argv[++i];
	  usePositionFile = TRUE;
	}
	else {
	  fprintf( stderr, "Position file name must follow keyword -posFile.\n");
	  wrongUsage( argv[0]);
	}
      }
      else {
	  fprintf( stderr, "Position file name must follow keyword -posFile.\n");
	wrongUsage( argv[0]);
      }
    }
    else if ((strcmp(argv[i],"-robots")==0)) {
      if ( i < argc - 1) {
	numberOfRobots = 0;
	i++;
	/* Determine robots. */
	while ((i < argc) && (argv[i][0] != '-')) {
	  if (numberOfRobots < NUMBER_OF_ROBOTS){ 
	    strcpy(robotName[numberOfRobots], argv[i]);
	    numberOfRobots++;
	  }
	  else {
	    fprintf(stderr, "Error: too many robots, fix that first\n");
	    wrongUsage( argv[0]);
	  }
	  i++;
	}
	i--;
      }
      else {
	fprintf( stderr, "You must specify robot names after keyword -robots.\n");
	wrongUsage( argv[0]);
      }
    }
    else if ((strcmp(argv[i],"-scale")==0)) {
      if ( i < argc - 1) {
	minScale = atof(argv[++i]);
      }
      else {
	fprintf( stderr, "integer must follow keyword -scale.\n");
	exit(0);
      }
    }
    else if ((strcmp(argv[i],"-update")==0)) {
      if ( i < argc - 1) {
	updateDistance = atof(argv[++i]);
      }
      else {
	fprintf( stderr, "distance must follow keyword -update.\n");
	exit(0);
      }
    }
    else {
      fprintf( stderr, "Unknown keyword %s.\n", argv[i]);
      wrongUsage( argv[0]);
    }
  }

  if ( ! mapDefined && ! usePositionFile)
    wrongUsage( argv[0]);

  /* Read the symbolic target points. */
  if ( usePositionFile)
    getTargetPositions( positionFileName);
  
  /* Set the robot names into the global structures. */
  if (numberOfRobots == 0){
    numberOfRobots = 1;
    strcpy(robotName[0], "");
    fprintf(stderr, "# Displaying one robot!\n");
    tcxSetModuleName(TCX_BASE_MODULE_NAME, NULL, baseName[0]);
    tcxSetModuleName(TCX_LOCALIZE_MODULE_NAME, NULL, localizeName[0]);
    tcxSetModuleName(TCX_PLAN_MODULE_NAME, NULL, planName[0]);
  }
  else {
    for(i=0; i < numberOfRobots; i++){
      tcxSetModuleName(TCX_BASE_MODULE_NAME, robotName[i], baseName[i]);
      tcxSetModuleName(TCX_LOCALIZE_MODULE_NAME, robotName[i], localizeName[i]);
      tcxSetModuleName(TCX_PLAN_MODULE_NAME, robotName[i], planName[i]);
    }
  }
  
  

  
  /* setting up graphic */
  if (! useGridMap) {
    /* Only set to initialize the graphics window. */
    gMap.resolution = 15;
    gMap.sizeX = (simMap.sizeX + 1) / (float) gMap.resolution + 1;
    gMap.sizeY = (simMap.sizeY + 1) / (float) gMap.resolution + 1;
    gMap.offsetX = gMap.offsetY = 0;
  }
  
  /* set some default colors. */
  beamColor[0] = C_BLUE;
  beamColor[1] = C_RED;
  beamColor[2] = C_BLACK;
  beamColor[3] = C_LAWNGREEN;
  
  for ( i = 4; i < NUMBER_OF_ROBOTS; i++)
    beamColor[i] = C_GREY40;
  

  window = createMapWindow(&gMap, "LOCALIZE-CONTROL", 0, 0, minScale);
  displayMap( useGridMap, &gMap, &simMap, window);
  
  createRobotListWindow(&robotListWindow, numberOfRobots,
			window->sizeX+2, 0);
  drawRobotList(robotListWindow, numberOfRobots);
  

  /* print information */
  fprintf(stderr, "Connecting to robots: ");
  for(i=0; i < numberOfRobots; i++)
    fprintf(stderr, "%s ", robotName[i]);
  fprintf(stderr, "\n");
  
  connectDev(&stdin_device);
  stdin_device.outputHnd = stdin_inputHnd;
  
  /* initializing TCX */
  initTcx(moduleName);
  
  while(TRUE) {
    wait = TRUE;
    update = FALSE;

    for (i = 0; i < numberOfRobots; i++){

      /* Reconnection? */
      if ( base[i] == NULL) {
	base[i] = tcxConnectOptional(baseName[i]);
	if (base[i]!=NULL){
	  fprintf(stderr, "Connected to %s\n", baseName[i]);
	  tcxSendMsg(base[i], "BASE_register_auto_update", &(baseUpdate[i]));
	  wait = FALSE ;
	}
      }
      if ( localize[i] == NULL) {
	localize[i] = tcxConnectOptional(localizeName[i]);
	if (localize[i] != NULL){
	  fprintf(stderr, "Connected to %s\n", localizeName[i]);
	  tcxSendMsg(localize[i],
		     "LOCALIZE_register_auto_update",
		     &(localizeUpdate[i]));
	  wait = FALSE;
	}
      }
    }
    
    if (TRUE){
      robot rob;
      EZX_EventType theEvent;
      /*      struct timeval block_waiting_time = { 999,0 }; */
      realPosition pos;
      int n = 0;
      int action = 0;
      EZX_GetEvent (&theEvent);
      rob.radius = 30;
      if (EZX_TestCursor(robotListWindow)){
	n = theEvent.PointerY / BUTTONHEIGHT;
	action = chosenAction(window->sizeX+2, 0);
	fprintf(stderr, "Action chosen: %d\n", action);
	if (action == SET_ROBOT_ACTION){
	  /* set robot position */
	  if (connectionEstablished( &localize[n], localizeName[n])){
	    fprintf(stderr, "Positioning robot %s\n", robotName[n]);
	    EZX_SetColor(C_YELLOW);
	    EZX_DrawRectangle(robotListWindow, 0, n*BUTTONHEIGHT, BUTTONWIDTH-1,
			      (n+1) * BUTTONHEIGHT-1);
	    rob.pos = mapPosition[n];
	    pos = 
	      positionRobotinWindow(SET_ROBOT_ACTION, *window,
				    &gMap,
				    &simMap,
				    rob, window->sizeX, numberOfRobots * BUTTONHEIGHT + 2,
				    TRUE);	
	    drawRobotList(robotListWindow, numberOfRobots);
	    if (pos.x != rob.pos.x || pos.y != rob.pos.y ||
		pos.rot != rob.pos.rot){
	      fprintf(stderr, "Setting the position of %s to %.2f %.2f %.2f\n",
		      robotName[n],
		      pos.x,
		      pos.y,
		      rad2Deg(pos.rot));
	      
	      if (connectionEstablished(&localize[n],localizeName[n])){
		LOCALIZE_set_robot_position_type locPos;
		locPos.x = pos.x;
		locPos.y = pos.y;
		locPos.rot = pos.rot;
		fprintf( stderr, "Send position (%f %f %f) to %s...",
			 locPos.x, locPos.y, locPos.rot, localizeName[n]);
		tcxSendMsg(localize[n], "LOCALIZE_set_robot_position", &locPos);
		fprintf( stderr, " done.\n"); 
	      }
	      
	    }
	  }
	  else {
	    fprintf( stderr, "Please try again when %s is reconnected\n",
		     localizeName[n]);
	  }
	}
	else if (action == SET_TARGET_ACTION){

	  if (connectionEstablished(&plan[n], planName[n])){

	    fprintf(stderr, "Setting target point for robot %s\n", robotName[n]);
	    EZX_SetColor(C_RED);
	    EZX_DrawRectangle(robotListWindow, 0, n*BUTTONHEIGHT, BUTTONWIDTH-1,
			      (n+1) * BUTTONHEIGHT-1);
	    rob.pos = mapPosition[n];
	    pos = 
	      positionRobotinWindow(SET_TARGET_ACTION, *window,
				    &gMap,
				    &simMap,
				    rob, window->sizeX + 5, 0, TRUE);
	    drawRobotList(robotListWindow, numberOfRobots);
	    if (pos.x != rob.pos.x || pos.y != rob.pos.y ||
		pos.rot != rob.pos.rot){
	      fprintf(stderr,
		      "Setting the target position of %s to %.2f %.2f %.2f\n",
		      robotName[n],
		      pos.x,
		      pos.y,
		      rad2Deg(pos.rot));
	      if (sendGoalPointToPlan( n, pos))
		(void) planStartAutonomous(n);
	      
	    }
	  }
	  else
	    fprintf(stderr, "%s not connected, try again later!\n", planName[i]);
	}
	else if (action == STOP_ROBOT_ACTION){
	  fprintf(stderr, "Deleting target points for robot %s\n",
		  robotName[n]);
	  if (removeAllGoalPointsInPlan(n))
	    (void) planStopAutonomous(n);
	  else
	    fprintf(stderr, "%s not connected, try again later!\n", planName[i]);
	}
	
      }
    }

    if ( usePositionFile)
      updatePlanStatus(0);

    if (wait) block_wait(NULL, 1, 0); 
    
    ProcessDevices();

    if (update) {
      displayMap(useGridMap, &gMap, &simMap, window);
      displayRobots(window);
      block_wait( NULL, 1, TRUE);
    }
    
  }
  exit(0);			/* should never reach here! */
}

