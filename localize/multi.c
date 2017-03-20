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
 ***** Date of revision $Date: 2002/09/14 20:45:40 $
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
#include "fakeSensors.hh" 
#ifdef UNIBONN
#include "localize.h"
#include "map.h"
#endif
#include "beeSoftVersion.h"


#define TCX_define_variables /* this makes sure variables are installed */
#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */
#include "multi.h"
#include "BASE-messages.h"
#ifdef UNIBONN
#include "LOCALIZE-messages.h"
#endif


#define TCX_USER_MODULE_NAME "MULTI_LOCALIZE"
#define NUMBER_OF_ROBOTS 20
#define MODULE_NAME_LENGTH 80

unsigned int numberOfRobots = 0;

/* module pointers */
TCX_MODULE_PTR base[NUMBER_OF_ROBOTS];
TCX_MODULE_PTR localize[NUMBER_OF_ROBOTS];

/* module names */
char robotName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];
char baseName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];
char localizeName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];

/* update structures */
BASE_register_auto_update_type baseUpdate[NUMBER_OF_ROBOTS];
LOCALIZE_register_auto_update_type localizeUpdate[NUMBER_OF_ROBOTS];


/************************************************************************
 * Global variables according to robot position.
 ************************************************************************/

correctionParameter correction[NUMBER_OF_ROBOTS];
char correctionParametersKnown[NUMBER_OF_ROBOTS];
realPosition basePosition[NUMBER_OF_ROBOTS];
char basePositionKnown[NUMBER_OF_ROBOTS];

realPosition mapPosition[NUMBER_OF_ROBOTS];
unsigned long numberOfUpdates[NUMBER_OF_ROBOTS];
char update = FALSE;


/************************************************************************
 * Forward declarations
 ************************************************************************/


int
block_wait( struct timeval *timeout, int tcx_initialized,
            int X_initialized);


void
computeNewMapPosition(unsigned int rob);

/************************************************************************
 * Utility functions
 ************************************************************************/

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

    windowPosition pos1, pos2, pos3;
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


void
displayRobots(gridWindow *win){
  unsigned int i;
  robot rob;
  rob.radius = ROB_RADIUS;

  for (i = 0; i < numberOfRobots; i++)
    if (basePositionKnown[i]){
      rob.pos = mapPosition[i];
      displayRobot(win, rob, robotName[i], C_YELLOW, TRUE);
    }
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





/************************************************************************
 *
 *   Name:         MULT_LOCALIZE_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
MULTI_LOCALIZE_close_handler(char *name, TCX_MODULE_PTR module)
{
  unsigned int i;

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

  unsigned int i, nr;
  
#ifdef TCX_debug
  fprintf( stderr, "TCX: Received a BASE_update_status_reply message.\n");
  fprintf( stderr, "robot: %g %g %g\n", 
	   status->pos_x, status->pos_y, status->orientation);
#endif

  nr = moduleNumber(ref, base, numberOfRobots);

  
  /* Set the new robot position. */
  basePositionKnown[nr] = TRUE;

  update = (basePosition[nr].x = status->pos_x != status->pos_x ||
	    basePosition[nr].y != status->pos_y ||
	    basePosition[nr].rot != status->orientation);
  basePosition[nr].x = status->pos_x;
  basePosition[nr].y = status->pos_y;
  basePosition[nr].rot = status->orientation;

  computeNewMapPosition(nr);

  if (0){
  fprintf(stderr, "Updates:");
  numberOfUpdates[nr]++;
  for (i = 0; i < numberOfRobots; i++)
    fprintf(stderr, " %d: %ld;", i,  numberOfUpdates[i]);
  fprintf(stderr,"\r");
  }
  
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
LOCALIZE_update_status_reply_handler( TCX_REF_PTR                 ref,
				      LOCALIZE_update_status_reply_ptr status)
{
  unsigned int rob;
  
#ifdef TCX_debug
  fprintf( stderr, "TCX: Received a LOCALIZE_update_status_reply message.\n");
  fprintf( stderr, "local: %d %f\n",
	   status->numberOfLocalMaxima, status->probOfGlobalMaximum);
#endif
  fprintf( stderr, "TCX: Received a LOCALIZE_update_status_reply message.\n");
  fprintf( stderr, "local: %d %f\n",
	   status->numberOfLocalMaxima, status->probOfGlobalMaximum);
  
  /* determine the module */
  rob = moduleNumber(ref, localize, numberOfRobots);

  /* Set the new correction parameters. */
  correction[rob].x = status->corrX;
  correction[rob].y = status->corrY;
  correction[rob].rot = status->corrRot;
  correction[rob].type = status->corrType;
  
  if (status->numberOfLocalMaxima < 3){
    correctionParametersKnown[rob] = TRUE;
    computeNewMapPosition(rob);
  }
  else
    correctionParametersKnown[rob] = FALSE;

  update = TRUE;
  
  tcxFree("LOCALIZE_update_status_reply", status); /* don't remove this! */
}




void
initTcx()
{
  char *tcxMachine = NULL;
  int i;
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    BASE_messages,
    LOCALIZE_messages
  };

  for (i=0; i < NUMBER_OF_ROBOTS; i++){
    base[i] = localize[i] = NULL;
    
    numberOfUpdates[i] = 0;
    baseUpdate[i].subscribe_status_report = 8;
    baseUpdate[i].subscribe_sonar_report = 0;
    baseUpdate[i].subscribe_laser_report = 0;
    baseUpdate[i].subscribe_ir_report = 0;
    baseUpdate[i].subscribe_colli_report = 0;

    localizeUpdate[i].subscribe = 3;

    correctionParametersKnown[i] = FALSE;
    basePositionKnown[i] = FALSE;
  }

  fprintf(stderr, "Connecting to TCX...");
  tcxMachine = getenv("TCXHOST");   
  
  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize(TCX_USER_MODULE_NAME, (void *) tcxMachine);
  fprintf(stderr, "done.\n");

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));
  

  tcxRegisterHandlers(BASE_reply_handler_array,
		      sizeof(BASE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(LOCALIZE_reply_handler_array,
		      sizeof(LOCALIZE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
  
  tcxRegisterCloseHnd(MULTI_LOCALIZE_close_handler); 
  
}



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



void
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
    if (1) fprintf( stderr, "New map position for %s: %f %f %f\n",
	     robotName[rob],
	     mapPosition[rob].x,
	     mapPosition[rob].y,
	     rad2Deg(mapPosition[rob].rot));
  }
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
displayMapWindow(probabilityGrid *m, gridWindow *mapwin) {
/* plots map in window */

    int x,y;
    
    int winX, winY;
    int endX = mapwin->startX + m->sizeX * mapwin->scale;
    int endY = mapwin->startY + m->sizeY * mapwin->scale;

/*     EZX_SetColor(COL_UNKNOWN); */
/*     EZX_FillRectangle(mapwin->window,mapwin->startX,mapwin->startY,endX,endY); */
    
    y=m->sizeY;
    for (winY=mapwin->startY; winY <  endY; winY += mapwin->scale){
	y--;
	x=0;
	for (winX=mapwin->startX; winX < endX; winX += mapwin->scale){
	  EZX_SetColor(fieldColor( m->prob[x][y]));
	  EZX_FillRectangle(mapwin->window,winX,winY,
			    mapwin->scale,mapwin->scale);
	  x++;
	}
      }

    EZX_Flush();

}


int
main( int argc, char** argv)
{
  unsigned int i;
  char wait;
  char gridMap = 0;
  struct timeval TCX_waiting_time = {0, 0};
  char oneRobot = 0;
  
  char *mapFileName = NULL;
  simulatorMap simMap;
  probabilityGrid map;
  gridWindow *window;

  i = 1;
  while (((int) i < argc) &&
	 ((strcmp(argv[i], "-map") !=0 &&  strcmp(argv[i], "-gridmap") != 0)))
    i++;
  i = i+1;
  if (strcmp(argv[i-1], "-gridmap") == 0)
    gridMap = 1;
  if ((int)i < argc && (argv[i][0] != '-'))
    mapFileName = argv[i];
  else {
    fprintf(stderr, "Error: missing map file name\n");
    exit(0);
  }
  
  if (mapFileName != NULL){
    fprintf(stderr, "Map file name: %s\n", mapFileName);
  }

  if (gridMap){
    if (!readGridMap(mapFileName, "", &map)){
    fprintf(stderr, "Error: could not read %s!\n", mapFileName);
    exit(1);
    }
  }
  else {
    if (!readSimulatorMapFile(mapFileName, &simMap)){
      fprintf(stderr, "Error: could not read %s!\n", mapFileName);
      exit(1);
    }
  }
  
  numberOfRobots = 0;
  i = 1;
  while (((int) i < argc) && (strcmp(argv[i], "-robots") != 0)) i++;
  i = i+1;
  if ((int)i < argc){
    while (((int)i < argc) && argv[i][0] != '-')
      if (numberOfRobots < NUMBER_OF_ROBOTS){ 
	strcpy(robotName[numberOfRobots], argv[i]);
	numberOfRobots++;
	i++;
      }
      else {
	fprintf(stderr, "Error: too many robots, fix that first\n");
	exit(0);
      }
  }

  
  if (numberOfRobots == 0){
    numberOfRobots = 1;
    strcpy(robotName[0], "");
    oneRobot = 1;
    fprintf(stderr, "# Warning: displaying only one robot!\n");
  }
  

  
  /* setting up graphic */

 
  if (!gridMap){
    map.resolution = 15;
    map.sizeX = (simMap.sizeX + 1) / (float) map.resolution + 1;
    map.sizeY = (simMap.sizeY + 1) / (float) map.resolution + 1;
    map.offsetX = map.offsetY = 0;
  }
  
  window = createMapWindow(&map, "MULTI-LOCALIZE", 100, 100, 1);
  clearMapWindow(window);
  if (gridMap) {
    displayMapWindow( &map, window);
  }
  else
    {
      drawSimulatorMapZRange(window, &simMap, C_BLACK, 0, 0, 0, 1000); 
    }
  
  /* initializing TCX */

  fprintf(stderr, "Connecting to robots: ");
  for(i=0; i < numberOfRobots; i++)
    fprintf(stderr, "%s ", robotName[i]);
  fprintf(stderr, "\n");

  if (oneRobot){
    tcxSetModuleName(TCX_BASE_MODULE_NAME, NULL, baseName[0]);
    tcxSetModuleName(TCX_LOCALIZE_MODULE_NAME, NULL, localizeName[0]);
  }
  else
    for(i=0; i < numberOfRobots; i++){
      tcxSetModuleName(TCX_BASE_MODULE_NAME, robotName[i], baseName[i]);
      tcxSetModuleName(TCX_LOCALIZE_MODULE_NAME, robotName[i], localizeName[i]);
    }

  fprintf(stderr, " %s %s\n", baseName[0], localizeName[0]);
  {
    getchar();
  }
  initTcx();
  
  
  
  while(TRUE) {
    wait = TRUE;
    update = FALSE;
    for (i = 0; i < numberOfRobots; i++){
      if ( base[i] == NULL) {
	wait = FALSE ;
	base[i] = tcxConnectOptional(baseName[i]);
	if (base[i]!=NULL){
	  fprintf(stderr, "Connected to %s\n", baseName[i]);
	  tcxSendMsg(base[i], "BASE_register_auto_update", &(baseUpdate[i]));
	}
      }
      if ( localize[i] == NULL) {
	wait = FALSE;
	localize[i] = tcxConnectOptional(localizeName[i]);
	if (localize[i] != NULL){
	  fprintf(stderr, "Connected to %s\n", localizeName[i]);
	  tcxSendMsg(localize[i],
		     "LOCALIZE_register_auto_update",
		     &(localizeUpdate[i]));
	}
      }
    }
    if (wait) block_wait(NULL, 1, 0);
    tcxRecvLoop((void *) &TCX_waiting_time);

    if (update) {
      clearMapWindow(window);
      drawSimulatorMapZRange(window, &simMap, C_BLACK, 0, 0, 0, 1000); 
      displayRobots(window);
    }
    
  }
  
  exit(0);			/* should never reach here! */
}

