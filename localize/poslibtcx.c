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
 ***** poslibtcx- poslib TCX interface
 ***** 
 ***** Brian Rudy (brudyNO@SPAMpraecogito.com)
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * 0.15 1/24/2003
 * Fixed bug in single goal code that would occasionally cause the robot 
 * not to detect arrival. The fix uses a timer to work around this.
 *
 * 0.14 7/19/2002
 * Changed local reaction startup/shutdown to remote.
 *
 * 0.13 7/9/2002
 * Reduced update frequency to 25.0 from 50.0
 *
 * 0.12 6/14/2002
 * Fixed bug in multiple goal arrival code that would call arrived_at_goal
 * for each goal.
 *
 * 0.11 6/7/2002
 * Planner goals are now flushed at arrival, then reloaded when the timer 
 * expires. This seems to improve performance with single goals, and prevent 
 * strange behavior when planStartAutonomous is called when the presentation 
 * timer expires. laserServer no longer crashes intermittently with a sigHnd 
 * 13 when reaction exits.
 *
 * 0.10 5/25/2002
 * Re-wrote pantilt interface code. Various bug fixes. 
 *
 * 0.09 5/26/2002
 * Added support for pantilt tracking of goal camera target, reaction3 
 * startup and shutdown. 
 *
 * 0.08 5/24/2002
 * Added support for stop at goal for COUNT_DOWN_TIME, then resume.
 * Added hooks for call_get_nearest_camerapos().
 #
 * 0.07 3/16/2002
 * Fixed bugs caused by numgoals being declared twice. (Oops!)
 * Multiple-goal arrival code works properly now. Still need to
 * update single goal arrival section.
 *
 * 0.06 1/15/2002
 * Updated 'reached goal' code. Now only executes call_mark_nearest_goal()
 * one time.
 *
 * 0.05 12/26/2001
 * Stripped-out most of 'reached goal' code and re-wrote. Now works with 
 * multiple active goals in the planner.
 *
 * 0.04 12/19/2001
 * Added plan status report polling in main loop. Removed 'new' goal 
 * detection as it is no longer required. If more than one goal is in the 
 * planner the 'reached goal' routine does not detect arrival.
 *
 * 0.03 12/14/2001
 * Minor debugging updates. Added new 'reached goal' test (does not work 
 * yet, but may be related to plan BUSY messages). First 
 * (partially) working version.
 *
 * 0.02 12/12/2001
 * Corrected typo in POSLIB_TCX_close_handler. still non-functional
 *
 * 0.01 12/7/2001
 * Innitial version based on trimmed-down displayLocalize.c
 * Connects to plan and localize, but does not get TCX messages.
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
#include <sys/signal.h>
#include <sys/types.h>

#include </home/brudy/posServer/poslib.h>
#include "tcx.h"
#include "tcxP.h"
#include "global.h"

#include "devUtils.h"
#include <baseClient.h>
/* #include <pantiltClient.h> */
/* #include <rai.h> */
#include <bUtils.h>

#define TCX_define_variables 1
/* this makes sure variables are installed */
#define DEFINE_REPLY_HANDLERS 1 /* this makes sure we'll get the handler array */
#include "inline.h"
#include "BASE-messages.h"
#include "LOCALIZE-messages.h"
#include "LASER-messages.h"
#include "PLAN-messages.h"
#include "PANTILT-messages.h"

#include "localize.h" 
#include "function.h"
#include "map.h"
#include "beeSoftVersion.h"
#include "librobot.h"

#define TCX_USER_MODULE_NAME "POSLIB_TCX"
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
#define PLAN_UPDATE_TIMER 0
#define PLAN_CHECK_PROGRESS_TIMER 1
#define COUNT_DOWN_TIME 60

#define SINGLE_GOAL_RADIUS 200
#define SINGLE_GOAL_TIME_INTERVAL 30.0
#define SINGLE_GOAL_TIMER 0
struct timeval goal_time;
static struct timeval last_goal_time  = {0, 0};
int nogoals;
int mystucktimer = 0;

/* The following are not currently in use */
/* #define PLAN_CHECK_PROGRESS_INTERVAL 30.0 */
/* #define MIN_PROGRESS_BETWEEN_PLAN_CHECKS 100.0 */


int numberOfRobots = 0;

/* module pointers */
TCX_MODULE_PTR base[NUMBER_OF_ROBOTS];
TCX_MODULE_PTR localize[NUMBER_OF_ROBOTS];
TCX_MODULE_PTR plan[NUMBER_OF_ROBOTS];
TCX_MODULE_PTR pantilt[NUMBER_OF_ROBOTS];


/* module names */
char robotName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];
char baseName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];
char localizeName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];
char planName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];
char pantiltName[NUMBER_OF_ROBOTS][MODULE_NAME_LENGTH];

/* Only update if a robot has moved enough. */
/* float updateDistance = 100.0; */
/* float updateDistance = 50.0; */
float updateDistance = 25.0;
float motionSinceLastUpdate[NUMBER_OF_ROBOTS];

/* update structures */
BASE_register_auto_update_type baseUpdate[NUMBER_OF_ROBOTS];
LOCALIZE_register_auto_update_type localizeUpdate[NUMBER_OF_ROBOTS];

/* Poslib globals */
int poslibres = 15;

/*  Usefull for debugging TCX message traffic */
/* #define TCX_debug 1 */
int mynumgoals = 0;
int arrived = 0;
int stopped = 0;
struct timeval this_time;
static struct timeval last_time  = {0, 0};


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



/************************************************************************
 * Forward declarations and some tools.
 ************************************************************************/

/*
 *typedef struct {
 *    int x;
 *    int y;
 *    int z;
 *} camPosition;
 */

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

void
checkNewgoals(int bypass);

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


/* Not currently used */
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



/* Not currently used */
void
swallowStatusReports()
{
  if ( tcx_initialized) {
    struct timeval TCX_waiting_time = {0, 0};
    tcxRecvLoop((void *) &TCX_waiting_time);
  }
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


/********************
 * Pantilt handlers
 ********************/

void 
PANTILT_position_reply_handler(TCX_REF_PTR                ref,
			       PANTILT_position_reply_ptr data)
{
  #ifdef TCX_debug
    fprintf(stderr, "TCX: Received a PANTILT_position_reply message.\n");
  #endif
  tcxFree("PANTILT_position_reply", data);
  ref=ref;
}


void 
PANTILT_init_reply_handler(TCX_REF_PTR             ref,
			   int                    *data)
{
  #ifdef TCX_debug
    fprintf(stderr, "TCX: Received a PANTILT_init_reply message.\n");
  #endif
  tcxFree("PANTILT_init_reply", data);
  ref=ref;
}


void 
PANTILT_limits_reply_handler(TCX_REF_PTR              ref,
			     PANTILT_limits_reply_ptr data)
{
  #ifdef TCX_debug
    fprintf(stderr, "TCX: Received a PANTILT_limits_reply message.\n");
  #endif
  tcxFree("PANTILT_limits_reply", data);
  ref=ref;
}


void 
PANTILT_status_update_handler(TCX_REF_PTR              ref,
			      PANTILT_status_update_ptr data)
{
  #ifdef TCX_debug
    fprintf(stderr, "TCX: Received a PANTILT_status_update message.\n");
    fprintf(stderr, "speed: %6.4f %6.4f\n",
	  data->pan_velocity, data->tilt_velocity);
  #endif
  tcxFree("PANTILT_status_update", data);
  ref=ref;
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


/***********************************
 * Start the presentation countdown timer
 ***********************************/

void
start_timer() {
  gettimeofday(&this_time, NULL);
  last_time.tv_sec  = this_time.tv_sec; 
  last_time.tv_usec = this_time.tv_usec;
}


/***********************************
 * Start the single goal countdown timer
 ***********************************/
 
void
start_goal_timer() {
  gettimeofday(&goal_time, NULL);
  last_goal_time.tv_sec  = goal_time.tv_sec;
  last_goal_time.tv_usec = goal_time.tv_usec;
}


/**********************************
 * Check the timer
 *********************************/

int 
check_timer() {
  float time_difference;
  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);

  if (time_difference >= COUNT_DOWN_TIME) {
    last_time.tv_sec  = this_time.tv_sec;
    last_time.tv_usec = this_time.tv_usec;
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************
 * Check the single goal timer
 *********************************/

int
check_goal_timer() {
  float time_difference;
  gettimeofday(&goal_time, NULL);
  time_difference =
    ((float) (goal_time.tv_sec - last_goal_time.tv_sec))
    + (((float) (goal_time.tv_usec - last_goal_time.tv_usec))
       /  1000000.0);

  if (time_difference >= 5) {
    fprintf(stderr, "Goat timer = %d/%d.\n", (int)time_difference, (int)SINGLE_GOAL_TIME_INTERVAL);
  }
  if (time_difference >= SINGLE_GOAL_TIME_INTERVAL) {
   /* 
    last_goal_time.tv_sec  = goal_time.tv_sec;
    last_goal_time.tv_usec = goal_time.tv_usec;
    */
    return 1;
  }
  else {
    return 0;
  }
}


/********************************
 * Goal arrival behavior
 *******************************/
void
arrived_at_goal(void) {
  cameraPosition campos;
  PANTILT_track_point_type trackPoint;
  float dummy;

  /* Ask poslib to delete the goal from the list of activegoals */
  call_mark_nearest_goal();

  /* Stop the robot */
  (void) planStopAutonomous(0);
  stopped = 1;
  (void) removeAllGoalPointsInPlan(0);

  /* Reaction4 should do all of this stuff, but in the mean time
     we will do it the kludgey way. */

  /* kill reaction */
  system("/home/brudy/posServer/reaction-down.pl &");

  /* Get the goals camera target position from poslib */
  campos = call_get_nearest_camerapos();
  fprintf(stderr, "Tracking Camerax=%d, Cameray=%d, Cameraz=%d.\n", campos.x, campos.y, campos.z);

  mapCoordinates2RobotCoordinates((float)campos.x,
                                   (float)campos.y,
                                   0.0,
                                   correction[0].x,
                                   correction[0].y,
                                   correction[0].rot,
                                   correction[0].type,
                                   &trackPoint.x,
                                   &trackPoint.y,
                                   &dummy);
  trackPoint.height = (float)campos.z; 

  /* point the camera at the goal */
  fprintf(stderr, "Sending tracking message x=%0.2f, y=%0.2f\n", trackPoint.x, trackPoint.y);
  tcxSendMsg(pantilt[0], "PANTILT_track_point", &trackPoint);
 
  /* Speak the infotext for this goal */
  call_speakinfotext();  

  /* Start the countdown timer */
  start_timer();
}


/***********************************************************************
 * Handles a PLAN status.
 ***********************************************************************/
void
PLAN_status_reply_handler( TCX_REF_PTR              ref,
			   PLAN_status_reply_ptr    status)
{
  int j;
  static float planDist = 0.0;

  /* Much too small, only use for testing */
  /* int goalradius = 100; */

  /* This one is pretty good */
   int goalradius = 147;


  #ifdef TCX_debug
    fprintf(stderr, "Got PLAN_status_reply.\n");
  #endif
  
  for (j = 0; j < (status->num_goals); j++){
    /* fprintf(stderr, "Checking goal %d.\n", j); */
    planDist = status->goal_distance[j]; 

    fprintf( stderr, "Goal %d at dist: %f\n", j, planDist);
    /* fprintf( stderr, "There are %d goals.\n", status->num_goals); */

    /* Check for progress. */
    if ((mynumgoals > status->num_goals) && (!stopped)) {
      fprintf(stderr, "Reached goal point.\n");
      /* This is invalid */
      /* fprintf(stderr, "%0.2f cm from goal.\n", planDist); */
      mystucktimer = 0;
      arrived_at_goal();
    } 
    else {
      /* Only one goal in the planner */
      if ((planDist <= goalradius) && (mynumgoals <= 1) && (!arrived) && (!stopped)) {
        fprintf(stderr, "Reached last goal point.\n");
        fprintf(stderr, "%0.2f cm from goal.\n", planDist);
        arrived = 1;
        mystucktimer = 0;
        arrived_at_goal();
      }
    }
  }
  
  if ((planDist <= SINGLE_GOAL_RADIUS) && (mynumgoals <= 1) && (!arrived) && (!stopped)) {
    mystucktimer++;
    fprintf(stderr, "Stuck timer incrementing: %d.\n", mystucktimer);
  }

  /* Special condition, outside of goal radius, but inside of SINGLE_GOAL_RADIUS */
  /* This only happens occasionally, but causes problems if it isn't caught */
  if (mystucktimer >= SINGLE_GOAL_TIME_INTERVAL) {
    fprintf(stderr, "Reached last goal point. (stuck timer expired)\n");
    fprintf(stderr, "%0.2f cm from goal.\n", planDist);
    arrived = 1;
    mystucktimer = 0;
    arrived_at_goal();
  }

  start_goal_timer();

  mynumgoals = status->num_goals;
  
  tcxFree("PLAN_status_reply", status);
  ref=ref;
}



/************************************************************************
 *
 *   Name:         POSLIB_TCX_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

  void
POSLIB_TCX_close_handler(char *name, TCX_MODULE_PTR module)
{
  int i;

  fprintf( stderr, "%s: closed connection detected: %s\n",
	   TCX_USER_MODULE_NAME, name);
  
  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    perlclose();
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

  /* Send the data to Perl poslib */
  /*call_pushpos(status->pos_x, status->pos_y, status->orientation);*/
  
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
  /*
  fprintf(stderr, "laser Position: x=%.2f, y=%.2f, rot=%.2f\n", laserPosition[rob].x/poslibres, laserPosition[rob].y/poslibres, rad2Deg(laserPosition[rob].rot));
  */
  /* Send the data to Perl poslib */
  call_pushpos(laserPosition[rob].x/poslibres, laserPosition[rob].y/poslibres, rad2Deg(laserPosition[rob].rot));
  if ((!stopped) && (mystucktimer <= 1)) {
    checkNewgoals(0);
  }
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
    PLAN_messages,
    PANTILT_messages
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

  tcxRegisterHandlers(PANTILT_reply_handler_array,
		      sizeof(PANTILT_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
    

  tcxRegisterCloseHnd(POSLIB_TCX_close_handler);
  tcx_initialized = TRUE;
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


/**************************************************
 * poslib routines
 * Check poslib newgoal semaphore, and load new goals
 * if there are any that haven't been visited.
 ***************************************************/
void
checkNewgoals(int bypass)
{
  realPosition goal;
  goalPosition goalPos;
  int numgoals;
  int i;
  nogoals = 0;

  if ((call_newgoal() == 1) || (bypass)) {
      numgoals = call_howmanygoals();
      fprintf( stderr, "New goal semaphore is set, getting %d goals.\n", 
               numgoals+1);
      for (i=0; numgoals >= i; i++) {
        goalPos = call_getgoal(i);
        goal.x   = goalPos.x;
        goal.y   = goalPos.y;
        goal.rot = 0.0;
     
        if (bypass) {
          fprintf( stderr, "Reloading goal %f %f to plan.\n",
                 goal.x, goal.y);
        }
        else {
          fprintf( stderr, "Found new goal. Sending %f %f to plan.\n",
                 goal.x, goal.y);
        }
  
        if (!sendGoalPointToPlan( 0, goal)) {
           fprintf( stderr, "Uh oh, something is wrong with the planner!\n");
        }
      }
      arrived = 0;
   
      if (!stopped) {
        (void) planStartAutonomous(0);
      }
      /* reset the semaphore */
      if (!bypass) {
        call_flushsemaphores();
      }
      fprintf( stderr, "Done.\n");
   }
}



void
updatePlanStatus( int module)
{
  static bool firstTime = TRUE;

  if ( firstTime) {
    setTimer( PLAN_UPDATE_TIMER);
    firstTime = FALSE;
    arrived = 1; /* This keeps the single goal countdown timer from misbehaving */
  }
  
  if ( connectionEstablished( &plan[module], planName[module])) {
    if ( timeExpired( PLAN_UPDATE_TIMER) > PLAN_UPDATE_TIME_INTERVAL) {
      tcxSendMsg ( plan[module], "PLAN_status_query", NULL);
      resetTimer( PLAN_UPDATE_TIMER);
    }

    /* We missed the arrival at the last goal. Do stuff to catch up */
    /*
     *if ((!nogoals) && (!stopped) && (!arrived) && (check_goal_timer() == 1)) {
     * fprintf(stderr, "Reached last goal point. (catching up)\n");
     * arrived = 1;
     * start_goal_timer();
     * arrived_at_goal();
    *}
    */
  }
  /*
  fprintf(stderr, "updatePlanStatus: nogoals=%d, stopped=%d, arrived=%d, check_goal_timer=%d.\n", nogoals, stopped, arrived, check_goal_timer());
  */
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
  /* struct timeval TCX_waiting_time = {0, 0}; */
  float minScale = 1;
  char *moduleName = TCX_USER_MODULE_NAME;
  char *mapFileName = NULL;
  char *positionFileName = NULL;
  bool usePositionFile = FALSE;
  simulatorMap simMap;
  probabilityGrid gMap;
  char *name;

  struct timeval block_waiting_time;
  struct timeval tcx_waiting_time;
  fd_set readMask;

  int data;
  PANTILT_move_type pan_data;

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



  /* Poslib init */
  initperl();
  call_poslibinit();

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
    tcxSetModuleName(TCX_PANTILT_MODULE_NAME, NULL, pantiltName[0]);
    name = "robot";
  }
  else {
    for(i=0; i < numberOfRobots; i++){
      name = robotName[i];
      tcxSetModuleName(TCX_BASE_MODULE_NAME, robotName[i], baseName[i]);
      tcxSetModuleName(TCX_LOCALIZE_MODULE_NAME, robotName[i], localizeName[i]);
      tcxSetModuleName(TCX_PLAN_MODULE_NAME, robotName[i], planName[i]);
      tcxSetModuleName(TCX_PANTILT_MODULE_NAME, robotName[i], pantiltName[i]);
    }
  }
   
 /* ptRegister(); */
  
  fprintf(stderr, "%s\n", name);

  /* print information */
  fprintf(stderr, "Connecting to robots: ");
  for(i=0; i < numberOfRobots; i++) {
    fprintf(stderr, "%s ", robotName[i]);
  }
  fprintf(stderr, "\n");
 

  /* initializing TCX */
  initTcx(moduleName);

 /* ptConnect(1); */

/*  PANTILT = tcxConnectOptional(TCX_PANTILT_MODULE_NAME); */

  start_goal_timer();
  nogoals = 1;

  /* 
   * Main loop
   */  
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
      if ( pantilt[i] == NULL) {
        pantilt[i] = tcxConnectOptional(pantiltName[i]);
        if (pantilt[i] != NULL){
          fprintf(stderr, "Connected to %s\n", pantiltName[i]);
          data = 1;       
          tcxSendMsg(pantilt[i], "PANTILT_init_query", &data);
        }
      }
    }
    
     /*
      * Set robot location
      *
      *LOCALIZE_set_robot_position_type locPos;
      *locPos.x = pos.x;
      *locPos.y = pos.y;
      *locPos.rot = pos.rot;
      *fprintf( stderr, "Send position (%f %f %f) to %s...",
      *	 	locPos.x, locPos.y, locPos.rot, localizeName[n]);
      *tcxSendMsg(localize[n], "LOCALIZE_set_robot_position", &locPos);
      *fprintf( stderr, " done.\n"); 
      */


     /* Hmm, might want to do this anyways */
    /* if ( usePositionFile) */
      updatePlanStatus(0);  

     if (stopped) {
       if(check_timer()) {
         /* We are done waiting, move on to next goal */
         stopped = 0;
         if ((mynumgoals <= 1) && (arrived)) {
           system("/home/brudy/posServer/speakit2.pl Timer expired.");
           /* If we send a planStartAutonomous(0) when there are
            * no goals in the planner, the robot begins
            * exploring autonomously!  
            */
            nogoals = 1;
         }
         else {
           nogoals = 0;
           system("/home/brudy/posServer/speakit2.pl Timer expired, I am going to the next goal.");
           checkNewgoals(1);
           (void) planStartAutonomous(0);
         }
         pan_data.pan_target = 0.0;
         pan_data.tilt_target = 0.0;

         tcxSendMsg(pantilt[0], "PANTILT_stop_tracking", NULL);
         tcxSendMsg(pantilt[0], "PANTILT_move", &pan_data);
         /* Restart reaction */
         system("/home/brudy/posServer/reaction-up.pl&"); 
       }
     }

    /*
     * do nothing for a while - "select" will automatically terminate
     * when a message has arrived, but won't use up precious CPU
     * time while waiting for it.
     */

    block_waiting_time.tv_sec  = 0;
    block_waiting_time.tv_usec = 100000;
    readMask = (Global->tcxConnectionListGlobal);
    select(FD_SETSIZE, &readMask, NULL, NULL, &block_waiting_time);


    /*
     * query TCX
     */ 
    
    tcx_waiting_time.tv_sec = 0;
    tcx_waiting_time.tv_usec = 0;
    tcxRecvLoop((void *) &tcx_waiting_time); 


  }
  exit(0);			/* should never reach here! */
}


