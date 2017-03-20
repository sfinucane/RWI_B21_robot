
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
 ***** Please read and make sure you understand the disclaimer below.
 *****
 ***** Contact thrun@cs.cmu.edu if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                                     (c) Sebastian Thrun, 1997
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/learn/io.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 16:36:05 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: io.c,v $
 * Revision 1.1  2002/09/14 16:36:05  rstone
 * *** empty log message ***
 *
 * Revision 1.26  1999/07/11 18:48:33  thrun
 * slight reorganization
 *
 * Revision 1.25  1998/08/22 03:08:36  thrun
 * .
 *
 * Revision 1.24  1998/06/27 22:13:42  thrun
 * more tuneups
 *
 * Revision 1.23  1998/05/05 04:00:34  thrun
 * Direct control mode (and button). Other stuff I don't fully remember.
 * Intermediate version.
 *
 * Revision 1.22  1998/04/18 20:42:25  thrun
 * 1. neuro net can now be called with the same variable as input
 * and output
 * 2. direct control modus for the robot. controls are
 * now generated exclusively through the display (left upper corner)
 *
 * Revision 1.21  1998/04/14 02:35:55  thrun
 * .
 *
 * Revision 1.20  1998/04/13 01:45:18  thrun
 * Thermomter encoding for sigmodial neural networks. Delay for
 * pausing (when recording data).
 *
 * Revision 1.19  1997/10/23 02:28:22  thrun
 * .
 *
 * Revision 1.18  1997/10/05 18:11:18  thrun
 * new data library "libdat.a"
 *
 * Revision 1.17  1997/07/30 23:29:34  thrun
 * recording of base motion commands (velocity).
 *
 * Revision 1.16  1997/07/30 21:02:02  thrun
 * baseServer now sends confirmations of all motion commands. Is used
 * for logging in the module "learn"
 *
 * Revision 1.15  1997/07/30 04:11:08  thrun
 * minor bug
 *
 * Revision 1.14  1997/07/30 03:46:53  thrun
 * New events: pantilt and buttons
 *
 * Revision 1.13  1997/07/29 22:44:47  thrun
 * .
 *
 * Revision 1.12  1997/07/26 14:19:04  thrun
 * changed display options
 *
 * Revision 1.11  1997/07/26 13:38:48  thrun
 * Reacted to some of Tyson's changes
 *
 * Revision 1.10  1997/07/07 04:45:23  thrun
 * Now with training and testing set support
 *
 * Revision 1.9  1997/07/04 19:26:54  thrun
 * yet another intermediate version
 *
 * Revision 1.8  1997/06/29 04:04:55  thrun
 * intrmediate version - not really good for anything
 *
 * Revision 1.7  1997/06/28 13:41:41  thrun
 * This is the fully functional recorder/display/analysis tool.
 * Check out this version if you'd like to have it (without any
 * of the learning stuff). What's missing is the ability to feed back
 * the data intp the baseServer, colli and camera. Also missing is a link
 * to the pantilt unit.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <signal.h>


#include "EZX11.h"
#include "o-graphics.h"


#include "rai.h"
#include "devUtils.h"
#include "raiClient.h"
#include "baseClient.h"
#include "sonarClient.h"
#include "tactileClient.h"
#include "rai.h"
#include "raiClient.h"
#include "cameraClient.h"
#include "buttonClient.h"
#include "pantiltClient.h"
#include "robot_specifications.h"


#define TCX_define_variables
#include <BASE-messages.h>
#define DEFINE_REPLY_HANDLERS
#include <LASER-messages.h>

#include "dat.h"
#include "mem.h"
#include "global.h"



/************************************************************************\
 ************************************************************************
\************************************************************************/


robot_position_type *current_pos  = NULL;


/*int cameraRequestPending = 0;*/
int baseConnected = 0;
int colliConnected = 0;


int first_irs = 1;		/* only relevant for recording of sensors */
int first_sonars = 1;
int first_tactiles = 1;
int first_lasers = 1;
int first_buttons = 1;
int first_pantilt = 1;
int first_control = 1;


int robot_stalled = 0;

int io_verbose = 0;


control_type desired_control = {0, 0.0, 0.0};

/************************************************************************\
 ************************************************************************
\************************************************************************/


void 
commShutdown(char *name, TCX_MODULE_PTR module)
{

  if (!strcmp(name, "baseTCXServer")){ /* BASE shut down */
    G_display_switch(BASE_CONNECT_BUTTON, 0);
    fprintf(stderr,
	    "TCX: disconnected from baseServer. Trying to reconnect.\n");
    baseConnected = 0;
    first_irs = 1;
    first_sonars = 1;
    first_tactiles = 1;
    first_control = 1;
    if (current_pos){
      free(current_pos);
      current_pos = NULL;
    }
    robot_stalled = 0;
    if (global_modus_recording == 1){
      global_modus_recording = 2;
      G_display_switch(RECORD_BUTTON, global_modus_recording);
    }
  }

  else if (!strcmp(name, "BASE")){ /* COLLI shut down */
    G_display_switch(COLLI_CONNECT_BUTTON, 0);
    fprintf(stderr,
	    "TCX: disconnected from colliServer. Trying to reconnect.\n");
    colliConnected = 0;
    first_lasers = 1;
  }

  else if (!strcmp(name, "CAMERA")){ /* CAMERA shut down */
    cameraConnected = 0;
    G_display_switch(CAMERA_CONNECT_BUTTON, 0);
    fprintf(stderr, 
	    "TCX: disconnected from cameraServer. Trying to reconnect.\n");
    /*cameraRequestPending = 0;*/
  }


  else if (!strcmp(name, "BUTTONS")){ /* BUTTONS shut down */
    buttonConnected = 0;
    G_display_switch(BUTTONS_CONNECT_BUTTON, 0);
    fprintf(stderr, 
	    "TCX: disconnected from buttonServer. Trying to reconnect.\n");
    first_buttons = 1;
  }

  else if (!strcmp(name, "PANTILT")){ /* PANTILT shut down */
    ptConnected = 0;
    G_display_switch(PANTILT_CONNECT_BUTTON, 0);
    fprintf(stderr, 
	    "TCX: disconnected from pantilterver. Trying to reconnect.\n");
    first_pantilt = 1;
  }

  else if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    fprintf(stderr, "TCX went down. Must exit.\n");
    exit(0);
  }
  else
      fprintf(stderr, "TCX: closed connection detected: %s\n", name);

}

/************************************************************************\
 ************************************************************************
\************************************************************************/


int
sonarCallback(sonarType *newSonar)
{
  int i, new;
  static int internal_sonars_initialized = 0;
  static float internal_sonars[NUM_SONAR_SENSORS];
  static int   internal_sonars_new[NUM_SONAR_SENSORS];
  struct pattern_type *pattern = NULL;
  robot_position_type pos;
  struct timeval time;
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference; 


  if (!internal_sonars_initialized){
    for (i = 0; i < NUM_SONAR_SENSORS; i++){
      internal_sonars[i] = 0.5 * (MAX_SONAR_DISPLAY_RANGE);
      internal_sonars_new[i] = 0;
    }
    internal_sonars_initialized = 1;
  }


  /*
   * copy sonars scan
   */

  for (i = 0; i < NUM_SONAR_SENSORS; i++)
    if (newSonar[i].mostRecent){
      internal_sonars[i] =  0.1 * newSonar[i].value; /* 1cm = 10Tysons */
      internal_sonars_new[i] = 1;
      newSonar[i].mostRecent = 0;
    }

  /*
   * count new
   */
  for (new = 0, i = 0; i < NUM_SONAR_SENSORS; i++)
    if (internal_sonars_new[i])
      new++;


  if (new == NUM_SONAR_SENSORS){

    /*
     * check, if sufficient time elapsed
     */

    gettimeofday(&this_time, NULL);
    time_difference = 
      ((float) (this_time.tv_sec - last_time.tv_sec))
      + (((float) (this_time.tv_usec - last_time.tv_usec))
	 /  1000000.0);


    if (time_difference >= MIN_TIME_DIFF_SONAR){
      
      pattern = mem_create_pattern();
      gettimeofday(&time, NULL);
      mem_fill_pattern_slot(pattern, "sonar", &time, NULL, internal_sonars,
			    NULL, NULL, NULL, NULL, NULL, current_pos, NULL,
			    NULL);

      mem_display_pattern(NULL, pattern, global_modus_live,
			  global_modus_execution);


      if (global_modus_recording == 2){
	if (current_patternset == NULL){
	  fprintf(stderr, "STRANGE: This shouldn't happen.\n");
	  current_patternset = mem_create_patternset();
	  mem_fill_pattern_set_slot(current_patternset, "recorded", -1);
	}
	mem_append_pattern(current_patternset, pattern);
	update_items_button();
      }
      else
	mem_clear_pattern(pattern);

      
      /*
       * mark as old
       */
      
      for (i = 0; i < NUM_SONAR_SENSORS; i++)
	internal_sonars_new[i] = 0;

      
      if (io_verbose)
	fprintf(stderr, "Sonar: %g sec\n", time_difference);
      last_time.tv_sec = this_time.tv_sec;
      last_time.tv_usec = this_time.tv_usec;
    }
  }    
}



/************************************************************************\
 ************************************************************************
\************************************************************************/


int
irCallback(irType **newIr)
{
  int i, j, new, index, change;
  static int internal_irs_initialized = 0;
  static float internal_irs[NUM_IR_SENSORS];
  static float prev_internal_irs[NUM_IR_SENSORS];
  static int   internal_irs_new[NUM_IR_SENSORS];
  struct pattern_type *pattern = NULL;
  robot_position_type pos;
  struct timeval time;
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference; 


  if (!internal_irs_initialized){
    for (i = 0; i < NUM_IR_SENSORS; i++){
      internal_irs[i] = 0.0;
      prev_internal_irs[i] = -1.0;
      internal_irs_new[i] = 0;
    }
    internal_irs_initialized = 1;
    first_irs = 1;
  }



  /*
   * copy irs scan
   */

  for (index = 0, i = 0, j = 0; index < NUM_IR_SENSORS; index++, j++){
    if (i == 0 && j == NUM_IR_SENSORS_AT_HEIGHT_1){
      i++;
      j = 0;
    }
    else if (i == 1 && j == NUM_IR_SENSORS_AT_HEIGHT_2){
      i++;
      j = 0;
    }
    else if (i == 2 && j == NUM_IR_SENSORS_AT_HEIGHT_3){
      fprintf(stderr, "STRANGE: internal counting error. Check global.h.\n");
      exit(-1);
    }
    /* fprintf(stderr, "ir: %d %d <- %d\n", i, j, index); */
    internal_irs[index] =  (float) newIr[i][j].value;
    internal_irs_new[index] = 1;
    newIr[i][j].mostRecent = 0;
  }

  
  
  /*
   * count new
   */
  for (new = 0, i = 0; i < NUM_IR_SENSORS; i++)
    if (internal_irs_new[i])
      new++;




  if (new == NUM_IR_SENSORS){


    /*
     * check, if sufficient time elapsed
     */
    
    gettimeofday(&this_time, NULL);
    time_difference = 
      ((float) (this_time.tv_sec - last_time.tv_sec))
      + (((float) (this_time.tv_usec - last_time.tv_usec))
	 /  1000000.0);
    
    if (time_difference >= MIN_TIME_DIFF_IR){
      
      /*
       * check, if anything changed since last reading
       * we'll only record, if there was a change.
       */

      change = first_irs;
      for (i = 0; i < NUM_IR_SENSORS; i++){
	if (prev_internal_irs[i] != internal_irs[i]){
	  prev_internal_irs[i] = internal_irs[i];
	  change = 1;
	}
      }



      /*
       * diaplay/record ir scan
       */
      
      pattern = mem_create_pattern();
      gettimeofday(&time, NULL);
      mem_fill_pattern_slot(pattern, "infrared", &time, NULL, NULL,
			    internal_irs, NULL, NULL, NULL, NULL,
			    current_pos, NULL,
			    NULL);
      mem_display_pattern(NULL, pattern, global_modus_live,
			  global_modus_execution);


      if (global_modus_recording == 2){
	if (current_patternset == NULL){
	  fprintf(stderr, "STRANGE: This shouldn't happen.\n");
	  current_patternset = mem_create_patternset();
	  mem_fill_pattern_set_slot(current_patternset, "recorded", -1);
	}
	mem_append_pattern(current_patternset, pattern);
	update_items_button();
      }
      else
	mem_clear_pattern(pattern);


      /*
       * mark as old
       */
    
    
      for (i = 0; i < NUM_IR_SENSORS; i++)
	internal_irs_new[i] = 0;
    
      first_irs = 0;

      if (io_verbose)
	fprintf(stderr, "Infrared: %g sec\n", time_difference);
      last_time.tv_sec = this_time.tv_sec;
      last_time.tv_usec = this_time.tv_usec;
    
    }
  }    

}  


/************************************************************************\
 ************************************************************************
\************************************************************************/

int
tactileCallback(tactileType **newTactile)
{

  int i, j, index, change;
  static int internal_tactiles_initialized = 0;
  static float internal_tactiles[NUM_TACTILE_SENSORS];
  static float prev_internal_tactiles[NUM_TACTILE_SENSORS];
  struct pattern_type *pattern = NULL;
  robot_position_type pos;
  struct timeval time;
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference; 


  if (!internal_tactiles_initialized){
    for (i = 0; i < NUM_TACTILE_SENSORS; i++){
      internal_tactiles[i] = 0.0;
      prev_internal_tactiles[i] = -1.0;
    }
    internal_tactiles_initialized = 1;
    first_tactiles = 1;
  }


  /*
   * copy tactiles scan
   */

  for (index = 0, i = 0, j = 0; index < NUM_TACTILE_SENSORS; index++, j++){
    if (i == 0 && j == NUM_TACTILE_SENSORS_AT_HEIGHT_1){
      i++;
      j = 0;
    }
    else if (i == 1 && j == NUM_TACTILE_SENSORS_AT_HEIGHT_2){
      i++;
      j = 0;
    }
    else if (i == 2 && j == NUM_TACTILE_SENSORS_AT_HEIGHT_3){
      i++;
      j = 0;
    }
    else if (i == 3 && j == NUM_TACTILE_SENSORS_AT_HEIGHT_4){
      fprintf(stderr, "STRANGE: internal counting error. Check global.h.\n");
      exit(-1);
    }
    /* fprintf(stderr, "tactile: %d %d <- %d\n", i, j, index); */
    internal_tactiles[index] =  (float) newTactile[i][j].value;
  }

  
  

  
  
  /*
   * check, if anything changed since last reading
   * we'll only record, if there was a change.
   */
  
  change = first_tactiles;
  for (i = 0; i < NUM_TACTILE_SENSORS; i++){
    if (prev_internal_tactiles[i] != internal_tactiles[i]){
      prev_internal_tactiles[i] = internal_tactiles[i];
      change = 1;
    }
  }



  /*
   * check, if sufficient time elapsed
   */
    
  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
    
  if (time_difference >= MIN_TIME_DIFF_TACTILE){
    
    /*
     * display/record tactile scan
     */
  
    
    pattern = mem_create_pattern();
    gettimeofday(&time, NULL);
    mem_fill_pattern_slot(pattern, "tactile", &time, NULL, NULL,
			  NULL, internal_tactiles, NULL, NULL, NULL,
			  current_pos, NULL,
			  NULL);
    mem_display_pattern(current_patternset, pattern, global_modus_live,
			global_modus_execution);
    
    if (global_modus_recording == 2 && change){    
      if (current_patternset == NULL){
	fprintf(stderr, "STRANGE: This shouldn't happen.\n");
	current_patternset = mem_create_patternset();
	mem_fill_pattern_set_slot(current_patternset, "recorded", -1);
      }
      mem_append_pattern(current_patternset, pattern);
      update_items_button();
    }
    else
      mem_clear_pattern(pattern);
    
    /*
     * mark as old
     */
    
    
    first_tactiles = 0;


    if (io_verbose)
      fprintf(stderr, "Tactile: %g sec\n", time_difference);
    last_time.tv_sec = this_time.tv_sec;
    last_time.tv_usec = this_time.tv_usec;


  }  
}

/************************************************************************\
 ************************************************************************
\************************************************************************/

void
statusCallback(statusReportType * newStatus)
{
  static int gotOrigin = FALSE;
  robot_position_type pos;
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference, time_difference_motion, diff; 
  float  delta_dist, delta_angle;
  static struct timeval last_motion_time = {0, 0};
  static unsigned long prevClock = 0;

  if (!gotOrigin) {
    bSetPosition(0.0, 0.0, 0.0);
    gotOrigin=TRUE;
  }


  if (!current_pos){
    current_pos = (robot_position_type *) malloc(sizeof(robot_position_type));
    current_pos->x = bRobotX(0.0);
    current_pos->y = bRobotY(0.0);
    current_pos->orientation = 
      (bNormalizeAngle(bRobotHeading(0.1)) * 180.0 / M_PI);
    if (!current_pos){
      fprintf(stderr, "ERROR: Out of memory in statusCallback(),\n");
      exit(-1);
    }
  }


  watchdogTimer(0x800);  

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);

  if (time_difference >= MIN_TIME_DIFF_STATUS){
    
    /*
     * copy robot coordinates
     */

    pos.x           = bRobotX(0.0);
    pos.y           = bRobotY(0.0);
    pos.orientation = (bNormalizeAngle(bRobotHeading(0.1)) * 180.0 / M_PI);


    /*
     * renew the motion command, and display it
     */

    set_velocity(desired_control.trans_velocity, 
		 desired_control.rot_velocity, 1);

    /*
     * check status
     */

    if (!current_pos ||
	(pos.x == current_pos->x &&
	 pos.y == current_pos->y &&
	 pos.orientation == current_pos->orientation) ||
	prevClock == newStatus->Clock){
      time_difference_motion = 
	((float) (this_time.tv_sec - last_motion_time.tv_sec))
	+ (((float) (this_time.tv_usec - last_motion_time.tv_usec))
	   /  1000000.0);
      delta_dist = delta_angle = 0.0;

      if (time_difference_motion >= MIN_STALLED_TIME_FOR_PAUSING &&
	  global_modus_suspend_recording_when_not_moving)
	robot_stalled = 1;
    }
    else{
      robot_stalled = 0;
      last_motion_time.tv_sec  = this_time.tv_sec;
      last_motion_time.tv_usec = this_time.tv_usec;

      delta_dist = sqrt(((current_pos->x - pos.x) * (current_pos->x - pos.x)) +
			((current_pos->y - pos.y) * (current_pos->y - pos.y)));
      delta_angle = pos.orientation - current_pos->orientation;
      for (; delta_angle < -180.0;) delta_angle += 360.0;
      for (; delta_angle >= 180.0;) delta_angle -= 360.0;
      diff = (float) (newStatus->Clock - prevClock);
      if (io_verbose)
	fprintf(stderr, "vel: %f %f   %f\n",
		delta_dist / diff, delta_angle / diff, diff);
    }
    
   
    
    if ((robot_stalled && global_modus_recording == 2) ||
	(!robot_stalled && global_modus_recording == 1)){
      global_modus_recording = 3 - global_modus_recording;
      G_display_switch(RECORD_BUTTON, global_modus_recording);
    }

    current_pos->x = pos.x;
    current_pos->y = pos.y;
    current_pos->orientation = pos.orientation;


    
    
    if (io_verbose){
      fprintf(stderr, "Status: %g sec (%d, %g %g %g), clock=%ld\n",
	      time_difference, robot_stalled, pos.x, pos.y, pos.orientation,
	      newStatus->Clock - prevClock);
    }
    last_time.tv_sec = this_time.tv_sec;
    last_time.tv_usec = this_time.tv_usec;
    prevClock = newStatus->Clock;
  }
}

/************************************************************************\
 ************************************************************************
\************************************************************************/

void
baseCommandCallback(confirmCommandDataType* data)
{
  int   operation;
  static float set_translate_velocity      = 0.0;
  static float set_rotate_velocity         = 0.0;
  static int   translating                 = 0;	/* 1=pos, -1=neg, 0=no */
  static int   rotating                    = 0;	/* 1=pos, -1=neg, 0=no */
  static control_type control, prev_control;
  static int prev_control_intialized = 0;
  struct timeval time;
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference; 
  struct pattern_type *pattern = NULL;
  int    change;
  

  /* NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE 
   * NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE 
   *
   *         this feature has been disabled.
   *
   * NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE 
   * NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE */


  return;

  /* NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE 
   * NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE 
   *
   *         this feature has been disabled.
   *
   * NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE 
   * NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE  NOTICE */



  /*
   * We are only interested in certain types of commands
   */

  operation = (int) data->operation;

  if (operation != BASE_baseKill             &&	/* 6 in baseMessages.h */
      operation != BASE_rotateHalt           &&	/* 16 */
      operation != BASE_rotateVelocityPos    &&	/* 17 */
      operation != BASE_rotateVelocityNeg    &&	/* 18 */
      operation != BASE_setRotateVelocity    &&	/* 31 */
      operation != BASE_translateHalt        &&	/* 33 */
      operation != BASE_translateVelocityPos &&	/* 34 */
      operation != BASE_translateVelocityNeg &&	/* 35 */
      operation != BASE_setTranslateVelocity  ) /* 47 */
    return;

  /*
   * update the status
   */


  /*fprintf(stderr, " <%d:%d>\n", (int) data->operation, (int) data->param);*/

  switch(operation){

  case BASE_baseKill:
    translating                  = 0;
    rotating                     = 0;
    break;

  case BASE_rotateHalt:
    rotating                     = 0;
    break;

  case BASE_rotateVelocityPos:
    rotating                     = 1;
    break;

  case BASE_rotateVelocityNeg:   
    rotating                     = -1;
    break;

  case BASE_setRotateVelocity:
    set_rotate_velocity          = (float) data->param;
    break;

  case BASE_translateHalt:
    translating                  = 0;
    break;

  case BASE_translateVelocityPos:
    translating                  = 1;
    break;

  case BASE_translateVelocityNeg:
    translating                  = -1;
    break;

  case BASE_setTranslateVelocity:
    set_translate_velocity       = (float) data->param;
    break;
  }
  if (translating == 1)
    control.trans_velocity       = set_translate_velocity;
  else if (translating == -1)
    control.trans_velocity       = -set_translate_velocity;
  else
    control.trans_velocity       = 0.0;

  if (rotating == 1)
    control.rot_velocity         = set_rotate_velocity;
  else if (rotating == -1)
    control.rot_velocity         = -set_rotate_velocity;
  else   
    control.rot_velocity         = 0.0;

  control.command_type = BASE_COMMAND;


  change = (first_control || !prev_control_intialized ||
	    control.trans_velocity != prev_control.trans_velocity ||
	    control.rot_velocity   != prev_control.rot_velocity);

/*   fprintf(stderr, "%d", change); */


  /*
   * Check, if we'd like to register this event
   */
  
  if (change){
    
    gettimeofday(&this_time, NULL);
    time_difference = 
      ((float) (this_time.tv_sec - last_time.tv_sec))
      + (((float) (this_time.tv_usec - last_time.tv_usec))
	 /  1000000.0);
    
    if (time_difference >= MIN_TIME_DIFF_CONTROL){
      
      /*
       * Display/Recording
       */
      
      pattern = mem_create_pattern();
      gettimeofday(&time, NULL);
      mem_fill_pattern_slot(pattern, "control", &time, NULL, NULL, NULL,
			    NULL, NULL, NULL, NULL, current_pos, NULL,
			    &control);
      mem_display_pattern(current_patternset, pattern, global_modus_live,
			  global_modus_execution);
      
      if (global_modus_recording == 2){
	if (current_patternset == NULL){
	  fprintf(stderr, "STRANGE: This shouldn't happen.\n");
	  current_patternset = mem_create_patternset();
	  mem_fill_pattern_set_slot(current_patternset, "recorded", -1);
	}
	mem_append_pattern(current_patternset, pattern);
	update_items_button();
      }
      else
	mem_clear_pattern(pattern);
      
      
      if (io_verbose){
	if (first_control)
	  fprintf(stderr, "Control:\n");
	else
	  fprintf(stderr, "Control: %g sec\n", time_difference);
      }
      first_control = 0;
      last_time.tv_sec  = this_time.tv_sec;
      last_time.tv_usec = this_time.tv_usec;
      prev_control.trans_velocity = control.trans_velocity;
      prev_control.rot_velocity   = control.rot_velocity;
      prev_control_intialized = 1;
      
    }
  }
}

/************************************************************************\
 ************************************************************************
\************************************************************************/

void 
LASER_laser_reply_handler(TCX_REF_PTR                ref,
			  LASER_laser_reply_ptr      data)
{
  int i;
  static int internal_lasers_initialized = 0;
  static float internal_lasers[NUM_LASER_SENSORS];
  struct pattern_type *pattern = NULL;
  robot_position_type pos;
  struct timeval time;
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference; 

  /*  fprintf(stderr, "LASER: %d %d\n", data->f_numberOfReadings, 
      data->r_numberOfReadings); */


  if (!internal_lasers_initialized){
    for (i = 0; i < NUM_LASER_SENSORS; i++)
      internal_lasers[i] = 0.5 * (MAX_LASER_DISPLAY_RANGE);
    internal_lasers_initialized = 1;
  }


  /*
   * copy lasers scan
   */

  for (i = 0; i < NUM_LASER_SENSORS_FRONT; i++)
    if (data->f_numberOfReadings > i)
      internal_lasers[i] = (float) data->f_reading[i];
    else
      internal_lasers[i] = -1.0;

  for (i = 0; i < NUM_LASER_SENSORS_REAR; i++)
    if (data->r_numberOfReadings > i)
      internal_lasers[i+NUM_LASER_SENSORS_FRONT] = (float) data->r_reading[i];
    else
      internal_lasers[i+NUM_LASER_SENSORS_FRONT] = -1.0;


  /*
   * check, if sufficient time elapsed
   */
    
  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
    
  if (time_difference >= MIN_TIME_DIFF_LASER){
    

    /*
     * display/record laser scan
     */

    pattern = mem_create_pattern();
    gettimeofday(&time, NULL);
    mem_fill_pattern_slot(pattern, "laser", &time, NULL, NULL,
			  NULL, NULL, internal_lasers, NULL, NULL, 
			  current_pos, NULL,
			  NULL);
    mem_display_pattern(current_patternset, pattern, global_modus_live,
			global_modus_execution);
    
    if (global_modus_recording == 2){
      if (current_patternset == NULL){
	fprintf(stderr, "STRANGE: This shouldn't happen.\n");
	current_patternset = mem_create_patternset();
	mem_fill_pattern_set_slot(current_patternset, "recorded", -1);
      }
      mem_append_pattern(current_patternset, pattern);
      update_items_button();
    }
    else
      mem_clear_pattern(pattern);
    
    
    if (io_verbose)
      fprintf(stderr, "Laser: %g sec\n", time_difference);
    last_time.tv_sec = this_time.tv_sec;
    last_time.tv_usec = this_time.tv_usec;
  }

  tcxFree("LASER_laser_reply", data);
}


/************************************************************************\
 ************************************************************************
\************************************************************************/

void
baseCallback(unsigned long opcode, unsigned long value)
{
  fprintf(stderr, "Base\n");
}

/************************************************************************\
 ************************************************************************
\************************************************************************/

void
watchdogCallback(unsigned long opcode, unsigned long value)
{
  fprintf(stderr, "Watchdog\n");
}


/************************************************************************\
 ************************************************************************
\************************************************************************/

void
demoPoll(RaiModule * demoModule)
{
  fprintf(stderr, "Demo\n");  
}

/************************************************************************\
 ************************************************************************
\************************************************************************/



int
cameraCallback(cameraImageType *camera)
{
  int x, y, i, j, k;
  struct pattern_type *pattern = NULL;
  unsigned char *image;
  struct timeval time;
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference; 


  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);


	  

  if (time_difference >= MIN_TIME_DIFF_CAMERA){

    
    /*
     * Display/record camera image
     */
    
    image = (unsigned char *) malloc(sizeof(unsigned char) 
				     * NUM_PIXELS * 3);
    for (i = 0, y = 0, k = 0; y < IMAGE_SIZE_Y; y++)
      for (x = 0; x < IMAGE_SIZE_X; x++, i++){
	image[k++] = camera->red[i];
	image[k++] = camera->green[i];
	image[k++] = camera->blue[i];
      }
    pattern = mem_create_pattern();
    gettimeofday(&time, NULL);
    mem_fill_pattern_slot(pattern, "image", &time, image, NULL,
			  NULL, NULL, NULL, NULL, NULL, current_pos, NULL,
			  NULL);
    mem_display_pattern(current_patternset, pattern, global_modus_live,
			global_modus_execution);

    if (global_modus_recording == 2){
      if (current_patternset == NULL){
	fprintf(stderr, "STRANGE: This shouldn't happen.\n");
	current_patternset = mem_create_patternset();
	mem_fill_pattern_set_slot(current_patternset, "recorded", -1);
      }
      mem_append_pattern(current_patternset, pattern);
      update_items_button();
    }
    else
      mem_clear_pattern(pattern);
    
    free(image);


    /*cameraRequestPending = 2;*/
    
    
    if (io_verbose)
      fprintf(stderr, "Camera: %g sec\n", time_difference);
    last_time.tv_sec = this_time.tv_sec;
    last_time.tv_usec = this_time.tv_usec;
  }
}

/************************************************************************\
 ************************************************************************
\************************************************************************/

int
buttonCallback(buttonStatusType *data)
{
  int i;
  struct pattern_type *pattern = NULL;
  buttons_type buttons;
  struct timeval time;
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference; 


  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
  if (time_difference >= MIN_TIME_DIFF_BUTTONS){

    /*
     * Copy values
     */

    if (data->red_light_status)
      buttons.lit[0] = 1;
    else
      buttons.lit[0] = 0;
    if (data->yellow_light_status)
      buttons.lit[1] = 1;
    else
      buttons.lit[1] = 0;
    if (data->green_light_status)
      buttons.lit[2] = 1;
    else
      buttons.lit[2] = 0;
    if (data->blue_light_status)
      buttons.lit[3] = 1;
    else
      buttons.lit[3] = 0;
    if (data->left_kill_switch_light_status)
      buttons.lit[4] = 1;
    else
      buttons.lit[4] = 0;
    if (data->right_kill_switch_light_status)
      buttons.lit[5] = 1;
    else
      buttons.lit[5] = 0;
  
    if (data->red_button_pressed)
      buttons.pushed[0] = 1;
    else
      buttons.pushed[0] = 0;
    if (data->yellow_button_pressed)
      buttons.pushed[1] = 1;
    else
      buttons.pushed[1] = 0;
    if (data->green_button_pressed)
      buttons.pushed[2] = 1;
    else
      buttons.pushed[2] = 0;
    if (data->blue_button_pressed)
      buttons.pushed[3] = 1;
    else
      buttons.pushed[3] = 0;

    
    /*
     * Display/Recording
     */
    
    pattern = mem_create_pattern();
    gettimeofday(&time, NULL);
    mem_fill_pattern_slot(pattern, "buttons", &time, NULL, NULL, NULL,
			  NULL, NULL, &buttons, NULL, current_pos, NULL,
			  NULL);
    mem_display_pattern(current_patternset, pattern, global_modus_live,
			global_modus_execution);
    
    if (global_modus_recording){
      if (current_patternset == NULL){
	fprintf(stderr, "STRANGE: This shouldn't happen.\n");
	current_patternset = mem_create_patternset();
	mem_fill_pattern_set_slot(current_patternset, "recorded", -1);
      }
      mem_append_pattern(current_patternset, pattern);
      update_items_button();
    }
    else
      mem_clear_pattern(pattern);
    
    
    if (io_verbose){
      if (first_buttons)
	fprintf(stderr, "Buttons:\n");
      else
	fprintf(stderr, "Buttons: %g sec\n", time_difference);
    }
    first_buttons = 0;
    last_time.tv_sec  = this_time.tv_sec;
    last_time.tv_usec = this_time.tv_usec;
  }
}


/************************************************************************\
 ************************************************************************
\************************************************************************/

void
pantiltCallback()
{
  int i;
  struct pattern_type *pattern = NULL;
  float pantilt[2];
  struct timeval time;
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference; 


  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
  if (time_difference >= MIN_TIME_DIFF_PANTILT){

    /*
     * Copy values
     */

    pantilt[0] = ptStatusPanPos;
    pantilt[1] = ptStatusTiltPos;

    
    /*
     * display/record
     */
    
    pattern = mem_create_pattern();
    gettimeofday(&time, NULL);
    mem_fill_pattern_slot(pattern, "pantilt", &time, NULL, NULL, NULL,
			  NULL, NULL, NULL, pantilt, current_pos, NULL,
			  NULL);
    mem_display_pattern(current_patternset, pattern, global_modus_live,
			global_modus_execution);

    if (global_modus_recording == 2){
      if (current_patternset == NULL){
	fprintf(stderr, "STRANGE: This shouldn't happen.\n");
	current_patternset = mem_create_patternset();
	mem_fill_pattern_set_slot(current_patternset, "recorded", -1);
      }
      mem_append_pattern(current_patternset, pattern);
      update_items_button();
    }
    else
      mem_clear_pattern(pattern);
    
    
    if (io_verbose){
      if (first_pantilt)
	fprintf(stderr, "Pantilt:\n");
      else
	fprintf(stderr, "Pantilt: %g sec\n", time_difference);
    }
    first_pantilt = 0;
    last_time.tv_sec  = this_time.tv_sec;
    last_time.tv_usec = this_time.tv_usec;
  }


}

/************************************************************************\
 ************************************************************************
\************************************************************************/

/*
 * This routine sets the velocity of the robot. It also
 * displays this velocity and, if we are in recording mode, 
 * records it.
 */

void
set_velocity(float desired_trans_velocity,
	     float desired_rot_velocity,
	     int immediate)
{
  unsigned long       vel;
  static control_type current_control;
  static              struct timeval last_time = {0, 0};
  struct              timeval this_time;
  float               time_difference;
  struct pattern_type *pattern = NULL;
  struct timeval      time;



  /*
   * safe the desired value (we might want to use it later)
   */ 

  desired_control.command_type   = 999;
  desired_control.trans_velocity = desired_trans_velocity;
  desired_control.rot_velocity   = desired_rot_velocity;

  /*
   * check, if we have to update/display the new velocity
   */

  if ((immediate || !baseConnected) && /* the velocity is only set
					* then the "immediate" flag
					* is raised (or when there is
					* no robot) */
      (current_control.command_type   != desired_control.command_type||
       current_control.trans_velocity != desired_trans_velocity ||
       current_control.rot_velocity   != desired_rot_velocity)){/* and there
								 * must be a
								 * change */


    /*
     * Now let's check if the timing is right
     */

    gettimeofday(&this_time, NULL);
    time_difference = 
      ((float) (this_time.tv_sec - last_time.tv_sec))
      + (((float) (this_time.tv_usec - last_time.tv_usec))
	 /  1000000.0);
    
    if (time_difference >= MIN_TIME_DIFF_CONTROL){

      /*
       * set the control
       */

      current_control.command_type   = desired_control.command_type;
      current_control.trans_velocity = desired_control.trans_velocity;
      current_control.rot_velocity   = desired_control.rot_velocity;

      if (current_control.trans_velocity == 0.0){
	if (baseConnected){
	  setTranslateVelocity(0);
	  /* fprintf(stderr, "setTranslateVelocity(0);\n"); */
	  translateHalt();
	  /* fprintf(stderr, "translateHalt();\n"); */
	}
	/* G_display_value(TRANS_VEL_POS, 0.0); */
	/* G_display_value(TRANS_VEL_NEG, MAX_TRANS_VEL); */
      }
      else if (current_control.trans_velocity > 0.0){
	vel = (unsigned long) current_control.trans_velocity;
	if (vel > MAX_TRANS_VEL) vel = MAX_TRANS_VEL;
	if (baseConnected){
	  setTranslateVelocity(vel);
	  /* fprintf(stderr, "setTranslateVelocity(%d);\n", (int) vel); */
	  translateVelocityPos();
	  /* fprintf(stderr, "setTranslateVelocityPos()\n"); */
	}
	/* G_display_value(TRANS_VEL_POS, current_control.trans_velocity); */
	/* G_display_value(TRANS_VEL_NEG, MAX_TRANS_VEL); */
      }
      else{
	vel = (unsigned long) (0.0 - current_control.trans_velocity);
	if (vel > MAX_TRANS_VEL) vel = MAX_TRANS_VEL;
	if (baseConnected){
	  setTranslateVelocity(vel);
	  /* fprintf(stderr, "setTranslateVelocity(%d);\n", (int) vel); */
	  translateVelocityNeg();
	  /* fprintf(stderr, "setTranslateVelocityNeg()\n"); */
	}
	/* G_display_value(TRANS_VEL_POS, 0.0); */
	/* G_display_value(TRANS_VEL_NEG, MAX_TRANS_VEL + 
			current_control.trans_velocity); */
      }

      if (current_control.rot_velocity == 0.0){
	if (baseConnected){
	  setRotateVelocity(0);
	  /* fprintf(stderr, "setRotateVelocity(0);\n"); */
	  rotateHalt();
	  /* fprintf(stderr, "rotateHalt();\n"); */
	}
	/* G_display_value(ROT_VEL_POS, 0.0); */
	/* G_display_value(ROT_VEL_NEG, MAX_ROT_VEL); */
      }
      else if (current_control.rot_velocity > 0.0){
	vel = (unsigned long) current_control.rot_velocity;
	if (vel > MAX_ROT_VEL) vel = MAX_ROT_VEL;
	if (baseConnected){
	  setRotateVelocity(vel);
	  /* fprintf(stderr, "setRotateVelocity(%d);\n", (int) vel); */
	  rotateVelocityPos();
	  /* fprintf(stderr, "setRotateVelocityPos()\n"); */
	}
	/* G_display_value(ROT_VEL_POS, current_control.rot_velocity); */
	/* G_display_value(ROT_VEL_NEG, MAX_ROT_VEL); */
      }
      else{
	vel = (unsigned long) (0.0 - current_control.rot_velocity);
	if (vel > MAX_ROT_VEL) vel = MAX_ROT_VEL;
	if (baseConnected){
	  setRotateVelocity(vel);
	  /* fprintf(stderr, "setRotateVelocity(%d);\n", (int) vel); */
	  rotateVelocityNeg();
	  /* fprintf(stderr, "setRotateVelocityNeg()\n"); */
	}
	/* G_display_value(ROT_VEL_POS, 0.0); */
	/* G_display_value(ROT_VEL_NEG, MAX_ROT_VEL + 
			current_control.rot_velocity); */
      }

      /*
       * graphical display
       */
      
      if (global_modus_live && global_modus_recording != 2){
	mem_display_control(&current_control);
	mem_display_pos(current_pos);
	mem_display_time(NULL, NULL);
      }
      
      /*
       * Recording
       */
      
      if (global_modus_recording == 2){
	if (current_patternset == NULL){
	  fprintf(stderr, "STRANGE: This shouldn't happen.\n");
	  current_patternset = mem_create_patternset();
	  mem_fill_pattern_set_slot(current_patternset, "recorded", -1);
	}
	
	pattern = mem_create_pattern();
	gettimeofday(&time, NULL);
	mem_fill_pattern_slot(pattern, "control", &time, NULL, NULL, NULL,
			      NULL, NULL, NULL, NULL, current_pos, NULL,
			      &current_control);
	mem_append_pattern(current_patternset, pattern);
	update_items_button();
	mem_display_pattern(current_patternset, pattern, global_modus_live,
			  global_modus_execution);
      }
      
      
      if (io_verbose){
	if (first_control)
	  fprintf(stderr, "Control:\n");
	else
	  fprintf(stderr, "Control: %g sec\n", time_difference);
      }
      first_control = 0;
      last_time.tv_sec  = this_time.tv_sec;
      last_time.tv_usec = this_time.tv_usec;

    }
  }
}


/************************************************************************\
 ************************************************************************
\************************************************************************/

void
connect_to_base_server()
{
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;
  

  if (!global_use_tcx || !automatic_base_connect)
    return;
  
  if (baseConnected)
    return;
  
  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
  if (time_difference < 3.0)
    return;
  
  
  G_display_switch(BASE_CONNECT_BUTTON, 1);
  if (baseConnect(0)){                   /* hook up to running server*/  
    G_display_switch(BASE_CONNECT_BUTTON, 2);
    baseConnected = 1;

    robot_stalled = 1;
    if (global_modus_recording == 2){
      global_modus_recording = 1;
      G_display_switch(RECORD_BUTTON, global_modus_recording);
    }

#ifdef junk    
    setRotateVelocity(0x40);          /* units = radians*512/PI/sec  */
    findRotIndex();
#endif
    loadPosition(0x80008000);         /* units = encoder counts/256  */
    statusReportPeriod(100*256/1000); /* Units = seconds*256         *
				       * communication for you */
    setTranslateAcceleration(1000);
    setRotateAcceleration(150);
    assumeWatchdog(); 
    watchdogTimer(0x800);
    baseSendServerFixed(BASE_subscribe, 2);


    if (current_pos){
      free(current_pos);
      current_pos = NULL;
    }

    first_irs = 1;
    first_sonars = 1;
    first_tactiles = 1;
    first_lasers = 1;
    first_control = 1;


  }
  else{
    if (automatic_base_connect)
      G_display_switch(BASE_CONNECT_BUTTON, 3);
    else
      G_display_switch(BASE_CONNECT_BUTTON, 0);
    baseConnected = 0;
  }

  last_time.tv_sec = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;
}

/************************************************************************\
 ************************************************************************
\************************************************************************/


void
connect_to_camera_server()
{
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;

  if (!global_use_tcx || !automatic_camera_connect)
    return;

  if (cameraConnected)
    return;

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
  if (time_difference < 3.0)
    return;

  G_display_switch(CAMERA_CONNECT_BUTTON, 1);
  cameraConnect(0);

  if (cameraConnected){
    G_display_switch(CAMERA_CONNECT_BUTTON, 2);
    /*cameraRequestImage(IMAGE_SIZE_X, IMAGE_SIZE_Y);*/
    /*cameraRequestPending = 1;*/
    cameraSubscribeImage(0, 90, ORIG_IMAGE_FIRST_X, ORIG_IMAGE_FIRST_Y,
			 ORIG_IMAGE_SIZE_X, ORIG_IMAGE_SIZE_Y,
			 IMAGE_SIZE_X, IMAGE_SIZE_Y);
  }
  else{
    if (automatic_camera_connect)
      G_display_switch(CAMERA_CONNECT_BUTTON, 3);
    else
      G_display_switch(CAMERA_CONNECT_BUTTON, 0);
  }

  last_time.tv_sec = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;
}

/************************************************************************\
 ************************************************************************
\************************************************************************/


void
connect_to_colli_server()
{
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;
  BASE_register_auto_update_type subscribe;

  if (!global_use_tcx || !automatic_colli_connect)
    return;

  if (colliConnected)
    return;

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
  if (time_difference < 3.0)
    return;

  G_display_switch(COLLI_CONNECT_BUTTON, 1);
  BASE = tcxConnectOptional(TCX_BASE_MODULE_NAME);
  if (BASE)
    colliConnected = 1;

  if (colliConnected){
    G_display_switch(COLLI_CONNECT_BUTTON, 2);
    subscribe.subscribe_status_report      = 0;
    subscribe.subscribe_sonar_report       = 0;
    subscribe.subscribe_laser_report       = 1;
    subscribe.subscribe_ir_report          = 0;
    subscribe.subscribe_colli_report       = 0;
    tcxSendMsg(BASE, "BASE_register_auto_update", &subscribe);
  }
  else{
    if (automatic_colli_connect)
      G_display_switch(COLLI_CONNECT_BUTTON, 3);
    else
      G_display_switch(COLLI_CONNECT_BUTTON, 0);
  }

  last_time.tv_sec = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;
}

/************************************************************************\
 ************************************************************************
\************************************************************************/


void
connect_to_buttons_server()
{
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;

  if (!global_use_tcx || !automatic_buttons_connect)
    return;


  if (buttonConnected)
    return;

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
  if (time_difference < 3.0)
    return;

  G_display_switch(BUTTONS_CONNECT_BUTTON, 1);
  buttonConnect(0);

  if (buttonConnected){
    G_display_switch(BUTTONS_CONNECT_BUTTON, 2);
    buttonSubscribeStatus(1);
    buttonRequestStatus();
    first_buttons = 1;
  }
  else{
    if (automatic_buttons_connect)
      G_display_switch(BUTTONS_CONNECT_BUTTON, 3);
    else
      G_display_switch(BUTTONS_CONNECT_BUTTON, 0);
  }

  last_time.tv_sec = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;
}

/************************************************************************\
 ************************************************************************
\************************************************************************/

void
connect_to_pantilt_server()
{
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;

  if (!global_use_tcx || !automatic_pantilt_connect)
    return;


  if (ptConnected)
    return;

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
  if (time_difference < 3.0)
    return;

  G_display_switch(PANTILT_CONNECT_BUTTON, 1);
  ptConnect(0);

  if (ptConnected){
    G_display_switch(PANTILT_CONNECT_BUTTON, 2);
  }
  else{
    if (automatic_pantilt_connect)
      G_display_switch(PANTILT_CONNECT_BUTTON, 3);
    else
      G_display_switch(PANTILT_CONNECT_BUTTON, 0);
  }

  last_time.tv_sec = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;
}


/************************************************************************\
 ************************************************************************
\************************************************************************/

void
register_laser()		/* this is a special routine, since
				 * laser is distributed by colli (right
				 * now) */
{
  int numberOfMessages;
  int numberOfHandlers;
  int i;

  TCX_REG_MSG_TYPE messages[] = { BASE_messages, LASER_messages }; 
  
  numberOfMessages = sizeof(messages)/sizeof(TCX_REG_MSG_TYPE); 
  numberOfHandlers = 
    sizeof(LASER_reply_handler_array)/sizeof(TCX_REG_HND_TYPE); 

  registerInterface("", numberOfMessages, messages,
		    numberOfHandlers, LASER_reply_handler_array);
}



/************************************************************************
 *
 *   NAME:         stdin_inputHnd()
 *                 
 *   FUNCTION:     Callback routine for characters from the keyboard
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 ************************************************************************/


void stdin_inputHnd(int fd, long chars_available)
{
  fprintf(stderr, ".\n");
}

/************************************************************************\
 ************************************************************************
\************************************************************************/

void
init_bee(int argc, char *argv[])
{
  RaiModule* demoModule;
  struct bParamList * params = NULL;

  if (!global_use_tcx)
    return;

  /* add some parameter files */
  params = bParametersAddFile(params, "etc/beeSoft.ini");

  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* Fill the global parameter structure */
  bParametersFillParams(params);


  registerBaseClient();
  cameraRegister();
  register_laser();
  buttonRegister();
  ptRegister();

  initClient("LEARN", commShutdown); /* close function called if *
				      * the base server dies */




  RaiInit();                          /* init (but not start) scheduler*/
  initClientModules();                /* set up Rai modules to do      */
  connect_to_camera_server();
  connect_to_base_server();
  connect_to_colli_server();
  connect_to_buttons_server();
  connect_to_pantilt_server();


  registerSonarCallback(sonarCallback);
  registerIrCallback(irCallback);
  registerTactileCallback(tactileCallback);
  registerCameraImageCallback(cameraCallback);
  registerButtonStatusCallback(buttonCallback);
  ptStatusCB(pantiltCallback);

  registerBaseCallback(baseCallback);
  registerStatusCallback(statusCallback);
  registerCommandConfirmationCallback(baseCommandCallback);
  registerWatchdogCallback(watchdogCallback);

  /* ask that your polling function be run every ?? msec */
  demoModule=makeModule("LEARN", NULL);
  addPolling(demoModule, main_loop, 1);




  RaiStart(); /* This will not return */
}
