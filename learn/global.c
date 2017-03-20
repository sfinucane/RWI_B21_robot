
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/learn/global.c,v $
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
 * $Log: global.c,v $
 * Revision 1.1  2002/09/14 16:36:05  rstone
 * *** empty log message ***
 *
 * Revision 1.1  1999/07/11 18:48:32  thrun
 * slight reorganization
 *
 * Revision 1.32  1998/08/22 03:08:37  thrun
 * .
 *
 * Revision 1.31  1998/07/04 15:21:38  thrun
 * variable logging.
 *
 * Revision 1.30  1998/07/03 23:59:59  thrun
 * .
 *
 * Revision 1.29  1998/06/20 21:05:32  thrun
 * Reorganization (created: fa.{c,h}, xtypes.h)
 *
 * Revision 1.28  1998/05/05 04:00:34  thrun
 * Direct control mode (and button). Other stuff I don't fully remember.
 * Intermediate version.
 *
 * Revision 1.27  1998/04/28 23:30:46  thrun
 * autosave of weights, we can now switch off learning in selected
 * networks and variables, which is important for incremental
 * programming/learning
 *
 * Revision 1.26  1998/04/26 15:03:08  thrun
 * subroutines appear to work fine. also changed the initialiation.
 *
 * Revision 1.25  1998/04/20 03:12:11  thrun
 * Intermediate version with subroutines.
 *
 * Revision 1.24  1998/04/18 20:42:25  thrun
 * 1. neuro net can now be called with the same variable as input
 * and output
 * 2. direct control modus for the robot. controls are
 * now generated exclusively through the display (left upper corner)
 *
 * Revision 1.23  1998/04/14 02:35:56  thrun
 * .
 *
 * Revision 1.22  1997/10/23 02:28:23  thrun
 * .
 *
 * Revision 1.21  1997/10/05 18:11:18  thrun
 * new data library "libdat.a"
 *
 * Revision 1.20  1997/08/05 04:15:22  thrun
 * (1) avg_value are saved for each stoch variable, (2) enhanced replay
 * functions.
 *
 * Revision 1.19  1997/07/29 22:44:48  thrun
 * .
 *
 * Revision 1.18  1997/07/16 17:09:43  thrun
 * Neat, working version. Simpliefied the application, too.
 *
 * Revision 1.17  1997/07/14 22:17:13  thrun
 * Fixed the bug (I believe). Now comes testing.
 *
 * Revision 1.16  1997/07/06 22:51:49  thrun
 * Autosaving of the parameters (every 2 minutes)
 *
 * Revision 1.15  1997/07/06 20:48:47  thrun
 * .
 *
 * Revision 1.14  1997/07/06 18:42:04  thrun
 * Radial basis networks implemented.
 *
 * Revision 1.13  1997/07/04 19:51:12  thrun
 * intermediate eversion
 *
 * Revision 1.12  1997/07/04 18:09:33  thrun
 * intermediate version - do not use
 *
 * Revision 1.11  1997/07/04 00:28:55  thrun
 * Intermediate verson of the learning stuff. What works is:
 * - training of deterministic variables (incl. chaining)
 * - training of multiple determinstic with a stochastic variables (incl chaining)
 * What doesn't yet work includes
 * - temporal chaining of stochastic variables, using Bayes rule
 * - convolving: gradients
 * - stochatic input
 *
 * Revision 1.10  1997/06/30 05:53:58  thrun
 * intermediate version - new neural network (which seems to work!).
 *
 * Revision 1.9  1997/06/29 04:04:55  thrun
 * intrmediate version - not really good for anything
 *
 * Revision 1.8  1997/06/28 13:41:42  thrun
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

#include "dat.h"
#include "global.h"
#include "mem.h"
#include "xtypes.h"
#include "vars.h"
#include "fa.h"

/************************************************************************\
 ************************************************************************
\************************************************************************/

float *sonar_angles = NULL;
float *sonar_values = NULL;		/* purely for display */
float *ir_values = NULL;
float *ir_angles = NULL;
float *tactile_values = NULL;
float *tactile_angles = NULL;
float *laser_values = NULL;
float *laser_angles = NULL;
float *d_image = NULL;		/* purely for display */
float *imageR = NULL;
float *imageG = NULL;
float *imageB = NULL;
robot_position_type robot_position = {0.0, 0.0, 0.0};
robot_position_type previous_robot_position = {0.0, 0.0, 0.0};




/************************************************************************\
 ************************************************************************
\************************************************************************/


struct timeval noWaitTime = {0, 0};


/************************************************************************\
 ************************************************************************
\************************************************************************/

int global_modus_recording = 0;
int global_modus_suspend_recording_when_not_moving = 1;
int global_modus_execution = 0;
int global_modus_logging = 0;
int global_modus_live = 0;
int global_modus_movie = 0;
int global_modus_real_time_display = 0;
struct timeval last_autosave_time          = {0, 0};
struct timeval global_movie_start_time     = {0, 0};
struct timeval global_movie_start_time_tag = {0, 0};
int global_modus_marking = 0;
int global_graphics_initialized = 0;
int global_modus_training = 0;
int global_modus_control = 0;


int image_pending = 0;

int global_use_tcx = 1;
int global_use_X = 1;
int global_prob_c = 0;
int global_small_X = 0;

int automatic_base_connect = DEFAULT_AUTOMATIC_BASE_CONNECT;
int automatic_camera_connect = DEFAULT_AUTOMATIC_CAMERA_CONNECT;
int automatic_colli_connect = DEFAULT_AUTOMATIC_COLLI_CONNECT;
int automatic_buttons_connect = DEFAULT_AUTOMATIC_BUTTONS_CONNECT;
int automatic_pantilt_connect = DEFAULT_AUTOMATIC_PANTILT_CONNECT;


FILE *log_iop = NULL;

/************************************************************************\
 ************************************************************************
\************************************************************************/



/************************************************************************
 *
 *   NAME:         exit_proc()
 *                 
 *   FUNCTION:     Handler for ^C interrupts
 *                 
 *   PARAMETERS:   none
 *                 
 *                 
 ************************************************************************/


void
exit_proc(int garbage)
{
  static struct timeval last_time = {0, 0};
  struct timeval current_time;
  long   difference;



  gettimeofday(&current_time, NULL);
  difference = (long) (1000000 * (current_time.tv_sec - last_time.tv_sec) +
		       (current_time.tv_usec - last_time.tv_usec));

  if (last_time.tv_sec != 0)
    fprintf(stderr, "%g secs.\n", (float) difference / 1000000.0);




  /*
   * check if we just hit ^C before - then exit
   */

  if (last_time.tv_sec != 0 && difference < 500000){
    fprintf(stderr, "Done.\n");
    exit(0);
  }
  
  X_save_parameters(NET_BACKUP_FILENAME, 
		    NET_BEST_BACKUP_FILENAME, 
		    0,		/* 0 = don't consider timer */
		    0);		/* 0 = don't display buttons */


  fprintf(stderr, "Hit ^C twice to terminate the program.\n");

  gettimeofday(&current_time, NULL);
  last_time.tv_sec  = current_time.tv_sec;
  last_time.tv_usec = current_time.tv_usec;
}




/************************************************************************
 *
 *   NAME:         main_loop()
 *                 
 *   FUNCTION:     main action loop
 *                 
 *   PARAMETERS:   command lines only
 *                 
 *                 
 ************************************************************************/


void
main_loop()
{
  struct timeval this_time;
  float  time_difference; 
  char   filename[80];
  static int autosave_counter = 0;
  /*
   * Connections
   */
  
  if (global_use_tcx && !cameraConnected)
    connect_to_camera_server();
  if (global_use_tcx && !baseConnected)
    connect_to_base_server();
  if (global_use_tcx && !colliConnected)
    connect_to_colli_server();
  if (global_use_tcx && !buttonConnected)
    connect_to_buttons_server();
  if (global_use_tcx && !ptConnected)
    connect_to_pantilt_server();


  /*
   * Training
   */

  if (global_modus_training)
    X_main_training_loop();

  /*
   * Button clicks
   */

  mouse_test_loop();

  /*
   * Request Image
   */
  /*  if (cameraConnected){
    if (cameraRequestPending == 2)
      cameraRequestPending = 0;
    else if (cameraRequestPending == 0){
      cameraRequestImage(IMAGE_SIZE_X, IMAGE_SIZE_Y);
      cameraRequestPending = 1;
    }
  }
  */

  /*
   * Movie routines
   */

  if (global_modus_movie){
    if (data == NULL ||
	(global_modus_movie == 2 && 
	 mem_query_last_pattern() &&  
	 mem_query_last_patternset()) ||
	(global_modus_movie == 1 && 
	 mem_query_last_pattern())){
      global_modus_movie = 0;
      G_display_switch(MOVIE_BUTTON, global_modus_movie);
      G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
    }
    else if (global_modus_movie == 2 && 
	     mem_query_last_pattern()){
      mem_display_next_patternset();
      /*X_process_pattern(current_pattern);*/
      gettimeofday(&global_movie_start_time, NULL);
      if (current_pattern && current_pattern->time){
	global_movie_start_time_tag.tv_sec  = current_pattern->time->tv_sec;
	global_movie_start_time_tag.tv_usec = current_pattern->time->tv_usec;
      }
      else{
	global_movie_start_time_tag.tv_sec  = 0;
	global_movie_start_time_tag.tv_usec = 0;
      }
    }
    else{
      if (global_modus_real_time_display && current_pattern &&
	  current_pattern->next && current_pattern->next->time){
	/*
	 * check if sufficient time elapsed to show next item
	 */
	gettimeofday(&this_time, NULL);
	time_difference = 
	  ((float) (this_time.tv_sec - global_movie_start_time.tv_sec
		    - (current_pattern->next->time->tv_sec -
		       global_movie_start_time_tag.tv_sec)))
	  + (((float) (this_time.tv_usec - global_movie_start_time.tv_usec
		       - (current_pattern->next->time->tv_usec -
			  global_movie_start_time_tag.tv_usec)))
	     /  1000000.0);
	if (time_difference >= 0.0){
	  mem_display_next_pattern();
	  /*X_process_pattern(current_pattern);*/
	  /*print_neuro_net(0);*/
	}
	else
	  usleep(50000);
      }
      else{
	mem_display_next_pattern(); 
	/*X_process_pattern(current_pattern);*/
	/*print_neuro_net(0);*/
      }
    }
  }
  /*
   *
   */ 

  if (AUTOSAVE_PERIOD_DURING_RECORDING >= 0.0 &&
      global_modus_recording && data &&
      data->total_number_patterns > 0){

    gettimeofday(&this_time, NULL);
    time_difference = 
      ((float) (this_time.tv_sec - last_autosave_time.tv_sec))
      + (((float) (this_time.tv_usec - last_autosave_time.tv_usec))
	 /  1000000.0);
    
    if (time_difference > AUTOSAVE_PERIOD_DURING_RECORDING){
      last_autosave_time.tv_sec = this_time.tv_sec;
      last_autosave_time.tv_usec = this_time.tv_usec;

      sprintf(filename, DATA_FILENAME3, autosave_counter++);
      mem_save_patterns(filename, SAVE_BUTTON);
      fprintf(stderr, "-1-");
      if (data && DELETE_PATTERNS_AFTER_AUTOSAVING){
	mem_delete_data();
	current_patternset = mem_create_patternset();
	mem_fill_pattern_set_slot(current_patternset, "recorded", 0);
	first_irs = 1;
	first_sonars = 1;
	first_tactiles = 1;
	first_lasers = 1;
      }
      fprintf(stderr, "-2-");
    }
  }
}



/************************************************************************
 *
 *   NAME:         init_all()
 *                 
 *   FUNCTION:     
 *                 
 *   PARAMETERS:   command lines only
 *                 
 *                 
 ************************************************************************/

void
init_all(int argc, char *argv[])
{
  int i, unknown_arg = 0;

  for (i = 1; i < argc; i++){
    if (!strcmp("-notcx", argv[i]) || !strcmp("-nt", argv[i]))
      global_use_tcx = 0;
    else if (!strcmp("-nodisplay", argv[i]) || !strcmp("-nd", argv[i]))
      global_use_X = 0;
    else if (!strcmp("-smalldisplay", argv[i]) || !strcmp("-sd", argv[i]))
      global_small_X = 1;
    else if (!strcmp("-prob", argv[i]) || !strcmp("-p", argv[i]))
      global_prob_c = 1;
    else if (!strcmp("-noautoconnect", argv[i]) || !strcmp("-na", argv[i])){
      automatic_base_connect = 0;
      automatic_camera_connect = 0;
      automatic_colli_connect = 0;
      automatic_buttons_connect = 0;
      automatic_pantilt_connect = 0;
    }
    else if (!strcmp("source", argv[i]))
      fprintf(stderr, "Found a stupid GDB, trying to screw me up.\n");
    else
      unknown_arg = 1;
  }
  if (unknown_arg){
    fprintf(stderr, "Usage: %s [-notcx|-nt] [-noautoconnect|-na] [-nodisplay|-nd] [-smalldisplay|-sd] [-prob|-p]\n",
	    argv[0]);
    exit (0);
  }
  


  init_graphics(global_use_X);
  X_initialize_all();
  init_graphics_2(global_use_X);

  signal(SIGINT,   (void *) exit_proc);
  signal(SIGTERM,  (void *) exit_proc);


  /*
   * SPECIAL CASE: No display, batch mode
   */


  if (!global_use_X){
    if (AUTOSAVE_PERIOD_DURING_RECORDING < 0.0){
      fprintf(stderr, "Recording does not make sense without auto-saving.\n");
      fprintf(stderr, "Must exit.\n");
      exit(-1);
    }
    /* begin recording */
    global_modus_recording = 1;
    global_modus_suspend_recording_when_not_moving = 0;
    G_display_switch(RECORD_BUTTON, global_modus_recording);
    current_patternset = mem_create_patternset();
    mem_fill_pattern_set_slot(current_patternset, "recorded", 0);
    first_irs = 1;
    first_sonars = 1;
    first_tactiles = 1;
    first_lasers = 1;
    gettimeofday(&last_autosave_time, NULL);
    fprintf(stderr, "### %d\n",
	    global_modus_recording);
  }

  init_bee(argc, argv);			/* returns only if no-tcx */

}
