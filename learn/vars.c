

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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/learn/vars.c,v $
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
 * $Log: vars.c,v $
 * Revision 1.1  2002/09/14 16:36:05  rstone
 * *** empty log message ***
 *
 * Revision 1.47  1999/07/11 18:48:34  thrun
 * slight reorganization
 *
 * Revision 1.46  1998/07/04 15:21:39  thrun
 * variable logging.
 *
 * Revision 1.45  1998/06/27 19:13:54  thrun
 * some tuning + training.
 *
 * Revision 1.44  1998/06/22 04:48:45  thrun
 * nice intermediate version: can send robot to a point now, but controller
 * doesn't dampen the robot's trajectory yet.
 *
 * Revision 1.43  1998/06/20 21:05:33  thrun
 * Reorganization (created: fa.{c,h}, xtypes.h)
 *
 * Revision 1.42  1998/05/28 22:45:23  thrun
 * Intermediate version, nothing much changed.
 *
 * Revision 1.41  1998/05/27 21:27:51  thrun
 * intermediate version, don't use
 *
 * Revision 1.40  1998/05/27 04:47:05  thrun
 * Lots of stuff (don't exactly remember). Probably in the applications,
 * a new mode where the robot learns control. Improved Gaussian convolution.
 *
 * Revision 1.39  1998/05/05 04:00:36  thrun
 * Direct control mode (and button). Other stuff I don't fully remember.
 * Intermediate version.
 *
 * Revision 1.38  1998/04/28 23:30:47  thrun
 * autosave of weights, we can now switch off learning in selected
 * networks and variables, which is important for incremental
 * programming/learning
 *
 * Revision 1.37  1998/04/26 15:03:09  thrun
 * subroutines appear to work fine. also changed the initialiation.
 *
 * Revision 1.36  1998/04/24 04:49:05  thrun
 * intermediate and buggy version - subroutines don't have their gradient
 * computation quite right yet.
 *
 * Revision 1.35  1998/04/20 03:28:01  thrun
 * Intermediate version with subroutines.
 *
 * Revision 1.33  1998/04/18 22:55:42  thrun
 * nice intermediate version, no apparent bugs
 *
 * Revision 1.32  1998/04/18 20:42:26  thrun
 * 1. neuro net can now be called with the same variable as input
 * and output
 * 2. direct control modus for the robot. controls are
 * now generated exclusively through the display (left upper corner)
 *
 * Revision 1.31  1998/04/16 02:42:50  thrun
 * copy function for random variables - use with caution!
 *
 * Revision 1.30  1998/04/15 01:58:11  thrun
 * intermediate version
 *
 * Revision 1.29  1998/04/14 02:35:56  thrun
 * .
 *
 * Revision 1.28  1998/04/13 01:45:19  thrun
 * Thermomter encoding for sigmodial neural networks. Delay for
 * pausing (when recording data).
 *
 * Revision 1.27  1997/10/05 18:11:20  thrun
 * new data library "libdat.a"
 *
 * Revision 1.26  1997/08/06 13:56:10  thrun
 * intermediate version (fixed a bug with the name field in variables/networks)
 *
 * Revision 1.25  1997/08/05 22:19:08  thrun
 * intermediate version - do not use (networks are now not linked
 * to variables any longer)
 *
 * Revision 1.24  1997/08/05 04:15:24  thrun
 * (1) avg_value are saved for each stoch variable, (2) enhanced replay
 * functions.
 *
 * Revision 1.23  1997/07/30 21:02:03  thrun
 * baseServer now sends confirmations of all motion commands. Is used
 * for logging in the module "learn"
 *
 * Revision 1.22  1997/07/30 03:46:54  thrun
 * New events: pantilt and buttons
 *
 * Revision 1.21  1997/07/26 14:19:05  thrun
 * changed display options
 *
 * Revision 1.20  1997/07/16 17:09:44  thrun
 * Neat, working version. Simpliefied the application, too.
 *
 * Revision 1.19  1997/07/15 22:15:43  thrun
 * Fixed a slight correspondence problem between targets and regular
 * variables. Need to do some more fixing - so don't use.
 *
 * Revision 1.18  1997/07/15 17:34:06  thrun
 * *** empty log message ***
 *
 * Revision 1.17  1997/07/14 22:17:14  thrun
 * Fixed the bug (I believe). Now comes testing.
 *
 * Revision 1.16  1997/07/14 14:13:18  thrun
 * First attempt to implement multi-phasing. (broken code, don't use).
 *
 * Revision 1.15  1997/07/07 13:53:11  thrun
 * Small change in the application
 *
 * Revision 1.14  1997/07/07 05:02:52  thrun
 * this version now contains all the necessary basic routines, and is
 * used for some initial testing.
 * Missing is: Combine n variables into one. Stochastic target variables.
 * Code can be made more efficient. Radial Basis networks with pre-specified
 * resolution> Reasonable display ov stochastic variables with 3 or more
 * dimensions, and other stuff.
 *
 * Revision 1.13  1997/07/07 04:45:25  thrun
 * Now with training and testing set support
 *
 * Revision 1.12  1997/07/07 00:46:55  thrun
 * minor optimizations
 *
 * Revision 1.11  1997/07/06 22:59:45  thrun
 * nothing
 *
 * Revision 1.10  1997/07/06 22:51:49  thrun
 * Autosaving of the parameters (every 2 minutes)
 *
 * Revision 1.9  1997/07/06 22:23:15  thrun
 * .
 *
 * Revision 1.8  1997/07/06 20:48:48  thrun
 * .
 *
 * Revision 1.7  1997/07/06 18:42:05  thrun
 * Radial basis networks implemented.
 *
 * Revision 1.6  1997/07/05 17:20:28  thrun
 * Temporal integration of stochastic variables works fine now.
 *
 * Revision 1.5  1997/07/04 20:57:29  thrun
 * Running version of: determinstic repr -> network -> stochastich repr ->
 *         network -> determinstic repr (chaining), tested with XOR.
 *
 * Revision 1.4  1997/07/04 19:26:54  thrun
 * yet another intermediate version
 *
 * Revision 1.3  1997/07/04 18:09:33  thrun
 * intermediate version - do not use
 *
 * Revision 1.2  1997/07/04 00:28:56  thrun
 * Intermediate verson of the learning stuff. What works is:
 * - training of deterministic variables (incl. chaining)
 * - training of multiple determinstic with a stochastic variables (incl chaining)
 * What doesn't yet work includes
 * - temporal chaining of stochastic variables, using Bayes rule
 * - convolving: gradients
 * - stochatic input
 *
 * Revision 1.1  1997/06/30 13:51:41  thrun
 * intermediate version - new neural network (which seems to work!).
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

/*#define pt(number) {static int a = 0; int i = 0; if (a == 0){fprintf(stderr, "\n### %d:\n", number); for (i = 0; i < X_number_of_variables; i++) fprintf(stderr, "\t%d:\"%s\"\n", i, X_variables[i].name); fprintf(stderr, "\n"); a = 0;}}*/




#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>

#include "bUtils.h"
#include "EZX11.h"
#include "o-graphics.h"
#include "dat.h"
#include "math.h"
#include "global.h"
#include "mem.h"
#include "xtypes.h"
#include "vars.h"
#include "fa.h"

/************************************************************************\
 ************************************************************************
\************************************************************************/



#define RANDOM_MAX ((float)(2147483647))   /* (2**31 - 1) */
#define RAND_PLUS_MINUS_ONE() ((2.0 * ((float)(random()) / ((float) RANDOM_MAX))) - 1.0)
#define RAND_ZERO_TO_ONE() ((float)(random()) / ((float) RANDOM_MAX))



/************************************************************************\
 ************************************************************************
\************************************************************************/

struct pattern_type *X_pattern = NULL;

int X_display = 1;



/************************************************************************\
 ************************************************************************
\************************************************************************/


variable_type X_variables[MAX_NUMBER_VARIABLES];

int X_number_of_variables = 0;


/************************************************************************\
 ************************************************************************
\************************************************************************/



int X_number_of_processes = 0;
process_type X_processes[MAX_NUMBER_PROCESSES];


/************************************************************************\
 ************************************************************************
\************************************************************************/



float X_prob = 0.0;



/************************************************************************\
 ************************************************************************
\************************************************************************/


/* **********************************************************************
 *
 * X_initialize_all(): initializes the entire learning code. 
 *                     Must be called internally
 *
 * **********************************************************************/

void
X_initialize_all()
{


  /*
   * initialize the internal pattern memory
   */

  X_pattern = mem_create_pattern();

  if (global_prob_c){

    /*
     * initialization of the user variables and stuff
     */
    
    X_init();  
    
    fprintf(stderr,
	    "%d variables, %d networks, and %d processes registered.\n",
	    X_number_of_variables,
	    X_number_of_networks,
	    X_number_of_processes);
    
    
    X_initialize_episode();
  }

}






/* **********************************************************************
 *
 * X_initialize_episode(): Initializes a particular episode. Called
 *                         during training/execution.
 *
 * **********************************************************************/

void
X_initialize_episode()
{
  int i;

  if (global_prob_c)
    X_initialize_variables();

  /*X_network_clear_pending_updates();*/

  /*
   * reset time stamps for all processes
   */
  for (i = 0; i < X_number_of_processes; i++){
    X_processes[i].time_stamp.tv_sec  = 0;
    X_processes[i].time_stamp.tv_usec = 0;
  }
  X_log_variables();
}



/************************************************************************\
 ************************************************************************
\************************************************************************/





/* **********************************************************************
 *
 * X_process_pattern(): Internal routine, for processing a new pattern
 *                      (from file or life). Effect: The values
 *                      are memorized, and, if applicable, the various
 *                      user processes are executed
 *
 * **********************************************************************/

void
X_process_pattern(struct pattern_type *pattern)
{
  int i;
  float X_time_elapsed_since_last_process_call;
  int verbose = 0;


  if (pattern->name)
    mem_fill_pattern_slot(X_pattern, pattern->name, NULL, NULL, NULL, NULL, 
			  NULL, NULL, NULL, NULL, NULL, NULL,
			  NULL);
  if (pattern->time){
    mem_fill_pattern_slot(X_pattern, NULL, pattern->time, NULL, NULL, NULL, 
			  NULL, NULL, NULL, NULL, NULL, NULL,
			  NULL);
    for (i = 0; i < X_number_of_processes; i++)
      if (X_processes[i].requires_time)
	X_processes[i].requires_time = 2;
  }

  if (pattern->image){
    mem_fill_pattern_slot(X_pattern, NULL, NULL, pattern->image, 
			  NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
			  NULL);
    for (i = 0; i < X_number_of_processes; i++)
      if (X_processes[i].requires_image)
	X_processes[i].requires_image = 2;
  }

  if (pattern->sonars){
    mem_fill_pattern_slot(X_pattern, NULL, NULL, NULL, pattern->sonars,
			  NULL, NULL, NULL, NULL, NULL, NULL, NULL,
			  NULL);
    for (i = 0; i < X_number_of_processes; i++)
      if (X_processes[i].requires_sonars)
	X_processes[i].requires_sonars = 2;
  }

  if (pattern->irs){
    mem_fill_pattern_slot(X_pattern, NULL, NULL, NULL, NULL, 
			  pattern->irs, NULL, NULL, NULL, NULL, NULL, NULL,
			  NULL);
    for (i = 0; i < X_number_of_processes; i++)
      if (X_processes[i].requires_irs)
	X_processes[i].requires_irs = 2;
  }

  if (pattern->tactiles){
    mem_fill_pattern_slot(X_pattern, NULL, NULL, NULL, NULL, NULL, 
			  pattern->tactiles, NULL, NULL, NULL, NULL, NULL,
			  NULL);
    for (i = 0; i < X_number_of_processes; i++)
      if (X_processes[i].requires_tactiles)
	X_processes[i].requires_tactiles = 2;
  }

  if (pattern->lasers){
    mem_fill_pattern_slot(X_pattern, NULL, NULL, NULL, NULL, NULL, NULL,
			  pattern->lasers, NULL, NULL, NULL, NULL,
			  NULL);
    for (i = 0; i < X_number_of_processes; i++)
      if (X_processes[i].requires_lasers)
	X_processes[i].requires_lasers = 2;
  }

  if (pattern->position){
    mem_fill_pattern_slot(X_pattern, NULL, NULL, NULL, NULL, NULL, NULL, 
			  NULL, NULL, NULL, pattern->position, NULL,
			  NULL);
    for (i = 0; i < X_number_of_processes; i++)
      if (X_processes[i].requires_position)
	X_processes[i].requires_position = 2;
  }

  if (pattern->buttons){
    mem_fill_pattern_slot(X_pattern, NULL, NULL, NULL, NULL, NULL, NULL,
			  NULL, pattern->buttons, NULL, NULL, NULL,
			  NULL);
    for (i = 0; i < X_number_of_processes; i++)
      if (X_processes[i].requires_buttons)
	X_processes[i].requires_buttons = 2;
  }

  if (pattern->pantilt){
    mem_fill_pattern_slot(X_pattern, NULL, NULL, NULL, NULL, NULL, NULL,
			  NULL, NULL, pattern->pantilt, NULL, NULL,
			  NULL);
    for (i = 0; i < X_number_of_processes; i++)
      if (X_processes[i].requires_pantilt)
	X_processes[i].requires_pantilt = 2;
  }

  if (pattern->markers){
    mem_fill_pattern_slot(X_pattern, NULL, NULL, NULL, NULL, NULL, NULL,
			  NULL, NULL, NULL, NULL, pattern->markers,
			  NULL);
    for (i = 0; i < X_number_of_processes; i++)
      if (X_processes[i].requires_markers)
	X_processes[i].requires_markers = 2;
  }

  if (pattern->control){
      mem_fill_pattern_slot(X_pattern, NULL, NULL, NULL, NULL, NULL, NULL,
			    NULL, NULL, NULL, NULL, NULL, pattern->control);
    for (i = 0; i < X_number_of_processes; i++)
      if (X_processes[i].requires_control)
	X_processes[i].requires_control = 2;
  }

  /*
   * check, if it's time to call one of the X_processes
   */

  for (i = 0; i < X_number_of_processes; i++)
    if (X_processes[i].requires_time != 1 &&
	X_processes[i].requires_position != 1 &&
	X_processes[i].requires_image != 1 &&
	X_processes[i].requires_sonars != 1 &&
	X_processes[i].requires_lasers != 1 &&
	X_processes[i].requires_irs != 1 &&
	X_processes[i].requires_tactiles != 1 &&
	X_processes[i].requires_buttons != 1 &&
	X_processes[i].requires_pantilt != 1 &&
	X_processes[i].requires_markers != 1 &&
	X_processes[i].requires_control != 1){

      /*
       * Compute how much time elapsed since that very procedure
       * was called 
       */

      if (pattern->time && X_processes[i].time_stamp.tv_sec > 0)
	X_time_elapsed_since_last_process_call = 
	  ((float) (X_pattern->time->tv_sec - 
		    X_processes[i].time_stamp.tv_sec))
	  + (((float) (X_pattern->time->tv_usec - 
		       X_processes[i].time_stamp.tv_usec))
	     /  1000000.0);
      else
	X_time_elapsed_since_last_process_call = -1.0;
      if (pattern->time){
	X_processes[i].time_stamp.tv_sec  = X_pattern->time->tv_sec;
	X_processes[i].time_stamp.tv_usec = X_pattern->time->tv_usec;
      }
      else{
	X_processes[i].time_stamp.tv_sec  = 0;
	X_processes[i].time_stamp.tv_usec = 0;
      }

      /*
       * NOW: Call the process!!
       */

      if (verbose)
	fprintf(stderr, "Executing \"%s\"\n", X_processes[i].name);

      X_processes[i].code(X_time_elapsed_since_last_process_call);

      if (verbose)
	fprintf(stderr, "done.\n");
      
      if (X_processes[i].requires_time == 2)
	X_processes[i].requires_time = 1;
      if (X_processes[i].requires_position == 2)
	X_processes[i].requires_position = 1;
      if (X_processes[i].requires_image == 2)
	X_processes[i].requires_image = 1;
      if (X_processes[i].requires_sonars == 2)
	X_processes[i].requires_sonars = 1;
      if (X_processes[i].requires_lasers == 2)
	X_processes[i].requires_lasers = 1;
      if (X_processes[i].requires_irs == 2)
	X_processes[i].requires_irs = 1;
      if (X_processes[i].requires_tactiles == 2)
	X_processes[i].requires_tactiles = 1;
      if (X_processes[i].requires_buttons == 2)
	X_processes[i].requires_buttons = 1;
      if (X_processes[i].requires_pantilt == 2)
	X_processes[i].requires_pantilt = 1;
      if (X_processes[i].requires_markers == 2)
	X_processes[i].requires_markers = 1;
      if (X_processes[i].requires_control == 2)
	X_processes[i].requires_control = 1;
    }

  X_log_variables();
}



/************************************************************************\
 ************************************************************************
\************************************************************************/





/* **********************************************************************
 *
 * X_register_process(): Registers a user process
 * 
 *
 * **********************************************************************/

void
X_register_process(char       *name,
		   void_code  *code)
{
  if (X_number_of_processes >= MAX_NUMBER_PROCESSES){
    fprintf(stderr, "WARNING: No more processes.\n");
    return;
  }


  X_processes[X_number_of_processes].name = 
    (char *) malloc(sizeof(char) * (strlen(name) + 1));
  strcpy(X_processes[X_number_of_processes].name, name);

  X_processes[X_number_of_processes].code                = code;
  X_processes[X_number_of_processes].time_stamp.tv_sec   = 0;
  X_processes[X_number_of_processes].time_stamp.tv_usec  = 0;
  X_processes[X_number_of_processes].requires_time       = 0;
  X_processes[X_number_of_processes].requires_position   = 0;
  X_processes[X_number_of_processes].requires_image      = 0;
  X_processes[X_number_of_processes].requires_sonars     = 0;
  X_processes[X_number_of_processes].requires_lasers     = 0;
  X_processes[X_number_of_processes].requires_pantilt    = 0;
  X_processes[X_number_of_processes].requires_irs        = 0;
  X_processes[X_number_of_processes].requires_tactiles   = 0;
  X_processes[X_number_of_processes].requires_buttons    = 0;
  X_processes[X_number_of_processes].requires_markers    = 0;
  X_processes[X_number_of_processes].requires_control    = 0;
  X_number_of_processes++;
  
  fprintf(stderr, "\tRegistered:  process \"%s\".\n", name);

}







/* **********************************************************************
 *
 * X_specify_trigger_condition(): After process registration, this routine
 *                                is called to specify under what conditions
 *                                this process shall be run
 * 
 *
 * **********************************************************************/

void
X_specify_trigger_condition(void_code  *code,
			    int  event_type,
			    int  activate)
{
  int i, n;

  /*
   * search, if this process is known
   */

  for (i = 0, n = -1; i < X_number_of_processes; i++)
    if (code == X_processes[i].code)
      n = i;

  /*
   * error message, when not found
   */

  if (n == -1){
    fprintf(stderr, "WARNING: Process %p not known. Check registration.\n",
	    code);
    return;
  }
  switch(event_type){
  case TIME_EVENT:
    X_processes[n].requires_time       = activate;
    break;
  case POSITION_EVENT:
    X_processes[n].requires_position   = activate;
    break;
  case IMAGE_EVENT:
    X_processes[n].requires_image      = activate;
    break;
  case SONARS_EVENT:
    X_processes[n].requires_sonars     = activate;
    break;
  case LASERS_EVENT:
    X_processes[n].requires_lasers     = activate;
    break;
  case IRS_EVENT:
    X_processes[n].requires_irs        = activate;
    break;
  case TACTILES_EVENT:
    X_processes[n].requires_tactiles   = activate;
    break;
  case BUTTONS_EVENT:
    X_processes[n].requires_buttons    = activate;
    break;
  case PANTILT_EVENT:
    X_processes[n].requires_pantilt    = activate;
    break;
  case MARKERS_EVENT:
    X_processes[n].requires_markers    = activate;
    break;
  case CONTROL_EVENT:
    X_processes[n].requires_control    = activate;
    break;
  default:
    fprintf(stderr, "Unknown event type: %d. Check events in vars.h\n",
	    event_type);
    return;
    break;
  }

}


/************************************************************************\
 ************************************************************************
\************************************************************************/




/* **********************************************************************
 *
 * X_register_variable(): Registers a regular (deterministic) variable.
 * 
 *
 * **********************************************************************/

int
X_register_variable(char *name, 
		    int dimension, 
		    int display,
		    int log)
{
  int i;

  /*
   * Sanity check
   */

  if (X_number_of_variables >= MAX_NUMBER_VARIABLES){
    fprintf(stderr, "WARNING: No more variables.\n");
    return -1;
  }


  if (dimension < 1){
    fprintf(stderr, "ERROR: The minimal dimension is 1.\n");
    return -1;
  }


  /*
   * copy into memory
   */
  
  X_variables[X_number_of_variables].type                = DETERMINISTIC;
  X_variables[X_number_of_variables].cyclic              = 0;
  X_variables[X_number_of_variables].dimension           = dimension;
  X_variables[X_number_of_variables].log_flag            = log;
  X_variables[X_number_of_variables].resolution          = 1;
  X_variables[X_number_of_variables].save_flag           = 0;
  X_variables[X_number_of_variables].display             = -1;
  X_variables[X_number_of_variables].avg_display         = -1;
  X_variables[X_number_of_variables].norm_display        = -1;
  X_variables[X_number_of_variables].num_values          = dimension;
  X_variables[X_number_of_variables].num_gradients       = 0;
  X_variables[X_number_of_variables].num_cumul_gradients = 0;
  X_variables[X_number_of_variables].num_avg_gradients   = 0;
  X_variables[X_number_of_variables].cumul_divisor       = 0;
  X_variables[X_number_of_variables].cumul_value         = NULL;
  X_variables[X_number_of_variables].avg_value           = NULL;
  X_variables[X_number_of_variables].norm_value          = NULL;
  X_variables[X_number_of_variables].learning_flag       =
    DEFAULT_VARIABLE_LEARNING_FLAG;
  for (i = 0; i < NUM_SET_TYPES; i++){
    X_variables[X_number_of_variables].error[i]          = -1.0;
    X_variables[X_number_of_variables].prev_error[i]     = -1.0;
  }

  X_variables[X_number_of_variables].name = 
    (char *) malloc(sizeof(char) * (strlen(name) + 1));
  strcpy(X_variables[X_number_of_variables].name, name);

  for (i = 0; i < MAX_NUMBER_NETWORKS; i++){
    X_variables[X_number_of_variables].gradients[i].network_id       = -1;
    X_variables[X_number_of_variables].gradients[i].value            = NULL;
    X_variables[X_number_of_variables].cumul_gradients[i].network_id = -1;
    X_variables[X_number_of_variables].cumul_gradients[i].value      = NULL;
    X_variables[X_number_of_variables].avg_gradients[i].network_id   = -1;
    X_variables[X_number_of_variables].avg_gradients[i].value        = NULL;
  }


  /*
   * allocate and initialize the memory
   */

  X_variables[X_number_of_variables].value            = 
    (float *) malloc(sizeof(float) * dimension);
  
  if (!X_variables[X_number_of_variables].value){
    fprintf(stderr, "ERROR: out of memory when allocating variable %s\n", 
	    name);
    exit(-1);
    }

  for (i = 0; i < dimension; i++)
    X_variables[X_number_of_variables].value[i]       = 0.0;

  
  /*
   * create the display
   */

  if (display)
    X_variables[X_number_of_variables].display =
      X_create_new_variable_display(X_variables[X_number_of_variables].name,
				    X_variables[X_number_of_variables].
				    dimension, 1,
				    X_variables[X_number_of_variables].value);


  /*
   * finally, increment number of variables
   */
  
  fprintf(stderr, "\tRegistered: variable \"%s\".\n", name);
  X_number_of_variables++;
  return (X_number_of_variables-1); /* this is the id of this variable */

}
    


    








/* **********************************************************************
 *
 * X_register_stochastic_variable(): Registers a stochastic variable
 * 
 *
 * **********************************************************************/

int
X_register_stochastic_variable(char *name, 
			       int dimension, 
			       int cyclic,
			       int resolution, 
			       int display,
			       int log)
{
  int i, n;
  char *display_name = NULL;
  int first_dim = 0, second_dim =0;

  /*
   * Sanity check
   */

  if (X_number_of_variables >= MAX_NUMBER_VARIABLES){
    fprintf(stderr, "WARNING: No more variables.\n");
    return -1;
  }

  if (dimension < 1){
    fprintf(stderr, "ERROR: The minimal dimension is 1.\n");
    return -1;
  }

  if (display == 1 && dimension > 4){
    fprintf(stderr, "WARNING: Display is currently not implemented for \n");
    fprintf(stderr, "stochastic float variables with more than four ");
    fprintf(stderr, "dimensions\n");
  }

  /*
   * copy into memory
   */

  X_variables[X_number_of_variables].type                = STOCHASTIC;
  X_variables[X_number_of_variables].cyclic              = cyclic;
  X_variables[X_number_of_variables].dimension           = dimension;
  X_variables[X_number_of_variables].log_flag            = log;
  X_variables[X_number_of_variables].resolution          = resolution;  
  X_variables[X_number_of_variables].save_flag           = 0;
  X_variables[X_number_of_variables].display             = -1;
  X_variables[X_number_of_variables].avg_display         = -1;
  X_variables[X_number_of_variables].norm_display        = -1;
  X_variables[X_number_of_variables].num_gradients       = 0;
  X_variables[X_number_of_variables].num_cumul_gradients = 0;
  X_variables[X_number_of_variables].num_avg_gradients   = 0;
  X_variables[X_number_of_variables].cumul_divisor       = 0;
  X_variables[X_number_of_variables].learning_flag       =
    DEFAULT_VARIABLE_LEARNING_FLAG;
  for (i = 0; i < NUM_SET_TYPES; i++){
    X_variables[X_number_of_variables].error[i]          = -1.0;
    X_variables[X_number_of_variables].prev_error[i]     = -1.0;
  }

  X_variables[X_number_of_variables].name = 
    (char *) malloc(sizeof(char) * (strlen(name) + 1));
  strcpy(X_variables[X_number_of_variables].name, name);

  /*
   * allocate and initialize the memory
   */

  for (i = 0, n = 1; i < dimension; i++)
    n = n * resolution;
 
  X_variables[X_number_of_variables].value             = 
    (float *) malloc(sizeof(float) * n);
  X_variables[X_number_of_variables].cumul_value       = 
    (float *) malloc(sizeof(float) * n);
  X_variables[X_number_of_variables].avg_value         = 
    (float *) malloc(sizeof(float) * n);
  X_variables[X_number_of_variables].norm_value        = 
    (float *) malloc(sizeof(float) * n);
  
  if (!X_variables[X_number_of_variables].value       ||
      !X_variables[X_number_of_variables].cumul_value ||
      !X_variables[X_number_of_variables].avg_value   ||
      !X_variables[X_number_of_variables].norm_value  ){
    fprintf(stderr, "ERROR: out of memory when allocating variable %s\n", 
	    name);
    exit(-1);
  }
  
  for (i = 0; i < n; i++){
    X_variables[X_number_of_variables].value[i]       = 1.0 / ((float) n);
    X_variables[X_number_of_variables].cumul_value[i] = 0.0;
    X_variables[X_number_of_variables].avg_value[i]   = 1.0; /* to avoid
							      * div-by-0 */
    X_variables[X_number_of_variables].norm_value[i]  = 0.0;
  }


  X_variables[X_number_of_variables].num_values = n;

  for (i = 0; i < MAX_NUMBER_NETWORKS; i++){
    X_variables[X_number_of_variables].gradients[i].network_id        = -1;
    X_variables[X_number_of_variables].gradients[i].value             = NULL;
    X_variables[X_number_of_variables].cumul_gradients[i].network_id  = -1;
    X_variables[X_number_of_variables].cumul_gradients[i].value       = NULL;
    X_variables[X_number_of_variables].avg_gradients[i].network_id    = -1;
    X_variables[X_number_of_variables].avg_gradients[i].value         = NULL;
  }

  
  /*
   * create the display
   */

  if (display && dimension >= 1 && dimension <= 4){
    if (dimension == 1){
      first_dim  = X_variables[X_number_of_variables].resolution;
      second_dim = 1;
    }
    else if (dimension == 2){
      first_dim  = X_variables[X_number_of_variables].resolution;
      second_dim = X_variables[X_number_of_variables].resolution;
    }
    else if (dimension == 3){
      first_dim  = X_variables[X_number_of_variables].resolution *
	X_variables[X_number_of_variables].resolution;
      second_dim = X_variables[X_number_of_variables].resolution;

    }
    else if (dimension == 4){
      first_dim  = X_variables[X_number_of_variables].resolution *
	X_variables[X_number_of_variables].resolution;
      second_dim = X_variables[X_number_of_variables].resolution *
	X_variables[X_number_of_variables].resolution;

    }
    else
      fprintf(stderr, "What a bug! I must be tired.\n");

    
    display_name = (char *) malloc(sizeof(char) * MAX_STRING_LENGTH);
    strcpy(display_name, X_variables[X_number_of_variables].name);

    if (display == X_DISPLAY_VALUE || 
	display == X_DISPLAY_ALL){
      X_variables[X_number_of_variables].display =
	X_create_new_variable_display(display_name,
				      first_dim, second_dim,
				      X_variables[X_number_of_variables].
				      value);
      if (display_name != NULL){
	free(display_name);
	display_name = NULL;
      }
    }


    if (display == X_DISPLAY_AVERAGE ||
	display == X_DISPLAY_ALL){
      X_variables[X_number_of_variables].avg_display =
	X_create_new_variable_display(display_name,
				      first_dim, second_dim,
				      X_variables[X_number_of_variables].
				      avg_value);
      if (display_name != NULL){
	free(display_name);
	display_name = NULL;
      }
    }

    if (display == X_DISPLAY_N_VALUE || 
	display == X_DISPLAY_ALL){
      X_variables[X_number_of_variables].norm_display =
	X_create_new_variable_display(display_name,
				      first_dim, second_dim,
				      X_variables[X_number_of_variables].
				      norm_value);
      if (display_name != NULL){
	free(display_name);
	display_name = NULL;
      }
    }
  }


  /*
   * finally, increment number of variables
   */
  
  fprintf(stderr, "\tRegistered: variable \"%s\".\n", name);
  X_number_of_variables++;

  return (X_number_of_variables-1); /* this is the id of this variable */
}


/************************************************************************\
 ************************************************************************
\************************************************************************/




/* **********************************************************************
 *
 * X_initialize_uniformly(): Initializes a stochastic variable uniformly
 *                          in a user-specified range
 *                          set range_min = range_max = NULL if
 *                          global uniform distribution wanted
 *
 * **********************************************************************/

void
X_initialize_uniformly(int variable_id, float *range_min, float *range_max)
{
  int i, j, done;
  float min, max;
  int *counter = NULL;
  int *nearest_neighbor_index = NULL;
  int index, inside;
  int verbose = 0;
  float value;
  float total_value, factor;


  /*
   * sanity checks
   */

  if (variable_id < 0 || variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    variable_id);
    return;
  }

  if (X_variables[variable_id].type != STOCHASTIC){
    fprintf(stderr, "ERROR: Only stochastic variables can be initialized uniformly.\n");
    return;
  }
  

  for (i = 0; i < X_variables[variable_id].dimension; i++){
    if (range_min != NULL)
      if (range_min[i] < 0.0){
	fprintf(stderr, "Invalid lower bound in X_initialize_uniformly: %g\n",
		range_min[i]);
	return;
      }
    if (range_max != NULL)
      if (range_max[i] < 0.0){
	fprintf(stderr, "Invalid upper bound in X_initialize_uniformly: %g\n",
		range_max[i]);
	return;
      }
    
    if (range_min != NULL && range_max != NULL)
      if (range_max[i] < 0.0){
	fprintf(stderr, "Invalid bounds in X_initialize_uniformly: %g %g\n",
		range_min[i], range_max[i]);
	return;
      }
  }    



  /*
   * ******************** Initialize ********************
   */


  /*
   * allocate and initialize memory 
   */

  counter = (int *) malloc(sizeof(int) * 
			   X_variables[variable_id].dimension);
  nearest_neighbor_index = (int *) malloc(sizeof(int) * 
				 X_variables[variable_id].dimension);
  if (!counter || !nearest_neighbor_index){
    fprintf(stderr, "ERROR: out of memory in X_initialize_uniformly.\n");
    exit(-1);
  }
  for (i = 0; i < X_variables[variable_id].dimension; i++)
    counter[i] = 0;
  

  if (verbose)
    fprintf(stderr, "----------------------------------------\n");

  /*
   * clear gradients
   */

  X_clear_all_gradients(variable_id, 0);

  /*
   * check what type range we have here? If nearest_neighbor_index = -1, then
   * the interval is large enough so that we'll find a point by just
   * iterating with the variable's resolution. If not, we have
   * to find the nearest neighbor.
   */

  for (i = 0; i < X_variables[variable_id].dimension; i++){
    nearest_neighbor_index[i] = 0;	/* doesn't contain a regular point */
    if (!range_min) min = 0.0; else min = range_min[i];
    if (!range_max) max = 1.0; else max = range_max[i];
    for (j = 0; j < X_variables[variable_id].resolution &&
	   nearest_neighbor_index[i] == 0; j++){
      value = (((float) j) + 0.5)
	/ ((float) X_variables[variable_id].resolution);
      if (value >= min && value <= max)
	nearest_neighbor_index[i] = -1;
    }
    if (nearest_neighbor_index[i] == 0){	/* didn't find a value */
      nearest_neighbor_index[i] = 
	(int) ((0.5 * (min + max)) *
	       ((float) X_variables[variable_id].resolution));
      if (nearest_neighbor_index[i] == X_variables[variable_id].resolution)
	nearest_neighbor_index[i] = X_variables[variable_id].resolution - 1;
    }    
  }
    
      
  /*
   * initialization loop
   */

  total_value = 0.0;
  for (index = 0; index < X_variables[variable_id].num_values; index++){
    
    /*
     * Print out the counters - just for testing purposes
     */
    
    if (verbose){
      fprintf(stderr, "index %d: ", index);
      for (i = 0; i < X_variables[variable_id].dimension; i++)
	fprintf(stderr, " %.2d", counter[i]);
      fprintf(stderr, "\n");
    }

    /*
     * check, if that value is inside the interval
     */

    inside = 1;
    for (i = 0; i < X_variables[variable_id].dimension && inside; i++){
      value = (((float) counter[i]) + 0.5)
	/ ((float) X_variables[variable_id].resolution);
      if (!range_min) min = 0.0; else min = range_min[i];
      if (!range_max) max = 1.0; else max = range_max[i];
      if (counter[i] != nearest_neighbor_index[i]){
	if (value < min)
	  inside = 0;
	if (value > max)
	  inside = 0;
      }
    }
    
    /*
     * Initialize value
     */

    if (inside){
      X_variables[variable_id].value[index] = 1.0;
      total_value += 1.0;
    }
    else
      X_variables[variable_id].value[index] = 0.0;
    

    /*
     * increment counter
     */
    
    for (i = 0, done = 0; 
	 i < X_variables[variable_id].dimension &&!done; i++){
      counter[i]++;
      if (counter[i] < X_variables[variable_id].resolution)
	done = 1;
      else
	counter[i] = 0;
    }
  }

  /*
   * normalize
   */


  if (total_value == 0.0){
    fprintf(stderr,
	    "BUG: Bounds are too tight - strange, this shouldn't happen.\n");
    exit(-1);
  }

  factor = 1.0 / total_value;
  for (i = 0; i < X_variables[variable_id].num_values; i++)
    X_variables[variable_id].value[i] *= factor;




  /*
   * find maximum value and rescale display, display result
   */

  if (X_variables[variable_id].display != -1 && global_graphics_initialized &&
      X_display){
    G_matrix_set_display_range(X_variables[variable_id].display, 0.0, factor);
    G_display_matrix(X_variables[variable_id].display);
  }


  /*
   * free memory
   */

  free(nearest_neighbor_index);
  free(counter);

}




/************************************************************************\
 ************************************************************************
\************************************************************************/





/* **********************************************************************
 *
 * X_set_variable(): Loads a particular value into a regular variable
 *
 * **********************************************************************/

void
X_set_variable(int variable_id, float *values)
{
  int i, k;


  /*
   * sanity checks
   */

  if (variable_id < 0 || variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    variable_id);
    return;
  }

  
  for (i = 0; i < X_variables[variable_id].dimension; i++)
    if (values[i] < 0.0 || values[i] > 1.0){
      fprintf(stderr,
	      "ERROR: values must be in [0,1] in X_set_variable.\n");
      return;
    }

  
  /*
   * ******************** initialize ********************
   */

  if (X_variables[variable_id].type == DETERMINISTIC){
    
    for (i = 0; i < X_variables[variable_id].dimension; i++)
      X_variables[variable_id].value[i] = values[i];


  }

  else{				/* stochastic variable */
    
    /*
     * boring special case
     */
    
    if (X_variables[variable_id].dimension == 0)
      X_variables[variable_id].value[0] = 1.0;
    
    else{       
      /*
       * find index of the one lucky point
       */
      
      for (i = 0, k = 0; i < X_variables[variable_id].dimension; i++){
	k += (int) (values[X_variables[variable_id].dimension - i - 1]  *
		    ((float) (X_variables[variable_id].resolution - 1)));
	if (i != X_variables[variable_id].dimension - 1)
	  k = k * X_variables[variable_id].resolution;
      }
      
      /*
       * initialize
       */
      
      for (i = 0; i < X_variables[variable_id].num_values; i++)
	if (i == k)
	  X_variables[variable_id].value[i] = 1.0;
	else
	  X_variables[variable_id].value[i] = 0.0;
    }
  }
    
  X_clear_all_gradients(variable_id, 0);


  /*
   * display the values
   */

  if (X_variables[variable_id].display != -1 &&
      global_graphics_initialized &&
      X_display){
    G_matrix_set_display_range(X_variables[variable_id].display, 0.0, 1.0);
    G_display_matrix(X_variables[variable_id].display);
  }
}



/************************************************************************\
 ************************************************************************
\************************************************************************/





/* **********************************************************************
 *
 * X_copy_variable(): Copies values of one variable into another
 *                    must both be of the same type (DETERMINISTIC
 *                    or STOCHASTIC)
 *
 * **********************************************************************/

void
X_copy_variable(int from_variable_id, int to_variable_id)
{
  int i, j;
  int net_id = -1;
  variable_type *from_variable = NULL;
  variable_type *to_variable = NULL;
  float max;

  /*
   * ******************** sanity checks ********************
   */

  if (to_variable_id < 0 || to_variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    to_variable_id);
    return;
  }

  if (from_variable_id < 0 || from_variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    from_variable_id);
    return;
  }

  if (X_variables[to_variable_id].type != X_variables[from_variable_id].type){
    fprintf(stderr, "ERROR: variables must be of same type.\n");
    return;
  }

  if (X_variables[to_variable_id].dimension !=
      X_variables[from_variable_id].dimension){
    fprintf(stderr, "ERROR: variables must have same dimension.\n");
    return;
  }

  if (X_variables[to_variable_id].resolution !=
      X_variables[from_variable_id].resolution){
    fprintf(stderr, "ERROR: variables must have same resolution.\n");
    return;
  }

  if (from_variable_id == to_variable_id){
    fprintf(stderr, "ERROR: identical variables in X_set_variable.\n");
    return;
  }

  if (X_variables[to_variable_id].num_values !=
      X_variables[from_variable_id].num_values){
    fprintf(stderr, "ERROR: variables must have same num_values.\n");
    fprintf(stderr, "ERROR: ...but this message shouldn't come up.\n");
    return;
  }

  /*
   * ******************** Okay, let's do it. ********************
   */

  from_variable = &(X_variables[from_variable_id]);
  to_variable   = &(X_variables[to_variable_id]);

  /* copy values */

  for (i = 0; i < X_variables[to_variable_id].num_values; i++){
    to_variable->value[i]         = from_variable->value[i];
    if (from_variable->type == STOCHASTIC){
      to_variable->cumul_value[i] = from_variable->cumul_value[i];
      to_variable->avg_value[i]   = from_variable->avg_value[i];
      to_variable->norm_value[i]  = from_variable->norm_value[i];
    }
  }

  /* copy errors */

  /*   for (i = 0; i < NUM_SET_TYPES; i++) */
  /*     to_variable->error[i] = from_variable->error[i]; */

  /* copy gradients */

  X_clear_all_gradients(to_variable_id, 1);

  to_variable->num_gradients = from_variable->num_gradients;

  for (i = 0; i < from_variable->num_gradients; i++){
    net_id = from_variable->gradients[i].network_id;
    to_variable->gradients[i].network_id = net_id;
    to_variable->gradients[i].value = 
      (float *) malloc(sizeof(float) * 
		       from_variable->num_values *
		       X_networks[net_id].num_params);
    for (j = 0;
	 j < from_variable->num_values * X_networks[net_id].num_params; j++)
      to_variable->gradients[i].value[j] = 
	from_variable->gradients[i].value[j];
  }


  for (i = 0; i < from_variable->num_cumul_gradients; i++){
    net_id = from_variable->cumul_gradients[i].network_id;
    to_variable->cumul_gradients[i].network_id = net_id;
    to_variable->cumul_gradients[i].value = 
      (float *) malloc(sizeof(float) * 
		       from_variable->num_values *
		       X_networks[net_id].num_params);
    for (j = 0;
	 j < from_variable->num_values * X_networks[net_id].num_params; j++)
      to_variable->cumul_gradients[i].value[j] = 
	from_variable->cumul_gradients[i].value[j];
  }
  to_variable->cumul_divisor = from_variable->cumul_divisor;


  for (i = 0; i < from_variable->num_avg_gradients; i++){
    net_id = from_variable->avg_gradients[i].network_id;
    to_variable->avg_gradients[i].network_id = net_id;
    to_variable->avg_gradients[i].value = 
      (float *) malloc(sizeof(float) * 
		       from_variable->num_values *
		       X_networks[net_id].num_params);
    for (j = 0;
	 j < from_variable->num_values * X_networks[net_id].num_params; j++)
      to_variable->avg_gradients[i].value[j] = 
	from_variable->avg_gradients[i].value[j];
  }



  /*
   * find maximum value and rescale display, display result
   */

  if (to_variable->display != -1 && global_graphics_initialized &&
      X_display){
    max = 0.0;
    for (i = 0; i < to_variable->num_values; i++)
      if (max < to_variable->value[i])
	max = to_variable->value[i];
    G_matrix_set_display_range(to_variable->display, 0.0, max);
    G_display_matrix(to_variable->display);
  }



}

/************************************************************************\
 ************************************************************************
\************************************************************************/




/* **********************************************************************
 *
 * X_convolve_with_Gaussian(): Convolves a stochastic variable with a 
 *                             Gaussian kernel
 *
 * **********************************************************************/

void
X_convolve_with_Gaussian(int variable_id,
			 float *means,
			 float *variances, 
			 float cut_off)
{
  int   *counter;
  float difference1, difference2;
  float *new_value;
  int    index1, index2, done, increment_done, i, j, k, l1, l2, net_id;
  int    verbose = 0;
  float  weight, weight1, weight2, sum, max;
  float  *new_gradients[MAX_NUMBER_NETWORKS];
  int    n_plus, n_minus;

  /*
   * ******************** sanity checks ********************
   */

  if (variable_id < 0 || variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    variable_id);
    return;
  }

  if (X_variables[variable_id].type != STOCHASTIC){
    fprintf(stderr, "ERROR: Only stochastic variables can be convolved.\n");
    return;
  }
  
  for (i = 0; i < X_variables[variable_id].dimension; i++)
    if (variances[i] < 0.0){
      fprintf(stderr, "ERROR: variance must be >= 0 for convolving.\n");
      return;
    }
  
  /*
   * ******************** Okay, let's do it. ********************
   */

  /*
   * memory allocation
   */

  counter = (int *) malloc(sizeof(int) * 2 * 
			   X_variables[variable_id].dimension);
  new_value = (float *) malloc(sizeof(float) * 
				X_variables[variable_id].num_values);

  if (!counter || !new_value){
    fprintf(stderr, "ERROR: out of memory in X_convolve_with_Gaussian.\n");
    exit(-1);
  }
  
  for (i = 0; i < 2 * X_variables[variable_id].dimension; i++)
    counter[i]    = 0;
  
  for (i = 0; i < X_variables[variable_id].num_values; i++)
    new_value[i] = 0.0;

  index1 = 0;
  index2 = 0;
  

  for (j = 0; j < X_variables[variable_id].num_gradients; j++){
    net_id           = X_variables[variable_id].gradients[j].network_id;
    new_gradients[j] = 
      (float *) malloc(sizeof(float) * 
		       X_variables[variable_id].num_values *
		       X_networks[net_id].num_params);
    if (!new_gradients[j]){
      fprintf(stderr, "Out of memory in X_convolve_with_Gaussian().\n");
      exit(-1);
    }
    for (i = 0; i <  X_variables[variable_id].num_values *
	   X_networks[net_id].num_params; i++)
      new_gradients[j][i] = 0.0;
  }


  /*
   * ******************** Convolve ********************
   */

  done = 0;
  n_plus = n_minus = 0;
  do{

    /*
     * Print out the counters - just for testing purposes
     */

    if (verbose){
      for (i = 0; i < 2 * X_variables[variable_id].dimension; i++){
	if (i == X_variables[variable_id].dimension)
	  fprintf(stderr, "      ");
	fprintf(stderr, " %.2d",
		counter[2 * X_variables[variable_id].dimension-i-1]);
      }
      fprintf(stderr, "      %d %d\n", index2, index1);
    }

    /*
     * Compute the difference along the different dimensions,
     * and the weight. This is only approximately correct!
     */

    weight1  = 0.0;
    weight2 = 1.0;
    for (i = 0; i < X_variables[variable_id].dimension; i++){
      /* difference under Gaussian */
      difference1 =
	(((float) (counter[i] - counter[i+X_variables[variable_id].dimension]))
	/ X_variables[variable_id].resolution) + means[i];
      if (X_variables[variable_id].cyclic){
	for (; difference1 >  0.5; ) difference1 -= 1.0;
	for (; difference1 < -0.5; ) difference1 += 1.0;/* cyclic */
      }
      if (variances[i] > 0.0)
	weight1 -= (difference1 * difference1 / (variances[i] * variances[i]));
      else if (fabs(difference1) > 0.0001)
	weight1 -= 99999999999.9;

      /* discretization effect */
      difference2 = 1.0 -
	fabs(difference1 * ((float) X_variables[variable_id].resolution));
      if (difference2 < 0.0)
	weight2 = 0.0;
      else
	weight2 *= difference2;
    }    
    weight = exp(weight1) + weight2;



    if (weight > cut_off){

      /*
       * Add to value
       */
      
      new_value[index2] += weight * X_variables[variable_id].value[index1];
      
      /*
       * Add to gradients
       */
      

      for (j = 0; j < X_variables[variable_id].num_gradients; j++){
	net_id = X_variables[variable_id].gradients[j].network_id;
	for (k = 0; k < X_networks[net_id].num_params; k++){
	  l1 = index1 * X_networks[net_id].num_params + k;
	  l2 = index2 * X_networks[net_id].num_params + k;
	  new_gradients[j][l2] +=
	    weight * X_variables[variable_id].gradients[j].value[l1];
	}      
      }      
    n_plus++;
    }

    else
      n_minus++;
      
      
    /*
     * increment
     */
    
    for (i = 0, increment_done = 0; 
	 i < 2 * X_variables[variable_id].dimension && 
	   !increment_done && !done; i++){
      counter[i]++;
      if (counter[i] < X_variables[variable_id].resolution)
	increment_done = 1;
      else
	counter[i] = 0;
      
      if (i == 0)
	index1++;		/* we incremented the first variable */
      else if (i == X_variables[variable_id].dimension){
	index2++;		/* now we are adding to the second varibale */
	index1 = 0;		/* and the first has been reset to 0 */
      }
    }
    if (!increment_done)
      done = 1;			/* overflow, we made it through once */
    

  } while (!done);

  if (verbose)
    fprintf(stderr, "Convolve: Updated %8.6f%% of all pairs of values.\n",
	    ((float) n_plus) / ((float) (n_plus + n_minus)) * 100.0);

  /*
   * copy over
   */

  for (i = 0; i < X_variables[variable_id].num_values; i++)
    X_variables[variable_id].value[i] = new_value[i];
  free(new_value);
  new_value = NULL;

  for (j = 0; j < X_variables[variable_id].num_gradients; j++){
    free(X_variables[variable_id].gradients[j].value);
    X_variables[variable_id].gradients[j].value = new_gradients[j]; 
    new_gradients[j] = NULL;
  }

  /*
   * normalize
   */

  X_normalize_variable(variable_id);



  /*
   * find maximum value and rescale display, display result
   */

  if (X_variables[variable_id].display != -1 &&
      global_graphics_initialized &&
      X_display){
    max = 0.0;
    for (i = 0; i < X_variables[variable_id].num_values; i++)
      if (max < X_variables[variable_id].value[i])
	max = X_variables[variable_id].value[i];
    G_matrix_set_display_range(X_variables[variable_id].display, 0.0, max);
    G_display_matrix(X_variables[variable_id].display);
  }


  /*
   * free memory (unless already happened)
   */

  free(counter);


}





/* **********************************************************************
 *
 * X_convolve_with_uniform_dist(): Convolves a stochastic variable with a 
 *                                 Gaussian kernel
 *
 * **********************************************************************/

void
X_convolve_with_uniform_dist(int variable_id,
			     float randomness) /* between 0 and 1 - 1 will
						* result in uniform 
						* distribution */
{
  float add_term, mult_term;
  int i, j, k, l, net_id;
  variable_type *variable = NULL;
  
  /*
   * ******************** sanity checks ********************
   */
  
  if (variable_id < 0 || variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    variable_id);
    return;
  }

  if (X_variables[variable_id].type != STOCHASTIC){
    fprintf(stderr, "ERROR: Only stochastic variables can be convolved.\n");
    return;
  }
  
  if (randomness < 0.0 || randomness > 1.0){
    fprintf(stderr, "ERROR: randomness value must be in [0,1].\n");
    return;
  }

  if (randomness == 0.0)
    return;
  
  /*
   * do the job
   */

  /*
   * change gradients
   */

  variable  = &(X_variables[variable_id]);
  add_term  = randomness / ((float) variable->num_values);
  mult_term = (1.0 - randomness);

  for (i = 0; i < variable->num_values; i++){
    variable->value[i] *= mult_term;    
    variable->value[i] += add_term;
  }
  
  for (j = 0; j < variable->num_gradients; j++){
    net_id = variable->gradients[j].network_id;
    for (i = 0, l = 0; i < variable->num_values; i++)
      for (k = 0; k < X_networks[net_id].num_params; k++, l++)
	variable->gradients[j].value[l] *= mult_term;
  }
}




/************************************************************************\
 ************************************************************************
\************************************************************************/




/* **********************************************************************
 *
 * X_normalize_variable(): normalizes a variable and changes the
 *                         gradients accordingly (chain rule)
 *
 * **********************************************************************/

void
X_normalize_variable(int variable_id)
{
  float  sum, factor;
  int    i, j, k, l, net_id;

  /*
   * Sanity checks
   */

  if (variable_id < 0 || variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    variable_id);
    return;
  }

  /*
   * Normalize
   */
  
  sum = 0.0;
  for (i = 0; i < X_variables[variable_id].num_values; i++)
    sum += X_variables[variable_id].value[i];

  if (sum <= 1e-30){
    fprintf(stderr, "ERROR: numerical underflow in X_normalize_variable: ");
    fprintf(stderr, "%d: [%s]\n", variable_id, X_variables[variable_id].name);
    X_initialize_uniformly(variable_id, NULL, NULL);
  }
  else{
    factor = 1.0 / sum;
    
    /*
     * change gradients
     */
    
    for (j = 0; j < X_variables[variable_id].num_gradients; j++){
      net_id = X_variables[variable_id].gradients[j].network_id;
      for (k = 0; k < X_networks[net_id].num_params; k++){
	sum = 0.0;
	for (i = 0; i < X_variables[variable_id].num_values; i++){
	  l = i * X_networks[net_id].num_params + k;
	  sum += X_variables[variable_id].gradients[j].value[l];
	}
	for (i = 0; i < X_variables[variable_id].num_values; i++){
	  l = i * X_networks[net_id].num_params + k;
	  X_variables[variable_id].gradients[j].value[l] =
	    factor * (X_variables[variable_id].gradients[j].value[l] -
		      (factor * X_variables[variable_id].value[i] * sum));
	}      
      }      
    }
    
    
    /*
     * change variable
     */
    for (i = 0; i < X_variables[variable_id].num_values; i++)
      X_variables[variable_id].value[i] *= factor;
  }

}


/* **********************************************************************
 *
 * X_truncate_variable(): cuts of small probabilities
 *
 * **********************************************************************/

void
X_truncate_variable(int variable_id, float threshold)
{
  float  max;
  int    i;
  static int msg = 0;

  if (!msg){
    fprintf(stderr, "\n\n\t### WARNING: Incorrect gradients in ");
    fprintf(stderr, "X_truncate_variable ###\n\n");
    msg = 1;
  }

  /*
   * Sanity checks
   */

  if (variable_id < 0 || variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    variable_id);
    return;
  }

  if (threshold > 1.0 || threshold <= 0.0){
    fprintf(stderr, 
	    "ERROR: Truncation threshold (%g) must be in (0,1].\n", threshold);
    return;
  }


  /*
   * Find maximum
   */

  max = 0.0;
  for (i = 0; i < X_variables[variable_id].num_values; i++)  
    if (X_variables[variable_id].value[i] > max)
      max = X_variables[variable_id].value[i];

  /*
   * truncate
   */

  max = max * threshold;

  for (i = 0; i < X_variables[variable_id].num_values; i++)  
    if (X_variables[variable_id].value[i] < max)
      X_variables[variable_id].value[i] = 0.0;
  

  /*
   * Warning: the gradient computation is incorrect
   */

  X_normalize_variable(variable_id);



  /*
   * find maximum value and rescale display, display result
   */

  if (X_variables[variable_id].display != -1 &&
      global_graphics_initialized &&
      X_display){
    max = 0.0;
    for (i = 0; i < X_variables[variable_id].num_values; i++)
      if (max < X_variables[variable_id].value[i])
	max = X_variables[variable_id].value[i];
    G_matrix_set_display_range(X_variables[variable_id].display, 0.0, max);
    G_display_matrix(X_variables[variable_id].display);
  }

}



/************************************************************************\
 ************************************************************************
\************************************************************************/




/* **********************************************************************
 *
 * X_integrate_variables(): takes a stochastic variable and integrates it into
 *                another one
 *
 * **********************************************************************/

void
X_integrate_variables(int belief_variable_id, int evidence_variable_id)
{
  int i, j, k, l;
  variable_type *belief_variable = NULL;
  variable_type *evidence_variable = NULL;
  network_type  *network = NULL;  
  float         max;
  float         factor;

  /*
   * Sanity checks
   */

  if (belief_variable_id < 0 || belief_variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    belief_variable_id);
    return;
  }

  if (evidence_variable_id < 0 ||
      evidence_variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    evidence_variable_id);
    return;
  }
  
  if (X_variables[belief_variable_id].type != STOCHASTIC ||
      X_variables[evidence_variable_id].type != STOCHASTIC){
    fprintf(stderr, 
	    "ERROR: Both variables in X_integrate_variables must be stochastic.\n");
     return;
  }

  if (X_variables[belief_variable_id].dimension !=
      X_variables[evidence_variable_id].dimension ||
      X_variables[belief_variable_id].resolution !=
      X_variables[evidence_variable_id].resolution ||
      X_variables[belief_variable_id].num_values !=
      X_variables[evidence_variable_id].num_values){
    fprintf(stderr, "ERROR: Both variables in X_integrate_variables must");
    fprintf(stderr, " have same dimension + resolution %d %d %d   %d %d %d.\n",
	    X_variables[belief_variable_id].dimension,
	    X_variables[belief_variable_id].resolution,
	    X_variables[belief_variable_id].num_values,
	    X_variables[evidence_variable_id].dimension,
	    X_variables[evidence_variable_id].resolution,
	    X_variables[evidence_variable_id].num_values);
    return;
  }

  /*
   * Okay, let's do it
   */


  belief_variable   = &(X_variables[belief_variable_id]);
  evidence_variable = &(X_variables[evidence_variable_id]);

  /*
   * Add the "evidence" variable's values and gradients to its
   * cumulative variables, so that we can later compute an accurate
   * average (and compute the proper gradients)
   */

  if (X_training_phase == 1){
    for (i = 0; i < evidence_variable->num_values; i++)
      evidence_variable->cumul_value[i] +=
	evidence_variable->value[i];
    evidence_variable->cumul_divisor++; /* increment counter */
    
    for (i = 0; i < evidence_variable->num_gradients; i++)
      X_add_gradients(evidence_variable_id,
		      evidence_variable->gradients[i].network_id,
		      1,	/* these gradients will be added on
				 * average gradients */
		      evidence_variable->gradients[i].value,
		      1.0,	/* no factor here */
		      -1);	/* all gradients here */
  }

  /*
   * Integrate variables
   */


  for (i = 0; i < belief_variable->num_values; i++){
    /* factor: P(evidence)^-1 */
    if (evidence_variable->avg_value[i] > 0.0)
      factor = 1.0 / evidence_variable->avg_value[i];
    else{
      static int warned = 0;
      factor = 1e+10;
      if (!warned){
	fprintf(stderr, "WARNING: Division by Zero - Computation might ");
	fprintf(stderr, "be numerically instable.\n");
	warned = 1;
      }
    }

    /* compute the gradients: part 1 */
    for (j = 0; j < belief_variable->num_gradients; j++){
      network = &(X_networks[belief_variable->gradients[j].network_id]);
      k = i * network->num_params; /* index of vector of gradients */
      X_add_gradients(belief_variable_id, 
		      belief_variable->gradients[j].network_id,
		      0,	/* 0=add regular gradients, 
				 * not "cumulative mode" */
		      &(belief_variable->gradients[j].value[k]),
		      ((factor * evidence_variable->value[i]) - 1.0),
		      i);	/* this is really a multiplication with
				 * evidence_variable->value[i],
				 * of course divided by P(evidence) */
    }

    /* compute the gradients: part 2 */
    for (j = 0; j < evidence_variable->num_gradients; j++){
      network = &(X_networks[evidence_variable->gradients[j].network_id]);
      k = i * network->num_params; /* index of vector of gradients */
      X_add_gradients(belief_variable_id, 
		      evidence_variable->gradients[j].network_id,
		      0,	/* 0=add regular gradients, 
				 * not "cumulative mode" */
		      &(evidence_variable->gradients[j].value[k]),
		      factor * belief_variable->value[i],
		      i);	/* this is really a multiplication with
				 * belief_variable->value[i],
				 * of course divided by P(evidence) */
    }


    /* compute the gradients: part 3 */
    for (j = 0; j < evidence_variable->num_avg_gradients; j++){
      network = &(X_networks[evidence_variable->avg_gradients[j].network_id]);
      k = i * network->num_params; /* index of vector of gradients */
      X_add_gradients(belief_variable_id, 
		      evidence_variable->avg_gradients[j].network_id,
		      0,	/* 0=add regular gradients, 
				 * not "cumulative mode" */
		      &(evidence_variable->avg_gradients[j].value[k]),
		      -1.0 * factor * factor * belief_variable->value[i]
		      * evidence_variable->value[i],
		      i);	
    }

    /* multiply the values (must come last) */
    evidence_variable->norm_value[i] = factor * evidence_variable->value[i];
    belief_variable->value[i] *= evidence_variable->norm_value[i];
  }


  X_normalize_variable(belief_variable_id);


  /*
   * display
   */

  X_display_variable(evidence_variable_id);
  X_display_variable(belief_variable_id);

}


/* **********************************************************************
 *
 * X_max_likelihood()  computes the most likeli value of a stochastic
 *                     variable
 *
 * **********************************************************************/

void
X_max_likelihood(int stochastic_variable_id, int deterministic_variable_id)
{
  X_convert_stochastic_to_deterministic(stochastic_variable_id, 
					deterministic_variable_id,
					0);
}



/* **********************************************************************
 *
 * X_average()  computes the medium value of a stochastic variable
 *
 * **********************************************************************/


void
X_weighted_average(int stochastic_variable_id, int deterministic_variable_id)
{
  X_convert_stochastic_to_deterministic(stochastic_variable_id, 
					deterministic_variable_id,
					1);
}


/* **********************************************************************
 *
 * X_convert_stochastic_to_deterministic: does what the name suggests.
 *                  conversion_type = 0: maximum likelihood
 *                  conversion_type = 1: weighted average
 *
 * **********************************************************************/

void
X_convert_stochastic_to_deterministic(int stochastic_variable_id, 
				      int deterministic_variable_id,
				      int conversion_type)
{
  float best_prob, factor;
  int i, j, k, done, increment_done;
  int *counter    = NULL;

  /*
   * Sanity checks
   */

  if (stochastic_variable_id < 0 ||
      stochastic_variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    stochastic_variable_id);
    return;
  }

  if (deterministic_variable_id < 0 ||
      deterministic_variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    deterministic_variable_id);
    return;
  }

  
  if (X_variables[stochastic_variable_id].type != STOCHASTIC){
    fprintf(stderr, 
	    "ERROR: First variable in X_average must be stochastic.\n");
     return;
  }

  if (X_variables[deterministic_variable_id].type != DETERMINISTIC){
    fprintf(stderr, 
	    "ERROR: Second variable in X_average must be deterministic.\n");
     return;
  }

  if (X_variables[stochastic_variable_id].dimension != 
      X_variables[deterministic_variable_id].dimension){
    fprintf(stderr, 
	    "ERROR: Dimension misfit in X_average: %d != %d.\n",
	    X_variables[stochastic_variable_id].dimension, 
	    X_variables[deterministic_variable_id].dimension);
     return;
  }

  if (conversion_type != 0 && conversion_type != 1){
    fprintf(stderr, "ERROR: Illegal conversion_type: %d\n",
	    conversion_type);
    return;
  }

  /*
   * allocate memory
   */

  counter = (int *) malloc(sizeof(int) * 
			   X_variables[stochastic_variable_id].dimension);
  if (!counter){
    fprintf(stderr, "Out of memory in X_average().\n");
    exit(-1);
  }

  for (i = 0; i < X_variables[stochastic_variable_id].dimension; i++)
    counter[i] = 0;


  /*
   * Clear target variable
   */

  for (j = 0; j < X_variables[deterministic_variable_id].num_values; j++)
    X_variables[deterministic_variable_id].value[j] = 0.0;
  X_clear_all_gradients(deterministic_variable_id, 0);
 
  best_prob = -1.0;


  /*
   * Okay, do the job
   */

  for (done = 0, k = 0; 
       k < X_variables[stochastic_variable_id].num_values && !done; k++){
    
    if (conversion_type == 0){
      /*
       * compute max likelihood
       */
      if (X_variables[stochastic_variable_id].value[k] > best_prob){
	best_prob = X_variables[stochastic_variable_id].value[k];
	for (j = 0; j < X_variables[deterministic_variable_id].dimension; j++)
	  X_variables[deterministic_variable_id].value[j] =
	    (((float) counter[j]) + 0.5) / 
	    ((float)  X_variables[stochastic_variable_id].resolution);
      }
    }

    else if (conversion_type == 1){
      /*
       * compute weighted average
       */
      factor = X_variables[stochastic_variable_id].value[k];
      for (j = 0; j < X_variables[deterministic_variable_id].dimension; j++)
	X_variables[deterministic_variable_id].value[j] +=
	  factor *
	  (((float) counter[j]) + 0.5) / 
	  ((float)  X_variables[stochastic_variable_id].resolution);
    }
    else
      fprintf(stderr, "???");
    
    /*
     * increment counter
     */
    
    for (i = 0, increment_done = 0; 
	 i < X_variables[stochastic_variable_id].dimension && 
	   !increment_done; i++){
      counter[i]++;
      if (counter[i] < X_variables[stochastic_variable_id].resolution)
	increment_done = 1;
      else 
	counter[i] = 0;
    }
    if (!increment_done)
      done = 1;			/* overflow, we made it through once */
    
  }
  
  /*
   * sanity check for the programmer
   */

  if (k != X_variables[stochastic_variable_id].num_values || !done){
    fprintf(stderr, "STRANGE: %d %d %d!!!\n",
	    k, X_variables[stochastic_variable_id].num_values, done);
    exit(-1);
  }

  /*
   * display
   */

  X_display_variable(deterministic_variable_id);

  /*
   * free memory
   */

  free(counter);

  /*
   * warning
   */

  {
    static int msg = 0;
    
    if (!msg){
      fprintf(stderr, "\n\n\t### WARNING: No gradients in ");
      fprintf(stderr, "X_convert_stochastic_to_deterministic ###\n\n");
      msg = 1;
    }
  }
}




/************************************************************************\
 ************************************************************************
\************************************************************************/




/* **********************************************************************
 *
 * X_execute_subroutine: executes a soubroutine (including all gradients)
 *   
 *
 * **********************************************************************/


void
X_execute_subroutine(X_subroutine_type subroutine(),
		     int   num_input_variables,
		     int   num_output_variables,
		     int  *input_variables,
		     int  *output_variables)
{
  float **input             = NULL;
  float **output            = NULL;
  float *input_values       = NULL;
  int   *input_dimensions   = NULL;
  float *output_values      = NULL;
  int   *output_dimensions  = NULL;
  int   *input_index        = NULL;
  int   *output_sub_index   = NULL;
  int   *output_sub_counter = NULL;
  float *output_sub_weight  = NULL;
  float *partial_prob       = NULL;
  float prob, max, factor, sum_factor;
  float *grad_value = NULL;	/* aux pointer */
  int i, j, k, l, m, n;
  int verbose = 0;
  int total_input_dimension, total_output_dimension;
  int input_deterministic, input_stochastic;
  int output_deterministic, output_stochastic;
  int *input_counter;
  int *input_resolution;
  int *input_type;
  int done, sub_done, increment_done;
  int output_index, local_index, index_factor;

  int defined;

  /*
   * sanity checks
   */

  if (num_input_variables < 0){
    fprintf(stderr, "ERROR: Too few input variables: %d\n",
	    num_input_variables);
    return;
  }

  if (num_output_variables < 1){
    fprintf(stderr, "ERROR: Too few output variables: %d\n",
	    num_output_variables);
    return;
  }

  if (num_input_variables > 0 && input_variables == NULL){
    fprintf(stderr, "ERROR: Input variables are missing.\n");
    return;
  }

  if (output_variables == NULL){
    fprintf(stderr, "ERROR: Output variables are missing.\n");
    return;
  }


  /*
   * count types of variables, and run a sanity check
   */

  input_deterministic   = input_stochastic = 0;
  output_deterministic  = output_stochastic = 0;
  total_input_dimension = total_output_dimension = 0;

  for (i = 0; i < num_input_variables; i++){
    if (X_variables[input_variables[i]].type == DETERMINISTIC)
      input_deterministic++;
    if (X_variables[input_variables[i]].type == STOCHASTIC)
      input_stochastic++;
    total_input_dimension += X_variables[input_variables[i]].dimension;
  }

  for (i = 0; i < num_output_variables; i++){
    if (X_variables[output_variables[i]].type == DETERMINISTIC)
      output_deterministic++;
    if (X_variables[output_variables[i]].type == STOCHASTIC)
      output_stochastic++;
    total_output_dimension += X_variables[output_variables[i]].dimension;
  }

  if (input_stochastic > 0 && output_deterministic > 0){
    fprintf(stderr, "ERROR: Deterministic output not possible ");
    fprintf(stderr, "when stochastic inputs are used.\n");
    return;
  }
  /*
  if (input_deterministic > 0 || output_deterministic > 0){
    fprintf(stderr,
	    "ERROR: Currently unable to handle deterministic variables.\n");
    return;
  }
  */
  
  /*
   * allocate space for input and output values
   */
  
  input            = (float **) malloc(sizeof(float*) * num_input_variables);
  input_values     = (float *)  malloc(sizeof(float)  * total_input_dimension);
  input_dimensions = (int *)    malloc(sizeof(int)    * num_input_variables);
  input_counter    = (int *)    malloc(sizeof(int)    * total_input_dimension);
  input_resolution = (int *)    malloc(sizeof(int)    * total_input_dimension);
  input_type       = (int *)    malloc(sizeof(int)    * total_input_dimension);
  input_index      = (int *)    malloc(sizeof(int)    * num_input_variables);
  if (input == NULL || input_values == NULL || input_dimensions == NULL ||
      input_counter == NULL || input_resolution == NULL || 
      input_type == NULL || input_index == NULL){
    fprintf(stderr, "ERROR: out of memory 1 in X_execute_subroutine.\n");
    exit(-1);
  }
  for (i = 0, k = 0; i < num_input_variables; i++){
    input[i] = &(input_values[k]);
    input_dimensions[i] = X_variables[input_variables[i]].dimension;
    for (j = 0; j < X_variables[input_variables[i]].dimension; j++, k++){
      input_counter[k]    = 0;
      input_resolution[k] = X_variables[input_variables[i]].resolution;
      input_type[k]       = X_variables[input_variables[i]].type;
      if (X_variables[input_variables[i]].type == DETERMINISTIC)
	input_values[k]   = X_variables[input_variables[i]].value[j];
      else
	input_values[k]   = 0.0;
    }
  }
  if (k != total_input_dimension){ /* sanity check */
    fprintf(stderr, "BUG(1): %d != %d\n", k, total_input_dimension);
    exit(-1);
  }
  

  output            = (float **) malloc(sizeof(float*) * num_output_variables);
  output_dimensions = (int *) malloc(sizeof(int)     * num_output_variables);
  output_values     = (float *)  malloc(sizeof(float)* total_output_dimension);
  output_sub_index  = (int *)    malloc(sizeof(int)  * total_output_dimension);
  output_sub_counter= (int *)    malloc(sizeof(int)  * total_output_dimension);
  output_sub_weight = (float *)  malloc(sizeof(float)* total_output_dimension);
  if (output == NULL        || output_dimensions == NULL || 
      output_values == NULL || output_sub_index == NULL ||
      output_sub_counter == NULL || output_sub_weight == NULL){
    fprintf(stderr, "ERROR: out of memory 2 in X_execute_subroutine.\n");
    exit(-1);
  }
  for (i = 0, k = 0; i < num_output_variables; 
       k += X_variables[output_variables[i]].dimension, i++){
    /* fprintf(stderr, "Set output %d to %d-th output value\n", i, k); */
    output[i] = &(output_values[k]);
    output_dimensions[i] = X_variables[output_variables[i]].dimension;
  }
  if (k != total_output_dimension){ /* sanity check */
    fprintf(stderr, "BUG(2): %d != %d\n", k, total_output_dimension);
    exit(-1);
  }

  /*
   * allocate space for gradients
   */

  if (X_calculate_gradients)
    partial_prob = (float *) malloc(sizeof(float) * num_input_variables);


  /*
   * clear the output variables and all their gradients
   */

  for (i = 0; i < num_output_variables; i++){
    for (j = 0; j < X_variables[output_variables[i]].num_values; j++)
      X_variables[output_variables[i]].value[j] = 0.0;
    X_clear_all_gradients(output_variables[i], 0);
  }


  /*
   * Now let's circle through all possible variable values
   * and build up our input-output table for this specific subroutine
   */

  for (done = 0; !done; ){
    
    /*
     * set values
     */

    for (i = 0; i < total_input_dimension; i++)
      if (input_type[i] == STOCHASTIC)
	/* input_values[i] = 
	   ((float) input_counter[i]) / ((float) (input_resolution[i] - 1));*/
	  input_values[i] = 
	    (((float) input_counter[i]) + 0.5)
	    / ((float) (input_resolution[i]));


    

    /*
     * how likely is that value?
     */

    prob = 1.0;			/* initialization */
    if (X_calculate_gradients)
      for (i = 0; i < num_input_variables; i++)
	partial_prob[i] = 1.0; /* only used for gradient computation */
    /* i points to the number of the input variable */
    /* k points to the corresponding field in input_counter */
    /* j points to the dimension in the i-th input variable */
    for (i = 0, k = 0; i < num_input_variables; i++){
      input_index[i] = 0;
      index_factor   = 1;
      for (j = 0; j < X_variables[input_variables[i]].dimension; j++, k++){
	input_index[i] += (index_factor * input_counter[k]);
	index_factor   *= X_variables[input_variables[i]].resolution;
      }
      if (verbose)
	fprintf(stderr, " %d:%d ", i, input_index[i]);
      if (X_variables[input_variables[i]].type == STOCHASTIC){
	prob *= X_variables[input_variables[i]].value[input_index[i]];
	if (X_calculate_gradients)
	  for (j = 0; j < num_input_variables; j++)
	    if (i != j)
	      partial_prob[j] *= 
		X_variables[input_variables[i]].value[input_index[i]];
      }
    }


    if (prob > 0.0){	/*! comparison is bogus for gradients?? */

      
      /*
       * initialize output values
       */
      for (i = 0; i < total_output_dimension; i++)
	output_values[i] = -1.0;



      /*
       * call the subroutine
       */

      X_prob = prob;

      subroutine(num_input_variables, /* input */
		 input_dimensions,
		 input,
		 num_output_variables, /* ouput */
		 output_dimensions,
		 output);
      
      /*
       * print the result
       */

      if (verbose){
	for (i = 0; i < total_input_dimension; i++)
	  fprintf(stderr, "%5.3f ", input_values[i]);
	fprintf(stderr, " -> ");
	for (i = 0; i < total_output_dimension; i++)
	  fprintf(stderr, "%5.3f ", output_values[i]);
	fprintf(stderr, " (p=%e)", prob);
      }
      
      
      /*
       * discretize output
       */
      
      for (i = 0, k = 0; i < num_output_variables; i++){ /* over all output
							  * variables */
	/* 
	 * determine the index of the output variable 
	 */

	index_factor = 1;
	defined = 1;		/* procedure might not have set the
				 * output value, in which case define -> 0 */
	
	for (j = 0; j < X_variables[output_variables[i]].dimension; j++, k++){
	  output_sub_index[j]  = 0;
	  output_sub_weight[j] = 1.0;
	  if (output_values[k] != output[i][j])
	    fprintf(stderr, "\nBANG-1\n");
	  if (output_values[k] != -1.0 &&
	      (output_values[k] < 0.0 || output_values[k] > 1.0)){
	    fprintf(stderr, "ERROR: Subroutines must output values ");
	    fprintf(stderr, "between 0 and 1 (%g)", output_values[k]);
	  }
	  if (output_values[k] == -1.0)
	    defined = 0;
	  else{
	    if (!(X_variables[output_variables[i]].cyclic) &&
		output_values[k] <= 
		0.5 / ((float) X_variables[output_variables[i]].resolution)){
	      local_index          = 0;
	      output_sub_weight[j] = 0.0;
	    }
	    else if (!(X_variables[output_variables[i]].cyclic) &&
		     output_values[k] >= 
		     1.0 - (0.5 / ((float) X_variables[output_variables[i]].
				   resolution))){
	      local_index = X_variables[output_variables[i]].resolution - 1;
	      output_sub_weight[j] = 0.0;
	    }
	    else{
	      output_sub_weight[j] = 
		(output_values[k] *
		((float) X_variables[output_variables[i]].resolution)) - 0.5;
	      local_index = ((int) (1.0 + output_sub_weight[j])) - 1;
	      output_sub_weight[j] -= (float) local_index;
	    }
	    if (local_index < 0)
	      local_index += X_variables[output_variables[i]].resolution;
	    if (local_index >= X_variables[output_variables[i]].resolution)
	      local_index -= X_variables[output_variables[i]].resolution;
	    output_sub_index[j] = local_index;
	    index_factor *= X_variables[output_variables[i]].resolution;
	  }
	}



	if (defined){


	  sum_factor = 0.0;
	  
	  /*
	   * go through all relevant cells
	   */
	  
	  /* initialize counter */
	  for (j = 0; j < X_variables[output_variables[i]].dimension; j++)
	    output_sub_counter[j] = 0;
	  
	  /* loop */
	  for (sub_done = 0; !sub_done; ){	    
	    
	    output_index = 0;
	    factor = 1.0;
	    index_factor = 1;
	    for (j = 0; j < X_variables[output_variables[i]].dimension; j++){
	      output_index += (index_factor * 
			       ((output_sub_index[j] + output_sub_counter[j])
				% X_variables[output_variables[i]].
				resolution));
	      index_factor *= X_variables[output_variables[i]].resolution;
	      if (output_sub_counter[j])
		factor *= output_sub_weight[j];
	      else
		factor *= (1.0 - output_sub_weight[j]);

	    }

	    if (factor > 0.0){
	      
	      sum_factor += factor;
	      
	      /*
	      fprintf(stderr, " [");
	      for (j = 0; j < X_variables[output_variables[i]].dimension; j++)
		fprintf(stderr, " %d",  output_sub_counter[j]);
	      fprintf(stderr, "] -> %d %g %g\n", output_index, factor,
		      output_sub_weight[0]); 
		      */
	      
	      
	      if (verbose)
		fprintf(stderr, " %d:%d ", i, output_index);
	      
	      /*
	       * update the probability
	       */
	      
	      X_variables[output_variables[i]].value[output_index] += 
		(prob * factor);
	      
	      
	      /*
	       * compute the gradients, if required
	       */
	      
	      if (X_calculate_gradients){
		
		/* loop over all input variables */
		for (j = 0; j < num_input_variables; j++) 
		  /* and all gradients for those input variables */
		  for (n = 0;/* loop over all nets/grads. for inp-variable j */
		       n < X_variables[input_variables[j]].num_gradients; n++){
		    /* m is the number of the network */
		    m = 
		      X_variables[input_variables[j]].gradients[n].network_id;
		    grad_value = &(X_variables[input_variables[j]].
				   gradients[n].
				   value[input_index[j]
					* X_networks[m].num_params]);
		    X_add_gradients(output_variables[i], /* output variable */
				    m, /* network number */
				    0,	/* but not "cumulative mode" */
				    grad_value,
				    partial_prob[j] * factor, /* product rule*/
				    output_index); /* that's the output here */
		  }
	      }
	    }
	    
	    /*
	     * increment counter
	     */
	    
	    for (l = 0, increment_done = 0; 
		 l < X_variables[output_variables[i]].dimension &&
		   !increment_done; l++){
	      output_sub_counter[l]++;
	      if (output_sub_counter[l] < 2)
		increment_done = 1;
	      else
		output_sub_counter[l] = 0;
	    }
	    if (!increment_done)
	      sub_done = 1;	/* overflow, we made it through once */
	  }
	  
	  if (fabs(sum_factor - 1.0) > 0.001){ /* sanity check */
	    fprintf(stderr, "ALARM: %g\n", sum_factor);
	    for (j = 0; j < X_variables[output_variables[i]].dimension; j++)
	      fprintf(stderr, "%d: %d %d, %g cyclic=%d\n", j, 
		      output_sub_index[j], 
		      X_variables[output_variables[i]].resolution,
		      output_sub_weight[j], 
		      X_variables[output_variables[i]].cyclic);
	    exit(-1);
	  }
	}
      }
      if (verbose)
	fprintf(stderr, "\n");
    }
    /*
     * increment counter
     */
    
    for (i = 0, increment_done = 0; 
	 i < total_input_dimension && !increment_done && !done; i++){
      input_counter[i]++;
      if (input_counter[i] < input_resolution[i])
	increment_done = 1;
      else
	input_counter[i] = 0;
    }
    if (!increment_done)
      done = 1;			/* overflow, we made it through once */
  }

  /*
   * so we are done - clean up!
   */

  for (i = 0; i < num_output_variables; i++){

    /*
     * stochastic output variables must be scaled to [0,1]
     */
    
    if (X_variables[output_variables[i]].type == STOCHASTIC)
      X_normalize_variable(output_variables[i]);
    
    /*
     * If the output is a stochastic variable, we will now have to
     * rescale the display
     */
    
    if (X_variables[output_variables[i]].type  == STOCHASTIC &&
	X_variables[output_variables[i]].display != -1 &&
	global_graphics_initialized){
      max = 0.0;
      for (j = 0; j < X_variables[output_variables[i]].num_values; j++)
	if (max < X_variables[output_variables[i]].value[j])
	  max = X_variables[output_variables[i]].value[j];
      G_matrix_set_display_range(X_variables[output_variables[i]].display, 
				 0.0, max);
    }
    
    
    /*
     * display result
     */
    
    if (X_variables[output_variables[i]].display != -1 &&
	global_graphics_initialized &&
	X_display)
      G_display_matrix(X_variables[output_variables[i]].display);
  }

  /*
   * free space
   */


  if (X_calculate_gradients)
    free(partial_prob);
  free(output_sub_weight);
  free(output_sub_counter);
  free(output_sub_index);
  free(output_values);
  free(output_dimensions);
  free(output);
  free(input_index);
  free(input_type);
  free(input_resolution);
  free(input_counter);
  free(input_dimensions);
  free(input_values);
  free(input);
}




/************************************************************************\
 ************************************************************************
\************************************************************************/




/* **********************************************************************
 *
 * X_print_value: prints out a value of a variable (to file)
 *
 * **********************************************************************/


void
X_print_value(int variable_id, int value_nr, FILE *iop)
{
  FILE *iop2;

  /*
   * Sanity checks
   */

  if (variable_id < 0 || variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    variable_id);
    return;
  }

  if (value_nr < 0 || value_nr >= X_variables[variable_id].num_values){
    fprintf(stderr, "ERROR: value %d not available (0..%d)\n",
	    value_nr, X_variables[variable_id].num_values-1);
    return;
  }
  
  /*
   * okay, do it
   */

  if (iop != NULL)
    iop2 = iop;
  else
    iop2 = stdout;

  fprintf(iop2, "%g", X_variables[variable_id].value[value_nr]);
  fflush(iop2);
}


/* **********************************************************************
 *
 * X_get_value: returns a value of a variable (to file)
 *
 * **********************************************************************/


float
X_get_value(int variable_id, int value_nr)
{

  /*
   * Sanity checks
   */

  if (variable_id < 0 || variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    variable_id);
    return -1.0;
  }

  if (value_nr < 0 || value_nr >= X_variables[variable_id].num_values){
    fprintf(stderr, "ERROR: value %d not available (0..%d)\n",
	    value_nr, X_variables[variable_id].num_values-1);
    return -1.0;
  }
  
  /*
   * okay, do it
   */

  return (X_variables[variable_id].value[value_nr]);
}



/************************************************************************\
 ************************************************************************
\************************************************************************/





#define X_MIN_X          11.6
#define X_WIDTH_X         2.0
#define X_MAX_Y           5.4
#define X_MIN_Y           (BUTTON_MIN_Y)
#define X_SEPARATOR       0.1
#define X_NAME_Y          0.4
#define X_MAX_FIELD_WIDTH 0.2


/*
 * graphical organization
 */


/* **********************************************************************
 *
 * X_create_new_variable_display(): Internal procedure, which is used to
 *                                  create a display of a variable
 *
 * **********************************************************************/

int
X_create_new_variable_display(char *name, 
			      int dim_x, 
			      int dim_y, 
			      float *value)
{
  float size_y, size_x, offs_x;
  static float current_x = X_MIN_X;
  static float current_y = X_MAX_Y;
  float window_pos[4];
  int   window_font = 1;
  int   window_colors[] = {NO_COLOR, C_GREY40, NO_COLOR}; 
  int   window_fonts[]           = {1};
  int   window_background_color[]= {NO_COLOR};
  int   window_frame_color[]     = {NO_COLOR};
  int   window_text_color[]      = {C_WHITE};
  char *switch_texts[]           = {"??? unkown ?????"};
  int   ret_value;
  int   help;


  if (((float) X_WIDTH_X) / ((float) dim_x) > X_MAX_FIELD_WIDTH)
    size_x = ((float) dim_x) * X_MAX_FIELD_WIDTH;
  else
    size_x = X_WIDTH_X;
  offs_x = 0.5 * (X_WIDTH_X - size_x);
  
  size_y = size_x / ((float) dim_x) * ((float) dim_y);
  if (name)
    size_y += X_NAME_Y;

  if (current_y - size_y < X_MIN_Y){
    current_y = X_MAX_Y;
    current_x = current_x + X_WIDTH_X + X_SEPARATOR;
  }
  graphics_max_x = current_x + X_WIDTH_X;
  
  if (name){
    window_pos[0] = current_x;
    window_pos[1] = current_x + X_WIDTH_X;
    window_pos[2] = current_y - X_NAME_Y;
    window_pos[3] = current_y;
    
    help = G_create_switch_object(window_pos, 1, switch_texts, 
				  window_background_color, window_frame_color, 
				  window_text_color, window_fonts);
    G_set_new_text(help, name, 0);
  }


  window_pos[0] = current_x + offs_x;
  window_pos[1] = current_x + offs_x + size_x;
  window_pos[2] = current_y - size_y;
  if (name)
    window_pos[3] = current_y - X_NAME_Y;
  else
    window_pos[3] = current_y;    

  ret_value =
    G_create_matrix_object(window_pos, "???", value, NULL, dim_x, dim_y,
			   0.0, 1.0, window_colors, window_font);
  
  current_y =  current_y - size_y - X_SEPARATOR;

  return ret_value;
}





/* **********************************************************************
 *
 * X_display_variable() - displays variable (-1 = all)
 *
 * **********************************************************************/


void
X_display_variable(int variable_id) /* -1 means: all of them */
{
  int i, j;
  float max;
  int from, to;

  if (!global_graphics_initialized || !X_display)
    return;
  

  /*
   * sanity checks
   */
  
  if (variable_id < -1 || variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    variable_id);
    return;
  }
  
  /*
   * do it
   */
  
  if (variable_id == -1){	/* all */
    from = 0;
    to   = X_number_of_variables;
  }
  else{
    from = variable_id;
    to   = variable_id + 1;
  }
  
  for (i = from; i < to; i++){
    if (X_variables[i].display != -1){
      
      if (X_variables[i].type == STOCHASTIC){
	max = 0.0;
	for (j = 0; j < X_variables[i].num_values; j++)
	  if (max < X_variables[i].value[j])
	    max = X_variables[i].value[j];
	G_matrix_set_display_range(X_variables[i].display, 0.0, max);
	if (max > 1.0)
	  fprintf(stderr, "ERROR A in X_display_variable: %g\n", max);
      }
      G_display_matrix(X_variables[i].display);
    }

    
    if (X_variables[i].avg_display != -1){
      if (X_variables[i].type == STOCHASTIC){
	max = 0.0;
	for (j = 0; j < X_variables[i].num_values; j++)
	  if (max < X_variables[i].avg_value[j])
	    max = X_variables[i].avg_value[j];
	G_matrix_set_display_range(X_variables[i].avg_display, 0.0, max);
	if (max > 1.0)
	  fprintf(stderr, "ERROR B in X_display_variable: %g\n", max);
      }
      
      G_display_matrix(X_variables[i].avg_display);
    }
    
    
    if (X_variables[i].norm_display != -1){
      if (X_variables[i].type == STOCHASTIC){
	max = 0.0;
	for (j = 0; j < X_variables[i].num_values; j++)
	  if (max < fabs(X_variables[i].norm_value[j]))
	    max = fabs(X_variables[i].norm_value[j]);
	G_matrix_set_display_range(X_variables[i].norm_display, -max, max);
      }
      
      G_display_matrix(X_variables[i].norm_display);
    }
  }
}

/************************************************************************\
 ************************************************************************
\************************************************************************/











/* **********************************************************************
 *
 * X_testing(): internal routine for testing during development
 *
 * **********************************************************************/

void
X_testing(int verbose)
{
  int i, j, n, m;
  int net_id, param_nr;
  float e1, e2, grad;
  float sum = 0.0;
  float sum2 = 0.0;
  int sign_wrong = 0;
  int found = 0;
  float quot;
  /*int verbose = 0;*/
  
  if (!global_prob_c)
    return;

  G_display_switch(TEST_BUTTON, 1);
  X_display = 0;
  X_training_phase      = 1;
  X_calculate_gradients = 1;
  for (net_id = 0; net_id < X_number_of_networks; net_id++)
    for (param_nr = 0; param_nr < X_networks[net_id].num_params; param_nr++){

      sum = 0.0;
      sum2 = 0.0;
      sign_wrong = 0;

      /*
       * run the test
       */
      
      for (m = 0, n = 0; n < NUM_TESTS && m < 2 * NUM_TESTS; m++){
	
	/*
	 * Initialize networks randomly
	 */
	for (i = 0; i < X_number_of_networks; i++){
	  /*X_network_set_weights_init_range(i, 0.0);*/
	  X_init_network_parameters_randomly(i);
	  X_network_set_stepsize(i, 0.0);
	}

	/*
	 * run first experiment
	 */

	for (X_training_phase = 1; X_training_phase <= 2; X_training_phase++){
	  /* fprintf(stderr, "A: phase %d\n", X_training_phase);  */
	  X_initialize_training_epoch();
	  X_initialize_episode();
	  for (i = 0, current_pattern = current_patternset->first;
	       current_pattern;
	       current_pattern = current_pattern->next, i++){
	    X_process_pattern(current_pattern);
	  }
	}

	/*
	 * Memorize gradient, and change the parameter
	 */
    
	grad = X_networks[net_id].d_error_d_params[param_nr];
	X_networks[net_id].params[param_nr] -= (0.5 * TEST_PERTURBATION);

	/*
	 * run second experiment
	 */

	for (X_training_phase = 1; X_training_phase <= 2; X_training_phase++){
	  /* fprintf(stderr, "B: phase %d\n", X_training_phase);  */
	  X_initialize_training_epoch();
	  X_initialize_episode();
	  e1 = 0.0;
	  for (i = 0, current_pattern = current_patternset->first;
	       current_pattern;
	       current_pattern = current_pattern->next, i++){
	    X_process_pattern(current_pattern);
	  }
	  for (j = 0; j < X_number_of_variables; j++)
	    for (i = 0; i < NUM_SET_TYPES; i++)
	      if (X_variables[j].error[i] >= 0.0)
		e1 += X_variables[j].error[i];
	}
	
	/*
	 * Change the parameter again
	 */

	X_networks[net_id].params[param_nr] += TEST_PERTURBATION;

	/*
	 * run third experiment
	 */

	for (X_training_phase = 1; X_training_phase <= 2; X_training_phase++){
	  /* fprintf(stderr, "C: phase %d\n", X_training_phase);  */
	  X_initialize_training_epoch();
	  e2 = 0.0;
	  X_initialize_episode();
	  for (i = 0, current_pattern = current_patternset->first;
	       current_pattern;
	       current_pattern = current_pattern->next, i++){
	    X_process_pattern(current_pattern);
	  }
	  for (j = 0; j < X_number_of_variables; j++)
	    for (i = 0; i < NUM_SET_TYPES; i++)
	      if (X_variables[j].error[i] >= 0.0)
		e2 += X_variables[j].error[i];
	}
	/*
	 * print + count the result
	 */

	if (fabs(e1-e2) >= MIN_CHANGE && fabs(grad) >= MIN_CHANGE
	    && grad != 0.0){
	  quot = ((e1 - e2) / TEST_PERTURBATION / grad);
	  if ((e1-e2) * grad < 0.0)
	    sign_wrong++;
	  if (verbose){
	    fprintf(stderr, "e1=%8.6g e2=%8.6g grad=%8.6g -> %8.6g ",
		    e1, e2, grad, quot);
	    if ((e1-e2) * grad < 0.0)
	      fprintf(stderr, " SIGN-ERR");
	    fprintf(stderr, "\n");
	  }
	  sum     += quot;
	  sum2    += quot * quot;
	  n++;
	}
      }
      if (n > 0){
	sum  /= ((float) n); /* average */
	sum2 /= ((float) n);
	sum2 = sqrt(sum2 - (sum * sum)); /* approx. standard deviation */
	if (sum - sum2 < 1.0 / 1.5 || sum + sum2 > 1.5){
	  fprintf(stderr, "TEST net=%d param=%d (%d): ", net_id, param_nr, n);
	  fprintf(stderr, "Quot: %8.6f +/- %8.6f (sign err=%4.2f%%)\n",
		  sum, sum2,
		  ((float) sign_wrong) / ((float) n) * 100.0);
	  found = 1;
	}
	else
	  fprintf(stderr, "TEST net=%d param=%d (%d): okay.\n",
		  net_id, param_nr, n);
      }
      else{
	fprintf(stderr, "TEST net=%d param=%d (%d): ---no results---\n", 
		net_id, param_nr, n);
	found = 1;
      }
    }

  if (!found)
    fprintf(stderr, "TEST: Passed. Nothing suspicious found.\n");

  X_training_phase      = 0;
  X_calculate_gradients = 0;
  G_display_switch(TEST_BUTTON, 0);
  return;
}


/************************************************************************\
 ************************************************************************
\************************************************************************/








/* **********************************************************************
 *
 * X_logging_on()
 * 
 *
 * **********************************************************************/

void
X_logging_on()
{
  if ((log_iop = fopen(LOG_FILENAME, "w")) == 0){
    fprintf(stderr, "ERROR: Could not open output file %s.\n", LOG_FILENAME);
    G_display_switch(LOGGING_BUTTON, 2);
    usleep(400000);
    G_display_switch(LOGGING_BUTTON, 0);
    global_modus_logging = 0;
    return;
  }

  G_display_switch(LOGGING_BUTTON, 1);
  global_modus_logging = 1;
  fprintf(stderr, "Log file %s successfully opened.\n", LOG_FILENAME);
}





/* **********************************************************************
 *
 * X_logging_off()
 * 
 *
 * **********************************************************************/

void
X_logging_off()
{
  if (!global_modus_logging || !log_iop)
    return;

  G_display_switch(LOGGING_BUTTON, 0);
  global_modus_logging = 0;
  log_iop = NULL;
  fprintf(stderr, "Log file %s closed.\n", LOG_FILENAME);
}




/* **********************************************************************
 *
 * X_log_variables()
 * 
 *
 * **********************************************************************/

void
X_log_variables()
{
  int variable_id, i;
  
  if (!global_modus_logging || !log_iop)
    return;

  fprintf(log_iop, "\n\n");
  for (variable_id = 0; variable_id < X_number_of_variables; variable_id++)
    if (X_variables[variable_id].log_flag == 1){
      fprintf(log_iop, "%s:  {", X_variables[variable_id].name);
      for (i = 0; i < X_variables[variable_id].num_values; i++){
	if (i > 0)
	  fprintf(log_iop, ", ");
	fprintf(log_iop, "%f", X_variables[variable_id].value[i]);
      }
      fprintf(log_iop, "}\n\n");
    }
    else if (X_variables[variable_id].log_flag == 2){
      fprintf(log_iop, "%s [values]:  {", X_variables[variable_id].name);
      for (i = 0; i < X_variables[variable_id].num_values; i++){
	if (i > 0)
	  fprintf(log_iop, ", ");
	fprintf(log_iop, "%f", X_variables[variable_id].value[i]);
      }
      fprintf(log_iop, "}\n");
      fprintf(log_iop, "%s [avg]:  {", X_variables[variable_id].name);
      for (i = 0; i < X_variables[variable_id].num_values; i++){
	if (i > 0)
	  fprintf(log_iop, ", ");
	fprintf(log_iop, "%f", X_variables[variable_id].avg_value[i]);
      }
      fprintf(log_iop, "}\n");
      fprintf(log_iop, "%s [norm]:  {", X_variables[variable_id].name);
      for (i = 0; i < X_variables[variable_id].num_values; i++){
	if (i > 0)
	  fprintf(log_iop, ", ");
	fprintf(log_iop, "%f", X_variables[variable_id].norm_value[i]);
      }
      fprintf(log_iop, "}\n\n");
    }
}


