

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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/learn/appl.c,v $
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
 * $Log: appl.c,v $
 * Revision 1.1  2002/09/14 16:36:05  rstone
 * *** empty log message ***
 *
 * Revision 1.58  1999/07/11 18:48:31  thrun
 * slight reorganization
 *
 * Revision 1.57  1998/07/08 20:50:58  thrun
 * very cheesy collision avoidance method (purely reactive)
 *
 * Revision 1.56  1998/07/06 14:29:59  thrun
 * minor changes.
 *
 * Revision 1.55  1998/07/04 15:21:36  thrun
 * variable logging.
 *
 * Revision 1.54  1998/07/03 23:59:58  thrun
 * .
 *
 * Revision 1.53  1998/06/28 17:48:16  thrun
 * First successful gesture-driven run. It works!
 *
 * Revision 1.52  1998/06/28 16:29:01  thrun
 * .
 *
 * Revision 1.51  1998/06/28 16:25:13  thrun
 * more tuning.
 *
 * Revision 1.50  1998/06/27 22:13:42  thrun
 * more tuneups
 *
 * Revision 1.49  1998/06/27 19:13:54  thrun
 * some tuning + training.
 *
 * Revision 1.48  1998/06/27 15:39:21  thrun
 * minor (new stepsize)
 *
 * Revision 1.47  1998/06/27 15:21:48  thrun
 * Fixed an inconsistency with they-dimension.
 *
 * Revision 1.46  1998/06/27 01:13:57  thrun
 * .
 *
 * Revision 1.45  1998/06/26 13:36:43  thrun
 * fixed y computation
 *
 * Revision 1.44  1998/06/26 01:06:11  thrun
 * .
 *
 * Revision 1.43  1998/06/24 12:50:05  thrun
 * initial gesture interface.
 *
 * Revision 1.42  1998/06/23 03:33:18  thrun
 * Nice prototpe of the mail-delivery robot. What's missing is:
 * gesture recognition and the real robot tests.
 *
 * Revision 1.41  1998/06/22 04:48:44  thrun
 * nice intermediate version: can send robot to a point now, but controller
 * doesn't dampen the robot's trajectory yet.
 *
 * Revision 1.40  1998/06/20 21:05:30  thrun
 * Reorganization (created: fa.{c,h}, xtypes.h)
 *
 * Revision 1.39  1998/05/27 21:27:50  thrun
 * intermediate version, don't use
 *
 * Revision 1.38  1998/05/27 04:47:04  thrun
 * Lots of stuff (don't exactly remember). Probably in the applications,
 * a new mode where the robot learns control. Improved Gaussian convolution.
 *
 * Revision 1.37  1998/05/05 04:00:32  thrun
 * Direct control mode (and button). Other stuff I don't fully remember.
 * Intermediate version.
 *
 * Revision 1.36  1998/04/28 23:30:45  thrun
 * autosave of weights, we can now switch off learning in selected
 * networks and variables, which is important for incremental
 * programming/learning
 *
 * Revision 1.35  1998/04/26 15:03:08  thrun
 * subroutines appear to work fine. also changed the initialiation.
 *
 * Revision 1.34  1998/04/24 04:49:04  thrun
 * intermediate and buggy version - subroutines don't have their gradient
 * computation quite right yet.
 *
 * Revision 1.33  1998/04/20 03:12:11  thrun
 * Intermediate version with subroutines.
 *
 * Revision 1.32  1998/04/18 22:55:41  thrun
 * nice intermediate version, no apparent bugs
 *
 * Revision 1.31  1998/04/18 20:42:24  thrun
 * 1. neuro net can now be called with the same variable as input
 * and output
 * 2. direct control modus for the robot. controls are
 * now generated exclusively through the display (left upper corner)
 *
 * Revision 1.30  1998/04/16 02:42:50  thrun
 * copy function for random variables - use with caution!
 *
 * Revision 1.29  1998/04/15 01:58:10  thrun
 * intermediate version
 *
 * Revision 1.28  1998/04/14 02:35:54  thrun
 * .
 *
 * Revision 1.27  1998/04/13 01:45:17  thrun
 * Thermomter encoding for sigmodial neural networks. Delay for
 * pausing (when recording data).
 *
 * Revision 1.26  1998/01/25 01:09:42  thrun
 * deactivated my stuff.
 *
 * Revision 1.25  1997/10/23 02:28:20  thrun
 * .
 *
 * Revision 1.24  1997/10/05 18:11:16  thrun
 * new data library "libdat.a"
 *
 * Revision 1.23  1997/08/06 13:56:09  thrun
 * intermediate version (fixed a bug with the name field in variables/networks)
 *
 * Revision 1.22  1997/08/05 22:19:08  thrun
 * intermediate version - do not use (networks are now not linked
 * to variables any longer)
 *
 * Revision 1.21  1997/08/05 04:15:21  thrun
 * (1) avg_value are saved for each stoch variable, (2) enhanced replay
 * functions.
 *
 * Revision 1.20  1997/07/26 14:19:04  thrun
 * changed display options
 *
 * Revision 1.19  1997/07/16 21:43:22  thrun
 * .
 *
 * Revision 1.18  1997/07/16 17:09:43  thrun
 * Neat, working version. Simpliefied the application, too.
 *
 * Revision 1.17  1997/07/15 17:34:05  thrun
 * *** empty log message ***
 *
 * Revision 1.16  1997/07/14 22:17:13  thrun
 * Fixed the bug (I believe). Now comes testing.
 *
 * Revision 1.15  1997/07/14 14:13:17  thrun
 * First attempt to implement multi-phasing. (broken code, don't use).
 *
 * Revision 1.14  1997/07/07 13:53:10  thrun
 * Small change in the application
 *
 * Revision 1.13  1997/07/06 22:51:48  thrun
 * Autosaving of the parameters (every 2 minutes)
 *
 * Revision 1.12  1997/07/06 22:23:14  thrun
 * .
 *
 * Revision 1.11  1997/07/06 20:48:47  thrun
 * .
 *
 * Revision 1.10  1997/07/06 18:42:03  thrun
 * Radial basis networks implemented.
 *
 * Revision 1.9  1997/07/05 17:20:27  thrun
 * Temporal integration of stochastic variables works fine now.
 *
 * Revision 1.8  1997/07/04 20:57:28  thrun
 * Running version of: determinstic repr -> network -> stochastich repr ->
 *         network -> determinstic repr (chaining), tested with XOR.
 *
 * Revision 1.7  1997/07/04 19:26:53  thrun
 * yet another intermediate version
 *
 * Revision 1.6  1997/07/04 18:09:32  thrun
 * intermediate version - do not use
 *
 * Revision 1.5  1997/07/04 00:28:54  thrun
 * Intermediate verson of the learning stuff. What works is:
 * - training of deterministic variables (incl. chaining)
 * - training of multiple determinstic with a stochastic variables 
 *   (incl chaining)
 * What doesn't yet work includes
 * - temporal chaining of stochastic variables, using Bayes rule
 * - convolving: gradients
 * - stochatic input
 *
 * Revision 1.4  1997/06/30 05:53:57  thrun
 * intermediate version - new neural network (which seems to work!).
 *
 * Revision 1.3  1997/06/29 04:04:54  thrun
 * intrmediate version - not really good for anything
 *
 * Revision 1.2  1997/06/28 13:41:40  thrun
 * This is the fully functional recorder/display/analysis tool.
 * Check out this version if you'd like to have it (without any
 * of the learning stuff). What's missing is the ability to feed back
 * the data intp the baseServer, colli and camera. Also missing is a link
 * to the pantilt process.
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

#include "bUtils.h"
#include "EZX11.h"
#include "o-graphics.h"
#include "dat.h"
#include "global.h"
#include "mem.h"
#include "xtypes.h"
#include "vars.h"
#include "fa.h"



/*
 * pick exactly one from the following options
 *
 */

#define WALL
/* #define PERSON */
/* #define XOR*/




/************************************************************************\
 ************************************************************************
\************************************************************************/

#define RANDOM_MAX ((float)(2147483647))   /* (2**31 - 1) */
#define RAND_PLUS_MINUS_ONE() ((2.0 * ((float)(random()) / ((float) RANDOM_MAX))) - 1.0)
#define RAND_ZERO_TO_ONE() ((float)(random()) / ((float) RANDOM_MAX))



/************************************************************************\
 ************************************************************************
\************************************************************************/

#ifdef WALL


#define NOISE 0


#define COMPUTE_ANGLE    1
#define COMPUTE_Y        1
#define COMPUTE_X        1
#define COMPUTE_COLLI    1
#define COMPUTE_GESTURE  1
#define COMPUTE_CONTROL  1

#define TRAINING_ANGLE   0
#define TRAINING_Y       0
#define TRAINING_X       0
#define TRAINING_COLLI   0
#define TRAINING_GESTURE 0

#define MAX_DIFF_TRANSLATION 30.0
#define RANGE_Y 230.0
#define RANGE_X 3000.0
#define INITIAL_X 0.2
#define INITIAL_Y 0.5

#define CONTINUOUS_ESTIMATION 1

/************************************************************************\
 ************************************************************************
\************************************************************************/

static int actual_action = 0;

typedef struct {
  int   move;
  float delta_x;
  float goal_x;
  float goal_y;
  float timeout;
  int   next_action_complete;
  int   next_action_timeout;
  int   next_action_red_button;
  int   next_action_yellow_button;
  int   next_action_green_button;
  int   next_action_blue_button;
} action_type;

action_type actions[] = {
  /* wait indefinitely for button */
  {0, 0.0, 0.0, 0.0, -1.0,    0,  2,    2,  4,  6,  1},

  /* quit: return to home base */
  {1, 0.2, 0.2, 0.5, -1.0,    0,  0,   -2, -2, -2,  1},

  /* red button: mail for location 1, wait, and return */
  {1, 0.8, 0.4, 0.5, -1.0,   -1, -1,   -2, -2, -2,  1},
  {0, 0.0, 0.0, 0.0,  9.0,    1,  1,    1,  1,  1,  1},


  /* yellow button: mail for location 2, wait, and return */
  {1, 0.8, 0.7, 0.5, -1.0,   -1, -1,   -2, -2, -2,  1},
  {0, 0.0, 0.0, 0.0,  9.0,    1,  1,    1,  1,  1,  1},

  /* green button: mail for location 1 and 2, wait, and return */
  {1, 0.8, 0.4, 0.5, -1.0,   -1, -1,   -2, -2, -2,  1},
  {0, 0.0, 0.0, 0.0,  9.0,   -1, -1,   -1, -1, -1,  1},
  {1, 0.8, 0.7, 0.5, -1.0,   -1, -1,   -2, -2, -2,  1},
  {0, 0.0, 0.0, 0.0,  9.0,    1,  1,    1,  1,  1,  1},
};


/************************************************************************\
 ************************************************************************
\************************************************************************/

/*
 * variable declarations
 */

#define X_NUM_SONAR_VALUES    24
#define X_NUM_ANGLE_VALUES     1
#define X_NUM_CONTROL_VALUES   1

#define X_RESOLUTION_ANGLE   10
#define X_RESOLUTION_SONAR   10
#define X_RESOLUTION_X       20
#define X_RESOLUTION_Y       10
#define X_RESOLUTION_CONTROL 10
#define STEPSIZE 0.1
#define STEPSIZE2 0.001
#define NUM_HIDDEN 5
#define NUM_THERMOMETER_UNITS 1


#define DOWNSAMPLED_IMAGE_SIZE_X 10
#define DOWNSAMPLED_IMAGE_SIZE_Y 10
#define X_NUM_IMAGE_VALUES (3*(DOWNSAMPLED_IMAGE_SIZE_X)*(DOWNSAMPLED_IMAGE_SIZE_Y))

static float prev_pos_x = 0.0;
static float prev_pos_y = 0.0;
static float prev_pos_orientation = 0.0;
static int   prev_pos_defined = 0;
static int   initial_pos_defined = 0;
static float prev_angle = 0.0;
static int   prev_angle_defined = 0;
static int   motion_detected = 0;

int X_sonar = -1;
int X_sonar_angle_estimate = -1;
int X_integrated_angle_estimate = -1;
int X_x_estimate = -1;
int X_integrated_x_estimate = -1;
int X_integrated_x_estimate2 = -1;
int X_integrated_x_estimate3 = -1;
int X_y_estimate = -1;
int X_integrated_y_estimate = -1;
int X_integrated_y_estimate2 = -1;
int X_integrated_y_estimate3 = -1;
int X_target_angle = -1;
int X_target_angle2 = -1;
int X_dist_moved = -1;

int X_sonar_orientation_estimate = -1;
int X_integrated_orientation_estimate = -1;
int X_target_orientation = -1;
int X_target_orientation2 = -1;
int X_coin_flip = -1;

int X_new_sonar = -1;
int X_phase = -1;

int X_colli_input = -1;
int X_colli = -1;
int X_colli_target = -1;

int X_delta_x;
int X_goal_x;
int X_goal_y;
int X_control_trans = -1;
int X_control_trans_avg = -1;
int X_control_trans_actual = -1;
int X_control_rot = -1;
int X_control_rot_avg = -1;
int X_control_rot_actual = -1;


int X_network_sonar__to__angle = -1;
int X_network_newsonar__to__x = -1;
int X_network_newsonar__to__y = -1;
int X_network__to__left_hand = -1;
int X_network__to__right_hand = -1;
int X_network__to__colli = -1;
int X_target_x_estimate = -1;
int X_target_x_estimate2 = -1;
int X_target_y_estimate = -1;
int X_target_y_estimate2 = -1;

int X_image = -1;
int X_left_hand    = -1;
int X_right_hand   = -1;
int X_left_hand_target    = -1;
int X_right_hand_target   = -1;


/*
 * filter declaration
 */

void X_sonar_filter(float time_elapsed);
void X_button_filter(float time_elapsed);
void X_image_filter(float time_elapsed);

/*
 * action model processes
 */

void X_action(float time_elapsed);

/*
 * processes that set targets from markers
 */

void X_sonar_markers(float time_elapsed);
void X_image_markers(float time_elapsed);

/*
 * processes that set targets from control
 */

void X_target_from_control(float time_elapsed);





X_subroutine_type
compute_orientation(int    num_in,
	    int    *in_dims,
	    float  **in,
	    int    num_out,
	    int    *out_dims,
	    float  **out);

X_subroutine_type
extract_sonar_pointed_at_wall(int    num_in,
			      int    *in_dims,
			      float  **in,
			      int    num_out,
			      int    *out_dims,
			      float  **out);
X_subroutine_type
update_x(int    num_in,
	 int    *in_dims,
	 float  **in,
	 int    num_out,
	 int    *out_dims,
	 float  **out);


X_subroutine_type
update_y(int    num_in,
	 int    *in_dims,
	 float  **in,
	 int    num_out,
	 int    *out_dims,
	 float  **out);


X_subroutine_type
compute_control(int    num_in,
		int    *in_dims,
		float  **in,
		int    num_out,
		int    *out_dims,
		float  **out);

void 
next_action(int init, int complete, 
	    int red_button, int yellow_button,
	    int green_button, int blue_button);



/************************************************************************\
 ************************************************************************
\************************************************************************/



/*
 * Initialization: Register all variables, set up the processes
 */


void
X_init()
{

  /*
   * ==================================================
   *
   *   REGISTER VARIABLES
   *
   * ==================================================
   */

  X_phase      = X_register_variable("phase", 1, 0, 1);
  /*
   * register sensor variables
   */

  X_sonar       = X_register_variable("raw sonar", X_NUM_SONAR_VALUES, 1, 0);
	    
  /*
   * register internal variables
   */

  X_sonar_angle_estimate = 
    X_register_stochastic_variable("angle (sonar)",
				   X_NUM_ANGLE_VALUES, 1, 
				   X_RESOLUTION_ANGLE, 
				   (X_DISPLAY_ALL) * (COMPUTE_ANGLE), 2);

  
  X_integrated_angle_estimate =
    X_register_stochastic_variable("angle (integrated)",
				   X_NUM_ANGLE_VALUES, 1, 
				   X_RESOLUTION_ANGLE, 
				   (X_DISPLAY_VALUE) * (COMPUTE_ANGLE), 1);

  

  /*
   * register output variables
   */

  X_target_angle2 =
    X_register_stochastic_variable("target angle", 
				   X_NUM_ANGLE_VALUES, 0,
				   X_RESOLUTION_ANGLE, 
				   (X_DISPLAY_VALUE) * (COMPUTE_ANGLE), 0);
  X_target_angle =
    X_register_variable("target angle D", X_NUM_ANGLE_VALUES, 0, 1);


  /*
   * the orientation
   */

  X_sonar_orientation_estimate = 
    X_register_stochastic_variable("orientation (sonar)",
				   X_NUM_ANGLE_VALUES, 1, 
				   2*(X_RESOLUTION_ANGLE), 
				   (X_DISPLAY_ALL) * (COMPUTE_ANGLE), 2);


  X_integrated_orientation_estimate =
    X_register_stochastic_variable("orientation (integr)",
				   X_NUM_ANGLE_VALUES, 1, 
				   2*(X_RESOLUTION_ANGLE), 
				   (X_DISPLAY_VALUE) * (COMPUTE_ANGLE), 1);
  
  X_target_orientation2 =
    X_register_stochastic_variable("target orientation", 
				   X_NUM_ANGLE_VALUES, 0,
				   2*(X_RESOLUTION_ANGLE), 
				   (X_DISPLAY_VALUE) * (COMPUTE_ANGLE), 0);

  X_target_orientation =
    X_register_variable("target orientation D", X_NUM_ANGLE_VALUES, 0, 1);

  X_coin_flip = X_register_stochastic_variable("coin flip", 1, 0, 2, 0, 0);



  /*
   * sensor readings along the different directions
   */


  X_new_sonar =
    X_register_stochastic_variable("^ north  > south", 2, 0, 
				   X_RESOLUTION_SONAR, 
				   (X_DISPLAY_VALUE) * (COMPUTE_ANGLE), 0);

  /*
   * x estimate
   */

  X_dist_moved = X_register_variable("dist moved", 1, 
				     (COMPUTE_X) || (COMPUTE_Y), 0);

  X_x_estimate =
    X_register_stochastic_variable("x (local)",
				   1, 0, X_RESOLUTION_X, 
				   (X_DISPLAY_ALL) * (COMPUTE_X), 2);

  X_integrated_x_estimate = 
    X_register_stochastic_variable("x (integrated)",
				   1, 0, X_RESOLUTION_X, 
				   (X_DISPLAY_VALUE) * (COMPUTE_X), 1);

  X_integrated_x_estimate2 = 
    X_register_stochastic_variable("x (integrated) 2",
				   1, 0, X_RESOLUTION_X, 
				   (X_DISPLAY_VALUE) * (COMPUTE_X), 0);

  X_integrated_x_estimate3 = X_register_variable("x (integrated) 3", 1, 0, 0);

  X_target_x_estimate2 = 
    X_register_stochastic_variable("target x", 
				   1, 0, X_RESOLUTION_X, 
				   (X_DISPLAY_VALUE) * (COMPUTE_X), 0);

  X_target_x_estimate = X_register_variable("target x (D)", 1, 0, 1);


  /*
   * y estimate
   */

  X_y_estimate =
    X_register_stochastic_variable("y (local)",
				   1, 0, X_RESOLUTION_Y, 
				   (X_DISPLAY_ALL) * (COMPUTE_Y), 2);
  

  X_integrated_y_estimate = 
    X_register_stochastic_variable("y (integrated)",
				   1, 0, X_RESOLUTION_Y, 
				   (X_DISPLAY_VALUE) * (COMPUTE_Y), 1);

  X_integrated_y_estimate2 = 
    X_register_stochastic_variable("y (integrated) 2",
				   1, 0, X_RESOLUTION_Y, 
				   (X_DISPLAY_VALUE) * (COMPUTE_Y), 0);

  X_integrated_y_estimate3 = X_register_variable("y (integrated) 3", 1, 0, 0);

  X_target_y_estimate2 = 
    X_register_stochastic_variable("target y", 
				   1, 0, X_RESOLUTION_Y, 
				   (X_DISPLAY_VALUE) * (COMPUTE_Y), 0);

  X_target_y_estimate = X_register_variable("target y (D)", 1, 0, 1);




  /*
   * gesture interface
   */

  X_left_hand  = 
    /* X_register_variable("left hand", 1, COMPUTE_GESTURE, 1); */
    X_register_stochastic_variable("left hand", 
				   1, 0, 2, COMPUTE_GESTURE, 1);


  X_left_hand_target  = 
    X_register_variable("target left hand", 1, COMPUTE_GESTURE, 1);

  X_right_hand = 
    /* X_register_variable("right hand", 1, COMPUTE_GESTURE, 1); */
    X_register_stochastic_variable("right hand", 
				   1, 0, 2, COMPUTE_GESTURE, 1);


  X_right_hand_target = 
    X_register_variable("target right hand", 1, COMPUTE_GESTURE, 1);


  X_image = X_register_variable("downsampled image", 
				X_NUM_IMAGE_VALUES, COMPUTE_GESTURE, 0);


  /*
   * colli
   */

  X_colli_input = X_register_variable("colli input", 2, COMPUTE_COLLI, 0);

  X_colli = X_register_stochastic_variable("colli", 
     1, 0, 2, COMPUTE_COLLI, 1); 
  
  /* X_colli = X_register_variable("colli", 1, COMPUTE_COLLI, 1);*/
  
  X_colli_target = X_register_variable("colli target", 1, COMPUTE_COLLI, 1);

  /*
   * control
   */


  if (COMPUTE_CONTROL){

    X_delta_x = X_register_variable("delta x (D)", 1, 0, 0);
    X_goal_x = X_register_variable("goal x (D)", 1, 0, 0);
    X_goal_y = X_register_variable("goal y (D)", 1, 0, 0);
    
    X_control_trans = 
      X_register_stochastic_variable("control: trans", X_NUM_CONTROL_VALUES, 
				     0, X_RESOLUTION_CONTROL,
				     (X_DISPLAY_VALUE), 1);

    X_control_trans_avg = X_register_variable("control: trans*", 1, 1, 0);
    X_control_trans_actual = X_register_variable("translation", 1, 1, 1);

    X_control_rot = 
      X_register_stochastic_variable("control: rot", X_NUM_CONTROL_VALUES, 
				     0, X_RESOLUTION_CONTROL,
				     (X_DISPLAY_VALUE), 1);

    X_control_rot_avg = X_register_variable("control: rot*", 1, 1, 0);
    X_control_rot_actual = X_register_variable("rotation", 1, 1, 1);
  }

  /*
   * ==================================================
   *
   *   REGISTER NEURAL NETWORKS
   *
   * ==================================================
   */

  X_network_sonar__to__angle =
    X_register_neural_network("sonar to angle",
			      DETERMINISTIC, X_NUM_SONAR_VALUES,
			      STOCHASTIC, X_NUM_ANGLE_VALUES,
			      NUM_THERMOMETER_UNITS, NUM_HIDDEN);
  X_network_set_stepsize(X_network_sonar__to__angle, STEPSIZE);

  X_network_newsonar__to__x =
    X_register_neural_network("newsonar to x",
			      STOCHASTIC, 2,
			      STOCHASTIC, 1,
			      NUM_THERMOMETER_UNITS, NUM_HIDDEN);
  X_network_set_stepsize(X_network_newsonar__to__x, STEPSIZE);

  X_network_newsonar__to__y =
    X_register_neural_network("newsonar to y",
			      STOCHASTIC, 2,
			      STOCHASTIC, 1,
			      NUM_THERMOMETER_UNITS, NUM_HIDDEN);
  X_network_set_stepsize(X_network_newsonar__to__y, STEPSIZE);




  X_network__to__left_hand =
    X_register_neural_network("image to left hand",
			      DETERMINISTIC, X_NUM_IMAGE_VALUES,
			      STOCHASTIC, 1,
			      NUM_THERMOMETER_UNITS, NUM_HIDDEN);
  X_network_set_stepsize(X_network__to__left_hand, STEPSIZE2);

  X_network__to__right_hand =
    X_register_neural_network("image to right hand",
			      DETERMINISTIC, X_NUM_IMAGE_VALUES,
			      STOCHASTIC, 1,
			      NUM_THERMOMETER_UNITS, NUM_HIDDEN);
  X_network_set_stepsize(X_network__to__right_hand, STEPSIZE2);


  X_network__to__colli =
    X_register_neural_network("sonar to colli",
			      DETERMINISTIC, 2,
			      STOCHASTIC, 1,
			      NUM_THERMOMETER_UNITS, NUM_HIDDEN);
  X_network_set_stepsize(X_network__to__colli, STEPSIZE2);




  /*
    X_network_angle_change =
    X_register_neural_network("action model",
    STOCHASTIC, X_NUM_ANGLE_VALUES,
    STOCHASTIC, X_NUM_ANGLE_VALUES,
    NUM_THERMOMETER_UNITS, NUM_HIDDEN);
    X_network_set_stepsize(X_network_angle_change, STEPSIZE);
    */


  /*
   * ==================================================
   *
   *   REGISTER PROCESSES
   *
   * ==================================================
   */

   
  /*
   * register action processes and their trigger conditions
   */


   X_register_process("action", X_action); 
   X_specify_trigger_condition(X_action, POSITION_EVENT, 1); 

  /*
   * register filter processes and their trigger conditions
   */

  X_register_process("sonar filter", X_sonar_filter);
  X_specify_trigger_condition(X_sonar_filter, SONARS_EVENT, 1);

  X_register_process("button filter", X_button_filter);
  X_specify_trigger_condition(X_button_filter, BUTTONS_EVENT, 1);

  X_register_process("image filter", X_image_filter);
  X_specify_trigger_condition(X_image_filter, IMAGE_EVENT, 1);

  /*
   * register processes that set target values
   */

  X_register_process("sonar markers", X_sonar_markers);
  X_specify_trigger_condition(X_sonar_markers, MARKERS_EVENT, 1);
  X_specify_trigger_condition(X_sonar_markers, SONARS_EVENT, 1);

  X_register_process("image markers", X_image_markers);
  X_specify_trigger_condition(X_image_markers, MARKERS_EVENT, 1);
  X_specify_trigger_condition(X_image_markers, IMAGE_EVENT, 1);

  X_register_process("control", X_target_from_control);
  X_specify_trigger_condition(X_target_from_control, CONTROL_EVENT, 1);


}


/************************************************************************\
 ************************************************************************
\************************************************************************/


void
X_initialize_variables()
{
  float r_min = 0.0;
  float r_max = 0.499;
  float x = INITIAL_X, y = INITIAL_Y;
  float zero = 0.0;

  X_initialize_uniformly(X_coin_flip, NULL, NULL); 
  if (COMPUTE_CONTROL)
    next_action(1, 0,   0, 0, 0, 0);

  if (COMPUTE_ANGLE){
    X_initialize_uniformly(X_integrated_angle_estimate, NULL, NULL); 
    X_initialize_uniformly(X_integrated_orientation_estimate, &r_min, &r_max); 
    X_set_variable(X_integrated_x_estimate, &x);
    X_initialize_uniformly(X_integrated_y_estimate, NULL, NULL);
    prev_pos_defined    = 0;
    initial_pos_defined = 0;
    prev_angle_defined  = 0;
  }
  motion_detected = 0;

  if (COMPUTE_GESTURE){
    X_set_variable(X_left_hand_target,  &zero);
    X_set_variable(X_right_hand_target, &zero);
  }
}

/************************************************************************\
 ************************************************************************
\************************************************************************/

/*
 * ----------------------------------------------------------------------
 *
 * The following routine pre-processes the sensor data and
 * copies the result into the corresponding X-variable 
 */




void
X_sonar_filter(float time_elapsed)
{
  int i;
  float value1[X_NUM_SONAR_VALUES], *value2;
  int   subroutine_input[10], subroutine_output[10];
  float diff_orientation = 0.0;
  float variance = 0.1;
  float trans, rot;
  float x, y;
  float colli_flag = 1.0;


  for (i = 0; i < X_NUM_SONAR_VALUES; i++){
    value1[i] = X_pattern->sonars[i] / 500.0;
    if (value1[i] > 1.0) value1[i] = 1.0;
  }
  value2 = &(value1[11]);

  /*
   * copy sensor values into the internal variables
   */
  

  X_set_variable(X_sonar, value1);


  /*
   * predict angle to wall
   */

  if (COMPUTE_ANGLE){

    X_neuro_net(X_network_sonar__to__angle, X_sonar, X_sonar_angle_estimate); 
    if (CONTINUOUS_ESTIMATION || motion_detected > 0)
      X_integrate_variables(X_integrated_angle_estimate, 
			    X_sonar_angle_estimate);
    
    /*
     * predict orientation (in 360 degree coordinates)
     */
    
    subroutine_input[0]  = X_sonar_angle_estimate;
    subroutine_input[1]  = X_coin_flip;
    subroutine_output[0] = X_sonar_orientation_estimate;
    X_execute_subroutine(compute_orientation, 2, 1, 
			 subroutine_input, subroutine_output);
    if (CONTINUOUS_ESTIMATION || motion_detected > 0)
      X_integrate_variables(X_integrated_orientation_estimate,
			    X_sonar_orientation_estimate); 
    
  }
  if (COMPUTE_X || COMPUTE_Y){
    /*
     * sonars in geographic directions
     */
    
    subroutine_input[0]  = X_sonar;
    subroutine_input[1]  = X_integrated_orientation_estimate;
    subroutine_output[0] = X_new_sonar;
    X_execute_subroutine(extract_sonar_pointed_at_wall,
			 2, 1, subroutine_input, subroutine_output);
    X_truncate_variable(X_new_sonar, 0.1);
  }
  
  if (COMPUTE_X){
    /*
     * estimate x location
     */

    /*
    X_neuro_net(X_network_newsonar__to__x, X_new_sonar, X_x_estimate); 

    if (CONTINUOUS_ESTIMATION || motion_detected > 0)
      X_integrate_variables(X_integrated_x_estimate, X_x_estimate);  
      */
  }
    
  if (COMPUTE_Y){
    /*
     * estimate y location
     */
    
    /* X_initialize_uniformly(X_integrated_y_estimate, NULL, NULL);/*!*/


    X_neuro_net(X_network_newsonar__to__y, X_new_sonar, X_y_estimate); 

    if (CONTINUOUS_ESTIMATION || motion_detected > 0)
      X_integrate_variables(X_integrated_y_estimate, X_y_estimate);  

  }
  
  if (COMPUTE_COLLI){
    static int xx = 0;
    /*
     * estimate colli flag
     */

    X_set_variable(X_colli_input, value2);
    X_neuro_net(X_network__to__colli, X_colli_input, X_colli); 
    colli_flag = X_get_value(X_colli, 0);

    if (!xx){
      fprintf(stderr, "\n\n\t\t### WARNING: checp collision avoidance.\n");
      xx = 1;
    }
    
    if (value2[0] < 0.1 || value2[1] < 0.1)
      colli_flag = 0.0;
    else
       colli_flag = 1.0;
  }
  
  if (COMPUTE_CONTROL){
    X_weighted_average(X_integrated_x_estimate, X_integrated_x_estimate3);
    X_weighted_average(X_integrated_y_estimate, X_integrated_y_estimate3);
    x = X_get_value(X_integrated_x_estimate3, 0);
    y = X_get_value(X_integrated_y_estimate3, 0);
    if (actions[actual_action].move &&
	((actions[actual_action].delta_x > 0.5 && 
	  x > actions[actual_action].goal_x) ||
	(actions[actual_action].delta_x < 0.5 && 
	 x < actions[actual_action].goal_x))){
      next_action(0, 1,   0, 0, 0, 0);
    }
    else
      next_action(0, 0,   0, 0, 0, 0);


    X_set_variable(X_delta_x, &(actions[actual_action].delta_x));
    X_set_variable(X_goal_y,  &(actions[actual_action].goal_y));


    subroutine_input[0]  = X_integrated_orientation_estimate;
    subroutine_input[1]  = X_integrated_x_estimate; 
    subroutine_input[2]  = X_integrated_y_estimate;
    subroutine_input[3]  = X_delta_x;
    subroutine_input[4]  = X_goal_y;
    subroutine_output[0] = X_control_trans;
    subroutine_output[1] = X_control_rot;
    

    X_execute_subroutine(compute_control,
			 5, 2, subroutine_input, subroutine_output);



    X_weighted_average(X_control_trans, X_control_trans_avg);
    X_weighted_average(X_control_rot,   X_control_rot_avg);


    if (actions[actual_action].move){
      trans = X_get_value(X_control_trans_avg, 0)
	- (0.5 / X_RESOLUTION_CONTROL);
      rot   = X_get_value(X_control_rot_avg, 0);
      trans = trans * 1500.0;
      rot   = (rot - 0.5) * 280.0;
      if (colli_flag < 0.5)
	trans = 0.0;
      X_copy_variable(X_control_trans_avg, X_control_trans_actual);
      X_copy_variable(X_control_rot_avg, X_control_rot_actual);
    }
    else{
      trans = 0.0;
      rot   = 0.0;
      X_set_variable(X_control_trans_actual, &trans);
      X_set_variable(X_control_rot_actual, &rot);
    }
    set_velocity(trans, rot, 0);
      

    /*
      fprintf(stderr, "Control: trans=");
      X_print_value(X_control_trans_avg, 0, stderr);
      fprintf(stderr, " rot=");
      X_print_value(X_control_rot_avg, 0, stderr);
      fprintf(stderr, "\n");
      */
  }
}


/************************************************************************\
 ************************************************************************
\************************************************************************/

/*
 * ----------------------------------------------------------------------
 *
 * The following routine pre-processes the button data 
 */




void
X_button_filter(float time_elapsed)
{
  if (X_pattern->buttons->pushed[0] &&
      actions[actual_action].next_action_red_button != -2){
    fprintf(stderr, "Pressed red button.\n");
    next_action(0, 0,   1, 0, 0, 0);
  }
  else if (X_pattern->buttons->pushed[1] &&
	   actions[actual_action].next_action_yellow_button != -2){
    fprintf(stderr, "Pressed yellow button.\n");
    next_action(0, 0,   0, 1, 0, 0);
  }
  else if (X_pattern->buttons->pushed[2] &&
	   actions[actual_action].next_action_green_button != -2){
    fprintf(stderr, "Pressed green button.\n");
    next_action(0, 0,   0, 0, 1, 0);
  }
  else if (X_pattern->buttons->pushed[3] &&
	   actions[actual_action].next_action_blue_button != -2){
    fprintf(stderr, "Pressed blue button.\n");
    next_action(0, 0,   0, 0, 0, 1);
  }
}



void 
next_action(int init, int complete, 
	    int red_button, int yellow_button,
	    int green_button, int blue_button)
{
  struct timeval time;
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference, last_timeout; 
  float help;


  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
  if (!complete && !init && 
      !red_button && !yellow_button && !green_button && !blue_button &&
      (actions[actual_action].timeout < 0.0 ||
       time_difference < actions[actual_action].timeout))
    return;


  last_timeout = actions[actual_action].timeout;
  if (init){
    actual_action = 0;
    fprintf(stderr, "*** action=%d (init) ***\n", actual_action);
  }
  else if (complete){
    if (actions[actual_action].next_action_complete >= 0)
      actual_action = actions[actual_action].next_action_complete;
    else if (actions[actual_action].next_action_complete == -1)
      actual_action++;
    fprintf(stderr, "*** action=%d (complete) ***\n", actual_action);
  }
  else if (red_button){
    if (actions[actual_action].next_action_red_button >= 0)
      actual_action = actions[actual_action].next_action_red_button;
    else if (actions[actual_action].next_action_red_button == -1)
      actual_action++;
    fprintf(stderr, "*** action=%d (red_button) ***\n", actual_action);
  }
  else if (yellow_button){
    if (actions[actual_action].next_action_yellow_button >= 0)
      actual_action = actions[actual_action].next_action_yellow_button;
    else if (actions[actual_action].next_action_yellow_button == -1)
      actual_action++;
    fprintf(stderr, "*** action=%d (yellow_button) ***\n", actual_action);
  }
  else if (green_button){
    if (actions[actual_action].next_action_green_button >= 0)
      actual_action = actions[actual_action].next_action_green_button;
    else if (actions[actual_action].next_action_green_button == -1)
      actual_action++;
    fprintf(stderr, "*** action=%d (green_button) ***\n", actual_action);
  }
  else if (blue_button){
    if (actions[actual_action].next_action_blue_button >= 0)
      actual_action = actions[actual_action].next_action_blue_button;
    else if (actions[actual_action].next_action_blue_button == -1)
      actual_action++;
    fprintf(stderr, "*** action=%d (blue_button) ***\n", actual_action);
  }
  else{
    if (actions[actual_action].next_action_timeout >= 0)
      actual_action = actions[actual_action].next_action_timeout;
    else if (actions[actual_action].next_action_timeout == -1)
      actual_action++;
    fprintf(stderr, "*** action=%d (timeout: %g %g) ***\n", actual_action,
	    last_timeout, time_difference);
  }
 
  last_time.tv_sec  = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;
  help = ((float) actual_action) * 0.01;
  X_set_variable(X_phase, &help);
}



/************************************************************************\
 ************************************************************************
\************************************************************************/

/*
 * ----------------------------------------------------------------------
 *
 * The following routine pre-processes images
 */


void
X_image_filter(float time_elapsed)
{
  int x, y, i, j, k, index1, index2;
  float image[3*(DOWNSAMPLED_IMAGE_SIZE_X)*(DOWNSAMPLED_IMAGE_SIZE_Y)];
  float divisor[3*(DOWNSAMPLED_IMAGE_SIZE_X)*(DOWNSAMPLED_IMAGE_SIZE_Y)];
  float gesture[2];

  if (COMPUTE_GESTURE){

    
    for (i = 0; i < 3 * (DOWNSAMPLED_IMAGE_SIZE_X) 
	   * (DOWNSAMPLED_IMAGE_SIZE_Y); i++){
      image[i] = 0.0;
      divisor[i] = 0.0;
    }      
    
    
    
    for (y = 0; y < IMAGE_SIZE_Y; y++)
      for (x = 0; x < IMAGE_SIZE_X; x++)
	for (k = 0; k < 3; k++){
	  i = (int) (((float) x) / ((float) IMAGE_SIZE_X) 
		     * ((float) (DOWNSAMPLED_IMAGE_SIZE_X)));
	  j = (int) (((float) y) / ((float) IMAGE_SIZE_Y) 
		     * ((float) (DOWNSAMPLED_IMAGE_SIZE_Y)));
	  index1 = (((y * IMAGE_SIZE_X) + x) * 3) + k;
	  index2 = (((j * (DOWNSAMPLED_IMAGE_SIZE_X)) + i) * 3) + k;
	  image[index2] += ((float) X_pattern->image[index1]) / 256.0;
	  divisor[index2] += 1.0;
	}

    for (i = 0; i < 3 * (DOWNSAMPLED_IMAGE_SIZE_X) 
	   * (DOWNSAMPLED_IMAGE_SIZE_Y); i++)
      image[i] = image[i] / divisor[i];
  

    if (0){			/* just for verification */
      for (y = 0; y < IMAGE_SIZE_Y; y++)
	for (x = 0; x < IMAGE_SIZE_X; x++)
	  for (k = 0; k < 3; k++){
	    i = (int) (((float) x) / ((float) IMAGE_SIZE_X) 
		       * ((float) (DOWNSAMPLED_IMAGE_SIZE_X)));
	    j = (int) (((float) y) / ((float) IMAGE_SIZE_Y) 
		       * ((float) (DOWNSAMPLED_IMAGE_SIZE_Y)));
	    index1 = (((y * IMAGE_SIZE_X) + x) * 3) + k;
	    index2 = (((j * (DOWNSAMPLED_IMAGE_SIZE_X)) + i) * 3) + k;
	    X_pattern->image[index1] = (unsigned char) (image[index2] * 256.0);
	  }  
      mem_display_image(X_pattern->image, NULL, NULL, NULL);
      usleep(500000);
    }

    /*
     * set the image
     */

    X_set_variable(X_image, image);

    /*
     * run neural networks
     */

    X_neuro_net(X_network__to__left_hand,  X_image, X_left_hand); 
    X_neuro_net(X_network__to__right_hand, X_image, X_right_hand); 

    /*
     * and initiate the appropriate behavior
     */

    gesture[0] = X_get_value(X_left_hand,  1);
    gesture[1] = X_get_value(X_right_hand, 1);
    
    if (1 || !TRAINING_GESTURE) 
      fprintf(stderr, "%g %g  -> ", gesture[0], gesture[1]);
    
    if ((gesture[0] > 0.4 && gesture[1] > 0.3) ||
	(gesture[0] > 0.3 && gesture[1] > 0.4)){
      if (COMPUTE_CONTROL)
	next_action(0, 0,   0, 0, 1, 0);
      if (1 || !TRAINING_GESTURE) fprintf(stderr, "green");
    }
    else if (gesture[0] > 0.4){
      if (COMPUTE_CONTROL)
	next_action(0, 0,   1, 0, 0, 0);
      if (1 || !TRAINING_GESTURE) fprintf(stderr, "red");
    }
    else if (gesture[1] > 0.4){
      if (COMPUTE_CONTROL)
	next_action(0, 0,   0, 1, 0, 0);
      if (1 || !TRAINING_GESTURE) fprintf(stderr, "yellow");
    }
    if (1 || !TRAINING_GESTURE) fprintf(stderr, "\n");
  }
}



void
X_image_markers(float time_elapsed)
{
  int i;
  float zero = 0.0;
  float one  = 1.0;


  if (COMPUTE_GESTURE){
    
    X_set_variable(X_left_hand_target,  &zero);
    X_set_variable(X_right_hand_target, &zero);
    
    if (X_pattern->markers->num_markers > 3)
      fprintf(stderr, "ERROR: Too many markers in image.\n");
    
    for (i = 0; i < X_pattern->markers->num_markers; i++)
      if (X_pattern->markers->y[i] > 0.5){
	if (X_pattern->markers->x[i] < 0.5){
	  X_set_variable(X_right_hand_target, &one);
	  if (1 || !TRAINING_GESTURE) fprintf(stderr, "target: yellow\n");
	}
	else{
	  X_set_variable(X_left_hand_target,  &one);
	  if (1 || !TRAINING_GESTURE) fprintf(stderr, "target: red\n");
	}
      }
    
    
    if (TRAINING_GESTURE){
      X_set_target(X_left_hand,  X_left_hand_target);
      X_set_target(X_right_hand, X_right_hand_target); 
    }
  }
}

/************************************************************************\
 ************************************************************************
\************************************************************************/

/*
 * ----------------------------------------------------------------------
 *
 * routine for generating  360 degree orientation. Notice that
 * subroutines may not possess internal state. All inputs and
 * outputs must be specified through the header
 * 
 */


X_subroutine_type
compute_orientation(int    num_in,
	    int    *in_dims,
	    float  **in,
	    int    num_out,
	    int    *out_dims,
	    float  **out)
{
  float angle = in[0][0];
  float rand  = in[1][0];

  SUBROUTINE_REPORT();

  out[0][0] = angle * 0.5;
  if (rand > 0.5)
    out[0][0] += 0.5;
}

/************************************************************************\
 ************************************************************************
\************************************************************************/

/*
 * ----------------------------------------------------------------------
 *
 * routine for extracting the sonar that point directly towards the
 * side walls
 * 
 */

X_subroutine_type
extract_sonar_pointed_at_wall(int    num_in,
			      int    *in_dims,
			      float  **in,
			      int    num_out,
			      int    *out_dims,
			      float  **out)
{
  int index, index_north, index_west, index_south, index_east, i;
  float threshold = 0.7;

  SUBROUTINE_REPORT();

  index       = (int) (in[1][0] * ((float) X_NUM_SONAR_VALUES));
  index_north = (X_NUM_SONAR_VALUES + 11 - index) % X_NUM_SONAR_VALUES;
  index_west  = (X_NUM_SONAR_VALUES +  5 - index) % X_NUM_SONAR_VALUES;
  index_south = (X_NUM_SONAR_VALUES + 23 - index) % X_NUM_SONAR_VALUES;
  index_east  = (X_NUM_SONAR_VALUES + 17 - index) % X_NUM_SONAR_VALUES;

  
  if (in[0][index_north] <= threshold) 
    out[0][0] = in[0][index_north] / threshold; /* north sonar */
  if (in[0][index_south] <= threshold) 
    out[0][1] = in[0][index_south] / threshold;  /* south sonar */

}

/************************************************************************\
 ************************************************************************
\************************************************************************/

/*
 * ----------------------------------------------------------------------
 *
 * routine for extracting the sonar that point directly towards the
 * side walls
 * 
 */

X_subroutine_type
update_x(int    num_in,
	 int    *in_dims,
	 float  **in,
	 int    num_out,
	 int    *out_dims,
	 float  **out)
{
  float angle, dx, x;
  static int xx = 0;

  SUBROUTINE_REPORT();

  if (!xx){
    fprintf(stderr, "\n\n\t\t### WARNING: contraction in update_x().\n");
    xx = 1;
  }

  angle     = (in[0][0] - 0.25) * 2.0 * M_PI;
  dx        = 
    1.8 * in[1][0] * cos(angle) * ((MAX_DIFF_TRANSLATION) / (RANGE_X));
  x         = in[2][0] + dx;
  if (x > 1.0) x = 1.0;
  if (x < 0.0) x = 0.0;
  out[0][0] = x;
}


X_subroutine_type
update_y(int    num_in,
	 int    *in_dims,
	 float  **in,
	 int    num_out,
	 int    *out_dims,
	 float  **out)
{
  float angle, dy, y;
  static int xx = 0;

  SUBROUTINE_REPORT();

  if (!xx){
    fprintf(stderr, "\n\n\t\t### WARNING: contraction in update_y().\n");
    xx = 1;
  }

  angle     = (in[0][0] - 0.25) * 2.0 * M_PI;
  dy        = 0.3 * in[1][0] * sin(angle) * (MAX_DIFF_TRANSLATION) / (RANGE_Y);
  y         = in[2][0] - dy;
  if (y > 1.0) y = 1.0;
  if (y < 0.0) y = 0.0;
  out[0][0] = y;
}



/************************************************************************\
 ************************************************************************
\************************************************************************/

/*
 * ----------------------------------------------------------------------
 *
 * routine for generating  360 degree orientation. Notice that
 * subroutines may not possess internal state. All inputs and
 * outputs must be specified through the header
 * 
 */


X_subroutine_type
compute_control(int    num_in,
		int    *in_dims,
		float  **in,
		int    num_out,
		int    *out_dims,
		float  **out)
{
  float orientation  = in[0][0];
  float x            = in[1][0];
  float y            = in[2][0];
  float delta_x       = in[3][0];
  float goal_y       = in[4][0];
  
  float target_x, target_y, target_angle, angle_diff, trans, rot;


  SUBROUTINE_REPORT();

  /* orientation: 
     0.25       = east
     0.5        = south
     0.75       = west
     0.0 = 1.0  = north

     turn:
     0.0        = counterclockwise (left)
     1.0        = clockwise (right)
     */

  /*
    subroutine_input[0]  = X_integrated_orientation_estimate;
    subroutine_input[1]  = X_integrated_x_estimate; 
    subroutine_input[2]  = X_integrated_y_estimate;
    subroutine_input[3]  = X_delta_x;
    subroutine_input[4]  = X_goal_y;
    subroutine_output[0] = X_control_trans;
    subroutine_output[1] = X_control_rot;
    */


  target_x = delta_x - 0.5 /* - x*/;
  target_y = y - goal_y;
  target_angle = atan2(target_y, target_x) / 2.0 / M_PI;
  target_angle += 0.25;
  for (; target_angle < 0.0;) target_angle += 1.0;
  for (; target_angle > 1.0;) target_angle -= 0.0;
  angle_diff = fabs(target_angle - orientation);

  if (angle_diff > 0.03)
    trans = 0.0;		/* stop */
  else
    trans = 1.0;		/* move */
  if ((orientation > target_angle && orientation < target_angle + 0.5) ||
      orientation < target_angle - 0.5)
    rot = 0.0;			/* turn counterclockwise */
  else
    rot = 1.0;			/* turn clockwise */

  out[0][0] = trans;
  out[1][0] = rot;
  
  /* fprintf(stderr, "P %6.4f  angle_diff: %6.4f    trans: %6.4f  rot: %6.4f\n",
     X_prob, angle_diff, trans, rot);*/

  return;

}

/************************************************************************\
 ************************************************************************
\************************************************************************/


/*
 * ----------------------------------------------------------------------
 *
 * The following routine pre-processes the control labels and
 * copies the result into the X-variable "X_target_control"
 */

  /* MAX_TRANS_VEL  800.0
     MAX_ROT_VEL    150.0 */
  /* trans vel goes from 0 to 415
     rot   vel goes from -99 to 99 
     ratio: (-1.73164,2.85277) */

void
X_target_from_control(float time_elapsed)
{
  float ratio, prod, trans, rot;
  float value[2];
  int i;


#ifdef xxx

  trans = X_pattern->control->trans_velocity;
  if (trans < 0.0) X_pattern->control->trans_velocity = 0.0;
  rot =  X_pattern->control->rot_velocity;

  if (trans == 0.0 && rot == 0.0)
    return;

  /*
   * compute the control
   */

  value[0] = trans / 300.0;
  value[1] = (rot / 180.0) + 0.5;


  for (i = 0; i < 2; i++){
    if (value[i] > 1.0) value[i] = 1.0;
    if (value[i] < 0.0) value[i] = 0.0;
  }
  
  X_set_variable(X_control_trans_target,  value);
  X_set_variable(X_control_trans_target2, value);

  X_set_variable(X_control_rot_target,  &(value[1]));
  X_set_variable(X_control_rot_target2, &(value[1]));
  
  
  if (TRAINING_CONTROL){
    X_set_target(X_control_trans, X_control_trans_target);
    X_set_target(X_control_rot, X_control_rot_target);
  }
#endif
}

/************************************************************************\
 ************************************************************************
\************************************************************************/


/*
 * ----------------------------------------------------------------------
 *
 * The following routine pre-processes the hand-labeled data and
 * copies the result into the X-variable "X_target_angle"
 */

void
X_sonar_markers(float time_elapsed)
{
  float value;
  float angle;
  float diff;
  float colli_target;
  float aux_float;
  float dist[2];
  int   side[2];
  float x[4];
  float y[4];
  int   i, aux_int;
  float initial_orientation;
  static float initial_x, initial_y, initial_o;
  float dx, dy, d;



  /*
   * sanity check: were the data labeled correctly? 
   */

  if (X_pattern->markers->num_markers != 1 &&
      X_pattern->markers->num_markers != 4){
    fprintf(stderr,
	    "That's strange, I expected 1 or 4 markers instead of %d.\n",
	    X_pattern->markers->num_markers);
    return;
  }


  /*
   * ====================================================================
   * ====================================================================
   * ====================================================================
   * 1 marker -> This is used to indicate the initial orientation,
   * and it's also used to set target values for the collision avoidance
   * ====================================================================
   * ====================================================================
   * ====================================================================
   */

  if (X_pattern->markers->num_markers == 1){
    initial_orientation = atan2(X_pattern->markers->y[0], 
				X_pattern->markers->x[0]) * 0.5 / M_PI + 0.25;
    for (;initial_orientation < 0.0;) initial_orientation += 1.0;
    for (;initial_orientation > 1.0;) initial_orientation -= 1.0;

    X_set_variable(X_integrated_orientation_estimate, &initial_orientation);

    if (TRAINING_COLLI){
      if (X_pattern->markers->x[0] < 0.0)
	colli_target = 0.0;
      else
	colli_target = 1.0;
    }
  }

  /*
   * ====================================================================
   * ====================================================================
   * ====================================================================
   * 4 markers -> This is used to mark the walls
   * ====================================================================
   * ====================================================================
   * ====================================================================
   */

  else{
    
    /*
     * Life is easier with the following local copies
     */
    
    for (i = 0; i < 4; i++){
      x[i] = X_pattern->markers->x[i];
      y[i] = X_pattern->markers->y[i];
    }
    
    for (i = 0; i < 2; i++)	/* sort the points so that x-values are */
      if (x[(2*i)+1] < x[2*i]){	/* increasing for both pairs of points */
	aux_float = x[(2*i)+1];
	x[(2*i)+1] = x[2*i];
	x[2*i] = aux_float;
	aux_float = y[(2*i)+1];
	y[(2*i)+1] = y[2*i];
	y[2*i] = aux_float;
      }
    
    
    /*
     * ====================================================================
     * 1. compute the angles relative to the robot, down to 180 degrees
     * ====================================================================
     */

    if (COMPUTE_ANGLE){
      
      angle = (atan2(y[1]-y[0], x[1]-x[0]) * 180.0 / M_PI) - 90.0;
      for (;angle >= 180.0;) angle -= 180.0;
      for (;angle <    0.0;) angle += 180.0;
      
      /*
       * copy the result into the X variable
       */
      
      value = (angle / 180.0);
      
      X_set_variable(X_target_angle,  &(value));
      X_set_variable(X_target_angle2, &(value));
      
      /*
       * ====================================================================
       * 2. compute the orientation
       * ====================================================================
       */
      
      
      if (prev_angle_defined){
	diff = angle - prev_angle;
	for (;diff >=  90.0;) diff -= 180.0;
	for (;diff <  -90.0;) diff += 180.0;
	angle = prev_angle + diff;
	for (;angle >=  360.0;) angle -= 360.0;
	for (;angle <     0.0;) angle += 360.0;
      }
      
      prev_angle = angle;
      prev_angle_defined = 1;
      
      
      /*
       * copy the result into the X variable
       */
      
      value = (angle / 360.0);
      X_set_variable(X_target_orientation,  &(value));
      X_set_variable(X_target_orientation2, &(value));
    }
    
    
    

    
    /*
     * ====================================================================
     * 3. compute distance to left and right wall
     * ====================================================================
     */
    
    
    if (!initial_pos_defined){
      initial_x = X_pattern->position->x;
      initial_y = X_pattern->position->y;
      initial_o = X_pattern->position->orientation;
      initial_pos_defined = 1;
    }

    if (COMPUTE_X){
      /* value = ((X_pattern->position->x - initial_x) / RANGE_X); */

      dx = X_pattern->position->x - initial_x;
      dy = X_pattern->position->y - initial_y;

      
      value = (dx * cos(initial_o * M_PI / 180.0)) +
	(dy * sin(initial_o * M_PI / 180.0));

      /* value = sqrt((dx * dx) + (dy * dy)); */
      value = value / RANGE_X + INITIAL_X;
      if (value > 1.0) value = 1.0;
      if (value < 0.0) value = 0.0;
      
      X_set_variable(X_target_x_estimate, &value);
      X_set_variable(X_target_x_estimate2, &value);
    }

    if (COMPUTE_Y){
      /* value = ((X_pattern->position->y - initial_y) / RANGE_Y) + 0.5; */
      
      for (i = 0; i < 2; i++){
	dx = x[(i*2)+1] - x[i*2];
	dy = y[(i*2)+1] - y[i*2];
	d = sqrt((dy*dy)+(dx*dx));
	dx /= d;
	dy /= d;
	dist[i] = fabs((y[i*2] * (dx)) - (x[i*2] * (dy)));
      }
      value = 1.0 - (dist[0] / RANGE_Y);
      if (value > 1.0) value = 1.0;
      if (value < 0.0) value = 0.0;

      /* fprintf(stderr, "dist: %g %g (%g)  x= %g %g   y= %g %g\n", 
	 dist[0], dist[1], value, x[0], x[1], y[0], y[1]); */

      
      X_set_variable(X_target_y_estimate, &value);
      X_set_variable(X_target_y_estimate2, &value);
    }


    /*
    fprintf(stderr, "x=%8.5f   y=%8.5f\n",
	    X_pattern->position->x - initial_x,
	    X_pattern->position->y - initial_y);
	    */
    
    /*
     * ====================================================================
     * 4. set targets
     * ====================================================================
     */
    
    if (TRAINING_ANGLE){

    /*
     * angle
     */
      
      /* X_set_target(X_sonar_angle_estimate, X_target_angle);   */
      X_set_target(X_integrated_angle_estimate, X_target_angle);
    }
    
    if (TRAINING_X){
      
      X_set_target(X_integrated_x_estimate, X_target_x_estimate); 
      /* X_set_target(X_x_estimate, X_target_x_estimate); */
    }

    if (TRAINING_Y){
      X_set_target(X_integrated_y_estimate, X_target_y_estimate);  
      /* X_set_target(X_y_estimate, X_target_y_estimate);     */
    }


  }
  if (TRAINING_COLLI){
    /*
     * estimate colli flag
       */
    
    X_set_variable(X_colli_target, &colli_target);
    X_set_target(X_colli, X_colli_target);
    
  }
}


/************************************************************************\
 ************************************************************************
\************************************************************************/



/*
 * ----------------------------------------------------------------------
 *
 * The following routine convolves the output in response to time 
 * passing by
 */

void
X_action(float time_elapsed)
{
  float diff_orientation, diff_translation;
  float variance, zero; 
  int input[2], output[1];
  int   subroutine_input[4], subroutine_output[4];


  if (prev_pos_defined){
    diff_orientation = 
      X_pattern->position->orientation - prev_pos_orientation;
    for (; diff_orientation < -90.0; ) diff_orientation += 180.0;
    for (; diff_orientation >  90.0; ) diff_orientation -= 180.0;
    diff_orientation /= 180.0;
    diff_orientation *= -1.0;
    diff_translation = 
      sqrt(((X_pattern->position->x - prev_pos_x) *
	    (X_pattern->position->x - prev_pos_x)) +
	   ((X_pattern->position->y - prev_pos_y) *
	    (X_pattern->position->y - prev_pos_y)));
  }
  else{
    diff_orientation = 0.0;
    diff_translation = 0.0;
  }

  
  /*
   * if no motion, then return
   */

  if (diff_orientation != 0.0 || diff_translation != 0.0){
    motion_detected = 3;

    /*
     * 10% random noise, plus some systematic rotational noise.
     */
    
    if (NOISE && (diff_orientation > 0.0 || diff_translation > 0.0)){
      static int warned = 0;
      if (!warned){
	fprintf(stderr, "\n\n\t### WARNING: 10% NOISE IN ODOMETRY\n\n");
	warned = 1;
      }
      diff_orientation += (0.1 * (0.1 + RAND_PLUS_MINUS_ONE()) 
			   * diff_orientation);
      if (diff_orientation < -0.5) diff_orientation = -0.5;
      if (diff_orientation >  0.5) diff_orientation =  0.5;
      diff_translation += (0.1 * RAND_PLUS_MINUS_ONE() * diff_translation);
      if (diff_translation < 0.0)
	diff_translation = 0.0;
    }
    

    diff_translation /= MAX_DIFF_TRANSLATION;
    if (diff_translation > 1.0)
      diff_translation = 1.0;
    X_set_variable(X_dist_moved, &diff_translation);
    
    
    if (COMPUTE_ANGLE){
      
      /*
       * update the wall angle
       */
      
      variance = fabs(diff_orientation) * 0.05;
      
      X_convolve_with_Gaussian(X_integrated_angle_estimate, 
			       &diff_orientation,
			       &variance, 0.0); 
      
      X_truncate_variable(X_integrated_angle_estimate, 0.05);
      
      
      /*
       * update global orientation
       */
      
      variance = variance / 2.0;
      diff_orientation = diff_orientation / 2.0;
      X_convolve_with_Gaussian(X_integrated_orientation_estimate, 
			       &diff_orientation,
			       &variance, 0.0); 
      
      X_truncate_variable(X_integrated_orientation_estimate, 0.05);
      
      
    }
    
    /*
     * update global x estimate
     */
    
    
    
    if (COMPUTE_X){
      subroutine_input[0]  = X_integrated_orientation_estimate;
      subroutine_input[1]  = X_dist_moved;
      subroutine_input[2]  = X_integrated_x_estimate;
      subroutine_output[0] = X_integrated_x_estimate2;
      X_execute_subroutine(update_x,
			   3, 1, subroutine_input, subroutine_output);
      X_copy_variable(X_integrated_x_estimate2, X_integrated_x_estimate);
      X_truncate_variable(X_integrated_x_estimate, 0.05);
      X_truncate_variable(X_integrated_x_estimate2, 1.0);
    }
    
    /*
     * update global y estimate
     */
    
    
    if (COMPUTE_Y){
      zero = 0.0;
      variance = fabs(diff_translation) * MAX_DIFF_TRANSLATION /
	RANGE_Y * 8.0;
      X_convolve_with_Gaussian(X_integrated_y_estimate, 
			       &zero, &variance, 0.0); 
      
      subroutine_input[0]  = X_integrated_orientation_estimate;
      subroutine_input[1]  = X_dist_moved;
      subroutine_input[2]  = X_integrated_y_estimate;
      subroutine_output[0] = X_integrated_y_estimate2;
      X_execute_subroutine(update_y,
			   3, 1, subroutine_input, subroutine_output);
      X_copy_variable(X_integrated_y_estimate2, X_integrated_y_estimate);
      X_truncate_variable(X_integrated_y_estimate, 0.05);
      X_truncate_variable(X_integrated_y_estimate2, 1.0);
    }
  }
    
  else{
    if (motion_detected > 0)
      motion_detected--;
  }

  /*
   * save previous position and angle estimate
   */
  
  prev_pos_x           = X_pattern->position->x;
  prev_pos_y           = X_pattern->position->y;
  prev_pos_orientation = X_pattern->position->orientation;
  prev_pos_defined     = 1;

}
 



#endif





























#ifdef XOR



/***********************************************************************\
 *                                                                     *
 *                            X O R                                    *
 *                                                                     *
\***********************************************************************/

int X_input1       = -1;
int X_input2       = -1;
int X_hidden1      = -1; 
int X_output1      = -1;
int X_output2      = -1;
int X_output2sub   = -1;
int X_output1sub   = -1;
int X_target1      = -1;
int X_target2      = -1;
int X_network1a    = -1;
int X_network1b    = -1;
int X_network2     = -1;

#define STEPSIZE 0.01
#define NETWORK1
#define NETWORK2


#define NUM_SUBROUTINE_INPUT  3
#define NUM_SUBROUTINE_OUTPUT 2



X_subroutine_type
subroutine(int    num_in,	/* number of input variables */
	   int    *in_dims,	/* dimensions of the input variables */
	   float  **in,		/* input variables (values) */
	   int    num_out,	/* number of output variables */
	   int    *out_dims,	/* dimensions of the output variables */
	   float  **out)	/* output variables (values) */
{
  SUBROUTINE_REPORT();

  out[0][0] = in[1][0]; 
  out[0][1] = in[1][1]; 
  out[1][0] = in[2][0];
}



void X_compute(float time_elapsed)
{
  int subroutine_input[NUM_SUBROUTINE_INPUT];
  int subroutine_output[NUM_SUBROUTINE_OUTPUT];
  

#ifdef NETWORK1
  X_set_variable(X_input1, X_pattern->lasers); 
  X_set_variable(X_target1, &(X_pattern->lasers[2])); 

  X_neuro_net(X_network1a, X_input1, X_hidden1); 
  X_neuro_net(X_network1b, X_hidden1, X_output1);  
#endif


#ifdef NETWORK2
  X_set_variable(X_input2, X_pattern->lasers);
  X_set_variable(X_target2, &(X_pattern->lasers[2]));

  X_neuro_net(X_network2, X_input2, X_output2); 
#endif

  /* subroutine_input[0]  = X_output1; */
  /* subroutine_input[1]  = X_output2; */
  /* subroutine_output[0] = X_output1sub; */
  /* subroutine_output[1] = X_output2sub; */


  subroutine_input[1]  = X_output1; 
  subroutine_output[0] = X_output1sub; 
  subroutine_input[2]  = X_output2;   
  subroutine_output[1] = X_output2sub;   
  subroutine_input[0]  = X_input1;   
  /*
  X_execute_subroutine(subroutine, 
		       NUM_SUBROUTINE_INPUT,
		       NUM_SUBROUTINE_OUTPUT,
		       subroutine_input,
		       subroutine_output);

		       */

#ifdef NETWORK1
   X_set_target(X_output1, X_target1);  
#endif

#ifdef NETWORK2
   X_set_target(X_output2, X_target2);  
#endif

}


void
X_init()
{

#ifdef NETWORK1
  X_input1     = X_register_variable("input1",  2, 1, 1); 
  X_hidden1    = X_register_variable("hidden1", 2, 1, 1);    
  /* X_hidden1 = X_register_stochastic_variable("hidden1", 2, 0, 10, 1, 1); */
  /* X_output1 = X_register_variable("output1", 1, 1, 1); */
  X_output1    = X_register_stochastic_variable("output1", 2, 0, 10, 1, 1); 
  X_output1sub = X_register_stochastic_variable("output1sub", 2, 0, 10, 1, 1);
  X_target1    = X_register_variable("target1", 2, 1, 1); 


  X_network1a =
    X_register_neural_network("network1a", DETERMINISTIC, 2,
			      DETERMINISTIC, 2,             1, 2);
  X_network1b = 
    X_register_neural_network("network1b", DETERMINISTIC, 2, 
   			      STOCHASTIC, 2,                1, 2); 
  X_network_set_stepsize(X_network1a, STEPSIZE);
  X_network_set_stepsize(X_network1b, STEPSIZE);
#endif

#ifdef NETWORK2
  X_input2     = X_register_variable("input2",  2, 1, 1);
  /* X_output2 = X_register_variable("output2", 2, 1, 1); */
  X_output2    = X_register_stochastic_variable("output2", 1, 0, 10, 1, 1);
  X_output2sub = X_register_stochastic_variable("output2sub", 1, 0, 10, 1, 1);
  X_target2    = X_register_variable("target2", 1, 1, 1);

  

  X_network2 =
    X_register_neural_network("network2", DETERMINISTIC, 2,
			      STOCHASTIC, 1,               1, 2);
  X_network_set_stepsize(X_network2,  STEPSIZE);

#endif  

  X_register_process("compute", X_compute);
  X_specify_trigger_condition(X_compute, LASERS_EVENT, 1);
}

void
X_initialize_variables()
{
}



#endif
