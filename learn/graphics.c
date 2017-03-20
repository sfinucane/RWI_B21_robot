
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/learn/graphics.c,v $
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
 * $Log: graphics.c,v $
 * Revision 1.1  2002/09/14 16:36:05  rstone
 * *** empty log message ***
 *
 * Revision 1.37  1999/07/11 18:48:33  thrun
 * slight reorganization
 *
 * Revision 1.36  1998/08/22 03:08:36  thrun
 * .
 *
 * Revision 1.35  1998/07/04 15:21:37  thrun
 * variable logging.
 *
 * Revision 1.34  1998/07/03 23:59:58  thrun
 * .
 *
 * Revision 1.33  1998/06/24 12:50:06  thrun
 * initial gesture interface.
 *
 * Revision 1.32  1998/06/23 03:33:19  thrun
 * Nice prototpe of the mail-delivery robot. What's missing is:
 * gesture recognition and the real robot tests.
 *
 * Revision 1.31  1998/06/20 21:05:32  thrun
 * Reorganization (created: fa.{c,h}, xtypes.h)
 *
 * Revision 1.30  1998/05/05 04:00:33  thrun
 * Direct control mode (and button). Other stuff I don't fully remember.
 * Intermediate version.
 *
 * Revision 1.29  1998/04/28 23:30:46  thrun
 * autosave of weights, we can now switch off learning in selected
 * networks and variables, which is important for incremental
 * programming/learning
 *
 * Revision 1.28  1998/04/20 03:12:11  thrun
 * Intermediate version with subroutines.
 *
 * Revision 1.27  1998/04/18 20:42:24  thrun
 * 1. neuro net can now be called with the same variable as input
 * and output
 * 2. direct control modus for the robot. controls are
 * now generated exclusively through the display (left upper corner)
 *
 * Revision 1.26  1998/04/14 02:35:55  thrun
 * .
 *
 * Revision 1.25  1998/04/13 01:45:18  thrun
 * Thermomter encoding for sigmodial neural networks. Delay for
 * pausing (when recording data).
 *
 * Revision 1.24  1997/10/23 02:28:22  thrun
 * .
 *
 * Revision 1.23  1997/10/05 18:11:17  thrun
 * new data library "libdat.a"
 *
 * Revision 1.22  1997/08/05 04:15:22  thrun
 * (1) avg_value are saved for each stoch variable, (2) enhanced replay
 * functions.
 *
 * Revision 1.21  1997/07/30 23:29:33  thrun
 * recording of base motion commands (velocity).
 *
 * Revision 1.20  1997/07/30 21:02:01  thrun
 * baseServer now sends confirmations of all motion commands. Is used
 * for logging in the module "learn"
 *
 * Revision 1.19  1997/07/30 13:12:12  thrun
 * Changed display slightly
 *
 * Revision 1.18  1997/07/30 03:46:52  thrun
 * New events: pantilt and buttons
 *
 * Revision 1.17  1997/07/29 22:44:47  thrun
 * .
 *
 * Revision 1.16  1997/07/07 04:45:23  thrun
 * Now with training and testing set support
 *
 * Revision 1.15  1997/07/06 22:51:48  thrun
 * Autosaving of the parameters (every 2 minutes)
 *
 * Revision 1.14  1997/07/06 18:42:04  thrun
 * Radial basis networks implemented.
 *
 * Revision 1.13  1997/07/04 18:09:32  thrun
 * intermediate version - do not use
 *
 * Revision 1.12  1997/07/04 00:28:54  thrun
 * Intermediate verson of the learning stuff. What works is:
 * - training of deterministic variables (incl. chaining)
 * - training of multiple determinstic with a stochastic variables (incl chaining)
 * What doesn't yet work includes
 * - temporal chaining of stochastic variables, using Bayes rule
 * - convolving: gradients
 * - stochatic input
 *
 * Revision 1.11  1997/06/30 05:53:57  thrun
 * intermediate version - new neural network (which seems to work!).
 *
 * Revision 1.10  1997/06/29 04:04:55  thrun
 * intrmediate version - not really good for anything
 *
 * Revision 1.9  1997/06/28 13:43:44  thrun
 * ..changed back display size...
 *
 * Revision 1.8  1997/06/28 13:41:40  thrun
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

#define DEFINE_SENSOR_LOCATIONS

#include "bUtils.h"
#include "EZX11.h"
#include "o-graphics.h"
#include "dat.h"
#include "mem.h"
#include "global.h"
#include "xtypes.h"
#include "vars.h"
#include "fa.h"
#include "robot_specifications.h"
#include "cameraClient.h"
#include "buttonClient.h"
#include "pantiltClient.h"


/************************************************************************\
 ************************************************************************
\************************************************************************/




int CONTROL_ROBOT1;
int CONTROL_ROBOT2;
int CONTROL_DOTS;
int CONTROL_FRAME;
int SONAR_ROBOT;
int LASER_ROBOT;
int IR_ROBOT1;
int IR_ROBOT2;
int IR_ROBOT3;
int TACTILE_ROBOT1;
int TACTILE_ROBOT2;
int TACTILE_ROBOT3;
int TACTILE_ROBOT4;
int POSITION_DATA;
int IMAGE;
int PAN_DIAL;
int TILT_DIAL;
int TIME_DATA;
int TRANS_VEL_POS;
int TRANS_VEL_NEG;
int ROT_VEL_POS;
int ROT_VEL_NEG;

int QUIT_BUTTON;

int CONTROL_MARKERS;
int SONAR_MARKERS;
int LASER_MARKERS;
int IR_MARKERS;
int TACTILE_MARKERS;
int IMAGE_MARKERS;

int CONTROL_TITLE;
int SONAR_TITLE;
int LASER_TITLE;
int IR_TITLE;
int TACTILE_TITLE;

int DATABASE_SIZE_BUTTON;
int DISPLAY_MODUS_BUTTON;
int RECORD_BUTTON;
int EXECUTION_BUTTON;
int LOGGING_BUTTON;

int FIRST_BUTTON;
int PREVIOUS_BUTTON;
int NEXT_BUTTON;
int SAVE_BUTTON;
int LOAD_BUTTON;
int APPEND_BUTTON;
int MOVIE_BUTTON;
int DIAL_BUTTON;

int SET_FIRST_BUTTON;
int SET_PREVIOUS_BUTTON;
int SET_NEXT_BUTTON;
int SET_DIAL_BUTTON;
int SET_MOVIE_BUTTON;

int BASE_CONNECT_BUTTON;
int CAMERA_CONNECT_BUTTON;
int COLLI_CONNECT_BUTTON;
int BUTTONS_CONNECT_BUTTON;
int PANTILT_CONNECT_BUTTON;

int MARK_BUTTON;
int TRAIN_BUTTON;
int TEST_BUTTON;
int SAVE_NET_BUTTON;
int LOAD_NET_BUTTON;

int BUTTON_LIT[6];
int BUTTON_PUSHED[4];

float graphics_max_x = 11.0;


/************************************************************************\
 ************************************************************************
\************************************************************************/

/*#define BLACKWHITE*/


/************************************************************************
 *
 *   NAME:         init_graphics()
 *                 
 *   FUNCTION:     initializes the graphics window
 *                 
 *   PARAMETERS:   int global_use_X    indicates, whether we really use X-Win.
 *                 
 *                 
 *   RETURN-PARAM: none yet
 *                 
 *
 *   RETURN-VALUE: none yet
 *                 
 ************************************************************************/

void
init_graphics(int global_use_X)
{
#ifdef BLACKWHITE
 static char *myfonts[] = {"5x8", "5x8", "5x8", 
			      "9x15bold", "10x20", "12x24", "lucidasans-bold-24"};
#else
 static char *myfonts[] = {"5x8", "6x10", "7x13bold", 
			      "9x15bold", "10x20", "12x24", "lucidasans-bold-24"};
 static char *myfonts_small[] = {"5x8", "5x8", "5x8", 
			      "6x10", "6x10", "6x10", "lucidasans-bold-24"};
#endif
 int i, j;

 
 /* =============================================================
    ================  1) INITIALIZATIONS VARIABLES  =============
    ============================================================= */
 
 if (sonar_angles == NULL || sonar_values == NULL){
   sonar_angles      = (float *) malloc(sizeof(float) * NUM_SONAR_SENSORS);
   sonar_values      = (float *) malloc(sizeof(float) * NUM_SONAR_SENSORS);
   ir_values         = (float *) malloc(sizeof(float) *(NUM_IR_SENSORS));
   ir_angles         = (float *) malloc(sizeof(float) * (NUM_IR_SENSORS));

   tactile_values    = (float *) malloc(sizeof(float) * (NUM_TACTILE_SENSORS));
   tactile_angles    = (float *) malloc(sizeof(float) * (NUM_TACTILE_SENSORS));

   laser_values      = (float *) malloc(sizeof(float) * NUM_LASER_SENSORS);
   laser_angles      = (float *) malloc(sizeof(float) * NUM_LASER_SENSORS);

   if (sonar_angles == NULL || sonar_values == NULL ||
       ir_values == NULL || ir_angles == NULL ||
       tactile_values == NULL || tactile_angles == NULL ||
       laser_values == NULL || laser_angles == NULL){
     fprintf(stderr, "Out of Memory in init_graphics(). Must exit.\n");
     exit(-1);
   }


   for (i = 0; i < NUM_SONAR_SENSORS; i++){
     sonar_angles[i] = sonar_locations[i].angle;
     sonar_values[i]      = 0.0;
   }
   for (i = 0; i < NUM_IR_SENSORS; i++){
     ir_angles[i] = ir_locations[i].angle;
     ir_values[i] = 0.0;
   }
   for (i = 0; i < NUM_TACTILE_SENSORS; i++){
     tactile_angles[i] = tactile_locations[i].angle;
     if (i < NUM_TACTILE_SENSORS_AT_HEIGHT_1)
       tactile_values[i] = 8.0;
     else if (i < (NUM_TACTILE_SENSORS_AT_HEIGHT_1) + 
	      (NUM_TACTILE_SENSORS_AT_HEIGHT_2))
       tactile_values[i] = 6.0;
     else if (i < (NUM_TACTILE_SENSORS_AT_HEIGHT_1) + 
	      (NUM_TACTILE_SENSORS_AT_HEIGHT_2) + 
	      (NUM_TACTILE_SENSORS_AT_HEIGHT_3))
       tactile_values[i] = 4.0;
     else
       tactile_values[i] = 2.0;
   }
   for (i = 0; i < NUM_LASER_SENSORS; i++){
     laser_angles[i] = laser_locations[i].angle;
     laser_values[i]      = 0.0;
   }

 }
 

 if (d_image == NULL){
   d_image = (float *) malloc(sizeof(float) * (IMAGE_SIZE_X * IMAGE_SIZE_Y));
   for (i = 0; i < IMAGE_SIZE_X * IMAGE_SIZE_Y; i++)
     d_image[i] = 0.5 * cos(((float) i) * 0.01) + 0.5;
 }
   


 
 /* =============================================================
    ================  2) INITIALIZATIONS GRAPHICS  ==============
    ============================================================= */

 G_set_display(global_use_X);
#ifdef BLACKWHITE
 G_initialize_fonts(7, myfonts);
 G_initialize_graphics("LEARN", 30.0, 10.0, C_WHITE);
#else
 if (global_small_X){
   G_initialize_fonts(7, myfonts_small);
   G_initialize_graphics("LEARN", 30.0, 10.0, C_STEELBLUE4);
 }
 else{
   G_initialize_fonts(7, myfonts);
   G_initialize_graphics("LEARN", 50.0, 10.0, C_STEELBLUE4);
 }
#endif
 G_markers_display_style = 0;
 



}




/************************************************************************
 *
 *   NAME:         init_graphics_2()
 *                 
 *   FUNCTION:     initializes the graphics window
 *                 
 *   PARAMETERS:   int global_use_X    indicates, whether we really use X-Win.
 *                 
 *                 
 *   RETURN-PARAM: none yet
 *                 
 *
 *   RETURN-VALUE: none yet
 *                 
 ************************************************************************/

void
init_graphics_2(int global_use_X)
{
  static float dummy = 0.0;



 /******** TRANS_VEL_POS *****************************/
 {
   static float value_pos[]            = {-2.8, -2.6, 4.2, 5.4};
   static char *value_text             = "+";
   static int value_font               = 1;
   int direction_value                 = 2;
   static float value                  = 0.0;
   float min_value                     = 0.0;
   float max_value                     = MAX_TRANS_VEL;
   static int value_colors[]           = {C_GREY70, C_RED, C_GREY30,
					  C_BLACK}; 
   
   TRANS_VEL_POS = G_create_value_object(value_pos, value_text, 
				      direction_value, value, 
				      min_value,
				      max_value, value_colors, 
				      value_font);
   
 }

 /******** TRANS_VEL_NEG *****************************/
 {
   static float value_pos[]            = {-2.8, -2.6, 3.0, 4.2};
   static char *value_text             = "-";
   static int value_font               = 1;
   int direction_value                 = 2;
   static float value                  = MAX_TRANS_VEL;
   float min_value                     = 0.0;
   float max_value                     = MAX_TRANS_VEL;
   static int value_colors[]           = {C_RED, C_GREY70, C_GREY30,
					  C_BLACK}; 
   
   TRANS_VEL_NEG = G_create_value_object(value_pos, value_text, 
				      direction_value, value, 
				      min_value,
				      max_value, value_colors, 
				      value_font);
   
 }



 /******** ROT_VEL_POS *****************************/
 {
   static float value_pos[]            = {-1.3, -0.1, 2.7, 2.9};
   static char *value_text             = "right";
   static int value_font               = 1;
   int direction_value                 = 1;
   static float value                  = 0.0;
   float min_value                     = 0.0;
   float max_value                     = MAX_ROT_VEL;
   static int value_colors[]           = {C_GREY70, C_RED, C_GREY30,
					  C_BLACK}; 
   
   ROT_VEL_POS = G_create_value_object(value_pos, value_text, 
				      direction_value, value, 
				      min_value,
				      max_value, value_colors, 
				      value_font);
   
 }

 /******** ROT_VEL_NEG *****************************/
 {
   static float value_pos[]            = {-2.5, -1.3, 2.7, 2.9};
   static char *value_text             = "left";
   static int value_font               = 1;
   int direction_value                 = 1;
   static float value                  = MAX_ROT_VEL;
   float min_value                     = 0.0;
   float max_value                     = MAX_ROT_VEL;
   static int value_colors[]           = {C_RED, C_GREY70, C_GREY30,
					  C_BLACK}; 
   
   ROT_VEL_NEG = G_create_value_object(value_pos, value_text, 
				      direction_value, value, 
				      min_value,
				      max_value, value_colors, 
				      value_font);
   
 }

 
 {
   /******** CONTROL_ROBOT *********************************/ 
   static float pos_r[]                 = 
     {-2.5, -0.1, 3.0, 5.4};
   static char *text_r                  = "control";
   static int robot_font                = 2;
#ifdef BLACKWHITE
   static int colors_r1[]                = {C_WHITE, C_GREY30, NO_COLOR,
					     NO_COLOR, NO_COLOR, NO_COLOR,
					     C_WHITE, NO_COLOR};
   static int colors_r2[]                = {NO_COLOR, NO_COLOR, C_WHITE,
					     C_BLACK, NO_COLOR, NO_COLOR,
					     NO_COLOR, NO_COLOR};
#else
   static int colors_r1[]                = {C_GREY80, C_GREY30, NO_COLOR,
					     NO_COLOR, NO_COLOR, NO_COLOR,
					     C_GREY85, NO_COLOR};
   static int colors_r2[]                = {NO_COLOR, NO_COLOR, C_WHITE,
					     C_BLACK, NO_COLOR, NO_COLOR,
					     NO_COLOR, NO_COLOR};
#endif   
      
   
   CONTROL_ROBOT1 =
     G_create_robot_object(pos_r, text_r, 
			   -(MAX_ROT_VEL)*1.01, 
			   (MAX_ROT_VEL)*1.01,
			   -(MAX_TRANS_VEL)*1.01,
			   (MAX_TRANS_VEL)*1.01,
			   0.0, 0.0, 90.0,
			   0.1*(MAX_ROT_VEL), 0, 
			   (MAX_ROT_VEL),
			   &dummy, &dummy,
			   colors_r1, robot_font);
   
   CONTROL_ROBOT2 =
     G_create_robot_object(pos_r, text_r, 
			   -(MAX_ROT_VEL)*1.01, 
			   (MAX_ROT_VEL)*1.01,
			   -(MAX_TRANS_VEL)*1.01, 
			   (MAX_TRANS_VEL)*1.01,
			   0.0, 0.0, 90.0,
			   0.1*(MAX_ROT_VEL), 0, 
			   (MAX_ROT_VEL),
			   &dummy, &dummy,
			   colors_r2, robot_font);
   
 }

 /******** CONTROL_MARKERS **************************************/
 {
   static float pos_r[]                   = 
     {-2.5, -0.1, 3.0, 5.4};
   int num_mar                            = 3;
   static char *text_mar[]                = {"", "", ""};
   int connected_mar                      = 1;
   static int mar_frame_color             = NO_COLOR;
   static int mar_background_color        = NO_COLOR;
   static int mar_foreground_color[]      = {C_RED, C_YELLOW, C_GREY85};
   static int mar_text_color[]            = {NO_COLOR, NO_COLOR, NO_COLOR};
   static int mar_fonts[]                 = {2, 2, 2};
      
   CONTROL_MARKERS =
     G_create_markers_object(pos_r, connected_mar, 
			     (MAX_ROT_VEL) / 15.0,
			     -(MAX_ROT_VEL)*1.01,
			     (MAX_ROT_VEL)*1.01,
			     -(MAX_TRANS_VEL)*1.01,
			     (MAX_TRANS_VEL)*1.01,
			     num_mar, text_mar, mar_background_color, 
			     mar_frame_color, mar_foreground_color, 
			     mar_text_color, mar_fonts);
 }

 

 {
   /******** CONTROL_DOTS **************************************/

   static float pos_r[]                   = 
     {-2.5, -0.1, 3.0, 5.4};
   int num_mar                            = 1;
   static char *text_mar[]                = {""};
   int connected_mar                      = 1;
   static int mar_frame_color             = NO_COLOR;
   static int mar_background_color        = NO_COLOR;
   static int mar_foreground_color[]      = {C_BLUE};
   static int mar_text_color[]            = {NO_COLOR};
   static int mar_fonts[]                 = {2};

   CONTROL_DOTS =
     G_create_markers_object(pos_r, connected_mar, 
			     0.05*(MAX_ROT_VEL),
			     -(MAX_ROT_VEL)*1.01,
			     (MAX_ROT_VEL)*1.01,
			     -(MAX_TRANS_VEL)*1.01,
			     (MAX_TRANS_VEL)*1.01,
			     num_mar, text_mar, mar_background_color, 
			     mar_frame_color, mar_foreground_color, 
			     mar_text_color, mar_fonts);
 }

 {
   /******** CONTROL_TITLE ****************************/
   int switch_num                      = 1;
   static float switch_pos[]           =
     {-2.5, -0.1, 5.2, 5.4};
   static char *switch_texts[]         = {"controls"};
   static int switch_fonts[]           = {1};
   static int switch_background_color[]= {NO_COLOR};
   static int switch_frame_color[]     = {NO_COLOR};
   static int switch_text_color[]      = {C_BLACK};

   CONTROL_TITLE
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
			      switch_background_color,switch_frame_color, 
			      switch_text_color, switch_fonts);
 }

 {
   /******** CONTROL_FRAME ****************************/
   int switch_num                      = 2;
   static float switch_pos[]           =
     {-2.5, -0.1, 3.0, 5.4};
   static char *switch_texts[]         = {"",""};
   static int switch_fonts[]           = {2,2};
   static int switch_background_color[]= {NO_COLOR, NO_COLOR};
   static int switch_frame_color[]     = {C_GREY30, C_RED};
   static int switch_text_color[]      = {NO_COLOR, NO_COLOR};

   CONTROL_FRAME
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
			      switch_background_color,switch_frame_color, 
			      switch_text_color, switch_fonts);
 }

 {
   /******** ROBOT (SMALL) *********************************/ 
   static float pos_r[]                 = 
     {0.0, 2.4, 3.0, 5.4};
   static char *text_r                  = "sonar";
   static int robot_font                = 2;
#ifdef BLACKWHITE
   static int colors_r[]                = {C_WHITE, C_GREY30, C_WHITE,
					     C_BLACK, C_BLACK, NO_COLOR,
					     C_WHITE, NO_COLOR};
#else
   static int colors_r[]                = {C_GREY80, C_GREY30, C_WHITE,
					     C_BLACK, C_TURQUOISE4, NO_COLOR,
					     C_GREY85, NO_COLOR};
#endif   
      
   
   SONAR_ROBOT =
     G_create_robot_object(pos_r, text_r, 
			   -(MAX_SONAR_DISPLAY_RANGE)*1.01,
			   (MAX_SONAR_DISPLAY_RANGE)*1.01,
			   -(MAX_SONAR_DISPLAY_RANGE)*1.01,
			   (MAX_SONAR_DISPLAY_RANGE)*1.01,
			   0.0, 0.0, 90.0,
			   25.0, NUM_SONAR_SENSORS, 
			   MAX_SONAR_DISPLAY_RANGE,
			   sonar_values, sonar_angles,
			   colors_r, robot_font);
   
 }
 
 {
   
   /******** SONAR_TITLE ****************************/
   int switch_num                      = 1;
   static float switch_pos[]           =
     {0.0, 2.4, 5.2, 5.4};
   static char *switch_texts[]         = {"sonar"};
   static int switch_fonts[]           = {1};
   static int switch_background_color[]= {NO_COLOR};
   static int switch_frame_color[]     = {NO_COLOR};
   static int switch_text_color[]      = {C_BLACK};

   SONAR_TITLE
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
			      switch_background_color,switch_frame_color, 
			      switch_text_color, switch_fonts);
 }

 /******** SONAR_MARKERS **************************************/
 {
   static float pos_r[]                   = 
   {0.0, 2.4, 3.0, 5.4};
   int num_mar                            = 3;
   static char *text_mar[]                = {"", "", ""};
   int connected_mar                      = 1;
   static int mar_frame_color             = NO_COLOR;
   static int mar_background_color        = NO_COLOR;
   static int mar_foreground_color[]      = {C_RED, C_YELLOW, C_GREY85};
   static int mar_text_color[]            = {NO_COLOR, NO_COLOR, NO_COLOR};
   static int mar_fonts[]                 = {2, 2, 2};
      
   SONAR_MARKERS =
     G_create_markers_object(pos_r, connected_mar, 
			     (MAX_SONAR_DISPLAY_RANGE) / 15.0,
			     -(MAX_SONAR_DISPLAY_RANGE)*1.01,
			     (MAX_SONAR_DISPLAY_RANGE)*1.01,
			     -(MAX_SONAR_DISPLAY_RANGE)*1.01,
			     (MAX_SONAR_DISPLAY_RANGE)*1.01,
			     num_mar, text_mar, mar_background_color, 
			     mar_frame_color, mar_foreground_color, 
			     mar_text_color, mar_fonts);
 }



 {
   /******** ROBOT (SMALL) *********************************/ 
   static float pos_r[]                 = 
   {2.5, 4.9, 3.0, 5.4};
   static char *text_r                  = "laser";
   static int robot_font                = 2;
#ifdef BLACKWHITE
   static int colors_r[]                = {C_WHITE, C_GREY30, C_WHITE,
					     C_BLACK, C_BLACK, NO_COLOR,
					     C_WHITE, NO_COLOR};
#else
   static int colors_r[]                = {C_GREY80, C_GREY30, C_WHITE,
					     C_BLACK, C_TURQUOISE4, NO_COLOR,
					     C_GREY85, NO_COLOR};
#endif   
      
   
   LASER_ROBOT =
     G_create_robot_object(pos_r, text_r, 
			   -(MAX_LASER_DISPLAY_RANGE)*1.01,
			   (MAX_LASER_DISPLAY_RANGE)*1.01,
			   -(MAX_LASER_DISPLAY_RANGE)*1.01,
			   (MAX_LASER_DISPLAY_RANGE)*1.01,
			   0.0, 0.0, 90.0,
			   25.0, NUM_LASER_SENSORS, 
			   MAX_LASER_DISPLAY_RANGE,
			   laser_values, laser_angles,
			   colors_r, robot_font);
   
 }
 

 /******** LASER_MARKERS **************************************/
 {
   static float pos_r[]                   = 
   {2.5, 4.9, 3.0, 5.4};
   int num_mar                            = 3;
   static char *text_mar[]                = {"", "", ""};
   int connected_mar                      = 1;
   static int mar_frame_color             = NO_COLOR;
   static int mar_background_color        = NO_COLOR;
   static int mar_foreground_color[]      = {C_RED, C_YELLOW, C_GREY85};
   static int mar_text_color[]            = {NO_COLOR, NO_COLOR, NO_COLOR};
   static int mar_fonts[]                 = {2, 2, 2};
      
   LASER_MARKERS =
     G_create_markers_object(pos_r, connected_mar, 
			     (MAX_LASER_DISPLAY_RANGE) / 15.0,
			     -(MAX_LASER_DISPLAY_RANGE)*1.01,
			     (MAX_LASER_DISPLAY_RANGE)*1.01,
			     -(MAX_LASER_DISPLAY_RANGE)*1.01,
			     (MAX_LASER_DISPLAY_RANGE)*1.01,
			     num_mar, text_mar, mar_background_color, 
			     mar_frame_color, mar_foreground_color, 
			     mar_text_color, mar_fonts);
 }


			 
 {
   
   /******** LASER_TITLE ****************************/
   int switch_num                      = 1;
   static float switch_pos[]           =
     {2.5, 4.9, 5.2, 5.4};
   static char *switch_texts[]         = {"laser"};
   static int switch_fonts[]           = {1};
   static int switch_background_color[]= {NO_COLOR};
   static int switch_frame_color[]     = {NO_COLOR};
   static int switch_text_color[]      = {C_BLACK};

   LASER_TITLE
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
			      switch_background_color,switch_frame_color, 
			      switch_text_color, switch_fonts);
 }



 {
   /******** ROBOT (SMALL) *********************************/ 
   static float pos_r[]                 = 
     {0.0, 2.4, 0.5, 2.9};
   static char *text_r                  = "IRs";
   static int robot_font                = 2;
#ifdef BLACKWHITE
   static int colors_r[]                = {C_WHITE, NO_COLOR, NO_COLOR,
					     NO_COLOR, C_BLACK, NO_COLOR,
					     C_WHITE, NO_COLOR};
#else
   static int colors_r[]                = {C_GREY80, NO_COLOR, NO_COLOR,
					     NO_COLOR, C_TURQUOISE4, NO_COLOR,
					     C_GREY85, NO_COLOR};
#endif   
      
   
   IR_ROBOT1 =
     G_create_robot_object(pos_r, text_r, 
			   -(MAX_IR_DISPLAY_RANGE)*1.01,
			   (MAX_IR_DISPLAY_RANGE)*1.01,
			   -(MAX_IR_DISPLAY_RANGE)*1.01,
			   (MAX_IR_DISPLAY_RANGE)*1.01,
			   0.0, 0.0, 90.0,
			   25.0, NUM_IR_SENSORS_AT_HEIGHT_1,
			   MAX_IR_DISPLAY_RANGE,
			   &(ir_values[0]), 
			   &(ir_angles[0]),
			   colors_r, robot_font);
   
 }
 

 {
   /******** ROBOT (SMALL) *********************************/ 
   static float pos_r[]                 = 
     {0.0, 2.4, 0.5, 2.9};
   static char *text_r                  = "IRs";
   static int robot_font                = 2;
#ifdef BLACKWHITE
   static int colors_r[]                = {NO_COLOR, NO_COLOR, NO_COLOR,
					     NO_COLOR, C_BLACK, NO_COLOR,
					     NO_COLOR, NO_COLOR};
#else
   static int colors_r[]                = {NO_COLOR, NO_COLOR, NO_COLOR,
					     NO_COLOR, C_RED, NO_COLOR,
					     NO_COLOR, NO_COLOR};
#endif   
      
   
   IR_ROBOT2 =
     G_create_robot_object(pos_r, text_r, 
			   -(MAX_IR_DISPLAY_RANGE)*1.01,
			   (MAX_IR_DISPLAY_RANGE)*1.01,
			   -(MAX_IR_DISPLAY_RANGE)*1.01,
			   (MAX_IR_DISPLAY_RANGE)*1.01,
			   0.0, 0.0, 90.0,
			   25.0, NUM_IR_SENSORS_AT_HEIGHT_2,
			   MAX_IR_DISPLAY_RANGE,
			   &(ir_values[NUM_IR_SENSORS_AT_HEIGHT_1]), 
			   &(ir_angles[NUM_IR_SENSORS_AT_HEIGHT_1]),
			   colors_r, robot_font);
   
 }
 {
   /******** ROBOT (SMALL) *********************************/ 
   static float pos_r[]                 = 
     {0.0, 2.4, 0.5, 2.9};
   static char *text_r                  = "IRs";
   static int robot_font                = 2;
#ifdef BLACKWHITE
   static int colors_r[]                = {NO_COLOR, C_GREY30, C_WHITE,
					     C_BLACK, C_BLACK, NO_COLOR,
					     NO_COLOR, NO_COLOR};
#else
   static int colors_r[]                = {NO_COLOR, C_GREY30, C_WHITE,
					     C_BLACK, C_BLUE, NO_COLOR,
					     NO_COLOR, NO_COLOR};
#endif   
      
   
   IR_ROBOT3 =
     G_create_robot_object(pos_r, text_r, 
			   -(MAX_IR_DISPLAY_RANGE)*1.01,
			   (MAX_IR_DISPLAY_RANGE)*1.01,
			   -(MAX_IR_DISPLAY_RANGE)*1.01,
			   (MAX_IR_DISPLAY_RANGE)*1.01,
			   0.0, 0.0, 90.0,
			   25.0, NUM_IR_SENSORS_AT_HEIGHT_3,
			   MAX_IR_DISPLAY_RANGE,
			   &(ir_values[(NUM_IR_SENSORS_AT_HEIGHT_1)+
				      (NUM_IR_SENSORS_AT_HEIGHT_2)]), 
			   &(ir_angles[(NUM_IR_SENSORS_AT_HEIGHT_1)+
				      (NUM_IR_SENSORS_AT_HEIGHT_2)]),
			   colors_r, robot_font);
   
 }
 {
   
   /******** IR_TITLE ****************************/
   int switch_num                      = 1;
   static float switch_pos[]           =
     {0.0, 2.4, 2.7, 2.9};
   static char *switch_texts[]         = {"infrared"};
   static int switch_fonts[]           = {1};
   static int switch_background_color[]= {NO_COLOR};
   static int switch_frame_color[]     = {NO_COLOR};
   static int switch_text_color[]      = {C_BLACK};

   IR_TITLE
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
			      switch_background_color,switch_frame_color, 
			      switch_text_color, switch_fonts);
 }

 

 /******** IR_MARKERS **************************************/
 {
   static float pos_r[]                   = 
   {0.0, 2.4, 0.5, 2.9};
   int num_mar                            = 3;
   static char *text_mar[]                = {"", "", ""};
   int connected_mar                      = 1;
   static int mar_frame_color             = NO_COLOR;
   static int mar_background_color        = NO_COLOR;
   static int mar_foreground_color[]      = {C_RED, C_YELLOW, C_GREY85};
   static int mar_text_color[]            = {NO_COLOR, NO_COLOR, NO_COLOR};
   static int mar_fonts[]                 = {2, 2, 2};
      
   IR_MARKERS =
     G_create_markers_object(pos_r, connected_mar, 
			     (MAX_IR_DISPLAY_RANGE) / 15.0,
			     -(MAX_IR_DISPLAY_RANGE)*1.01,
			     (MAX_IR_DISPLAY_RANGE)*1.01,
			     -(MAX_IR_DISPLAY_RANGE)*1.01,
			     (MAX_IR_DISPLAY_RANGE)*1.01,
			     num_mar, text_mar, mar_background_color, 
			     mar_frame_color, mar_foreground_color, 
			     mar_text_color, mar_fonts);
 }



 {
   /******** ROBOT (SMALL) *********************************/ 
   static float pos_r[]                 = 
     {2.5, 4.9, 0.5, 2.9};
   static char *text_r                  = "tactiles";
   static int robot_font                = 2;
#ifdef BLACKWHITE
   static int colors_r[]                = {C_WHITE, NO_COLOR, C_WHITE,
					     C_BLACK, C_BLACK, NO_COLOR,
					     C_WHITE, NO_COLOR};
#else
   static int colors_r[]                = {C_GREY80, NO_COLOR, C_GREY85,
					     C_TURQUOISE4, C_TURQUOISE4, NO_COLOR,
					     C_GREY85, NO_COLOR};
#endif   
      
   TACTILE_ROBOT1 =
     G_create_robot_object(pos_r, text_r, 
			   -(MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   (MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   -(MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   (MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   0.0, 0.0, 90.0,
			   8.0, NUM_TACTILE_SENSORS_AT_HEIGHT_1,
			   9.0,
			   &(tactile_values[0]), 
			   &(tactile_angles[0]),
			   colors_r, robot_font);
   


 }
 

 {
   /******** ROBOT (SMALL) *********************************/ 
   static float pos_r[]                 = 
     {2.5, 4.9, 0.5, 2.9};
   static char *text_r                  = "tactiles";
   static int robot_font                = 2;
#ifdef BLACKWHITE
   static int colors_r[]                = {NO_COLOR, NO_COLOR, C_WHITE,
					     C_BLACK, C_BLACK, NO_COLOR,
					     NO_COLOR, NO_COLOR};
#else
   static int colors_r[]                = {NO_COLOR, NO_COLOR, C_GREY85,
					     C_TURQUOISE4, C_TURQUOISE4,
					   NO_COLOR,
					     NO_COLOR, NO_COLOR};
#endif   
      
   
   TACTILE_ROBOT2 =
     G_create_robot_object(pos_r, text_r, 
			   -(MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   (MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   -(MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   (MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   0.0, 0.0, 90.0,
			   6.0, NUM_TACTILE_SENSORS_AT_HEIGHT_2,
			   9.0,
			   &(tactile_values[NUM_TACTILE_SENSORS_AT_HEIGHT_1]), 
			   &(tactile_angles[NUM_TACTILE_SENSORS_AT_HEIGHT_1]),
			   colors_r, robot_font);
   
 }

 {
   /******** ROBOT (SMALL) *********************************/ 
   static float pos_r[]                 = 
     {2.5, 4.9, 0.5, 2.9};
   static char *text_r                  = "tactiles";
   static int robot_font                = 2;
#ifdef BLACKWHITE
   static int colors_r[]                = {NO_COLOR, NO_COLOR, C_WHITE,
					     C_BLACK, C_BLACK, NO_COLOR,
					     NO_COLOR, NO_COLOR};
#else
   static int colors_r[]                = {NO_COLOR, NO_COLOR, C_GREY85,
					     C_BLUE, C_BLUE, NO_COLOR,
					     NO_COLOR, NO_COLOR};
#endif   
      
   
   TACTILE_ROBOT3 =
     G_create_robot_object(pos_r, text_r, 
			   -(MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   (MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   -(MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   (MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   0.0, 0.0, 90.0,
			   4.0, NUM_TACTILE_SENSORS_AT_HEIGHT_2,
			   9.0,
			   &(tactile_values[(NUM_TACTILE_SENSORS_AT_HEIGHT_1)+
				      (NUM_TACTILE_SENSORS_AT_HEIGHT_2)]), 
			   &(tactile_angles[(NUM_TACTILE_SENSORS_AT_HEIGHT_1)+
				      (NUM_TACTILE_SENSORS_AT_HEIGHT_2)]),
			   colors_r, robot_font);
   
 }
 {
   /******** ROBOT (SMALL) *********************************/ 
   static float pos_r[]                 = 
     {2.5, 4.9, 0.5, 2.9};
   static char *text_r                  = "tactiles";
   static int robot_font                = 2;
#ifdef BLACKWHITE
   static int colors_r[]                = {NO_COLOR, C_GREY30, C_WHITE,
					     C_BLACK, C_BLACK, NO_COLOR,
					     NO_COLOR, NO_COLOR};
#else
   static int colors_r[]                = {NO_COLOR, C_GREY30, C_WHITE,
					     C_BLUE, C_BLUE, NO_COLOR,
					     NO_COLOR, NO_COLOR};
#endif   
      
   
   TACTILE_ROBOT4 =
     G_create_robot_object(pos_r, text_r, 
			   -(MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   (MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   -(MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   (MAX_TACTILE_DISPLAY_RANGE)*1.01,
			   0.0, 0.0, 90.0,
			   2.0, NUM_TACTILE_SENSORS_AT_HEIGHT_3,
			   9.0,
			   &(tactile_values[(NUM_TACTILE_SENSORS_AT_HEIGHT_1)+
				      (NUM_TACTILE_SENSORS_AT_HEIGHT_2)+
				      (NUM_TACTILE_SENSORS_AT_HEIGHT_3)]), 
			   &(tactile_angles[(NUM_TACTILE_SENSORS_AT_HEIGHT_1)+
				      (NUM_TACTILE_SENSORS_AT_HEIGHT_2)+
				      (NUM_TACTILE_SENSORS_AT_HEIGHT_3)]),
			   colors_r, robot_font);
   
 }
 
 {
   
   /******** TACTILE_TITLE ****************************/
   int switch_num                      = 1;
   static float switch_pos[]           =
     {2.5, 4.9, 2.7, 2.9};
   static char *switch_texts[]         = {"tactile"};
   static int switch_fonts[]           = {1};
   static int switch_background_color[]= {NO_COLOR};
   static int switch_frame_color[]     = {NO_COLOR};
   static int switch_text_color[]      = {C_BLACK};

   TACTILE_TITLE
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
			      switch_background_color,switch_frame_color, 
			      switch_text_color, switch_fonts);
 }

 

 

 /******** TACTILE_MARKERS **************************************/
 {
   static float pos_r[]                   = 
   {2.5, 4.9, 0.5, 2.9};
   int num_mar                            = 3;
   static char *text_mar[]                = {"", "", ""};
   int connected_mar                      = 1;
   static int mar_frame_color             = NO_COLOR;
   static int mar_background_color        = NO_COLOR;
   static int mar_foreground_color[]      = {C_RED, C_YELLOW, C_GREY85};
   static int mar_text_color[]            = {NO_COLOR, NO_COLOR, NO_COLOR};
   static int mar_fonts[]                 = {2, 2, 2};
      
   TACTILE_MARKERS =
     G_create_markers_object(pos_r, connected_mar, 
			     (MAX_TACTILE_DISPLAY_RANGE) / 15.0,
			     -(MAX_TACTILE_DISPLAY_RANGE)*1.01,
			     (MAX_TACTILE_DISPLAY_RANGE)*1.01,
			     -(MAX_TACTILE_DISPLAY_RANGE)*1.01,
			     (MAX_TACTILE_DISPLAY_RANGE)*1.01,
			     num_mar, text_mar, mar_background_color, 
			     mar_frame_color, mar_foreground_color, 
			     mar_text_color, mar_fonts);
 }




 {
   
   /******** POSITION_DATA ****************************/
   int switch_num                      = 1;
   static float switch_pos[]           =
     {(LEFT_MARGIN), -0.1, 1.0, 1.4};
   static char *switch_texts[]         = {"pos: ???? ???? ????"};
   static int switch_fonts[]           = {1};
   static int switch_background_color[]= {C_GREY70};
   static int switch_frame_color[]     = {C_GREY30};
   static int switch_text_color[]      = {C_BLACK};

   POSITION_DATA
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
			      switch_background_color,switch_frame_color, 
			      switch_text_color, switch_fonts);
 }

 
 {
   
   /******** TIME_DATA ****************************/
   int switch_num                      = 1;
   static float switch_pos[]           =
     {(LEFT_MARGIN), -0.1, 0.5, 0.9};
   static char *switch_texts[]         = {"time: ??:??:??.??"};
   static int switch_fonts[]           = {1};
   static int switch_background_color[]= {C_GREY70};
   static int switch_frame_color[]     = {C_GREY30};
   static int switch_text_color[]      = {C_BLACK};

   TIME_DATA
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
			      switch_background_color,switch_frame_color, 
			      switch_text_color, switch_fonts);
 }

 
 {
   /******** IMAGE *******************************************/
   static float matrix_pos[]            = {5.8, 11.5, 0.8, 5.4};
   static char *matrix_text             = "Oops - should not be here";
   static int matrix_font               = 0;
   static int matrix_colors[]           = {NO_COLOR, C_GREY30, NO_COLOR}; 
   
   IMAGE = 
     G_create_matrix_object(matrix_pos, matrix_text, 
			    d_image, NULL, IMAGE_SIZE_X, IMAGE_SIZE_Y,
			    0.0, 1.0, matrix_colors, matrix_font);
 }
  
 {
   /******** BUTTON_LIT, BUTTON_PUSHED ************************************/
   int switch_num                      = 2;
   static float switch_pos[]           = {5.0, 5.4, 5.4, 5.4};
   static char *switch_texts[]         = {"?","?"};
   static int switch_fonts[]           = {2,2};
   static int switch_background_color[]= {C_GREY90, C_RED};
   static int switch_frame_color[]     = {C_GREY30, C_GREY30};
   static int switch_text_color[]      = {NO_COLOR, NO_COLOR};
   int i;

   for (i = 0; i < 6; i++){

     if (i == 0)
       switch_background_color[1] = C_RED;
     else if (i == 1)
       switch_background_color[1] = C_YELLOW;
     else if (i == 2)
       switch_background_color[1] = C_LAWNGREEN;
     else if (i == 3)
       switch_background_color[1] = C_BLUE;
     else if (i == 4)
       switch_background_color[1] = C_RED;
     else if (i == 5)
       switch_background_color[1] = C_RED;
     if (i != 0)
       switch_pos[3] = switch_pos[2] - 0.25; /* separator */
     switch_pos[2] = 
       switch_pos[3] - (((5.4 - 0.5) - (0.25 * 5.0) - (0.05 * 4.0)) / 10.0);

     BUTTON_LIT[i] = 
       G_create_switch_object(switch_pos, switch_num, switch_texts,
			      switch_background_color,switch_frame_color, 
			      switch_text_color, switch_fonts);

     if (i < 4){
       switch_pos[3] = switch_pos[2] - 0.05; /* separator */
       switch_pos[2] = 
	 switch_pos[3] - (((5.4 - 0.5) - (0.25 * 5.0) - (0.05 * 4.0)) / 10.0);
       BUTTON_PUSHED[i] = 
	 G_create_switch_object(switch_pos, switch_num, switch_texts,
				switch_background_color,switch_frame_color, 
				switch_text_color, switch_fonts);
     }
     
   }
 }
 
 /******** PAN_DIAL *****************************/
 {
   static float value_pos[]            = {5.8, 11.5, 0.5, 0.7};
   static char *value_text             = "pan";
   static int value_font               = 3;
   int direction_value                 = 1;
   static float value                  = 1.0;
   float min_value                     = MIN_PAN_ANGLE;
   float max_value                     = MAX_PAN_ANGLE;
   static int value_colors[]           = {C_GREY70, C_BLUE, C_GREY30,
					  NO_COLOR}; 
   
   PAN_DIAL = G_create_value_object(value_pos, value_text, 
				      direction_value, value, 
				      min_value,
				      max_value, value_colors, 
				      value_font);
   
 }


 /******** TILT_DIAL *****************************/
 {
   static float value_pos[]            = {5.5, 5.7, 0.5, 5.4};
   static char *value_text             = "tilt";
   static int value_font               = 3;
   int direction_value                 = 2;
   static float value                  = 1.0;
   float min_value                     = MIN_TILT_ANGLE;
   float max_value                     = MAX_TILT_ANGLE;
   static int value_colors[]           = {C_GREY70, C_BLUE, C_GREY30,
					  NO_COLOR}; 
   
   TILT_DIAL = G_create_value_object(value_pos, value_text, 
				      direction_value, value, 
				      min_value,
				      max_value, value_colors, 
				      value_font);
   
 }


 

 /******** IMAGE_MARKERS **************************************/
 {
   static float pos_r[]                   = {5.8, 11.5, 0.8, 5.4};
   /* static float pos_r[]                   = {5.0, 11.0, 0.5, 5.4}; */
   int num_mar                            = 3;
   static char *text_mar[]                = {"", "", ""};
   int connected_mar                      = 1;
   static int mar_frame_color             = NO_COLOR;
   static int mar_background_color        = NO_COLOR;
   static int mar_foreground_color[]      = {C_RED, C_YELLOW, C_GREY50};
   static int mar_text_color[]            = {NO_COLOR, NO_COLOR, NO_COLOR};
   static int mar_fonts[]                 = {2, 2, 2};
      
   IMAGE_MARKERS =
     G_create_markers_object(pos_r, connected_mar, 
			     1.0 / 70.0,
			     0.0, 1.0, 0.0, 1.0,
			     num_mar, text_mar, mar_background_color, 
			     mar_frame_color, mar_foreground_color, 
			     mar_text_color, mar_fonts);
 }


 {
   /******** QUIT_BUTTON ****************************/
   int switch_num                      = 2;
   static float switch_pos[]           = {(LEFT_MARGIN), 0.0, 5.5, 6.0};
   static char *switch_texts[]         = {"QUIT", "QUIT"};
   static int switch_fonts[]           = {2,2};
   static int switch_background_color[]= {C_GREY90, C_RED};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_WHITE};
   if (graphics_max_x < BUTTON_MAX_X)
     switch_pos[1] = BUTTON_MAX_X;
   else
     switch_pos[1] = graphics_max_x;

   QUIT_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
			      switch_background_color,switch_frame_color, 
			      switch_text_color, switch_fonts);
 }
 
 {
   /******** DATABASE_SIZE_BUTTON ****************************/
   int switch_num                      = 1;
   static float switch_pos[4];
   static char *switch_texts[]         = {"0 items"};
   static int switch_fonts[]           = {1};
   static int switch_background_color[]= {C_GREY90};
   static int switch_frame_color[]     = {C_GREY30};
   static int switch_text_color[]      = {C_BLACK};
   SET_BUTTON(0,0);

   DATABASE_SIZE_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
			      switch_background_color,switch_frame_color, 
			      switch_text_color, switch_fonts);
 }


 {
   /******** DISPLAY_MODUS_BUTTON ****************************/
   int switch_num                      = 2;
   static float switch_pos[4];
   static char *switch_texts[]         = {"display (off)", "display (on)"};
   static int switch_fonts[]           = {1,1};
   static int switch_background_color[]= {C_GREY90, C_YELLOW};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_BLACK};
   SET_BUTTON(0,1);

   DISPLAY_MODUS_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }



 {
   
   /******** RECORD_BUTTON ****************************/
   int switch_num                      = 4;
   static float switch_pos[4];
   static char *switch_texts[]         = 
     {"recording (off)", "...pausing", "recording (on)", 
      "recording (contin.)"};
   static int switch_fonts[]           = {1,1,1,1};
   static int switch_background_color[]= {C_GREY90, C_YELLOW, C_RED, C_RED};
   static int switch_frame_color[]     = 
   {C_GREY30, C_GREY70, C_GREY70, C_GREY70};
   static int switch_text_color[]      = 
   {C_BLACK, C_BLACK, C_WHITE, C_WHITE};
   SET_BUTTON(0,2);

   RECORD_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }

 {
   
   /******** EXECUTION_BUTTON ****************************/
   int switch_num                      = 2;
   static float switch_pos[4];
   static char *switch_texts[]         = 
     {"execution (off)", "execution (on)"};
   static int switch_fonts[]           = {1,1};
   static int switch_background_color[]= {C_GREY90, C_RED};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_WHITE};
   SET_BUTTON(0,3);

   EXECUTION_BUTTON 
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }


 {
   
   /******** LOGGING_BUTTON ****************************/
   int switch_num                      = 3;
   static float switch_pos[4];
   static char *switch_texts[]         = 
     {"logging (off)", "logging (on)", "error"};
   static int switch_fonts[]           = {1,1,1};
   static int switch_background_color[]= {C_GREY90, C_YELLOW, C_RED};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_BLACK, C_WHITE};
   SET_BUTTON(0,4);

   LOGGING_BUTTON 
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }


 {
   
   /******** BASE_CONNECT_BUTTON ****************************/
   int switch_num                      = 4;
   static float switch_pos[4];
   static char *switch_texts[]         = 
     {"no baseServer", "?? baseServer ??", "baseServer", "-> baseServer"};
   static int switch_fonts[]           = {1,1,1,1};
   static int switch_background_color[]= {C_GREY90, C_RED, C_LAWNGREEN, C_YELLOW};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70, C_GREY70, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_WHITE, C_BLACK, C_BLACK};
   SET_BUTTON(1,0);

   BASE_CONNECT_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }

 {
   
   /******** CAMERA_CONNECT_BUTTON ****************************/
   int switch_num                      = 4;
   static float switch_pos[4];
   static char *switch_texts[]         = 
     {"no cameraServer", "?? cameraServer ??", "cameraServer", "-> cameraServer"};
   static int switch_fonts[]           = {1,1,1,1};
   static int switch_background_color[]= {C_GREY90, C_RED, C_LAWNGREEN, C_YELLOW};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70, C_GREY70, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_WHITE, C_BLACK, C_BLACK};
   SET_BUTTON(1,1);

   CAMERA_CONNECT_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }

 {
   
   /******** COLLI_CONNECT_BUTTON ****************************/
   int switch_num                      = 4;
   static float switch_pos[4];
   static char *switch_texts[]         = 
     {"no colliServer", "?? colliServer ??", "colliServer", "-> colliServer"};
   static int switch_fonts[]           = {1,1,1,1};
   static int switch_background_color[]= {C_GREY90, C_RED, C_LAWNGREEN, C_YELLOW};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70, C_GREY70, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_WHITE, C_BLACK, C_BLACK};
   SET_BUTTON(1,2);

   COLLI_CONNECT_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }


 {
   
   /******** BUTTONS_CONNECT_BUTTON ****************************/
   int switch_num                      = 4;
   static float switch_pos[4];
   static char *switch_texts[]         = 
     {"no buttonServer", "?? buttonServer ??", "buttonServer", "-> buttonServer"};
   static int switch_fonts[]           = {1,1,1,1};
   static int switch_background_color[]= {C_GREY90, C_RED, C_LAWNGREEN, C_YELLOW};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70, C_GREY70, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_WHITE, C_BLACK, C_BLACK};
   SET_BUTTON(1,3);

   BUTTONS_CONNECT_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }


 {
   
   /******** PANTILT_CONNECT_BUTTON ****************************/
   int switch_num                      = 4;
   static float switch_pos[4];
   static char *switch_texts[]         = 
     {"no pantiltServer", "?? pantiltServer ??", "pantiltServer", "-> pantiltServer"};
   static int switch_fonts[]           = {1,1,1,1};
   static int switch_background_color[]= {C_GREY90, C_RED, C_GREY90, C_YELLOW};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70, C_GREY70, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_WHITE, C_BLACK, C_BLACK};
   SET_BUTTON(1,4);

   PANTILT_CONNECT_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }


 {
   /******** DIAL_BUTTON *******************************************/

   static float switch_pos[4];
   static char *value_text             = "pattern 0 (of 0)";
   static int value_font               = 1;
   static int value_colors[]           = 
     {C_GREY90, C_GREY60, C_GREY30, C_BLACK}; 
   SET_BAR(1);

   DIAL_BUTTON = 
     G_create_value_object(switch_pos, value_text, 1, 0.0, 0.0, 1.0, 
			   value_colors, value_font);
 }


 {
   
   /******** FIRST_BUTTON ****************************/
   int switch_num                      = 1;
   static float switch_pos[4];
   static char *switch_texts[]         = {"<<"};
   static int switch_fonts[]           = {1};
   static int switch_background_color[]= {C_GREY90};
   static int switch_frame_color[]     = {C_GREY30};
   static int switch_text_color[]      = {C_BLACK};
   SET_BAR_BUTTON1(1);

   FIRST_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }


 {
   
   /******** PREVIOUS_BUTTON ****************************/
   int switch_num                      = 1;
   static float switch_pos[4];
   static char *switch_texts[]         = {"<"};
   static int switch_fonts[]           = {1};
   static int switch_background_color[]= {C_GREY90};
   static int switch_frame_color[]     = {C_GREY30};
   static int switch_text_color[]      = {C_BLACK};
   SET_BAR_BUTTON2(1);

   PREVIOUS_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }


 {
   
   /******** NEXT_BUTTON ****************************/
   int switch_num                      = 1;
   static float switch_pos[4];
   static char *switch_texts[]         = {">"};
   static int switch_fonts[]           = {1};
   static int switch_background_color[]= {C_GREY90};
   static int switch_frame_color[]     = {C_GREY30};
   static int switch_text_color[]      = {C_BLACK};
   SET_BAR_BUTTON3(1);


   NEXT_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }

 {
   
   /******** MOVIE_BUTTON ****************************/
   int switch_num                      = 3;
   static float switch_pos[4];
   static char *switch_texts[]         = 
   {"movie", "movie", "movie"};
   static int switch_fonts[]           = {1,1,1};
   static int switch_background_color[]= {C_GREY90, C_YELLOW, C_GREY90};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70, C_GREY30};
   static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};
   SET_BAR_BUTTON4(1);


   MOVIE_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }

 {
   /******** SET_DIAL_BUTTON *******************************************/

   static float switch_pos[4];
   static char *value_text             = "pattern set 0 (of 0)";
   static int value_font               = 1;
   static int value_colors[]           = 
     {C_GREY90, C_GREY60, C_GREY30, C_BLACK}; 
   SET_BAR(0);

   SET_DIAL_BUTTON = 
     G_create_value_object(switch_pos, value_text, 1, 0.0, 0.0, 1.0, 
			   value_colors, value_font);
 }


 {
   
   /******** SET_FIRST_BUTTON ****************************/
   int switch_num                      = 1;
   static float switch_pos[4];
   static char *switch_texts[]         = {"<<"};
   static int switch_fonts[]           = {1};
   static int switch_background_color[]= {C_GREY90};
   static int switch_frame_color[]     = {C_GREY30};
   static int switch_text_color[]      = {C_BLACK};
   SET_BAR_BUTTON1(0);


   SET_FIRST_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }


 {
   
   /******** SET_PREVIOUS_BUTTON ****************************/
   int switch_num                      = 1;
   static float switch_pos[4];
   static char *switch_texts[]         = {"<"};
   static int switch_fonts[]           = {1};
   static int switch_background_color[]= {C_GREY90};
   static int switch_frame_color[]     = {C_GREY30};
   static int switch_text_color[]      = {C_BLACK};
   SET_BAR_BUTTON2(0);


   SET_PREVIOUS_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }


 {
   
   /******** SET_NEXT_BUTTON ****************************/
   int switch_num                      = 1;
   static float switch_pos[4];
   static char *switch_texts[]         = {">"};
   static int switch_fonts[]           = {1};
   static int switch_background_color[]= {C_GREY90};
   static int switch_frame_color[]     = {C_GREY30};
   static int switch_text_color[]      = {C_BLACK};
   SET_BAR_BUTTON3(0);


   SET_NEXT_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }


 {
   
   /******** SET_MOVIE_BUTTON ****************************/
   int switch_num                      = 3;
   static float switch_pos[4];
   static char *switch_texts[]         = {"movie", "movie", "movie"};
   static int switch_fonts[]           = {1,1,1};
   static int switch_background_color[]= {C_GREY90, C_GREY90, C_YELLOW};
   static int switch_frame_color[]     = {C_GREY30, C_GREY30, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};
   SET_BAR_BUTTON4(0);


   SET_MOVIE_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }

 {
   
   /******** LOAD_BUTTON ****************************/
   int switch_num                      = 3;
   static float switch_pos[4];
   static char *switch_texts[]         = {"load data", "...loading data", "error"};
   static int switch_fonts[]           = {1,1,1};
   static int switch_background_color[]= {C_GREY90, C_YELLOW, C_RED};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};
   SET_BUTTON(2, 0);

   LOAD_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }

 {
   
   /******** APPEND_BUTTON ****************************/
   int switch_num                      = 3;
   static float switch_pos[4];
   static char *switch_texts[]         = {"append data", "...appending data", "error"};
   static int switch_fonts[]           = {1,1,1};
   static int switch_background_color[]= {C_GREY90, C_YELLOW, C_RED};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};
   SET_BUTTON(2, 1);

   APPEND_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }

 {
   
   /******** SAVE_BUTTON ****************************/
   int switch_num                      = 3;
   static float switch_pos[4];
   static char *switch_texts[]         = {"save data", "...saving data", "error"};
   static int switch_fonts[]           = {1,1,1};
   static int switch_background_color[]= {C_GREY90, C_YELLOW, C_RED};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};
   SET_BUTTON(2, 2);

   SAVE_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }





 {
   
   /******** MARK_BUTTON ****************************/
   int switch_num                      = 2;
   static float switch_pos[4];
   static char *switch_texts[]         = 
   {"labeling (off)", "labeling (on)"};
   static int switch_fonts[]           = {1,1};
   static int switch_background_color[]= {C_GREY90, C_YELLOW};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_BLACK};
   SET_BUTTON(3,0);

   MARK_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }

 {
   
   /******** TRAIN_BUTTON ****************************/
   int switch_num                      = 2;
   static float switch_pos[4];
   static char *switch_texts[]         = 
   {"learning (off)", "learning (on)"};
   static int switch_fonts[]           = {1,1};
   static int switch_background_color[]= {C_GREY90, C_YELLOW};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_BLACK};
   SET_BUTTON(3,1);

   TRAIN_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }
 {
   
   /******** TEST_BUTTON ****************************/
   int switch_num                      = 2;
   static float switch_pos[4];
   static char *switch_texts[]         = 
   {"internal test", "testing"};
   static int switch_fonts[]           = {1,1};
   static int switch_background_color[]= {C_GREY90, C_RED};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_WHITE};
   SET_BUTTON(3,2);

   TEST_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }
 {
   
   /******** LOAD_NET_BUTTON ****************************/
   int switch_num                      = 3;
   static float switch_pos[4];
   static char *switch_texts[]         = {"load parameters", "...loading parameters", "error"};
   static int switch_fonts[]           = {1,1,1};
   static int switch_background_color[]= {C_GREY90, C_YELLOW, C_RED};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};
   SET_BUTTON(3, 3);

   LOAD_NET_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }


 {
   
   /******** SAVE_NET_BUTTON ****************************/
   int switch_num                      = 3;
   static float switch_pos[4];
   static char *switch_texts[]         = {"save parameters", "...saving parameters", "error"};
   static int switch_fonts[]           = {1,1,1};
   static int switch_background_color[]= {C_GREY90, C_YELLOW, C_RED};
   static int switch_frame_color[]     = {C_GREY30, C_GREY70, C_GREY70};
   static int switch_text_color[]      = {C_BLACK, C_BLACK, C_BLACK};
   SET_BUTTON(3, 4);

   SAVE_NET_BUTTON
     = G_create_switch_object(switch_pos, switch_num, switch_texts,
		switch_background_color,switch_frame_color, 
		switch_text_color, switch_fonts);
 }


 /* =============================================================
    ====================  2) DISPLAY  ===========================
    ============================================================= */
 
 G_set_matrix_display_style(1);
 if (!global_prob_c){
   G_deactivate(SAVE_NET_BUTTON);
   G_deactivate(LOAD_NET_BUTTON);
   G_deactivate(TEST_BUTTON);
   G_deactivate(TRAIN_BUTTON);
   G_deactivate(EXECUTION_BUTTON);
   G_deactivate(LOGGING_BUTTON);
 }

 G_display_all();
 G_display_switch(RECORD_BUTTON, global_modus_recording);
 G_display_switch(EXECUTION_BUTTON, global_modus_execution);
 G_display_switch(LOGGING_BUTTON, global_modus_logging);
 G_display_switch(DISPLAY_MODUS_BUTTON, global_modus_live);
 global_graphics_initialized = 1;


 if (automatic_base_connect)
   G_display_switch(BASE_CONNECT_BUTTON, 3);
 if (automatic_camera_connect)
   G_display_switch(CAMERA_CONNECT_BUTTON, 3);
 if (automatic_colli_connect)
   G_display_switch(COLLI_CONNECT_BUTTON, 3);
 if (automatic_buttons_connect)
   G_display_switch(BUTTONS_CONNECT_BUTTON, 0);
 if (automatic_pantilt_connect)
   G_display_switch(PANTILT_CONNECT_BUTTON, 3);

}




/************************************************************************
 *
 *   NAME:         mouse_test_loop
 *                 
 *   FUNCTION:     Checks mouse events and changes the variables
 *                 "action" and "Program_state" correspondingly
 *                 
 *                 
 ************************************************************************/




int 
mouse_test_loop()
{
  G_mouse_ptr mouse_events;
  int num_mouse_events, button;
  float mouse_x, mouse_y;
  int number;
  int mouse_event_detected;
  buttonStatusType buttons;
  
  buttons.red_light_status                   = 0;
  buttons.yellow_light_status                = 0;
  buttons.green_light_status                 = 0;
  buttons.blue_light_status                  = 0;
  buttons.left_kill_switch_light_status      = 0;
  buttons.right_kill_switch_light_status     = 0;
  buttons.red_button_pressed                 = 0;
  buttons.red_button_changed                 = 0;
  buttons.yellow_button_pressed              = 0;
  buttons.yellow_button_changed              = 0;
  buttons.green_button_pressed               = 0;
  buttons.green_button_changed               = 0;
  buttons.blue_button_pressed                = 0;
  buttons.blue_button_changed                = 0;




  /****************** CHECK FOR MOUSE EVENT *******************/

  mouse_event_detected = G_test_mouse(0);

  if (mouse_event_detected){	/* reacts to button events and
				 * motion events */

    mouse_events = G_evaluate_mouse(&mouse_x, &mouse_y, 
				    &button, &num_mouse_events);

    if (mouse_event_detected == 1){ /* reglar mouse press */


      
      
      /****************** EVALUATE MOUSE EVENT *******************/
      
      if (global_modus_training){
	if (G_mouse_event_at(TRAIN_BUTTON, mouse_events, &number)){
	  global_modus_training ^= 1;
	  G_display_switch(TRAIN_BUTTON, global_modus_training);
	}
      }
      
      else{
	
	
	/* -------------------------------------------------- *
	 *            quit
	 * -------------------------------------------------- */
	
	
	if (G_mouse_event_at(QUIT_BUTTON, mouse_events, &number)){
	  G_display_switch(QUIT_BUTTON, 1);
	  if (global_modus_movie){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  fprintf(stderr, "\n");
	  usleep(200000);
	  exit(0);
	}
	
	/* -------------------------------------------------- *
	 *            database handling
	 * -------------------------------------------------- */
	
	else if (G_mouse_event_at(DATABASE_SIZE_BUTTON, mouse_events,
				  &number)){
	  G_display_switch(DATABASE_SIZE_BUTTON, 0);
	}
	
	else if (G_mouse_event_at(DISPLAY_MODUS_BUTTON, 
				  mouse_events, &number)){
	  global_modus_live ^= 1;
	  G_display_switch(DISPLAY_MODUS_BUTTON, global_modus_live);
	  if (global_modus_movie){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	}
	
	
	else if (G_mouse_event_at(FIRST_BUTTON, mouse_events, &number) &&
		 !global_modus_execution && !global_modus_recording){
	  if (global_modus_live){	/* switch live modus off */
	    global_modus_live = 0;
	    G_display_switch(DISPLAY_MODUS_BUTTON, global_modus_live);
	  }
	  if (global_modus_movie && button == LEFT_BUTTON){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  mem_display_nth_pattern(current_patternset, 0);
	}
	
	
	else if (G_mouse_event_at(PREVIOUS_BUTTON, mouse_events, &number) &&
		 !global_modus_execution && !global_modus_recording){
	  if (global_modus_live){	/* switch live modus off */
	    global_modus_live = 0;
	    G_display_switch(DISPLAY_MODUS_BUTTON, global_modus_live);
	  }
	  if (global_modus_movie && button == LEFT_BUTTON){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  mem_display_previous_pattern();
	}
	
	
	else if (G_mouse_event_at(NEXT_BUTTON, mouse_events, &number) &&
		 !global_modus_execution && !global_modus_recording){
	  if (global_modus_movie && button == LEFT_BUTTON){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  mem_display_next_pattern();
	}
	
	
	
	else if (G_mouse_event_at(MOVIE_BUTTON, mouse_events, &number) &&
		 !global_modus_execution && !global_modus_recording){
	  if (global_modus_live){	/* switch live modus off */
	    global_modus_live = 0;
	    G_display_switch(DISPLAY_MODUS_BUTTON, global_modus_live);
	  }
	  if (!global_modus_movie && data != NULL){
	    if (mem_query_last_pattern()){
	      mem_display_nth_pattern(current_patternset, 0);
	      /*X_initialize_episode();*/
	      /*X_process_pattern(current_pattern);*/
	      /*print_neuro_net(0);*/
	    }
	    if (mem_query_last_pattern())
	      global_modus_movie = 0;
	    else
	      global_modus_movie = 1;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	    gettimeofday(&global_movie_start_time, NULL);
	    if (current_pattern && current_pattern->time){
	      global_movie_start_time_tag.tv_sec  = current_pattern->time->tv_sec;
	      global_movie_start_time_tag.tv_usec = current_pattern->time->tv_usec;
	    }
	    else{
	      global_movie_start_time_tag.tv_sec  = 0;
	      global_movie_start_time_tag.tv_usec = 0;
	    }
	    if (button == LEFT_BUTTON)
	      global_modus_real_time_display = 1;
	    else
	      global_modus_real_time_display = 0;
	  }
	  else{
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	}
	
	else if (G_mouse_event_at(DIAL_BUTTON, mouse_events, &number) &&
		 !global_modus_execution && !global_modus_recording){
	  if (global_modus_live){	/* switch live modus off */
	    global_modus_live = 0;
	    G_display_switch(DISPLAY_MODUS_BUTTON, global_modus_live);
	  }
	  if (global_modus_movie && button == LEFT_BUTTON){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  if (current_patternset != NULL)
	    mem_display_nth_pattern(current_patternset, 
				    (int) (mouse_events[number].value
					   * ((float) 
					      current_patternset->num_patterns)));
	}
	
	
	else if (G_mouse_event_at(SET_FIRST_BUTTON, mouse_events, &number) &&
		 !global_modus_execution && !global_modus_recording){
	  if (global_modus_live){	/* switch live modus off */
	    global_modus_live = 0;
	    G_display_switch(DISPLAY_MODUS_BUTTON, global_modus_live);
	  }
	  if (global_modus_movie && button == LEFT_BUTTON){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  mem_display_nth_patternset(0);
	}
	
	
	else if (G_mouse_event_at(SET_PREVIOUS_BUTTON, mouse_events, &number) &&
		 !global_modus_execution && !global_modus_recording){
	  if (global_modus_live){	/* switch live modus off */
	    global_modus_live = 0;
	    G_display_switch(DISPLAY_MODUS_BUTTON, global_modus_live);
	  }
	  if (global_modus_movie && button == LEFT_BUTTON){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  mem_display_previous_patternset();
	}
	
	
	else if (G_mouse_event_at(SET_NEXT_BUTTON, mouse_events, &number) &&
		 !global_modus_execution && !global_modus_recording){
	  if (global_modus_movie && button == LEFT_BUTTON){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  mem_display_next_patternset();
	}
	
	
	
	else if (G_mouse_event_at(SET_MOVIE_BUTTON, mouse_events, &number) &&
		 !global_modus_execution && !global_modus_recording){
	  if (global_modus_live){	/* switch live modus off */
	    global_modus_live = 0;
	    G_display_switch(DISPLAY_MODUS_BUTTON, global_modus_live);
	  }
	  if (!global_modus_movie && data != NULL){
	    if (mem_query_last_pattern() && mem_query_last_patternset()){
	      mem_display_nth_patternset(0);
	      X_initialize_episode();
	      /*X_process_pattern(current_pattern);*/
	    }
	    if (mem_query_last_pattern() && mem_query_last_patternset())
	      global_modus_movie = 0;
	    else
	      global_modus_movie = 2;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	    gettimeofday(&global_movie_start_time, NULL);
	    if (current_pattern && current_pattern->time){
	      global_movie_start_time_tag.tv_sec  = current_pattern->time->tv_sec;
	      global_movie_start_time_tag.tv_usec = current_pattern->time->tv_usec;
	    }
	    else{
	      global_movie_start_time_tag.tv_sec  = 0;
	      global_movie_start_time_tag.tv_usec = 0;
	    }
	    if (button == LEFT_BUTTON)
	      global_modus_real_time_display = 1;
	    else
	      global_modus_real_time_display = 0;
	  }
	  else{
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	}
	
	
	else if (G_mouse_event_at(SET_DIAL_BUTTON, mouse_events, &number) &&
		 !global_modus_execution && !global_modus_recording){
	  if (global_modus_live){	/* switch live modus off */
	    global_modus_live = 0;
	    G_display_switch(DISPLAY_MODUS_BUTTON, global_modus_live);
	  }
	  if (global_modus_movie && button == LEFT_BUTTON){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  if (data != NULL)
	    mem_display_nth_patternset((int) (mouse_events[number].value
					      * ((float) 
						 data->num_sets)));
	}
	
	/* -------------------------------------------------- *
	 *            loading and saving from/to file
	 * -------------------------------------------------- */
	
	else if (G_mouse_event_at(SAVE_BUTTON, mouse_events, &number)){
	  if (global_modus_movie && button == LEFT_BUTTON){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  if (global_modus_live){	/* switch live modus off */
	    global_modus_live = 0;
	    G_display_switch(DISPLAY_MODUS_BUTTON, global_modus_live);
	  }
	  mem_save_patterns(DATA_FILENAME2, SAVE_BUTTON);
	}
	
	else if (G_mouse_event_at(LOAD_BUTTON, mouse_events, &number)){
	  if (global_modus_movie && button == LEFT_BUTTON){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  if (global_modus_live){	/* switch live modus off */
	    global_modus_live = 0;
	    G_display_switch(DISPLAY_MODUS_BUTTON, global_modus_live);
	  }
	  mem_load_patterns(DATA_FILENAME, 0, LOAD_BUTTON);
	  update_items_button();
	  mem_display_nth_patternset(0);
	}
	
	
	else if (G_mouse_event_at(APPEND_BUTTON, mouse_events, &number)){
	  if (global_modus_movie && button == LEFT_BUTTON){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  if (global_modus_live){	/* switch live modus off */
	    global_modus_live = 0;
	    G_display_switch(DISPLAY_MODUS_BUTTON, global_modus_live);
	  }
	  mem_load_patterns(DATA_FILENAME, 1, APPEND_BUTTON);
	  update_items_button();
	  mem_display_nth_patternset(0);
	}
	
	else if (G_mouse_event_at(SAVE_NET_BUTTON, mouse_events, &number)){
	  if (global_modus_movie && button == LEFT_BUTTON){ /* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  if (global_modus_live){	/* switch live modus off */
	    global_modus_live = 0;
	    G_display_switch(DISPLAY_MODUS_BUTTON, global_modus_live);
	  }
	  X_save_parameters(NET_SAVE_FILENAME, 
			    NET_BEST_SAVE_FILENAME, 
			    0,		/* 0 = don't consider timer */
			    1);		/* 1 = display buttons */
	}
	
	else if (G_mouse_event_at(LOAD_NET_BUTTON, mouse_events, &number)){
	  if (global_modus_movie && button == LEFT_BUTTON){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  if (global_modus_live){	/* switch live modus off */
	    global_modus_live = 0;
	    G_display_switch(DISPLAY_MODUS_BUTTON, global_modus_live);
	  }
	  X_load_parameters(NET_LOAD_FILENAME);
	}
	
	
	/* -------------------------------------------------- *
	 *            recording modus
	 * -------------------------------------------------- */
	
	
	else if (G_mouse_event_at(RECORD_BUTTON, mouse_events, &number)){
	  if (global_modus_movie){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  if (global_modus_recording)
	    global_modus_recording = 0;
	  else if (baseConnected)
	    global_modus_recording = 1;
	  else
	    global_modus_recording = 2;
	    if (button == RIGHT_BUTTON)
	      G_display_switch(RECORD_BUTTON, 3);
	    else
	      G_display_switch(RECORD_BUTTON, global_modus_recording);
	  if (global_modus_recording){
	    if (button == RIGHT_BUTTON)
	      global_modus_suspend_recording_when_not_moving = 0;
	    else
	      global_modus_suspend_recording_when_not_moving = 1;
	    current_patternset = mem_create_patternset();
	    mem_fill_pattern_set_slot(current_patternset, "recorded", 0);
	    first_irs = 1;
	    first_sonars = 1;
	    first_tactiles = 1;
	    first_lasers = 1;
	    gettimeofday(&last_autosave_time, NULL);
	  }
	}

	
	/* -------------------------------------------------- *
	 *            execution modus
	 * -------------------------------------------------- */
	
	
	else if (G_mouse_event_at(EXECUTION_BUTTON, mouse_events, &number)){
	  if (global_modus_movie){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  global_modus_execution ^= 1;
	  G_display_switch(EXECUTION_BUTTON, global_modus_execution);
	  if (global_modus_execution)
	    X_initialize_episode();
	}

	
	/* -------------------------------------------------- *
	 *            logging modus
	 * -------------------------------------------------- */
	
	
	else if (G_mouse_event_at(LOGGING_BUTTON, mouse_events, &number)){
	  if (global_modus_movie){	/* switch movie off */
	    global_modus_movie = 0;
	    G_display_switch(MOVIE_BUTTON, global_modus_movie);
	    G_display_switch(SET_MOVIE_BUTTON, global_modus_movie);
	  }
	  if (!global_modus_logging)
	    X_logging_on();
	  else
	    X_logging_off();
	}

	
	/* -------------------------------------------------- *
	 *            marking modus
	 * -------------------------------------------------- */
	
	else if (G_mouse_event_at(MARK_BUTTON, mouse_events, &number)){
	  global_modus_marking ^= 1;
	  G_display_switch(MARK_BUTTON, global_modus_marking);
	  if (global_modus_marking && global_modus_control){
	    global_modus_control = 0;
	    G_display_switch(CONTROL_FRAME, global_modus_control);
	  }
	}
	
	/* -------------------------------------------------- *
	 *            learning / testing
	 * -------------------------------------------------- */

	else if (G_mouse_event_at(TRAIN_BUTTON, mouse_events, &number)){
	  global_modus_training ^= 1;
	  G_display_switch(TRAIN_BUTTON, global_modus_training);
	}
	
	
	
	else if (G_mouse_event_at(TEST_BUTTON, mouse_events, &number)){
	  if (button == RIGHT_BUTTON)
	    X_testing(1);
	  else
	    X_testing(0);
	}
	
	
	/* -------------------------------------------------- *
	 *            auto-connect
	 * -------------------------------------------------- */

	else if (G_mouse_event_at(BASE_CONNECT_BUTTON,
				  mouse_events, &number)){	
	  if (automatic_base_connect){
	    automatic_base_connect = 0;
	    G_display_switch(BASE_CONNECT_BUTTON, 0);
	    baseConnected = 0;
	  }
	  else{
	    automatic_base_connect = 1;
	    if (!baseConnected)
	      G_display_switch(BASE_CONNECT_BUTTON, 3);
	    else
	      G_display_switch(BASE_CONNECT_BUTTON, 2);
	  }
	}
	else if (G_mouse_event_at(CAMERA_CONNECT_BUTTON,
				  mouse_events, &number)){	
	  if (automatic_camera_connect){
	    automatic_camera_connect = 0;
	    G_display_switch(CAMERA_CONNECT_BUTTON, 0);
	    cameraConnected = 0;
	  }
	  else{
	    automatic_camera_connect = 1;
	    if (!cameraConnected)
	      G_display_switch(CAMERA_CONNECT_BUTTON, 3);
	    else
	      G_display_switch(CAMERA_CONNECT_BUTTON, 2);
	  }
	}
	else if (G_mouse_event_at(PANTILT_CONNECT_BUTTON,
				  mouse_events, &number)){	
	  if (automatic_pantilt_connect){
	    automatic_pantilt_connect = 0;
	    G_display_switch(PANTILT_CONNECT_BUTTON, 0);
	    ptConnected = 0;
	  }
	  else{
	    automatic_pantilt_connect = 1;
	    if (!ptConnected)
	      G_display_switch(PANTILT_CONNECT_BUTTON, 3);
	    else
	      G_display_switch(PANTILT_CONNECT_BUTTON, 2);
	  }
	}
	  
	else if (G_mouse_event_at(COLLI_CONNECT_BUTTON,
				  mouse_events, &number)){	
	  if (automatic_colli_connect){
	    automatic_colli_connect = 0;
	    G_display_switch(COLLI_CONNECT_BUTTON, 0);
	    colliConnected = 0;
	  }
	  else{
	    automatic_colli_connect = 1;
	    if (!colliConnected)
	      G_display_switch(COLLI_CONNECT_BUTTON, 3);
	    else
	      G_display_switch(COLLI_CONNECT_BUTTON, 2);
	  }
	}
	else if (G_mouse_event_at(BUTTONS_CONNECT_BUTTON,
				  mouse_events, &number)){	
	  if (automatic_buttons_connect){
	    automatic_buttons_connect = 0;
	    G_display_switch(BUTTONS_CONNECT_BUTTON, 0);
	    buttonConnected = 0;
	  }
	  else{
	    automatic_buttons_connect = 1;
	    if (!buttonConnected)
	      G_display_switch(BUTTONS_CONNECT_BUTTON, 3);
	    else
	      G_display_switch(BUTTONS_CONNECT_BUTTON, 2);
	  }
	}
	
	/* -------------------------------------------------- *
	 *               direct control modus
	 * -------------------------------------------------- */
	
	
	else if (G_mouse_event_at(CONTROL_ROBOT1, mouse_events, &number)){
	  if (!global_modus_marking && button == LEFT_BUTTON){
	    if (!global_modus_control){
	      global_modus_control = 1;
	      G_display_switch(CONTROL_FRAME, global_modus_control);
	    }
	    set_velocity(mouse_events[number].value_y,
			 mouse_events[number].value_x, 0);
	  }
	  else if (button == RIGHT_BUTTON){
	    if (global_modus_control){
	      global_modus_control = 0;
	      G_display_switch(CONTROL_FRAME, global_modus_control);
	    }
	    set_velocity(0.0, 0.0, 1);
	  }
	  
	}

	/* -------------------------------------------------- *
	 *            buttons
	 * -------------------------------------------------- */

	
	else if (!buttonConnected && 
		 G_mouse_event_at(BUTTON_PUSHED[0], mouse_events, &number)){
	  G_display_switch(mouse_events[number].name,
			   mouse_events[number].actual_text+1);
	  buttons.red_button_pressed                 = 1;
	  buttons.red_button_changed                 = 1;
	  buttonCallback(&buttons);
	  buttons.red_button_pressed                 = 0;
	  usleep(200000);
	  buttonCallback(&buttons);
	  buttons.red_button_changed                 = 0;
	  G_display_switch(mouse_events[number].name,
			   mouse_events[number].actual_text);
	}
	else if (!buttonConnected && 
		 G_mouse_event_at(BUTTON_PUSHED[1], mouse_events, &number)){
	  G_display_switch(mouse_events[number].name,
			   mouse_events[number].actual_text+1);
	  buttons.yellow_button_pressed                 = 1;
	  buttons.yellow_button_changed                 = 1;
	  buttonCallback(&buttons);
	  buttons.yellow_button_pressed                 = 0;
	  usleep(200000);
	  buttonCallback(&buttons);
	  buttons.yellow_button_changed                 = 0;
	  G_display_switch(mouse_events[number].name,
			   mouse_events[number].actual_text);
	}
	else if (!buttonConnected && 
		 G_mouse_event_at(BUTTON_PUSHED[2], mouse_events, &number)){
	  G_display_switch(mouse_events[number].name,
			   mouse_events[number].actual_text+1);
	  buttons.green_button_pressed                 = 1;
	  buttons.green_button_changed                 = 1;
	  buttonCallback(&buttons);
	  buttons.green_button_pressed                 = 0;
	  usleep(200000);
	  buttonCallback(&buttons);
	  buttons.green_button_changed                 = 0;
	  G_display_switch(mouse_events[number].name,
			   mouse_events[number].actual_text);
	}
	else if (!buttonConnected && 
		 G_mouse_event_at(BUTTON_PUSHED[3], mouse_events, &number)){
	  G_display_switch(mouse_events[number].name,
			   mouse_events[number].actual_text+1);
	  buttons.blue_button_pressed                 = 1;
	  buttons.blue_button_changed                 = 1;
	  buttonCallback(&buttons);
	  buttons.blue_button_pressed                 = 0;
	  usleep(200000);
	  buttonCallback(&buttons);
	  buttons.blue_button_changed                 = 0;
	  G_display_switch(mouse_events[number].name,
			   mouse_events[number].actual_text);
	}

	
	
	/* -------------------------------------------------- *
	 *              Marking modus
	 * -------------------------------------------------- */
	
	else if (global_modus_marking &&
		 G_mouse_event_at(IMAGE_MARKERS, mouse_events, &number)){
	  /* fprintf(stderr, "Click in Image at %g %g.\n",
	     mouse_events[number].rel_x,
	     mouse_events[number].rel_y); */
	  mem_mark_point(IMAGE_WINDOW, IMAGE_MARKERS,
			 mouse_events[number].rel_x,
			 mouse_events[number].rel_y,
			 button);
	}
	else if (global_modus_marking &&
		 G_mouse_event_at(SONAR_MARKERS, mouse_events, &number)){
	  /* fprintf(stderr, "Click in Sonar Window at %g %g.\n",
	     mouse_events[number].value_x,
	     mouse_events[number].value_y); */
	  mem_mark_point(SONAR_WINDOW, SONAR_MARKERS,
			 mouse_events[number].value_y,
			 - mouse_events[number].value_x,
			 button);
	}
	else if (global_modus_marking &&
		 G_mouse_event_at(CONTROL_MARKERS, mouse_events, &number)){
	  /* fprintf(stderr, "Click in Control Window at %g %g.\n",
	     mouse_events[number].value_x,
	     mouse_events[number].value_y); */
	  mem_mark_point(CONTROL_WINDOW, CONTROL_MARKERS,
			 mouse_events[number].value_y,
			 - mouse_events[number].value_x,
			 button);
	}
	else if (global_modus_marking &&
		 G_mouse_event_at(IR_MARKERS, mouse_events, &number)){
	  /* fprintf(stderr, "Click in Infrared Window at %g %g.\n",
	     mouse_events[number].value_x,
	     mouse_events[number].value_y); */
	  mem_mark_point(IR_WINDOW, IR_MARKERS,
			 mouse_events[number].value_y,
			 - mouse_events[number].value_x,
			 button);
	}
	else if (global_modus_marking &&
		 G_mouse_event_at(LASER_MARKERS, mouse_events, &number)){
	  /* fprintf(stderr, "Click in Laser Window at %g %g.\n",
	     mouse_events[number].value_x,
	     mouse_events[number].value_y); */
	  mem_mark_point(LASER_WINDOW, LASER_MARKERS,
			 mouse_events[number].value_y,
			 - mouse_events[number].value_x,
			 button);
	}
	else if (global_modus_marking &&
		 G_mouse_event_at(TACTILE_MARKERS, mouse_events, &number)){
	  /* fprintf(stderr, "Click in Tactile Window at %g %g.\n",
	     mouse_events[number].value_x,
	     mouse_events[number].value_y); */
	  mem_mark_point(TACTILE_WINDOW, TACTILE_MARKERS,
			 mouse_events[number].value_y,
			 - mouse_events[number].value_x,
			 button);
	}
	
	/* insert new graphics object here XXXXX */
	
	
	else if (button == RIGHT_BUTTON){
	  G_display_all();
	}
      }
      
    }
    else{			/* mouse motion only */
	/* -------------------------------------------------- *
	 *               direct control modus
	 * -------------------------------------------------- */

      if (global_modus_control){
	if (G_mouse_event_at(CONTROL_ROBOT1, mouse_events, &number)){
	  set_velocity(mouse_events[number].value_y,
		       mouse_events[number].value_x, 0);
	}
	else if (global_modus_control){
	  global_modus_control = 0;
	  set_velocity(0.0, 0.0, 1);
	  G_display_switch(CONTROL_FRAME, global_modus_control);
	}
      }
    }
  }

  
  return mouse_event_detected;
}

