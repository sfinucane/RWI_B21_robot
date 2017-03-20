
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/learn/mem.c,v $
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
 * $Log: mem.c,v $
 * Revision 1.1  2002/09/14 16:36:05  rstone
 * *** empty log message ***
 *
 * Revision 1.21  1999/07/11 18:48:34  thrun
 * slight reorganization
 *
 * Revision 1.20  1998/06/20 21:05:32  thrun
 * Reorganization (created: fa.{c,h}, xtypes.h)
 *
 * Revision 1.19  1998/05/05 04:00:35  thrun
 * Direct control mode (and button). Other stuff I don't fully remember.
 * Intermediate version.
 *
 * Revision 1.18  1998/04/18 22:55:41  thrun
 * nice intermediate version, no apparent bugs
 *
 * Revision 1.17  1998/04/18 20:42:26  thrun
 * 1. neuro net can now be called with the same variable as input
 * and output
 * 2. direct control modus for the robot. controls are
 * now generated exclusively through the display (left upper corner)
 *
 * Revision 1.16  1997/10/05 18:11:19  thrun
 * new data library "libdat.a"
 *
 * Revision 1.15  1997/08/05 04:15:23  thrun
 * (1) avg_value are saved for each stoch variable, (2) enhanced replay
 * functions.
 *
 * Revision 1.14  1997/07/30 21:02:03  thrun
 * baseServer now sends confirmations of all motion commands. Is used
 * for logging in the module "learn"
 *
 * Revision 1.13  1997/07/30 03:46:53  thrun
 * New events: pantilt and buttons
 *
 * Revision 1.12  1997/07/07 04:45:24  thrun
 * Now with training and testing set support
 *
 * Revision 1.11  1997/07/06 18:42:05  thrun
 * Radial basis networks implemented.
 *
 * Revision 1.10  1997/07/04 00:28:55  thrun
 * Intermediate verson of the learning stuff. What works is:
 * - training of deterministic variables (incl. chaining)
 * - training of multiple determinstic with a stochastic variables (incl chaining)
 * What doesn't yet work includes
 * - temporal chaining of stochastic variables, using Bayes rule
 * - convolving: gradients
 * - stochatic input
 *
 * Revision 1.9  1997/06/29 04:04:56  thrun
 * intrmediate version - not really good for anything
 *
 * Revision 1.8  1997/06/28 13:41:43  thrun
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

#include "bUtils.h"
#include "EZX11.h"
#include "o-graphics.h"
#include "dat.h"
#include "mem.h"
#include "global.h"
#include "mem.h"
#include "xtypes.h"
#include "vars.h"
#include "fa.h"
#include "robot_specifications.h"









/************************************************************************
 *
 *   NAME:         update_items_button()
 *                 
 *   FUNCTION:     updates the button that displays number of items in data
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void
update_items_button()
{
  char num_pat_txt[128];

  if (data)
    sprintf(num_pat_txt, "%d items", data->total_number_patterns);
  else
    sprintf(num_pat_txt, "0 items");
  G_set_new_text(DATABASE_SIZE_BUTTON, num_pat_txt, 0);
  G_display_switch(DATABASE_SIZE_BUTTON, 0);
}










/************************************************************************
 *
 *   NAME:         mem_display_pos()
 *                 
 *   FUNCTION:     displays a position at the screen
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/


void
mem_display_pos(robot_position_type *pos)
{
  char data_txt[128];

  if (pos)
    sprintf(data_txt, "pos: %4.1f %4.1f %3.1f", 
	    pos->x, pos->y, pos->orientation);
  else
    sprintf(data_txt, "pos: ???? ???? ????");
  G_set_new_text(POSITION_DATA, data_txt, 0);
  G_display_switch(POSITION_DATA, 0);
}





/************************************************************************
 *
 *   NAME:         mem_display_time()
 *                 
 *   FUNCTION:     displays time value
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/


void
mem_display_time(struct patternset_type *patternset, struct timeval *time)
{
  char data_txt[128];
  float t;
  int h, m;


  sprintf(data_txt, "time: ??:??:??.??");
  if (patternset)
    if (patternset->first)
      if (patternset->first->time){
	t = ((float) (time->tv_sec - patternset->first->time->tv_sec));
	t += ((float) (time->tv_usec - patternset->first->time->tv_usec))
	  / 1000000.0;
	h = (int) (t / 3600.0);
	t -= (((float) h) * 3600.0);
	m = (int) (t / 60.0);
	t -= (((float) m) * 60.0);
	if (t < 10.0)
	  sprintf(data_txt, "time: %.2d:%.2d:0%2.2f", h, m, t);
	else
	  sprintf(data_txt, "time: %.2d:%.2d:%2.2f", h, m, t);
      }
  
  G_set_new_text(TIME_DATA, data_txt, 0);
  G_display_switch(TIME_DATA, 0);
}



/************************************************************************
 *
 *   NAME:         mem_display_sonars()
 *                 
 *   FUNCTION:     displays sonar data
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/


void
mem_display_sonars(float *sonars)
{    
  int i;

  if (!sonars)
    return;
  
  /*
   * copy sonars scan
   */
  for (i = 0; i < NUM_SONAR_SENSORS; i++){
    sonar_values[i] = sonars[i];
    if (sonar_values[i] > MAX_SONAR_DISPLAY_RANGE)
      sonar_values[i] = MAX_SONAR_DISPLAY_RANGE;
  }
  G_display_robot(SONAR_ROBOT, 0.0, 0.0, 90.0,
		  NUM_SONAR_SENSORS, sonar_values);
  G_display_switch(SONAR_TITLE, 0);
  G_display_markers(SONAR_MARKERS);  

}



/************************************************************************
 *
 *   NAME:         mem_display_lasers()
 *                 
 *   FUNCTION:     displays laser data
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/


void
mem_display_lasers(float *lasers)
{    
  int i;

  if (!lasers)
    return;
  
  /*
   * copy lasers scan
   */
  for (i = 0; i < NUM_LASER_SENSORS; i++){
    laser_values[i] = lasers[i];
    if (laser_values[i] > MAX_LASER_DISPLAY_RANGE)
      laser_values[i] = MAX_LASER_DISPLAY_RANGE;
    if (laser_values[i] < 0.0)
      laser_values[i] = 0.0;
  }
  G_display_robot(LASER_ROBOT, 0.0, 0.0, 90.0,
		  NUM_LASER_SENSORS, laser_values);
  G_display_switch(LASER_TITLE, 0);
  G_display_markers(LASER_MARKERS);  

}


/************************************************************************
 *
 *   NAME:         mem_display_buttons()
 *                 
 *   FUNCTION:     displays buttons data
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/


void
mem_display_buttons(buttons_type *buttons)
{    
  int i;

  if (!buttons)
    return;

  for (i = 0; i < 6; i++){
    G_display_switch(BUTTON_LIT[i], buttons->lit[i]);
    if (i < 4)
      G_display_switch(BUTTON_PUSHED[i], buttons->pushed[i]);
  }
}


/************************************************************************
 *
 *   NAME:         mem_display_pantilt()
 *                 
 *   FUNCTION:     displays laser data
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/


void
mem_display_pantilt(float *pantilt)
{    
  int i;
  float pan, tilt;

  if (!pantilt)
    return;
  
  /*
   * copy pantilt scan
   */
  
  pan  = pantilt[0];
  if (pan < MIN_PAN_ANGLE)
    pan = MIN_PAN_ANGLE;
  if (pan > MAX_PAN_ANGLE)
    pan = MAX_PAN_ANGLE;

  tilt  = pantilt[1];
  if (tilt < MIN_TILT_ANGLE)
    tilt = MIN_TILT_ANGLE;
  if (tilt > MAX_TILT_ANGLE)
    tilt = MAX_TILT_ANGLE;

  G_display_value(PAN_DIAL, - (pan * 180.0 / M_PI));
  G_display_value(TILT_DIAL, tilt * 180.0 / M_PI);
}



/************************************************************************
 *
 *   NAME:         mem_display_irs()
 *                 
 *   FUNCTION:     displays IR data
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/


void
mem_display_irs(float *irs)
{    
  int i;

  if (!irs)
    return;
  
  
  for (i = 0; i < NUM_IR_SENSORS; i++){
    ir_values[i] = irs[i];
    if (ir_values[i] > MAX_IR_DISPLAY_RANGE)
      ir_values[i] = MAX_IR_DISPLAY_RANGE;
  }
  
  G_display_robot(IR_ROBOT1, 0.0, 0.0, 90.0, NUM_IR_SENSORS_AT_HEIGHT_1,
		  &(ir_values[0]));
  G_display_robot(IR_ROBOT2, 0.0, 0.0, 90.0, NUM_IR_SENSORS_AT_HEIGHT_2,
		  &(ir_values[NUM_IR_SENSORS_AT_HEIGHT_1]));
  G_display_robot(IR_ROBOT3, 0.0, 0.0, 90.0, NUM_IR_SENSORS_AT_HEIGHT_3,
		  &(ir_values[(NUM_IR_SENSORS_AT_HEIGHT_1) +
			     (NUM_IR_SENSORS_AT_HEIGHT_2)]));
  G_display_switch(IR_TITLE, 0);
}


/************************************************************************
 *
 *   NAME:         mem_display_tactiles()
 *                 
 *   FUNCTION:     displays tactile data
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/


void
mem_display_tactiles(float *tactiles)
{    
  int i;

  if (!tactiles)
    return;

  for (i = 0; i < NUM_TACTILE_SENSORS; i++){
    tactile_values[i] = tactiles[i];
    if (tactile_values[i] > MAX_TACTILE_VALUE_RANGE ||
	tactile_values[i] < 0.0)
      fprintf(stderr, "Malicious tactile value %d: %g.\n",
	      i, tactile_values[i]);
    if (tactile_values[i] > MAX_TACTILE_VALUE_RANGE)
      tactile_values[i] = MAX_TACTILE_VALUE_RANGE;
    if (tactile_values[i] < 0.0)
      tactile_values[i] = 0.0;

    if (i < NUM_TACTILE_SENSORS_AT_HEIGHT_1)
      tactile_values[i] += 8.0;
    else if (i < (NUM_TACTILE_SENSORS_AT_HEIGHT_1) + 
	     (NUM_TACTILE_SENSORS_AT_HEIGHT_2))
      tactile_values[i] += 6.0;
    else if (i < (NUM_TACTILE_SENSORS_AT_HEIGHT_1) + 
	     (NUM_TACTILE_SENSORS_AT_HEIGHT_2) + 
	     (NUM_TACTILE_SENSORS_AT_HEIGHT_3))
      tactile_values[i] += 4.0;
    else
      tactile_values[i] += 2.0;
  }
      
  G_display_robot(TACTILE_ROBOT1, 0.0, 0.0, 90.0,
		  NUM_TACTILE_SENSORS_AT_HEIGHT_1,
		  &(tactile_values[0]));
  G_display_robot(TACTILE_ROBOT2, 0.0, 0.0, 90.0, 
		  NUM_TACTILE_SENSORS_AT_HEIGHT_2,
		  &(tactile_values[NUM_TACTILE_SENSORS_AT_HEIGHT_1]));
  G_display_robot(TACTILE_ROBOT3, 0.0, 0.0, 90.0, 
		  NUM_TACTILE_SENSORS_AT_HEIGHT_3,
		  &(tactile_values[(NUM_TACTILE_SENSORS_AT_HEIGHT_1) +
				  (NUM_TACTILE_SENSORS_AT_HEIGHT_2)]));
  G_display_robot(TACTILE_ROBOT4, 0.0, 0.0, 90.0, 
		  NUM_TACTILE_SENSORS_AT_HEIGHT_4,
		  &(tactile_values[(NUM_TACTILE_SENSORS_AT_HEIGHT_1) +
				  (NUM_TACTILE_SENSORS_AT_HEIGHT_2) +
				  (NUM_TACTILE_SENSORS_AT_HEIGHT_3)]));
  G_display_switch(TACTILE_TITLE, 0);
  G_display_markers(TACTILE_MARKERS);  
}


/************************************************************************
 *
 *   NAME:         mem_compute_markers()
 *                 
 *   FUNCTION:     updates the markers for the display
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/


void
mem_compute_markers(marker_type *markers)
{    
  int i;
  
  G_clear_markers(IMAGE_MARKERS);
  G_clear_markers(SONAR_MARKERS);
  G_clear_markers(LASER_MARKERS);
  G_clear_markers(IR_MARKERS);
  G_clear_markers(TACTILE_MARKERS);
  G_clear_markers(CONTROL_MARKERS);
  
  if (!markers)
    return;
  
  /*
   * update the markers
   */
  
  
  for (i = 0; i < markers->num_markers; i++){
    
    if (markers->window[i] == IMAGE_WINDOW){
      G_add_marker(IMAGE_MARKERS, 
		   markers->x[i], markers->y[i], markers->type[i]);
      G_display_markers(IMAGE_MARKERS);
    }
    else if (markers->window[i] == CONTROL_WINDOW){
      G_add_marker(CONTROL_MARKERS, 
		   - markers->y[i], markers->x[i], markers->type[i]);
      G_display_markers(CONTROL_MARKERS);
    }
    else if (markers->window[i] == SONAR_WINDOW){
      G_add_marker(SONAR_MARKERS, 
		   - markers->y[i], markers->x[i], markers->type[i]);
      G_display_markers(SONAR_MARKERS);
    }
    else if (markers->window[i] == LASER_WINDOW){
      G_add_marker(LASER_MARKERS, 
		   - markers->y[i], markers->x[i], markers->type[i]);
      G_display_markers(LASER_MARKERS);
    }
    else if (markers->window[i] == IR_WINDOW){
      G_add_marker(IR_MARKERS, 
		   - markers->y[i], markers->x[i], markers->type[i]);
      G_display_markers(IR_MARKERS);
    }
    else if (markers->window[i] == TACTILE_WINDOW){
      G_add_marker(TACTILE_MARKERS, 
		   - markers->y[i], markers->x[i], markers->type[i]);
      G_display_markers(TACTILE_MARKERS);
    }
  }
}


/************************************************************************
 *
 *   NAME:         mem_display_image()
 *                 
 *   FUNCTION:     displays image data (2 different formats)
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/



void
mem_display_image(unsigned char *image, /* Format 1 */
		  unsigned char *red,           /* Format 2 */
		  unsigned char *green,         /* Format 2 */
		  unsigned char *blue)          /* Format 2 */
{
  int i, j, x, y;

  if (!image && (!red || !green || !blue))
    return;

  if (image && (red || green || blue))
    return;


  /*
   * construct image display from red, green, blue (raw data)
   */
  if (!image){
    for (i = 0, y = 0; y < IMAGE_SIZE_Y; y++)
      for (x = 0; x < IMAGE_SIZE_X; x++, i++){
        j = x * IMAGE_SIZE_Y + (IMAGE_SIZE_Y - y - 1);
        d_image[j] = (((float) red[i]) + ((float) green[i])
                      + ((float) blue[i])) / 256.0 / 3.0;
      }
  }

  /*
   * construct image display from image (internal format)
   */
  else{
    for (i = 0, y = 0; y < IMAGE_SIZE_Y; y++)
      for (x = 0; x < IMAGE_SIZE_X; x++, i++){
        j = x * IMAGE_SIZE_Y + (IMAGE_SIZE_Y - y - 1);
        d_image[j] = (((float) image[i*3]) + ((float) image[i*3+1])
                      + ((float) image[i*3+2])) / 256.0 / 3.0;
      }
  }

  
  /*
   * and display it.
   */
  
  G_display_matrix(IMAGE);
  G_display_markers(IMAGE_MARKERS);  
}


/************************************************************************
 *
 *   NAME:         mem_display_control()
 *                 
 *   FUNCTION:     displays control data
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/


void
mem_display_control(control_type *control)
{    
  static float actual_translate_velocity   = 0.0;
  static float actual_rotate_velocity      = 0.0;
  float  new_translate_velocity, new_rotate_velocity;
  float  vel_display_x, vel_display_y;

  if (!control)
    return;
  
  /*
   * copy pantilt scan
   */
  new_translate_velocity = control->trans_velocity;
  if (new_translate_velocity > MAX_TRANS_VEL)
    new_translate_velocity = MAX_TRANS_VEL;
  if (new_translate_velocity < -MAX_TRANS_VEL)
    new_translate_velocity = -MAX_TRANS_VEL;

  new_rotate_velocity    = control->rot_velocity;
  if (new_rotate_velocity > MAX_ROT_VEL)
    new_rotate_velocity = MAX_ROT_VEL;
  if (new_rotate_velocity < -MAX_ROT_VEL)
    new_rotate_velocity = -MAX_ROT_VEL;

  /*
    if (control->command_type == BASE_COMMAND &&
    (actual_translate_velocity != new_translate_velocity ||
    actual_rotate_velocity    != new_rotate_velocity)){
    */

  /* display the control window */
  
  vel_display_y = new_translate_velocity;
  vel_display_x = new_rotate_velocity;
  if (vel_display_x < -(MAX_ROT_VEL))
    vel_display_x = -(MAX_ROT_VEL);
  if (vel_display_x > (MAX_ROT_VEL))
    vel_display_x = (MAX_ROT_VEL);
  if (vel_display_y < -(MAX_TRANS_VEL))
    vel_display_y = -(MAX_TRANS_VEL);
  if (vel_display_y > (MAX_TRANS_VEL))
    vel_display_y = (MAX_TRANS_VEL);
  G_display_robot(CONTROL_ROBOT1, 0.0, 0.0, 90.0, 0, NULL); 
  G_clear_markers(CONTROL_DOTS);
  G_add_marker(CONTROL_DOTS, 0.0, 0.0, 0);
  G_add_marker(CONTROL_DOTS, vel_display_x, vel_display_y, 0);
  G_display_markers(CONTROL_DOTS);
  G_display_robot(CONTROL_ROBOT2, 0.0, 0.0, 90.0, 0, NULL);
  G_display_markers(CONTROL_MARKERS);
  G_display_switch(CONTROL_FRAME, global_modus_control);
  

  /* display the speed dials */
  
  if (new_translate_velocity >= 0.0){
    G_display_value(TRANS_VEL_POS, new_translate_velocity);
    G_display_value(TRANS_VEL_NEG, MAX_TRANS_VEL);
  }
  else{
    G_display_value(TRANS_VEL_POS, 0.0);
    G_display_value(TRANS_VEL_NEG, MAX_TRANS_VEL + new_translate_velocity);
  }
  
  if (new_rotate_velocity >= 0.0){
    G_display_value(ROT_VEL_POS, new_rotate_velocity);
    G_display_value(ROT_VEL_NEG, MAX_ROT_VEL);
  }
  else{
    G_display_value(ROT_VEL_POS, 0.0);
    G_display_value(ROT_VEL_NEG, MAX_ROT_VEL + new_rotate_velocity);
  }
  
  actual_translate_velocity = new_translate_velocity;
  actual_rotate_velocity    = new_rotate_velocity;
}



/************************************************************************
 *
 *   NAME:         mem_display_pattern()
 *                 
 *   FUNCTION:     displays a pattern
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/



int
mem_display_pattern(struct patternset_type *patternset, 
		    struct pattern_type *pattern,
		    int display_data,
		    int execute_flag)
{
  int i, j, index;
  char num_pat_txt[128];
  char type_text[128];


  if (pattern == NULL){
    fprintf(stderr, "ERROR: Pattern does not exist. Cannot display.");
    return 0;
  }


  if (patternset != NULL){
    current_patternset = patternset;
    current_pattern    = pattern;
  }


  if (execute_flag)
    X_process_pattern(pattern);


  if (display_data){

    /*
      G_undisplay_markers(IMAGE_MARKERS,   -1, C_GREY20);
      G_undisplay_markers(SONAR_MARKERS,   -1, C_GREY85);
      G_undisplay_markers(LASER_MARKERS,   -1, C_GREY85);
      G_undisplay_markers(IR_MARKERS,      -1, C_GREY85);
      G_undisplay_markers(TACTILE_MARKERS, -1, C_GREY85);
      */
    mem_compute_markers(pattern->markers);
    mem_display_image(pattern->image, NULL, NULL, NULL);
    mem_display_sonars(pattern->sonars);
    mem_display_lasers(pattern->lasers);
    mem_display_tactiles(pattern->tactiles);
    mem_display_irs(pattern->irs);
    mem_display_buttons(pattern->buttons);
    mem_display_pantilt(pattern->pantilt);
    mem_display_pos(pattern->position);
    mem_display_time(patternset, pattern->time);
    mem_display_control(pattern->control);


    if (patternset){
      if (pattern->name != NULL)
	sprintf(num_pat_txt, "pattern \"%s\" (%d of %d)", 
		pattern->name, pattern->number, patternset->num_patterns);
      
      else
	sprintf(num_pat_txt, "pattern (%d of %d)", 
		pattern->number, patternset->num_patterns);
      G_set_new_text(DIAL_BUTTON, num_pat_txt, 0);
      if (patternset->num_patterns == 0)
	G_display_value(DIAL_BUTTON, 0.0);
      else if (patternset->num_patterns == 1)
	G_display_value(DIAL_BUTTON, 1.0);
      else
	G_display_value(DIAL_BUTTON, 
			((float) pattern->number) 
			/ ((float) (patternset->num_patterns - 1)));
      
      if (patternset->type == UNKNOWN_SET_TYPE)
	strcpy(type_text, "");
      else if (patternset->type == TRAINING_SET_TYPE)
	strcpy(type_text, " [training set]");
      else if (patternset->type == TESTING_SET_TYPE)
	strcpy(type_text, " [testing set]");
      
      
      if (patternset->name != NULL)
	sprintf(num_pat_txt, "pattern set \"%s\" (%d of %d)%s", 
		patternset->name, patternset->number, data->num_sets,
		type_text);
      else
	sprintf(num_pat_txt, "pattern set (%d of %d)%s", 
		patternset->number, data->num_sets, type_text);
      G_set_new_text(SET_DIAL_BUTTON, num_pat_txt, 0);
      if (data->num_sets == 0)
	G_display_value(SET_DIAL_BUTTON, 0.0);
      else if (data->num_sets == 1)
	G_display_value(SET_DIAL_BUTTON, 1.0);
      else
	G_display_value(SET_DIAL_BUTTON, 
			((float) patternset->number) 
			/ ((float) (data->num_sets - 1)));
    }
  }
  return 1;
}












/************************************************************************
 *
 *   NAME:         mem_display_nth_pattern
 *                 
 *   FUNCTION:     displays the n-th pattern (careful, is slow)
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



int
mem_display_nth_pattern(struct patternset_type *patternset, int n)
{
  struct pattern_type *pattern;


  if (patternset == NULL){
    fprintf(stderr, 
	    "ERROR: Patternset does not exist. Cannot display n-th pat.");
    return 0;
  }

  pattern = mem_get_nth_pattern(patternset, n);

  if (n == 0)
    X_initialize_episode();

  return mem_display_pattern(patternset, pattern, 1, 1);
}










/************************************************************************
 *
 *   NAME:         mem_display_next_pattern
 *                 
 *   FUNCTION:     displays the next pattern
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if not the last pattern
 *                 
 ************************************************************************/



int
mem_display_next_pattern()
{

  if (current_pattern == NULL){
    fprintf(stderr, "ERROR: Pattern does not exist.\n");
    return 0;
  }

  if (current_pattern->next == NULL){
    fprintf(stderr, "ERROR: Next Pattern does not exist. Cannot display.\n");
    return 0;
  }

  current_pattern = current_pattern->next;
  mem_display_pattern(current_patternset, current_pattern, 1, 1);
  if (current_pattern->next)
    return 1;
  else
    return 0;
}








/************************************************************************
 *
 *   NAME:         mem_query_last_pattern()
 *                 
 *   FUNCTION:     displays the next pattern
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if the last pattern
 *                 
 ************************************************************************/



int
mem_query_last_pattern()
{

  if (current_pattern == NULL){
    fprintf(stderr, "ERROR: Pattern does not exist. Cannot display next.\n");
    return 0;
  }

  if (current_pattern->next)
    return 0;
  else
    return 1;
}





/************************************************************************
 *
 *   NAME:         mem_query_last_patternset()
 *                 
 *   FUNCTION:     displays the next pattern
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if the last pattern
 *                 
 ************************************************************************/



int
mem_query_last_patternset()
{

  if (current_patternset == NULL){
    fprintf(stderr, "ERROR: Pattern set does not exist.\n");
    return 0;
  }

  if (current_patternset->next_set)
    return 0;
  else
    return 1;
}







/************************************************************************
 *
 *   NAME:         mem_display_previous_pattern
 *                 
 *   FUNCTION:     displays the previous pattern
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if not the first pattern
 *                 
 ************************************************************************/



int
mem_display_previous_pattern()
{

  if (current_pattern == NULL){
    fprintf(stderr, "ERROR: Pattern does not exist. Cannot display previous.\n");
    return 0;
  }

  if (current_pattern->previous == NULL){
    fprintf(stderr, "ERROR: Previous Pattern does not exist. Cannot display.\n");
    return 0;
  }

  current_pattern = current_pattern->previous;
  mem_display_pattern(current_patternset, current_pattern, 1, 1);
  if (current_pattern->previous)
    return 1;
  else
    return 0;
}




/************************************************************************
 *
 *   NAME:         mem_display_next_patternset
 *                 
 *   FUNCTION:     displays the next pattern
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if not the last pattern
 *                 
 ************************************************************************/



int
mem_display_next_patternset()
{
  if (current_patternset == NULL){
    fprintf(stderr, 
	    "ERROR: Pattern set does not exist. Cannot display next.\n");
    return 0;
  }

  if (current_patternset->next_set == NULL){
    fprintf(stderr, 
	    "ERROR: Next pattern set does not exist. Cannot display.\n");
    return 0;
  }

  current_patternset = current_patternset->next_set;
  current_pattern = current_patternset->first;
  X_initialize_episode();
  mem_display_pattern(current_patternset, current_pattern, 1, 1);
  if (current_pattern->next)
    return 1;
  else
    return 0;
}




/************************************************************************
 *
 *   NAME:         mem_display_previous_patternset
 *                 
 *   FUNCTION:     displays the previous pattern
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 1, if not the last pattern
 *                 
 ************************************************************************/



int
mem_display_previous_patternset()
{
  if (current_patternset == NULL){
    fprintf(stderr, 
	    "ERROR: Pattern set does not exist. Cannot display previous.\n");
    return 0;
  }

  if (current_patternset->previous_set == NULL){
    fprintf(stderr, 
	    "ERROR: Previous pattern set does not exist. Cannot display.\n");
    return 0;
  }

  current_patternset = current_patternset->previous_set;
  current_pattern = current_patternset->first;
  X_initialize_episode();
  mem_display_pattern(current_patternset, current_pattern, 1, 1);
  if (current_pattern->previous)
    return 1;
  else
    return 0;
}


/************************************************************************
 *
 *   NAME:         mem_display_nth_patternset
 *                 
 *   FUNCTION:     displays the n-th pattern (careful, is slow)
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



int
mem_display_nth_patternset(int n)
{
  struct patternset_type *patternset;
  struct pattern_type *pattern;


  if (data == NULL){
    fprintf(stderr, 
	    "ERROR: Data does not exist. Cannot display n-th set.");
    return 0;
  }

  patternset = mem_get_nth_patternset(n);
  X_initialize_episode();

  if (patternset->first != NULL){
    pattern = patternset->first;
    return mem_display_pattern(patternset, pattern, 1, 1);
  }
  else
    return 0;
}




/************************************************************************
 *
 *   NAME:         mem_mark_point
 *                 
 *   FUNCTION:     marks a point (response to button click)
 *                 
 *   PARAMETERS:   
 *                 
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void
mem_mark_point(int window, int markers_window, float x, float y, int button)
{
  float dist, best_dist, dx, dy, x2, y2;
  int   i, j, best_i, type, outside;
  struct pattern_type *initial_pattern;
  int stop_pattern_found = 0;

  if (!current_pattern)
    return;			/* failure */

  if (!global_modus_marking)
    return;

  /*
   * --------------------------------------------------
   *              insert new point
   * --------------------------------------------------
   */


  if (button == LEFT_BUTTON){  

    /*
     * check, if we need to allocate memory
     */
    if (!current_pattern->markers){
      current_pattern->markers = (marker_type *) malloc(sizeof(marker_type));
      if (current_pattern->markers == NULL){
	fprintf(stderr, 
		"ERROR: Out of memory in mem_mark_point().\n");
	exit (-1);
      }
      current_pattern->markers->num_markers = 0;
    }

    if (current_pattern->markers->num_markers >= MAX_NUM_MARKERS){
      fprintf(stderr, "WARNING: marker overflow.\n");
      return;
    }
    
    current_pattern->markers->x[current_pattern->markers->num_markers] = x;
    current_pattern->markers->y[current_pattern->markers->num_markers] = y;
    current_pattern->markers->window[current_pattern->markers->num_markers] =
      window;
    current_pattern->markers->type[current_pattern->markers->num_markers] = 0;
    current_pattern->markers->num_markers += 1;
    

    /*
     * display
     */

    if (window == IMAGE_WINDOW)
      G_add_marker(markers_window, x, y, 0);
    else
      G_add_marker(markers_window, -y, x, 0);
    G_display_markers(markers_window);
  }


  /*
   * --------------------------------------------------
   *              remove all points
   * --------------------------------------------------
   */

  else if (button == RIGHT_BUTTON){
    if (!current_pattern->markers)
      return;

    if (window == IMAGE_WINDOW)    
      G_undisplay_markers(markers_window, -1, C_GREY50);
    else
      G_undisplay_markers(markers_window, -1, C_GREY85);

    G_clear_markers(markers_window);

    for (i = 0, j = 0; j < current_pattern->markers->num_markers; ){
      for (; current_pattern->markers->window[j] == window &&
	     j < current_pattern->markers->num_markers; j++);
      if (j < current_pattern->markers->num_markers){
	if (j > i){
	  current_pattern->markers->x[i]      = current_pattern->markers->x[j];
	  current_pattern->markers->y[i]      = current_pattern->markers->y[j];
	  current_pattern->markers->window[i] = 
	    current_pattern->markers->window[j];
	  current_pattern->markers->type[i]   = 
	    current_pattern->markers->type[j];
	  /*fprintf(stderr, "copy image %d to %d\n", j, i);*/
	}
	j++;
	i++;
      }
    }
    current_pattern->markers->num_markers = i;
    /*fprintf(stderr, "remaining: %d\n", i);*/
      
    if (current_pattern->markers->num_markers == 0){
      free(current_pattern->markers);
      current_pattern->markers = NULL;
    }

    /* G_display_all();*/

  }

#ifdef OLD
  /*
   * --------------------------------------------------
   *              remove point
   * --------------------------------------------------
   */

  else if (button == RIGHT_BUTTON){
    if (!current_pattern->markers)
      return;

    best_i = -1;
    best_dist = 0.0;
      
    for (i = 0; i < current_pattern->markers->num_markers; i++){
      if (current_pattern->markers->window[i] == window){
	dist = ((x - current_pattern->markers->x[i]) *
		(x - current_pattern->markers->x[i])) +
	  ((y - current_pattern->markers->y[i]) * 
	   (y - current_pattern->markers->y[i]));
	if (best_i == -1 || dist < best_dist){
	  best_i = i;
	  best_dist = dist;
	}
      }
    }


    if (best_i == -1)
      return;			/* no point left in that window */



    G_clear_markers(markers_window);
    for (i = 0; i < current_pattern->markers->num_markers; i++)
      if (current_pattern->markers->window[i] == window){
	if (i != best_i)
	  type = current_pattern->markers->type[i];
	else
	  type = 2;
	if (window == IMAGE_WINDOW)
	  G_add_marker(markers_window, current_pattern->markers->x[i],
		       current_pattern->markers->y[i], type);
	else
	  G_add_marker(markers_window, -current_pattern->markers->y[i],
		       current_pattern->markers->x[i], type);
      }

    G_display_markers(markers_window);    


    for (i = best_i; i < current_pattern->markers->num_markers; i++){
      current_pattern->markers->x[i]      = current_pattern->markers->x[i+1];
      current_pattern->markers->y[i]      = current_pattern->markers->y[i+1];
      current_pattern->markers->window[i] = 
	current_pattern->markers->window[i+1];
      current_pattern->markers->type[i]   = 
	current_pattern->markers->type[i+1];
    }

    current_pattern->markers->num_markers -= 1;

    if (current_pattern->markers->num_markers == 0){
      free(current_pattern->markers);
      current_pattern->markers = NULL;
    }

  }
#endif

  /*
   * --------------------------------------------------
   *              go to next pattern (and copy markers)
   * --------------------------------------------------
   */

  else if (button == MIDDLE_BUTTON){
    if (!current_pattern->next)
      return;

    initial_pattern = current_pattern;

    do{
      mem_display_next_pattern();
      stop_pattern_found =
	((window == CONTROL_WINDOW && current_pattern->control) ||
	 (window == SONAR_WINDOW && current_pattern->sonars) ||
	 (window == LASER_WINDOW && current_pattern->lasers) ||
	 (window == IR_WINDOW && current_pattern->irs) ||
	 (window == TACTILE_WINDOW && current_pattern->tactiles) ||
	 (window == IMAGE_WINDOW && current_pattern->image));
    }
    while (current_pattern->next && /* not the last pattern */
	   !stop_pattern_found);

    if (stop_pattern_found){
      
      /*
       * allocate the memory
       */
      if (initial_pattern->markers && !current_pattern->markers){
	current_pattern->markers = 
	  (marker_type *) malloc(sizeof(marker_type));
	if (current_pattern->markers == NULL){
	  fprintf(stderr, 
		  "ERROR: Out of memory in mem_mark_point().\n");
	  exit (-1);
	}

	/*
	 * copy the marker over
	 */

	for (i = 0, j = 0; i < initial_pattern->markers->num_markers; i++){
	  outside = 0;
	  if (initial_pattern->markers->window[i] == IMAGE_WINDOW ||
	      initial_pattern->markers->window[i] == CONTROL_WINDOW ||
	      !initial_pattern->position ||
	      !current_pattern->position){
	    /*
	     * literal copy
	     */
	    current_pattern->markers->x[j]      = 
	      initial_pattern->markers->x[i];
	    current_pattern->markers->y[j]      = 
	      initial_pattern->markers->y[i];
	  }
	  else{
	    /*
	     * copy taking robot motion into account (here we need
	     * some fancy coordinate transformations...)
	     */
	    dx = (cos(initial_pattern->position->orientation * M_PI / 180.0)
		  * (current_pattern->position->x -
		     initial_pattern->position->x)) +
	      (sin(initial_pattern->position->orientation * M_PI / 180.0)
	       * (current_pattern->position->y -
		  initial_pattern->position->y));
	    dy =  (cos(initial_pattern->position->orientation * M_PI / 180.0)
		   * (current_pattern->position->y -
		      initial_pattern->position->y)) -
	      (sin(initial_pattern->position->orientation * M_PI / 180.0)
	       * (current_pattern->position->x -
		  initial_pattern->position->x));
	    x2 = (cos((current_pattern->position->orientation -
		       initial_pattern->position->orientation) * M_PI / 180.0)
		  * (initial_pattern->markers->x[i] - dx)) +
	      (sin((current_pattern->position->orientation -
		    initial_pattern->position->orientation) * M_PI / 180.0)
	       * (initial_pattern->markers->y[i] - dy));
	    y2 = (cos((current_pattern->position->orientation -
		       initial_pattern->position->orientation) * M_PI / 180.0)
		  * (initial_pattern->markers->y[i] - dy)) -
	      (sin((current_pattern->position->orientation -
		    initial_pattern->position->orientation) * M_PI / 180.0)
	       * (initial_pattern->markers->x[i] - dx));
	    current_pattern->markers->x[j]      = x2;
	    current_pattern->markers->y[j]      = y2;
	    outside = ((initial_pattern->markers->window[i] == SONAR_WINDOW &&
			(fabs(x2) > (MAX_SONAR_DISPLAY_RANGE)*1.01 ||
			 fabs(y2) > (MAX_SONAR_DISPLAY_RANGE)*1.01)) ||
		       (initial_pattern->markers->window[i] == LASER_WINDOW &&
			(fabs(x2) > (MAX_LASER_DISPLAY_RANGE)*1.01 ||
			 fabs(y2) > (MAX_LASER_DISPLAY_RANGE)*1.01)) ||
		       (initial_pattern->markers->window[i] == IR_WINDOW &&
			(fabs(x2) > (MAX_IR_DISPLAY_RANGE)*1.01 ||
			 fabs(y2) > (MAX_IR_DISPLAY_RANGE)*1.01)) ||
		       (initial_pattern->markers->window[i] == TACTILE_WINDOW &&
			(fabs(x2) > (MAX_TACTILE_DISPLAY_RANGE)*1.01 ||
			 fabs(y2) > (MAX_TACTILE_DISPLAY_RANGE)*1.01)));
	  }
	  current_pattern->markers->window[j] = 
	    initial_pattern->markers->window[i];
	  current_pattern->markers->type[j]   = 1;
	  if (!outside)
	    j++;
	}
	current_pattern->markers->num_markers = j;
	mem_compute_markers(current_pattern->markers);
      }
    }
    
  }
}

