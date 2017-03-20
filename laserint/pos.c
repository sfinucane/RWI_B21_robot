


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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/laserint/pos.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:34:07 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: pos.c,v $
 * Revision 1.1  2002/09/14 15:34:07  rstone
 * *** empty log message ***
 *
 * Revision 1.28  2000/09/06 05:42:20  thrun
 * Map editing buttons.
 *
 * Revision 1.27  2000/03/14 05:23:58  thrun
 * fixed a few problems with the 3D mapping. This version
 * pruced the results first shown to the DAPRA folks.
 * In particular, it reads the texture entirely from file and
 * constructs a new texture file (laserint.ppm, which has
 * to be converted manyally into laserint.jpg)
 *
 * Revision 1.26  2000/03/13 02:54:17  thrun
 * slight inprovement to 3D version.
 *
 * Revision 1.25  2000/03/13 00:31:13  thrun
 * first version of he texturing module. Still a bit clumpsy.
 *
 * Revision 1.24  2000/01/11 12:26:17  schneid1
 * disabled the BEEP in handlers.c and pos.c if flag FGAN is set in Makefile
 *
 * Revision 1.23  2000/01/02 16:58:02  thrun
 * intermediate version with 2 objects
 *
 * Revision 1.22  1999/12/27 03:43:16  thrun
 * Magic: Gradient descent for adapting
 * wall parameters xw yw and aw seems to work!!!
 *
 * Revision 1.21  1999/12/20 20:15:27  thrun
 * step towards object matching - shouldn't affect the
 * regular performance.
 *
 * Revision 1.20  1999/12/19 03:25:18  thrun
 * graphics for 3D object matching.
 *
 * Revision 1.19  1999/10/15 18:07:20  thrun
 * more 3D formats, more compact files.
 *
 * Revision 1.18  1999/10/15 01:30:47  thrun
 * improved interface for robot positioning
 *
 * Revision 1.17  1999/10/14 04:38:20  thrun
 * Fixed a problem of passing on the wrong correction
 * parameters with MAP. Also, ficex a seg-fault problem with
 * laserint when rnu without display.
 *
 * Revision 1.16  1999/10/13 20:50:27  thrun
 * unkown.
 *
 * Revision 1.15  1999/10/02 04:50:12  thrun
 * Minor tuning and changes. THis version built the first
 * nice map of the San Jose Tech Museum
 *
 * Revision 1.14  1999/10/01 17:58:19  thrun
 * reinstatiated the close-the-cycle feature.
 *
 * Revision 1.13  1999/09/28 21:49:55  thrun
 * Multi-robot support in various modules.
 * In particular, I had to introduce a new message for MAP that
 * communicates back robot positions and correction parameters.
 * This might make it necessary to modify existing modules that use
 * MAP - I tried to do this for all I am aware of, but I might have missed some.
 *
 * Revision 1.12  1999/09/28 04:50:01  thrun
 * Fixed minor bug in server architecture that caused the map
 * (in MAP) to be inconsistent.
 *
 * Revision 1.11  1999/09/28 03:31:01  thrun
 * Improved version in the server mode.
 *
 * Revision 1.10  1999/09/06 03:19:45  thrun
 * first working version of the multi-robot server architecture with
 * positon control.
 *
 * Revision 1.9  1999/09/05 21:57:23  thrun
 * initial client server robot for multi-robot mapping
 *
 * Revision 1.8  1999/09/05 17:18:52  thrun
 * 3D ceiling mapping
 *
 * Revision 1.7  1999/07/05 18:57:19  thrun
 * .
 *
 * Revision 1.6  1999/07/03 21:44:37  thrun
 * Fixed several bugs in LASERINT and tuned the parameters of
 * LASERINT, MAP, and PLAN for the new Urban Robot
 *
 * Revision 1.5  1999/07/03 18:49:36  thrun
 * LOASERINT can now send correction parameters directly to MAP.
 *
 * Revision 1.4  1999/07/01 19:20:59  fox
 * Nothing special.
 *
 * Revision 1.3  1999/06/21 18:30:25  thrun
 * Took out the sampling stuff (which never worked corretly). It's now
 * up to Dieter to put it in again...
 *
 * Revision 1.2  1999/05/08 19:47:29  thrun
 * drastically improved version with recursive matching and samples.
 * Not quite ready yet: samples aren't used in the right way, queue
 * seems not to work, and some constants are hardcoded (phase!) But
 * impressive results!
 *
 * Revision 1.1  1999/05/04 19:47:26  thrun
 * new file "pos.c"
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
#include "tcx.h"
#include "tcxP.h"
#include "Application.h"
#include "Net.h"
#include "EZX11.h"
#include "o-graphics.h"
#include "bUtils.h"
#include "corr.h"


#include "MAP-messages.h"
#include "LASER-messages.h"
#include "SONAR-messages.h"
#include "BASE-messages.h"


#include "LASERINT.h"

#include "beeSoftVersion.h"
#include "libbUtils.h"
#include "librobot.h"
#include "libezx.h"


/***************************************************************************
 ***************************************************************************
 ***************************************************************************/

FILE *plot_iop = NULL;
FILE *vr_iop1 = NULL;
FILE *vr_iop2 = NULL;
FILE *vr_iop3 = NULL;
FILE *vr_iop4 = NULL;
FILE *smf_iop1 = NULL;
FILE *smf_iop2 = NULL;

/************************************************************************\
 ************************************************************************
\************************************************************************/

#define RANDOM_MAX ((float)(2147483647))   /* (2**31 - 1) */
#define RAND_PLUS_MINUS_ONE() ((2.0 * ((float)(random()) / ((float) RANDOM_MAX))) - 1.0)
#define RAND_ZERO_TO_ONE() ((float)(random()) / ((float) RANDOM_MAX))


/***************************************************************************
 ***************************************************************************
 ***************************************************************************/


void
initiate_position_control(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			  PROGRAM_STATE_PTR program_state,
			  ROBOT_STATE_PTR robot_state)
{
  int i, done;

  
  if (program_state->map_update_pending)
    terminate_position_control(robot_specifications, 
			       program_state, robot_state);

  /* fprintf(stderr, " #%d", program_state->data_count); */

  G_clear_markers(MATCHES);

  /*
   * work in the robot's generic drift:
   * compute the relative difference to previous coordinates
   * and update the position parameters accordingly
   */

  drift_adjust(robot_specifications, program_state, robot_state);
  
  /*
   * copy into stack of readings (only to simplify computation, we
   * might not retain it there
   */

  temp_cache_reading(robot_specifications, program_state, robot_state);
  
  /*
   * set flags
   */

  program_state->map_update_pending = 1;
  program_state->position_control_iteration = 0;

  /*
   * diagnosis
   */

  if (robot_specifications->diagnose_step >= 0 &&
      program_state->data_count >= robot_specifications->diagnose_step){
    program_state->delay_in_replay = 50.0;
    display_scans_and_map(robot_specifications, program_state, robot_state,
			  0); 
    usleep(500000);
  }

  /*
   * check if we have to terminate
   */

  if (!robot_specifications->do_position_correction ||
      program_state->first_stack_item == program_state->last_stack_item ||
      fabs(robot_state->rotation_between_scans_1 +
	   robot_state->rotation_between_scans_2) > 
      robot_specifications->pos_corr_rot_cutoff_angle)

    terminate_position_control(robot_specifications, 
			       program_state, robot_state);
}





void
do_position_control(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		    PROGRAM_STATE_PTR program_state,
		    ROBOT_STATE_PTR robot_state)
{
  int count;


  if (!program_state->map_update_pending)
    return;


  count = correct_position(robot_specifications, robot_state);
  
  if (count == 0)
    fprintf(stderr, "   ### NO MATCH ###   ");


  program_state->position_control_iteration += 1;
  /*if (program_state->position_control_iteration % 100 == 0)
    fprintf(stderr, "{%d}", program_state->position_control_iteration);*/

  /*
   * diagnosis
   */
  if (robot_specifications->diagnose_step >= 0 &&
      program_state->data_count >= robot_specifications->diagnose_step){
    display_scans_and_map(robot_specifications, program_state, robot_state, 
			  0); 
    usleep(500000);
  }
  
  if (program_state->position_control_iteration >=
      robot_specifications->pos_corr_max_num_iterations || count == 0){
    terminate_position_control(robot_specifications, program_state, 
			       robot_state);    
  }
}




void
terminate_position_control(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			  PROGRAM_STATE_PTR program_state,
			  ROBOT_STATE_PTR robot_state)
{
  stack_item_ptr s = &(program_state->stack[program_state->last_stack_item]);  
  float local_x, local_y, local_orientation;
  float local_x2, local_y2, local_orientation2;
  float *corr_x, *corr_y, *corr_angle;
  int   *corr_type;


  if (!program_state->map_update_pending)
    return;


  G_display_value(ERROR_DIAL, program_state->remaining_error);

  fprintf(stderr, " %d", program_state->position_control_iteration);


  /*
   * adjust internal parameters
   */

  if (s->robot_number == -1){
    corr_x = &(robot_state->correction_parameter_x);
    corr_y = &(robot_state->correction_parameter_y);
    corr_angle = &(robot_state->correction_parameter_angle);
    corr_type = &(robot_state->correction_type);
  }
  else{
    corr_x = &(auto_update_modules[s->robot_number].correction_parameter_x);
    corr_y = &(auto_update_modules[s->robot_number].correction_parameter_y);
    corr_angle = &(auto_update_modules[s->robot_number].correction_parameter_angle);
    corr_type = &(auto_update_modules[s->robot_number].correction_type);
  }
    
  compute_backward_correction(robot_state->laser_x,
			      robot_state->laser_y,
			      robot_state->laser_orientation,
			      *corr_x, *corr_y, *corr_angle, *corr_type,
			      &local_x,
			      &local_y,
			      &local_orientation);

  compute_backward_correction(robot_state->x,
			      robot_state->y,
			      robot_state->orientation,
			      *corr_x, *corr_y, *corr_angle, *corr_type,
			      &local_x2,
			      &local_y2,
			      &local_orientation2);

  compute_correction_parameters(local_x,
				local_y,
				local_orientation,
				s->laser_x,
				s->laser_y,
				s->laser_orientation,
				corr_x, corr_y, corr_angle, corr_type);

  compute_forward_correction(local_x,
			     local_y,
			     local_orientation,
			     *corr_x, *corr_y, *corr_angle, *corr_type,
			     &robot_state->laser_x,
			     &robot_state->laser_y,
			     &robot_state->laser_orientation);

  compute_forward_correction(local_x2,
			     local_y2,
			     local_orientation2,
			     *corr_x, *corr_y, *corr_angle, *corr_type,
			     &robot_state->x,
			     &robot_state->y,
			     &robot_state->orientation);

  /* fprintf(stderr, "### Correction %g %g %g: %g %g %g -> %g %g %g\n",
	  robot_state->correction_parameter_x,
	  robot_state->correction_parameter_y,
	  robot_state->correction_parameter_angle,
	  local_x2, local_y2, local_orientation2,
	  robot_state->x, robot_state->y, robot_state->orientation);
	  */

  /*
    robot_state->raw_odometry_x = s->raw_odometry_laser_x;
  robot_state->raw_odometry_y = s->raw_odometry_laser_y;
  robot_state->raw_odometry_orientation = s->raw_odometry_laser_orientation;
  fprintf(stderr, "RESET: %g %g %g\n",
	  robot_state->raw_odometry_x,
	  robot_state->raw_odometry_y,
	  robot_state->raw_odometry_orientation);
  /*  @@@ */

  /*
   * send correction parameters to MAP
   */

  if (robot_specifications->send_corr_parameters_to_map)
    send_correction_parameters_to_map(robot_specifications, 
				      program_state, robot_state,
				      *corr_x, *corr_y, *corr_angle,
				      *corr_type);



  /*
   * send out notices to everyone
   */
  
  inform_clients(robot_specifications, program_state, robot_state,
		 s->robot_number);

  /*
   * add a little square
   */

  /*!*/
  /*
  if (program_state->object_matching_mode &&
      program_state->number_objects < MAX_NUMBER_OBJECTS){
    program_state->objects[program_state->number_objects] =
      (object_item_ptr) malloc(sizeof(object_item_type));

    program_state->objects[program_state->number_objects]->x =
      s->laser_x - 100.0;
    program_state->objects[program_state->number_objects]->y =
      s->laser_y-50.0;
    program_state->objects[program_state->number_objects]->z = 0.0;
    program_state->objects[program_state->number_objects]->angle = 
      s->laser_orientation;
    program_state->objects[program_state->number_objects]->length = 400.0;
    program_state->objects[program_state->number_objects]->width  = 100.0;
    program_state->objects[program_state->number_objects]->height = 200.0;
    program_state->number_objects++;
  }
  */

  /*
   * display scans and map
   */
  
  display_scans_and_map(robot_specifications, program_state, robot_state, 0);


  /*
   * generate plot file
   */

  plot_pos(robot_specifications, program_state, robot_state);
  
  
  /*
   * Save this reading
   */

  if (robot_specifications->do_position_correction &&
      fabs(robot_state->rotation_between_scans_1 +
	   robot_state->rotation_between_scans_2) <= 
      robot_specifications->pos_corr_rot_cutoff_angle)
    cache_reading(robot_specifications, program_state, robot_state);

  /*
   * reset flags
   */


  program_state->map_update_pending = 0;
  program_state->position_control_iteration = 0;

  /*
   * estimate the drift
   */

  drift_adapt(robot_specifications, program_state, robot_state);

}



/***************************************************************************
 ***************************************************************************
 ***************************************************************************/



void
drift_adjust(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	     PROGRAM_STATE_PTR program_state,
	     ROBOT_STATE_PTR robot_state)
{
  float trans_drift, rot_drift;
  float local_x, local_y, local_orientation;

  if (robot_state->prev_laser_defined){
    
    compute_polar_difference(robot_state->prev_laser_x,
			     robot_state->prev_laser_y,
			     robot_state->prev_laser_orientation,
			     robot_state->x,
			     robot_state->y,
			     robot_state->orientation,
			     &(robot_state->translation_between_scans),
			     &(robot_state->rotation_between_scans_1),
			     &(robot_state->rotation_between_scans_2));
    
    trans_drift = robot_specifications->pos_corr_drift_trans *
      robot_state->translation_between_scans;
    rot_drift = robot_specifications->pos_corr_drift_rot *
      robot_state->translation_between_scans;
    
    robot_state->laser_x =
      (robot_state->prev_laser_x +
       ((robot_state->translation_between_scans + trans_drift) *
	cos((robot_state->prev_laser_orientation + rot_drift +
	     robot_state->rotation_between_scans_1) * M_PI / 180.0)));
    robot_state->laser_y =
      (robot_state->prev_laser_y +
       ((robot_state->translation_between_scans + trans_drift) *
	sin((robot_state->prev_laser_orientation + rot_drift +
	     robot_state->rotation_between_scans_1) * M_PI / 180.0)));
    robot_state->laser_orientation =
      (robot_state->prev_laser_orientation +
       robot_state->rotation_between_scans_1 +
       robot_state->rotation_between_scans_2 + rot_drift);
    
    for (; robot_state->laser_orientation >= 180.0; ) 
      robot_state->laser_orientation -= 360.0;
    for (; robot_state->laser_orientation < -180.0; ) 
      robot_state->laser_orientation += 360.0;
    
    for (; robot_state->orientation >= 180.0; ) 
      robot_state->orientation -= 360.0;
    for (; robot_state->orientation < -180.0; ) 
      robot_state->orientation += 360.0;

    
    compute_backward_correction(robot_state->x,
				robot_state->y,
				robot_state->orientation,
				robot_state->correction_parameter_x,
				robot_state->correction_parameter_y,
				robot_state->correction_parameter_angle,
				robot_state->correction_type,
				&local_x,
				&local_y,
				&local_orientation);

    compute_correction_parameters(local_x,
				  local_y,
				  local_orientation,
				  robot_state->laser_x,
				  robot_state->laser_y,
				  robot_state->laser_orientation,
				  &robot_state->correction_parameter_x,
				  &robot_state->correction_parameter_y,
				  &robot_state->correction_parameter_angle,
				  &robot_state->correction_type);

    compute_polar_difference(robot_state->prev_laser_x,
			     robot_state->prev_laser_y,
			     robot_state->prev_laser_orientation,
			     robot_state->laser_x,
			     robot_state->laser_y,
			     robot_state->laser_orientation,
			     &(robot_state->translation_between_scans),
			     &(robot_state->rotation_between_scans_1),
			     &(robot_state->rotation_between_scans_2));
  }
  else{
    robot_state->translation_between_scans =
      robot_state->rotation_between_scans_1 =
      robot_state->rotation_between_scans_2 = 0.0;
    
    robot_state->laser_x = robot_state->x;
    robot_state->laser_y = robot_state->y;
    robot_state->laser_orientation = robot_state->orientation;
  }
}




void
drift_adapt(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	    PROGRAM_STATE_PTR program_state,
	    ROBOT_STATE_PTR robot_state)
{
  float new_translation, new_rotation_1, new_rotation_2;
  float new_drift_trans = 0.0, new_drift_rot = 0.0;
  stack_item_ptr s = &(program_state->stack[program_state->last_stack_item]);  


  COMMENT("No drift");
  return;


  if (robot_state->prev_laser_defined && 
      robot_specifications->pos_corr_do_est_drift &&
      robot_specifications->do_position_correction &&
      fabs(robot_state->rotation_between_scans_1 +
	   robot_state->rotation_between_scans_2) <= 
      robot_specifications->pos_corr_rot_cutoff_angle){
    compute_polar_difference(robot_state->prev_laser_x,
			     robot_state->prev_laser_y,
			     robot_state->prev_laser_orientation,
			     robot_state->laser_x,
			     robot_state->laser_y,
			     robot_state->laser_orientation,
			     &new_translation,
			     &new_rotation_1,
			     &new_rotation_2);
    
    if (robot_state->translation_between_scans > 0.0){
      new_drift_trans = 
	(new_translation - robot_state->translation_between_scans)
	/ robot_state->translation_between_scans;
      new_drift_rot = (new_rotation_1 + new_rotation_2)
	- (robot_state->rotation_between_scans_1 
	   + robot_state->rotation_between_scans_2);
      for (; new_drift_rot >= 180.0; ) new_drift_rot -= 360.0;
      for (; new_drift_rot < -180.0; ) new_drift_rot += 360.0;
      new_drift_rot = new_drift_rot / robot_state->translation_between_scans;

      robot_specifications->pos_corr_drift_trans += 
	(robot_specifications->pos_corr_lrate_drift *
	 new_drift_trans);
      robot_specifications->pos_corr_drift_rot += 
	(robot_specifications->pos_corr_lrate_drift *
	 new_drift_rot);
    }
  }
  /* fprintf(stderr, "DRIFT: %8.6f %8.6f -> %8.6f %8.6f\n", 
	  new_drift_trans, new_drift_rot,
	  robot_specifications->pos_corr_drift_trans,
	  robot_specifications->pos_corr_drift_rot);
  */

  /*
   * update the memory for the previous position
   */

  robot_state->prev_laser_x = robot_state->laser_x;
  robot_state->prev_laser_y = robot_state->laser_y;
  robot_state->prev_laser_orientation = robot_state->laser_orientation;
  robot_state->prev_laser_defined = 1;
}



/***************************************************************************
 ***************************************************************************
 ***************************************************************************/

inline float
hinge_dist(stack_item_ptr s1, stack_item_ptr s2)
{
  float hdiff_x = s1->hinge_x - s2->hinge_x;
  float hdiff_y = s1->hinge_y - s2->hinge_y;

  return sqrt((hdiff_x*hdiff_x)+(hdiff_y*hdiff_y));
}



inline char
check_hinge_dist(stack_item_ptr s1, stack_item_ptr s2, float dist)
{
  float hdiff_x, hdiff_y, factor = 1.0;

  hdiff_x = fabs(s1->hinge_x - s2->hinge_x);
  if (hdiff_x > dist)
    return 0;
  hdiff_y = fabs(s1->hinge_y - s2->hinge_y);
  if (hdiff_y > dist)
    return 0;
  if ((hdiff_x*hdiff_x) + (hdiff_y*hdiff_y) > dist * dist)
    return 0;
  return 1;
}


/***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#define GRANULARITY 3


inline void
match_items(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	    ROBOT_STATE_PTR robot_state,
	    int display_flag,
	    stack_item_ptr s1, stack_item_ptr s2,
	    float *error1, float *dx1, float *dy1, float *dorientation1,
	    int   *count)
{
  float hdiff_x, hdiff_y;
  int i, j, k, n;
  float x, y, x1, y1, x2, y2, nearest_x, nearest_y;
  float cos1, sin1, cos2, sin2, cos2_1, sin2_1, cos1_2, sin1_2;
  float dx1_2, dy1_2, dx2_1, dy2_1, dx_global_coord, dy_global_coord;
  float deltax1, deltax2, deltay1, deltay2;
  int   xint, yint;
  float local_e;
  float threshold;
  float factor = 2.0;

  /**error1 = *dx1 = *dy1 = *dorientation1 = 0.0;*/

  if (program_state->is_server)
    factor = 4.0;

  if (s1 == s2)
    return;

  /* ================================================== *
   *   check if there is sufficient overlap (pruning)
   * ================================================== */

  if (!check_hinge_dist(s1, s2, factor * 
			robot_specifications->pos_corr_hinge_dist_threshold))
    return;


  /* ================================================== *
   *   compute auxiliary variables
   * ================================================== */

  cos1 = cos(s1->laser_orientation * M_PI / 180.0);
  sin1 = sin(s1->laser_orientation * M_PI / 180.0);

  cos2 = cos(s2->laser_orientation * M_PI / 180.0);
  sin2 = sin(s2->laser_orientation * M_PI / 180.0);

  cos2_1 = cos((s2->laser_orientation - s1->laser_orientation) * M_PI / 180.0);
  sin2_1 = sin((s2->laser_orientation - s1->laser_orientation) * M_PI / 180.0);

  cos1_2 = cos((s1->laser_orientation - s2->laser_orientation) * M_PI / 180.0);
  sin1_2 = sin((s1->laser_orientation - s2->laser_orientation) * M_PI / 180.0);

  dx1_2 = s1->laser_x - s2->laser_x;
  dy1_2 = s1->laser_y - s2->laser_y;
  dx2_1 = -dx1_2;
  dy2_1 = -dy1_2;

  deltax1 =  (dx1_2*cos1) + (dy1_2*sin1);
  deltay1 = -(dx1_2*sin1) + (dy1_2*cos1);
  deltax2 =  (dx2_1*cos2) + (dy2_1*sin2);
  deltay2 = -(dx2_1*sin2) + (dy2_1*cos2);

  /* ================================================== *
   *    Match A
   * ================================================== */



  /*
   * compute error and gradients
   */


  for (i = 0; i < robot_specifications->num_sensors; i += GRANULARITY)


    if (s2->admissble_sensor_value[i] &
	(!(robot_specifications->ignore_front_laser) || !(is_front(i))) &&
	(!(robot_specifications->ignore_rear_laser) || !(is_rear(i)))){
      
      
      x2 = deltax1 + (s2->sensor_x[i]*cos2_1) - (s2->sensor_y[i]*sin2_1);
      y2 = deltay1 + (s2->sensor_x[i]*sin2_1) + (s2->sensor_y[i]*cos2_1);

      xint = (int) ((x2 + robot_specifications->pos_corr_max_range)
		    / robot_specifications->pos_resolution + 0.5);
      yint = (int) ((y2 + robot_specifications->pos_corr_max_range)
		    / robot_specifications->pos_resolution + 0.5);

      if (xint >= 0 && xint < robot_specifications->pos_map_dim &&
	  yint >= 0 && yint < robot_specifications->pos_map_dim){

	local_e = s1->error[xint][yint];

	if (local_e > 0.0 && local_e < 1.0){
	  x1 = nearest_x = s1->nearest_x[xint][yint];
	  y1 = nearest_y = s1->nearest_y[xint][yint];

	  /* count error */
	  *error1 += local_e;
	  *count  += 1;

	  /* display match */
	  if (display_flag && !program_state->is_server){
	    G_add_marker(MATCHES, y1, -x1,  0);
	    G_add_marker(MATCHES, y2, -x2, 0);
	  }

	  /* compute gradients */
	  dx_global_coord = ((x2-x1) * cos1) - ((y2-y1) * sin1);
	  dy_global_coord = ((x2-x1) * sin1) + ((y2-y1) * cos1);
          *dx1 -= (dx_global_coord * (1.0 - local_e));
          *dy1 -= (dy_global_coord * (1.0 - local_e));



	  n = s1->nearest_index[xint][yint];
	  if (n == -1)
	    fprintf(stderr, "UNUSUAL ERROR\n");
	  else
	  *dorientation1 -= 
	    (((x2-x1) * s1->dist_to_base[n] 
	      * s1->sin_sensor_angle_to_base[n]) -
	     ((y2-y1) * s1->dist_to_base[n] 
	      * s1->cos_sensor_angle_to_base[n]));
	}
      }
    }

  /* ================================================== *
   *    Match B
   * ================================================== */



  /*
   * compute error and gradients
   */

  for (i = 0; i < robot_specifications->num_sensors; i += GRANULARITY)
    if (s1->admissble_sensor_value[i] &&
	(!(robot_specifications->ignore_front_laser) || !(is_front(i))) &&
	(!(robot_specifications->ignore_rear_laser) || !(is_rear(i)))){

      x1 = s1->sensor_x[i];
      y1 = s1->sensor_y[i];
      x  = deltax2 + (x1*cos1_2) - (y1*sin1_2);
      y  = deltay2 + (x1*sin1_2) + (y1*cos1_2);

      xint = (int) ((x + robot_specifications->pos_corr_max_range)
		    / robot_specifications->pos_resolution + 0.5);
      yint = (int) ((y + robot_specifications->pos_corr_max_range)
		    / robot_specifications->pos_resolution + 0.5);
      if (xint >= 0 && xint < robot_specifications->pos_map_dim &&
	  yint >= 0 && yint < robot_specifications->pos_map_dim){

	local_e = s2->error[xint][yint];

	if (local_e > 0.0 && local_e < 1.0){

	  nearest_x = s2->nearest_x[xint][yint];
	  nearest_y = s2->nearest_y[xint][yint];

	  x2 = deltax1 + (nearest_x*cos2_1) - (nearest_y*sin2_1);
	  y2 = deltay1 + (nearest_x*sin2_1) + (nearest_y*cos2_1);

	  /* count error */
	  *error1 += local_e;
	  *count  += 1;

	  /* display match */
	  if (display_flag && !program_state->is_server){
	    G_add_marker(MATCHES, y2, -x2,  1);
	    G_add_marker(MATCHES, y1, -x1, 1);
	  }

	  /* compute gradients */
	  dx_global_coord = ((x2-x1) * cos1) - ((y2-y1) * sin1);
	  dy_global_coord = ((x2-x1) * sin1) + ((y2-y1) * cos1);
          *dx1 -= (dx_global_coord * (1.0 - local_e));
          *dy1 -= (dy_global_coord * (1.0 - local_e));

	  *dorientation1 -= 
	    (((x2-x1) * s1->dist_to_base[i] 
	      * s1->sin_sensor_angle_to_base[i]) -
	     ((y2-y1) * s1->dist_to_base[i] 
	      * s1->cos_sensor_angle_to_base[i]));

	}
      }
    }

  /* -------------------------------------------------- *
   * add up total error
   * -------------------------------------------------- */
}



/***************************************************************************
 ***************************************************************************
 ***************************************************************************/




inline void 
update_position(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		ROBOT_STATE_PTR robot_state, stack_item_ptr s1, 
		float error1, float dx1, float dy1, float dorientation1,
		int count)
{
  float factor;
  float delta_x, delta_y, delta_orientation;

  if (count > 0){
    factor = 1.0 / ((float) count);
    
    /* collect cumulative change */
    delta_x = robot_specifications->pos_corr_lrate_trans * dx1 * factor;
    delta_y = robot_specifications->pos_corr_lrate_trans * dy1 * factor;
    delta_orientation = 
      robot_specifications->pos_corr_lrate_rot * dorientation1 * factor;
    /* truncate */
    if (delta_x > robot_specifications->pos_corr_max_d_trans)
      delta_x = robot_specifications->pos_corr_max_d_trans;
    else if (delta_x < -robot_specifications->pos_corr_max_d_trans)
      delta_x = -robot_specifications->pos_corr_max_d_trans;
    
    if (delta_y > robot_specifications->pos_corr_max_d_trans)
      delta_y = robot_specifications->pos_corr_max_d_trans;
    else if (delta_y < -robot_specifications->pos_corr_max_d_trans)
      delta_y = -robot_specifications->pos_corr_max_d_trans;
    
    if (delta_orientation > robot_specifications->pos_corr_max_d_rot)
      delta_orientation = robot_specifications->pos_corr_max_d_rot;
    else if (delta_orientation < -robot_specifications->pos_corr_max_d_rot)
      delta_orientation = -robot_specifications->pos_corr_max_d_rot;
      
    
    /* apply cumulative change */
    
    s1->laser_x += delta_x;
    s1->laser_y += delta_y;
    s1->laser_orientation += delta_orientation;

    program_state->remaining_error = error1 * factor;
  }
  else
    program_state->remaining_error = 0.0;
}

/***************************************************************************
 ***************************************************************************
 ***************************************************************************/


/* --------------------------------------------------
 *
 * Phase 1: determine pose of latest reading relaitve to
 * its predecessor
 */

inline int
correct_position_phase1(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			ROBOT_STATE_PTR robot_state)
{
  float error1, dx1, dy1, dorientation1;
  stack_item_ptr s1, s2;
  int count;
  int found = 0;

  G_clear_markers(MATCHES);

  error1 = dx1 = dy1 = dorientation1 = 0.0;
  count = 0;
  s1 = &(program_state->stack[program_state->last_stack_item]);

  s2 = &(program_state->stack[(program_state->last_stack_item - 1 +
			       robot_specifications->stack_size) %
			     robot_specifications->stack_size]);
			    
  /*s2 = &(program_state->stack[program_state->nearest_stack_item]);*/

  match_items(robot_specifications, robot_state, 
	      !robot_specifications->display_fixed_rotation,
	      s1, s2,
	      &error1, &dx1, &dy1, &dorientation1, &count);

  update_position(robot_specifications, robot_state, s1,
		error1, dx1, dy1, dorientation1, count);

  return count;
}


/* --------------------------------------------------
 *
 * Phase 2: find out how latest reading relates to mesurements
 * further away in the past
 */





inline int
correct_position_phase2(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			ROBOT_STATE_PTR robot_state)
{
  float error1, dx1, dy1, dorientation1;
  stack_item_ptr s1, s2;
  int count, n, m;
  float factor = 2.0;
  
  if (program_state->is_server)
    factor = 4.0;


  G_clear_markers(MATCHES);

  error1 = dx1 = dy1 = dorientation1 = 0.0;
  count = 0;
  s1 = &(program_state->stack[program_state->last_stack_item]);



  for (n = program_state->last_stack_item;
       n != program_state->first_stack_item /* && count == 0 */;
       n = (n-1+robot_specifications->stack_size) %
	 robot_specifications->stack_size){

    m = (n-1+robot_specifications->stack_size)
      % robot_specifications->stack_size;
    s2 = &(program_state->stack[m]);
    
    if (check_hinge_dist(s1, s2, factor * 
			 robot_specifications->pos_corr_hinge_dist_threshold))
      match_items(robot_specifications, robot_state, 
		  !robot_specifications->display_fixed_rotation,
		  s1, s2,
		  &error1, &dx1, &dy1, &dorientation1, &count);
  }
  
  if (count == 0 && program_state->nearest_stack_item >= 0){
    s2 = &(program_state->stack[program_state->nearest_stack_item]);
    
    match_items(robot_specifications, robot_state, 
		!robot_specifications->display_fixed_rotation,
		s1, s2,
		&error1, &dx1, &dy1, &dorientation1, &count);
  }

  /*if (count == 0)
    fprintf(stderr, " $$$ ");*/

  update_position(robot_specifications, robot_state, s1,
		  error1, dx1, dy1, dorientation1, count);


  return count;
}


/* --------------------------------------------------
 *
 * Phase 3: adjust all other readings
 */

inline int
correct_position_phase3(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			ROBOT_STATE_PTR robot_state)
{
  float error1, dx1, dy1, dorientation1;
  stack_item_ptr s1, s2;
  int count, m, n, done, newest;


  for (m = program_state->last_stack_item, done = 0, newest = 1;
       !done;
       m = (m - 1 + robot_specifications->stack_size)
	 % robot_specifications->stack_size, newest = 0){ /* decrement m */

    s1 = &(program_state->stack[m]);
    error1 = dx1 = dy1 = dorientation1 = 0.0;
    count = 0;

    if (!newest)
      for (n = program_state->first_stack_item;
	   n != program_state->last_stack_item;
	   n = (n+1) % robot_specifications->stack_size)
	match_items(robot_specifications, robot_state, 0, s1, 
		    &(program_state->stack[n]),
		    &error1, &dx1, &dy1, &dorientation1, &count);
    
    update_position(robot_specifications, robot_state, s1,
		    error1, dx1, dy1, dorientation1, count);

    if (m == program_state->first_stack_item) done = 1;
  }

  return count;

}


/***************************************************************************
 ***************************************************************************
 ***************************************************************************/

inline void
fast_linear_adjustment(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		       ROBOT_STATE_PTR robot_state,
		       float target_x, float target_y, 
		       float target_orientation)
{
  float delta_x, delta_y, delta_orientation;
  stack_item_ptr s1;
  int count, i, j;
  float factor;


  s1 = &(program_state->stack[program_state->last_stack_item]); /* current */
    
  delta_x = s1->laser_x - target_x;
  delta_y = s1->laser_y - target_y;
  delta_orientation = s1->laser_orientation - target_orientation;

  for (; delta_orientation <= -180.0; ) delta_orientation += 360.0;
  for (; delta_orientation >   180.0; ) delta_orientation -= 360.0;
    
  /*fprintf(stderr,
     "\nMove %5.3f %5.3f %5.3f to %5.3f %5.3f %5.3f by %5.3f %5.3f %5.3f.\n",
     target_x, target_y, target_orientation,
     s1->laser_x, s1->laser_y, s1->laser_orientation,
     delta_x, delta_y, delta_orientation);*/
  
  factor =  1.0 / ((float) ((program_state->last_stack_item
			     - program_state->first_stack_item
			     + robot_specifications->stack_size) %
			    robot_specifications->stack_size));
  
  for (i = program_state->first_stack_item, j = 0;
       i != program_state->last_stack_item;
       i = (i+1) % robot_specifications->stack_size, j++){
    s1 = &(program_state->stack[i]);
    s1->laser_x += (((float) j) * factor * delta_x);
    s1->laser_y += (((float) j) * factor * delta_y);
    s1->laser_orientation += (((float) j) * factor * delta_orientation);
  }
}

/***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#define PHASE1 200
#define PHASE2 400



inline int
correct_position(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		 ROBOT_STATE_PTR robot_state)
{
  static stack_item_ptr s1;
  static float target_x = 0.0, target_y = 0.0, target_orientation = 0.0;
  static float best_E;
  static int phase = 0;
  int count;

  /*
   * Check if more than one data point available
   */

  if (program_state->last_stack_item == program_state->first_stack_item)
    return 0;			/* stack empty */
  
  


  /*
   * Control the phase
   */

  if (program_state->is_server || 
      !robot_specifications->do_backwards_corrections)
    phase = 2;
  else{
    if (program_state->position_control_iteration == 0){
      s1 = &(program_state->stack[program_state->last_stack_item]); /* current */
      phase = 1;
    }
    if (phase == 1 && program_state->position_control_iteration == PHASE1){
      target_x = s1->laser_x;
      target_y = s1->laser_y;
      target_orientation = s1->laser_orientation;
      phase  = 2;
    }
    if (phase == 2 && program_state->position_control_iteration == PHASE2){
      /*
	fast_linear_adjustment(robot_specifications, robot_state,
			     target_x, target_y, target_orientation);
			     */
      phase  = 3;
    }
  }



  /*
   * Loop over all sensor readings in the data base 
   */

  switch(phase){


  case 1:
    return correct_position_phase1(robot_specifications, robot_state);
    break;


  case 2:
    return correct_position_phase2(robot_specifications, robot_state);
    break;

  case 3:
    return correct_position_phase3(robot_specifications, robot_state);
    break;


  default:
    fprintf(stderr, "Strange error.\n");
    exit(-1);
  }


}



/***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include "gd.h"


inline void
gif_pixel(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	  gdImagePtr GifPic, float target_x, float target_y, 
	  int color, int color2, int big)
{
  int   int_x, int_y;
  int   n, max_n;
  int   dx[4] = {0, 0, 1, 1};
  int   dy[4] = {0, 1, 0, 1};

  int_x = (int) ((target_x / robot_specifications->world_size + 0.5) *
		 ((float) robot_specifications->gif_image_size));
  int_y = (int) ((target_y / robot_specifications->world_size + 0.5) *
		 ((float) robot_specifications->gif_image_size));
  if (big)
    max_n = 4;
  else
    max_n = 1;
  for (n = 0; n < max_n; n++){
    if (int_x+dx[n] >= 0 && 
	int_x+dx[n] < robot_specifications->gif_image_size &&
	int_y+dy[n] >= 0 &&
	int_y+dy[n] < robot_specifications->gif_image_size)
      gdImageSetPixel(GifPic, int_y+dy[n], int_x+dx[n], color);
    if (color2 >= 0 &&
	int_x+dx[n]-1 >= 0 && 
	int_x+dx[n]-1 < robot_specifications->gif_image_size &&
	int_y+dy[n]-1 >= 0 &&
	int_y+dy[n]-1 < robot_specifications->gif_image_size)
      gdImageSetPixel(GifPic, int_y+dy[n]-1, int_x+dx[n]-1, color2);
  }
}



/***************************************************************************
 ***************************************************************************
 ***************************************************************************/


float path_x[NUM_PATH_ITEMS];
float path_y[NUM_PATH_ITEMS];
float path_dist[NUM_PATH_ITEMS];
int   path_robot[NUM_PATH_ITEMS];
int num_path_items = 0;

void
display_scans_and_map(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		      PROGRAM_STATE_PTR program_state,
		      ROBOT_STATE_PTR robot_state, int force_all)
{
  float rel_x = 0.0, rel_y = 0.0, rel_o = 0.0, rel_o2;
  float display_rel_x = 0.0, display_rel_y = 0.0, display_rel_o = 0.0;
  float display_rel_o2;
  int i, j, k, min_k, max_k, max_i, increment;
  float display_x, display_y, display_orientation;
  float *display_sensor_values;
  float x, y, orientation, orientation_aux, sin_o, cos_o, sin_o_aux, cos_o_aux;
  stack_item_ptr s;
  float target_x, target_y, dist, fact;
  float target_x2 = 0.0, target_y2 = 0.0, target_x3, target_y3;
  float min_x, min_y, max_x, max_y, offs_x_aux, offs_y_aux;

  FILE *iop = NULL;
  gdImagePtr GifPic = NULL;
  static int white, black, red, green, blue, grey230;
  static int colors_initialized = 0;
  static int fileno = 1;
  static int counter = 0;
  static int counter2 = 0;
  char filename[128];


  if (program_state->graphics_initialized){

    
    if (robot_specifications->display_density >= 1 && !force_all)
      increment = robot_specifications->display_density;
    else
      increment = 1;


    /*
     * Open GIF file
     */
    if (robot_specifications->gif_image_frequency >= 1 &&
	counter++ % robot_specifications->gif_image_frequency == 0){
      
      GifPic = gdImageCreate(robot_specifications->gif_image_size,
			     robot_specifications->gif_image_size);
      
      sprintf(filename, "laserint%.5d.gif", fileno++);
      if ((iop = fopen(filename, "w")) == 0)
	fprintf(stderr, "WARNING: Could not open output file %s.\n", filename);
      
      white   = gdImageColorAllocate(GifPic, 255, 255, 255);
      grey230 = gdImageColorAllocate(GifPic, 230, 230, 230);
      red     = gdImageColorAllocate(GifPic, 255, 0, 0);
      black   = gdImageColorAllocate(GifPic, 0, 0, 0);
      green   = gdImageColorAllocate(GifPic, 0, 128, 0);
      blue    = gdImageColorAllocate(GifPic, 0, 0, 255);
      
      if (iop){
	for (i = 0; i < robot_specifications->gif_image_size; i++)
	  for (j = 0; j < robot_specifications->gif_image_size; j++)
	    gdImageSetPixel(GifPic, i, j, grey230);
	
      }
    }

    /*
     * Set pointer
     */
	
    s = &(program_state->stack[program_state->last_stack_item]);


    /*
     * save path
     */ 
    
    if (num_path_items > 0)
      dist = sqrt(((s->laser_x - path_x[num_path_items-1]) *
		   (s->laser_x - path_x[num_path_items-1])) +
		  ((s->laser_y - path_y[num_path_items-1]) *
		   (s->laser_y - path_y[num_path_items-1])));
    else
      dist = 0.0;
    

    if (num_path_items == 0 || dist >= robot_specifications->resolution){
      path_x[num_path_items] = s->laser_x;
      path_y[num_path_items] = s->laser_y;
      path_dist[num_path_items] = dist;
      path_robot[num_path_items] = s->robot_number;
      num_path_items++;
    }



    if (force_all ||
	(robot_specifications->display_interval >= 1 &&
	 counter++ % robot_specifications->display_interval == 0)){

      /*
       * Compute obstacle coordinates
       */


      x = s->laser_x;
      y = s->laser_y;
	

      if (robot_specifications->display_fixed_rotation){
	orientation = 0.0;
	orientation_aux = 90.0 - s->laser_orientation;

	min_x = max_x = s->laser_x;
	min_y = max_y = s->laser_y;
	for (i = program_state->first_stack_item;
	     i != program_state->last_stack_item;
	     i = (i + 1) % robot_specifications->stack_size){
	  if (program_state->stack[i].laser_x < min_x)
	    min_x = program_state->stack[i].laser_x;
	  if (program_state->stack[i].laser_x > max_x)
	    max_x = program_state->stack[i].laser_x;
	  if (program_state->stack[i].laser_y < min_y)
	    min_y = program_state->stack[i].laser_y;
	  if (program_state->stack[i].laser_y > max_y)
	    max_y = program_state->stack[i].laser_y;
	}
	program_state->offs_x = 0.5 * (max_x + min_x) - s->laser_x;
	program_state->offs_y = 0.5 * (max_y + min_y) - s->laser_y;
      }
      else{
	orientation = s->laser_orientation;
	orientation_aux = 90.0;
	program_state->offs_x = 0.0;
	program_state->offs_y = 0.0;
      }
      sin_o = sin(orientation * M_PI / 180.0);
      cos_o = cos(orientation * M_PI / 180.0);
      sin_o_aux = sin(orientation_aux * M_PI / 180.0);
      cos_o_aux = cos(orientation_aux * M_PI / 180.0);

      G_clear_markers(PREV_OBSTACLES);


      for (k = program_state->first_stack_item;
	   k != (program_state->last_stack_item + 1) %
	     robot_specifications->stack_size;
	   k = (k + 1) % robot_specifications->stack_size){
	{
	  s = &(program_state->stack[k]);

	  display_sensor_values = s->sensor_values;
	  display_x = s->laser_x;
	  display_y = s->laser_y;
	  display_orientation = s->laser_orientation;
	
	  display_rel_x = ((cos_o * (display_x - x)) +
			   (sin_o * (display_y - y)));
	  display_rel_y = ((cos_o * (display_y - y)) -
			   (sin_o * (display_x - x)));
	  display_rel_o = display_orientation - orientation;
	  display_rel_o2 = display_rel_o * M_PI / 180.0;


	  for (i = 0; i < robot_specifications->num_sensors &&
		 display_sensor_values; i += increment)
	  
	    if (display_sensor_values[i] >= 0.0 &&
		display_sensor_values[i] < 
		robot_specifications->pos_corr_max_range){

	      target_y = program_state->offs_y - display_rel_y
		+ (cos(display_rel_o2) * s->sensor_y[i])
		+ (sin(display_rel_o2) * s->sensor_x[i]);
	      target_x = program_state->offs_x - display_rel_x 
		+ (cos(display_rel_o2) * s->sensor_x[i])
		- (sin(display_rel_o2) * s->sensor_y[i]);

	      if ((robot_specifications->ignore_front_laser && is_front(i)) ||
		  (robot_specifications->ignore_rear_laser && is_rear(i))){
	      
		G_add_marker(PREV_OBSTACLES, target_y, -target_x, 2);
		if (iop) gif_pixel(robot_specifications,
				   GifPic, target_x, target_y, black, white, 0);
	      }
	      else{
		G_add_marker(PREV_OBSTACLES, target_y, -target_x,
			     k == program_state->last_stack_item);
		if (iop) gif_pixel(robot_specifications,
				   GifPic, target_x, target_y, black, white, 0);
	      }
	    }
	}
      }
    
      /*
       * draw path
       */



      if (program_state->is_server)
	for (i = 0; i < MAX_NUMBER_ROBOTS; i++)
	  G_clear_markers(PATH[i]);
      else
	G_clear_markers(PATH[0]);

      for (k = 0; k < num_path_items; k++){
      
	display_x = path_x[k];
	display_y = path_y[k];
      
	target_x = program_state->offs_x - (((cos_o * (display_x - x)) +
			      (sin_o * (display_y - y))));
	target_y = program_state->offs_y - (((cos_o * (display_y - y)) -
			      (sin_o * (display_x - x))));

      
	if (program_state->is_server){
	  G_add_marker(PATH[path_robot[k]], target_y, -target_x, 0);
	}
	else
	  G_add_marker(PATH[0], target_y, -target_x, 0);
      

	if (k > 0){
	  for (i = 0; i < (int) path_dist[k]; i++){
	
	    fact = ((float) i) / path_dist[k];
	    target_x3 = (fact * target_x) + ((1.0 - fact) * target_x2);
	    target_y3 = (fact * target_y) + ((1.0 - fact) * target_y2);




	    if (iop) gif_pixel(robot_specifications,
			       GifPic, target_x3, target_y3, red, -1, 0);
	  }
	}

	target_x2 = target_x;
	target_y2 = target_y;
      }


      /*
       * draw objects
       */


      for (k = 0; k < program_state->number_objects; k++){
	G_clear_markers(OBJECTS[k]);
	if (program_state->objects[k]){
	  for (i = 0; i < 5; i++){
	    switch(i){
	    case 0:
	      display_x = program_state->objects[k]->x;
	      display_y = program_state->objects[k]->y;
	      break;
	    case 1:
	      display_x = program_state->objects[k]->x +
		(cos(program_state->objects[k]->angle * M_PI / 180.0)
		 * program_state->objects[k]->length);
	      display_y = program_state->objects[k]->y +
		(sin(program_state->objects[k]->angle * M_PI / 180.0)
		 * program_state->objects[k]->length);
	      break;
	    case 2:
	      display_x = program_state->objects[k]->x +
		(cos(program_state->objects[k]->angle * M_PI / 180.0)
		 * program_state->objects[k]->length) -
		(sin(program_state->objects[k]->angle * M_PI / 180.0)
		 * program_state->objects[k]->width);
	      display_y = program_state->objects[k]->y +
		(sin(program_state->objects[k]->angle * M_PI / 180.0)
		 * program_state->objects[k]->length) +
		(cos(program_state->objects[k]->angle * M_PI / 180.0)
		 * program_state->objects[k]->width);
	      break;
	    case 3:
	      display_x = program_state->objects[k]->x -
		(sin(program_state->objects[k]->angle * M_PI / 180.0)
		 * program_state->objects[k]->width);
	      display_y = program_state->objects[k]->y +
		(cos(program_state->objects[k]->angle * M_PI / 180.0)
		 * program_state->objects[k]->width);
	      break;
	    case 4:
	      display_x = program_state->objects[k]->x;
	      display_y = program_state->objects[k]->y;
	      break;
	    }
	    
	    
	    display_x *= -1.0;
	    display_y *= -1.0;

	    target_x = program_state->offs_x - (((cos_o * (display_x - x)) +
						 (sin_o * (display_y - y))));
	    target_y = program_state->offs_y - (((cos_o * (display_y - y)) -
						 (sin_o * (display_x - x))));
	    G_add_marker(OBJECTS[k], target_y, -target_x, 0);
	  }
	}    
      }

    
      /*
       * draw robot
       */
      if (iop){
	float long_x, short_x;
	gdPoint p[5];

	long_x  = robot_specifications->robot_size
	  / robot_specifications->world_size;
	short_x = 0.8 * long_x;
	offs_x_aux = program_state->offs_x / robot_specifications->world_size;
	offs_y_aux = program_state->offs_y / robot_specifications->world_size;

	p[0].x = (int) ((offs_y_aux
			 + (- sin_o_aux * short_x + cos_o_aux * long_x + 0.5))
			* ((float) robot_specifications->gif_image_size));
	p[0].y = (int) ((offs_x_aux
			 + (cos_o_aux * short_x + sin_o_aux * long_x + 0.5))
			* ((float) robot_specifications->gif_image_size));
	p[1].x = (int) ((offs_y_aux
			 + (-sin_o_aux * short_x - cos_o_aux * long_x + 0.5))
			* ((float) robot_specifications->gif_image_size));
	p[1].y = (int) ((offs_x_aux
			 + (cos_o_aux * short_x - sin_o_aux * long_x + 0.5))
			* ((float) robot_specifications->gif_image_size));
	p[2].x = (int) ((offs_y_aux
			 + (sin_o_aux * short_x - cos_o_aux * long_x + 0.5))
			* ((float) robot_specifications->gif_image_size));
	p[2].y = (int) ((offs_x_aux
			 + (-cos_o_aux * short_x - sin_o_aux * long_x + 0.5))
			* ((float) robot_specifications->gif_image_size));
	p[3].x = (int) ((offs_y_aux
			 + (sin_o_aux * short_x + cos_o_aux * long_x + 0.5))
			* ((float) robot_specifications->gif_image_size));
	p[3].y = (int) ((offs_x_aux
			 + (-cos_o_aux * short_x + sin_o_aux * long_x + 0.5))
			* ((float) robot_specifications->gif_image_size));
      
	gdImageFilledPolygon(GifPic, p, 4, red);
	p[0].x = (int) ((offs_y_aux - cos_o_aux * long_x + 0.5)
			* ((float) robot_specifications->gif_image_size));
	p[0].y = (int) ((offs_x_aux - sin_o_aux * long_x + 0.5)
			* ((float) robot_specifications->gif_image_size));
	p[1].x = (int) ((0.5 + offs_y_aux) 
			* ((float) robot_specifications->gif_image_size));
	p[1].y = (int) ((0.5 + offs_x_aux) 
			* ((float) robot_specifications->gif_image_size));

	gdImageFilledPolygon(GifPic, p, 2, white);
      }


      /* 
       * Now, finally, the new sensor values are displayed.
       */
    
      G_display_switch(LOCAL_BACKGROUND, 0);
      G_display_matrix(LOCAL_ERROR);
      G_display_matrix(LOCAL_MAPVALUES);
      G_display_matrix(OBJ_DIST);
      G_display_markers(MATCHES);
      G_display_markers(REGRESSION);
      G_display_markers(PREV_OBSTACLES);
      G_display_markers(OBSTACLES);
      if (program_state->is_server)
	for (i = 0; i < MAX_NUMBER_ROBOTS; i++)
	  G_display_markers(PATH[i]);
      else
	G_display_markers(PATH[0]);
      for (i = 0; i < program_state->number_objects; i++)
	if (program_state->objects[i])
	  G_display_markers(OBJECTS[i]);
      if (robot_specifications->display_fixed_rotation){
	G_display_robot(LOCAL_ROBOT, program_state->offs_y, 
			-program_state->offs_x, 90.0+s->laser_orientation,
			0, NULL);
	/*fprintf(stderr, "OFFS: %g %g\n", 
		program_state->offs_y, 
		-program_state->offs_x);*/
      }
      else
	G_display_robot(LOCAL_ROBOT, 0.0, 0.0, 90.0, 0, NULL);
    }

    /*
     * Close GIF file
     */
    if (iop){
      gdImageGif(GifPic, iop);
      fclose(iop);
    }
  }

}
   
/***************************************************************************
 ***************************************************************************
 ***************************************************************************/
   


int 
find_matching_scan(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		   PROGRAM_STATE_PTR program_state,
		   int *match, int *cache)
{
  stack_item_ptr s = &(program_state->stack[program_state->last_stack_item]);
  stack_item_ptr s2;
  float distance, angular_distance, sum_distance;
  float min_dist = 0.0;
  int nearest_item = -1;
  int i;
  
  *match = 0;
  *cache = 1;

  for (i = program_state->first_stack_item;
       i != program_state->last_stack_item;
       i = (i + 1) % robot_specifications->stack_size){
       
    s2 = &(program_state->stack[i]);

    distance = 
      sqrt(((s->hinge_x - s2->hinge_x) *(s->hinge_x - s2->hinge_x)) +
	   ((s->hinge_y - s2->hinge_y) *(s->hinge_y - s2->hinge_y)));

    angular_distance = s->laser_orientation - s2->laser_orientation;
    for (;angular_distance < -180.0;) angular_distance += 360.0;
    for (;angular_distance > 180.0;)  angular_distance -= 360.0;
    angular_distance = fabs(angular_distance);
    
    sum_distance = angular_distance + distance;
    
    if (i == program_state->first_stack_item ||	sum_distance < min_dist){
      
      min_dist = sum_distance;
      nearest_item = i;
      
      
      if (match != NULL &&
	  distance < robot_specifications->pos_corr_max_dist &&
	  fabs(angular_distance) < robot_specifications->pos_corr_max_angle)
	*match = 1;


      if (cache != NULL &&
	  (distance < robot_specifications->pos_corr_hinge_dist_threshold &&
	   fabs(angular_distance) <
	   robot_specifications->pos_corr_rot_cutoff_angle)){
	/*if (*cache == 1)
	  fprintf(stderr, " <%d: %g %g>", i, distance, fabs(angular_distance));*/
	*cache = 0;
      }
    }
  }    

  if (nearest_item == -1)
    nearest_item = (program_state->last_stack_item - 1 +
		    robot_specifications->stack_size) % 
      robot_specifications->stack_size;

  return nearest_item;
}


/***************************************************************************
 ***************************************************************************
 ***************************************************************************/


void
temp_cache_reading(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		   PROGRAM_STATE_PTR program_state,
		   ROBOT_STATE_PTR robot_state)
{
  robot_state->prev_laser_x = robot_state->x;
  robot_state->prev_laser_y = robot_state->y;
  robot_state->prev_laser_orientation = robot_state->orientation;
  robot_state->prev_laser_defined = 1;

  compute_aux_values(robot_specifications, program_state, robot_state,
		     robot_state->laser_x,
		     robot_state->laser_y,
		     robot_state->laser_orientation,
		     robot_state->raw_odometry_x,
		     robot_state->raw_odometry_y,
		     robot_state->raw_odometry_orientation,
		     robot_specifications->num_sensors,
		     robot_state->sensor_values,
		     robot_specifications->ignore_front_laser,
		     robot_specifications->ignore_rear_laser,
		     robot_specifications->front_laser_offset_x,
		     robot_specifications->front_laser_offset_y,
		     robot_specifications->rear_laser_offset_x,
		     robot_specifications->rear_laser_offset_y,
		     -1);


}




void
compute_aux_values(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		   PROGRAM_STATE_PTR program_state,
		   ROBOT_STATE_PTR robot_state,
		   float input__laser_x,
		   float input__laser_y,
		   float input__laser_orientation,
		   float input__raw_odometry_x,
		   float input__raw_odometry_y,
		   float input__raw_odometry_orientation,
		   int   input__num_sensors,
		   float *input__sensor_values,
		   int   input__ignore_front_laser,
		   int   input__ignore_rear_laser,
		   float input__front_laser_offset_x,
		   float input__front_laser_offset_y,
		   float input__rear_laser_offset_x,
		   float input__rear_laser_offset_y,
		   int   robot_number)
{
  stack_item_ptr s = &(program_state->stack[program_state->last_stack_item]);
  int i, j, k, a, local_index, nearest_k;
  float x, y, x2, y2, alpha, dist, measurement, dist_x, dist_y;
  float min_dist, error, nearest_x, nearest_y, angle;
  float translation, rotation_1, rotation_2;
  int close_enough;

  static int local_table_initialized = 0;
  static float **local_table_dist = NULL;
  static int   **local_table_sensor_index = NULL;

  /*fprintf(stderr, "-a-");*/
  /*
   * set robot number
   */ 

  s->robot_number = robot_number;

  /*
   * allocate memory
   */ 

  allocate_stack_item(robot_state, program_state, robot_specifications,
		      program_state->last_stack_item);

/*   fprintf(stderr, "-b-"); */
  /*
   * save the coordinates on stack
   */ 


  s->laser_x = input__laser_x;
  s->laser_y = input__laser_y;
  s->laser_orientation = input__laser_orientation;

  s->raw_odometry_laser_x = input__raw_odometry_x;
  s->raw_odometry_laser_y = input__raw_odometry_y;
  s->raw_odometry_laser_orientation = input__raw_odometry_orientation;

  /*   fprintf(stderr, "-c-"); */
  /*
   * save the sensor item on stack (queue)
   */ 

  for (i = 0; i < input__num_sensors; i++){
    s->sensor_values[i] = input__sensor_values[i];
    if (input__sensor_values[i] < 0.0 ||
	input__sensor_values[i] > 
	robot_specifications->pos_corr_max_range ||
	(input__ignore_front_laser && is_front(i)) ||
	(input__ignore_rear_laser && is_rear(i)))
      s->admissble_sensor_value[i] = 0;
    else
      s->admissble_sensor_value[i] = 1;
  }
  
  s->hinge_x = input__laser_x 
    + (robot_specifications->pos_corr_hinge_point_offset
       * cos(input__laser_orientation * M_PI / 180.0));
  s->hinge_y = input__laser_y 
    + (robot_specifications->pos_corr_hinge_point_offset
       * sin(input__laser_orientation * M_PI / 180.0));
  
  /*   fprintf(stderr, "-d-"); */
  /*
   * initialize stack
   */

  

  /*
   * initialize the look-up table, makes later computation faster
   */
  
  if (!local_table_initialized){

    if (input__num_sensors != 360){
      fprintf(stderr, "ERROR: Currently works only for 360 laser beams\n");
      exit(1);      
    }
    local_table_dist = 
      (float **) malloc(sizeof(float *) * robot_specifications->pos_map_dim);
    local_table_sensor_index = 
      (int **) malloc(sizeof(int *) * robot_specifications->pos_map_dim);
    if (!local_table_dist || !local_table_sensor_index){
      printf("ABORT: out of memory X!\n");
      exit(1);
    }
    
    for (i = 0; i < robot_specifications->pos_map_dim; i++){
      local_table_dist[i] = 
	(float *) malloc(sizeof(float) * robot_specifications->pos_map_dim);
      local_table_sensor_index[i] = 
	(int *) malloc(sizeof(int) * robot_specifications->pos_map_dim);
      
      if (!local_table_dist[i] || !local_table_sensor_index[i]){
	printf("ABORT: out of memory X!\n");
	exit(1);
      }
    }

    for (i = 0, x = -robot_specifications->pos_corr_max_range +
	   0.5 * robot_specifications->pos_resolution;
	 i < robot_specifications->pos_map_dim; 
	 i++, x += robot_specifications->pos_resolution){
      for (j = 0, y = -robot_specifications->pos_corr_max_range +
	     0.5 * robot_specifications->pos_resolution;
	   j < robot_specifications->pos_map_dim; 
	   j++, y += robot_specifications->pos_resolution){


	local_table_sensor_index[i][j] = -1;
	local_table_dist[i][j] = -1.0;
	/* first check front laser */
	x2 = x + input__front_laser_offset_x;
	y2 = y + input__front_laser_offset_y;
	dist = sqrt((x2*x2)+(y2*y2));
	alpha = (atan2(y2, x2) * 180.0 / M_PI) - 90.0;
	for (; alpha <= -180.0; ) alpha += 360.0;
	for (; alpha >=  180.0; ) alpha -= 360.0;
	if (alpha >= 0.0 && alpha < 180.0 &&
	    !input__ignore_front_laser){
	  if (dist > robot_specifications->pos_corr_max_range){
	    local_table_sensor_index[i][j] = -1;
	    local_table_dist[i][j] = -1.0;
	  }
	  else{
	    local_table_sensor_index[i][j] = (int) alpha;
	    local_table_dist[i][j] = dist;
	  }
	}
	else{      /* then check rear laser */
	  x2 = x + input__rear_laser_offset_x;
	  y2 = y + input__rear_laser_offset_y;
	  dist = sqrt((x2*x2)+(y2*y2));
	  alpha = (atan2(y2, x2) * 180.0 / M_PI) + 90.0;
	  for (; alpha <= -180.0; ) alpha += 360.0;
	  for (; alpha >=  180.0; ) alpha -= 360.0;
	  if (alpha >= 0.0 && alpha < 180.0 &&
	    !input__ignore_rear_laser){
	    if (dist > robot_specifications->pos_corr_max_range){
	      local_table_sensor_index[i][j] = -1;
	      local_table_dist[i][j] = -1.0;
	    }
	    else{
	      local_table_sensor_index[i][j] = ((int) alpha) + 180;
	      local_table_dist[i][j] = dist;
	    }
	  }
	}

      }
    }
    local_table_initialized = 1;
    fprintf(stderr, "Lookup tables initialized\n");
  }

  /*   fprintf(stderr, "-e-"); */
  /*
   * compute sensor endpoints
   */

  for (i = 0; i < input__num_sensors; i++){
    if (s->sensor_values[i] >= 0.0){
      s->sensor_x[i] = 
	(cos(((float) (i+90)) * M_PI / 180.0) * s->sensor_values[i]);
      s->sensor_y[i] = 
	(sin(((float) (i+90)) * M_PI / 180.0) * s->sensor_values[i]);

      if ((input__ignore_front_laser && is_front(i)) ||
	  (input__ignore_rear_laser && is_rear(i))){
	s->sensor_z[i] = s->sensor_x[i];
	s->sensor_x[i] = 0.0;
      }
      else
	s->sensor_z[i] = 0.0;

      if (i < input__num_sensors / 2){
	s->sensor_x[i] -= input__front_laser_offset_x;
	s->sensor_y[i] -= input__front_laser_offset_y;
      }
      else{
	s->sensor_x[i] -= input__rear_laser_offset_x;
	s->sensor_y[i] -= input__rear_laser_offset_y;
      }
      angle = atan2(s->sensor_y[i], s->sensor_x[i]);
      s->sensor_angle_to_base[i]     = angle * 180.0 / M_PI;
      s->sin_sensor_angle_to_base[i] = sin(angle);
      s->cos_sensor_angle_to_base[i] = cos(angle);
      s->dist_to_base[i] =
	sqrt((s->sensor_x[i] * s->sensor_x[i]) 
	     + (s->sensor_y[i] * s->sensor_y[i]));
    }
    else
      s->sensor_x[i] = 
	s->sensor_y[i] = 
	s->sin_sensor_angle_to_base[i] =
	s->cos_sensor_angle_to_base[i] = 
	s->dist_to_base[i] = 0.0;
  }
  /*   fprintf(stderr, "-f-"); */

  /*
   * construct error map
   */

  
  for (i = 0; i < robot_specifications->pos_map_dim; i++)
    for (j = 0; j < robot_specifications->pos_map_dim; j++){
      nearest_x = nearest_y = 0.0;
      nearest_k = -1;
      if (local_table_sensor_index[i][j] == -1)	/* not in sensor reach */
	error = 0.0;
      else{
	measurement = s->sensor_values[local_table_sensor_index[i][j]];
	if (measurement < 0.0) /* no sensor data */
	  error = 0.0;
	else if (local_table_dist[i][j] > measurement + /* behind obstacle */
		 0.5 * robot_specifications->pos_resolution)
	  error = 0.0;
	else{
	  x = (((float) i) * robot_specifications->pos_resolution)
	    - robot_specifications->pos_corr_max_range;
	  y = (((float) j) * robot_specifications->pos_resolution)
	    - robot_specifications->pos_corr_max_range;
	  min_dist = -1.0;
	  for (k = 0; k < input__num_sensors; k++){
	    if ((!(input__ignore_front_laser) ||
		 !(is_front(k))) &&
		(!(input__ignore_rear_laser) || 
		 !(is_rear(k)))){
	      if (s->sensor_values[k] >= 0.0){
		dist_x = s->sensor_x[k] - x;
		dist_y = s->sensor_y[k] - y;
		dist = (dist_x * dist_x) + (dist_y * dist_y);
		if (dist < min_dist || min_dist < 0.0){
		  min_dist = dist;
		  nearest_k = k;
		  nearest_x = s->sensor_x[k];
		  nearest_y = s->sensor_y[k];
		}
	      }
	    }
	  }
	  error = sqrt(min_dist) / robot_specifications->pos_corr_max_dist;
	  if (error > 1.0) error = 1.0;
	}
      }

      s->error[i][j]         = error;
      s->nearest_x[i][j]     = nearest_x;
      s->nearest_y[i][j]     = nearest_y;
      s->nearest_index[i][j] = nearest_k;

      /*
       * update display
       */

      local_index = (j * robot_specifications->pos_map_dim) 
	+ (robot_specifications->pos_map_dim - i - 1);
      if (program_state->regular_local_map_display <= 1)
	local_error[local_index] = error;
    }

  /*   fprintf(stderr, "-g-"); */
  /*
   * find first scan for cross-matching
   */ 

  program_state->nearest_stack_item =
    find_matching_scan(robot_specifications, program_state,
		       &(program_state->found_nearest_stack_item),
		       &(program_state->cache));

  /*   fprintf(stderr, "-h-"); */
}  
		   





void
cache_reading(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	      PROGRAM_STATE_PTR program_state,
	      ROBOT_STATE_PTR robot_state)
{
  stack_item_ptr s = &(program_state->stack[program_state->last_stack_item]);
  int n_prev;
  float distance, angular_distance;
  int cache = 0;

  program_state->nearest_stack_item =
    find_matching_scan(robot_specifications, program_state,
		       &(program_state->found_nearest_stack_item),
		       &(program_state->cache));


  if (program_state->last_stack_item == program_state->first_stack_item)
    cache = 1;			/* stack is empty, cache the reading */

  else if (program_state->cache)
    cache = 1;			/* to far away from anything known */

  if (program_state->strong_turn > 0)
    cache = 0;
  

  /*
   * increment stack pointer
   */

  if (cache){



    if (!program_state->is_server && s->robot_number == -1)
      send_stack_item_to_server(robot_specifications, program_state,
				robot_state, 1);


    program_state->last_stack_item = 
      (program_state->last_stack_item + 1) % robot_specifications->stack_size;
    
    /* need more space? */
    if (program_state->last_stack_item == program_state->first_stack_item)
      program_state->first_stack_item = 
	(program_state->first_stack_item + 1)
	% robot_specifications->stack_size;
#ifndef FGAN
    putc(7,stderr);
#endif
    fprintf(stderr, " [%d,%d]", program_state->first_stack_item,
	    program_state->last_stack_item);
  }
  else if (!program_state->is_server && s->robot_number == -1)
    send_stack_item_to_server(robot_specifications, program_state,
			      robot_state, 0);

}



void
compute_polar_difference(float from_x, float from_y, float from_o,
			 float   to_x, float   to_y, float   to_o,
			 float *translation, float *rotation_1, 
			 float *rotation_2)
{
  if (from_x == to_x && from_y == to_y){
    *translation = 0.0;
    *rotation_1  = to_o - from_o;
    for (; *rotation_1 >   180.0 ;) *rotation_1 -= 360.0;
    for (; *rotation_1 <= -180.0 ;) *rotation_1 += 360.0;
    *rotation_2  = 0.0;
  }
  else{
    *translation  = sqrt(((to_x - from_x) * (to_x - from_x)) +
			 ((to_y - from_y) * (to_y - from_y)));
    *rotation_1 = 
      (atan2(to_y - from_y, to_x - from_x) * 180.0 / M_PI) - from_o;
    for (; *rotation_1 >   180.0 ;) *rotation_1 -= 360.0;
    for (; *rotation_1 <= -180.0 ;) *rotation_1 += 360.0;
    *rotation_2 = to_o - from_o - *rotation_1;
    for (; *rotation_2 >   180.0 ;) *rotation_2 -= 360.0;
    for (; *rotation_2 <= -180.0 ;) *rotation_2 += 360.0;
  }
}


/***************************************************************************
 ***************************************************************************
 ***************************************************************************/


static int first_scan = 1;
static int first_item_1 = 1;
static int first_item_2 = 1;
static float* prev_sensors_x = NULL;
static float* prev_sensors_y = NULL;
static float* prev_sensors_z = NULL;
static float* prev_sensor_values = NULL;
static int num_items = 0;
static int plot_is_on = 0;

void
plot_on(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	PROGRAM_STATE_PTR program_state,
	ROBOT_STATE_PTR robot_state,
	char *filename1, char *filename2, char *filename3)
{
  char local_filename[128];

  if (plot_iop != NULL || vr_iop1 != NULL || vr_iop2 != NULL || 
      vr_iop3 != NULL || vr_iop4 != NULL || 
      smf_iop1 != NULL || smf_iop2 != NULL)
    return;

  if (!robot_specifications->generate_3D_vrml &&
      !robot_specifications->generate_3D_smf &&
      !robot_specifications->generate_3D_math)
    return;


  if (program_state->graphics_initialized)
    G_display_switch(PLOT_BUTTON, 1);

  if (filename1 && robot_specifications->generate_3D_math){
    if ((plot_iop = fopen(filename1, "w")) == 0){
      fprintf(stderr, "WARNING: Could not open plot file %s.\n", filename1);
      if (program_state->graphics_initialized)
	G_display_switch(PLOT_BUTTON, 0);
      plot_iop = NULL;
      vr_iop1 = NULL;
      vr_iop2 = NULL;
      vr_iop3 = NULL;
      vr_iop4 = NULL;
      return;
    }
  }


  if (filename2 && robot_specifications->generate_3D_vrml){
    sprintf(local_filename, "%s.1", filename2);
    if ((vr_iop1 = fopen(local_filename, "w")) == 0){
      fprintf(stderr, "WARNING: Could not open vrml file %s.\n", local_filename);
      if (program_state->graphics_initialized)
      G_display_switch(PLOT_BUTTON, 0);
      plot_iop = NULL;
      vr_iop1 = NULL;
      vr_iop2 = NULL;
      vr_iop3 = NULL;
      vr_iop4 = NULL;
      smf_iop1 = NULL;
      smf_iop2 = NULL;
      return;
    }


    sprintf(local_filename, "%s.2", filename2);
    if ((vr_iop2 = fopen(local_filename, "w")) == 0){
      fprintf(stderr, "WARNING: Could not open vrml file %s.\n", local_filename);
      if (program_state->graphics_initialized)
	G_display_switch(PLOT_BUTTON, 0);
      plot_iop = NULL;
      vr_iop1 = NULL;
      vr_iop2 = NULL;
      vr_iop3 = NULL;
      vr_iop4 = NULL;
      smf_iop1 = NULL;
      smf_iop2 = NULL;
    return;
    }

    sprintf(local_filename, "%s.3", filename2);
    if ((vr_iop3 = fopen(local_filename, "w")) == 0){
      fprintf(stderr, "WARNING: Could not open vrml file %s.\n", local_filename);
      if (program_state->graphics_initialized)
	G_display_switch(PLOT_BUTTON, 0);
      plot_iop = NULL;
      vr_iop1 = NULL;
      vr_iop2 = NULL;
      vr_iop3 = NULL;
      vr_iop4 = NULL;
      smf_iop1 = NULL;
      smf_iop2 = NULL;
    return;
    }

    sprintf(local_filename, "%s.4", filename2);
    if ((vr_iop4 = fopen(local_filename, "w")) == 0){
      fprintf(stderr, "WARNING: Could not open vrml file %s.\n", local_filename);
      if (program_state->graphics_initialized)
	G_display_switch(PLOT_BUTTON, 0);
      plot_iop = NULL;
      vr_iop1 = NULL;
      vr_iop2 = NULL;
      vr_iop3 = NULL;
      vr_iop4 = NULL;
      smf_iop1 = NULL;
      smf_iop2 = NULL;
    return;
    }
  }

  if (filename3 && robot_specifications->generate_3D_smf){
    sprintf(local_filename, "%s.1", filename3);
    if ((smf_iop1 = fopen(local_filename, "w")) == 0){
      fprintf(stderr, "WARNING: Could not open smf file %s.\n", local_filename);
      if (program_state->graphics_initialized)
	G_display_switch(PLOT_BUTTON, 0);
      plot_iop = NULL;
      vr_iop1 = NULL;
      vr_iop2 = NULL;
      vr_iop3 = NULL;
      vr_iop4 = NULL;
      smf_iop1 = NULL;
      smf_iop2 = NULL;
      return;
    }

    sprintf(local_filename, "%s.2", filename3);
    if ((smf_iop2 = fopen(local_filename, "w")) == 0){
      fprintf(stderr, "WARNING: Could not open smf file %s.\n", local_filename);
      if (program_state->graphics_initialized)
	G_display_switch(PLOT_BUTTON, 0);
      plot_iop = NULL;
      vr_iop1 = NULL;
      vr_iop2 = NULL;
      vr_iop3 = NULL;
      vr_iop4 = NULL;
      smf_iop1 = NULL;
      smf_iop2 = NULL;
      return;
    }
  }

  prev_sensors_x = (float *) malloc(sizeof(float) * 
				    robot_specifications->num_sensors);
  prev_sensors_y = (float *) malloc(sizeof(float) * 
				    robot_specifications->num_sensors);
  prev_sensors_z = (float *) malloc(sizeof(float) * 
				    robot_specifications->num_sensors);
  prev_sensor_values = (float *) malloc(sizeof(float) * 
				       robot_specifications->num_sensors);
  if (!prev_sensors_x || !prev_sensors_y || !prev_sensors_z ||
      !prev_sensor_values){
    fprintf(stderr, "Error: Out of memory\n");
    exit(-1);
  }

  
  if (plot_iop)
    fprintf(plot_iop, "l={EdgeForm[],\n");

  if (vr_iop1){
    fprintf(vr_iop1, "#VRML V1.0 ascii\n\n");
    fprintf(vr_iop1, "Separator {\n");
    // fprintf(vr_iop1, "Material { diffuseColor 1 0 1 }\n\n");
    fprintf(vr_iop1, "Rotation { rotation 1 0 0 -1.5708 }\n\n");
    fprintf(vr_iop1, "Texture2 { filename \"laserint.jpg\" }\n\n");
    fprintf(vr_iop1, "Coordinate3 { point [\n");
  }
  if (vr_iop2){
    fprintf(vr_iop2, "USE Mat_red\n");
    fprintf(vr_iop2, "IndexedFaceSet { coordIndex [\n");
  }
  if (vr_iop3){
    fprintf(vr_iop3, "TextureCoordinate2 { point [\n");
  }
  if (vr_iop4){
    fprintf(vr_iop4, "textureCoordIndex [\n");
  }

  if (smf_iop1)
    fprintf(smf_iop1, "#$SMF 1.0\n");

  bg_image = random_image();
  fill_region_fraction(bg_image, 0.0, 1.0, 0.0, 1.0, 255, 0, 0);


  num_items = 0;
  first_scan = 1;
  first_item_1 = 1;
  first_item_2 = 1;
  plot_is_on = 1;


}




void
plot_off(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	 PROGRAM_STATE_PTR program_state,
	 ROBOT_STATE_PTR robot_state)
{
  int i;


  if (plot_iop){
    fprintf(plot_iop, "};\n");
    fprintf(plot_iop, "g1=Show[Graphics3D[l]];\n");
    fprintf(plot_iop, "Display[\"laserint1.ps2\", g1]\n");
    fprintf(plot_iop, "!psfix -rmarg 0.5 laserint1.ps2 > laserint1.ps\n");
    fprintf(plot_iop, "g2=Show[Graphics3D[l,ViewPoint->{-.3,.3,-1}]];\n");
    fprintf(plot_iop, "Display[\"laserint2.ps2\", g2]\n");
    fprintf(plot_iop, "!psfix -rmarg 0.5 laserint2.ps2 > laserint2.ps\n");
    fprintf(plot_iop, "g3=Show[Graphics3D[l,ViewPoint->{-1,0.5,0.3}]];\n");
    fprintf(plot_iop, "Display[\"laserint3.ps2\", g3]\n");
    fprintf(plot_iop, "!psfix -rmarg 0.5 laserint3.ps2 > laserint3.ps\n");
  }

  if (vr_iop1)
    fprintf(vr_iop1, "]\n} # %d vertices\n\n", num_items);
  if (vr_iop2)
    fprintf(vr_iop2, "]\n\n");      
  if (vr_iop3)
    fprintf(vr_iop3, "]\n} # %d vertices\n\n", num_items);
  if (vr_iop4)
    fprintf(vr_iop4, "]\n}}\n");      

  if (plot_iop){
    fclose(plot_iop);
    plot_iop = NULL;
  }
  if (vr_iop1){
    fclose(vr_iop1);
    vr_iop1 = NULL;
  }
  if (vr_iop2){
    fclose(vr_iop2);
    vr_iop2 = NULL;
  }
  if (vr_iop3){
    fclose(vr_iop3);
    vr_iop3 = NULL;
  }
  if (vr_iop4){
    fclose(vr_iop4);
    vr_iop4 = NULL;
  }
  if (smf_iop1){
    fclose(smf_iop1);
    smf_iop1 = NULL;
  }
  if (smf_iop2){
    fclose(smf_iop2);
    smf_iop2 = NULL;
  }


  if (program_state->graphics_initialized)
    G_display_switch(PLOT_BUTTON, 0);

  if (prev_sensors_x != NULL){
    free(prev_sensors_x);
    prev_sensors_x = NULL;
  }
  if (prev_sensors_y != NULL){
    free(prev_sensors_y);
    prev_sensors_y = NULL;
  }
  if (prev_sensors_z != NULL){
    free(prev_sensors_z);
    prev_sensors_z = NULL;
  }
  if (prev_sensor_values != NULL){
    free(prev_sensor_values);
    prev_sensor_values = NULL;
  }
  plot_is_on = 0;

  if (bg_image)
    save_image(bg_image, "random.ppm");
  bg_image = NULL;
  if (texture_image)
    save_image(texture_image, "laserint.ppm");
  texture_image = NULL;

}



void
plot_pos(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	 PROGRAM_STATE_PTR program_state,
	 ROBOT_STATE_PTR robot_state)
{
  int i, j;
  stack_item_ptr s;
  float cos2, sin2, cos2_1, sin2_1, cos1_2, sin1_2;
  float dx1_2, dy1_2, dx2_1, dy2_1, dx_global_coord, dy_global_coord;
  float deltax1, deltax2, deltay1, deltay2;
  float* x = (float *) malloc(sizeof(float) * 
			      robot_specifications->num_sensors);
  float* y = (float *) malloc(sizeof(float) * 
			      robot_specifications->num_sensors);
  float* z = (float *) malloc(sizeof(float) * 
			      robot_specifications->num_sensors);
  static int texture_counter = 0;

  if (!plot_is_on)
    return;

  if (!x || !y || !z){
    fprintf(stderr, "Error: Out of memory\n");
    exit(-1);
  }


  s = &(program_state->stack[program_state->last_stack_item]);

  // fprintf(stderr, "\n\nXXX ");
  
  // for (j = 0; j < 985; j++)
  // fprintf(stderr, "%d %d %d  ", red[j], green[j], blue[j]);
  // fprintf(stderr, "\n\n");


  for (j = 0; j < robot_specifications->num_sensors; j++){
      
    
    /* ================================================== *
     *   compute auxiliary variables
     * ================================================== */
    

    cos2 = cos(s->laser_orientation * M_PI / 180.0);
    sin2 = sin(s->laser_orientation * M_PI / 180.0);

    cos2_1 = cos(s->laser_orientation * M_PI / 180.0);
    sin2_1 = sin(s->laser_orientation * M_PI / 180.0);
      
    cos1_2 = cos((- s->laser_orientation) * M_PI / 180.0);
    sin1_2 = sin((- s->laser_orientation) * M_PI / 180.0);
      
    dx1_2 = - s->laser_x;
    dy1_2 = - s->laser_y;
    dx2_1 = -dx1_2;
    dy2_1 = -dy1_2;
      
    deltax1 = (dx1_2);
    deltay1 = (dy1_2);
    deltax2 = (dx2_1*cos2) + (dy2_1*sin2);
    deltay2 = -(dx2_1*sin2) + (dy2_1*cos2);

    x[j] = deltax1 + (s->sensor_x[j]*cos2_1) - (s->sensor_y[j]*sin2_1);
    y[j] = deltay1 + (s->sensor_x[j]*sin2_1) + (s->sensor_y[j]*cos2_1);
    z[j] = s->sensor_z[j];

    if (!first_item_1){
      if (vr_iop1)
	fprintf(vr_iop1, ",\n");
    }
    else
      first_item_1 = 0;
    if (vr_iop1)
      fprintf(vr_iop1, "    %f %f %f", x[j], y[j], z[j]);
    if (smf_iop1)
      fprintf(smf_iop1, "v %f %f %f\n", x[j], y[j], z[j]);

    if (!first_scan && j > 0 &&
	s->sensor_z[j] > 0.0 &&
	s->sensor_z[j-1] > 0.0 &&
	prev_sensors_z[j] > 0.0 &&
	prev_sensors_z[j-1] > 0.0 &&
	s->sensor_values[j] <= robot_specifications->max_sensors_range &&
	s->sensor_values[j-1] <= robot_specifications->max_sensors_range &&
	prev_sensor_values[j] <= robot_specifications->max_sensors_range &&
	prev_sensor_values[j-1] <= robot_specifications->max_sensors_range){


      if ((sqrt(((x[j] - x[j-1]) * (x[j] - x[j-1])) +
	       ((y[j] - y[j-1]) * (y[j] - y[j-1])) +
	       ((z[j] - z[j-1]) * (z[j] - z[j-1]))) < 150.0 &&
	  
	  sqrt(((x[j] - prev_sensors_x[j]) * (x[j] - prev_sensors_x[j])) +
	       ((y[j] - prev_sensors_y[j]) * (y[j] - prev_sensors_y[j])) +
	       ((z[j] - prev_sensors_z[j]) * (z[j] - prev_sensors_z[j])))
	  < 100.0 &&
	  
	  sqrt(((x[j-1] - prev_sensors_x[j-1]) *
		(x[j-1] - prev_sensors_x[j-1])) +
	       ((y[j-1] - prev_sensors_y[j-1]) *
		(y[j-1] - prev_sensors_y[j-1])) +
	       ((z[j-1] - prev_sensors_z[j-1]) *
		(z[j-1] - prev_sensors_z[j-1]))) < 100.0 &&
	   
	  sqrt(((prev_sensors_x[j] - prev_sensors_x[j-1]) * 
		(prev_sensors_x[j] - prev_sensors_x[j-1])) +
	       ((prev_sensors_y[j] - prev_sensors_y[j-1]) * 
		(prev_sensors_y[j] - prev_sensors_y[j-1])) +
	       ((prev_sensors_z[j] - prev_sensors_z[j-1]) * 
		(prev_sensors_z[j] - prev_sensors_z[j-1]))) < 100.0)){
	  
	if (!first_item_2){
	  if (plot_iop)
	    fprintf(plot_iop, ",\n");
	  // if (vr_iop4)
	  // fprintf(vr_iop4, ",\n");
	}
	else
	  first_item_2 = 0;
	
	if (plot_iop){

	  fprintf(plot_iop, "Polygon[{");
	  fprintf(plot_iop, "{%f, %f, %f}, ", x[j], y[j], z[j]);
	  fprintf(plot_iop, "{%f, %f, %f}, ", x[j-1], y[j-1], z[j-1]);
	  fprintf(plot_iop, "{%f, %f, %f}, ", prev_sensors_x[j-1], 
		  prev_sensors_y[j-1], prev_sensors_z[j-1]);
	  fprintf(plot_iop, "{%f, %f, %f}, ", prev_sensors_x[j], 
		  prev_sensors_y[j], prev_sensors_z[j]);
	  fprintf(plot_iop, "{%f, %f, %f}}]", x[j], y[j], z[j]);

	}


	if (vr_iop2){
	  static float img_x = 0.0;
	  static float img_y = 0.0;
	  static int img_c = 0;
	  static jj;
	  
	  fprintf(vr_iop2, "    %d, %d, %d, %d, -1,\n", 
		  num_items-robot_specifications->num_sensors-1,
		  num_items-robot_specifications->num_sensors,
		  num_items, num_items-1);

	  if (red && green && blue){
	    static int junk = 0;
	    
	    jj = (int) (((float) (359-j)) * 985.0 / 180.0);

	    /*
	      if (junk++ % 2 == 0)
	      fill_region_fraction(bg_image, 
				   img_x, img_x + 0.005,
				   img_y, img_y + 0.005, 0, 255, 0);
	    else
	      fill_region_fraction(bg_image, 
				   img_x, img_x + 0.003,
				   img_y, img_y + 0.003, 255, 0, 255);
	    */
	    /*
	      fill_region_fraction(bg_image, 
				   img_x, img_x + 0.003,
				   img_y, img_y + 0.003, 
				   (unsigned char) (RAND_ZERO_TO_ONE() * 255.0),
				   (unsigned char) (RAND_ZERO_TO_ONE() * 255.0),
				   (unsigned char) (RAND_ZERO_TO_ONE() * 255.0));
	      */	    

	    fprintf(vr_iop3, "%g %g,\n",
		    ((float) program_state->image_most_recent_x) / 
		    ((float) program_state->image_file_size_x),
		    ((float) (j-180)) / 180.0);
	    fprintf(vr_iop3, "%g %g,\n",
		    ((float) program_state->image_most_recent_x) / 
		    ((float) program_state->image_file_size_x),
		    ((float) (j-179)) / 180.0);

	    fprintf(vr_iop3, "%g %g,\n",
		    ((float) program_state->image_current_x) / 
		    ((float) program_state->image_file_size_x),
		    ((float) (j-179)) / 180.0);
	    fprintf(vr_iop3, "%g %g,\n",
		    ((float) program_state->image_current_x) / 
		    ((float) program_state->image_file_size_x),
		    ((float) (j-180)) / 180.0);
		    
	    fprintf(vr_iop4, "    %d, %d, %d, %d, -1,\n", 
		    img_c, img_c + 1, img_c + 2, img_c + 3);
	    img_c += 4;
	    

#ifdef JUNK
	    fill_region_fraction(bg_image, 
				 img_x, img_x + 0.003,
				 img_y, img_y + 0.003,
				 red[jj], green[jj], blue[jj]);

	    fprintf(vr_iop3, "%g %g,\n", 
		    img_x + (0.003/2.0),
		    1.0-img_y - (0.003/2.0));
	    fprintf(vr_iop3, "%g %g,\n",
		    img_x + 0.003 - (0.003/2.0), 
		    1.0-img_y - (0.003/2.0));
	    fprintf(vr_iop3, "%g %g,\n", 
		    img_x + 0.003 - (0.003/2.0),
		    1.0-(img_y + 0.003) + (0.003/2.0));
	    fprintf(vr_iop3, "%g %g,\n", 
		    img_x + (0.003/2.0), 
		    1.0-(img_y + 0.003) + (0.003/2.0));
	    
	    fprintf(vr_iop4, "    %d, %d, %d, %d, -1,\n", 
		    img_c, img_c + 1, img_c + 2, img_c + 3);
	    img_c += 4;
	    
	    img_x += 0.003;
	    if (img_x > 1.0 - (0.003/2.0)){
	      img_x = 0.0;
	      img_y += 0.003;
	      if (img_y > 1.0 - (0.003/2.0))
		img_y = 0.0;
	    }
#endif
	  }
	}

	if (smf_iop2){
	  fprintf(smf_iop2, "f %d %d %d\n", 
		  num_items-robot_specifications->num_sensors,
		  num_items-robot_specifications->num_sensors+1,
		  num_items+1);
	  fprintf(smf_iop2, "f %d %d %d\n", 
		  num_items+1, 
		  num_items,
		  num_items-robot_specifications->num_sensors);
	}


      }
    }
    num_items++;
  }

  program_state->image_most_recent_x = program_state->image_current_x;
  // fprintf(stderr, "\n\n");

  for (j = 0; j < robot_specifications->num_sensors; j++){
    prev_sensors_x[j] = x[j];
    prev_sensors_y[j] = y[j];
    prev_sensors_z[j] = z[j];
    prev_sensor_values[j] = s->sensor_values[j];
  }
  first_scan = 0;

  if (vr_iop1)
    fflush(vr_iop1);
  if (vr_iop2)
    fflush(vr_iop2);
  if (vr_iop3)
    fflush(vr_iop3);
  if (vr_iop4)
    fflush(vr_iop4);
  if (smf_iop1)
    fflush(smf_iop1);
  if (smf_iop2)
    fflush(smf_iop2);

  free(x);
  free(y);
  free(z);

}





/***************************************************************************
 ***************************************************************************
 ***************************************************************************/




void
send_stack_item_to_server(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			  PROGRAM_STATE_PTR  program_state,
			  ROBOT_STATE_PTR  robot_state, int force)
{
  static LASERINT_scan_type scan;
  static int allocated = 0;
  int i;
  float transl, turn;
  stack_item_ptr s = &(program_state->stack[program_state->last_stack_item]);

  static float last_x = 0.0;
  static float last_y = 0.0;
  static float last_o = 0.0;
  static float cumul_transl_since_send = 0.0;
  static float cumul_turn_since_send = 0.0;
  static int last_initialized = 0;


  if (program_state->is_server)
     return;

  if(!program_state->server_connected)
    connect_to_SERVER(program_state);
  
  if(!program_state->server_connected)
    return;
  

  if (!allocated){
    scan.sensor_values = 
      (float *) malloc(sizeof(float) * robot_specifications->num_sensors);
    
    if (!scan.sensor_values){
      fprintf(stderr,
	      "ERROR: Out of memory in send_stack_item_to_server().\n");
      fprintf(stderr, "Must abort.\n");
      exit(-1);
    }
    allocated = 1;
  }

  /* compute distance */

  if (last_initialized){
    transl = sqrt(((last_x - robot_state->laser_x) *
		   (last_x - robot_state->laser_x)) +
		  ((last_y - robot_state->laser_y) *
		   (last_y - robot_state->laser_y)));
    turn = robot_state->laser_orientation - last_o;
    for (; turn >   180.0 ;) turn -= 360.0;
    for (; turn <= -180.0 ;) turn += 360.0;
    turn = fabs(turn);
    cumul_transl_since_send += transl;
    cumul_turn_since_send   += turn;
  }
  else
    last_initialized = 1;
  last_x = robot_state->laser_x;
  last_y = robot_state->laser_y;
  last_o = robot_state->laser_orientation;


  /* fprintf(stderr, "### %d  %g %g\n", force, cumul_transl_since_send,
     cumul_turn_since_send ); */

  if (force ||
      cumul_transl_since_send > 
      robot_specifications->pos_corr_max_dist_for_server ||
      cumul_turn_since_send   > 
      robot_specifications->pos_corr_max_angle_for_server){

    cumul_transl_since_send = 0.0;
    cumul_turn_since_send   = 0.0;

    scan.num_sensors = robot_specifications->num_sensors;
    scan.laser_x = robot_state->laser_x;
    scan.laser_y = robot_state->laser_y;
    scan.laser_orientation = robot_state->laser_orientation;



    scan.raw_odometry_x = s->raw_odometry_laser_x;
    scan.raw_odometry_y = s->raw_odometry_laser_y;
    scan.raw_odometry_orientation = s->raw_odometry_laser_orientation;
    /* @@@
    scan.raw_odometry_x = robot_state->raw_odometry_x;
    scan.raw_odometry_y = robot_state->raw_odometry_y;
    scan.raw_odometry_orientation = robot_state->raw_odometry_orientation;
    */
    scan.ignore_front_laser = robot_specifications->ignore_front_laser;
    scan.ignore_rear_laser = robot_specifications->ignore_rear_laser;
    scan.front_laser_offset_x = robot_specifications->front_laser_offset_x;
    scan.front_laser_offset_y = robot_specifications->front_laser_offset_y;
    scan.rear_laser_offset_x = robot_specifications->rear_laser_offset_x;
    scan.rear_laser_offset_y = robot_specifications->rear_laser_offset_y;
    scan.force = force;

    for (i = 0; i < robot_specifications->num_sensors; i++)
      scan.sensor_values[i] = robot_state->sensor_values[i];

    /*fprintf(stderr, "SEND TO SERVER: %5.3f %5.3f %5.3f (RAW: %5.3f %5.3f %5.3f)\n",
	    robot_state->laser_x,
	    robot_state->laser_y,
	    robot_state->laser_orientation,
	    robot_state->raw_odometry_x,
	    robot_state->raw_odometry_y,
	    robot_state->raw_odometry_orientation);
	    */

  /*
   * send the message
   */

    tcxSendMsg(SERVER, "LASERINT_scan", &scan);
  }
}



/***************************************************************************
 ***************************************************************************
 ***************************************************************************/

void
edit(ROBOT_SPECIFICATIONS_PTR robot_specifications,
     PROGRAM_STATE_PTR  program_state,
     ROBOT_STATE_PTR  robot_state, 
     int button,
     float d_x, float d_y, float d_o)
{
  char txt[128];
  stack_item_ptr s1;
  stack_item_ptr s2 = NULL;




  if (program_state->last_stack_item > 0){
    s1 = &(program_state->stack[program_state->last_stack_item-1]);
    s2 = &(program_state->stack[program_state->last_stack_item]);
  }
  else
    s1 = &(program_state->stack[program_state->last_stack_item]);
  


  s1->laser_x += d_x * 5.0;
  s1->laser_y += d_y * 5.0;
  s1->laser_orientation += d_o * 2.0;

  s2->laser_x += d_x * 5.0;
  s2->laser_y += d_y * 5.0;
  s2->laser_orientation += d_o * 2.0;



}
