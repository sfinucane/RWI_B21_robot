
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/laserint/file.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:34:06 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: file.c,v $
 * Revision 1.1  2002/09/14 15:34:06  rstone
 * *** empty log message ***
 *
 * Revision 1.33  2000/03/21 03:44:01  thrun
 * slight tune-ups.
 *
 * Revision 1.32  2000/03/14 05:23:57  thrun
 * fixed a few problems with the 3D mapping. This version
 * pruced the results first shown to the DAPRA folks.
 * In particular, it reads the texture entirely from file and
 * constructs a new texture file (laserint.ppm, which has
 * to be converted manyally into laserint.jpg)
 *
 * Revision 1.31  2000/03/13 02:54:16  thrun
 * slight inprovement to 3D version.
 *
 * Revision 1.30  2000/03/13 00:31:12  thrun
 * first version of he texturing module. Still a bit clumpsy.
 *
 * Revision 1.29  1999/12/27 03:43:15  thrun
 * Magic: Gradient descent for adapting
 * wall parameters xw yw and aw seems to work!!!
 *
 * Revision 1.28  1999/10/15 18:07:19  thrun
 * more 3D formats, more compact files.
 *
 * Revision 1.27  1999/10/02 04:50:11  thrun
 * Minor tuning and changes. THis version built the first
 * nice map of the San Jose Tech Museum
 *
 * Revision 1.26  1999/10/01 17:58:18  thrun
 * reinstatiated the close-the-cycle feature.
 *
 * Revision 1.25  1999/09/28 21:49:54  thrun
 * Multi-robot support in various modules.
 * In particular, I had to introduce a new message for MAP that
 * communicates back robot positions and correction parameters.
 * This might make it necessary to modify existing modules that use
 * MAP - I tried to do this for all I am aware of, but I might have missed some.
 *
 * Revision 1.24  1999/09/05 21:57:22  thrun
 * initial client server robot for multi-robot mapping
 *
 * Revision 1.23  1999/09/05 17:18:51  thrun
 * 3D ceiling mapping
 *
 * Revision 1.21  1999/07/03 21:44:36  thrun
 * Fixed several bugs in LASERINT and tuned the parameters of
 * LASERINT, MAP, and PLAN for the new Urban Robot
 *
 * Revision 1.20  1999/07/03 18:49:35  thrun
 * LOASERINT can now send correction parameters directly to MAP.
 *
 * Revision 1.19  1999/06/21 18:30:25  thrun
 * Took out the sampling stuff (which never worked corretly). It's now
 * up to Dieter to put it in again...
 *
 * Revision 1.18  1999/05/08 19:47:28  thrun
 * drastically improved version with recursive matching and samples.
 * Not quite ready yet: samples aren't used in the right way, queue
 * seems not to work, and some constants are hardcoded (phase!) But
 * impressive results!
 *
 * Revision 1.17  1999/05/04 01:14:21  thrun
 * queue instead of stack.
 *
 * Revision 1.16  1999/04/23 20:00:09  thrun
 * slight extensions - limited stack (is now a queue) and flag that
 * prevents integration of readings after sharp turn.
 *
 * Revision 1.15  1998/11/18 04:21:09  thrun
 * incorporated wall angle into estimation.
 *
 * Revision 1.14  1998/11/17 05:03:13  thrun
 * small, incremental improvements: better handling of sensor
 * displacement, and new parametr that enables laserint to ignore
 * scans when the robot is rotating too much.
 *
 * Revision 1.13  1998/11/16 01:50:40  thrun
 * positon control - works nice, includes global mapping and learning
 * the drift parameters.
 *
 * Revision 1.12  1998/11/15 16:19:17  thrun
 * initial position control method - works reasonably well. But
 * e careful with the timing, isn't an anytime algorithm yet.
 *
 * Revision 1.11  1998/11/12 14:22:55  thrun
 * ?.
 *
 * Revision 1.10  1998/08/11 22:18:25  thrun
 * PLANNER: fixed a problem with large shifts, a problem with th
 * function "pow()" under the latest Linux.
 *
 * Revision 1.9  1998/01/27 16:58:41  thrun
 * fixed an inconsistency in the laserint format. Now laser readings
 * have two length parameters, which both are either 0 or 180. This
 * replaces the single value, which was used before Dieter touched
 * our files. ;-)
 *
 * Revision 1.8  1998/01/07 22:52:28  thrun
 * This version does NOT use neural networks. Instead, it uses
 * a geometric procedure. Difference: The obstacles are now
 * not grown by the robot diameter.
 *
 * Revision 1.7  1997/12/30 00:40:11  thrun
 * Non-neural network version of mapping.
 *
 * Revision 1.6  1997/10/05 20:17:27  thrun
 * Dual script formats (old and new)
 *
 * Revision 1.5  1997/08/12 03:06:02  thrun
 * intermediate version, provides some logging of sensor scans and partial
 * maps within map.
 *
 * Revision 1.4  1997/04/27 13:02:51  thrun
 * Can now handle 1 or 2 lasers (from file)
 *
 * Revision 1.3  1997/04/27 12:27:20  thrun
 * Extended parameter set (copied from SONARINT)
 *
 * Revision 1.2  1997/03/11 17:14:09  tyson
 * added IR simulation and other work
 *
 * Revision 1.1.1.1  1996/09/22 16:46:28  rhino
 * General reorganization of the directories/repository, fusion with the
 * RWI software.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/



#include <signal.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include <sys/time.h>
#include "tcx.h"
#include "tcxP.h"
#include "Application.h"
#include "Net.h"
#include "MAP-messages.h"
#include "LASER-messages.h"
#include "BASE-messages.h"
#include "LASERINT.h"
#include "EZX11.h"
#include "o-graphics.h"


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/*** 
 *** These variables are used to communicate parameters when reading 
 *** script files
 ***/

static FILE *script_iop = NULL;
static char script_filename[256];
       FILE *log_iop = NULL;
static char log_filename[256];
static int robot_pos_defined = 0;

unsigned char *red = NULL;
unsigned char *green = NULL;
unsigned char *blue = NULL;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/************************************************************************
 *
 *   NAME:         save_parameters()
 *                 
 *   FUNCTION:     saves a map into a file. Map-intrinsic format.
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


int save_parameters(char *filename)
{
  int i, x, y, index;
  FILE *iop;
  char filename2[256];
  

  sprintf(filename2, "%s", filename);
  if ((iop = fopen(filename2, "w")) == 0){
    fprintf(stderr, "WARNING: Could not open output file %s.\n", filename2);
    sprintf(filename2, "../etc/%s", filename);
    if ((iop = fopen(filename2, "w")) == 0){
      fprintf(stderr, "WARNING: Could not open output file %s.\n", filename2);
      sprintf(filename2, "../../etc/%s", filename);
      if ((iop = fopen(filename2, "w")) == 0){
	fprintf(stderr, 
		"WARNING: Could not open output file %s. Not saved.\n",
		filename2);
	return 0;
      }
    }
  }



  fprintf(iop, "robot_specifications->local_mapsize_x                  %g\n",
	  robot_specifications->local_mapsize_x);
  fprintf(iop, "robot_specifications->local_mapsize_y                  %g\n",
	  robot_specifications->local_mapsize_y);
  fprintf(iop, "robot_specifications->resolution                       %g\n",
	  robot_specifications->resolution);
  fprintf(iop, "robot_specifications->pos_resolution                   %g\n",
	  robot_specifications->pos_resolution);
  fprintf(iop, "robot_specifications->robot_size                       %g\n",
	  robot_specifications->robot_size);
  fprintf(iop, "robot_specifications->front_laser_offset_x             %g\n",
	  robot_specifications->front_laser_offset_x);
  fprintf(iop, "robot_specifications->front_laser_offset_y             %g\n",
	  robot_specifications->front_laser_offset_y);
  fprintf(iop, "robot_specifications->rear_laser_offset_x              %g\n",
	  robot_specifications->rear_laser_offset_x);
  fprintf(iop, "robot_specifications->rear_laser_offset_y              %g\n",
	  robot_specifications->rear_laser_offset_y);


  fprintf(iop, "robot_specifications->smooth_radius                    %d\n",
	  robot_specifications->smooth_radius);

  fprintf(iop, "robot_specifications->num_sensors                      %d\n",
	  robot_specifications->num_sensors);
  fprintf(iop, "robot_specifications->first_sensor_angle               %g\n",
	  robot_specifications->first_sensor_angle);
  fprintf(iop, "robot_specifications->max_sensors_range                %g\n",
	  robot_specifications->max_sensors_range);
  fprintf(iop, "robot_specifications->min_sensors_range                %g\n",
	  robot_specifications->min_sensors_range);
  fprintf(iop, "robot_specifications->sensor_increment                 %g\n",
	  robot_specifications->sensor_increment);
  fprintf(iop, "robot_specifications->neuronet_max_sensors_range       %g\n",
	  robot_specifications->neuronet_max_sensors_range);
  fprintf(iop, "robot_specifications->neuronet_min_sensors_range       %g\n",
	  robot_specifications->neuronet_min_sensors_range);
  fprintf(iop, "robot_specifications->max_occupied_sensors_range       %g\n",
	  robot_specifications->max_occupied_sensors_range);
  fprintf(iop, "robot_specifications->occupied_outer_width             %g\n",
	  robot_specifications->occupied_outer_width);
  fprintf(iop, "robot_specifications->network_occupied_value           %g\n",
	  robot_specifications->network_occupied_value);

  fprintf(iop, "robot_specifications->use_neural_networks              %d\n",
	  robot_specifications->use_neural_networks);
  fprintf(iop, "robot_specifications->network_value_mean               %g\n",
	  robot_specifications->network_value_mean);
  fprintf(iop, "robot_specifications->decay_with_distance              %g\n",
	  robot_specifications->decay_with_distance);
  fprintf(iop, "robot_specifications->standard_free                    %g\n",
	  robot_specifications->standard_free);
  fprintf(iop, "robot_specifications->standard_occupied                %g\n",
	  robot_specifications->standard_occupied);

  fprintf(iop, "robot_specifications->obstacle_width                   %g\n",
	  robot_specifications->obstacle_width);

  fprintf(iop, "robot_specifications->line_recognition_neighbors       %d\n",
	  robot_specifications->line_recognition_neighbors);
  fprintf(iop, "robot_specifications->line_recognition_threshold       %g\n",
	  robot_specifications->line_recognition_threshold);
  fprintf(iop, "robot_specifications->min_advancement_between_interpretations %g\n",
	  robot_specifications->min_advancement_between_interpretations);
  fprintf(iop, "robot_specifications->broadcast_sensor_data_to_map     %d\n",
	  robot_specifications->broadcast_sensor_data_to_map);

  fprintf(iop, "program_state->delay_in_replay                         %g\n",
	  program_state->delay_in_replay);

  fprintf(iop, "robot_specifications->ignore_odometry                  %d\n",
	  robot_specifications->ignore_odometry);

  fprintf(iop, "robot_specifications->stack_size                       %d\n",
	  robot_specifications->stack_size);
  fprintf(iop, "robot_specifications->do_position_correction           %d\n",
	  robot_specifications->do_position_correction);
  fprintf(iop, "robot_specifications->pos_corr_angle_margin            %g\n",
	  robot_specifications->pos_corr_angle_margin);
  fprintf(iop, "robot_specifications->pos_corr_max_dist                %g\n",
	  robot_specifications->pos_corr_max_dist);
  fprintf(iop, "robot_specifications->pos_corr_max_angle               %g\n",
	  robot_specifications->pos_corr_max_angle);
  fprintf(iop, "robot_specifications->pos_corr_max_range               %g\n",
	  robot_specifications->pos_corr_max_range);
  fprintf(iop, "robot_specifications->pos_corr_max_d_trans             %g\n",
	  robot_specifications->pos_corr_max_d_trans);
  fprintf(iop, "robot_specifications->pos_corr_max_d_rot               %g\n",
	  robot_specifications->pos_corr_max_d_rot);
  fprintf(iop, "robot_specifications->pos_corr_lrate_trans             %g\n",
	  robot_specifications->pos_corr_lrate_trans);
  fprintf(iop, "robot_specifications->pos_corr_lrate_rot               %g\n",
	  robot_specifications->pos_corr_lrate_rot);
  fprintf(iop, "robot_specifications->pos_corr_min_num_iterations      %d\n",
	  robot_specifications->pos_corr_min_num_iterations);
  fprintf(iop, "robot_specifications->pos_corr_max_num_iterations      %d\n",
	  robot_specifications->pos_corr_max_num_iterations);
  fprintf(iop, "robot_specifications->pos_corr_hinge_point_offset      %g\n",
	  robot_specifications->pos_corr_hinge_point_offset);
  fprintf(iop, "robot_specifications->pos_corr_hinge_dist_threshold    %g\n",
	  robot_specifications->pos_corr_hinge_dist_threshold);
  fprintf(iop, "robot_specifications->pos_corr_rot_cutoff_angle        %g\n",
	  robot_specifications->pos_corr_rot_cutoff_angle);

  fprintf(iop, "robot_specifications->pos_corr_max_dist_for_server     %g\n",
	  robot_specifications->pos_corr_max_dist_for_server);
  fprintf(iop, "robot_specifications->pos_corr_max_angle_for_server    %g\n",
	  robot_specifications->pos_corr_max_angle_for_server);

  fprintf(iop, "robot_specifications->pos_corr_do_est_drift            %d\n",
	  robot_specifications->pos_corr_do_est_drift);
  fprintf(iop, "robot_specifications->pos_corr_drift_trans             %g\n",
	  robot_specifications->pos_corr_drift_trans);
  fprintf(iop, "robot_specifications->pos_corr_drift_rot               %g\n",
	  robot_specifications->pos_corr_drift_rot);
  fprintf(iop, "robot_specifications->pos_corr_lrate_drift             %g\n",
	  robot_specifications->pos_corr_lrate_drift);
  fprintf(iop, "robot_specifications->angle_dev_threshold              %g\n",
	  robot_specifications->angle_dev_threshold);
  fprintf(iop, "robot_specifications->strong_turn_threshold            %g\n",
	  robot_specifications->strong_turn_threshold);
  fprintf(iop, "robot_specifications->strong_turn_decay                %d\n",
	  robot_specifications->strong_turn_decay);
  fprintf(iop, "robot_specifications->show_all_points                  %d\n",
	  robot_specifications->show_all_points);
  fprintf(iop, "robot_specifications->send_corr_parameters_to_map      %d\n",
	  robot_specifications->send_corr_parameters_to_map);
  fprintf(iop, "robot_specifications->reposition_robot_initially       %d\n",
	  robot_specifications->reposition_robot_initially);
  fprintf(iop, "robot_specifications->gif_image_frequency              %d\n",
	  robot_specifications->gif_image_frequency);
  fprintf(iop, "robot_specifications->gif_image_size                   %d\n",
	  robot_specifications->gif_image_size);
  fprintf(iop, "robot_specifications->diagnose_step                    %d\n",
	  robot_specifications->diagnose_step);
  fprintf(iop, "robot_specifications->ignore_rear_laser                %d\n",
	  robot_specifications->ignore_rear_laser);
  fprintf(iop, "robot_specifications->ignore_front_laser               %d\n",
	  robot_specifications->ignore_front_laser);
  fprintf(iop, "robot_specifications->do_backwards_corrections         %d\n",
	  robot_specifications->do_backwards_corrections);
  fprintf(iop, "robot_specifications->generate_3D_vrml                 %d\n",
	  robot_specifications->generate_3D_vrml);
  fprintf(iop, "robot_specifications->generate_3D_smf                  %d\n",
	  robot_specifications->generate_3D_smf);
  fprintf(iop, "robot_specifications->generate_3D_math                 %d\n",
	  robot_specifications->generate_3D_math);
  fprintf(iop, "robot_specifications->plot_auto_on                     %d\n",
	  robot_specifications->plot_auto_on);
  fprintf(iop, "robot_specifications->display_fixed_rotation           %d\n",
	  robot_specifications->display_fixed_rotation);
  fprintf(iop, "robot_specifications->display_interval                 %d\n",
	  robot_specifications->display_interval);
  fprintf(iop, "robot_specifications->display_density                 %d\n",
	  robot_specifications->display_density);
  fprintf(iop, "robot_specifications->world_size                       %g\n",
	  robot_specifications->world_size);

  
  fclose(iop);

  fprintf(stderr, "File %s successfully written.\n", filename2);

  return 1;
}



/************************************************************************
 *
 *   NAME:         load_parameters()
 *                 
 *   FUNCTION:     loads a map from a file. Map-intrinsic format.
 *                 
 *   PARAMETERS:   filename
 *                 init        1, if MAP is initialized
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/


int load_parameters(char *filename, int init)
{
  int i, x, y, index, int_value, int_value1, int_value2;
  float float_value;
  FILE *iop;
  int file_ended, error;
  char command[256];

  char filename2[256];
  
  
  sprintf(filename2, "%s", filename);
  if ((iop = fopen(filename2, "r")) == 0){
    fprintf(stderr, "Could not open input file %s.\n", filename2);
    sprintf(filename2, "../etc/%s", filename);
    if ((iop = fopen(filename2, "r")) == 0){
      fprintf(stderr, "Could not open input file %s.\n", filename2);
      sprintf(filename2, "../../etc/%s", filename);
      if ((iop = fopen(filename2, "r")) == 0){
	fprintf(stderr, "Could not open input file %s. File not loaded.\n",
		filename2);
	fprintf(stderr, "WARNING: Failed to read file %s.\n", filename);
	return 0;
      }
    }
  }


  file_ended = 0;
  error = 0;
  do{
    if (fscanf(iop, "%s", command) == EOF)
      file_ended = 1;
    
    else{
      
      if (!strcmp(command, "end"))
	file_ended = 2;
      
      else if (command[0] == '#'){ /* comment */
	if (!fgets(command,sizeof(command),iop))
	  file_ended = 2;	/* file may end with hash-sign /  comment */
	}


      else if (!strcmp(command, "robot_specifications->local_mapsize_x")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init){
	  robot_specifications->local_mapsize_x = float_value;
	  robot_specifications->local_map_dim_x   =
	    1 + (int) /* 1=safety margin */
	    (robot_specifications->local_mapsize_x 
	     / robot_specifications->resolution);
	}
	else if (float_value != robot_specifications->local_mapsize_x){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->local_mapsize_x");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->local_mapsize_y")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init){
	  robot_specifications->local_mapsize_y = float_value;
	  robot_specifications->local_map_dim_y   =
	    1 + (int) /* 1=safety margin */
	    (robot_specifications->local_mapsize_y 
	     / robot_specifications->resolution);
	}
	else if (float_value != robot_specifications->local_mapsize_y){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->local_mapsize_y");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      
      
      else if (!strcmp(command, "robot_specifications->resolution")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init){
	  robot_specifications->resolution = float_value;
	  robot_specifications->local_map_dim_x   =
	    1 + (int) /* 1=safety margin */
	    (robot_specifications->local_mapsize_x 
	     / robot_specifications->resolution);
	  robot_specifications->local_map_dim_y   =
	    1 + (int) /* 1=safety margin */
	    (robot_specifications->local_mapsize_y 
	     / robot_specifications->resolution);
	}
	else if (float_value != robot_specifications->resolution){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->resolution");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
    
      else if (!strcmp(command, "robot_specifications->pos_resolution")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init){
	  robot_specifications->pos_resolution = float_value;
	  robot_specifications->pos_map_dim = 2 * 
	    ((int) (robot_specifications->pos_corr_max_range /
		    robot_specifications->pos_resolution));
 	}
	else if (float_value != robot_specifications->pos_resolution){
	  fprintf(stderr, "Error when parsing %s (%d): value for ", 
		  filename2, init);
	  fprintf(stderr, "robot_specifications->pos_resolution");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->robot_size")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->robot_size = float_value;
	else if (float_value != robot_specifications->robot_size){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->robot_size");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->front_laser_offset_x")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (init) */
	  robot_specifications->front_laser_offset_x = float_value;
      }
      
     
      else if (!strcmp(command, "robot_specifications->front_laser_offset_y")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (init) */
	  robot_specifications->front_laser_offset_y = float_value;
      }
      
      
      else if (!strcmp(command, "robot_specifications->rear_laser_offset_x")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (init) */
	  robot_specifications->rear_laser_offset_x = float_value;
      }
      
     
      else if (!strcmp(command, "robot_specifications->rear_laser_offset_y")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (init) */
	  robot_specifications->rear_laser_offset_y = float_value;
      }
      
      
      else if (!strcmp(command, "robot_specifications->smooth_radius")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->smooth_radius = int_value;
	else if (int_value != robot_specifications->smooth_radius){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->smooth_radius");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      

      else if (!strcmp(command, "robot_specifications->num_sensors")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->num_sensors = int_value;
	else if (int_value != robot_specifications->num_sensors){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->num_sensors");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->first_sensor_angle")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->first_sensor_angle = float_value;
	else if (float_value != robot_specifications->first_sensor_angle){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->first_sensor_angle");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->max_sensors_range")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_sensors_range = float_value;
	else if (float_value != robot_specifications->max_sensors_range){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->max_sensors_range");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->min_sensors_range")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->min_sensors_range = float_value;
	else if (float_value != robot_specifications->min_sensors_range){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->min_sensors_range");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      else if (!strcmp(command, "robot_specifications->sensor_increment")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (init) */
	  robot_specifications->sensor_increment = float_value;
      }
      
      

      else if (!strcmp(command, "robot_specifications->neuronet_max_sensors_range")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->neuronet_max_sensors_range = float_value;
	else if (float_value != robot_specifications->neuronet_max_sensors_range){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->neuronet_max_sensors_range");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, "robot_specifications->neuronet_min_sensors_range")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->neuronet_min_sensors_range = float_value;
	else if (float_value != robot_specifications->neuronet_min_sensors_range){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->neuronet_min_sensors_range");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command,
		       "robot_specifications->max_occupied_sensors_range")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_occupied_sensors_range = float_value;
	else if (float_value != 
		 robot_specifications->max_occupied_sensors_range){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->max_occupied_sensors_range");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command,
		       "robot_specifications->occupied_outer_width")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->occupied_outer_width = float_value;
	else if (float_value != 
		 robot_specifications->occupied_outer_width){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->occupied_outer_width");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      
      else if (!strcmp(command,
		       "robot_specifications->network_occupied_value")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->network_occupied_value = float_value;
	else if (float_value != 
		 robot_specifications->network_occupied_value){
	  fprintf(stderr, "Error when parsing %s: value for ", filename2);
	  fprintf(stderr, "robot_specifications->network_occupied_value");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      else if (!strcmp(command, 
		       "robot_specifications->use_neural_networks")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->use_neural_networks = int_value;
      }

      else if (!strcmp(command, "robot_specifications->network_value_mean")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->network_value_mean = float_value;
      }


      else if (!strcmp(command, "robot_specifications->decay_with_distance")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->decay_with_distance = float_value;
      }

      else if (!strcmp(command, "robot_specifications->standard_free")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->standard_free = float_value;
      }

      else if (!strcmp(command, "robot_specifications->standard_occupied")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->standard_occupied = float_value;
      }

      else if (!strcmp(command, "robot_specifications->obstacle_width")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->obstacle_width = float_value;
      }

      else if (!strcmp(command, 
		       "robot_specifications->line_recognition_neighbors")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->line_recognition_neighbors = int_value;
      }

      else if (!strcmp(command,
		       "robot_specifications->line_recognition_threshold")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->line_recognition_threshold = float_value;
      }


      else if (!strcmp(command,
		       "robot_specifications->min_advancement_between_interpretations")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->min_advancement_between_interpretations
	    = float_value;
      }


      else if (!strcmp(command, 
		       "robot_specifications->broadcast_sensor_data_to_map")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->broadcast_sensor_data_to_map = int_value;
      }


      else if (!strcmp(command, "program_state->delay_in_replay")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  program_state->delay_in_replay = float_value;
      }

      
      else if (!strcmp(command, "robot_specifications->ignore_odometry")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->ignore_odometry = int_value;
      }

      else if (!strcmp(command, "robot_specifications->stack_size")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->stack_size = int_value;
      }

      else if (!strcmp(command, "robot_specifications->do_position_correction")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->do_position_correction = int_value;
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_angle_margin")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_angle_margin = float_value;
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_max_dist")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_max_dist = float_value;
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_max_angle")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_max_angle = float_value;
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_max_range")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else if (init){
	  robot_specifications->pos_corr_max_range = float_value;
	  robot_specifications->pos_map_dim = 2 * 
	    ((int) (robot_specifications->pos_corr_max_range /
		    robot_specifications->pos_resolution));
 	}
	else if (float_value != robot_specifications->pos_corr_max_range){
	  fprintf(stderr, "Error when parsing %s (%d): value for ", 
		  filename2, init);
	  fprintf(stderr, "robot_specifications->pos_corr_max_range");
	  fprintf(stderr, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_max_d_trans")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_max_d_trans = float_value;
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_max_d_rot")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_max_d_rot = float_value;
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_lrate_trans")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_lrate_trans = float_value;
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_lrate_rot")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_lrate_rot = float_value;
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_min_num_iterations")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_min_num_iterations = int_value;
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_max_num_iterations")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_max_num_iterations = int_value;
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_hinge_point_offset")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_hinge_point_offset = float_value;
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_hinge_dist_threshold")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_hinge_dist_threshold = float_value;
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_rot_cutoff_angle")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_rot_cutoff_angle = float_value;
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_max_dist_for_server")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_max_dist_for_server = float_value;
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_max_angle_for_server")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_max_angle_for_server = float_value;
      }

      else if (!strcmp(command, "robot_specifications->pos_corr_do_est_drift")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_do_est_drift = int_value;
      }


      else if (!strcmp(command, "robot_specifications->pos_corr_drift_trans")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_drift_trans = float_value;
      }


      else if (!strcmp(command, "robot_specifications->pos_corr_drift_rot")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_drift_rot = float_value;
      }


      else if (!strcmp(command, "robot_specifications->pos_corr_lrate_drift")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->pos_corr_lrate_drift = float_value;
      }


      else if (!strcmp(command, "robot_specifications->angle_dev_threshold")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->angle_dev_threshold = float_value;
      }


      else if (!strcmp(command,
		       "robot_specifications->strong_turn_threshold")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->strong_turn_threshold = float_value;
      }

      else if (!strcmp(command, "robot_specifications->strong_turn_decay")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->strong_turn_decay = int_value;
      }

      else if (!strcmp(command, "robot_specifications->show_all_points")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->show_all_points = int_value;
      }

      else if (!strcmp(command, "robot_specifications->send_corr_parameters_to_map")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->send_corr_parameters_to_map = int_value;
      }

      else if (!strcmp(command, 
		       "robot_specifications->reposition_robot_initially")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->reposition_robot_initially = int_value;
      }

      
      else if (!strcmp(command, "robot_specifications->gif_image_frequency")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->gif_image_frequency = int_value;
      }

      else if (!strcmp(command, "robot_specifications->gif_image_size")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->gif_image_size = int_value;
      }
      else if (!strcmp(command, "robot_specifications->diagnose_step")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->diagnose_step = int_value;
      }


      else if (!strcmp(command, "robot_specifications->ignore_rear_laser")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->ignore_rear_laser = int_value;
      }


      else if (!strcmp(command, "robot_specifications->ignore_front_laser")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->ignore_front_laser = int_value;
      }


      else if (!strcmp(command, "robot_specifications->do_backwards_corrections")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->do_backwards_corrections = int_value;
      }


      else if (!strcmp(command, "robot_specifications->generate_3D_vrml")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->generate_3D_vrml = int_value;
      }


      else if (!strcmp(command, "robot_specifications->generate_3D_smf")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->generate_3D_smf = int_value;
      }


      else if (!strcmp(command, "robot_specifications->generate_3D_math")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->generate_3D_math = int_value;
      }


      else if (!strcmp(command, "robot_specifications->plot_auto_on")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->plot_auto_on = int_value;
      }


      else if (!strcmp(command, "robot_specifications->display_fixed_rotation")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->display_fixed_rotation = int_value;
      }

      else if (!strcmp(command, "robot_specifications->display_interval")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->display_interval = int_value;
      }

      else if (!strcmp(command, "robot_specifications->display_density")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->display_density = int_value;
      }


      else if (!strcmp(command, "robot_specifications->world_size")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->world_size = float_value;
      }





      else{
	fprintf(stderr, "ERROR: Unknown keyword \"%s\" in %s. Must exit.\n ", 
		command, filename2);
	error = 1;
      }
 
      if (file_ended == 1)
	fprintf(stderr, "Surprising end of file %s.\n ", filename2);
      
    }
  } while (!file_ended);
  fclose(iop);
  
  if (!error)
    fprintf(stderr, "File %s successfully read.\n", filename2);

  return (1 - error);
}


/************************************************************************
 *
 *   NAME:         initiate_read_script()
 *                 
 *   FUNCTION:     opens a script file. Clears all maps.
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/



int initiate_read_script(ROBOT_STATE_PTR    robot_state,
			 PROGRAM_STATE_PTR  program_state,
			 ROBOT_SPECIFICATIONS_PTR robot_specifications,
			 char *filename)
{
  if (program_state->graphics_initialized)
    G_display_switch(SCRIPT_BUTTON, 1);

  if ((script_iop = fopen(filename, "r")) == 0){
    fprintf(stderr, "WARNING: Could not open script file %s.\n", filename);
    if (program_state->graphics_initialized)
      G_display_switch(SCRIPT_BUTTON, 0);
    script_iop = NULL;
    return 0;
  }

  strcpy(script_filename, filename);
  clear_all_maps(robot_state, program_state, robot_specifications);
  program_state->processing_script = 1;
  robot_pos_defined = 0;
  return 1;
}





/************************************************************************
 *
 *   NAME:         read_script()
 *                 
 *   FUNCTION:     opens a script file. Clears all maps.
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/

static int   last_reading_defined = 0;
static int   this_reading_defined = 0;
static float this_reading_time = 0.0;
static float last_reading_time = 0.0;
static float last_reading_x = 0.0;
static float last_reading_y = 0.0;
static int   velocity_defined = 0;
static float min_velocity = 0.0;
static float max_velocity = 0.0;
static float cumul_velocity = 0.0;
static float cumul_time = 0.0;

int read_script(ROBOT_STATE_PTR    robot_state,
		PROGRAM_STATE_PTR  program_state,
		ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int i, j, int_value;
  int int_value1, int_value2;
  float float_value, float_value1, float_value2, float_value3;
  float float_value4, float_value5;
  int file_ended, error;
  char command[256];
  static BASE_update_status_reply_type status;
  static LASER_laser_reply_type laser;
  struct timeval read_end_time;
  int print_flag = 0;
  int done_for_now;
  long int sleep_duration;
  float velocity, velocity2; 
  struct timeval time;

  static int new_format = 0;
  static int reading_pattern = 0;
  static int laser_data_found = 0;
  static int sonar_data_found = 0;
  static int position_found = 0;

  /*fprintf(stderr, "### Enter\n");*/

  file_ended = 0;
  error = 0;
  done_for_now = 0;


  laser.f_reading = 
    (int *) malloc(sizeof(int) * (robot_specifications->num_sensors / 2));
  laser.r_reading = 
    (int *) malloc(sizeof(int) * (robot_specifications->num_sensors / 2));
  if (laser.f_reading == NULL || laser.r_reading == NULL){
    printf("ABORT: out of memory in read_script!\n");
    exit(1);
  }
  
    

  do{
    if (script_iop == NULL || fscanf(script_iop, "%s", command) == EOF)
      file_ended = 1;
    else if (!strcmp(command, "#END"))
      file_ended = 1;
    
    else{

      /*----------------------------------------------*\
       *        begin(patternset)
       *----------------------------------------------*/

      if (!strcmp(command, "begin(patternset)")){
	new_format = 1;
      }

      /*----------------------------------------------*\
       *        begin(pattern)
       *----------------------------------------------*/

      else if (new_format && !strcmp(command, "begin(pattern)")){
	reading_pattern = 1;
	laser_data_found = 0;
	sonar_data_found = 0;
	position_found = 0;
      }


      /*----------------------------------------------*\
       *        lasers
       *----------------------------------------------*/

      else if (new_format && reading_pattern &&
	       !strcmp(command, "lasers")){
	if (fscanf(script_iop, "%d %d :", &int_value1, &int_value2) == EOF)
	  file_ended = 1;
	else{
	  if (int_value1 + int_value2 > robot_specifications->num_sensors){
	    fprintf(stderr, 
		    "ERROR: Format mismatch. Too many laser values (%d).\n",
		    int_value1 + int_value2);
	    error = 1;
	  }
	  else{
	    laser.f_numberOfReadings = int_value1;
	    laser.r_numberOfReadings = int_value2;
	    for (i = 0; i < int_value1; i++)
	      if (fscanf(script_iop, "%f", &float_value) == EOF)
		file_ended = 1;
	      else
		laser.f_reading[i] = (int) float_value;
	    for (i = 0; i < int_value2; i++)
	      if (fscanf(script_iop, "%f", &float_value) == EOF)
		file_ended = 1;
	      else
		laser.r_reading[i] = (int) float_value;
	  }
	  if (!file_ended)
	    laser_data_found = 1;
	}
      }

      /*----------------------------------------------*\
       *        sonars
       *----------------------------------------------*/

    
      else if (new_format && reading_pattern &&
	       !strcmp(command, "sonars")){
	if (fscanf(script_iop, "%d :", &int_value) == EOF)
	  file_ended = 1;
	else{
	  /* 
	     if (int_value != robot_specifications->num_sensors){
	    fprintf(stderr, 
	    "ERROR: Format mismatch. Wrong number of sonar values (%d).\n",
	    int_value);
	    error = 1;
	    }
	    else*/{
	      for (i = 0, j = robot_specifications->num_sensors / 2;
		   i < int_value;
		   i++, j = (j+1) % robot_specifications->num_sensors)
		if (fscanf(script_iop, "%f", &float_value) == EOF)
		  file_ended = 1;
	      /*
		else if (i < bRobot.sonar_cols[0]){
		sonar.values[j] = float_value;
		}
		else
		fprintf(stderr, " overflow");
	      */
	    }
	    if (!file_ended)
	      sonar_data_found = 1;
	}
      }
      

      /*----------------------------------------------*\
       *        position:
       *----------------------------------------------*/

      else if (new_format && reading_pattern &&
	       !strcmp(command, "position:")){

	if (fscanf(script_iop, "%f %f %f", &float_value1, &float_value2, 
		   &float_value3) == EOF)
	  file_ended = 1;
	else{

	  float_value3 = 90.0 - float_value3; /* correction for different
					       * ideas of what an angle is */

	  if (!robot_pos_defined){
	    status.trans_current_speed = status.rot_current_speed = 0.0;
	  }
	  else{
	    status.trans_current_speed = 
	      0.5 * sqrt(((status.pos_x - float_value1) 
			  * (status.pos_x - float_value1)) 
			 + ((status.pos_y - float_value2) 
			    * (status.pos_y - float_value2)));
	    status.rot_current_speed = fabs(status.orientation
					    - float_value3);
	    if (status.rot_current_speed >= 180.0)
	      status.rot_current_speed = 360.0 - status.rot_current_speed;
	  }
	  status.pos_x       = float_value1;
	  status.pos_y       = float_value2;
	  status.orientation = float_value3;
	  
	  if (print_flag) 
	    printf("#ROBOT %f %f %f (%f %f)\n",
		   float_value1, float_value2, float_value3,
		   status.trans_current_speed, status.rot_current_speed);
	  BASE_update_status_reply_handler(NULL, &status);
	  robot_pos_defined = 1;

	  /*
	   * statistics - velocity in the script file
	   */

	  if (this_reading_defined){
	    if (last_reading_defined && 
		last_reading_time < this_reading_time){
	      velocity2 = sqrt(((last_reading_x - status.pos_x) * 
				(last_reading_x - status.pos_x)) +
			       ((last_reading_y - status.pos_y) * 
				(last_reading_y - status.pos_y)));
	      velocity = velocity2 / (this_reading_time - last_reading_time);
	      if (!velocity_defined){
		velocity_defined = 1;
		min_velocity = max_velocity = velocity;
	      }
	      else{
		if (velocity < min_velocity)
		  min_velocity = velocity;
		if (velocity > max_velocity)
		  max_velocity = velocity;
	      }
	      cumul_velocity += velocity2;
	      cumul_time += (this_reading_time - last_reading_time);
	      
	      if (print_flag)
		printf("\tVelocity: current=%g min=%g max=%g avg=%g dist=%g\n",
		       velocity,
		       min_velocity, max_velocity, cumul_velocity /
		       cumul_time, cumul_velocity);
	      
	    }
	    
	    last_reading_defined = 1;
	    last_reading_time = this_reading_time;
	    this_reading_defined = 0;
	    last_reading_x = status.pos_x;
	    last_reading_y = status.pos_y;
	  }
	}
	position_found = 1;
      }

      /*----------------------------------------------*\
       *        time:
       *----------------------------------------------*/

      else if (new_format && reading_pattern &&
	       !strcmp(command, "time:")){

	if (fscanf(script_iop, "%ld %ld",
		   &(time.tv_sec), &(time.tv_usec)) == EOF)
	  file_ended = 1;
	else if (0){
	  fprintf(stderr, "\ntime: %ld %ld\n", time.tv_sec,
		  time.tv_usec);
	}
      }


      /*----------------------------------------------*\
       *        end(pattern)
       *----------------------------------------------*/

      else if (new_format && reading_pattern &&
	       !strcmp(command, "end(pattern)")){
	if (position_found && laser_data_found){
	  /*fprintf(stderr, "L");*/
	  LASER_laser_reply_handler(NULL, &laser);
	  /* usleep(500000); */
	  /* if (new_laser_reading) */
	  /* 	    done_for_now = 1; */
	  if (new_laser_reading)
	    done_for_now = 1;
	}
	if (position_found && sonar_data_found){
	  /*fprintf(stderr, "S");*/
	  /* SONAR_sonar_reply_handler(NULL, &sonar); */
	}
	reading_pattern = 0;
      }

      

      /*----------------------------------------------*\
       *++++++ @SENS +++++++++++++++++++++++++++++++++*|
       *----------------------------------------------*/

      else if (!new_format && !strcmp(command, "@SENS")){
	if (fscanf(script_iop, "%f-%f-%f", 
		   &float_value1, &float_value2, &float_value3) == EOF)
	  file_ended = 1;
	else{
	  if (0 || print_flag) 
	    printf("\n@SENS %.2d-%.2d-%.2d",
		   (int) float_value1, (int) float_value2, (int) float_value3);
	  if (fscanf(script_iop, "%f:%f:%f",
		     &float_value1, &float_value2, &float_value3) == EOF){
	    file_ended = 1;
	    if (print_flag) 
	      printf("\n");
	  }
	  else{
	    if (0 || print_flag) 
	      printf(" %.2d:%.2d:%6.3f\n", 
		     (int) float_value1, (int) float_value2, float_value3);
	    this_reading_time = (float_value1 * 3600.0) +
	      (float_value2 * 60.0) + float_value3;
	    this_reading_defined = 1;
	  }
	}
      }

      /*----------------------------------------------*\
       *++++++ @OPEN +++++++++++++++++++++++++++++++++*|
       *----------------------------------------------*/


      else if (!new_format && !strcmp(command, "@OPEN")){
	if (fscanf(script_iop, "%f-%f-%f", 
		   &float_value1, &float_value2, &float_value3) == EOF)
	  file_ended = 1;
	else{
	  if (print_flag) 
	    printf("@OPEN %g-%g-%g",
		   float_value1, float_value2, float_value3);
	  if (fscanf(script_iop, "%f:%f:%f",
		     &float_value1, &float_value2, &float_value3) == EOF){
	    file_ended = 1;
	    if (print_flag) 
	      printf("\n");
	  }
	  else
	    if (print_flag) 
	      printf(" %g:%g:%6.3f\n",
		     float_value1, float_value2, float_value3);
	}
      } 


      /*----------------------------------------------*\
       *++++++ #ROBOT ++++++++++++++++++++++++++++++++*|
       *----------------------------------------------*/


      else if (!new_format && !strcmp(command, "#ROBOT")){
	if (fscanf(script_iop, "%f %f %f %d", &float_value1, &float_value2, 
		   &float_value3, &int_value) == EOF)
	  file_ended = 1;
	else{

	  float_value3 = 90.0 - float_value3; /* correction for different
					       * ideas of what an angle is */

	  if (!robot_pos_defined){
	    status.trans_current_speed = status.rot_current_speed = 0.0;
	  }
	  else{
	    status.trans_current_speed = 
	      0.5 * sqrt(((status.pos_x - float_value1) 
			  * (status.pos_x - float_value1)) 
			 + ((status.pos_y - float_value2) 
			    * (status.pos_y - float_value2)));
	    status.rot_current_speed = fabs(status.orientation
					    - float_value3);
	    if (status.rot_current_speed >= 180.0)
	      status.rot_current_speed = 360.0 - status.rot_current_speed;
	  }
	  status.pos_x       = float_value1;
	  status.pos_y       = float_value2;
	  status.orientation = float_value3;

	  if (print_flag) 
	    printf("#ROBOT %f %f %f (%f %f)\n",
		   float_value1, float_value2, float_value3,
		   status.trans_current_speed, status.rot_current_speed);
	  BASE_update_status_reply_handler(NULL, &status);
	  robot_pos_defined = 1;

	  /*
	   * statistics - velocity in the script file
	   */

	  if (this_reading_defined){
	    if (last_reading_defined && 
		last_reading_time < this_reading_time){
	      velocity2 = sqrt(((last_reading_x - status.pos_x) * 
				(last_reading_x - status.pos_x)) +
			       ((last_reading_y - status.pos_y) * 
				(last_reading_y - status.pos_y)));
	      velocity = velocity2 / (this_reading_time - last_reading_time);
	      if (!velocity_defined){
		velocity_defined = 1;
		min_velocity = max_velocity = velocity;
	      }
	      else{
		if (velocity < min_velocity)
		  min_velocity = velocity;
		if (velocity > max_velocity)
		  max_velocity = velocity;
	      }
	      cumul_velocity += velocity2;
	      cumul_time += (this_reading_time - last_reading_time);
	      
	      if (print_flag)
		printf("\tVelocity: current=%g min=%g max=%g avg=%g dist=%g\n",
		       velocity,
		       min_velocity, max_velocity, cumul_velocity /
		       cumul_time, cumul_velocity);
	      
	    }
	    
	    last_reading_defined = 1;
	    last_reading_time = this_reading_time;
	    this_reading_defined = 0;
	    last_reading_x = status.pos_x;
	    last_reading_y = status.pos_y;
	  }
	}
      } 
      

      /*----------------------------------------------*\
       *++++++ #FLAG ++++++++++++++++++++++++++++++++*|
       *----------------------------------------------*/


      else if (!new_format && !strcmp(command, "#FLAG")){
	if (fscanf(script_iop, " %d", &int_value) == EOF)
	  file_ended = 1;
	/*	else
		fprintf(stderr, "#FLAG %d\n", int_value);*/
      } 
      
      /*----------------------------------------------*\
       *++++++ #LASER ++++++++++++++++++++++++++++++++*|
       *----------------------------------------------*/


      else if (!new_format && !strcmp(command, "#LASER")){
	if (fscanf(script_iop, "%d %d:", &int_value1, &int_value2) == EOF)
	  file_ended = 1;
	else{
	  int_value = int_value1 + int_value2;
	  laser.f_numberOfReadings = int_value1;
	  laser.r_numberOfReadings = int_value2;
	  if (int_value > robot_specifications->num_sensors){
	    fprintf(stderr, 
		    "ERROR: Format mismatch. Too many laser values (%d).\n",
		    int_value);
	    error = 1;
	  }
	  else{
	    if (print_flag) 
	      printf("#LASER %d:", int_value);
	    for (i = 0; i < robot_specifications->num_sensors && 
		 !file_ended; i++){
	      if (i < int_value){
		if (fscanf(script_iop, " %f", &float_value) == EOF)
		  file_ended = 1;
	      }
	      else
		float_value = -1.0; /* not defined */
	      if (file_ended)
		;
	      else if (i < robot_specifications->num_sensors / 2){	
		laser.f_reading[i] = (int) float_value;
		if (print_flag) 
		  printf(" %f", float_value);
	      }
	      else if (i < robot_specifications->num_sensors){	
		laser.r_reading[i - (robot_specifications->num_sensors / 2)]
		  = (int) float_value;
		if (print_flag) 
		  printf(" %f", float_value);
	      }
	      else
		fprintf(stderr, " overflow");
	    }
	    if (print_flag) 
	      printf("\n");
	    if (!file_ended){
	      LASER_laser_reply_handler(NULL, &laser);
	      if (new_laser_reading)
		done_for_now = 1;
	    }
	  }
	}
      }
     
      /*----------------------------------------------*\
       *++++++ #IMAGEFILE ++++++++++++++++++++++++++++++++*|
       *----------------------------------------------*/


      else if (!new_format && !strcmp(command, "#IMAGEFILE")){
	if (fscanf(script_iop, "%d %d", &int_value1, &int_value2) == EOF)
	  file_ended = 1;
	else{
	  if (int_value1 <= 0 || int_value2 <= 0){
	    fprintf(stderr, 
		      "ERROR: Invalid image size: %d %d.\n", 
		    int_value1, int_value2);
	    error = 1;
	  }
	  else if (vr_iop1 && robot_specifications->generate_3D_vrml){
	    program_state->image_file_size_x = int_value1;
	    program_state->image_file_size_y = int_value2;
	    program_state->image_current_x = 0;
	    program_state->image_most_recent_x = 0;
	    texture_image = make_image(program_state->image_file_size_x,
				       program_state->image_file_size_y);
	  }
	}
      }
 
      /*----------------------------------------------*\
       *++++++ #IMAGELINE ++++++++++++++++++++++++++++++++*|
       *----------------------------------------------*/


      else if (!new_format && !strcmp(command, "#IMAGELINE")){

	if (fscanf(script_iop, "%d:", &int_value) == EOF)
	  file_ended = 1;
	else{
	  if (int_value != program_state->image_file_size_y){
	    fprintf(stderr, 
		    "ERROR: Format mismatch. Wrong numer image values (%d %d).\n",

		    int_value, program_state->image_file_size_y);
	    error = 1;
	  }
	  else{
	    int r2, b2, g2;

	    if (print_flag) 
	      printf("#IMAGELINE %d:\n", int_value);

	    if (!red){
	      red = (unsigned char *) malloc(sizeof(unsigned char) 
					     * int_value);
	      green = (unsigned char *) malloc(sizeof(unsigned char) 
					       * int_value);
	      blue = (unsigned char *) malloc(sizeof(unsigned char) 
					      * int_value);
	    }
	    for (i = 0; i < int_value && !file_ended; i++)
	      if (fscanf(script_iop, "%2x", &r2) == EOF){
		fprintf(stderr, "-1-");
		file_ended = 1;
	      }
	      else{
		if (fscanf(script_iop, "%2x", &g2) == EOF){
		  fprintf(stderr, "-2-");
		  file_ended = 1;
		}
		else{
		  if (fscanf(script_iop, "%2x", &b2) == EOF){
		    fprintf(stderr, "-3-");
		    file_ended = 1;
		  }
		  else{
		    
		    // red[i] = (unsigned char) (255.0 - (((float) (255-r2)) / 1.5));
		    // green[i] = (unsigned char) (255.0 - (((float) (255-g2)) / 1.5));
		    // blue[i] = (unsigned char) (255.0 - (((float) (255-b2)) / 1.5));
		    red[i] = (unsigned char) r2;
		    green[i] = (unsigned char) g2;
		    blue[i] = (unsigned char) b2;
		  }
		}
	      }
	    
	    if (program_state->image_file_size_x > 0 &&
		program_state->image_file_size_y > 0){

	      if (texture_image){
		for (j = 0; j < program_state->image_file_size_y; j++){
		  int ind =  j * program_state->image_file_size_x +
		    program_state->image_current_x;
		  texture_image->red[ind] = red[j];
		  texture_image->green[ind] = green[j];
		  texture_image->blue[ind] = blue[j];
		}
		program_state->image_current_x++;
	      }


	      /* ------------------------------ */
	      if (0)
		{
		  int j;
		  fprintf(stderr, "\nXXX: ");
		  for (j = 0; j < program_state->image_file_size_y; j++){
		    fprintf(stderr, "%d ", (int) red[j]);
		    fprintf(stderr, "%d ", (int) green[j]);
		    fprintf(stderr, "%d ", (int) blue[j]);
		  }
		  fprintf(stderr, "\n");

		}

	      if (0)
		{
		  static float img_x = 0.0;
		  static float img_y = 0.0;
		  int j;
	      
		  for (j = 0; j < program_state->image_file_size_y; j++){
		    fill_region_fraction(bg_image, 
					 img_x, img_x + 0.01,
					 img_y, img_y + 0.01,
					 red[j], green[j], blue[j]);
		    img_x += 0.01;
		    if (img_x > 0.995){
		      img_x = 0.0;
		      img_y += 0.01;
		      if (img_y > 0.995)
			img_y = 0.0;
		    }
		  }
		  save_image(bg_image, "random.ppm");
		  exit(-1);
		}
	      /* ------------------------------ */
	    }
	  }
	}
      }      
      
      /*----------------------------------------------*\
       *++++++ #SONAR ++++++++++++++++++++++++++++++++*|
       *----------------------------------------------*/


      else if (!new_format && !strcmp(command, "#SONAR")){
	if (fscanf(script_iop, "%d:", &int_value) == EOF)
	  file_ended = 1;
	else{
	  if (int_value != 24){	/* current max. value on SONAR-messages.h */
	    fprintf(stderr, 
		    "ERROR: Format mismatch. Wrong number sonar values.\n");
	    error = 1;
	  }
	  else{
	    if (print_flag) 
	      printf("#SONAR %d:", int_value);
	    for (i = 0; i < int_value && !file_ended; i++){
	      if (fscanf(script_iop, "%f", &float_value) == EOF)
		file_ended = 1;
	      else if (i < 24){	/* current max. value on SONAR-messages.h */
		if (print_flag) 
		  printf(" %f", float_value);
	      }
	      else
		fprintf(stderr, " overflow");
	    }
	    if (print_flag) 
	      printf("\n");
	  }
	}
      }
      

      /*----------------------------------------------*\
       *++++++ #PARAMS ++++++++++++++++++++++++++++++++*|
       *----------------------------------------------*/
      

      else if (!new_format && !strcmp(command, "#PARAMS")){
	if (fscanf(script_iop, "%f %f %f %d %f %f", 
		   &float_value1, &float_value2, &float_value3, 
		   &int_value, &float_value4, &float_value5) == EOF)
	  file_ended = 1;
	else{

	  robot_state->correction_parameter_x = float_value1;
	  robot_state->correction_parameter_y = float_value2;
	  robot_state->correction_parameter_angle = float_value3;
	  robot_state->correction_type = int_value;
	  robot_specifications->pos_corr_drift_trans = float_value4;
	  robot_specifications->pos_corr_drift_rot = float_value5;

	  if (print_flag) 
	    printf("#PARAMS %f %f %f %d   %f %f\n",
		   float_value1, float_value2, float_value3, int_value,
		   float_value4, float_value5);

	}
      }


      
      if (file_ended == 1){
	  fprintf(stderr, "Surprising end of script file %s.\n ", 
		  script_filename);
	error = 1;
      }
      
    }
  } while (!file_ended && !error && !done_for_now);


  if (file_ended || error){
    fclose(script_iop);
    program_state->processing_script      = 0;
    if (program_state->graphics_initialized)
      G_display_switch(SCRIPT_BUTTON, 0);
    script_iop = NULL;
    last_reading_defined = 0;
    this_reading_defined = 0;
    velocity_defined = 0;
  }

  free(laser.f_reading);
  free(laser.r_reading);
  /*fprintf(stderr, "### Exit\n");*/
  return (!error);
}




/************************************************************************
 *
 *   NAME:         close_script()
 *                 
 *   FUNCTION:     opens a script file. Clears all maps.
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/



void close_script(ROBOT_STATE_PTR    robot_state,
		  PROGRAM_STATE_PTR  program_state,
		  ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  if (program_state->processing_script){
    fclose(script_iop);
    program_state->processing_script = 0;
    if (program_state->graphics_initialized)
      G_display_switch(SCRIPT_BUTTON, 0);
    script_iop = NULL;
  }
}


/************************************************************************
 *
 *   NAME:         open_log()
 *                 
 *   FUNCTION:     Opens a log file
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

int open_log(char *filename)
{
  char filename2[256];
  time_t current_time;
  char out_text[256];
  struct timeval t; 

  if (!program_state->logging_on){
    log_iop = NULL;
    fprintf(stderr, "No logging.\n");
    return 0;
  }
  
  if (!robotname)
    sprintf(filename2, "%s", filename);
  else
    sprintf(filename2, "%s.%s", filename, robotname);
  if ((log_iop = fopen(filename2, "a")) == 0){
    fprintf(stderr, "AARNING: Could not open log file %s.\n", filename2);
    sprintf(filename2, "../etc/%s", filename);
    if ((log_iop = fopen(filename2, "a")) == 0){
      fprintf(stderr, "AARNING: Could not open log file %s.\n", filename2);
      sprintf(filename2, "../../etc/%s", filename);
      if ((log_iop = fopen(filename2, "a")) == 0){
	fprintf(stderr, 
		"WARNING: Could not open log file %s. Not saved.\n",
		filename2);
	log_iop = NULL;
	return 0;
      }
    }
  }
  
  
  
  time(&current_time);
  strftime(out_text, 50 , "@OPEN %d-%m-%y %H:%M:%S", 
	   localtime(&current_time));
  fprintf(log_iop, "%s.%d\n\n", out_text, (int) t.tv_usec);

  fprintf(stderr, "Log file %s successfully opened. Logging is on.\n", 
	  filename2);
  return 1;
}


/************************************************************************
 *
 *   NAME:         close_log()
 *                 
 *   FUNCTION:     Closes a log file
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void close_log()
{
  if (log_iop == NULL)
    return;


  fclose(log_iop);

  fprintf(stderr, "Log file closed. Logging is off.\n");
}




/************************************************************************
 *
 *   NAME:         save_gif()
 *                 
 *   FUNCTION:     saves map into a file. Map-intrinsic format.
 *                 
 *   PARAMETERS:   filename
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

#include "gd.h"
static struct timeval last_gif = {0, 0};

int save_gif(char *filename)
{
  int i, j;
  FILE *iop;
  register int gifSizeX, gifSizeY;
  gdImagePtr GifPic;
  int unknownColor, pathColor;
  int col;



  if ((iop = fopen(filename, "w")) == 0){
    fprintf(stderr, "WARNING: Could not open output file %s.\n", filename);
    return -1;
  }
  
  gifSizeX = robot_specifications->gif_image_size;
  gifSizeY = robot_specifications->gif_image_size;
  GifPic = gdImageCreate( gifSizeX, gifSizeY);
  
  unknownColor = gdImageColorAllocate(GifPic, 0, 134, 138);
  pathColor    = gdImageColorAllocate(GifPic, 0, 30, 255);

  
  for (i = 0; i < robot_specifications->gif_image_size; i++)
    for (j = 0; j < robot_specifications->gif_image_size; j++){
      if ((i+j)%2)
	col = unknownColor;
      else
	col = pathColor;
      gdImageSetPixel( GifPic, i, gifSizeY - j - 1, col );
    }

  gdImageGif( GifPic, iop );
  fclose(iop);
    
  fprintf(stderr, "File %s successfully written.\n", filename);
  return 0;
}

