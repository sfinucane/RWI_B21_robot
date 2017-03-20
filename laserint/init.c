
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/laserint/init.c,v $
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
 * $Log: init.c,v $
 * Revision 1.1  2002/09/14 15:34:07  rstone
 * *** empty log message ***
 *
 * Revision 1.34  2000/09/26 01:11:33  thrun
 * ,
 *
 * Revision 1.33  2000/03/14 05:23:57  thrun
 * fixed a few problems with the 3D mapping. This version
 * pruced the results first shown to the DAPRA folks.
 * In particular, it reads the texture entirely from file and
 * constructs a new texture file (laserint.ppm, which has
 * to be converted manyally into laserint.jpg)
 *
 * Revision 1.32  2000/03/13 00:31:12  thrun
 * first version of he texturing module. Still a bit clumpsy.
 *
 * Revision 1.31  2000/01/11 12:27:10  schneid1
 * command line options are now shown comp.
 *
 * Revision 1.30  1999/12/27 03:43:16  thrun
 * Magic: Gradient descent for adapting
 * wall parameters xw yw and aw seems to work!!!
 *
 * Revision 1.29  1999/12/19 03:25:17  thrun
 * graphics for 3D object matching.
 *
 * Revision 1.28  1999/10/15 18:07:20  thrun
 * more 3D formats, more compact files.
 *
 * Revision 1.27  1999/10/15 01:30:47  thrun
 * improved interface for robot positioning
 *
 * Revision 1.26  1999/10/02 04:50:12  thrun
 * Minor tuning and changes. THis version built the first
 * nice map of the San Jose Tech Museum
 *
 * Revision 1.25  1999/10/01 17:58:18  thrun
 * reinstatiated the close-the-cycle feature.
 *
 * Revision 1.24  1999/09/28 21:49:54  thrun
 * Multi-robot support in various modules.
 * In particular, I had to introduce a new message for MAP that
 * communicates back robot positions and correction parameters.
 * This might make it necessary to modify existing modules that use
 * MAP - I tried to do this for all I am aware of, but I might have missed some.
 *
 * Revision 1.23  1999/09/28 04:50:01  thrun
 * Fixed minor bug in server architecture that caused the map
 * (in MAP) to be inconsistent.
 *
 * Revision 1.22  1999/09/06 03:19:45  thrun
 * first working version of the multi-robot server architecture with
 * positon control.
 *
 * Revision 1.21  1999/09/05 21:57:22  thrun
 * initial client server robot for multi-robot mapping
 *
 * Revision 1.20  1999/09/05 17:18:52  thrun
 * 3D ceiling mapping
 *
 * Revision 1.18  1999/07/03 21:44:37  thrun
 * Fixed several bugs in LASERINT and tuned the parameters of
 * LASERINT, MAP, and PLAN for the new Urban Robot
 *
 * Revision 1.17  1999/07/03 18:49:36  thrun
 * LOASERINT can now send correction parameters directly to MAP.
 *
 * Revision 1.16  1999/06/21 18:30:25  thrun
 * Took out the sampling stuff (which never worked corretly). It's now
 * up to Dieter to put it in again...
 *
 * Revision 1.15  1999/05/08 19:47:29  thrun
 * drastically improved version with recursive matching and samples.
 * Not quite ready yet: samples aren't used in the right way, queue
 * seems not to work, and some constants are hardcoded (phase!) But
 * impressive results!
 *
 * Revision 1.14  1999/05/04 01:14:22  thrun
 * queue instead of stack.
 *
 * Revision 1.13  1999/04/23 20:00:10  thrun
 * slight extensions - limited stack (is now a queue) and flag that
 * prevents integration of readings after sharp turn.
 *
 * Revision 1.12  1998/11/18 04:21:11  thrun
 * incorporated wall angle into estimation.
 *
 * Revision 1.11  1998/11/17 05:03:14  thrun
 * small, incremental improvements: better handling of sensor
 * displacement, and new parametr that enables laserint to ignore
 * scans when the robot is rotating too much.
 *
 * Revision 1.10  1998/11/16 01:50:41  thrun
 * positon control - works nice, includes global mapping and learning
 * the drift parameters.
 *
 * Revision 1.9  1998/11/15 16:19:18  thrun
 * initial position control method - works reasonably well. But
 * e careful with the timing, isn't an anytime algorithm yet.
 *
 * Revision 1.8  1998/11/12 14:22:55  thrun
 * ?.
 *
 * Revision 1.7  1998/08/11 22:18:25  thrun
 * PLANNER: fixed a problem with large shifts, a problem with th
 * function "pow()" under the latest Linux.
 *
 * Revision 1.6  1997/12/30 00:40:11  thrun
 * Non-neural network version of mapping.
 *
 * Revision 1.5  1997/08/12 03:06:02  thrun
 * intermediate version, provides some logging of sensor scans and partial
 * maps within map.
 *
 * Revision 1.4  1997/05/28 12:37:34  thrun
 * minor changes.
 *
 * Revision 1.3  1997/04/27 12:27:20  thrun
 * Extended parameter set (copied from SONARINT)
 *
 * Revision 1.2  1997/02/02 22:32:36  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
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




#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif

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
#include "MAP-messages.h"
#include "LASER-messages.h"
#include "BASE-messages.h"
#include "LASERINT.h"
#include "bUtils.h"


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

char init_file_name[256];
char *robotname = NULL;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/************************************************************************
 *
 *   NAME:         check_commandline_parameters
 *                 
 *   FUNCTION:     checks the command line options!
 *                 
 *   PARAMETERS:   
 *                 
 *                 NOTE: "NULL" disables initialization
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void check_commandline_parameters(int argc, char **argv, 
				  PROGRAM_STATE_PTR program_state)
{
  int i, bug = 0;



  program_state->use_graphics             = 1; /* default */
  program_state->use_tcx                  = 1; /* default */
  program_state->delay_in_replay          = 0.75; /* default, like RHINO */
  program_state->logging_on               = 1; /* default */
  program_state->force_logging_on         = 0; /* default */
  program_state->preprocess_images        = 0; /* default */
  program_state->is_server  = 0; /* default */


  strcpy(init_file_name, LASERINT_INIT_NAME);

  for (i = 1; i < argc && !bug; i++){
    if (!strcmp(argv[i], "-nodisplay") || !strcmp(argv[i], "-nd"))
      program_state->use_graphics = 0;
    else if (!strcmp(argv[i], "-notcx") || !strcmp(argv[i], "-nt"))
      program_state->use_tcx = 0;
    else if (!strcmp(argv[i], "-server") || !strcmp(argv[i], "-s"))
      program_state->is_server = 1;
    else if (!strcmp(argv[i], "-nolog")  || !strcmp(argv[i], "-nl")){
      program_state->logging_on = 0;
      program_state->force_logging_on = 0;
    }
    else if (!strcmp(argv[i], "-magic")){
      program_state->object_matching_mode = 1;
    }
    else if (!strcmp(argv[i], "-images")){
      program_state->preprocess_images = 1;
    }
    else if (!strcmp(argv[i], "-forcelog")  || !strcmp(argv[i], "-fl")){
      program_state->logging_on = 1;
      program_state->force_logging_on = 1;
    }
    else if (!strcmp(argv[i], "-delay") || !strcmp(argv[i], "-de")){
      if (++i >= argc)
	bug = 3;
      else if (argv[i][0] == '-')
	bug = 3;
      else
	program_state->delay_in_replay = atof(argv[i]);
    }
    else if (!strcmp(argv[i], "-file") || !strcmp(argv[i], "-file")){
      if (i < argc - 1){
	strcpy(init_file_name, argv[++i]);
	if (init_file_name[0] == '-')
	bug = 2;
      }
      else
	bug = 2;
    }
    else if (!strcmp(argv[i], "-robot") || !strcmp(argv[i], "-r")){
      if (i < argc - 1){
	robotname = (char *) malloc(sizeof(char) * 256);
	strcpy(robotname, argv[++i]);
	if (init_file_name[0] == '-')
	bug = 2;
      }
      else
	bug = 2;
    }
    else
      bug = 1;
  }
  
  if (bug == 1)
    fprintf(stderr, "\nUsage: '%s [-nodisplay] [-notcx] [-file <filename>] [-robot <robotname>] [-delay <sec>] [-nolog|-forcelog] [-server] [-magic]\n", 
	    argv[0]);

  else if (bug == 2)
    fprintf(stderr, "\nError: Option -file must be followed by file name.\n");

  else if (bug == 3)
    fprintf(stderr, "\nError: Option -delay must be followed by float.\n");

  if (bug >= 1)
    exit(1);

}



/************************************************************************
 *
 *   NAME:         init_program
 *                 
 *   FUNCTION:     Initializes the structure "program_state" and "robot_state"
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_STATE_PTR  robot_state      Pointer  to general
 *                                                   robot state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *                 NEURAL_NETWORK_PTR neural_network   pointer to network
 *                 ALL_PTR            structure to all variables
 *                 
 *                 
 *                 NOTE: "NULL" disables initialization
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void init_program(PROGRAM_STATE_PTR        program_state,
		  ROBOT_STATE_PTR          robot_state,
		  ROBOT_SPECIFICATIONS_PTR robot_specifications,
		  NEURAL_NETWORK_PTR       neural_network,
		  ALL_PTR                  all)
{
  int i;
  if (program_state != NULL){
    program_state->tcx_initialized                         = 0;
    program_state->graphics_initialized                    = 0;
    /* program_state->use_graphics           = 1; command line option */
    /* program_state->use_tcx                = 1; command line option */
    program_state->regular_local_map_display               = 0;
    program_state->regular_local_error_display             = 0;
    program_state->maps_allocated                          = 0;
    program_state->base_connected                          = 0;
    program_state->map_connected                           = 0;
    /* program_state->delay_in_replay        = 0.75; command line option */
    program_state->quit                                    = 0; 
    program_state->processing_script                       = 0;
    /* program_state->logging_on             = 1; command line option */
    /* program_state->force_logging_on       = 0; command line option */
    program_state->map_update_pending                      = 0;
    program_state->position_control_iteration              = 0;
    program_state->first_stack_item                        = 0;
    program_state->last_stack_item                         = 0;
    program_state->stack                                   = NULL;
    program_state->strong_turn                             = 0;
    program_state->remaining_error                         = 0.0;
    program_state->data_count                              = 0;
    program_state->found_nearest_stack_item                = 0;
    program_state->nearest_stack_item                      = 0;
    program_state->cache                                   = 0;
    program_state->first_base_report_received              = 0;
    program_state->calibration                             = -1;
    /* program_state->is_server  = 0; command line option*/

    program_state->offs_x                                  = 0.0;
    program_state->offs_y                                  = 0.0;
    program_state->number_objects                          = 0;
    for (i = 0; i < MAX_NUMBER_OBJECTS; i++)
      program_state->objects[i]                            = NULL;

    program_state->image_file_size_x                       = -1;
    program_state->image_file_size_y                       = -1;
    program_state->image_current_x                         = 0;
    program_state->image_most_recent_x                     = 0;
    /* program_state->object_matching_mode = 0; command line option*/
    /* program_state->preprocess_images = 0; command line option*/
  }


  if (robot_state != NULL){
    robot_state->known                                     = 0;
    robot_state->x                                         = 0.0;
    robot_state->y                                         = 0.0;
    robot_state->orientation                               = 0.0;
    robot_state->raw_odometry_x                            = 0.0;
    robot_state->raw_odometry_y                            = 0.0;
    robot_state->raw_odometry_orientation                  = 0.0;
    robot_state->laser_x                                   = 0.0;
    robot_state->laser_y                                   = 0.0;
    robot_state->laser_orientation                         = 0.0;
    robot_state->prev_laser_x                              = 0.0;
    robot_state->prev_laser_y                              = 0.0;
    robot_state->prev_laser_orientation                    = 0.0;
    robot_state->prev_laser_defined                        = 0;
    robot_state->translation_between_scans                 = 0.0;
    robot_state->rotation_between_scans_1                  = 0.0;
    robot_state->rotation_between_scans_2                  = 0.0;
    robot_state->translational_speed                       = 0.0;
    robot_state->rotational_speed                          = 0.0;
    robot_state->correction_parameter_x                    = 0.0;
    robot_state->correction_parameter_y                    = 0.0;
    robot_state->correction_parameter_angle                = 0.0;
    robot_state->correction_type                           = 0;

    robot_state->angle_to_wall_found                       = 0;
    robot_state->wall_angle                                = 0.0;

  }


  if (robot_specifications != NULL){
    robot_specifications->local_mapsize_x                  = 600.0;
    robot_specifications->local_mapsize_y                  = 600.0;
    robot_specifications->local_map_dim_x                  = 61;
    robot_specifications->local_map_dim_y                  = 61;
    robot_specifications->resolution                       = 10.0;
    robot_specifications->pos_resolution                   = 15.0;
    robot_specifications->smooth_radius                    = 3;
  
    robot_specifications->robot_size                       = 30.0;
    robot_specifications->front_laser_offset_x             = 0.0;
    robot_specifications->front_laser_offset_y             = 0.0;
    robot_specifications->rear_laser_offset_x              = 0.0;
    robot_specifications->rear_laser_offset_y              = 0.0;
  
    robot_specifications->num_sensors                      = 360;
    robot_specifications->first_sensor_angle               = -7.5;
    robot_specifications->max_sensors_range                = 300.0;
    robot_specifications->min_sensors_range                = 1.0;
    robot_specifications->sensor_increment                 = 0.0;
    robot_specifications->neuronet_max_sensors_range       = 300.0;
    robot_specifications->neuronet_min_sensors_range       = 1.0;
    robot_specifications->sensor_angles                    = NULL;
    robot_specifications->sin_sensor_angles                = NULL;
    robot_specifications->cos_sensor_angles                = NULL;
    robot_specifications->aux_sin_sensor_angles            = NULL;
    robot_specifications->aux_cos_sensor_angles            = NULL;

    robot_specifications->max_occupied_sensors_range       = 300.0;
    robot_specifications->occupied_outer_width             = 300.0;
    robot_specifications->network_occupied_value           = 0.75;

    robot_specifications->use_neural_networks              = 1;
    robot_specifications->network_value_mean               = 0.75;
    robot_specifications->decay_with_distance              = 1;
    robot_specifications->standard_free                    = 0.9;
    robot_specifications->standard_occupied                = 0.2;
    robot_specifications->obstacle_width                   = 30.0;

    robot_specifications->line_recognition_neighbors       = 5;
    robot_specifications->line_recognition_threshold       = 0.01;
    robot_specifications->min_advancement_between_interpretations = 5.0;
    robot_specifications->broadcast_sensor_data_to_map     = 0;


    robot_specifications->do_position_correction           = 1;
    robot_specifications->pos_corr_angle_margin            = 2.0;
    robot_specifications->pos_corr_max_dist                = 30.0;
    robot_specifications->pos_corr_max_angle               = 180.0;
    robot_specifications->pos_corr_max_range               = 500.0;
    robot_specifications->pos_corr_max_d_trans             = 20.0;
    robot_specifications->pos_corr_max_d_rot               = 5.0;
    robot_specifications->pos_corr_lrate_trans             = 0.05;
    robot_specifications->pos_corr_lrate_rot               = 0.01;
    robot_specifications->pos_corr_min_num_iterations      = 5;
    robot_specifications->pos_corr_max_num_iterations      = 20;
    robot_specifications->pos_corr_hinge_point_offset      = 150.0;
    robot_specifications->pos_corr_hinge_dist_threshold    = 100.0;
    robot_specifications->pos_corr_rot_cutoff_angle        = 10.0;
    robot_specifications->ignore_odometry                  = 0;
    robot_specifications->stack_size                       = 1000;
    robot_specifications->strong_turn_threshold            = 5.0;
    robot_specifications->strong_turn_decay                = 10;
    robot_specifications->pos_corr_max_dist_for_server     = 50.0;
    robot_specifications->pos_corr_max_angle_for_server    = 30.0;

    robot_specifications->pos_corr_do_est_drift            = 1;
    robot_specifications->pos_corr_drift_trans             = 0.0;
    robot_specifications->pos_corr_drift_rot               = 0.0;
    robot_specifications->pos_corr_lrate_drift             = 0.01;

    robot_specifications->angle_dev_threshold              = 5.0;
    robot_specifications->show_all_points                  = 0;
    robot_specifications->send_corr_parameters_to_map      = 0;
    robot_specifications->reposition_robot_initially       = 1;

    robot_specifications->gif_image_size                   = 400;
    robot_specifications->gif_image_frequency              = -1;
    robot_specifications->display_fixed_rotation           = 0;
    robot_specifications->display_interval                 = 1;
    robot_specifications->display_density                  = 1;
    robot_specifications->world_size                       = 4000.0;

    robot_specifications->diagnose_step                    = -1;
    robot_specifications->ignore_rear_laser                = 0;
    robot_specifications->ignore_front_laser               = 0;

    robot_specifications->plot_auto_on                     = 0;
    robot_specifications->do_backwards_corrections         = 0;
    

    robot_specifications->generate_3D_vrml                 = 0;
    robot_specifications->generate_3D_smf                  = 0;
    robot_specifications->generate_3D_math                 = 0;

    robot_specifications->pos_map_dim = 2 * 
      ((int) (robot_specifications->pos_corr_max_range /
	      robot_specifications->pos_resolution));
  }

  
  if (all != NULL){
    all->program_state        = program_state;
    all->robot_specifications = robot_specifications;
    all->robot_state          = robot_state;
    all->neural_network       = neural_network;
  }
}


/************************************************************************
 *
 *   NAME:         allocate_everything
 *                 
 *   FUNCTION:     allocates memory for internal maps
 *                 
 *   PARAMETERS:   ROBOT_STATE_PTR    robot_state    Pointer to robot state
 *                                                   description structure
 *                 PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void allocate_everything(ROBOT_STATE_PTR    robot_state,
			 PROGRAM_STATE_PTR  program_state,
			 ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int i, j, k, global_size, local_size, local_error_size;


  if (!program_state->maps_allocated){
    
    robot_specifications->local_mapsize_x   = 
      robot_specifications->max_sensors_range * 2.0;
    robot_specifications->local_mapsize_y   = 
      robot_specifications->max_sensors_range * 2.0;
    robot_specifications->local_map_dim_x   = 1 + (int) /* 1=safety margin */
      (robot_specifications->local_mapsize_x 
       / robot_specifications->resolution);
    robot_specifications->local_map_dim_y   = 1 + (int) /* 1=safety margin */
      (robot_specifications->local_mapsize_y 
       / robot_specifications->resolution);


    
    /* ALLOCATE NEW MEMORY FOR THE MAPS */
    
    local_size = robot_specifications->local_map_dim_x
      * robot_specifications->local_map_dim_y;
    local_error_size = robot_specifications->pos_map_dim *
      robot_specifications->pos_map_dim;
    if (!program_state->maps_allocated){
      local_map     = (float *) (malloc(sizeof(float) * local_size));
      local_robot   = (int *)   (malloc(sizeof(int)   * local_size));
      local_active  = (int *)   (malloc(sizeof(int)   * local_size));
      local_error   = (float *) (malloc(sizeof(float) * local_error_size));

      if (local_map == NULL || local_active == NULL || local_robot == NULL ||
	  local_error == NULL){
	printf("ABORT: out of memory 1!\n");
	exit(1);
      }
      program_state->maps_allocated = 1;
    }
    
    /* ALLOCATE NEW MEMORY FOR THE SENSORS */
    
    if (robot_specifications->sensor_angles != NULL)
      free(robot_specifications->sensor_angles);
    if (robot_specifications->sin_sensor_angles != NULL)
      free(robot_specifications->sin_sensor_angles);
    if (robot_specifications->cos_sensor_angles != NULL)
      free(robot_specifications->cos_sensor_angles);
    if (robot_specifications->aux_sin_sensor_angles != NULL)
      free(robot_specifications->aux_sin_sensor_angles);
    if (robot_specifications->aux_cos_sensor_angles != NULL)
      free(robot_specifications->aux_cos_sensor_angles);
    if (robot_state->sensor_values != NULL)
      free(robot_state->sensor_values);
    if (robot_state->sensor_endpoint_x != NULL)
      free(robot_state->sensor_endpoint_x);
    if (robot_state->sensor_endpoint_y != NULL)
      free(robot_state->sensor_endpoint_y);
    if (program_state->stack){
      for (i = 0; i < robot_specifications->stack_size; i++)
	free(program_state->stack[i].sensor_values);
      free(program_state->stack);
      program_state->stack = NULL;
    }
    
    
    robot_specifications->sensor_angles = 
      (float *) (malloc(sizeof(float) * 
			robot_specifications->num_sensors));
    robot_specifications->sin_sensor_angles = 
      (float *) (malloc(sizeof(float) * 
			robot_specifications->num_sensors));
    robot_specifications->cos_sensor_angles = 
      (float *) (malloc(sizeof(float) * 
			robot_specifications->num_sensors));
    robot_specifications->aux_sin_sensor_angles = 
      (float *) (malloc(sizeof(float) * 
			robot_specifications->num_sensors));
    robot_specifications->aux_cos_sensor_angles = 
      (float *) (malloc(sizeof(float) * 
			robot_specifications->num_sensors));
    robot_state->sensor_values = 
      (float *) (malloc(sizeof(float) * 
			robot_specifications->num_sensors));
    robot_state->sensor_endpoint_x = 
      (float *) (malloc(sizeof(float) * 
			robot_specifications->num_sensors));
    robot_state->sensor_endpoint_y = 
      (float *) (malloc(sizeof(float) * 
			robot_specifications->num_sensors));
    program_state->stack = 
      (stack_item_ptr) malloc(sizeof(stack_item_type) *
			      robot_specifications->stack_size);

    if (robot_specifications->sensor_angles == NULL ||
	robot_specifications->sin_sensor_angles == NULL ||
	robot_specifications->cos_sensor_angles == NULL ||
	robot_specifications->aux_sin_sensor_angles == NULL ||
	robot_specifications->aux_cos_sensor_angles == NULL ||
	robot_state->sensor_values == NULL ||
	robot_state->sensor_endpoint_x == NULL ||
	robot_state->sensor_endpoint_y == NULL ||
	program_state->stack == NULL){
      printf("ABORT: out of memory 2!\n");
      exit(1);
    }


    for (i = 0; i < robot_specifications->stack_size; i++){
      program_state->stack[i].sensor_values = NULL;
      program_state->stack[i].error = NULL;
      program_state->stack[i].nearest_x = NULL;
      program_state->stack[i].nearest_y = NULL;
      program_state->stack[i].nearest_index = NULL;
      program_state->stack[i].admissble_sensor_value = NULL;
      program_state->stack[i].sensor_x = NULL;
      program_state->stack[i].sensor_y = NULL;
      program_state->stack[i].sensor_z = NULL;
      program_state->stack[i].sensor_angle_to_base = NULL;
      program_state->stack[i].sin_sensor_angle_to_base = NULL;
      program_state->stack[i].cos_sensor_angle_to_base = NULL;
      program_state->stack[i].dist_to_base = NULL;

    }

    for (i = 0; i < robot_specifications->num_sensors; i++){
      robot_specifications->sensor_angles[i] = 
	robot_specifications->first_sensor_angle 
	  + (360.0 * ((float) i)
	     / ((float)  robot_specifications->num_sensors));
      robot_specifications->sin_sensor_angles[i] = 
	robot_specifications->aux_sin_sensor_angles[i] = 
	sin(robot_specifications->sensor_angles[i] * M_PI / 180.0);
      robot_specifications->cos_sensor_angles[i] = 
	robot_specifications->aux_cos_sensor_angles[i] = 
	cos(robot_specifications->sensor_angles[i] * M_PI / 180.0);
	robot_state->sensor_values[i] = 
	robot_specifications->max_sensors_range
	* 0.5 * (sin(((float) i) * 0.3) + 1.0); 
      robot_state->sensor_endpoint_x[i] = 0.0;
      robot_state->sensor_endpoint_y[i] = 0.0;
    }
    
    
    /* INITIALIZE MAPS */
    for (i = 0; i < local_size; i++){
      local_map[i]     = 0.5;
      local_robot[i]   = 0;
      local_active[i]  = 0;
    }
  }
  else 
    printf("ERROR: Maps were already allocated. Ignored.\n");
}









void
allocate_stack_item(ROBOT_STATE_PTR    robot_state,
		    PROGRAM_STATE_PTR  program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications,
		    int item_number)
{
  int j;

  if (item_number < 0 || item_number >= robot_specifications->stack_size)
    return;
  
  if (program_state->stack[item_number].sensor_values == NULL){
    program_state->stack[item_number].sensor_values = 
      (float *) malloc(sizeof(float) * robot_specifications->num_sensors);
    if (program_state->stack[item_number].sensor_values == NULL){
      fprintf(stderr, "ERROR: Out of memory. Must abort.\n");
      exit(-1);
    }
  }

  if (program_state->stack[item_number].error == NULL){
    program_state->stack[item_number].error = 
      (float **) (malloc(sizeof(float *)
		       * robot_specifications->pos_map_dim));
    if (program_state->stack[item_number].error == NULL){
      fprintf(stderr, "ERROR: Out of memory. Must abort.\n");
      exit(-1);
    }
    for (j = 0; j < robot_specifications->pos_map_dim; j++){
      program_state->stack[item_number].error[j] = 
	(float *) (malloc(sizeof(float) 
			  * robot_specifications->pos_map_dim));
      if (program_state->stack[item_number].error[j] == NULL){
	fprintf(stderr, "ERROR: Out of memory.. Must abort.\n");
	exit(-1);
      }
    }
  }

  if (program_state->stack[item_number].nearest_x == NULL){
    program_state->stack[item_number].nearest_x = 
      (float **) (malloc(sizeof(float *)
		       * robot_specifications->pos_map_dim));
    if (program_state->stack[item_number].nearest_x == NULL){
      fprintf(stderr, "ERROR: Out of memory. Must abort.\n");
      exit(-1);
    }
    for (j = 0; j < robot_specifications->pos_map_dim; j++){
      program_state->stack[item_number].nearest_x[j] = 
	(float *) (malloc(sizeof(float) 
			  * robot_specifications->pos_map_dim));
      if (program_state->stack[item_number].nearest_x[j] == NULL){
	fprintf(stderr, "ERROR: Out of memory.. Must abort.\n");
	exit(-1);
      }
    }
  }

  if (program_state->stack[item_number].nearest_y == NULL){
    program_state->stack[item_number].nearest_y = 
      (float **) (malloc(sizeof(float *)
		       * robot_specifications->pos_map_dim));
    if (program_state->stack[item_number].nearest_y == NULL){
      fprintf(stderr, "ERROR: Out of memory. Must abort.\n");
      exit(-1);
    }
    for (j = 0; j < robot_specifications->pos_map_dim; j++){
      program_state->stack[item_number].nearest_y[j] = 
	(float *) (malloc(sizeof(float) 
			  * robot_specifications->pos_map_dim));
      if (program_state->stack[item_number].nearest_y[j] == NULL){
	fprintf(stderr, "ERROR: Out of memory.. Must abort.\n");
	exit(-1);
      }
    }
  }

  if (program_state->stack[item_number].nearest_index == NULL){
    program_state->stack[item_number].nearest_index = 
      (int **) (malloc(sizeof(int *)
		     * robot_specifications->pos_map_dim));
    if (program_state->stack[item_number].nearest_index == NULL){
      fprintf(stderr, "ERROR: Out of memory. Must abort.\n");
      exit(-1);
    }
    for (j = 0; j < robot_specifications->pos_map_dim; j++){
      program_state->stack[item_number].nearest_index[j] = 
	(int *) (malloc(sizeof(int) 
			* robot_specifications->pos_map_dim));
      if (program_state->stack[item_number].nearest_index[j] == NULL){
	fprintf(stderr, "ERROR: Out of memory.. Must abort.\n");
	exit(-1);
      }
    }
  }

  if (program_state->stack[item_number].admissble_sensor_value == NULL){
    program_state->stack[item_number].admissble_sensor_value = 
      (char *) (malloc(sizeof(char)
		     * robot_specifications->num_sensors));
    if (program_state->stack[item_number].admissble_sensor_value == NULL){
      fprintf(stderr, "ERROR: Out of memory. Must abort.\n");
      exit(-1);
    }
  }

  if (program_state->stack[item_number].sensor_x == NULL){
    program_state->stack[item_number].sensor_x = 
      (float *) (malloc(sizeof(float)
		      * robot_specifications->num_sensors));
    if (program_state->stack[item_number].sensor_x == NULL){
      fprintf(stderr, "ERROR: Out of memory. Must abort.\n");
      exit(-1);
    }
  }

  if (program_state->stack[item_number].sensor_y == NULL){
    program_state->stack[item_number].sensor_y = 
      (float *) (malloc(sizeof(float)
		      * robot_specifications->num_sensors));
    if (program_state->stack[item_number].sensor_y == NULL){
      fprintf(stderr, "ERROR: Out of memory. Must abort.\n");
      exit(-1);
    }
  }

  if (program_state->stack[item_number].sensor_z == NULL){
    program_state->stack[item_number].sensor_z = 
      (float *) (malloc(sizeof(float)
		      * robot_specifications->num_sensors));
    if (program_state->stack[item_number].sensor_z == NULL){
      fprintf(stderr, "ERROR: Out of memory. Must abort.\n");
      exit(-1);
    }
  }

  if (program_state->stack[item_number].sensor_angle_to_base == NULL){
    program_state->stack[item_number].sensor_angle_to_base = 
      (float *) (malloc(sizeof(float)
		      * robot_specifications->num_sensors));
    if (program_state->stack[item_number].sensor_angle_to_base == NULL){
      fprintf(stderr, "ERROR: Out of memory. Must abort.\n");
      exit(-1);
    }
  }

  if (program_state->stack[item_number].sin_sensor_angle_to_base == NULL){
    program_state->stack[item_number].sin_sensor_angle_to_base = 
      (float *) (malloc(sizeof(float)
		      * robot_specifications->num_sensors));
    if (program_state->stack[item_number].sin_sensor_angle_to_base == NULL){
      fprintf(stderr, "ERROR: Out of memory. Must abort.\n");
      exit(-1);
    }
  }

  if (program_state->stack[item_number].cos_sensor_angle_to_base == NULL){
    program_state->stack[item_number].cos_sensor_angle_to_base = 
      (float *) (malloc(sizeof(float)
		      * robot_specifications->num_sensors));
    if (program_state->stack[item_number].cos_sensor_angle_to_base == NULL){
      fprintf(stderr, "ERROR: Out of memory. Must abort.\n");
      exit(-1);
    }
  }

  if (program_state->stack[item_number].dist_to_base == NULL){
    program_state->stack[item_number].dist_to_base = 
      (float *) (malloc(sizeof(float)
		      * robot_specifications->num_sensors));
    if (program_state->stack[item_number].dist_to_base == NULL){
      fprintf(stderr, "ERROR: Out of memory. Must abort.\n");
      exit(-1);
    }
  }
}



/************************************************************************
 *
 *   NAME:         read_init_file()
 *                 
 *   FUNCTION:     reads the initialization file. If not found,
 *                 the default values are kept
 *                 
 *   PARAMETERS:   ROBOT_STATE_PTR    robot_state    Pointer to robot state
 *                                                   description structure
 *                 PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void read_init_file(ROBOT_STATE_PTR    robot_state,
		    PROGRAM_STATE_PTR  program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  if (load_parameters(init_file_name, 1))
    fprintf(stderr, "Initialization file %s successfully read.\n",
	    init_file_name);    
  else
    exit(-1);
}



/************************************************************************
 *
 *   NAME:         clear_all_maps()
 *                 
 *   FUNCTION:     Clears all the maps, and also clears path-memory
 *                 
 *   PARAMETERS:   ROBOT_STATE_PTR    robot_state    Pointer to robot state
 *                                                   description structure
 *                 PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void clear_all_maps(ROBOT_STATE_PTR    robot_state,
		    PROGRAM_STATE_PTR  program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int x, y, index;
  
  for (x = 0; x < robot_specifications->local_map_dim_x; x++)
    for (y = 0; y < robot_specifications->local_map_dim_y; y++){
      index = x * robot_specifications->local_map_dim_y + y;
      if (local_map != NULL)
	local_map[index] = 0.5;
      if (local_active != NULL)
	local_active[index] = 0;
      if (local_robot != NULL)
	local_robot[index] = 0;
    }
}






/************************************************************************
 *
 *   NAME:         init_network
 *                 
 *   FUNCTION:     Initializes and creates network.
 *                 
 *   PARAMETERS:   NEURAL_NETWORK_PTR neural_network   pointer to network
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

#define HS_NUM_INPUTS  6
#define HS_NUM_OUTPUTS 1

#define new

void init_network(NEURAL_NETWORK_PTR neural_network)
{
  int i, j, ii;
#ifdef old
#define HS_NUM_HIDDEN  8
  static float hs_hidden_weights[HS_NUM_HIDDEN][HS_NUM_INPUTS] =
    {{-2.05827,9.81856,5.19281,-1.38505,-10.012,-0.617538},
       {-1.10835,2.52366,1.93989,-1.4488,-6.27143,-0.950382},
       {-1.78398,31.7569,4.81413,0.475091,-12.2744,-1.92544},
       {-1.2781,7.0219,4.34878,-1.52254,-9.0912,-0.891812},
       {0.144838,13.6954,10.7992,0.195507,-24.1377,-0.0327454},
       {-2.44775,11.7664,23.3102,-0.867641,-29.2833,3.60979},
       {-0.592681,17.2338,-0.158817,1.66644,-20.435,-3.8556},
       {-0.944728,7.98625,2.1898,4.92418,-18.9219,-7.10565}};
  static float hs_output_weights[HS_NUM_HIDDEN] =
    {2.88664,0.254665,16.4488,2.03083,-6.97433,7.76537,21.5508,-14.1887};
  static float hs_biases[HS_NUM_HIDDEN+HS_NUM_OUTPUTS] =
    {-9.54385,-5.98636,-22.2858,-8.56083,1.58459,-4.63209,
       -0.180325,0.351602,0.435347};

#endif

#ifdef old
/* 29. Erster Run ueber nacht, erster run mit Winkel-noise, low noise params */
#define HS_NUM_HIDDEN  10
  static float hs_hidden_weights[HS_NUM_HIDDEN][HS_NUM_INPUTS] =
    {{0.451211, 0.565258, 19.1794, 0.435424, -18.9606, 1.83088},
       {0.974816, -0.176986, 2.11199, 1.16313, 13.5604, -0.0819151},
       {-0.252529, 8.33473, 2.12479, -16.7232, 7.75783, 1.27509},
       {4.70693, 1.01073, -0.100825, 6.08254, -8.39754, -0.511258},
       {-0.808649, 32.8859, 1.24902, -0.201688, -31.834, -0.581324},
       {-1.03353, 26.6343, 0.296703, 0.0765959, -23.6871, 1.01494},
       {0.819786, -0.260834, -0.0961089, 1.54047, -21.2746, 0.293654},
       {-0.225093, 3.44549, 0.257066, 1.26886, -1.70415, -0.601072},
       {0.468051, 0.151056, 29.301, -1.09633, -27.3064, 0.495083},
       {1.52523, 2.30916, 2.83315, 2.71737, -2.49694, -2.56493}};
  
  static float hs_output_weights[HS_NUM_HIDDEN] =
    {-10.8154, 9.63425, -1.98927, 3.03774, -14.0025, 16.6945, 
       11.2689, -11.1621, 10.7734, 2.34629};
  
  static float hs_biases[HS_NUM_HIDDEN+HS_NUM_OUTPUTS] =
    {-1.56704, -18.7414, -7.29566, -5.75324, 1.68459, -1.02394,
       -1.35751, -4.25297, -1.64473, -6.65456, 0.857793};
#endif

#ifdef new
#define HS_NUM_HIDDEN  10
  static float hs_hidden_weights[HS_NUM_HIDDEN][HS_NUM_INPUTS] =
    {{4.9788, 1.62944, -0.341989, -4.27919, -8.5487, 0.728123},
       {1.41565, 2.34086, 16.2411, 4.46966, 2.36868, 1.60035},
       {-0.0623119, 7.63422, 3.34079, -0.29597, -2.6555, 0.309042},
       {0.151993, 17.336, 14.9557, -3.26972, -10.2095, -0.670753},
       {-0.564835, 24.5744, -0.776519, 0.363162, -24.3041, -0.692061},
       {-1.26829, 27.5586, -0.163926, 0.820662, -24.242, -0.218101},
       {2.49868, -0.704481, 0.130852, -0.235837, -12.9077, 0.495915},
       {0.168114, 0.126811, 0.168029, -0.0288237, -2.97025, -0.459604},
       {2.54263, 4.73341, 4.63776, 0.968635, -6.87045, 0.89195},
       {0.680483, 0.5075, 0.593876, 0.478556, -3.62527, -0.521582}};
  static float hs_output_weights[HS_NUM_HIDDEN] =
    {-10.8793, 11.959, -17.662, 6.87394, -13.0402, 15.8301, 
       14.7835, 0.0916997, 6.85235, 0.128403};
  static float hs_biases[HS_NUM_HIDDEN+HS_NUM_OUTPUTS] =
    {-4.65525, -28.897, -8.47242, -18.7972, 1.97622, -1.65051, 
       -1.0335, -4.45921, -8.44385, -5.37972, 0.533241};
#endif

  
  
  neural_network->network_size[0] = HS_NUM_INPUTS; /* number input units */
  neural_network->network_size[1] = HS_NUM_HIDDEN; /* number hidden, layer 1 */
  neural_network->network_size[2] = 0;             /* number hidden, layer 2 */
  neural_network->network_size[3] = HS_NUM_OUTPUTS; /* number output units */
  
  
  neural_network->net = create_network(neural_network->network_size[0],
					neural_network->network_size[1],
					neural_network->network_size[2],
					neural_network->network_size[3],
					0, 0, 0);


  for (i = neural_network->net->first_hidden1, ii = 0;
       i < neural_network->net->first_output; i++, ii++)
    for (j = 0; j < neural_network->net->first_hidden1; j++)
      neural_network->net->w[i][j] = hs_hidden_weights[ii][j];
  
  for (i = neural_network->net->first_hidden1, ii = 0;
       i < neural_network->net->first_output; i++, ii++)
    neural_network->net->w[neural_network->net->first_output][i] =
      hs_output_weights[ii];

 
  for (i = neural_network->net->first_hidden1, ii = 0;
       i < neural_network->net->nunits; i++, ii++)
    neural_network->net->bias[i] = hs_biases[ii];
 


  
/*
  if (!read_weights(neural_network->net, "hs.wts"))
    printf("WARNING: error while reading weights file %s.\n", "hs.wts");

*/    
}
