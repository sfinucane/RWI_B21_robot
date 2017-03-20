
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/map/init.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:40:22 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: init.c,v $
 * Revision 1.1  2002/09/14 15:40:22  rstone
 * *** empty log message ***
 *
 * Revision 1.18  2000/01/11 12:44:04  schneid1
 * disabled the BEEP in map-graphics.c, handlers.c and init.c if flag FGAN is set in Makefile
 *
 * Revision 1.17  1999/11/09 03:30:56  thrun
 * ?
 *
 * Revision 1.16  1999/10/15 00:29:33  thrun
 * improved multi-robot handling
 *
 * Revision 1.15  1999/09/28 21:49:57  thrun
 * Multi-robot support in various modules.
 * In particular, I had to introduce a new message for MAP that
 * communicates back robot positions and correction parameters.
 * This might make it necessary to modify existing modules that use
 * MAP - I tried to do this for all I am aware of, but I might have missed some.
 *
 * Revision 1.14  1999/08/25 10:05:55  schneid1
 * Some minor changes to the init procedures.
 *
 * Revision 1.13  1998/11/19 03:14:19  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.12  1997/08/12 03:06:06  thrun
 * intermediate version, provides some logging of sensor scans and partial
 * maps within map.
 *
 * Revision 1.11  1997/05/28 15:10:15  thrun
 * variable window size
 *
 * Revision 1.10  1997/05/28 12:37:05  thrun
 * minor changes.
 *
 * Revision 1.9  1997/05/27 09:55:29  thrun
 * Fixed a problem that arose in the communication with the planner.
 *
 * Revision 1.8  1997/05/27 07:57:30  thrun
 * new message: MAP_clear_all_sensor_maps empties all acquired maps.
 *
 * Revision 1.7  1997/05/26 14:32:50  thrun
 * .
 *
 * Revision 1.6  1997/05/25 12:39:04  thrun
 * .
 *
 * Revision 1.5  1997/05/10 18:26:42  thrun
 * Generates a .gif file in regular time intervals. Needed for Museum
 * Tourguide.
 *
 * Revision 1.4  1997/04/28 17:17:50  thrun
 * Map now suspends position tracking when LOCALIZE is up. Instead,
 * it uses LOCALIZE's correction parameters and broadcasts those.
 * Dieter, I didn't have a chance to test this!
 *
 * Revision 1.3  1997/03/28 03:48:30  tyson
 * finding .ini files and minor stuff
 *
 * Revision 1.2  1997/02/02 22:32:39  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.1.1.1  1996/09/22 16:46:29  rhino
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
#include "EZX11.h"
#include "o-graphics.h"
#include "MAP-messages.h"
#include "SONAR-messages.h"
#include "BASE-messages.h"
#include "MAP.h"


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

char init_file_name[256];
extern int read_map_dat;

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
  char* robotName = NULL;
  
  program_state->use_graphics             = 1; /* default */
  program_state->use_tcx                  = 1; /* default */
  robot_specifications->broadcasts_on     = 1; /* default */

  strcpy(init_file_name, MAP_INIT_NAME);
  
  
  for (i = 1; i < argc && !bug; i++){
    if (!strcmp(argv[i], "-nodisplay") || !strcmp(argv[i], "-nd"))
      program_state->use_graphics = 0;
    else if (!strcmp(argv[i], "source"))
      ;				/* needed for this stupid gdb... */
    else if (!strcmp(argv[i], "-notcx") || !strcmp(argv[i], "-nt"))
      program_state->use_tcx = 0;
    else if (!strcmp(argv[i], "-nobroadcasts") || !strcmp(argv[i], "-nb")){
      robot_specifications->broadcasts_on = 0;
      fprintf(stderr, "ALL BROADCASTS ARE SWITCHED OFF.\n");
    }
    else if (!strcmp(argv[i], "-file") || !strcmp(argv[i], "-file")){
      read_map_dat = 1;
      if (i < argc - 1){
	strcpy(init_file_name, argv[++i]);
	if (init_file_name[0] == '-')
	  bug = 2;
      }
      else
	bug = 2;
    }
    else if (!strcmp(argv[i], "-robot")){
      if (i < argc - 1){
	robotName = argv[++i];
	if (robotName[0] == '-')
	  bug = 3;
      }
      else
	bug = 3;
    }
    else
      bug = 1;
  }
  if (bug == 1)
    fprintf(stderr, "\nUsage: '%s [-nodisplay] [-notcx] [-nobroadcast] [-file <filename>] [-robot <robotname>]\n", 
	    argv[0]);
  else if (bug == 2)
    fprintf(stderr, "\nError: Option -file must be followed by file name.\n");
  else if (bug == 2)
    fprintf(stderr, "\nError: Option -robot must be followed by robot name.\n");
  
  
  if (bug >= 1)
    exit(1);

  if ( robotName != NULL) {
    tcxSetModuleNameExtension( robotName);
  }
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
		  ALL_PTR                  all)
{
  int i;
  if (program_state != NULL){
    program_state->tcx_initialized                         = 0;
    program_state->graphics_initialized                    = 0;
    /* program_state->use_graphics           = 1; command line option */
    /* program_state->use_tcx                = 1; command line option */
    program_state->tcx_base_connected                      = 0;
    program_state->tcx_localize_connected                  = 0;
    program_state->regular_global_map_display              = 0;
    program_state->regular_local_map_display               = 0;
    program_state->maps_allocated                          = 0;
    program_state->quit                                    = 0;
    program_state->map_update_pending                      = 0;
    program_state->actual_map                              = 0;
    program_state->global_map_matching_mode                = 0;
    program_state->map_update_on                           = 1;
    program_state->force_map_update                        = 0;
    program_state->actual_topology_option                  = 0;
    program_state->first_base_report_received              = 0;
    program_state->editing_mode                            = 0;
  }


  if (robot_state != NULL){
    robot_state->x                                         = 0.0;
    robot_state->y                                         = 0.0;
    robot_state->orientation                               = 0.0;
    robot_state->translational_speed                       = 0.0;
    robot_state->rotational_speed                          = 0.0;
    robot_state->known                                     = 0;
    robot_state->sensor_x                                  = 0.0;
    robot_state->sensor_y                                  = 0.0;
    robot_state->sensor_orientation                        = 0.0;
    robot_state->sensor_translational_speed                = 0.0;
    robot_state->sensor_rotational_speed                   = 0.0;
    robot_state->sensor_org_x                              = 0.0;
    robot_state->sensor_org_y                              = 0.0;
    robot_state->sensor_org_orientation                    = 0.0;
    robot_state->sensor_best_x                             = 0.0;
    robot_state->sensor_best_y                             = 0.0;
    robot_state->sensor_best_orientation                   = 0.0;
    robot_state->sensor_best_fit                           = 1.0;
    robot_state->sensor_values                             = NULL;

    robot_state->last_change_x                             = 0.0;
    robot_state->last_change_y                             = 0.0;
    robot_state->last_change_orientation                   = 0.0;


    robot_state->correction_parameter_x                    = 0.0;
    robot_state->correction_parameter_y                    = 0.0;
    robot_state->correction_parameter_angle                = 0.0;
    robot_state->correction_type                           = 0;

    robot_state->map_orientation_defined              = 0;
    robot_state->map_orientation                      = 0.0;
      
    robot_state->sensor_uncertainty                        = 0.001;

    robot_state->automatch_on                              = 0;
    robot_state->automatch_pos_x                           = 0.0;
    robot_state->automatch_pos_y                           = 0.0;
    robot_state->automatch_pos_orientation                 = 0.0;
    robot_state->automatch_cumul_distance                  = 0.0;
  }


  if (robot_specifications != NULL){
    robot_specifications->global_mapsize_x                 = 3000.0;
    robot_specifications->global_mapsize_y                 = 3000.0;
    robot_specifications->global_map_dim_x                 = 300;
    robot_specifications->global_map_dim_y                 = 300;
    robot_specifications->local_map_dim_x                  = 2;
    robot_specifications->local_map_dim_y                  = 3;
    robot_specifications->resolution                       = 10.0;

    robot_specifications->local_map_origin_x               = 0.0;
    robot_specifications->local_map_origin_y               = 0.0;
    robot_specifications->local_map_origin_orientation     = 0.0;
    robot_specifications->smooth_radius                    = 3;

    robot_specifications->robot_size                       = 30.0;
    robot_specifications->drift                            = 0.0;
  
    robot_specifications->autoshift                        = 1;
    robot_specifications->autoshift_safety_margin          = 400.0;
    robot_specifications->autoshift_distance               = 1000.0;
    robot_specifications->autoshifted_x                    = 0.0;
    robot_specifications->autoshifted_y                    = 0.0;
    robot_specifications->autoshifted_int_x                = 0;
    robot_specifications->autoshifted_int_y                = 0;
  

    robot_specifications->do_position_correction           = 1;
    robot_specifications->max_distance_in_match            = 100.0;
    robot_specifications->map_fit_norm_L2                  = 1;
    robot_specifications->prev_pos_norm_L2                 = 1;
    robot_specifications->max_niterations_in_search        = 200;
    robot_specifications->niterations_in_map_fitting       = 50;
    robot_specifications->niterations_in_search            = 0;
    robot_specifications->max_translation_in_search        = 100.0;
    robot_specifications->translation_weight_fit           = 10000.0;
    robot_specifications->translation_weight_fit_global_match = 10000.0;
    robot_specifications->translation_weight_prev_position = 1.0;
    robot_specifications->translation_stepsize             = 0.01;
    robot_specifications->translation_momentum             = 0.8;
    robot_specifications->max_rotation_in_search           = 100.0;
    robot_specifications->rotation_weight_fit              = 2000.0;
    robot_specifications->rotation_weight_fit_global_match = 2000.0;
    robot_specifications->rotation_weight_prev_position    = 1.0;
    robot_specifications->rotation_stepsize                = 0.03;
    robot_specifications->rotation_momentum                = 0.7;
    robot_specifications->search_granularity               = 2;

    robot_specifications->do_path_fitting                  = 1;
    robot_specifications->weight_path_fit                  = 1.0;
    robot_specifications->n_path_points_in_fit             = 100;

    robot_specifications->wall_error_threshold             = 10.0;
    robot_specifications->wall_weight                      = 0.1;
    robot_specifications->number_subsequent_adjacent_walls = 5;
    robot_specifications->min_advance_for_map_fitting      = 400.0;

    robot_specifications->decay_old                        = 0.99;
    robot_specifications->decay_new                        = 0.5;
    robot_specifications->prior                            = 0.5;
    robot_specifications->update_extreme_likelihoods       = 0;

    robot_specifications->lower_clipping_value             = 0.05;
    robot_specifications->upper_clipping_value             = 0.95;

    robot_specifications->reposition_robot_initially       = 1;
    robot_specifications->regular_gif_output_in_sec        = -1;
    /* robot_specifications->broadcasts_on                 = 1; */
    robot_specifications->X_window_size                    = 70.0;
    robot_specifications->data_logging                     = 0;

    for (i = 0; i != NUM_GLOBAL_MAPS; i++){
      robot_specifications->min_display_index_x[i]  = 
	robot_specifications->global_map_dim_x + 1;
      robot_specifications->max_display_index_x[i]  =  -1;
      robot_specifications->min_display_index_y[i]  = 
	robot_specifications->global_map_dim_x + 1;
      robot_specifications->max_display_index_y[i]  =  -1;
    }
  }


  if (all != NULL){
    all->program_state        = program_state;
    all->robot_state          = robot_state;
    all->robot_specifications = robot_specifications;
  }

  n_path_entries = 0;
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
  int i, j, global_size, local_size, out_of_mem;


  if (!program_state->maps_allocated){
    
    
    /* IF MAPS ALREADY ALLOCATED - FREE THE MEMORY (currently commented out)
     * if (program_state->maps_allocated){
     *   free(global_map);
     *   free(global_active);
     *   free(local_map);
     *   free(local_active);
     *   free(global_map_ext); 
     *   program_state->maps_allocated = 0;
     * } 
     */
    
    robot_specifications->global_map_dim_x = 
      (int) (robot_specifications->global_mapsize_x
	     / robot_specifications->resolution);
    robot_specifications->global_map_dim_y = 
      (int) (robot_specifications->global_mapsize_y
	     / robot_specifications->resolution);
    
    /* ALLOCATE NEW MEMORY FOR THE MAPS */
    
    global_size = robot_specifications->global_map_dim_x
      * robot_specifications->global_map_dim_y;

    local_size = robot_specifications->local_map_dim_x
      * robot_specifications->local_map_dim_y;

    if (!program_state->maps_allocated){
      out_of_mem = 0;
      for (i = 0;i != NUM_GLOBAL_MAPS; i++){
	global_map_x[i]     = (float *) (calloc(global_size,sizeof(float)));
	global_active_x[i]  = (int *)   (calloc(global_size,sizeof(int)));
	global_label_x[i]   = (unsigned char *)
	  (calloc(global_size,sizeof(unsigned char)));
	if (global_map_x[i] == NULL || global_active_x[i] == NULL
	    || global_label_x[i] == NULL)
	  out_of_mem = 1;
      }
      global_map    = global_map_x[0];
      global_active = global_active_x[0];
      global_label  = global_label_x[0];

      global_local_correlations  = (float *) (calloc(global_size,sizeof(float)));
      global_voronoi  = (float *) (calloc(global_size,sizeof(float)));
      global_voronoi_active  = (int *) (calloc(global_size,sizeof(int)));

      local_map     = (float *) (calloc(global_size,sizeof(float)));
      local_smooth_map  = (float *) (calloc(global_size,sizeof(float)));
      local_active  = (int *)   (calloc(global_size,sizeof(int)));
      
      if (out_of_mem ||
	  global_map                 == NULL || 
	  global_active              == NULL ||
	  global_label               == NULL ||
	  local_map                  == NULL ||
	  local_active               == NULL ||
	  local_smooth_map           == NULL ||
	  global_local_correlations  == NULL){

	printf("ABORT: out of memory!\n");
	exit(1);
      }
      program_state->maps_allocated = 1;
    }
    
    /* INITIALIZE MAPS */
    
    for (i = 0; i != global_size; i++){
      for (j = 0; j != NUM_GLOBAL_MAPS; j++){
	global_map_x[j][i]     = robot_specifications->prior;
	/* Since we use calloc we do NOT need the following lines */
	//	global_active_x[j][i]  = 0;
	//	global_label_x[j][i]   = (unsigned char) 0;
      }
      //      global_local_correlations[i] = 0.0;
    }

    for (i = 0; i != local_size; i++){
      local_map[i]                = 0.8;
      local_smooth_map[i]         = 0.8;
      //      local_active[i]             = 0;
    }
    

    frozen = (int *) calloc(4, sizeof(int));
    frozen[0] = 0;
    frozen[1] = 1;
    frozen[2] = 0;
    frozen[3] = 0;		/* see MAP.c for a definition of frozen */
    

  }
  else 
    printf("ERROR: Maps were already allocated. Ignored.\n");
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
  if (!load_parameters(init_file_name, 1)){
#ifndef FGAN
    putc(7,stderr);
#endif
    usleep(800000);
#ifndef FGAN
    putc(7,stderr);
#endif
    fprintf(stderr, "WARNING: Initialization file %s not found.",
	    init_file_name);
    fprintf(stderr, " Take default values.\n");
  }
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

void clear_maps(int number,	/* map number - -1 for all maps,
				 * -2 for all but the CAD map */
		ROBOT_STATE_PTR    robot_state,
		PROGRAM_STATE_PTR  program_state,
		ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  register int i, x, y, index, from_i, num_i;
  
  if (number == -1){
    from_i = 1;
    num_i  = NUM_GLOBAL_MAPS - 1;
  }
  else if (number == -2){
    from_i = 1;
    num_i  = NUM_GLOBAL_MAPS - 2;
  }
  else{
    from_i = number;
    num_i  = 1;
  }
  if (from_i < 0 || from_i + num_i > NUM_GLOBAL_MAPS)
    fprintf(stderr, "ERROR in bounds: %d %d\n", from_i, num_i);
  for (i = from_i; i < from_i + num_i; i++){
    for (x = 0; x != robot_specifications->global_map_dim_x; x++)
      for (y = 0; y != robot_specifications->global_map_dim_y; y++){
	index = x * robot_specifications->global_map_dim_y + y;
	if (global_map_x[i] != NULL)
	  global_map_x[i][index] = 0.5;
	if (global_active_x[i] != NULL)
	  global_active_x[i][index] = 0;
	if (global_label_x[i] != NULL)
	  global_label_x[i][index] = (unsigned char) 0;
	if (global_local_correlations != NULL)
	  global_local_correlations[index] = 0.0;
      }
    robot_specifications->min_display_index_x[i]  = 
      robot_specifications->global_map_dim_x + 1;
    robot_specifications->max_display_index_x[i]  =  -1;
    robot_specifications->min_display_index_y[i]  = 
      robot_specifications->global_map_dim_x + 1;
    robot_specifications->max_display_index_y[i]  =  -1;
  }

  program_state->force_map_update = 1;

  /* compute_map_0(robot_specifications, program_state, robot_state); */

  
  /*
     for (x = 0; x < robot_specifications->local_map_dim_x; x++)
     for (y = 0; y < robot_specifications->local_map_dim_y; y++){
     index = x * robot_specifications->local_map_dim_y + y;
     if (local_map != NULL)
     local_map[index] = 0.5;
     if (local_smooth_map != NULL)
     local_smooth_map[index] = 0.5;
     if (local_active != NULL)
     local_active[index] = 0;
     }
     */

}  
void clear_path(ROBOT_STATE_PTR    robot_state,
		PROGRAM_STATE_PTR  program_state,
		ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int i;

  n_path_entries = 0;
  if (program_state->graphics_initialized)
    for (i = 0; i < MAX_NUM_ROBOTS; i++)
      G_clear_markers(PATH[i]);
}


