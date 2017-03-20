
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/controller/file.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 16:53:36 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: file.c,v $
 * Revision 1.1  2002/09/14 16:53:36  rstone
 * *** empty log message ***
 *
 * Revision 1.5  1998/01/25 01:07:49  thrun
 * tours: timeout. variable display update rate.
 *
 * Revision 1.4  1997/03/25 21:44:44  tyson
 * Many bug fixes.
 *
 * Revision 1.3  1997/02/02 22:32:32  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.2  1996/11/27 23:20:22  thrun
 * (a) Modifications of Tyson's Makefile: they now work under Solaris again
 * (b) Major modifications of the CONTROLLER module.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:26  rhino
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






#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include "all.h"



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


int save_parameters(char *filename, ALL_PARAMS)
{
  int i, j, x, y, index;
  FILE *iop;
  char filename2[256];

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


  fprintf(iop, "robot_specifications->robot_size                       %f\n",
	  robot_specifications->robot_size);
  fprintf(iop, "robot_specifications->global_worldsize_x               %f\n",
	  robot_specifications->global_worldsize_x);
  fprintf(iop, "robot_specifications->global_worldsize_y               %f\n",
	  robot_specifications->global_worldsize_y);
  fprintf(iop, "robot_specifications->map_resolution                   %f\n",
	  robot_specifications->map_resolution);
  fprintf(iop, "robot_specifications->refresh_interval                 %d\n",
	  (int) robot_specifications->refresh_interval);
  fprintf(iop, "robot_specifications->translational_speed              %f\n",
	  robot_specifications->translational_speed);
  fprintf(iop, "robot_specifications->rotational_speed                 %f\n",
	  robot_specifications->rotational_speed);
#ifdef UNIBONN
  fprintf(iop, "robot_specifications->num_sonar_sensors                %d\n",
	  robot_specifications->num_sonar_sensors);
  fprintf(iop, "robot_specifications->first_sonar_angle                %f\n",
	  robot_specifications->first_sonar_angle);
  fprintf(iop, "robot_specifications->last_sonar_angle                 %f\n",
	  robot_specifications->last_sonar_angle);
#endif /*UNIBONN*/
  fprintf(iop, "robot_specifications->min_sonar_range                  %f\n",
	  robot_specifications->min_sonar_range);
  fprintf(iop, "robot_specifications->max_sonar_range                  %f\n",
	  robot_specifications->max_sonar_range);
  fprintf(iop, "robot_specifications->num_laser_sensors                %d\n",
	  robot_specifications->num_laser_sensors);
  fprintf(iop, "robot_specifications->first_laser_angle                %f\n",
	  robot_specifications->first_laser_angle);
  fprintf(iop, "robot_specifications->last_laser_angle                 %f\n",
	  robot_specifications->last_laser_angle);
  fprintf(iop, "robot_specifications->min_laser_range                  %f\n",
	  robot_specifications->min_laser_range);
  fprintf(iop, "robot_specifications->max_laser_range                  %f\n",
	  robot_specifications->max_laser_range);
  fprintf(iop, "robot_specifications->autoshift                        %d\n",
	  robot_specifications->autoshift);
  fprintf(iop, "robot_specifications->safety_margin                    %f\n",
	  robot_specifications->safety_margin);
  fprintf(iop, "robot_specifications->autoshift_distance               %f\n",
	  robot_specifications->autoshift_distance);
  fprintf(iop, "robot_specifications->statistics_time_decay_rate       %f\n",
	  robot_specifications->statistics_time_decay_rate);
  fprintf(iop, "robot_specifications->dist_action_achieved             %f\n",
	  robot_specifications->dist_action_achieved);
  fprintf(iop, "robot_specifications->base_status_update_rate          %d\n",
	  robot_specifications->base_status_update_rate);
  fprintf(iop, "robot_specifications->X_window_size                    %f\n",
	  robot_specifications->X_window_size);
#ifdef UNIBONN
  fprintf(iop, "robot_specifications->min_dist_between_objects         %f\n",
	  robot_specifications->min_dist_between_objects);
  fprintf(iop, "robot_specifications->camera_1_opening_angle           %f\n",
	  robot_specifications->camera_1_opening_angle);
  fprintf(iop, "robot_specifications->camera_1_min_perceptual_dist     %f\n",
	  robot_specifications->camera_1_min_perceptual_dist);
  fprintf(iop, "robot_specifications->camera_1_max_perceptual_dist     %f\n",
	  robot_specifications->camera_1_max_perceptual_dist);
  fprintf(iop, "robot_specifications->camera_2_opening_angle           %f\n",
	  robot_specifications->camera_2_opening_angle);
  fprintf(iop, "robot_specifications->camera_2_min_perceptual_dist     %f\n",
	  robot_specifications->camera_2_min_perceptual_dist);
  fprintf(iop, "robot_specifications->camera_2_max_perceptual_dist     %f\n",
	  robot_specifications->camera_2_max_perceptual_dist);
  fprintf(iop, "robot_specifications->max_num_allowed_failures_in_object_recognition %d\n",
	  robot_specifications->max_num_allowed_failures_in_object_recognition);
  fprintf(iop, "robot_specifications->approach_object_distance         %f\n",
	  robot_specifications->approach_object_distance);
  fprintf(iop, "robot_specifications->set_points_when_searching_objects %d\n",
	  robot_specifications->set_points_when_searching_objects);
  fprintf(iop, "robot_specifications->set_points_when_searching_bins   %d\n",
	  robot_specifications->set_points_when_searching_bins);


#endif /* UNIBONN */
  fprintf(iop, "robot_specifications->map_update_interval              %f\n",
	  robot_specifications->map_update_interval);
  fprintf(iop, "robot_specifications->sensor_update_interval           %f\n",
	  robot_specifications->sensor_update_interval);
  fprintf(iop, "program_state->map_regular_update_on            %d\n",
	  program_state->map_regular_update_on);
  fprintf(iop, "robot_specifications->display_update_interval          %f\n",
	  robot_specifications->display_update_interval);

  

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


int load_parameters(char *filename, ALL_PARAMS)
{
  int i, x, y, index, int_value;
  float float_value;
  FILE *iop;
  int file_ended, error;
  char command[256];
  char filename2[256];
  char *cmd_ptr;
  char *filename3;

  sprintf(filename2, "etc/%s", filename);

  filename3 = bFindFileM(filename2);
  
  if ((iop = fopen(filename3, "r")) == 0){
    fprintf(stderr, "Could not open input file %s. File not loaded.\n",
	    filename3);
    fprintf(stderr, "WARNING: Failed to read file %s.\n", filename);
    free(filename3);
    return 0;
  }

  free(filename3);


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



      else if (!strcmp(command, "robot_specifications->robot_size")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->robot_size = float_value;
      }
      


      else if (!strcmp(command, "robot_specifications->global_worldsize_x")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->global_worldsize_x = float_value;
      }
      


      else if (!strcmp(command, "robot_specifications->global_worldsize_y")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->global_worldsize_y = float_value;
      }
      


      else if (!strcmp(command, "robot_specifications->map_resolution")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->map_resolution = float_value;
      }
      


      else if (!strcmp(command, "robot_specifications->refresh_interval")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->refresh_interval = (long) int_value;
      }
      


      else if (!strcmp(command, "robot_specifications->translational_speed")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->translational_speed = float_value;
      }
      


      else if (!strcmp(command, "robot_specifications->rotational_speed")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->rotational_speed = float_value;
      }
      

#ifdef UNIBONN

      else if (!strcmp(command, "robot_specifications->num_sonar_sensors")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->num_sonar_sensors = int_value;
      }
      


      else if (!strcmp(command, "robot_specifications->first_sonar_angle")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->first_sonar_angle = float_value;
      }
      


      else if (!strcmp(command, "robot_specifications->last_sonar_angle")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->last_sonar_angle = float_value;
      }
      
#endif /*UNIBONN*/


      else if (!strcmp(command, "robot_specifications->min_sonar_range")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->min_sonar_range = float_value;
      }
      


      else if (!strcmp(command, "robot_specifications->max_sonar_range")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->max_sonar_range = float_value;
      }
      


      else if (!strcmp(command, "robot_specifications->num_laser_sensors")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->num_laser_sensors = int_value;
      }
      


      else if (!strcmp(command, "robot_specifications->first_laser_angle")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->first_laser_angle = float_value;
      }
      


      else if (!strcmp(command, "robot_specifications->last_laser_angle")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->last_laser_angle = float_value;
      }
      


      else if (!strcmp(command, "robot_specifications->min_laser_range")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->min_laser_range = float_value;
      }
      


      else if (!strcmp(command, "robot_specifications->max_laser_range")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->max_laser_range = float_value;
      }
      


      else if (!strcmp(command, "robot_specifications->autoshift")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->autoshift = int_value;
      }
      


      else if (!strcmp(command, "robot_specifications->safety_margin")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->safety_margin = float_value;
      }
      


      else if (!strcmp(command, "robot_specifications->autoshift_distance")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->autoshift_distance = float_value;
      }
      


      else if (!strcmp(command, 
		       "robot_specifications->statistics_time_decay_rate")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->statistics_time_decay_rate = float_value;
      }
      


#ifdef UNIBONN
      else if (!strcmp(command, 
		       "robot_specifications->min_dist_between_objects")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->min_dist_between_objects = float_value;
      }
      


      
      else if (!strcmp(command, 
		       "robot_specifications->camera_1_opening_angle")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->camera_1_opening_angle = float_value;
      }
      



      
      else if (!strcmp(command, 
		       "robot_specifications->camera_1_min_perceptual_dist")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->camera_1_min_perceptual_dist = float_value;
      }
      



      
      else if (!strcmp(command, 
		       "robot_specifications->camera_1_max_perceptual_dist")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->camera_1_max_perceptual_dist = float_value;
      }
      



      
      else if (!strcmp(command, 
		       "robot_specifications->camera_2_opening_angle")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->camera_2_opening_angle = float_value;
      }
      
      
      else if (!strcmp(command, 
		       "robot_specifications->camera_2_min_perceptual_dist")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->camera_2_min_perceptual_dist = float_value;
      }
      



      
      else if (!strcmp(command, 
		       "robot_specifications->camera_2_max_perceptual_dist")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->camera_2_max_perceptual_dist = float_value;
      }
      



      else if (!strcmp(command, "robot_specifications->max_num_allowed_failures_in_object_recognition")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->max_num_allowed_failures_in_object_recognition = int_value;
      }
      

      else if (!strcmp(command, 
		       "robot_specifications->approach_object_distance")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->approach_object_distance = float_value;
      }
      


      else if (!strcmp(command, "robot_specifications->set_points_when_searching_objects")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->set_points_when_searching_objects = int_value;
      }
      
      else if (!strcmp(command, "robot_specifications->set_points_when_searching_bins")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->set_points_when_searching_bins = int_value;
      }
#endif /* UNIBONN */

      else if (!strcmp(command, 
		       "robot_specifications->map_update_interval")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->map_update_interval = float_value;
      }
      

      else if (!strcmp(command, 
		       "robot_specifications->sensor_update_interval")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->sensor_update_interval = float_value;
      }
      
      else if (!strcmp(command, "program_state->map_regular_update_on")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else
	  program_state->map_regular_update_on = int_value;
      }
      


      else if (!strcmp(command, "robot_specifications->display_update_interval")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->display_update_interval = float_value;
      }
      

      else if (!strcmp(command, 
		       "robot_specifications->dist_action_achieved")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->dist_action_achieved = float_value;
      }
      

      
      else if (!strcmp(command, 
		       "robot_specifications->base_status_update_rate")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->base_status_update_rate = int_value;
      }
      

      else if (!strcmp(command, 
		       "robot_specifications->X_window_size")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->X_window_size = float_value;
      }
      

      
      else{
	fprintf(stderr, "ERROR: Unknown keyword \"%s\" in %s. Must exit.\n", 
		command, filename2);
	error = 1;
      }
 
      if (file_ended == 1)
	fprintf(stderr, "Surprising end of file %s.\n", filename2);
      
 

   }
  } while (!file_ended);
  fclose(iop);
  


  robot_specifications->global_map_dim_x     =
    (int) (robot_specifications->global_worldsize_x / 
	   robot_specifications->map_resolution);
  robot_specifications->global_map_dim_y     =
    (int) (robot_specifications->global_worldsize_y / 
	   robot_specifications->map_resolution);

  if (!error)
    fprintf(stderr, "File %s successfully read.\n", filename2);

  return (1 - error);
}








