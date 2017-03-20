
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/plan/file.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:47:04 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: file.c,v $
 * Revision 1.1  2002/09/14 15:47:04  rstone
 * *** empty log message ***
 *
 * Revision 1.11  1999/10/23 02:48:55  thrun
 * Various improvements at DARPA demo.
 *
 * Revision 1.10  1999/10/07 23:01:33  thrun
 * central map feature.
 *
 * Revision 1.9  1999/03/13 04:22:50  thrun
 * new parameter "robot_specifications->initial_map_file_name",
 * makes it possible to read maps from file.
 *
 * Revision 1.8  1999/02/07 20:52:12  thrun
 * inroduced subgoals, modified PLAN_action_reply to return entire plan.
 *
 * Revision 1.7  1998/09/01 13:51:22  thrun
 * .
 *
 * Revision 1.6  1998/08/22 01:55:49  thrun
 * Increased efficiency, planner can now wait for initial
 * maps and adpot their size/resolution.
 *
 * Revision 1.5  1997/05/28 15:17:08  thrun
 * variable size window
 *
 * Revision 1.4  1997/04/28 17:56:56  thrun
 * Fixed a bug that sometimes made the adjusted action come very
 * close to walls. Also introduced a new parameter that makes plan
 * generate new plans everytime a status report is received from
 * colliServer (generate_actions_continuously).
 *
 * Revision 1.3  1997/03/28 03:48:32  tyson
 * finding .ini files and minor stuff
 *
 * Revision 1.2  1997/01/08 15:54:28  fox
 * Added a message PLAN_parameter_message to set some parameters with other
 * modules.
 * IMPORTANT: maps are updated even if busy > 3.
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
#define TCX_define_variables /* this makes sure variables are installed */


#include "PLAN-messages.h"
#include "COLLI-messages.h"
#include "MAP-messages.h"
#include "BASE-messages.h"
#include "SONAR-messages.h"


#include "PLAN.h"
#include "o-graphics.h"

#include <bUtils.h>

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


int save_parameters(char *filename,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications)
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



  fprintf(iop,"robot_specifications->global_mapsize_x             %g\n",
	  robot_specifications->global_mapsize_x);
  fprintf(iop,"robot_specifications->global_mapsize_y             %g\n",
	  robot_specifications->global_mapsize_y);
  fprintf(iop,"robot_specifications->resolution                   %g\n",
	  robot_specifications->resolution);
  fprintf(iop,"robot_specifications->robot_size                   %g\n",
	  robot_specifications->robot_size);
  fprintf(iop,"robot_specifications->max_security_dist            %g\n",
	  robot_specifications->max_security_dist);
  fprintf(iop,"robot_specifications->autoshift                    %d\n",
	  robot_specifications->autoshift);
  fprintf(iop,"robot_specifications->autoshift_for_maps           %d\n",
	  robot_specifications->autoshift_for_maps);
  fprintf(iop,"robot_specifications->autoshift_for_rob_pos        %d\n",
	  robot_specifications->autoshift_for_rob_pos);
  fprintf(iop,"robot_specifications->autoshift_distance           %g\n",
	  robot_specifications->autoshift_distance);
  fprintf(iop,"robot_specifications->autoshift_safety_margin      %g\n",
	  robot_specifications->autoshift_safety_margin);
  fprintf(iop,"robot_specifications->average_value                %g\n",
	  robot_specifications->average_value);
  fprintf(iop,"robot_specifications->collision_threshold          %g\n",
	  robot_specifications->collision_threshold);
  fprintf(iop,"robot_specifications->costs_exponent               %g\n",
	  robot_specifications->costs_exponent);
  fprintf(iop,"robot_specifications->min_base                     %g\n",
	  robot_specifications->min_base);
  fprintf(iop,"robot_specifications->max_base                     %g\n",
	  robot_specifications->max_base);
  fprintf(iop,"robot_specifications->max_adjust_angle             %g\n",
	  robot_specifications->max_adjust_angle);
  fprintf(iop,"robot_specifications->max_goal_distance            %g\n",
	  robot_specifications->max_goal_distance);
  fprintf(iop,"robot_specifications->max_approach_distance        %g\n",
	  robot_specifications->max_approach_distance);
  fprintf(iop,"robot_specifications->max_final_approach_distance  %g\n",
	  robot_specifications->max_final_approach_distance);
  fprintf(iop,"robot_specifications->intermediate_point_dist      %g\n",
	  robot_specifications->intermediate_point_dist);
  fprintf(iop,"robot_specifications->map_update_frequency         %d\n",
	  robot_specifications->map_update_frequency);
  fprintf(iop,"robot_specifications->exploration_circle_size      %g\n",
	  robot_specifications->exploration_circle_size);
  fprintf(iop,"robot_specifications->min_mapvalue_along_path      %g\n",
	  robot_specifications->min_mapvalue_along_path);
  fprintf(iop,"robot_specifications->border_to_interior           %g\n",
	  robot_specifications->border_to_interior);
  fprintf(iop,"robot_specifications->exterior_factor              %g\n",
	  robot_specifications->exterior_factor);
  fprintf(iop,"robot_specifications->max_bounding_box_size        %g\n",
	  robot_specifications->max_bounding_box_size);
  fprintf(iop,"robot_specifications->fast_exploration             %d\n",
	  robot_specifications->fast_exploration);
  fprintf(iop,"robot_specifications->generate_actions_continuously %d\n",
	  robot_specifications->generate_actions_continuously);
  fprintf(iop, "robot_specifications->X_window_size               %g\n",
	  robot_specifications->X_window_size);
  fprintf(iop, "robot_specifications->map_growing_radius               %g\n",
	  robot_specifications->map_growing_radius);
  fprintf(iop, "robot_specifications->wait_for_initial_map        %d\n",
	  robot_specifications->wait_for_initial_map);
  fprintf(iop, "robot_specifications->initial_map_file_name       %s\n",
	  robot_specifications->initial_map_file_name);
  fprintf(iop, "robot_specifications->wait_for_initial_corr       %d\n",
	  robot_specifications->wait_for_initial_corr);
  fprintf(iop,"robot_specifications->use_central_map              %d\n",
	  robot_specifications->use_central_map);


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

int load_parameters( char *filename, int init,
		     ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int i, x, y, index, int_value, int_value1, int_value2;
  float float_value;
  FILE *iop;
  int file_ended, error;
  char command[256];
  char filename2[256];
  
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






      else if (!strcmp(command, "robot_specifications->global_mapsize_x")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->global_mapsize_x = float_value;
	else if (float_value != robot_specifications->global_mapsize_x){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->global_mapsize_x");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->global_mapsize_y")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->global_mapsize_y = float_value;
	else if (float_value != robot_specifications->global_mapsize_y){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->global_mapsize_y");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      



      else if (!strcmp(command, "robot_specifications->resolution")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->resolution = float_value;
	else if (float_value != robot_specifications->resolution){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->resolution");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->robot_size")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->robot_size = float_value;
	else if (float_value != robot_specifications->robot_size){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->robot_size");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->max_security_dist")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_security_dist = float_value;
	else if (float_value != robot_specifications->max_security_dist){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_security_dist");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->autoshift")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->autoshift = int_value;
	else if (int_value != robot_specifications->autoshift){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->autoshift");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->autoshift_for_maps")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else 
	  robot_specifications->autoshift_for_maps = int_value;
      }
      


      else if (!strcmp(command, "robot_specifications->autoshift_for_rob_pos")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else
	  robot_specifications->autoshift_for_rob_pos = int_value;
      }
      


      else if (!strcmp(command, "robot_specifications->autoshift_distance")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->autoshift_distance = float_value;
	else if (float_value != robot_specifications->autoshift_distance){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->autoshift_distance");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->autoshift_safety_margin")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->autoshift_safety_margin = float_value;
	else if (float_value != robot_specifications->autoshift_safety_margin){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->autoshift_safety_margin");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->average_value")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->average_value = float_value;
	else if (float_value != robot_specifications->average_value){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->average_value");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      else if (!strcmp(command, "robot_specifications->collision_threshold")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->collision_threshold = float_value;
	else if (float_value != robot_specifications->collision_threshold){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->collision_threshold");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      


      
      else if (!strcmp(command, "robot_specifications->costs_exponent")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->costs_exponent = float_value;
	else if (float_value != robot_specifications->costs_exponent){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->costs_exponent");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      else if (!strcmp(command, "robot_specifications->min_base")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->min_base = float_value;
	else if (float_value != robot_specifications->min_base){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->min_base");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      else if (!strcmp(command, "robot_specifications->max_base")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_base = float_value;
	else if (float_value != robot_specifications->max_base){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_base");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      else if (!strcmp(command, "robot_specifications->max_adjust_angle")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_adjust_angle = float_value;
	else if (float_value != robot_specifications->max_adjust_angle){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_adjust_angle");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      else if (!strcmp(command, "robot_specifications->max_goal_distance")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_goal_distance = float_value;
	else if (float_value != robot_specifications->max_goal_distance){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_goal_distance");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
      
      
      else if (!strcmp(command, 
		       "robot_specifications->max_approach_distance")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_approach_distance = float_value;
	else if (float_value != robot_specifications->max_approach_distance){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_approach_distance");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
          
        
      else if (!strcmp(command,
		       "robot_specifications->max_final_approach_distance")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_final_approach_distance = float_value;
	else if (float_value !=
		 robot_specifications->max_final_approach_distance){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_final_approach_distance");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      
        
      else if (!strcmp(command,
		       "robot_specifications->intermediate_point_dist")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->intermediate_point_dist = float_value;
	else if (float_value !=
		 robot_specifications->intermediate_point_dist){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->intermediate_point_dist");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      
      else if (!strcmp(command, "robot_specifications->map_update_frequency")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->map_update_frequency = int_value;
	else if (int_value != robot_specifications->map_update_frequency){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->map_update_frequency");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        
      else if (!strcmp(command, 
		       "robot_specifications->exploration_circle_size")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->exploration_circle_size = float_value;
	else if (float_value != robot_specifications->exploration_circle_size){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->exploration_circle_size");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      else if (!strcmp(command, 
		       "robot_specifications->min_mapvalue_along_path")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->min_mapvalue_along_path = float_value;
	else if (float_value != robot_specifications->min_mapvalue_along_path){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->min_mapvalue_along_path");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      else if (!strcmp(command, 
		       "robot_specifications->border_to_interior")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->border_to_interior = float_value;
	else if (float_value != robot_specifications->border_to_interior){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->border_to_interior");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        


      else if (!strcmp(command, 
		       "robot_specifications->exterior_factor")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->exterior_factor = float_value;
	else if (float_value != robot_specifications->exterior_factor){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->exterior_factor");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        


      else if (!strcmp(command, 
		       "robot_specifications->max_bounding_box_size")){
	if (fscanf(iop, "%f", &float_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->max_bounding_box_size = float_value;
	else if (float_value != robot_specifications->max_bounding_box_size){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->max_bounding_box_size");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }

      else if (!strcmp(command, "robot_specifications->fast_exploration")){
	if (fscanf(iop, "%d", &int_value) == EOF)
	  file_ended = 1;
	else if (init)
	  robot_specifications->fast_exploration = int_value;
	else if (int_value != robot_specifications->fast_exploration){
	  fprintf(iop, "Error when parsing %s: value for ", filename2);
	  fprintf(iop, "robot_specifications->fast_exploration");
	  fprintf(iop, " does not match internal specifications.\n\n");
	  error = 1;
	}
      }
        

      else if (!strcmp(command, "robot_specifications->X_window_size")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->X_window_size = float_value;
      }

      else if (!strcmp(command, "robot_specifications->map_growing_radius")){
	if (fscanf(iop, "%g", &float_value) == EOF)
	  file_ended = 1;
	else /* if (!init) */
	  robot_specifications->map_growing_radius = float_value;
      }

      
      
      else if (!strcmp(command, 
		       "robot_specifications->generate_actions_continuously")){
	if (fscanf(iop, "%f", &int_value) == EOF)
	  file_ended = 1;
	else 
	  robot_specifications->generate_actions_continuously = int_value;
      }
      
      
      else if (!strcmp(command, 
		       "robot_specifications->use_central_map")){
	if (fscanf(iop, "%f", &int_value) == EOF)
	  file_ended = 1;
	else 
	  robot_specifications->use_central_map = int_value;
      }
      
      
      else if (!strcmp(command, 
		       "robot_specifications->wait_for_initial_map")){
	if (fscanf(iop, "%f", &int_value) == EOF)
	  file_ended = 1;
	else 
	  robot_specifications->wait_for_initial_map = int_value;
      }
      
      
      else if (!strcmp(command, 
		       "robot_specifications->initial_map_file_name")){
	if (!robot_specifications->initial_map_file_name)
	  free(robot_specifications->initial_map_file_name);
	robot_specifications->initial_map_file_name = 
	  (char *) malloc(sizeof(char) * 1024);
	if (fscanf(iop, "%s", 
		   robot_specifications->initial_map_file_name) == EOF)
	  file_ended = 1;
      }
      
      
      else if (!strcmp(command, 
		       "robot_specifications->wait_for_initial_corr")){
	if (fscanf(iop, "%f", &int_value) == EOF)
	  file_ended = 1;
	else 
	  robot_specifications->wait_for_initial_corr = int_value;
      }
      
      
      else{
	fprintf(stderr, "ERROR: Unknown keyword \"%s\" in %s. Must exit.\n ", 
		command, filename2);
	error = 1;
      }
 
      if (file_ended == 1)
	fprintf(iop, "Surprising end of file %s.\n ", filename2);
      
    }
  } while (!file_ended);
  fclose(iop);
  
  if (!error)
    fprintf(stderr, "File %s successfully read.\n", filename2);
  return (1 - error);
}


