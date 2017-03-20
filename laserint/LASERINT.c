
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/laserint/LASERINT.c,v $
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
 * $Log: LASERINT.c,v $
 * Revision 1.1  2002/09/14 15:34:06  rstone
 * *** empty log message ***
 *
 * Revision 1.23  2000/09/26 01:11:32  thrun
 * ,
 *
 * Revision 1.22  2000/03/13 00:31:11  thrun
 * first version of he texturing module. Still a bit clumpsy.
 *
 * Revision 1.21  1999/12/19 03:25:16  thrun
 * graphics for 3D object matching.
 *
 * Revision 1.20  1999/10/15 18:07:19  thrun
 * more 3D formats, more compact files.
 *
 * Revision 1.19  1999/10/14 04:38:19  thrun
 * Fixed a problem of passing on the wrong correction
 * parameters with MAP. Also, ficex a seg-fault problem with
 * laserint when rnu without display.
 *
 * Revision 1.18  1999/10/02 04:50:10  thrun
 * Minor tuning and changes. THis version built the first
 * nice map of the San Jose Tech Museum
 *
 * Revision 1.17  1999/09/28 03:31:00  thrun
 * Improved version in the server mode.
 *
 * Revision 1.16  1999/09/06 03:19:43  thrun
 * first working version of the multi-robot server architecture with
 * positon control.
 *
 * Revision 1.15  1999/09/05 21:57:21  thrun
 * initial client server robot for multi-robot mapping
 *
 * Revision 1.14  1999/09/05 17:18:50  thrun
 * 3D ceiling mapping
 *
 * Revision 1.13  1999/05/08 19:47:28  thrun
 * drastically improved version with recursive matching and samples.
 * Not quite ready yet: samples aren't used in the right way, queue
 * seems not to work, and some constants are hardcoded (phase!) But
 * impressive results!
 *
 * Revision 1.12  1999/05/04 19:47:25  thrun
 * new file "pos.c"
 *
 * Revision 1.11  1998/11/18 04:21:09  thrun
 * incorporated wall angle into estimation.
 *
 * Revision 1.10  1998/11/17 05:03:12  thrun
 * small, incremental improvements: better handling of sensor
 * displacement, and new parametr that enables laserint to ignore
 * scans when the robot is rotating too much.
 *
 * Revision 1.9  1998/11/16 01:50:39  thrun
 * positon control - works nice, includes global mapping and learning
 * the drift parameters.
 *
 * Revision 1.8  1998/11/15 16:19:16  thrun
 * initial position control method - works reasonably well. But
 * e careful with the timing, isn't an anytime algorithm yet.
 *
 * Revision 1.7  1998/11/12 14:22:54  thrun
 * ?.
 *
 * Revision 1.6  1998/08/11 22:18:24  thrun
 * PLANNER: fixed a problem with large shifts, a problem with th
 * function "pow()" under the latest Linux.
 *
 * Revision 1.5  1998/01/07 22:52:28  thrun
 * This version does NOT use neural networks. Instead, it uses
 * a geometric procedure. Difference: The obstacles are now
 * not grown by the robot diameter.
 *
 * Revision 1.4  1997/12/30 00:40:10  thrun
 * Non-neural network version of mapping.
 *
 * Revision 1.3  1997/04/27 12:27:19  thrun
 * Extended parameter set (copied from SONARINT)
 *
 * Revision 1.2  1997/02/02 22:32:35  tyson
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






#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <signal.h>
#include <math.h>
#include <sys/syslog.h>
#include "tcx.h"
#include "tcxP.h"
#include "global.h"
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

struct bParamList * bParamList = NULL;
const char *bRobotType;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

int block_wait(struct timeval *timeout, int tcx_initialized,
	       int X_initialized);

struct timeval block_waiting_time = {1, 0};

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

PROGRAM_STATE            program_state_data;
ROBOT_STATE              robot_state_data;
ROBOT_SPECIFICATIONS     robot_specifications_data;
NEURAL_NETWORK           neural_network_data;
PROGRAM_STATE_PTR        program_state        = &program_state_data;
ROBOT_STATE_PTR          robot_state          = &robot_state_data;
ROBOT_SPECIFICATIONS_PTR robot_specifications = &robot_specifications_data;
NEURAL_NETWORK_PTR       neural_network       = &neural_network_data;
ALL_TYPE                 all;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

struct timeval noWaitTime = {0, 0};


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* MAPS */


float *local_map         = NULL;
int   *local_robot       = NULL; /* grid cells under the robot */
int   *local_active      = NULL;
float *local_error       = NULL;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

struct timeval TCX_no_waiting_time = {0, 0};


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/





/************************************************************************
 *
 *   NAME:         evaluate_network
 *                 
 *   FUNCTION:     Main network interpretation routine: maps sensor
 *                 reading into a network interpretation
 *                 
 ************************************************************************/

#define N_SENSORS_INPUT 4


float evaluate_network(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		       ROBOT_STATE_PTR robot_state,
		       net_ptr netX,
		       float distance, float angle, float prediction)
{
  int i,j,k;
  int first_sensor, last_sensor;
  float angle_to_sensor_0, local_angle;
  float net_input[100];
  float net_output[100];
  float diff, result = 0.0;
  
  /* compute angle relative to r._spec.->sensor_angle[0] */
  angle_to_sensor_0 = angle - robot_specifications->first_sensor_angle;
  for (;angle_to_sensor_0 >= 360.0;) angle_to_sensor_0 -= 360.0;
  for (;angle_to_sensor_0 <    0.0;) angle_to_sensor_0 += 360.0;
  
  /* comput index of sensor left to angle */
  first_sensor = (int) (angle_to_sensor_0 * 
			robot_specifications->num_sensors / 360.0);


  /* new stuff: conventional interpretation, no neural network,
   * only good for laser */

  if (!(robot_specifications->use_neural_networks)){
    for (;first_sensor < 0;) first_sensor += robot_specifications->num_sensors;
    for (;first_sensor >= robot_specifications->num_sensors;)
      first_sensor += robot_specifications->num_sensors;
    diff = (distance * robot_specifications->neuronet_max_sensors_range)
      - robot_state->sensor_values[first_sensor];
    
    if (robot_state->sensor_values[first_sensor] < 0.0 ||
	(robot_specifications->ignore_front_laser && is_front(first_sensor)) ||
	(robot_specifications->ignore_rear_laser && is_rear(first_sensor)))
      result = -1.0;
    /* else if (fabs(diff) < robot_specifications->resolution) */
    /* result = 0.1; */
    else if (diff > 0.0)
      result = robot_specifications->network_value_mean;
    else
      result = robot_specifications->standard_free;
    return result;
  }

  /* from here on, we will use neural networks */
  
  /* compute angular position of point between nearest sensor on the
     left and nearest sensor on the right, normalized to [0,1] */
  local_angle = 
    (angle_to_sensor_0 * robot_specifications->num_sensors / 360.0)
      - ((float) first_sensor);
  
  /* compute verctor of sensors which are net-input */
  first_sensor += 1 - (N_SENSORS_INPUT/2);
  last_sensor   = first_sensor + N_SENSORS_INPUT;
  
  /* compute input to network */
  for (i = first_sensor, k = 0; i < last_sensor; i++, k++){
    j = i;
    if (j < 0) j += robot_specifications->num_sensors;
    if (j >= robot_specifications->num_sensors) 
      j -= robot_specifications->num_sensors;
    if (robot_state->sensor_values[j] < 0.0){
      return -1.0;		/* no laser available here */
    }
    net_input[k] = robot_state->sensor_values[j] 
      / robot_specifications->neuronet_max_sensors_range;
  }
  net_input[k++] = distance /* * 0.85714*/; 
  net_input[k++] = local_angle;
  net_input[k++] = prediction;	/* will be ignored for netA */
  
  /* run network to comput prediction */
  run_network(netX, 0, net_input, net_output, NULL);
  
  /* return prediction */
  return (net_output[0]);
}





/************************************************************************
 *
 *   NAME:         compute_local_map
 *                 
 *   FUNCTION:     Constructs a local map from the most recent sensor input
 *                 
 ************************************************************************/
  
	

void compute_local_map(NEURAL_NETWORK_PTR neural_network,
			   ROBOT_SPECIFICATIONS_PTR robot_specifications,
			   PROGRAM_STATE_PTR  program_state,
			   ROBOT_STATE_PTR  robot_state)
{
  int from_x, to_x, from_y, to_y, x, y, shifted_x, shifted_y, i, n;
  int local_map_index;
  float point_distance, point_x, point_y, point_angle;
  float value_prediction;
  int *sensor_end_point = NULL;
  float dx, dy, dn;
  float n_sampling;
  float sensor_value;

  /* new stuff: conventional interpretation, no neural network,
   * only good for laser */



  if (!(robot_specifications->use_neural_networks)){
    /* compute the sensor end points */
    sensor_end_point = 
      (int *) malloc(sizeof(int) 
		     * robot_specifications->local_map_dim_x
		     * robot_specifications->local_map_dim_y);
    if (!sensor_end_point){
      fprintf(stderr, "ERROR: Out of memory in compute_local_map().\n");
      exit(-1);
    }

    for (i = 0; i < robot_specifications->local_map_dim_x *
	   robot_specifications->local_map_dim_y; i++)
      sensor_end_point[i] = 0;

    n_sampling = (int) (robot_specifications->obstacle_width 
			* 3.0 / robot_specifications->resolution);
    
    for (i = 0; i < robot_specifications->num_sensors; i++){

      if ((!(robot_specifications->ignore_front_laser) || !(is_front(i))) &&
	  (!(robot_specifications->ignore_rear_laser) || !(is_rear(i)))){

	sensor_value = robot_state->sensor_values[i] + 
	  robot_specifications->sensor_increment;
	if (sensor_value >= 0.0 &&
	    sensor_value < 
	    robot_specifications->max_sensors_range){
	  if (i < (robot_specifications->num_sensors / 2)){
	    dx = robot_specifications->front_laser_offset_x;
	    dy = robot_specifications->front_laser_offset_y;
	  }
	  else{
	    dx = robot_specifications->rear_laser_offset_x;
	    dy = robot_specifications->rear_laser_offset_y;
	  }	  
	  for (n = 0; n <= n_sampling; n++){
	    dn = ((float) n) / ((float) n_sampling) * 
	      robot_specifications->obstacle_width;
	    
	    x = (int) (((cos((robot_specifications->sensor_angles[i] + 90.0)
			     * M_PI / 180.0)
			 * (sensor_value + dn)) + dy )
		       / robot_specifications->resolution);
	    y = (int) (((sin((robot_specifications->sensor_angles[i] + 90.0) 
			     * M_PI / 180.0)
			 * (sensor_value + dn)) + dx )
		       / robot_specifications->resolution);
	    
	    shifted_x = x + ((int) (robot_specifications->max_sensors_range
				    / robot_specifications->resolution));
	    shifted_y = y + ((int) (robot_specifications->max_sensors_range
				    / robot_specifications->resolution));
	    if (shifted_x >= 0 && 
		shifted_x < robot_specifications->local_map_dim_x &&
		shifted_y >= 0 && 
		shifted_y < robot_specifications->local_map_dim_y){
	      
	      local_map_index = 
		shifted_x * robot_specifications->local_map_dim_y + shifted_y;
	      
	      sensor_end_point[local_map_index] = 1;
	    }
	  }
	}
      }
    }
  } /* end of (!use_neural_networks) */


  /* superset of points to make predictions at */

  from_x = (int) (- robot_specifications->max_sensors_range
		  / robot_specifications->resolution);
  to_x   = ((int) (robot_specifications->max_sensors_range
		   / robot_specifications->resolution));
  from_y = (int) (- robot_specifications->max_sensors_range
		  / robot_specifications->resolution);
  to_y   = ((int) (robot_specifications->max_sensors_range
		   / robot_specifications->resolution));
  
  /* loop over all points */
  for (shifted_x = 0, x = from_x; x <= to_x; x++, shifted_x++){
    
    for (shifted_y = 0, y = from_y; y <= to_y; y++, shifted_y++){
      /* if point happens to be *in* the map - sanity check */
      if (shifted_x < robot_specifications->local_map_dim_x &&
	  shifted_y < robot_specifications->local_map_dim_y){
	
	local_map_index = shifted_x * robot_specifications->local_map_dim_y
	  + shifted_y;
	/* compute corresponding robot positions in cm */
	point_x = ((float) x) * robot_specifications->resolution;
	point_y = ((float) y) * robot_specifications->resolution;
	point_distance = sqrt((point_x)*(point_x)+(point_y)*(point_y));

	COMMENT("We are ignoring sensor displacement here!");
	/* if point happens within the reach of laser sensors */
	if (point_distance > robot_specifications->max_sensors_range
	    + robot_specifications->resolution){
	  local_robot[local_map_index] = 0;
	  local_active[local_map_index] = 0;
	}

	else if (point_distance < robot_specifications->robot_size){
	  local_active[local_map_index] = 1;
	  local_robot[local_map_index] = 1;
	  local_map[local_map_index] = 1.0;
	}

	else{
	  /* compute relative angle
	   * the constant 90.0 was formerly "robot_state->orientation" */
	  if (point_y == 0.0 && point_x == 0.0)
	    point_angle = - 90.0;
	  else
	    point_angle = (atan2(point_y, point_x) / M_PI * 180.0)
	      - 90.0; /* relative angle to (x,y) */

	  
	  /*================================*\
	   *========== NEURO NET ===========*
	  \*================================*/
	  
	  
	  /* 
	   * compute network predictions 
	   */

	  value_prediction 
	    = evaluate_network(robot_specifications,
			       robot_state,
			       neural_network->net, 
			       point_distance /
			       robot_specifications->neuronet_max_sensors_range, 
			       point_angle, 0.0);

	  if (!(robot_specifications->use_neural_networks))
	    if (sensor_end_point[local_map_index])
	      value_prediction = robot_specifications->standard_occupied;
	  
	  if (value_prediction < -0.5){
	    local_robot[local_map_index] = 0;
	    local_active[local_map_index] = 0;
	  }
	  
	  else{
	    /*
	     * normalize value_prediction to lie in [0,1] 
	     * so that 0.5 is at VALUE_MEAN 
	     * Also: decay value by point-distance towards 0.5.
	     */
	    
	    if (robot_specifications->decay_with_distance > 0.0){
	      if (value_prediction >= robot_specifications->network_value_mean)
		value_prediction =
		  (value_prediction - robot_specifications->network_value_mean)
		    / (1.0 - robot_specifications->network_value_mean) * 0.5
		      * (robot_specifications->max_sensors_range
			 - robot_specifications->robot_size
			 - (robot_specifications->decay_with_distance * 
			    point_distance))
			/ robot_specifications->max_sensors_range
			  + 0.5;
	      else
		value_prediction = 
		  ((value_prediction
		    / robot_specifications->network_value_mean * 0.5) - 0.5) 
		    * (robot_specifications->max_sensors_range 
		       - robot_specifications->robot_size
		       - (robot_specifications->decay_with_distance * 
			  point_distance))
		      / robot_specifications->max_sensors_range  
			+ 0.5;
	    }
	    else{
	      if (value_prediction >= robot_specifications->network_value_mean)
		value_prediction =
		  (value_prediction - robot_specifications->network_value_mean)
		    / (1.0 - robot_specifications->network_value_mean) * 0.5
		      + 0.5;
	      else
		value_prediction = 
		  ((value_prediction
		    / robot_specifications->network_value_mean * 0.5) - 0.5) 
		    + 0.5;
	    }

	  
	    /* copy network output into the local display */
	    local_map[local_map_index] = value_prediction;
	    local_active[local_map_index] = 1;
	    local_robot[local_map_index] = 0;
	  }
	}	
      }
      else
	printf("STRANGE: Seems your array bounds are too tight: %d %d.\n",
	       shifted_x, shifted_y);
    }
  }


  if (sensor_end_point){
    free(sensor_end_point);
    sensor_end_point = NULL;
  }
}
	






/************************************************************************
 *
 *   NAME:         smooth_local_map
 *                 
 *   FUNCTION:     Smoothes a local map
 *                 
 ************************************************************************/
  
	
/******  Table for smoothing maps - this is important for gradient search */
static float *smooth = NULL;
static float *local_smooth_map;

void smooth_local_map(ROBOT_SPECIFICATIONS_PTR robot_specifications)
{

  register int i, j, ii, jj;
  int local_size;
  int index1, index2, index3;
  float count;

  if (robot_specifications->smooth_radius <= 1)
    return;			/* no smoothing */

  /* 
   * compute smoothing table. this is done only once.
   */
  
  if (smooth == NULL){
    smooth = (float *) 
      malloc (sizeof(float) * robot_specifications->smooth_radius
	      * robot_specifications->smooth_radius);
    for (i = 0; i < robot_specifications->smooth_radius; i++)
      for (j = 0; j < robot_specifications->smooth_radius; j++)
	smooth[i * robot_specifications->smooth_radius + j] =
	  1.0 / ((float) (i+j+2));
  }
  
  
  /* 
   * allocate memory
   */
  
  local_size = robot_specifications->local_map_dim_x
    * robot_specifications->local_map_dim_y;

  if (local_smooth_map == NULL){


    local_smooth_map  = (float *) (malloc(sizeof(float) * local_size));
    
    if (local_smooth_map == NULL){
      printf("ABORT: out of memory (x)!\n");
      exit(1);
    }
    
  }
    
  
  /* 
   * compute smoothed map
   */
  
  for (i = 0; i < local_size; i++)
    local_smooth_map[i] = 0.0;
  
  for (i = 0; i < robot_specifications->local_map_dim_x; i++)
    for (j = 0; j < robot_specifications->local_map_dim_y; j++){
      index1 = i * robot_specifications->local_map_dim_y + j;
      if (local_active[index1] && !local_robot[index1]){
	count = 0.0;
	for (ii = 0; ii < robot_specifications->smooth_radius; ii++){
	  if (i + ii < robot_specifications->local_map_dim_x){
	    for (jj = 0; jj < robot_specifications->smooth_radius; jj++)
	      if (j + jj < robot_specifications->local_map_dim_y){
		index2 = (i+ii) * robot_specifications->local_map_dim_y 
		  + (j+jj);
		index3 = ii * robot_specifications->smooth_radius + jj;
		if (local_active[index2] && !local_robot[index2]){
		  local_smooth_map[index1] += 
		    smooth[index3] * local_map[index2];
		  count += smooth[index3];
		}
	      }
	    for (jj = 0; jj < robot_specifications->smooth_radius; jj++)
	      if (j - jj >= 0){
		index2 = (i+ii) * robot_specifications->local_map_dim_y 
		  + (j-jj);
		index3 = ii * robot_specifications->smooth_radius + jj;
		if (local_active[index2] && !local_robot[index2]){
		  local_smooth_map[index1] += 
		    smooth[index3] * local_map[index2];
		  count += smooth[index3];
		}
	      }
	  }
	  if (i - ii >= 0){
	    for (jj = 0; jj < robot_specifications->smooth_radius; jj++)
	      if (j + jj < robot_specifications->local_map_dim_y){
		index2 = (i-ii) * robot_specifications->local_map_dim_y
		  + (j+jj);
		index3 = ii * robot_specifications->smooth_radius + jj;
		if (local_active[index2] && !local_robot[index2]){
		  local_smooth_map[index1] += 
		    smooth[index3] * local_map[index2];
		  count += smooth[index3];
		}
	      }
	    for (jj = 0; jj < robot_specifications->smooth_radius; jj++)
	      if (j - jj >= 0){
		index2 = (i-ii) * robot_specifications->local_map_dim_y 
		  + (j-jj);
		index3 = ii * robot_specifications->smooth_radius + jj;
		if (local_active[index2] && !local_robot[index2]){
		  local_smooth_map[index1] += 
		    smooth[index3] * local_map[index2];
		  count += smooth[index3];
		}
	      }
	  }
	}
	if (count >= 0)
	  local_smooth_map[index1] /= count;
      }
      else
	  local_smooth_map[index1] = local_map[index1];
    }

  /* 
   * and copy result back into the local map
   */
  
  for (i = 0; i < local_size; i++)
    local_map[i] = local_smooth_map[i];

}




/************************************************************************
 *
 *   NAME:         clip_local_map
 *                 
 *   FUNCTION:     Smoothes a local map
 *                 
 ************************************************************************/
  
	

void clip_local_map(ROBOT_SPECIFICATIONS_PTR robot_specifications)
{

  int i, j, k, index, found, index2, mmax;
  float dist, distx, disty, float_i, float_j, d;

  mmax = robot_specifications->local_map_dim_x;
  if (robot_specifications->local_map_dim_y > mmax)
    mmax = robot_specifications->local_map_dim_y;
    

  if (robot_specifications->max_sensors_range > 
      robot_specifications->max_occupied_sensors_range &&
      robot_specifications->max_occupied_sensors_range >
      robot_specifications->occupied_outer_width){
    for (i = 0; i < robot_specifications->local_map_dim_x; i++)
      for (j = 0; j < robot_specifications->local_map_dim_y; j++){
	index = i * robot_specifications->local_map_dim_y + j;
	distx = ((float) i) * robot_specifications->resolution
	  - robot_specifications->max_sensors_range;
	disty = ((float) j) * robot_specifications->resolution
	  - robot_specifications->max_sensors_range;
	dist = sqrt((distx*distx)+(disty*disty));
	if (dist > robot_specifications->max_occupied_sensors_range){
	  for (k = 0, found = 0; k <= mmax && !found; k++){
	    d = (((float) k) / ((float) (mmax)))
	      / robot_specifications->max_occupied_sensors_range
	      * (robot_specifications->max_occupied_sensors_range -
		 robot_specifications->occupied_outer_width);
	    float_i = 
	      (d * ((float) i)) + 
		((1.0-d) * 0.5 *
		 ((float) (robot_specifications->local_map_dim_x)));
	    float_j = 
	      (d * ((float) j)) + 
		((1.0-d) * 0.5 *
		 ((float) (robot_specifications->local_map_dim_y)));
	    index2 = ((int) float_i) * robot_specifications->local_map_dim_y 
	      + ((int) float_j);
	    if (local_active[index2] == 0 || local_map[index2] < 
		robot_specifications->network_occupied_value)
	      found = 1;
	  }
	  if (found)
	    local_active[index] = 0;
	}
      }
  }
}



/************************************************************************
 *
 *   NAME:         angle_to_wall
 *                 
 *   FUNCTION:     checks, if there is an Obvious walls next to the robot.
 *                 returns the angle, if adjacent wall is found.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: 1, if wall found, 0 if not
 *                 
 ************************************************************************/



int angle_to_wall(float *lasers, /* vector */
		  float *laser_angles, /* vector */
		  float *angle) /* return value - parameter */
{
  int i, j, k;
  float endpoints_x[400];
  float endpoints_y[400];
  float line_x, line_y, line_dx, line_dy;
  float best_line_x, best_line_y, best_line_dx, best_line_dy;
  float distance, best_distance;
  int   best_i, best_def;
  float total_sum, length, summand;
  float p, q, discr, lambda1, lambda2;
  float x0, x1, y0, y1;
  int max_range_detected;

  if (robot_specifications->num_sensors > 400){
    fprintf(stderr, "ERROR: too many snsors. Fix that first.\n");
    return 0;
  }

  if (robot_specifications->line_recognition_threshold <= 0.0)
    return 0;
  
  best_def = 0;

  /*
   * Initializations to make the compiler happy
   */
  
  best_line_x  = 0.0;
  best_line_y  = 0.0;
  best_line_dx = 1.0;
  best_line_dy = 1.0;
  best_distance = 999.9;
  best_i = 0;


  /*
   * Compute the coordinates of the sensor endpoints
   */

  for (i = 0; i < robot_specifications->num_sensors; i++){
    endpoints_x[i] = lasers[i] * cos(laser_angles[i] / 180.0 * M_PI);
    endpoints_y[i] = lasers[i] * sin(laser_angles[i] / 180.0 * M_PI);
    /*fprintf(stderr, "\t %6.4f %6.4f -> %6.4f %6.4f\n",
      lasers[i], laser_angles[i], endpoints_x[i], endpoints_y[i]);*/
  }


  



  /*
   * Now fit lines for all n-tupels of adjacent values (points)
   */

  
  for (i = 0; i < robot_specifications->num_sensors; i++){ /* i=first value */

    max_range_detected = 0;
    
    /*
     * check, if one of the sensors is max-range
     */
    
    for (j = 0, k = i; 
	 j < robot_specifications->line_recognition_neighbors;
	 j++, k++){ /* j: over all values */
      if (k >= robot_specifications->num_sensors) 
	k = 0;
      if (lasers[k] >= robot_specifications->max_sensors_range ||
	  lasers[k] < -0.5)
	max_range_detected = 1;
    }
    

    distance = 0.0;
    
    
    if (!max_range_detected){	/* ony with *real* sensor values */
      
      
      /*
       * Guess an initial line
       */

      line_x  = 0.0;
      line_y  = 0.0;
      line_dx = 0.0;
      line_dy = 0.0;
      for (j = 0, k = i; j < robot_specifications->line_recognition_neighbors;
	   j++, k++){ /* j: over all values */
	if (k >= robot_specifications->num_sensors) k = 0;
	line_x += endpoints_x[k];
	line_y += endpoints_y[k];
	if (j < (robot_specifications->line_recognition_neighbors / 2)){
	  line_dx += endpoints_x[k];
	  line_dy += endpoints_y[k];
	}
	else if (j >= ((robot_specifications->line_recognition_neighbors 
			+ 1) / 2)){
	  line_dx -= endpoints_x[k];
	  line_dy -= endpoints_y[k];
	}
      }
      line_x /= ((float) robot_specifications->line_recognition_neighbors);
      line_y /= ((float) robot_specifications->line_recognition_neighbors);
      length = sqrt((line_dx * line_dx) + (line_dy * line_dy));
      if (length != 0.0){
	line_dx /= length;	/* normalized */
	line_dy /= length;
      }
      else{
	fprintf(stderr, "STRANGE1: Bug in the program %g\n", length);
	return 0;
      }

      /*
       * Mesaure distance
       */

      distance = 0.0;
      total_sum = 0.0;
      for (j = 0, k = i; j < robot_specifications->line_recognition_neighbors;
	   j++, k++){ /* j: over all values */
	if (k >= robot_specifications->num_sensors) k = 0;
	summand  = ((endpoints_y[k] - line_y) * line_dx) /* product with
							  * line normal */
	  - ((endpoints_x[k] - line_x) * line_dy);
	distance  += summand * summand;
	total_sum += fabs(lasers[k]);
      }
      
      if (total_sum != 0.0){
	distance /= total_sum;
	
	/*
	 * Check, if the new distance is better than anything else
	 */
	
	if ((!best_def || best_distance > distance) &&
	    !max_range_detected){
	  best_line_x      = line_x;
	  best_line_y      = line_y;
	  best_line_dx     = line_dx;
	  best_line_dy     = line_dy;
	  best_distance    = distance;
	  best_i           = i;
	  best_def         = 1;
	}
      }
      else
	fprintf(stderr, "STRANGE2: Bug in the program %g\n", total_sum);
	
    }
  }    
    


  if (!best_def ||
      best_distance > robot_specifications->line_recognition_threshold){
    if (program_state->graphics_initialized){
      G_clear_markers(REGRESSION);
      G_display_markers(REGRESSION);}
    return 0;
  }

#ifdef LASERINT_debug
  printf("BEST: laser %d to %d, dist=%g",
	 best_i, (best_i + robot_specifications->line_recognition_neighbors)
	 % robot_specifications->num_sensors,
	 best_distance);
#endif  
  /*
   * Compute the angle
   */

  *angle = atan2(best_line_dy, best_line_dx) * 180.0 / M_PI;
  for (; *angle < -180.0;) *angle += 360.0;
  for (; *angle >  180.0;) *angle -= 360.0;
  
  /*
   * Display the best guy
   */



  p = 2.0 * ((best_line_dx * best_line_x) + (best_line_dy * best_line_y));
  q = (best_line_x * best_line_x) + (best_line_y * best_line_y)
    - (0.9 * robot_specifications->max_sensors_range
       * robot_specifications->max_sensors_range);
  discr = 0.25 * p * p - q;
  if (discr < 0.0){
    fprintf(stderr, "STRANGE3: Bug in the program: %g %g %g.\n", p, q, discr);
    return 0;
  }

  discr = sqrt(discr);
  lambda1 = - 0.5 * p + discr;
  lambda2 = - 0.5 * p - discr;
  x0 = best_line_x + lambda1 * best_line_dx;
  y0 = best_line_y + lambda1 * best_line_dy;
  x1 = best_line_x + lambda2 * best_line_dx;
  y1 = best_line_y + lambda2 * best_line_dy;

    
  if (program_state->graphics_initialized){
    G_clear_markers(REGRESSION);
    G_add_marker(REGRESSION, -y0, x0, 0); 
    G_add_marker(REGRESSION, -y1, x1, 0);
  }
  return 1;
}




/************************************************************************\
|************************************************************************|
\************************************************************************/

/************************************************************************
 *
 *   NAME:         main 
 *                 
 *   FUNCTION:     main loop - checks for tcx events
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/




main(int argc, char **argv)
{
  long int sleep_duration;
  fd_set readMask;
  struct timeval last_script_read_time = {0, 0};
  struct timeval this_time             = {0, 0};  
  float time_difference;
  


  bParamList = bParametersAddEntry(bParamList, "robot", "name", "B21");
  bParamList = bParametersAddEntry(bParamList, "", "TCXHOST", "localhost");
  bParamList = bParametersAddFile (bParamList, "etc/beeSoft.ini");
  bRobotType = bParametersGetParam(bParamList, "robot", "name");

  if (getenv("TCXHOST") != NULL){
    bParamList = bParametersAddEntry(bParamList, "", "TCXHOST", \
				     getenv("TCXHOST"));
  }



  check_commandline_parameters(argc, argv, program_state);
  process_images();
  init_program(program_state, robot_state, 
	       robot_specifications, neural_network, &all);
  init_network(neural_network);
  init_tcx(program_state);
  read_init_file(robot_state, program_state, robot_specifications);
  allocate_everything(robot_state, program_state, robot_specifications);
  init_graphics(robot_state, program_state, robot_specifications);
  open_log(LOG_NAME);

  if (robot_specifications->plot_auto_on && !program_state->is_server)
    plot_on(robot_specifications, program_state, robot_state, 
	    PLOT_NAME, VR_NAME, SMF_NAME); 
  
  do{
    /*
     * connect to clients, if not connected
     */ 


    if (!program_state->base_connected)
      connect_to_BASE(program_state);
    if (!program_state->map_connected)
      connect_to_MAP(program_state);
    if (!program_state->server_connected)
      connect_to_SERVER(program_state);


    /*
     * sleep + wait for the next event
     */ 

    if (!program_state->map_update_pending &&
	!program_state->processing_script){
      block_waiting_time.tv_sec  = 0;
      block_waiting_time.tv_usec = 100000;
      if (program_state->graphics_initialized){
	block_wait(&block_waiting_time, program_state->tcx_initialized,
		   program_state->use_graphics &&
		   program_state->graphics_initialized);   
      }
      else{
	readMask = (Global->tcxConnectionListGlobal);
	select(FD_SETSIZE, &readMask, NULL, NULL, &block_waiting_time);
      }
    }
    /*
     * position control
     */
    if (program_state->map_update_pending && program_state->calibration == -1)
      do_position_control(robot_specifications, program_state, robot_state);
    /*
     * handle tcx events
     */ 

    if (program_state->tcx_initialized) 
      tcxRecvLoop((void *) &TCX_no_waiting_time);


    /*
     * check for mouse events
     */ 

    if (program_state->use_graphics &&
	program_state->graphics_initialized)
      mouse_test_loop(robot_state, program_state, robot_specifications);
    /*
     * read event from script
     */ 

    if (program_state->processing_script){
      gettimeofday(&this_time, NULL);
      time_difference = 
	((float) (this_time.tv_sec - last_script_read_time.tv_sec))
	+ (((float) (this_time.tv_usec - last_script_read_time.tv_usec))
	   /  1000000.0);
      if (time_difference > program_state->delay_in_replay){
	last_script_read_time.tv_sec  = this_time.tv_sec;
	last_script_read_time.tv_usec = this_time.tv_usec;
	read_script(robot_state, program_state, robot_specifications);
      }
      else if (!program_state->map_update_pending){
	usleep(100000);
      }
    }

  } while (!program_state->quit);

  plot_off(robot_specifications, program_state, robot_state);
  close_log();
  exit(0);
}
