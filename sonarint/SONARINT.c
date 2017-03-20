
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Welcome!
 *****
 ***** This file is part of the BeeSoft robot control software.
 ***** The version number is:
 *****
 *****                  v1.2   (released October 24, 1997)
 *****
 ***** Please refer to bee/src/COPYRIGHT for copyright and liability
 ***** information.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#ifdef VMS
#include "vms.h"
#endif


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
#include "SONAR-messages.h"
#include "BASE-messages.h"
#include "SONARINT.h"
#include "bUtils.h"

/*wasi*/
int useSSP      = FALSE;
int useNbrInner = FALSE;
int useNbrOuter = FALSE; 
float test=0.0;
/******/

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

struct bParamList * bParamList = NULL;

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


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

struct timeval TCX_no_waiting_time = {0, 0};


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/************************************************************************
 *wasi   
 *  NAME:	EASY_SUBST_FUNCTION (esf)
 *  FUNCTION:	Linear approximation of below used exp-function in easy_
		sonarinterpreter for speedup
 *  AUTHOR:	Wasilios Ntallis
 *  LAST UPD:   20.05.1999
 ************************************************************************/
float esf(float pred, float s_diff)
{
				/*pred=old prediction, 
				  s_diff=sonar value difference		
				  s_diff>=0 !!!				*/
   float new_pred=0.5;		/*New prediction value			*/
   float help=s_diff/560.0;       /*560 is the maximal distance-difference
				  which will be measured. If the distance
				  difference is > 280 (oversteps the half 
				  of the maximum) then prediction is 0.5*/
   
   /*-------------------------------------------------------------------*/
   /*  Update function is:   p*(1-d/280)+d/560, for 0<=d<=280           */
   /*                        0.5              , for d>280               */
   /*  In case of low distance difference it converts to the old proba- */
   /*  bility value (pred) and in case of big distance it is 0.5        */
   /*-------------------------------------------------------------------*/
   if (s_diff>280.0) new_pred = 0.5;
   else              new_pred = pred*(1.0-2.0*help)+help;
   
   return (new_pred);
} 


/************************************************************************
 *wasi
 *  NAME:	EASY_SONAR_INTERPRETER
 *  FUNCTION:	sonar-data interpretation
 *  AUTHOREN:	Wasilios Ntallis, Mark Moors
 *  LAST UPD:   10.07.1998
 *
 ************************************************************************/

float easy_sonar_interpreter(ROBOT_SPECIFICATIONS_PTR robot_specifications,
                  	     ROBOT_STATE_PTR robot_state,
                  	     float distance, float angle)
{
  int    act_sensor;	/*index of actual sensor 			*/
  float  diff_angle;    /*difference between given relative angle and 
  			  sonar_angle					*/
  float  inop_angle;	/*inner open angle, openangle which borders the 
  			  inner sonar-wedge, where the occupancy-problt.
  			  is larger than this within the outer wedge	*/
  float  psd; 		/*Permitted Sensor Disparity of two neighbour-  
  			  sensors as measurement for the correctness of 
  			  the actual sensor				*/
  float  prediction=0.5;/*output of this function			*/
  float  s_diff1=0.0;   /*difference between sensorvalues		*/
  float  s_diff2=0.0;	/*difference between sensorvalues		*/
  float  s_diffr=0.0;	/*resulting difference from s_diff1 and s_diff2 */  
  float  side_dist;	/*side-distance for left and right wall calculated 
  			  trinometrically from the sensor 1 and 5 respec
  			  tively					*/
  float  obs_front= 10;  /*Estimates front width of obstacle		*/
  float  obs_width= 30;  /*Estimatet width of obstabcle (wall)		*/
  float mittel_offset=0;

  /*--------------------------------------------------------------------*/  
  /* define the actual sonar for given relative angle of point (x,y) 	*/
  /* !!! angle in [-270;90] !!!                                         */
  /*--------------------------------------------------------------------*/
  if      ((angle>= -97.5)&&(angle<= -82.5))
    act_sensor = 0;
  else if ((angle>= -37.5)&&(angle<= -22.5))
    act_sensor = 1;  
  else if ((angle>= -22.5)&&(angle<=  -7.5))
    act_sensor = 2;  
  else if ((angle>=  -7.5)&&(angle<=   7.5))
    act_sensor = 3;  
  else if ((angle>=   7.5)&&(angle<=  22.5))
    act_sensor = 4;    
  else if ((angle>=  22.5)&&(angle<=  37.5))
    act_sensor = 5;   
  else if(((angle>=  82.5)&&(angle<=  90.0))||
	  ((angle>=-270.0)&&(angle<=-262.5)))
    act_sensor = 6;
  else if ((angle>=-187.5)&&(angle<=-172.5))
    act_sensor = 7;
  else
    act_sensor = -1;	/*in case of non visibility			*/

  if (act_sensor>=0) {
    /*------------------------------------------------------------------*/
    /*compute difference between given relative angle and sonar_angle,	*/
    /*set permitted sensor disparity and inner open angle		*/
    /*------------------------------------------------------------------*/
    diff_angle = robot_specifications->sensor_angles[act_sensor] - angle;
    
    if (diff_angle<0)
      diff_angle=-diff_angle;
    
    inop_angle =  0.0;	
    psd        = 10.0;
    
    /*------------------------------------------------------------------*/
    /*set prediction for the field specified by diff_angle and distance */
    /*note: distance>=0!!!						*/
    /*------------------------------------------------------------------*/
    if (diff_angle > inop_angle) {
      if (distance+mittel_offset < robot_state->sensor_values[act_sensor]) {
        if (((robot_state->sensor_values[act_sensor])-(distance+mittel_offset))>obs_front)
     	  prediction = 0.7;
     	else  
     	  prediction = 0.15;
      }
      else {
	if (distance+mittel_offset-(robot_state->sensor_values[act_sensor])>obs_width)
	  prediction = 0.5;
	else
          prediction = 0.15;	
      }
    }
    else {
      if (distance+mittel_offset < robot_state->sensor_values[act_sensor]) {
	if (((robot_state->sensor_values[act_sensor])-(distance+mittel_offset))>obs_front)
          prediction = 0.75;
        else
          prediction = 0.15;
      }
      else {
        if ((distance+mittel_offset)-(robot_state->sensor_values[act_sensor])>obs_width)
          prediction = 0.5;
        else  
          prediction = 0.15;
      }
    }
    
    /*------------------------------------------------------------------*/
    /*  dirty hack: case of having exceeded the sensor-max-range	*/
    /*------------------------------------------------------------------*/
    if((distance+mittel_offset)>280.0) 
      prediction=0.5;

    /*------------------------------------------------------------------*/
    /*calculate to prediction the prob. with respect to the neighbour-  */
    /*sensors.								*/
    /*The update-function is:  	p*e(-x^2)+0.5*(1-e(-x^2))		*/
    /*or the above defined  :   esf(p,x)                                */
    /*------------------------------------------------------------------*/
    //    act_sensor=-1;   //ACHTUNG: nur ne Massnahme!!!    
    switch (act_sensor) {  
    case 0 : 
      if (useNbrOuter) {
	/*----------------------------------------------------*/
	/*get distance calculated trigonometrically from the  */
	/*right neighbour (heading -30 towards -90 of actual  */
	/*sensor. 0.5=cos(60) = cos(90-30)		      */
	/*----------------------------------------------------*/
	side_dist = 0.5 * robot_state->sensor_values[1];  
	
	s_diff1 = robot_state->sensor_values[act_sensor] - side_dist;
	if (s_diff1<0) s_diff1=-s_diff1;
	
	if (s_diff1 > psd) {		/*if s_diff1 too much  	*/
	  // prediction = (prediction * exp(-(s_diff1*s_diff1/1000.0)) + 
	  // (0.5*(1.0-exp(-(s_diff1*s_diff1/1000.0)))));
	  prediction = esf(prediction,s_diff1/1000.0);
	}
      }				
      break;  
    case 1 : if (useNbrInner) {
      /*------------------------------------------------------*/
      /*get value of the right neighbour		      */
      /*------------------------------------------------------*/
      s_diff1 = robot_state->sensor_values[act_sensor] -
	robot_state->sensor_values[act_sensor+1];
      if (s_diff1<0.0) s_diff1=-s_diff1;
      
      if (s_diff1>psd)	{
	// prediction = (prediction * exp(-(s_diff1*s_diff1/1000.0)) + 
	// (0.5*(1.0-exp(-(s_diff1*s_diff1/1000.0)))));
	prediction = esf(prediction,s_diff1/1000.0);
      }
    }  	  
    break;
    case 2 :
    case 3 :
    case 4 : 
      if (useNbrInner) {
	/*----------------------------------------------------*/
	/*get values of the right and left neighbours	      */
	/*----------------------------------------------------*/
	s_diff1 = robot_state->sensor_values[act_sensor] -
	  robot_state->sensor_values[act_sensor-1];
	s_diff2 = robot_state->sensor_values[act_sensor] -
	  robot_state->sensor_values[act_sensor+1];
	if (s_diff1<0.0) s_diff1=-s_diff1;
	if (s_diff2<0.0) s_diff2=-s_diff2;
	
	if ((s_diff1>psd)&&(s_diff2>psd)) {
	  s_diffr = (s_diff1*s_diff1)+(s_diff2*s_diff2); /*Pythagoras^2*/
  		   
//    		     prediction = (prediction * exp(-(s_diffr*s_diffr/1000.0)) + 
//				  (0.5*(1.0-exp(-(s_diffr*s_diffr/1000.0)))));
	  prediction = esf(prediction,s_diffr/1000.0);
	}
      }  	  
      break;
    case 5 : 
      if (useNbrInner) {
	/*----------------------------------------------------*/
	/*get value of the left neighbour		      */
	/*----------------------------------------------------*/
	s_diff1 = robot_state->sensor_values[act_sensor] -
	  robot_state->sensor_values[act_sensor-1];
	if (s_diff1<0.0) s_diff1=-s_diff1;
	
	if (s_diff1>psd) {
//    		     prediction = (prediction * exp(-(s_diff1*s_diff1/1000.0)) + 
//				  (0.5*(1.0-exp(-(s_diff1*s_diff1/1000.0)))));
       		     prediction = esf(prediction,s_diff1/1000.0);
	} 
      }                  
      break; 	
    case 6 : 
      if (useNbrOuter){
	/*----------------------------------------------------*/
	/*get distance calculated trigonometrically from the  */
	/*left neighbour. 0.5 = cos(60) (analog with case 0)  */
	/*----------------------------------------------------*/ 
	side_dist = 0.5 * robot_state->sensor_values[5];  
	
	s_diff1 = robot_state->sensor_values[act_sensor] - side_dist;
	if (s_diff1<0.0) s_diff1=-s_diff1;
	
	if (s_diff1 > psd) {
//    		     prediction = (prediction * exp(-(s_diff1*s_diff1/1000.0)) + 
//				  (0.5*(1.0-exp(-(s_diff1*s_diff1/1000.0)))));
		     prediction = esf(prediction,s_diff1/1000.0);
	}	
      }			                  
      break;          	
    }
    
    /*------------------------------------------------------------------*/
    /*calculate decay. 							*/
    /*The update-function is:	p*e(-x)+0.5*(1-e(-x))	 (eigentlich)	*/
    /*------------------------------------------------------------------*/
    //    prediction=-0.5*(((distance)/300.0)*(prediction-0.5))+prediction;
    
    
    
  }  /*if (act_sensor>=0)*/
  else
    prediction = 0.5;	/*in case of no visibility return abs. uncertainty*/

if ((prediction>1.0)||(prediction<0.0))  
fprintf(stderr,"MEEEEP!!! \nPrediction:%f   Sensor:%d\n",prediction,act_sensor);

  return (prediction);  
}  	      
/******/


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
  int i= 0,j= 0,k= 0;
  int first_sensor= 0, last_sensor= 0;
  float angle_to_sensor_0= 0.0, local_angle= 0.0;
  float net_input[100];
  float net_output[100];
  
  /* compute angle relative to r._spec.->sensor_angle[0] */
  angle_to_sensor_0 = angle - robot_specifications->first_sensor_angle;

  for (;angle_to_sensor_0 >= 360.0;) angle_to_sensor_0 -= 360.0;
  for (;angle_to_sensor_0 <    0.0;) angle_to_sensor_0 += 360.0;

  /* comput index of sensor left to angle */
  first_sensor = (int) (angle_to_sensor_0 * 
			robot_specifications->num_sensors / 360.0);
  
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
  int from_x= 0, to_x= 0, from_y= 0, to_y= 0, x= 0, y= 0, shifted_x= 0, shifted_y= 0;
  int local_map_index= 0;
  float point_distance= 0.0, point_x= 0.0, point_y= 0.0, point_angle= 0.0;
  float value_prediction= 0.0;
  


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
	
	/* if point happens within the reach of sonar sensors */
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
	
	else {
          if (!useSSP) {	
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
	  } 
/*wasi*/
	  else { /* if(useSSP) */
	    
	    /* compute relative angle to (x,y) */
	    if (point_y == 0.0 && point_x == 0.0)
	      point_angle = -90.0;       /* robot_state->orientation;*/
	    else	  
	      point_angle = (atan2(point_y, point_x) / M_PI * 180.0)
	        -90.0;       /*robot_state->orientation;*/
	    
	    value_prediction = 
	      easy_sonar_interpreter (robot_specifications, robot_state,
				      point_distance, point_angle);
	  }	  
	  if ((value_prediction>=1.0)||(value_prediction<=0.0))
	    fprintf(stderr,"1:%f\n",value_prediction);

/*******/
          if (!useSSP) { 
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
	    
	  } /*if (!useSSP)*/  
	  
	  
	  /* Copy Network output into the local display */
	  local_map[local_map_index] = value_prediction;
	  local_active[local_map_index] = 1;
/*wasi*/	  

	  if(useSSP) {  
	    /*
	     * Deaktiviere Lokalkartenbildung an den toten Winkeln Pioneers
	     * ACHTUNG: Da point_angle in [-270;90] liegt wird der linke
	     * untere tote Winkel (eigentich innerhalb 97.5 und 172.5) durch
	     * -187.5 und -262.5 begrenzt (siehe unterste Bedingung)
	     */
	    if(((point_angle>-172.5)&&(point_angle< -97.5))||
	       ((point_angle> -82.5)&&(point_angle< -37.5))||
	       ((point_angle>  37.5)&&(point_angle<  82.5))||
	       ((point_angle>-262.5)&&(point_angle<-187.5))) {
	      local_active[local_map_index]=0;  
	      local_robot [local_map_index]=0;
       	    }
	  } /*useSSP*/
/******/
	} /*else*/	
      }
      else
	printf("STRANGE: Seems your array bounds are too tight: %d %d.\n",
	       shifted_x, shifted_y);
    }
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
static float *local_smooth_map=NULL;

void smooth_local_map(ROBOT_SPECIFICATIONS_PTR robot_specifications)
{

  register int i= 0, j= 0, ii= 0, jj= 0;
  int local_size= 0;
  int index1= 0, index2= 0, index3= 0;
  float count= 0.0;

  if (robot_specifications->smooth_radius <= 1)
    return;			/* no smoothing */

  /* 
   * compute smoothing table. this is done only once.
   */
  
  if (smooth == NULL){
    smooth = (float *) 
      calloc ((robot_specifications->smooth_radius
	      * robot_specifications->smooth_radius),sizeof(float));
    for (i = 0; i < robot_specifications->smooth_radius; i++)
      for (j = 0; j < robot_specifications->smooth_radius; j++)
	smooth[i * robot_specifications->smooth_radius + j] =
	  1.0 / ((float) (i+j+2.0));
  }
  
  
  /* 
   * allocate memory
   */
  
  local_size = robot_specifications->local_map_dim_x
    * robot_specifications->local_map_dim_y;

  if (local_smooth_map == NULL){


    local_smooth_map  = (float *) (calloc(local_size, sizeof(float)));
    
    if (local_smooth_map == NULL){
      printf("ABORT: out of memory (x)!\n");
      exit(1);
    }
    
  }
    
  
  /* 
   * compute smoothed map
   */
  
//  for (i = 0; i < local_size; i++)
//    local_smooth_map[i] = 0.0;
  
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
		  local_smooth_map[index1] += smooth[index3] * local_map[index2];
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
  
//  for (i = 0; i < local_size; i++)
//    local_map[i] = local_smooth_map[i];
    memcpy(local_map,local_smooth_map,local_size*sizeof(float));

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

  int i=0, j=0, k=0, index=0, found=0, index2=0, mmax=0;
  float dist=0.0, distx=0.0, disty=0.0, float_i=0.0, float_j=0.0, d=0.0;

if (useSSP) return ;

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



int angle_to_wall(float *sonars, /* vector */
		  float *sonar_angles, /* vector */
		  float *angle) /* return value - parameter */
{
  int i=0, j=0, k=0;
  float endpoints_x[100];
  float endpoints_y[100];
  float line_x=0.0, line_y=0.0, line_dx=0.0, line_dy=0.0;
  float best_line_x=0.0, best_line_y=0.0, best_line_dx=0.0, best_line_dy=0.0;
  float distance=0.0, best_distance=0.0;
  int   best_i=0, best_def=0;
  float total_sum=0.0, length=0.0, summand=0.0;
  float p=0.0, q=0.0, discr=0.0, lambda1=0.0, lambda2=0.0;
  float x0=0.0, x1=0.0, y0=0.0, y1=0.0;
  int max_range_detected=0;
  
  if (robot_specifications->num_sensors > 100){
    fprintf(stderr, "ERROR: too many snsors. Fix that first.\n");
    return 0;
  }
  
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
    endpoints_x[i] = sonars[i] * cos(sonar_angles[i] / 180.0 * M_PI);
    endpoints_y[i] = sonars[i] * sin(sonar_angles[i] / 180.0 * M_PI);
    /*fprintf(stderr, "\t %6.4f %6.4f -> %6.4f %6.4f\n",
      sonars[i], sonar_angles[i], endpoints_x[i], endpoints_y[i]);*/
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
      if (sonars[k] >= robot_specifications->max_sensors_range)
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
	total_sum += fabs(sonars[k]);
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

#ifdef SONARINT_debug
  printf("BEST: sonar %d to %d, dist=%g",
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

/**********************************************************************
 *
 * VMS does not seem to provide support for setting SIGALRM with
 * sub-second resolution.  As a result, the ualarm() method of 
 * controlling the speed of script playback can't work.  Instead,
 * the value of struct timeval block_waiting_time is computed to
 * cause the select() calls to exit at the proper time.  Though
 * the result is a little more code to determine if when select()
 * exited it is actually time to read another line from the script,
 * I think that the results are quite satisfactory. 
 *                                   - Tyson
 *
 **********************************************************************/


main(int argc, char **argv)
{
  struct timeval read_begin_time;
  struct timeval read_end_time;
  long int sleep_duration=0, test =0, local_size = 0;
  static int something_happened = 1;

  bParamList = bParametersAddFile(bParamList, "etc/beeSoft.ini");

  check_commandline_parameters(argc, argv, program_state);

  if (useSSP)
    /* add some parameter files */
    bParamList = bParametersAddEntry(bParamList, "robot", "pioneer","yes");
  else
    bParamList = bParametersAddEntry(bParamList, "robot", "pioneer","no");

  /* add some enviroment variables */
  bParamList = bParametersAddEnv(bParamList, "", "TCXHOST");
  
  /* add command line arguements */
  bParamList = bParametersAddArray(bParamList, "", argc, argv);
  
  /* Fill the global parameter structure */
  bParametersFillParams(bParamList);

  if (useSSP) {
    /* This is a bad hack. Frank :) */
    /* I will fix this later */
    bRobot.sonar_cols[0] = bRobot.pio_sonar_number;
  }
  
  init_program(program_state, robot_state, 
	       robot_specifications, neural_network, &all);

  read_init_file(robot_state, program_state, robot_specifications);

  allocate_everything(robot_state, program_state, robot_specifications);

  init_network(neural_network);

  init_tcx(program_state);

  init_graphics(robot_state, program_state, robot_specifications);

  open_log(LOG_NAME);

  fprintf(stderr, "\nbRobot.sonar_cols[0]:%d --- %d",bRobot.sonar_cols[0],robot_specifications->num_sensors);
  fprintf(stderr, "\nfirst_angle: %f",robot_specifications->first_sensor_angle);

#ifndef VMS
  signal(SIGALRM, alarm_handler);
#else 
  gettimeofday(&read_begin_time, NULL);
#endif 


  do{

    if (!program_state->base_connected)
      connect_to_BASE(program_state);
    if (!program_state->map_connected)
      connect_to_MAP(program_state);


    if (!something_happened){
#ifdef SONARINT_debug
      gettimeofday(&read_begin_time, NULL);
#endif
      block_waiting_time.tv_sec  = 1;
      block_waiting_time.tv_usec = 0;
      if (program_state->graphics_initialized){
	(void) block_wait(&block_waiting_time, program_state->tcx_initialized,
			  program_state->graphics_initialized);   
      }
      else{
	fd_set readMask;
	readMask = (Global->tcxConnectionListGlobal);
	select(FD_SETSIZE, &readMask, NULL, NULL, &block_waiting_time);
      }
#ifdef SONARINT_debug
      gettimeofday(&read_end_time, NULL);
      sleep_duration = read_end_time.tv_usec - read_begin_time.tv_usec;
      sleep_duration += 1000000 
	* (read_end_time.tv_sec - read_begin_time.tv_sec);
      fprintf(stderr, "SONARINT:main:skipping = %f sec\n",
	      (float)sleep_duration*0.000001);
#endif
    }

    
    if (program_state->tcx_initialized){
      TCX_no_waiting_time.tv_sec = 0;
      TCX_no_waiting_time.tv_usec = 0;
      tcxRecvLoop((void *) &TCX_no_waiting_time);
    }
    
    if (program_state->graphics_initialized)
      something_happened =
	mouse_test_loop(robot_state, program_state, robot_specifications);

#ifdef VMS
    gettimeofday(&read_end_time, NULL);
    sleep_duration = read_end_time.tv_usec - read_begin_time.tv_usec;
    sleep_duration += 1000000 
      * (read_end_time.tv_sec - read_begin_time.tv_sec);

    /* if time for next read_script() */
    if (sleep_duration>(long)(program_state->delay_in_replay*1000000.0)-5000){
	gettimeofday(&read_begin_time, NULL);
	program_state->read_next_script_event = 1;
#ifdef SONARINT_debug
	fprintf(stderr, "SONARINT:main:sleep period was  = %f sec ***\n",
	     (float)sleep_duration*0.000001);
#endif
    }
#ifdef SONARINT_debug
    else {
	fprintf(stderr, "SONARINT:main:skipping = %f sec\n",
		(float)sleep_duration*0.000001);
    }
#endif
#endif  /* VMS */
    something_happened = 0;

    if (program_state->processing_script &&
	program_state->read_next_script_event){
#ifdef SONARINT_debug
fprintf(stderr, "SONARINT:main:read_script()     -------\n"); 
#endif
	program_state->read_next_script_event = 0;
      read_script(robot_state, program_state, robot_specifications);
      something_happened = 1;
    }
      
#ifdef VMS
    sleep_duration  = (long) (program_state->delay_in_replay 
                              * 1000000.0); /* total duration between two
                                             * readings, set by command
                                             * line argument and/or file */
    gettimeofday(&read_end_time, NULL);

    sleep_duration -= read_end_time.tv_usec - read_begin_time.tv_usec;
    sleep_duration -= 1000000
	    * (read_end_time.tv_sec - read_begin_time.tv_sec);

    sleep_duration -= 500;  /* and that we lost in the computaton here */
    if (sleep_duration<0) sleep_duration=0;
    block_waiting_time.tv_sec=(int)((sleep_duration)*0.000001);
    block_waiting_time.tv_usec=(int)(sleep_duration)%1000000;
#endif  /* VMS */

  } while (!program_state->quit);

  close_log();
  exit(0);
}
