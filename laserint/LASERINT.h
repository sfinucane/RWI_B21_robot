
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/laserint/LASERINT.h,v $
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
 * $Log: LASERINT.h,v $
 * Revision 1.1  2002/09/14 15:34:06  rstone
 * *** empty log message ***
 *
 * Revision 1.39  2000/09/26 01:11:32  thrun
 * ,
 *
 * Revision 1.38  2000/09/06 05:42:19  thrun
 * Map editing buttons.
 *
 * Revision 1.37  2000/03/26 16:32:08  thrun
 * Can write image strips now.
 *
 * Revision 1.36  2000/03/14 05:23:56  thrun
 * fixed a few problems with the 3D mapping. This version
 * pruced the results first shown to the DAPRA folks.
 * In particular, it reads the texture entirely from file and
 * constructs a new texture file (laserint.ppm, which has
 * to be converted manyally into laserint.jpg)
 *
 * Revision 1.33  1999/12/27 04:12:11  thrun
 * Intermediate version, with dww and dlw (might not work).
 *
 * Revision 1.32  1999/12/27 03:43:15  thrun
 * Magic: Gradient descent for adapting
 * wall parameters xw yw and aw seems to work!!!
 *
 * Revision 1.35  2000/03/13 00:31:12  thrun
 * first version of he texturing module. Still a bit clumpsy.
 *
 * Revision 1.34  2000/01/02 16:58:02  thrun
 * intermediate version with 2 objects
 *
 * Revision 1.33  1999/12/27 04:12:11  thrun
 * Intermediate version, with dww and dlw (might not work).
 *
 * Revision 1.32  1999/12/27 03:43:15  thrun
 * Magic: Gradient descent for adapting
 * wall parameters xw yw and aw seems to work!!!
 *
 * Revision 1.31  1999/12/19 03:25:17  thrun
 * graphics for 3D object matching.
 *
 * Revision 1.30  1999/10/15 18:07:19  thrun
 * more 3D formats, more compact files.
 *
 * Revision 1.29  1999/10/15 01:30:45  thrun
 * improved interface for robot positioning
 *
 * Revision 1.28  1999/10/02 04:50:10  thrun
 * Minor tuning and changes. THis version built the first
 * nice map of the San Jose Tech Museum
 *
 * Revision 1.27  1999/10/01 17:58:17  thrun
 * reinstatiated the close-the-cycle feature.
 *
 * Revision 1.26  1999/09/28 21:49:53  thrun
 * Multi-robot support in various modules.
 * In particular, I had to introduce a new message for MAP that
 * communicates back robot positions and correction parameters.
 * This might make it necessary to modify existing modules that use
 * MAP - I tried to do this for all I am aware of, but I might have missed some.
 *
 * Revision 1.25  1999/09/28 04:50:00  thrun
 * Fixed minor bug in server architecture that caused the map
 * (in MAP) to be inconsistent.
 *
 * Revision 1.24  1999/09/28 03:31:00  thrun
 * Improved version in the server mode.
 *
 * Revision 1.23  1999/09/06 03:19:44  thrun
 * first working version of the multi-robot server architecture with
 * positon control.
 *
 * Revision 1.22  1999/09/05 21:57:21  thrun
 * initial client server robot for multi-robot mapping
 *
 * Revision 1.21  1999/09/05 17:18:51  thrun
 * 3D ceiling mapping
 *
 * Revision 1.19  1999/07/03 21:44:36  thrun
 * Fixed several bugs in LASERINT and tuned the parameters of
 * LASERINT, MAP, and PLAN for the new Urban Robot
 *
 * Revision 1.18  1999/07/03 18:49:35  thrun
 * LOASERINT can now send correction parameters directly to MAP.
 *
 * Revision 1.17  1999/06/21 18:30:24  thrun
 * Took out the sampling stuff (which never worked corretly). It's now
 * up to Dieter to put it in again...
 *
 * Revision 1.16  1999/05/08 19:47:28  thrun
 * drastically improved version with recursive matching and samples.
 * Not quite ready yet: samples aren't used in the right way, queue
 * seems not to work, and some constants are hardcoded (phase!) But
 * impressive results!
 *
 * Revision 1.15  1999/05/04 19:47:26  thrun
 * new file "pos.c"
 *
 * Revision 1.14  1999/05/04 01:14:20  thrun
 * queue instead of stack.
 *
 * Revision 1.13  1999/04/23 20:00:09  thrun
 * slight extensions - limited stack (is now a queue) and flag that
 * prevents integration of readings after sharp turn.
 *
 * Revision 1.12  1998/11/18 04:21:09  thrun
 * incorporated wall angle into estimation.
 *
 * Revision 1.11  1998/11/17 05:03:13  thrun
 * small, incremental improvements: better handling of sensor
 * displacement, and new parametr that enables laserint to ignore
 * scans when the robot is rotating too much.
 *
 * Revision 1.10  1998/11/16 01:50:40  thrun
 * positon control - works nice, includes global mapping and learning
 * the drift parameters.
 *
 * Revision 1.9  1998/11/15 16:19:17  thrun
 * initial position control method - works reasonably well. But
 * e careful with the timing, isn't an anytime algorithm yet.
 *
 * Revision 1.8  1998/11/12 14:22:54  thrun
 * ?.
 *
 * Revision 1.7  1998/08/11 22:18:24  thrun
 * PLANNER: fixed a problem with large shifts, a problem with th
 * function "pow()" under the latest Linux.
 *
 * Revision 1.6  1997/12/30 00:40:10  thrun
 * Non-neural network version of mapping.
 *
 * Revision 1.5  1997/08/12 03:06:01  thrun
 * intermediate version, provides some logging of sensor scans and partial
 * maps within map.
 *
 * Revision 1.4  1997/04/27 12:27:19  thrun
 * Extended parameter set (copied from SONARINT)
 *
 * Revision 1.3  1997/03/11 17:14:09  tyson
 * added IR simulation and other work
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






/*--- 'LASERINT_debug' prints out messages upon receipt of a TCX message ----*/
/*#define LASERINT_debug*/


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



#define TCX_LASERINT_MODULE_NAME "LASERINT"
#define TCX_LASERINT_SERVER_NAME "LASERINT-SERVER"


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define MAX_NUMBER_ROBOTS 10

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define is_front(i) ((i) < (robot_specifications->num_sensors / 2))
#define is_rear(i)  ((i) >= (robot_specifications->num_sensors / 2))

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define NUM_PATH_ITEMS 10000
extern float path_x[NUM_PATH_ITEMS];
extern float path_y[NUM_PATH_ITEMS];
extern float path_dist[NUM_PATH_ITEMS];
extern int   path_robot[NUM_PATH_ITEMS];
extern int num_path_items;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#define LASERINT_register_auto_format "string"

typedef struct{
  int   num_sensors;
  int   ignore_front_laser;
  int   ignore_rear_laser;
  float *sensor_values;
  float laser_x;
  float laser_y;
  float laser_orientation;
  float raw_odometry_x;
  float raw_odometry_y;
  float raw_odometry_orientation;
  float front_laser_offset_x;
  float front_laser_offset_y;
  float rear_laser_offset_x;
  float rear_laser_offset_y;
  int   force;
} LASERINT_scan_type, *LASERINT_scan_ptr;




#define LASERINT_scan_format "{int, int, int, <float : 1>, float, float, float, float, float, float, float, float, float, float, int}"

#define LASERINT_messages \
  {"LASERINT_register_auto_update", LASERINT_register_auto_format}, \
  {"LASERINT_scan",  LASERINT_scan_format}, \

extern TCX_MODULE_PTR SERVER;


/************************************************************************\
 ************************************************************************
\************************************************************************/

#define MAX_N_AUTO_UPDATE_MODULES 100

typedef struct{
  TCX_MODULE_PTR module;	/* pointer to TCX module */
  char           *name;
  int            scan;		/* 1=subscribed to regular scan updates */
  int            calibrated;	/* 1=user calibrated this one */
  int            active;

  float correction_parameter_x;	/* used for position correction */
  float correction_parameter_y;
  float correction_parameter_angle;
  int   correction_type;	/* rotation or translation correction */
} auto_update_type;

extern auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];


/************************************************************************\
 ************************************************************************
\************************************************************************/


#define RANDOM_MAX ((float)(2147483647))   /* (2**31 - 1) */
#define RAND_PLUS_MINUS_ONE() ((2.0 * ((float)(random()) / ((float) RANDOM_MAX))) - 1.0)
#define RAND_ZERO_TO_ONE() ((float)(random()) / ((float) RANDOM_MAX))



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#define LASERINT_NAME "laserint.dat"
#define LASERINT_INIT_NAME "laserint.ini"
#define SCRIPT_NAME "laserint.script"
#define PLOT_NAME "laserint.plot"
#define VR_NAME "laserint.wrl"
#define SMF_NAME "laserint.smf"
#define LOG_NAME "laserint.log"

extern FILE *log_iop;
extern char *robotname;


extern FILE *plot_iop;
extern FILE *vr_iop1;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define EXITERR(s) { fprintf(stderr, "ERROR: %s\n", s); exit(-1); }
#define WARNING(s) { fprintf(stderr, "WARNING: %s\n", s); }
#define COMMENT(s) { static int p = 0; if (!p) { fprintf(stderr, "\n\n\t### %s ###\n\n", s); p = 1; }}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define MAX_NUMBER_OBJECTS 100

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* GENERAL STATE VARIABLE TYPES */

typedef struct{
  float laser_x;
  float laser_y;
  float laser_orientation;
  float *sensor_values;
  char  *admissble_sensor_value;
  float *sensor_x;
  float *sensor_y;
  float *sensor_z;
  float *sensor_angle_to_base;
  float *sin_sensor_angle_to_base;
  float *cos_sensor_angle_to_base;
  float *dist_to_base;
  float hinge_x;
  float hinge_y;
  float **error;
  float **nearest_x;
  float **nearest_y;
  int   **nearest_index;
  float raw_odometry_laser_x;
  float raw_odometry_laser_y;
  float raw_odometry_laser_orientation;
  int   robot_number;		/* always -1 if not server */
} stack_item_type, *stack_item_ptr;



typedef struct{
  float x, y, z;
  float length, width, height;
  float angle;
} object_item_type, *object_item_ptr;



typedef struct{
  int   tcx_initialized;
  int   graphics_initialized;
  int   use_graphics;
  int   use_tcx;
  int   regular_local_map_display;
  int   regular_local_error_display;
  int   maps_allocated;
  int   base_connected;
  int   map_connected;
  int   server_connected;
  int   processing_script;
  float delay_in_replay;	/* delay between laser readings in script 
				 * in sec.*/

  int   map_update_pending;
  int   position_control_iteration;
  int   logging_on;
  int   force_logging_on;
  int   quit;

  int   first_stack_item;
  int   last_stack_item;
  stack_item_ptr stack;
  float remaining_error;
  int   data_count;

  int   strong_turn;
  int   found_nearest_stack_item;
  int   nearest_stack_item;
  int   cache;
  int   first_base_report_received;
  int   calibration;

  int   is_server;
  float offs_x, offs_y;

  int   object_matching_mode;
  int   number_objects;
  object_item_ptr objects[MAX_NUMBER_OBJECTS];

  int image_file_size_x;
  int image_file_size_y;
  int image_current_x;
  int image_most_recent_x;

  int preprocess_images;

} PROGRAM_STATE, *PROGRAM_STATE_PTR;



typedef struct{
  int   known;
  float x;
  float y;
  float orientation;
  float laser_x;
  float laser_y;
  float laser_orientation;
  float raw_odometry_x;
  float raw_odometry_y;
  float raw_odometry_orientation;
  float prev_laser_x;
  float prev_laser_y;
  float prev_laser_orientation;
  int   prev_laser_defined;
  float translation_between_scans;
  float rotation_between_scans_1;
  float rotation_between_scans_2;
  float translational_speed;
  float rotational_speed;
  float *sensor_values;
  float *sensor_endpoint_x;
  float *sensor_endpoint_y;
  int angle_to_wall_found;
  float wall_angle;

  float correction_parameter_x;	/* used for position correction */
  float correction_parameter_y;
  float correction_parameter_angle;
  int   correction_type;	/* rotation or translation correction */
} ROBOT_STATE, *ROBOT_STATE_PTR;



typedef struct{
  float local_mapsize_x;	/* size of the local window in cm */
  float local_mapsize_y;	/* size of the local window in cm */
  int   local_map_dim_x;	/* dimension of the local map in #fields */
  int   local_map_dim_y;	/* dimension of the local map in #fields */
  float resolution;		/* number of cm per displayed pixel */
  float pos_resolution;		/* resolution in position control */
  int   pos_map_dim;		/* position control: internal maps */
  int smooth_radius;		/* smoothing wondow size */

  float robot_size;		/* size of the robot in cm */
  float front_laser_offset_x;	/* where is the laser range finder? */
  float front_laser_offset_y;	/* Notice: robot moves in y direction */
  float rear_laser_offset_x;	/* the x-axis pointstowards its right */
  float rear_laser_offset_y;


  int   num_sensors;		/* number of sensors */
  float first_sensor_angle;	/* offset of the first sensor */
  float max_sensors_range;	/* maximum sensor range in cm */
  float min_sensors_range;	/* minimum sensor range in cm */
  float sensor_increment;	/* for LOCALIZE: add a small value to
				 * each sensor reading, which increases the
				 * freespace*/
  float neuronet_max_sensors_range; /* maximum sensor range in cm
				     * when training the neural net */
  float neuronet_min_sensors_range; /* maximum sensor range in cm
				     * when training the neural net */
  float max_occupied_sensors_range; /* cut-off range for occupied regions */
  float occupied_outer_width;	/* max width of obstacle */
  float network_occupied_value; /* cutt-off threshold */
  float *sensor_angles;		/* sensor angle values (vector) */
  float *sin_sensor_angles;	/* sensor angle values (vector) */
  float *cos_sensor_angles;	/* sensor angle values (vector) */
  float *aux_sin_sensor_angles;	/* sensor angle values (vector) */
  float *aux_cos_sensor_angles;	/* sensor angle values (vector) */

  int   use_neural_networks;	/* when set to 1, we'l use the neural nets */

  float network_value_mean;	/* where is the zero interpretation - the mean
				 * value for neural network predicitons? */
  float decay_with_distance;	/* If 1 laser interpretations will be
				 * weighted by the distance to the robot 
				 * If 0, they won't. In between: rate of 
				 * deradation */

  float standard_free;		/* likelihood for free, when neuro nets 
				 * aren't used */
  float standard_occupied;	/* likelihood for occupied, when neuro
				 * nets aren't used */
  float obstacle_width;		/* width of obstacles (when not neuro nets) */

  int   line_recognition_neighbors; /* number of neighbors to be considered
				     * for the recognition of lines */
  float line_recognition_threshold; /* threshold in recognition */
  float min_advancement_between_interpretations; /* How far shall the
						  * robot move, before we 
						  * consider a reading
						  * worth interpreting? */

  int   broadcast_sensor_data_to_map; /* this flag makes laserint forward
				      * sensor data to MAP */

  int ignore_odometry;

  int  stack_size;



  int   do_position_correction;
  float pos_corr_angle_margin;
  float pos_corr_max_dist;
  float pos_corr_max_angle;
  float pos_corr_max_range;
  float pos_corr_max_d_trans;
  float pos_corr_max_d_rot;
  float pos_corr_lrate_trans;
  float pos_corr_lrate_rot;
  int   pos_corr_min_num_iterations;
  int   pos_corr_max_num_iterations;
  float pos_corr_hinge_point_offset;
  float pos_corr_hinge_dist_threshold;
  float pos_corr_rot_cutoff_angle;

  int   pos_corr_do_est_drift;
  float pos_corr_drift_trans;
  float pos_corr_drift_rot;
  float pos_corr_lrate_drift;

  float pos_corr_max_dist_for_server;
  float pos_corr_max_angle_for_server;

  float angle_dev_threshold;
  float strong_turn_threshold;
  int   strong_turn_decay;

  int show_all_points;
  
  int send_corr_parameters_to_map;
  int reposition_robot_initially;

  int gif_image_frequency;
  int gif_image_size;

  int display_interval;
  int display_density;
  int display_fixed_rotation;
  float world_size;
  int   diagnose_step;

  int ignore_rear_laser;
  int ignore_front_laser;
  int plot_auto_on;

  int do_backwards_corrections;


  int generate_3D_vrml;
  int generate_3D_smf;
  int generate_3D_math;


} ROBOT_SPECIFICATIONS, *ROBOT_SPECIFICATIONS_PTR;




/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* NEURAL NETWORK VARIABLES */

typedef struct{
  int network_size[4];	/* ninputs,nhidden1/2, noutput for net */
  net_ptr net;		/* pointer to the network */
} NEURAL_NETWORK, *NEURAL_NETWORK_PTR;



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


typedef struct{
  PROGRAM_STATE_PTR        program_state;
  ROBOT_STATE_PTR          robot_state;
  ROBOT_SPECIFICATIONS_PTR robot_specifications;
  NEURAL_NETWORK_PTR       neural_network;
} ALL_TYPE, *ALL_PTR;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern PROGRAM_STATE            program_state_data;
extern ROBOT_STATE              robot_state_data;
extern ROBOT_SPECIFICATIONS     robot_specifications_data;
extern NEURAL_NETWORK           neural_network_data;
extern PROGRAM_STATE_PTR        program_state;
extern ROBOT_STATE_PTR          robot_state;
extern ROBOT_SPECIFICATIONS_PTR robot_specifications;
extern NEURAL_NETWORK_PTR       neural_network;
extern ALL_TYPE                 all;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* MAPS */

extern float *local_map;
extern int   *local_active;
extern int   *local_robot;
extern float *local_error;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern int new_laser_reading;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


extern int DISPLAY_LOCAL_MAPVALUES_BUTTON;
extern int DISPLAY_LOCAL_ERROR_BUTTON;
extern int LOCAL_ROBOT;
extern int LOCAL_BACKGROUND;
extern int LOCAL_MAPVALUES;
extern int LOCAL_ERROR;
extern int QUIT_BUTTON;
extern int SCRIPT_BUTTON;
extern int BASE_CONNECTED_BUTTON;
extern int MAP_CONNECTED_BUTTON;
extern int SERVER_CONNECTED_BUTTON;
extern int REGRESSION;
extern int OBSTACLES;
extern int MATCHES;
extern int PATH[MAX_NUMBER_ROBOTS];
extern int PREV_OBSTACLES;
extern int ERROR_DIAL;
extern int PLOT_BUTTON;
extern int EDIT_BUTTON_PLUS_ORIENT;
extern int EDIT_BUTTON_MINUS_ORIENT;
extern int EDIT_BUTTON_PLUS_X;
extern int EDIT_BUTTON_MINUS_X;
extern int EDIT_BUTTON_PLUS_Y;
extern int EDIT_BUTTON_MINUS_Y;
extern int SERVER_BUTTON;
extern int CLIENT_BUTTON;
extern int ROBOT_BUTTON[MAX_NUMBER_ROBOTS];
extern int OBJECTS[MAX_NUMBER_OBJECTS];
extern int OBJ_DIST;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern struct bParamList * bParamList;
extern const char *bRobotType;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


typedef struct {
  char *name;
  int size_x, size_y, size;
  unsigned char *red, *green, *blue;
} image_type, *image_ptr;

extern unsigned char *red;
extern unsigned char *green;
extern unsigned char *blue;

extern image_ptr bg_image;
extern image_ptr texture_image;

#define NORMAL_IMAGE_SIZE_X 720
#define NORMAL_IMAGE_SIZE_Y 480
#define PIXEL(x, y) (((y) * (NORMAL_IMAGE_SIZE_X)) + (x))


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

static inline float myatan2(float y, float x)
{
  if (x > 0.0) return atan(y / x);
  else if (x < 0.0){
    float z = y / x;
    if (z >= 0.0){
      if (y >= 0.0)
	return (M_PI - atan(z));
      else
	return (atan(z) - M_PI);
    }      
    else{ /* z < 0 */
      if (y >= 0.0)
	return (M_PI - atan(-z));
      else
	return (atan(-z) - M_PI);
    }
  }
  else{ /* x == 0.0 */
    if (y == 0.0)
      return 0.0;
    else{
      if (y >= 0.0)
	return (M_PI / 2.0);
      else
	return (- M_PI / 2.0);
    }
  }
}

static inline float d_atan2__d_x(float y, float x)
{
  return (-(y / ((x*x) + (y*y))));
}


static inline float d_atan2__d_y(float y, float x)
{
  return (x / ((x*x) + (y*y)));
}


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

float evaluate_network(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		       ROBOT_STATE_PTR robot_state,
		       net_ptr netX,
		       float distance, float angle, float prediction);

void compute_local_map(NEURAL_NETWORK_PTR neural_network,
		       ROBOT_SPECIFICATIONS_PTR robot_specifications,
		       PROGRAM_STATE_PTR  program_state,
		       ROBOT_STATE_PTR  robot_state);

void init_graphics(ROBOT_STATE_PTR    robot_state,
		   PROGRAM_STATE_PTR  program_state,
		   ROBOT_SPECIFICATIONS_PTR robot_specifications);

int mouse_test_loop(ROBOT_STATE_PTR          robot_state,
		    PROGRAM_STATE_PTR        program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications);

void check_commandline_parameters(int argc, char **argv, 
				  PROGRAM_STATE_PTR program_state);

void init_program(PROGRAM_STATE_PTR        program_state,
		  ROBOT_STATE_PTR          robot_state,
		  ROBOT_SPECIFICATIONS_PTR robot_specifications,
		  NEURAL_NETWORK_PTR       neural_network,
		  ALL_PTR                  all);

void allocate_everything(ROBOT_STATE_PTR    robot_state,
			 PROGRAM_STATE_PTR  program_state,
			 ROBOT_SPECIFICATIONS_PTR robot_specifications);

void read_init_file(ROBOT_STATE_PTR    robot_state,
		    PROGRAM_STATE_PTR  program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications);

void clear_all_maps(ROBOT_STATE_PTR    robot_state,
		    PROGRAM_STATE_PTR  program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications);

void init_network(NEURAL_NETWORK_PTR neural_network);

int save_parameters(char *filename);

int load_parameters(char *filename, int init);

int initiate_read_script(ROBOT_STATE_PTR    robot_state,
			 PROGRAM_STATE_PTR  program_state,
			 ROBOT_SPECIFICATIONS_PTR robot_specifications,
			 char *filename);

int read_script(ROBOT_STATE_PTR    robot_state,
		PROGRAM_STATE_PTR  program_state,
		ROBOT_SPECIFICATIONS_PTR robot_specifications);


void close_script(ROBOT_STATE_PTR    robot_state,
		  PROGRAM_STATE_PTR  program_state,
		  ROBOT_SPECIFICATIONS_PTR robot_specifications);

void BASE_update_status_reply_handler(TCX_REF_PTR                  ref,
				      BASE_update_status_reply_ptr status);


void
initiate_position_control(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			  PROGRAM_STATE_PTR program_state,
			  ROBOT_STATE_PTR robot_state);

void
do_position_control(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		    PROGRAM_STATE_PTR program_state,
		    ROBOT_STATE_PTR robot_state);

void
terminate_position_control(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			   PROGRAM_STATE_PTR program_state,
			   ROBOT_STATE_PTR robot_state);

inline int
correct_position(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		 ROBOT_STATE_PTR robot_state);

void
inform_clients(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	       PROGRAM_STATE_PTR program_state,
	       ROBOT_STATE_PTR robot_state, int robot_number);

void
temp_cache_reading(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		   PROGRAM_STATE_PTR program_state,
		   ROBOT_STATE_PTR robot_state);

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
		   int   robot_number);

void
cache_reading(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	      PROGRAM_STATE_PTR program_state,
	      ROBOT_STATE_PTR robot_state);

void
display_scans_and_map(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		      PROGRAM_STATE_PTR program_state,
		      ROBOT_STATE_PTR robot_state, int force_all);

void LASER_laser_reply_handler(TCX_REF_PTR           ref,
			       LASER_laser_reply_ptr laser);

void LASERINT_close_handler(char *name, TCX_MODULE_PTR module);

void broadcast_local_map_to_MAP(ROBOT_SPECIFICATIONS_PTR robot_specifications,
				PROGRAM_STATE_PTR        program_state,
				ROBOT_STATE_PTR          robot_state,
				int robot_number);

void connect_to_BASE(PROGRAM_STATE_PTR program_state);

void connect_to_MAP(PROGRAM_STATE_PTR program_state);

void connect_to_SERVER(PROGRAM_STATE_PTR program_state);

void init_tcx(PROGRAM_STATE_PTR program_state);

void smooth_local_map(ROBOT_SPECIFICATIONS_PTR robot_specifications);

int open_log(char *filename);

void close_log();

int angle_to_wall(float *lasers, /* vector */
		  float *laser_angles, /* vector */
		  float *angle); /* return value - parameter */


void
initiate_position_control(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			  PROGRAM_STATE_PTR program_state,
			  ROBOT_STATE_PTR robot_state);


void
do_position_control(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		    PROGRAM_STATE_PTR program_state,
		    ROBOT_STATE_PTR robot_state);


void
terminate_position_control(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			  PROGRAM_STATE_PTR program_state,
			  ROBOT_STATE_PTR robot_state);






void
cache_reading(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	      PROGRAM_STATE_PTR program_state,
	      ROBOT_STATE_PTR robot_state);


void
compute_polar_difference(float from_x, float from_y, float from_o,
			 float   to_x, float   to_y, float   to_o,
			 float *translation, float *rotation_1, 
			 float *rotation_2);

void
drift_adjust(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	     PROGRAM_STATE_PTR program_state,
	     ROBOT_STATE_PTR robot_state);
void
drift_adapt(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	    PROGRAM_STATE_PTR program_state,
	    ROBOT_STATE_PTR robot_state);

inline void
match_items(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	    ROBOT_STATE_PTR robot_state,
	    int display_flag,
	    stack_item_ptr s1, stack_item_ptr s2,
	    float *error1, float *dx1, float *dy1, float *dorientation1,
	    int   *count);

inline void 
update_position(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		ROBOT_STATE_PTR robot_state, stack_item_ptr s1, 
		float error1, float dx1, float dy1, float dorientation1,
		int count);

inline int
correct_position_phase1(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			ROBOT_STATE_PTR robot_state);

inline int
correct_position_phase2(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			ROBOT_STATE_PTR robot_state);

inline int
correct_position_phase3(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			ROBOT_STATE_PTR robot_state);

inline float
hinge_dist(stack_item_ptr s1, stack_item_ptr s2);

inline char
check_hinge_dist(stack_item_ptr s1, stack_item_ptr s2, float dist);

inline void
fast_linear_adjustment(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		       ROBOT_STATE_PTR robot_state,
		       float target_x, float target_y, 
		       float target_orientation);

int save_gif(char *filename);

int 
find_matching_scan(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		   PROGRAM_STATE_PTR program_state,
		   int *close_enough_match,
		   int *close_enough_cache);

void
send_correction_parameters_to_map(ROBOT_SPECIFICATIONS_PTR robot_specifications,
				  PROGRAM_STATE_PTR program_state,
				  ROBOT_STATE_PTR robot_state,
				  float corr_x, float corr_y, 
				  float corr_angle, int corr_type);
void
allocate_stack_item(ROBOT_STATE_PTR    robot_state,
		    PROGRAM_STATE_PTR  program_state,
		    ROBOT_SPECIFICATIONS_PTR robot_specifications,
		    int item_number);


void
plot_pos(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	 PROGRAM_STATE_PTR program_state,
	 ROBOT_STATE_PTR robot_state);

void
plot_on(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	 PROGRAM_STATE_PTR program_state,
	 ROBOT_STATE_PTR robot_state,
	 char *filename1, char *filename2, char *filename3);


void
plot_off(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	 PROGRAM_STATE_PTR program_state,
	 ROBOT_STATE_PTR robot_state);

void
send_stack_item_to_server(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			  PROGRAM_STATE_PTR  program_state,
			  ROBOT_STATE_PTR  robot_state, int force);

void 
LASERINT_register_auto_update_handler(TCX_REF_PTR ref,
				      char **data);

void 
LASERINT_scan_handler(TCX_REF_PTR ref, LASERINT_scan_ptr scan);

void 
count_auto_update_modules();

void 
activate_buttons();

int 
add_auto_update_module(TCX_MODULE_PTR module, int scan, char *robotname);


int
remove_auto_update_module(TCX_MODULE_PTR module);


void 
send_automatic_scan_update(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			   PROGRAM_STATE_PTR  program_state, 
			   LASERINT_scan_ptr scan,
			   float new_x, float new_y, float new_orientation, 
			   int info);


int
find_info(TCX_MODULE_PTR module);

float wall1x1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall1y1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall1x2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall1y2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall1a(float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall2x1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall2y1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall2x2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall2y2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall2a(float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall3x1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall3y1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall3x2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall3y2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall3a(float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall4x1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall4y1(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall4x2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall4y2(float xw, float yw, float lw, float ww, float aw,
	      float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wall4a(float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wallx1(int nr, float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wallx2(int nr, float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wally1(int nr, float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float wally2(int nr, float xw, float yw, float lw, float ww, float aw,
	     float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float walla(int nr, float xw, float yw, float lw, float ww, float aw,
	    float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float sensx(float xr, float yr, float or, float dr, float ar);


float sensy(float xr, float yr, float or, float dr, float ar);


float sensa(float xr, float yr, float or, float dr, float ar);


float distPointLineAux(float x, float y, float x1, float y1, float x2, float y2,
		       float *dx1, float *dy1, float *dx2, float *dy2);


float distPointLine(float x, float y, float x1, float y1, float x2, float y2,
		    float *dx1, float *dy1, float *dx2, float *dy2);


float distPointWall(float x, float y, int nr, float xw, float yw, float lw, float ww, float aw,
		    float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float distPointWallWithAngle(float x, float y, int nr, float xw, float yw, float lw, float ww, float aw, float xr, float yr, float or, float dr, float ar,
			     float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float distPointToObj(float x, float y, float xw, float yw, float lw, float ww, float aw, float xr, float yr, float or, float dr, float ar,
		     float *dxw, float *dyw, float *dlw, float *dww, float *daw);


float distSensorToObj(float xw, float yw, float lw, float ww, float aw, float xr, float yr, float or, float dr, float ar,
		      float *dxw, float *dyw, float *dlw, float *dww, float *daw);

image_ptr 
read_image(char *filename);

void
destroy_image(image_ptr *image);

void
find_bound(image_ptr image);

void
image_loop();

void
make_kernels();
int
find_kernel(float angle);

void
shift_image(image_ptr image, int dx, int dy);

void
add_ref_line_to_image(image_ptr image);

void
set_pixel(image_ptr image, int x, int y, 
	  unsigned char r, unsigned char g, unsigned char b, int size);

int
save_image(image_ptr image, char *filename);

image_ptr 
random_image();

void
fill_region_fraction(image_ptr image, 
		     float from_x, float to_x, 
		     float from_y, float to_y,
		     unsigned char red, unsigned char green,
		     unsigned char blue);



image_ptr 
make_image(int size_x, int size_y);


void
process_images();



void
edit(ROBOT_SPECIFICATIONS_PTR robot_specifications,
     PROGRAM_STATE_PTR  program_state,
     ROBOT_STATE_PTR  robot_state, 
     int button,
     float d_x, float d_y, float d_o);

