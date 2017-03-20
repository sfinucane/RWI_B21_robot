
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Source file:     $Source: /usr/local/cvs/bee/src/maploc/main3d.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 16:16:24 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: main3d.h,v $
 * Revision 1.1  2002/09/14 16:16:24  rstone
 * *** empty log message ***
 *
 * Revision 1.42  1999/04/18 19:00:12  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.41  1998/08/13 18:35:14  thrun
 * Window size is now declared in .h file.
 *
 * Revision 1.40  1998/08/09 04:39:16  thrun
 * Swapping of densities.
 *
 * Revision 1.38  1998/01/30 04:18:07  thrun
 * new version with drift parameter
 *
 * Revision 1.37  1998/01/25 01:16:01  thrun
 * *** empty log message ***
 *
 * Revision 1.36  1997/10/23 02:39:06  thrun
 * .
 *
 * Revision 1.35  1997/10/15 01:46:37  thrun
 * Intermediate version (unfinished) for multiple data sets.
 *
 * Revision 1.34  1997/10/08 19:44:03  thrun
 * fixed a seg-fault upon clicking in the demo window
 *
 * Revision 1.33  1997/10/08 12:46:50  thrun
 * .
 *
 * Revision 1.32  1997/10/07 21:13:51  thrun
 * Fixed another bug, whichy was recently introduced (~2 days ago) when I
 * reorganized the memory management. Symptom: Forward propagation
 * would not consider map/landmarks.
 *
 * Revision 1.31  1997/10/07 14:01:59  thrun
 * .
 *
 * Revision 1.30  1997/10/06 22:46:26  thrun
 * Fixed a serioue memory leak, saved additional space.
 *
 * Revision 1.29  1997/10/06 19:17:02  thrun
 * Fixed two REALLY BAD BUGS, one of which had to do with the initialization
 * of the forward_probs table. As a result, this table was partially
 * undefined, and the forward convolution was random.
 *
 * Revision 1.28  1997/10/06 12:41:51  thrun
 * less memory
 *
 * Revision 1.27  1997/10/05 22:16:49  thrun
 * File interface, fixed bugs, improved XWindows interface.
 *
 * Revision 1.26  1997/10/04 16:16:58  thrun
 * .
 *
 * Revision 1.25  1997/09/19 03:39:08  thrun
 * .
 *
 * Revision 1.24  1997/09/18 12:57:11  thrun
 * Improved the interpolation. Wean data set can now be plotted.
 *
 * Revision 1.23  1997/09/17 13:00:48  thrun
 * Fixed some stuff - the interpolation is still broken
 *
 * Revision 1.22  1997/09/17 01:25:08  thrun
 * Some intermediate version - I don't know.
 *
 * Revision 1.21  1997/09/15 22:43:01  thrun
 * Improved handling of the observation matrix: Probabilities cannot
 * get down to 0 any longer, which should help avoid
 * local minima.
 *
 * Revision 1.20  1997/09/15 14:15:27  thrun
 * *** This version "cracked" the Wean Hall mapping problem
 * *** (wean-hall-2-circles.dat) for the first time.
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

/* noise level */
#define TURN_CONTROL_NOISE_FACTOR 1.7
#define TRANSLATION_CONTROL_NOISE_FACTOR (0.3 * (TURN_CONTROL_NOISE_FACTOR))

/* optimization flag */
#define RUN_TIME_OPTIMIZE 1

/* selective updates */
#define LOWER_PROBABILITY_BOUND 0.001

/* max number of events */
#define MAX_NUM_STEPS 15000

/* observation matrix: lower probability bound */
#define MIN_OBS_PROB 0.000138888 

/* defalt scaling parameter, sets the resolution of the grid */
#define DEFAULT_ROBOT_POSITION_ABSOLUTE_MULTIPLIER 0.01

/* max number of iterations */
#define MAX_NUM_ITERATIONS 500

/* max number of total datasets */
#define MAX_NUM_DATASETS 20

/* if set to 1, densities are swapped onto file */
<<<<<<< main3d.h
#define SWAPPING 0

/* Size of the X-Window */
#define X_WINDOW_SIZE 40.0
=======
#define SWAPPING 0

/* Size of the X-Window */
#define X_WINDOW_SIZE 60.0
>>>>>>> 1.41

/************************************************************************\
 ************************************************************************
\************************************************************************/


#define TOLERANCE_THRESHOLD 0.001 /* for building the forward table,
				   * threshold under which two actions are
				   * considered the same */


<<<<<<< main3d.h
#define WORLD_SIZE_X 40	/* number of pixels */
=======
#define WORLD_SIZE_X 100	/* number of pixels */
>>>>>>> 1.41
#define WORLD_SIZE_Y (WORLD_SIZE_X)
#define NUMBER_ANGLES 72	/* 72 number of angles (resolution) */

#define ANGULAR_RESOLUTION (360.0 / ((float) (NUMBER_ANGLES)))
#define VISUAL_RANGE 10.0
#define TARGET_RANGE 2.0
#define LANDMARK_PEAK 10.0

#define ROBOT_DRIFT 0.005	/* AMELIA: -0.002439  smithsonian new -0.0017
				 * in cm per degree, only for
				 * data read from file 
				 * 
				 * */

#define NUM_MATRICES_PER_ROW 10	/* change this, if you want */


#define MATRIX_SIZE_X        2.0
#define MATRIX_SIZE_Y        (MATRIX_SIZE_X)
#define BUTTON_SIZE_Y        0.4
#define MATRIX_SEPARATOR     0.1
#define MAX_X                ((NUM_MATRICES_PER_ROW) * ((MATRIX_SIZE_X) + (MATRIX_SEPARATOR)))


#define NUM_WINDOWS_PER_DATA_ITEM 3


#define DUMP_FILENAME "maploc3d.dump"
#define INI_FILENAME "maploc3d.ini"

extern int num_items;

#define NUM_PATHS 5
#define NUM_PATHS2 ((NUM_PATHS) *2)

/************************************************************************\
 ************************************************************************
\************************************************************************/

extern float robot_position_absolute_multiplier;
extern float robot_position_absolute_offset_x[MAX_NUM_DATASETS];
extern float robot_position_absolute_offset_y[MAX_NUM_DATASETS];

/************************************************************************\
 ************************************************************************
\************************************************************************/

void
exit_proc(int garbage);


void
init_graphics();


void
register_density_window(float window_probs[NUM_WINDOWS_PER_DATA_ITEM]
			[WORLD_SIZE_X][WORLD_SIZE_Y],
			int *button_window_id,
			int probs_window_id[NUM_WINDOWS_PER_DATA_ITEM],
			int path_window_id[NUM_WINDOWS_PER_DATA_ITEM]
			[NUM_PATHS2]);


void
add_data_item(int new_episode,	/* if new episode, the "correct position"
				 * will be used, otherwise the translation/turn
				 * comands will be used */
	      int   position_known,
	      float correct_pos_x,
	      float correct_pos_y,
	      float correct_pos_orientation,
	      float correct_turn,
	      float correct_translation,
	      float error_turn,
	      float error_translation);


void
malloc_data(int num);

void
set_initial_position(float pos_x, 
		     float pos_y, 
		     float pos_orientation, 
		     int pos_known);


void
saw_landmark_polar(float rel_angle, float rel_distance);

void
saw_landmark_absolute(float obs_x, float obs_y);

void
saw_no_landmark();

void
robot_motion_polar(float correct_turn,
		   float correct_translation,
		   float error_turn,
		   float error_translation);

void
robot_motion_carthesian(float correct_x,
			float correct_y,
			float error_x,
			float error_y);

void
robot_position_absolute_scaled(float correct_x,
			       float correct_y,
			       float correct_orientation,
			       float measured_x,
			       float measured_y,
			       float measured_orientation,
			       int   new_set, int position_known);

void
robot_position_absolute(float correct_x,
			float correct_y,
			float correct_orientation,
			float measured_x,
			float measured_y,
			float measured_orientation);

int 
mouse_test_loop();


void
update_density(int n, int density_number);


void 
forward_phase();


void 
backward_phase();


void 
integration_phase();


int
main(int argc, char *argv[]);


void
find_motion_interval(float x,
		     float y,
		     float orientation,
		     float turn,
		     float translation,
		     float max_turn_error, /* must be nonnegative */
		     float max_translation_error, /* must be nonnegative */
		     int   *min_index_x,
		     int   *max_index_x,
		     int   *min_index_y,
		     int   *max_index_y,
		     int   *min_index_orientation,
		     int   *max_index_orientation);


void
compute_forward_factor(float turn,
		       float translation,
		       float **forward_prob_factor[NUMBER_ANGLES]
		       [NUMBER_ANGLES],
		       int  forward_prob_index_min_x[NUMBER_ANGLES]
		       [NUMBER_ANGLES],
		       int  forward_prob_index_max_x[NUMBER_ANGLES]
		       [NUMBER_ANGLES],
		       int   forward_prob_index_min_y[NUMBER_ANGLES]
		       [NUMBER_ANGLES],
		       int   forward_prob_index_max_y[NUMBER_ANGLES]
		       [NUMBER_ANGLES]);

void
compute_observation_matrix();

void
display_observation_matrix();

void
update_intermediate_path_items();


void
demo_data();

void
straight_line_data();

void
square_data();

void
back_and_forth_data();

int
load_raw_data(char *filename);

int
save_raw_data(char *filename);

int
load_data(char *filename);

void
save_data(char *filename);

void
print_statistics(FILE *iop, int init);

int
load_data(char *filename);

void
swap_out_density(int n, int density_number);

void
swap_in_density(int n, int density_number);
