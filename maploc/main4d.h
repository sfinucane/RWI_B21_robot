
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Source file:     $Source: /usr/local/cvs/bee/src/maploc/main4d.h,v $
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
 * $Log: main4d.h,v $
 * Revision 1.1  2002/09/14 16:16:24  rstone
 * *** empty log message ***
 *
 * Revision 1.5  1999/04/18 19:00:13  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.4  1998/08/14 22:09:37  thrun
 * .
 *
 * Revision 1.3  1998/08/14 21:40:36  thrun
 * Version with sampling + trees
 *
 * Revision 1.2  1998/08/14 04:25:41  thrun
 * ??
 *
 * Revision 1.1  1998/08/14 03:51:32  thrun
 * intermediate version, don't use
 *
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
#define TURN_CONTROL_NOISE_FACTOR 2.5
/*#define TURN_CONTROL_NOISE_FACTOR 2.5*/
#define TRANSLATION_CONTROL_NOISE_FACTOR (0.1 * (TURN_CONTROL_NOISE_FACTOR))

/* amount of noise in perception: angular range and distance range */
#define OBS_ANGLE_ERROR_RANGE 5.0
#define OBS_DIST_ERROR_RANGE  5.0


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


/* Size of the X-Window */
#define X_WINDOW_SIZE 45.0

/* minmal likelihood */
#define MIN_LIKELIHOOD 0.0001

/* maximum tree depth */
#define MAX_TREE_DEPTH 9912
#define MIN_ITEMS_PER_NODE 20

/************************************************************************\
 ************************************************************************
\************************************************************************/




#define NUM_SAMPLE_POINTS 10000	/* sampling for locations */


#define VISUAL_RANGE 20.0
#define TARGET_RANGE 2.0
#define LANDMARK_PEAK 10.0

#define WORLD_SIZE 50.0
#define DISPLAY_SIZE 200

#define ROBOT_DRIFT 0.0		/* AMELIA: -0.002439  smithsonian new -0.0017
				 * in cm per degree, only for
				 * data read from file 
				 * */

#define NUM_MATRICES_PER_ROW 14	/* change this, if you want */


#define MATRIX_SIZE_X        2.0
#define MATRIX_SIZE_Y        (MATRIX_SIZE_X)
#define BUTTON_SIZE_Y        0.4
#define MATRIX_SEPARATOR     0.1
#define MAX_X                ((NUM_MATRICES_PER_ROW) * ((MATRIX_SIZE_X) + (MATRIX_SEPARATOR)))


#define NUM_WINDOWS_PER_DATA_ITEM 3


#define DUMP_FILENAME "maploc4d.dump"
#define INI_FILENAME "maploc4d.ini"

extern int num_items;

#define NUM_PATHS 4
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

struct node_type {
  float  min_x, max_x, min_y, max_y, min_o, max_o;
  float  volume;
  int    depth;
  int    splitting_variable;	/* 0=x, 1=y, 2=o */
  struct node_type *child[2];
  int    leaf;
  float  likelihood;
};

typedef struct {
  float x;
  float y;
  float orientation;
  float likelihood;
  float cumul_likelihood;
} single_sample_type;

typedef struct {
  single_sample_type data[2 * (NUM_SAMPLE_POINTS)];
  int num_samples;
  int method;
  int use_orientation;
  struct node_type *tree;
} samples_type;


/************************************************************************\
 ************************************************************************
\************************************************************************/


void
exit_proc(int garbage);


void
init_graphics();


void
register_density_window(float window_probs[NUM_WINDOWS_PER_DATA_ITEM]
			[DISPLAY_SIZE][DISPLAY_SIZE],
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
saw_landmark_carthesian(float rel_x, float rel_y);


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
robot_position_absolute(float correct_x,
			float correct_y,
			float correct_orientation,
			float measured_x,
			float measured_y,
			float measured_orientation);


void
robot_position_absolute_scaled(float correct_x,
			       float correct_y,
			       float correct_orientation,
			       float measured_x,
			       float measured_y,
			       float measured_orientation,
			       int   new_set, int position_known);


int 
mouse_test_loop();


void
update_density(int n, int density_number);


void
compute_map();


void
display_map();


void
update_intermediate_path_items();


int
load_raw_data(char *filename);


int
save_raw_data(char *filename);


void
save_data(char *filename);


int
load_data(char *filename);


void
print_statistics(FILE *iop, int init);


void 
forward_phase();


void 
backward_phase();


void 
integration_phase();


void
alarm_handler();


int
main(int argc, char *argv[]);

void
normalize_and_display_density(int n, int density_number, int normalize);

int 
select_random_sample(samples_type *samples);

void
demo_data();

void
straight_line_data();

void
square_data();

void
square_data_short();

void
back_and_forth_data();

float
compute_value(samples_type *samples, 
	      float tmp_x, 
	      float tmp_y, 
	      float tmp_o);


void
generate_node(samples_type *samples, 
	      struct node_type *node,
	      int *list, int list_length);


void
generate_tree(samples_type *samples);



void
destroy_node(struct node_type *node);

void
traverse_node(samples_type *samples, 
	      struct node_type *node,
	      float x, float y, float o, 
	      float *likelihood,
	      int *done);


