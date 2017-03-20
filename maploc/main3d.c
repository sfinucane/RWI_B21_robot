
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Source file:     $Source: /usr/local/cvs/bee/src/maploc/main3d.c,v $
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
 * $Log: main3d.c,v $
 * Revision 1.1  2002/09/14 16:16:24  rstone
 * *** empty log message ***
 *
 * Revision 1.53  1999/04/18 19:00:12  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.52  1998/08/14 04:25:40  thrun
 * ??
 *
 * Revision 1.51  1998/08/14 03:51:32  thrun
 * intermediate version, don't use
 *
 * Revision 1.50  1998/08/13 18:35:13  thrun
 * Window size is now declared in .h file.
 *
 * Revision 1.49  1998/08/09 04:39:15  thrun
 * Swapping of densities.
 *
 * Revision 1.48  1998/08/09 02:37:46  thrun
 * .
 *
 * Revision 1.47  1998/01/30 04:18:07  thrun
 * new version with drift parameter
 *
 * Revision 1.46  1998/01/25 01:16:00  thrun
 * *** empty log message ***
 *
 * Revision 1.45  1997/10/23 02:49:22  thrun
 * .
 *
 * Revision 1.44  1997/10/15 01:46:36  thrun
 * Intermediate version (unfinished) for multiple data sets.
 *
 * Revision 1.43  1997/10/09 02:28:17  thrun
 * .
 *
 * Revision 1.42  1997/10/08 19:44:02  thrun
 * fixed a seg-fault upon clicking in the demo window
 *
 * Revision 1.41  1997/10/08 12:46:49  thrun
 * .
 *
 * Revision 1.40  1997/10/07 22:53:44  thrun
 * improved graphics (big demo window)
 *
 * Revision 1.39  1997/10/07 21:13:51  thrun
 * Fixed another bug, whichy was recently introduced (~2 days ago) when I
 * reorganized the memory management. Symptom: Forward propagation
 * would not consider map/landmarks.
 *
 * Revision 1.38  1997/10/07 14:01:59  thrun
 * .
 *
 * Revision 1.37  1997/10/06 23:47:11  thrun
 * saved even more memory
 *
 * Revision 1.36  1997/10/06 22:46:25  thrun
 * Fixed a serioue memory leak, saved additional space.
 *
 * Revision 1.35  1997/10/06 19:17:01  thrun
 * Fixed two REALLY BAD BUGS, one of which had to do with the initialization
 * of the forward_probs table. As a result, this table was partially
 * undefined, and the forward convolution was random.
 *
 * Revision 1.34  1997/10/06 12:41:50  thrun
 * less memory
 *
 * Revision 1.33  1997/10/05 22:16:48  thrun
 * File interface, fixed bugs, improved XWindows interface.
 *
 * Revision 1.32  1997/10/04 16:16:58  thrun
 * .
 *
 * Revision 1.31  1997/09/19 03:39:08  thrun
 * .
 *
 * Revision 1.30  1997/09/18 12:57:10  thrun
 * Improved the interpolation. Wean data set can now be plotted.
 *
 * Revision 1.29  1997/09/17 13:00:48  thrun
 * Fixed some stuff - the interpolation is still broken
 *
 * Revision 1.28  1997/09/17 01:25:06  thrun
 * Some intermediate version - I don't know.
 *
 * Revision 1.27  1997/09/15 22:43:00  thrun
 * Improved handling of the observation matrix: Probabilities cannot
 * get down to 0 any longer, which should help avoid
 * local minima.
 *
 * Revision 1.26  1997/09/15 14:13:55  thrun
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


#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <signal.h>


#include "EZX11.h"
#include "o-graphics.h"
#include "dat.h"
#include "main3d.h"

/************************************************************************\
 ************************************************************************
\************************************************************************/

/*
 * Statistics only
 */

int iteration = 0;
float num_of_floats_allocated[(MAX_NUM_ITERATIONS)+1];
float num_of_non_zero_floats[(MAX_NUM_ITERATIONS)+1];
float num_floats_bound[(MAX_NUM_ITERATIONS)+1];
int   num_entities[(MAX_NUM_ITERATIONS)+1];
struct timeval computation_time[(MAX_NUM_ITERATIONS)+1];

int helphelp = 0;
int button_alarm = 0;

/************************************************************************\
 ************************************************************************
\************************************************************************/


int OBS_PROBS;
int QUIT_BUTTON;
int PATH_DISPLAY[3][NUM_PATHS2];
int ALL_PROBS;
int DEMO_PROBS;
int PATH_DISPLAY_BUTTON[NUM_PATHS];


#define uniform() (1.0 / ((float) ((WORLD_SIZE_X) * (WORLD_SIZE_Y) * (NUMBER_ANGLES))))

#define truncint(x) ((int) (x+0.5))

int run_time_optimize = RUN_TIME_OPTIMIZE;
int current_dataset_no = 0;

/************************************************************************\
 ************************************************************************
\************************************************************************/



float robot_position_absolute_multiplier = 
DEFAULT_ROBOT_POSITION_ABSOLUTE_MULTIPLIER;
int   robot_position_absolute_offset_defined[MAX_NUM_DATASETS];
float robot_position_absolute_offset_x[MAX_NUM_DATASETS];
float robot_position_absolute_offset_y[MAX_NUM_DATASETS];

char *save_file_name = NULL;

/************************************************************************\
 ************************************************************************
\************************************************************************/


/* 
 * The following data type is used for the probability tables
 * in state estimation. Not every data item will have such a table
 */

typedef struct {
  /* probability tables */
  float **probs[NUM_WINDOWS_PER_DATA_ITEM][NUMBER_ANGLES];
  float max_prob;
  int   non_zero_index_x[NUM_WINDOWS_PER_DATA_ITEM][WORLD_SIZE_X];
  int   non_zero_index_y[NUM_WINDOWS_PER_DATA_ITEM][WORLD_SIZE_Y];
  int   non_zero_index_o[NUM_WINDOWS_PER_DATA_ITEM][NUMBER_ANGLES];
  int   min_non_zero_index_x[NUM_WINDOWS_PER_DATA_ITEM];
  int   min_non_zero_index_y[NUM_WINDOWS_PER_DATA_ITEM];
  int   max_non_zero_index_x[NUM_WINDOWS_PER_DATA_ITEM];
  int   max_non_zero_index_y[NUM_WINDOWS_PER_DATA_ITEM];
  int   internal_dimension_x[NUM_WINDOWS_PER_DATA_ITEM];
  int   internal_dimension_y[NUM_WINDOWS_PER_DATA_ITEM];
  int   swapped_out[NUM_WINDOWS_PER_DATA_ITEM];
  /* update information */
  float **forward_prob_factor[NUMBER_ANGLES][NUMBER_ANGLES];
  int   forward_prob_index_min_x[NUMBER_ANGLES][NUMBER_ANGLES];
  int   forward_prob_index_max_x[NUMBER_ANGLES][NUMBER_ANGLES];
  int   forward_prob_index_min_y[NUMBER_ANGLES][NUMBER_ANGLES];
  int   forward_prob_index_max_y[NUMBER_ANGLES][NUMBER_ANGLES];
  /* display information */
  int   button_window_id;
  int   path_window_id[NUM_WINDOWS_PER_DATA_ITEM][NUM_PATHS2];
  float window_probs[NUM_WINDOWS_PER_DATA_ITEM][WORLD_SIZE_X][WORLD_SIZE_Y];
  int   probs_window_id[NUM_WINDOWS_PER_DATA_ITEM];
} data_type;

/* 
 * general data type for items (corresponds to a new robot position)
 */

typedef struct {
  /* motion information */
  int   new_episode;
  int   position_known;
  int   dataset_no;
  float correct_pos_x;
  float correct_pos_y;
  float correct_pos_orientation;
  float measured_pos_x;
  float measured_pos_y;
  float measured_pos_orientation;
  float best_pos_x;
  float best_pos_y;
  float best_pos_orientation;
  float correct_turn;
  float correct_translation;
  float error_turn;
  float error_translation;
  float measured_turn;
  float measured_translation;
  float measured_turn_since_last_estimate;
  float measured_translation_since_last_estimate;
  float measured_pseudo_orientation_since_last_estimate;
  float correct_heading;
  float measured_heading;
  float best_heading;
  int   landmark_found;
  float landmark_rel_angle;
  float landmark_rel_distance;
  float landmark_measured_x;
  float landmark_measured_y;
  /* probabilistic data */
  data_type *data;		/* NULL, if none */
  int   prev_data_item;
  int   next_data_item;
} item_type;

item_type item[MAX_NUM_STEPS];
int   num_items = 0;
int   first_data_item = -1;
int   last_data_item  = -1;

float obs_probs[WORLD_SIZE_X][WORLD_SIZE_Y];
float all_probs[WORLD_SIZE_X][WORLD_SIZE_Y];
float demo_probs[WORLD_SIZE_X][WORLD_SIZE_Y];
float temp_probs[NUMBER_ANGLES][WORLD_SIZE_X][WORLD_SIZE_Y];

/************************************************************************\
 ************************************************************************
\************************************************************************/

/*
 * The following code is only useful for debugging.
 */

#define check_forward(a)  \
  { \
    int x, y, k, n, d, l; \
    float v; \
    for (n = 0; n < num_items; n++) \
      if (item[n].data) \
	for (k = 0; k < NUMBER_ANGLES; k++) \
	  for (l = 0; l < NUMBER_ANGLES; l++) \
	    if (item[n].data->forward_prob_factor[k][l]) \
	      for (x = 0; x <= item[n].data->forward_prob_index_max_x[k][l] \
		     - item[n].data->forward_prob_index_min_x[k][l]; x++) \
		for (y = 0; y <= item[n].data->forward_prob_index_max_y[k][l] \
		       - item[n].data->forward_prob_index_min_y[k][l]; y++){ \
		  v = item[n].data->forward_prob_factor[k][l][x][y]; \
		  if (!(v >= 0 && v <= 1.0)) \
		    fprintf(stderr,  \
			    "NAN: forw(%d) n=%d k=%d l=%d x=%d y=%d: %g\n", \
			    a, n, k, l, x, y, v); \
		} \
  }


#define check_items(a) \
  { \
    int x, y, k, n, d, l; \
    float v; \
    for (n = 0; n < num_items; n++) \
      if (item[n].data) \
	for (d = 0; d < NUM_WINDOWS_PER_DATA_ITEM; d++) \
	  for (k = 0; k < NUMBER_ANGLES; k++) \
	    if (item[n].data->probs[d][k]) \
	      for (x = 0; x <= item[n].data->max_non_zero_index_x[d] - \
		     item[n].data->min_non_zero_index_x[d]; x++) \
		for (y = 0; y <= item[n].data->max_non_zero_index_y[d] - \
		       item[n].data->min_non_zero_index_y[d]; y++){ \
		  v = item[n].data->probs[d][k][x][y]; \
		  if (!(v >= 0 && v <= 1.0)) \
		    fprintf(stderr, \
			    "NAN: prob(%d) n=%d d=%d k=%d x=%d y=%d: %g\n", \
			    a, n, d, k, x, y, v); \
		} \
  }




#define check_temp(a) \
  { \
    int x, y, k, n, d, l; \
    float v; \
    fprintf(stderr, "|"); \
    for (k = 0; k < NUMBER_ANGLES; k++) \
      for (x = 0; x < WORLD_SIZE_X; x++) \
	for (y = 0; y < WORLD_SIZE_Y; y++){ \
	  v = temp_probs[k][x][y]; \
	  if (!(v >= 0 && v <= 1.0)) \
	    fprintf(stderr, "NAN: temp(%d) k=%d x=%d y=%d: %g\n", \
		    a, k, x, y, v); \
	} \
  }




/************************************************************************\
 ************************************************************************
\************************************************************************/



/************************************************************************
 *
 *   NAME:         exit_proc()
 *                 
 *   FUNCTION:     Handler for ^C interrupts
 *                 
 *   PARAMETERS:   none
 *                 
 *                 
 ************************************************************************/


void
exit_proc(int garbage)
{
  fprintf(stderr, "\n");
  exit(0);
}





/************************************************************************\
 ************************************************************************
\************************************************************************/




/************************************************************************
 *
 *   NAME:         init_graphics()
 *                 
 *   FUNCTION:     initializes the graphics window
 *                 
 *   PARAMETERS:   int global_use_X    indicates, whether we really use X-Win.
 *                 
 *                 
 *   RETURN-PARAM: none yet
 *                 
 *
 *   RETURN-VALUE: none yet
 *                 
 ************************************************************************/


void
init_graphics()
{
  /*
  static char *myfonts[] = {"5x8", "6x10", "7x13bold", 
			    "9x15bold", "10x20", "12x24",
			    "lucidasans-bold-24"};
			    */
  static char *myfonts[] = {"5x8", "5x8", "5x8", 
			    "9x15bold", "10x20", "12x24",
			    "lucidasans-bold-24"};
   
  int i, j;
  
  
  G_initialize_graphics("MAP-LOC", X_WINDOW_SIZE, 10.0, C_TURQUOISE4); 
  G_set_matrix_display_style(1);
  G_initialize_fonts(7, myfonts);
  G_markers_display_style = 0;	/* 0 = circles, not filled */
  G_invert_display = 1;

  for (i = 0; i < WORLD_SIZE_X; i++)
    for (j = 0; j < WORLD_SIZE_Y; j++){
      obs_probs[i][j] = uniform();
      all_probs[i][j] = uniform();
      demo_probs[i][j] = uniform();
    }

  for (i = 0; i < MAX_NUM_DATASETS; i++){
    robot_position_absolute_offset_x[i] = 0.0;
    robot_position_absolute_offset_y[i] = 0.0;
    robot_position_absolute_offset_defined[i] = 0;
  }


  {
   
    /******** QUIT_BUTTON ****************************/
    int switch_num                      = 2;
    static float switch_pos[]           = 
    {0.0, (((float) ((NUM_WINDOWS_PER_DATA_ITEM)+1)) * (MATRIX_SIZE_X))
     + (((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SEPARATOR)),
     -(BUTTON_SIZE_Y), 0.0};
    static char *switch_texts[]         =
    {"QUIT", "QUIT"};
    static int switch_fonts[]           = {2,2};
    static int switch_background_color[]= {C_GREY90, C_RED};
    static int switch_frame_color[]     = {C_GREY30, C_GREY70};
    static int switch_text_color[]      = {C_BLACK, C_WHITE};

    QUIT_BUTTON
      = G_create_switch_object(switch_pos, switch_num, switch_texts,
			       switch_background_color,switch_frame_color, 
			       switch_text_color, switch_fonts);
  }

  {
    /******** DEMO_PROBS ****************************/
    float  matrix_pos[4]                     = 
    {0.0,
     (((float) (NUM_WINDOWS_PER_DATA_ITEM)) * (MATRIX_SIZE_X))
     + (((float) ((NUM_WINDOWS_PER_DATA_ITEM)-1)) * (MATRIX_SEPARATOR)),
     - (BUTTON_SIZE_Y)
     - (((float) (NUM_WINDOWS_PER_DATA_ITEM)) *
	((MATRIX_SIZE_Y)+(MATRIX_SEPARATOR))),
     - (BUTTON_SIZE_Y) - (MATRIX_SEPARATOR)};
    static char *matrix_text             = "Oops - should not be here";
    static int matrix_font               = 0;
    /*static int matrix_colors[]         = {NO_COLOR, C_GREY30, NO_COLOR}; */
    static int matrix_colors[]           = 
    {NO_COLOR, C_DARKSLATEGREY, NO_COLOR}; 

    
    DEMO_PROBS =
      G_create_matrix_object(matrix_pos, matrix_text, 
			     &(demo_probs[0][0]),
			     NULL, WORLD_SIZE_X, WORLD_SIZE_Y,
			     0.0, 1.0, matrix_colors, matrix_font);
  }

  {
    /******** OBS_PROBS ****************************/
    float  matrix_pos[4]                     = 
    {(((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SIZE_X))
     + (((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SEPARATOR)),
     (((float) ((NUM_WINDOWS_PER_DATA_ITEM)+1)) * (MATRIX_SIZE_X))
     + (((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SEPARATOR)),
     -((BUTTON_SIZE_Y)+(MATRIX_SIZE_Y)+(MATRIX_SEPARATOR)),
     -((BUTTON_SIZE_Y)+(MATRIX_SEPARATOR))};
    static char *matrix_text             = "Oops - should not be here";
    static int matrix_font               = 0;
    /*static int matrix_colors[]         = {NO_COLOR, C_GREY30, NO_COLOR}; */
    static int matrix_colors[]           = 
    {NO_COLOR, C_DARKSLATEGREY, NO_COLOR}; 

    
    OBS_PROBS =
      G_create_matrix_object(matrix_pos, matrix_text, 
			     &(obs_probs[0][0]),
			     NULL, WORLD_SIZE_X, WORLD_SIZE_Y,
			     0.0, 1.0, matrix_colors, matrix_font);
  }

  {
    /******** ALL_PROBS ****************************/
    float  matrix_pos[4]                     = 
    {(((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SIZE_X))
     + (((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SEPARATOR)),
     (((float) ((NUM_WINDOWS_PER_DATA_ITEM)+1)) * (MATRIX_SIZE_X))
     + (((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SEPARATOR)),
     -((BUTTON_SIZE_Y)+(2*((MATRIX_SIZE_Y)+(MATRIX_SEPARATOR)))),
     -((BUTTON_SIZE_Y)+(MATRIX_SIZE_Y)+(2*(MATRIX_SEPARATOR)))};
    static char *matrix_text             = "Oops - should not be here";
    static int matrix_font               = 0;
    /*static int matrix_colors[]         = {NO_COLOR, C_GREY30, NO_COLOR}; */
    static int matrix_colors[]           = 
    {NO_COLOR, C_DARKSLATEGREY, NO_COLOR}; 

    
    ALL_PROBS =
      G_create_matrix_object(matrix_pos, matrix_text, 
			     &(all_probs[0][0]),
			     NULL, WORLD_SIZE_X, WORLD_SIZE_Y,
			     0.0, 1.0, matrix_colors, matrix_font);
  }


  {

    /******** PATH_DISPLAY_BUTTON ****************************/
    int switch_num                      = 2;
    static float switch_pos[]           =
    {(((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SIZE_X))
     + (((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SEPARATOR)),
     (((float) ((NUM_WINDOWS_PER_DATA_ITEM)+1)) * (MATRIX_SIZE_X))
     + (((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SEPARATOR)),
     0.0, -((BUTTON_SIZE_Y)+2*((MATRIX_SIZE_Y))+(3*(MATRIX_SEPARATOR)))};
    static char *switch_texts[]         =
    {"landmarks (off)", "landmarks (on)",
     "measured (off)", "measured (on)",
     "correct (off)", "correct (on)",
     "guessed (off)", "guessed (on)",
     "b-box (off)", "b-box (on)"};
    static int switch_fonts[]           = {1,1,1,1,1};
    static int switch_background_color[]= {C_GREY90, C_YELLOW};
    static int switch_frame_color[]     = {C_GREY30, C_GREY70};
    static int switch_text_color[]      = {C_BLACK, C_BLACK};
    int i;


    if (NUM_PATHS != 5){
      fprintf(stderr, "Something is wrong with NUM_PATHS\n");
      exit(-1);
    }

    for (i = 0; i < NUM_PATHS; i++){
      switch_pos[2] = 
	switch_pos[3] - 
	(((MATRIX_SIZE_Y) - ((((float) NUM_PATHS)-1.0) *
			     (MATRIX_SEPARATOR))) / ((float) NUM_PATHS));

      PATH_DISPLAY_BUTTON[i]    
	= G_create_switch_object(switch_pos, switch_num, 
				 &(switch_texts[i*2]),
				 switch_background_color,switch_frame_color, 
				 switch_text_color, switch_fonts);
      switch_pos[3] = switch_pos[2] - (MATRIX_SEPARATOR);
    }
  }



  {
    /******** PATH_DISPLAY ****************************/
    int i, j, k;
    float  mar_pos0[4]                     = 
    {0.0,
     (((float) (NUM_WINDOWS_PER_DATA_ITEM)) * (MATRIX_SIZE_X))
     + (((float) ((NUM_WINDOWS_PER_DATA_ITEM)-1)) * (MATRIX_SEPARATOR)),
     - (BUTTON_SIZE_Y)
     - (((float) (NUM_WINDOWS_PER_DATA_ITEM)) *
	((MATRIX_SIZE_Y)+(MATRIX_SEPARATOR))),
     - (BUTTON_SIZE_Y) - (MATRIX_SEPARATOR)};
    float  mar_pos1[4]                     = 
    {(((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SIZE_X))
     + (((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SEPARATOR)),
     (((float) ((NUM_WINDOWS_PER_DATA_ITEM)+1)) * (MATRIX_SIZE_X))
     + (((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SEPARATOR)),
     -((BUTTON_SIZE_Y)+(MATRIX_SIZE_Y)+(MATRIX_SEPARATOR)),
     -((BUTTON_SIZE_Y)+(MATRIX_SEPARATOR))};
    float  mar_pos2[4]                     = 
    {(((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SIZE_X))
     + (((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SEPARATOR)),
     (((float) ((NUM_WINDOWS_PER_DATA_ITEM)+1)) * (MATRIX_SIZE_X))
     + (((float) ((NUM_WINDOWS_PER_DATA_ITEM))) * (MATRIX_SEPARATOR)),
     -((BUTTON_SIZE_Y)+(2*((MATRIX_SIZE_Y)+(MATRIX_SEPARATOR)))),
     -((BUTTON_SIZE_Y)+(MATRIX_SIZE_Y)+(2*(MATRIX_SEPARATOR)))};
    int num_mar                            = 1;
    static char *text_mar[]                = {""};
    static int mar_frame_color             = NO_COLOR;
    static int mar_background_color        = NO_COLOR;
    static int mar_foreground_color[]      =
    {C_BLUE, C_LAWNGREEN, C_RED, C_CADETBLUE, C_SIENNA4};
    static int mar_text_color[]            = {NO_COLOR};
    static int mar_fonts[]                 = {2};
   
    if (NUM_PATHS != 5){
      fprintf(stderr, "Something is wrong with NUM_PATHS\n");
      exit(-1);
    }

    for (i = 0, j = 0; i < NUM_PATHS; i++){
      
      PATH_DISPLAY[0][j] =
	G_create_markers_object(mar_pos0, 1, 0.0,
				0.0, ((float) WORLD_SIZE_X),
				0.0, ((float) WORLD_SIZE_Y),
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, &(mar_foreground_color[i]), 
				  mar_text_color, mar_fonts);
      G_deactivate(PATH_DISPLAY[0][j]);

      PATH_DISPLAY[1][j] =
	G_create_markers_object(mar_pos1, 1, 0.0,
				0.0, ((float) WORLD_SIZE_X),
				0.0, ((float) WORLD_SIZE_Y),
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, &(mar_foreground_color[i]), 
				  mar_text_color, mar_fonts);
      G_deactivate(PATH_DISPLAY[1][j]);

      PATH_DISPLAY[2][j] =
	G_create_markers_object(mar_pos2, 1, 0.0,
				0.0, ((float) WORLD_SIZE_X),
				0.0, ((float) WORLD_SIZE_Y),
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, &(mar_foreground_color[i]), 
				  mar_text_color, mar_fonts);
      G_deactivate(PATH_DISPLAY[2][j++]);

      PATH_DISPLAY[0][j] =
	G_create_markers_object(mar_pos0, 0, 
				0.022222222222 * ((float) (WORLD_SIZE_X)),
				0.0, ((float) WORLD_SIZE_X),
				0.0, ((float) WORLD_SIZE_Y),
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, &(mar_foreground_color[i]), 
				mar_text_color, mar_fonts);
      G_deactivate(PATH_DISPLAY[0][j]);

      PATH_DISPLAY[1][j] =
	G_create_markers_object(mar_pos1, 0, 
				0.022222222222 * ((float) (WORLD_SIZE_X)),
				0.0, ((float) WORLD_SIZE_X),
				0.0, ((float) WORLD_SIZE_Y),
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, &(mar_foreground_color[i]), 
				mar_text_color, mar_fonts);
      G_deactivate(PATH_DISPLAY[1][j]);

      PATH_DISPLAY[2][j] =
	G_create_markers_object(mar_pos2, 0, 
				0.022222222222 * ((float) (WORLD_SIZE_X)),
				0.0, ((float) WORLD_SIZE_X),
				0.0, ((float) WORLD_SIZE_Y),
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, &(mar_foreground_color[i]), 
				mar_text_color, mar_fonts);
      G_deactivate(PATH_DISPLAY[2][j++]);
    }
  }
}



/************************************************************************
 *
 *   NAME:         register_density_window
 *                 
 *   FUNCTION:     Auxiliary graphical routine, which constructs the
 *                 various density display windows (and what goes with it)
 *                 
 *                 
 ************************************************************************/

void
register_density_window(float window_probs[NUM_WINDOWS_PER_DATA_ITEM]
			[WORLD_SIZE_X][WORLD_SIZE_Y],
			int *button_window_id,
			int probs_window_id[NUM_WINDOWS_PER_DATA_ITEM],
			int path_window_id[NUM_WINDOWS_PER_DATA_ITEM]
			[NUM_PATHS2])
{
  static float current_x = 
    ((NUM_WINDOWS_PER_DATA_ITEM) + 1) * ((MATRIX_SEPARATOR) + (MATRIX_SIZE_X));
  static float current_y = 0.0;
  static int num_x = (NUM_WINDOWS_PER_DATA_ITEM) + 1;
  float size_x, size_y;
  static char *matrix_text            = "Oops - should not be here";
  static int matrix_font              = 0;
  /*static int matrix_colors[]        = {NO_COLOR, C_GREY30, NO_COLOR}; */
  static int matrix_colors[]          = {NO_COLOR, C_DARKSLATEGREY, NO_COLOR}; 
  float  matrix_pos[4];
  int num_mar                            = 5;
  static char *text_mar[]                = {"", "", "", "", ""};
  static int mar_frame_color             = NO_COLOR;
  static int mar_background_color        = NO_COLOR;
  static int mar_foreground_color[]      = 
  {C_RED, C_LAWNGREEN, C_BLUE, C_TURQUOISE4, C_SIENNA4};
  static int mar_text_color[]            = 
  {NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR};
  static int mar_fonts[]                 = {2, 2, 2, 2, 2};
  int switch_num                      = 4;
  static char *switch_texts[]         = {"", "> > >", "< < <", "| | |"};
  static int switch_fonts[]           = {2,2,2,2};
  static int switch_background_color[]= 
  {C_GREY90, C_RED, C_RED, C_RED};
  static int switch_frame_color[]     = 
  {C_GREY30, C_GREY70, C_GREY70, C_GREY70};
  static int switch_text_color[]      = 
  {C_BLACK, C_WHITE, C_WHITE, C_WHITE};
  static char *text_mar2[]                = {""};
  static int mar2_frame_color             = NO_COLOR;
  static int mar2_background_color        = NO_COLOR;
  static int mar2_foreground_color[]      = 
  {C_BLUE, C_LAWNGREEN, C_RED, C_CADETBLUE, C_SIENNA4};
  static int mar2_text_color[]            = {NO_COLOR};
  static int mar2_fonts[]                 = {2};
  int i, j, k;


  size_x = (MATRIX_SEPARATOR) + (MATRIX_SIZE_X);
  size_y = (((MATRIX_SEPARATOR) + (MATRIX_SIZE_Y))
	    * (NUM_WINDOWS_PER_DATA_ITEM)) + 
    (MATRIX_SEPARATOR) + (BUTTON_SIZE_Y);
  
  if (num_x >= NUM_MATRICES_PER_ROW){
    current_y -= size_y;
    current_x = 0.0;
    num_x = 0;
  }

  
  matrix_pos[0] = current_x;
  matrix_pos[1] = current_x + MATRIX_SIZE_X;
  matrix_pos[2] = current_y;
  matrix_pos[3] = current_y;

  current_x += size_x;
  num_x++;

  matrix_pos[2] = matrix_pos[3] - (BUTTON_SIZE_Y);
  *button_window_id
    = G_create_switch_object(matrix_pos, switch_num, switch_texts,
			     switch_background_color,switch_frame_color, 
			     switch_text_color, switch_fonts);
  matrix_pos[3] = matrix_pos[2] - (MATRIX_SEPARATOR);


   
  for (i = 0; i < NUM_WINDOWS_PER_DATA_ITEM; i++){
    matrix_pos[2] = matrix_pos[3] - (MATRIX_SIZE_Y);
    probs_window_id[i] = 
      G_create_matrix_object(matrix_pos, matrix_text, 
			     &(window_probs[i][0][0]), 
			     NULL, WORLD_SIZE_X, WORLD_SIZE_Y,
			     0.0, 1.0, matrix_colors, matrix_font);


    G_matrix_set_display_range(probs_window_id[i], 0.0, uniform());
    
    
    for (k = 0, j = 0; k < NUM_PATHS; k++){
      path_window_id[i][j] =
	G_create_markers_object(matrix_pos, 1, 0.0,
				0.0, ((float) WORLD_SIZE_X),
				0.0, ((float) WORLD_SIZE_Y),
				1, text_mar2, mar2_background_color, 
				mar2_frame_color, 
				&(mar2_foreground_color[k]), 
				mar2_text_color, mar2_fonts);
      G_deactivate(path_window_id[i][j++]);

      path_window_id[i][j] =
	G_create_markers_object(matrix_pos, 0,
				0.022222222222 * ((float) (WORLD_SIZE_X)),
				0.0, ((float) WORLD_SIZE_X),
				0.0, ((float) WORLD_SIZE_Y),
				1, text_mar2, mar2_background_color, 
				mar2_frame_color,
				&(mar2_foreground_color[k]), 
				mar2_text_color, mar2_fonts);
      G_deactivate(path_window_id[i][j++]);
    }
      

    matrix_pos[3] = matrix_pos[2] - (MATRIX_SEPARATOR);
  }
}


/************************************************************************
 *
 *   NAME:         add_data_item
 *                 
 *   FUNCTION:     General internal routine for the registration
 *                 of robot motion. Do not call at user level
 *                 
 *                 
 ************************************************************************/



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
	      float error_translation)
{
  int n, found;
  int verbose = 0;

  if (num_items >= MAX_NUM_STEPS){
    fprintf(stderr, 
	    "ERROR: Memory overflow. Change MAX_NUM_STEPS in main3d.h\n");
    exit(-1);
  }

  if (!new_episode && num_items == 0){
    fprintf(stderr, "ERROR: Data must begin with a new episode.\n");
    exit(-1);
  }

  if (new_episode && num_items != 0){
    current_dataset_no++;
    if (current_dataset_no >= MAX_NUM_DATASETS){
      fprintf(stderr, "ERROR: Too many datasets: %d.\n", current_dataset_no);
      fprintf(stderr, "Increase MAX_NUM_DATASETS\n");
      exit(-1);
    }
  }


  /* copy the various values */
  item[num_items].dataset_no                  = current_dataset_no;
  item[num_items].data                        = NULL;
  item[num_items].new_episode                 = new_episode;
  item[num_items].position_known              = position_known;
  item[num_items].next_data_item              = -1; /* none yet */
  item[num_items].prev_data_item              = -1; /* none yet */
  for (n = num_items - 1, found = 0; n >= 0 && !found; n--)
    if (item[n].data){
      item[num_items].prev_data_item          = n;
      found = 1;
    }
  

  item[num_items].correct_heading             = 0.0;
  item[num_items].measured_heading            = 0.0;
  item[num_items].best_heading                = 0.0;


  if (new_episode){
    item[num_items].correct_turn              = 0.0;
    item[num_items].correct_translation       = 0.0;
    item[num_items].error_turn                = 0.0;
    item[num_items].error_translation         = 0.0;
    item[num_items].correct_pos_x             = correct_pos_x;
    item[num_items].correct_pos_y             = correct_pos_y;
    item[num_items].correct_pos_orientation   = correct_pos_orientation;
    item[num_items].measured_pos_x            = correct_pos_x;
    item[num_items].measured_pos_y            = correct_pos_y;
    item[num_items].measured_pos_orientation  = correct_pos_orientation;
    item[num_items].measured_turn             = 0.0;
    item[num_items].measured_translation      = 0.0;
    item[num_items].measured_turn_since_last_estimate        = 0.0;
    item[num_items].measured_translation_since_last_estimate = 0.0;
    item[num_items].measured_pseudo_orientation_since_last_estimate =
      correct_pos_orientation;


    while (item[num_items].correct_pos_orientation >= 360.0)
      item[num_items].correct_pos_orientation -= 360.0;
    while (item[num_items].correct_pos_orientation < 0.0)
      item[num_items].correct_pos_orientation += 360.0;
    while (item[num_items].measured_pos_orientation >= 360.0)
      item[num_items].measured_pos_orientation -= 360.0;
    while (item[num_items].measured_pos_orientation < 0.0)
      item[num_items].measured_pos_orientation += 360.0;
    while (item[num_items].measured_pseudo_orientation_since_last_estimate
	   >= 360.0)
      item[num_items].measured_pseudo_orientation_since_last_estimate -=
	360.0;
    while (item[num_items].measured_pseudo_orientation_since_last_estimate <
	   0.0)
      item[num_items].measured_pseudo_orientation_since_last_estimate += 
	360.0;

  }
  else{
    if (item[num_items].prev_data_item == -1){
      fprintf(stderr, "Unusual internal error - is initial pos defined?\n");
      exit(-1);
    }
    item[num_items].correct_turn              = correct_turn;
    item[num_items].correct_translation       = correct_translation;
    item[num_items].error_turn                = error_turn;
    item[num_items].error_translation         = error_translation;
    item[num_items].measured_turn             = correct_turn + error_turn;
    item[num_items].measured_translation      = 
      correct_translation + error_translation;

    item[num_items].correct_pos_orientation   = 
      item[num_items-1].correct_pos_orientation + correct_turn;
    while (item[num_items].correct_pos_orientation >= 360.0)
      item[num_items].correct_pos_orientation -= 360.0;
    while (item[num_items].correct_pos_orientation < 0.0)
      item[num_items].correct_pos_orientation += 360.0;
    item[num_items].correct_pos_x             = 
      item[num_items-1].correct_pos_x + 
      (cos(item[num_items].correct_pos_orientation * M_PI / 180.0) *
       correct_translation);
    item[num_items].correct_pos_y             = 
      item[num_items-1].correct_pos_y + 
      (sin(item[num_items].correct_pos_orientation * M_PI / 180.0) *
       correct_translation);

    item[num_items].measured_pos_orientation   = 
      item[num_items-1].measured_pos_orientation + correct_turn + error_turn;
    while (item[num_items].measured_pos_orientation >= 360.0)
      item[num_items].measured_pos_orientation -= 360.0;
    while (item[num_items].measured_pos_orientation < 0.0)
      item[num_items].measured_pos_orientation += 360.0;
    item[num_items].measured_pos_x             = 
      item[num_items-1].measured_pos_x + 
      (cos(item[num_items].measured_pos_orientation * M_PI / 180.0) *
       (correct_translation + error_translation));
    item[num_items].measured_pos_y             = 
      item[num_items-1].measured_pos_y + 
      (sin(item[num_items].measured_pos_orientation * M_PI / 180.0) *
       (correct_translation + error_translation));

    if (item[num_items].measured_pos_x == 
	item[item[num_items].prev_data_item].measured_pos_x &&
	item[num_items].measured_pos_y == 
	item[item[num_items].prev_data_item].measured_pos_y){
      item[num_items].measured_turn_since_last_estimate = 0.0;
      item[num_items].measured_translation_since_last_estimate = 0.0;
      item[num_items].measured_pseudo_orientation_since_last_estimate =
	item[item[num_items].prev_data_item].
	measured_pseudo_orientation_since_last_estimate;
    }
    else{
      item[num_items].measured_turn_since_last_estimate =
	(atan2(item[num_items].measured_pos_y - 
	       item[item[num_items].prev_data_item].measured_pos_y,
	       item[num_items].measured_pos_x - 
	       item[item[num_items].prev_data_item].measured_pos_x)
	 * 180.0 / M_PI) - 
	item[item[num_items].prev_data_item].
	measured_pseudo_orientation_since_last_estimate;
      while (item[num_items].measured_turn_since_last_estimate > 180.0)
	item[num_items].measured_turn_since_last_estimate -= 360.0;
      while (item[num_items].measured_turn_since_last_estimate <= -180.0)
	item[num_items].measured_turn_since_last_estimate += 360.0;

      item[num_items].measured_pseudo_orientation_since_last_estimate =
	item[item[num_items].prev_data_item].
	measured_pseudo_orientation_since_last_estimate +
	item[num_items].measured_turn_since_last_estimate;
      while (item[num_items].measured_pseudo_orientation_since_last_estimate
	     > 180.0)
	item[num_items].measured_pseudo_orientation_since_last_estimate 
	  -= 360.0;
      while (item[num_items].measured_pseudo_orientation_since_last_estimate 
	     <= -180.0)
	item[num_items].measured_pseudo_orientation_since_last_estimate 
	  += 360.0;

      item[num_items].measured_translation_since_last_estimate = 
	sqrt(((item[num_items].measured_pos_x - 
	       item[item[num_items].prev_data_item].measured_pos_x) *
	      (item[num_items].measured_pos_x - 
	       item[item[num_items].prev_data_item].measured_pos_x)) +
	     ((item[num_items].measured_pos_y - 
	       item[item[num_items].prev_data_item].measured_pos_y) *
	      (item[num_items].measured_pos_y - 
	       item[item[num_items].prev_data_item].measured_pos_y)));
    }
  }
  if (verbose){
    fprintf(stderr, "\n--------------------------------------------------\n");
    fprintf(stderr, "num_items                                  %d\n",
	    num_items);
    fprintf(stderr, "measured_pos_x                             %g\n",
	    item[num_items].measured_pos_x);
    fprintf(stderr, "measured_pos_y                             %g\n",
	    item[num_items].measured_pos_y);
    fprintf(stderr, "measured_pos_orientation                   %g\n",
	    item[num_items].measured_pos_orientation);
    fprintf(stderr, "measured_turn                              %g\n",
	    item[num_items].measured_turn);
    fprintf(stderr, "measured_translation                       %g\n",
	    item[num_items].measured_translation);
    fprintf(stderr, "measured_turn_since_last_estimate          %g\n",
	    item[num_items].measured_turn_since_last_estimate);
    fprintf(stderr, "measured_pseudo_orientation_since_last_est %g\n",
	    item[num_items].measured_pseudo_orientation_since_last_estimate);
    fprintf(stderr, "measured_translation_since_last_estimate   %g\n",
	    item[num_items].measured_translation_since_last_estimate);
    fprintf(stderr, "prev_data_item                             %d\n",
	    item[num_items].prev_data_item);
    fprintf(stderr, "PREV: measured_pos_x                       %g\n",
	    item[item[num_items].prev_data_item].measured_pos_x);
    fprintf(stderr, "PREV: measured_pos_y                       %g\n",
	    item[item[num_items].prev_data_item].measured_pos_y);
    fprintf(stderr, "PREV: measured_pos_orientation             %g\n",
	    item[item[num_items].prev_data_item].measured_pos_orientation);
    fprintf(stderr, "PREV: meas_pseudo_orient_since_last_est    %g\n",
	    item[item[num_items].prev_data_item].
	    measured_pseudo_orientation_since_last_estimate);
  }

  item[num_items].best_pos_x = item[num_items].measured_pos_x;
  item[num_items].best_pos_y = item[num_items].measured_pos_y;
  item[num_items].best_pos_orientation = 
    item[num_items].measured_pos_orientation;


  item[num_items].landmark_found            = 0;
  item[num_items].landmark_rel_angle        = 0.0;
  item[num_items].landmark_rel_distance     = 0.0;
  item[num_items].landmark_measured_x       = 0.0;
  item[num_items].landmark_measured_y       = 0.0;

  if (new_episode || position_known)
    malloc_data(num_items);

  num_items++;

}


/************************************************************************
 *
 *   NAME:         malloc_data
 *                 
 *   FUNCTION:     Allocates and initializes the various distributions etc
 *                 
 *                 
 ************************************************************************/



void
malloc_data(int num)
{
  int i, j, k, l, n, found;
  float min_x, min_y, max_x, max_y, min_o, max_o;
  data_type *data;


  if (item[num].data != NULL)
    return;

  /* allocate data and set pointers to previous/next data item */
  item[num].data = (data_type *) malloc(sizeof(data_type));
  if (item[num].data == NULL){
    fprintf(stderr, "Out of memory (3).\n");
    exit(-1);
  }
  else
    fprintf(stderr, "Data item %d: allocated %d bytes.\n", 
	    num, sizeof(data_type));

  data = item[num].data;
  


  if (first_data_item == -1)
    first_data_item              = num;
  item[num].next_data_item = -1; /* none yet */
  item[num].prev_data_item = -1; /* none yet */
  last_data_item                 = num;
  for (i = num - 1, found = 0; i >= 0 && !found; i--)
    if (item[i].data){
      item[i].next_data_item = num;
      item[num].prev_data_item = i;
      found = 1;
    }


  /* initialize density functions */

  for (i = 0; i < NUM_WINDOWS_PER_DATA_ITEM; i++)
    for (k = 0; k < NUMBER_ANGLES; k++)
      data->probs[i][k] = NULL;	/* NULL stands for "0.0" */

  data->max_prob = 0.0;


  for (i = 0; i < NUM_WINDOWS_PER_DATA_ITEM; i++)
    data->swapped_out[i] = 0;

  for (j = 0; j < NUM_WINDOWS_PER_DATA_ITEM; j++){
    for (i = 0; i < WORLD_SIZE_X; i++)
      data->non_zero_index_x[j][i] = 0;
    for (i = 0; i < WORLD_SIZE_Y; i++)
      data->non_zero_index_y[j][i] = 0;
    for (i = 0; i < NUMBER_ANGLES; i++)
      data->non_zero_index_o[j][i] = 0;
    data->min_non_zero_index_x[j] = WORLD_SIZE_X;
    data->min_non_zero_index_y[j] = WORLD_SIZE_Y;
    data->max_non_zero_index_x[j] = -1;
    data->max_non_zero_index_y[j] = -1;
    data->internal_dimension_x[j] = 0;
    data->internal_dimension_y[j] = 0;
  }  

  /* compute forward probability table */

  if (!item[num].new_episode){

    for (n = 0, found = 0; n < num && !found; n++)
      if (item[n].data &&
	  fabs(item[n].measured_turn_since_last_estimate -
	       item[num].measured_turn_since_last_estimate) <=
	  TOLERANCE_THRESHOLD &&
	  fabs(item[n].measured_translation_since_last_estimate -
	       item[num].measured_translation_since_last_estimate) <=
	  TOLERANCE_THRESHOLD)
	found = 1;

    if (!found)
      compute_forward_factor(item[num].
			     measured_turn_since_last_estimate,
			     item[num].
			     measured_translation_since_last_estimate,
			     data->forward_prob_factor,
			     data->forward_prob_index_min_x,
			     data->forward_prob_index_max_x,
			     data->forward_prob_index_min_y,
			     data->forward_prob_index_max_y);
    else{
      n--;
      for (i = 0; i < NUMBER_ANGLES; i++)
	for (j = 0; j < NUMBER_ANGLES; j++){
	  data->forward_prob_factor[i][j] =
	    item[n].data->forward_prob_factor[i][j];
	  data->forward_prob_index_min_x[i][j] =
	    item[n].data->forward_prob_index_min_x[i][j];
	  data->forward_prob_index_max_x[i][j] =
	    item[n].data->forward_prob_index_max_x[i][j];
	  data->forward_prob_index_min_y[i][j] =
	    item[n].data->forward_prob_index_min_y[i][j];
	  data->forward_prob_index_max_y[i][j] =
	    item[n].data->forward_prob_index_max_y[i][j];
	}
    }
  }



  /* register windows */
  
  register_density_window(data->window_probs,
			  &(data->button_window_id),
			  data->probs_window_id,
			  data->path_window_id);
}



/************************************************************************
 *
 *   NAME:         set_initial_position
 *                 
 *   FUNCTION:     Routine that marks the beginning of a new episide
 *                 
 *                 
 *                 
 ************************************************************************/


void
set_initial_position(float pos_x, 
		     float pos_y, 
		     float pos_orientation, 
		     int pos_known)
{
  add_data_item(1, pos_known, pos_x, pos_y, pos_orientation, 
		0.0, 0.0, 0.0, 0.0);
}



/************************************************************************
 *
 *   NAME:         saw_landmark_polar
 *                 
 *   FUNCTION:     User-level routine that indicates the event of observing a
 *                 landmark relative to the robot
 *                 
 *                 
 ************************************************************************/

void
saw_landmark_polar(float rel_angle, float rel_distance)
{


  if (num_items == 0){
    fprintf(stderr, "Define position first, before observing landmark.\n");
    exit(-1);
  }


  if (rel_distance > VISUAL_RANGE){
    fprintf(stderr, "ERROR: Cannot see that far: %g %g\n",
	    rel_angle, rel_distance);
    exit(-1);
  }

  if (item[num_items-1].data == NULL)
    malloc_data(num_items-1);

  item[num_items-1].landmark_found            = 1;
  item[num_items-1].landmark_rel_angle        = rel_angle;
  item[num_items-1].landmark_rel_distance     = rel_distance;
  item[num_items-1].landmark_measured_x       = 
    item[num_items-1].measured_pos_x +
    (cos((item[num_items-1].measured_pos_orientation + 
	  rel_angle) * M_PI / 180.0) * rel_distance);
  item[num_items-1].landmark_measured_y       = 
    item[num_items-1].measured_pos_y +
    (sin((item[num_items-1].measured_pos_orientation + 
	  rel_angle) * M_PI / 180.0) * rel_distance);

}


/************************************************************************
 *
 *   NAME:         saw_landmark_carthesian
 *                 
 *   FUNCTION:     User-level routine that indicates the event of observing a
 *                 landmark relative to the robot
 *                 
 *                 
 ************************************************************************/

void
saw_landmark_carthesian(float rel_x, float rel_y)
{
  float rel_angle, rel_distance;

  if (num_items == 0){
    fprintf(stderr, "Define position first, before observing landmark.\n");
    exit(-1);
  }

  if (item[num_items-1].data == NULL)
    malloc_data(num_items-1);


  rel_distance = sqrt((rel_x * rel_x) + (rel_y * rel_y));

  if (rel_distance > VISUAL_RANGE){
    fprintf(stderr, "ERROR: Cannot see that far: %g\n", rel_distance);
    exit(-1);
  }

  rel_angle = (atan2(rel_y, rel_x) * 180.0 / M_PI)
    - item[num_items-1].measured_pos_orientation;
  while (rel_angle <= -180.0) rel_angle += 360.0;
  while (rel_angle >   180.0) rel_angle -= 360.0;
  
  
  item[num_items-1].landmark_found            = 1;
  item[num_items-1].landmark_rel_angle        = rel_angle;
  item[num_items-1].landmark_rel_distance     = rel_distance;
  item[num_items-1].landmark_measured_x       = 
    item[num_items-1].measured_pos_x +
    (cos((item[num_items-1].measured_pos_orientation + 
	  rel_angle) * M_PI / 180.0) * rel_distance);
  item[num_items-1].landmark_measured_y       = 
    item[num_items-1].measured_pos_y +
    (sin((item[num_items-1].measured_pos_orientation + 
	  rel_angle) * M_PI / 180.0) * rel_distance);
}



/************************************************************************
 *
 *   NAME:         saw_landmark_absolute
 *                 
 *   FUNCTION:     User-level routine that indicates the event of observing a
 *                 landmark, in absolute coordinates
 *                 
 *                 
 ************************************************************************/

void
saw_landmark_absolute(float obs_x, float obs_y)
{
  
  saw_landmark_carthesian(obs_x - item[num_items-1].measured_pos_x,
			  obs_y - item[num_items-1].measured_pos_y);
}


/************************************************************************
 *
 *   NAME:         saw_no_landmark
 *                 
 *   FUNCTION:     User-level routine that indicates the event of
 *                 not observng a landmark. This routine is used to 
 *                 incorporate a state into position estimation, even if
 *                 no landmark war observed
 *                 
 ************************************************************************/

void
saw_no_landmark()
{
  
  if (item[num_items-1].data == NULL)
    malloc_data(num_items-1);

}



/************************************************************************
 *
 *   NAME:         robot_motion_polar
 *                 
 *   FUNCTION:     User-level routine for specifying robot motion
 *                 
 *                 
 ************************************************************************/

void
robot_motion_polar(float correct_turn,
		   float correct_translation,
		   float error_turn,
		   float error_translation)
{
  add_data_item(0, 0, 0.0, 0.0, 0.0,
		correct_turn, correct_translation,
		error_turn, error_translation);
}




/************************************************************************
 *
 *   NAME:         robot_motion_carthesian
 *                 
 *   FUNCTION:     User-level routine for specifying robot motion
 *                 Internally, this routine translates carthesian
 *                 motion into polar coordinates
 *                 
 ************************************************************************/

void
robot_motion_carthesian(float correct_x,
			float correct_y,
			float error_x,
			float error_y)
{
  float correct_turn;
  float correct_translation;
  float error_turn;
  float error_translation;

  if (num_items == 0){
    fprintf(stderr, "ERROR: Data must begin with a new episode.\n");
    return;
  }

  correct_translation = 
    sqrt((correct_x * correct_x) + (correct_y * correct_y));

  correct_turn = (atan2(correct_y, correct_x) * 180.0 / M_PI)
    - item[num_items-1].correct_pos_orientation;
  while (correct_turn <= -180.0) correct_turn += 360.0;
  while (correct_turn >   180.0) correct_turn -= 360.0;
  
 
  error_translation = 
    sqrt(((correct_x + error_x) * (correct_x + error_x))
	 + ((correct_y + error_y) * (correct_y + error_y)));
  error_translation -= correct_translation;
  
  error_turn = (atan2(correct_y + error_y, correct_x + error_x) * 180.0 / M_PI)
    - item[num_items-1].measured_pos_orientation;
  error_turn -= correct_turn;
  while (error_turn <= -180.0) error_turn += 360.0;
  while (error_turn >   180.0) error_turn -= 360.0;

  /*
  fprintf(stderr, 
	  "Carthesian motion, correct: x:%g y:%g -> turn:%g trans:%g\n",
	  correct_x, correct_y, correct_turn, correct_translation);
  fprintf(stderr,
	  "                   error  : x:%g y:%g -> turn:%g trans:%g\n",
	  error_x, error_y, error_turn, error_translation);
  */

  add_data_item(0, 0, 0.0, 0.0, 0.0,
		correct_turn, correct_translation,
		error_turn, error_translation);
}



/************************************************************************
 *
 *   NAME:         robot_position_absolute
 *                 
 *   FUNCTION:     User-level routine for specifying robot motion
 *                 Internally, this routine translates carthesian
 *                 motion into polar coordinates
 *                 
 ************************************************************************/

void
robot_position_absolute(float correct_x,
			float correct_y,
			float correct_orientation,
			float measured_x,
			float measured_y,
			float measured_orientation)
{

  if (num_items == 0)
    set_initial_position(correct_x, correct_y, 
			 correct_orientation, 1);

  else
    robot_motion_carthesian(correct_x - item[num_items-1].correct_pos_x,
			    correct_y - item[num_items-1].correct_pos_y,
			    (measured_x - item[num_items-1].measured_pos_x)
			    - (correct_x - item[num_items-1].correct_pos_x),
			    (measured_y - item[num_items-1].measured_pos_y)
			    - (correct_y - item[num_items-1].correct_pos_y));

  item[num_items-1].correct_heading             = correct_orientation;
  item[num_items-1].measured_heading            = measured_orientation;
  item[num_items-1].best_heading                = measured_orientation;

}


/************************************************************************
 *
 *   NAME:         robot_position_absolute_scaled
 *                 
 *   FUNCTION:     User-level routine for specifying robot motion
 *                 Internally, this routine translates carthesian
 *                 motion into polar coordinates
 *                 
 *                 This version scales and shifts the coordinates
 *                 
 *                 
 ************************************************************************/

void
robot_position_absolute_scaled(float correct_x,
			       float correct_y,
			       float correct_orientation,
			       float measured_x,
			       float measured_y,
			       float measured_orientation,
			       int   new_set, int position_known)
{

  float correct_x_adjusted;
  float correct_y_adjusted;
  float measured_x_adjusted;
  float measured_y_adjusted;
  

  if (!robot_position_absolute_offset_defined[current_dataset_no] ||
      current_dataset_no >= MAX_NUM_DATASETS ||
      current_dataset_no < 0){
    fprintf(stderr, "ERROR: Internal error: %d",
	    current_dataset_no);
    exit(-1);
  }

  /*
   * scales the coordinates
   */

  correct_x_adjusted = 
    (correct_x * robot_position_absolute_multiplier)
    + robot_position_absolute_offset_x[current_dataset_no];
  correct_y_adjusted = 
    (correct_y * robot_position_absolute_multiplier)
    + robot_position_absolute_offset_y[current_dataset_no];
  measured_x_adjusted = 
    (measured_x * robot_position_absolute_multiplier)
    + robot_position_absolute_offset_x[current_dataset_no];
  measured_y_adjusted = 
    (measured_y * robot_position_absolute_multiplier)
    + robot_position_absolute_offset_y[current_dataset_no];

  if (new_set)
    set_initial_position(correct_x_adjusted,
			 correct_y_adjusted,
			 correct_orientation,
			 position_known);
  else
    robot_position_absolute(correct_x_adjusted,
			    correct_y_adjusted,
			    correct_orientation,
			    measured_x_adjusted,
			    measured_y_adjusted,
			    measured_orientation);
}


/************************************************************************
 *
 *   NAME:         mouse_test_loop
 *                 
 *   FUNCTION:     Checks mouse events and changes the variables
 *                 "action" and "Program_state" correspondingly
 *                 
 *                 
 ************************************************************************/




int 
mouse_test_loop()
{
  
  G_mouse_ptr mouse_events;
  int num_mouse_events, button;
  float mouse_x, mouse_y, x, y;
  int number, type;
  int test, found, i, j, n;
  static int update_pending = 0;
  float max, *probs_ptr = NULL;
  int *paths = NULL;
  int   display_demo = 0;

  found = 0;



  /****************** CHECK FOR MOUSE EVENT *******************/

  test = G_test_mouse(0);

  if (test == 1){
    mouse_events = G_evaluate_mouse(&mouse_x, &mouse_y, 
				    &button, &num_mouse_events);




    
    /****************** EVALUATE MOUSE EVENT *******************/
    
    /*
     * Buttons that switch on/off path displays
     */

    for (i = 0; i < NUM_PATHS && !found; i++)
      if (G_mouse_event_at(PATH_DISPLAY_BUTTON[i], mouse_events, &number)){
	G_display_switch(PATH_DISPLAY_BUTTON[i], 
			 mouse_events[number].actual_text ^ 1);	
	if (!mouse_events[number].actual_text){
	  for (n = 0; n < 3; n++){
	    G_activate(PATH_DISPLAY[n][2*i]);
	    G_activate(PATH_DISPLAY[n][2*i+1]);
	  }
	  for (n = 0; n < num_items; n++)
	    if (item[n].data)
	      for (j = 0; j < NUM_WINDOWS_PER_DATA_ITEM; j++){
		G_activate(item[n].data->path_window_id[j][2*i]);
		G_activate(item[n].data->path_window_id[j][2*i+1]);
	      }
	}
	else{
	  for (n = 0; n < 3; n++){
	    G_deactivate(PATH_DISPLAY[n][2*i]);
	    G_deactivate(PATH_DISPLAY[n][2*i+1]);
	  }
	  for (n = 0; n < num_items; n++)
	    if (item[n].data)
	      for (j = 0; j < NUM_WINDOWS_PER_DATA_ITEM; j++){
		G_deactivate(item[n].data->path_window_id[j][2*i]);
		G_deactivate(item[n].data->path_window_id[j][2*i+1]);
	      }
	}
	found = 1;
	update_pending = 1;
      }

    /*
     * Copying values into the large demo display
     */

    if (!found && G_mouse_event_at(DEMO_PROBS, mouse_events, &number)){
      probs_ptr = NULL;
      paths     = NULL;
      display_demo = 1;
      found = 1;
    }

    if (!found && G_mouse_event_at(OBS_PROBS, mouse_events, &number)){
      probs_ptr = &(obs_probs[0][0]);
      paths     = &(PATH_DISPLAY[1][0]);
      display_demo = 1;
      found = 1;
    }

    if (!found && G_mouse_event_at(ALL_PROBS, mouse_events, &number)){
      probs_ptr = &(all_probs[0][0]);
      paths     = &(PATH_DISPLAY[2][0]);
      display_demo = 1;
      found = 1;
    }

    for (n = 0; !found && n < num_items; n++)
      if (item[n].data)
	for (i = 0; i < NUM_WINDOWS_PER_DATA_ITEM; i++)
	  if (G_mouse_event_at(item[n].data->probs_window_id[i], 
			       mouse_events, &number)){
	    probs_ptr = &(item[n].data->window_probs[i][0][0]);
	    paths     = &(item[n].data->path_window_id[i][0]);
	    display_demo = 1;
	    found = 1;
	  }
    

    
    if (display_demo){
      if (probs_ptr)
	max = 0.0;
      else
	max = 1.0;
      for (i = 0; i < WORLD_SIZE_X; i++)
	for (j = 0; j < WORLD_SIZE_Y; j++){
	  if (probs_ptr)
	    demo_probs[i][j] = *probs_ptr++;
	  else
	    demo_probs[i][j] = 0.0;
	  if (demo_probs[i][j] > max)
	    max = demo_probs[i][j];
	}
      G_matrix_set_display_range(DEMO_PROBS, 0.0, max);
      G_display_matrix(DEMO_PROBS);

      for (i = 0; i < NUM_PATHS2; i++){
	if (paths){
	  G_clear_markers(PATH_DISPLAY[0][i]);
	  for (j = 0; j < G_return_num_markers(paths[i], 1); j++){
	    G_return_marker_coordinates(paths[i], j, &x, &y, &type);
	    G_add_marker(PATH_DISPLAY[0][i], x, y, type);
	  }
	}
	G_display_markers(PATH_DISPLAY[0][i]);
      }
    }

    /*
     * all others
     */ 

    if (found){
    }

    else if (G_mouse_event_at(QUIT_BUTTON, mouse_events, &number)){
      G_display_switch(QUIT_BUTTON, 1);
      usleep(200000);
      fprintf(stderr, "\n");
      exit(0);
    }
    /* insert new graphics object here XXXXX */
    
    
    else if (button == RIGHT_BUTTON){

      G_display_all();
      update_pending = 0;
      helphelp = 1;
    }
  }
  
  else if (update_pending){
    update_pending++;
    if (update_pending >= 13){
      G_display_all();
      update_pending = 0;
    }
  }
  
  return test;
}






/************************************************************************
 *
 *   NAME:         compute_forward_factor
 *                 
 *   FUNCTION:     computes a probability table that models the
 *                 motion of the robot
 *                 
 *                 
 ************************************************************************/



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
		       [NUMBER_ANGLES])
{
  int index_x, index_y, index_to_o, index_from_o, phase, num_x, num_y, i, j;
  float delta_x, delta_y, o_from, o_to, original_angle;
  float error_translation, error_turn1, error_turn2, error_turn;
  float factor_turn, factor_translation, help;
  int found = 0, min_volume = 0, actual_volume = 0;
  int bytes_allocated = 0;
  int bytes_allocated_local;
  int verbose = 0;


   /* fprintf(stderr, " [[%g %g]] ", turn, translation);  */


  /* G_display_switch(item[n].data->button_window_id, 3); */

  for (index_from_o = 0; index_from_o < NUMBER_ANGLES; index_from_o++){
    o_from = ((float) index_from_o) * ANGULAR_RESOLUTION;

    /* initialize the index from/to tables */
    for (index_to_o = 0; index_to_o < NUMBER_ANGLES; index_to_o++){
      o_to = ((float) index_to_o) * ANGULAR_RESOLUTION;
      forward_prob_index_min_x[index_from_o][index_to_o] = (WORLD_SIZE_X);
      forward_prob_index_max_x[index_from_o][index_to_o] = -(WORLD_SIZE_X);
      forward_prob_index_min_y[index_from_o][index_to_o] = (WORLD_SIZE_Y);
      forward_prob_index_max_y[index_from_o][index_to_o] = -(WORLD_SIZE_Y);
      forward_prob_factor[index_from_o][index_to_o]      = NULL;
    }

    for (phase = 0; phase < 2; phase++){ /* phase 1: find min/max
					  * phase 2: set factor */
      found = 0;
      for (index_to_o = 0; index_to_o < NUMBER_ANGLES; index_to_o++){
	o_to = ((float) index_to_o) * ANGULAR_RESOLUTION;

	/* first version of the turning error */
	error_turn1 = o_to - (o_from + turn);
	while (error_turn1 >= 180.0) error_turn1 -= 360.0;
	while (error_turn1 < -180.0) error_turn1 += 360.0;
	error_turn1 = fabs(error_turn1);
	
	if (error_turn1 < (fabs(translation) * TURN_CONTROL_NOISE_FACTOR)){
	  
	  for (index_x = - (WORLD_SIZE_X) + 1; index_x < (WORLD_SIZE_X);
	       index_x++){
	    delta_x = (float) index_x;

	    if (fabs(delta_x) <= 
		(fabs(translation) * 
		 (1.0 + TRANSLATION_CONTROL_NOISE_FACTOR)) /*! + 0.707 */){

	      for (index_y = - (WORLD_SIZE_Y) + 1; index_y < (WORLD_SIZE_Y); 
		   index_y++){
		delta_y = (float) index_y;
		
		if (fabs(delta_y) <= 
		    (fabs(translation) * 
		     (1.0 + TRANSLATION_CONTROL_NOISE_FACTOR)) /*! + 0.707 */){

		  /* second version of the turning error */
		  error_turn2 = (atan2(delta_y, delta_x) * 180.0 / M_PI) - 
		    (o_from + turn);
		  while (error_turn2 >= 180.0) error_turn2 -= 360.0;
		  while (error_turn2 < -180.0) error_turn2 += 360.0;
		  error_turn2 = fabs(error_turn2);
		  help = (atan(1.0 / (translation + 1.0)) * 180.0 / M_PI);
		  while (help >= 180.0) help -= 360.0;
		  while (help < -180.0) help += 360.0;
		  /*! error_turn2 -= 0.5 * help;
		    if (error_turn2 < 0.0)
		    error_turn2 = 0.0; */
		  
		  /* final turning error */
		  error_turn = error_turn2 + error_turn1;
		  
		  /* translation error */
		  error_translation = 
		    sqrt((delta_x * delta_x) + (delta_y * delta_y))
		    - translation;
		  error_translation = fabs(error_translation) /*! - 0.707 */;
		  if (error_translation < 0.0)
		    error_translation = 0.0;
		  

		  /* factor */
		  factor_turn = 
		    1.0 - (error_turn / (fabs(translation) *
					 TURN_CONTROL_NOISE_FACTOR));
		  factor_translation = 
		    1.0 - (error_translation / 
			   (fabs(translation) *
			    TRANSLATION_CONTROL_NOISE_FACTOR));
		  
		  
		  /* update the tables */
		  if (factor_translation > 0.0 && factor_turn > 0.0){
		    found = 1;
		    if (phase == 0){
		      if (index_x < 
			  forward_prob_index_min_x[index_from_o][index_to_o]) 
			forward_prob_index_min_x[index_from_o][index_to_o] = 
			  index_x;
		      if (index_x >
			  forward_prob_index_max_x[index_from_o][index_to_o]) 
			forward_prob_index_max_x[index_from_o][index_to_o] =
			  index_x;
		      if (index_y <
			  forward_prob_index_min_y[index_from_o][index_to_o]) 
			forward_prob_index_min_y[index_from_o][index_to_o] =
			  index_y;
		      if (index_y >
			  forward_prob_index_max_y[index_from_o][index_to_o]) 
			forward_prob_index_max_y[index_from_o][index_to_o] = 
			  index_y;
		      min_volume++;
		    }
		    else{
		      forward_prob_factor
			[index_from_o]
			[index_to_o]
			[index_x -
			forward_prob_index_min_x[index_from_o][index_to_o]]
			[index_y -
			forward_prob_index_min_y[index_from_o][index_to_o]]
			= factor_translation * factor_turn;
		    }
		  }
		}
	      }
	    }
	  }
	}
	if (phase == 0 &&
	    forward_prob_index_max_x[index_from_o][index_to_o] >=
	    forward_prob_index_min_x[index_from_o][index_to_o] &&
	    forward_prob_index_max_y[index_from_o][index_to_o] >=
	    forward_prob_index_min_y[index_from_o][index_to_o])
	  actual_volume += 
	    (forward_prob_index_max_x[index_from_o][index_to_o] -
	     forward_prob_index_min_x[index_from_o][index_to_o] + 1) *
	    (forward_prob_index_max_y[index_from_o][index_to_o] - 
	     forward_prob_index_min_y[index_from_o][index_to_o] + 1);
	
      }
      
      if (!found){
	o_to = o_from + turn;
	while (o_to >= 360.0) o_to -= 360.0;
	while (o_to <    0.0) o_to += 360.0;
	delta_x    = translation * cos(o_to * M_PI / 180.0);
	delta_y    = translation * sin(o_to * M_PI / 180.0);
	index_x    = (int) delta_x;
	index_y    = (int) delta_y;
	index_to_o = (int) (o_to / ANGULAR_RESOLUTION);
	if (phase == 0){
	  if (verbose)
	    fprintf(stderr, " set ");
	  forward_prob_index_min_x[index_from_o][index_to_o] = 
	    forward_prob_index_max_x[index_from_o][index_to_o] = index_x;
	  forward_prob_index_min_y[index_from_o][index_to_o] =
	    forward_prob_index_max_y[index_from_o][index_to_o] = index_y;
	  min_volume++;
	  actual_volume++;
	}
	else
	  forward_prob_factor
	    [index_from_o]
	    [index_to_o]
	    [index_x -
	    forward_prob_index_min_x[index_from_o][index_to_o]]
	    [index_y -
	    forward_prob_index_min_y[index_from_o][index_to_o]] = 1.0;
      }

      
      if (phase == 0){		/* allocate memory */

	for (index_to_o = 0; index_to_o < NUMBER_ANGLES; index_to_o++){
	  o_to = ((float) index_to_o) * ANGULAR_RESOLUTION;
	  if (forward_prob_index_max_x[index_from_o][index_to_o] >=
	      forward_prob_index_min_x[index_from_o][index_to_o] &&
	      forward_prob_index_max_y[index_from_o][index_to_o] >=
	      forward_prob_index_min_y[index_from_o][index_to_o]){
	    if (verbose)
	      fprintf(stderr, "Match: %g and %g, x:%d..%d  y:%d..%d\n",
		      o_from, o_to,
		      forward_prob_index_min_x[index_from_o][index_to_o],
		      forward_prob_index_max_x[index_from_o][index_to_o],
		      forward_prob_index_min_y[index_from_o][index_to_o],
		      forward_prob_index_max_y[index_from_o][index_to_o]); 
	    num_x = forward_prob_index_max_x[index_from_o][index_to_o] -
	      forward_prob_index_min_x[index_from_o][index_to_o] + 1;
	    num_y = forward_prob_index_max_y[index_from_o][index_to_o] -
	      forward_prob_index_min_y[index_from_o][index_to_o] + 1;
	    bytes_allocated_local = sizeof(float *) * num_x;
	    forward_prob_factor[index_from_o][index_to_o] = 
	      (float **) malloc(bytes_allocated_local);
	    if (forward_prob_factor[index_from_o][index_to_o] == NULL){
	      fprintf(stderr, "Out of memory (1).\n");
	      exit(-1);
	    }
	    bytes_allocated += bytes_allocated_local;
	    for (i = 0; i < num_x; i++){
	      bytes_allocated_local = sizeof(float) * num_y;
	      forward_prob_factor[index_from_o][index_to_o][i] = 
		(float *) malloc(sizeof(float) * num_y);
	      if (forward_prob_factor[index_from_o][index_to_o][i] == NULL){
		fprintf(stderr, "Out of memory (2).\n");
		exit(-1);
	      }
	      for (j = 0; j < num_y; j++)
		forward_prob_factor[index_from_o][index_to_o][i][j] = 0.0;
	      bytes_allocated += bytes_allocated_local;
	    }
	  }
	  else
	    forward_prob_factor[index_from_o][index_to_o] = NULL;
	}    

      }
    }
  }


  fprintf(stderr, 
	  "Found %d values of which %3.1f%% are non-zero, allocated %d bytes.\n",
	  actual_volume, 
	  ((float) min_volume) / ((float) actual_volume) * 100.0,
	  bytes_allocated);

  num_of_floats_allocated[(MAX_NUM_ITERATIONS)] += (float) actual_volume; 
  num_of_non_zero_floats[(MAX_NUM_ITERATIONS)]  += (float) min_volume;
  num_floats_bound[(MAX_NUM_ITERATIONS)]        += 
    ((float) (WORLD_SIZE_X)) *
    ((float) (WORLD_SIZE_X)) * 
    ((float) (WORLD_SIZE_Y)) * 
    ((float) (WORLD_SIZE_Y)) * 
    ((float) (NUMBER_ANGLES)) *
    ((float) (NUMBER_ANGLES));
  num_entities[(MAX_NUM_ITERATIONS)] += 1;
  gettimeofday(&(computation_time[MAX_NUM_ITERATIONS]), NULL);

  /*   G_display_switch(item[n].data->button_window_id, 0); */
  
}



		    


/************************************************************************
 *
 *   NAME:         update_density
 *                 
 *   FUNCTION:     This is the "main" routine: Here it is determined
 *                 how densities are updated.
 *                 
 *                 
 ************************************************************************/



void
update_density(int n, int density_number)
{
  int i, j, k, l, m, num_mults, i1, i2, j1, j2;
  float sum, max, max2;
  float factor, value;
  int   x_to, y_to, o_to, x_from, y_from, o_from;
  int   x_to2, y_to2, o_to2, x_from2, y_from2, o_from2;
  int   delta_x, delta_y, delta_o;
  float dist_x, dist_y, dist;
  float sum_pos, sum_neg;
  float best_prob, norm_prob;
  float min_x, min_y, max_x, max_y, min_o, max_o;
  float error_translation, error_turn1, error_turn2, error_turn;
  float factor_turn, factor_translation;
  float fact1, fact2, fact3, fact;
  int   x_delta, y_delta, x_delta0, y_delta0;
  int   min_index_x, max_index_x, min_index_y, max_index_y;
  float angle_to, angle_from;
  int   normalize_flag = 0;
  int   observation_flag = 0;
  int   land_x, land_y, land_x2, land_y2;
  int   bytes_allocated, bytes_freed;
  int   min_volume = 0, actual_volume = 0;
  float probability_bound;
  data_type *actual_data;
  data_type *prev_data;
  data_type *next_data;
  int   malloc_debug = 0;
  int next_n, next_next_n, prev_n, prev_prev_n;

  /* ********************************************************************* *\
   * ********************************************************************* *
   * ********************************************************************* *
   *                                                                       *
   *                    INITIALIZE                                         *
   *                                                                       *
   * ********************************************************************* *
   * ********************************************************************* *
  \* ********************************************************************* */


  /* fprintf(stderr, " ud(%d,%d) ", n, density_number);  */

  if (density_number == 1)
    G_display_switch(item[n].data->button_window_id, 1);
  else if (density_number == 2)
    G_display_switch(item[n].data->button_window_id, 2);
  else
    G_display_switch(item[n].data->button_window_id, 3);

  actual_data = item[n].data;

  prev_n = item[n].prev_data_item;
  if (item[n].prev_data_item >= 0){
    prev_prev_n = item[prev_n].prev_data_item;
    prev_data = item[prev_n].data;
  }
  else{
    prev_prev_n = -1;
    prev_data = NULL;
  }

  next_n = item[n].next_data_item;
  if (item[n].next_data_item >= 0){
    next_next_n = item[next_n].next_data_item;
    next_data = item[next_n].data;
  }
  else{
    next_data = NULL;
    next_next_n = -1;
  }



  if (button_alarm)
    ualarm(200, 0);		/* testing mouse events */


  for (k = 0; k < NUMBER_ANGLES; k++)
    for (i = 0; i < WORLD_SIZE_X; i++)
      for (j = 0; j < WORLD_SIZE_Y; j++)
	temp_probs[k][i][j] = 0.0;


  /* ********************************************************************* *\
   * ********************************************************************* *
   * ********************************************************************* *
   *                                                                       *
   *                    SWAPPING                                           *
   *                                                                       *
   * ********************************************************************* *
   * ********************************************************************* *
  \* ********************************************************************* */


  if (SWAPPING){
    if (density_number == 0){
      if (prev_n >= 0){
	swap_out_density(prev_n, 0);
	swap_out_density(prev_n, 1);
	swap_out_density(prev_n, 2);
      }
      swap_in_density(n, 1);
      swap_in_density(n, 2);
    }
    if (density_number == 1){
      if (prev_prev_n >= 0)
	swap_out_density(prev_prev_n, 1);
      if (item[n].data->swapped_out[1])
	swap_in_density(n, 1);  
    }
    else if (density_number == 2){
      if (next_next_n >= 0)
	swap_out_density(next_next_n, 2);
      if (item[n].data->swapped_out[2])
	swap_in_density(n, 2);  
    }
  }



  /* ********************************************************************* *\
   * ********************************************************************* *
   * ********************************************************************* *
   *                                                                       *
   *                    FORWARD PHASE                                      *
   *                                                                       *
   * ********************************************************************* *
   * ********************************************************************* *
  \* ********************************************************************* */


  if (density_number == 1){
    
    
    /* ==================================================
     *           Special case: known position
     * ================================================== */
    
    
    if (item[n].position_known){	/* includes the first data item */
      

      for (x_to = 0; x_to < (WORLD_SIZE_X); x_to++)
	actual_data->non_zero_index_x[density_number][x_to] = 0;
      for (y_to = 0; y_to < (WORLD_SIZE_Y); y_to++)
	actual_data->non_zero_index_y[density_number][y_to] = 0;
      for (o_to = 0; o_to < NUMBER_ANGLES; o_to++)
	actual_data->non_zero_index_o[density_number][o_to] = 0;

      x_to = truncint(item[n].correct_pos_x);
      y_to = truncint(item[n].correct_pos_y);
      o_to = truncint(item[n].correct_pos_orientation / ANGULAR_RESOLUTION);
      
      
      temp_probs[o_to][x_to][y_to] = 1.0;
      actual_data->non_zero_index_x[density_number][x_to] = 1;
      actual_data->non_zero_index_y[density_number][y_to] = 1;
      actual_data->non_zero_index_o[density_number][o_to] = 1;
      actual_data->min_non_zero_index_x[density_number]   = x_to;
      actual_data->max_non_zero_index_x[density_number]   = x_to;
      actual_data->min_non_zero_index_y[density_number]   = y_to;
      actual_data->max_non_zero_index_y[density_number]   = y_to;


      /* fprintf(stderr, "[%d,%d,%d=%g]\n", x_to, y_to, o_to, */
      /* temp_probs[o_to][x_to][y_to]); */


    }


    /* ==================================================
     *           Special case: unknown (new) position
     * ================================================== */

    
    else if (!item[n].position_known && item[n].new_episode){



      for (x_to = 0; x_to < (WORLD_SIZE_X); x_to++)
	actual_data->non_zero_index_x[density_number][x_to] = 1;
      for (y_to = 0; y_to < (WORLD_SIZE_Y); y_to++)
	actual_data->non_zero_index_y[density_number][y_to] = 1;
      for (o_to = 0; o_to < NUMBER_ANGLES; o_to++)
	actual_data->non_zero_index_o[density_number][o_to] = 1;
      actual_data->min_non_zero_index_x[density_number]     = 0;
      actual_data->max_non_zero_index_x[density_number]     = (WORLD_SIZE_X)-1;
      actual_data->min_non_zero_index_y[density_number]     = 0;
      actual_data->max_non_zero_index_y[density_number]     = (WORLD_SIZE_Y)-1;


      for (k = 0; k < NUMBER_ANGLES; k++)
	for (i = 0; i < WORLD_SIZE_X; i++)
	  for (j = 0; j < WORLD_SIZE_Y; j++)
	    temp_probs[k][i][j] = uniform();


      normalize_flag   = 0;
      observation_flag = 1;
    }


    /* ==================================================
     *           Regular case: constrained position
     * ================================================== */
    
    
    else{				/* all other data items */
      
      /* 
       * Compute the probabilities for time n
       */
      
      for (o_from = 0; o_from < NUMBER_ANGLES; o_from++)
	if (!run_time_optimize ||
	    prev_data->non_zero_index_o[density_number][o_from])
	  for (o_to = 0; o_to < NUMBER_ANGLES; o_to++)
	    if (actual_data->forward_prob_factor[o_from][o_to])
	      for (x_to = 0; x_to < WORLD_SIZE_X; x_to++)
		for (x_delta = 
		       actual_data->forward_prob_index_min_x[o_from][o_to],
		       x_delta0 = 0;
		     x_delta <=
		       actual_data->forward_prob_index_max_x[o_from][o_to]; 
		     x_delta++, x_delta0++){
		  x_from = x_to - x_delta;
		  x_from2 = x_from -
		    prev_data->min_non_zero_index_x[density_number];
		  if (x_from >= 0 && x_from < WORLD_SIZE_X &&
		      x_from2 >= 0 && x_from2 <= 
		      prev_data->max_non_zero_index_x[density_number] &&
		      (!run_time_optimize || 
		       prev_data->non_zero_index_x[density_number][x_from]))
		    for (y_to = 0; y_to < WORLD_SIZE_Y; y_to++)
		      for (y_delta = 
			     actual_data->
			     forward_prob_index_min_y[o_from][o_to],
			     y_delta0 = 0;
			   y_delta <= 
			     actual_data->
			     forward_prob_index_max_y[o_from][o_to]; 
			   y_delta++, y_delta0++){
			y_from = y_to - y_delta;
			y_from2 = y_from -
			  prev_data->min_non_zero_index_y[density_number];
			if (y_from >= 0 && y_from < WORLD_SIZE_Y &&
			    y_from2 >= 0 && y_from2 <= 
			    prev_data->max_non_zero_index_y[density_number] &&
			    (!run_time_optimize ||
			     prev_data->non_zero_index_y[density_number]
			     [y_from]) &&
			    prev_data->probs[density_number]
			    [o_from][x_from2][y_from2] > 0.0){
			  fact1 = actual_data->forward_prob_factor
			    [o_from][o_to][x_delta0][y_delta0];
			  fact2 = 
			    prev_data->probs[density_number]
			    [o_from][x_from2][y_from2];
			  fact = fact1 * fact2;
			  if (fact != 0.0)
			    temp_probs[o_to][x_to][y_to] += fact;
			}
		      }
		}

      
      normalize_flag   = 1;
      observation_flag = 1;

    }



    /* ==================================================
     *           Work in the observation
     * ================================================== */

    if (observation_flag && item[n].landmark_found){
      
      for (k = 0; k < NUMBER_ANGLES; k++){
	land_x2 = (int) ((item[n].landmark_rel_distance * 
			  cos(((((float) k) * ANGULAR_RESOLUTION) 
			       + item[n].landmark_rel_angle) * M_PI / 180.0))
			 + 0.5);
	land_y2 = (int) ((item[n].landmark_rel_distance * 
			  sin(((((float) k) * ANGULAR_RESOLUTION) 
			       + item[n].landmark_rel_angle) * M_PI / 180.0))
			 + 0.5);
	for (i = 0; i < WORLD_SIZE_X; i++)
	  for (j = 0; j < WORLD_SIZE_Y; j++){
	    land_x = land_x2 + i;
	    land_y = land_y2 + j;
	    if (land_x >= 0 && land_x < WORLD_SIZE_X &&
		land_y >= 0 && land_y < WORLD_SIZE_Y)
	      temp_probs[k][i][j] *= obs_probs[land_x][land_y];
	    else
	      temp_probs[k][i][j] = 0.0;
	  }
      }
      normalize_flag   = 1;
    }
  }
  


  /* ********************************************************************* *\
   * ********************************************************************* *
   * ********************************************************************* *
   *                                                                       *
   *                    BACKWARD PHASE                                     *
   *                                                                       *
   * ********************************************************************* *
   * ********************************************************************* *
  \* ********************************************************************* */


  else if (density_number == 2){

        
    /* ==================================================
     *           Special case: known position
     * ================================================== */
    
    
    if (item[n].position_known){	/* includes the first data item */

      for (x_to = 0; x_to < (WORLD_SIZE_X); x_to++)
	actual_data->non_zero_index_x[density_number][x_to] = 0;
      for (y_to = 0; y_to < (WORLD_SIZE_Y); y_to++)
	actual_data->non_zero_index_y[density_number][y_to] = 0;
      for (o_to = 0; o_to < NUMBER_ANGLES; o_to++)
	actual_data->non_zero_index_o[density_number][o_to] = 0;

      x_to = truncint(item[n].correct_pos_x);
      y_to = truncint(item[n].correct_pos_y);
      o_to = truncint(item[n].correct_pos_orientation / ANGULAR_RESOLUTION);
      
      temp_probs[o_to][x_to][y_to] = 1.0;
      actual_data->non_zero_index_x[density_number][x_to] = 1;
      actual_data->non_zero_index_y[density_number][y_to] = 1;
      actual_data->non_zero_index_o[density_number][o_to] = 1;
      actual_data->min_non_zero_index_x[density_number]   = x_to;
      actual_data->max_non_zero_index_x[density_number]   = x_to;
      actual_data->min_non_zero_index_y[density_number]   = y_to;
      actual_data->max_non_zero_index_y[density_number]   = y_to;

    }


    /* ==================================================
     *           Special case: unknown (new) position
     * ================================================== */


    
    else if (item[n].next_data_item < 0 ||
	     item[item[n].next_data_item].new_episode){

      for (x_to = 0; x_to < (WORLD_SIZE_X); x_to++)
	actual_data->non_zero_index_x[density_number][x_to] = 1;
      for (y_to = 0; y_to < (WORLD_SIZE_Y); y_to++)
	actual_data->non_zero_index_y[density_number][y_to] = 1;
      for (o_to = 0; o_to < NUMBER_ANGLES; o_to++)
	actual_data->non_zero_index_o[density_number][o_to] = 1;
      actual_data->min_non_zero_index_x[density_number]     = 0;
      actual_data->max_non_zero_index_x[density_number]     = (WORLD_SIZE_X)-1;
      actual_data->min_non_zero_index_y[density_number]     = 0;
      actual_data->max_non_zero_index_y[density_number]     = (WORLD_SIZE_Y)-1;

      for (k = 0; k < NUMBER_ANGLES; k++)
	for (i = 0; i < WORLD_SIZE_X; i++)
	  for (j = 0; j < WORLD_SIZE_Y; j++)
	    temp_probs[k][i][j] = uniform();


    }

    /* ==================================================
     *           Regular case: constrained position
     * ================================================== */
    
    

    else{

      for (o_from = 0; o_from < NUMBER_ANGLES; o_from++)
	for (o_to = 0; o_to < NUMBER_ANGLES; o_to++)
	  if (next_data->non_zero_index_o[density_number][o_to] &&
	      next_data->forward_prob_factor[o_from][o_to]){
	    if (item[item[n].next_data_item].landmark_found){
	      land_x2 = (int) ((item[item[n].next_data_item].
				landmark_rel_distance * 
				cos(((((float) o_to) * ANGULAR_RESOLUTION) 
				     + item[item[n].next_data_item].
				     landmark_rel_angle)
				    * M_PI / 180.0)) + 0.5);
	      land_y2 = (int) ((item[item[n].next_data_item].
				landmark_rel_distance * 
				sin(((((float) o_to) * ANGULAR_RESOLUTION) 
				     + item[item[n].next_data_item].
				     landmark_rel_angle)
				    * M_PI / 180.0)) + 0.5);
	    }
	    else
	      land_x2 = land_y2 = 0; /* won't be used */
	    for (x_to =  next_data->min_non_zero_index_x[density_number],
		   land_x = 
		   land_x2 + next_data->min_non_zero_index_x[density_number],
		   x_to2 = 0;
		 x_to <= next_data->max_non_zero_index_x[density_number];
		 x_to++, land_x++, x_to2++)
	      if (next_data->non_zero_index_x[density_number][x_to])
		for (x_delta = 
		       next_data->forward_prob_index_min_x[o_from][o_to],
		       x_delta0 = 0;
		     x_delta <=
		       next_data->forward_prob_index_max_x[o_from][o_to]; 
		     x_delta++, x_delta0++){
		  x_from = x_to - x_delta;
		  if (x_from >= 0 && x_from < WORLD_SIZE_X)
		    for (y_to =
			   next_data->min_non_zero_index_y[density_number],
			   land_y = land_y2
			   + next_data->min_non_zero_index_y[density_number], 
			   y_to2 = 0;
			 y_to <=
			   next_data->max_non_zero_index_y[density_number];
			 y_to++, land_y++, y_to2++)
		      if (next_data->non_zero_index_y[density_number][y_to] &&
			  next_data->probs[density_number][o_to][x_to2][y_to2]
			  > 0.0){
			/* compute the landmark-obs factor */
			if (item[item[n].next_data_item].landmark_found){
			  if (land_x >= 0 && land_x < WORLD_SIZE_X &&
			      land_y >= 0 && land_y < WORLD_SIZE_Y)
			    fact3 = obs_probs[land_x][land_y];
			  else
			    fact3 = 0.0;
			}
			else	/* no landmark observation here */
			  fact3 = 1.0;
			if (fact3 > 0.0)
			  for (y_delta = 
				 next_data->
				 forward_prob_index_min_y[o_from][o_to],
				 y_delta0 = 0;
			       y_delta <= 
				 next_data->
				 forward_prob_index_max_y[o_from][o_to]; 
			       y_delta++, y_delta0++){
			    y_from = y_to - y_delta;
			    if (y_from >= 0 && y_from < WORLD_SIZE_Y){
			      fact1 = next_data->forward_prob_factor
				[o_from][o_to][x_delta0][y_delta0];
			      fact2 = 
				next_data->probs[density_number]
				[o_to][x_to2][y_to2];
			      fact = fact1 * fact2 * fact3;
			      if (fact != 0.0)
				temp_probs[o_from][x_from][y_from]
				  += fact;
			    }
			  }
		      }		
		}
	  }
      
      normalize_flag = 1;
    }
  }


  /* check_items(1);   */
  /* check_forward(1);  */
  /* check_temp(1);   */




  /* if (n == 317 && density_number == 2)
      temp_probs[o_from][10][20] = 0.0;
      */
  /* ********************************************************************* *\
   * ********************************************************************* *
   * ********************************************************************* *
   *                                                                       *
   *                    COMPUTE STATE                                      *
   *                                                                       *
   * ********************************************************************* *
   * ********************************************************************* *
  \* ********************************************************************* */

  else if (density_number == 0){

    for (k = 0; k < NUMBER_ANGLES; k++)
      if (actual_data->probs[1][k] && actual_data->probs[2][k])
	for (i = 0; i < WORLD_SIZE_X; i++) /* change! */
	  if (actual_data->non_zero_index_x[1][i] &&
	      actual_data->non_zero_index_x[2][i]){
	    i1 = i - actual_data->min_non_zero_index_x[1];
	    i2 = i - actual_data->min_non_zero_index_x[2];
	    if (i1 < 0 || i2 < 0){
	      fprintf(stderr, "!@#$%^a\n");
	      exit(-1);
	    }
	    for (j = 0; j < WORLD_SIZE_Y; j++) /* change! */
	      if (actual_data->non_zero_index_y[1][j] &&
		  actual_data->non_zero_index_y[2][j]){
		j1 = j - actual_data->min_non_zero_index_y[1];
		j2 = j - actual_data->min_non_zero_index_y[2];
		if (j1 < 0 || j2 < 0){
		  fprintf(stderr, "!@#$%^b\n");
		  exit(-1);
		}
		temp_probs[k][i][j] = actual_data->probs[1][k][i1][j1] *
		  actual_data->probs[2][k][i2][j2];
	      }
	  }


    normalize_flag = 1;
    
  }





  /* ********************************************************************* *\
   * ********************************************************************* *
   * ********************************************************************* *
   *                                                                       *
   *                    SUMMING UP                                         *
   *                                                                       *
   * ********************************************************************* *
   * ********************************************************************* *
  \* ********************************************************************* */
  
  
  /* ==================================================
   *           Normalization and Truncation
   * ================================================== */

  /*
   * update index and normalize
   */
  
  if (normalize_flag){
    
    /*
     * initialize the target density and the various pointers/flags
     */
    
    for (x_to = 0; x_to < (WORLD_SIZE_X); x_to++)
      actual_data->non_zero_index_x[density_number][x_to] = 0;
    for (y_to = 0; y_to < (WORLD_SIZE_Y); y_to++)
      actual_data->non_zero_index_y[density_number][y_to] = 0;
    for (o_to = 0; o_to < NUMBER_ANGLES; o_to++)
      actual_data->non_zero_index_o[density_number][o_to] = 0;


    actual_data->min_non_zero_index_x[density_number]     = WORLD_SIZE_X;
    actual_data->max_non_zero_index_x[density_number]     = -1;
    actual_data->min_non_zero_index_y[density_number]     = WORLD_SIZE_Y;
    actual_data->max_non_zero_index_y[density_number]     = -1;
      
    
    sum = 0.0;
    max = 0.0; 
    for (k = 0; k < NUMBER_ANGLES; k++)
      for (i = 0; i < WORLD_SIZE_X; i++)
	for (j = 0; j < WORLD_SIZE_Y; j++){
	  if (temp_probs[k][i][j] >= 0.0)
	    sum += temp_probs[k][i][j];
 	  if (max < temp_probs[k][i][j]) 
 	    max = temp_probs[k][i][j]; 
	}
    if (sum > 0.0){
      sum = 1.0 / sum;
      max = max * sum; 
      for (k = 0; k < NUMBER_ANGLES; k++)
	for (i = 0; i < WORLD_SIZE_X; i++)
	  for (j = 0; j < WORLD_SIZE_Y; j++)
	    temp_probs[k][i][j] *= sum;
    }
    
    sum = 0.0;
    best_prob = 0.0;
    probability_bound = LOWER_PROBABILITY_BOUND * max;
    for (k = 0; k < NUMBER_ANGLES; k++)
      for (i = 0; i < WORLD_SIZE_X; i++)
	for (j = 0; j < WORLD_SIZE_Y; j++)

	  if (temp_probs[k][i][j] >= probability_bound){

	    sum += temp_probs[k][i][j];
	    actual_data->non_zero_index_x[density_number][i] = 1;
	    actual_data->non_zero_index_y[density_number][j] = 1;
	    actual_data->non_zero_index_o[density_number][k] = 1;

	    if (i < actual_data->min_non_zero_index_x[density_number])
	      actual_data->min_non_zero_index_x[density_number] = i;
	    if (i > actual_data->max_non_zero_index_x[density_number])
	      actual_data->max_non_zero_index_x[density_number] = i;
	    if (j < actual_data->min_non_zero_index_y[density_number])
	      actual_data->min_non_zero_index_y[density_number] = j;
	    if (j > actual_data->max_non_zero_index_y[density_number])
	      actual_data->max_non_zero_index_y[density_number] = j;

	    if (density_number == 0 && temp_probs[k][i][j] > best_prob){
	      best_prob = temp_probs[k][i][j];
	      item[n].best_pos_x           =  ((float) i) + 0.0;
	      item[n].best_pos_y           =  ((float) j) + 0.0;
	      item[n].best_pos_orientation = (((float) k) + 0.0)
		* ANGULAR_RESOLUTION;
	    }
	  }
	  else
	    temp_probs[k][i][j] = 0.0;
    if (sum > 0.0 && LOWER_PROBABILITY_BOUND > 0.0){
      sum = 1.0 / sum;
      for (k = 0; k < NUMBER_ANGLES; k++)
	for (i = 0; i < WORLD_SIZE_X; i++)
	  for (j = 0; j < WORLD_SIZE_Y; j++)
	    temp_probs[k][i][j] *= sum;
    }
  }
  /*
    actual_data->min_non_zero_index_x[density_number] = 0;
    actual_data->max_non_zero_index_x[density_number] = WORLD_SIZE_X-1;
    actual_data->min_non_zero_index_y[density_number] = 0;
    actual_data->max_non_zero_index_y[density_number] = WORLD_SIZE_Y-1;
    fprintf(stderr, "?");
    */
  /*
   fprintf(stderr, "\nBounds (n=%d density=%d): x:%d %d   y:%d %d\n",
	  n, density_number,
	  actual_data->min_non_zero_index_x[density_number],
	  actual_data->max_non_zero_index_x[density_number],
	  actual_data->min_non_zero_index_y[density_number],
	  actual_data->max_non_zero_index_y[density_number]);
	  */
  

  /* ==================================================
   *           Construct Target Density
   * ================================================== */
  

    /*@*/




  /*
   * Check, if the current target dimension seems sufficient
   */


  bytes_freed = bytes_allocated = 0;
  if (actual_data->max_non_zero_index_x[density_number] -
      actual_data->min_non_zero_index_x[density_number] + 1 >
      actual_data->internal_dimension_x[density_number] ||
      actual_data->max_non_zero_index_y[density_number] -
      actual_data->min_non_zero_index_y[density_number] + 1 >
      actual_data->internal_dimension_y[density_number]){
    /*
     * Free, if there is something to free
     */
    if (actual_data->internal_dimension_x[density_number] > 0 &&
	actual_data->internal_dimension_y[density_number] > 0){
      for (k = 0; k < NUMBER_ANGLES; k++){
	for (i = 0; 
	     i < actual_data->internal_dimension_x[density_number]; i++){
	  if (actual_data->probs[density_number][k] == NULL)
	    fprintf(stderr, "There is a bug in the memory management (1)\n");
	  else if (actual_data->probs[density_number][k][i] == NULL)
	    fprintf(stderr, "There is a bug in the memory management (2)\n");
	  else
	    free(actual_data->probs[density_number][k][i]);
	  if (malloc_debug)
	    fprintf(stderr, " [free(1): %d %d %d] ", density_number, k, i);
	  if (actual_data->probs[density_number][k] != NULL)
	    actual_data->probs[density_number][k][i] = NULL;
	  bytes_freed += 
	    (sizeof(float) * 
	     actual_data->internal_dimension_y[density_number]);
	}
	if (actual_data->probs[density_number][k] == NULL)
	  fprintf(stderr, "There is a bug in the memory management (3)\n");
	else
	  free(actual_data->probs[density_number][k]);
	if (malloc_debug)
	  fprintf(stderr, " [free(2): %d %d -] ", density_number, k);
	bytes_freed += 
	  (sizeof(float *)
	   * actual_data->internal_dimension_x[density_number]);
	actual_data->probs[density_number][k] = NULL;
      }
    }
    /*
     * set the new dimension
     */
    actual_data->internal_dimension_x[density_number] = 
      actual_data->max_non_zero_index_x[density_number] -
      actual_data->min_non_zero_index_x[density_number] + 1;
    actual_data->internal_dimension_y[density_number] = 
      actual_data->max_non_zero_index_y[density_number] -
      actual_data->min_non_zero_index_y[density_number] + 1;
  }

  /*
   * Check if we can free further memory
   */

#ifndef new
  for (k = 0; k < NUMBER_ANGLES; k++)
    if (actual_data->probs[density_number][k] != NULL &&
	!(actual_data->non_zero_index_o[density_number][k])){
      for (i = 0; i < actual_data->internal_dimension_x[density_number]; i++){
	if (malloc_debug)
	  fprintf(stderr, " [free(3): %d %d %d] ", density_number, k, i);
	if (actual_data->probs[density_number][k] == NULL)
	  fprintf(stderr, "There is a bug in the memory management (4)\n");
	else if (actual_data->probs[density_number][k][i] == NULL)
	  fprintf(stderr, "There is a bug in the memory management (5)\n");
	else
	  free(actual_data->probs[density_number][k][i]);
	actual_data->probs[density_number][k][i] = NULL;
	bytes_freed += 
	  (sizeof(float) * actual_data->internal_dimension_y[density_number]);
      }
      if (malloc_debug)
	fprintf(stderr, " [free(4): %d %d -] ", density_number, k);
      if (actual_data->probs[density_number][k] == NULL)
	fprintf(stderr, "There is a bug in the memory management (6)\n");
      else
	free(actual_data->probs[density_number][k]);
      bytes_freed += 
	(sizeof(float *)
	 * actual_data->internal_dimension_x[density_number]);
      if (actual_data->probs[density_number][k] != NULL)
	actual_data->probs[density_number][k] = NULL;
    }
#else
  fprintf(stderr, "?");
#endif  

  /*
   * Check if we need to allocate more memory
   */

  for (k = 0; k < NUMBER_ANGLES; k++)
    if (actual_data->probs[density_number][k] == NULL &&
	actual_data->non_zero_index_o[density_number][k]){ /* memory missing */
      /*
       * sanity check
       */
      if (actual_data->internal_dimension_x[density_number] <= 0 ||
	  actual_data->internal_dimension_y[density_number] <= 0){
	fprintf(stderr, "STRANGE ERROR: allocate %d and %d\n",
		actual_data->internal_dimension_x[density_number],
		actual_data->internal_dimension_y[density_number]);
	exit(-1);
      }
      /*
       * allocate an array
       */
      if (malloc_debug)
	fprintf(stderr, " [malloc(1): %d %d -] ", density_number, k);
      actual_data->probs[density_number][k] =
	(float **) malloc(sizeof(float *) *
			  actual_data->internal_dimension_x[density_number]);
      if (actual_data->probs[density_number][k] == NULL){
	fprintf(stderr, "Out of memory in update_density().\n");
	exit(-1);
      }
      bytes_allocated += 
	(sizeof(float *) * actual_data->internal_dimension_x[density_number]);

      for (i = 0; i < actual_data->internal_dimension_x[density_number]; i++){
	if (malloc_debug)
	  fprintf(stderr, " [malloc(2): %d %d %d] ", density_number, k, i);
	actual_data->probs[density_number][k][i] = 
	  (float *) malloc(sizeof(float ) *
			   actual_data->internal_dimension_y[density_number]);
	if (actual_data->probs[density_number][k][i] == NULL){
	  fprintf(stderr, "Out of memory in update_density().\n");
	  exit(-1);
	}
	for (j = 0; j < actual_data->internal_dimension_y[density_number]; j++)
	  actual_data->probs[density_number][k][i][j] = 0.0;
	bytes_allocated += 
	  (sizeof(float) * actual_data->internal_dimension_y[density_number]);
      }
    }

  /*
   * Print receipt
   */

  if (bytes_freed > 0 || bytes_allocated > 0){
    if (bytes_allocated > bytes_freed)
      fprintf(stderr, "+");
    else
      fprintf(stderr, "-");
    fprintf(stderr, 
	    "\nUpdate_density(%d,%d): freed %d allocated %d bytes.\n",
	    n, density_number, bytes_freed, bytes_allocated);
      }
  
  
  /*
   * And copy the density over
   */


  for (k = 0; k < NUMBER_ANGLES; k++){
    if (actual_data->non_zero_index_o[density_number][k]){ /* there's 
							    * something */
      for (i =  actual_data->min_non_zero_index_x[density_number], i2 = 0;
	   i <= actual_data->max_non_zero_index_x[density_number]; i++, i2++)
	for (j =  actual_data->min_non_zero_index_y[density_number], j2 = 0;
	     j <= actual_data->max_non_zero_index_y[density_number]; 
	     j++, j2++){
	  actual_data->probs[density_number][k][i2][j2] = temp_probs[k][i][j];
	  if (temp_probs[k][i][j] > 0.0)
	    min_volume++;	/* just for statistics */
	  actual_volume++;	/* just for statistics */
	  if (i2 >= actual_data->internal_dimension_x[density_number] ||
	      j2 >= actual_data->internal_dimension_y[density_number]){
	    fprintf(stderr, 
		    "STRANGE ERROR: exceeded internal bounds %d %d %d %d.\n",
		    i2, actual_data->internal_dimension_x[density_number],
		    j2, actual_data->internal_dimension_y[density_number]);
	    exit(-1);
	  }
	    
	}
      }
  }

  /* the following command shows the amount memory saved here */

  /* fprintf(stderr, " [%d=%4.2f%%] ", bytes_allocated,
     ((float) bytes_allocated) * 100.0 / 
     ((float) ((WORLD_SIZE_X)*(WORLD_SIZE_Y)*(NUMBER_ANGLES))));
     */


  /* ==================================================
   *           Display
   * ================================================== */

  if (button_alarm)
    ualarm(0, 0);		/* stop testing mouse events now, 
				 * to avoid interference */
  
  /*
   * copy into window_probs
   */


  for (i = 0; i < WORLD_SIZE_X; i++)
    for (j = 0; j < WORLD_SIZE_Y; j++){
      max = 0.0;
      for (l = 0; l < NUMBER_ANGLES; l++)
	if (temp_probs[l][i][j] > max)
	  max = temp_probs[l][i][j]; 
      item[n].data->window_probs[density_number][i][j] = max;
    }
  
  /*
   * set the new display range and display densities
   */


  max = 0.0;
  for (i = 0; i < WORLD_SIZE_X; i++)
    for (j = 0; j < WORLD_SIZE_Y; j++)
      if (item[n].data->window_probs[density_number][i][j] > max)
	max = item[n].data->window_probs[density_number][i][j];
  G_matrix_set_display_range(item[n].data->probs_window_id[density_number], 
			     0.0, max);
  G_display_matrix(item[n].data->probs_window_id[density_number]);
  if (density_number == 0)
    actual_data->max_prob = max;

  /*
   * construct the all_probs display (superposition)
   */

  
  if (density_number == 0){
    max2 = 0.0;
    for (i = 0; i < WORLD_SIZE_X; i++)
      for (j = 0; j < WORLD_SIZE_Y; j++){
	max = 0.0;
	for (m = 0; m < num_items; m++)
	  if (item[m].data && item[m].data->max_prob > 0.0){
	    value = item[m].data->window_probs[density_number][i][j]
	      / item[m].data->max_prob;
	    if (value > max)
	      max = value;
	  }
	all_probs[i][j] = max;
	if (max > max2)
	  max2 = max;
      }
    G_matrix_set_display_range(ALL_PROBS, 0.0, max2);
    G_display_matrix(ALL_PROBS);
  }  
  



  
  /*
   * display landmark position, if any
   */
  
  for (i = 0; i < NUM_PATHS2; i++)
    G_clear_markers(item[n].data->path_window_id[density_number][i]);
    
  
  if (item[n].landmark_found)
    G_add_marker(item[n].data->path_window_id[density_number][1], 
		 item[n].landmark_measured_x + 0.5,
		 item[n].landmark_measured_y + 0.5, 0);
  
  
  /*
   * display measured path
   */
  
  for (m = 0; m <= n; m++)
    G_add_marker(item[n].data->path_window_id[density_number][2], 
		 item[m].measured_pos_x + 0.5,
		 item[m].measured_pos_y + 0.5, 0);
  
  
  /*
   * display measured position
   */
  
  G_add_marker(item[n].data->path_window_id[density_number][3], 
	       item[n].measured_pos_x + 0.5,
	       item[n].measured_pos_y + 0.5, 0);
  
  /*
   * display correct path
   */
  
  for (m = 0; m <= n; m++)
    G_add_marker(item[n].data->path_window_id[density_number][4], 
		 item[m].correct_pos_x + 0.5,
		 item[m].correct_pos_y + 0.5, 0);
  
  
  /*
   * display correct position
   */
    
  G_add_marker(item[n].data->path_window_id[density_number][5], 
	       item[n].correct_pos_x + 0.5,
	       item[n].correct_pos_y + 0.5, 0);
  
  /*
   * display max likelihood path
   */
  
  for (m = 0; m <= n; m++)
    G_add_marker(item[n].data->path_window_id[density_number][6], 
		 item[m].best_pos_x + 0.5,
		 item[m].best_pos_y + 0.5, 0);
  
  /*
   * display max likelihood position
   */
  
  G_add_marker(item[n].data->path_window_id[density_number][7], 
	       item[n].best_pos_x + 0.5,
	       item[n].best_pos_y + 0.5, 0);
  



  /*
   * display bounding box
   */

  min_index_x = max_index_x = -1;
  min_index_y = max_index_y = -1;
  for (x_to = 0; x_to < (WORLD_SIZE_X); x_to++)
    if (actual_data->non_zero_index_x[density_number][x_to]){
      if (min_index_x == -1)
	min_index_x = x_to;
      max_index_x   = x_to;
    }
  for (y_to = 0; y_to < (WORLD_SIZE_Y); y_to++)
    if (actual_data->non_zero_index_y[density_number][y_to]){
      if (min_index_y == -1)
	min_index_y = y_to;
      max_index_y   = y_to;
    }
  
  /* sanity check */
  if (min_index_x != actual_data->min_non_zero_index_x[density_number] ||
      max_index_x != actual_data->max_non_zero_index_x[density_number] ||
      min_index_y != actual_data->min_non_zero_index_y[density_number] ||
      max_index_y != actual_data->max_non_zero_index_y[density_number])
    fprintf(stderr,
	    "\n\n\tWARNING: Internal error: %d %d %d %d vs %d %d %d %d\n\n",
	    min_index_x, max_index_x, min_index_y, max_index_y,
	    actual_data->min_non_zero_index_x[density_number],
	    actual_data->max_non_zero_index_x[density_number],
	    actual_data->min_non_zero_index_y[density_number],
	    actual_data->max_non_zero_index_y[density_number]);
  

  G_clear_markers(item[n].data->path_window_id[density_number][8]);
  G_add_marker(item[n].data->path_window_id[density_number][8],
	       ((float) min_index_x), ((float) min_index_y), 0);
  G_add_marker(item[n].data->path_window_id[density_number][8],
	       ((float) (max_index_x+1)), ((float) min_index_y), 0);
  G_add_marker(item[n].data->path_window_id[density_number][8],
	       ((float) (max_index_x+1)), ((float) (max_index_y+1)), 0);
  G_add_marker(item[n].data->path_window_id[density_number][8],
	       ((float) min_index_x), ((float) (max_index_y+1)), 0);
  G_add_marker(item[n].data->path_window_id[density_number][8],
	       ((float) min_index_x), ((float) min_index_y), 0);

  
  for (i = 0; i < NUM_PATHS2; i++)
    G_display_markers(item[n].data->path_window_id[density_number][i]);


  /*
   * display observationb matrix
   */

  display_observation_matrix();

  /*
   * statistics
   */

  if (iteration < MAX_NUM_ITERATIONS){
    num_of_floats_allocated[iteration] += (float) actual_volume; 
    num_of_non_zero_floats[iteration]  += (float) min_volume;
    num_floats_bound[iteration] += (float) 
      ((WORLD_SIZE_X)* (WORLD_SIZE_Y) * (NUMBER_ANGLES));
    num_entities[iteration] += 1;
    gettimeofday(&(computation_time[iteration]), NULL);
  }
  else 
    fprintf(stderr, "WARNING: exceeded legal number of iterations: %d\n",
	    iteration);

  /*
   * done
   */

  G_display_switch(item[n].data->button_window_id, 0);
  
}



		    


/************************************************************************
 *
 *   NAME:         compute_observation_matrix
 *                 
 *   FUNCTION:     This routine computes the observation
 *                 matrix (M step in EM)
 *                 
 *                 
 ************************************************************************/

void
compute_observation_matrix()
{
  float obs_probs_1[WORLD_SIZE_X][WORLD_SIZE_Y];
  float obs_probs_2[WORLD_SIZE_X][WORLD_SIZE_Y];
  int land_x, land_y, land_x2, land_y2;
  int delta_i, delta_j;
  int i, j, k, m, i2, j2;
  float max, dist;

  fprintf(stderr, "Observations.");
  
  for (i = 0; i < WORLD_SIZE_X; i++)
    for (j = 0; j < WORLD_SIZE_Y; j++){
      obs_probs_1[i][j] = 0.0;
      obs_probs_2[i][j] = 0.0;
    }

  /*
   * build the observation table 
   */

  for (m = 0; m < num_items; m++)
    if (item[m].data){
      if (SWAPPING)
	swap_in_density(m, 0);
      for (k = 0; k < NUMBER_ANGLES; k++)
	if (item[m].data->probs[0][k] &&
	    item[m].data->non_zero_index_o[0][k]){
	  land_x2 = (int) ((item[m].landmark_rel_distance * 
			    cos(((((float) k) * ANGULAR_RESOLUTION) 
				 + item[m].landmark_rel_angle) 
				* M_PI / 180.0)) + 0.5);
	  land_y2 = (int) ((item[m].landmark_rel_distance * 
			    sin(((((float) k) * ANGULAR_RESOLUTION) 
				 + item[m].landmark_rel_angle)
				* M_PI / 180.0)) + 0.5);
	  for (i =  item[m].data->min_non_zero_index_x[0], i2 = 0;
	       i <= item[m].data->max_non_zero_index_x[0];
	       i++, i2++)
	    if (item[m].data->non_zero_index_x[0][i])
	      for (j =  item[m].data->min_non_zero_index_y[0], j2 = 0;
		   j <= item[m].data->max_non_zero_index_y[0];
		   j++, j2++)
		if (item[m].data->non_zero_index_y[0][j])
		  if (item[m].data->probs[0][k][i2][j2] > 0.0)
		    if (item[m].landmark_found){
		      land_x = i + land_x2;
		      land_y = j + land_y2;
		      for (delta_i = - ((int) (VISUAL_RANGE));
			   delta_i <= ((int) (VISUAL_RANGE)); delta_i++)
			if (i + delta_i >= 0 && i + delta_i < WORLD_SIZE_X)
			  for (delta_j = - ((int) (VISUAL_RANGE));
			       delta_j <= ((int) (VISUAL_RANGE)); delta_j++)
			    if (j + delta_j >= 0 && 
				j + delta_j < WORLD_SIZE_Y){
			      dist = sqrt(((float) (delta_i * delta_i)) +
					  ((float) (delta_j * delta_j)));
			      if (dist <= VISUAL_RANGE){
				if (i + delta_i == land_x &&
				    j + delta_j == land_y)
				  obs_probs_1[i + delta_i][j + delta_j] +=
				    item[m].data->probs[0][k][i2][j2];
				else
				  obs_probs_1[i + delta_i][j + delta_j] +=
				    MIN_OBS_PROB * 
				    item[m].data->probs[0][k][i2][j2];
				obs_probs_2[i + delta_i][j + delta_j] +=
				  item[m].data->probs[0][k][i2][j2];
			      }
			    }
		    }
	}
      fprintf(stderr, ".");
      if (SWAPPING)
	swap_out_density(m, 0);
    }
  
  max = 0.0;
  for (i = 0; i < WORLD_SIZE_X; i++)
    for (j = 0; j < WORLD_SIZE_Y; j++)
      if (obs_probs_2[i][j] == 0.0)
	obs_probs[i][j] = MIN_OBS_PROB;
      else{
	obs_probs[i][j] = obs_probs_1[i][j] / obs_probs_2[i][j];
	if (obs_probs[i][j] > max)
	  max = obs_probs[i][j];
      }

  G_matrix_set_display_range(OBS_PROBS, 0.0, max);


  display_observation_matrix();
  fprintf(stderr, "done.\n");

}
		    


/************************************************************************
 *
 *   NAME:         display_observation_matrix
 *                 
 *   FUNCTION:     This routine  displays the observation
 *                 matrix
 *                 
 *                 
 ************************************************************************/

void
display_observation_matrix()
{
  int i, j, k, m;
  
  /*
   * display global window: display the maximum over all probabilities
   */
  
  G_display_matrix(OBS_PROBS);
  
  /*
   * interpolate path
   */

  update_intermediate_path_items();


  /*
   * display global window: display paths
   */
  
  
  for (k = 1; k < 3; k++){
    for (i = 0; i < NUM_PATHS2; i++)
      G_clear_markers(PATH_DISPLAY[k][i]);
    for (m = 0; m < num_items; m++){
      G_add_marker(PATH_DISPLAY[k][2], 
		   item[m].measured_pos_x + 0.5,
		   item[m].measured_pos_y + 0.5, 0);
      if (item[m].data)
	G_add_marker(PATH_DISPLAY[k][3], 
		     item[m].measured_pos_x + 0.5,
		     item[m].measured_pos_y + 0.5, 0);
      G_add_marker(PATH_DISPLAY[k][4], 
		   item[m].correct_pos_x + 0.5,
		   item[m].correct_pos_y + 0.5, 0);
      if (item[m].data)
	G_add_marker(PATH_DISPLAY[k][5], 
		     item[m].correct_pos_x + 0.5,
		     item[m].correct_pos_y + 0.5, 0);
      /*!*/
      /*
      if (k == 0)
	fprintf(stderr, "%d: [%g %g %g]\n", m, 
		(item[m].best_pos_x - robot_position_absolute_offset_x)
		/ robot_position_absolute_multiplier,
		(item[m].best_pos_y - robot_position_absolute_offset_y)
		/ robot_position_absolute_multiplier,
		item[m].best_pos_orientation);
		*/
      
      G_add_marker(PATH_DISPLAY[k][6], 
		   item[m].best_pos_x + 0.5,
		   item[m].best_pos_y + 0.5, 0);
      if (item[m].data)
	G_add_marker(PATH_DISPLAY[k][7], 
		     item[m].best_pos_x + 0.5,
		     item[m].best_pos_y + 0.5, 0);
    }
    for (i = 0; i < NUM_PATHS2; i++)
      G_display_markers(PATH_DISPLAY[k][i]);
  }
}


/************************************************************************
 *
 *   NAME:         update_intermediate_path_items
 *                 
 *   FUNCTION:     updates the path (linear interpolation where necessary)
 *                 
 *                 
 ************************************************************************/


void
update_intermediate_path_items()
{
  int i, from_i, to_i, j, k, n;
  float diff_turn, diff_translation;
  float translate, turn, total_turn, orientation;

  total_turn = 0.0;

  for (n = 0; n < num_items; n++){
    
    if (!item[n].new_episode && item[n].prev_data_item >= 0 &&
	item[n].data != NULL){

      /* recalculate the best orientation - this improves
       * the interpolation */

      /*fprintf(stderr, "%d: %8.6f -> ", n, item[n].best_pos_orientation);*/
      item[n].best_pos_orientation
	= atan2(item[n].best_pos_y - 
		item[item[n].prev_data_item].best_pos_y,
		item[n].best_pos_x - 
		item[item[n].prev_data_item].best_pos_x) * 180.0 / M_PI;
      /*fprintf(stderr, "%8.6f\n", item[n].best_pos_orientation);*/
      

      diff_translation = 
	sqrt(((item[n].best_pos_x - 
	       item[item[n].prev_data_item].best_pos_x) *
	      (item[n].best_pos_x - 
	       item[item[n].prev_data_item].best_pos_x)) +
	     ((item[n].best_pos_y - 
	       item[item[n].prev_data_item].best_pos_y) *
	      (item[n].best_pos_y - 
	       item[item[n].prev_data_item].best_pos_y)))
	- item[n].measured_translation_since_last_estimate;
      diff_turn = 
	(atan2(item[n].best_pos_y - 
	       item[item[n].prev_data_item].best_pos_y,
	       item[n].best_pos_x - 
	       item[item[n].prev_data_item].best_pos_x) * 180.0 / M_PI)
	- item[item[n].prev_data_item].best_pos_orientation
	- item[n].measured_turn_since_last_estimate;
      while (diff_turn >   180.0) diff_turn -= 360.0;
      while (diff_turn <= -180.0) diff_turn += 360.0;
      
      /*
      fprintf(stderr, "n=%d/%d: Dturn=%g (%g) Dtrans=%g (%g %g -> %g %g)\n",
	      n, item[n].prev_data_item,
	      diff_turn, 
	      item[item[n].prev_data_item].best_pos_orientation,
	      diff_translation,
	      item[item[n].prev_data_item].best_pos_x,
	      item[item[n].prev_data_item].best_pos_y,
	      item[n].best_pos_x, item[n].best_pos_y);
	      */
      
      
      k = n - item[n].prev_data_item;
      if (k > 0)
	for (i = item[n].prev_data_item + 1, j = 1; i <= n; i++, j++){
	  translate = item[i].measured_translation_since_last_estimate
	    + (diff_translation * ((float) j) / ((float) k));
	  turn = item[i].measured_turn_since_last_estimate
	    + (diff_turn * ((float) j) / ((float) k));
	  orientation = 
	    turn + item[item[n].prev_data_item].best_pos_orientation;
	  item[i].best_pos_x = 
	    item[item[n].prev_data_item].best_pos_x +
	    (translate * cos(orientation * M_PI / 180.0));
	  item[i].best_pos_y = 
	    item[item[n].prev_data_item].best_pos_y +
	    (translate * sin(orientation * M_PI / 180.0));
	  item[i].best_heading = item[i].measured_heading +
	    total_turn + (diff_turn * ((float) j) / ((float) k));
	  while (item[i].best_heading < -180.0)
	    item[i].best_heading += 360.0;
	  while (item[i].best_heading >= 180.0)
	    item[i].best_heading -= 360.0;
	  /*
	  fprintf(stderr,
		  "#%d: correction turn:%8.6f trans:%8.6f tot-turn:%8.6f => %8.6f\n",
		  i, (diff_turn * ((float) j) / ((float) k)),
		  (diff_translation * ((float) j) / ((float) k)),
		  total_turn + (diff_turn * ((float) j) / ((float) k)),
		  item[i].best_heading);
		  */
	}
      total_turn += diff_turn;
    }
  }
}
	    


/************************************************************************
 *
 *   NAME:         load_raw_data(char *filename)
 *                 
 *   FUNCTION:     Loads raw sensor data from file
 *                 Returns 1, if successful
 *                 
 ************************************************************************/

int
load_raw_data(char *filename)
{
  int def = 0, newset;
  float min_x = 0.0, max_x = 0.0;
  float min_y = 0.0, max_y = 0.0;
  float prev_raw_x, prev_raw_y, prev_raw_o;
  float prev_corrected_x, prev_corrected_y, prev_corrected_o;
  float raw_x, raw_y, raw_o;
  float corrected_x, corrected_y, corrected_o;
  float angle, dist;
  int success = 1;
  int n;

  if (save_file_name && num_items){
    fprintf(stderr, "ERROR: Data has alread been loaded/defined.\n");
    return 0;
  }


  mem_create_data();
  success = mem_load_patterns(filename, 0, -1);
  if (!success)
    return 0;

  /*
   * work in the robot's drift
   */

  prev_raw_x = prev_raw_y = prev_raw_o = 0.0;
  prev_corrected_x = prev_corrected_y = prev_corrected_o = 0.0;

  for (n = 0; n < mem_number_pattern_sets() && n < MAX_NUM_DATASETS; n++){

    current_patternset = mem_get_nth_patternset(n);
    def = 0;

    /*
     * find min, max
     */
    for (current_pattern = mem_get_first_pattern(current_patternset);
	 current_pattern != NULL;
	 current_pattern = mem_get_next_pattern(current_pattern)){
      if (current_pattern->position){
	/* current raw data */
	raw_x = current_pattern->position->x;
	raw_y = current_pattern->position->y;
	raw_o = current_pattern->position->orientation;

	/* calculate the corrected data */
	if (!def){
	  corrected_x = raw_x;
	  corrected_y = raw_y;
	  corrected_o = raw_o;
	  def = 1;
	}
	else{
	  dist = sqrt(((raw_x - prev_raw_x) * (raw_x - prev_raw_x)) +
		      ((raw_y - prev_raw_y) * (raw_y - prev_raw_y)));
	  angle = ((atan2(raw_y - prev_raw_y, raw_x - prev_raw_x) 
		    * 180.0 / M_PI) - prev_raw_o);
	  angle += (ROBOT_DRIFT * dist);
	  while (angle < -180.0) angle += 360.0;
	  while (angle >= 180.0) angle -= 360.0;
	  corrected_x = prev_corrected_x + 
	    dist * cos((angle + prev_corrected_o) * M_PI / 180.0);
	  corrected_y = prev_corrected_y + 
	    dist * sin((angle + prev_corrected_o) * M_PI / 180.0);
	  corrected_o =
	    raw_o + (ROBOT_DRIFT * dist) + prev_corrected_o - prev_raw_o;
	  while (corrected_o < -180.0) corrected_o += 360.0;
	  while (corrected_o >= 180.0) corrected_o -= 360.0;	  
	}

	current_pattern->position->x           = corrected_x;
	current_pattern->position->y           = corrected_y;
	current_pattern->position->orientation = corrected_o;

	prev_raw_x = raw_x;
	prev_raw_y = raw_y;
	prev_raw_o = raw_o;

	prev_corrected_x = corrected_x;
	prev_corrected_y = corrected_y;
	prev_corrected_o = corrected_o;


      }
    }
  }
  
  /*
   * 
   */

  for (n = 0; n < mem_number_pattern_sets() && n < MAX_NUM_DATASETS; n++){

    current_patternset = mem_get_nth_patternset(n);
    def = 0;

    /*
     * find min, max
     */
    for (current_pattern = mem_get_first_pattern(current_patternset);
	 current_pattern != NULL;
	 current_pattern = mem_get_next_pattern(current_pattern)){
      if (current_pattern->position){
	if (!def || current_pattern->position->x < min_x)
	  min_x = current_pattern->position->x;
	if (!def || current_pattern->position->x > max_x)
	  max_x = current_pattern->position->x;
	if (!def || current_pattern->position->y < min_y)
	min_y = current_pattern->position->y;
	if (!def || current_pattern->position->y > max_y)
	  max_y = current_pattern->position->y;
	def = 1;
      }
    }

    
    /* 
     * set the _optimal_ offset, so that data is in the center 
     */
    if ((max_x - min_x) * robot_position_absolute_multiplier
	> ((float) WORLD_SIZE_X) ||
	(max_y - min_y)  * robot_position_absolute_multiplier
	> ((float) WORLD_SIZE_Y)){
      fprintf(stderr, "World is too small, or resolution too high.\n");
      fprintf(stderr, "%g > %d or %g > %d (res=%g)\n",
	      (max_x - min_x) * robot_position_absolute_multiplier,
	      WORLD_SIZE_X,
	      (max_y - min_y) * robot_position_absolute_multiplier,
	      WORLD_SIZE_Y,
	      robot_position_absolute_multiplier);
      return 0;
    }
    if (!robot_position_absolute_offset_defined[n]){
      robot_position_absolute_offset_x[n]   = 
	0.5 * (((float) WORLD_SIZE_X) -
	       ((max_x + min_x) * robot_position_absolute_multiplier));
      robot_position_absolute_offset_y[n]   = 
	0.5 * (((float) WORLD_SIZE_Y) -
	       ((max_y + min_y) * robot_position_absolute_multiplier));
      robot_position_absolute_offset_defined[n] = 1;
    }
    else{
      fprintf(stderr, "-?-");
    }
    
    fprintf(stderr, "Offset %d(x=%g..%g y=%g..%g res:%g):  %g %g\n",
	    n, min_x, max_x, min_y, max_y,
	    robot_position_absolute_multiplier,
	    robot_position_absolute_offset_x[n],
	    robot_position_absolute_offset_y[n]);
  }



  for (n = 0; n < mem_number_pattern_sets() && n < MAX_NUM_DATASETS; n++){

    current_patternset = mem_get_nth_patternset(n);
    newset = 1;

    /* 
     * copy into internal memory 
     */
    for (current_pattern = mem_get_first_pattern(current_patternset);
	 current_pattern != NULL;
	 current_pattern = mem_get_next_pattern(current_pattern)){
      if (current_pattern->position){
	robot_position_absolute_scaled(current_pattern->position->x,
				       current_pattern->position->y,
				       current_pattern->position->orientation,
				       current_pattern->position->x,
				       current_pattern->position->y,
				       current_pattern->position->orientation,
				       newset, (n == 0));
	if (current_pattern->buttons && current_pattern->buttons->pushed[0])
	  saw_landmark_polar(0.0, 0.0);
	newset = 0;
      }
    }
  }
  /*
   * construct filename for saving the corrected data
   */

  save_file_name = (char *) malloc(sizeof(char) * (strlen(filename) + 10));
  sprintf(save_file_name, "%s.corrected", filename);

  return 1;
}


/************************************************************************
 *
 *   NAME:         save_raw_data(char *filename)
 *                 
 *   FUNCTION:     Saves raw data into file, using guessed pos estimates
 *                 Returns 1, if successful
 *                 
 ************************************************************************/

int
save_raw_data(char *filename)
{
  int n, m;

  current_patternset = mem_get_nth_patternset(0);
  n = 0;

  for (m = 0; m < mem_number_pattern_sets() && m < MAX_NUM_DATASETS; m++){

    current_patternset = mem_get_nth_patternset(m);
    
    for (current_pattern = mem_get_first_pattern(current_patternset);
	 current_pattern != NULL;
	 current_pattern = mem_get_next_pattern(current_pattern)){
      if (current_pattern->position){
	current_pattern->position->x = 
	  (item[n].best_pos_x - robot_position_absolute_offset_x[m])
	  / robot_position_absolute_multiplier;
	current_pattern->position->y =
	  (item[n].best_pos_y - robot_position_absolute_offset_y[m])
	  / robot_position_absolute_multiplier;
	current_pattern->position->orientation = item[n].best_heading;
	/* and not: item[n].best_pos_orientation; */
	n++;
      }
    }
  }
  return (mem_save_patterns(filename, -1));
}



/************************************************************************
 *
 *   NAME:         save_data(char *filename)
 *                 
 *   FUNCTION:     Dumps everything onto a file
 *                 
 *                 
 ************************************************************************/

void
save_data(char *filename)
{
  static FILE *iop = NULL;
  static int iop_error = 0;
  int i, j, n;

  if (iop_error)		/* once an error, we'll never save again */
    return;

  if (iop == NULL)
    if ((iop = fopen(filename, "w")) == 0){
      iop_error = 1;
      fprintf(stderr, "WARNING: Could not open %s. No disk backups.\n");
      return;
    }

  /* save data */

  fprintf(iop, "Items: %d\n", num_items);
  for (n = 0; n < num_items; n++)
    if (item[n].data)
      fprintf(iop, "Item %d: x: %g  y: %g  theta: %g\n", n,
	      (item[n].best_pos_x - 
	       robot_position_absolute_offset_x[item[n].dataset_no])
	      / robot_position_absolute_multiplier,
	      (item[n].best_pos_y - 
	       robot_position_absolute_offset_y[item[n].dataset_no])
	      / robot_position_absolute_multiplier,
	      item[n].best_pos_orientation);

  for (n = 0; n < MAX_NUM_DATASETS; n++)
    if (robot_position_absolute_offset_defined[n]){
      fprintf(iop, "\nrobot_position_absolute_offset_x %d    %g\n",
	      n, robot_position_absolute_offset_x[n]);
      fprintf(iop, "robot_position_absolute_offset_y %d    %g\n",
	      n, robot_position_absolute_offset_y[n]);
    }
  fprintf(iop, "robot_position_absolute_multiplier    %g\n",
	  robot_position_absolute_multiplier);
  
  fprintf(iop, "\nObservationMatrix: %d %d\n",
	  WORLD_SIZE_X, WORLD_SIZE_Y);
  for (i = 0; i < WORLD_SIZE_X; i++){
    for (j = 0; j < WORLD_SIZE_Y; j++)
      fprintf(iop, "%8.5f ", obs_probs[i][j]);
    fprintf(iop, "\n");
  }
  fprintf(iop, "\n");

  print_statistics(iop, 0);
  
  fflush(iop);
  /* fclose(iop); */
}




/************************************************************************
 *
 *   NAME:         load_data(char *filename)
 *                 
 *   FUNCTION:     Loads an observation matrix from file
 *                 
 *                 returns 0, if file does not exist
 *                 returns 1, if success
 *                 returns -1, if file exists, but error occurred
 *                 
 ************************************************************************/

int
load_data(char *filename)
{
  FILE *iop = NULL;
  int error = 0;
  int i, j, n, size_i, size_j, int_value;
  int file_ended = 0;
  float value, max = 0.0;
  char command[128];

  if ((iop = fopen(filename, "r")) == 0){
    fprintf(stderr, "Could not open initialization file %s.\n", filename);
    return 0;
  }
  

  for (file_ended = 0, error = 0; !file_ended && error < 5; ){
    if (fscanf(iop, "%s", command) == EOF)
      file_ended = 1;
    
    else{
      
      /*
       * ObservationMatrix:
       */
      if (!strcmp(command, "ObservationMatrix:")){
	max = 0.0;
	if (fscanf(iop, "%d %d : ", &size_i, &size_j) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error++;
	  file_ended = 1;
	}
	else{
	  if (size_i != WORLD_SIZE_X || size_j != WORLD_SIZE_Y){
	    fprintf(stderr, "ERROR: Format mismatch in %s\n", filename);
	    fprintf(stderr, "\tWorld size %d %d (code) vs. %d %d (file).\n",
		    WORLD_SIZE_X, WORLD_SIZE_Y, size_i, size_j);
	    error = 5;
	  }
	  else{
	    for (i = 0; i < WORLD_SIZE_X; i++)
	      for (j = 0; j < WORLD_SIZE_Y; j++)
		if (fscanf(iop, "%f", &value) == EOF){
		  fprintf(stderr, "ERROR: surprising end of file %s.\n", 
			  filename);
		  error++;
		  file_ended = 1;
		}
		else{		/* okay, we found a valid value */
		  obs_probs[i][j] = value;
		  if (value > max)
		    max = value;
		}
	  }
	}
      }


      else if (!strcmp(command, "robot_position_absolute_multiplier")){
	if (fscanf(iop, "%f", &value) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error++;
	  file_ended = 1;
	}
	else{
	  if (value <= 0.0){
	    fprintf(stderr, "ERROR: Illegal value in %s\n", filename);
	    fprintf(stderr, "\trobot_position_absolute_multiplier %g <= 0.\n",
		    value);
	    error++;
	  }
	  else
	    robot_position_absolute_multiplier = value;
	}
      }
	  


      else if (!strcmp(command, "robot_position_absolute_offset_x")){
	if (fscanf(iop, "%f", &value) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error++;
	  file_ended = 1;
	}
	else if (fscanf(iop, "%d", &int_value) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error++;
	  file_ended = 1;
	}
	else if (int_value < 0 || int_value >= MAX_NUM_DATASETS){
	  fprintf(stderr, "ERROR: illegal offset_x record number ds.\n",
		  int_value);
	  error++;
	}
	else{
	  robot_position_absolute_offset_x[int_value] = value;
	  robot_position_absolute_offset_defined[int_value] = 1;  
	}
      }
      

      else if (!strcmp(command, "robot_position_absolute_offset_y")){
	if (fscanf(iop, "%f", &value) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error++;
	  file_ended = 1;
	}
	else if (fscanf(iop, "%d", &int_value) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error++;
	  file_ended = 1;
	}
	else if (int_value < 0 || int_value >= MAX_NUM_DATASETS){
	  fprintf(stderr, "ERROR: illegal offset_y record number ds.\n",
		  int_value);
	  error++;
	}
	else{
	  robot_position_absolute_offset_y[int_value] = value;
	  robot_position_absolute_offset_defined[int_value] = 1;  
	}
      }
      


      else{
	fprintf(stderr, "ERROR: Unknown keyword [%s] in %s.\n",
		command, filename);
	error++;
      }
    }
  }

  if (error){
    if (error == 5)
      fprintf(stderr, 
	      "File processing aborted, there might be more errors.\n");
    for (i = 0; i < WORLD_SIZE_X; i++)
      for (j = 0; j < WORLD_SIZE_Y; j++)
	obs_probs[i][j] = uniform();
    G_matrix_set_display_range(OBS_PROBS, 0.0, uniform());
    /* G_display_matrix(OBS_PROBS); */
    return -1;
  }
  else{
    G_matrix_set_display_range(OBS_PROBS, 0.0, max);
    fprintf(stderr, "Initialization file %s successfully read.\n", filename);
    /* G_display_matrix(OBS_PROBS); */
    return 1;
  }
}


/************************************************************************
 *
 *   NAME:         swap_out_density
 *                 
 *   FUNCTION:     swaps densities onto disk
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 ************************************************************************/

void
swap_out_density(int n, int density_number)
{
  FILE *iop = NULL;
  data_type *actual_data;
  int i, j, k;
  char filename[128];

  /* ==================================================
   *           Sanity checks
   * ================================================== */

  if (!SWAPPING)
    return;

  if (n < 0 || n >= num_items || !item[n].data){
    fprintf(stderr, "Item %d unknown or not a data item.\n");
    return;
  }

  if (density_number < 0 || density_number > 2){
    fprintf(stderr, "%d is not a valid density number.\n");
    return;
  }


  if (item[n].data->swapped_out[density_number]){
    fprintf(stderr, "Item [%d,%d] is already swapped out. Ignored.\n",
	    n, density_number);
    return;
  }

  /* ==================================================
   *           Swap onto disk
   * ================================================== */

  fprintf(stderr, " >>>%d,%d ", n, density_number);
  
  /* item[n].data->swapped_out[density_number] = 1;  return; */

  sprintf(filename, "maploc3d.swap.%d.%d", n, density_number);

  if ((iop = fopen(filename, "w")) == 0){
    fprintf(stderr, "WARNING: Could not open %s.\n");
    return;
  }


  actual_data = item[n].data;

  for (k = 0; k < NUMBER_ANGLES; k++)
    if (actual_data->non_zero_index_o[density_number][k])
      for (i = 0; i < actual_data->internal_dimension_x[density_number]; i++){
	for (j = 0; j < actual_data->internal_dimension_y[density_number]; j++)
	  fprintf(iop, "%g ", actual_data->probs[density_number][k][i][j]);
	fprintf(iop, "\n");
      }

  fclose(iop);

  for (k = 0; k < NUMBER_ANGLES; k++)
    if (actual_data->non_zero_index_o[density_number][k])
      for (i = 0; i < actual_data->internal_dimension_x[density_number]; i++)
	free(actual_data->probs[density_number][k][i]);

  for (k = 0; k < NUMBER_ANGLES; k++){
    if (actual_data->non_zero_index_o[density_number][k]){
      free(actual_data->probs[density_number][k]);
      actual_data->probs[density_number][k] = NULL;
    }
  }

  item[n].data->swapped_out[density_number] = 1;
  fprintf(stderr, ">>> ");
}


/************************************************************************
 *
 *   NAME:         swap_in_density
 *                 
 *   FUNCTION:     swaps densities back into main memory
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 ************************************************************************/

void
swap_in_density(int n, int density_number)
{
  FILE *iop = NULL;
  data_type *actual_data;
  int i, j, k;
  char filename[128];
  float value;

  /* ==================================================
   *           Sanity checks
   * ================================================== */

  if (!SWAPPING)
    return;

  if (n < 0 || n >= num_items || !item[n].data){
    fprintf(stderr, "Item %d unknown or not a data item.\n");
    return;
  }

  if (density_number < 0 || density_number > 2){
    fprintf(stderr, "%d is not a valid density number.\n");
    return;
  }


  if (!(item[n].data->swapped_out[density_number])){
    fprintf(stderr, "Item [%d,%d] is not swapped out. Ignored.\n",
	    n, density_number);
    return;
  }
  

  /* ==================================================
   *           Swap back into main memory
   * ================================================== */

  fprintf(stderr, " <<<%d,%d", n, density_number);

  sprintf(filename, "maploc3d.swap.%d.%d", n, density_number);
  
  if ((iop = fopen(filename, "r")) == 0){
    fprintf(stderr, "ERROR: Could not open swap file %s.\n");
    exit(-1);
  }

  actual_data = item[n].data;

  for (k = 0; k < NUMBER_ANGLES; k++)
    if (actual_data->non_zero_index_o[density_number][k]){
      actual_data->probs[density_number][k] =
	(float **) malloc(sizeof(float *) *
			  actual_data->internal_dimension_x[density_number]);
      if (actual_data->probs[density_number][k] == NULL){
	fprintf(stderr, "Out of memory in swap_in_density().\n");
	exit(-1);
      }

      for (i = 0; i < actual_data->internal_dimension_x[density_number]; i++){

	actual_data->probs[density_number][k][i] = 
	  (float *) malloc(sizeof(float ) *
			   actual_data->internal_dimension_y[density_number]);
	if (actual_data->probs[density_number][k][i] == NULL){
	  fprintf(stderr, "Out of memory in swap_in_density().\n");
	  exit(-1);
	}

	for (j = 0; j < actual_data->internal_dimension_y[density_number];j++){
	  if (fscanf(iop, "%f", &value) == EOF){
	    fprintf(stderr, "ERROR: surprising end of swap file %s.\n", 
		    filename);
	    exit(-1);
	  }
	  actual_data->probs[density_number][k][i][j] = value;
	}
      }
    }

  fclose(iop);
  fprintf(stderr, "<<< ");

  item[n].data->swapped_out[density_number] = 0;
}




/************************************************************************
 *
 *   NAME:         print_statistics()
 *                 
 *   FUNCTION:     prints the statistics
 *                 
 *   PARAMETERS:   
 *                 
 *                 
 ************************************************************************/

void
print_statistics(FILE *iop, int init)
{
  int i, j;
  float  time_difference; 

  /*
   * initlialization
   */

  if (init){
    if (iteration > 0)
      fprintf(stderr, "WARNING: initialize print_statistics() first!\n");
    for (i = 0; i < MAX_NUM_ITERATIONS; i++){
      num_of_floats_allocated[i] = 0.0;
      num_of_non_zero_floats[i] = 0.0;
      num_floats_bound[i] = 0.0;
      num_entities[i] = 0;
    }
  }


  if (iop == NULL){
    /*
     * Statistics on the screen
     */

    fprintf(stderr, "----------------------------------------------------\n");
    if (init){
      fprintf(stderr, "Memory Statistics for the Action Model\n");
      fprintf(stderr, "num: %d simple: %g actual: %g optimal: %g\n",
	      num_entities[(MAX_NUM_ITERATIONS)],
	      num_floats_bound[(MAX_NUM_ITERATIONS)],
	      num_of_floats_allocated[(MAX_NUM_ITERATIONS)], 
	      num_of_non_zero_floats[(MAX_NUM_ITERATIONS)]);
    }
    else{
      if (iteration == 0)
	j = MAX_NUM_ITERATIONS;
      else 
	j = iteration -1;
      time_difference = 
	((float) (computation_time[iteration].tv_sec -
		  computation_time[j].tv_sec))
	+ (((float) (computation_time[iteration].tv_usec -
		     computation_time[j].tv_usec))
	   /  1000000.0);
      fprintf(stderr, "Memory Statistics for iteration %d:\n", iteration);
      fprintf(stderr, "num: %d simple: %g actual: %g optimal: %g time: %g\n",
	      num_entities[iteration], num_floats_bound[iteration],
	      num_of_floats_allocated[iteration],
	      num_of_non_zero_floats[iteration], time_difference);
    }
    fprintf(stderr,
	    "----------------------------------------------------\n\n");
  }

  else{
    fprintf(iop, "Statistics: %d\n", iteration + 1);
    fprintf(iop, "ActModel: num: %d  simple: %g  actual: %g  optimal: %g\n",
	    num_entities[(MAX_NUM_ITERATIONS)],
	    num_floats_bound[(MAX_NUM_ITERATIONS)],
	    num_of_floats_allocated[(MAX_NUM_ITERATIONS)], 
	    num_of_non_zero_floats[(MAX_NUM_ITERATIONS)]);
    for (i = 0; i <= iteration; i++){
      if (i == 0)
	j = MAX_NUM_ITERATIONS;
      else 
	j = i - 1;
      time_difference = 
	((float) (computation_time[i].tv_sec - computation_time[j].tv_sec))
	+ (((float) (computation_time[i].tv_usec -computation_time[j].tv_usec))
	   /  1000000.0);
      fprintf(iop, "Step: %d num: %d  simple: %g  actual: %g  optimal: %g time: %g\n",
	      i, num_entities[i], num_floats_bound[i],
	      num_of_floats_allocated[i], num_of_non_zero_floats[i],
	      time_difference);
    }
  }
}


/************************************************************************
 *
 *   NAME:         forward_phase
 *                 
 *   FUNCTION:     Refines densities in forward order
 *                 
 *                 
 ************************************************************************/



void 
forward_phase()
{
  int it;

  fprintf(stderr, "Forward...");

  for (it = first_data_item; it >= 0 && it < num_items;
       it = item[it].next_data_item){
    update_density(it, 1);
    fprintf(stderr, "%d...", it);
    mouse_test_loop();
  }
  fprintf(stderr, "done.\n");
}



/************************************************************************
 *
 *   NAME:         backward_phase
 *                 
 *   FUNCTION:     Refines densities in backwards order
 *                 
 *                 
 ************************************************************************/


void 
backward_phase()
{
  int it;

  fprintf(stderr, "Backward...");
  for (it = last_data_item; it >= 0; it = item[it].prev_data_item){
    if (it <= num_items){
      update_density(it, 2);
      fprintf(stderr, "%d...", it);
      mouse_test_loop();
    }
  }
  fprintf(stderr, "done.\n");
}



/************************************************************************
 *
 *   NAME:         intergration_phase
 *                 
 *   FUNCTION:     Integrates result of forward and backward phase
 *                 
 *                 
 ************************************************************************/


void 
integration_phase()
{
  int it;

  fprintf(stderr, "Integrating...");

  for (it = first_data_item; it >= 0 && it < num_items;
       it = item[it].next_data_item){
    update_density(it, 0);
    fprintf(stderr, "%d...", it);
    mouse_test_loop();
  }
  fprintf(stderr, "done.\n");
}



/************************************************************************
 *
 *   NAME:         alarm_handler()
 *                 
 *   FUNCTION:     checks mouse events in regular intervals
 *                 
 *   PARAMETERS:   command lines only (if any)
 *                 
 *                 
 ************************************************************************/

void
alarm_handler()
{
  mouse_test_loop();
  if (button_alarm)
    ualarm(200, 0);  
}



/************************************************************************
 *
 *   NAME:         main()
 *                 
 *   FUNCTION:     main program - initialization and main loop
 *                 
 *   PARAMETERS:   command lines only (if any)
 *                 
 *                 
 ************************************************************************/

int
main(int argc, char *argv[])
{
  float err = 5.0;

  signal(SIGINT,  (void *) exit_proc);
  signal(SIGTERM, (void *) exit_proc);
  signal(SIGALRM, (void *) alarm_handler);

  init_graphics();

  if (load_data(INI_FILENAME) == -1) exit(-1); 
  
  /* wean_data_short(0);  */
  
<<<<<<< main3d.c
  /*square_data_short(); */
=======
   square_data(); 
>>>>>>> 1.52
  
  /* wean_data_long(0); */
  
  if (!load_raw_data("maploc3d.dat")) exit(-1);  
  

  /* check_forward(2);  */
  /* check_items(2);   */

  G_display_all();
  print_statistics(NULL, 1);
  fprintf(stderr, "Successfully memorized %d items.\n", num_items);



  for (iteration = 0; iteration < MAX_NUM_ITERATIONS; iteration++){
    if (data) save_raw_data(save_file_name);
    forward_phase(); 
    backward_phase();
    integration_phase();
    compute_observation_matrix();
    save_data(DUMP_FILENAME);
    print_statistics(NULL, 0);

  }  

  if (button_alarm)
    ualarm(200, 0);  

  for (;;)
    sleep(1);

}


