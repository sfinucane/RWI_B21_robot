
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <signal.h>


#include "EZX11.h"
#include "o-graphics.h"


#include "rai.h"
#include "raiClient.h"
#include "baseClient.h"
#include "sonarClient.h"
#include "tactileClient.h"
#include "rai.h"
#include "raiClient.h"
#include "cameraClient.h"
#include "buttonClient.h"
#include "pantiltClient.h"

#include "main.h"




/************************************************************************\
 ************************************************************************
\************************************************************************/


#define CONTROL_NOISE_FACTOR 0.4

/************************************************************************\
 ************************************************************************
\************************************************************************/

int OBS_PROBS;
int QUIT_BUTTON;
int PATH_DISPLAY[3];



int run_time_optimize = 1;



#define uniform() (1.0 / ((float) ((WORLD_SIZE_X) * (WORLD_SIZE_Y))))



/************************************************************************\
 ************************************************************************
\************************************************************************/

typedef struct {
  float probs[NUM_WINDOWS_PER_DATA_ITEM][WORLD_SIZE_X][WORLD_SIZE_Y];
  float max_prob;
  int   min_non_zero_index_x[NUM_WINDOWS_PER_DATA_ITEM];
  int   max_non_zero_index_x[NUM_WINDOWS_PER_DATA_ITEM];
  int   min_non_zero_index_y[NUM_WINDOWS_PER_DATA_ITEM];
  int   max_non_zero_index_y[NUM_WINDOWS_PER_DATA_ITEM];
  int   delta_x, delta_y;
  int   error_x, error_y;
  int   correct_x, correct_y;
  int   measured_x, measured_y;
  int   best_x, best_y;
  int   landmark_found, landmark_rel_x, landmark_rel_y;
  float perceived_dist;
  int   button_window_id;
  int   probs_window_id[NUM_WINDOWS_PER_DATA_ITEM];
  int   markers_window_id;
  int   bounding_box_window_id[NUM_WINDOWS_PER_DATA_ITEM];
  int   new_episode;
} data_type;

data_type data[MAX_NUM_STEPS];
int       num_data = 0;


float obs_probs[WORLD_SIZE_X][WORLD_SIZE_Y];

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
  static char *myfonts[] = {"5x8", "6x10", "7x13bold", 
			    "9x15bold", "10x20", "12x24", 
			    "lucidasans-bold-24"};

  int i, j;

 
  G_initialize_graphics("MAP-LOC", 50.0, 10.0, C_TURQUOISE4); /* 50.0 */
  G_set_matrix_display_style(1);
  G_initialize_fonts(7, myfonts);
  G_markers_display_style = 0; /* 1=fill them up */


  {
   
    /******** QUIT_BUTTON ****************************/
    int switch_num                      = 2;
    static float switch_pos[]           = 
    {0.0, 2.0 * (MATRIX_SEPARATOR) + 3.0 * (MATRIX_SIZE_X), 
     -(BUTTON_SIZE_Y), 0.0};
    static char *switch_texts[]         =
    {"QUIT", "(c) S. Thrun, 1997"};
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
    /******** OBS_PROBS ****************************/
    float  matrix_pos[4]                     = 
    {0.0, 2.0 * (MATRIX_SEPARATOR) + 3.0 * (MATRIX_SIZE_X), 
     - (3.0 * ((MATRIX_SIZE_X) + (MATRIX_SEPARATOR)))-(BUTTON_SIZE_Y),
     0.0-(BUTTON_SIZE_Y) - (MATRIX_SEPARATOR)};
    static char *matrix_text             = "Oops - should not be here";
    static int matrix_font               = 0;
    static int matrix_colors[]           = {NO_COLOR, C_GREY30, NO_COLOR}; 

    
    OBS_PROBS =
      G_create_matrix_object(matrix_pos, matrix_text, 
			     &(obs_probs[0][0]),
			     NULL, WORLD_SIZE_X, WORLD_SIZE_Y,
			     0.0, 1.0, matrix_colors, matrix_font);

  }

  {
    /******** PATH_DISPLAY ****************************/
    int i;
    float  mar_pos[4]                     = 
    {0.0, 2.0 * (MATRIX_SEPARATOR) + 3.0 * (MATRIX_SIZE_X), 
     - (3.0 * ((MATRIX_SIZE_X) + (MATRIX_SEPARATOR)))-(BUTTON_SIZE_Y),
     0.0-(BUTTON_SIZE_Y) - (MATRIX_SEPARATOR)};
    int num_mar                            = 1;
    static char *text_mar[]                = {""};
    static int mar_frame_color             = C_GREY30;
    static int mar_background_color        = NO_COLOR;
    static int mar_foreground_color[]      = {C_RED, C_LAWNGREEN, C_CADETBLUE};
    static int mar_text_color[]            = {NO_COLOR};
    static int mar_fonts[]                 = {2};
   

    for (i = 0; i < 3; i++)
     
      PATH_DISPLAY[i] =
	G_create_markers_object(mar_pos, 1, 0.75,
				0.0, ((float) WORLD_SIZE_X),
				0.0, ((float) WORLD_SIZE_Y),
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, &(mar_foreground_color[i]), 
				mar_text_color, mar_fonts);
  }


  for (i = 0; i < WORLD_SIZE_X; i++)
    for (j = 0; j < WORLD_SIZE_Y; j++)
      obs_probs[i][j] = uniform();
}

void
register_density_window(float probs[NUM_WINDOWS_PER_DATA_ITEM]
			[WORLD_SIZE_X][WORLD_SIZE_Y],
			int *button_window_id,
			int probs_window_id[NUM_WINDOWS_PER_DATA_ITEM],
			int *markers_window_id,
			int bounding_box_window_id[NUM_WINDOWS_PER_DATA_ITEM])
{
  static float current_x = 3.0 * ((MATRIX_SEPARATOR) + (MATRIX_SIZE_X));
  static float current_y = 0.0;
  static int num_x = 3;
  float size_x, size_y;
  static char *matrix_text             = "Oops - should not be here";
  static int matrix_font               = 0;
  static int matrix_colors[]           = {NO_COLOR, C_GREY30, NO_COLOR}; 
  float  matrix_pos[4];
  int num_mar                            = 5;
  static char *text_mar[]                = {"", "", "", "", ""};
  static int mar_frame_color             = NO_COLOR;
  static int mar_background_color        = NO_COLOR;
  static int mar_foreground_color[]      = 
  {C_RED, C_LAWNGREEN, C_YELLOW, C_CADETBLUE, C_SIENNA4};
  static int mar_text_color[]            = 
  {NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR, NO_COLOR};
  static int mar_fonts[]                 = {2, 2, 2, 2, 2};
  int switch_num                      = 3;
  static char *switch_texts[]         = {"", "> > >", "< < <"};
  static int switch_fonts[]           = {2,2,2};
  static int switch_background_color[]= {C_GREY90, C_RED, C_RED};
  static int switch_frame_color[]     = {C_GREY30, C_GREY70, C_GREY70};
  static int switch_text_color[]      = {C_BLACK, C_WHITE, C_WHITE};
  int i;


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
			     &(probs[i][0][0]), 
			     NULL, WORLD_SIZE_X, WORLD_SIZE_Y,
			     0.0, 1.0, matrix_colors, matrix_font);


    G_matrix_set_display_range(probs_window_id[i], 0.0, uniform());
    
    if (i == 0)
      *markers_window_id = 
	G_create_markers_object(matrix_pos, 0, 1.5,
				0.0, ((float) WORLD_SIZE_X),
				0.0, ((float) WORLD_SIZE_Y),
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, mar_foreground_color, 
				mar_text_color, mar_fonts);
    bounding_box_window_id[i] =
      G_create_markers_object(matrix_pos, 1, 0.0,
			      0.0, ((float) WORLD_SIZE_X),
			      0.0, ((float) WORLD_SIZE_Y),
			      num_mar, text_mar, mar_background_color, 
			      mar_frame_color, mar_foreground_color, 
			      mar_text_color, mar_fonts);

    matrix_pos[3] = matrix_pos[2] - (MATRIX_SEPARATOR);
  }
}



void
internal_add_data_item(int delta_x, int delta_y,
		       int error_x, int error_y, int new_episode)
{
  int i, j, k;

  if (num_data >= MAX_NUM_STEPS){
    fprintf(stderr, "ERROR: MEMORY OVERFLOW\n");
    return;
  }

  data[num_data].delta_x = delta_x;
  data[num_data].delta_y = delta_y;
  data[num_data].error_x = error_x;
  data[num_data].error_y = error_y;

  data[num_data].landmark_found = 0;
  data[num_data].landmark_rel_x = 0;
  data[num_data].landmark_rel_y = 0;




  if (num_data == 0 || new_episode){
    data[num_data].correct_x      = delta_x;
    data[num_data].correct_y      = delta_y;
    data[num_data].measured_x     = delta_x;
    data[num_data].measured_y     = delta_y;
    data[num_data].perceived_dist = 0.0;
    data[num_data].new_episode    = 1;
  }
  else{
    data[num_data].correct_x      = data[num_data-1].correct_x + delta_x;
    data[num_data].correct_y      = data[num_data-1].correct_y + delta_y;
    data[num_data].measured_x     = 
      data[num_data-1].measured_x + delta_x + error_x;
    data[num_data].measured_y     = 
      data[num_data-1].measured_y + delta_y + error_y;
    data[num_data].perceived_dist = 
      sqrt((((float) (data[num_data].delta_x + data[num_data].error_x)) *
	    ((float) (data[num_data].delta_x + data[num_data].error_x))) +
	   (((float) (data[num_data].delta_y + data[num_data].error_y)) *
	    ((float) (data[num_data].delta_y + data[num_data].error_y))));
  data[num_data].new_episode      = 0;    
  }

  data[num_data].best_x = data[num_data].measured_x;
  data[num_data].best_y = data[num_data].measured_y;



  for (i = 0; i < NUM_WINDOWS_PER_DATA_ITEM; i++)
    for (j = 0; j < WORLD_SIZE_X; j++)
      for (k = 0; k < WORLD_SIZE_Y; k++)
	data[num_data].probs[i][j][k] = uniform();

  data[num_data].max_prob = uniform();

  if (run_time_optimize){
    for (i = 0; i < NUM_WINDOWS_PER_DATA_ITEM; i++){    
      data[num_data].min_non_zero_index_x[i] = (WORLD_SIZE_X) + 1;
      data[num_data].max_non_zero_index_x[i] = -1;
      data[num_data].min_non_zero_index_y[i] = (WORLD_SIZE_Y) + 1;
      data[num_data].max_non_zero_index_y[i] = -1;
    }
  }
  else{
    for (i = 0; i < NUM_WINDOWS_PER_DATA_ITEM; i++){    
      data[num_data].min_non_zero_index_x[i] = 0;
      data[num_data].max_non_zero_index_x[i] = (WORLD_SIZE_X) - 1;
      data[num_data].min_non_zero_index_y[i] = 0;
      data[num_data].max_non_zero_index_y[i] = (WORLD_SIZE_Y) - 1;
    }
  }
  
  
  register_density_window(data[num_data].probs,
			  &(data[num_data].button_window_id),
			  data[num_data].probs_window_id,
			  &(data[num_data].markers_window_id),
			  data[num_data].bounding_box_window_id);
  

  num_data++;

}

void
set_initial_position(int pos_x, int pos_y)
{
  internal_add_data_item(pos_x, pos_y, 0, 0, 1);
}

void
add_data_item(int delta_x, int delta_y,
	      int error_x, int error_y)
{
  internal_add_data_item(delta_x, delta_y,
			 error_x, error_y, 0);
}


void
saw_landmark(int rel_x, int rel_y)
{
  float dist;

  if (num_data == 0)
    return;

  dist = sqrt((((float) rel_x) * ((float) rel_x)) +
	      (((float) rel_y) * ((float) rel_y)));

  if (dist > VISUAL_RANGE){
    fprintf(stderr, "ERROR: Cannot see that far: %d %d (%g)\n",
	    rel_x, rel_y, dist);
    return;
  }

  data[num_data-1].landmark_found = 1;
  data[num_data-1].landmark_rel_x = rel_x;
  data[num_data-1].landmark_rel_y = rel_y;

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
  float mouse_x, mouse_y;
  int number;
  int test;




  /****************** CHECK FOR MOUSE EVENT *******************/

  test = G_test_mouse(0);

  if (test == 1){
    mouse_events = G_evaluate_mouse(&mouse_x, &mouse_y, 
				    &button, &num_mouse_events);




    
    /****************** EVALUATE MOUSE EVENT *******************/

    
    
    if (G_mouse_event_at(QUIT_BUTTON, mouse_events, &number)){
      G_display_switch(QUIT_BUTTON, 1);
      usleep(200000);
      fprintf(stderr, "\n");
      exit(0);
    }
    /* insert new graphics object here XXXXX */
    
    
    else if (button == RIGHT_BUTTON){
      G_display_all();
    }
  }
  
  return test;
}



/************************************************************************
 *
 *   NAME:         normalize_density
 *                 
 *   FUNCTION:     normalizes a density, and computes the bounds (if
 *                 the optimization flag has been set)
 *                 n=data item, k=number of density
 *                 
 ************************************************************************/



void 
normalize_density(int n, int k)
{
  float sum;
  int i, j;

  /*
   * Normalize, and determine new bounds
   */
  
  sum = 0.0;
  if (run_time_optimize){
    data[n].min_non_zero_index_x[k] = (WORLD_SIZE_X) + 1;
    data[n].max_non_zero_index_x[k] = -1;
    data[n].min_non_zero_index_y[k] = (WORLD_SIZE_Y) + 1;
    data[n].max_non_zero_index_y[k] = -1;
  }
  for (i = 0; i < WORLD_SIZE_X; i++)
    for (j = 0; j < WORLD_SIZE_Y; j++){
      if (data[n].probs[k][i][j] > 0.0){
	if (run_time_optimize){
	  if (i < data[n].min_non_zero_index_x[k])
	    data[n].min_non_zero_index_x[k] = i;
	  if (i > data[n].max_non_zero_index_x[k])
	    data[n].max_non_zero_index_x[k] = i;
	  if (j < data[n].min_non_zero_index_y[k])
	    data[n].min_non_zero_index_y[k] = j;
	  if (j > data[n].max_non_zero_index_y[k])
	    data[n].max_non_zero_index_y[k] = j;
	}
	sum += data[n].probs[k][i][j];
      }
    }
  if (sum > 0.0){
    sum = 1.0 / sum;
    for (i = 0; i < WORLD_SIZE_X; i++)
      for (j = 0; j < WORLD_SIZE_Y; j++)
	data[n].probs[k][i][j] *= sum;
  }
}


/************************************************************************
 *
 *   NAME:       display_density  
 *                 
 *   FUNCTION:    displays a density. n=data item, k=number of density
 *                 
 *                 
 ************************************************************************/


void 
display_density(int n, int k)
{
  float max;
  int i, j;
  
  /*
   * display the result
   */
  
  max = 0.0;
  for (i = 0; i < WORLD_SIZE_X; i++)
    for (j = 0; j < WORLD_SIZE_Y; j++)
      if (data[n].probs[k][i][j] > max)
	max = data[n].probs[k][i][j];
  G_matrix_set_display_range(data[n].probs_window_id[k], 0.0, max);
  G_display_matrix(data[n].probs_window_id[k]);
  
  
  G_markers_display_style = 1; /* 1=fill them up */
  G_clear_markers(data[n].bounding_box_window_id[k]);
  G_add_marker(data[n].bounding_box_window_id[k],
	       ((float) data[n].min_non_zero_index_x[k]),
	       ((float) data[n].min_non_zero_index_y[k]), 4);
  G_add_marker(data[n].bounding_box_window_id[k],
	       ((float) (data[n].max_non_zero_index_x[k]+1)),
	       ((float) data[n].min_non_zero_index_y[k]), 4);
  G_add_marker(data[n].bounding_box_window_id[k],
	       ((float) (data[n].max_non_zero_index_x[k]+1)),
	       ((float) (data[n].max_non_zero_index_y[k]+1)), 4);
  G_add_marker(data[n].bounding_box_window_id[k],
	       ((float) data[n].min_non_zero_index_x[k]),
	       ((float) (data[n].max_non_zero_index_y[k]+1)), 4);
  G_add_marker(data[n].bounding_box_window_id[k],
	       ((float) data[n].min_non_zero_index_x[k]),
	       ((float) data[n].min_non_zero_index_y[k]), 4);
  G_display_markers(data[n].bounding_box_window_id[k]);
  
  
  G_display_switch(data[n].button_window_id, 0);
  
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
update_density()
{
  int i, j, k, m, num_mults, n, x, y;
  float action_dist, f_dist, f_dist_x, f_dist_y, max;
  float factor;
  int x_to, y_to, x_from, y_from, x_land, y_land;
  int x_from_0, x_from_1, y_from_0, y_from_1;
  int x_to_0, x_to_1, y_to_0, y_to_1;
  int dist_x, dist_y;
  float best_prob;
  int   int_visual_range, delta_x, delta_y;



  /* ==================================================
   *           Forward phase
   * ================================================== */
  

  for (n = 0; n < num_data; n++){
    G_display_switch(data[n].button_window_id, 1);
    
    /*
     * CASE 1: First step, position is known 
     */

    if (n == 0){			/* first data item */
      for (i = 0; i < WORLD_SIZE_X; i++)
	for (j = 0; j < WORLD_SIZE_Y; j++)
	  data[n].probs[1][i][j] = 0.0;
      data[n].probs[1][data[n].correct_x][data[n].correct_y] = 1.0;

      if (run_time_optimize){
	data[n].min_non_zero_index_x[1] = data[n].correct_x;
	data[n].max_non_zero_index_x[1] = data[n].correct_x;
	data[n].min_non_zero_index_y[1] = data[n].correct_y;
	data[n].max_non_zero_index_y[1] = data[n].correct_y;
      }
    }


    /*
     * CASE 2: New episode, position is unknown 
     */    

    else if (data[n].new_episode){
      for (i = 0; i < WORLD_SIZE_X; i++)
	for (j = 0; j < WORLD_SIZE_Y; j++)
	  data[n].probs[1][i][j] = uniform();

      if (run_time_optimize){
	data[n].min_non_zero_index_x[1] = 0;
	data[n].max_non_zero_index_x[1] = WORLD_SIZE_X - 1;
	data[n].min_non_zero_index_y[1] = 0;
	data[n].max_non_zero_index_y[1] = WORLD_SIZE_Y - 1;
      }
    }

    /*
     * CASE 3: Position estimate is based on the previous position 
     */    

    else{

      /* 
       * Compute the probabilities for time n
       */

      for (i = 0; i < WORLD_SIZE_X; i++)
	for (j = 0; j < WORLD_SIZE_Y; j++)
	  data[n].probs[1][i][j] = 0.0;
      
      action_dist = data[n].perceived_dist * CONTROL_NOISE_FACTOR;
      for (x_to = 0; x_to < WORLD_SIZE_X; x_to++)
	for (y_to = 0; y_to < WORLD_SIZE_Y; y_to++){
	  x_from_0 = x_to - data[n].delta_x - data[n].error_x - 
	    ((int) (action_dist));
	  x_from_1 = x_to - data[n].delta_x - data[n].error_x + 
	    ((int) (action_dist));
	  y_from_0 = y_to - data[n].delta_y - data[n].error_y - 
	    ((int) (action_dist));
	  y_from_1 = y_to - data[n].delta_y - data[n].error_y + 
	    ((int) (action_dist));
	  
	  /* the following is done for run-time optimization */
	  if (x_from_0 < data[n-1].min_non_zero_index_x[1])
	    x_from_0 = data[n-1].min_non_zero_index_x[1];
	  if (x_from_1 > data[n-1].max_non_zero_index_x[1])
	    x_from_1 = data[n-1].max_non_zero_index_x[1];
	  if (y_from_0 < data[n-1].min_non_zero_index_y[1])
	    y_from_0 = data[n-1].min_non_zero_index_y[1];
	  if (y_from_1 > data[n-1].max_non_zero_index_y[1])
	    y_from_1 = data[n-1].max_non_zero_index_y[1];
	  
	  /* 
	   * Compute the probability for the possibility "x_to/y_to"
	   * contribution from the adjacent distribution
	   */
	  for (x_from = x_from_0; x_from <= x_from_1; x_from++)
	    for (y_from = y_from_0; y_from <= y_from_1; y_from++)
	      if (data[n-1].probs[1][x_from][y_from] > 0.0){
		f_dist_x = (float) (x_to - data[n].delta_x - 
				    data[n].error_x - x_from);
		f_dist_y = (float) (y_to - data[n].delta_y - 
				    data[n].error_y - y_from);
		f_dist   = sqrt((f_dist_x * f_dist_x) +
				(f_dist_y * f_dist_y));
		factor = 1.0 - (f_dist / action_dist);
		if (factor > 0.0)
		  data[n].probs[1][x_to][y_to] += 
		    factor * data[n-1].probs[1][x_from][y_from];
	      }
	}


      /*
       * Normalize, and determine new bounds
       */

      normalize_density(n, 1);
      
    }
    
    /*
     * If landmark found, multiply with landmark/observation probabilities
     */

    if (data[n].landmark_found){
      for (x =  data[n].min_non_zero_index_x[1];
	   x <= data[n].max_non_zero_index_x[1]; x++){
	x_land = x + data[n].landmark_rel_x;
	for (y =  data[n].min_non_zero_index_y[1];
	     y <= data[n].max_non_zero_index_y[1]; y++){
	  y_land = y + data[n].landmark_rel_y;
	  if (x_land >= 0 && x_land < WORLD_SIZE_X &&
	      y_land >= 0 && y_land < WORLD_SIZE_Y)
	    data[n].probs[1][x][y] *= obs_probs[x_land][y_land];
	  else
	    data[n].probs[1][x][y] = 0.0;
	}
      }
      normalize_density(n, 1);
    }


    /*
     * display the result
     */

    display_density(n, 1);
  }

  
  /* ==================================================
   *           Backward phase
   * ================================================== */

  
  for (n = num_data - 1; n >= 0; n--){
    G_display_switch(data[n].button_window_id, 2);

    /*
     * CASE 1: Last item of an episode, position unknown
     */    

    if (n == num_data - 1 || data[n+1].new_episode){ /* last data item */
      for (i = 0; i < WORLD_SIZE_X; i++)
	for (j = 0; j < WORLD_SIZE_Y; j++)
	  data[n].probs[2][i][j] = uniform();

      if (run_time_optimize){
	data[n].min_non_zero_index_x[2] = 0;
	data[n].max_non_zero_index_x[2] = WORLD_SIZE_X - 1;
	data[n].min_non_zero_index_y[2] = 0;
	data[n].max_non_zero_index_y[2] = WORLD_SIZE_Y - 1;
      }
    }

    /*
     * CASE 2: Position estimate is based on the successor position 
     *         no landmark information
     */    

    else if (!data[n+1].landmark_found){

      /* 
       * Compute the probabilities for time n
       */

      for (i = 0; i < WORLD_SIZE_X; i++)
	for (j = 0; j < WORLD_SIZE_Y; j++)
	  data[n].probs[2][i][j] = 0.0;
      
      action_dist = data[n+1].perceived_dist * CONTROL_NOISE_FACTOR;
      for (x_to = 0; x_to < WORLD_SIZE_X; x_to++)
	for (y_to = 0; y_to < WORLD_SIZE_Y; y_to++){

	  x_from_0 = x_to + data[n+1].delta_x + data[n+1].error_x - 
	    ((int) (action_dist));
	  x_from_1 = x_to + data[n+1].delta_x + data[n+1].error_x +
	    ((int) (action_dist));
	  y_from_0 = y_to + data[n+1].delta_y + data[n+1].error_y - 
	    ((int) (action_dist));
	  y_from_1 = y_to + data[n+1].delta_y + data[n+1].error_y + 
	    ((int) (action_dist));
	  
	  /* the following is done for run-time optimization */
	  if (x_from_0 < data[n+1].min_non_zero_index_x[2])
	    x_from_0 = data[n+1].min_non_zero_index_x[2];
	  if (x_from_1 > data[n+1].max_non_zero_index_x[2])
	    x_from_1 = data[n+1].max_non_zero_index_x[2];
	  if (y_from_0 < data[n+1].min_non_zero_index_y[2])
	    y_from_0 = data[n+1].min_non_zero_index_y[2];
	  if (y_from_1 > data[n+1].max_non_zero_index_y[2])
	    y_from_1 = data[n+1].max_non_zero_index_y[2];
	  
	  /* 
	   * Compute the probability for the possibility "x_to/y_to"
	   * contribution from the adjacent distribution
	   */

	  for (x_from = x_from_0; x_from <= x_from_1; x_from++)
	    for (y_from = y_from_0; y_from <= y_from_1; y_from++)
	      if (data[n+1].probs[2][x_from][y_from] > 0.0){
		f_dist_x = (float) (x_to + data[n+1].delta_x +
				    data[n+1].error_x - x_from);
		f_dist_y = (float) (y_to + data[n+1].delta_y +
				    data[n+1].error_y - y_from);
		f_dist   = sqrt((f_dist_x * f_dist_x) + (f_dist_y * f_dist_y));
		factor = 1.0 - (f_dist / action_dist);
		if (factor > 0.0)
		  data[n].probs[2][x_to][y_to] += 
		    factor * data[n+1].probs[2][x_from][y_from];
	      }
	}
      /*
       * Normalize, and determine new bounds
       */

      normalize_density(n, 2);
      
    }

    /*
     * CASE 3: Position estimate is based on the successor position 
     *         with landmark information
     */    

    else{

      /* 
       * Compute the probabilities for time n
       */

      for (i = 0; i < WORLD_SIZE_X; i++)
	for (j = 0; j < WORLD_SIZE_Y; j++)
	  data[n].probs[2][i][j] = 0.0;
      
      action_dist = data[n+1].perceived_dist * CONTROL_NOISE_FACTOR;
      for (x_to = 0; x_to < WORLD_SIZE_X; x_to++)
	for (y_to = 0; y_to < WORLD_SIZE_Y; y_to++){

	  x_from_0 = x_to + data[n+1].delta_x + data[n+1].error_x - 
	    ((int) (action_dist));
	  x_from_1 = x_to + data[n+1].delta_x + data[n+1].error_x +
	    ((int) (action_dist));
	  y_from_0 = y_to + data[n+1].delta_y + data[n+1].error_y - 
	    ((int) (action_dist));
	  y_from_1 = y_to + data[n+1].delta_y + data[n+1].error_y + 
	    ((int) (action_dist));
	  
	  /* the following is done for run-time optimization */
	  if (x_from_0 < data[n+1].min_non_zero_index_x[2])
	    x_from_0 = data[n+1].min_non_zero_index_x[2];
	  if (x_from_1 > data[n+1].max_non_zero_index_x[2])
	    x_from_1 = data[n+1].max_non_zero_index_x[2];
	  if (y_from_0 < data[n+1].min_non_zero_index_y[2])
	    y_from_0 = data[n+1].min_non_zero_index_y[2];
	  if (y_from_1 > data[n+1].max_non_zero_index_y[2])
	    y_from_1 = data[n+1].max_non_zero_index_y[2];
	  
	  /* 
	   * Compute the probability for the possibility "x_to/y_to"
	   * contribution from the adjacent distribution
	   */

	  for (x_from = x_from_0; x_from <= x_from_1; x_from++){
	    x_land = x_from + data[n+1].landmark_rel_x;
	    for (y_from = y_from_0; y_from <= y_from_1; y_from++){
	      y_land = y_from + data[n+1].landmark_rel_y;
	      if (x_land >= 0 && x_land < WORLD_SIZE_X &&
		  y_land >= 0 && y_land < WORLD_SIZE_Y &&
		  data[n+1].probs[2][x_from][y_from] > 0.0){
		f_dist_x = (float) (x_to + data[n+1].delta_x +
				    data[n+1].error_x - x_from);
		f_dist_y = (float) (y_to + data[n+1].delta_y +
				    data[n+1].error_y - y_from);
		f_dist   = sqrt((f_dist_x * f_dist_x) + (f_dist_y * f_dist_y));
		factor = 1.0 - (f_dist / action_dist);
		if (factor > 0.0)
		  data[n].probs[2][x_to][y_to] += 
		    (factor * data[n+1].probs[2][x_from][y_from]
		     * obs_probs[x_land][y_land]);
	      }
	    }
	  }
	}
      /*
       * Normalize, and determine new bounds
       */

      normalize_density(n, 2);

    }

    /*
     * display the result
     */

    display_density(n, 2);

  }




  /* ==================================================
   *           Compute state probabilities
   * ================================================== */
  
  best_prob = 0.0;
  for (n = 0; n < num_data; n++){
    for (i = 0; i < WORLD_SIZE_X; i++)
      for (j = 0; j < WORLD_SIZE_Y; j++){
	data[n].probs[0][i][j] = 
	  data[n].probs[1][i][j] * data[n].probs[2][i][j];
	if (data[n].probs[0][i][j] > best_prob){
	  best_prob = data[n].probs[0][i][j];
	  data[n].best_x = i;	  
	  data[n].best_y = j;	  
	}
      }
    
    normalize_density(n, 0);
    display_density(n, 0);

    /*
     * display measures, best, and correct location
     */

    G_clear_markers(data[n].markers_window_id);
    G_add_marker(data[n].markers_window_id, 
		 ((float) (data[n].correct_x + 0.5)),
		 ((float) (data[n].correct_y + 0.5)), 0);
    G_add_marker(data[n].markers_window_id, 
		 ((float) (data[n].measured_x + 0.5)),
		 ((float) (data[n].measured_y + 0.5)), 1);
    if (data[n].landmark_found)
      G_add_marker(data[n].markers_window_id, 
		   ((float) (data[n].measured_x + 
			     data[n].landmark_rel_x + 0.5)),
		   ((float) (data[n].measured_y + 
			     data[n].landmark_rel_y + 0.5)),
		   2);
    G_add_marker(data[n].markers_window_id, 
		 ((float) (data[n].best_x + 0.5)),
		 ((float) (data[n].best_y + 0.5)), 3);
    G_markers_display_style = 0; /* 1=fill them up */
    G_display_markers(data[n].markers_window_id);

  }



  /* ==================================================
   *           Compute observation probabilities
   * ================================================== */

  {
    float obs_probs_1[WORLD_SIZE_X][WORLD_SIZE_Y];
    float obs_probs_2[WORLD_SIZE_X][WORLD_SIZE_Y];
    
    for (x = 0; x < WORLD_SIZE_X; x++)
      for (y = 0; y < WORLD_SIZE_Y; y++){
	obs_probs_1[x][y] = uniform();
	obs_probs_2[x][y] = 1.0;
      }
    
    int_visual_range = ((int) ((VISUAL_RANGE) + 0.5));
    
    for (n = 0; n < num_data; n++)
      for (x = data[n].min_non_zero_index_x[0];
	   x <= data[n].max_non_zero_index_x[0]; x++)
	for (delta_x = -int_visual_range; 
	     delta_x <= int_visual_range; delta_x++)
	  if (x + delta_x >= 0 && x + delta_x < WORLD_SIZE_X)
	    for (y = data[n].min_non_zero_index_y[0];
		 y <= data[n].max_non_zero_index_y[0]; y++)
	      if (data[n].probs[0][x][y] > 0.0)
		for (delta_y = -int_visual_range; 
		     delta_y <= int_visual_range; delta_y++)
		  if (y + delta_y >= 0 && y + delta_y < WORLD_SIZE_Y){
		    f_dist = sqrt((((float) delta_x) * ((float) delta_x)) +
				  (((float) delta_y) * ((float) delta_y)));
		    if (f_dist <= VISUAL_RANGE){
		      if (data[n].landmark_found && 
			  delta_x == data[n].landmark_rel_x &&
			  delta_y == data[n].landmark_rel_y)
			obs_probs_1[x+delta_x][y+delta_y] += 
			  data[n].probs[0][x][y];
		      obs_probs_2[x+delta_x][y+delta_y] +=
			data[n].probs[0][x][y];
		    }
		  }
      
    for (x = 0; x < WORLD_SIZE_X; x++)
      for (y = 0; y < WORLD_SIZE_Y; y++)
	if (obs_probs_2[x][y] == 0.0)
	  obs_probs[x][y] = 0.0;
	else
	  obs_probs[x][y] = obs_probs_1[x][y] / obs_probs_2[x][y];

    
    max = 0.0;
    for (x = 0; x < WORLD_SIZE_X; x++)
      for (y = 0; y < WORLD_SIZE_Y; y++)
	if (obs_probs[x][y] > max)
	  max = obs_probs[x][y];
    
    G_matrix_set_display_range(OBS_PROBS, 0.0, max);
    G_display_matrix(OBS_PROBS);

  }

#ifdef OLD_OLD
#define DEFAULT_OBS_PROB 0.02

  for (i = 0; i < WORLD_SIZE_X; i++)
    for (j = 0; j < WORLD_SIZE_Y; j++)
      obs_probs[i][j] = DEFAULT_OBS_PROB;


  for (n = 0; n < num_data; n++){
    if (data[n].landmark_found)
      for (i = data[n].min_non_zero_index_x[0];
	   i <= data[n].max_non_zero_index_x[0]; i++){
	x = i + data[n].landmark_rel_x;
	if (x >= 0 && x < WORLD_SIZE_X)
	  for (j = data[n].min_non_zero_index_y[0];
	       j <= data[n].max_non_zero_index_y[0]; j++){
	    y = j + data[n].landmark_rel_y;
	    if (y >= 0 && y < WORLD_SIZE_Y)
	      obs_probs[x][y] += data[n].probs[0][i][j];
	  }
      }
  }
  max = 0.0;
  for (i = 0; i < WORLD_SIZE_X; i++)
    for (j = 0; j < WORLD_SIZE_Y; j++)
      if (obs_probs[i][j] > max)
	max = obs_probs[i][j];

  G_matrix_set_display_range(OBS_PROBS, 0.0, max);
  G_display_matrix(OBS_PROBS);

#endif
  




  /* ==================================================
   *           Display path
   * ================================================== */
  


  for (i = 0; i < 3; i++)
    G_clear_markers(PATH_DISPLAY[i]);
  for (m = 0; m < num_data; m++){
    G_add_marker(PATH_DISPLAY[0], 
		 ((float) data[m].correct_x) + 0.5,
		 ((float) data[m].correct_y) + 0.5, 0); 
    G_add_marker(PATH_DISPLAY[1], 
		 ((float) data[m].measured_x) + 0.5,
		 ((float) data[m].measured_y) + 0.5, 0);
    G_add_marker(PATH_DISPLAY[2], 
		 ((float) data[m].best_x) + 0.5,
		 ((float) data[m].best_y) + 0.5, 0);
  }
  for (i = 0; i < 3; i++)
    G_display_markers(PATH_DISPLAY[i]);





}



/************************************************************************
 *
 *   NAME:         main()
 *                 
 *   FUNCTION:     main program - initialization and devMainLoop
 *                 
 *   PARAMETERS:   command lines only
 *                 
 *                 
 ************************************************************************/


int
main(int argc, char *argv[])
{
  int i, x, y, dx, dy;

  signal(SIGINT,   (void *) exit_proc);
  signal(SIGTERM,  (void *) exit_proc);

  init_graphics();


  set_initial_position(10, 10); 
  add_data_item( 0,   20,   0,  0); 
  saw_landmark( -2,   0);
  add_data_item( 0,   10,   0,  0); 
  saw_landmark( -2,   0);
  add_data_item(30,   0,   0,  -3); 
  saw_landmark( 2,   2);
  add_data_item(-30,   0,  0,  -2); 
  saw_landmark( -2,   0);


#ifndef iueahfuehf
  /* --- */

  add_data_item( 0,  -10,  0,  0); 
  saw_landmark( -5,   0);

  /* --- */

  set_initial_position(40, 40); 
  saw_landmark( 2,   2);
  add_data_item(-30,   0,  0,  0); 
  saw_landmark( -2,   0);
  add_data_item( 0,  -10,  0,  0); 
  saw_landmark( -2,   0);



#endif

  G_display_all();


  for (;;){
    update_density();
    mouse_test_loop();
  }  
}





