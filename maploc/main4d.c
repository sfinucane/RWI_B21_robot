
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Source file:     $Source: /usr/local/cvs/bee/src/maploc/main4d.c,v $
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
 * $Log: main4d.c,v $
 * Revision 1.1  2002/09/14 16:16:24  rstone
 * *** empty log message ***
 *
 * Revision 1.5  1999/04/18 19:00:12  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.4  1998/08/14 22:09:36  thrun
 * .
 *
 * Revision 1.3  1998/08/14 21:40:35  thrun
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



#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <signal.h>


#include "EZX11.h"
#include "o-graphics.h"
#include "dat.h"
#include "main4d.h"

/************************************************************************\
 ************************************************************************
\************************************************************************/

#define truncint(x) ((int) (x+0.5))
#define RANDOM_MAX ((float)(2147483647))   /* (2**31 - 1) */
#define RAND_PLUS_MINUS_ONE() ((2.0 * ((float)(random()) / ((float) RANDOM_MAX))) - 1.0)
#define RAND_ZERO_TO_ONE() ((float)(random()) / ((float) RANDOM_MAX))
#define SQUARE(x) ((x)*(x))


/************************************************************************\
 ************************************************************************
\************************************************************************/

/*
 * Statistics only
 */

int iteration = 0;
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
  samples_type samples[NUM_WINDOWS_PER_DATA_ITEM];
  /* display information */
  int   button_window_id;
  int   path_window_id[NUM_WINDOWS_PER_DATA_ITEM][NUM_PATHS2];
  float window_probs[NUM_WINDOWS_PER_DATA_ITEM][DISPLAY_SIZE][DISPLAY_SIZE];
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





samples_type map;


float obs_probs[DISPLAY_SIZE][DISPLAY_SIZE];
float all_probs[DISPLAY_SIZE][DISPLAY_SIZE];
float demo_probs[DISPLAY_SIZE][DISPLAY_SIZE];


/************************************************************************\
 ************************************************************************
\************************************************************************/

/*
 * The following code is only useful for debugging.
 */



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

  for (i = 0; i < DISPLAY_SIZE; i++)
    for (j = 0; j < DISPLAY_SIZE; j++){
      obs_probs[i][j] = 0.0;
      all_probs[i][j] = 0.0;
      demo_probs[i][j] = 0.0;
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
			     NULL, DISPLAY_SIZE, DISPLAY_SIZE,
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
			     NULL, DISPLAY_SIZE, DISPLAY_SIZE,
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
			     NULL, DISPLAY_SIZE, DISPLAY_SIZE,
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
     "guessed (off)", "guessed (on)"};
    static int switch_fonts[]           = {1,1,1,1};
    static int switch_background_color[]= {C_GREY90, C_YELLOW};
    static int switch_frame_color[]     = {C_GREY30, C_GREY70};
    static int switch_text_color[]      = {C_BLACK, C_BLACK};
    int i;


    if (NUM_PATHS != 4){
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
    {C_BLUE, C_LAWNGREEN, C_RED, C_CADETBLUE};
    static int mar_text_color[]            = {NO_COLOR};
    static int mar_fonts[]                 = {2};
   
    if (NUM_PATHS != 4){
      fprintf(stderr, "Something is wrong with NUM_PATHS\n");
      exit(-1);
    }

    for (i = 0, j = 0; i < NUM_PATHS; i++){
      
      PATH_DISPLAY[0][j] =
	G_create_markers_object(mar_pos0, 1, 0.0,
				0.0, ((float) WORLD_SIZE),
				0.0, ((float) WORLD_SIZE),
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, &(mar_foreground_color[i]), 
				  mar_text_color, mar_fonts);
      G_deactivate(PATH_DISPLAY[0][j]);

      PATH_DISPLAY[1][j] =
	G_create_markers_object(mar_pos1, 1, 0.0,
				0.0, ((float) WORLD_SIZE),
				0.0, ((float) WORLD_SIZE),
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, &(mar_foreground_color[i]), 
				  mar_text_color, mar_fonts);
      G_deactivate(PATH_DISPLAY[1][j]);

      PATH_DISPLAY[2][j] =
	G_create_markers_object(mar_pos2, 1, 0.0,
				0.0, ((float) WORLD_SIZE),
				0.0, ((float) WORLD_SIZE),
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, &(mar_foreground_color[i]), 
				  mar_text_color, mar_fonts);
      G_deactivate(PATH_DISPLAY[2][j++]);

      PATH_DISPLAY[0][j] =
	G_create_markers_object(mar_pos0, 0, 
				0.022222222222 * ((float) (WORLD_SIZE)),
				0.0, ((float) WORLD_SIZE),
				0.0, ((float) WORLD_SIZE),
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, &(mar_foreground_color[i]), 
				mar_text_color, mar_fonts);
      G_deactivate(PATH_DISPLAY[0][j]);

      PATH_DISPLAY[1][j] =
	G_create_markers_object(mar_pos1, 0, 
				0.022222222222 * ((float) (WORLD_SIZE)),
				0.0, ((float) WORLD_SIZE),
				0.0, ((float) WORLD_SIZE),
				num_mar, text_mar, mar_background_color, 
				mar_frame_color, &(mar_foreground_color[i]), 
				mar_text_color, mar_fonts);
      G_deactivate(PATH_DISPLAY[1][j]);

      PATH_DISPLAY[2][j] =
	G_create_markers_object(mar_pos2, 0, 
				0.022222222222 * ((float) (WORLD_SIZE)),
				0.0, ((float) WORLD_SIZE),
				0.0, ((float) WORLD_SIZE),
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
			[DISPLAY_SIZE][DISPLAY_SIZE],
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
			     NULL, DISPLAY_SIZE, DISPLAY_SIZE,
			     0.0, 1.0, matrix_colors, matrix_font);


    G_matrix_set_display_range(probs_window_id[i], 0.0, 1.0);
    
    
    for (k = 0, j = 0; k < NUM_PATHS; k++){
      path_window_id[i][j] =
	G_create_markers_object(matrix_pos, 1, 0.0,
				0.0, ((float) WORLD_SIZE),
				0.0, ((float) WORLD_SIZE),
				1, text_mar2, mar2_background_color, 
				mar2_frame_color, 
				&(mar2_foreground_color[k]), 
				mar2_text_color, mar2_fonts);
      G_deactivate(path_window_id[i][j++]);

      path_window_id[i][j] =
	G_create_markers_object(matrix_pos, 0,
				0.022222222222 * ((float) (WORLD_SIZE)),
				0.0, ((float) WORLD_SIZE),
				0.0, ((float) WORLD_SIZE),
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
	    "ERROR: Memory overflow. Change MAX_NUM_STEPS in main4d.h\n");
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
  data_type *data;
  int i, found;

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

  for (i = 0; i < NUM_WINDOWS_PER_DATA_ITEM; i++){
    data->samples[i].num_samples = 0;
    data->samples[i].tree = NULL;
    data->samples[i].method = 1;
    data->samples[i].use_orientation = 1;
  }

  

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
    fprintf(stderr, "ERROR: Internal error: %d", current_dataset_no);
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
      for (i = 0; i < DISPLAY_SIZE; i++)
	for (j = 0; j < DISPLAY_SIZE; j++){
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
 *   NAME:         select_random_sample()
 *                 
 *   FUNCTION:     Selects a random sample from a distribution
 *                 
 *                 
 ************************************************************************/


int 
select_random_sample(samples_type *samples)
{
  int index, increment, found;
  float ran_val;

  
  ran_val = RAND_ZERO_TO_ONE();
  increment = samples->num_samples / 2;
  index     = samples->num_samples / 2;
  found     = 0;

  if (samples->num_samples <= 0){
    fprintf(stderr, "ERROR: cannot select sample from empty set.\n");
    return 0;
  }
  

  do{
    if (samples->data[index].cumul_likelihood >= ran_val){
      if (index == 0 || samples->data[index-1].cumul_likelihood < ran_val)
	found = 1;
      else{
	if (increment > 1)
	  increment = increment / 2;
	index -= increment;
      }
    }
    else{
      if (increment > 1)
	increment = increment / 2;
      index += increment;
    }
  } while (!found && index < samples->num_samples - 1);


  if (index >= samples->num_samples)
    index = samples->num_samples - 1;

  if (index < 0 || index >= samples->num_samples){
    fprintf(stderr, " ??? %d ", index);
    exit(-1);
  }
  return index;
}
		    


/************************************************************************
 *
 *   NAME:         normalize_and_display_density()
 *                 
 *   FUNCTION:     Displays a density, and normalizes it
 *                 
 *                 
 ************************************************************************/



void
normalize_and_display_density(int n, int density_number, int normalize)
{
  int i, j, k, m;
  float cumul_prob;
  float max_prob;
  samples_type *samples;
  data_type *actual_data;



  samples = &(item[n].data->samples[density_number]);
  actual_data = item[n].data;


  /*
   * generate tree
   */

  generate_tree(samples);

  /*
   * initialize
   */

  for (i = 0; i < DISPLAY_SIZE; i++)
    for (j = 0; j < DISPLAY_SIZE; j++)
      actual_data->window_probs[density_number][i][j] = 0.0;
  
  /*
   * copy samples into the display array
   */

  cumul_prob = 0.0;
  for (k = 0; k < samples->num_samples; k++){
    i = (int) (samples->data[k].x / WORLD_SIZE * DISPLAY_SIZE);
    j = (int) (samples->data[k].y / WORLD_SIZE * DISPLAY_SIZE);
    if (i < 0 || i >= DISPLAY_SIZE || j < 0 || j >= DISPLAY_SIZE)
      fprintf(stderr, "\n\tSTRANGE: %g %g\n", 
	      samples->data[k].x, samples->data[k].y);
    actual_data->window_probs[density_number][i][j] += 
      samples->data[k].likelihood;
    cumul_prob += samples->data[k].likelihood;
  }


  /*
   * 
   */
  if (0){ 
    int x, y, o;
    float value;
    
    cumul_prob = 0.0;
    for (x = 0; x < DISPLAY_SIZE; x++)
      for (y = 0; y < DISPLAY_SIZE; y++){
	actual_data->window_probs[density_number][x][y] = 0.0;	
	for (o = 0; o < 360; o++){
	  value = compute_value(samples, 
				((float) x) / ((float) DISPLAY_SIZE)
				* WORLD_SIZE,
				((float) y) / ((float) DISPLAY_SIZE)
				* WORLD_SIZE,
				(float) o);
	  if (value > actual_data->window_probs[density_number][x][y])
	    actual_data->window_probs[density_number][x][y] = value;
	}
	cumul_prob += actual_data->window_probs[density_number][x][y];
      }
  }
  


  /*
   * normalize the display array
   */

  max_prob   = 0.0;
  if (cumul_prob > 0.0){
    cumul_prob = 1.0 / cumul_prob;
    for (i = 0; i < DISPLAY_SIZE; i++)
      for (j = 0; j < DISPLAY_SIZE; j++){
	actual_data->window_probs[density_number][i][j] *= cumul_prob;
	if (actual_data->window_probs[density_number][i][j] > max_prob)
	  max_prob = actual_data->window_probs[density_number][i][j];
      }

    /*
     * normalize the samples and compute cumulative density
     */

    if (normalize){
      for (k = 0; k < actual_data->samples[density_number].num_samples; k++){
	actual_data->samples[density_number].data[k].likelihood *= cumul_prob;
	if (k == 0)
	  actual_data->samples[density_number].data[k].cumul_likelihood =
	    actual_data->samples[density_number].data[k].likelihood;
	else
	  actual_data->samples[density_number].data[k].cumul_likelihood =
	    actual_data->samples[density_number].data[k-1].cumul_likelihood +
	    actual_data->samples[density_number].data[k].likelihood;
      }
    }
  }
  
  /*
   * display density
   */

  G_matrix_set_display_range(actual_data->probs_window_id[density_number], 
			     0.0, max_prob);
  G_display_matrix(actual_data->probs_window_id[density_number]);


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
  





  
  for (i = 0; i < NUM_PATHS2; i++)
    G_display_markers(item[n].data->path_window_id[density_number][i]);


  /*
   * display observationb matrix
   */

  display_map();

}


/************************************************************************
 *
 *   NAME:         generate_tree()
 *                 
 *   FUNCTION:     Converts all data into a tree
 *                 
 *                 
 ************************************************************************/


void
generate_node(samples_type *samples, 
	      struct node_type *node,
	      int *list, int list_length)
{
  struct node_type *new_node[2];
  int i, k, c;
  int *new_list[2];
  int terminate[2];
  float cumul_likelihood[2];
  int cumul_count[2];
  int new_length[2];
  int number_of_splits;

  
  if (samples->use_orientation)
    number_of_splits = 3;
  else
    number_of_splits = 2;
  
  for (i = 0; i < 2; i++){
    new_node[i] = (struct node_type *) malloc(sizeof(struct node_type));
    new_list[i] = (int *) malloc(sizeof(int) * list_length);
    if (!new_node[i] || !new_list[i]){
      fprintf(stderr, "Error: Out of memory in generate_node()\n");
      exit(-1);
    }
    new_node[i]->min_x = node->min_x;
    new_node[i]->max_x = node->max_x;
    new_node[i]->min_y = node->min_y;
    new_node[i]->max_y = node->max_y;
    new_node[i]->min_o = node->min_o;
    new_node[i]->max_o = node->max_o;
    new_node[i]->splitting_variable =
      (node->splitting_variable + 1) % number_of_splits;
    new_node[i]->depth = node->depth + 1;
    new_node[i]->child[0]       = NULL;
    new_node[i]->child[1]       = NULL;
    new_node[i]->leaf           = 0;
    new_node[i]->likelihood     = 0.0;
    node->child[i]              = new_node[i];
    new_length[i]               = 0;
    terminate[i]                = 1;
    cumul_likelihood[i]         = 0.0;
    cumul_count[i]              = 0;
  }
  if (node->splitting_variable == 0){
    new_node[1]->min_x = new_node[0]->max_x = 
      0.5 * (node->min_x + node->max_x);
    for (k = 0; k < list_length; k++)
      if (samples->data[list[k]].x < new_node[0]->max_x){
	new_list[0][new_length[0]] = list[k];
	new_length[0] += 1;
	cumul_likelihood[0] += samples->data[list[k]].likelihood;
	cumul_count[0]      += 1;
	if (samples->data[list[k]].x != samples->data[new_list[0][0]].x ||
	    samples->data[list[k]].y != samples->data[new_list[0][0]].y ||
	    samples->data[list[k]].orientation !=
	    samples->data[new_list[0][0]].orientation)
	  terminate[0] = 0;
      }
      else{
	new_list[1][new_length[1]] = list[k];
	new_length[1] += 1;
	cumul_likelihood[1] += samples->data[list[k]].likelihood;
	cumul_count[1]      += 1;
	if (samples->data[list[k]].x != samples->data[new_list[1][0]].x ||
	    samples->data[list[k]].y != samples->data[new_list[1][0]].y ||
	    samples->data[list[k]].orientation !=
	    samples->data[new_list[1][0]].orientation)
	  terminate[1] = 0;
      }
  }
  else if (node->splitting_variable == 1){
    new_node[1]->min_y = new_node[0]->max_y = 
      0.5 * (node->min_y + node->max_y);
    for (k = 0; k < list_length; k++)
      if (samples->data[list[k]].y < new_node[0]->max_y){
	new_list[0][new_length[0]] = list[k];
	new_length[0] += 1;
	cumul_likelihood[0] += samples->data[list[k]].likelihood;
	cumul_count[0]      += 1;
	if (samples->data[list[k]].x != samples->data[new_list[0][0]].x ||
	    samples->data[list[k]].y != samples->data[new_list[0][0]].y ||
	    samples->data[list[k]].orientation !=
	    samples->data[new_list[0][0]].orientation)
	  terminate[0] = 0;
      }
      else{
	new_list[1][new_length[1]] = list[k];
	new_length[1] += 1;
	cumul_likelihood[1] += samples->data[list[k]].likelihood;
	cumul_count[1]      += 1;
	if (samples->data[list[k]].x != samples->data[new_list[1][0]].x ||
	    samples->data[list[k]].y != samples->data[new_list[1][0]].y ||
	    samples->data[list[k]].orientation != 
	    samples->data[new_list[1][0]].orientation)
	  terminate[1] = 0;
      }
  }
  else if (node->splitting_variable == 2){
    if (!samples->use_orientation){
      fprintf(stderr, "Sebastian, plug in your brain!!\n");
      exit(-1);
    }
    new_node[1]->min_o = new_node[0]->max_o = 
      0.5 * (node->min_o + node->max_o);
    for (k = 0; k < list_length; k++)
      if (samples->data[list[k]].orientation < new_node[0]->max_o){
	new_list[0][new_length[0]] = list[k];
	new_length[0] += 1;
	cumul_likelihood[0] += samples->data[list[k]].likelihood;
	cumul_count[0]      += 1;
	if (samples->data[list[k]].x != samples->data[new_list[0][0]].x ||
	    samples->data[list[k]].y != samples->data[new_list[0][0]].y ||
	    samples->data[list[k]].orientation !=
	    samples->data[new_list[0][0]].orientation)
	  terminate[0] = 0;
      }
      else{
	new_list[1][new_length[1]] = list[k];
	new_length[1] += 1;
	cumul_likelihood[1] += samples->data[list[k]].likelihood;
	cumul_count[1]      += 1;
	if (samples->data[list[k]].x != samples->data[new_list[1][0]].x ||
	    samples->data[list[k]].y != samples->data[new_list[1][0]].y ||
	    samples->data[list[k]].orientation != 
	    samples->data[new_list[1][0]].orientation)
	  terminate[1] = 0;
      }
  }
  else
    fprintf(stderr, " ????? %d ??? ", node->splitting_variable);


  for (i = 0; i < 2; i++){
    new_node[i]->volume =     
      (new_node[i]->max_x - new_node[i]->min_x) *
      (new_node[i]->max_y - new_node[i]->min_y) *
      (new_node[i]->max_o - new_node[i]->min_o);
    if (samples->method == 0)     
      new_node[i]->likelihood = cumul_likelihood[i] / ((float) cumul_count[i]);
    else
      new_node[i]->likelihood = cumul_likelihood[i];

    if (!terminate[i] && new_node[i]->depth < MAX_TREE_DEPTH &&
	new_length[i] > MIN_ITEMS_PER_NODE){
      if (0) fprintf(stderr,
		     "New child(%d): depth=%d %g,%g,%g ->%g,%g,%g   %g %g %g\n", 
		     i, new_node[i]->depth,
		     new_node[i]->min_x, new_node[i]->min_y,
		     new_node[i]->min_o, new_node[i]->max_x,
		     new_node[i]->max_y, new_node[i]->max_o,
		     new_node[i]->likelihood, new_node[i]->volume,
		     new_node[i]->likelihood / new_node[i]->volume);
      generate_node(samples, new_node[i], &(new_list[i][0]), new_length[i]);
    }
    else if (new_length[i] >= 1){
      if (0) fprintf(stderr,
		     "New leaf(%d): depth=%d index=%d %g,%g,%g ->%g,%g,%g   %g %g %g\n",
		     i, new_node[i]->depth, new_list[i][0],
		     new_node[i]->min_x, new_node[i]->min_y,
		     new_node[i]->min_o, new_node[i]->max_x,
		     new_node[i]->max_y, new_node[i]->max_o,
		     new_node[i]->likelihood, new_node[i]->volume,
		     new_node[i]->likelihood / new_node[i]->volume);

      new_node[i]->leaf = 1;
    }
    else{
      free(new_node[i]);
      node->child[i] = NULL;
      if (0) fprintf(stderr, "nope.\n");
    }
  }

  for (i = 0; i < 2; i++)
    free(new_list[i]);

  /*fprintf(stderr, "Node %p: %p %p %d   \t%d\n", node, node->child[0],
    node->child[1], node->depth, node->leaf);*/
}



void
generate_tree(samples_type *samples)
{
  struct node_type *new_node;
  int i, k, c, index, N;
  int list[2 * (NUM_SAMPLE_POINTS)];


  if (samples->tree)
    destroy_node(samples->tree);


  new_node = (struct node_type *) malloc(sizeof(struct node_type));
  new_node->min_x = 0.0;
  new_node->max_x = WORLD_SIZE;
  new_node->min_y = 0.0;
  new_node->max_y = WORLD_SIZE;
  new_node->min_o = 0.0;
  new_node->max_o = 360.0;
  new_node->volume = (WORLD_SIZE * WORLD_SIZE * 360.0);
  new_node->splitting_variable = 0;
  new_node->depth      = 0;
  new_node->child[0]   = NULL;
  new_node->child[1]   = NULL;
  new_node->leaf       = 0;
  new_node->likelihood = 0.0;
  for (k = 0; k < samples->num_samples; k++)
    new_node->likelihood += samples->data[k].likelihood;

  if (0) fprintf(stderr, "\n\nNew root: depth=%d, tree=%p\n", 
		 new_node->depth, samples);

  if (samples->num_samples == 0){
    new_node->leaf           = 1;
    new_node->volume         = WORLD_SIZE * WORLD_SIZE * 360.0;
  }
  if (samples->num_samples == 1){
    new_node->leaf           = 1;
    new_node->volume         = WORLD_SIZE * WORLD_SIZE * 360.0;
   }
  else{
    for (i = 0; i < samples->num_samples; i++)
      list[i] = i;
    generate_node(samples, new_node, list, samples->num_samples);
  }
  samples->tree = new_node; 
}



void
destroy_node(struct node_type *node)
{
  if (node->child[0])
    destroy_node(node->child[0]);
  if (node->child[1])
    destroy_node(node->child[1]);
  free(node);
}

void
traverse_node(samples_type *samples, 
	      struct node_type *node,
	      float x, float y, float o, 
	      float *likelihood,
	      int *done)
{
  float dist = 0.0;
  int i;
  int flag = 0;
  float dist_x, dist_y, dist_o, help;

  if (!node || *done) return;

  if (x < node->min_x || x > node->max_x ||
      y < node->min_y || y > node->max_y ||
      (samples->use_orientation && (o < node->min_o ||
				    o > node->max_o)))
    return;

  if (samples->method == 0)
    *likelihood = 0.5 * node->likelihood + 0.5;
  else
    *likelihood = node->likelihood / node->volume;
  /*fprintf(stderr, " [%g %g %g]", 
    node->likelihood, node->volume, *likelihood);*/

  if (node->leaf) /* found the right leaf node */
    *done = 1; 
  else{
    for (i = 0; i < 2; i++)
      if (node->child[i] && !(*done))
	traverse_node(samples, node->child[i], x, y, o, likelihood, done);
  }
}


	

/************************************************************************
 *
 *   NAME:         compute_value()
 *                 
 *   FUNCTION:     Estimates the likelihood of a robot pose
 *                 
 *                 
 ************************************************************************/




      


float
compute_value(samples_type *samples, 
	      float query_x, 
	      float query_y, 
	      float query_o)
{
  int i;
  data_type *actual_data;
  float likelihood, dist, dist_x, dist_y, dist_o, help;
  int done;

  if (samples->method != 0 && samples->method != 1){
    fprintf(stderr, "Unknown method: %d. Must exit.\n", samples->method);
    exit(-1);
  }

  if (!samples->tree){
    fprintf(stderr, "God in Heaven! Where's that tree?\n");
    exit(-1);
  }

  if (samples->num_samples == 0){
    if (samples->method == 0)
      likelihood = 1.0;
    else
      likelihood = 0.5;
  }
  else{
    likelihood  = 0.0;
    done        = 0;
    traverse_node(samples, samples->tree, query_x, query_y, query_o, 
		  &likelihood, &done);
  }


  if (likelihood < MIN_LIKELIHOOD)
    likelihood = MIN_LIKELIHOOD;

  return likelihood;
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
  int i, j, k, l, m, d;
  data_type *actual_data;
  data_type *prev_data;
  data_type *next_data;
  int   normalize_flag = 0;
  int   observation_flag = 0;
  int   found;
  float help;
  float ran_val, max;
  float likelihood;
  float tmp_x, tmp_y, tmp_o, tmp_o_rad;
  int   prev_n, next_n;
  float turn, trans, range;
  samples_type *actual_samples;
  samples_type *prev_samples;
  samples_type *next_samples;


  /* @@@ */

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
  actual_samples = &(actual_data->samples[density_number]);

  prev_n = item[n].prev_data_item;
  if (item[n].prev_data_item >= 0){
    prev_data    = item[prev_n].data;
    prev_samples = &(prev_data->samples[density_number]);
  }
  else{
    prev_data = NULL;
    prev_samples = NULL;
  }

  next_n = item[n].next_data_item;
  if (item[n].next_data_item >= 0){
    next_data = item[next_n].data;
    next_samples = &(next_data->samples[density_number]);
  }
  else{
    next_data = NULL;
    next_samples = NULL;
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
      
      actual_samples->data[0].x = item[n].correct_pos_x;
      actual_samples->data[0].y = item[n].correct_pos_y;
      actual_samples->data[0].orientation = 
	item[n].correct_pos_orientation;
      actual_samples->data[0].likelihood = 1.0;
      actual_samples->data[0].cumul_likelihood = 1.0;

      actual_samples->num_samples = 1;

    }


    /* ==================================================
     *           Special case: unknown (new) position
     * ================================================== */

    
    else if (item[n].new_episode){
      help = 1.0 / ((float) NUM_SAMPLE_POINTS);

      for (k = 0; k < NUM_SAMPLE_POINTS; k++){
	actual_samples->data[k].x = 
	  RAND_ZERO_TO_ONE() * WORLD_SIZE;
	actual_samples->data[k].y = 
	  RAND_ZERO_TO_ONE() * WORLD_SIZE;
	actual_samples->data[k].orientation = 
	  RAND_ZERO_TO_ONE() * 360.0;
	actual_samples->data[k].likelihood = help;
	actual_samples->data[k].cumul_likelihood = 
	  ((float) (k+1)) * help;
      }
      actual_samples->num_samples = NUM_SAMPLE_POINTS;

      observation_flag = 1;
    }


    /* ==================================================
     *           Regular case: constrained position
     * ================================================== */
    
    
    else{				/* all other data items */
      if (!prev_data){
	fprintf(stderr, "SEVERE DATA ERROR 1.\n");
	exit(-1);
      }

      for (k = 0; k < NUM_SAMPLE_POINTS; k++){
	do{
	  l = select_random_sample(&(prev_data->samples[density_number]));
		  
	  turn  = item[n].measured_turn_since_last_estimate;
	  trans = item[n].measured_translation_since_last_estimate;
	  range = RAND_ZERO_TO_ONE();
	  turn += 0.5 * RAND_PLUS_MINUS_ONE() * range * fabs(trans) *
	    TURN_CONTROL_NOISE_FACTOR;
	  range = RAND_ZERO_TO_ONE();
	  trans += RAND_PLUS_MINUS_ONE() * range * fabs(trans) *
	    TRANSLATION_CONTROL_NOISE_FACTOR;

	  tmp_o = prev_samples->data[l].orientation
	    + turn;
	  tmp_o_rad = tmp_o / 180.0 * M_PI;
	  tmp_x = prev_samples->data[l].x +
	    (trans * cos(tmp_o_rad));
	  tmp_y = prev_samples->data[l].y +
	    (trans * sin(tmp_o_rad));
	  range = RAND_ZERO_TO_ONE();
	  tmp_o += 0.5 * RAND_PLUS_MINUS_ONE() * range * fabs(trans) *
	    TURN_CONTROL_NOISE_FACTOR;
	  for (; tmp_o <    0.0; ) tmp_o += 360.0;
	  for (; tmp_o >= 360.0; ) tmp_o -= 360.0;

	} while (tmp_x < 0.0 || tmp_x >= WORLD_SIZE ||
		 tmp_y < 0.0 || tmp_y >= WORLD_SIZE);

	/*fprintf(stderr, " [%g %g %g] ", tmp_x, tmp_y, tmp_o); */
	actual_samples->data[k].x = tmp_x;
	actual_samples->data[k].y = tmp_y;
	actual_samples->data[k].orientation = tmp_o;
	actual_samples->data[k].likelihood = 1.0;
      }	

      actual_samples->num_samples = NUM_SAMPLE_POINTS;

      normalize_flag   = 1;
      observation_flag = 1;
    }


    /* ==================================================
     *           Work in the observation
     * ================================================== */

    if (observation_flag && item[n].landmark_found){

      for (k = 0; k < actual_samples->num_samples; k++){
	tmp_o = actual_samples->data[k].orientation +
	  item[n].landmark_rel_angle;
	tmp_o_rad = tmp_o * M_PI / 180.0;
	tmp_x = actual_samples->data[k].x 
	  + (item[n].landmark_rel_distance * cos(tmp_o_rad));
	tmp_y = actual_samples->data[k].y 
	  + (item[n].landmark_rel_distance * sin(tmp_o_rad));
	/*
	if (tmp_x >= 0.0 && tmp_x < WORLD_SIZE &&
	    tmp_y >= 0.0 && tmp_y < WORLD_SIZE){
	  i = (int) (tmp_x / ((float) WORLD_SIZE) * ((float) DISPLAY_SIZE));
	  j = (int) (tmp_y / ((float) WORLD_SIZE) * ((float) DISPLAY_SIZE));
	  
	  actual_samples->data[k].likelihood *= obs_probs[i][j];
	}
	*/
	actual_samples->data[k].likelihood *= 
	  compute_value(&(map), tmp_x, tmp_y, tmp_o);

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

      actual_samples->data[0].x = item[n].correct_pos_x;
      actual_samples->data[0].y = item[n].correct_pos_y;
      actual_samples->data[0].orientation = 
	item[n].correct_pos_orientation;
      actual_samples->data[0].likelihood = 1.0;
      actual_samples->data[0].cumul_likelihood = 1.0;


      actual_samples->num_samples = 1;
 
    }


    /* ==================================================
     *           Special case: unknown (new) position
     * ================================================== */


    
    else if (item[n].next_data_item < 0 ||
	     item[item[n].next_data_item].new_episode){
      

      help = 1.0 / ((float) NUM_SAMPLE_POINTS);

      for (k = 0; k < NUM_SAMPLE_POINTS; k++){
	actual_samples->data[k].x = 
	  RAND_ZERO_TO_ONE() * WORLD_SIZE;
	actual_samples->data[k].y = 
	  RAND_ZERO_TO_ONE() * WORLD_SIZE;
	actual_samples->data[k].orientation = 
	  RAND_ZERO_TO_ONE() * 360.0;
	  
	actual_samples->data[k].likelihood = help;
	actual_samples->data[k].cumul_likelihood = 
	  ((float) (k+1)) * help;
      }
      actual_samples->num_samples = NUM_SAMPLE_POINTS;
      
    }


    /* ==================================================
     *           Regular case: constrained position
     * ================================================== */
    
    

    else{

      if (!next_data){
	fprintf(stderr, "SEVERE DATA ERROR 2.\n");
	exit(-1);
      }

      for (k = 0; k < NUM_SAMPLE_POINTS; k++){
	do{
	  if (item[next_n].landmark_found){
	    do{
	      l = select_random_sample(&(next_data->samples[density_number]));
	      tmp_o = next_samples->data[l].orientation +
		item[next_n].landmark_rel_angle;
	      tmp_o_rad = tmp_o * M_PI / 180.0;
	      tmp_x = next_samples->data[l].x 
		+ (item[next_n].landmark_rel_distance * cos(tmp_o_rad));
	      tmp_y = next_samples->data[l].y 
		+ (item[next_n].landmark_rel_distance * sin(tmp_o_rad));
	      help = compute_value(&(map), tmp_x, tmp_y, tmp_o);
	    } while (help <= 0.0 || 
		     tmp_x < 0.0 || tmp_x >= WORLD_SIZE ||
		     tmp_y < 0.0 || tmp_y >= WORLD_SIZE);
	    
	  }
	  else{
	    l = select_random_sample(next_samples);
	    help = 1.0;
	  }
	  
	  turn  = item[n].measured_turn_since_last_estimate;
	  trans = item[n].measured_translation_since_last_estimate;
	  range = RAND_ZERO_TO_ONE();
	  turn += 0.5 * RAND_PLUS_MINUS_ONE() * range * fabs(trans) *
	    TURN_CONTROL_NOISE_FACTOR;
	  range = RAND_ZERO_TO_ONE();
	  trans += RAND_PLUS_MINUS_ONE() * range * fabs(trans) *
	    TRANSLATION_CONTROL_NOISE_FACTOR;

	  range = RAND_ZERO_TO_ONE();
	  tmp_o = next_samples->data[l].orientation 
	    - (0.5 * RAND_PLUS_MINUS_ONE() * range * fabs(trans) *
	       TURN_CONTROL_NOISE_FACTOR);
	  tmp_o_rad = tmp_o / 180.0 * M_PI;
	  tmp_x = next_samples->data[l].x -
	    (trans * cos(tmp_o_rad));
	  tmp_y = next_samples->data[l].y -
	    (trans * sin(tmp_o_rad));
	  tmp_o -= turn;

	  for (; tmp_o <    0.0; ) tmp_o += 360.0;
	  for (; tmp_o >= 360.0; ) tmp_o -= 360.0;

	} while (tmp_x < 0.0 || tmp_x >= WORLD_SIZE ||
		 tmp_y < 0.0 || tmp_y >= WORLD_SIZE);

	/*fprintf(stderr, " [%g %g %g] ", tmp_x, tmp_y, tmp_o); */
	actual_samples->data[k].x = tmp_x;
	actual_samples->data[k].y = tmp_y;
	actual_samples->data[k].orientation = tmp_o;
	actual_samples->data[k].likelihood = help;
      }	

      actual_samples->num_samples = NUM_SAMPLE_POINTS;

      /*
      for (k = 0; k < next_data->num_samples; k++){
	actual_samples->data[k].x = 
	  next_samples->data[k].x;
	actual_samples->data[k].y = 
	  next_samples->data[k].y;
	actual_samples->data[k].orientation = 
	  next_samples->data[k].orientation;
	actual_samples->data[k].likelihood = 1.0;
      }
      actual_data->num_samples = next_data->num_samples;
      */

      normalize_flag = 1;
    }

  }

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


    /* ==================================================
     *           Special case: known position
     * ================================================== */
    
    
    if (item[n].position_known){	/* includes the first data item */
      
      actual_samples->data[0].x = item[n].correct_pos_x;
      actual_samples->data[0].y = item[n].correct_pos_y;
      actual_samples->data[0].orientation = 
	item[n].correct_pos_orientation;
      actual_samples->data[0].likelihood = 1.0;
      actual_samples->data[0].cumul_likelihood = 1.0;

      actual_samples->num_samples = 1;

    }


    /* ==================================================
     *           Regular case: multiply densities
     * ================================================== */
    
    else{
      k = 0;
      for (l = 0; l < actual_data->samples[1].num_samples; l++){
	help = compute_value(&(actual_data->samples[2]), 
			     actual_data->samples[1].data[l].x, 
			     actual_data->samples[1].data[l].y, 
			     actual_data->samples[1].data[l].orientation);
	if (help > 0.0){
	  actual_samples->data[k].x = actual_data->samples[1].data[l].x;
	  actual_samples->data[k].y = actual_data->samples[1].data[l].y;
	  actual_samples->data[k].orientation = 
	    actual_data->samples[1].data[l].orientation;
	  actual_samples->data[k].likelihood = 
	    actual_data->samples[1].data[l].likelihood * help;
	  k++;
	}
      }
      for (l = 0; l < actual_data->samples[2].num_samples; l++){
	help = compute_value(&(actual_data->samples[1]), 
			     actual_data->samples[2].data[l].x, 
			     actual_data->samples[2].data[l].y, 
			     actual_data->samples[2].data[l].orientation);
	if (help > 0.0){
	  actual_samples->data[k].x = actual_data->samples[2].data[l].x;
	  actual_samples->data[k].y = actual_data->samples[2].data[l].y;
	  actual_samples->data[k].orientation = 
	    actual_data->samples[2].data[l].orientation;
	  actual_samples->data[k].likelihood = 
	    actual_data->samples[2].data[l].likelihood * help;
	  k++;
	}
      }
      actual_samples->num_samples = k;

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


  normalize_and_display_density(n, density_number, normalize_flag);


  /* ==================================================
   *           Display
   * ================================================== */


  
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
  


  
  for (i = 0; i < NUM_PATHS2; i++)
    G_display_markers(item[n].data->path_window_id[density_number][i]);


  /*
   * statistics
   */

  if (iteration < MAX_NUM_ITERATIONS){
    gettimeofday(&(computation_time[iteration]), NULL);
  }
  else 
    fprintf(stderr, "WARNING: exceeded legal number of iterations: %d\n",
	    iteration);

  /*
   * done
   */




  /* ==================================================
   *           Done!
   * ================================================== */

  G_display_switch(item[n].data->button_window_id, 0);
}

		    





/************************************************************************
 *
 *   NAME:         compute_map
 *                 
 *   FUNCTION:     This routine computes the observation
 *                 matrix (M step in EM)
 *                 
 *                 
 ************************************************************************/

void
compute_map()
{
  int i, j, k, l, n;
  float tmp_x, tmp_y, tmp_o, tmp_o_rad, dx, dy, da, dl;
  data_type *actual_data;
  samples_type *actual_samples;
  float help, weight, max;
  float obs_count[DISPLAY_SIZE][DISPLAY_SIZE];


  fprintf(stderr, "Computing map...");

  if (0){
    static int i = 0;
    if (!i){
      fprintf(stderr, "\n\n\t### WARNING: Static Map! ###\n\n");
      i = 1;
    }
    
    k = 0;
    map.data[k].x = 15.0;
    map.data[k].y = 15.0;
    map.data[k++].likelihood = 0.125;

    map.data[k].x = 35.0;
    map.data[k].y = 15.0;
    map.data[k++].likelihood = 0.125;

    map.data[k].x = 35.0;
    map.data[k].y = 35.0;
    map.data[k++].likelihood = 0.125;

    map.data[k].x = 15.0;
    map.data[k].y = 35.0;
    map.data[k++].likelihood = 0.125;

    for (; k < NUM_SAMPLE_POINTS; k++){
      map.data[k].x = WORLD_SIZE * RAND_ZERO_TO_ONE();
      map.data[k].y = WORLD_SIZE * RAND_ZERO_TO_ONE();
      map.data[k++].likelihood = - 0.5 / ((float) NUM_SAMPLE_POINTS);
    }
    map.num_samples = NUM_SAMPLE_POINTS;
  }

  else{

    /*
     * Calculate the new map
     */
    
    n = -1;
    help = 1.0 / ((float) NUM_SAMPLE_POINTS);
    for (k = 0; k < NUM_SAMPLE_POINTS;){
      if (n < 0)
	n = first_data_item;      

      if (item[n].landmark_found){
	actual_data    = item[n].data;
	actual_samples = &(actual_data->samples[0]);

	l = select_random_sample(actual_samples);
	

      if (RAND_ZERO_TO_ONE() >= 0.5){
	  do {
	    dx = RAND_PLUS_MINUS_ONE() * RAND_ZERO_TO_ONE() * 3.0;/*!*/
	    dy = RAND_PLUS_MINUS_ONE() * RAND_ZERO_TO_ONE() * 3.0;
	  } while ((dx * dx) + (dy * dy) > SQUARE(3.0));
	  tmp_o = actual_samples->data[l].orientation + 
	    item[n].landmark_rel_angle;
	  tmp_o_rad = tmp_o * M_PI / 180.0;
	  for (; tmp_o >= 360.0; ) tmp_o -= 360.0;
	  for (; tmp_o <    0.0; ) tmp_o += 360.0;
	  tmp_x = actual_samples->data[l].x + dx +
	    (item[n].landmark_rel_distance * cos(tmp_o_rad));
	  tmp_y = actual_samples->data[l].y + dy +
	    (item[n].landmark_rel_distance * sin(tmp_o_rad));
	  weight = 1.0;
	}
	else{
	  do {
	    dx = RAND_PLUS_MINUS_ONE();
	    dy = RAND_PLUS_MINUS_ONE();
	  } while ((dx * dx) + (dy * dy) > 1.0);
	  tmp_x = actual_samples->data[l].x + (dx * VISUAL_RANGE);
	  tmp_y = actual_samples->data[l].y + (dy * VISUAL_RANGE);
	  weight = -1.0;
	}
	
	if (tmp_x >= 0.0 && tmp_x < WORLD_SIZE &&
	    tmp_y >= 0.0 && tmp_y < WORLD_SIZE){
	  map.data[k].x = tmp_x;
	  map.data[k].y = tmp_y;
	  map.data[k].likelihood = weight;
	  k++;
	  if (k % 1000 == 0)
	    fprintf(stderr, ".");
	}
      }
      n = item[n].next_data_item;
    }
    map.num_samples = NUM_SAMPLE_POINTS;
  }


    

  /*
   * generate tree
   */

  generate_tree(&(map));
  
  /*
   * copy samples into the display array
   */


  /*
   * copy samples into the display array
   */

  if (0){
    max   = 0.0;
    for (i = 0; i < DISPLAY_SIZE; i++)
      for (j = 0; j < DISPLAY_SIZE; j++)
	obs_probs[i][j] = 0.0;
    for (k = 0; k < map.num_samples; k++){
      i = (int) (map.data[k].x / WORLD_SIZE * DISPLAY_SIZE);
      j = (int) (map.data[k].y / WORLD_SIZE * DISPLAY_SIZE);
      obs_probs[i][j] = 0.5 * map.data[k].likelihood + 0.5;
      if (obs_probs[i][j] > max)
	max = obs_probs[i][j];
    }
  }
  else{
    max = 0.0;
    for (i = 0; i < DISPLAY_SIZE; i++)
      for (j = 0; j < DISPLAY_SIZE; j++){
	tmp_x = ((float) i) * ((float) WORLD_SIZE) / ((float) DISPLAY_SIZE);
	tmp_y = ((float) j) * ((float) WORLD_SIZE) / ((float) DISPLAY_SIZE);
	tmp_o = 0.0;
	obs_probs[i][j] = compute_value(&(map), tmp_x, tmp_y, tmp_o);
	fprintf(stderr, " %g", obs_probs[i][j]);
	if (obs_probs[i][j] > max)
	  max = obs_probs[i][j];
      }
  }
  /*
   * display the map
   */

  G_matrix_set_display_range(OBS_PROBS, 0.0, max);
  update_intermediate_path_items();
  display_map();

  fprintf(stderr, "done.\n");
}




		    

		    


/************************************************************************
 *
 *   NAME:         display_map
 *                 
 *   FUNCTION:     This routine  displays the observation
 *                 matrix
 *                 
 *                 
 ************************************************************************/

void
display_map()
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
    if ((max_x - min_x) * robot_position_absolute_multiplier > WORLD_SIZE ||
	(max_y - min_y)  * robot_position_absolute_multiplier > WORLD_SIZE){
      fprintf(stderr, "World is too small, or resolution too high.\n");
      fprintf(stderr, "%g > %g or %g > %g (res=%g)\n",
	      (max_x - min_x) * robot_position_absolute_multiplier,
	      WORLD_SIZE,
	      (max_y - min_y) * robot_position_absolute_multiplier,
	      WORLD_SIZE,
	      robot_position_absolute_multiplier);
      return 0;
    }
    if (!robot_position_absolute_offset_defined[n]){
      robot_position_absolute_offset_x[n]   = 
	0.5 * (WORLD_SIZE -
	       ((max_x + min_x) * robot_position_absolute_multiplier));
      robot_position_absolute_offset_y[n]   = 
	0.5 * (WORLD_SIZE -
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

  fprintf(stderr, "save_data() not implemented.\n");
#ifdef JUNK
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
#endif
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

  fprintf(stderr, "load_data() not implemented.\n");
#ifdef JUNK

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
#endif
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
  }

  
  if (iop == NULL){
    /*
     * Statistics on the screen
     */
    
      if (iteration == 0)
	j = MAX_NUM_ITERATIONS;
      else 
	j = iteration -1;
    fprintf(stderr, "----------------------------------------------------\n");
      time_difference = 
	((float) (computation_time[iteration].tv_sec -
		  computation_time[j].tv_sec))
	+ (((float) (computation_time[iteration].tv_usec -
		     computation_time[j].tv_usec))
	   /  1000000.0);
      fprintf(stderr, "Statistics for iteration %d:\n", iteration);
      fprintf(stderr, "time: %g\n", time_difference);
      fprintf(stderr,
	      "----------------------------------------------------\n\n");
  }

  else{
    /*
     * Statistics on the screen
     */
    
      if (iteration == 0)
	j = MAX_NUM_ITERATIONS;
      else 
	j = iteration -1;
    fprintf(iop, "----------------------------------------------------\n");
      time_difference = 
	((float) (computation_time[iteration].tv_sec -
		  computation_time[j].tv_sec))
	+ (((float) (computation_time[iteration].tv_usec -
		     computation_time[j].tv_usec))
	   /  1000000.0);
      fprintf(iop, "Statistics for iteration %d:\n", iteration);
      fprintf(iop, "time: %g\n", time_difference);
      fprintf(iop,
	      "----------------------------------------------------\n\n");
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
  int i, j;

  signal(SIGINT,  (void *) exit_proc);
  signal(SIGTERM, (void *) exit_proc);
  signal(SIGALRM, (void *) alarm_handler);


  init_graphics();
  map.tree = NULL;
  map.num_samples = 0;
  map.method = 0;
  map.use_orientation = 0;
  generate_tree(&(map));

  for (i = 0; i < DISPLAY_SIZE; i++)
    for (j = 0; j < DISPLAY_SIZE; j++)
      obs_probs[i][j] = 0.5;


  if (load_data(INI_FILENAME) == -1) exit(-1); 
  
  /* wean_data_short(0);  */
  
  square_data_short();
  
  /* wean_data_long(0); */
  
  /*if (!load_raw_data("maploc3d.dat")) exit(-1); */
  


  G_display_all();
  print_statistics(NULL, 1);
  fprintf(stderr, "Successfully memorized %d items.\n", num_items);



  for (iteration = 0; iteration < MAX_NUM_ITERATIONS; iteration++){
    /*if (data) save_raw_data(save_file_name);*/
    forward_phase(); 
    backward_phase();
    integration_phase();
    compute_map();
    save_data(DUMP_FILENAME);
    print_statistics(NULL, 0);
  }  

  if (button_alarm)
    ualarm(200, 0);  


}


