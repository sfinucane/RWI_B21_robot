

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
 ***** Please read and make sure you understand the disclaimer below.
 *****
 ***** Contact thrun@cs.cmu.edu if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                                     (c) Sebastian Thrun, 1997
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/learn/xtypes.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 16:36:05 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: xtypes.h,v $
 * Revision 1.1  2002/09/14 16:36:05  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1998/07/04 15:21:39  thrun
 * variable logging.
 *
 * Revision 1.1  1998/06/20 21:05:34  thrun
 * Reorganization (created: fa.{c,h}, xtypes.h)
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/




/************************************************************************\
 ************************************************************************
\************************************************************************/


#define MAX_NUMBER_VARIABLES 100
#define MAX_NUMBER_PROCESSES 100



#define MAX_NUMBER_NETWORKS 100

#define DEFAULT_WEIGHTS_INIT_RANGE 1.0
#define DEFAULT_STEP_SIZE 0.01
#define DEFAULT_MOMENTUM  0.9
#define DEFAULT_NETWORK_LEARNING_FLAG 1	/* flag: shall we train this net? */
#define DEFAULT_VARIABLE_LEARNING_FLAG 1 /* flag: train this variable? */
#define INITIAL_MIN_GRADIENT 0.1
#define INITIAL_MAX_GRADIENT 0.1
#define DEFAULT_NUM_BASES_PER_INPUT 3 /* radial basis networks only */



/*
 * Constants for the etsting routines
 */

#define TEST_PERTURBATION 1.0
#define MIN_CHANGE 0.0000
#define NUM_TESTS  10



/*
 * Type declaration for code of the individual processing processes
 */


typedef void void_code(float time_elapsed);


/************************************************************************\
 ************************************************************************
\************************************************************************/



/*
 * internal variable organization: declaraitons and data types
 */


typedef struct {
  int   network_id;
  float *value;
} gradients_type;

/*
 * A variable type
 */

typedef struct {
  char                       *name;
  int                        type;
  int                        cyclic;
  int                        dimension;
  int                        resolution;
  int                        num_values;
  int                        save_flag;
  int                        learning_flag;
  int                        log_flag;
  float                      error[NUM_SET_TYPES];
  float                      prev_error[NUM_SET_TYPES];

  float                      *value; /* this is the value of the variable */
  int                        num_gradients;
  gradients_type             gradients[MAX_NUMBER_NETWORKS];
  int                        display;

  float                      *cumul_value; /* auxiliary, for computing avg */
  int                        num_cumul_gradients;
  gradients_type             cumul_gradients[MAX_NUMBER_NETWORKS];
  int                        cumul_divisor;

  float                      *avg_value; /* average value of variable */
  int                        num_avg_gradients;
  gradients_type             avg_gradients[MAX_NUMBER_NETWORKS];
  int                        avg_display;

  float                      *norm_value; /* normalized value */
  int                        norm_display;
} variable_type;





/*
 * internal process organization: declaraitons and data types
 */




typedef struct {
  char           *name;
  void_code      *code;
  struct timeval time_stamp;
  int            requires_time;
  int            requires_position;
  int            requires_image;
  int            requires_sonars;
  int            requires_lasers;
  int            requires_irs;
  int            requires_tactiles;
  int            requires_buttons;
  int            requires_pantilt;
  int            requires_markers;
  int            requires_control;
} process_type;



/*
 * internal process organization: declaraitons and data types
 */

#define LOGISTIC_NETWORK      0
#define RADIAL_BASIS_NETWORK  1


typedef struct {
  char           *name;
  int            type;

  int            from_variable_type;
  int            from_variable_dimension;
  int            to_variable_type;
  int            to_variable_dimension;

  int            num_inputs;
  int            num_pseudo_inputs;
  int            num_hidden;
  int            num_outputs;
  int            num_pseudo_inputs_per_input;
  int            *input_info;
  int            *input_resolution;
  int            *num_bases_per_input;
  int            learning_flag;
  float          weights_init_range;
  float          step_size;
  float          momentum;
  float          min_gradient;
  float          max_gradient;
  /* ----------------------------- */
  int            num_params;
  float          *params;
  float          *best_params;
  float          *input_to_hidden_weights;
  float          *hidden_biases;
  float          *hidden_to_output_weights;
  float          *output_biases;
  float          *input_to_output_weights;

  float          *input;
  float          *pseudo_input;
  float          *netin_hidden;
  float          *hidden;
  float          *netin_output;
  float          *output;

  float          *d_output_d_params;
  float          *d_hidden_d_input;
  float          *d_output_d_input;
  float          *d_params;
  float          *d_error_d_params;
  float          *prev_d_error_d_params;
} network_type;


/*
 * >>> The following defines are for the field "input_info"
 *     input units can be one of three types, and which one it is
 *     is important for the stochastic computation
 * >>> The number of values to iterate over during a single stochastic
 *     integration is kept in "input_resolution", which is directly copied 
 *     from the resolution of the corresponding variables
 */

#define NETWORK_UNIT_IS_DETERMINISTIC_INPUT 0
#define NETWORK_UNIT_IS_STOCHASTIC_INPUT    1
#define NETWORK_UNIT_IS_STOCHASTIC_OUTPUT   2



/************************************************************************\
 ************************************************************************
\************************************************************************/

/************************************************************************\
 ************************************************************************
\************************************************************************/
/*
 * Variable types
 */

#define DETERMINISTIC 0
#define STOCHASTIC    1

/*
 * Events for triggering a process 
 */

#define POSITION_EVENT          0
#define IMAGE_EVENT             1
#define SONARS_EVENT            2
#define LASERS_EVENT            3
#define IRS_EVENT               4
#define TACTILES_EVENT          5
#define BUTTONS_EVENT           6
#define PANTILT_EVENT           7
#define MARKERS_EVENT           8
#define TIME_EVENT              9
#define CONTROL_EVENT          10


/*
 * Parameters for the conguration of the display
 *
 * VALUE    is the value of a variable (deterministic or stochastic)
 * AVERAGE  is the average value of a variable - makes only sense for
 *          stochastic variables
 * N_VALUE  is the normalized value (value divided by average), again
 *          makes only sense for stochastic variables
 * ALL      displays all of them
 */


#define X_NO_DISPLAY                0
#define X_DISPLAY_VALUE             1
#define X_DISPLAY_AVERAGE           2
#define X_DISPLAY_N_VALUE           3
#define X_DISPLAY_ALL               4


/************************************************************************\
 ************************************************************************
\************************************************************************/

/*
 * These are the necessary definitions for subroutines, followed by an
 * example subroutine
 */

typedef void (*X_subroutine_type)(int, int, float**, float**);

#define SUBROUTINE_REPORT()  \
  { \
    int i; \
    static int msg_printed = 0; \
    if (!msg_printed){ \
      fprintf(stderr, "\tFound subroutine \"%s()\".\n", __FUNCTION__); \
      fprintf(stderr, "\t\t%d input  variable(s) with dimensions:", num_in); \
      for (i = 0; i < num_in; i++) \
	fprintf(stderr, " %d", in_dims[i]);       \
      fprintf(stderr, "\n\t\t%d output variable(s) with dimensions:", num_out); \
      for (i = 0; i < num_out; i++) \
	fprintf(stderr, " %d", out_dims[i]);       \
      fprintf(stderr, "\n"); \
      msg_printed = 1; \
    } \
  }


/*
 * EXAMPLE SUBROUTINE:
 *

             X_subroutine_type
             my_subroutine(int    num_in,
                           int    *in_dims,
                           float  **in,
                           int    num_out,
                           int    *out_dims,
                           float  **out)
             {
               SUBROUTINE_REPORT();
             
               out[0][0] = in[1][0]; 
               out[0][1] = in[1][1]; 
               out[1][0] = in[2][0];
             }
*/
