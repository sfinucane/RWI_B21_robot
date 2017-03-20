
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/learn/dat.h,v $
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
 * $Log: dat.h,v $
 * Revision 1.1  2002/09/14 16:36:05  rstone
 * *** empty log message ***
 *
 * Revision 1.4  1999/07/11 18:48:32  thrun
 * slight reorganization
 *
 * Revision 1.3  1998/08/22 03:08:36  thrun
 * .
 *
 * Revision 1.2  1997/10/23 02:28:21  thrun
 * .
 *
 * Revision 1.1  1997/10/05 18:11:17  thrun
 * new data library "libdat.a"
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/



#define MAX_STRING_LENGTH 80

#define MAX_SONAR_DISPLAY_RANGE   300.0
#define MAX_IR_DISPLAY_RANGE      100.0
#define MAX_LASER_DISPLAY_RANGE   3000.0
#define MAX_TACTILE_VALUE_RANGE     1.0
#define MAX_TACTILE_DISPLAY_RANGE  10.0

/*------------------------------------------------------------*/

#define NUM_SONAR_SENSORS 24
/*
  #define IMAGE_FREQ   30
  #define ORIG_IMAGE_FIRST_X 148
  #define ORIG_IMAGE_FIRST_Y 108
  #define ORIG_IMAGE_SIZE_X 25
  #define ORIG_IMAGE_SIZE_Y 25
  #define IMAGE_SIZE_X 25
  #define IMAGE_SIZE_Y 25
*/

#define IMAGE_FREQ   15
#define ORIG_IMAGE_FIRST_X 148
#define ORIG_IMAGE_FIRST_Y 108
#define ORIG_IMAGE_SIZE_X 25
#define ORIG_IMAGE_SIZE_Y 25
#define IMAGE_SIZE_X 25
#define IMAGE_SIZE_Y 25


#define NUM_PIXELS ((IMAGE_SIZE_X)*(IMAGE_SIZE_Y))
#define MAX_STRING_LENGTH 80


#define NUM_IR_SENSORS_AT_HEIGHT_1 24
#define NUM_IR_SENSORS_AT_HEIGHT_2 24
#define NUM_IR_SENSORS_AT_HEIGHT_3 8
#define NUM_IR_SENSORS ((NUM_IR_SENSORS_AT_HEIGHT_1)+(NUM_IR_SENSORS_AT_HEIGHT_2)+(NUM_IR_SENSORS_AT_HEIGHT_3))

#define MAX_TACTILES_PER_ROW 16
#define NUM_TACTILE_SENSORS_AT_HEIGHT_1 (MAX_TACTILES_PER_ROW)
#define NUM_TACTILE_SENSORS_AT_HEIGHT_2 (MAX_TACTILES_PER_ROW)
#define NUM_TACTILE_SENSORS_AT_HEIGHT_3 (MAX_TACTILES_PER_ROW)
#define NUM_TACTILE_SENSORS_AT_HEIGHT_4 (MAX_TACTILES_PER_ROW)
#define NUM_TACTILE_SENSORS ((NUM_TACTILE_SENSORS_AT_HEIGHT_1)+(NUM_TACTILE_SENSORS_AT_HEIGHT_2)+(NUM_TACTILE_SENSORS_AT_HEIGHT_3)+(NUM_TACTILE_SENSORS_AT_HEIGHT_4))

#define NUM_LASER_SENSORS_FRONT 180
#define NUM_LASER_SENSORS_REAR  180
#define NUM_LASER_SENSORS ((NUM_LASER_SENSORS_FRONT)+(NUM_LASER_SENSORS_REAR))

/************************************************************************\
 ************************************************************************
\************************************************************************/


typedef struct {
  float x, y, orientation;
} robot_position_type;


extern robot_position_type robot_position;

#define MAX_NUM_MARKERS 20

typedef struct {
  float x[MAX_NUM_MARKERS]; 
  float y[MAX_NUM_MARKERS];
  int   window[MAX_NUM_MARKERS]; /* see window list above */
  int   type[MAX_NUM_MARKERS];	/* 1=authentic, 0=derived */
  int   num_markers;
} marker_type;


typedef struct {
  int lit[6];
  int pushed[4];
} buttons_type;


#define BASE_COMMAND 0

typedef struct {
  int   command_type;
  float trans_velocity;
  float rot_velocity;
} control_type;

struct pattern_type {
  int                  number;
  struct timeval       *time;
  char                 *name;
  float                *sonars;
  unsigned char        *image;
  robot_position_type  *position;
  float                *irs;
  float                *tactiles;
  float                *lasers;
  buttons_type         *buttons;
  float                *pantilt;
  control_type         *control;
  marker_type          *markers;
  struct pattern_type  *next;       
  struct pattern_type  *previous;
};


/*
 * Type definitions for pattern sets
 */
#define UNKNOWN_SET_TYPE  0
#define TESTING_SET_TYPE  1
#define TRAINING_SET_TYPE 2
#define NUM_SET_TYPES     3	/* must be here - expand, if you introduce
				 * a new set type*/

struct patternset_type {
  int                    number;
  char                   *name;
  int                    type;	/* 1=TRAINING_SET, 0=TESTING_SET */
  int                    num_patterns;
  struct pattern_type    *first;       
  struct patternset_type *next_set;
  struct patternset_type *previous_set;
};



struct data_type {
  int                    num_sets;
  struct patternset_type *first_set;
  int                    total_number_patterns;
};


extern struct patternset_type *current_patternset;
extern struct pattern_type *current_pattern;
extern struct data_type *data;



/************************************************************************\
 ************************************************************************
\************************************************************************/


int 
mem_append_pattern(struct patternset_type *patternset,
		   struct pattern_type *pattern);


void
mem_create_data();


struct patternset_type *
mem_create_patternset();


struct pattern_type  *
mem_create_pattern();


void
mem_clear_pattern(struct pattern_type *pattern);


void 
mem_delete_pattern(struct pattern_type **patternPTR);


void 
mem_delete_pattern_set(struct patternset_type **patternsetPTR);


void 
mem_delete_data();


int
mem_fill_pattern_slot(struct pattern_type  *pattern,  /* NULL = leave as is */
		      char                 *name,     /* NULL = leave as is */
		      struct timeval       *time,     /* NULL = leave as is */
		      unsigned char        *image,    /* NULL = leave as is */
		      float                *sonars,   /* NULL = leave as is */
		      float                *irs,      /* NULL = leave as is */
		      float                *tactiles, /* NULL = leave as is */
		      float                *lasers,   /* NULL = leave as is */
		      buttons_type         *buttons,  /* NULL = leave as is */
		      float                *pantilt,  /* NULL = leave as is */
		      robot_position_type  *position, /* NULL = leave as is */
		      marker_type          *markers,  /* NULL = leave as is */
		      control_type         *control);


int
mem_fill_pattern_set_slot(struct patternset_type *patternset, 
			  char                   *name, /* NULL=leave as is */
			  int                    type);


struct pattern_type  *
mem_get_first_pattern(struct patternset_type *patternset);


struct pattern_type  *
mem_get_next_pattern(struct pattern_type *pattern);


struct pattern_type  *
mem_get_previous_pattern(struct pattern_type *pattern);


struct pattern_type  *
mem_get_nth_pattern(struct patternset_type *patternset, int n);


int
mem_number_pattern_sets();


struct patternset_type  *
mem_get_nth_patternset(int n);


int
mem_load_patterns(char *filename, 
		  int append,
		  int G_button);


int
mem_save_patterns(char *filename, int G_button);

