
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/learn/dat.c,v $
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
 * $Log: dat.c,v $
 * Revision 1.1  2002/09/14 16:36:05  rstone
 * *** empty log message ***
 *
 * Revision 1.4  1998/08/22 03:08:35  thrun
 * .
 *
 * Revision 1.3  1997/10/23 02:28:21  thrun
 * .
 *
 * Revision 1.2  1997/10/07 00:37:36  thrun
 * .
 *
 * Revision 1.1  1997/10/05 18:11:16  thrun
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

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>

#include "EZX11.h"
#include "o-graphics.h"
#include "robot_specifications.h"
#include "dat.h"

/************************************************************************\
 ************************************************************************
\************************************************************************/

struct patternset_type *current_patternset = NULL;
struct pattern_type *current_pattern = NULL;
struct data_type *data = NULL;



/************************************************************************\
 ************************************************************************
\************************************************************************/






/************************************************************************
 *
 *   NAME:         mem_append_pattern
 *                 
 *   FUNCTION:     appends a pattern into a pattern set, the mattern
 *                 must have been created
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/




int 
mem_append_pattern(struct patternset_type *patternset,
		   struct pattern_type *pattern)
{
  struct pattern_type *last;
  int    i;

  if (patternset == NULL){
    fprintf(stderr, "ERROR: Pattern set not allocated. Cannot append.");
    return 0;
  }
  
  else if (patternset->first == NULL){
    patternset->first          = pattern;
    pattern->previous          = NULL;
    pattern->next              = NULL;
    pattern->number            = 0;
    patternset->num_patterns   = 1;

  }
    
  else{
    for (last = patternset->first, i = 1;
	 last->next != NULL;
	 last = last->next, i++);
    last->next = pattern;
    pattern->previous = last;
    pattern->next     = NULL;
    pattern->number   = i;
    patternset->num_patterns   = patternset->num_patterns + 1;

  }

  data->total_number_patterns++;

  
  return 1;
}


/************************************************************************
 *
 *   NAME:         mem_create_data
 *                 
 *   FUNCTION:     creates a new data set (=set of set of patterns)
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
mem_create_data()
{
  if (data != NULL){
    fprintf(stderr, "ERROR: data already defined. Can only define one.\n");
    return;
  }
  
  data = malloc(sizeof(struct data_type));
  if (data == NULL){
    fprintf(stderr, "ERROR: Out of memory in mem_create_data().\n");
    exit (-1);
  }
  
  data->num_sets  = 0;
  data->first_set = NULL;
  data->total_number_patterns = 0;
  return;
}



/************************************************************************
 *
 *   NAME:         mem_create_patternset
 *                 
 *   FUNCTION:     creates a new pattern set (incl memory allocation)
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


struct patternset_type *
mem_create_patternset()
{
  struct patternset_type *patternset;
  struct patternset_type *patternset2;
  int i;

  patternset = (struct patternset_type *)
    malloc(sizeof(struct patternset_type));
  if (patternset == NULL){
    fprintf(stderr, "ERROR: Out of memory in mem_create_patternset().\n");
    exit (-1);
  }
  patternset->type = UNKNOWN_SET_TYPE;
  patternset->name = NULL;
  patternset->num_patterns  = 0;
  patternset->first = NULL;
  patternset->next_set = NULL;
  
  if (data == NULL){
    mem_create_data();		/* creates the structure "data" */
    data->first_set = patternset;
    data->num_sets = 1;
    patternset->number = 0;
    patternset->previous_set = NULL;
  }
  
  else{
    for (patternset2 = data->first_set, i = 1; 
	 patternset2->next_set != NULL; 
	 patternset2 = patternset2->next_set, i++);
    patternset2->next_set = patternset;
    patternset->previous_set = patternset2;
    data->num_sets = data->num_sets + 1;
    patternset->number = i;
  }

  return patternset;
}





/************************************************************************
 *
 *   NAME:         mem_create_pattern
 *                 
 *   FUNCTION:     creates a pattern (allocation)
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


struct pattern_type  *
mem_create_pattern()
{
  struct pattern_type *pattern;
  
  pattern = (struct pattern_type *) malloc(sizeof(struct pattern_type));
  if (pattern == NULL){
    fprintf(stderr, "ERROR: Out of memory in mem_create_pattern().\n");
    exit (-1);
  }
  pattern->name     = NULL;
  pattern->time     = NULL;
  pattern->sonars   = NULL;
  pattern->image    = NULL;
  pattern->position = NULL;
  pattern->irs      = NULL;
  pattern->lasers   = NULL;
  pattern->tactiles = NULL;
  pattern->buttons  = NULL;
  pattern->pantilt  = NULL;
  pattern->control  = NULL;
  pattern->markers  = NULL;
  pattern->next     = NULL;
  pattern->previous = NULL;
  
  return pattern;



/************************************************************************
 *
 *   NAME:         mem_clear_pattern
 *                 
 *   FUNCTION:     clears a pattern (frees some of the memory)
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/
}  

void
mem_clear_pattern(struct pattern_type *pattern)
{
  if (pattern->name != NULL){
    free(pattern->name);
    pattern->name = NULL;
  }
  
  if (pattern->time != NULL){
    free(pattern->time);
    pattern->time = NULL;
  }

  if (pattern->sonars != NULL){
    free(pattern->sonars);
    pattern->sonars = NULL;
  }

  if (pattern->image != NULL){
    free(pattern->image);
    pattern->image = NULL;
  }

  if (pattern->irs != NULL){
    free(pattern->irs);
    pattern->irs = NULL;
  }

  if (pattern->lasers != NULL){
    free(pattern->lasers);
    pattern->lasers = NULL;
  }

  if (pattern->buttons != NULL){
    free(pattern->buttons);
    pattern->buttons = NULL;
  }

  if (pattern->pantilt != NULL){
    free(pattern->pantilt);
    pattern->pantilt = NULL;
  }

  if (pattern->tactiles != NULL){
    free(pattern->tactiles);
    pattern->tactiles = NULL;
  }

  if (pattern->markers != NULL){
    free(pattern->markers);
    pattern->markers = NULL;
  }

  if (pattern->position != NULL){
    free(pattern->position);
    pattern->position = NULL;
  }

  if (pattern->control != NULL){
    free(pattern->control);
    pattern->control = NULL;
  }
}





/************************************************************************
 *
 *   NAME:         mem_delete_pattern
 *                 
 *   FUNCTION:     deletes a pattern and frees all memory
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void 
mem_delete_pattern(struct pattern_type **patternPTR)
{
  char num_pat_txt[128];

  mem_clear_pattern(*patternPTR);
  free(*patternPTR);
  *patternPTR = NULL;

  data->total_number_patterns--;

}




/************************************************************************
 *
 *   NAME:         mem_delete_pattern_set
 *                 
 *   FUNCTION:     deletes an entire pattern set and frees all memory
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void 
mem_delete_pattern_set(struct patternset_type **patternsetPTR)
{
  struct pattern_type  *pattern, *next;
  struct patternset_type *patternset = *patternsetPTR;
  int verbose = 0;

  if (patternset == NULL){
    fprintf(stderr, 
	    "ERROR: must create patternset first before deleting it.\n");
    return;
  }

  pattern = patternset->first;

  if (pattern != NULL){
    do{
      if (verbose)
	fprintf(stderr, "Deleting pattern %s\n", pattern->name);
      next = pattern->next;
      mem_delete_pattern(&pattern);
      pattern = next;
    } while (pattern != NULL);
  }

  if (verbose)
    fprintf(stderr, "Deleting pattern set %s\n", patternset->name);
  
  if (patternset->name != NULL){
    free(patternset->name);
    patternset->name = NULL;
  }
  
  free(patternset);

  *patternsetPTR = NULL;

}




/************************************************************************
 *
 *   NAME:         mem_delete_data_set
 *                 
 *   FUNCTION:     deletes all pattern sets
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void 
mem_delete_data()
{
  struct patternset_type  *patternset, *next_set;
  int verbose = 0;

  if (data == NULL){
    fprintf(stderr, 
	    "ERROR: must create data first before deleting it.\n");
    return;
  }

  patternset = data->first_set;

  if (patternset != NULL){
    do{
      if (verbose)
	fprintf(stderr, "Deleting pattern set %s...\n", patternset->name);
      next_set = patternset->next_set;
      mem_delete_pattern_set(&patternset);
      patternset = next_set;
    } while (patternset != NULL);
  }

  if (verbose)
    fprintf(stderr, "Deleting data.\n");
  
  free(data);

  data = NULL;

  /* update_items_button(); */
}





/************************************************************************
 *
 *   NAME:         mem_fill_pattern_slot
 *                 
 *   FUNCTION:     adds a slot value into a pattern, allocates memory if
 *                 necessary
 * 
 *   PARAMETERS:   NULL does not affect that slot
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/


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
		      control_type         *control)  /* NULL = leave as is */
{
  if (pattern == NULL){
    fprintf(stderr, 
	    "ERROR: must create pattern first before filling slots.\n");
    return 0;
  }

  if (name != NULL){
    if (pattern->name == NULL){
      pattern->name = (char *) malloc(sizeof(char) * MAX_STRING_LENGTH);
      if (pattern->name == NULL){
	fprintf(stderr, "ERROR: Out of memory in mem_fill_pattern_slot().\n");
	exit (-1);
      }
    }
    bcopy(name, pattern->name, sizeof(char) * MAX_STRING_LENGTH);
  }

  if (time != NULL){
    if (pattern->time == NULL){
      pattern->time = (struct timeval *) malloc(sizeof(struct timeval));
      if (pattern->time == NULL){
	fprintf(stderr, "ERROR: Out of memory in mem_fill_pattern_slot().\n");
	exit (-1);
      }
    }
    bcopy(time, pattern->time, sizeof(struct timeval));
  }

  if (sonars != NULL){
    if (pattern->sonars == NULL){
      pattern->sonars = (float *) malloc(sizeof(float) * NUM_SONAR_SENSORS);
      if (pattern->sonars == NULL){
	fprintf(stderr, "ERROR: Out of memory in mem_fill_pattern_slot().\n");
	exit (-1);
      }
    }
    bcopy(sonars, pattern->sonars, sizeof(float) * NUM_SONAR_SENSORS);
  }
  
  if (image != NULL){
    if (pattern->image == NULL){
      pattern->image = (unsigned char *) malloc(sizeof(unsigned char) 
						* NUM_PIXELS * 3);
      if (pattern->image == NULL){
	fprintf(stderr, "ERROR: Out of memory in mem_fill_pattern_slot().\n");
	exit (-1);
      }
    }

    bcopy(image, pattern->image, sizeof(unsigned char) * NUM_PIXELS * 3);
  }

  if (position != NULL){
    if (pattern->position == NULL){
      pattern->position = (robot_position_type *)
	malloc(sizeof(robot_position_type));
      if (pattern->position == NULL){
	fprintf(stderr, "ERROR: Out of memory in mem_fill_pattern_slot().\n");
	exit (-1);
      }
    }
    bcopy(position, pattern->position, sizeof(robot_position_type));
  }

  if (irs != NULL){
    if (pattern->irs == NULL){
      pattern->irs = (float *) malloc(sizeof(float) * (NUM_IR_SENSORS));
      if (pattern->irs == NULL){
	fprintf(stderr, "ERROR: Out of memory in mem_fill_pattern_slot().\n");
	exit (-1);
      }
    }
    bcopy(irs, pattern->irs, sizeof(float) * (NUM_IR_SENSORS));
  }
  
  if (tactiles != NULL){
    if (pattern->tactiles == NULL){
      pattern->tactiles = (float *) malloc(sizeof(float) *
					  (NUM_TACTILE_SENSORS));
      if (pattern->tactiles == NULL){
	fprintf(stderr, "ERROR: Out of memory in mem_fill_pattern_slot().\n");
	exit (-1);
      }
    }
    bcopy(tactiles, pattern->tactiles, 
	  sizeof(float) * (NUM_TACTILE_SENSORS));
  }

  
  if (lasers != NULL){
    if (pattern->lasers == NULL){
      pattern->lasers = (float *) malloc(sizeof(float) * NUM_LASER_SENSORS);
      if (pattern->lasers == NULL){
	fprintf(stderr, "ERROR: Out of memory in mem_fill_pattern_slot().\n");
	exit (-1);
      }
    }
    bcopy(lasers, pattern->lasers, sizeof(float) * NUM_LASER_SENSORS);
  }
  
  if (buttons != NULL){
    if (pattern->buttons == NULL){
      pattern->buttons = (buttons_type *) malloc(sizeof(buttons_type));
      if (pattern->buttons == NULL){
	fprintf(stderr, "ERROR: Out of memory in mem_fill_pattern_slot().\n");
	exit (-1);
      }
    }
    bcopy(buttons, pattern->buttons, sizeof(buttons_type));
  }
  
  if (pantilt != NULL){
    if (pattern->pantilt == NULL){
      pattern->pantilt = (float *) malloc(sizeof(float) * 2);
      if (pattern->pantilt == NULL){
	fprintf(stderr, "ERROR: Out of memory in mem_fill_pattern_slot().\n");
	exit (-1);
      }
    }
    bcopy(pantilt, pattern->pantilt, sizeof(float) * 2);
  }
  
  if (markers != NULL){
    if (pattern->markers == NULL){
      pattern->markers = (marker_type *) malloc(sizeof(marker_type));
      if (pattern->markers == NULL){
	fprintf(stderr, "ERROR: Out of memory in mem_fill_pattern_slot().\n");
	exit (-1);
      }
    }
    bcopy(markers, pattern->markers, sizeof(marker_type));
  }
  

  if (control != NULL){
    if (pattern->control == NULL){
      pattern->control = (control_type *) malloc(sizeof(control_type));
      if (pattern->control == NULL){
	fprintf(stderr, "ERROR: Out of memory in mem_fill_pattern_slot().\n");
	exit (-1);
      }
    }
    bcopy(control, pattern->control, sizeof(control_type));
  }
  

  return 1;
}




/************************************************************************
 *
 *   NAME:         mem_fill_pattern_set_slot
 *                 
 *   FUNCTION:     adds a slot value into a pattern set
 *                 
 *   PARAMETERS:   NULL does not affect that slot
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/


int
mem_fill_pattern_set_slot(struct patternset_type *patternset, 
			  char                   *name, /* NULL=leave as is */
			  int                    type) /* -1 = no change */
{
  if (patternset == NULL){
    fprintf(stderr, 
	    "ERROR: must create patternset first before filling slots.\n");
    return 0;
  }

  if (name != NULL){
    if (patternset->name == NULL){
      patternset->name = 
	(char *) malloc(sizeof(char) * MAX_STRING_LENGTH);
      if (patternset->name == NULL){
	fprintf(stderr, 
		"ERROR: Out of memory in mem_fill_pattern_set_slot().\n");
	exit (-1);
      }
    }
    bcopy(name, patternset->name, sizeof(char) * MAX_STRING_LENGTH);
  }

  if (type != -1)
    patternset->type = type;

  return 1;
}




/************************************************************************
 *
 *   NAME:         mem_get_first_pattern
 *                 
 *   FUNCTION:     returns a pointer to the first pattern in a set
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


struct pattern_type  *
mem_get_first_pattern(struct patternset_type *patternset)
{
  if (patternset == NULL){
    fprintf(stderr, "ERROR: Pattern set not allocated. Cannot get first.");
    return NULL;
  }
  else
    return patternset->first;
}




/************************************************************************
 *
 *   NAME:         mem_get_next_pattern
 *                 
 *   FUNCTION:     returns a pointer to the next pattern (NULL if not exist)
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


struct pattern_type  *
mem_get_next_pattern(struct pattern_type *pattern)
{
  if (pattern == NULL){
    fprintf(stderr, "ERROR: Pattern does not exist. Cannot get next.");
    return NULL;
  }
  else
    return pattern->next;
}





/************************************************************************
 *
 *   NAME:         mem_get_previous_pattern
 *                 
 *   FUNCTION:     returns a pointer to the previous pattern 
 *                 (NULL if not exist)
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


struct pattern_type  *
mem_get_previous_pattern(struct pattern_type *pattern)
{
  if (pattern == NULL){
    fprintf(stderr, "ERROR: Pattern does not exist. Cannot get next.");
    return NULL;
  }
  else
    return pattern->previous;
}




/************************************************************************
 *
 *   NAME:         mem_get_nth_pattern
 *                 
 *   FUNCTION:     returns a pointer to the n-th pattern in a pattern set
 *                 (NULL if not exist)
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


struct pattern_type  *
mem_get_nth_pattern(struct patternset_type *patternset, int n)
{
  int i;
  struct pattern_type  *pattern;

  if (patternset == NULL){
    fprintf(stderr, 
	    "ERROR: Pattern set does not exist. Cannot get n-th pattern.");
    return NULL;
  }
  else if (n < 0 || n >= patternset->num_patterns){
    fprintf(stderr, 
	    "ERROR: Pattern set contains only %d patterns (%d).\n", 
	    patternset->num_patterns, n);
    return NULL;
  }
    
  else{
    for (pattern = patternset->first, i = 0; i < n; i++){
      if (pattern->next == NULL){
	fprintf(stderr, "STRANGE: I am a bug. Will you find me?\n");
	return NULL;
      }
      pattern = pattern->next;
    }
    return pattern;
  }
}


/************************************************************************
 *
 *   NAME:         mem_number_pattern_sets
 *                 
 *   FUNCTION:     returns the number of pattern sets
 *                 
 *   PARAMETERS:   none
 *
 *   RETURN-VALUE: int
 *                 
 ************************************************************************/


int
mem_number_pattern_sets()
{
  if (!data)
    return 0;
  else
    return (data->num_sets);
}


/************************************************************************
 *
 *   NAME:         mem_get_nth_pattern_set
 *                 
 *   FUNCTION:     returns a pointer to the n-th patternset in the data
 *                 (NULL if not exist)
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


struct patternset_type  *
mem_get_nth_patternset(int n)
{
  int i;
  struct patternset_type  *patternset;

  if (data == NULL){
    fprintf(stderr, 
	    "ERROR: Data does not exist. Cannot get n-th pattern set.");
    return NULL;
  }
  else if (n < 0 || n >= data->num_sets){
    fprintf(stderr, 
	    "ERROR: Only %d pattern sets defined (%d).\n", 
	    data->num_sets, n);
    return NULL;
  }
    
  else{
    for (patternset = data->first_set, i = 0; i < n; i++){
      if (patternset->next_set == NULL){
	fprintf(stderr, "STRANGE: I am a bug. Will you find me?\n");
	return NULL;
      }
      patternset = patternset->next_set;
    }
    return patternset;
  }
}








/************************************************************************
 *
 *   NAME:         mem_load_patterns()
 *                 
 *   FUNCTION:     loads a patterns set from the pattern file
 *                 
 *   PARAMETERS:   filename
 *                 append  (1=append)
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/


int
mem_load_patterns(char *filename, 
		  int append,
		  int G_button)
{
  FILE  *iop;
  int   i, j, file_ended, error, size_i, size_j, size_k, size_l, n, m;
  char  load_button_text[128], colon;
  char  name[MAX_STRING_LENGTH];
  char  text[MAX_STRING_LENGTH];
  char  command[MAX_STRING_LENGTH];  
  int   reading_pattern_set, reading_pattern;
  int   value, value2;
  unsigned char *image;
  float *sonars, float_value, float_value2;
  float *lasers, *irs;
  float *tactiles, *pantilt;
  marker_type *markers;
  buttons_type *buttons;
  control_type *control;
  robot_position_type position;
  struct patternset_type *patternset = NULL;
  struct pattern_type *pattern = NULL;
  int verbose = 0;
  struct timeval time;

  image    = (unsigned char *) malloc(sizeof(unsigned char) * NUM_PIXELS * 3);
  sonars   = (float *) malloc(sizeof(float) * (NUM_SONAR_SENSORS));
  irs      = (float *) malloc(sizeof(float) * (NUM_IR_SENSORS));
  tactiles = (float *) malloc(sizeof(float) * (NUM_TACTILE_SENSORS));
  lasers   = (float *) malloc(sizeof(float) * (NUM_LASER_SENSORS));
  buttons  = (buttons_type *) malloc(sizeof(buttons_type));
  pantilt  = (float *) malloc(sizeof(float) * 2);
  markers  = (marker_type *) malloc(sizeof(marker_type));
  control  = (control_type *) malloc(sizeof(control_type));

  if (image == NULL || sonars == NULL || irs == NULL ||
      tactiles == NULL || lasers == NULL){
    fprintf(stderr, "ERROR: Out of memory in mem_load_patterns().\n");
    exit (-1);
  }

  if (G_button >= 0){
    if (append)
      sprintf(load_button_text, "...appending");
    else
      sprintf(load_button_text, "...loading");
    G_set_new_text(G_button, load_button_text, 1);
    G_display_switch(G_button, 1);
  }
  else
    fprintf(stderr, "Loading patterns from file %s...", filename);


  if ((iop = fopen(filename, "r")) == 0){
    fprintf(stderr, "ERROR: Could not open output file %s.\n", filename);
    if (G_button >= 0){
      G_display_switch(G_button, 2);
      usleep(400000);
      G_display_switch(G_button, 0);
    }
    else
      fprintf(stderr, "failed.\n");
    return 0;
  }


  if (!append && data != NULL)
    mem_delete_data();


  reading_pattern_set = 0;
  reading_pattern     = 0;
  n = 0, m = 0;

  for (file_ended = 0, error = 0; !file_ended && !error; ){
    if (fscanf(iop, "%s", command) == EOF)
      file_ended = 1;
    
    else{
      /*fprintf(stderr, "[%s]\n", command);*/


      /*
       * begin(patternset)
       */
      if (reading_pattern_set == 0 &&
	  reading_pattern == 0 &&
	  !strcmp(command, "begin(patternset)")){
	reading_pattern_set = 1;
	if (verbose)
	  fprintf(stderr, "begin(patternset)\n");
	patternset = mem_create_patternset();
      }

      /*
       * name:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 0 &&
	       !strcmp(command, "name:")){
	if (fscanf(iop, "%s", &name) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "set-name: %s\n", name);
	  mem_fill_pattern_set_slot(patternset, name, -1);
	}
      }
      /*
       * type:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 0 &&
	       !strcmp(command, "type:")){
	if (fscanf(iop, "%s", &text) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "type: %s\n", text);
	  if (!strcmp(text, "training"))
	    mem_fill_pattern_set_slot(patternset, NULL, TRAINING_SET_TYPE);
	  else if (!strcmp(text, "testing"))
	    mem_fill_pattern_set_slot(patternset, NULL, TESTING_SET_TYPE);
	  else if (!strcmp(text, "unknown"))
	    mem_fill_pattern_set_slot(patternset, NULL, UNKNOWN_SET_TYPE);
	  else{
	    fprintf(stderr, "ERROR: Set type %s not recognized in %s.\n", 
		    text, filename);
	    error = 1;
	    file_ended = 1;
	  }
	}
      }
      
      /*
       * begin(pattern)
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 0 &&
	       !strcmp(command, "begin(pattern)")){
	reading_pattern = 1;
	if (verbose)
	  fprintf(stderr, "begin(pattern)\n");
	pattern = mem_create_pattern();
      }

      /*
       * name:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "name:")){
	if (fscanf(iop, "%s", &name) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "pat-name: %s\n", name);
	  mem_fill_pattern_slot(pattern, name, NULL, NULL, NULL,
				NULL, NULL, NULL, NULL, NULL, NULL, NULL,
				NULL);
	}
      }

      /*
       * time:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "time:")){
	if (fscanf(iop, "%ld", &(time.tv_sec)) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (fscanf(iop, "%ld", &(time.tv_usec)) == EOF){
	    fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	    error = 1;
	    file_ended = 1;
	  }
	  else{
	    if (verbose)
	      fprintf(stderr, "time: %ld %ld\n", time.tv_sec, time.tv_usec);
	    mem_fill_pattern_slot(pattern, NULL, &time, NULL, NULL,
				  NULL, NULL, NULL, NULL, NULL, NULL, NULL,
				  NULL);
	  }
	}
      }

      /*
       * image:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "image")){
	if (fscanf(iop, "%d %d : ", &size_i, &size_j) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "image: %d %d\n", size_i, size_j);
	  if (size_i != IMAGE_SIZE_X || size_j != IMAGE_SIZE_Y){
	    fprintf(stderr, 
		    "ERROR: image format mismatch (%d %d vs %d %d).\n", 
		    size_i, size_j, IMAGE_SIZE_X, IMAGE_SIZE_Y);
	    error = 1;
	  }
	  for (i = 0; i < size_i * size_j * 3 && !error; i++)
	    if (fscanf(iop, "%2x", &value) == EOF){
	      fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	      error = 1;
	      file_ended = 1;
	    }
	    else
	      image[i] = (unsigned char) value;
	  if (!error)
	    mem_fill_pattern_slot(pattern, NULL, NULL, image, NULL,
				  NULL, NULL, NULL, NULL, NULL, NULL, NULL,
				  NULL);
	}
      }

      /*
       * sonars:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "sonars")){
	if (fscanf(iop, "%d : ", &size_i) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "sonars: %d\n", size_i);
	  if (size_i != NUM_SONAR_SENSORS){
	    fprintf(stderr, 
		    "ERROR: sonars format mismatch (%d vs %d).\n", 
		    size_i, NUM_SONAR_SENSORS);
	    error = 1;
	  }
	  for (i = 0; i < size_i && !error; i++)
	    if (fscanf(iop, "%f", &float_value) == EOF){
	      fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	      error = 1;
	      file_ended = 1;
	    }
	    else
	      sonars[i] = float_value;
	  if (!error)
	    mem_fill_pattern_slot(pattern, NULL, NULL, NULL, sonars,
				  NULL, NULL, NULL, NULL, NULL, NULL, NULL,
				  NULL);
	}
      }


      /*
       * position:
       */

      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "position:")){
	if (verbose)
	  fprintf(stderr, "position:\n");

	if (fscanf(iop, "%f", &float_value) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  position.x = float_value;
	  if (fscanf(iop, "%f", &float_value) == EOF){
	    fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	    error = 1;
	    file_ended = 1;
	  }
	  else{
	    position.y = float_value;
	    if (fscanf(iop, "%f", &float_value) == EOF){
	      fprintf(stderr, "ERROR: surprising end of file %s.\n",
		      filename);
	      error = 1;
	      file_ended = 1;
	    }
	    else{
	      position.orientation = float_value;
	      mem_fill_pattern_slot(pattern, NULL, NULL, NULL, NULL, NULL, 
				    NULL, NULL, NULL, NULL, &position, NULL,
				    NULL);
	    }
	  }
	}
      }


      /*
       * irs:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "infrareds")){
	if (fscanf(iop, "%d %d %d : ", &size_i, &size_j, &size_k) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "infrareds: %d %d %d\n", size_i, size_j, size_k);
	  if (size_i != NUM_IR_SENSORS_AT_HEIGHT_1 ||
	      size_j != NUM_IR_SENSORS_AT_HEIGHT_2 ||
	      size_k != NUM_IR_SENSORS_AT_HEIGHT_3){
	    fprintf(stderr, 
		    "ERROR: infrared format mismatch (%d %d %d vs %d %d %d).\n", 
		    size_i, size_j, size_k,
		    NUM_IR_SENSORS_AT_HEIGHT_1,
		    NUM_IR_SENSORS_AT_HEIGHT_2,
		    NUM_IR_SENSORS_AT_HEIGHT_3);
	    error = 1;
	  }
	  for (i = 0; i < size_i + size_j + size_k && !error; i++)
	    if (fscanf(iop, "%g", &float_value) == EOF){
	      fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	      error = 1;
	      file_ended = 1;
	    }
	    else
	      irs[i] = float_value;
	  if (!error)
	    mem_fill_pattern_slot(pattern, NULL, NULL, NULL, NULL,
				  irs, NULL, NULL, NULL, NULL, NULL, NULL,
				  NULL);
	}
      }


      /*
       * tactiles:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "tactiles")){
	if (fscanf(iop, "%d %d %d %d : ", 
		   &size_i, &size_j, &size_k, &size_l) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "tactiles: %d %d %d %d\n",
		    size_i, size_j, size_k, size_l);
	  if (size_i != NUM_TACTILE_SENSORS_AT_HEIGHT_1 &&
	      size_j != NUM_TACTILE_SENSORS_AT_HEIGHT_2 &&
	      size_k != NUM_TACTILE_SENSORS_AT_HEIGHT_3 &&
	      size_l != NUM_TACTILE_SENSORS_AT_HEIGHT_4){
	    fprintf(stderr, 
		    "ERROR: tactile format mismatch (%d %d %d %d vs %d %d %d %d).\n", 
		    size_i, size_j, size_k, size_l,
		    NUM_TACTILE_SENSORS_AT_HEIGHT_1,
		    NUM_TACTILE_SENSORS_AT_HEIGHT_2,
		    NUM_TACTILE_SENSORS_AT_HEIGHT_3,
		    NUM_TACTILE_SENSORS_AT_HEIGHT_4);
	    error = 1;
	  }
	  for (i = 0; i < size_i + size_j +size_k + size_l && !error; i++)
	    if (fscanf(iop, "%f", &float_value) == EOF){
	      fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	      error = 1;
	      file_ended = 1;
	    }
	    else
	      tactiles[i] = float_value;
	  if (!error)
	    mem_fill_pattern_slot(pattern, NULL, NULL, NULL, NULL, NULL, 
				  tactiles, NULL, NULL, NULL, NULL, NULL,
				  NULL);
	}
      }




      /*
       * lasers:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "lasers")){
	if (fscanf(iop, "%d %d : ", &size_i, &size_j) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "lasers: %d %d %d\n", size_i, size_j);
	  if (size_i != NUM_LASER_SENSORS_FRONT ||
	      size_j != NUM_LASER_SENSORS_REAR){
	    fprintf(stderr, 
		    "ERROR: infrared format mismatch (%d %d vs %d %d).\n", 
		    size_i, size_j,
		    NUM_LASER_SENSORS_FRONT, NUM_LASER_SENSORS_REAR);
	    error = 1;
	  }
	  for (i = 0; i < size_i + size_j && !error; i++)
	    if (fscanf(iop, "%g", &float_value) == EOF){
	      fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	      error = 1;
	      file_ended = 1;
	    }
	    else
	      lasers[i] = float_value;
	  if (!error)
	    mem_fill_pattern_slot(pattern, NULL, NULL, NULL, NULL, NULL, 
				  NULL, lasers, NULL, NULL, NULL, NULL,
				  NULL);
	}
      }


      /*
       * buttons:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "buttons:")){
	if (verbose)
	  fprintf(stderr, "buttons:\n");
	for (i = 0; i < 6 && !error; i++)
	  if (fscanf(iop, "%d", &value) == EOF){
	    fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	    error = 1;
	    file_ended = 1;
	  }
	  else
	    buttons->lit[i] = value;
	for (i = 0; i < 4 && !error; i++)
	  if (fscanf(iop, "%d", &value) == EOF){
	    fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	    error = 1;
	    file_ended = 1;
	  }
	  else
	    buttons->pushed[i] = value;
	if (!error)
	  mem_fill_pattern_slot(pattern, NULL, NULL, NULL, NULL,
				NULL, NULL, NULL, buttons, NULL, NULL, NULL,
				NULL);
      }



      /*
       * pantilt:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "pantilt:")){
	if (verbose)
	  fprintf(stderr, "pantilt:\n");
	for (i = 0; i < 2 && !error; i++)
	  if (fscanf(iop, "%g", &float_value) == EOF){
	    fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	    error = 1;
	    file_ended = 1;
	  }
	  else
	    pantilt[i] = float_value;
	if (!error)
	  mem_fill_pattern_slot(pattern, NULL, NULL, NULL, NULL,
				NULL, NULL, NULL, NULL, pantilt, NULL, NULL,
				NULL);
      }


      /*
       * markers:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "markers")){
	if (fscanf(iop, "%d : ", &size_i) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "markers: %d\n", size_i);
	  if (size_i > MAX_NUM_MARKERS){
	    fprintf(stderr, 
		    "ERROR: markers format mismatch (%d larger than %d).\n", 
		    size_i, MAX_NUM_MARKERS);
	    error = 1;
	  }
	  for (i = 0; i < size_i && !error; i++)
	    if (fscanf(iop, "%f %f %d %d", &float_value, &float_value2,
		       &value, &value2) == EOF){
	      fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	      error = 1;
	      file_ended = 1;
	    }
	    else{
	      markers->x[i]      = float_value;
	      markers->y[i]      = float_value2;
	      markers->window[i] = value;
	      markers->type[i]   = value2;
	    }
	  if (!error){
	    markers->num_markers = size_i;
	    mem_fill_pattern_slot(pattern, NULL, NULL, NULL, NULL,
				  NULL, NULL, NULL, NULL, NULL, NULL, markers,
				  NULL);
	  }
	}
      }


      /*
       * control:
       */
      else if (reading_pattern_set == 1 &&
	       reading_pattern == 1 &&
	       !strcmp(command, "control:")){
	if (verbose)
	  fprintf(stderr, "control:\n");
	if (fscanf(iop, "%d", &value) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  control->command_type = value;
	  if (fscanf(iop, "%g", &float_value) == EOF){
	    fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	    error = 1;
	    file_ended = 1;
	  }
	  else{
	    control->trans_velocity = float_value;
	    if (fscanf(iop, "%g", &float_value) == EOF){
	      fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	      error = 1;
	      file_ended = 1;
	    }
	    else{
	      control->rot_velocity = float_value;
	      if (!error)
		mem_fill_pattern_slot(pattern, NULL, NULL, NULL, NULL,
				      NULL, NULL, NULL, NULL, NULL, NULL, NULL,
				      control);
	    }
	  }
	}
      }


      /*
       * end(pattern)
       */
      else if (reading_pattern_set == 1 &&
	  reading_pattern == 1 &&
	  !strcmp(command, "end(pattern)")){
	reading_pattern = 0;
	mem_append_pattern(patternset, pattern);
	n++;
	if (patternset->num_patterns % 50 == 0){
	  if (G_button >= 0){
	    if (append)
	      sprintf(load_button_text, "...appending [%d:%d]", 
		      patternset->number, patternset->num_patterns);
	    else
	      sprintf(load_button_text, "...loading [%d:%d]", 
		      patternset->number, patternset->num_patterns);
	    G_set_new_text(G_button, load_button_text, 1);
	    G_display_switch(G_button, 1);
	  }
	  else
	    fprintf(stderr, "[%d:%d]...", 
		    patternset->number, patternset->num_patterns);
	}
	if (verbose)
	  fprintf(stderr, "end(pattern)\n");
      }

      /*
       * end(patternset)
       */
      else if (reading_pattern_set == 1 &&
	  reading_pattern == 0 &&
	  !strcmp(command, "end(patternset)")){
	reading_pattern_set = 0;
	m++;
	if (verbose)
	  fprintf(stderr, "end(patternset)\n");
      }
    }
  }


  fclose(iop);

  free(control);
  free(markers);
  free(pantilt);
  free(buttons);
  free(lasers);
  free(tactiles);
  free(irs);
  free(image);
  free(sonars);

  /* update_items_button(); */

  if (error){
    if (G_button >= 0){
      G_display_switch(G_button, 2);
      usleep(400000);
      G_display_switch(G_button, 0);
    }
    else
      fprintf(stderr, "failed.\n");
    return 0;
  }

  if (G_button >= 0)
    G_display_switch(G_button, 0);
  else
    fprintf(stderr, "done.\n");
  fprintf(stderr, "%d patterns read in %d sets.\n", n, m);
  return 1;
}






/************************************************************************
 *
 *   NAME:         mem_save_patterns()
 *                 
 *   FUNCTION:     saves all patterns into the pattern file
 *                 
 *   PARAMETERS:   filename
 *                 ordered:  1 = in increasing order
 *
 *   RETURN-VALUE: 1, if successful
 *                 
 ************************************************************************/


int
mem_save_patterns(char *filename, int G_button)
{
  FILE *iop;
  int i, j, k, n, m;
  char save_button_text[128];
  struct pattern_type *pattern = NULL;
  struct patternset_type *patternset = NULL;
		  

  sprintf(save_button_text, "...saving");
  if (G_button >= 0){
    G_set_new_text(G_button, save_button_text, 1);
    G_display_switch(G_button, 1);
  }
  

  /*
   * sanity checks
   */

  if (data == NULL){
    fprintf(stderr, "ERROR: Data set not allocated. Cannot save.");
    if (G_button >= 0){
      G_display_switch(G_button, 2);
      usleep(400000);
      G_display_switch(G_button, 0);
    }
    return 0;
  }

  if (data->first_set == NULL){
    fprintf(stderr, "ERROR: No dataset allocated. Cannot save.");
    if (G_button >= 0){
      G_display_switch(G_button, 2);
      usleep(400000);
      G_display_switch(G_button, 0);
    }
    return 0;
  }

 
  /*
   * attempt to open the file
   */

  if ((iop = fopen(filename, "w")) == 0){
    fprintf(stderr, "ERROR: Could not open output file %s.\n", filename);
    if (G_button >= 0){
      G_display_switch(G_button, 2);
      usleep(400000);
      G_display_switch(G_button, 0);
    }
    return 0;
  }

  
  for (patternset = data->first_set, n = 0, m = 0; patternset != NULL;
       patternset = patternset->next_set, n++){

    fprintf(iop, "\nbegin(patternset)\n");
    if (patternset->name != NULL)
      fprintf(iop, "name: %s\n", patternset->name);
    if (patternset->type == TRAINING_SET_TYPE)
      fprintf(iop, "type: training\n");
    else if (patternset->type == TESTING_SET_TYPE)
      fprintf(iop, "type: testing\n");
    else if (patternset->type == UNKNOWN_SET_TYPE)
      fprintf(iop, "type: unknown\n");
    
    for (i = 0, pattern = mem_get_first_pattern(patternset);
	 pattern != NULL;
	 i++, m++, pattern = mem_get_next_pattern(pattern)){

      /*
       * update the button - for the user's convenience
       */
    

      if (i % 50 == 0 && G_button >= 0){
	sprintf(save_button_text, "...saving [%d:%d]", n, i);
	G_set_new_text(G_button, save_button_text, 1);
	G_display_switch(G_button, 1);
      }

    
      /*
       * write data to file
       */

      fprintf(iop, "\nbegin(pattern)\n");

      if (pattern->name != NULL)
	fprintf(iop, "name: %s\n", pattern->name);

      if (pattern->time != NULL)
	fprintf(iop, "time: %ld %ld\n", 
		pattern->time->tv_sec,  pattern->time->tv_usec);


      if (pattern->image != NULL){
	fprintf(iop, "image %d %d : ", IMAGE_SIZE_X, IMAGE_SIZE_Y);

	for (j = 0; j < NUM_PIXELS * 3; j++)
	  fprintf(iop, "%.2X", pattern->image[j]);
	fprintf(iop, "\n");
      }

      if (pattern->sonars != NULL){
	fprintf(iop, "sonars %d :", NUM_SONAR_SENSORS);
	for (j = 0; j < NUM_SONAR_SENSORS; j++)
	  fprintf(iop, " %g", pattern->sonars[j]);
	fprintf(iop, "\n");
      }

      if (pattern->irs != NULL){
	fprintf(iop, "infrareds %d %d %d :", 
		    NUM_IR_SENSORS_AT_HEIGHT_1,
		    NUM_IR_SENSORS_AT_HEIGHT_2,
		    NUM_IR_SENSORS_AT_HEIGHT_3);
	for (j = 0; j < (NUM_IR_SENSORS); j++)
	  fprintf(iop, " %g", pattern->irs[j]);
	fprintf(iop, "\n");
      }

      if (pattern->tactiles != NULL){
	fprintf(iop, "tactiles %d %d %d %d :", 
		NUM_TACTILE_SENSORS_AT_HEIGHT_1,
		NUM_TACTILE_SENSORS_AT_HEIGHT_2,
		NUM_TACTILE_SENSORS_AT_HEIGHT_3,
		NUM_TACTILE_SENSORS_AT_HEIGHT_4);
	for (j = 0; j < (NUM_TACTILE_SENSORS); j++)
	  fprintf(iop, " %d", (int) pattern->tactiles[j]);
	fprintf(iop, "\n");
      }

      if (pattern->lasers != NULL){
	fprintf(iop, "lasers %d %d :",
		NUM_LASER_SENSORS_FRONT, NUM_LASER_SENSORS_REAR);
	for (j = 0; j < NUM_LASER_SENSORS; j++)
	  fprintf(iop, " %g", pattern->lasers[j]);
	fprintf(iop, "\n");
      }

      if (pattern->buttons != NULL){
	fprintf(iop, "buttons:");
	for (j = 0; j < 6; j++)
	  fprintf(iop, " %d", pattern->buttons->lit[j]);
	fprintf(iop, " ");
	for (j = 0; j < 4; j++)
	  fprintf(iop, " %d", pattern->buttons->pushed[j]);
	fprintf(iop, "\n");
      }

      if (pattern->pantilt != NULL)
	fprintf(iop, "pantilt: %g %g\n",
		pattern->pantilt[0], pattern->pantilt[1]);

      if (pattern->position != NULL){
	fprintf(iop, "position:");
	fprintf(iop, " %g", pattern->position->x);
	fprintf(iop, " %g", pattern->position->y);
	fprintf(iop, " %g", pattern->position->orientation);
	fprintf(iop, "\n");
      }

      if (pattern->markers != NULL){
	fprintf(iop, "markers %d :", pattern->markers->num_markers);
	for (j = 0; j < pattern->markers->num_markers; j++)
	  fprintf(iop, "  %g %g %d %d",
		  pattern->markers->x[j],
		  pattern->markers->y[j],
		  pattern->markers->window[j],
		  pattern->markers->type[j]);
	fprintf(iop, "\n");
      }


      if (pattern->control != NULL)
	fprintf(iop, "control: %d  %g %g\n",
		pattern->control->command_type,
		pattern->control->trans_velocity,
		pattern->control->rot_velocity);

      fprintf(iop, "end(pattern)\n");

    }


    fprintf(iop, "end(patternset)\n");
  }
  
  fclose(iop);
  fprintf(stderr, "%d patterns (%d sets) saved in %s.\n", m, n, filename);

  if (G_button >= 0)
    G_display_switch(G_button, 0);

  return 1;
}

