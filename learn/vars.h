
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/learn/vars.h,v $
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
 * $Log: vars.h,v $
 * Revision 1.1  2002/09/14 16:36:05  rstone
 * *** empty log message ***
 *
 * Revision 1.22  1998/07/04 15:21:39  thrun
 * variable logging.
 *
 * Revision 1.21  1998/06/27 19:13:55  thrun
 * some tuning + training.
 *
 * Revision 1.20  1998/06/20 21:05:33  thrun
 * Reorganization (created: fa.{c,h}, xtypes.h)
 *
 * Revision 1.19  1998/05/05 04:00:37  thrun
 * Direct control mode (and button). Other stuff I don't fully remember.
 * Intermediate version.
 *
 * Revision 1.18  1998/04/28 23:30:48  thrun
 * autosave of weights, we can now switch off learning in selected
 * networks and variables, which is important for incremental
 * programming/learning
 *
 * Revision 1.17  1998/04/26 15:03:09  thrun
 * subroutines appear to work fine. also changed the initialiation.
 *
 * Revision 1.16  1998/04/24 04:49:05  thrun
 * intermediate and buggy version - subroutines don't have their gradient
 * computation quite right yet.
 *
 * Revision 1.15  1998/04/20 03:12:12  thrun
 * Intermediate version with subroutines.
 *
 * Revision 1.14  1998/04/16 02:42:51  thrun
 * copy function for random variables - use with caution!
 *
 * Revision 1.13  1998/04/13 01:45:20  thrun
 * Thermomter encoding for sigmodial neural networks. Delay for
 * pausing (when recording data).
 *
 * Revision 1.12  1997/08/05 22:19:09  thrun
 * intermediate version - do not use (networks are now not linked
 * to variables any longer)
 *
 * Revision 1.11  1997/07/30 03:46:55  thrun
 * New events: pantilt and buttons
 *
 * Revision 1.10  1997/07/26 14:19:06  thrun
 * changed display options
 *
 * Revision 1.9  1997/07/15 17:34:07  thrun
 * *** empty log message ***
 *
 * Revision 1.8  1997/07/14 14:13:19  thrun
 * First attempt to implement multi-phasing. (broken code, don't use).
 *
 * Revision 1.7  1997/07/06 22:51:50  thrun
 * Autosaving of the parameters (every 2 minutes)
 *
 * Revision 1.6  1997/07/06 22:23:15  thrun
 * .
 *
 * Revision 1.5  1997/07/06 18:42:06  thrun
 * Radial basis networks implemented.
 *
 * Revision 1.4  1997/07/05 17:20:29  thrun
 * Temporal integration of stochastic variables works fine now.
 *
 * Revision 1.3  1997/07/04 18:09:34  thrun
 * intermediate version - do not use
 *
 * Revision 1.2  1997/07/04 00:28:57  thrun
 * Intermediate verson of the learning stuff. What works is:
 * - training of deterministic variables (incl. chaining)
 * - training of multiple determinstic with a stochastic variables (incl chaining)
 * What doesn't yet work includes
 * - temporal chaining of stochastic variables, using Bayes rule
 * - convolving: gradients
 * - stochatic input
 *
 * Revision 1.1  1997/06/30 13:51:42  thrun
 * intermediate version - new neural network (which seems to work!).
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




extern struct pattern_type *X_pattern;
extern int X_display;
extern variable_type X_variables[MAX_NUMBER_VARIABLES];
extern int X_number_of_variables;
extern int X_number_of_processes;
extern process_type X_processes[MAX_NUMBER_PROCESSES];
extern float X_prob;

/************************************************************************\
 ************************************************************************
\************************************************************************/

/*
 * application procedures
 */



void
X_initialize_all();


void
X_initialize_episode();


void
X_process_pattern(struct pattern_type *pattern);


void
X_register_process(char       *name,
		   void_code  *code);


void
X_specify_trigger_condition(void_code  *code,
			    int  event_type,
			    int  activate);


int
X_register_variable(char *name, 
		    int dimension, 
		    int display,
		    int log);


int
X_register_stochastic_variable(char *name, 
			       int dimension, 
			       int cyclic,
			       int resolution, 
			       int display,
			       int log);


void
X_initialize_uniformly(int variable_id, float *range_min, float *range_max);


void
X_set_variable(int variable_id, float *values);


void
X_copy_variable(int from_variable_id, int to_variable_id);


void
X_convolve_with_Gaussian(int variable_id,
			 float *means,
			 float *variances, 
			 float cut_off);


void
X_convolve_with_uniform_dist(int variable_id,
			     float randomness);


void
X_normalize_variable(int variable_id);


void
X_truncate_variable(int variable_id, float threshold);


void
X_integrate_variables(int belief_variable_id, int evidence_variable_id);


void
X_max_likelihood(int stochastic_variable_id, int deterministic_variable_id);


void
X_weighted_average(int stochastic_variable_id, int deterministic_variable_id);


void
X_convert_stochastic_to_deterministic(int stochastic_variable_id, 
				      int deterministic_variable_id,
				      int conversion_type);


void
X_execute_subroutine(X_subroutine_type subroutine(),
		     int   num_input_variables,
		     int   num_output_variables,
		     int  *input_variables,
		     int  *output_variables);


void
X_print_value(int variable_id, int value_nr, FILE *iop);


float
X_get_value(int variable_id, int value_nr);


int
X_create_new_variable_display(char *name, 
			      int dim_x, 
			      int dim_y, 
			      float *value);


void
X_display_variable(int variable_id);


void
X_testing(int verbose);



void
X_logging_on();

void
X_logging_off();

void
X_log_variables();

