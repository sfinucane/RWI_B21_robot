

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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/learn/fa.h,v $
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
 * $Log: fa.h,v $
 * Revision 1.1  2002/09/14 16:36:05  rstone
 * *** empty log message ***
 *
 * Revision 1.1  1998/06/20 21:05:31  thrun
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


extern int X_number_of_networks;
extern network_type X_networks[MAX_NUMBER_NETWORKS];
extern int X_training_phase;
extern int X_calculate_gradients;




/************************************************************************\
 ************************************************************************
\************************************************************************/




int
X_register_neural_network(char *name,
			  int input_type, /* det. vs. stochastic */
			  int input_dimension, 
			  int output_type,  /* det. vs. stochastic */
			  int output_dimension, 
			  int num_pseudo_inputs_per_input,
			  int n_hidden_units);


int
X_register_radial_basis_network(char *name,
				int input_type, 
				int input_dimension, 
				int output_type, 
				int output_dimension,
				int num_bases_per_input);


int
X_register_network_internal(char *name,
			    int input_type, 
			    int input_dimension, 
			    int output_type, 
			    int output_dimension,
			    int num_pseudo_inputs_per_input,
			    int n_hidden_units,
			    int num_bases_per_input,
			    int type);


void
X_init_network_parameters_randomly(int net_id);


void
X_variable_set_learning_flag(int variable_id, int mode);


void
X_network_clear_pending_updates();


void
X_network_set_learning_flag(int net_id, int mode);


void
X_network_set_stepsize(int net_id, float step_size);


void
X_network_set_momentum(int net_id, float momentum);


void
X_network_set_weights_init_range(int net_id, float weights_init_range);


void
X_initialize_training_epoch();


void
X_compute_neuro_net(int net_id);


float 
X_logistic(float x);


void
X_chain_gradients(float *grad_in1, /* e.g., the gradients wrt. to input 
				    * dimension: dim1 x dim2 */
		  float *grad_in2, /* e.g., the output-input gradients
				    * dimension: dim2 x dim3 */
		  float *grad_out, /* the result, grad1 x grad2 (product)
				    * dimenion: dim1 x dim3 
				    * memory must already be allocated */
		  int dim1, int dim2, int dim3);


void
X_extract_partial_matrix(float *matrix_in, /* from */
			 float *matrix_out, /* to */
			 int matrix_in_dim_outer, /* from-dim outer */
			 int matrix_in_dim_inner, /* from-dim inner */
			 int matrix_out_dim_outer, /* to-dim outer */
			 int matrix_out_dim_inner, /* to-dim inner */
			 int offset_outer, /* offset outer */
			 int offset_inner);


void
X_print_neuro_net(int net_id);


void
X_neuro_net(int net_id, int variable_id_from, int variable_id_to);


void
X_set_target(int output_variable_id, int target_variable_id);


void
X_train(int print_error);


void
X_clear_all_gradients(int variable_id, int all_gradients);


void
X_add_gradients(int variable_id, /* variable, under which gradients will be
				  * memorized */
		int net_id,	 /* network, to which gradients aply */
		int cumul,	 /* set to 1, if you want to add gradients
				  * to the average gradients - 0 otherwise.
				  * 0 is the usual value here */
		float *gradient_value,  /* array of gradients */
		float weighting_factor,	/* all gradients will be multiplied
					 * with this value */
		int specific_output_index);


int
X_save_parameters(char *filename, 
		  char *best_filename, 
		  int check_timer, 
		  int display);


int
X_load_parameters(char *filename);


void
X_update_best_parameters(int net_id);


void
X_main_training_loop();

