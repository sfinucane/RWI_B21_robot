

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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/learn/fa.c,v $
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
 * $Log: fa.c,v $
 * Revision 1.1  2002/09/14 16:36:05  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1999/07/11 18:48:32  thrun
 * slight reorganization
 *
 * Revision 1.2  1998/07/04 15:21:37  thrun
 * variable logging.
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

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>

#include "bUtils.h"
#include "EZX11.h"
#include "o-graphics.h"
#include "dat.h"
#include "math.h"
#include "global.h"
#include "mem.h"
#include "xtypes.h"
#include "vars.h"
#include "fa.h"

/************************************************************************\
 ************************************************************************
\************************************************************************/


#define RANDOM_MAX ((float)(2147483647))   /* (2**31 - 1) */
#define RAND_PLUS_MINUS_ONE() ((2.0 * ((float)(random()) / ((float) RANDOM_MAX))) - 1.0)
#define RAND_ZERO_TO_ONE() ((float)(random()) / ((float) RANDOM_MAX))


/************************************************************************\
 ************************************************************************
\************************************************************************/




int X_number_of_networks = 0;
network_type X_networks[MAX_NUMBER_NETWORKS];

/************************************************************************\
 ************************************************************************
\************************************************************************/


float X_error_during_training[NUM_SET_TYPES] = {0.0, 0.0, 0.0};
float X_lowest_error_during_training[NUM_SET_TYPES] = {-1.0, -1.0, -1.0};
float X_prev_error_during_training[NUM_SET_TYPES] = {0.0, 0.0, 0.0};
int   X_error_during_training_increased[NUM_SET_TYPES] = {0, 0, 0};

/************************************************************************\
 ************************************************************************
\************************************************************************/

int   X_training_phase = 0;	/* 0 = no training,  testing and/or display
				 * 1 = compute averages and their gradients
				 * 2 = compute network gradients and 
				 *     update weights */
int   X_calculate_gradients = 0;


static int training_step = 0;



int L1_error_norm = 0;		/* if set to 1, we will use the L1 norm
				 * (whn applicable). Otherwise, we'll use
				 * L2 */


/************************************************************************\
 ************************************************************************
\************************************************************************/




/* **********************************************************************
 *
 * X_register_neural_network(): Used to register a new neural network
 * 
 *
 * **********************************************************************/

int
X_register_neural_network(char *name,
			  int input_type, /* det. vs. stochastic */
			  int input_dimension, 
			  int output_type,  /* det. vs. stochastic */
			  int output_dimension, 
			  int num_pseudo_inputs_per_input,
			  int n_hidden_units)
{
  return X_register_network_internal(name,
				     input_type, 
				     input_dimension, 
				     output_type, 
				     output_dimension, 
				     num_pseudo_inputs_per_input,
				     n_hidden_units,
				     0,
				     LOGISTIC_NETWORK);
}





/* **********************************************************************
 *
 * X_register_radial_basis_network(): Used to register a new neural network
 * 
 *
 * **********************************************************************/

int
X_register_radial_basis_network(char *name,
				int input_type, 
				int input_dimension, 
				int output_type, 
				int output_dimension,
				int num_bases_per_input)
{
  return X_register_network_internal(name,
				     input_type, 
				     input_dimension, 
				     output_type, 
				     output_dimension, 
				     1,
				     0,
				     num_bases_per_input,
				     RADIAL_BASIS_NETWORK);

}





/* **********************************************************************
 *
 * X_register_network internal(): Used to register a new neural network
 * 
 *
 * **********************************************************************/


int
X_register_network_internal(char *name,
			    int input_type, 
			    int input_dimension, 
			    int output_type, 
			    int output_dimension,
			    int num_pseudo_inputs_per_input,
			    int n_hidden_units,
			    int num_bases_per_input,
			    int type)
{
  network_type  *network = NULL;
  int i, j;
    
  /*
   * check, if there's still space in the memory
   */
  
  if (X_number_of_networks >= MAX_NUMBER_NETWORKS){
    fprintf(stderr, "WARNING: No more networks.\n");
    return -1;
  }

  /*
   * sanity checks
   */

   if (input_type != DETERMINISTIC && input_type != STOCHASTIC){
     fprintf(stderr, "ERROR: Unknown input type: %d.\n", input_type);
     return -1;
   }

   if (input_dimension <= 0){
     fprintf(stderr, "ERROR: Invalid input dimension: %d.\n", input_dimension);
     return -1;
   }

   if (output_type != DETERMINISTIC && output_type != STOCHASTIC){
     fprintf(stderr, "ERROR: Unknown output type: %d.\n", output_type);
     return -1;
   }

   if (output_dimension <= 0){
     fprintf(stderr, "ERROR: Invalid output dimension: %d.\n",
	     output_dimension);
     return -1;
   }

   if (num_pseudo_inputs_per_input < 1){
     fprintf(stderr, "ERROR: num_pseudo_inputs_per_input (%d) must be >= 1.\n",
	     num_pseudo_inputs_per_input);
     return -1;
   }


   if (num_pseudo_inputs_per_input != 1 && type == RADIAL_BASIS_NETWORK){
     fprintf(stderr, "Error: RADIAL BASIS functions can only be used ");
     fprintf(stderr, "with num_pseudo_inputs_per_input=1 (%d)\n",
	     num_pseudo_inputs_per_input);
     return -1;
   }

   /*
    * set the network's variables
    */

   network = &(X_networks[X_number_of_networks]);


   network->name                        = name;
   network->type                        = type;
   network->from_variable_type          = input_type;
   network->from_variable_dimension     = input_dimension;
   network->to_variable_type            = output_type;
   network->to_variable_dimension       = output_dimension;
   network->num_pseudo_inputs_per_input = num_pseudo_inputs_per_input;
   network->min_gradient                = INITIAL_MIN_GRADIENT;
   network->max_gradient                = INITIAL_MAX_GRADIENT;
   network->name = (char *) malloc(sizeof(char) * (strlen(name) + 1));
   strcpy(network->name, name);

   {
     static int warning = 0;
     if (INITIAL_MIN_GRADIENT == 0.0 && !warning){
       fprintf(stderr, "\n\n\t### WARNING: no initial min gradient ###\n\n");
       warning = 1;
     }
   }

   /*
    * compute number of inputs and outputs - this is tricky
    */

   network->num_inputs          =  0;
   network->num_outputs         =  0;

   /*
    * input variables directly correspond to the input dimensions,
    * no matter what. 
    */

   network->num_inputs += input_dimension;

   /*
    * output variables are more difficult: 
    *      DETERMINISTIC variables are predicted directly
    *      STOCHASTIC    variables are encoded as input...
    */


   if (output_type == DETERMINISTIC)
     network->num_outputs += output_dimension; 
   else if (output_type == STOCHASTIC){
     network->num_inputs += output_dimension; 
     network->num_outputs = 1;
   }
   else{
     fprintf(stderr, "#$%^&*: What the heck is going on here?\n");
     exit(1);
   }

   /*
    * compute the number of pseudo input units. Pseudo input units
    * are just like input units, but there are more when
    * thermometer encodings are used
    */

   network->num_pseudo_inputs =
     (network->num_inputs * num_pseudo_inputs_per_input);

   /*
    * allocate memory for input_info, input_resolution
    */

   network->input_info  =
     (int *) malloc(sizeof(int) *
		    network->num_inputs);
   
   network->input_resolution =
     (int *) malloc(sizeof(int) *
		    network->num_inputs);

   if (type == RADIAL_BASIS_NETWORK)
     network->num_bases_per_input =
     (int *) malloc(sizeof(int) *
		    network->num_inputs);
   else
     network->num_bases_per_input = NULL;
   
   if (!network->input_info ||
       !network->input_resolution ||
       (type == RADIAL_BASIS_NETWORK &&
	!network->num_bases_per_input)){
     fprintf(stderr, "ERROR: out of memory when allocating network %s\n", 
	     name);
     exit(-1);
   }

   /*
    * set the type if input unit, and the numbers over which to iterate
    */
   
   i = 0;
   /* input units, deterministic variable */
   for (j = 0; j < input_dimension &&
	  input_type == DETERMINISTIC; j++, i++){
     network->input_info[i] = NETWORK_UNIT_IS_DETERMINISTIC_INPUT;
     /*network->input_resolution[i] = 1;*/
     if (type == RADIAL_BASIS_NETWORK)
       network->num_bases_per_input[i] = DEFAULT_NUM_BASES_PER_INPUT;
   }
   /* input units, stochastic variable */
   for (j = 0; j < input_dimension &&
	  input_type == STOCHASTIC; j++, i++){
     network->input_info[i] = NETWORK_UNIT_IS_STOCHASTIC_INPUT;
     /*network->input_resolution[i] = input_resolution;*/
     if (type == RADIAL_BASIS_NETWORK)
       network->num_bases_per_input[i] = num_bases_per_input;
   }
   /* output units, stochastic variable */
   for (j = 0; j < output_dimension &&
	  output_type == STOCHASTIC; j++, i++){
     network->input_info[i] = NETWORK_UNIT_IS_STOCHASTIC_OUTPUT;
     /*network->input_resolution[i] = output_resolution;*/
     if (type == RADIAL_BASIS_NETWORK)
       network->num_bases_per_input[i] = num_bases_per_input;
   }

   /*
    * set the default parameters
    */

   network->weights_init_range = DEFAULT_WEIGHTS_INIT_RANGE;
   network->step_size          = DEFAULT_STEP_SIZE;
   network->momentum           = DEFAULT_MOMENTUM;
   network->learning_flag      = DEFAULT_NETWORK_LEARNING_FLAG;


   /*
    * Just in case: initialize all pointers with NULL. Some of them will
    * be overridden further down, but which ones will depend on the
    * type network. 
    */

   network->params                   = NULL;
   network->input_to_hidden_weights  = NULL;
   network->hidden_biases            = NULL;
   network->hidden_to_output_weights = NULL;
   network->output_biases            = NULL;
   network->input_to_output_weights  = NULL;

   network->input                    = NULL;
   network->pseudo_input             = NULL;
   network->netin_hidden             = NULL;
   network->hidden                   = NULL;
   network->netin_output             = NULL;
   network->output                   = NULL;

   network->d_output_d_params        = NULL;
   network->d_hidden_d_input         = NULL;
   network->d_output_d_input         = NULL;
   network->d_params                 = NULL;
   network->d_error_d_params         = NULL;
   network->prev_d_error_d_params    = NULL;


   /*
    * create the network
    */

   if (network->type == LOGISTIC_NETWORK){

     /*
      * CASE 1: LOGISTIC NETWORK
      */

     network->num_hidden = n_hidden_units;

     if (n_hidden_units > 0)
       network->num_params = 
	 (network->num_pseudo_inputs * network->num_hidden) + 
	 network->num_hidden + 
	 (network->num_hidden * network->num_outputs) +
	 network->num_outputs;
     else
       network->num_params = 
	 (network->num_pseudo_inputs * network->num_outputs) 
	 + network->num_outputs;

     network->params = (float *) malloc(sizeof(float) * network->num_params);
     network->best_params = 
       (float *) malloc(sizeof(float) * network->num_params);
     network->d_output_d_params = 
       (float *) malloc(sizeof(float) * network->num_outputs *
			network->num_params);
     network->d_params = (float *) malloc(sizeof(float) * network->num_params);
     network->d_error_d_params =
       (float *) malloc(sizeof(float) * network->num_params);
     network->prev_d_error_d_params =
       (float *) malloc(sizeof(float) * network->num_params);
     network->d_output_d_input =
       (float *) malloc(sizeof(float) * network->num_outputs *
			network->num_inputs);
     network->input = (float *) malloc(sizeof(float) * network->num_inputs);
     network->pseudo_input = 
       (float *) malloc(sizeof(float) * network->num_pseudo_inputs);
     network->output =
       (float *) malloc(sizeof(float) * network->num_outputs);
     network->netin_output =
       (float *) malloc(sizeof(float) * network->num_outputs);

     if (n_hidden_units > 0){
       network->d_hidden_d_input =
	 (float *) malloc(sizeof(float) * network->num_hidden *
			  network->num_inputs);
       network->netin_hidden =
	 (float *) malloc(sizeof(float) * network->num_hidden);
       network->hidden =
	 (float *) malloc(sizeof(float) * network->num_hidden);
     }
     else{
       network->d_hidden_d_input = NULL;
       network->netin_hidden = NULL;
       network->hidden = NULL;
     }



     if (!network->params                 ||
	 !network->best_params            ||
	 !network->d_output_d_params      ||
	 !network->d_params               ||
	 !network->d_error_d_params       ||
	 !network->prev_d_error_d_params  ||
	 !network->d_output_d_input       ||
	 !network->input                  ||
	 !network->pseudo_input           ||
	 !network->output                 ||
	 !network->netin_output           ||
	 (n_hidden_units > 0              &&
	  (!network->d_hidden_d_input     ||
	   !network->netin_hidden         ||
	   !network->hidden                 ))){
       fprintf(stderr, "ERROR: out of memory when allocating network %s\n", 
	       name);
       exit(-1);
     }

     for (i = 0; i < network->num_outputs *
	    network->num_params; i++)
       network->d_output_d_params[i] = 0.0;
     for (i = 0; i < network->num_params; i++){
       network->d_params[i]              = 0.0;
       network->d_error_d_params[i]      = 0.0;
       network->prev_d_error_d_params[i] = 0.0;
     }
     for (i = 0; i < network->num_outputs * network->num_inputs; i++)
       network->d_output_d_input[i] = 0.0;
     for (i = 0; i < network->num_inputs; i++)
       network->input[i] = 0.0;
     for (i = 0; i < network->num_pseudo_inputs; i++)
       network->pseudo_input[i] = 0.0;
     for (i = 0; i < network->num_outputs; i++)
       network->output[i] = 0.0;
     for (i = 0; i < network->num_outputs; i++)
       network->netin_output[i] = 0.0;
     for (i = 0; i < network->num_params; i++)
       network->params[i] = network->best_params[i] =
	 RAND_PLUS_MINUS_ONE() * network->weights_init_range;

     if (n_hidden_units > 0){
       for (i = 0; i < network->num_hidden * network->num_inputs; i++)
	 network->d_hidden_d_input[i] = 0.0;
       for (i = 0; i < network->num_hidden; i++)
	 network->netin_hidden[i] = 0.0;
       for (i = 0; i < network->num_hidden; i++)
	 network->hidden[i] = 0.0;


       i = 0;
       network->input_to_hidden_weights = &(network->params[i]);
       i += (network->num_pseudo_inputs * network->num_hidden);
     
       network->hidden_biases = &(network->params[i]);
       i += network->num_hidden;
     
       network->hidden_to_output_weights = &(network->params[i]);
       i += (network->num_hidden * network->num_outputs);
     
       network->output_biases = &(network->params[i]);
       i += network->num_outputs;
     
       if (i != network->num_params){
	 fprintf(stderr, "There is something badly wrong here (1)\n");
	 exit(-1);
       } 
       network->input_to_output_weights = NULL;
     } 

     else{
       i = 0;
       network->input_to_output_weights = &(network->params[i]);
       i += (network->num_outputs * network->num_pseudo_inputs);
     
       network->output_biases = &(network->params[i]);
       i += network->num_outputs;
     
       if (i != network->num_params){
	 fprintf(stderr, "There is something badly wrong here (2)\n");
	 exit(-1);
       } 
       network->input_to_hidden_weights  = NULL;
       network->hidden_biases            = NULL;
       network->hidden_to_output_weights = NULL;
     }
   }






   else{
     /*
      * CASE 2: RADIAL BASIS NETWORK
      */

     /* no pseudo inputs */


     if (num_pseudo_inputs_per_input != 1){
       fprintf(stderr, "Error: RADIAL BASIS functions can only be used ");
       fprintf(stderr, "with num_pseudo_inputs_per_input=1 (%d)\n",
	       num_pseudo_inputs_per_input);
       return -1;		/* this is also checked above */
     }
     network->num_pseudo_inputs = 0;
     network->pseudo_input = NULL;

     /*
      * currently, there are exponentially many
      * units, which makes those networks
      * similar to a look-up table 
      */

     network->num_hidden = 1;

     for (i = 0; i < network->num_inputs; i++)
       network->num_hidden *=
	 network->num_bases_per_input[i]; 

     network->num_params = 
       (network->num_hidden * network->num_outputs)
       + network->num_outputs;	

     network->params = (float *) malloc(sizeof(float) * network->num_params);
     network->best_params = 
       (float *) malloc(sizeof(float) * network->num_params);
     network->d_output_d_params =
       (float *) malloc(sizeof(float) * network->num_outputs *
			network->num_params);
     network->d_params = (float *) malloc(sizeof(float) * network->num_params);
     network->d_error_d_params = 
       (float *) malloc(sizeof(float) * network->num_params);
     network->prev_d_error_d_params =
       (float *) malloc(sizeof(float) * network->num_params);
     network->d_hidden_d_input =
       (float *) malloc(sizeof(float) * network->num_hidden *
			network->num_inputs);
     network->d_output_d_input =
       (float *) malloc(sizeof(float) * network->num_outputs *
			network->num_inputs);
     network->input  = (float *) malloc(sizeof(float) * network->num_inputs);
     network->hidden = (float *) malloc(sizeof(float) * network->num_hidden);
     network->output = (float *) malloc(sizeof(float) * network->num_outputs);
     network->netin_output =
       (float *) malloc(sizeof(float) * network->num_outputs);

     if (!network->params                 ||
	 !network->best_params            ||
	 !network->d_output_d_params      ||
	 !network->d_params               ||
	 !network->d_error_d_params       ||
	 !network->prev_d_error_d_params  ||
	 !network->d_output_d_input       ||
	 !network->d_hidden_d_input       ||
	 !network->input                  ||
	 !network->hidden                 ||
	 !network->output                 ||
	 !network->netin_output           ){
       fprintf(stderr, "ERROR: out of memory when allocating network %s\n", 
	       name);
       exit(-1);
     }

     for (i = 0; i < network->num_outputs * network->num_params; i++)
       network->d_output_d_params[i] = 0.0;
     for (i = 0; i < network->num_params; i++){
       network->d_params[i]              = 0.0;
       network->d_error_d_params[i]      = 0.0;
       network->prev_d_error_d_params[i] = 0.0;
     }
     for (i = 0; i < network->num_outputs * network->num_inputs; i++)
       network->d_output_d_input[i] = 0.0;
     for (i = 0; i < network->num_hidden * network->num_inputs; i++)
       network->d_hidden_d_input[i] = 0.0;
     for (i = 0; i < network->num_inputs; i++)
       network->input[i] = 0.0;
     for (i = 0; i < network->num_hidden; i++)
       network->hidden[i] = 0.0;
     for (i = 0; i < network->num_outputs; i++)
       network->output[i] = 0.0;
     for (i = 0; i < network->num_outputs; i++)
       network->netin_output[i] = 0.0;
     for (i = 0; i < network->num_params; i++)
       network->params[i] = network->best_params[i] =
	 RAND_PLUS_MINUS_ONE() * network->weights_init_range;

     
     network->hidden_to_output_weights = network->params;
     network->output_biases =
       &(network->params[network->num_hidden * network->num_outputs]);
     
     network->netin_hidden             = NULL;
     network->input_to_output_weights  = NULL;
     network->input_to_hidden_weights  = NULL;
     network->hidden_biases            = NULL;

   }


   /*
    * increment number of networks, and return
    */

   fprintf(stderr, "\tRegistered:  network \"%s\".\n", name);
   X_number_of_networks++;


   return (X_number_of_networks - 1);
}








/* **********************************************************************
 *
 * X_init_network_parameters_randomly() initializes a neural network
 *              (this is also done automatically when registering a network)
 *
 * **********************************************************************/

void
X_init_network_parameters_randomly(int net_id)
{
  int i;

  
  /*
   * sanity check
   */
  if (net_id < 0 || net_id >= X_number_of_networks){
    fprintf(stderr, 
	    "ERROR: Network %d not known. Check the id and registration.\n",
	    net_id);
    return;
  }
  /*
   * initialize
   */

  for (i = 0; i < X_networks[net_id].num_params; i++){
    X_networks[net_id].params[i] = 
      RAND_PLUS_MINUS_ONE() * X_networks[net_id].weights_init_range;
    X_networks[net_id].d_params[i]              = 0.0;
    X_networks[net_id].d_error_d_params[i]      = 0.0;
    X_networks[net_id].prev_d_error_d_params[i] = 0.0;
  }
}


/************************************************************************\
 ************************************************************************
\************************************************************************/



/* **********************************************************************
 *
 * X_variable_set_XXXX(): routines for the specification of certain 
 *                        parameters
 *
 * **********************************************************************/


void
X_variable_set_learning_flag(int variable_id, int mode)
{
  /*
   * sanity check
   */
  
  if (variable_id < 0 || variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    variable_id);
    return;
  }
  if (mode != 0 && mode != 1){
    fprintf(stderr, "ERROR: Learning mode must be 0 (off) or 1 (on).\n");
    return;

  }

  /*
   * do it
   */

  X_variables[variable_id].learning_flag = mode;

}



/* **********************************************************************
 *
 * X_network_clear_pending_updates() clears pending parameter updates
 *              (this is also done automatically when registering a network)
 *
 * **********************************************************************/

void
X_network_clear_pending_updates()
{
  int i, n;

  /*
   * do it
   */

  for (n = 0; n < X_number_of_networks; n++)
    for (i = 0; i < X_networks[n].num_params; i++)
      X_networks[n].d_error_d_params[i]           = 0.0;

}






/* **********************************************************************
 *
 * X_network_set_XXXX(): routines for the specification of certain 
 *                       parameters
 *
 * **********************************************************************/


void
X_network_set_learning_flag(int net_id, int mode)
{
  /*
   * sanity check
   */
  
  if (net_id < 0 || net_id >= X_number_of_networks){
    fprintf(stderr, 
	    "ERROR: Network %d not known. Check the id and registration.\n",
	    net_id);
    return;
  }
  if (mode != 0 && mode != 1){
    fprintf(stderr, "ERROR: Learning mode must be 0 (off) or 1 (on).\n");
    return;

  }

  /*
   * do it
   */

  X_networks[net_id].learning_flag = mode;

}


void
X_network_set_stepsize(int net_id, float step_size)
{
  /*
   * sanity check
   */
  
  if (net_id < 0 || net_id >= X_number_of_networks){
    fprintf(stderr, 
	    "ERROR: Network %d not known. Check the id and registration.\n",
	    net_id);
    return;
  }
  if (step_size < 0.0 || step_size > 1.0){
    fprintf(stderr, "ERROR: A network's step size must be in [0,1].\n");
    return;

  }

  /*
   * do it
   */

  X_networks[net_id].step_size = step_size;
}




void
X_network_set_momentum(int net_id, float momentum)
{
  /*
   * sanity check
   */
  
  if (net_id < 0 || net_id >= X_number_of_networks){
    fprintf(stderr, 
	    "ERROR: Network %d not known. Check the id and registration.\n",
	    net_id);
    return;
  }
  if (momentum < 0.0 || momentum > 1.0){
    fprintf(stderr, "ERROR: A network's momentum must be in [0,1].\n");
    return;

  }

  /*
   * do it
   */

  X_networks[net_id].momentum = momentum;
}




void
X_network_set_weights_init_range(int net_id, float weights_init_range)
{
  /*
   * sanity check
   */
  
  if (net_id < 0 || net_id >= X_number_of_networks){
    fprintf(stderr, 
	    "ERROR: Network %d not known. Check the id and registration.\n",
	    net_id);
    return;
  }
  if (weights_init_range < 0.0){
    fprintf(stderr, "ERROR: A network's weights_init_range must be >= 0.\n");
    return;

  }

  /*
   * do it
   */

  X_networks[net_id].weights_init_range = weights_init_range;
}


/* ---------------------------------------------------------------------- */







/* **********************************************************************
 *
 * X_initialize_training_epoch(): Initializes error + cumul variables
 *
 * **********************************************************************/

void
X_initialize_training_epoch()
{
  int i, j, k;
  float factor, max;
  int verbose = 0;

  /*
   * Initialize all errors for all variables
   */

  for (i = 0; i < X_number_of_variables; i++)
    for (j = 0; j < NUM_SET_TYPES; j++)
      X_variables[i].error[j]      = -1.0;
  
  /*
   * Initialize all global errors
   */

  for (j = 0; j < NUM_SET_TYPES; j++)
    X_error_during_training[j] = 0.0;

  /*
   * Compute average activation + gradient, and (re-)initialize cumul table
   */

  for (i = 0; i < X_number_of_variables; i++){
    if (X_variables[i].cumul_divisor > 0 && /* only then something happened */
	X_variables[i].learning_flag){ /* only then do we want to change it */

      /* 1. check the sanity of the programmer */
      if (X_variables[i].type != STOCHASTIC){
	fprintf(stderr, "Surprising error in X_initialize_training_epoch()\n");
	exit(-1);
      }

      /* 2. remove previous gradients in avg */
      for (j = 0; j < X_variables[i].num_avg_gradients; j++){
	X_variables[i].avg_gradients[j].network_id = -1;
	free(X_variables[i].avg_gradients[j].value);
	X_variables[i].avg_gradients[j].value = NULL;
      }
      X_variables[i].num_avg_gradients = 0; /* no effect, but safer */

      /* 3. compute the new average */
      factor = 1.0 / ((float) X_variables[i].cumul_divisor);
      for (j = 0; j < X_variables[i].num_values; j++)
	X_variables[i].avg_value[j] = 
	  factor * X_variables[i].cumul_value[j];

      /* 4. compute the new gradients (by multiplication) */
      for (j = 0; j < X_variables[i].num_cumul_gradients; j++)
	for (k = 0; k < X_variables[i].num_values *
	       X_networks[X_variables[i].cumul_gradients[j].network_id].
	       num_params; k++)
	  X_variables[i].cumul_gradients[j].value[k] *= factor;

      /* 5. copy gradients from cumul to avg */
      for (j = 0; j < X_variables[i].num_cumul_gradients; j++){
	X_variables[i].avg_gradients[j].network_id = 
	  X_variables[i].cumul_gradients[j].network_id;
	X_variables[i].avg_gradients[j].value = 
	  X_variables[i].cumul_gradients[j].value;
      }
      X_variables[i].num_avg_gradients   = X_variables[i].num_cumul_gradients;

      /* 6. Clear cumulative variables */
      for (j = 0; j < X_variables[i].num_cumul_gradients; j++){
	X_variables[i].cumul_gradients[j].network_id = -1;
	X_variables[i].cumul_gradients[j].value = NULL;
      }
      for (j = 0; j < X_variables[i].num_values; j++)
	X_variables[i].cumul_value[j] = 0.0;
      X_variables[i].num_cumul_gradients = 0;
      
      /* 8. and clear the normalized values */
      for (j = 0; j < X_variables[i].num_values; j++)
	X_variables[i].norm_value[j] = 0.0;
      
      /* 9. set flag indicating this variable needs saved (P'burgh accent!) */
      X_variables[i].save_flag = 1;
  
      /* 10. Print something */
      if (verbose)
	fprintf(stderr, "Updated average for %s (%d)\n", X_variables[i].name,
		X_variables[i].cumul_divisor);

      /* 11. Reset divisor */
      X_variables[i].cumul_divisor       = 0;

      /* 12. Display all */
      X_display_variable(i);

    }
  }

}



/* **********************************************************************
 *
 * X_compute_neuro_net(): internal routine that propagates values and
 *                        gradients through a neural network. Basis for
 *                        backpropagation. Cryptic, but well-tested.
 *
 * **********************************************************************/

void
X_compute_neuro_net(int net_id)
{
  int   i, j, k, l, m, n, err;
  network_type  *network = NULL;
  int  *counter = NULL;
  int  *pseudo_input_index = NULL;
  int   done, increment_done, index_hidden_unit;
  float difference, help, factor;
  int   verbose = 0;


  int            type;
  int            num_inputs;
  int            num_pseudo_inputs;
  int            num_hidden;
  int            num_outputs;
  int            num_pseudo_inputs_per_input;
  int            learning;
  int            *num_bases_per_input       = NULL;
  int            num_params;
  float          *params                    = NULL;
  float          *input_to_hidden_weights   = NULL;
  float          *hidden_biases             = NULL;
  float          *hidden_to_output_weights  = NULL;
  float          *output_biases             = NULL;
  float          *input_to_output_weights   = NULL;
  float          *input                     = NULL;
  float          *pseudo_input              = NULL;
  float          *netin_hidden              = NULL;
  float          *hidden                    = NULL;
  float          *netin_output              = NULL;
  float          *output                    = NULL;
  float          *d_output_d_params         = NULL;
  float          *d_hidden_d_input          = NULL;
  float          *d_output_d_input          = NULL;
  float          *d_error_d_params          = NULL;


  /*
   * sanity checks
   */
  
  if (net_id < 0 || net_id >= X_number_of_networks){
    fprintf(stderr, 
	    "ERROR: Network %d not known. Check the id and registration.\n",
	    net_id);
    return;
  }


  /*
   * set pointers and values
   */

  network = &(X_networks[net_id]);
  type                             = network->type;
  num_inputs                       = network->num_inputs;
  num_pseudo_inputs                = network->num_pseudo_inputs;
  num_hidden                       = network->num_hidden;
  num_outputs                      = network->num_outputs;
  num_pseudo_inputs_per_input      = network->num_pseudo_inputs_per_input;
  num_bases_per_input              = network->num_bases_per_input;
  num_params                       = network->num_params;
  learning                         = network->learning_flag;
  params                           = network->params;
  input_to_hidden_weights          = network->input_to_hidden_weights;
  hidden_biases                    = network->hidden_biases;
  hidden_to_output_weights         = network->hidden_to_output_weights;
  output_biases                    = network->output_biases;
  input_to_output_weights          = network->input_to_output_weights;
  input                            = network->input;
  pseudo_input                     = network->pseudo_input;
  netin_hidden                     = network->netin_hidden;
  hidden                           = network->hidden;
  netin_output                     = network->netin_output;
  output                           = network->output;
  d_output_d_params                = network->d_output_d_params;
  d_hidden_d_input                 = network->d_hidden_d_input;
  d_output_d_input                 = network->d_output_d_input;
  d_error_d_params                 = network->d_error_d_params;


  /*************************************************************
   *
   *
   * (1)   LOGISTIC NEURAL NETWORK (BACK-PROGAPATION STYLE)
   *
   *
   *************************************************************/

  if (type == LOGISTIC_NETWORK){
    
    /*
     * thermometer encoding, with plenty sanity checking
     */
    
    pseudo_input_index = (int *) malloc(sizeof(int) * num_inputs);
    if (!pseudo_input_index){
      fprintf(stderr, "ERROR: out of memory in X_compute_neuro_net.\n");
      exit(-1);
    }
    
    for (i = 0, k = 0; i < num_inputs; i++){
      if (input[i] == 1.0)
	pseudo_input_index[i] = num_pseudo_inputs_per_input - 1;
      else
	pseudo_input_index[i] = 
	  ((int) (input[i] * ((float) num_pseudo_inputs_per_input)));
      if (pseudo_input_index[i] < 0 || /* sanity check */
	  pseudo_input_index[i] >= num_pseudo_inputs_per_input){
	fprintf(stderr, "ERROR: thermometer 1: %f -> %d (%d)\n",
		input[i], pseudo_input_index[i], num_pseudo_inputs_per_input);
	exit(-1);
      }
      for (j = 0; j < num_pseudo_inputs_per_input; j++, k++){
	if (j < pseudo_input_index[i])
	  pseudo_input[k] = 1.0;
	else if (j > pseudo_input_index[i])
	  pseudo_input[k] = 0.0;
	else
	  pseudo_input[k] = 
	    (input[i] * ((float) num_pseudo_inputs_per_input))
	    - ((float) j);
	if (pseudo_input[k] < 0.0 || pseudo_input[k] > 1.0){ /* sane? */
	  fprintf(stderr, "ERROR: thermometer 2: %f -> %d %f\n",
		  input[i], pseudo_input_index[i], pseudo_input[k]);
	  exit(-1);
	}
      }
    }
    if (k != num_pseudo_inputs){ /* sanity check */
      fprintf(stderr, "ERROR: thermometer : %d != %d\n",
	      k, num_pseudo_inputs);
      exit(-1);
    }

    /*
      for (i = 0; i < num_inputs; i++)
      fprintf(stderr, " %g", input[i]);
      fprintf(stderr, " -> (");
      for (i = 0; i < num_inputs; i++)
      fprintf(stderr, " %d", pseudo_input_index[i]);
      fprintf(stderr, ") ->");
      for (i = 0; i < num_pseudo_inputs; i++)
      fprintf(stderr, " %g", pseudo_input[i]);
      fprintf(stderr, "\n");
      */

    /* ================================================================= *\
     *            case 1; network with hidden layer
    \* ================================================================= */

    if (num_hidden > 0){
      
      /*
       * forward propagation of values
       */
      
      for (i = 0, k = 0; i < num_hidden; i++){
	netin_hidden[i] = hidden_biases[i];
	for (j = 0; j < num_pseudo_inputs; j++, k++)
	  netin_hidden[i] += 
	    pseudo_input[j] * input_to_hidden_weights[k];
	hidden[i] = X_logistic(netin_hidden[i]);
      }
      
      for (i = 0, k = 0; i < num_outputs; i++){
	netin_output[i] = output_biases[i];
	for (j = 0; j < num_hidden; j++, k++)
	  netin_output[i] += 
	    hidden[j] * hidden_to_output_weights[k];
	output[i] = X_logistic(netin_output[i]);
      }
      
      if (learning && X_calculate_gradients){
	
	/*
	 * forward propagation of input-hidden/output derivatives
	 */
	
	
	/* derivative of thermomter encoding */
	for (i = 0, k = 0; i < num_hidden; i++)
	  for (j = 0; j < num_inputs; j++, k++){
	    /* l = index of "active" thermomter unit */
	    l = (j * num_pseudo_inputs_per_input) + pseudo_input_index[j];
	    /*if (l != j) printf("?[%d %d]?  ", l, j); /* remove this one! */
	    /* m = index of the weight of hidden unit to that active unit */
	    m = (i * num_pseudo_inputs) + l;
	    /*if (m != k) printf("?(%d %d)?  ", m, k); /* remove this one! */
	    d_hidden_d_input[k] = 
	      hidden[i] * (1.0 - hidden[i])
	      * input_to_hidden_weights[m] /* weight of active unit */
	      * ((float) num_pseudo_inputs_per_input);
	  }
	
	
	for (i = 0, k = 0; i < num_outputs; i++)
	  for (j = 0; j < num_inputs; j++, k++)
	    d_output_d_input[k] = 0.0;
	
	for (i = 0, k = 0; i < num_outputs; i++)
	  for (j = 0, n = 0; j < num_hidden; j++, k++)
	    for (l = 0; l < num_inputs; l++, n++){
	      m = i * num_inputs + l;
	      d_output_d_input[m] +=
		(hidden_to_output_weights[k] * 
		 d_hidden_d_input[n]);
	    }
	
	for (i = 0, k = 0; i < num_outputs; i++)
	  for (j = 0; j < num_inputs; j++, k++)
	    d_output_d_input[k] *=
	      (output[i] * (1.0 - output[i]));
	
	
	/*
	 * Forward Propagation of parameter--hidden/output derivatives
	 */
	
	
	for (i = 0, l = 0, m = 0, n = 0; i < num_outputs; i++){
	  /* input-hidden weights */
	  for (j = 0; j < num_hidden; j++, m++)
	    for (k = 0; k < num_pseudo_inputs; k++, l++)
	      d_output_d_params[l] = 
		output[i] * (1.0 - output[i]) *
		hidden_to_output_weights[m] *
		hidden[j] * (1.0 - hidden[j]) *
		pseudo_input[k];
	  /* hidden biases */
	  for (j = 0; j < num_hidden; j++, l++, n++)
	    d_output_d_params[l] = 
	      output[i] * (1.0 - output[i]) *
	      hidden_to_output_weights[n] *
	      hidden[j] * (1.0 - hidden[j]);
	  /* hidden-output weights */
	  for (j = 0; j < num_outputs; j++)
	    for (k = 0; k < num_hidden; k++, l++)
	      if (i == j)
		d_output_d_params[l] = 
		  output[i] * (1.0 - output[i]) *
		  hidden[k];
	      else
		d_output_d_params[l] = 0.0;
	  /* output biases */
	  for (j = 0; j < num_outputs; j++, l++)
	    if (i == j)
	      d_output_d_params[l] =
		output[i] * (1.0 - output[i]);
	    else
	      d_output_d_params[l] = 0.0;
	}
      }
      
    }
    /* ================================================================= *\
     *            case 2: network without hidden layer
    \* ================================================================= */

    else if (num_hidden == 0){
      
      /*
       * forward propagation of values
       */
      
      for (i = 0, k = 0; i < num_outputs; i++){
	netin_output[i] = output_biases[i];
	for (j = 0; j < num_pseudo_inputs; j++, k++)
	  netin_output[i] += 
	    pseudo_input[j] * input_to_output_weights[k];
	output[i] = X_logistic(netin_output[i]);
      }
      
      
      if (learning && X_calculate_gradients){

	/*
	 * forward propagation of input-output derivatives
	 */
	
	/* derivative of thermomter encoding */
	for (i = 0, k = 0; i < num_outputs; i++)
	  for (j = 0; j < num_inputs; j++, k++){
	    /* l = index of "active" thermomter unit */
	    l = (j * num_pseudo_inputs_per_input) + pseudo_input_index[j];
	    /*if (l != j) printf("?[%d %d]?  ", l, j); /* remove this one! */
	    /* m = index of the weight of outputs unit to that active unit */
	    m = (i * num_pseudo_inputs) + l;
	    /*if (m != k) printf("?(%d %d)?  ", m, k); /* remove this one! */
	    d_output_d_input[k] = 
	      output[i] * (1.0 - output[i])
	      * input_to_output_weights[m] /* weight of active unit */
	      * ((float) num_pseudo_inputs_per_input);
	  }
	
	
	/*
	 * Forward Propagation of parameter--output derivatives
	 */
	
	
	for (i = 0, l = 0, m = 0, n = 0; i < num_outputs; i++){
	  /* input-output weights */
	  for (j = 0; j < num_outputs; j++)
	    for (k = 0; k < num_pseudo_inputs; k++, l++)
	      if (i == j)
		d_output_d_params[l] = 
		  output[i] * (1.0 - output[i]) *
		  pseudo_input[k];
	      else
		d_output_d_params[l] = 0.0;
	  /* output biases */
	  for (j = 0; j < num_outputs; j++, l++)
	    if (i == j)
	      d_output_d_params[l] =
		output[i] * (1.0 - output[i]);
	    else
	      d_output_d_params[l] = 0.0;
	}
      }
    }
    
    free(pseudo_input_index);
    pseudo_input_index = NULL;
  }


  /*************************************************************
   *
   *
   * (2) RADIAL BASIS NETWORK 
   *
   *     (with exponentially many, fixed basis functions)
   *
   *
   *************************************************************/

  else if (type == RADIAL_BASIS_NETWORK){

    counter = (int *) malloc(sizeof(int) * num_inputs);
    if (!counter){
      fprintf(stderr, "ERROR: out of memory in X_compute_neuro_net.\n");
      exit(-1);
    }
    for (i = 0; i < num_inputs; i++)
      counter[i] = 0;
    index_hidden_unit = 0;
    done = 0;

    if (verbose){
      fprintf(stderr, "Input:");
      for (i = 0; i < num_inputs; i++)
	fprintf(stderr, " %8.6f", input[i]);
      fprintf(stderr, "\n");
    }

    /*
     * Compute hidden unit activations and derivatives
     */
    
    do{
      
      /*
       * Response of the current radial basis unit
       */

      difference = 0.0;
      for (i = 0, j = index_hidden_unit * num_inputs;
	   i < num_inputs; i++, j++){
	help = 
	  ((input[i] * (float) (num_bases_per_input[i]-1)) -
	   ((float) counter[i]));
	difference += (help * help);
	if (learning && X_calculate_gradients)
	  d_hidden_d_input[j] =
	    -2.0 * ((input[i] * (float) (num_bases_per_input[i] - 1)));
      }
      
      hidden[index_hidden_unit] = exp(-difference);

      if (learning && X_calculate_gradients){
	for (i = 0, j = index_hidden_unit * num_inputs;
	     i < num_inputs; i++, j++)
	  d_hidden_d_input[j] *= hidden[index_hidden_unit];
      }
      

      /*
       * Print out the counters - just for testing purposes
       */
      
      if (verbose){
	for (i = 0; i < num_inputs; i++)
	  fprintf(stderr, " %.2d", counter[i]);
	fprintf(stderr, "    index=%d  diff=%8.6f activation=%8.6f\n",
		index_hidden_unit, difference, 
		hidden[index_hidden_unit]);
      }
      
      /*
       * increment counter and index
       */
      
      for (i = 0, increment_done = 0; 
	 i < num_inputs && !increment_done && !done; i++){
	counter[i]++;
	if (counter[i] < num_bases_per_input[i])
	  increment_done = 1;
	else
	  counter[i] = 0;
      }
      if (!increment_done)
	done = 1;		/* overflow, we made it through once */
      
      index_hidden_unit++;
      
    } while (!done);

    free(counter);

    
    /*
     * Compute output activations and derivatives
     */
    
    for (i = 0, k = 0; i < num_outputs; i++){
      netin_output[i] = output_biases[i];
      for (j = 0; j < num_hidden; j++, k++)
	netin_output[i] += 
	  hidden[j] * hidden_to_output_weights[k];
      output[i] = X_logistic(netin_output[i]);
    }
    
    if (learning && X_calculate_gradients){
    
      for (i = 0, k = 0; i < num_outputs; i++)
	for (j = 0; j < num_inputs; j++, k++)
	  d_output_d_input[k] = 0.0;
    
      for (i = 0, k = 0; i < num_outputs; i++)
	for (j = 0, n = 0; j < num_hidden; j++, k++)
	  for (l = 0; l < num_inputs; l++, n++){
	    m = i * num_inputs + l;
	    d_output_d_input[m] +=
	      (hidden_to_output_weights[k] * 
	       d_hidden_d_input[n]);
	  }
    
      for (i = 0, k = 0; i < num_outputs; i++)
	for (j = 0; j < num_inputs; j++, k++)
	  d_output_d_input[k] *=
	    (output[i] * (1.0 - output[i]));
    


      for (i = 0, l = 0; i < num_outputs; i++){
	/* hidden-output weights */
	for (j = 0; j < num_outputs; j++)
	  for (k = 0; k < num_hidden; k++, l++)
	    if (i == j)
	      d_output_d_params[l] = 
		output[i] * (1.0 - output[i]) *
		hidden[k];
	    else
	      d_output_d_params[l] = 0.0;
	/* output biases */
	for (j = 0; j < num_outputs; j++, l++)
	  if (i == j)
	    d_output_d_params[l] =
	      output[i] * (1.0 - output[i]);
	  else
	    d_output_d_params[l] = 0.0;
      }
    }
  }


  else{
    fprintf(stderr, "What on earth is going on here? Unknown network type.\n");
    exit(-1);
  }
}









/* **********************************************************************
 *
 * float X_logistic(): internal routine, that realizes a logistic function
 * 
 *
 * **********************************************************************/

float 
X_logistic(float x)
{
  int index;
  int i;
  float xx;
  static int X_logistic_initialized = 0;
  static float table_logistic[32000];
  

  return (1.0 / (1.0 + exp(((-1.0) * x))));
  
  /* don't use that table-lookup version (which used to be here), it
   * doesn't work in this context, the error will occasionally explode
   * since the networks often operate very close to 0 or 1 */
}







/* **********************************************************************
 *
 * void X_chain_gradients(): Internal routine for matrix
 *                           multiplication memory must already be
 *                           allocated
 *
 * **********************************************************************/

void
X_chain_gradients(float *grad_in1, /* e.g., the gradients wrt. to input 
				    * dimension: dim1 x dim2 */
		  float *grad_in2, /* e.g., the output-input gradients
				    * dimension: dim2 x dim3 */
		  float *grad_out, /* the result, grad1 x grad2 (product)
				    * dimenion: dim1 x dim3 
				    * memory must already be allocated */
		  int dim1, int dim2, int dim3)
{
  int i, j, k, l, m, n;
  
  for (i = 0, l = 0; i < dim3; i++)
    for (j = 0; j < dim1; j++, l++){
      grad_out[l] = 0.0;
      for (k = 0; k < dim2; k++){
	m = k * dim1 + j;	/* index for grad_in1 */
	n = i * dim2 + k;	/* index for grad_in2 */
	grad_out[l] += (grad_in1[m] * grad_in2[n]);
      }  
    }
}

/************************************************************************\
 ************************************************************************
\************************************************************************/




/* **********************************************************************
 *
 * X_extract_partial_matrix(): Internal routine for the extraction of
 *                             a smaller from a larger matrix
 *
 * **********************************************************************/

void
X_extract_partial_matrix(float *matrix_in, /* from */
			 float *matrix_out, /* to */
			 int matrix_in_dim_outer, /* from-dim outer */
			 int matrix_in_dim_inner, /* from-dim inner */
			 int matrix_out_dim_outer, /* to-dim outer */
			 int matrix_out_dim_inner, /* to-dim inner */
			 int offset_outer, /* offset outer */
			 int offset_inner) /* offset inner */
{
  int i, j, k, l, m, n;

  /*
   * Sanity check
   */

  if (matrix_in_dim_outer  <= 0 || matrix_in_dim_inner  <= 0 ||
      matrix_out_dim_outer <= 0 || matrix_out_dim_inner <= 0 ||
      offset_outer < 0 || offset_inner < 0 ||
      matrix_out_dim_outer + offset_outer > matrix_in_dim_outer ||
      matrix_out_dim_inner + offset_inner > matrix_in_dim_inner){
    fprintf(stderr, "ERROR: Invalid parameters in X_extract_partial_matrix");
    fprintf(stderr, " %d %d %d %d %d %d\n", 
	    matrix_in_dim_outer, matrix_in_dim_inner,
	    matrix_out_dim_outer, matrix_out_dim_inner,
	    offset_outer, offset_inner);
    return;
  }

  /*
   * extract
   */

  for (i = 0, k = 0; i < matrix_out_dim_outer; i++)
    for (j = 0; j < matrix_out_dim_inner; j++, k++){
      l = ((i + offset_outer) * matrix_in_dim_inner) + (j + offset_inner);
      matrix_out[k] = matrix_in[l];
    }
}


/************************************************************************\
 ************************************************************************
\************************************************************************/




/* **********************************************************************
 *
 * X_print_neuro_net(): prints parameters and activations/derivatives
 *                      of a network
 *
 * **********************************************************************/

void
X_print_neuro_net(int net_id)
{
  int i, j, k, l, m, n;
  network_type  *network = NULL;

  /*
   * sanity checks
   */
  
  if (net_id < 0 || net_id >= X_number_of_networks){
    fprintf(stderr, 
	    "ERROR: Network %d not known. Check the id and registration.\n",
	    net_id);
    return;
  }

  network = &(X_networks[net_id]);


  
  /*
   * printing
   */

  if (network->num_hidden > 0){



    fprintf(stderr, "--------------------------------------------------\n");
    fprintf(stderr, "******************** %s ********************\n",
	    network->name);
    fprintf(stderr, "NETWORK PARAMETERS\n");
    for (i = 0, k = 0; i < network->num_hidden; i++){
      fprintf(stderr, "Hidden unit %d:", i);
      for (j = 0; j < network->num_inputs; j++, k++)
	fprintf(stderr, " %8.6f", network->input_to_hidden_weights[k]);
      fprintf(stderr, " / %8.6f\n", network->hidden_biases[i]);
    }
    for (i = 0, k = 0; i < network->num_outputs; i++){
      fprintf(stderr, "Output unit %d:", i);
      for (j = 0; j < network->num_hidden; j++, k++)
	fprintf(stderr, " %8.6f", network->hidden_to_output_weights[k]);
      fprintf(stderr, " / %8.6f\n", network->output_biases[i]);
    }
    fprintf(stderr, "ACTIVATIONS\n");
    fprintf(stderr, "Input:");
    for (i = 0; i < network->num_inputs; i++)
      fprintf(stderr, " %8.6f", network->input[i]);
    fprintf(stderr, "\nHidden:");
    for (i = 0; i < network->num_hidden; i++)
      fprintf(stderr, " %8.6f->%8.6f", 
	      network->netin_hidden[i], network->hidden[i]);
    fprintf(stderr, "\nOutput:");
    for (i = 0; i < network->num_outputs; i++)
      fprintf(stderr, " %8.6f->%8.6f", 
	      network->netin_output[i], network->output[i]);
    fprintf(stderr, "\n");
    fprintf(stderr, "INPUT DERIVATIVES\n");
    for (i = 0, k = 0; i < network->num_hidden; i++){
      fprintf(stderr, "Hidden unit %d:", i);
      for (j = 0; j < network->num_inputs; j++, k++)
	fprintf(stderr, " %8.6f", network->d_hidden_d_input[k]);
      fprintf(stderr, "\n");
    }
    for (i = 0, k = 0; i < network->num_outputs; i++){
      fprintf(stderr, "Output unit %d:", i);
      for (j = 0; j < network->num_inputs; j++, k++)
	fprintf(stderr, " %8.6f", network->d_output_d_input[k]);
      fprintf(stderr, "\n");
    }

    fprintf(stderr, "OUTPUT-PARAMETER DERIVATIVES\n");
    for (i = 0, k = 0; i < network->num_outputs; i++){
      fprintf(stderr, "Output unit %d:", i);
      for (j = 0; j < network->num_params; j++, k++)
	fprintf(stderr, " %8.6f", network->d_output_d_params[k]);
      fprintf(stderr, "\n");
    }

    fprintf(stderr, "--------------------------------------------------\n");
  }


  else{				/* no hidden units */

    fprintf(stderr, "--------------------------------------------------\n");
    fprintf(stderr, "******************** %s ********************\n",
	    network->name);
    fprintf(stderr, "NETWORK PARAMETERS\n");
    for (i = 0, k = 0; i < network->num_outputs; i++){
      fprintf(stderr, "Output unit %d:", i);
      for (j = 0; j < network->num_inputs; j++, k++)
	fprintf(stderr, " %8.6f", network->input_to_output_weights[k]);
      fprintf(stderr, " / %8.6f\n", network->output_biases[i]);
    }
    fprintf(stderr, "ACTIVATIONS\n");
    fprintf(stderr, "Input:");
    for (i = 0; i < network->num_inputs; i++)
      fprintf(stderr, " %8.6f", network->input[i]);
    fprintf(stderr, "\nOutput:");
    for (i = 0; i < network->num_outputs; i++)
      fprintf(stderr, " %8.6f->%8.6f", 
	      network->netin_output[i], network->output[i]);
    fprintf(stderr, "\n");
    fprintf(stderr, "INPUT DERIVATIVES\n");
    for (i = 0, k = 0; i < network->num_outputs; i++){
      fprintf(stderr, "Output unit %d:", i);
      for (j = 0; j < network->num_inputs; j++, k++)
	fprintf(stderr, " %8.6f", network->d_output_d_input[k]);
      fprintf(stderr, "\n");
    }

    fprintf(stderr, "OUTPUT-PARAMETER DERIVATIVES\n");
    for (i = 0, k = 0; i < network->num_outputs; i++){
      fprintf(stderr, "Output unit %d:", i);
      for (j = 0; j < network->num_params; j++, k++)
	fprintf(stderr, " %8.6f", network->d_output_d_params[k]);
      fprintf(stderr, "\n");
    }

    fprintf(stderr, "--------------------------------------------------\n");
  }

}


/* ---------------------------------------------------------------------- */





/* **********************************************************************
 *
 * X_neuro_net(): Routine for running a neural network
 * 
 *
 * **********************************************************************/

void
X_neuro_net(int net_id, int variable_id_from, int variable_id_to)
{
  int    i, j, k, l, m;
  int    index_stochastic_input, index_stochastic_output;
  int    done, increment_done;
  int   *counter;
  int    verbose = 0;
  float  max, factor;
  float *new_gradients = NULL;
  float *new_gradients1 = NULL;
  float *new_gradients2 = NULL;
  int   X_auxiliary_variable = -1;
  variable_type *from_variable = NULL;
  variable_type *to_variable   = NULL;
  variable_type *aux_variable  = NULL;
  network_type  *network = NULL;


  /*
   * sanity checks
   */
  
  if (net_id < 0 || net_id >= X_number_of_networks){
    fprintf(stderr, 
	    "ERROR: Network %d not known. Check the id and registration.\n",
	    net_id);
    return;
  }
  
  if (variable_id_from < 0 || variable_id_from >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    variable_id_from);
    return;
  }

  if (variable_id_to < 0 || variable_id_to >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    variable_id_to);
    return;
  }

  if (X_variables[variable_id_from].type != 
      X_networks[net_id].from_variable_type){
    fprintf(stderr, "ERROR: Type mismatch network %s, from-variable.\n",
	    X_networks[net_id].name);
    return;
  }

  if (X_variables[variable_id_from].dimension != 
      X_networks[net_id].from_variable_dimension){
    fprintf(stderr, "ERROR: Dimension mismatch network %s, from-variable.\n",
	    X_networks[net_id].name);
    return;
  }

  if (X_variables[variable_id_to].type != X_networks[net_id].to_variable_type){
    fprintf(stderr, "ERROR: Type mismatch network %s, to-variable.\n",
	    X_networks[net_id].name);
    return;
  }

  if (X_variables[variable_id_to].dimension != 
      X_networks[net_id].to_variable_dimension){
    fprintf(stderr, "ERROR: Dimension mismatch network %s, to-variable.\n",
	    X_networks[net_id].name);
    return;
  }


  if (X_number_of_variables >= MAX_NUMBER_VARIABLES){
    fprintf(stderr, 
	    "WARNING: Cannot allocate another aux variable in X_neuro_net.\n");
    exit(-1);
  }

  /*
   * allocate and initialize the various variables involved
   */

  X_auxiliary_variable = X_number_of_variables;
  X_variables[X_number_of_variables].save_flag = 0;
  X_number_of_variables++;

  network       = &(X_networks[net_id]);
  from_variable = &(X_variables[variable_id_from]);
  to_variable   = &(X_variables[variable_id_to]);
  aux_variable  = &(X_variables[X_auxiliary_variable]);

  counter   = (int *) malloc(sizeof(int) * network->num_inputs);

  if (!counter){
    fprintf(stderr, "ERROR: out of memory in X_neuro_net.\n");
    exit(-1);
  }

  for (i = 0; i < network->num_inputs; i++)
    counter[i] = 0;
  index_stochastic_input = index_stochastic_output = 0;
  factor = 1.0;
  done = 0;


  /*
   * copy some key numbers from the to_variable into aux_variable
   * and allocate memory just like in the target variable,
   * since we'll first pipe the result to the aux variable, then
   * copy it into the to_variable
   */
  
  aux_variable->dimension  = to_variable->dimension;
  aux_variable->type       = to_variable->type;
  aux_variable->resolution = to_variable->resolution;
  aux_variable->num_values = to_variable->num_values;

  aux_variable->value       = malloc(sizeof(float) * to_variable->num_values);

  if (aux_variable->value == NULL){ /* sufficient memory left? */
    fprintf(stderr, "ERROR: out of memory in X_neuro_net.\n");
    exit(-1);
  }
  if (aux_variable->num_gradients != 0){
    fprintf(stderr, "STRANGE: Memory leak somewhere in your code.\n");
  }

  /*
   * determine sample intervals / number of iterations per input unit
   */
  
  
   /* input units, deterministic variable */
  for (i = 0, j = 0; j < from_variable->dimension && 
	 from_variable->type == DETERMINISTIC; j++, i++)
    network->input_resolution[i] = 1;
  /* input units, stochastic variable */
  for (j = 0; j < from_variable->dimension &&
	 from_variable->type == STOCHASTIC; j++, i++)
    network->input_resolution[i] = from_variable->resolution;
  /* output units, stochastic variable */
  for (j = 0; j < aux_variable->dimension &&
	  aux_variable->type == STOCHASTIC; j++, i++)
    network->input_resolution[i] = aux_variable->resolution;
  
  
  
  /*
   * set those inputs that don't change
   */
  
  for (i = 0, j = 0; i < network->num_inputs; i++)
    if (network->input_info[i] == 
	NETWORK_UNIT_IS_DETERMINISTIC_INPUT)
      network->input[i] = from_variable->value[j++];


  /*
   * clear the output variable and its gradients
   */

  for (i = 0; i < aux_variable->num_values; i++)
    aux_variable->value[i] = 0.0;
  X_clear_all_gradients(X_auxiliary_variable, 0);

  /*
   * ******************** Run the networks ********************
   */

  do{

    /*
     * set those input variables that change in this loop
     */
    
    for (i = 0, j = 0; i < network->num_inputs; i++)
      if (network->input_info[i] == 
	NETWORK_UNIT_IS_STOCHASTIC_INPUT ||
	  network->input_info[i] == 
	NETWORK_UNIT_IS_STOCHASTIC_OUTPUT)
	network->input[i] = ((float) counter[i]) / 
	  ((float) (network->input_resolution[i] - 1));
    
    /*
     * run the network
     */

    X_compute_neuro_net(net_id);


    /*
     * special treatment: stochastic input variable. Here we
     * weigh the output according to the probability in the input
     * variable
     */

    if (from_variable->type == STOCHASTIC)
      factor = from_variable->value[index_stochastic_input];
    else
      factor = 1.0;

    /*
     * special treatment: deterministic output variable.
     * Here we'll just copy the network's output over
     */

    if (aux_variable->type == DETERMINISTIC){
      /* comput output value */
      for (i = 0; i < network->num_outputs; i++)
	aux_variable->value[i] += (factor * network->output[i]);
      if (network->learning_flag && X_calculate_gradients){
	/* and the gradients wrt to the current network */
	X_add_gradients(X_auxiliary_variable, 
			net_id, 
			0,	/* 0=add regular gradients, 
				 * not "cumulative mode" */
			network->d_output_d_params, 
			factor, -1);
	/* convert gradients of previous computations accordingly */
	for (i = 0; i < from_variable->num_gradients; i++){
	  /* a consistency check */
	  if (from_variable->dimension != network->num_inputs ||
	      network->num_outputs != aux_variable->num_values){
	    fprintf(stderr, "Arrgh %d %d  %d %d!\n", 
		    from_variable->num_values, network->num_inputs,
		    network->num_outputs, aux_variable->num_values);
	    exit(-1);
	  }
	  /* allocate memory for the new gradients */
	  new_gradients = 
	    (float *) 
	    malloc(sizeof(float) *
		   X_networks[from_variable->gradients[i].network_id].
		   num_params
		   * network->num_outputs);
	  if (!new_gradients){
	    fprintf(stderr, "ERROR: out of memory in X_neuro_net.\n");
	    exit(-1);
	  }
	  /* chain back gradients of other networks */
	  if (from_variable->type == DETERMINISTIC) /* deterministic input */
	    X_chain_gradients(from_variable->gradients[i].value,
			      network->d_output_d_input,
			      new_gradients,
			      X_networks[from_variable->gradients[i].
					network_id].
			      num_params,
			      from_variable->num_values,
			      network->num_outputs);
	  else{			/* stochastic input variable */
	    for (j = 0, l = 0; j < network->num_outputs; j++)
	      for (k = 0; k < X_networks[from_variable->gradients[i].
					network_id].num_params; k++, l++){
		m = index_stochastic_input *
		  X_networks[from_variable->gradients[i].network_id].num_params
		  + k;
		new_gradients[l] = network->output[j] * 
		  from_variable->gradients[i].value[m];
	      }
	  }
	  X_add_gradients(X_auxiliary_variable, 
			  from_variable->gradients[i].network_id, 
			  0,	/* 0=add regular gradients, 
				 * not "cumulative mode" */
			  new_gradients, 
			  1.0, -1);
	  free(new_gradients);
	  new_gradients = NULL;
	}
      }
    }
    
    /*
     * special treatment: stochastic output variable.
     * Here the network generates a probability, and there is only
     * one output
     */
    
    else if (aux_variable->type == STOCHASTIC){
      if (network->num_outputs != 1){
	fprintf(stderr, "Bug in X_neuro_net(), number of outputs.\n");
	exit(-1);
      }
      aux_variable->value[index_stochastic_output] += 
	(factor * network->output[0]);
      if (network->learning_flag && X_calculate_gradients){
	X_add_gradients(X_auxiliary_variable, 
			net_id, 
			0,	/* 0=add regular gradients, 
				 * not "cumulative mode" */
			network->d_output_d_params, 
			factor, 
			index_stochastic_output);
	/* convert gradients of previous computations accordingly */
	for (i = 0; i < from_variable->num_gradients; i++){
	  new_gradients1 = 
	    (float *) 
	    malloc(sizeof(float) *
		   X_networks[from_variable->gradients[i].network_id].
		   num_params
		   * from_variable->num_values);
	  new_gradients2 = 
	    (float *) 
	    malloc(sizeof(float) *
		   X_networks[from_variable->gradients[i].network_id].
		   num_params);
	  if (!new_gradients1 || !new_gradients2){
	    fprintf(stderr, "ERROR: out of memory in X_neuro_net.\n");
	    exit(-1);
	  }
	  if (from_variable->type == DETERMINISTIC){ /* deterministic input */
	    X_extract_partial_matrix(network->d_output_d_input, /* from */
				     new_gradients1, /* to */
				     network->num_outputs, /*from-dim (outer)*/
				     network->num_inputs, /* from-dim (inner)*/
				     network->num_outputs, /* to-dim (outer) */
				     from_variable->dimension, /*to-dim(innr)*/
				     0, /* offset (outer) */
				     0); /* offset (inner) */
	    X_chain_gradients(from_variable->gradients[i].value,
			      new_gradients1,
			      new_gradients2,
			      X_networks[from_variable->gradients[i].
					network_id].
			      num_params,
			      from_variable->num_values,
			      network->num_outputs);
	  }
	  else{			/* stochastic input variable */
	    for (k = 0; k < X_networks[from_variable->gradients[i].
				      network_id].num_params; k++){
	      m = index_stochastic_input *
		X_networks[from_variable->gradients[i].network_id].num_params
		+ k;
	      new_gradients2[k] = (network->output[0] * 
				   from_variable->gradients[i].value[m]);
	    }
	  }

	  X_add_gradients(X_auxiliary_variable, 
			  from_variable->gradients[i].network_id, 
			  0,	/* 0=add regular gradients, 
				 * not "cumulative mode" */
			  new_gradients2, 
			  1.0, index_stochastic_output);
	  free(new_gradients2);
	  free(new_gradients1);
	  new_gradients2 = NULL;
	  new_gradients1 = NULL;
	}
      }
    }

    /*
     * Print out the counters - just for testing purposes
     */

    if (verbose){
      for (i = 0; i < network->num_inputs; i++)
	fprintf(stderr, " %.2d", counter[i]);
      fprintf(stderr, "    %d %d\n", 
	      index_stochastic_input, index_stochastic_output);
      for (i = 0; i < network->num_inputs; i++)
	fprintf(stderr, " %10.8e", network->input[i]);
      fprintf(stderr, " -> ");
      for (i = 0; i < network->num_outputs; i++)
	fprintf(stderr, " %10.8e", network->output[i]);
      fprintf(stderr, "\n\n");
    }

    /*
     * increment counter and index{1,2}
     */


    
    for (i = 0, increment_done = 0; 
	 i < network->num_inputs && !increment_done && !done; i++){
      counter[i]++;
      if (counter[i] < network->input_resolution[i])
	increment_done = 1;
      else
	counter[i] = 0;
    }
    if (!increment_done)
      done = 1;			/* overflow, we made it through once */

    if (network->input_info[i-1] == NETWORK_UNIT_IS_STOCHASTIC_INPUT)
      index_stochastic_input++;
    else if (network->input_info[i-1] ==
	     NETWORK_UNIT_IS_STOCHASTIC_OUTPUT){
      index_stochastic_input = 0;
      index_stochastic_output++;
    }

  } while (!done);


  /*
   * stochastic output variables must be scaled to [0,1]
   */


  if (aux_variable->type == STOCHASTIC)
    X_normalize_variable(X_auxiliary_variable);


  /*
   * Now copy the auxiliary values back to the target variable.
   * Here we can do some fast moving of pointers.
   */

  for (i = 0; i < aux_variable->num_values; i++)
    to_variable->value[i] = aux_variable->value[i];
  free(aux_variable->value);

  X_clear_all_gradients(variable_id_to, 0);
  to_variable->num_gradients = aux_variable->num_gradients;
  for (i = 0; i < MAX_NUMBER_NETWORKS; i++){
    to_variable->gradients[i].network_id = 
      aux_variable->gradients[i].network_id;
    aux_variable->gradients[i].network_id = -1;
    to_variable->gradients[i].value = 
      aux_variable->gradients[i].value;
    aux_variable->gradients[i].value = NULL;
  }
  aux_variable->num_gradients = 0;

  

  /*
   * If the output is a stochastic variable, we will now have to
   * rescale the display
   */

  if (to_variable->type == STOCHASTIC &&
      to_variable->display != -1 &&
      global_graphics_initialized){
    max = 0.0;
    for (i = 0; i < to_variable->num_values; i++)
      if (max < to_variable->value[i])
	max = to_variable->value[i];
    G_matrix_set_display_range(to_variable->display, 0.0, max);
  }


  /*
   * display result
   */

  if (to_variable->display != -1 &&
      global_graphics_initialized &&
      X_display)
    G_display_matrix(to_variable->display);

  
  /*
   * free memory
   */

  free(counter);
  X_number_of_variables--;
}





/************************************************************************\
 ************************************************************************
\************************************************************************/




/* **********************************************************************
 *
 * X_set_target: trains the various networks so that the first variable
 *               comes closer to the second
 *
 * **********************************************************************/


void
X_set_target(int output_variable_id, int target_variable_id)
{
  int i, j, k, l;
  network_type  *network = NULL;  
  variable_type *output_variable = NULL;
  variable_type *target_variable = NULL;
  float error, e, weight, difference;
  int *counter;
  float *value;
  int index, increment_done, done;
  int verbose = 0;


  /*
   * Sanity checks
   */

  if (output_variable_id < 0 || output_variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    output_variable_id);
    return;
  }

  if (target_variable_id < 0 || target_variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    target_variable_id);
    return;
  }

  if (/*X_variables[output_variable_id].type != DETERMINISTIC ||*/
      X_variables[target_variable_id].type != DETERMINISTIC){
    fprintf(stderr, "ERROR: set_target: Currently only implemented for deterministic target variables.\n");
    return;
  }
  
  if (X_variables[output_variable_id].type ==
      X_variables[target_variable_id].type &&
      (X_variables[output_variable_id].num_values !=
       X_variables[target_variable_id].num_values ||
       X_variables[output_variable_id].dimension !=
       X_variables[target_variable_id].dimension ||
       X_variables[output_variable_id].resolution !=
       X_variables[target_variable_id].resolution)){
    fprintf(stderr, "ERROR: set_target: parameter mismatch (1).\n");
    return;
  }

  if (X_variables[output_variable_id].type !=
      X_variables[target_variable_id].type &&
      X_variables[output_variable_id].dimension !=
      X_variables[target_variable_id].dimension){
    fprintf(stderr, "ERROR: set_target: parameter mismatch (2).\n");
    return;
  }


  /*
   * ==================================================
   * ==================================================
   * ==================================================
   * Okay, set the target
   * ==================================================
   * ==================================================
   * ==================================================
   */

  output_variable = &(X_variables[output_variable_id]);
  target_variable = &(X_variables[target_variable_id]);
  error = 0.0;

  if (output_variable->type == DETERMINISTIC &&
      target_variable->type == DETERMINISTIC ){
    /*
     * ==================================================
     * deterministic output and target
     * ==================================================
     */
    for (i = 0; i < output_variable->num_gradients; i++){
      network = &(X_networks[output_variable->gradients[i].network_id]);
      for (j = 0, l = 0; j < output_variable->num_values; j++){
	if (L1_error_norm){
	  if (target_variable->value[j] > output_variable->value[j])
 	    e = 1.0;
	  else if (target_variable->value[j] < output_variable->value[j])
 	    e = -1.0;
	  else
 	    e = 0.0;
	  if (i == 0)
	    error += fabs(target_variable->value[j] - 
			  output_variable->value[j]);
	}
	else{			/* L2 norm */
	  e = (target_variable->value[j] - output_variable->value[j]);
	  if (i == 0)
	    error += (0.5 * e * e);
	}
	if (X_training_phase == 2)
	  for (k = 0; k < network->num_params; k++, l++)
	    network->d_error_d_params[k] += 
	      (e * output_variable->gradients[i].value[l]);
      }
    }
    if (output_variable->error[current_patternset->type] < 0.0)
      output_variable->error[current_patternset->type] = error;
    else
      output_variable->error[current_patternset->type] += error;

  }
  


  else if (output_variable->type == STOCHASTIC &&
	   target_variable->type == DETERMINISTIC){

    /*
     * ==================================================
     * stochastich output, deterministic target
     * ==================================================
     */

    counter = (int *) malloc(sizeof(int) * output_variable->dimension);
    value = (float *) malloc(sizeof(float) * output_variable->dimension);
    if (!counter || !value){
      fprintf(stderr, "ERROR: out of memory in X_set_target().\n");
      exit(-1);
    }

    
    for (i = 0; i < output_variable->dimension; i++)
      counter[i] = 0;
    index = 0;
    done = 0;

    /*
     * loop: increment over all output values
     */
    
    do{

      for (i = 0; i < output_variable->dimension; i++)
	value[i] = ((float) counter[i]) / 
	  ((float) (output_variable->resolution - 1));

      /*
       * compute the difference between output coordinates and target value
       * and the weight (for training)
       */


      
      weight = 0.0;
      for (i = 0; i < output_variable->dimension; i++){
	difference = fabs(value[i] - target_variable->value[i]);
	if (output_variable->cyclic && difference > 0.5)
	  difference = 1.0 - difference; /* cyclic */
	weight += difference;
      }    

      /*
       * update gradients
       */

      if (L1_error_norm){
	if (output_variable->value[index] > 0.0)
	  e = -1.0;
	else if (output_variable->value[index] < 0.0)
	  e = 1.0;
	else
	  e = 0.0;
	error += weight * fabs(output_variable->value[index]);
      }
      else{			/* L2 error norm */
	e      = (0.0 - output_variable->value[index]);
	error += (0.5 * weight * e * e);
      }
      if (X_training_phase == 2)
	for (i = 0; i < output_variable->num_gradients; i++){
	  network = &(X_networks[output_variable->gradients[i].network_id]);
	  for (k = 0, l = index * network->num_params;
	       k < network->num_params; k++, l++){
	    network->d_error_d_params[k] += 
	      (weight * e * output_variable->gradients[i].value[l]);
	  }
	}
      
      
      /*
       * print out counter and index
       */

      if (verbose){
	fprintf(stderr, "%d:", index);
	for (i = 0; i < output_variable->dimension; i++)
	  fprintf(stderr, " %d", counter[i]);
	fprintf(stderr, "   ");
	for (i = 0; i < output_variable->dimension; i++)
	  fprintf(stderr, " %5.3f", value[i]);
	fprintf(stderr, "  ->");
	for (i = 0; i < target_variable->dimension; i++)
	  fprintf(stderr, " %5.3f", target_variable->value[i]);
	fprintf(stderr, "   weight=%5.3f\n", weight);
      }
      
      /*
       * increment counter and index
       */
      
      index++;
      for (i = 0, increment_done = 0; 
	   i < output_variable->dimension && !increment_done && !done; i++){
	counter[i]++;
	if (counter[i] < output_variable->resolution)
	  increment_done = 1;
	else
	  counter[i] = 0;
      }

      if (!increment_done)
	done = 1;			/* overflow, we made it through once */


    } while (!done);

    /*
     * update error
     */

    if (output_variable->error[current_patternset->type] < 0.0)
      output_variable->error[current_patternset->type] = error;
    else
      output_variable->error[current_patternset->type] += error;

    
    
    free(value);
    free(counter);

  }

#ifdef new_new

  else if (output_variable->type == STOCHASTIC &&
	   target_variable->type == STOCHASTIC){

    /*
     * ==================================================
     * stochastich output, stochastic target
     * ==================================================
     */

    counter = (int *) malloc(sizeof(int) * output_variable->dimension);
    value = (float *) malloc(sizeof(float) * output_variable->dimension);
    if (!counter || !value){
      fprintf(stderr, "ERROR: out of memory in X_set_target().\n");
      exit(-1);
    }

    
    for (i = 0; i < output_variable->dimension; i++)
      counter[i] = 0;
    index = 0;
    done = 0;

    /*
     * loop: increment over all output values
     */
    
    do{

      for (i = 0; i < output_variable->dimension; i++)
	value[i] = ((float) counter[i]) / 
	  ((float) (output_variable->resolution - 1));

      /*
       * compute the difference between output coordinates and target value
       * and the weight (for training)
       */


      
      weight = 0.0;
      for (i = 0; i < output_variable->dimension; i++){
	difference = fabs(value[i] - target_variable->value[i]);
	if (output_variable->cyclic && difference > 0.5)
	  difference = 1.0 - difference; /* cyclic */
	weight += difference;
      }    

      /*
       * update gradients
       */

      if (L1_error_norm){
	if (output_variable->value[index] > 0.0)
	  e = -1.0;
	else if (output_variable->value[index] < 0.0)
	  e = 1.0;
	else
	  e = 0.0;
	error += weight * fabs(output_variable->value[index]);
      }
      else{			/* L2 error norm */
	e      = (0.0 - output_variable->value[index]);
	error += (0.5 * weight * e * e);
      }
      if (X_training_phase == 2)
	for (i = 0; i < output_variable->num_gradients; i++){
	  network = &(X_networks[output_variable->gradients[i].network_id]);
	  for (k = 0, l = index * network->num_params;
	       k < network->num_params; k++, l++){
	    network->d_error_d_params[k] += 
	      (weight * e * output_variable->gradients[i].value[l]);
	  }
	}
      
      
      /*
       * print out counter and index
       */

      if (verbose){
	fprintf(stderr, "%d:", index);
	for (i = 0; i < output_variable->dimension; i++)
	  fprintf(stderr, " %d", counter[i]);
	fprintf(stderr, "   ");
	for (i = 0; i < output_variable->dimension; i++)
	  fprintf(stderr, " %5.3f", value[i]);
	fprintf(stderr, "  ->");
	for (i = 0; i < target_variable->dimension; i++)
	  fprintf(stderr, " %5.3f", target_variable->value[i]);
	fprintf(stderr, "   weight=%5.3f\n", weight);
      }
      
      /*
       * increment counter and index
       */
      
      index++;
      for (i = 0, increment_done = 0; 
	 i < output_variable->dimension && !increment_done && !done; i++){
	counter[i]++;
	if (counter[i] < output_variable->resolution)
	  increment_done = 1;
	else
	  counter[i] = 0;
      }

      if (!increment_done)
	done = 1;			/* overflow, we made it through once */


    } while (!done);

    /*
     * update error
     */

    if (output_variable->error[current_patternset->type] < 0.0)
      output_variable->error[current_patternset->type] = error;
    else
      output_variable->error[current_patternset->type] += error;

    
    
    free(value);
    free(counter);

  }
#endif
}



/* **********************************************************************
 *
 * X_train(): trains all networks
 *
 * **********************************************************************/


void
X_train(int print_error)
{
  int i, j;
  network_type  *network = NULL;  
  float   sum_xy, sum_x2, sum_y2, corr, max_x, max_y, fabs_x, fabs_y;
  float factor_x, factor_y;
  int verbose = 0;
  int number_of_variables_with_error;


  /*
   * count the variables that have some error
   */


  number_of_variables_with_error = 0;
  for (i = 0; i < X_number_of_variables; i++)
    if (X_variables[i].error[TRAINING_SET_TYPE] >= 0.0)
      number_of_variables_with_error++;


  if (number_of_variables_with_error == 0){
    fprintf(stderr,
	    "WARNING: No target has been set before calling X_train()\n");
  }

  /*
   * update the parameters
   */


  for (i = 0; i < X_number_of_networks; i++){
    network = &(X_networks[i]);
    /*
     * find largest gradient (in either field)
     */
    max_x = max_y = 0.0;
    for (j = 0; j < X_networks[i].num_params; j++){
      fabs_x = fabs(network->prev_d_error_d_params[j]);
      if (fabs_x > max_x)
	max_x = fabs_x;
      fabs_y = fabs(network->d_error_d_params[j]);
      if (fabs_y > max_y)
	max_y = fabs_y;
    }    
    if (max_x > 0.0 && max_y > 0.0){
      /*
       * compute the correlation
       */
      sum_xy = sum_x2 = sum_y2 = 0.0;
      factor_x = 1.0 / max_x;
      factor_y = 1.0 / max_y;
      for (j = 0; j < X_networks[i].num_params; j++){
	sum_xy += (factor_x * network->prev_d_error_d_params[j] * 
		   factor_y * network->d_error_d_params[j]);
	sum_x2 += (factor_x * network->prev_d_error_d_params[j] * 
		   factor_x * network->prev_d_error_d_params[j]);
	sum_y2 += (factor_y * network->d_error_d_params[j] * 
		   factor_y * network->d_error_d_params[j]);
      }
      if (sum_x2 * sum_y2 > 0.0)
	corr = sum_xy / sqrt(sum_x2 * sum_y2);
      else
	corr = 1.0;
      /*
       * scale up the gradients, if there is a minimum size requirement
       */
      if (max_y < network->min_gradient)
	factor_y = network->min_gradient / max_y;
      else if (network->max_gradient > 0.0 && max_y > network->max_gradient)
	factor_y = network->max_gradient / max_y;
      else
	factor_y = 1.0;
    }
    else{
      corr = 1.0;
      factor_y = 1.0;
    }
    /*
     * set the momentum
     */
#ifndef XXX 
    if (corr > 0.9 && !X_error_during_training_increased[TRAINING_SET_TYPE])
      network->momentum = corr * corr * corr;
    else{
      network->momentum = 0.0;	/* black magic! */
      network->min_gradient *= 0.5;
    }
#else
    {static int i = 0;
    if (!i){
      i = 1;
      fprintf(stderr, "\n\n\n\t\tWARNING: momvar=0\n\n");
    }
    network->momentum           = DEFAULT_MOMENTUM;
    }
#endif
    for (j = 0; j < X_networks[i].num_params; j++){
      network->d_error_d_params[j] *= factor_y;
      network->d_params[j] += network->d_error_d_params[j];
      network->params[j]   += network->step_size * network->d_params[j];
      network->d_params[j] *= network->momentum;
      network->prev_d_error_d_params[j] = network->d_error_d_params[j];
      network->d_error_d_params[j] = 0.0;
    }
    if (verbose)
      fprintf(stderr, 
	      "Network \"%s\" (%d): corr=%6.4f mom=%6.4f factor=%6.4f (%d)\n",
	      network->name, i, corr, network->momentum, factor_y,
	      X_error_during_training_increased[TRAINING_SET_TYPE]);
  }



  /*
   * print out (and update) the individual errors
   */


  for (i = 0; i < X_number_of_variables; i++){
    X_clear_all_gradients(i, 1);
    if (X_variables[i].error[TRAINING_SET_TYPE] >= 0.0){
      if (verbose ||
	  (print_error && number_of_variables_with_error > 1)){
	if (X_variables[i].prev_error[TRAINING_SET_TYPE] < 0.0 ||
	     X_variables[i].error[TRAINING_SET_TYPE] <= 
	     X_variables[i].prev_error[TRAINING_SET_TYPE])
	  fprintf(stderr, 
		  "\t\t\"%s\" (%d): E(train)=%8.6f E(test)=%8.6f\n",
		  X_variables[i].name, i,
		  X_variables[i].error[TRAINING_SET_TYPE],
		  X_variables[i].error[TESTING_SET_TYPE]);
	else
	  fprintf(stderr, 
		  "\t\t\"%s\": E(train)=%8.6f E(test)=%8.6f (increased)\n",
		  X_variables[i].name, 
		  X_variables[i].error[TRAINING_SET_TYPE],
		  X_variables[i].error[TESTING_SET_TYPE]);
      }
    }
    for (j = 0; j < NUM_SET_TYPES; j++){
      if (X_variables[i].error[j] >= 0.0)
	X_error_during_training[j]   += X_variables[i].error[j];
      X_variables[i].prev_error[j] = X_variables[i].error[j];
      X_variables[i].error[j]      = -1.0;
    }
  }

}




/* **********************************************************************
 *
 * X_clear_all_gradients(): internal routine, that clears all gradient
 *                          fields attached to a variable
 *
 * **********************************************************************/

void
X_clear_all_gradients(int variable_id, int all_gradients)
{
  int i;

  /*
   * Sanity check
   */

  if (variable_id < 0 || variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    variable_id);
    return;
  }

  /*
   * Clear
   */


  for (i = 0; i < X_variables[variable_id].num_gradients; i++){
    if (X_variables[variable_id].gradients[i].value == NULL)
      fprintf(stderr, "WEIRED: Error in gradients - cannot clear.\n");
    else{
      free(X_variables[variable_id].gradients[i].value);
    }
    X_variables[variable_id].gradients[i].network_id  = -1;
    X_variables[variable_id].gradients[i].value       = NULL;
  }

  X_variables[variable_id].num_gradients = 0;



  if (all_gradients){

    for (i = 0; i < X_variables[variable_id].num_avg_gradients; i++){
      if (X_variables[variable_id].avg_gradients[i].value == NULL)
	fprintf(stderr, "WEIRED: Error in gradients - cannot clear.\n");
      else{
	free(X_variables[variable_id].avg_gradients[i].value);
      }
      X_variables[variable_id].avg_gradients[i].network_id  = -1;
      X_variables[variable_id].avg_gradients[i].value       = NULL;
    }
    
    X_variables[variable_id].num_avg_gradients = 0;


    for (i = 0; i < X_variables[variable_id].num_cumul_gradients; i++){
      if (X_variables[variable_id].cumul_gradients[i].value == NULL)
	fprintf(stderr, "WEIRED: Error in gradients - cannot clear.\n");
      else{
	free(X_variables[variable_id].cumul_gradients[i].value);
      }
      X_variables[variable_id].cumul_gradients[i].network_id  = -1;
      X_variables[variable_id].cumul_gradients[i].value       = NULL;
    }
    
    X_variables[variable_id].num_cumul_gradients = 0;
  }
}




/************************************************************************\
 ************************************************************************
\************************************************************************/





/* **********************************************************************
 *
 * X_add_gradients(): internal routine, tages a gradient field and
 *                    adds it to the internal list of gradients of a variable
 *
 * **********************************************************************/



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
		int specific_output_index) /* set to -1 if the gradients
					    * apply to all output values 
					    * use a specific value, if you
					    * want to set gradients for a 
					    * specific ouput value only */
{
  int i, j, k, known;
  variable_type *variable = NULL;
  int *current_num_gradients_ptr;
  gradients_type  *current_gradients;
  int verbose = 0;

  if (X_calculate_gradients == 0)
    return;

  /*
   * Sanity check
   */

  if (variable_id < 0 || variable_id >= X_number_of_variables){
    fprintf(stderr, 
	    "ERROR: Variable %d not known. Check the id and registration.\n",
	    variable_id);
    return;
  }
  
  if (net_id < 0 || net_id >= X_number_of_networks){
    fprintf(stderr, 
	    "ERROR: Network %d not known. Check the id and registration.\n",
	    net_id);
    return;
  }

  /*
   * set various pointers
   */


  variable          = &(X_variables[variable_id]);
  
  if (!cumul){
    current_num_gradients_ptr = &(variable->num_gradients);
    current_gradients         = &(variable->gradients[0]);
  }
  else{
    current_num_gradients_ptr = &(variable->num_cumul_gradients);
    current_gradients         = &(variable->cumul_gradients[0]);
  }

  /*
   * check, if those gradients exist already exist
   */



  for (i = 0, known = 0;
       i < *current_num_gradients_ptr && !known; i++)
    if (net_id == current_gradients[i].network_id){
      known = 1;
      i--;
    }


  /*
   * if not yet known, allocate memory and introduce the new gradient
   */

  if (!known){
    if (i != *current_num_gradients_ptr){
      fprintf(stderr, "???\n");
      exit(-1);
    }
    current_gradients[i].value =
      (float *) malloc(sizeof(float) * 
		       variable->num_values *
		       X_networks[net_id].num_params);
    if (!current_gradients[i].value){
      fprintf(stderr, "Out of memory in X_add_gradients().\n");
      exit(-1);
    }
    current_gradients[i].network_id = net_id;
    *current_num_gradients_ptr    += 1;
    for (j = 0; j < variable->num_values *
	   X_networks[net_id].num_params; j++)
      current_gradients[i].value[j] = 0.0; /* initialize */
    if (verbose) 
      fprintf(stderr, 
	      "Created grad-var %d for network %d (mem=%d) => %d networks.\n",
	      variable_id, net_id,
	      variable->num_values * X_networks[net_id].num_params,
	      *current_num_gradients_ptr);
  }

  /*
   * Now, add in the values
   */

  if (verbose) fprintf(stderr, "add %d %d\n", i, specific_output_index);

  if (specific_output_index == -1) /* large gradient field, all 
				    * output values */
    for (j = 0; j < variable->num_values *
	   X_networks[net_id].num_params; j++)
      current_gradients[i].value[j] +=
	(weighting_factor * gradient_value[j]);

  else				/* small gradient field, just a single
				 * output value*/

    for (j = 0; j < X_networks[net_id].num_params; j++){
      k = specific_output_index * X_networks[net_id].num_params + j;
      current_gradients[i].value[k] +=
	(weighting_factor * gradient_value[j]);
    }
}



/************************************************************************\
 ************************************************************************
\************************************************************************/





/* **********************************************************************
 *
 * X_save_parameters(): saves all parameters (networks, variables) onto file
 *
 * **********************************************************************/


int
X_save_parameters(char *filename, 
		  char *best_filename, 
		  int check_timer, 
		  int display)
{
  FILE *iop;
  int i, n, net_id, var_id;
  network_type  *network = NULL;
  variable_type *variable = NULL;
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float time_difference;
  int n_nets, n_vars;
  char *fname = NULL;
  float *params = NULL;


  

  /*
   * if timer is swiched on, we'll check the clock first
   */

  if (check_timer){
    gettimeofday(&this_time, NULL);
    time_difference = 
      ((float) (this_time.tv_sec - last_time.tv_sec))
      + (((float) (this_time.tv_usec - last_time.tv_usec))
	 /  1000000.0);
    if (time_difference < NET_AUTOSAVE_FREQUENCY)
      return 0;
  }
  
  
  if (display)
    G_display_switch(SAVE_NET_BUTTON, 1);

  /*
   * sanity checks
   */

  if (X_number_of_networks == 0 && X_number_of_variables == 0){
    if (global_prob_c){
      fprintf(stderr, "ERROR: No network/variable registered. Cannot save.");
      if (display){
	G_display_switch(SAVE_NET_BUTTON, 2);
	usleep(400000);
      }
    }
    if (display)
      G_display_switch(SAVE_NET_BUTTON, 0);
    return 0;
  }


  for (n = 0; n < 2; n++){
    if (!n)
      fname = filename;
    else
      fname = best_filename; 

    if (fname != NULL){

      /*
       * attempt to open the file
       */
      
      if ((iop = fopen(fname, "w")) == 0){
	fprintf(stderr, "ERROR: Could not open parameter output file %s.\n",
		fname);
	if (display){
	  G_display_switch(SAVE_NET_BUTTON, 2);
	  usleep(400000);
	  G_display_switch(SAVE_NET_BUTTON, 0);
	}
	return 0;
      }
      
      /*
       * Save networks first
       */
      
      for (net_id = 0, n_nets = 0; net_id < X_number_of_networks; net_id++){
	network = &(X_networks[net_id]);
	
	fprintf(iop, "\nbegin(network)\n");
	fprintf(iop, "name: %s\n", network->name);
	fprintf(iop, "learning_flag: %d\n", network->learning_flag);
	if (network->type == LOGISTIC_NETWORK){
	  fprintf(iop, "type: logistic\n");
	  fprintf(iop, "num_inputs: %d\n", network->num_inputs);
	  fprintf(iop, "num_pseudo_inputs_per_input: %d\n", 
		  network->num_pseudo_inputs_per_input);
	  fprintf(iop, "num_hidden: %d\n", network->num_hidden);
	}
	else if (network->type == RADIAL_BASIS_NETWORK){
	  fprintf(iop, "type: radial-basis\n");
	  fprintf(iop, "num_inputs: %d\n", network->num_inputs);
	  fprintf(iop, "num_hidden: %d\n", network->num_hidden);
	  fprintf(iop, "num_bases_per_input:\n");
	  for (i = 0; i < network->num_inputs; i++)
	    fprintf(iop, " %d", network->num_bases_per_input[i]);
	  fprintf(iop, "\n");
	}
	else{
	  fprintf(stderr, "Unknown network type encountered.\n");
	  exit(-1);
	}
	fprintf(iop, "num_outputs: %d\n", network->num_outputs);
	fprintf(iop, "num_params: %d\n", network->num_params);
	fprintf(iop, "parameters:\n");
	if (!n)
	  params = network->params;
	else
	  params = network->best_params;
	for (i = 0; i < network->num_params; i++)
	  fprintf(iop, "%g ", params[i]);
	fprintf(iop, "\nend(network)\n");
	n_nets++;
      }
      
      
      /*
       * Save average values of stochastic variables
       */
      
      for (var_id = 0, n_vars = 0; var_id < X_number_of_variables; var_id++){
	variable = &(X_variables[var_id]);
	if (variable->type == STOCHASTIC && variable->save_flag){
	  fprintf(iop, "\nbegin(variable)\n");
	  fprintf(iop, "name: %s\n", variable->name);
	  fprintf(iop, "learning_flag: %d\n", variable->learning_flag);
	  fprintf(iop, "type: stochastic\n");
	  fprintf(iop, "dimension:  %d\n", variable->dimension);
	  fprintf(iop, "resolution: %d\n", variable->resolution);
	  fprintf(iop, "num_values: %d\n", variable->num_values);
	  fprintf(iop, "avg_values:\n");
	  for (i = 0; i < variable->num_values; i++)
	    fprintf(iop, "%g ", variable->avg_value[i]);
	  fprintf(iop, "\nend(variable)\n");
	  n_vars++;
	}
      }
      
      /*
       * done: close file
       */
      
      fclose(iop);
      fprintf(stderr, "%d network(s) and %d variable(s) saved in file %s.\n", 
	      n_nets, n_vars, fname);
    }
  }


  if (display){
    usleep(400000);
    G_display_switch(SAVE_NET_BUTTON, 0);
  }

  last_time.tv_sec = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;
  return 1;
}




/* **********************************************************************
 *
 * X_load_parameters(): loads all parameters from file
 *
 * **********************************************************************/

int
X_load_parameters(char *filename)
{
  FILE  *iop;
  int   i, j, file_ended, error, size, flag;
  char  text[MAX_STRING_LENGTH];
  char  command[80];  
  int   value, value2;
  float float_value, float_value2;
  int verbose = 0;
  int reading_network;
  int reading_network_number;
  int reading_variable;
  int reading_variable_number;
  int num_nets = 0;
  int num_vars = 0;

  G_display_switch(LOAD_NET_BUTTON, 1);

  /*
   * open file
   */
  
  if ((iop = fopen(filename, "r")) == 0){
    fprintf(stderr, "ERROR: Could not open output file %s.\n", filename);
    G_display_switch(LOAD_NET_BUTTON, 2);
    usleep(400000);
    G_display_switch(LOAD_NET_BUTTON, 0);
    return 0;
  }

  reading_variable        = reading_network        =  0;
  reading_variable_number = reading_network_number = -1;
  
  /*
   * read file
   */
  

  for (file_ended = 0, error = 0; !file_ended && !error; ){
    if (fscanf(iop, "%s", command) == EOF)
      file_ended = 1;
    
    else{
      if (verbose)
	fprintf(stderr, "[%s]\n", command);

      /* ================================================== *
       *    network
       * ================================================== */

      /*
       * begin(network)
       */
      if (!reading_network && reading_network_number == -1 &&
	  !reading_variable && reading_variable_number == -1 &&
	  !strcmp(command, "begin(network)")){
	reading_network = 1;
	if (verbose)
	  fprintf(stderr, "begin(network)\n");
      }

      /*
       * name:
       */

      else if (reading_network && reading_network_number == -1 &&
	       !reading_variable && reading_variable_number == -1 &&
	       !strcmp(command, "name:")){
	if (!fgets(text, MAX_STRING_LENGTH, iop)){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  for (i = 0, j = 0; text[j] != '\0' && j < MAX_STRING_LENGTH; j++){
	    text[i] = text[j];
	    if ((i > 0 || text[i] != ' ') && text[i] != 13 && text[i] != 10)
	      i++;
	  }
	  text[i] = '\0';

	  if (verbose)
	    fprintf(stderr, "name: \"%s\"\n", text);
	  for (i = 0; i < X_number_of_networks && 
		 reading_network_number == -1; i++)
	    if (!strcmp(text, X_networks[i].name))
	      reading_network_number = i;
	  if (reading_network_number == -1)
	    fprintf(stderr, 
		    "WARNING: Network \"%s\" in %s not known. Ignored.\n",
		    text, filename);
	}
      }
      
      
      /*
       * learning_flag:
       */
      else if (reading_network && reading_network_number >= 0 &&
	       !reading_variable && reading_variable_number == -1 &&
	       !strcmp(command, "learning_flag:")){
	if (fscanf(iop, "%d", &flag) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "learning_flag: %d\n", flag);
	  if (flag != 0 && flag != 1){
	    fprintf(stderr,
		    "ERROR: learning_flag (%d) must be 0 or 1 in %s.\n", flag,
		    X_networks[reading_network_number].name);
	    error = 1;
	    file_ended = 1;
	  }	    
	  else
	    X_networks[reading_network_number].learning_flag = flag;
	}
      }
      
      /*
       * type:
       */

      else if (reading_network && reading_network_number >= 0 &&
	       !reading_variable && reading_variable_number == -1 &&
	       !strcmp(command, "type:")){
	if (!fgets(text, MAX_STRING_LENGTH, iop)){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  for (i = 0, j = 0; text[i] != '\0'; j++){
	    text[i] = text[j];
	    if ((i > 0 || text[i] != ' ') && text[i] != 13 && text[i] != 10)
	      i++;
	  }
	  text[i] = '\0';
	  if (verbose)
	    fprintf(stderr, "type: \"%s\"\n", text);
	  if (!strcmp(text, "logistic") && !strcmp(text, "radial-basis")){
	    fprintf(stderr, "ERROR: unknown type %s for network %s.\n", text,
		    X_networks[reading_network_number].name);
	    error = 1;
	    file_ended = 1;
	  }	    
	  if ((!strcmp(text, "logistic") && 
	       X_networks[reading_network_number].type == 
	       RADIAL_BASIS_NETWORK) ||
	      (!strcmp(text, "radial-basis") && 
	       X_networks[reading_network_number].type == LOGISTIC_NETWORK)){
	    fprintf(stderr, "ERROR: type mismatch for network %s.\n",
		    X_networks[reading_network_number].name);
	    error = 1;
	    file_ended = 1;
	  }
	}
      }

      /*
       * num_inputs:
       */
      else if (reading_network && reading_network_number >= 0 &&
	       !reading_variable && reading_variable_number == -1 &&
	       !strcmp(command, "num_inputs:")){
	if (fscanf(iop, "%d", &size) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "num_inputs: %d\n", size);
	  if (size != X_networks[reading_network_number].num_inputs){
	    fprintf(stderr, "ERROR: size mismatch (#inputs) for %s.\n", 
		    X_networks[reading_network_number].name);
	    error = 1;
	    file_ended = 1;
	  }	    
	}
      }


      /*
       * num_pseudo_inputs_per_input
       */
      else if (reading_network && reading_network_number >= 0 &&
	       !reading_variable && reading_variable_number == -1 &&
	       !strcmp(command, "num_pseudo_inputs_per_input:")){
	if (fscanf(iop, "%d", &size) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "num_pseudo_inputs_per_input: %d\n", size);
	  if (size != 
	      X_networks[reading_network_number].num_pseudo_inputs_per_input){
	    fprintf(stderr, "ERROR: size mismatch ");
	    fprintf(stderr, "(num_pseudo_inputs_per_input) for %s.\n", 
		    X_networks[reading_network_number].name);
	    error = 1;
	    file_ended = 1;
	  }	    
	}
      }
      

      /*
       * num_hidden:
       */
      else if (reading_network && reading_network_number >= 0 &&
	       !reading_variable && reading_variable_number == -1 &&
	       !strcmp(command, "num_hidden:")){
	if (fscanf(iop, "%d", &size) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "num_hidden: %d\n", size);
	  if (size != X_networks[reading_network_number].num_hidden){
	    fprintf(stderr, "ERROR: size mismatch (#hidden) for %s.\n", 
		    X_networks[reading_network_number].name);
	    error = 1;
	    file_ended = 1;
	  }	    
	}
      }


      /*
       * num_outputs:
       */
      else if (reading_network && reading_network_number >= 0 &&
	       !reading_variable && reading_variable_number == -1 &&
	       !strcmp(command, "num_outputs:")){
	if (fscanf(iop, "%d", &size) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "num_outputs: %d\n", size);
	  if (size != X_networks[reading_network_number].num_outputs){
	    fprintf(stderr, "ERROR: size mismatch (#outputs) for %s.\n", 
		    X_networks[reading_network_number].name);
	    error = 1;
	    file_ended = 1;
	  }	    
	}
      }

      /*
       * num_params:
       */
      else if (reading_network && reading_network_number >= 0 &&
	       !reading_variable && reading_variable_number == -1 &&
	       !strcmp(command, "num_params:")){
	if (fscanf(iop, "%d", &size) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "num_params: %d\n", size);
	  if (size != X_networks[reading_network_number].num_params){
	    fprintf(stderr, "ERROR: size mismatch (#params) for %s.\n", 
		    X_networks[reading_network_number].name);
	    error = 1;
	    file_ended = 1;
	  }	    
	}
      }


      /*
       * num_bases_per_input:
       */
      else if (reading_network && reading_network_number >= 0 &&
	       !reading_variable && reading_variable_number == -1 &&
	       !strcmp(command, "num_bases_per_input:")){
	if (verbose)
	  fprintf(stderr, "num_bases_per_input:");
	for (i = 0; i < X_networks[reading_network_number].num_inputs &&
	       !file_ended; i++)
	  if (fscanf(iop, "%d", &size) == EOF){
	    fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	    error = 1;
	    file_ended = 1;
	  }
	  else{
	    if (size != 
		X_networks[reading_network_number].num_bases_per_input[i]){
	      fprintf(stderr, "ERROR: mismatch (#bases_per_input) for %s.\n", 
		      X_networks[reading_network_number].name);
	      error = 1;
	      file_ended = 1;
	    }	    
	    else if (verbose)
	      fprintf(stderr, " %d", size);
	  }
	if (verbose)
	  fprintf(stderr, "\n");
      }



      /*
       * parameters:
       */
      else if (reading_network && reading_network_number >= 0 &&
	       !reading_variable && reading_variable_number == -1 &&
	       !strcmp(command, "parameters:")){
	if (verbose)
	  fprintf(stderr, "parameters: (...)\n");
	for (i = 0; i < X_networks[reading_network_number].num_params &&
	       !error; i++)
	    if (fscanf(iop, "%f", &float_value) == EOF){
	      fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	      error = 1;
	      file_ended = 1;
	    }
	    else
	      X_networks[reading_network_number].best_params[i] = 
		X_networks[reading_network_number].params[i] = float_value;
      }




      /*
       * end(network)
       */
      else if (reading_network && reading_network_number >= 0 &&
	       !reading_variable && reading_variable_number == -1 &&
	       !strcmp(command, "end(network)")){
	if (!error)
	  fprintf(stderr, "\tInitialized:  network \"%s\" (file: %s)\n",
		  X_networks[reading_network_number].name, filename);
	reading_network = 0;
	reading_network_number = -1;
	num_nets++;
	if (verbose)
	  fprintf(stderr, "end(network)\n");
      }

      /* ================================================== *
       *    variables
       * ================================================== */

      /*
       * begin(variable)
       */
      else if (!reading_network && reading_network_number == -1 &&
	  !reading_variable && reading_variable_number == -1 &&
	  !strcmp(command, "begin(variable)")){
	reading_variable = 1;
	if (verbose)
	  fprintf(stderr, "begin(variable)\n");
      }


      /*
       * name:
       */

      else if (!reading_network && reading_network_number == -1 &&
	       reading_variable && reading_variable_number == -1 &&
	       !strcmp(command, "name:")){
	if (!fgets(text, MAX_STRING_LENGTH, iop)){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  for (i = 0, j = 0; text[i] != '\0'; j++){
	    text[i] = text[j];
	    if ((i > 0 || text[i] != ' ') && text[i] != 13 && text[i] != 10)
	      i++;
	  }
	  text[i] = '\0';
	  if (verbose)
	    fprintf(stderr, "name: \"%s\"\n", text);
	  for (i = 0; i < X_number_of_variables && 
		 reading_variable_number == -1; i++)
	    if (!strcmp(text, X_variables[i].name))
	      reading_variable_number = i;
	  if (reading_variable_number == -1)
	    fprintf(stderr, 
		    "WARNING: Variable \"%s\" in %s not known. Ignored.\n",
		    text, filename);
	}
      }


      /*
       * learning_flag:
       */
      else if (!reading_network && reading_network_number == -1 &&
	       reading_variable && reading_variable_number >= 0 &&
	       !strcmp(command, "learning_flag:")){
	if (fscanf(iop, "%d", &flag) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "learning_flag: %d\n", flag);
	  if (flag != 0 && flag != 1){
	    fprintf(stderr,
		    "ERROR: learning_flag (%d) must be 0 or 1 in %s.\n", flag,
		    X_variables[reading_variable_number].name);
	    error = 1;
	    file_ended = 1;
	  }	    
	  else
	    X_variables[reading_variable_number].learning_flag = flag;
	}
      }

      /*
       * type:
       */

      else if (!reading_network && reading_network_number == -1 &&
	       reading_variable && reading_variable_number >= 0 &&
	       !strcmp(command, "type:")){
	if (!fgets(text, MAX_STRING_LENGTH, iop)){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  for (i = 0, j = 0; text[i] != '\0'; j++){
	    text[i] = text[j];
	    if ((i > 0 || text[i] != ' ') && text[i] != 13 && text[i] != 10)
	      i++;
	  }
	  text[i] = '\0';
	  if (verbose)
	    fprintf(stderr, "type: \"%s\"\n", text);
	  if (!strcmp(text, "stochastic") && !strcmp(text, "deterministic")){
	    fprintf(stderr, "ERROR: unknown type %s for variable %s.\n", text,
		    X_variables[reading_variable_number].name);
	    error = 1;
	    file_ended = 1;
	  }	    
	  if ((!strcmp(text, "deterministic") && 
	       X_variables[reading_variable_number].type != 
	       DETERMINISTIC) ||
	      (!strcmp(text, "stochastic") && 
	       X_variables[reading_variable_number].type != STOCHASTIC)){
	    fprintf(stderr, "ERROR: type mismatch for variable %s.\n",
		    X_variables[reading_variable_number].name);
	    error = 1;
	    file_ended = 1;
	  }
	}
      }

      /*
       * dimension:
       */
      else if (!reading_network && reading_network_number == -1 &&
	       reading_variable && reading_variable_number >= 0 &&
	       !strcmp(command, "dimension:")){
	if (fscanf(iop, "%d", &size) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "dimension: %d\n", size);
	  if (size != X_variables[reading_variable_number].dimension){
	    fprintf(stderr, "ERROR: size mismatch (dimension) for %s.\n", 
		    X_variables[reading_variable_number].name);
	    error = 1;
	    file_ended = 1;
	  }	    
	}
      }


      /*
       * resolution:
       */
      else if (!reading_network && reading_network_number == -1 &&
	       reading_variable && reading_variable_number >= 0 &&
	       !strcmp(command, "resolution:")){
	if (fscanf(iop, "%d", &size) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "resolution: %d\n", size);
	  if (size != X_variables[reading_variable_number].resolution){
	    fprintf(stderr, "ERROR: size mismatch (resolution) for %s.\n", 
		    X_variables[reading_variable_number].name);
	    error = 1;
	    file_ended = 1;
	  }	    
	}
      }


      /*
       * num_values:
       */
      else if (!reading_network && reading_network_number == -1 &&
	       reading_variable && reading_variable_number >= 0 &&
	       !strcmp(command, "num_values:")){
	if (fscanf(iop, "%d", &size) == EOF){
	  fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	  error = 1;
	  file_ended = 1;
	}
	else{
	  if (verbose)
	    fprintf(stderr, "num_values: %d\n", size);
	  if (size != X_variables[reading_variable_number].num_values){
	    fprintf(stderr, "ERROR: size mismatch (num_values) for %s.\n", 
		    X_variables[reading_variable_number].name);
	    error = 1;
	    file_ended = 1;
	  }	    
	}
      }


      /*
       * avg_values:
       */
      else if (!reading_network && reading_network_number == -1 &&
	       reading_variable && reading_variable_number >= 0 &&
	       !strcmp(command, "avg_values:")){
	if (verbose)
	  fprintf(stderr, "avg_values: (...)\n");
	for (i = 0; i < X_variables[reading_variable_number].num_values &&
	       !error; i++)
	    if (fscanf(iop, "%f", &float_value) == EOF){
	      fprintf(stderr, "ERROR: surprising end of file %s.\n", filename);
	      error = 1;
	      file_ended = 1;
	    }
	    else
	      X_variables[reading_variable_number].avg_value[i] = float_value;
      }

      /*
       * end(variable)
       */
      else if (!reading_network && reading_network_number == -1 &&
	       reading_variable && reading_variable_number >= 0 &&
	       !strcmp(command, "end(variable)")){
	if (!error)
	  fprintf(stderr, "\tInitialized: variable \"%s\" (file: %s)\n",
		  X_variables[reading_variable_number].name, filename);
        X_variables[reading_variable_number].save_flag = 1;
	reading_variable = 0;
	reading_variable_number = -1;
	num_vars++;
	if (verbose)
	  fprintf(stderr, "end(variable)\n");
      }



      else
	fprintf(stderr, "Unknown or unexpected keyword in %s: \"%s\"\n",
		filename, command);
    }
  }

  fclose(iop);

  usleep(400000);

  if (error){
    G_display_switch(LOAD_NET_BUTTON, 2);
    G_display_switch(LOAD_NET_BUTTON, 0);
    return 0;
  }


  fprintf(stderr, 
	  "%d variables and %d networks successfully initialized from %s.\n",
	  num_vars, num_nets, filename);
  G_display_switch(LOAD_NET_BUTTON, 0);
  return 1;
}



/* **********************************************************************
 *
 * X_update_best_parameters(): updates the internal "best" parameters
 *
 * **********************************************************************/

void
X_update_best_parameters(int net_id)
{
  int i;

  /*
   * sanity checks
   */
  
  if (net_id < 0 || net_id >= X_number_of_networks){
    fprintf(stderr, 
	    "ERROR: Network %d not known. Check the id and registration.\n",
	    net_id);
    return;
  }

  /*
   * do it, then.
   */


  for (i = 0; i < X_networks[net_id].num_params; i++)
    X_networks[net_id].best_params[i] = X_networks[net_id].params[i];
}

/************************************************************************\
 ************************************************************************
\************************************************************************/


/* **********************************************************************
 *
 * X_training(): internal top-level training routine
 *
 * **********************************************************************/




void
X_main_training_loop()
{
  int j, i;
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float time_difference;
  int print_error;
  static int error_report_period = 1;
  static int error_report_period_defined = 0;
  
  /*
   * Ssanity check: Is there data?
   */
  if (!data || !data->first_set){
    global_modus_training = 0;
    G_display_switch(TRAIN_BUTTON, global_modus_training);
    return;
  }

  /*
   * prepare: clear thing and switch dieplay off
   */

  X_display = 0;
  X_network_clear_pending_updates();

  /*
   * save the parameters
   */

  X_save_parameters(NET_AUTOSAVE_FILENAME, 
		    NET_BEST_AUTOSAVE_FILENAME, 
		    1,		/* 1 = consider timer */
		    1);		/* 1 = display buttons */

  /*
   * Outer loop: Training iterations
   */

  for (; 
       (X_error_during_training[TRAINING_SET_TYPE] >= 0.00001 ||
	training_step == 0) &&
	 global_modus_training; training_step++){
    
    /*
     * check if we'd like to generate a printout on the screen
     */

    if (training_step == 0 || last_time.tv_sec == 0){
      gettimeofday(&last_time, NULL); /* okay, take time */
      /* fprintf(stderr, "### init\n"); */
      print_error = 1;
    }
    else{
      print_error = (training_step % error_report_period == 0);

      if (print_error && !error_report_period_defined){
	/* fprintf(stderr, "### seeking: %d\n", error_report_period); */
	gettimeofday(&this_time, NULL);
	time_difference = 
	  ((float) (this_time.tv_sec - last_time.tv_sec))
	  + (((float) (this_time.tv_usec - last_time.tv_usec))
	     /  1000000.0);
	if (time_difference > MIN_ERROR_REPORT_FREQUENCEY){
	  error_report_period_defined = 1;
	  /* fprintf(stderr, "### found: %d (%g)\n", error_report_period, 
	     time_difference); */
	}
	else{
	  error_report_period *= 10;
	  print_error = 0;
	}
      }
    }





    for (X_training_phase = 1; X_training_phase <= 2; X_training_phase++){


      /*fprintf(stderr, "-phase %d-\n", X_training_phase);*/

      /*
       * Initialize the training epoch
       */
      
      X_initialize_training_epoch();
      
      /*
       * Inner loop: Loop over all pattern sets
       */
      
      for (j = 0, current_patternset = data->first_set; current_patternset;
	   current_patternset = current_patternset->next_set, j++)
	if (current_patternset->type == TRAINING_SET_TYPE ||
	    current_patternset->type == TESTING_SET_TYPE){
	  X_calculate_gradients = 
	    (current_patternset->type == TRAINING_SET_TYPE);
	  
	  /*
	   * Initialize the episode
	   */
	  
	  X_initialize_episode();
	  
	  /*
	   * Process all data within a pattern set
	   */
	  for (current_pattern = current_patternset->first; current_pattern;
	       current_pattern = current_pattern->next)
	    X_process_pattern(current_pattern);
	}
    }

    /*
     * Back to outer loop: train, and save new parameters on disk
     */
    
    X_train(print_error);

    X_save_parameters(NET_AUTOSAVE_FILENAME, 
		      NET_BEST_AUTOSAVE_FILENAME, 
		      1,		/* 1 = consider timer */
		      1);		/* 1 = display buttons */
    
    /*
     * Update global error
     */
    


    for (j = 0; j < NUM_SET_TYPES; j++){

      X_error_during_training_increased[j] = 
	(X_prev_error_during_training[j] >= 0.0 && 
	 X_error_during_training[j] >= 0.0 && training_step >= 1 &&
	 X_prev_error_during_training[j] < X_error_during_training[j]);

      if (X_lowest_error_during_training[j] < 0.0 ||
	  X_lowest_error_during_training[j] > X_error_during_training[j])
	X_lowest_error_during_training[j] = X_error_during_training[j];
    }
    /*
     * Check if this is the best error thus far, and if so, save params.
     */
    
    if ((X_error_during_training[TESTING_SET_TYPE] <= 0.0 &&
	X_lowest_error_during_training[TRAINING_SET_TYPE] == 
	X_error_during_training[TRAINING_SET_TYPE]) ||
	(X_error_during_training[TESTING_SET_TYPE] > 0.0 &&
	X_lowest_error_during_training[TESTING_SET_TYPE] == 
	X_error_during_training[TESTING_SET_TYPE]))
      for (j = 0; j < X_number_of_networks; j++)
	X_update_best_parameters(j);


    /*
     * Print out the error
     */

    if (print_error){
      fprintf(stderr, "\t#%d:", training_step);
      for (j = TRAINING_SET_TYPE; j >= TESTING_SET_TYPE; j--){
	if (j == TRAINING_SET_TYPE)
	  fprintf(stderr, "  E(train)=");
	else
	  fprintf(stderr, "  E(test)=");
	fprintf(stderr, "%8.6f ", X_error_during_training[j]);
	if (X_lowest_error_during_training[j] == X_error_during_training[j])
	  fprintf(stderr, "        ");
	else{
	  if (X_error_during_training_increased[j])
	    fprintf(stderr, "*");
	  else
	    fprintf(stderr, " ");
	  fprintf(stderr, "(%4.2f%%)", 
		  (X_error_during_training[j] - 
		   X_lowest_error_during_training[j])
		  * 100.0 / X_lowest_error_during_training[j]);
	}
      }
      fprintf(stderr, "\n");
    }



    for (j = 0; j < NUM_SET_TYPES; j++)
      X_prev_error_during_training[j] = X_error_during_training[j];
    
    /*
     * Check for mouse events
     */
    mouse_test_loop();
  }

  /*
   * Done, swich display back on, unlight button, stop training
   */

  X_display = 1;
  mem_display_nth_patternset(0);
  global_modus_training = 0;
  X_display_variable(-1);
  G_display_switch(TRAIN_BUTTON, global_modus_training);
}



