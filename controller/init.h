
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
 ***** All copyrights are by Real World Interface Inc., Carnegie
 ***** Mellon University, and the University of Bonn, Germany.
 ***** Removing the copyright information is illegal. Please read
 ***** and make sure you understand the disclaimer below.
 *****
 ***** Contact tyson@rwii.com if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                      Tyson D. Sawyer and Sebastian Thrun
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/controller/init.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 16:53:36 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: init.h,v $
 * Revision 1.1  2002/09/14 16:53:36  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:27  rhino
 * General reorganization of the directories/repository, fusion with the
 * RWI software.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/







extern float *dummy_sensors;
extern float *null_sensors;
extern char *ModuleNameGlobal;



/************************************************************************
 *
 *   NAME:         check_commandline_parameters
 *                 
 *   FUNCTION:     browses the command-line parameters for useful info.
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void check_commandline_parameters(int argc, char **argv, ALL_PARAMS);


/************************************************************************
 *
 *   NAME:         init_program
 *                 
 *   FUNCTION:     Initializes the three structures "robot_state",
 *                 "action_ptr", and "program_state"
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 NOTE: "NULL" disables initialization
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void init_program(ALL_PARAMS);





/************************************************************************
 *
 *   NAME:         allocate_memory
 *                 
 *   FUNCTION:     Allocales all memory
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void allocate_memory(ALL_PARAMS);



/************************************************************************
 *
 *   NAME:         free_memory
 *                 
 *   FUNCTION:     Counterpart to "init_program()"
 *                 frees memory
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 NOTE: "NULL" disables free-ing
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void free_memory(ALL_PARAMS);





/************************************************************************
 *
 *   NAME:         clear_maps
 *                 
 *   FUNCTION:     Clears the maps
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void clear_maps(ALL_PARAMS);



/************************************************************************
 *
 *   NAME:         connect_to_tca
 *                 
 *   FUNCTION:     connects and registers messages
 *                 
 *   PARAMETERS:   ALL_PARAMS: List of all semi-global parameters,
 *                             includes robot_state, robot_specification,
 *                             action, sensation, program_state
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void connect_to_tca(ALL_PARAMS);




/************************************************************************
 *
 *   NAME:         interrupt_handler()
 *                 
 *   FUNCTION:     some ^C signal or kill command arrived
 *                 
 *   PARAMETERS:   int sig           signal
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void interrupt_handler(int sig);
