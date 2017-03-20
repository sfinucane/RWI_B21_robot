
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/controller/main.c,v $
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
 * $Log: main.c,v $
 * Revision 1.1  2002/09/14 16:53:36  rstone
 * *** empty log message ***
 *
 * Revision 1.11  1998/08/11 22:05:27  thrun
 * New correction parameters - experimental version.
 *
 * Revision 1.10  1998/01/25 01:07:50  thrun
 * tours: timeout. variable display update rate.
 *
 * Revision 1.9  1997/10/24 20:58:44  tyson
 * The previous X changes where munged.
 *
 * Revision 1.8  1997/10/24 19:06:33  tyson
 * fixed problems with X events and abus
 *
 * Revision 1.7  1997/07/17 17:31:49  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.6  1997/03/25 21:44:44  tyson
 * Many bug fixes.
 *
 * Revision 1.5  1997/03/11 17:11:04  tyson
 * added IR simulation and other work
 *
 * Revision 1.4  1997/02/13 12:50:46  tyson
 * minor fixes.  Fixed stdin on COLLI.
 *
 * Revision 1.3  1997/02/02 22:32:33  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.2  1996/11/27 23:20:25  thrun
 * (a) Modifications of Tyson's Makefile: they now work under Solaris again
 * (b) Major modifications of the CONTROLLER module.
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





#include <ctype.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#ifndef i386
#include <sys/unistd.h>
#endif
#include <signal.h>
#include "all.h"
#include "tcx.h"
#include "tcxP.h"
#define MAX_RANDOM 1073741823.5
#define RAND() ((((float) random())/MAX_RANDOM)-1.0)
#define RAND_POS() ((((float) random())/MAX_RANDOM)/2.0)
#define pi  3.14159265358979323846
#define pi2 6.28318530717958647692


ALL_TYPE                 allGlobal;

/**********8**********8**********8**********8**********8*********/

int block_wait(struct timeval *timeout, int tcx_initialized,
	       int X_initialized);


struct timeval block_waiting_time = {1, 0};


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

struct bParamList * bParamList = NULL;

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



/************************************************************************
 *
 *   NAME:          main routine
 *                 
 *   FUNCTION:     controls everything
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


int main(int argc, char **argv)
{
  ROBOT_SPECIFICATIONS     robot_specifications_data;
  PROGRAM_STATE            program_state_data;
  ROBOT_STATE              robot_state_data;
  ACTION                   action_data;
  SENSATION                sensation_data;

  ROBOT_SPECIFICATIONS_PTR robot_specifications = &robot_specifications_data;
  PROGRAM_STATE_PTR        program_state        = &program_state_data;
  ROBOT_STATE_PTR          robot_state          = &robot_state_data;
  ACTION_PTR               action               = &action_data;
  SENSATION_PTR            sensation            = &sensation_data;

  struct timeval TCX_waiting_time = {0, 0};


  robot_specifications->is_initialized = 0;
  program_state->is_initialized        = 0;
  robot_state->is_initialized          = 0;
  action->is_initialized               = 0;
  sensation->is_initialized            = 0;
  allGlobal.is_initialized             = 0;

  signal(SIGTERM, &interrupt_handler); /* kill interupt handler */
  signal(SIGINT,  &interrupt_handler); /* control-C interupt handler */
#if 0
  signal(SIGALRM, &alarm_handler);	/* handler for regular interrupts */
#endif  

  bParamList = bParametersAddEntry(bParamList, "robot", "name", "B21");
  bParamList = bParametersAddEntry(bParamList, "", "TCXHOST", "localhost");
  bParamList  = bParametersAddEntry(bParamList, "", "fork", "yes");

  /* add some parameter files */
  bParamList = bParametersAddFile (bParamList, "etc/beeSoft.ini");

  /* add some enviroment variables */
  bParamList = bParametersAddEnv(bParamList, "", "TCXHOST");

  bParamList = bParametersAddArray(bParamList, "", argc, argv);

  bParametersFillParams(bParamList);

  check_commandline_parameters(argc, argv, ALL);
  init_program(ALL);
  if (!load_parameters(RHINO_INIT_NAME, ALL))
    exit(-1);
  allocate_memory(ALL);
  /*save_parameters(RHINO_INIT_NAME, ALL);*/
  init_graphics(ALL);
  connect_to_tcx(ALL);
  G_display_switch(TITLE_BUTTON, 1);
#if 0
  alarm((unsigned) robot_specifications->alarm_interval); /* set up alarm */
#endif
  tcx_reset_joystick(ALL); 


  for (;!program_state->quit;){
    program_state->something_happened = 0;
    
    if (program_state->tcx_autoconnect)
      connect_to_tcx(ALL);

    if (test_mouse(ALL))       program_state->something_happened = 1;
    
    if (refresh_action(ALL))   program_state->something_happened = 1;

    if (initiate_action(ALL))  program_state->something_happened = 1;

    /* if (terminate_action(ALL)) program_state->something_happened = 1; */
    

    if (program_state->tcx_initialized){
      TCX_waiting_time.tv_sec   = 0;
      TCX_waiting_time.tv_usec  = 0;
      tcxRecvLoop((void *) &TCX_waiting_time);
    }

    if (!program_state->something_happened){
      block_waiting_time.tv_sec  = 1;
      block_waiting_time.tv_usec = 0;
      block_wait(&block_waiting_time, program_state->tcx_initialized,
		 program_state->graphics_initialized);  
    }

#ifdef TOURGUIDE_VERSION
    tourguide_check_for_timeout(ALL);
#endif /* TOURGUIDE_VERSION */

  }



  /* close_log_file(ALL);*/
  /*  if (program_state->tcx_initialized)
      tcxCloseAll();*/
  G_display_switch(TITLE_BUTTON, 2);
  fprintf(stderr, "\nGood-bye.\n");
  sleep(1);
  exit(0);

}
