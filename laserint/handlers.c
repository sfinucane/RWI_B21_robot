
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/laserint/handlers.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:34:06 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: handlers.c,v $
 * Revision 1.1  2002/09/14 15:34:06  rstone
 * *** empty log message ***
 *
 * Revision 1.42  2000/09/06 05:42:19  thrun
 * Map editing buttons.
 *
 * Revision 1.41  2000/03/21 03:44:01  thrun
 * slight tune-ups.
 *
 * Revision 1.40  2000/01/11 12:25:55  schneid1
 * disabled the BEEP in handlers.c and pos.c if flag FGAN is set in Makefile
 *
 * Revision 1.39  1999/10/15 01:30:46  thrun
 * improved interface for robot positioning
 *
 * Revision 1.38  1999/10/15 00:29:30  thrun
 * improved multi-robot handling
 *
 * Revision 1.37  1999/10/14 04:38:19  thrun
 * Fixed a problem of passing on the wrong correction
 * parameters with MAP. Also, ficex a seg-fault problem with
 * laserint when rnu without display.
 *
 * Revision 1.36  1999/10/02 04:50:11  thrun
 * Minor tuning and changes. THis version built the first
 * nice map of the San Jose Tech Museum
 *
 * Revision 1.35  1999/09/28 21:49:54  thrun
 * Multi-robot support in various modules.
 * In particular, I had to introduce a new message for MAP that
 * communicates back robot positions and correction parameters.
 * This might make it necessary to modify existing modules that use
 * MAP - I tried to do this for all I am aware of, but I might have missed some.
 *
 * Revision 1.34  1999/09/28 04:50:00  thrun
 * Fixed minor bug in server architecture that caused the map
 * (in MAP) to be inconsistent.
 *
 * Revision 1.33  1999/09/28 03:31:01  thrun
 * Improved version in the server mode.
 *
 * Revision 1.32  1999/09/06 03:19:44  thrun
 * first working version of the multi-robot server architecture with
 * positon control.
 *
 * Revision 1.31  1999/09/05 21:57:22  thrun
 * initial client server robot for multi-robot mapping
 *
 * Revision 1.30  1999/09/05 17:18:51  thrun
 * 3D ceiling mapping
 *
 * Revision 1.29  1999/07/03 21:44:36  thrun
 * Fixed several bugs in LASERINT and tuned the parameters of
 * LASERINT, MAP, and PLAN for the new Urban Robot
 *
 * Revision 1.28  1999/07/03 18:49:35  thrun
 * LOASERINT can now send correction parameters directly to MAP.
 *
 * Revision 1.27  1999/07/01 19:20:58  fox
 * Nothing special.
 *
 * Revision 1.26  1999/05/08 19:47:29  thrun
 * drastically improved version with recursive matching and samples.
 * Not quite ready yet: samples aren't used in the right way, queue
 * seems not to work, and some constants are hardcoded (phase!) But
 * impressive results!
 *
 * Revision 1.25  1999/05/04 19:47:26  thrun
 * new file "pos.c"
 *
 * Revision 1.24  1999/05/04 01:14:21  thrun
 * queue instead of stack.
 *
 * Revision 1.23  1999/04/23 20:00:09  thrun
 * slight extensions - limited stack (is now a queue) and flag that
 * prevents integration of readings after sharp turn.
 *
 * Revision 1.22  1998/11/19 03:14:15  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.21  1998/11/18 04:21:10  thrun
 * incorporated wall angle into estimation.
 *
 * Revision 1.20  1998/11/17 05:03:13  thrun
 * small, incremental improvements: better handling of sensor
 * displacement, and new parametr that enables laserint to ignore
 * scans when the robot is rotating too much.
 *
 * Revision 1.19  1998/11/16 01:50:41  thrun
 * positon control - works nice, includes global mapping and learning
 * the drift parameters.
 *
 * Revision 1.18  1998/11/15 16:19:18  thrun
 * initial position control method - works reasonably well. But
 * e careful with the timing, isn't an anytime algorithm yet.
 *
 * Revision 1.17  1998/11/12 14:22:55  thrun
 * ?.
 *
 * Revision 1.16  1998/08/11 22:18:25  thrun
 * PLANNER: fixed a problem with large shifts, a problem with th
 * function "pow()" under the latest Linux.
 *
 * Revision 1.15  1997/08/16 14:16:00  thrun
 * Dieter changed something in his laser messages - and this change
 * makes laserint compile again.
 *
 * Revision 1.14  1997/08/12 03:06:02  thrun
 * intermediate version, provides some logging of sensor scans and partial
 * maps within map.
 *
 * Revision 1.13  1997/06/03 11:49:18  fox
 * Museum version.
 *
 * Revision 1.12  1997/05/28 12:37:33  thrun
 * minor changes.
 *
 * Revision 1.11  1997/05/27 10:48:02  thrun
 * .
 *
 * Revision 1.10  1997/04/27 13:02:51  thrun
 * Can now handle 1 or 2 lasers (from file)
 *
 * Revision 1.9  1997/04/27 12:27:20  thrun
 * Extended parameter set (copied from SONARINT)
 *
 * Revision 1.8  1997/02/22 05:16:39  thrun
 * Version numbers are now also checked for libraries.
 *
 * Revision 1.7  1997/02/22 00:59:04  thrun
 * Introduced version number support
 *
 * Revision 1.6  1997/02/17 15:07:42  rhino
 * irgendwas
 *
 * Revision 1.5  1997/02/13 12:50:46  tyson
 * minor fixes.  Fixed stdin on COLLI.
 *
 * Revision 1.4  1997/02/02 22:32:36  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.3  1996/12/03 05:35:27  thrun
 * If TCXHOST is not se, the software will now assume "localhost" and
 * won't terminate.
 *
 * Revision 1.2  1996/11/27 23:21:16  thrun
 * (a) Modifications of Tyson's Makefile: they now work under Solaris again
 * (b) Major modifications of the CONTROLLER module.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:28  rhino
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


#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif


#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include "tcx.h"
#include "tcxP.h"
#include "Application.h"
#include "Net.h"
#include "EZX11.h"
#include "o-graphics.h"
#include "bUtils.h"
#include "corr.h"

#define TCX_define_variables /* this makes sure variables are installed */

#include "MAP-messages.h"


#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */
#include "LASER-messages.h"
#include "SONAR-messages.h"
#include "BASE-messages.h"


#include "LASERINT.h"

#include "beeSoftVersion.h"
#include "libbUtils.h"
#include "librobot.h"
#include "libezx.h"



int new_laser_reading = 0;





/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


TCX_REG_HND_TYPE LASERINT_reply_handler_array[] = {
  {"LASERINT_register_auto_update", "LASERINT_register_auto_update_handler",
     LASERINT_register_auto_update_handler, TCX_RECV_ALL, NULL},
  {"LASERINT_scan", "LASERINT_scan_handler",
     LASERINT_scan_handler, TCX_RECV_ALL, NULL},
};


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/******* auto-reply stuff, data definitions ***************/




int n_auto_update_modules = 0; /* number of processes to whom
				* position should be forwarded
				* automatically upon change */

int n_auto_scan_update_modules = 0;
int n_active_modules           = 0;
int n_calibrated_modules       = 0;



auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];
                                /* collection of TCX-module ptrs */



/************************************************************************
 *
 *   NAME:         count_auto_update_modules()
 *                 
 *   FUNCTION:     Counts, how many modules of the different types require
 *                 auto-updates
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/

void count_auto_update_modules()
{
  int i;


  n_auto_scan_update_modules = 0;
  n_active_modules           = 0;
  n_calibrated_modules       = 0;
 
  for (i = 0; i < n_auto_update_modules; i++)
    if (auto_update_modules[i].active){
      n_active_modules++;
      if (auto_update_modules[i].scan)
	n_auto_scan_update_modules++;
      if (auto_update_modules[i].calibrated == 2)
	n_calibrated_modules++;
    }
}

/************************************************************************
 *
 *   NAME:         activate_buttons()
 *                 
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/

void activate_buttons()
{
  int i;

  for (i = 0; i < n_auto_update_modules && i < MAX_NUMBER_ROBOTS; i++){
    char new_text[128];


    sprintf(new_text, "%s ???", auto_update_modules[i].name);
    G_set_new_text(ROBOT_BUTTON[i], new_text, 0);
    sprintf(new_text, "SET %s NOW!", auto_update_modules[i].name);
    G_set_new_text(ROBOT_BUTTON[i], new_text, 1);
    sprintf(new_text, "%s OKAY", auto_update_modules[i].name);
    G_set_new_text(ROBOT_BUTTON[i], new_text, 2);
    sprintf(new_text, "%s DOWN", auto_update_modules[i].name);
    G_set_new_text(ROBOT_BUTTON[i], new_text, 3);

    G_activate(ROBOT_BUTTON[i]);
    if (auto_update_modules[i].active)
      G_display_switch(ROBOT_BUTTON[i], auto_update_modules[i].calibrated);
    else
      G_display_switch(ROBOT_BUTTON[i], 3);

  }

  for (; i < MAX_NUMBER_ROBOTS; i++)
    G_deactivate(ROBOT_BUTTON[i]);

}


/************************************************************************
 *
 *   NAME:         add_auto_update_module()
 *                 
 *   FUNCTION:     Adds a module to the list of modules that
 *                 get automatical map updates
 *                 
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *                 int map                     1, if subscribe to map
 *                 int correction              1, if subscribe to correction
 *                 
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/



int add_auto_update_module(TCX_MODULE_PTR module, int scan, char *robotname)
{
  int i;

  if (n_auto_update_modules >= MAX_N_AUTO_UPDATE_MODULES){
    fprintf(stderr, "ERROR: auto-module memory overflow. Request rejected.\n");
    return 0;
  }
  else
    for (i = 0; i < n_auto_update_modules; i++)
      if (auto_update_modules[i].module == module){
	fprintf(stderr, 
		"Module %s already known. Subscription modified: %d\n",
		tcxModuleName(module), scan);
	auto_update_modules[i].scan = scan; /* subsrc? */
	auto_update_modules[i].active = 1;
	count_auto_update_modules();
	activate_buttons();
	return 1;
      }
  fprintf(stderr, "Add %s to auto-reply list: %d.\n",
	  tcxModuleName(module), scan);
  auto_update_modules[n_auto_update_modules].module      = module; /* pointer*/
  auto_update_modules[n_auto_update_modules].active      = 1;
  auto_update_modules[n_auto_update_modules].scan        = scan; /* subsrc? */
  auto_update_modules[n_auto_update_modules].name        = robotname;
  auto_update_modules[n_auto_update_modules].calibrated  = 0; /* not yet */
  auto_update_modules[n_auto_update_modules].correction_parameter_x = 0.0;
  auto_update_modules[n_auto_update_modules].correction_parameter_y = 0.0;
  auto_update_modules[n_auto_update_modules].correction_parameter_angle = 0.0;

  auto_update_modules[n_auto_update_modules].correction_type = 0;
  n_auto_update_modules++;
  count_auto_update_modules();
  activate_buttons();
  return 1;
}



/************************************************************************
 *
 *   NAME:         remove_auto_update_module()
 *                 
 *   FUNCTION:     Attempts to remove a module from the list of 
 *                 modules that get automatical map updates 
 *                 
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *                 
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/


int remove_auto_update_module(TCX_MODULE_PTR module)
{     
  int i, j, found = 0;;
  for (i = 0; i < n_auto_update_modules; i++) /* search for module */
    if (auto_update_modules[i].module == module){ /* if module found */
      fprintf(stderr, "Remove %s from auto-reply list.\n",
	      tcxModuleName(module));
      found++;
      auto_update_modules[i].active = 0;
      auto_update_modules[i].calibrated = 0;
      G_display_switch(ROBOT_BUTTON[i], 3);
    }
  if (!found)
    fprintf(stderr, "(%s: no auto-replies removed)\n",
	    tcxModuleName(module));
  count_auto_update_modules();
  return found;
}
  



/************************************************************************
 *
 *   NAME:         send_automatic_scan_update
 *                 
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



void 
send_automatic_scan_update(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			   PROGRAM_STATE_PTR  program_state, 
			   LASERINT_scan_ptr scan,
			   float new_x, float new_y, float new_orientation, 
			   int info)
{
  int i;

  for (i = 0; i < n_auto_update_modules; i++)
    if (auto_update_modules[i].scan > 0)
      
      if (i != info){
	fprintf(stderr, "Send scan update to %s.\n",
		tcxModuleName(auto_update_modules[i].module));
	
	
	compute_backward_correction(new_x,
				    new_y,
				    new_orientation,
				    auto_update_modules[i].
				    correction_parameter_x,
				    auto_update_modules[i].
				    correction_parameter_y,
				    auto_update_modules[i].
				    correction_parameter_angle,
				    auto_update_modules[i].correction_type,
				    &(scan->laser_x),
				    &(scan->laser_y),
				    &(scan->laser_orientation));
	
	tcxSendMsg(auto_update_modules[i].module, "LASERINT_scan", scan);
	
	
      }
  
  
}






/************************************************************************
 *
 *   NAME:         find_info()
 *                 
 *   FUNCTION:     Attempts to remove a module from the list of 
 *                 modules that get automatical map updates 
 *                 
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *                 
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/


int
find_info(TCX_MODULE_PTR module)
{     
  int i;

  if (program_state->is_server){
    for (i = 0; i < n_auto_update_modules; i++)
      if (auto_update_modules[i].module == module) /* if module found */
	return i;
    return -1;
  }
  else
    return 1;
}







/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         BASE_update_status_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void BASE_update_status_reply_handler(TCX_REF_PTR                  ref,
				      BASE_update_status_reply_ptr status)
{
  float uncertainty;
  float diff_x, diff_y, distance, angle, new_x, new_y;
  float status_new_x, status_new_y;
  char out_text[256];
  struct timeval t; 
  time_t current_time;
  float corr_robot_x, corr_robot_y, corr_robot_orientation;




#ifdef LASERINT_debug
  fprintf(stderr, "TCX: Received a BASE_update_status_reply message.\n");
  fprintf(stderr, "robot: %g %g %g\n", 
	  status->pos_x, status->pos_y, status->orientation);
#endif



  if (robot_specifications->reposition_robot_initially &&
      !program_state->first_base_report_received){
    compute_correction_parameters(status->pos_x, 
				  status->pos_y, 
				  90.0 - status->orientation,
				  0.0,
				  0.0,
				  0.0,
				  &(robot_state->correction_parameter_x),
				  &(robot_state->correction_parameter_y),
				  &(robot_state->correction_parameter_angle),
				  &(robot_state->correction_type));
    program_state->first_base_report_received = 1;
  }



  if (robot_specifications->ignore_odometry){
    status->pos_x = 0.0;
    status->pos_y = 0.0;
    status->orientation = 0.0;
  }

  /*
   * check if unreasonably large rotation
   */
  {
    static float previous_orientation;
    static int previous_orientation_defined = 0;
    float diff;

    if (previous_orientation_defined){
      diff = previous_orientation - status->orientation;
      if (diff < 180.0) diff += 360.0;
      if (diff > 180.0) diff -= 360.0;
      diff = fabs(diff);
      if (diff > robot_specifications->strong_turn_threshold){
	program_state->strong_turn = robot_specifications->strong_turn_decay;
	/*fprintf(stderr, "ERROR: DONT USE THAT FEATURE\n");*/
      }
      else if (diff > 0.01 && program_state->strong_turn > 0)
	program_state->strong_turn--;
      else
	program_state->strong_turn = 0;
    }
    previous_orientation_defined = 1;
    previous_orientation = status->orientation;
  }
    
  
  /*
   * LOGGING DATA
   */

  if ((ref != NULL || /* NULL indicates internal call, no TCX */
       program_state->force_logging_on) &&	
      log_iop != NULL){	/* NULL: No logging */
    time(&current_time);
    strftime (out_text, 50, "\n@SENS %d-%m-%y %H:%M:%S",
	      localtime(&current_time));
    gettimeofday (&t,NULL);
    fprintf (log_iop, "%s.%d\n", out_text, t.tv_usec);
    fprintf (log_iop,"#ROBOT %f %f %f\n\n",
	     status->pos_x, status->pos_y, 90.0 - status->orientation);

  }
  
  
  /*
   * When processing a script file, robot position that has been received
   * via TCX will be ignored
   */


  compute_forward_correction(status->pos_x,
			     status->pos_y,
			     90.0 - status->orientation,
			     robot_state->correction_parameter_x,
			     robot_state->correction_parameter_y,
			     robot_state->correction_parameter_angle,
			     robot_state->correction_type,
			     &corr_robot_x,
			     &corr_robot_y, 
			     &corr_robot_orientation);


  if (!program_state->processing_script || ref == NULL){

    if (!robot_state->known ||	/* sanity check! */
	fabs(robot_state->x - corr_robot_x)
	+ fabs(robot_state->y - corr_robot_y) < 1000.0){
      
      
      /*
       * And now compute the internal belief.
       */


      robot_state->x                   = corr_robot_x;
      robot_state->y                   = corr_robot_y;
      robot_state->orientation         = corr_robot_orientation;
      robot_state->translational_speed = status->trans_current_speed;
      robot_state->rotational_speed    = status->rot_current_speed;
      robot_state->known       = 1;
      for (;robot_state->orientation > 360.0;)
	robot_state->orientation -= 360.0;
      for (;robot_state->orientation < 0.0;)
	robot_state->orientation += 360.0;
      robot_state->raw_odometry_x           = status->pos_x;
      robot_state->raw_odometry_y           = status->pos_y;
      robot_state->raw_odometry_orientation = 90.0 - status->orientation;
      /*fprintf(stderr, "STORE FROM BASE: %g %g %g\n",
	      robot_state->raw_odometry_x,
	      robot_state->raw_odometry_y,
	      robot_state->raw_odometry_orientation);*/
    }
    else{
#ifndef FGAN
      putc(7,stderr);
#endif
      fprintf(stderr, "## STATE UNKNOWN ##\n");
      robot_state->known = 0;
    }
  }

  if (ref != NULL)		/* NULL indicates internal call, no TCX */
    tcxFree("BASE_update_status_reply", status);
}

/***************************************************************************
 ***************************************************************************
 ***************************************************************************/


void
inform_clients(ROBOT_SPECIFICATIONS_PTR robot_specifications,
	       PROGRAM_STATE_PTR program_state,
	       ROBOT_STATE_PTR robot_state, int robot_number)
{      
  MAP_saw_a_wall_type wall_msg;


  if (robot_state->angle_to_wall_found){
#ifdef LASERINT_debug
    fprintf(stderr, "\t--> Found a wall, orientation:%6.2f (%g)\n",
	    robot_state->wall_angle,
	    robot_state->wall_angle + robot_state->laser_orientation);  
#endif
    wall_msg.robot_x           = robot_state->laser_x;
    wall_msg.robot_y           = robot_state->laser_y;
    wall_msg.robot_orientation = robot_state->laser_orientation;
    wall_msg.wall_angle        = robot_state->wall_angle;
    
    if (program_state->tcx_initialized &&
	program_state->map_connected)
      tcxSendMsg(MAP, "MAP_saw_a_wall", &wall_msg);
  }
      
  /*
   * Now that we have the local map, let's communicate it to MAP
   */
  
  broadcast_local_map_to_MAP(robot_specifications, program_state,
			     robot_state, robot_number);
}



/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         LASER_laser_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void LASER_laser_reply_handler(TCX_REF_PTR           ref,
			       LASER_laser_reply_ptr laser)
{
  static float last_laser_x;
  static float last_laser_y;
  static float last_laser_orientation;
  static int   last_laser_def = 0;
  int i, j, c, max_c;
  time_t current_time;
  struct timeval t; 
  char out_text[256];
  float help, distance, dist_rot;
  static int count = 1;
  /*
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference; 
  */

  /*
   * increment counter
   */
  program_state->data_count++;

#ifdef LASERINT_debug
  fprintf(stderr, "TCX: Received a LASER_laser_reply message (%d, %d).\n",
	  laser->f_numberOfReadings, laser->r_numberOfReadings);
  for (i = 0; i < robot_specifications->num_sensors; i++){
    j = i % (robot_specifications->num_sensors / 2);
    if (i < robot_specifications->num_sensors / 2){
      if (laser->f_reading != NULL)
	fprintf(stderr, " %d", laser->f_reading[i]);
      else
	fprintf(stderr, " -1");
    }
    else{
      if (laser->r_reading != NULL)
	fprintf(stderr, " %d", laser->r_reading[j]);
      else
	fprintf(stderr, " -1");
    }
  }
  fprintf(stderr, "\n");
#endif
  
#ifdef JUNK  
  if (program_state->strong_turn > 0){
    /*program_state->strong_turn -= 1;*/
    if (ref != NULL)		/* NULL indicates internal call, no TCX */
      tcxFree("LASER_laser_reply", laser);
    return;
  }
#endif
  /*
  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);
  last_time.tv_usec = this_time.tv_usec;
  last_time.tv_sec = this_time.tv_sec;

  fprintf(stderr, "enter: <%d %d %d> %f sec\n",
	  program_state->map_update_pending,
	  program_state->position_control_iteration,
	  robot_specifications->pos_corr_min_num_iterations, time_difference);*/


  /*
   * LOGGING DATA
   */

  if ((ref != NULL || /* NULL indicates internal call, no TCX */
       program_state->force_logging_on) &&	
      log_iop != NULL){	/* NULL: No logging */

    if (ref != NULL){
      time(&current_time);
      strftime (out_text, 50, "\n@SENS %d-%m-%y %H:%M:%S",
		localtime(&current_time));
      gettimeofday (&t,NULL);
      fprintf(log_iop, "%s.%d\n", out_text, t.tv_usec);
    }
    
    fprintf(log_iop,"#LASER %d %d:", laser->f_numberOfReadings, 
	    laser->r_numberOfReadings);

    for (i = 0; i < laser->f_numberOfReadings; i++){
	if (laser->f_reading != NULL)
	  fprintf(log_iop, " %d", laser->f_reading[i]);
	else
	  fprintf(log_iop, " -1");
      }
    for (i = 0; i < laser->r_numberOfReadings; i++){
	if (laser->r_reading != NULL)
	  fprintf(log_iop, " %d", laser->r_reading[i]);
	else
	  fprintf(log_iop, " -1");
      }
      fprintf (log_iop, "\n");
    if (robot_specifications->do_position_correction){
      fprintf (log_iop, "params: %f %f %f %d    %f %f\n",
	       robot_state->correction_parameter_x,
	       robot_state->correction_parameter_y,
	       robot_state->correction_parameter_angle,
	       robot_state->correction_type,
	       robot_specifications->pos_corr_drift_trans,
	       robot_specifications->pos_corr_drift_rot);
      fprintf (log_iop,"position: %f %f %f\n",
	       robot_state->x, robot_state->y, robot_state->orientation);

    fprintf (log_iop,"end(pattern)\n");
    }
  }
  
  /*
   * When processing a script file, laser information received
   * via TCX will be ignored
   */

  new_laser_reading = 0;
  /*fprintf(stderr, "-1-");*/

  if (!program_state->processing_script || ref == NULL){

    /*fprintf(stderr, "-2-");    */
    
    /* *******************************************************************
     *
     * Take this reading if (and only if) the robot position is known,
     * and the robot moved. Otherwise, we'd better drop it.
     */
    

    distance = sqrt(((last_laser_x - robot_state->x)
		     * (last_laser_x - robot_state->x))
		    + ((last_laser_y - robot_state->y)
		       * (last_laser_y - robot_state->y)));
    dist_rot = last_laser_orientation - robot_state->orientation;
    for (; dist_rot >   180.0; ) dist_rot -= 360.0;
    for (; dist_rot <= -180.0; ) dist_rot += 360.0;
    dist_rot = fabs(dist_rot);
    /*
    fprintf(stderr, " dist=<%g %g> ", distance, dist_rot);
    fprintf(stderr, " state=<%g %g %g> old=<%g %g %g> ", robot_state->x, robot_state->y, robot_state->orientation, last_laser_x, last_laser_y, last_laser_orientation);
    */  

    if (robot_state->known && program_state->maps_allocated &&
	(robot_specifications->ignore_odometry ||
	 program_state->last_stack_item == program_state->first_stack_item ||
	 distance + dist_rot >= 
	 robot_specifications->min_advancement_between_interpretations ||
	 count > 0)){

      

      /*fprintf(stderr, "-3-");*/
      new_laser_reading = 1;
      count--;
      if (distance + dist_rot >= 
	 robot_specifications->min_advancement_between_interpretations)
	count = 5;

      

      if (program_state->map_update_pending &&
	  program_state->position_control_iteration <
	  robot_specifications->pos_corr_min_num_iterations){

	fprintf(stderr, " - ");	/* ignore this sensor item */
      }
      else{
	fprintf(stderr, " + ");	 /* use that sensor item */
	
	/*
	 * terminate any on-going position control thing
	 */
	
	terminate_position_control(robot_specifications, program_state, 
				   robot_state);    
	
	/*
	 * save position of last accepted laser reading
	 */
	
	last_laser_x = robot_state->x;
	last_laser_y = robot_state->y;
	last_laser_orientation = robot_state->orientation;
	last_laser_def = 1;
	
	
	/* 
	 * Copy the sensor values into memory. Special care has been taken
	 * for corrupted sensor information (-1)
	 */
	
	
	for (i = 0; i < robot_specifications->num_sensors; i++){
	  
	  if (i < robot_specifications->num_sensors / 2){
	    if (laser->f_reading != NULL)
	      help = (float) laser->f_reading[i];
	    else
	      help = -1.0;
	  }
	  else{
	    if (laser->r_reading != NULL)
	      help = (float) 
		laser->r_reading[i - (robot_specifications->num_sensors / 2)];
	    else
	      help = -1.0;
	  }
	  robot_state->sensor_values[i] = help;
	}

	
	if (0)
	  {
	    float *s = (float *) malloc(sizeof(float) * 
					robot_specifications->num_sensors);
	    for (i = robot_specifications->num_sensors / 2 + 1;
		 i < robot_specifications->num_sensors - 1; i++){
	      if (fabs(robot_state->sensor_values[i] - robot_state->sensor_values[i-1]) < 40.0 && fabs(robot_state->sensor_values[i] - robot_state->sensor_values[i+1]) < 40.0)
		s[i] = (0.5 * robot_state->sensor_values[i]) +
		  (0.25 * robot_state->sensor_values[i-1]) +
		  (0.25 * robot_state->sensor_values[i+1]);
	      else
		s[i] = robot_state->sensor_values[i];
	    }
	    for (i = robot_specifications->num_sensors / 2 + 1;
		 i < robot_specifications->num_sensors - 1; i++)
	      robot_state->sensor_values[i] = s[i];
	    free(s);
	  }
	
	/* 
	 * Constructs a "local" map from these reading only
	 */
	
	compute_local_map(neural_network, robot_specifications, program_state,
			  robot_state);
	/*
	 * smooth and clip the local map 
	 */	

	smooth_local_map(robot_specifications);
	
	clip_local_map(robot_specifications);
	
	
	/*
	 * correct for small odometry errors
	 */
	
	
	initiate_position_control(robot_specifications, program_state, 
				  robot_state);
	/*fprintf(stderr, "initiate: <%d %d %d>\n",
		program_state->map_update_pending,
		program_state->position_control_iteration,
		robot_specifications->pos_corr_min_num_iterations);*/
	
      }
    }
  }    

  /*fprintf(stderr, "exit: <%d %d %d>\n\n",
	  program_state->map_update_pending,
	  program_state->position_control_iteration,
	  robot_specifications->pos_corr_min_num_iterations);*/

  
  /* 
   * Very Important: Free Memory.
   */
  
  if (ref != NULL)		/* NULL indicates internal call, no TCX */
    tcxFree("LASER_laser_reply", laser);
}



/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         SONAR_ir_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SONAR_ir_reply_handler (TCX_REF_PTR          ref,
			     SONAR_ir_reply_ptr   ir)
{


/*#ifdef TCX_debug*/
  fprintf(stderr, "TCX: Received a SONAR_ir_reply message.\n");
/*#endif*/

  tcxFree("SONAR_ir_reply", ir);

}


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         SONAR_sonar_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void SONAR_sonar_reply_handler(TCX_REF_PTR           ref,
			       SONAR_sonar_reply_ptr sonar)
{
  int i;
  time_t current_time;
  struct timeval t; 
  char out_text[256];

#ifdef LASERINT_debug
  fprintf(stderr, "TCX: Received a SONAR_sonar_reply message.\n");
  for (i = 0; i < 24; i++)
    fprintf(stderr, " %5.2f", sonar->values[i]);
  fprintf(stderr, "\n");
#endif
  
  /*
   * LOGGING DATA
   */

  if ((ref != NULL ||	/* NULL indicates internal call, no TCX */
       program_state->force_logging_on) &&
      log_iop != NULL){	/* NULL: No logging */
    time(&current_time);
    strftime (out_text, 50, "\n@SENS %d-%m-%y %H:%M:%S",
	      localtime(&current_time));
    gettimeofday (&t,NULL);
    fprintf(log_iop, "%s.%d\n", out_text, t.tv_usec);
    fprintf(log_iop,"#SONAR %d:", 24);
    for (i = 0; i < 24; i++)
      fprintf(log_iop, " %f", sonar->values[i]);
    fprintf(log_iop, "\n\n");
  }

  /* 
   * Very Important: Free Memory.
   */
  
  if (ref != NULL)		/* NULL indicates internal call, no TCX */
    tcxFree("SONAR_sonar_reply", sonar);
}


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         LASERINT_register_auto_update_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void LASERINT_register_auto_update_handler(TCX_REF_PTR ref, char **robot)
{
#ifdef LASERINT_debug
  fprintf(stderr, "TCX: Received a LASERINT_register_auto_update message from %s.\n", *robot);
#endif
  add_auto_update_module(ref->module, 1, *robot);
} 






/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         LASERINT_scan_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/




void LASERINT_scan_handler(TCX_REF_PTR ref, LASERINT_scan_ptr scan)
{
  int i;
  int info;
  float new_x, new_y, new_o;

  info = find_info(ref->module);

#ifdef LASERINT_debug
  if (program_state->is_server)
    fprintf(stderr, 
	    "TCX: Received a LASERINT_scan message from %s: %g %g %g\n",
	    auto_update_modules[info].name,
	    scan->raw_odometry_x,
	    scan->raw_odometry_y,
	    scan->raw_odometry_orientation);
  else
    fprintf(stderr, "TCX: Received a LASERINT_scan message.\n");
#endif

  if (program_state->calibration != -1)
    return;
  
			       
  if (robot_specifications->num_sensors != scan->num_sensors){
    fprintf(stderr, "Format mismatch 'num_sensors': %d %d\n",
	    robot_specifications->num_sensors, scan->num_sensors);
    exit(-1);
  }

  if (robot_specifications->ignore_front_laser != scan->ignore_front_laser){
    fprintf(stderr, "Format mismatch 'ignore_front_laser': %d %d\n",
	    robot_specifications->ignore_front_laser, 
	    scan->ignore_front_laser);
    exit(-1);
  }

  if (robot_specifications->ignore_rear_laser != scan->ignore_rear_laser){
    fprintf(stderr, "Format mismatch 'ignore_rear_laser': %d %d\n",
	    robot_specifications->ignore_rear_laser, 
	    scan->ignore_rear_laser);
    exit(-1);
  }

  if (robot_specifications->front_laser_offset_x != 
      scan->front_laser_offset_x){
    fprintf(stderr, "Format mismatch 'front_laser_offset_x': %f %f\n",
	    robot_specifications->front_laser_offset_x, 
	    scan->front_laser_offset_x);
    exit(-1);
  }

  if (robot_specifications->front_laser_offset_y !=
      scan->front_laser_offset_y){
    fprintf(stderr, "Format mismatch 'front_laser_offset_y': %f %f\n",
	    robot_specifications->front_laser_offset_y, 
	    scan->front_laser_offset_y);
    exit(-1);
  }

  if (robot_specifications->rear_laser_offset_x != scan->rear_laser_offset_x){
    fprintf(stderr, "Format mismatch 'rear_laser_offset_x': %f %f\n",
	    robot_specifications->rear_laser_offset_x,
	    scan->rear_laser_offset_x);
    exit(-1);
  }

  if (robot_specifications->rear_laser_offset_y != scan->rear_laser_offset_y){
    fprintf(stderr, "Format mismatch 'rear_laser_offset_y': %f %f\n",
	    robot_specifications->rear_laser_offset_y,
	    scan->rear_laser_offset_y);
    exit(-1);
  }


  if (program_state->map_update_pending &&
      program_state->position_control_iteration <
      robot_specifications->pos_corr_min_num_iterations)
    fprintf(stderr, " -");
  else{
    fprintf(stderr, " +");
    terminate_position_control(robot_specifications, 
			       program_state, robot_state);
    
    compute_forward_correction(scan->laser_x,
			       scan->laser_y,
			       scan->laser_orientation,
			       auto_update_modules[info].correction_parameter_x,
			       auto_update_modules[info].correction_parameter_y,
			       auto_update_modules[info].
			       correction_parameter_angle,
			       auto_update_modules[info].correction_type,
			       &new_x,
			       &new_y,
			       &new_o);
    
    
    
    

    if (program_state->is_server){
      robot_state->laser_x = new_x;
      robot_state->laser_y = new_y;
      robot_state->laser_orientation = new_o;
      robot_state->x = new_x;
      robot_state->y = new_y;
      robot_state->orientation = new_o;
    
      robot_state->prev_laser_x = robot_state->x;
      robot_state->prev_laser_y = robot_state->y;
      robot_state->prev_laser_orientation = robot_state->orientation;
      robot_state->prev_laser_defined = 1;
    
    
      robot_state->raw_odometry_x = scan->raw_odometry_x;
      robot_state->raw_odometry_y = scan->raw_odometry_y;
      robot_state->raw_odometry_orientation = scan->raw_odometry_orientation;
      for (i = 0; i < robot_specifications->num_sensors; i++)
	robot_state->sensor_values[i] = scan->sensor_values[i];
    }


    compute_local_map(neural_network, robot_specifications, program_state,
		      robot_state);
    smooth_local_map(robot_specifications);
    clip_local_map(robot_specifications);


    /*
  if (program_state->is_server)
     send_automatic_scan_update(robot_specifications, program_state, scan,
				new_x, new_y, new_o, info);
  */

    compute_aux_values(robot_specifications, program_state, robot_state,
		       new_x, new_y, new_o,
		       scan->raw_odometry_x,
		       scan->raw_odometry_y,
		       scan->raw_odometry_orientation,
		       scan->num_sensors,
		       scan->sensor_values,
		       scan->ignore_front_laser,
		       scan->ignore_rear_laser,
		       scan->front_laser_offset_x,
		       scan->front_laser_offset_y,
		       scan->rear_laser_offset_x,
		       scan->rear_laser_offset_y,
		       info);

    program_state->map_update_pending = 1;
    program_state->position_control_iteration = 0;
    program_state->cache = 1;





    if (info > -1 && !(auto_update_modules[info].calibrated)){

      if (program_state->first_stack_item == program_state->last_stack_item){
	auto_update_modules[info].calibrated = 2;
	G_display_switch(ROBOT_BUTTON[info], 2);
	program_state->calibration = -1;
	display_scans_and_map(robot_specifications, program_state, 
			      robot_state, 0);
      }
      else{
	auto_update_modules[info].calibrated = 1;
	G_display_switch(ROBOT_BUTTON[info], 1);
	program_state->calibration = info;
	display_scans_and_map(robot_specifications, program_state, 
			      robot_state, 1);

      }
    }
  }



    /*
  display_scans_and_map(robot_specifications, program_state, robot_state);
  program_state->strong_turn = 0;
  cache_reading(robot_specifications, program_state, robot_state);
  */



  tcxFree("LASERINT_scan", scan);
}






/************************************************************************
 *
 *   NAME:         LASERINT_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void LASERINT_close_handler(char *name, TCX_MODULE_PTR module)
{
#ifdef LASERINT_debug
  fprintf(stderr, "LASERINT: closed connection detected: %s\n", name);
#endif
  
  remove_auto_update_module(module);
  
  if (!strcmp(name, "MAP")){ /* MAP shut down */
    if (program_state->graphics_initialized)
      G_display_switch(MAP_CONNECTED_BUTTON, 0);
    program_state->map_connected = 0;
    close_script(robot_state, program_state, robot_specifications);
  }
  else if (!strcmp(name, "BASE")){ /* BASE shut down */
    if (program_state->graphics_initialized)
      G_display_switch(BASE_CONNECTED_BUTTON, 0);
    program_state->base_connected = 0;
  }
  else if (!strcmp(name, "LASERINT-SERVER")){ /* LASERINT-SERVER shut down */
    if (program_state->graphics_initialized)
      G_display_switch(SERVER_CONNECTED_BUTTON, 0);
    program_state->server_connected = 0;
  }
  else if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    exit(0);
  }
}



/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         broadcast_local_map_to_MAP
 *                 
 *   FUNCTION:     sends a partial local map to MAP
 *                 
 *   PARAMETERS:   as below
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void broadcast_local_map_to_MAP(ROBOT_SPECIFICATIONS_PTR robot_specifications,
				PROGRAM_STATE_PTR        program_state,
				ROBOT_STATE_PTR          robot_state,
				int robot_number)
{
  MAP_sensor_interpretation_type local; 
  int i, j;
  char robot_name[256] = "";

  /*
   * Did we ever allocate maps? If not, something weird is going on.
   * Try to prevent the worst and quit.
   */
  
  if (!program_state->maps_allocated)
    return;

  /*
   * Are we connected to map? If not, let's try to establish a connection
   */
  
  if(!program_state->map_connected)
    connect_to_MAP(program_state);

  /*
   * Okay, if we are not connected by now, we'd give up the idea of 
   * broadcasting our local map altogether.
   */

  if(!program_state->map_connected)
    return;

  /*
   * Where was the robot when the sensor  reading was taken?
   * Update these values by status report! 
   */

  
  if (program_state->is_server){
    compute_backward_correction(robot_state->laser_x,
				robot_state->laser_y,
				robot_state->laser_orientation,
				auto_update_modules[robot_number].
				correction_parameter_x,
				auto_update_modules[robot_number].
				correction_parameter_y,
				auto_update_modules[robot_number].
				correction_parameter_angle,
				auto_update_modules[robot_number].
				correction_type,
				&(local.robot_x),
				&(local.robot_y),
				&(local.robot_orientation));
    local.robot_orientation = 90.0 - local.robot_orientation;
  }
  else{
    compute_backward_correction(robot_state->laser_x,
				robot_state->laser_y,
				robot_state->laser_orientation,
				robot_state->correction_parameter_x,
				robot_state->correction_parameter_y,
				robot_state->correction_parameter_angle,
				robot_state->correction_type,
				&(local.robot_x),
				&(local.robot_y),
				&(local.robot_orientation));
    local.robot_orientation = 90.0 - local.robot_orientation;
  }

  /* local.robot_x             = robot_state->laser_x;
     local.robot_y             = robot_state->laser_y;
     local.robot_orientation   = robot_state->laser_orientation;
     */
  local.translational_speed = robot_state->translational_speed;
  local.rotational_speed    = robot_state->rotational_speed;
  local.position_corrected_flag = robot_specifications->do_position_correction;
  
  if (robot_specifications->broadcast_sensor_data_to_map){
    local.num_sensor_values_enclosed = robot_specifications->num_sensors;
    local.sensor_values              = robot_state->sensor_values;
  }
  else{
    local.num_sensor_values_enclosed = 0;
    local.sensor_values              = NULL;
  }

  /*
   * Where is the origin of the local map in this message with respect to the
   * robot? Assume that the robot is at (0,0) and facing into the direction
   * of the x-axis. What is the coordinate of the pixel values[0][0]?,
   * and what is the angle of the x-coordinate in values[] in the robot's
   * local coordinate system?
   */

  local.origin_x           = robot_specifications->max_sensors_range;
  local.origin_y           = robot_specifications->max_sensors_range; 
  local.origin_orientation = 90.0;

  local.resolution         = robot_specifications->resolution;

  local.size_x             = robot_specifications->local_map_dim_x;
  local.size_y             = robot_specifications->local_map_dim_y;

  local.delete_previous_map = 0;
  local.map_number         = LASERINT_MAP_NUMBER;
  
  local.char_likelihoods   = (unsigned char *)
    malloc(sizeof(float) * local.size_x * local.size_y);

  for (i = 0; i < local.size_x * local.size_y; i++)
    if (local_active[i]){
      if (local_robot[i])	/* everything under the robot is free! */
	local.char_likelihoods[i] = (unsigned char) 255;
      else if (local_map[i] >= 1.0)
	local.char_likelihoods[i] = (unsigned char) 254;
      else if (local_map[i] <= 0.0)
	local.char_likelihoods[i] = (unsigned char) 1;
      else
	local.char_likelihoods[i] = ((unsigned char) 
				     (local_map[i] * 253.0)) + 1;
    }
    else
      local.char_likelihoods[i] = 0; /* passive! */

  /*fprintf(stderr, "robot_name: [%s] [%d]\n",
    robot_name, robot_number);*/

  if (robot_number >= 0)
    sprintf(robot_name, "%s", auto_update_modules[robot_number].name);


  local.robot_name = robot_name;

  local.raw_robot_x = robot_state->raw_odometry_x;
  local.raw_robot_y = robot_state->raw_odometry_y;
  local.raw_robot_orientation = robot_state->raw_odometry_orientation;

  
  /*
   * for (i = 0; i < local.size_x * local.size_y; i++)
   * fprintf(stderr, " %d", (int) local.char_likelihoods[i]);
   */

#ifdef LASERINT_debug
  fprintf(stderr, "TCX: MAP_sensor_interpretation\n");
#endif

  /* 
   * send the message
   */
  

  if (program_state->map_connected)
    tcxSendMsg(MAP, "MAP_sensor_interpretation", &local);

  free(local.char_likelihoods);
}



/************************************************************************
 *
 *                 *** TCX ROUTINE ***
 *
 *   NAME:         send_correction_parameters_to_map
 *                 
 *   FUNCTION:     sends correction parameters to MAP
 *                 
 *   PARAMETERS:   as below
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
send_correction_parameters_to_map(ROBOT_SPECIFICATIONS_PTR robot_specifications,
				  PROGRAM_STATE_PTR program_state,
				  ROBOT_STATE_PTR robot_state,
				  float corr_x, float corr_y, 
				  float corr_angle, int corr_type)
{
  MAP_correction_parameters_inform_type params;

  if (!robot_specifications->send_corr_parameters_to_map)
    return;

  if(!program_state->map_connected)
    connect_to_MAP(program_state);
  
  if(!program_state->map_connected)
    return;

  /* fprintf(stderr, "\n-CORRECT: %g %g %g-\n",
	  robot_state->correction_parameter_x,
	  robot_state->correction_parameter_y,
	  robot_state->correction_parameter_angle);  */

  params.parameter_x  = corr_x;
  params.parameter_y  = corr_y;
  params.parameter_angle  = corr_angle;
  params.type  = corr_type;

  tcxSendMsg(MAP, "MAP_correction_parameters_inform", &params);
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


static struct timeval last_attempt_connect_BASE = {0, 0};


/************************************************************************
 *
 *   NAME:         connect_to_BASE
 *                 
 *   FUNCTION:     checks, and connects to BASE, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void connect_to_BASE(PROGRAM_STATE_PTR program_state)
{
  BASE_register_auto_update_type data;
  struct timeval current_time;

  if (!program_state->use_tcx)
    return;

  if (program_state->is_server)
    return;

  if(!program_state->base_connected){

    gettimeofday(&current_time, NULL);
    if (current_time.tv_sec < last_attempt_connect_BASE.tv_sec + 5
	|| (current_time.tv_sec == last_attempt_connect_BASE.tv_sec + 5 &&
	    current_time.tv_usec < last_attempt_connect_BASE.tv_usec))
      return;
    
    last_attempt_connect_BASE.tv_sec  = current_time.tv_sec;
    last_attempt_connect_BASE.tv_usec = current_time.tv_usec;
    

    BASE = tcxConnectOptional(TCX_BASE_MODULE_NAME); /* checks, but does 
						      * not wait */

    if (BASE != NULL){
      if (program_state->graphics_initialized)
	G_display_switch(BASE_CONNECTED_BUTTON, 1);

      data.subscribe_status_report = 1;
      data.subscribe_laser_report  = 1;
      data.subscribe_colli_report  = 0;
      data.subscribe_sonar_report  = program_state->logging_on;
      data.subscribe_ir_report     = 0;
      
      tcxSendMsg(BASE, "BASE_register_auto_update", &data);

      program_state->base_connected = 1;
    }
    else if (program_state->graphics_initialized)
      G_display_switch(BASE_CONNECTED_BUTTON, 0);
  }
}




/************************************************************************
 *
 *   NAME:         connect_to_MAP
 *                 
 *   FUNCTION:     checks, and connects to MAP, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

static struct timeval last_attempt_connect_MAP = {0, 0};

void connect_to_MAP(PROGRAM_STATE_PTR program_state)
{
  MAP_register_auto_update_type data;
  struct timeval current_time;

  if (!program_state->use_tcx)
    return;

  /* if (program_state->is_server)
     return;*/

  if(!program_state->map_connected){

    gettimeofday(&current_time, NULL);
    if (current_time.tv_sec < last_attempt_connect_MAP.tv_sec + 5
	|| (current_time.tv_sec == last_attempt_connect_MAP.tv_sec + 5 &&
	    current_time.tv_usec < last_attempt_connect_MAP.tv_usec))
      return;
    
    last_attempt_connect_MAP.tv_sec  = current_time.tv_sec;
    last_attempt_connect_MAP.tv_usec = current_time.tv_usec;
    

    MAP = tcxConnectOptional(TCX_MAP_MODULE_NAME); /* checks, but does 
						    * not wait */

    if (MAP != NULL){
      if (program_state->graphics_initialized)
	G_display_switch(MAP_CONNECTED_BUTTON, 1);

      program_state->map_connected = 1;
    }
    else if (program_state->graphics_initialized)
      G_display_switch(MAP_CONNECTED_BUTTON, 0);
  }
}



/************************************************************************
 *
 *   NAME:         connect_to_SERVER
 *                 
 *   FUNCTION:     checks, and connects to SERVER, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

static struct timeval last_attempt_connect_SERVER = {0, 0};
TCX_MODULE_PTR SERVER;

extern int extendedModuleNameSet;

void connect_to_SERVER(PROGRAM_STATE_PTR program_state)
{
  struct timeval current_time;

  if (!program_state->use_tcx)
    return;

  if (program_state->is_server)
    return;

  if(!program_state->server_connected){

    gettimeofday(&current_time, NULL);
    if (current_time.tv_sec < last_attempt_connect_SERVER.tv_sec + 5
	|| (current_time.tv_sec == last_attempt_connect_SERVER.tv_sec + 5 &&
	    current_time.tv_usec < last_attempt_connect_SERVER.tv_usec))
      return;
    
    last_attempt_connect_SERVER.tv_sec  = current_time.tv_sec;
    last_attempt_connect_SERVER.tv_usec = current_time.tv_usec;
    
    extendedModuleNameSet = 0;	/* discards robot name */
    SERVER = tcxConnectOptional(TCX_LASERINT_SERVER_NAME); /* doesn't wait */
    extendedModuleNameSet = 1;



    if (SERVER != NULL){
      if (program_state->graphics_initialized)
	G_display_switch(SERVER_CONNECTED_BUTTON, 1);

      program_state->server_connected = 1;
      if (robotname)
	tcxSendMsg(SERVER, "LASERINT_register_auto_update", &robotname);
      else{
	char *dummyname;
	dummyname = (char *) malloc(80 * sizeof(char));
	sprintf(dummyname, "%s", "default");
	tcxSendMsg(SERVER, "LASERINT_register_auto_update", &dummyname);
	free(dummyname);
      }

    }
    else if (program_state->graphics_initialized)
      G_display_switch(SERVER_CONNECTED_BUTTON, 0);
  }
}





/************************************************************************
 *
 *   NAME:         init_tcx
 *                 
 *   FUNCTION:     connects to tcx and the two clients.
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/






void init_tcx(PROGRAM_STATE_PTR program_state)
{
  const char *tcxMachine = NULL;
  
  char tcxName[128], hostname[80], pidname[80];
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    BASE_messages,
    LASER_messages,
    SONAR_messages,
    MAP_messages,
    LASERINT_messages
    };
  
  if (!program_state->use_tcx)
    return;

  fprintf(stderr, "Connecting to TCX...");
  fflush(stderr);
  
  

  /*
   * Initialize TCX, register yourself.
   */
  

  /* ====== INITIALIZING TCX ============ */
  tcxMachine = bParametersGetParam(bParamList, "", "TCXHOST");


  if (tcxMachine != NULL){


    printf("Initializing TCX...");
    fflush(stdout);
    
    if (1){
     static  char *name;
      name = (char *) malloc(256 * sizeof(char));
      if (robotname && !program_state->is_server){
	tcxSetModuleNameExtension(robotname);
	sprintf(name, "%s_%s", TCX_LASERINT_MODULE_NAME, robotname);
      }
      else if (!program_state->is_server)
	sprintf(name, "%s", TCX_LASERINT_MODULE_NAME);
      else
	sprintf(name, "%s", TCX_LASERINT_SERVER_NAME);
      tcxInitialize(name, (char *)tcxMachine);
      /*free(name);*/
    }
    else{
      if (robotname && !program_state->is_server)
	tcxSetModuleNameExtension(robotname);
      if (program_state->is_server)
	tcxInitialize(TCX_LASERINT_SERVER_NAME, (char *)tcxMachine);
      else
	tcxInitialize(TCX_LASERINT_MODULE_NAME, (char *)tcxMachine);
    }


    check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
			 BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
			 NULL, 0);
    check_version_number(libezx_major, libezx_minor,
			 libezx_robot_type, libezx_date,
			 "libezx", 0);
    check_version_number(librobot_major, librobot_minor,
			 librobot_robot_type, librobot_date,
			 "librobot", 0);
    check_version_number(libbUtils_major, libbUtils_minor,
			 libbUtils_robot_type, libbUtils_date,
			 "libbUtils", 1);




    

    fprintf(stderr, "done.\n");
    fflush(stderr);
    
    
    tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
			/ sizeof(TCX_REG_MSG_TYPE));
    
  
    tcxRegisterHandlers(BASE_reply_handler_array,
			sizeof(BASE_reply_handler_array)
			/ sizeof(TCX_REG_HND_TYPE));
    tcxRegisterHandlers(LASER_reply_handler_array,
			sizeof(LASER_reply_handler_array)
			/ sizeof(TCX_REG_HND_TYPE));
    tcxRegisterHandlers(SONAR_reply_handler_array,
			sizeof(SONAR_reply_handler_array)
			/ sizeof(TCX_REG_HND_TYPE));
    tcxRegisterHandlers(LASERINT_reply_handler_array,
			sizeof(LASERINT_reply_handler_array)
			/ sizeof(TCX_REG_HND_TYPE));
  
    tcxRegisterCloseHnd(LASERINT_close_handler);
  
    connect_to_BASE(program_state);

    connect_to_MAP(program_state);

    connect_to_SERVER(program_state);
  
    program_state->tcx_initialized = 1;
  
  }
  else{
    fprintf(stderr, "Error in TCX: TCXHOST not set appropriately\n");
  }
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/**** the following handlers won't be used! *****/

void BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				       BASE_robot_position_reply_ptr pos){
  ;
}

void BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
					BASE_action_executed_reply_ptr data)

{
;
}

