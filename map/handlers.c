

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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/map/handlers.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:40:22 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: handlers.c,v $
 * Revision 1.1  2002/09/14 15:40:22  rstone
 * *** empty log message ***
 *
 * Revision 1.43  2000/01/11 12:43:29  schneid1
 * disabled the BEEP in map-graphics.c, handlers.c and init.c if flag FGAN is set in Makefile
 *
 * Revision 1.42  1999/11/09 03:30:56  thrun
 * ?
 *
 * Revision 1.41  1999/10/15 00:29:32  thrun
 * improved multi-robot handling
 *
 * Revision 1.40  1999/10/14 19:20:19  thrun
 * Planner: gets correct robot position at startup.
 * Mapper: memorizes each robot and sends around initial position
 * parameters when starting up.
 *
 * Revision 1.39  1999/10/14 04:38:21  thrun
 * Fixed a problem of passing on the wrong correction
 * parameters with MAP. Also, ficex a seg-fault problem with
 * laserint when rnu without display.
 *
 * Revision 1.38  1999/09/28 23:35:20  thrun
 * Minor change (fixed a small bug)
 *
 * Revision 1.37  1999/09/28 21:49:57  thrun
 * Multi-robot support in various modules.
 * In particular, I had to introduce a new message for MAP that
 * communicates back robot positions and correction parameters.
 * This might make it necessary to modify existing modules that use
 * MAP - I tried to do this for all I am aware of, but I might have missed some.
 *
 * Revision 1.36  1999/08/26 12:33:51  schneid1
 * small updates
 *
 * Revision 1.35  1999/08/25 10:05:54  schneid1
 * Some minor changes to the init procedures.
 *
 * Revision 1.34  1999/07/03 18:49:31  thrun
 * LOASERINT can now send correction parameters directly to MAP.
 *
 * Revision 1.33  1999/04/18 19:00:11  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.32  1999/03/08 16:39:33  wolfram
 * Added tcx_free in LOCALIZE_update_status_reply_handler
 *
 * Revision 1.31  1998/11/19 03:14:18  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.30  1998/08/11 22:19:15  thrun
 * ?
 *
 * Revision 1.29  1998/02/22 18:07:44  thrun
 * drift is now handled differently.
 * maps can now be edited by hand.
 *
 * Revision 1.28  1997/08/16 17:15:04  thrun
 * fixed a float-excep bug
 *
 * Revision 1.27  1997/08/12 03:06:05  thrun
 * intermediate version, provides some logging of sensor scans and partial
 * maps within map.
 *
 * Revision 1.26  1997/05/28 15:10:14  thrun
 * variable window size
 *
 * Revision 1.25  1997/05/28 12:37:04  thrun
 * minor changes.
 *
 * Revision 1.24  1997/05/27 10:45:46  thrun
 * fixed minor bug: the display had an impact on what was sent to PLAN.
 *
 * Revision 1.23  1997/05/27 09:55:28  thrun
 * Fixed a problem that arose in the communication with the planner.
 *
 * Revision 1.22  1997/05/27 07:57:29  thrun
 * new message: MAP_clear_all_sensor_maps empties all acquired maps.
 *
 * Revision 1.21  1997/05/26 14:32:49  thrun
 * .
 *
 * Revision 1.20  1997/05/26 10:29:51  thrun
 * .
 *
 * Revision 1.19  1997/05/26 09:32:02  thrun
 * towards multiple map support (cad maps and the alike)
 *
 * Revision 1.18  1997/05/25 12:39:02  thrun
 * .
 *
 * Revision 1.17  1997/05/11 16:17:14  thrun
 * now the .gif also works with the dump_handler (from the commander)
 *
 * Revision 1.16  1997/05/10 14:15:26  thrun
 * Compiles now without LOCALIZE when not UNI_BONN
 *
 * Revision 1.15  1997/05/06 20:14:18  tyson
 * minor stuff
 *
 * Revision 1.14  1997/05/02 09:00:42  fox
 * Minor change.
 *
 * Revision 1.13  1997/05/02 08:56:42  fox
 * Removed detection reply handler.
 *
 * Revision 1.12  1997/04/28 17:17:50  thrun
 * Map now suspends position tracking when LOCALIZE is up. Instead,
 * it uses LOCALIZE's correction parameters and broadcasts those.
 * Dieter, I didn't have a chance to test this!
 *
 * Revision 1.11  1997/04/01 22:46:21  thrun
 * minor changes (mathematica output only)
 *
 * Revision 1.10  1997/04/01 20:59:31  thrun
 * fixed a minor problem that caused problems when position control
 * was switched off.
 *
 * Revision 1.9  1997/03/11 17:14:10  tyson
 * added IR simulation and other work
 *
 * Revision 1.8  1997/02/22 05:16:42  thrun
 * Version numbers are now also checked for libraries.
 *
 * Revision 1.7  1997/02/22 01:59:38  thrun
 * (fixed some compiler problems)
 *
 * Revision 1.6  1997/02/22 00:59:06  thrun
 * Introduced version number support
 *
 * Revision 1.5  1997/02/13 12:50:47  tyson
 * minor fixes.  Fixed stdin on COLLI.
 *
 * Revision 1.4  1997/02/05 16:02:40  fox
 * Changed BASE_setmode message.
 *
 * Revision 1.3  1997/02/02 22:32:38  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.2  1996/12/03 05:35:28  thrun
 * If TCXHOST is not se, the software will now assume "localhost" and
 * won't terminate.
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
#include "EZX11.h"
#include "o-graphics.h"
#include "bUtils.h"



#define TCX_define_variables /* this makes sure variables are installed */

#include "MAP-messages.h"


#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */

#include "SONAR-messages.h"
#include "LASER-messages.h"
#include "BASE-messages.h"

#ifdef UNIBONN
#include "LOCALIZE-messages.h"
#endif
#include "MAP.h"


#include "beeSoftVersion.h"
#include "libbUtils.h"
#include "librobot.h"
#include "libezx.h"



static int just_learned_new_correctpion_params = 0;

FILE *log_iop = NULL;


float corr_x2 = 0.0;
float corr_y2 = 0.0;
float corr_orientation2 = 0.0;
int   corr_type2 = 0;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/******* auto-reply stuff, data definitions ***************/


#define MAX_N_AUTO_UPDATE_MODULES 100


int n_auto_update_modules = 0; /* number of processes to whom
				* position should be forwarded
				* automatically upon change */

int n_auto_map_update_modules         = 0;
int n_auto_correction_update_modules  = 0;
int n_auto_position_update_modules  = 0;



typedef struct{
  TCX_MODULE_PTR module;	/* pointer to TCX module */
  int            map;		/* 1=subscribed to regular map updates */
  int            correction;	/* 1=subscribed to regular correct. updates */
  int            position;	/* 1=subscribed to regular correct. updates */
  int            map_auto_update_pending; /* 1, if we did not send 
					   * an update in the
					   * past where we should have */
  int            min_x, min_y, max_x, max_y; /* borders of this update 
						    * to be sent */
} auto_update_type;


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

  n_auto_map_update_modules         = 0;
  n_auto_correction_update_modules  = 0;
  n_auto_position_update_modules    = 0;

  for (i = 0; i < n_auto_update_modules; i++){
    if (auto_update_modules[i].map)         n_auto_map_update_modules++;
    if (auto_update_modules[i].correction)  n_auto_correction_update_modules++;
    if (auto_update_modules[i].position)    n_auto_position_update_modules++;
  }
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



static int add_auto_update_module(TCX_MODULE_PTR module, int map,
				  int correction, int position)
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
		"Module %s already known. Subscription modified: %d %d\n",
		tcxModuleName(module), map, correction);
	auto_update_modules[i].map = map; /* subsrc? */
	auto_update_modules[i].correction  = correction; /* subsrc? */
	auto_update_modules[i].position    = position; /* subsrc? */
	count_auto_update_modules();
	return 1;
      }
  fprintf(stderr, "Add %s to auto-reply list: %d %d.\n",
	  tcxModuleName(module), map, correction);
  auto_update_modules[n_auto_update_modules].module      = module; /* pointer*/
  auto_update_modules[n_auto_update_modules].map         = map; /* subsrc? */
  auto_update_modules[n_auto_update_modules].correction  = correction; /*sub?*/
  auto_update_modules[n_auto_update_modules].position    = position; /*sub?*/
  auto_update_modules[n_auto_update_modules].map_auto_update_pending = 0;
  n_auto_update_modules++;
  count_auto_update_modules();
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


static int remove_auto_update_module(TCX_MODULE_PTR module)
{     
  int i, j, found = 0;;
  for (i = 0; i < n_auto_update_modules; i++) /* search for module */
    if (auto_update_modules[i].module == module){ /* if module found */
      fprintf(stderr, "Remove %s from auto-reply list.\n",
	      tcxModuleName(module));
      found++;
      n_auto_update_modules--;	/* remove that entry, one less now */
      for (j = i; j < n_auto_update_modules; j++){
	auto_update_modules[j].module = 
	  auto_update_modules[j+1].module; /* shift back */
	auto_update_modules[j].map = 
	  auto_update_modules[j+1].map; /* shift back */
	auto_update_modules[j].correction = 
	  auto_update_modules[j+1].correction; /* shift back */
	auto_update_modules[j].position = 
	  auto_update_modules[j+1].position; /* shift back */
      }
    }
  if (!found)
    fprintf(stderr, "(%s: no auto-replies removed)\n",
	    tcxModuleName(module));
  count_auto_update_modules();
  return found;
}
  


/************************************************************************
 *
 *   NAME:         send_automatic_map_update
 *                 
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


static int auto_update_counter = 0;

void send_automatic_map_update(ROBOT_SPECIFICATIONS_PTR robot_specifications,
			       PROGRAM_STATE_PTR  program_state,
			       int first_x, int num_x, int first_y, int num_y,
			       int delete_previous_map)
{
  int i, n_scheduled;
  int scheduled[MAX_N_AUTO_UPDATE_MODULES];
  MAP_partial_map_reply_ptr partial_map_ptr;
  int min_x, max_x, min_y, max_y;


  /*
   * the following option disables the broadcast
   */
  if (!robot_specifications->broadcasts_on)
     return;

  if (delete_previous_map)
    fprintf(stderr, "\t\t-> delete previous map\n");
  
  /*
   * calculate what (minimum) size map we need to send 
   */

  min_x = first_x;
  max_x = first_x + num_x;
  min_y = first_y;
  max_y = first_y + num_y;

  n_scheduled = 0;
  
  for (i = 0; i < n_auto_update_modules; i++){
    if (auto_update_modules[i].map > 0 &&
	(delete_previous_map || 
	 program_state->force_map_update ||
	 (auto_update_counter % auto_update_modules[i].map == 0))){
      /*
       * update boundaries
       */
      if (auto_update_modules[i].map_auto_update_pending &&
	  min_x > auto_update_modules[i].min_x)
	min_x = auto_update_modules[i].min_x;
      if (auto_update_modules[i].map_auto_update_pending &&
	  min_y > auto_update_modules[i].min_y)
	min_y = auto_update_modules[i].min_y;
      if (auto_update_modules[i].map_auto_update_pending &&
	  max_x < auto_update_modules[i].max_x)
	max_x = auto_update_modules[i].max_x;
      if (auto_update_modules[i].map_auto_update_pending &&
	  max_y < auto_update_modules[i].max_y)
	max_y = auto_update_modules[i].max_y;
      /*
       * mark as scheduled
       */
      scheduled[i] = 1;
      n_scheduled++;
    }
    else
      scheduled[i] = 0;
  }


  auto_update_counter++;
  program_state->force_map_update = 0;

  /*
   * are we done?
   */

  if (n_scheduled == 0)
    return;

  /*
   * build a message that has the required size
   */
  
  partial_map_ptr = make_map_message(robot_specifications, program_state, robot_state,
				     min_x, max_x - min_x,
				     min_y, max_y - min_y,
				     delete_previous_map);

  /*
   * and now send the message to all clients that deserve it
   *
   * NOTICE: Currently we send only a single message, which I hope
   * will actually save time. If this turns out to be a problem,
   * one should generate multiple messages.
   */

  for (i = 0; i < n_auto_update_modules; i++){
    
    if (scheduled[i]){

	/*
	 * and sent this message tho this and all other clients, who
	 * are scheduled and use the same size.
	 */
      
#ifdef MAP_debug
      fprintf(stderr, "Send map update to %s. Size %d %d\n",
	      tcxModuleName(auto_update_modules[i].module),
	      partial_map_ptr->size_x, partial_map_ptr->size_y);
#endif
      
      tcxSendMsg(auto_update_modules[i].module, "MAP_partial_map_reply",
		 partial_map_ptr);

      auto_update_modules[i].map_auto_update_pending = 0;
    }


    else{
      /*
       * put the message on hold - combine with a later message
       */

      if (!auto_update_modules[i].map_auto_update_pending ||
	  auto_update_modules[i].min_x > first_x)
	auto_update_modules[i].min_x = first_x;
      if (!auto_update_modules[i].map_auto_update_pending ||
	  auto_update_modules[i].min_y > first_y)
	auto_update_modules[i].min_y = first_y;
    
      if (!auto_update_modules[i].map_auto_update_pending ||
	  auto_update_modules[i].max_x < first_x + num_x)
	auto_update_modules[i].max_x = first_x + num_x;

      if (!auto_update_modules[i].map_auto_update_pending ||
	  auto_update_modules[i].max_y < first_y + num_y)
	auto_update_modules[i].max_y = first_y + num_y;
    
      auto_update_modules[i].map_auto_update_pending = 1;
    }
    
  }

  /*
   * Free at last. What a relief.
   */

  if (partial_map_ptr != NULL){
    if (partial_map_ptr->char_values != NULL){
      free(partial_map_ptr->char_values);
      partial_map_ptr->char_values = NULL;
    }
    free(partial_map_ptr);
    partial_map_ptr = NULL;
  }

}






/************************************************************************
 *
 *   NAME:         send_automatic_correction_update
 *                 
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void send_automatic_correction_update(void)
{
  MAP_correction_parameters_reply_type corr;
  int i;

  /*
   * the following option disables the broadcast
   */
  if (!robot_specifications->broadcasts_on)
     return;

  
  corr.parameter_x         = robot_state->correction_parameter_x;
  corr.parameter_y         = robot_state->correction_parameter_y;
  corr.parameter_angle     = robot_state->correction_parameter_angle;
  corr.type                = robot_state->correction_type;
  corr.current_wall_angle  = 
    robot_state->map_orientation;
  corr.current_wall_angle_defined =
    robot_state->map_orientation_defined;

  
  for (i = 0; i < n_auto_update_modules; i++){
    if (auto_update_modules[i].correction){

#ifdef MAP_debug
      fprintf(stderr, "Send correction update to %s.\n",
	      tcxModuleName(auto_update_modules[i].module));
#endif

      tcxSendMsg(auto_update_modules[i].module, 
		 "MAP_correction_parameters_reply", &corr); 
    }
  }
}







/************************************************************************
 *
 *   NAME:         send_automatic_position_update
 *                 
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


typedef struct {
  float x;
  float y;
  float orientation;
  float raw_x;
  float raw_y;
  float raw_orientation;
  char *robot_name;
} robot_info_type;
robot_info_type robot_info[MAX_NUM_ROBOTS];
int num_robots = 0;




int find_name(char *robot_name)
{
  int i;


  for (i = 0; i < num_robots; i++){
    if (!(robot_name) && !(robot_info[i].robot_name))
      return i;
    else if (robot_name && robot_info[i].robot_name &&
	     !strcmp(robot_info[i].robot_name, robot_name))
      return i;
  }

  
  if (num_robots == MAX_NUM_ROBOTS){
    fprintf(stderr, "ERROR: Too many different robots.\n");
    return -1;
  }

  if (robot_name){
    robot_info[num_robots].robot_name = 
      (char *) malloc(sizeof(char) * 256);
    strcpy(robot_info[num_robots].robot_name, robot_name);
  }
  else
    robot_info[num_robots].robot_name = NULL;
  num_robots++;

  return num_robots-1;
}





void check_and_add_robot(float x, float y, float orientation,
			 float raw_x, float raw_y, 
			 float raw_orientation,
			 char *robot_name)
{
  int n;

  n = find_name(robot_name);
  if (n == -1)
    return;

  robot_info[n].x = x;
  robot_info[n].y = y;
  robot_info[n].orientation = orientation;
  robot_info[n].raw_x = raw_x;
  robot_info[n].raw_y = raw_y;
  robot_info[n].raw_orientation = raw_orientation;
}


void send_initial_position_update(TCX_MODULE_PTR module)
{
  MAP_robot_position_reply_type rob;
  int i;

  for (i = 0; i < num_robots; i++){

    rob.x                   = robot_info[i].x;
    rob.y                   = robot_info[i].y;
    rob.orientation         = robot_info[i].orientation;
    rob.raw_x               = robot_info[i].raw_x;
    rob.raw_y               = robot_info[i].raw_y;
    rob.raw_orientation     = robot_info[i].raw_orientation;
    rob.robot_name          = robot_info[i].robot_name;
    compute_correction_parameters(rob.raw_x, rob.raw_y, rob.raw_orientation,
				  rob.x, rob.y, rob.orientation,
				  &(rob.parameter_x),
				  &(rob.parameter_y),
				  &(rob.parameter_angle),
				  &(rob.parameter_type));
    
#ifdef MAP_debug
    fprintf(stderr, "########### Send initial position update to %s:\n",
	    tcxModuleName(module));
    fprintf(stderr, "pos: %g %g %g  raw: %g %g %g  corr: %g %g %g",
	    rob.x, rob.y, rob.orientation,
	    rob.raw_x, rob.raw_y, rob.raw_orientation,
	    rob.parameter_x, rob.parameter_y, rob.parameter_angle);
    if (rob.robot_name)
      fprintf(stderr, "%s\n", rob.robot_name);
    else
      fprintf(stderr, "(no name)\n");
#endif
    tcxSendMsg(module, "MAP_robot_position_reply", &rob); 
  }
}


void send_automatic_position_update(float x, float y, float orientation,
				    float raw_x, float raw_y, 
				    float raw_orientation,
				    char *robot_name)
{
  MAP_robot_position_reply_type rob;
  int i;
  char local_name[128];

  check_and_add_robot(x, y, orientation, raw_x, raw_y, raw_orientation,
		      robot_name);

  rob.x                   = x;
  rob.y                   = y;
  rob.orientation         = orientation;
  rob.raw_x               = raw_x;
  rob.raw_y               = raw_y;
  rob.raw_orientation     = raw_orientation;
  rob.robot_name          = &(local_name[0]);

  if (robot_name)
    sprintf(local_name, "%s", robot_name);
  else
    sprintf(local_name, "");

  compute_correction_parameters(raw_x, raw_y, raw_orientation,
				x, y, orientation,
				&(rob.parameter_x),
				&(rob.parameter_y),
				&(rob.parameter_angle),
				&(rob.parameter_type));

  for (i = 0; i < n_auto_update_modules; i++){
    if (auto_update_modules[i].position){
      
#ifdef MAP_debug
      fprintf(stderr, "Send position update to %s:\n",
	      tcxModuleName(auto_update_modules[i].module));
      fprintf(stderr, "pos: %g %g %g  raw: %g %g %g  name: %s:\n",
	      rob.x,
	      rob.y,
	      rob.orientation,
	      rob.raw_x,
	      rob.raw_y,
	      rob.raw_orientation,
	      rob.robot_name);

#endif
      
      tcxSendMsg(auto_update_modules[i].module, 
		 "MAP_robot_position_reply", &rob); 
    }
  }
}








/*********************************************************************\
|*********************************************************************|
\*********************************************************************/





/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         MAP_dump_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void MAP_dump_handler(TCX_REF_PTR ref,
		      void       *data)
{
  int i,j, index1;
  char *dat_file_name = "map.math";
  FILE *dat_iop;
  int min_i, max_i, min_j, max_j;
  float size;
  
#ifdef MAP_debug
  fprintf(stderr, "TCX: Received a MAP_dump message.\n");
#endif

  something_happened = 1;	/* general flag, indicates that a TCX message
				 * was processed. */


  if (ref != NULL)
    save_parameters(MAP_NAME);

#ifdef RHINO_OLD_JUNK
  
  min_i = min_j = -1;
  max_i = max_j = -2;
  for (i = 0; i != robot_specifications->global_map_dim_x; i++)
    for (j = 0; j != robot_specifications->global_map_dim_y; j++){
      index1 = i * robot_specifications->global_map_dim_y + j;
      if (global_active_x[0][index1]){
	if (min_i < 0 || i < min_i)
	  min_i = i;
	if (min_j < 0 || j < min_j)
	  min_j = j;
	if (max_i < 0 || i > max_i)
	  max_i = i;
	if (max_j < 0 || j > max_j)
	  max_j = j;
      }
    }
  
  
  if (max_i >= min_i && max_j >= min_j){
    
    printf("Dumping model into %s...", dat_file_name);
    fflush(stdout);
    dat_iop = fopen(dat_file_name, "w");
    fprintf(dat_iop, "d=0.7\n");
    
    fprintf(dat_iop, "map={");
    for (i = min_i; i <= max_i; i++){
      fprintf(dat_iop, "{");
      for (j = max_j; j >= min_j; j--){
	index1 = i * robot_specifications->global_map_dim_y + j;
	if (global_active_x[0][index1] == 1)
	  fprintf(dat_iop,"%5.3f", global_map_x[0][index1]);
	else
	  fprintf(dat_iop,"d");
	if (j > min_j)
	  fprintf(dat_iop,",");
      }
      fprintf(dat_iop, "}");
      if (i < max_i)
	fprintf(dat_iop,",");
    }
    fprintf(dat_iop, "};\n");
    
    
    
    fprintf(dat_iop, "path={");
    for (i = 0; i != n_path_entries[program_state->actual_map]; i++){
      fprintf(dat_iop, "{%6.1f,%6.1f}", 
	      - (path[program_state->actual_map][i][1] 
		 / robot_specifications->resolution
		 + ((float) robot_specifications->autoshifted_int_y))
	      + ((float) max_j),
	      path[program_state->actual_map][i][0] 
	      / robot_specifications->resolution
	      + ((float) robot_specifications->autoshifted_int_x)
	      - ((float) min_i));
      if (i < n_path_entries[program_state->actual_map]-1)  
	fprintf(dat_iop, ",");
    }
    fprintf(dat_iop, "};\n");
    
    
    
    fprintf(dat_iop, "robot={");
    for (i = /*0*/n_path_entries[program_state->actual_map]-1;
	 i < n_path_entries[program_state->actual_map]; i++){
      size = robot_specifications->robot_size;
      if (i < n_path_entries[program_state->actual_map]-1)
	size *= 0.3;
      fprintf(dat_iop, "GrayLevel[1]");
      fprintf(dat_iop, ",Disk[{%6.1f,%6.1f},%3.0f]", 
	      - (path[program_state->actual_map][i][1] 
		 / robot_specifications->resolution
		 + ((float) robot_specifications->autoshifted_int_y))
	      + ((float) max_j),
	      path[program_state->actual_map][i][0]
	      / robot_specifications->resolution
	      + ((float) robot_specifications->autoshifted_int_x)
	      - ((float) min_i),
	      size
	      / robot_specifications->resolution);
      
      fprintf(dat_iop, ",RGBColor[1,0,0]");
      fprintf(dat_iop, ",Circle[{%6.1f,%6.1f},%3.0f]", 
	      - (path[program_state->actual_map][i][1]
		 / robot_specifications->resolution
		 + ((float) robot_specifications->autoshifted_int_y))
	      + ((float) max_j),
	      path[program_state->actual_map][i][0]
	      / robot_specifications->resolution
	      + ((float) robot_specifications->autoshifted_int_x)
	      - ((float) min_i),
	      size
	      / robot_specifications->resolution);
      
      fprintf(dat_iop, ",Line[{{%6.1f,%6.1f},{%6.1f,%6.1f}}]", 
	      - (path[program_state->actual_map][i][1]
		 / robot_specifications->resolution
		 + ((float) robot_specifications->autoshifted_int_y))
	      + ((float) max_j),
	      path[program_state->actual_map][i][0]
	      / robot_specifications->resolution
	      + ((float) robot_specifications->autoshifted_int_x)
	      - ((float) min_i),
	      
	      - ((path[program_state->actual_map][i][1]
		  + (size
		     * (sin(path[program_state->actual_map][i][2]
			    / 180.0 * M_PI))))
		 / robot_specifications->resolution
		 + ((float) robot_specifications->autoshifted_int_y))
	      + ((float) max_j),
	      
	      ((path[program_state->actual_map][i][0]
		+ (size		/* (st) + */
		   * (cos(path[program_state->actual_map][i][2]
			  / 180.0 * M_PI))))
	       / robot_specifications->resolution
	       + ((float) robot_specifications->autoshifted_int_x))
	      - ((float) min_i));
      
      if (i < n_path_entries[program_state->actual_map]-1)  
	fprintf(dat_iop, ",");
    }
    fprintf(dat_iop, "};\n");
    
    

    
    
    fprintf(dat_iop, "Xmap=ListDensityPlot[map,Mesh->False,");
    fprintf(dat_iop, "Axes->False,AspectRatio->Automatic,Frame->False,");
    fprintf(dat_iop, "PlotRange->All,");
    fprintf(dat_iop, "DisplayFunction->Identity];\n");
    fprintf(dat_iop, "Xpath=Graphics[{Thickness[0.001],RGBColor[1,0,0],Line[path]}];\n");
    fprintf(dat_iop, "Xrobot=Graphics[robot];\n");
    fprintf(dat_iop, "X=Show[Xmap,Xpath,Xrobot,\n");
    fprintf(dat_iop, "DisplayFunction->$DisplayFunction]\n");
    fprintf(dat_iop, "Display[\"map.ps2\", X];\n");

    
    fclose(dat_iop);
    printf("...finished\n");

    printf("To generate a postscript file run Mathematica");
    printf(" and \"psfix map.ps2 > map.ps\n\"");
    
  }
  else
    printf("empty map - function not defined.\n");
  fflush(stdout);

#endif

  save_gif(MAP_GIF_NAME, 0, 1);


#ifdef MAP_debug
  fprintf(stderr, "handler done.\n");
#endif
}






/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         MAP_dump_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void MAP_dump_handler2(TCX_REF_PTR ref,
		      void       *data)
{
  int i,j, k, index1;
  char *dat_file_name = "map.math";
  FILE *dat_iop;
  int min_i, max_i, min_j, max_j;
  float size, f_min;
  
#ifdef MAP_debug
  fprintf(stderr, "TCX: Received a MAP_dump message.\n");
#endif

  something_happened = 1;	/* general flag, indicates that a TCX message
				 * was processed. */

  min_i = min_j = -1;
  max_i = max_j = -2;
  for (k = 0; k != NUM_GLOBAL_MAPS; k++)
    for (i = 0; i != robot_specifications->global_map_dim_x; i++)
      for (j = 0; j != robot_specifications->global_map_dim_y; j++){
	index1 = i * robot_specifications->global_map_dim_y + j;
	if (global_active_x[k][index1]){
	  if (min_i < 0 || i < min_i)
	    min_i = i;
	  if (min_j < 0 || j < min_j)
	    min_j = j;
	  if (max_i < 0 || i > max_i)
	    max_i = i;
	  if (max_j < 0 || j > max_j)
	    max_j = j;
	}
      }

  
  if (max_i >= min_i && max_j >= min_j){
    
    printf("Dumping model into %s...", dat_file_name);
    fflush(stdout);
    dat_iop = fopen(dat_file_name, "w");
    fprintf(dat_iop, "d=0.7\n");

    for (k = 0; k != NUM_GLOBAL_MAPS; k++){
      fprintf(dat_iop, "map%d={", k);
      for (i = min_i; i <= max_i; i++){
	fprintf(dat_iop, "{");
	for (j = max_j; j >= min_j; j--){
	  index1 = i * robot_specifications->global_map_dim_y + j;
	  if (global_active_x[k][index1] == 1)
	    fprintf(dat_iop,"%5.3f", global_map_x[k][index1]);
	  else
	    fprintf(dat_iop,"d");
	  if (j > min_j)
	    fprintf(dat_iop,",");
	}
	fprintf(dat_iop, "}");
	if (i < max_i)
	  fprintf(dat_iop,",");
      }
      fprintf(dat_iop, "};\n");
    }
    
    fprintf(dat_iop, "map%d={", NUM_GLOBAL_MAPS);
    for (i = min_i; i <= max_i; i++){
      fprintf(dat_iop, "{");
      for (j = max_j; j >= min_j; j--){
	index1 = i * robot_specifications->global_map_dim_y + j;
	f_min = 2.0;
	for (k = 0; k != NUM_GLOBAL_MAPS; k++)
	  if (global_active_x[k][index1] == 1 &&
	      f_min > global_map_x[k][index1])
	    f_min = global_map_x[k][index1]; /* find minimum occ value */
	if (f_min == 2.0)
	  fprintf(dat_iop,"d");
	else
	  fprintf(dat_iop,"%5.3f", f_min);
	if (j > min_j)
	  fprintf(dat_iop,",");
      }
      fprintf(dat_iop, "}");
      if (i < max_i)
	fprintf(dat_iop,",");
    }
    fprintf(dat_iop, "};\n");

    
    for (k = 0; k != NUM_GLOBAL_MAPS+1; k++){
      fprintf(dat_iop, "Xmap%d=ListDensityPlot[map%d,Mesh->False,", k, k);
      fprintf(dat_iop, "Axes->False,AspectRatio->Automatic,Frame->False,");
      fprintf(dat_iop, "PlotRange->{All,All,{0,1}},");
      fprintf(dat_iop, "DisplayFunction->$DisplayFunction];\n");
      fprintf(dat_iop, "Display[\"map%d.ps2\", Xmap%d];\n", k, k);
    }
    
    fclose(dat_iop);
    printf("...finished\n");


    for (k = 0; k != NUM_GLOBAL_MAPS+1; k++){
      printf("To generate a postscript file run Mathematica");
      printf(" and \"psfix map%d.ps2 > map%d.ps\"\n", k, k);
    }
    
    
  }
  else
    printf("empty map - function not defined.\n");
  fflush(stdout);
  
#ifdef MAP_debug
    fprintf(stderr, "handler done.\n");
#endif
}


/************************************************************************
 *
 *   NAME:         log_map()
 *                 
 *   FUNCTION:     Dumps a map into the log file
 *                 
 *   PARAMETERS:   ROBOT_STATE_PTR    robot_state    Pointer to robot state
 *                                                   description structure
 *                 PROGRAM_STATE_PTR  program_state  Pointer  to general
 *                                                   program state description
 *                                                   structure 
 *                 ROBOT_SPECIFICATIONS_PTR robot_specifications
 *                                                   Pointer to robot spe-
 *                                                   cifications (see above)
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void
log_map(int                map_number,
	ROBOT_STATE_PTR    robot_state,
	PROGRAM_STATE_PTR  program_state,
	ROBOT_SPECIFICATIONS_PTR robot_specifications)
{
  int i, j, index;
  int min_i, max_i, min_j, max_j;

  if (robot_specifications->data_logging &&
      !log_iop)		/* file not yet open */
    if ((log_iop = fopen(MAP_LOG_NAME, "w")) == 0){
      fprintf(log_iop, 
	      "WARNING: Could not open log file %s. No logging.\n",
	      MAP_LOG_NAME);
	  robot_specifications->data_logging = 0; /* never try again */
	  log_iop = NULL;
    }
  if (log_iop){

    min_i = min_j = -1;
    max_i = max_j = -2;
    for (i = 0; i != robot_specifications->global_map_dim_x; i++)
      for (j = 0; j != robot_specifications->global_map_dim_y; j++){
	index = i * robot_specifications->global_map_dim_y + j;
	if (global_active_x[map_number][index]){
	  if (min_i < 0 || i < min_i)
	    min_i = i;
	  if (min_j < 0 || j < min_j)
	    min_j = j;
	  if (max_i < 0 || i > max_i)
	    max_i = i + 1;
	  if (max_j < 0 || j > max_j)
	    max_j = j + 1;
	}
      }
    
    if (max_i > min_i && max_j > min_j){

      fprintf(log_iop, "global_map[%d]: %d %d\n",
	      map_number, max_i - min_i, max_j - min_j);
      for (i = min_i; i < max_i; i++){
	for (j = min_j; j < max_j; j++){
	  index = i * robot_specifications->global_map_dim_y + j;
	  if (global_active_x[map_number][index])
	    fprintf(log_iop, " %g", global_map_x[map_number][index]);
	  else
	    fprintf(log_iop, " -1");
	}
	fprintf(log_iop, "\n\n");
      }
    }
  }
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


#ifdef TCX_debug
  fprintf(stderr, "TCX: Received a SONAR_ir_reply message.\n");
#endif

  tcxFree("SONAR_ir_reply", ir);

  something_happened = 1; /* important for main loop!
			   * must be in every hadler*/
}


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
  float corr_robot_x,corr_robot_y, corr_robot_orientation;
  int k;

#ifdef MAP_debug
  fprintf(stderr, "\nTCX: Received a BASE_update_status_reply message from ");
  if (ref) fprintf(stderr, "%s\n", tcxModuleName(ref->module));
  else fprintf(stderr, "NULL\n");
  fprintf(stderr, "BASE_update_status_reply_handler: robot: %g %g %g\n", 
	  status->pos_x, status->pos_y, status->orientation);
#endif
  
  something_happened = 1;	/* general flag, indicates that a TCX message
				 * was processed. */


  if (robot_specifications->reposition_robot_initially &&
      !program_state->first_base_report_received){

    if (!program_state->tcx_localize_connected)  /*!*/
      compute_correction_parameters(status->pos_x, 
				    status->pos_y, 
				    90.0 - status->orientation,
				    0.5*robot_specifications->global_mapsize_x,
				    0.5*robot_specifications->global_mapsize_y,
				    90.0 - status->orientation 
				    + robot_state->correction_parameter_angle,
				    &(robot_state->correction_parameter_x),
				    &(robot_state->correction_parameter_y),
				    &(robot_state->correction_parameter_angle),
				    &(robot_state->correction_type));
    corr_x2 = robot_state->correction_parameter_x;
    corr_y2 =robot_state->correction_parameter_y;
    corr_orientation2 = robot_state->correction_parameter_angle;
    corr_type2 = robot_state->correction_type;
    program_state->first_base_report_received = 1;
    just_learned_new_correctpion_params = 1;
  }
  /* 
   *  Apply corrections to the status report robot position data
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

  /* fprintf(stderr, "### Correction %g %g %g: %g %g %g -> %g %g %g\n",
	  robot_state->correction_parameter_x,
	  robot_state->correction_parameter_y,
	  robot_state->correction_parameter_angle,
	  status->pos_x, status->pos_y, 
	  90.0 - status->orientation,
	  corr_robot_x, corr_robot_y, corr_robot_orientation); */

#ifdef MAP_debug
  fprintf(stderr, "corrected: %g %g %g (%g %g %g %d)\n", 
	  corr_robot_x, corr_robot_y, corr_robot_orientation,
	  robot_state->correction_parameter_x,
	  robot_state->correction_parameter_y,
	  robot_state->correction_parameter_angle,
	  robot_state->correction_type);
#endif


  /* 
   *  Add in the drift
   */

  distance = sqrt(((robot_state->x - corr_robot_x) * 
		   (robot_state->x - corr_robot_x)) +
		  ((robot_state->y - corr_robot_y) * 
		   (robot_state->y - corr_robot_y)));
  
  if (distance > 300.0) 
    distance = 300.0;
#ifdef JUNK  
  if (!just_learned_new_correctpion_params){
    update_correction_parameters(corr_robot_x,
				 corr_robot_y, 
				 corr_robot_orientation,
				 0.0,
				 0.0,
				 robot_specifications->drift * distance,
				 &(robot_state->correction_parameter_x),
				 &(robot_state->correction_parameter_y),
				 &(robot_state->correction_parameter_angle),
				 &(robot_state->correction_type));
    just_learned_new_correctpion_params = 0;
  }



  /* 
   * Apply, once more, the *new* 
   * corrections to the status report robot position data
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
#endif  
  if (!robot_state->known ||	/* sanity check! */
      distance < 1000.0){
    

    /*
     * Update the internal belief for the robot position
     */

    robot_state->x           = corr_robot_x;
    robot_state->y           = corr_robot_y;
    robot_state->orientation = corr_robot_orientation;

    robot_state->translational_speed = status->trans_current_speed;
    robot_state->rotational_speed    = status->rot_current_speed;
    robot_state->known       = 1;

    /*
     * Shift the display to include the new coordinates, if necessary
     */
    
    if (program_state->graphics_initialized)
      autoshift_display(program_state, robot_specifications,
			robot_state->x, robot_state->y); 

    /*
     * Add the travel distance to the distance memory, if we are
     * doing automatching. Reason: We'd like to travel a little before
     * we initiate the matching.
     */ 

    if (robot_state->automatch_on && distance < 300.0){
      robot_state->automatch_cumul_distance += distance;
      check_and_combine_global_maps(robot_specifications, program_state, 
				    robot_state);
    }

    /* 
     * Enter position into path memory
     */

    k = find_name(NULL);
    /*fprintf(stderr, " [%d %d] ", k, num_robots);*/

    if (num_robots <= 2){	/* don't do this in multi-robot mode */

      if (n_path_entries < MAX_N_PATH_ENTRIES &&
	  (status->pos_x != robot_state->x ||
	   status->pos_y != robot_state->y)){
	path[n_path_entries][0] = robot_state->x;
	path[n_path_entries][1] = robot_state->y;
	path[n_path_entries][2] = robot_state->orientation;
	path[n_path_entries][3] = 0;
	n_path_entries++;
      }
      
      if (program_state->graphics_initialized){
	G_add_marker(PATH[k], robot_state->x, robot_state->y, 0);
      }
    }
  }
  else{
    fprintf(stderr, "BASE_update_status_reply_handler: ## STATE UNKNOWN ##\n");
    robot_state->known = 0;
  }

  /*
   * finally, display!
   */

  if (program_state->graphics_initialized){
    /*G_display_markers(PATH);*/
    G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y,
		    robot_state->orientation, 0, NULL);
  }
  
  if (ref != NULL) {		/* NULL indicates internal call, no TCX */
#ifdef MAP_debug
     fprintf(stderr, "BASE_update_status_reply_handler: tcxFree()\n");
#endif
     if (status != NULL){
       tcxFree("BASE_update_status_reply", status);
       status = NULL;
     }
   }
#ifdef MAP_debug
  fprintf(stderr, "BASE_update_status_reply_handler: complete\n\n");
  fflush(stderr);
#endif
}



#ifdef UNIBONN


/************************************************************************
 *
 *   NAME:         LOCALIZE_update_status_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void
LOCALIZE_map_reply_handler(TCX_REF_PTR                 ref,
			   LOCALIZE_map_reply_ptr map){
  fprintf(stderr, "TCX: Received a LOCALIZE_map_reply message.\n");
}




/************************************************************************
 *
 *   NAME:         LOCALIZE_update_status_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void LOCALIZE_samples_reply_handler(TCX_REF_PTR ref, LOCALIZE_samples_reply_ptr samples)
{
  fprintf(stderr, "TCX: Received a LOCALIZE_samples_reply message.\n");
  if (samples != NULL){
      tcxFree("LOCALIZE_samples_reply", samples);
      samples = NULL;
    }
}




/************************************************************************
 *
 *   NAME:         LOCALIZE_update_status_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
LOCALIZE_update_status_reply_handler(TCX_REF_PTR                 ref,
				     LOCALIZE_update_status_reply_ptr status){
        float uncorr_robot_x, uncorr_robot_y, uncorr_robot_orientation;

#ifdef MAP_debug
  fprintf(stderr, "TCX: Received a LOCALIZE_update_status_reply message:");
  fprintf(stderr, "%5.3f %5.3f %5.3f %d\n",
	  status->corrX, status->corrY, status->corrRot, status->corrType);
#endif

  compute_backward_correction(robot_state->x, robot_state->y, 
			      robot_state->orientation,
			      robot_state->correction_parameter_x,
			      robot_state->correction_parameter_y,
			      robot_state->correction_parameter_angle,
			      robot_state->correction_type,
			      &uncorr_robot_x, &uncorr_robot_y,  
			      &uncorr_robot_orientation);

  robot_state->correction_parameter_x = status->corrX;
  robot_state->correction_parameter_y = status->corrY;
  robot_state->correction_parameter_angle = status->corrRot;
  robot_state->correction_type = status->corrType;

    compute_forward_correction(uncorr_robot_x, 
			     uncorr_robot_y, 
			     uncorr_robot_orientation,
			     robot_state->correction_parameter_x,
			     robot_state->correction_parameter_y,
			     robot_state->correction_parameter_angle,
			     robot_state->correction_type,
			     &(robot_state->x),
			     &(robot_state->y),
			     &(robot_state->orientation));

  send_automatic_correction_update();

  if (ref != NULL) {		/* NULL indicates internal call, no TCX */
#ifdef MAP_debug
    fprintf(stderr, "LOCALIZE_update_status_reply_handler: tcxFree()\n");
#endif
    if (status != NULL){
      tcxFree("LOCALIZE_update_status_reply", status);
      status = NULL;
    }
  }
#ifdef MAP_debug
  fprintf(stderr, "LOCALIZE_update_status_reply_handler: complete\n\n");
  fflush(stderr);
#endif

  
}
#endif


/************************************************************************
 *
 *   NAME:         MAP_quit_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void MAP_quit_handler(TCX_REF_PTR   ref,
		      void         *data)
{
#ifdef MAP_debug
  fprintf(stderr, "Received a MAP_quit message from %s.\n",
	  tcxModuleName(ref->module));
#endif
  
  something_happened = 1;	/* general flag, indicates that a TCX message
				 * was processed. */

  program_state->quit = 1;
#ifdef MAP_debug
  fprintf(stderr, "handler done.\n");
#endif
  if (log_iop) fclose(log_iop);
  exit(0);
}





/************************************************************************
 *
 *   NAME: MAP_register_auto_update_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void MAP_register_auto_update_handler(TCX_REF_PTR  ref,
				      MAP_register_auto_update_ptr data)
{
  
#ifdef MAP_debug
  fprintf(stderr, "Received a  MAP_register_auto_update_message from %s.\n",
	  tcxModuleName(ref->module));
#endif
  
  something_happened = 1;	/* general flag, indicates that a TCX message
				 * was processed. */
  
  add_auto_update_module(ref->module, 
			 data->subscribe_maps,
			 data->subscribe_position_correction,
			 data->subscribe_robot_positions);

  if (data->subscribe_position_correction)
     send_automatic_correction_update();
  if (data->subscribe_robot_positions)
    send_initial_position_update(ref->module);
  if (data->subscribe_maps)
    send_complete_map(robot_specifications, program_state, robot_state, ref);
  
  
  if (data != NULL){
    tcxFree("MAP_register_auto_update", data);
    data = NULL;
  }
#ifdef MAP_debug
  fprintf(stderr, "handler done.\n");
#endif
}






/************************************************************************
 *
 *   NAME:         MAP_partial_map_query_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void MAP_partial_map_query_handler(TCX_REF_PTR               ref,
				   MAP_partial_map_query_ptr data)
{
#ifdef MAP_debug
  fprintf(stderr, "Received a MAP_partial_map_query message from %s.\n",
	  tcxModuleName(ref->module));
#endif
  
  if (data->resolution != robot_specifications->resolution){
    fprintf(stderr, "ERROR: Resolution mismatch. Request ignored.\n");
  }
  else{
    
    something_happened = 1;	/* general flag, indicates that a TCX message
				 * was processed. */
    
    update_maps(robot_specifications, program_state, robot_state,
		data->first_x, data->size_x, 
		data->first_y, data->size_y, ref, 0); /* this also sends
						       * the tcxReply */
  }

  if (data != NULL){
    tcxFree("MAP_partial_map_query", data);
    data = NULL;
  }
#ifdef MAP_debug
  fprintf(stderr, "handler done.\n");
#endif
}




/************************************************************************
 *
 *   NAME:         MAP_correction_parameters_query_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void MAP_correction_parameters_query_handler(TCX_REF_PTR  ref,
					     void        *data)
{
  MAP_correction_parameters_reply_type corr;

#ifdef MAP_debug
  fprintf(stderr,
	  "Received a MAP_correction_parameters_query message from %s.\n",
	  tcxModuleName(ref->module));
#endif
  
  
  something_happened = 1;	/* general flag, indicates that a TCX message
				 * was processed. */


  corr.parameter_x         = robot_state->correction_parameter_x;
  corr.parameter_y         = robot_state->correction_parameter_y;
  corr.parameter_angle     = robot_state->correction_parameter_angle;
  corr.type                = robot_state->correction_type;
  corr.current_wall_angle  = 
    robot_state->map_orientation;
  corr.current_wall_angle_defined =
    robot_state->map_orientation_defined;

  
  tcxReply(ref, "MAP_correction_parameters_reply", &corr); 
#ifdef MAP_debug
  fprintf(stderr, "handler done.\n");
#endif
}





/************************************************************************
 *
 *   NAME:         MAP_correction_parameters_inform_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void MAP_correction_parameters_inform_handler(TCX_REF_PTR  ref,
					      MAP_correction_parameters_inform_ptr params)
{
  MAP_correction_parameters_reply_type corr;

#ifdef MAP_debug
  fprintf(stderr,
	  "Received a MAP_correction_parameters_inform message from %s: ",
	  tcxModuleName(ref->module));
  fprintf(stderr, "%g %g %g %d\n",
	  params->parameter_x, params->parameter_y, params->parameter_angle,
	  params->type);
#endif
  
  
  something_happened = 1;	/* general flag, indicates that a TCX message
				 * was processed. */

  robot_state->correction_parameter_x = params->parameter_x;
  robot_state->correction_parameter_y = params->parameter_y;
  robot_state->correction_parameter_angle = params->parameter_angle;
  robot_state->correction_type = params->type;
  
  program_state->first_base_report_received = 1;
  just_learned_new_correctpion_params = 1;
  
  send_automatic_correction_update();

#ifdef MAP_debug
  fprintf(stderr, "handler done.\n");
#endif
  tcxFree("MAP_correction_parameters_inform", params);
}




/************************************************************************
 *
 *   NAME:         send_complete_map
 *                 
 *   FUNCTION:     sends TCX complete maps
 *                 
 *   PARAMETERS:   int    first_x
 *                 int    num_x
 *                 int    first_y
 *                 int    num_y
 *                 TCX_REF_PTR  ref   (NULL, if regular update)
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void send_complete_map(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		       PROGRAM_STATE_PTR  program_state,
		       ROBOT_STATE_PTR robot_state,
		       TCX_REF_PTR  ref)
{
  int min_x, max_x, min_y, max_y;
  int x, y, index;

  min_x = -1;
  max_x = -1;
  min_y = -1;
  max_y = -1;


  for (x = 0, index =0; x != robot_specifications->global_map_dim_x; x++)
    for (y = 0; y != robot_specifications->global_map_dim_y; y++, index++)
      if (global_active_x[0][index]){
	if (x < min_x || min_x == -1)
	  min_x = x;
	if (x > max_x || max_x == -1)
	  max_x = x;
	if (y < min_y || min_y == -1)
	  min_y = y;
	if (y > max_y || max_y == -1)
	  max_y = y;
      }

#if 0
  if (min_x == -1 || max_x == -1 || min_y == -1 || max_y == -1)
    return;			/* there is nothing to be sent */
#endif
  
  update_maps(robot_specifications, program_state, robot_state,
	      min_x - robot_specifications->autoshifted_int_x, 
	      max_x-min_x + 1, 
	      min_y - robot_specifications->autoshifted_int_y, 
	      max_y-min_y + 1, ref, 1);
}



/************************************************************************
 *
 *   NAME:         make_map_message
 *                 
 *   FUNCTION:     constructs a map message
 *                 
 *   PARAMETERS:   int    first_x
 *                 int    num_x
 *                 int    first_y
 *                 int    num_y
 *                 TCX_REF_PTR  ref   (NULL, if regular update)
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/



MAP_partial_map_reply_ptr make_map_message(ROBOT_SPECIFICATIONS_PTR 
					   robot_specifications,
					   PROGRAM_STATE_PTR  
					   program_state,
					   ROBOT_STATE_PTR robot_state,
					   int first_x, int num_x, 
					   int first_y, int num_y,
					   int delete_previous_map)
{
  int x, y, shifted_x, shifted_y;
  int partial_map_index, global_map_index;
  MAP_partial_map_reply_ptr partial_map;


  partial_map = (MAP_partial_map_reply_ptr)
    (calloc(1, sizeof(MAP_partial_map_reply_type)));

  partial_map->size_x  = num_x;
  partial_map->first_x = first_x;
  partial_map->size_y  = num_y;
  partial_map->first_y = first_y;
  partial_map->delete_previous_map = delete_previous_map;
  partial_map->number_of_map = 0;
  partial_map->resolution = robot_specifications->resolution;
  partial_map->robot_x = robot_state->x;
  partial_map->robot_y = robot_state->y;
  partial_map->robot_orientation = robot_state->orientation;
  
  partial_map->char_values  = 
    (unsigned char *) (calloc((num_x * num_y), sizeof(char)));


  for (x = 0; x != partial_map->size_x; x++){
    shifted_x = x + first_x + robot_specifications->autoshifted_int_x;
    for (y = 0; y != partial_map->size_y; y++){
      shifted_y = y + first_y + robot_specifications->autoshifted_int_y;
      partial_map_index = x * partial_map->size_y + y;
      global_map_index  = shifted_x
	* robot_specifications->global_map_dim_y + shifted_y;
      if (shifted_x >= 0 &&
	  shifted_x < robot_specifications->global_map_dim_x &&
	  shifted_y >= 0 &&
	  shifted_y < robot_specifications->global_map_dim_y &&
	  global_active_x[0][global_map_index]){
	if (global_map_x[0][global_map_index] <= 0.0)
	  partial_map->char_values[partial_map_index] = (unsigned char) 1;
	else if (global_map_x[0][global_map_index] >= 1.0)
	  partial_map->char_values[partial_map_index] = (unsigned char) 254;
	else 
	  partial_map->char_values[partial_map_index] = 
	    ((unsigned char) (global_map_x[0][global_map_index] * 253.0)) + 1;
      }
      else
	partial_map->char_values[partial_map_index] = (unsigned char) 0;
    }
  }
  
  return partial_map;
}




/************************************************************************
 *
 *   NAME:         update_maps
 *                 
 *   FUNCTION:     sends TCX partial map reply.
 *                 
 *   PARAMETERS:   int    first_x
 *                 int    num_x
 *                 int    first_y
 *                 int    num_y
 *                 TCX_REF_PTR  ref   (NULL, if regular update)
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void update_maps(ROBOT_SPECIFICATIONS_PTR robot_specifications,
		 PROGRAM_STATE_PTR  program_state,
		 ROBOT_STATE_PTR robot_state,
		 int first_x, int num_x, int first_y, int num_y,
		 TCX_REF_PTR  ref, /* NULL, if to be broadcasted everywhere! */
		 int delete_previous_map)
{
 MAP_partial_map_reply_ptr partial_map;
   
 /*
   * We take this event as an reason to communicate the current position
   * correction values to all clients which have subscribed to this report.
   * Notice: In principle, we would send it more frequently, but
   * this will swamp other modules with useless details.
   */


  send_automatic_correction_update();


  if (ref == NULL)		/* regular update */
    send_automatic_map_update(robot_specifications, program_state,
			      first_x, num_x, first_y, num_y,
			      delete_previous_map); 

  else{				/* update upon request */

    partial_map = make_map_message(robot_specifications, program_state, robot_state,
				   first_x, num_x, first_y, num_y,
				   delete_previous_map);

    tcxReply(ref, "MAP_partial_map_reply", partial_map);
    
    if (partial_map != NULL){
      if (partial_map->char_values != NULL){
	free(partial_map->char_values);
	partial_map->char_values = NULL;
      }
      free(partial_map);
      partial_map = NULL;
    }
  }
}



/************************************************************************
 *
 *   NAME:         MAP_sensor_interpretation_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void MAP_sensor_interpretation_handler(TCX_REF_PTR                   ref,
				       MAP_sensor_interpretation_ptr interpret)
{
  BASE_update_status_reply_type pseudo_status;
  float diff_x, diff_y, distance, angle, new_x, new_y, new_orientation;
  float status_new_x, status_new_y;
  float corr_robot_x,corr_robot_y, corr_robot_orientation;
  float uncertainty, count;
  char  module_name[256];
  register int   i;		/* No procedure without a variable "i"! */
  unsigned char char_value;
  int maps_identical, k;
  static int local_counter = 0;
  uint tmp = 0;
  
#ifdef MAP_debug
  fprintf(stderr, "\nTCX: Received MAP_sensor_interpretation message from %s\n",
	  tcxModuleName(ref->module));
  fprintf(stderr, "MAP_sensor_interpretation_handler: Sensor: %g %g %g\n",
          interpret->robot_x, interpret->robot_y, interpret->robot_orientation);
  fprintf(stderr, "MAP_sensor_interpretation_handler: Origin: %g %g %g\n",
	  interpret->origin_x, interpret->origin_y, interpret->origin_orientation);
  fprintf(stderr, "MAP_sensor_interpretation_handler: Raw: %g %g %g\n",
          interpret->raw_robot_x, interpret->raw_robot_y, interpret->raw_robot_orientation);
  fprintf(stderr, "MAP_sensor_interpretation_handler: Robot: %s\n\n",
          interpret->robot_name);
#endif

  if (interpret->resolution != robot_specifications->resolution){
    fprintf(stderr, "ERROR: Resolution mismatch. Map ignored.\n");
  }

  else if (interpret->map_number <= 0 ||
	   interpret->map_number >= NUM_GLOBAL_MAPS)
    fprintf(stderr, "ERROR: map_number must be between 1 and %d\n",
	    NUM_GLOBAL_MAPS-1);
  
  else{

    something_happened = 1;	/* general flag, indicates that a TCX message
				 * was processed. */


    local_counter++;
    
    /*
     * If we are *not* connected to BASE, which will not be the case
     * when running with the robot, we will take the position data in this
     * message for granted. This should only affect the display stuff.
     */
    
    if (!program_state->tcx_base_connected){
      pseudo_status.pos_x               = interpret->robot_x;
      pseudo_status.pos_y               = interpret->robot_y;
      pseudo_status.orientation         = interpret->robot_orientation;
      pseudo_status.trans_current_speed = interpret->translational_speed;
      pseudo_status.rot_current_speed   = interpret->rotational_speed;
      BASE_update_status_reply_handler(NULL, &pseudo_status);
    }

    if (interpret->num_sensor_values_enclosed > 0){
      if (robot_specifications->data_logging &&
	  !log_iop)		/* file not yet open */
	if ((log_iop = fopen(MAP_LOG_NAME, "w")) == 0){
	  fprintf(log_iop, 
		  "WARNING: Could not open log file %s. No logging.\n",
		  MAP_LOG_NAME);
	  robot_specifications->data_logging = 0; /* never try again */
	  log_iop = NULL;
	}
      if (log_iop){
	new_orientation = interpret->robot_orientation;
	for (;new_orientation >   180.0;) new_orientation -= 360.0;
	for (;new_orientation <= -180.0;) new_orientation += 360.0;
	fprintf (log_iop,"#ROBOT %f %f %f\n\n",
		 interpret->robot_x, interpret->robot_y, new_orientation);
	fprintf(log_iop,"#SONAR %d:", interpret->num_sensor_values_enclosed);
	for (i = 0; i != interpret->num_sensor_values_enclosed; i++)
	  fprintf(log_iop, " %f", interpret->sensor_values[i]);
	fprintf(log_iop, "\n\n");
      }
    }


    /*==============================================================
     *              HERE IT ALL BEGINS...
     *==============================================================
     *
     * First we have to make sure that
     * there is no previous reading in memory that needs to be mapped.
     */

    if (program_state->map_update_pending)
      update_internal_map(0, robot_specifications, program_state,
			  robot_state);
  
    /* 
     * We received a new local map. Remove previous local map.
     */

    if (local_map == local_smooth_map)
      maps_identical = 1;
    else
      maps_identical = 0;
    
    if (local_map != NULL){
      free(local_map);
      local_map = NULL;
    }
    if (local_smooth_map != NULL){
      if (!maps_identical)
	free(local_smooth_map);
      local_smooth_map = NULL;
    }
    if (local_active != NULL){
      free(local_active);
      local_active = NULL;
    }

    /* 
     * and accept the new local map as the current local map.
     */
    tmp = (int)(interpret->size_x * interpret->size_y);


    local_map    = (float *) (calloc(tmp, sizeof(float)));
    local_active = (int *) (calloc(tmp,sizeof(int)));
  
    for (i = 0; i != (interpret->size_x * interpret->size_y); i++){
      char_value = interpret->char_likelihoods[i];
      if (char_value == (unsigned char) 0)
	local_active[i] = 0;
      else if (char_value == (unsigned char) 255){
	local_active[i] = 99;	/* special value, indicates that we firmly
				 * believe this */
	local_map[i]    = 1.0;
      }
      else{
	local_active[i] = 1;
	local_map[i]    = ((float) (char_value-1)) / 253.0;
      }
    }

    /* 
     * compute smoothed map
     */
    
    if (robot_specifications->smooth_radius <= 1 ||
	interpret->delete_previous_map)	/* no smoothing for new maps */
      local_smooth_map = local_map;
    
    else{
    
      local_smooth_map    = (float *)
	(calloc((interpret->size_x * interpret->size_y), sizeof(float)));
    
      smooth_map(local_map, local_active, local_smooth_map, 
		 interpret->size_x, interpret->size_y, 
		 robot_specifications->smooth_radius);
    }
  

    /* 
     * update map origin and size
     */

    robot_specifications->local_map_origin_x = interpret->origin_x;
    robot_specifications->local_map_origin_y = interpret->origin_y;
    robot_specifications->local_map_origin_orientation
      = interpret->origin_orientation;
    if (interpret->map_number >= NUM_GLOBAL_MAPS ||
	interpret->map_number < 0)
      robot_specifications->local_map_number   = 0;
    else
      robot_specifications->local_map_number   = interpret->map_number;

    robot_specifications->local_map_dim_x = interpret->size_x;
    robot_specifications->local_map_dim_y = interpret->size_y;

    /*
     * clear existing map if selected
     */
    if (robot_specifications->map_erasing_period > 0 &&
	local_counter % robot_specifications->map_erasing_period == 0){
      log_map(0, robot_state, program_state, robot_specifications);
      clear_maps(-1, robot_state, program_state, robot_specifications);
      /*fprintf(stderr, "Maps cleared.\n");*/
    }


    if (interpret->delete_previous_map)
      clear_maps(interpret->map_number, robot_state, program_state,
		 robot_specifications);


 
    /* 
     *  Apply corrections to the status report robot position data
     */
    
    if (interpret->map_number != CAD_MAP_NUMBER){
      /*fprintf(stderr, "Current corr: %g %g %g\n",
	      robot_state->correction_parameter_x,
	      robot_state->correction_parameter_y,
	      robot_state->correction_parameter_angle);*/
	      
      if (1 || !interpret->position_corrected_flag) /*!*/
	compute_forward_correction(interpret->robot_x,
				   interpret->robot_y,
				   90.0 - interpret->robot_orientation,
				   robot_state->correction_parameter_x,
				   robot_state->correction_parameter_y,
				   robot_state->correction_parameter_angle,
				   robot_state->correction_type,
				   &corr_robot_x,
				   &corr_robot_y, 
				   &corr_robot_orientation);
      else
	compute_forward_correction(interpret->robot_x,
				   interpret->robot_y,
				   interpret->robot_orientation,
				   corr_x2,
				   corr_y2,
				   corr_orientation2,
				   corr_type2,
				   &corr_robot_x,
				   &corr_robot_y, 
				   &corr_robot_orientation);


    }
    else{			/* WARNING: CAD-maps are sent in
				 * corrected coordinates */
      corr_robot_x = interpret->robot_x;
      corr_robot_y = interpret->robot_y;
      corr_robot_orientation = interpret->robot_orientation;
    }
    
    /*
     * Add the new point to the path.
     */


    k = find_name(interpret->robot_name);
    if (n_path_entries < MAX_N_PATH_ENTRIES &&
	k >= 0 &&
	(corr_robot_x != robot_state->sensor_org_x 
	 || corr_robot_y != robot_state->sensor_org_y)){
      path[n_path_entries][0] = corr_robot_x;
      path[n_path_entries][1] = corr_robot_y;
      path[n_path_entries][2] = corr_robot_orientation;
      path[n_path_entries][3] = k;
      n_path_entries++;
    }
    if (program_state->graphics_initialized &&
	!program_state->tcx_base_connected &&
	k >= 0){
      G_add_marker(PATH[k], robot_state->x, robot_state->y, 0);
    }
    
    /*
     * And now update the internal belief.
     */
  
    robot_state->sensor_org_x = robot_state->sensor_x
      = robot_state->sensor_best_x = corr_robot_x;
    robot_state->sensor_org_y = robot_state->sensor_y 
      = robot_state->sensor_best_y = corr_robot_y;
    robot_state->sensor_org_orientation
      = robot_state->sensor_orientation = robot_state->sensor_best_orientation
      = corr_robot_orientation;

    robot_state->sensor_translational_speed = interpret->translational_speed;
    robot_state->sensor_rotational_speed    = interpret->rotational_speed;

    send_automatic_position_update(corr_robot_x,
				   corr_robot_y,
				   corr_robot_orientation,
				   interpret->raw_robot_x, 
				   interpret->raw_robot_y, 
				   interpret->raw_robot_orientation,
				   interpret->robot_name);


    
#ifdef MAP_debug
    fprintf(stderr, "MAP_sensor_interpretation_handler: \n"
	    " \t[corr: %6.4f params: %6.4f %6.4f %6.4f %d]\n",/*!*/
	    robot_state->sensor_best_fit,
	    robot_state->correction_parameter_x,
	    robot_state->correction_parameter_y,
	    robot_state->correction_parameter_angle,
	    robot_state->correction_type);
#endif

    robot_state->sensor_best_fit = -99999.0;

    robot_state->last_change_x = 0.0;  
    robot_state->last_change_y = 0.0;  
    robot_state->last_change_orientation = 0.0;  


    /* 
     * compute an "uncertainty" term, which is related to the
     * speed of the robot.
     *
     * The whole formula is somehow ad hoc, and I am not really sure the
     * parameters really matter. In essence, I want to trust in the robot's
     * wheels lesser when it moved faster
     */

    uncertainty = (fabs(robot_state->sensor_rotational_speed ) +
		   2.0 * fabs(robot_state->sensor_translational_speed)) * 0.01;
    if (uncertainty > 1.0)
      uncertainty = 1.0;
    if (uncertainty > robot_state->sensor_uncertainty)
      robot_state->sensor_uncertainty = uncertainty;
    else
      robot_state->sensor_uncertainty = (0.8 * robot_state->sensor_uncertainty)
	+ (0.1 * uncertainty);
    if (robot_state->sensor_uncertainty < 0.001)
      robot_state->sensor_uncertainty = 0.001;

  
    /*
     * shift the global map appropriately
     */
    
    autoshift_display(program_state, robot_specifications,
		      robot_state->x, robot_state->y); 


    autoshift_display(program_state, robot_specifications,
		      corr_robot_x, corr_robot_y);
    autoshift_display(program_state, robot_specifications,
		      corr_robot_x +
		      (robot_specifications->local_map_dim_x
		       * robot_specifications->resolution),
		      corr_robot_y +
		      (robot_specifications->local_map_dim_y
			 * robot_specifications->resolution));
    
    /*
     * At some point we will have to map the local map into the global map.
     * If we don't do any position estimation, let's do this now.
     */
  
  
    if (interpret->delete_previous_map){
      update_internal_map(1, robot_specifications, program_state,
			  robot_state);
    }
    
    else if (!robot_specifications->do_position_correction ||
	     program_state->tcx_localize_connected)
      update_internal_map(0, robot_specifications, program_state,
			  robot_state);

  
    /*
     * Otherwise New search, if we do search.
     * The map_update_pending flag indicates that we haven't 
     * written the adjusted local map into global memory.
     */

    else{
      robot_specifications->niterations_in_search = 0;
      program_state->map_update_pending           = 1;
    }


    /*
     * Make sure the display says who sent the sensor values
     */

    if (program_state->graphics_initialized){
      strcpy(module_name, tcxModuleName(ref->module));
      for (i = 0; i != 256; i++)
	if (module_name[i] == '_')
	  module_name[i] = '\0';
      G_set_new_text(LOCAL_MAPVALUES,  module_name, 0);
      G_set_new_text(LOCAL_BACKGROUND, module_name, 0);
    }  
  
    /* 
     * Now, finally, the new sensor values and the path are displayed.
     */
  
    if (program_state->graphics_initialized){
      G_display_matrix(GLOBAL_MAPVALUES);
      /*G_display_markers(PATH);*/
      G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y,
		      robot_state->orientation, 0, NULL);
      if (program_state->regular_local_map_display){
	G_display_switch(LOCAL_BACKGROUND, 0);
	G_change_matrix(LOCAL_MAPVALUES, local_smooth_map, local_active,
			robot_specifications->local_map_dim_x,
			robot_specifications->local_map_dim_y);
	G_display_matrix(LOCAL_MAPVALUES);
      }
    }

  
    /* 
     * And free the message memory. 
     *
     * ...there is the danger that tcxFree frees also the local map,
     * despite the
     * fact that we just set the pointers to NULL. If this is the case,
     * weird things will happen in the global map, and we'll have to use
     * free instead of tcxFree.
     *
     * Some days later: Seems this doesn't happen! Cannot report any trouble.
     * Sebastian
     */
    /*
       fprintf(stderr,
       "MAP_sensor_interpretation_handler: tcxFree not called - mem leak\n");
       */
  }
#ifdef MAP_debug
  fprintf(stderr, "MAP_sensor_interpretation_handler: complete\n");
  fflush(stderr);
#endif
  if (interpret != NULL){
    tcxFree("MAP_sensor_interpretation", interpret);
    interpret = NULL;
  }
  
#ifdef MAP_debug
  fprintf(stderr, "MAP_sensor_interpretation_handler: complete\n");
  fflush(stderr);
#endif
}  






/************************************************************************
 *
 *   NAME:         MAP_saw_a_wall_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

static int similar_wall_counter[NUM_GLOBAL_MAPS];
static float cumul_wall_angle[NUM_GLOBAL_MAPS];
static int n_confidence_walls[NUM_GLOBAL_MAPS];
static int local_values_initialized = 0;




void MAP_saw_a_wall_handler(TCX_REF_PTR        ref,
			    MAP_saw_a_wall_ptr wall_msg)
{
  float wall_correction;
  /* float uncorr_robot_x, uncorr_robot_y, uncorr_robot_orientation; */
  float corr_robot_at_wall_x, corr_robot_at_wall_y;
  float corr_robot_at_wall_orientation;
  float average_angle;
  char  angle_txt[128];
  int i;
  float change_rate;
  float BASE_wall_correction;

#ifdef MAP_debug
  fprintf(stderr, "\nTCX: Received MAP_saw_a_wall message from %s: %g\n",
	  tcxModuleName(ref->module), wall_msg->wall_angle);
#endif

  /*
   * Ignore, if we are not interested in estimating a robot position
   */

  if (!robot_specifications->do_position_correction ||
      program_state->tcx_localize_connected)
    return;

  /*
   * Initialize values, if we haven't done that yet
   */

  if (!local_values_initialized){
    for (i = 0; i != NUM_GLOBAL_MAPS; i++){
      similar_wall_counter[i] = 0;
      cumul_wall_angle[i] = 0.0;
      n_confidence_walls[i] = 
	robot_specifications->number_subsequent_adjacent_walls;
    }
    local_values_initialized = 1;
    fprintf(stderr, "!Oops!");/*!*/
  }

  /*
   * Save pending updates here!
   */


  /*
   * if (program_state->map_update_pending)
   * update_internal_map(robot_specifications, program_state,
   * robot_state);
   */

  /*
   * Compute uncorrected robot pos
   */
  
  /*    
     compute_backward_correction(robot_state->x,
     robot_state->y,
     robot_state->orientation,
     robot_state->correction_parameter_x,
     robot_state->correction_parameter_y,
     robot_state->correction_parameter_angle,
     robot_state->correction_type,
     &uncorr_robot_x,
     &uncorr_robot_y, 
     &uncorr_robot_orientation);
     */

  /* 
   *  Apply corrections to the message position data
   */

  compute_forward_correction(wall_msg->robot_x,
			     wall_msg->robot_y,
			     wall_msg->robot_orientation,
			     robot_state->correction_parameter_x,
			     robot_state->correction_parameter_y,
			     robot_state->correction_parameter_angle,
			     robot_state->correction_type,
			     &corr_robot_at_wall_x,
			     &corr_robot_at_wall_y, 
			     &corr_robot_at_wall_orientation);
    

  /*
   * Correct the parameters
   */
  
  wall_correction = wall_msg->wall_angle + corr_robot_at_wall_orientation;
  if (robot_state->map_orientation_defined)
    wall_correction -=
      robot_state->map_orientation; 
  
  /* fprintf(stderr, " %6.2f\n", wall_correction); */


  for(; wall_correction < -45.0;)
    wall_correction += 90.0; /* we consider only parameters in -45..45 */
  for(; wall_correction > 45.0;)
    wall_correction -= 90.0; /* we consider only parameters in -45..45 */
#ifdef MAP_debug
  fprintf(stderr, "MAP_saw_a_wall_handler: wall_correction = %6.2f\n", wall_correction);
#endif
  if (!robot_state->map_orientation_defined){
    /*
     * We have not yet found our reference angle
     */
#ifdef MAP_debug
    fprintf(stderr, "MAP_saw_a_wall_handler: similar_wall_counter[%d] = %d\n",
        program_state->actual_map, similar_wall_counter[program_state->actual_map]);
#endif
    if (similar_wall_counter[program_state->actual_map] >=
	n_confidence_walls[program_state->actual_map]){
      /*
       * But we are about to find it :-)
       */

      average_angle = cumul_wall_angle[program_state->actual_map] / 
	((float) similar_wall_counter[program_state->actual_map]);

      robot_state->map_orientation = 
	average_angle;

      robot_state->map_orientation_defined = 1;

      fprintf(stderr, "\n\t+===========================================+\n");
      fprintf(stderr, "\t+===   new orientation: %6.4f          ====+\n",
	      average_angle);
      fprintf(stderr, "\t+===========================================+\n");

      sprintf(angle_txt, "(walls: %5.1f)", 
	      robot_state->map_orientation);
      G_set_new_text(LINE_ANGLE_BUTTON, angle_txt, 1);
      G_display_switch(LINE_ANGLE_BUTTON, 1);
      cumul_wall_angle[program_state->actual_map] = 0.0;
      similar_wall_counter[program_state->actual_map] = 0;
#ifndef FGAN
      putc(7,stderr);
#endif
    }
    else{
      /*
       * Make sure that the new angle is "similar" to previously found ones
       */
      if (similar_wall_counter[program_state->actual_map]!=0) {
	average_angle = cumul_wall_angle[program_state->actual_map]
	  / ((float) similar_wall_counter[program_state->actual_map]);
      }
      else {
	average_angle = 0.0;
      }       
 
      if (similar_wall_counter[program_state->actual_map] == 0 || 
	  fabs(average_angle - wall_correction)
	  <= robot_specifications->wall_error_threshold){
      /*
       * 
       */
	cumul_wall_angle[program_state->actual_map] += wall_correction;
	similar_wall_counter[program_state->actual_map]++;
      }
      else{
      /*
       * 
       */
	cumul_wall_angle[program_state->actual_map] = 0.0;
	similar_wall_counter[program_state->actual_map] = 0;
      }
    }
  }

  if (robot_state->map_orientation_defined){

    if (robot_specifications->wall_error_threshold <= 0.0)
      change_rate = 0.0;
    else if (fabs(wall_correction) 
	     < robot_specifications->wall_error_threshold)
      change_rate = 
	(fabs(wall_correction) - robot_specifications->wall_error_threshold)
	  /  robot_specifications->wall_error_threshold;
    else
      change_rate = 0.01;

    /*
     * ...this better be a Kalman filter at some point.
     */
    wall_correction *= 
      change_rate * change_rate * robot_specifications->wall_weight;
    
    update_correction_parameters(corr_robot_at_wall_x,
				 corr_robot_at_wall_y, 
				 corr_robot_at_wall_orientation,
				 0.0,
				 0.0,
				 -wall_correction,
				 &(robot_state->correction_parameter_x),
				 &(robot_state->correction_parameter_y),
				 &(robot_state->correction_parameter_angle),
				 &(robot_state->correction_type));
    
    
    
    /*
     * And generate the new robot pos
     */
    
    robot_state->orientation -= wall_correction;
    for (;robot_state->orientation >  180.0;)
      robot_state->orientation -= 360.0;
    for (;robot_state->orientation < -180.0;) 
      robot_state->orientation += 360.0;


    if (program_state->map_update_pending){
      robot_state->sensor_org_orientation  -= wall_correction;
      for (;robot_state->sensor_org_orientation >  180.0;)
	robot_state->sensor_org_orientation -= 360.0;
      for (;robot_state->sensor_org_orientation < -180.0;) 
	robot_state->sensor_org_orientation += 360.0;

      robot_state->sensor_orientation      -= wall_correction;
      for (;robot_state->sensor_orientation >  180.0;)
	robot_state->sensor_orientation -= 360.0;
      for (;robot_state->sensor_orientation < -180.0;) 
	robot_state->sensor_orientation += 360.0;

      robot_state->sensor_best_orientation -= wall_correction;
      for (;robot_state->sensor_best_orientation >  180.0;)
	robot_state->sensor_best_orientation -= 360.0;
      for (;robot_state->sensor_best_orientation < -180.0;) 
	robot_state->sensor_best_orientation += 360.0;
    }
    /*
       compute_forward_correction(uncorr_robot_x,
       uncorr_robot_y, 
       uncorr_robot_orientation,
       robot_state->correction_parameter_x,
       robot_state->correction_parameter_y,
       robot_state->correction_parameter_angle,
       robot_state->correction_type,
       &robot_state->x,
       &robot_state->y,
       &robot_state->orientation);
       */

    /* 
     * Tell base the great news
     */

#ifdef junk    
    BASE_wall_correction = wall_correction;
    for(; wall_correction < 0.0;)
      wall_correction += 90.0; /* BASE, however, needs valies in 0..90 */
    for(; wall_correction >= 90.0;)
      wall_correction -= 90.0; 
    
    if(program_state->tcx_base_connected)
      tcxSendMsg(BASE, "BASE_notify_wall_orientation", 
		 &BASE_wall_correction);
#endif
    /* 
     * Yup, display!
     */

    /*

       if (program_state->graphics_initialized)
       G_display_robot(GLOBAL_ROBOT, robot_state->x, robot_state->y,
       robot_state->orientation, 0, NULL);
       */
  }
  
  if (wall_msg != NULL){
    tcxFree("MAP_saw_a_wall", wall_msg);
    wall_msg = NULL;
  }
#ifdef MAP_debug
  fprintf(stderr, "MAP_saw_a_wall_handler: complete\n");
#endif
}






/************************************************************************
 *
 *   NAME:         MAP_label_grid_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void MAP_label_grid_handler(TCX_REF_PTR         ref,
			     MAP_label_grid_ptr info_msg)
{
  int x, y, global_x, global_y;
  int info_msg_index, global_map_index;
  unsigned char char_value;

#ifdef MAP_debug
  fprintf(stderr, "TCX: Received MAP_label_grid message from %s\n",
	  tcxModuleName(ref->module));
#endif

  /* currently, we just insert the doors into the map */


  /* for (x = 0; x < info_msg->size_y * info_msg->size_y; x++)
     printf("%d", (int) info_msg->labels[x]);
     */
  
  for (x = 0, global_x = info_msg->first_x 
       + robot_specifications->autoshifted_int_x;
       x != info_msg->size_x; x++, global_x++)
    for (y = 0, global_y = info_msg->first_y 
	 + robot_specifications->autoshifted_int_y;
	 y != info_msg->size_y; y++, global_y++){
      info_msg_index = x * info_msg->size_y + y;
      global_map_index  = global_x * 
	robot_specifications->global_map_dim_y + global_y;
      if (global_x >= 0 && global_y >= 0 &&
	  global_x < robot_specifications->global_map_dim_x &&
	  global_y < robot_specifications->global_map_dim_y){

	/* ====== set label =========== */
	char_value = info_msg->labels[info_msg_index];
	if (char_value == (unsigned char) CELL_TYPE_DOOR){
	  global_label[global_map_index] = (unsigned char) 3; /* door */
	  global_map[global_map_index] = robot_specifications->prior;/*!*/
	}
      }
    }


  if (info_msg != NULL){
    tcxFree("MAP_label_grid", info_msg);
    info_msg = NULL;
  }
#ifdef MAP_debug
  fprintf(stderr, "handler done.\n");
#endif
}



/************************************************************************
 *
 *   NAME:         MAP_enable_map_update_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void MAP_enable_map_update_handler(TCX_REF_PTR         ref,
			   void *data)
{

#ifdef MAP_debug
  fprintf(stderr, "TCX: Received MAP_enable_map_update message from %s\n",
	  tcxModuleName(ref->module));
#endif

  program_state->map_update_on = 0;
  G_display_switch(MAP_UPDATE_ENABLE_BUTTON, program_state->map_update_on);
}


/************************************************************************
 *
 *   NAME:         MAP_disable_map_update_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void MAP_disable_map_update_handler(TCX_REF_PTR         ref,
			   void *data)
{

#ifdef MAP_debug
  fprintf(stderr, "TCX: Received MAP_disable_map_update message from %s\n",
	  tcxModuleName(ref->module));
#endif
  
  program_state->map_update_on = 0;
  G_display_switch(MAP_UPDATE_ENABLE_BUTTON, program_state->map_update_on);
}


/************************************************************************
 *
 *   NAME:         MAP_clear_all_sensor_maps_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void MAP_clear_all_sensor_maps_handler(TCX_REF_PTR         ref,
			   void *data)
{
#ifdef MAP_debug
  fprintf(stderr, "TCX: Received MAP_clear_all_sensor_maps message from %s\n",
	  tcxModuleName(ref->module));
#endif
  clear_maps(-2, robot_state, program_state, robot_specifications);
  compute_map_0(robot_specifications, program_state, robot_state);
}


/************************************************************************
 *
 *   Name:         MAP_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void MAP_close_handler(char *name, TCX_MODULE_PTR module)
{
#ifdef MAP_debug
  fprintf(stderr, "MAP: closed connection detected: %s\n", name);
#endif
  
  remove_auto_update_module(module);
  
  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    if (log_iop) fclose(log_iop);
    exit(0);
  }
  else if (!strcmp(name, "BASE")){ /* BASE shut down */
    if (program_state->graphics_initialized)
      G_display_switch(BASE_CONNECTED_BUTTON, 0);
    program_state->tcx_base_connected = 0;
  }
  else if (!strcmp(name, "LOCALIZE")){ /* LOCALIZE shut down */
    if (program_state->graphics_initialized)
      G_display_switch(LOCALIZE_CONNECTED_BUTTON, 0);
    program_state->tcx_localize_connected = 0;
  }
  else if (!strcmp(name, "LASERINT-SERVER")){ /* LOCALIZE shut down */
    num_robots = 0;    
  }
#ifdef MAP_debug
  fprintf(stderr, "handler done.\n");
#endif
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



TCX_REG_HND_TYPE MAP_handler_array[] = {
  {"MAP_register_auto_update", "MAP_register_auto_update_handler",
     MAP_register_auto_update_handler, TCX_RECV_ALL, NULL},
  {"MAP_dump", "MAP_dump_handler",
     MAP_dump_handler, TCX_RECV_ALL, NULL},
  {"MAP_quit", "MAP_quit_handler",
     MAP_quit_handler, TCX_RECV_ALL, NULL},
  {"MAP_partial_map_query", "MAP_partial_map_query_handler",
     MAP_partial_map_query_handler, TCX_RECV_ALL, NULL},
  {"MAP_correction_parameters_query", 
     "MAP_correction_parameters_query_handler",
     MAP_correction_parameters_query_handler, TCX_RECV_ALL, NULL},
  {"MAP_correction_parameters_inform", 
     "MAP_correction_parameters_inform_handler",
     MAP_correction_parameters_inform_handler, TCX_RECV_ALL, NULL},
  {"MAP_sensor_interpretation", "MAP_sensor_interpretation_handler",
     MAP_sensor_interpretation_handler, TCX_RECV_ALL, NULL},
  {"MAP_saw_a_wall", "MAP_saw_a_wall_handler",
     MAP_saw_a_wall_handler, TCX_RECV_ALL, NULL},
  {"MAP_label_grid", "MAP_label_grid_handler",
     MAP_label_grid_handler, TCX_RECV_ALL, NULL},
  {"MAP_enable_map_update", "MAP_enable_map_update_handler",
     MAP_enable_map_update_handler, TCX_RECV_ALL, NULL},
  {"MAP_disable_map_update", "MAP_disable_map_update_handler",
     MAP_disable_map_update_handler, TCX_RECV_ALL, NULL},
  {"MAP_clear_all_sensor_maps", "MAP_clear_all_sensor_maps_handler",
     MAP_clear_all_sensor_maps_handler, TCX_RECV_ALL, NULL}
};




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

static struct timeval last_attempt_connect_BASE = {0, 0};

void connect_to_BASE(PROGRAM_STATE_PTR program_state)
{
  BASE_register_auto_update_type data;
  struct timeval current_time;

  if (!program_state->use_tcx)
    return;


  if(!program_state->tcx_base_connected){

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
      fprintf(stderr, "tcx: COLLI connection established.\n");

      if (program_state->graphics_initialized)
	G_display_switch(BASE_CONNECTED_BUTTON, 1);

      data.subscribe_status_report = 4;
      data.subscribe_sonar_report  = 0;
      data.subscribe_colli_report  = 0;
      data.subscribe_laser_report  = 0;
      data.subscribe_ir_report     = 0;
      
      tcxSendMsg(BASE, "BASE_register_auto_update", &data);

      program_state->tcx_base_connected = 1;
    }
    else
      if (program_state->graphics_initialized)
	G_display_switch(BASE_CONNECTED_BUTTON, 0);
  }
}

/************************************************************************
 *
 *   NAME:         connect_to_LOCALIZE
 *                 
 *   FUNCTION:     checks, and connects to LOCALIZE, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

static struct timeval last_attempt_connect_LOCALIZE = {0, 0};

void connect_to_LOCALIZE(PROGRAM_STATE_PTR program_state)
{
#ifdef UNIBONN
  LOCALIZE_register_auto_update_type data;
  struct timeval current_time;


  /* Fox: no more correction parameters from localize. */
  {
    static int firstTime = TRUE;

    if ( firstTime) {
      fprintf(stderr, "We don't register for localize updates any more,");
      fprintf(stderr, " since the correction parameters are provided by laserint.\n");
    }
    firstTime = FALSE;
    return;
  }

  if (!program_state->use_tcx)
    return;


  if(!program_state->tcx_localize_connected){

    gettimeofday(&current_time, NULL);
    if (current_time.tv_sec < last_attempt_connect_LOCALIZE.tv_sec + 5
	|| (current_time.tv_sec == last_attempt_connect_LOCALIZE.tv_sec + 5 &&
	    current_time.tv_usec < last_attempt_connect_LOCALIZE.tv_usec))
      return;
    
    last_attempt_connect_LOCALIZE.tv_sec  = current_time.tv_sec;
    last_attempt_connect_LOCALIZE.tv_usec = current_time.tv_usec;
    

    LOCALIZE = tcxConnectOptional(TCX_LOCALIZE_MODULE_NAME); /* checks,
							      * but does 
							      * not wait */

    if (LOCALIZE != NULL){
      fprintf(stderr, "tcx: LOCALIZE connection established.\n");

      if (program_state->graphics_initialized)
	G_display_switch(LOCALIZE_CONNECTED_BUTTON, 1);

      data.subscribe = 1;
      
      tcxSendMsg(LOCALIZE, "LOCALIZE_register_auto_update", &data);

      program_state->tcx_localize_connected = 1;

      
      if (program_state->map_update_pending) /* There is still an update
					      * to be done. Then do it.
					      * Apparently, the search 
					      * is over.
					      */
	update_internal_map(0, robot_specifications, program_state,
			    robot_state);

    }
    else
      if (program_state->graphics_initialized)
	G_display_switch(LOCALIZE_CONNECTED_BUTTON, 0);
  }
#endif
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
  
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    BASE_messages,
#ifdef UNIBONN
    LOCALIZE_messages,
#endif
    SONAR_messages,
    LASER_messages,
    MAP_messages
    };
  
  if (!program_state->use_tcx)
    return;

  
  
  tcxMachine = bRobot.TCXHOST;


  if (tcxMachine != NULL){
    fprintf(stderr, "Connecting to TCX...");
    fflush(stderr);


    tcxInitialize(TCX_MAP_MODULE_NAME, (char *) tcxMachine);
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
    
    
    tcxRegisterHandlers(MAP_handler_array, 
			sizeof(MAP_handler_array)
			/ sizeof(TCX_REG_HND_TYPE));
    tcxRegisterHandlers(BASE_reply_handler_array,
			sizeof(BASE_reply_handler_array)
			/ sizeof(TCX_REG_HND_TYPE));
#ifdef UNIBONN
    tcxRegisterHandlers(LOCALIZE_reply_handler_array,
			sizeof(LOCALIZE_reply_handler_array)
			/ sizeof(TCX_REG_HND_TYPE));
#endif
    tcxRegisterHandlers(SONAR_reply_handler_array,
			sizeof(SONAR_reply_handler_array)
			/ sizeof(TCX_REG_HND_TYPE));
    
    tcxRegisterHandlers(LASER_reply_handler_array,
			sizeof(LASER_reply_handler_array)
			/ sizeof(TCX_REG_HND_TYPE));
    
    tcxRegisterCloseHnd(MAP_close_handler);
    
    
    program_state->tcx_initialized = 1;
    
    connect_to_BASE(program_state);
    connect_to_LOCALIZE(program_state);
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
  static int i = 0;
  if (!i){
    fprintf(stderr, "\n\n\t???????? TCX: Received aBASE_robot_position_reply message.\n\n");
#ifndef FGAN
    putc(7,stderr);
#endif
  }
  i = 1;

  tcxFree("BASE_robot_position_reply", pos);
#ifdef MAP_debug
  fprintf(stderr, "handler done3.\n");
#endif
}

void SONAR_sonar_reply_handler(TCX_REF_PTR           ref,
			       SONAR_sonar_reply_ptr sonar)
{
  static int i = 0;
  if (!i)
    fprintf(stderr, "\n\n\t???????? TCX: Received a SONAR_sonar_reply message.\n\n");
  i = 1;

  tcxFree("SONAR_sonar_reply", sonar);
#ifdef MAP_debug
  fprintf(stderr, "handler done2.\n");
#endif
}




void LASER_laser_reply_handler(TCX_REF_PTR                ref,
			       LASER_laser_reply_ptr      data)
{
  static int i = 0;
  if (!i)
    fprintf(stderr, "\n\n\t???????? TCX: Received a LASER_laser_reply message.\n\n");
  i++;
  
  tcxFree("LASER_laser_reply", data);
}



void BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
					BASE_action_executed_reply_ptr data)
{
;
#ifdef MAP_debug
fprintf(stderr, "handler done1.\n");
#endif
}



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/
