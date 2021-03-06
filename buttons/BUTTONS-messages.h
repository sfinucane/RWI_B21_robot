
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/buttons/BUTTONS-messages.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:44:40 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: BUTTONS-messages.h,v $
 * Revision 1.1  2002/09/14 15:44:40  rstone
 * *** empty log message ***
 *
 * Revision 1.7  1999/04/27 09:07:15  schulz
 * Added tcx.h to includes
 *
 * Revision 1.6  1997/07/30 02:51:21  thrun
 * Oops. Something went wrong.
 * This version has more consistent status update, and a message
 * for simulating button presses
 *
 * Revision 1.5  1997/07/04 17:26:02  swa
 * Added lib support for buttons. Renamed the executable buttonServer to be
 * more consistent. No root permissions are necessary to run the buttonServer.
 * Renamed tons of things to be backward-compatible.
 *
 * Revision 1.4  1997/06/29 22:00:35  thrun
 * Changed BUTTONS-messages to handle client/server.
 *
 * Revision 1.3  1997/06/29 00:17:27  thrun
 * Replaced inb and outb with functions that no longer require root priviliges
 * for ioperm(). Instead we use Linux's device /dev/port. /dev/port must have
 * rw permissions for all users. Thou shalt not include header files in other
 * header files. Fixed some minor "-Wall" errors.                         (swa)
 *
 * Revision 1.2  1997/05/02 10:09:37  haehnel
 * nice effect for BUTTONS
 *
 * Revision 1.1.1.1  1996/09/22 16:46:26  rhino
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



#ifndef BUTTONS_messages_defined
#define BUTTONS_messages_defined


#include "tcx.h"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* 
   Notice: when including this file, you need to have the flag

   TCX_define_variables

   be defined exactly once. This will allocate memory for the
   module pointer and the message arrays.
*/

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** TCX module name - this is a unique identifier for TCX *******/

#define TCX_BUTTONS_MODULE_NAME "BUTTONS"

void tcxRegisterCloseHnd(void (*closeHnd)());

#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR BUTTONS;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR BUTTONS;	/* otherwise: reference */

#endif

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define BUTTON_LEFT_KILL    0
#define BUTTON_RED          1
#define BUTTON_YELLOW       2
#define BUTTON_GREEN        3
#define BUTTON_BLUE         4
#define BUTTON_RIGHT_KILL   5

/**** BUTTON data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/

/*
 * The stati and commands for the lights
 */

#define BUTTON_LIGHT_STATUS_OFF                     0 /* always off */
#define BUTTON_LIGHT_STATUS_ON                      1 /* always on */
#define BUTTON_LIGHT_STATUS_FLASHING                2 /* always flashing */
#define BUTTON_LIGHT_STATUS_FLASHING_TILL_PRESSED   3 /* flashing, until
						       * someone presses the 
						       * button, then off */
#define BUTTON_LIGHT_STATUS_ON_TILL_PRESSED         4 /* on, until
						       * someone presses the 
						       * button, then off */
#define BUTTON_LIGHT_STATUS_OFF_TILL_PRESSED        5 /* off, until
						       * someone presses the 
						       * button, then oon */
#define BUTTON_LIGHT_STATUS_TOGGLE_ON               6 /* toggles upon button
						       * press, currently on */
#define BUTTON_LIGHT_STATUS_TOGGLE_OFF              7 /* toggles upon button
						       * press, currently off*/
#define BUTTON_LIGHT_STATUS_DONT_CHANGE             8 /* only command param:
						       * TCX-message will not
						       * change the status */

#define BUTTON_UNPRESSED     0	/* button not pressed */
#define BUTTON_PRESSED       1	/* button pressed */

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define BUTTONS_effect_format NULL

/*
 * set the lights
 */

typedef struct {
  int red_light_status;
  int yellow_light_status;
  int green_light_status;
  int blue_light_status;
  int left_kill_switch_light_status;
  int right_kill_switch_light_status;
} BUTTONS_set_lights_type, *BUTTONS_set_lights_ptr;
#define BUTTONS_set_lights_format "{int, int, int, int, int, int}"

/*
 * ask for a status report (not necessary if you subscribed to it)
 */

#define BUTTONS_status_query_format NULL

/*
 * reply of the status report
 */

typedef struct {
  int red_light_status;
  int yellow_light_status;
  int green_light_status;
  int blue_light_status;
  int left_kill_switch_light_status;
  int right_kill_switch_light_status;
  int red_button_changed;
  int red_button_pressed;
  int yellow_button_pressed;
  int yellow_button_changed;
  int green_button_pressed;
  int green_button_changed;
  int blue_button_pressed;
  int blue_button_changed;
} BUTTONS_status_reply_type, *BUTTONS_status_reply_ptr;
#define BUTTONS_status_reply_format "{int, int, int, int, int, int, int, int, int, int, int, int, int, int}"

typedef struct {
  int status;			/* 0=don't send, n>0 send every n-th frame */
} BUTTONS_register_auto_update_type, *BUTTONS_register_auto_update_ptr;
#define BUTTONS_register_auto_update_format "{int}"

typedef struct {
  int num;
  int status;
} BUTTONS_setButton_type, *BUTTONS_setButton_ptr;
#define BUTTONS_setButton_format "{int, int}"

#define BUTTONS_simulate_button_press_format "int"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** BUTTON commands - these are the commands/queries understood by BUTTON ****/


#ifdef TCX_define_variables		/* do this exactly once! */

#define BUTTONS_messages \
  {"BUTTONS_register_auto_update",   BUTTONS_register_auto_update_format},\
  {"BUTTONS_status_query",           BUTTONS_status_query_format},\
  {"BUTTONS_status_reply",           BUTTONS_status_reply_format},\
  {"BUTTONS_effect",                 BUTTONS_effect_format},\
  {"BUTTONS_setButton",              BUTTONS_setButton_format},\
  {"BUTTONS_set_lights",             BUTTONS_set_lights_format},\
  {"BUTTONS_simulate_button_press",  BUTTONS_simulate_button_press_format}
#endif

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS

/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with BUTTON ******/

/******* (a) Procedure headers ******/

void BUTTONS_status_reply_handler(TCX_REF_PTR              ref,
				  BUTTONS_status_reply_ptr data);

/******* (b) Handler array ******/

TCX_REG_HND_TYPE BUTTONS_reply_handler_array[] = {
  {"BUTTONS_status_reply", "BUTTONS_status_reply_handler",
    (void (*)()) BUTTONS_status_reply_handler, TCX_RECV_ALL, NULL}
};


#endif

#endif
