
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/faceServer.old/FACE.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 21:06:45 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: FACE.c,v $
 * Revision 1.1  2002/09/14 21:06:45  rstone
 * *** empty log message ***
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <bUtils.h>
#include "robot_specifications.h"
#include "tcx.h"
#include "tcxP.h"
#include "global.h"

#include "face_interface.h"

#define TCX_define_variables /* this makes sure variables are installed */
#include "FACE-messages.h"

#include "beeSoftVersion.h"
#include "librobot.h"
#include "libezx.h"


struct timeval TCX_waiting_time = {1,0};

FACE_status_update_type global_status = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, NULL};

face_state state;

#define MAX_N_AUTO_UPDATE_MODULES 100

static int n_auto_update_modules = 0; /* number of processes to whom
				      * position should be forwarded
				      * automatically upon change */


static TCX_MODULE_PTR auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];
                       /* collection of TCX-module ptrs */


static int add_auto_update_module(TCX_MODULE_PTR module) {
  int i;

  if (n_auto_update_modules >= MAX_N_AUTO_UPDATE_MODULES){
    fprintf(stderr, "ERROR: auto-module memory overflow. Request rejected.\n");
    return 0;
  }
  else
    for (i = 0; i < n_auto_update_modules; i++)
      if (auto_update_modules[i] == module){
	fprintf(stderr, "ERROR: module %s already known. Must be a bug.\n",
		tcxModuleName(module));
	return 0;
      }
  fprintf(stderr, "Add %s to auto-reply list.\n",
	  tcxModuleName(module));
  auto_update_modules[n_auto_update_modules++] = module; /* insert pointer */
  return 1;
}


static int remove_auto_update_module(TCX_MODULE_PTR module)
{     
  int i, j, found = 0;;


  for (i = 0; i < n_auto_update_modules; i++) /* search for module */
    if (auto_update_modules[i] == module){ /* if module found */
      fprintf(stderr, "Remove %s from auto-reply list.\n",
	      tcxModuleName(module));
      found++;
      n_auto_update_modules--;	/* remove that entry, one less now */
      for (j = i; j < n_auto_update_modules; j++)  
	auto_update_modules[j] = auto_update_modules[j+1]; /* shift back */
    }
  if (!found)
    fprintf(stderr, "(%s: no auto-replies removed)\n",
	    tcxModuleName(module));
  return found;
}
  
void send_automatic_status_update(void) {
  int i;
  static float prev_eye_pos = -10000.0, 
    prev_mouth_pos = -10000.0, 
    prev_brow_pos = -10000.0;
  
#ifdef FACE_debug
  struct timeval time;
  int time2;
#endif

  for (i = 0; i < n_auto_update_modules; i++){
    if (prev_eye_pos != global_status.eye0_pos ||
	prev_brow_pos != global_status.brow0_pos || 
	prev_mouth_pos != global_status.mouth0_pos)
      {
#ifdef FACE_debug
      gettimeofday(&time, NULL);
      time2 = (time.tv_sec % 3600) * 10 + time.tv_usec/100000;

      fprintf(stderr, "%s:%6d:%s() - [%6d]  %s.\n",
	      __FILE__, __LINE__, __FUNCTION__, time2,
	      tcxModuleName(auto_update_modules[i]));
#endif
      tcxSendMsg(auto_update_modules[i], "FACE_status_update",
		 &global_status);
      prev_eye_pos = global_status.eye0_pos;
      prev_brow_pos = global_status.brow0_pos;
      prev_mouth_pos = global_status.mouth0_pos;
    }
  }
}

void FACE_init_query_handler(TCX_REF_PTR      ref,
			        int             *data) {
  int return_code = 0;

#ifdef FACE_debug
  fprintf(stderr, "Received a FACE_init_query message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  if (*data) add_auto_update_module(ref->module);

  tcxReply(ref, "FACE_init_reply", &return_code);

  tcxSendMsg(ref->module, "FACE_status_update", &global_status);

}


void FACE_state_query_handler(TCX_REF_PTR ref, void *data) {
  FACE_state pos;

#ifdef FACE_debug
  fprintf(stderr, "Received a FACE_state_query message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  pos.eyes = state.eye0_pos;
  pos.brows = state.brow0_pos;
  pos.mouth = state.mouth0_pos;

  tcxReply(ref, "FACE_state_reply", &pos);
}

void FACE_set_eyes_handler(TCX_REF_PTR ref, int *data)
{

#ifdef FACE_debug
  fprintf(stderr, "Received a FACE_set_eyes message: %d from %s.\n",
	  *data, tcxModuleName(ref->module));
#endif

  FaceSetEyes(&state, *data);

  send_automatic_status_update();
  tcxFree("FACE_set_eyes", data);
}


void FACE_set_brows_handler(TCX_REF_PTR ref, int *data) {
#ifdef FACE_debug
  fprintf(stderr, "Received a FACE_set_brows message: %d from %s.\n",
	  *data, tcxModuleName(ref->module));
#endif

  FaceSetBrows(&state, *data);

  send_automatic_status_update();
  tcxFree("FACE_set_brows", data);
}


void FACE_set_mouth_handler(TCX_REF_PTR ref, int *data) {

#ifdef FACE_debug
  fprintf(stderr, "Received a FACE_set_eyes message: %d from %s.\n",
	  *data, tcxModuleName(ref->module));
#endif

  FaceSetMouth(&state, *data);

  send_automatic_status_update();
  tcxFree("FACE_set_mouth", data);
}


void FACE_set_state_handler(TCX_REF_PTR ref, FACE_state_ptr data) {

#ifdef FACE_debug
  fprintf(stderr, "Received a FACE_set_state message: (%d %d %d) from %s.\n",
	  data->eyes, data->brows, data->mouth, tcxModuleName(ref->module));
#endif

  state.eye0_pos = data->eyes;  
  state.eye1_pos = data->eyes;  
  state.brow0_pos = data->brows;  
  state.brow1_pos = data->brows;  
  state.mouth0_pos = data->mouth;  
  state.mouth1_pos = data->mouth;  

  FaceSetState(&state);

  send_automatic_status_update();
  tcxFree("FACE_set_state", data);
}


void FACE_adjust_mood_handler(TCX_REF_PTR ref, int *data) {

#ifdef FACE_debug
  fprintf(stderr, "Received a FACE_adjust_mood message: %d from %s.\n",
	  *data, tcxModuleName(ref->module));
#endif

  FaceAdjustMood(&state, *data);

  send_automatic_status_update();
  tcxFree("FACE_adjust_mood", data);
}


void FACE_adjust_eyes_handler(TCX_REF_PTR ref, int *data) {

#ifdef FACE_debug
  fprintf(stderr, "Received a FACE_adjust_eyes message: %d from %s.\n",
	  *data, tcxModuleName(ref->module));
#endif

  FaceAdjustEyes(&state, *data);

  send_automatic_status_update();
  tcxFree("FACE_adjust_eyes", data);
}


void FACE_reset_handler(TCX_REF_PTR ref, void *data) {

#ifdef FACE_debug
  fprintf(stderr, "Received a FACE_reset message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  FaceReset(&state);

  send_automatic_status_update();
}

void FACE_disconnect_handler(TCX_REF_PTR ref, void *data) {

#ifdef FACE_debug
  fprintf(stderr, "Received a FACE_disconnect message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  remove_auto_update_module(ref->module);  

}

void FACE_close_handler(char *name, TCX_MODULE_PTR module) {

#ifdef FACE_debug
  fprintf(stderr, "FACE: closed connection detected: %s\n", name);
#endif
  remove_auto_update_module(module);

  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    exit(0);
  } 
}

TCX_REG_HND_TYPE FACE_handler_array[] = {
  {"FACE_set_eyes", "FACE_set_eyes_handler",
     FACE_set_eyes_handler, TCX_RECV_ALL, NULL},
  {"FACE_set_brows", "FACE_set_brows_handler",
     FACE_set_brows_handler, TCX_RECV_ALL, NULL},
  {"FACE_set_mouth", "FACE_set_mouth_handler",
     FACE_set_mouth_handler, TCX_RECV_ALL, NULL},
  {"FACE_set_state", "FACE_set_state_handler",
     FACE_set_state_handler, TCX_RECV_ALL, NULL},
  {"FACE_reset", "FACE_reset_handler",
     FACE_reset_handler, TCX_RECV_ALL, NULL},
  {"FACE_adjust_mood", "FACE_adjust_mood_handler",
     FACE_adjust_mood_handler, TCX_RECV_ALL, NULL},
  {"FACE_adjust_eyes", "FACE_adjust_eyes_handler",
     FACE_adjust_eyes_handler, TCX_RECV_ALL, NULL},
  {"FACE_disconnect", "FACE_disconnect_handler",
     FACE_disconnect_handler, TCX_RECV_ALL, NULL},
};


/************************************************************************
 *
 *   NAME:         main()
 *                 
 *   FUNCTION:     Installs handlers, runs TCX loop
 *                 
 *   PARAMETERS:   none, yet
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


main(int argc, char **argv) {
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    FACE_messages,
  };
  fd_set readMask;

  fprintf(stderr, "Connecting to TCX...");

  tcxInitialize("FACE", getenv("TCXHOST"));

  check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
		       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
		       NULL, 0);
  check_version_number(libezx_major, libezx_minor,
		       libezx_robot_type, libezx_date,
		       "libezx", 0);
  check_version_number(librobot_major, librobot_minor,
		       librobot_robot_type, librobot_date,
		       "librobot", 1);

  fprintf(stderr, "done.\n");

  
#ifdef i386
  fprintf(stderr, "Connecting to FACE...");
  if (FaceOpen(&state)) {
    fprintf(stderr, "Couldn't connect to face.\n");
  }
    
  fprintf(stderr, "done.\n");
#endif

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));

  tcxRegisterHandlers(FACE_handler_array, 
		      sizeof(FACE_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterCloseHnd(FACE_close_handler);

  state.eye0_pos = 0;
  state.eye1_pos = 0;
  state.brow0_pos = 0;
  state.brow1_pos = 0;
  state.mouth0_pos = 0;
  state.mouth1_pos = 0;
  FaceSetState(&state);
  FaceSetEyes(&state, 10);

  if (bRobot.fork) {
    bDaemonize("PANTILT.log");
  }

  for (;;){
    send_automatic_status_update();
    TCX_waiting_time.tv_sec    = 0;
    TCX_waiting_time.tv_usec   = 100000;
    tcxRecvLoop((void *) &TCX_waiting_time);
  }
}
