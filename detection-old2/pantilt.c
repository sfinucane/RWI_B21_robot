
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection-old2/pantilt.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:04 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: pantilt.c,v $
 * Revision 1.1  2002/09/14 20:45:04  rstone
 * *** empty log message ***
 *
 * Revision 1.10  1998/09/05 00:25:28  fox
 * Works fine to detect people in the museum.
 *
 * Revision 1.9  1998/08/23 22:57:42  fox
 * First version of building maps of humans.
 *
 * Revision 1.8  1997/06/03 11:49:16  fox
 * Museum version.
 *
 * Revision 1.7  1997/05/28 14:40:02  thrun
 * .
 *
 * Revision 1.6  1997/05/28 14:04:11  fox
 * Fixed a bug.
 *
 * Revision 1.5  1997/05/28 09:01:29  wolfram
 * added motion-only mode
 *
 * Revision 1.4  1997/05/28 07:17:56  fox
 * is egal.
 *
 * Revision 1.3  1997/05/09 16:28:40  fox
 * Works quiet fine.
 *
 * Revision 1.2  1997/05/06 14:22:59  fox
 * Nothing special.
 *
 * Revision 1.1  1997/05/05 16:54:08  fox
 * Incorporated graphics from people.c.
 *
 * Revision 1.1.1.1  1997/04/30 16:17:09  fox
 * Module to detect unexpected obstacles.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include "general.h"
#include "PANTILT-messages.h"
#include "detection.h"
#include "graphics.h"
#include "localTcx.h"
#include "function.h"
#include "o-graphics.h"
#include <unistd.h>
#include <time.h>

#define TRACK_THRESHOLD 5.0

static int TRACKING_MODE = FALSE;



void
movePanTilt( detectionStruct obstacles)
{
  float angle;
  PANTILT_move_type data;
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;
  
  if ( connectPantilt) {
    
    PANTILT_track_point_type trackPoint;
    
    if ( PANTILT != NULL && obstacles.foundSomething) {
      
      static point prevTrackPoint = {0.0, 0.0};
      
      if ( ! TRACKING_MODE ||
	   positionDist( prevTrackPoint.x, prevTrackPoint.y,
			 obstacles.detected.x, obstacles.detected.y)
	   > TRACK_THRESHOLD) {
	
	gettimeofday(&this_time, NULL);
	time_difference = 
	  ((float) (this_time.tv_sec - last_time.tv_sec))
	    + (((float) (this_time.tv_usec - last_time.tv_usec))
	       /  1000000.0);
	if (time_difference > 0.5){
	  
	  trackPoint.x = obstacles.detected.x;
	  trackPoint.y = obstacles.detected.y;
	  trackPoint.height = TRACK_HEIGHT;
	  prevTrackPoint = obstacles.detected;
	  if (0) fprintf(stderr, "send %f %f\n", trackPoint.x, trackPoint.y);
	  tcxSendMsg(PANTILT, "PANTILT_track_point", &trackPoint);
	  
	  last_time.tv_sec = this_time.tv_sec;
	  last_time.tv_usec = this_time.tv_usec;
	}
      }
      TRACKING_MODE = TRUE;
    }
  }
}


void
stopPanTilt()
{
  if ( connectPantilt) {
    
    PANTILT_move_type pan_data;
    
    if (PANTILT != NULL) {
      pan_data.pan_target = -90.0;
      pan_data.tilt_target = -20.0;
      tcxSendMsg(PANTILT, "PANTILT_stop_tracking", NULL);
      tcxSendMsg(PANTILT, "PANTILT_move", &pan_data);
    }
    TRACKING_MODE = FALSE;
  }
}



void 
PANTILT_position_reply_handler(TCX_REF_PTR                ref,
			       PANTILT_position_reply_ptr data)
{
#ifdef PEOPLE_debug
  fprintf(stderr, "TCX: Received a PANTILT_position_reply message.\n");
#endif
}

void 
PANTILT_init_reply_handler(TCX_REF_PTR             ref,
			   int                    *data)
{
#ifdef PEOPLE_debug
  fprintf(stderr, "TCX: Received a PANTILT_init_reply message.\n");
#endif
}

void 
PANTILT_limits_reply_handler(TCX_REF_PTR              ref,
			     PANTILT_limits_reply_ptr data)
{
#ifdef PEOPLE_debug
  fprintf(stderr, "TCX: Received a PANTILT_limits_reply message.\n");
#endif
}


void 
PANTILT_status_update_handler(TCX_REF_PTR              ref,
			      PANTILT_status_update_ptr data)
{
#ifdef PEOPLE_debug
  fprintf(stderr, "TCX: Received a PANTILT_status_update message.\n");
  fprintf(stderr, "speed: %6.4f %6.4f\n",
	  data->pan_velocity, data->tilt_velocity);
#endif
  tcxFree("PANTILT_status_update", data);
}




static struct timeval last_attempt_connect_PANTILT = {0, 0};


/************************************************************************
 *
 *   NAME:         connect_to_PANTILT
 *                 
 *   FUNCTION:     checks, and connects to PANTILT, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void
connect_to_PANTILT(PROGRAM_STATE_PTR program_state)
{
  if ( connectPantilt) {
    
    int data;
    struct timeval current_time;
    PANTILT_move_type pan_data;
    
    if(!program_state->pantilt_connected){
      
      
      gettimeofday(&current_time, NULL);
      if (current_time.tv_sec < last_attempt_connect_PANTILT.tv_sec + 3
	  || (current_time.tv_sec == last_attempt_connect_PANTILT.tv_sec + 3 &&
	      current_time.tv_usec < last_attempt_connect_PANTILT.tv_usec))
	return;
      
      
      if (program_state->graphics_initialized)
	G_display_switch(PANTILT_CONNECTED_BUTTON, 2);
      
      last_attempt_connect_PANTILT.tv_sec  = current_time.tv_sec;
      last_attempt_connect_PANTILT.tv_usec = current_time.tv_usec;
      
      PANTILT = tcxConnectOptional(TCX_PANTILT_MODULE_NAME); /* checks, but does 
							      * not wait */
      
      fprintf(stderr, "done\n");
      if (PANTILT != NULL){
	if (program_state->graphics_initialized)
	  G_display_switch(PANTILT_CONNECTED_BUTTON, 1);
	
	data = 1;
	tcxSendMsg(PANTILT, "PANTILT_init_query", &data);
	sleep(1);
	pan_data.pan_target = -90.0;
	pan_data.tilt_target = -20.0;
	tcxSendMsg(PANTILT, "PANTILT_stop_tracking", NULL);
	tcxSendMsg(PANTILT, "PANTILT_move", &pan_data);
	
	
	
      program_state->pantilt_connected = 1;
      }
      else if (program_state->graphics_initialized){
	/*usleep(200000);*/
	G_display_switch(PANTILT_CONNECTED_BUTTON, 0);
      }
    }
  }
}


