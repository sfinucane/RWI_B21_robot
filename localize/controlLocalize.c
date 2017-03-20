
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/controlLocalize.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:39 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: controlLocalize.c,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.11  1998/03/04 14:46:43  fox
 * This version should run.
 *
 * Revision 1.10  1997/11/26 15:47:42  fox
 * Added some structures for questions.
 *
 * Revision 1.9  1997/06/20 07:36:08  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.8  1997/03/26 09:42:57  fox
 * Updated correction parameter message.
 *
 * Revision 1.7  1997/03/14 17:58:17  fox
 * This version should run quite stable now.
 *
 * Revision 1.6  1997/02/22 05:16:40  thrun
 * Version numbers are now also checked for libraries.
 *
 * Revision 1.5  1997/02/22 00:59:05  thrun
 * Introduced version number support
 *
 * Revision 1.4  1997/02/04 18:11:48  fox
 * Updated the module.
 *
 * Revision 1.3  1997/01/30 13:34:11  fox
 * Minor changes.
 *
 * Revision 1.2  1997/01/30 10:50:02  fox
 * Minor changes.
 *
 * Revision 1.1  1997/01/29 12:23:02  fox
 * First version of restructured LOCALIZE.
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


/* #define USER_debug */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "tcx.h"
#include "tcxP.h"
#include "localize.h"

#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */
#define TCX_define_variables /* this makes sure variables are installed */

#include "LOCALIZE-messages.h"

#include "beeSoftVersion.h"
#include "liblocalize.h"
#include "librobot.h"
#include "libezx.h"




#define TCX_USER_MODULE_NAME "CONTROL_LOCALIZE"

int
block_wait( struct timeval *timeout, int tcx_initialized,
	    int X_initialized);

static void
localizeStartActiveLocalization();
static void
localizeStopActiveLocalization();
static void
localizeQuit();

int localizeReconnected = TRUE;

/*****************************************************************
 * Receive a map.
 *****************************************************************/
void
LOCALIZE_map_reply_handler( TCX_REF_PTR                 ref,
			    LOCALIZE_map_reply_ptr map)
{
  fprintf(stderr, "Got map.\n");

  tcxFree( "LOCALIZE_map_reply", map);
}


/*****************************************************************
 * Receive status update.
 *****************************************************************/
void
LOCALIZE_update_status_reply_handler( TCX_REF_PTR                 ref,
				      LOCALIZE_update_status_reply_ptr status)
{

  if ( localizeReconnected) {
    localizeStartActiveLocalization();
    localizeReconnected = FALSE;
  }
    
  fprintf(stderr, "Got status.\n");
  fprintf(stderr, "Corr: %f %f %f %d\n", status->corrX, status->corrY, status->corrRot,
	  status->corrType);
  fprintf(stderr, "Number %d\nProb %f\n", status->numberOfLocalMaxima, status->probOfGlobalMaximum);
  
  if ( status->numberOfLocalMaxima == 1 && status->probOfGlobalMaximum > 0.8)
    localizeStopActiveLocalization();

  
  tcxFree( "LOCALIZE_update_status_reply", status);
}


/*****************************************************************
 * Let localize do active localization.
 *****************************************************************/
static void
localizeStartActiveLocalization()
{
  fprintf(stderr, "start active\n");
  tcxSendMsg( LOCALIZE, "LOCALIZE_start_active_localization", NULL);
}


/*****************************************************************
 * Let localize stop with active localization.
 *****************************************************************/
static void
localizeStopActiveLocalization()
{
  tcxSendMsg( LOCALIZE, "LOCALIZE_stop_active_localization", NULL);
}


/*****************************************************************
 * Let localize do active localization.
 *****************************************************************/
static void
localizeQuit()
{
  tcxSendMsg( LOCALIZE, "LOCALIZE_quit", NULL);
}


void
control_close_handler(char *name, TCX_MODULE_PTR module)
{
#ifdef USER_debug
  fprintf(stderr, "CONTROL_LOCALIZE: closed connection detected: %s\n", name);
#endif

  if (!strcmp(name, TCX_LOCALIZE_MODULE_NAME)){ /* LOCALIZE shut down */
    LOCALIZE = NULL;
  }
  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    exit(0);
  }
}


static void
initTcx()
{
  char *tcxMachine = NULL;
 
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    LOCALIZE_messages
  };

  fprintf(stderr, "Connecting to TCX...");
  tcxMachine = getenv("TCXHOST");   
  
  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }
  
  tcxInitialize( TCX_USER_MODULE_NAME, (void *) tcxMachine);
  check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
		       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
		       NULL, 0);
  check_version_number(libezx_major, libezx_minor,
		       libezx_robot_type, libezx_date,
		       "libezx", 0);
  check_version_number(librobot_major, librobot_minor,
		       librobot_robot_type, librobot_date,
		       "librobot", 0);
  check_version_number(liblocalize_major, liblocalize_minor,
		       liblocalize_robot_type, liblocalize_date,
		       "liblocalize", 1);






  fprintf(stderr, "done.\n");

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));

  tcxRegisterHandlers( LOCALIZE_reply_handler_array,
		       sizeof(LOCALIZE_reply_handler_array)
		       / sizeof(TCX_REG_HND_TYPE));
  {
    extern void
      tcxRegisterCloseHnd(void (*closeHnd)());
    tcxRegisterCloseHnd(control_close_handler);
  }
}


/******************************************************************************
 ******************************************************************************
 * Main control for LOCALIZE.
 ******************************************************************************
 ******************************************************************************/
void
main( int argc, char** argv)
{
  struct timeval TCX_waiting_time = {0, 0};

  initTcx();
  
  for (;;){
    if (LOCALIZE == NULL){
      LOCALIZE_register_auto_update_type data;
      data.subscribe = 1;
      fprintf(stderr, "Connecting to %s...", TCX_LOCALIZE_MODULE_NAME);
      LOCALIZE = tcxConnectModule(TCX_LOCALIZE_MODULE_NAME);
      fprintf(stderr, "done.\n");
      tcxSendMsg(LOCALIZE, "LOCALIZE_register_auto_update", &data);
      localizeReconnected = TRUE;
    }
    else {
      block_wait(NULL, 1, 0);
    }
    
    tcxRecvLoop((void *) &TCX_waiting_time);
  }

  exit(0);			/* should never reach here! */
}

