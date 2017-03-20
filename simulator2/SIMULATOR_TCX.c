
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File:                        SIMULATOR_TCX.c
 *****
 ***** Part of:                     RHINO SOFTWARE
 *****
 ***** Creator:                     Peter Wallossek, University of Bonn
 *****
 ***** Date of creation:            Jan 1994
 *****
 *****
 *****
 *****
 ***** $Source: /usr/local/cvs/bee/src/simulator2/SIMULATOR_TCX.c,v $
 *****
 ***** $Revision: 1.1 $
 *****
 ***** $Date: 2002/09/14 16:11:07 $
 *****
 ***** $Author: rstone $
 *****
 *****
 *****
 ***** Contact thrun@carbon.cs.bonn.edu or thrun@cs.cmu.edu.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY
 ***** APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING THE
 ***** COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS"
 ***** WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
 ***** INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 ***** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE ENTIRE
 ***** RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.
 ***** SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL
 ***** NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF
 ***** DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU
 ***** OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY
 ***** OTHER PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN
 ***** ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: SIMULATOR_TCX.c,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.18  1999/08/27 09:02:55  schulz
 * Fixed TCX-stuff and conflicts with Dieter's changes
 *
 * Revision 1.17  1999/08/26 21:17:29  fox
 * First changes for simulated multi robot localization.
 *
 * Revision 1.16  1999/08/26 16:53:51  schulz
 * -- The robot position can now be specified on the command line.
 * -- SIMULATOR send the robot's global position to modules, which
 *    have registered for it (Not tested yet!).
 *
 * Revision 1.15  1999/02/19 14:30:34  schulz
 * replaced Tysons block_wait by a private copy without the XSetFocus stuff!!!
 * Removed any dependency on librobot and libezx.
 *
 * Revision 1.14  1998/04/12 15:55:07  wolfram
 * Added option -robot for multi-robot support
 *
 * Revision 1.13  1997/03/14 17:21:46  tyson
 * Added tactile support to simulator
 *
 * Revision 1.12  1997/03/11 17:16:42  tyson
 * added IR simulation and other work
 *
 * Revision 1.11  1997/02/22 05:16:53  thrun
 * Version numbers are now also checked for libraries.
 *
 * Revision 1.10  1997/02/22 00:59:11  thrun
 * Introduced version number support
 *
 * Revision 1.9  1997/02/12 13:11:22  tyson
 * more parameter utils.  Updated wander.c. Some processes now support -fork=[y|n] instead of [+|-]stdin
 *
 * Revision 1.8  1997/01/30 13:33:06  schulz
 * minor fixes
 *
 * Revision 1.7  1997/01/28 13:58:27  schulz
 * Added a TCX message port to move the robot from a program.
 * Not yet tested!
 *
 * Revision 1.6  1996/12/11 12:11:09  schulz
 * fixed a bus error in baseServer()
 *
 * Revision 1.5  1996/12/04 05:42:29  tyson
 * simulator2 <-> baseServer partially tested.  Cleaning up bumble and sensor tests as demos.  Added frameWork module.
 *
 * Revision 1.4  1996/12/03 05:37:36  thrun
 * If TCXHOST is not se, the software will now assume "localhost" and
 * won't terminate.
 *
 * Revision 1.3  1996/11/18 04:34:58  tyson
 * More baseServer<->simulator2 work. Still not done.
 *
 * Revision 1.2  1996/11/14 12:58:02  ws
 * Introduced a scheduler with priorities and aging, to
 * get better performance on heavily loaded systems.    DS
 *
 * Revision 1.1.1.1  1996/09/30 16:17:43  schulz
 * reimport on new source tree
 *
 * Revision 1.2  1996/08/26 10:05:31  schulz
 * added laser interface (without visualization, yet) D.S.
 *
 * Revision 1.12  1994/12/12 16:32:34  wallosse
 * now target points can be deleted (right mouse button)
 * now there is a smooth stop after RR and TR
 *
 * Revision 1.11  1994/10/09  11:57:52  wallosse
 * changed some handling features. one more zoom-level. sonar display
 *
 * Revision 1.10  1994/09/18  11:22:28  wallosse
 * Now it is possible to select between different types of surfaces.
 * I think it works !!!!
 *
 * Revision 1.9  1994/07/22  11:57:21  wallosse
 * Fixed a lot of bugs !
 * Vewrsion now runs again under LINUX when the optimize flag is not used when compiling.
 *
 * Revision 1.8  1994/06/22  13:46:10  wallosse
 * Changed a lot ! :	now events are scheduled with routine schedule().
 * 			obstacles can be added or chenged while robot
 * 			is runnung.
 * 			TARGET is not ok !!!
 *
 * Revision 1.7  1994/06/13  13:02:32  wallosse
 * sebastian changed some =+ to += !!! Congratulations !!!
 *
 * Revision 1.6  1994/06/13  11:29:36  wallosse
 * changes to send target point via TCX to base
 *
 * Revision 1.5  1994/06/13  11:00:48  wallosse
 * again filled in some temporarily delays and TCX message for target
 *
 * Revision 1.4  1994/06/13  08:45:51  fox
 * changed translate and rotate relative
 *
 * Revision 1.3  1994/06/08  13:44:29  wallosse
 * Some prelimnary times/delays for sonar and X-graphics.
 *
 * Revision 1.2  1994/06/07  15:05:22  wallosse
 * New header files. Don't they look great?
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
#include <signal.h>
#include <sys/time.h>

#include "tcx.h"
#include "tcxP.h"


#define TCX_define_variables 		/*** this makes sure variables are installed ***/
#include "SIMULATOR-messages.h"
#include "autoupdates.h"

#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */

#include "BASE-messages.h"
#include "robot.h"

#include "beeSoftVersion.h"
#include "libbUtils.h"
//#include "librobot.h"


#define LENGTH_OF_MODULE_NAMES 128
#define NUMBER_OF_TCX_MODULES 3
#define SIMULATOR_MODULE 0
#define BASE_MODULE 1
#define VIEWER_MODULE 2

char moduleName[NUMBER_OF_TCX_MODULES][LENGTH_OF_MODULE_NAMES];
extern char *robotName;


extern Boolean use_baseServer;
extern Boolean use_sonarServer;

extern char just_viewing;

void setRobotPosition(float,float,float);
void BeamRobot3(float,float,float);

static int tcx = 1;			/*** indicats if TCX shall be used, If not,
				 	 *** we will allow a simple simulator
				 	 *** test, nothing else. 
				 	 ***/

char	BaseCommand[81];
char	SonarCommand[81];



/*** TCX routines by S. Thrun ***/


/************************************************************************
 *
 *   NAME:         SIMULATOR_message_from_base_handler()
 *                 
 *   FUNCTION:     Receiving handler for BASE bessages.
 *                 
 *   PARAMETERS:   standard TCX parameters and a text string (message)
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SIMULATOR_message_from_base_handler(TCX_REF_PTR   ref,
					 char        **message)
{
  void	 base(char command[]);

#ifdef SIMULATOR_debug
  fprintf(stderr, "Received a SIMULATOR_message_from_base from %s: [%s]\n",
	  tcxModuleName(ref->module), *message);
#endif

  /*=================================================================
   *======= The first module that sends anything with this message
   *======= will be considered BASE. From now on, every outgoing BASE
   *======= message will be sent to this module.
   *======= Here we "register" this module in MODULE_BASE ===========*/

  if (MODULE_BASE == NULL){
    MODULE_BASE = ref->module;
    fprintf(stderr, "Registered module %s as MODULE_BASE.\n",
	    tcxModuleName(ref->module));
    use_baseServer = FALSE;
  }


  /***======= Reply to BASE - just for fun. =======***/

  /*
  if (MODULE_BASE != NULL){
    char *reply_msg;

    reply_msg = (char *) malloc(256);
    strcpy(reply_msg, "base connection established !");
    
    fprintf(stderr, "%10s:%5d:%14s(): \n",
    __FILE__, __LINE__, __FUNCTION__);
    
    tcxSendMsg(MODULE_BASE, "SIMULATOR_message_to_base", &reply_msg);

    free(reply_msg);
  }  
  */

  strcpy(BaseCommand, *message);
  base(BaseCommand);

#ifdef SIMULATOR_debug
	fprintf(stderr, "BaseCommand = %s\n", BaseCommand);
#endif

  /***======= free the message memory - do not remove! =======***/

  tcxFree("SIMULATOR_message_from_base", message);
}



/************************************************************************
 *
 *   NAME:         SIMULATOR_message_from_baseServer_handler()
 *                 
 *   FUNCTION:     Receiving handler for baseServer bessages.
 *                 
 *   PARAMETERS:   Binary character array - unsigned char[6]
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SIMULATOR_message_from_baseServer_handler(TCX_REF_PTR   ref,
					       char *message)
{
  void	 baseServer(unsigned char *cmd);

#ifdef SIMULATOR_debug
  fprintf(stderr, "%10s:%5d:%14s(): \n",
	  __FILE__, __LINE__, __FUNCTION__);

  fprintf(stderr,
	  "Received a SIMULATOR_message_from_baseServer from %s:\n",
	  tcxModuleName(ref->module));
  fprintf(stderr, "\t\t\t%02X:%02X:%02X:%02X:%02X:%02X\n",
	  message[0],  message[1],  message[2],
	  message[3],  message[4],  message[5]);
#endif

  /*=================================================================
   *======= The first module that sends anything with this message
   *======= will be considered BASE. From now on, every outgoing BASE
   *======= message will be sent to this module.
   *======= Here we "register" this module in MODULE_BASE ===========*/

  if (MODULE_BASE == NULL){
    MODULE_BASE = ref->module;

    fprintf(stderr, "%10s:%5d:%14s(): \n",
	    __FILE__, __LINE__, __FUNCTION__);

    fprintf(stderr, "Registered module %s as MODULE_BASE.\n",
	    tcxModuleName(ref->module));
    use_baseServer = TRUE;
  }

  baseServer(message);

  /***======= free the message memory - do not remove! =======***/

  tcxFree("SIMULATOR_message_from_baseServer", message);

}



/************************************************************************
 *
 *   NAME:         SIMULATOR_message_from_sonar_handler()
 *                 
 *   FUNCTION:     Receiving handler for SONAR bessages.
 *                 
 *   PARAMETERS:   standard TCX parameters and a text string (message)
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SIMULATOR_message_from_sonar_handler(TCX_REF_PTR   ref,
					 char        **message)
{

  void	sonar(char command[]);

  

#ifdef SIMULATOR_debug
  fprintf(stderr, "Received a SIMULATOR_message_from_sonar from %s: [%s]\n",
	  tcxModuleName(ref->module), *message);
#endif

  /*==================================================================
   *======= The first module that sends anything with this message
   *======= will be considered SONAR. From now on, every outgoing SONAR
   *======= message will be sent to this module.
   *======= Here we "register" this module in MODULE_SONAR ===========*/

  if (MODULE_SONAR == NULL){
    MODULE_SONAR = ref->module;
    fprintf(stderr, "Registered module %s as MODULE_SONAR.\n",
	    tcxModuleName(ref->module));
    use_sonarServer = FALSE;
  }


  /***======= Reply to SONAR - just for fun. =======***/

  /*
  if (MODULE_SONAR != NULL){
    char *reply_msg;

    reply_msg = (char *) malloc(256);
    strcpy(reply_msg, "sonar connection established !");

    fprintf(stderr, "%10s:%5d:%14s(): tcxSendMsg\n",
    __FILE__, __LINE__, __FUNCTION__);

    tcxSendMsg(MODULE_SONAR, "SIMULATOR_message_to_sonar", &reply_msg);

    free(reply_msg);
  }  
  */

  strcpy(SonarCommand, *message);
  sonar(SonarCommand);

  /***======= free the message memory - do not remove! =======***/


  tcxFree("SIMULATOR_message_from_sonar", message);
}



/************************************************************************
 *
 *   NAME:         SIMULATOR_message_from_sonarServer_handler()
 *                 
 *   FUNCTION:     Receiving handler for SONAR bessages from sonarServer
 *                 
 *   PARAMETERS:
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
SIMULATOR_message_from_sonarServer_handler(TCX_REF_PTR   ref,
					   char        *message)
{

#ifdef SIMULATOR_debug
  fprintf(stderr,
	  "Received a SIMULATOR_message_from_sonarServer from %s: [%s]\n",
	  tcxModuleName(ref->module), "[binary message]");
#endif

  /*==================================================================
   *======= The first module that sends anything with this message
   *======= will be considered SONAR. From now on, every outgoing SONAR
   *======= message will be sent to this module.
   *======= Here we "register" this module in MODULE_SONAR ===========*/

  if (MODULE_SONAR == NULL){
    MODULE_SONAR = ref->module;
    fprintf(stderr, "Registered module %s as MODULE_SONAR.\n",
	    tcxModuleName(ref->module));
    use_sonarServer = TRUE;
  }

  sonarServer(message);

  /***======= free the message memory - do not remove! =======***/

  tcxFree("SIMULATOR_message_from_sonarServer", message);
}



/************************************************************************
 *
 *   NAME:         SIMULATOR_message_from_irServer_handler()
 *                 
 *   FUNCTION:     Receiving handler for IR bessages from irServer
 *                 
 *   PARAMETERS:
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
SIMULATOR_message_from_irServer_handler(TCX_REF_PTR   ref,
					char        *message)
{

#ifdef SIMULATOR_debug
  fprintf(stderr, "Received a SIMULATOR_message_from_irServer from %s: [%s]\n",
	  tcxModuleName(ref->module), "[binary message]");
#endif

  /*==================================================================
   *======= The first module that sends anything with this message
   *======= will be considered IR. From now on, every outgoing SONAR
   *======= message will be sent to this module.
   *======= Here we "register" this module in MODULE_IR ===========*/

  if (MODULE_IR == NULL){
    MODULE_IR = ref->module;
    fprintf(stderr, "Registered module %s as MODULE_IR.\n",
	    tcxModuleName(ref->module));
  }

  irServer(message);

  /***======= free the message memory - do not remove! =======***/

  tcxFree("SIMULATOR_message_from_irServer", message);
}



/************************************************************************
 *
 *   NAME:         SIMULATOR_message_from_tactileServer_handler()
 *                 
 *   FUNCTION:     Receiving handler for TACTILE bessages from tactileServer
 *                 
 *   PARAMETERS:
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
SIMULATOR_message_from_tactileServer_handler(TCX_REF_PTR   ref,
					char        *message)
{

#ifdef SIMULATOR_debug
  fprintf(stderr, "Received a SIMULATOR_message_from_tactileServer from %s: [%s]\n",
	  tcxModuleName(ref->module), "[binary message]");
#endif

  /*==================================================================
   *======= The first module that sends anything with this message
   *======= will be considered TACTILE. From now on, every outgoing SONAR
   *======= message will be sent to this module.
   *======= Here we "register" this module in MODULE_TACTILE ===========*/

  if (MODULE_TACTILE == NULL){
    MODULE_TACTILE = ref->module;
    fprintf(stderr, "Registered module %s as MODULE_TACTILE.\n",
	    tcxModuleName(ref->module));
  }

  tactileServer(message);

  /***======= free the message memory - do not remove! =======***/

  tcxFree("SIMULATOR_message_from_tactileServer", message);
}


void SIMULATOR_register_auto_update_handler(TCX_REF_PTR   ref,
					    SIMULATOR_register_auto_update_ptr skip)
{
  add_auto_update_module(ref->module, skip->update);
  tcxFree("SIMULATOR_register_auto_update", skip);
}

void SIMULATOR_cancel_auto_update_handler(TCX_REF_PTR   ref,
				       void *nil)
{
  remove_auto_update_module(ref->module);
  tcxFree("SIMULATOR_register_auto_update", NULL);
}

/************************************************************************
 *
 *   NAME:         SIMULATOR_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void SIMULATOR_close_handler(char *name, TCX_MODULE_PTR module)
{
#ifdef SIMULATOR_debug
  fprintf(stderr, "SIMULATOR: closed connection detected: %s\n", name);
#endif

  if (module == MODULE_BASE){
    fprintf(stderr, "Cancelled registration for MODULE_BASE.\n");
    MODULE_BASE = NULL;
    fprintf(stderr, "BASE died, and so will I. Bye-bye.\n");

    exit(0);
  }

  if (module == MODULE_SONAR){
    fprintf(stderr, "Cancelled registration for MODULE_SONAR.\n");
    MODULE_SONAR = NULL;
    exit(0);
  }

  if (module == MODULE_IR){
    fprintf(stderr, "Cancelled registration for MODULE_IR.\n");
    MODULE_SONAR = NULL;
    exit(0);
  }

  if (module == MODULE_TACTILE){
    fprintf(stderr, "Cancelled registration for MODULE_TACTILE.\n");
    MODULE_SONAR = NULL;
    exit(0);
  }
  
  else if (!strcmp(name, "TCX Server")) { 		/*** TCX shut down ***/
    exit(0);
  }
  else {
    remove_auto_update_module(module);
  }
}


void SIMULATOR_set_robot_position_handler(TCX_REF_PTR ref,
				      SIMULATOR_set_robot_position_ptr position)
{
#ifdef USER_debug
  fprintf(stderr, "TCX: Received a set_robot_position message.\n");
  fprintf(stderr, "robot: %g %g %g\n", 
	  position->x, position->y, position->rot);
#endif
  BeamRobot3(position->x,position->y,position->rot); 
  tcxFree("SIMULATOR_set_robot_position", position); /* don't remove this! */
}


/* -------------------------------------------------------------------- */


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         BASE_update_status_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void BASE_update_status_reply_handler(TCX_REF_PTR ref,
				      BASE_update_status_reply_ptr status)
{
#ifdef USER_debug
  fprintf(stderr, "TCX: Received a BASE_update_status_reply message.\n");
  fprintf(stderr, "robot: %g %g %g\n", 
	  status->pos_x, status->pos_y, status->orientation);
#endif
  setRobotPosition(status->pos_y,status->pos_x,status->orientation); 
  tcxFree("BASE_update_status_reply", status); /* don't remove this! */
}

void BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				       BASE_robot_position_reply_ptr pos){
  ;
}




void BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
					BASE_action_executed_reply_ptr data)
{
  fprintf(stderr,"Received a BASE_action_executed_reply\n");
}





/* -------------------------------------------------------------------- */



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/*** internal handlers for external messages ***/

TCX_REG_HND_TYPE SIMULATOR_handler_array[] = {
  {"SIMULATOR_message_from_base", "SIMULATOR_message_from_base_handler",
     SIMULATOR_message_from_base_handler, TCX_RECV_ALL, NULL},
  {"SIMULATOR_message_from_sonar", "SIMULATOR_message_from_sonar_handler",
     SIMULATOR_message_from_sonar_handler, TCX_RECV_ALL, NULL},
  {
    "SIMULATOR_message_from_baseServer",
    "SIMULATOR_message_from_baseServer_handler",
    SIMULATOR_message_from_baseServer_handler,
    TCX_RECV_ALL, NULL
  },
  {
    "SIMULATOR_message_from_sonarServer",
    "SIMULATOR_message_from_sonarServer_handler",
    SIMULATOR_message_from_sonarServer_handler,
    TCX_RECV_ALL, NULL
  },
  {
    "SIMULATOR_message_from_irServer",
    "SIMULATOR_message_from_irServer_handler",
    SIMULATOR_message_from_irServer_handler,
    TCX_RECV_ALL, NULL
  },
  {
    "SIMULATOR_message_from_tactileServer",
    "SIMULATOR_message_from_tactileServer_handler",
    SIMULATOR_message_from_tactileServer_handler,
    TCX_RECV_ALL, NULL
  },
  {
    "SIMULATOR_set_robot_position",
    "SIMULATOR_set_robot_position_handler",
    SIMULATOR_set_robot_position_handler,
    TCX_RECV_ALL, NULL
  },
  {"SIMULATOR_register_auto_update", 
   "SIMULATOR_register_auto_update_handler",
   SIMULATOR_register_auto_update_handler, 
   TCX_RECV_ALL, NULL},
  {"SIMULATOR_cancel_auto_update", 
   "SIMULATOR_cancel_auto_update_handler",
   SIMULATOR_cancel_auto_update_handler, 
   TCX_RECV_ALL, NULL}
};


void
initializeModuleNames(){
  tcxSetModuleName(TCX_SIMULATOR_MODULE_NAME, robotName, moduleName[SIMULATOR_MODULE]);
  tcxSetModuleName(TCX_BASE_MODULE_NAME, robotName, moduleName[BASE_MODULE]);
  tcxSetModuleName("VIEWER", robotName, moduleName[VIEWER_MODULE]);
  {
    int i;
    fprintf(stderr, "Known modules:");
    for (i = 0; i < NUMBER_OF_TCX_MODULES; i++)
      fprintf(stderr, " %s", moduleName[i]);
    fprintf(stderr, ".\n");
  }

}

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/************************************************************************
 *
 *   NAME:         SIMULATOR_initialize_tcx()
 *                 
 *   FUNCTION:     Initializes and connects to TCX.
 *                 
 *   PARAMETERS:   name of the module
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



int
SIMULATOR_initialize_tcx(char *tcx_simulator_module_name,
			 const char *tcxMachine)
{
  char *simulator_module;
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    BASE_messages,
    SIMULATOR_messages
  };

  /* initialize Module Names */
  initializeModuleNames();

  if (strcmp(tcx_simulator_module_name,TCX_SIMULATOR_MODULE_NAME) == 0)
    simulator_module = moduleName[SIMULATOR_MODULE];
  else
    simulator_module = moduleName[VIEWER_MODULE];
      
  
  /*=========================================*
   *      initialize and connect to TCX
   *=========================================*/


  if (tcx_simulator_module_name != NULL){
    tcx = 1;
    
    fprintf(stderr, "Connecting to TCX...");
    tcxInitialize(simulator_module, (char *)tcxMachine);
    check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
			 BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
			 NULL, 0);
    /*
    check_version_number(librobot_major, librobot_minor,
			 librobot_robot_type, librobot_date,
			 "librobot", 0);
    check_version_number(libbUtils_major, libbUtils_minor,
			 libbUtils_robot_type, libbUtils_date,
			 "libbUtils", 1);

    */

  

    
    if(just_viewing) {
      BASE_register_auto_update_type data;
      data.subscribe_status_report = 1;
      data.subscribe_sonar_report  = 0;
      data.subscribe_colli_report  = 0; 
      
      tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
			  / sizeof(TCX_REG_MSG_TYPE));  
      
      tcxRegisterHandlers(BASE_reply_handler_array,
			  sizeof(BASE_reply_handler_array)
			  / sizeof(TCX_REG_HND_TYPE));
/*  tcxRegisterHandlers(SONAR_reply_handler_array,
		      sizeof(SONAR_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE)); */

      fprintf(stderr, "Connecting to %s...", moduleName[BASE_MODULE]);
      BASE = tcxConnectModule(moduleName[BASE_MODULE]);
      fprintf(stderr, "done.\n");
      
      fprintf(stderr, "%10s:%5d:%14s(): tcxSendMsg\n",
	      __FILE__, __LINE__, __FUNCTION__);

      tcxSendMsg(BASE, "BASE_register_auto_update", &data);
    }
    else {
      tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
			  / sizeof(TCX_REG_MSG_TYPE));
      tcxRegisterHandlers(BASE_reply_handler_array,
			  sizeof(BASE_reply_handler_array)
			  / sizeof(TCX_REG_HND_TYPE));      
      tcxRegisterHandlers(SIMULATOR_handler_array, 
			  sizeof(SIMULATOR_handler_array)
			  / sizeof(TCX_REG_HND_TYPE));
    } 
    tcxRegisterCloseHnd(SIMULATOR_close_handler);
    
    fprintf(stderr, "done.\n");
  }
  else
    tcx = 0;
}
