/*
 * $Id: laserHandlers.c,v 1.1 2002/09/14 16:33:01 rstone Exp $
 *
 */ 
#include <stdlib.h>
#include <stdio.h>
#include <rai.h>
#include <bUtils.h>
#include "tcx.h"
#include "robot_specifications.h"
#include "tcxP.h"
#include "global.h"
#define TCX_define_variables  /* this makes sure variables are installed */
#include "devUtils.h"
#include "handlers.h"
#include "io.h"
#include "LASER_SERVER-messages.h"
#include <signal.h>
#include <sys/mman.h>
#include <Common.h>
#include "laserHandlers.h"
#include "laserClient.h"
#include "beeSoftVersion.h"
#include "raiClient.h"
#include "mainlaser.h"

#include "raiClient.h"
#include "raiServer.h"
#include "baseMessages.h"

void commShutdown( );

TCX_MODULE_PTR  BASESERVER  = NULL;
int base_server_connected = 0;
extern int status;
void
handleBaseClientFixed(TCX_REF_PTR message,RAI_FixedMsgType *msg_ptr);
void
handleBaseClientVariable(TCX_REF_PTR message,RAI_VariableMsgType *msg_ptr);
void connect_to_BaseServer(void);
TCX_REG_HND_TYPE BASE_CLIENT_HANDLERS[] = {
  {"baseFixed", "handleBaseClientFixed",
     (void (*)()) handleBaseClientFixed, TCX_RECV_ALL, NULL},
  {"baseVar", "handleBaseClientVariable",
     (void (*)()) handleBaseClientVariable, TCX_RECV_ALL, NULL}
};


TCX_REG_HND_TYPE LASER_SERVER_handler_array[] = {

  {"LASER_SERVER_sweep_query", "LaserSweepQueryHandler",
   LaserSweepQueryHandler, TCX_RECV_ALL, NULL},
  
  {"LASER_SERVER_register_auto_update", "LaserRegisterAutoUpdateHandler",
   LaserRegisterAutoUpdateHandler, TCX_RECV_ALL, NULL}

};

/******* auto-reply stuff, data definitions ***************/


#define MAX_N_AUTO_UPDATE_MODULES 100


int n_auto_update_modules = 0; /* number of processes to whom
				* position should be forwarded
				* automatically upon change */

typedef struct{
  TCX_MODULE_PTR module;	/* pointer to TCX module */
  int            sweep0;	/* >=1=subscribed to regular sweep updates */
  int            last_sweep0;
  int            sweep1;	/* >=1=subscribed to regular sweep updates */
  int            last_sweep1;
} auto_update_type;


auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];
                                /* collection of TCX-module ptrs */

/************************************************************************
 *   FUNCTION:     Counts, how many modules of the different types require
 *                 auto-updates
 *   RETURN-VALUE: 1, if successful, 0 if not
 ************************************************************************/
void count_auto_update_modules()
{

  int i=0;

#if TOTAL_debug
  fprintf(stdout,"\n--->count_auto_update_modules\n");
  fflush( stdout );
#endif

  n_auto_sweep0_update_modules      = 0;
  n_auto_sweep1_update_modules      = 0;

  for (i = 0; i < n_auto_update_modules; i++) {

    if (auto_update_modules[i].sweep0)
      n_auto_sweep0_update_modules++;

    if (auto_update_modules[i].sweep1)
      n_auto_sweep1_update_modules++;

  }

}

/************************************************************************
 *   FUNCTION:     Adds a module to the list of modules that
 *                 get automatical map updates
 *   RETURN-VALUE: 1, if successful, 0 if not
 ************************************************************************/
static int add_auto_update_module( TCX_MODULE_PTR                 module,
				   LASER_SERVER_register_auto_update_ptr data)
{
  int i=0;

#if TOTAL_debug
  fprintf(stdout,"\n--->add_auto_update_module\n");
  fflush( stdout );
#endif

  if (n_auto_update_modules >= MAX_N_AUTO_UPDATE_MODULES){

    fprintf(stderr, "ERROR: auto-module memory overflow. Request rejected.\n");

    return 0;

  } else for (i = 0; i != n_auto_update_modules; i++)

    if (auto_update_modules[i].module == module) {
      fprintf( stderr, 
	       "Module %s already known. Subscription modified: %d %d\n",
	       tcxModuleName(module), 
	       data->sweep0,
	       data->sweep1 );
      auto_update_modules[i].sweep0      = data->sweep0; /* subsrc? */
      auto_update_modules[i].last_sweep0 = -1;
      auto_update_modules[i].sweep1      = data->sweep1; /* subsrc? */
      auto_update_modules[i].last_sweep1 = -1;
      count_auto_update_modules();
      return 1;
    }

  fprintf( stderr, "Add %s to auto-reply list: %d %d.\n",
	   tcxModuleName(module), 
	   data->sweep0,
	   data->sweep1 );

  auto_update_modules[n_auto_update_modules].module     = module; /* pointer*/
  auto_update_modules[n_auto_update_modules].sweep0      = data->sweep0;
  auto_update_modules[n_auto_update_modules].last_sweep0 = -1;
  auto_update_modules[n_auto_update_modules].sweep1      = data->sweep1; 
  auto_update_modules[n_auto_update_modules].last_sweep1 = -1;

  n_auto_update_modules++;
  count_auto_update_modules();

  return 1;
}

/************************************************************************
 *   FUNCTION:     Attempts to remove a module from the list of 
 *                 modules that get automatical map updates 
 *   RETURN-VALUE: 1, if successful, 0 if not
 ************************************************************************/
static int remove_auto_update_module(TCX_MODULE_PTR module)
{     

  int i=0, j=0, found = 0, tmp = 0;

#if TOTAL_debug
  fprintf(stdout,"\n--->remove_auto_update_module\n");
  fflush( stdout );
#endif

  for (i = 0; i < n_auto_update_modules; i++) { /* search for module */
    if (auto_update_modules[i].module == module){ /* if module found */

      fprintf( stderr, "Remove %s from auto-reply list.\n", tcxModuleName(module));

      found++;

      n_auto_update_modules--;	/* remove that entry, one less now */
      for (j = i; j < n_auto_update_modules; j++){
	tmp = j +1;
	auto_update_modules[j].module = auto_update_modules[tmp].module;

	auto_update_modules[j].sweep0 =  auto_update_modules[tmp].sweep0;
	auto_update_modules[j].last_sweep0 = auto_update_modules[tmp].last_sweep0;

	auto_update_modules[j].sweep1 =  auto_update_modules[tmp].sweep1;
	auto_update_modules[j].last_sweep1 = auto_update_modules[tmp].last_sweep1;
      }
    }
  }
  
  if (!found)
    fprintf(stderr, "(%s: no auto-replies removed)\n", tcxModuleName(module));

  count_auto_update_modules();

  return found;

}

/************************************************************************
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 ************************************************************************/
void send_automatic_sweep_update( int numLaser, float *values )
{

  int i=0;

#if TOTAL_debug
  fprintf(stdout,"\n--->send_automatic_sweep_update\n");
  fflush( stdout );
#endif

#if ( defined(TCX_debug) )
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if ( numLaser == 0 ) { 

    for (i = 0; i != n_auto_update_modules; i++){
      if (auto_update_modules[i].sweep0 > 0){
	
	auto_update_modules[i].last_sweep0++;
	if (auto_update_modules[i].last_sweep0 %
	    auto_update_modules[i].sweep0 == 0){
#ifdef LASER_debug	
	  fprintf(stderr, "Send sweep from laser 0 to %s.\n", 
		  tcxModuleName(auto_update_modules[i].module));
#endif
	  /*fprintf(stderr, "+");*/
	  auto_update_modules[i].last_sweep0 = 0;
	  
	  LaserSendSweepTo( auto_update_modules[i].module, 0, values );
	}
      }
    }

  } else if ( numLaser == 1 ) {

    for (i = 0; i != n_auto_update_modules; i++){
      if (auto_update_modules[i].sweep1 > 0){
	
	auto_update_modules[i].last_sweep1++;
	if (auto_update_modules[i].last_sweep1 %
	    auto_update_modules[i].sweep1 == 0){
#ifdef LASER_debug	
	  fprintf(stderr, "Send sweep from laser 1 to %s.\n", 
		  tcxModuleName(auto_update_modules[i].module));
#endif
	  fprintf(stderr, "+");
	  auto_update_modules[i].last_sweep1 = 0;
	  
	  LaserSendSweepTo( auto_update_modules[i].module, 1, values );
	}
      }
    }

  } else {

    /* ouch! */

  }

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void LaserRegisterAutoUpdateHandler( TCX_REF_PTR  ref,
					 LASER_SERVER_register_auto_update_ptr data)
{
#if TOTAL_debug
  fprintf(stdout,"\n--->LaserRegisterAutoUpdateHandler\n");
  fflush( stdout );
#endif

  fprintf(stderr, "Received an auto update request from %s: %d %d.",
	tcxModuleName(ref->module), data->sweep0, data->sweep1);

  add_auto_update_module(ref->module, data);
  
  if (data != NULL){
    tcxFree("LASER_SERVER_register_auto_update", data);
    data = NULL;
  }
}

/************************************************************************
 *   FUNCTION:     general initialization routine - must be called before
 *                 anything else. Returns error value (0=success, 1=error)
 ************************************************************************/
int LaserInitializeTCX( char* robotName)
{
  
  /* messages used by LASER */
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    LASER_SERVER_messages,
    {"baseTCXServer", NULL},
    {"baseFixed", "{int, long}"},
    {"baseVar", "{int,int,<char: 2>}"}
  };
  
  /* TCX */

#if TOTAL_debug
  fprintf(stdout,"\n--->LaserInitializeTCX\n");
  fflush( stdout );
#endif
  
  fprintf(stderr, "Connecting to TCX...");

  if ( robotName != NULL)
    tcxSetModuleNameExtension( robotName);

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize(TCX_LASER_SERVER_MODULE_NAME, tcxMachine);

  check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
		       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
		       NULL, 0);

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));
  
  /* Handlers */
  
  tcxRegisterHandlers(LASER_SERVER_handler_array, 
		      sizeof(LASER_SERVER_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers(BASE_CLIENT_HANDLERS,
		      sizeof(BASE_CLIENT_HANDLERS)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterCloseHnd(LaserCloseHandler);
  
  /* Return */
  return (1) ; 
  
}

/************************************************************************
 *   FUNCTION:     handles a close message (special case)
 ************************************************************************/
void LaserCloseHandler(char *name, TCX_MODULE_PTR module)
{

#if TOTAL_debug
  fprintf(stdout,"\n--->LaserCloseHandler\n");
  fflush( stdout );
#endif

#ifdef LASER_debug
  fprintf(stderr, "LASER: closed connection detected: %s\n", name);
#endif

  fprintf(stderr, "LASER: closed connection detected: %s\n", name);
  remove_auto_update_module(module);
  fprintf(stderr, "REMOVED\n");
  if ( BASESERVER != NULL) {
    if (!strcmp( name, tcxModuleName(BASESERVER))){ /* BASE shut down */
      base_server_connected = 0;
      BASESERVER = NULL;
      fprintf(stderr, "TCX: disconnected from BaseServer.\n");
    }
  }

  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    commShutdown(NULL);
  }

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void LaserSweepQueryHandler( TCX_REF_PTR            ref,
			     LASER_SERVER_sweep_query_ptr  data )
{

#if TOTAL_debug
  fprintf(stdout,"\n--->LaserSweepQueryHandler\n");
  fflush( stdout );
#endif

#ifdef TCX_debug
  fprintf(stderr, "Received a  LASER_SERVER_sweep_query_message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  if (data->numLaser != 0 && data->numLaser != 1)
    fprintf( stderr,"%s: laser %d not yet supported\n", __FILE__,
	     data->numLaser);
  else
    LaserSendSweepTo( ref->module, data->numLaser,
		      LaserSensors[data->numLaser].values );



  tcxFree( "LASER_SERVER_sweep_query", data );

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void LaserSendSweepTo( TCX_MODULE_PTR module, int numLaser, float *values )
{

  LASER_SERVER_sweep_reply_type sweep;
  int i=0;
 
#if TOTAL_debug
  fprintf(stdout,"\n--->LaserSendSweepTo\n");
  fflush( stdout );
#endif
 
  /* get the current reading and store the stuff in sweep.value[i] */

  sweep.numberLasers = NUMBER_LASERS;

  sweep.numLaser = numLaser;

  sweep.value = (float *) calloc(NUMBER_LASERS, sizeof(float));
  
  for (i=0; i != NUMBER_LASERS; i++)
    sweep.value[i] = values[i];

#ifdef TCX_debug 
  fprintf(stderr, "TCX: Sending LASER_SERVER_sweep_reply.\n");
#endif

  tcxSendMsg(module, "LASER_SERVER_sweep_reply", &sweep);

  free( sweep.value );

}



/************************************************************************
 *
 *   NAME:         connect_to_BaseServer
 *                 
 *   FUNCTION:     checks, and connects to BASESERVER, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void connect_to_BaseServer(void)
{
  struct timeval current_time;
  static struct timeval last_attempt_connect_BASESERVER = {0, 0};

#if TOTAL_debug
  fprintf(stdout,"\n--->connect_to_BaseServer\n");
  fflush( stdout );
#endif

  if(!base_server_connected){

    gettimeofday(&current_time, NULL);
    if (current_time.tv_sec < last_attempt_connect_BASESERVER.tv_sec + 5
	|| (current_time.tv_sec == last_attempt_connect_BASESERVER.tv_sec + 5 &&
	    current_time.tv_usec < last_attempt_connect_BASESERVER.tv_usec))
      return;
    
    last_attempt_connect_BASESERVER.tv_sec  = current_time.tv_sec;
    last_attempt_connect_BASESERVER.tv_usec = current_time.tv_usec;
    

    BASESERVER = tcxConnectOptional("baseTCXServer"); /* checks, but does 
						       * not wait */

    if (BASESERVER != NULL){
      base_server_connected = 1;
      fprintf(stderr, "TCX: connected to BaseServer.\n");
      {
	RAI_FixedMsgType command;
	command.operation = BASE_subscribe;
	command.parameter = 1;
	tcxSendMsg(BASESERVER,"baseFixed",&command);
      }
    }
  }
}


void
handleBaseClientFixed(TCX_REF_PTR message,RAI_FixedMsgType *msg_ptr)
{
#if TOTAL_debug
  fprintf(stdout,"\n--->handleBaseClientFixed\n");
  fflush( stdout );
#endif

  fprintf(stderr, "-1-");
  tcxFree("baseFixed", msg_ptr);
}

statusReportType activeStatusReport;

void
incorporateStatusReport(statusReportType* newReport)
{
  /*
   * NOTE!
   *
   * This only works because the entire struct
   * is longs and not a mix of types.  If the struct becomes
   * other than all longs this will likely fail do to different
   * architectures packing structures differently.  -tds
   */

#if TOTAL_debug
  fprintf(stdout,"\n--->incorporateStatusReport\n");
  fflush( stdout );
#endif

  activeStatusReport.Request            = ntohl(newReport->Request);
  activeStatusReport.Clock              = ntohl(newReport->Clock);
  activeStatusReport.GeneralStatus      = ntohl(newReport->GeneralStatus);
  activeStatusReport.Xpos               = ntohl(newReport->Xpos);
  activeStatusReport.Ypos               = ntohl(newReport->Ypos);
  activeStatusReport.Heading            = ntohl(newReport->Heading);
  activeStatusReport.BaseRelativeHeading
                                   = ntohl(newReport->BaseRelativeHeading);
  activeStatusReport.TranslateError     = ntohl(newReport->TranslateError);
  activeStatusReport.TranslateVelocity  = ntohl(newReport->TranslateVelocity);
  activeStatusReport.TranslateStatus    = ntohl(newReport->TranslateStatus);
  activeStatusReport.RotateError        = ntohl(newReport->RotateError);
  activeStatusReport.RotateVelocity     = ntohl(newReport->RotateVelocity);
  activeStatusReport.RotateStatus       = ntohl(newReport->RotateStatus);

  /*  fprintf(stderr, "Status Report: cl=%ld x=%ld y=%ld o=%ld\n",
	  activeStatusReport.Clock,
	  activeStatusReport.Xpos,
	  activeStatusReport.Ypos,
	  activeStatusReport.Heading);*/

  fprintf(stderr, "?");
}

void
handleBaseClientVariable(TCX_REF_PTR message,RAI_VariableMsgType *msg_ptr)
{
  int operation=0;
  int bufsize=0;
  unsigned char* buffer=NULL;
  operation = msg_ptr->operation;
  bufsize = msg_ptr->bufsize;
  buffer = msg_ptr->buffer;

#if TOTAL_debug
  fprintf(stdout,"\n--->handleBaseClientVariable\n");
  fflush( stdout );
#endif

  if (operation == BASE_statusReport)
    incorporateStatusReport( (statusReportType*) buffer);
  tcxFree("baseVar", msg_ptr);
}

/*
 * $Log: laserHandlers.c,v $
 * Revision 1.1  2002/09/14 16:33:01  rstone
 * *** empty log message ***
 *
 * Revision 1.14  1999/09/24 14:35:33  fox
 * Added support for scout.
 *
 * Revision 1.13  1999/07/20 14:25:44  schneid1
 * laserServer now uses beeSoft.ini & -display works again
 *
 * Revision 1.12  1998/11/04 00:02:59  fox
 * Added usleep in waitForAck.
 *
 * Revision 1.11  1998/10/30 18:58:03  fox
 * Added support for pioneers and multiple robots.
 *
 * Revision 1.10  1998/08/16 20:22:14  thrun
 * Can now receive status information, but doesn't do anything with it
 * yes. Only activated with "-status=1" option.
 *
 * Revision 1.9  1998/08/16 19:02:22  thrun
 * Will this run at 38400 baud?
 * THere is still stuff missing. Right now, it basically
 * ignores beeSoft.ini and runs with both lasers always.
 *
 * Revision 1.8  1998/08/14 16:30:45  thrun
 * .
 *
 * Revision 1.7  1998/08/06 03:22:55  thrun
 * supports 2 lasers.
 *
 * Revision 1.6  1998/01/13 23:13:38  thrun
 * .
 *
 * Revision 1.5  1998/01/13 22:41:41  thrun
 * resolved a naming conflict. CHanged LASER to LASER_SERVER.
 *
 * Revision 1.4  1997/11/06 17:56:21  swa
 * Fixed a bug with TCXHOST. If the environment variable TCXHOST is set,
 * take it. If not, take the TCXHOST from beeSoft.ini. If this doesn't
 * work, take localhost. If this doesn't work: bummer.
 *
 * Revision 1.3  1997/08/07 15:42:53  swa
 * First working version of the laserServer. Seems to run fine. Has support for
 * a second laser clientwise, yet the device stuff for the second one is not
 * implemented.
 *
 * Revision 1.2  1997/08/07 03:50:21  swa
 * Fixed a bunch of bugs. Still not working.
 *
 * Revision 1.1  1997/08/07 02:45:52  swa
 * First version of a laserServer. Compiles nicely, yet produces nothing useful. ;)
 * Just to copy things to a safe place ...
 *
 *
 */
