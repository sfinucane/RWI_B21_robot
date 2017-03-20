/*
 * $Id: laserHandlers.c,v 1.1 2002/09/14 16:33:04 rstone Exp $
 *
 */ 
#include <signal.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "tcx.h"
#include "robot_specifications.h"
#include "tcxP.h"
#include "global.h"

#define TCX_define_variables  /* this makes sure variables are installed */
#include "io.h"
#include "LASER_SERVER-messages.h"
#include "laserHandlers.h"
#include "mainlaser.h"

int n_auto_sweep0_update_modules;
int n_auto_sweep1_update_modules;

TCX_MODULE_PTR  BASESERVER  = NULL;

TCX_REG_HND_TYPE LASER_SERVER_handler_array[] = {
  {"LASER_SERVER_sweep_query", "LaserSweepQueryHandler",
   LaserSweepQueryHandler, TCX_RECV_ALL, NULL},
  {"LASER_SERVER_register_auto_update", "LaserRegisterAutoUpdateHandler",
   LaserRegisterAutoUpdateHandler, TCX_RECV_ALL, NULL}
};

#define MAX_N_AUTO_UPDATE_MODULES 100

extern void commShutdown( void );

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

void 
count_auto_update_modules() {

  int i= 0;

  n_auto_sweep0_update_modules      = 0;
  n_auto_sweep1_update_modules      = 0;

  for (i = 0; i < n_auto_update_modules; i++) {

    if (auto_update_modules[i].sweep0)
      n_auto_sweep0_update_modules++;

    if (auto_update_modules[i].sweep1)
      n_auto_sweep1_update_modules++;

  }

}

int 
add_auto_update_module( TCX_MODULE_PTR                 module,
			LASER_SERVER_register_auto_update_ptr data)
{
  int i= 0;

  if (n_auto_update_modules >= MAX_N_AUTO_UPDATE_MODULES){
#ifdef TCX_DEBUG
    fprintf(stderr, "ERROR: auto-module memory overflow. Request rejected.\n");
#endif
    return 0;
  } else for (i = 0; i < n_auto_update_modules; i++)

    if (auto_update_modules[i].module == module) {
#ifdef TCX_DEBUG
      fprintf( stderr, 
	       "Module %s already known. Subscription modified: %d %d\n",
	       tcxModuleName(module), 
	       data->sweep0,
	       data->sweep1 );
#endif
      auto_update_modules[i].sweep0      = data->sweep0; /* subsrc? */
      auto_update_modules[i].last_sweep0 = -1;
      auto_update_modules[i].sweep1      = data->sweep1; /* subsrc? */
      auto_update_modules[i].last_sweep1 = -1;
      count_auto_update_modules();
      return 1;
    }

#ifdef TCX_DEBUG
  fprintf( stderr, "Add %s to auto-reply list: %d %d.\n",
	   tcxModuleName(module), 
	   data->sweep0,
	   data->sweep1 );
#endif

  auto_update_modules[n_auto_update_modules].module     = module; /* pointer*/
  auto_update_modules[n_auto_update_modules].sweep0      = data->sweep0;
  auto_update_modules[n_auto_update_modules].last_sweep0 = -1;
  auto_update_modules[n_auto_update_modules].sweep1      = data->sweep1; 
  auto_update_modules[n_auto_update_modules].last_sweep1 = -1;

  n_auto_update_modules++;
  count_auto_update_modules();

  return 1;
}

int 
remove_auto_update_module(TCX_MODULE_PTR module) {     

  int i= 0, j= 0, found = 0;

  for (i = 0; i < n_auto_update_modules; i++) /* search for module */

    if (auto_update_modules[i].module == module){ /* if module found */

#ifdef TCX_DEBUG
      fprintf( stderr, "Remove %s (module nr.%d/%d) from auto-reply list.\n", 
	       tcxModuleName(module), i, n_auto_update_modules );
#endif
      found++;

      n_auto_update_modules--;	/* remove that entry, one less now */

      for (j = i; j < n_auto_update_modules; j++){
	auto_update_modules[j].module = auto_update_modules[j+1].module;

	auto_update_modules[j].sweep0 =  auto_update_modules[j+1].sweep0;
	auto_update_modules[j].last_sweep0 = auto_update_modules[j+1].last_sweep0;

	auto_update_modules[j].sweep1 =  auto_update_modules[j+1].sweep1;
	auto_update_modules[j].last_sweep1 = auto_update_modules[j+1].last_sweep1;

      }
    }

#ifdef TCX_DEBUG
  if (!found)
    fprintf(stderr, "(%s: no auto-replies removed)\n", tcxModuleName(module));
#endif

  count_auto_update_modules();

  return found;

}

void 
send_automatic_sweep_update( int numLaser, int numValues, float *values,
			     struct timeval timestamp ) {

  int i= 0;

#ifdef TCX_DEBUG
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  if ( numLaser == 0 ) { 

    for (i = 0; i < n_auto_update_modules; i++){
      if (auto_update_modules[i].sweep0 > 0){
	
	auto_update_modules[i].last_sweep0++;
	if (auto_update_modules[i].last_sweep0 %
	    auto_update_modules[i].sweep0 == 0){
#ifdef TCX_DEBUG
	  fprintf(stderr, "Send sweep from laser 0 to %s.\n", 
		  tcxModuleName(auto_update_modules[i].module));
#endif
	  auto_update_modules[i].last_sweep0 = 0;
	  
	  LaserSendSweepTo( auto_update_modules[i].module, 0, 
			    numValues, values, timestamp );
	}
      }
    }


  } else if ( numLaser == 1 ) {

    for (i = 0; i < n_auto_update_modules; i++){
      if (auto_update_modules[i].sweep1 > 0){
	
	auto_update_modules[i].last_sweep1++;
	if (auto_update_modules[i].last_sweep1 %
	    auto_update_modules[i].sweep1 == 0){
#ifdef TCX_DEBUG
	  fprintf(stderr, "Send sweep from laser 1 to %s.\n", 
		  tcxModuleName(auto_update_modules[i].module));
#endif
	  /*fprintf(stderr, "+");*/
	  auto_update_modules[i].last_sweep1 = 0;
	  
	  LaserSendSweepTo( auto_update_modules[i].module, 1, 
			    numValues, values, timestamp );
	}
      }
    }


  } else {

    /* ouch! */

  }

}

/* ---------------------------------------------------------
 * --------------------------------------------------------*/

void
LaserRegisterAutoUpdateHandler( TCX_REF_PTR  ref,
				LASER_SERVER_register_auto_update_ptr data)
{
#ifdef TCX_DEBUG
  fprintf(stderr, "Received an auto update request from %s: %d %d.",
	tcxModuleName(ref->module), data->sweep0, data->sweep1);
#endif

  add_auto_update_module(ref->module, data);
  
  if (data != NULL){
    tcxFree("LASER_SERVER_register_auto_update", data);
    data = NULL;
  }
}

/************************************************************************
 ************************************************************************/

int 
LaserInitializeTCX( char* robotName) {
  
  /* messages used by LASER */
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    LASER_SERVER_messages
  };
  
  /* TCX */
  
  fprintf(stderr, "Connecting to TCX...");

  if ( (robotName != NULL) && (strcasecmp(robotName,"")) )
    tcxSetModuleNameExtension( robotName);

  tcxInitialize(TCX_LASER_SERVER_MODULE_NAME, (char *) getenv("TCXHOST"));

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));
  
  /* Handlers */
  
  tcxRegisterHandlers(LASER_SERVER_handler_array, 
		      sizeof(LASER_SERVER_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterCloseHnd(LaserCloseHandler);
  
  /* Return */
  return (1) ; 
  
}

/************************************************************************
 *   FUNCTION:     handles a close message (special case)
 ************************************************************************/
void
LaserCloseHandler(char *name, TCX_MODULE_PTR module) {

#ifdef TCX_DEBUG
  fprintf(stderr, "LASER: closed connection detected: %s\n", name);
#endif

  remove_auto_update_module(module);
  
  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    commShutdown();
  }

}

/* ---------------------------------------------------------
 *
 *
 * --------------------------------------------------------*/
void
LaserSweepQueryHandler( TCX_REF_PTR            ref,
			LASER_SERVER_sweep_query_ptr  data ) {

#ifdef TCX_DEBUG
  fprintf(stderr, "Received a  LASER_SERVER_sweep_query_message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  if ((data->numLaser != 0) && (data->numLaser != 1)) {
#ifdef TCX_DEBUG
    fprintf( stderr,"%s: laser %d not yet supported\n", __FILE__,
	     data->numLaser);
#endif
  } else {
    LaserSendSweepTo( ref->module, data->numLaser,
		      LaserSensors[data->numLaser].num_values,
		      LaserSensors[data->numLaser].values,
		      LaserSensors[data->numLaser].timestamp );
  }


  tcxFree( "LASER_SERVER_sweep_query", data );

}

/* ---------------------------------------------------------
 * --------------------------------------------------------*/

void 
LaserSendSweepTo( TCX_MODULE_PTR module, int numLaser, 
		  int numValues, float *values, struct timeval timestamp ) {

  LASER_SERVER_sweep_reply_type sweep;
  int i= 0;
  
  /* get the current reading and store the stuff in sweep.value[i] */

  sweep.numberLasers = numValues;

  sweep.numLaser = numLaser;
  sweep.timeStamp.tv_sec = timestamp.tv_sec;
  sweep.timeStamp.tv_usec = timestamp.tv_usec;

  sweep.value = (float *) calloc(numValues, sizeof(float));
  
  for (i=0; i != numValues; i++)
    sweep.value[i] = values[i];

#ifdef TCX_DEBUG
  fprintf(stderr, "TCX: Sending LASER_SERVER_sweep_reply.\n");
#endif

  tcxSendMsg(module, "LASER_SERVER_sweep_reply", &sweep);
  free( sweep.value );

}







