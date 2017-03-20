#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>

#define TCX_define_variables  
#define DEFINE_REPLY_HANDLERS


#include "tcx.h"
#include "tcxP.h"

#include "RobotServer-messages.h"

char* TCX_MODULE_NAME = NULL;

uint RobotServerConnected = 0;
uint auto_update_active   = 0;
struct timeval query_time;

uint finish = 0;

struct timeval TCX_waiting_time = {0, 0};  /* This is for TCX only */

/************************************************************************
 *
 *   NAME:         subscribe_to_auto_update
 *                 
 *   FUNCTION:     Subscribes to the module >hook< for auto-updates
 *                 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void auto_update_control(int number, TCX_MODULE_PTR hook)
{
  RobotServer_register_auto_update_type control;
  char message[255];
  struct timeval control_time;

  /* If number = 0 then unsubscribe from auto_update
     If number = 1 the subscribe to auto_update */
  
#if TOTAL_debug
  if (number)
    fprintf(stdout,"--->subscribe_to_auto_update\n");
  else
    fprintf(stdout,"--->unsubscribe_to_auto_update\n");    
  fflush(stdout);
#endif

  strcpy(message, tcxModuleName(hook));
  if (number)
    strcat(message, "_register_auto_update");
  else
    strcat(message, "_remove_auto_update");

  gettimeofday(&control_time, NULL);

  control.status = number;
  control.ID     = control_time.tv_sec;
  control.type   = 0;

  tcxSendMsg(RobotServer, message, &control);

  auto_update_active = number;
}


/**************************************************************/
void RobotServer_query(uint lswitch)
{
  RobotServer_query_type query;
  char message[255];
  
#if TOTAL_debug
  fprintf(stdout,"--->RobotServer_query\n");
  fflush(stdout);
#endif

  gettimeofday(&query_time, NULL);

  query.ID     = query_time.tv_sec;
  query.type   = lswitch; /* 0:get list of robots 1: get into list */
  
  if (RobotServerConnected) {
    
    strcpy(message, "RobotServer");
    strcat(message, "_query");

    tcxSendMsg(RobotServer, message, &query);
  }
  else {
    fprintf(stderr, "\n\nRobotServer_query failed - RobotServer not connected\n\n");
    exit(0);
  }
}


void RobotServer_reply_handler(TCX_REF_PTR ref, RobotServer_reply_ptr RS_module)
{
  unsigned int counter = 0;
  char message[255];

#if TOTAL_debug
  fprintf(stdout,"\n--->RobotServer_reply_handler\n");
  fflush(stdout);
#endif

  for (counter = 0; counter < RS_module->counter; counter++) {
    fprintf(stderr, "\nRobot >%s< is active", RS_module->robot_list[counter]);
  }

  //fprintf(stderr, "\n\n--->%ld<--->%d<---reply",RS_module->ID, RS_module->counter);
  strcpy(message, "RobotServer");
  strcat(message, "_reply");

  tcxFree( message, RS_module);

  if (!(auto_update_active))
  finish = 1;
}



/************************************************************************
 *
 *   NAME:         connect_to_module
 *                 
 *   FUNCTION:     Connects to a module pointed to by hook
 *                 
 *   PARAMETERS:   wait_till_established 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

int connect_to_module(int wait_till_established, int *module_connected, char *name, TCX_MODULE_PTR *hook)
{

#if TOTAL_debug
  fprintf(stdout,"--->connect_to_module\n");
  fflush(stdout);
#endif
  
  if ( wait_till_established ) { /* 1 */
    fprintf(stderr, "%s: Connecting to %s...\n", TCX_MODULE_NAME, name);
    *hook = tcxConnectModule(name);
    *module_connected = 1;
    fprintf(stderr, "Connected.\n");
  } else {                      /* 0 */
    if ( (*module_connected) == 0 || !(*hook) ) { /* doppelt haelt besser */
      fprintf(stderr, "%s: Connecting to %s...\n", TCX_MODULE_NAME, name);
      *hook  = tcxConnectOptional(name);
      if( (*hook) ) {
        *module_connected = 1;
        fprintf(stderr, "Connected.\n");
      } else {
        *module_connected = 0;
      }
    }
  }

  return 0;
}


/************************************************************************
 *
 *   NAME:         close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void close_handler(char *name, TCX_MODULE_PTR module)
{

#ifdef TOTAL_debug
  fprintf(stderr, "%s: closed connection detected: %s\n",
          TCX_MODULE_NAME,
          name);
#endif

  /* TCX shut down */
  if (!strcmp(name, "TCX Server")) {
    fprintf(stderr,"\n\n!!! TCX died and so do I !!!\n\n");
    exit(0);
  }
  
  if (!strcmp(name, "RobotServer")) {
    fprintf(stderr,"\n\n!!! RobotServer died and so do I !!!\n\n");
    exit(0);
  }
}


/************************************************************************
 *
 *   NAME:         interrupt_handler()
 *                 
 *   FUNCTION:     some ^C signal or kill command arrived
 *                 
 *   PARAMETERS:   int sig           signal
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void interrupt_handler(int sig)
{

#if TOTAL_debug
  fprintf(stdout,"--->interrupt_handler\n");
  fflush(stdout);
#endif

  fprintf(stderr, "\n\nProgrm interrupteed by signal >%d<.\n\n", sig);
  fflush(stderr);
  fflush(stdout);
  exit(0);
}


/************************************************************************
 *
 *   NAME:         init_tcx()
 *                 
 *   FUNCTION:     initializes TCX communication and connects
 *                 to the RobotServer.
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void init_tcx(char* robotName)
{
  char *tcxMachine = NULL;
  
  TCX_REG_MSG_TYPE TCX_message_array[] = { RobotServer_messages };

#if TOTAL_debug
  fprintf(stdout,"--->init_tcx\n");
  fflush(stdout);
#endif

  signal(SIGTERM, &interrupt_handler);  /* kill interupt handler      */
  signal(SIGINT,  &interrupt_handler);  /* control-C interupt handler */
  
  fprintf(stderr, "Connecting to TCX...");
  tcxMachine = getenv("TCXHOST");   
  
  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize(TCX_MODULE_NAME, (void *) tcxMachine);
  
  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));
  
  tcxRegisterHandlers(RobotServer_reply_handler_array,
		      sizeof(RobotServer_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
  
  tcxRegisterCloseHnd(close_handler); 

  fprintf(stderr, "done.\n");

  RobotServer = NULL;

  connect_to_module(1, &RobotServerConnected, "RobotServer", &RobotServer);
  /* 1 -> wait until connection has been established */

}

main( int argc, char* argv[])
{
  char *robotName = NULL;
    
  int i = 0;
  uint bug = 0;

  uint get   = 0,
       add   = 0,
       sub   = 0,
       unsub = 0;

  robotName = (char *) calloc(64, sizeof(char));
  
  for (i = 1; i < argc; i++) {
    if ((strcmp(argv[i],"-robot")==0)){
      if ((i+1 < argc) && (argv[i+1][0] != '-')) {
        i++;
        strncpy( robotName, argv[i], 64 );
      }
      else {
        fprintf(stderr, "ERROR: robot name must follow keyword robot.\n");
        exit(0);
      }
    }
    else if(strcmp(argv[i], "-get")==0) {
      get = 1;
    }
    else if(strcmp(argv[i], "-add")==0) {
      add = 1;
    }
    else if(strcmp(argv[i], "-sub")==0) {
      sub = 1;
    }
    else if(strcmp(argv[i], "-unsub")==0) {
      unsub = 1;
    }
    else
      bug = 1;
  }

  if (bug) {
    fprintf(stderr, "\nRobotServer_query_example [-get] [-add] [-sub] [-unsub] [-robot <name>]\n");
    exit(0);
  }

  
  if ( (robotName != NULL) && (strcmp(robotName,"")))
    TCX_MODULE_NAME = strdup(robotName);
  else
    TCX_MODULE_NAME = strdup("ask_for_robots");
  
  
  init_tcx(robotName);
  
  while(!finish){

    if (add) {
      RobotServer_query(1);
      add =0;
    } 

    if (get) {
      RobotServer_query(0);
      get = 0;
    } 
    
    if (sub) {
      auto_update_control(1, RobotServer);
      sub = 0;
    }
    
    if (unsub) {
      auto_update_control(0, RobotServer);
      unsub = 0;
      finish = 1;
    }
    
    usleep (1000000);
    
    tcxRecvLoop((void *) &TCX_waiting_time);
    
  }

  fprintf(stdout,"\n");
  fflush(stdout);
  
  exit (0);
}
