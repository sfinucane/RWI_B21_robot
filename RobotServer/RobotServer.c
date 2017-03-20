#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>

#define TCX_define_variables  

void RobotServer_register_auto_update_handler();
void RobotServer_remove_auto_update_handler();
static int  add_auto_update_module();
void send_auto_updates();

#define DEFINE_QUERY_HANDLERS

#include "tcx.h"
#include "tcxP.h"

#include "RobotServer-messages.h"

#define TCX_MODULE_NAME "RobotServer"

/* last time when a robot requested a membership */
struct timeval last_register_time; 

/* number of active robots */
uint number_of_robots = 0; 

/* list of active robots */
unsigned char **list_of_robots = NULL; 

struct timeval TCX_waiting_time = {0, 0};  /* This is for TCX only */
#define MAX_N_AUTO_UPDATE_MODULES 100

uint verbose = 0;

uint nr_auto_update_modules = 0;
RobotServer_auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];
                               /* collection of TCX-module ptrs */
extern int errno;

/************************************************************************
 *
 *   NAME:         remove_auto_update
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void RobotServer_remove_auto_update_handler(TCX_REF_PTR ref,
				  RobotServer_register_auto_update_ptr data)
{
  char message[255];
  uint test = 0;
  
#ifdef TOTAL_debug
  fprintf(stdout,"\n--->RobotServer_remove_auto_update_handler");
  fflush(stderr);
#endif

  if (verbose)
    fprintf(stderr, "\n%s: Received a remove_auto_update message from >%s<",
	    TCX_MODULE_NAME,
	    tcxModuleName(ref->module));

  test = remove_auto_update_module(ref->module);

  strcpy(message, tcxModuleName(ref->module));
  strcat(message, "_remove_auto_update");

  if (test && verbose)
    fprintf(stderr, "\n%s: %ld modules where removed", TCX_MODULE_NAME, test);
  
  if (data != NULL){
    tcxFree(message, data);
    data = NULL;
  }
}


/************************************************************************
 *
 *   NAME:         register_auto_update
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void RobotServer_register_auto_update_handler(TCX_REF_PTR ref,
				  RobotServer_register_auto_update_ptr data)
{
  char message[255];
  uint test = 0;
  
#ifdef TOTAL_debug
  fprintf(stdout,"\n--->RobotServer_register_auto_update_handler");
  fflush(stderr);
#endif

  if (verbose)
    fprintf(stderr, "\n%s: Received a register_auto_update message from >%s<.",
	  TCX_MODULE_NAME,
	  tcxModuleName(ref->module));

  test = add_auto_update_module(ref->module, data);

  strcpy(message, tcxModuleName(ref->module));
  strcat(message, "_register_auto_update");

  
  if (data != NULL){
    tcxFree(message, data);
    data = NULL;
  }

  if (test)
    send_auto_updates();
  
}



/************************************************************************
 *
 *   NAME:         add_auto_update_module()
 *                 
 *   FUNCTION:     Adds a module to the list of modules that
 *                 get automatical status updates
 *                 
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/

static int add_auto_update_module(TCX_MODULE_PTR module, RobotServer_register_auto_update_ptr data)
{
  uint i = 0;
  
#if TOTAL_debug
  fprintf(stdout,"\n--->add_auto_update_module");
  fflush(stdout);
#endif

  errno = 0;
  
  if (nr_auto_update_modules >= MAX_N_AUTO_UPDATE_MODULES){
    fprintf(stderr, "\nERROR: auto-module memory overflow. Request rejected.\n");
    return 0;
  }
  else
    for (i = 0; i < nr_auto_update_modules; i++) {
      if (auto_update_modules[i].module == module){
	if (verbose)
	  fprintf(stderr, "\n%s: Module >%s< already known. Subscription modified: %d",
		  TCX_MODULE_NAME,
		  tcxModuleName(module),
		  data->status);
	
	auto_update_modules[i].data = calloc(1, sizeof(RobotServer_reply_type));

	if (auto_update_modules[i].data == NULL) {
	  fprintf (stderr, "\nERROR: Out of memory !\n");
	  exit(EXIT_FAILURE);
	}

	if (errno) {
	  perror("\nadd_auto_update_module ");
	  exit(EXIT_FAILURE);
	}

	auto_update_modules[i].status      = data->status; /* subsrc? */
	auto_update_modules[i].last_status = -1;
	auto_update_modules[i].data->ID    = data->ID;
	auto_update_modules[i].data->type  = data->type;
	
	//if (data != NULL){
	//  free(data);
	//  data = NULL;
	//}

	return 1;
      }
    }

  if (verbose)
    fprintf(stderr, "\n%s: Add >%s< to auto-reply list: %d.", TCX_MODULE_NAME,
	    tcxModuleName(module),
	    data->status);
  
  auto_update_modules[nr_auto_update_modules].data = calloc(1, sizeof(RobotServer_reply_type));

  if (auto_update_modules[nr_auto_update_modules].data == NULL) {
    fprintf (stderr, "\nERROR: Out of memory !\n");
      exit(EXIT_FAILURE);
  }
  
  if (errno) {
    perror("\nadd_auto_update_module ");
    exit(EXIT_FAILURE);
  }

  auto_update_modules[nr_auto_update_modules].module      = module; /*new pointer*/
  auto_update_modules[nr_auto_update_modules].status      = data->status; /* subsrc? */
  auto_update_modules[nr_auto_update_modules].last_status = -1;
  auto_update_modules[nr_auto_update_modules].data->ID    = data->ID;          
  auto_update_modules[nr_auto_update_modules].data->type  = data->type;
  
  nr_auto_update_modules++;

  //if (data != NULL){
  //  free(data);
  //  data = NULL;
  //}

  return 1;
}


/************************************************************************
 *
 *   NAME:         remove_auto_update_module()
 *                 
 *   FUNCTION:     Attempts to remove a module from the list of 
 *                 modules that get automatical status updates 
 *                 
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *                 
 *                 
 *   RETURN-VALUE: number of modules removed
 *                 
 ************************************************************************/

int remove_auto_update_module(TCX_MODULE_PTR module)
{     
  uint i     = 0,
       j     = 0,
       found = 0;

#if TOTAL_debug
  fprintf(stdout,"\n--->remove_auto_update_module");
  fflush(stdout);
#endif
  
  for (i = 0; i < nr_auto_update_modules; i++) /* search for module */
    if (auto_update_modules[i].module == module){ /* if module found */
      if (verbose)
	fprintf(stderr, "\n%s: Remove >%s< from auto-reply list.",
		TCX_MODULE_NAME,
		tcxModuleName(module));
      found++;
      nr_auto_update_modules--;	/* remove that entry, one less now */

      if (nr_auto_update_modules < 0) {
	fprintf (stderr, "\nERROR: nr_auto_update_modules samller zero !\n");
	exit(EXIT_FAILURE);
      }

      for (j = i; j < nr_auto_update_modules; j++) {
	auto_update_modules[j].module = 
	  auto_update_modules[j+1].module; /* shift back */
	auto_update_modules[j].status = 
	  auto_update_modules[j+1].status; /* shift back */
	auto_update_modules[j].last_status = 
	  auto_update_modules[j+1].last_status; /* shift back */
	auto_update_modules[j].data = 
	  auto_update_modules[j+1].data; /* shift back */
      }
      free(auto_update_modules[nr_auto_update_modules].data);
    }
  
  if ((!found) && verbose)
    fprintf(stderr, "\n%s: no auto-replies removed", tcxModuleName(module));
  
  
  return found;
}



/************************************************************************/
/* Add a new robot to the list
************************************************************************/
void add_member_to_list(char *robot_name)
{
  uint test    = 0,
       counter = 0;

#if TOTAL_debug
  fprintf(stdout,"\n--->add_member_to_list");
  fflush(stdout);
#endif

  /* Check for duplicates! If robot is alreadz in list: Do not add again */
  for (counter = 0; counter < number_of_robots; counter++) {
    if (!strcasecmp(robot_name, list_of_robots[counter])) {
      if (verbose)
	fprintf(stderr, "\nRobot >%s< was already in list !", robot_name);
      test = 1;
    }
  }

  /* if new robot add to list */
  if (! test) {
    list_of_robots[number_of_robots] = strdup(robot_name);  
    number_of_robots++;
    if (verbose)
      fprintf(stderr, "\nAdd >%s< to robot list.", robot_name);      
  }
}

/************************************************************************/
/* send the updated list to all robots 
************************************************************************/
void send_auto_updates()
{
  uint counter = 0, counter2 = 0, first_time = 1;
  char message[255];
  
  RobotServer_reply_type reply;
  
#if TOTAL_debug
  fprintf(stdout,"\n--->send_auto_updates");
  fflush(stdout);
#endif

  errno = 0;

  for (counter = 0; counter < nr_auto_update_modules; counter++){
      if (auto_update_modules[counter].status > 0){
	
	auto_update_modules[counter].last_status++;
	
	if (auto_update_modules[counter].last_status %
	    auto_update_modules[counter].status == 0){
	  
	  auto_update_modules[counter].last_status = 0;
	  
	  if (verbose)
	    fprintf(stderr, "\n%s: Send update to >%s<.", TCX_MODULE_NAME,
		    tcxModuleName(auto_update_modules[counter].module));
	  
	  if (first_time) {
	    first_time = 0;
	    
	    strcpy(message, TCX_MODULE_NAME);
	    strcat(message, "_reply");
	    
	    reply.ID      = last_register_time.tv_sec;
	    reply.counter = number_of_robots;
	    
	    reply.robot_list = NULL;
	    reply.robot_list = (unsigned char **) calloc(number_of_robots, sizeof(unsigned char *));

	    if (reply.robot_list == NULL) {
	      fprintf (stderr, "\nERROR: Out of memory error! \n");
		exit(EXIT_FAILURE);
	    }
	    
	    if (errno) {
	      perror("\nsend_auto_updates ");
	      exit(EXIT_FAILURE);
	    }
	    
	    for (counter2 = 0; counter2 < number_of_robots; counter2++) 
	      reply.robot_list[counter2] = list_of_robots[counter2];

	  }
	  
	  tcxSendMsg(auto_update_modules[counter].module, message, &reply);
	}
      }
    }
    
    if (first_time == 0) { 
      reply.ID = 0;
      reply.counter = 0;
      reply.robot_list =  NULL;
      free(reply.robot_list);
    }
}


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         RobotServer_query_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void RobotServer_query_handler(TCX_REF_PTR ref, RobotServer_query_ptr RS_module)
{

  RobotServer_reply_type reply;

  uint counter = 0;
  char message[255];

#if TOTAL_debug
  fprintf(stdout,"\n--->RobotServer_query_handler");
  fflush(stdout);
#endif

  errno = 0;
  
  if (verbose)
    fprintf (stderr, "\n%s: Received a RobotServer_query message from >%s<.",
	     TCX_MODULE_NAME,
	     tcxModuleName(ref->module));
  
  
  /* if this is a membership request (type == 1): add robot to list */
  if (RS_module->type) {
    add_member_to_list(tcxModuleName(ref->module));
    
    if (gettimeofday(&last_register_time, NULL)) {
      if (errno) {
	perror("\nRobotServer_query_handler ");
	exit(EXIT_FAILURE);
      }
      else
	fprintf (stderr, "\nUndefined ERROR with gettimeofday");		     
    }

    send_auto_updates();
  }
  else { /* if this is a list request (type == 0): reply with list */
    reply.ID      = last_register_time.tv_sec;
    reply.counter = number_of_robots;

    reply.robot_list = NULL;
    reply.robot_list = (unsigned char **) calloc(number_of_robots, sizeof(unsigned char *));

    if (reply.robot_list == NULL) {
      fprintf (stderr, "\nERROR: Out of memory error! \n");
	exit(EXIT_FAILURE);
    }
    
    if (errno) {
      perror("\nRobotServer_query_handler ");
      exit(EXIT_FAILURE);
    }
    
    for (counter = 0; counter < number_of_robots; counter++)
      reply.robot_list[counter] = list_of_robots[counter];

    
    strcpy(message, TCX_MODULE_NAME);
    strcat(message, "_reply");

    tcxReply(ref, message, &reply);
  }

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
  uint counter  = 0,
       counter2 = 0,
       found    = 0;

#ifdef TOTAL_debug
  fprintf(stdout,"\n--->close_handler");
  fflush(stdout);
#endif

  if (verbose)
      fprintf(stderr, "\n%s: closed connection detected >%s<", TCX_MODULE_NAME, name);

  /* TCX shut down */
  if (!strcmp(name, "TCX Server")) {
    fprintf(stderr,"\nERROR: TCX died !\n");
    exit(EXIT_FAILURE);
  }

  if (nr_auto_update_modules)
    remove_auto_update_module(module);

  /* is the module / robot that closed down in the list ?? */
  /*   if robot is in the list remove and shift the rest of the list */
  /*   reduce the number of robots active */

  for (counter = 0; counter < number_of_robots; counter++) {
    if (!strcasecmp(name, list_of_robots[counter])) {
      if (verbose)
	fprintf(stderr,"\nRobot >%s< died and is removed from list !", name);

      found++;
      
      for (counter2 = counter; counter2 < (number_of_robots-1); counter2++) {
	list_of_robots[counter2] = list_of_robots[counter2+1];
      }
      number_of_robots--;
    }
  }
  if (found)
    send_auto_updates();
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
  fprintf(stdout,"\n--->interrupt_handler");
  fflush(stdout);
#endif

  fprintf(stderr, "\nProgrm interrupteed by signal >%d<.\n", sig);
  fflush(stderr);
  fflush(stdout);
  exit(EXIT_FAILURE);
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

void init_tcx(void)
{
  char *tcxMachine = NULL;
  
  TCX_REG_MSG_TYPE TCX_message_array[] = { RobotServer_messages };

#if TOTAL_debug
  fprintf(stdout,"--->init_tcx\n");
  fflush(stdout);
#endif

  errno = 0;

  signal(SIGTERM, &interrupt_handler);  /* kill interupt handler      */

  if (errno) {
    perror("\ninit_tcx ");
    exit(EXIT_FAILURE);
  }
  
  signal(SIGINT,  &interrupt_handler);  /* control-C interupt handler */

  if (errno) {
    perror("\ninit_tcx ");
    exit(EXIT_FAILURE);
  }

  
  fprintf(stderr, "Connecting to TCX...");
  tcxMachine = getenv("TCXHOST");   

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize(TCX_MODULE_NAME, (void *) tcxMachine);

  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));

  tcxRegisterHandlers(RobotServer_query_handler_array,
		      sizeof(RobotServer_query_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterCloseHnd(close_handler); 

  fprintf(stderr, "done.\n");

  RobotServer = NULL;

}

main(int argc, char* argv[])
{

  int finish=0, test = 1, i = 0;

  for (i = 1; i < argc; i++) {
    if(strcasecmp(argv[i], "-verbose") == 0) {
      verbose = 1;
    }
    else {
      fprintf(stderr, "\nRobotServer [-verbose]\n");
    exit(0);
    }
  }
  
  /* I hope there will there will be not more then 255 robots ;) */
  list_of_robots = (unsigned char **) calloc(255, sizeof(unsigned char *));

  errno = 0;

  if (list_of_robots == NULL) {
    fprintf (stderr, "\nERROR: Out of memory error! \n");
      exit(EXIT_FAILURE);
  }
  
  if (errno) {
    perror("\nmain ");
    exit(EXIT_FAILURE);
  }

  init_tcx();

  while(!finish) {
    
    tcxRecvLoop((void *) &TCX_waiting_time);

    usleep (100000);
        
  }
  exit (0);
}

 
