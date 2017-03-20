/*
 * $Id: handlers.c,v 1.1 2002/09/14 16:37:46 rstone Exp $
 *
 */

#define SEBASTIAN2

#include <stdio.h>

#include <rai.h>

#include <bUtils.h>

#include "tcx.h"
#include "robot_specifications.h"
#include "tcxP.h"
#include "global.h"

#define TCX_define_variables /* this makes sure variables are installed */
#include "CAMERA-messages.h"   /* messages for VISION   */

#define DEFINE_REPLY_HANDLERS
#ifdef SEBASTIAN
#include "BASE-messages.h"     /* messages for BASE     */
#include "PANTILT-messages.h"  /* messages for PANTILT  */
#endif

#include "libezx.h"

#include <signal.h>
#include <sys/mman.h>

#ifdef HAVE_MATROX_METEOR
#include <sys/fcntl.h> 
#include <ioctl_meteor.h> 
#endif

#include "grab.h"
#include "imagesize.h"

#include <Common.h>
#include <EZX11.h>
#include "display.h"


#include "misc.h"
#include "cameraClient.h"
#include "beeSoftVersion.h"
#include "raiClient.h"
#include "raiServer.h"

#ifdef SEBASTIAN2
#include "baseMessages.h"

TCX_MODULE_PTR  BASESERVER  = NULL;
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

#endif

#include "handlers.h"

/* The following is a hack: <jpeglib.h> and <Common.h> both define
   EXTERN (differently).  Common.h only uses it for variable
   declarations in .h files (or not), which is a Bad Idea, IMHO.  In
   any case, that process will be complete by the time the
   pre-processor reaches this point (after all other .h files have
   been processed), so undefining it here should not hurt anything. */
#undef EXTERN
#include <jpeglib.h>

TCX_REG_HND_TYPE CAMERA_handler_array[] = {

  {"CAMERA_image_query", "CAMERA_image_query_handler",
   CAMERA_image_query_handler, TCX_RECV_ALL, NULL},

  {"CAMERA_save", "CAMERA_save",
   CAMERA_save_handler, TCX_RECV_ALL, NULL},

  {"CAMERA_load", "CAMERA_load",
   CAMERA_load_handler, TCX_RECV_ALL, NULL},

  {"CAMERA_startstop", "CAMERA_startstop",
   CAMERA_startstop_handler, TCX_RECV_ALL, NULL},

  {"CAMERA_shmid_query", "CAMERA_shmid_query_handler",
   CAMERA_shmid_query_handler, TCX_RECV_ALL, NULL},
  
  {"CAMERA_register_auto_update", "CAMERA_register_auto_update_handler",
   CAMERA_register_auto_update_handler, TCX_RECV_ALL, NULL}

};


#ifdef SEBASTIAN
static int base_connected = 0;
static int pantilt_connected = 0;
#endif
#ifdef SEBASTIAN2
static int base_server_connected = 0;
#endif

float savingdelay = 0.0;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/******* auto-reply stuff, data definitions ***************/


#define MAX_N_AUTO_UPDATE_MODULES 100


int n_auto_update_modules = 0; /* number of processes to whom
				* position should be forwarded
				* automatically upon change */

int n_auto_image_update_modules      = 0;



typedef struct{
  TCX_MODULE_PTR module;	/* pointer to TCX module */
  int            numGrabber;	/* zero for the first grabber */
  int            image;		/* >=1=subscribed to regular image updates */
  int            last_image;
  int            orig_image_xmin;
  int            orig_image_ymin;
  int            orig_image_xsize;
  int            orig_image_ysize;
  int            return_image_xsize;
  int            return_image_ysize;
  int            image_format;
} auto_update_type;


auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];
                                /* collection of TCX-module ptrs */

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


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

  n_auto_image_update_modules      = 0;

  for (i = 0; i < n_auto_update_modules; i++){
    if (auto_update_modules[i].image)      n_auto_image_update_modules++;
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
 *                 data                        subscription information
 *                 
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/



static int add_auto_update_module(TCX_MODULE_PTR                  module,
				  CAMERA_register_auto_update_ptr data)
{
  int i;

  if (n_auto_update_modules >= MAX_N_AUTO_UPDATE_MODULES){
    fprintf(stderr, "ERROR: auto-module memory overflow. Request rejected.\n");
    return 0;
  }
  else
    for (i = 0; i < n_auto_update_modules; i++)
      if (auto_update_modules[i].module == module){
	fprintf( stderr, 
		 "Module %s already known. Subscription modified: %d (%dx%d+%d+%d,%dx%d,%d)\n",
		 tcxModuleName(module), 
		 data->image,
		 data->orig_image_xsize, 
		 data->orig_image_ysize,
		 data->orig_image_xmin, 
		 data->orig_image_ymin,
		 data->return_image_xsize, 
		 data->return_image_ysize,
		 data->numGrabber );
	auto_update_modules[i].image       = data->image; /* subsrc? */
	auto_update_modules[i].numGrabber  = data->numGrabber;
        auto_update_modules[i].orig_image_xmin    = data->orig_image_xmin;
        auto_update_modules[i].orig_image_ymin    = data->orig_image_ymin;
        auto_update_modules[i].orig_image_xsize   = data->orig_image_xsize;
        auto_update_modules[i].orig_image_ysize   = data->orig_image_ysize;
        auto_update_modules[i].return_image_xsize = data->return_image_xsize;
        auto_update_modules[i].return_image_ysize = data->return_image_ysize;
        auto_update_modules[i].image_format       = data->image_format;
	auto_update_modules[i].last_image         = -1;

	count_auto_update_modules();
	return 1;
      }
  fprintf(stderr, "Add %s to auto-reply list: %d (%dx%d+%d+%d,%dx%d,%d).\n",
	  tcxModuleName(module), data->image,
	  data->orig_image_xsize, 
	  data->orig_image_ysize,
	  data->orig_image_xmin, 
	  data->orig_image_ymin,
	  data->return_image_xsize, 
	  data->return_image_ysize,
	  data->numGrabber ); 

  auto_update_modules[n_auto_update_modules].module     = module; /* pointer*/
  auto_update_modules[n_auto_update_modules].image       =
    data->image; /* subsrc? */
  auto_update_modules[n_auto_update_modules].numGrabber  =
    data->numGrabber;
  auto_update_modules[n_auto_update_modules].orig_image_xmin    =
    data->orig_image_xmin;
  auto_update_modules[n_auto_update_modules].orig_image_ymin    =
    data->orig_image_ymin;
  auto_update_modules[n_auto_update_modules].orig_image_xsize   =
    data->orig_image_xsize;
  auto_update_modules[n_auto_update_modules].orig_image_ysize   =
    data->orig_image_ysize;
  auto_update_modules[n_auto_update_modules].return_image_xsize =
    data->return_image_xsize;
  auto_update_modules[n_auto_update_modules].return_image_ysize =
    data->return_image_ysize;
  auto_update_modules[n_auto_update_modules].image_format =
    data->image_format;
  auto_update_modules[n_auto_update_modules].last_image = -1;
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
	  auto_update_modules[j+1].module;
	auto_update_modules[j].numGrabber =
	  auto_update_modules[j+1].numGrabber;
	auto_update_modules[j].image =
	  auto_update_modules[j+1].image;
	auto_update_modules[j].last_image =
	  auto_update_modules[j+1].last_image;
	auto_update_modules[j].orig_image_xmin =
	  auto_update_modules[j+1].orig_image_xmin;
	auto_update_modules[j].orig_image_ymin =
	  auto_update_modules[j+1].orig_image_ymin;
	auto_update_modules[j].orig_image_xsize =
	  auto_update_modules[j+1].orig_image_xsize;
	auto_update_modules[j].orig_image_ysize =
	  auto_update_modules[j+1].orig_image_ysize;
	auto_update_modules[j].return_image_xsize =
	  auto_update_modules[j+1].return_image_xsize;
	auto_update_modules[j].return_image_ysize =
	  auto_update_modules[j+1].return_image_ysize;
	auto_update_modules[j].image_format =
	  auto_update_modules[j+1].image_format;
      }
    }
  if (!found)
    fprintf(stderr, "(%s: no auto-replies removed)\n", tcxModuleName(module));
  count_auto_update_modules();
  return found;
}
  


/************************************************************************
 *
 *   NAME:         send_automatic_image_update
 *                 
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void send_automatic_image_update( int numGrabber, unsigned char *pCameraImage )
{
  int i;

  for (i = 0; i < n_auto_update_modules; i++){
    if (auto_update_modules[i].image > 0){

      auto_update_modules[i].last_image++;
      if (auto_update_modules[i].last_image %
	  auto_update_modules[i].image == 0){
#ifdef CAMERA_debug	
	fprintf(stderr, "Send image from grabber %d of size %dx%d to %s.\n",
		numGrabber,
		auto_update_modules[i].image_xsize,
		auto_update_modules[i].image_ysize,
		tcxModuleName(auto_update_modules[i].module));
#endif
	auto_update_modules[i].last_image = 0;

	CAMERA_send_camera_image_to(auto_update_modules[i].module,
				    numGrabber,
				    pCameraImage,
				    auto_update_modules[i].orig_image_xmin,
				    auto_update_modules[i].orig_image_ymin,
				    auto_update_modules[i].orig_image_xsize,
				    auto_update_modules[i].orig_image_ysize,
				    auto_update_modules[i].return_image_xsize,
				    auto_update_modules[i].return_image_ysize,
                                    auto_update_modules[i].image_format);
      }
    }
  }
}


/************************************************************************
 *
 *   NAME:         check_commandline_parameters
 *                 
 *   FUNCTION:     checks the command line options!
 *                 
 *   PARAMETERS:   
 *                 
 *                 NOTE: "NULL" disables initialization
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void check_commandline_parameters(int argc, char **argv)
{
  int i, j, e, bug = 0;
  char *robotName = NULL;

  for (i = 1; i < argc && !bug; i++){
    for (j = 0, e = 0; j < (int) strlen(argv[i])-1; j++)
      if (argv[i][j] == '=')
	e = 1;
    if (!strcmp(argv[i], "-nodisplay") || !strcmp(argv[i], "-nd"))
      display = 0;
    else if (!strcmp(argv[i], "-notcx") || !strcmp(argv[i], "-nt"))
      tcx = 0;
    else if (!strcmp(argv[i], "-color")  || !strcmp(argv[i], "-co"))
      color = 1;
    else if (!strcmp(argv[i], "source"))
      fprintf(stderr, "WARNING: Found a stupid gdb.\n");
    else if (!strcmp(argv[i], "-fileonly")  || !strcmp(argv[i], "-fo")){
      saving[0]       = 1;
      savefileopen[0] = 0;
      saveframes[0]   = 0;
      sprintf(filename[0], "images.dat");
    }
    else if (!strcmp(argv[i], "-rate")  || !strcmp(argv[i], "-ra")){
      if (i == argc - 1){
	fprintf(stderr, "Need argument for -rate\n");
	bug = 1;
      }
      else
	savingdelay = atof(argv[++i]);
    }
    else if (!strcmp(argv[i], "-robot")) {
      if (i == argc - 1){
	fprintf(stderr, "Need argument for -robot\n");
	bug = 1;
      }
      else
	robotName = argv[++i];
    }
    else if (argv[i][0] != '-' || !e){
      fprintf(stderr, "Cannot parse argument: %s\n", argv[i]);
      bug = 1;
    }
  }

  if (bug){
    fprintf(stderr, "Usage: '%s [-nodisplay] [-notcx] [-color] [-fileonly] [-rate <sec>]\n", 
	    argv[0]);
    exit(1);
  }

  if ( robotName != NULL) {
    fprintf(stderr, "Robot name: %s.\n", robotName);
    tcxSetModuleNameExtension( robotName);
  }
}



/************************************************************************
 *
 *   NAME: CAMERA_register_auto_update_handler()
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/




void CAMERA_register_auto_update_handler(TCX_REF_PTR  ref,
					 CAMERA_register_auto_update_ptr data)
{
  add_auto_update_module(ref->module, data);

  if (data != NULL){
    tcxFree("CAMERA_register_auto_update", data);
    data = NULL;
  }
}


/************************************************************************
 *
 *   NAME:         CAMERA_initialize_board_and_tcx
 *                 
 *   FUNCTION:     general initialization routine - must be called before
 *                 anything else. Returns error value (0=success, 1=error)
 *                 
 *   RETURN-VALUE: Returns error value (0=success, 1=error)
 *                 
 ************************************************************************/

int CAMERA_initialize_tcx()
{
  
  /* messages used by CAMERA */
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    CAMERA_messages,
#ifdef SEBASTIAN
    BASE_messages,
    PANTILT_messages,
#endif
#ifdef SEBASTIAN2
    {"baseTCXServer", NULL},
    {"baseFixed", "{int, long}"},
    {"baseVar", "{int,int,<char: 2>}"}
#endif
  };
  
  /* TCX */
  
  fprintf(stderr, "Connecting to TCX...");

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }

  tcxInitialize(TCX_CAMERA_MODULE_NAME, tcxMachine);
  check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
		       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
		       NULL, 0);
  check_version_number(libezx_major, libezx_minor,
		       libezx_robot_type, libezx_date,
		       "libezx", 0);



  tcxRegisterMessages(TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));
  
  /* Handlers */
  
  tcxRegisterHandlers(CAMERA_handler_array, 
		      sizeof(CAMERA_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
#ifdef SEBASTIAN
  tcxRegisterHandlers(BASE_reply_handler_array,
		      sizeof(BASE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));
    
  tcxRegisterHandlers(PANTILT_reply_handler_array,
		      sizeof(PANTILT_reply_handler_array)
			/ sizeof(TCX_REG_HND_TYPE));
#endif
  
#ifdef SEBASTIAN2
  tcxRegisterHandlers(BASE_CLIENT_HANDLERS,
		      sizeof(BASE_CLIENT_HANDLERS)
		      / sizeof(TCX_REG_HND_TYPE));
#endif

  tcxRegisterCloseHnd(CAMERA_close_handler);
  
  /* Connections */

#ifdef SEBASTIAN
  connect_to_Base();
  connect_to_Pantilt();
#endif
#ifdef SEBASTIAN2
  connect_to_BaseServer();
#endif
  
  /* Return */
  return (1) ; 
  
}


/************************************************************************
 *
 *   NAME:         connect_to_Base
 *                 
 *   FUNCTION:     checks, and connects to BASE, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

#ifdef SEBASTIAN			/* no clue what this is for ... */

void connect_to_Base(void)
{
  struct timeval current_time;
  static struct timeval last_attempt_connect_BASE = {0, 0};


  if(!base_connected){

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
      base_connected = 1;
      fprintf(stderr, "TCX: connected to BASE.\n");
      tcx_base_subscribe();
    }
  }
}



/************************************************************************
 *
 *   NAME:         connect_to_Pantilt
 *                 
 *   FUNCTION:     checks, and connects to PANTILT, if necessary
 *                 
 *   PARAMETERS:   PROGRAM_STATE_PTR program_state
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/



void connect_to_Pantilt(void)
{
  struct timeval current_time;
  static struct timeval last_attempt_connect_PANTILT = {0, 0};


  if(!pantilt_connected){

    gettimeofday(&current_time, NULL);
    if (current_time.tv_sec < last_attempt_connect_PANTILT.tv_sec + 5
	|| (current_time.tv_sec == last_attempt_connect_PANTILT.tv_sec + 5 &&
	    current_time.tv_usec < last_attempt_connect_PANTILT.tv_usec))
      return;
    
    last_attempt_connect_PANTILT.tv_sec  = current_time.tv_sec;
    last_attempt_connect_PANTILT.tv_usec = current_time.tv_usec;
    

    PANTILT = tcxConnectOptional(TCX_PANTILT_MODULE_NAME); /* checks, but does 
						      * not wait */

    if (PANTILT != NULL){
      pantilt_connected = 1;
      fprintf(stderr, "TCX: connected to PANTILT.\n");
    }
  }
}

#endif


#ifdef SEBASTIAN2


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
	command.parameter = 0;
	tcxSendMsg(BASESERVER,"baseFixed",&command);

	command.operation = BASE_loadPosition;
	command.parameter = 0x80008000;
	tcxSendMsg(BASESERVER,"baseFixed",&command);
	command.operation = BASE_statusReportPeriod;
	command.parameter = 100*256/1000;
	tcxSendMsg(BASESERVER,"baseFixed",&command);
	command.operation = BASE_setTranslateAcceleration;
	command.parameter = 1000;
	tcxSendMsg(BASESERVER,"baseFixed",&command);
	command.operation = BASE_setRotateAcceleration;
	command.parameter = 150;
	tcxSendMsg(BASESERVER,"baseFixed",&command);
	command.operation = BASE_assumeWatchdog;
	command.parameter = 0;
	tcxSendMsg(BASESERVER,"baseFixed",&command);
	command.operation = BASE_watchdogTimer;
	command.parameter = 0x800;
	tcxSendMsg(BASESERVER,"baseFixed",&command);

      }
    }
  }
}


void
handleBaseClientFixed(TCX_REF_PTR message, RAI_FixedMsgType *msg_ptr)
{
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

  /*
    fprintf(stderr, "Status Report: cl=%ld x=%ld y=%ld o=%ld\n",
	  activeStatusReport.Clock,
	  activeStatusReport.Xpos,
	  activeStatusReport.Ypos,
	  activeStatusReport.Heading);
  */
}

void
handleBaseClientVariable(TCX_REF_PTR message, RAI_VariableMsgType *msg_ptr)
{
  int operation;
  int bufsize;
  unsigned char* buffer;
  operation = msg_ptr->operation;
  bufsize = msg_ptr->bufsize;
  buffer = msg_ptr->buffer;

  if (operation == BASE_statusReport)
    incorporateStatusReport( (statusReportType*) buffer);
  tcxFree("baseVar", msg_ptr);
}
#endif

/************************************************************************
 *
 *   NAME:         CAMERA_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void CAMERA_close_handler(char *name, TCX_MODULE_PTR module)
{
#ifdef CAMERA_debug
  fprintf(stderr, "CAMERA: closed connection detected: %s\n", name);
#endif

  remove_auto_update_module(module);

#ifdef SEBASTIAN
  if (!strcmp(name, "BASE")){ /* BASE shut down */
    base_connected = 0;
    BASE = NULL;
    fprintf(stderr, "TCX: disconnected from BASE.\n");
  }

  if (!strcmp(name, "PANTILT")){ /* PANTILT shut down */
    pantilt_connected = 0;
    PANTILT = NULL;
    fprintf(stderr, "TCX: disconnected from PANTILT.\n");
  }
#endif
#ifdef SEBASTIAN2
  if (!strcmp(name, "baseTCXServer")){ /* BASE shut down */
    base_server_connected = 0;
    BASESERVER = NULL;
    fprintf(stderr, "TCX: disconnected from BaseServer.\n");
  }
#endif
  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    commShutdown();
  }
}




#ifdef SEBASTIAN			/* no clue what this is for ... */
void PANTILT_position_reply_handler(TCX_REF_PTR                ref,
				    PANTILT_position_reply_ptr data)
{
  tcxFree("PANTILT_position_reply", data);
}

void PANTILT_init_reply_handler(TCX_REF_PTR             ref,
				int                    *data)
{
  tcxFree("PANTILT_init_reply", data);

}


void PANTILT_limits_reply_handler(TCX_REF_PTR              ref,
				  PANTILT_limits_reply_ptr data)
{
  tcxFree("PANTILT_limits_reply", data);
}



void PANTILT_status_update_handler(TCX_REF_PTR              ref,
				  PANTILT_status_update_ptr data)
{
  tcxFree("PANTILT_status_update", data);
}



void tcx_pantilt_init()
{
  int auto_reply_position = 1;

  if (pantilt_connected)
    tcxSendMsg(PANTILT, "PANTILT_init_query", &auto_reply_position);
}

/************************************************************************
 *
 *   NAME:         tcx_base_subscribe
 *                 
 *   FUNCTION:     subscribe for BASE messages 
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void tcx_base_subscribe()
{
  BASE_register_auto_update_type data;

  data.subscribe_status_report = 1;
  data.subscribe_sonar_report  = 0;
  data.subscribe_colli_report  = 0;
  data.subscribe_ir_report     = 0;
  data.subscribe_laser_report  = 0;

  if (base_connected)
    tcxSendMsg(BASE, "BASE_register_auto_update", &data);
}




void BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
					BASE_action_executed_reply_ptr data){
;}

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         BASE_robot_position_reply_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
				       BASE_robot_position_reply_ptr pos)
{
}



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


void BASE_update_status_reply_handler(TCX_REF_PTR                   ref,
				      BASE_update_status_reply_ptr  pos)
{

#ifdef CAMERA_debug
  fprintf(stderr, "TCX: Received a BASE_update_status_reply message.\n");
  fprintf(stderr, "time stamp: %g robot: %g %g %g\n", pos->time,
	  pos->pos_x, pos->pos_y, pos->orientation);
#endif
  
  tcxFree("BASE_robot_position_reply", pos);

}


#endif /* of SEBASTIAN */



void jpg_init_destination(j_compress_ptr cinfo)
{
#ifdef CAMERA_debug
  printf("jpg_init_destination\n");
#endif
}

int jpg_empty_output_buffer(j_compress_ptr cinfo)
{
#ifdef CAMERA_debug
  printf("jpg_empty_output_buffer\n");
#endif
  return TRUE;
}

void jpg_term_destination(j_compress_ptr cinfo)
{
#ifdef CAMERA_debug
  printf("jpg_term_destination\n");
#endif
}

/* JPEG compression function.  Requires libjpeg.  Requires dest_memory
   to be allocated large enough for any compressed image data.
   Typically, just allocate width * height * 3 bytes/pixel, and any
   image should compress to less than that.

   buf is the image data pointer: RGBRGB format

   dest_memory_used is a return value: the number of bytes taken by
   the compressed image data. */
void compress_image(unsigned char *dest_memory,
                    int dest_memory_size,
                    int *dest_memory_used,
                    unsigned char *buf, int width, int height,
                    int quality) {

  struct jpeg_compress_struct cinfo;
  struct jpeg_destination_mgr dest_man;
  struct jpeg_error_mgr jerr;
  JSAMPROW row_pointer[1];     /* pointer to JSAMPLE row(s) */
  int row_stride;              /* physical row width in image buffer */

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  /* we don't want standard out, we want memory: dest_memory */
  /* jpeg_stdio_dest(&cinfo, output); */

  dest_man.next_output_byte = dest_memory;
  dest_man.free_in_buffer = dest_memory_size;
  dest_man.init_destination = jpg_init_destination;
  dest_man.empty_output_buffer = jpg_empty_output_buffer;
  dest_man.term_destination = jpg_term_destination;

  cinfo.dest = &dest_man;

  cinfo.image_width = width;          /* image width and height, in pixels */
  cinfo.image_height = height;
  cinfo.input_components = 3;         /* # of color components per pixel */
  cinfo.in_color_space = JCS_RGB;     /* colorspace of input image */

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);  /* limit to baseline-JPEG values */
  jpeg_start_compress(&cinfo, TRUE);
  row_stride = cinfo.input_components*width;  /* JSAMPLE units per row */
  while (cinfo.next_scanline < cinfo.image_height) {
    /* jpeg_write_scanlines expects an array of pointers to scanlines.
     * Here the array is only one element long, but you could pass
     * more than one scanline at a time if that's more convenient.
     */
    row_pointer[0] = &buf[cinfo.next_scanline * row_stride];
    jpeg_write_scanlines(&cinfo, row_pointer, 1);  
  }
  jpeg_finish_compress(&cinfo);

  if(dest_memory_used != NULL)
    *dest_memory_used = dest_man.next_output_byte - dest_memory;

  jpeg_destroy_compress(&cinfo);
}


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         CAMERA_send_camera_image_to
 *                 
 *   FUNCTION: sends a camera image to a module, but refuses to increase
 *             the image size.  I.e. scaling down OK, scaling up bad.
 *             When scaling down, pixel averaging is used, not subsampling.
 *                 
 *   PARAMETERS: pImage is a pointer to RGBxRGBxRGBx image data, of
 *               size ROWS x COLS x 4 bytes.  The "x" byte is
 *               padding.
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void CAMERA_send_camera_image_to( TCX_MODULE_PTR module,
				  int   numGrabber,
				  unsigned char *pImage,
				  int orig_image_xmin,
				  int orig_image_ymin,
				  int orig_image_xsize,
				  int orig_image_ysize,
				  int return_image_xsize,
				  int return_image_ysize,
                                  int image_format)
{
  CAMERA_image_reply_type image;
  int i, x, y, index_orig, index_copy;
  int x_orig, y_orig, x_copy, y_copy;
  float *R, *G, *B;
  float *c;

  if (orig_image_xsize <= 0 || orig_image_xsize > COLS ||
      orig_image_ysize <= 0 || orig_image_ysize > ROWS ||
      return_image_xsize <= 0 || return_image_xsize > COLS ||
      return_image_ysize <= 0 || return_image_ysize > ROWS ||
      orig_image_xsize < return_image_xsize ||
      orig_image_ysize < return_image_ysize){
    fprintf(stderr, "WARNING: Invalid image size: %d %d. Request ignored.\n",
	    orig_image_xsize, orig_image_ysize);
  }

  else {

    image.numGrabber = numGrabber;
    image.xsize = return_image_xsize;
    image.ysize = return_image_ysize;


    R = (float *) malloc(sizeof(float) * image.xsize * image.ysize);
    G = (float *) malloc(sizeof(float) * image.xsize * image.ysize);
    B = (float *) malloc(sizeof(float) * image.xsize * image.ysize);
    c = (float *) malloc(sizeof(float) * image.xsize * image.ysize);

    /* clear the pixel arrays. */
    for (i = 0, x = 0; x < image.xsize; x++)
      for (y = 0; y < image.ysize; y++, i++)
	c[i] = R[i] = G[i] = B[i] = 0.0;


    /* scale the image appropriately. */
    for (x = 0; x < orig_image_xsize; x++)
      for (y = 0; y < orig_image_ysize; y++){
	x_orig = x + orig_image_xmin;
	y_orig = y + orig_image_ymin;
	x_copy = (int) (((float) x) * ((float) return_image_xsize)
			/ ((float) orig_image_xsize));
	y_copy = (int) (((float) y) * ((float) return_image_ysize)
			/ ((float) orig_image_ysize));
	if (x_orig >= 0 && x_orig < COLS &&
	    y_orig >= 0 && y_orig < ROWS &&
	    x_copy >= 0 && x_copy < return_image_xsize &&
	    y_copy >= 0 && y_copy < return_image_ysize){
	  index_orig = 4*(y_orig*COLS+x_orig);
	  index_copy = y_copy * return_image_xsize + x_copy;
	  R[index_copy] = R[index_copy] + ((float) pImage[index_orig+0]);
	  G[index_copy] = G[index_copy] + ((float) pImage[index_orig+1]);
	  B[index_copy] = B[index_copy] + ((float) pImage[index_orig+2]);
	  c[index_copy] = c[index_copy] + 1.0;
	}
	/* else
	  fprintf(stderr, "?: %d %d %d %d (%d %d)",
	  x_orig, y_orig, x_copy, y_copy, x, y);*/
      }

    /* handle destination pixels which had more than one source pixel
       land on them. */
    for (i = 0, x = 0; x < image.xsize; x++)
      for (y = 0; y < image.ysize; y++, i++){
	if (c[i] > 0.0){
	  R[i] = R[i] / c[i];
	  G[i] = G[i] / c[i];
	  B[i] = B[i] / c[i];
	}
      }

    if(image_format == JPEG_FORMAT) {
      static unsigned char *rgb_data = NULL;
      static unsigned char *compressed_data = NULL;
      static int rgb_data_size = 0;

      unsigned char *rgb_ptr;
      int compressed_data_size;

      /* manage static rgb and compressed data buffers. */
      if(rgb_data == NULL ||
         rgb_data_size < image.xsize * image.ysize * 3) {

        if(rgb_data != NULL) {
          free(rgb_data);
          free(compressed_data);
        }

        rgb_data = (unsigned char *) malloc(image.xsize * image.ysize * 3);
        compressed_data =
          (unsigned char *) malloc(image.xsize * image.ysize * 3);

        rgb_data_size = image.xsize * image.ysize * 3;
      }

      rgb_ptr = rgb_data;

      /* copy the scaled image into the rgb data storage area. */
      for (i = 0, x = 0; x < image.xsize; x++) {
        for (y = 0; y < image.ysize; y++, i++) {
          *rgb_ptr++ = ((unsigned char) (R[i] + 0.5));
          *rgb_ptr++ = ((unsigned char) (G[i] + 0.5));
          *rgb_ptr++ = ((unsigned char) (B[i] + 0.5));
        }
      }
      
      compress_image(compressed_data, image.xsize * image.ysize * 3,
                     &compressed_data_size,
                     rgb_data, image.xsize, image.ysize, 75);

      image.compressedImage.image_format = JPEG_FORMAT;
      image.compressedImage.size = compressed_data_size;
      image.compressedImage.image = compressed_data;

      image.size = 0;
      image.red = NULL;
      image.green = NULL;
      image.blue = NULL;

#ifndef TCX_debug
      fprintf( stderr, "TCX: Sending CAMERA_image_reply from grabber %d.\n",
               numGrabber);
#endif

      tcxSendMsg(module, "CAMERA_image_reply", &image); 
      
    }
    else { /* treat all unknown formats as UNCOMPRESSED_FORMAT. */
    
      image.size  = return_image_xsize * return_image_ysize;
      image.red   = (unsigned char *) malloc(image.size);
      image.green = (unsigned char *) malloc(image.size);
      image.blue  = (unsigned char *) malloc(image.size);

      image.compressedImage.image_format = UNCOMPRESSED_FORMAT;
      image.compressedImage.size = 0;
      image.compressedImage.image = NULL;

      /* copy the scaled image into an output image structure. */
      for (i = 0, x = 0; x < image.xsize; x++)
        for (y = 0; y < image.ysize; y++, i++){
          image.red[i]   = ((unsigned char) (R[i] + 0.5));
          image.green[i] = ((unsigned char) (G[i] + 0.5));
          image.blue[i]  = ((unsigned char) (B[i] + 0.5));
        }
    
#ifndef TCX_debug
      fprintf( stderr, "TCX: Sending CAMERA_image_reply from grabber %d.\n",
               numGrabber);
#endif

      tcxSendMsg(module, "CAMERA_image_reply", &image); 
    
      free(image.blue);
      free(image.green);
      free(image.red);
    }

    free(c);
    free(B);
    free(G);
    free(R);
  }

}

/************************************************************************
 *
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 ************************************************************************/

void CAMERA_save_handler( TCX_REF_PTR      ref,
			  CAMERA_save_ptr  data )
{

#ifdef TCX_debug
  fprintf(stderr, "Received a CAMERA_save_message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  ref = ref;			/* -Wall */

  if ( data->frames == 0 ) {

    saving[data->numGrabber]       = 0;
    savefileopen[data->numGrabber] = 0;
    saveframes[data->numGrabber]   = 0;
    sprintf( filename[data->numGrabber], "%s", data->filename );
    fclose( videofp[data->numGrabber] );

  } else if ( data->frames == -1 ) {

    saving[data->numGrabber]       = 1;
    savefileopen[data->numGrabber] = 0;
    saveframes[data->numGrabber]   = 0;
    sprintf( filename[data->numGrabber], "%s", data->filename );

  } else {

    saveframes[data->numGrabber]   = data->frames;
    saving[data->numGrabber]       = 1;
    savefileopen[data->numGrabber] = 0;
    sprintf( filename[data->numGrabber], "%s", data->filename );

  }

  tcxFree("CAMERA_save", data);

}
				 
/************************************************************************
 *
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 ************************************************************************/

void CAMERA_load_handler( TCX_REF_PTR      ref,
			  CAMERA_load_ptr  data )
{

  CAMERA_load_reply_type info;

#ifdef TCX_debug
  fprintf(stderr, "Received a CAMERA_load_message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  /* this will disable grabs before it loads anything. */
  openAndReadFile( data->numGrabber, data->filename );

  /* finished -> send reply to module and start the grabber again */
  info.dummy = 1;
#ifdef TCX_debug
  fprintf( stderr, "TCX: Sending CAMERA_load_reply.\n");
#endif
  tcxSendMsg( ref->module, "CAMERA_load_reply", &info );
  
  startGrabber( data->numGrabber );
      
  tcxFree("CAMERA_file", data);

}

/************************************************************************
 *
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 ************************************************************************/

void CAMERA_startstop_handler( TCX_REF_PTR           ref,
			       CAMERA_startstop_ptr  data )
{

#ifdef TCX_debug
  fprintf(stderr, "Received a CAMERA_startstop_message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  ref = ref;			/* -Wall */

  if ( data->action == 1 ) {
    startGrabber(data->numGrabber);
  } else if ( data->action == 0 ) {
    stopGrabber(data->numGrabber);
  } else {
    /* bummer */
  }

  tcxFree("CAMERA_file", data);

}

/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         CAMERA_image_query_handler
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void CAMERA_image_query_handler(TCX_REF_PTR             ref,
				CAMERA_image_query_ptr  data)
{

#ifndef TCX_debug
  fprintf(stderr, "Received a CAMERA_image_query_message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  /* this does not handle the case in which we load frames from file
     and request one of them using this function. Too bad. */

  if ( !((data->numGrabber+1)&useGrabber) && useGrabber!=0 ) {
    fprintf( stderr,"%s: numGrabber %d is NOT in use.\n",
	     __FUNCTION__, data->numGrabber );
      return;
  }

  CAMERA_send_camera_image_to( ref->module, 
			       data->numGrabber,
			       pCameraImage[data->numGrabber],
			       data->orig_image_xmin,
			       data->orig_image_ymin,
			       data->orig_image_xsize,
			       data->orig_image_ysize,
			       data->return_image_xsize,
			       data->return_image_ysize,
                               data->image_format);

  tcxFree("CAMERA_image_query", data);
}


/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void CAMERA_shmid_query_handler( TCX_REF_PTR             ref,
				 CAMERA_shmid_query_ptr  data)

{
  CAMERA_shmid_reply_type info;

#ifdef TCX_debug
  fprintf(stderr, "Received a CAMERA_shmid_query_message from %s.\n",
	  tcxModuleName(ref->module));
#endif

  info.numGrabber = data->numGrabber;
  info.shmid      = shmid[data->numGrabber]; /* smid defined in grab.h */

#ifdef TCX_debug
  fprintf(stderr, "TCX: Sending CAMERA_shmid_reply.\n");
#endif

  tcxFree("CAMERA_shmid_query", data);

  tcxSendMsg( ref->module, "CAMERA_shmid_reply", &info ); 

}

/*
 * $Log: handlers.c,v $
 * Revision 1.1  2002/09/14 16:37:46  rstone
 * *** empty log message ***
 *
 * Revision 1.31  1999/04/21 22:25:01  fox
 * New support for quickcam. Should be ok.
 *
 * Revision 1.30  1998/09/18 16:04:17  thrun
 * .
 *
 * Revision 1.29  1998/08/16 20:27:04  thrun
 * ficed a serious memory leak!
 *
 * Revision 1.28  1998/08/16 20:00:46  thrun
 * .
 *
 * Revision 1.27  1998/08/16 19:50:01  thrun
 * can now genrate arbitrary sub-images and downsample them.
 *
 * Revision 1.26  1998/08/08 22:19:56  thrun
 * -rate as commandline parameter: specifies frequency for
 * saving images on file.
 *
 * Revision 1.25  1998/08/08 18:44:43  thrun
 * new version that direclty connects to baseServer and
 * annodates images with positon information.
 *
 * Revision 1.24  1998/02/08 00:13:14  swa
 * now works with redhat5.0
 *
 * Revision 1.23  1998/01/13 00:35:14  swa
 * Added two new functions -- start and stop. They can start and stop either
 * of the two frame grabbers.
 *
 * Revision 1.22  1997/11/06 17:54:38  swa
 * Fixed a bug with TCXHOST. If the environment variable TCXHOST is set,
 * take it. If not, take the TCXHOST from beeSoft.ini. If this doesn't
 * work, take localhost. If this doesn't work: bummer.
 *
 * Revision 1.21  1997/10/04 18:01:06  swa
 * Fixed a bug in CAMERA-messages so that sending images over TCX works again.
 *
 * Revision 1.20  1997/10/04 01:06:46  swa
 * Fixed some bugs and inconsistencies wrt to both frame grabbers.
 *
 * Revision 1.19  1997/10/04 00:13:07  swa
 * First working version of the cameraServer that supports two (count'em
 * two!) framegrabbers.  Although the server seems to work just fine, it
 * has not yet been fully tested.
 *
 * Revision 1.18  1997/09/19 23:03:18  swa
 * Image-over-TCX works now correctly when using file as the source. A bunch
 * of function calls were missing in the load-file function.
 *
 * Revision 1.17  1997/08/02 18:50:54  swa
 * Support for file-only mode (SUN and Linux) added.
 *
 * Revision 1.16  1997/07/24 18:48:01  swa
 * Renamed two internal functions. Tested it again with the Tcl program. Works
 * fine. :)
 *
 * Revision 1.15  1997/07/24 00:54:25  swa
 * Fixed some minor bugs, extended the README file and tested the version. The
 * cameraAttachExample does not yet die when the cameraServer dies. Will be
 * fixed in future versions.
 *
 * Revision 1.14  1997/07/23 22:43:33  swa
 * This is the first version of camControl, a Tcl program that remotely controls
 * the cameraServer. The user can save and read frames. recorder was renamed
 * to camControl, since the program will not only record but also read frames.
 *
 * Revision 1.13  1997/07/22 22:47:16  swa
 * Added file handling (reading/saving). Renamed two (three?) functions to
 * be consistent.
 *
 * Revision 1.12  1997/07/04 17:28:31  swa
 * Left and right mouseclicks have been added to the example-program. Left
 * mouse saves the current cameraimage to a 4 byte .ppm format, right mouse
 * shows the current coordinates.
 *
 * Revision 1.11  1997/06/25 23:52:44  thrun
 * Various changes. Makde display in o-graphics much faster. changed
 * some of the parameter files to more sensible values. improves the
 * display in "learn". the commander now displays high-resolution
 * images.
 *
 * Revision 1.10  1997/06/25 18:46:50  thrun
 * Added support for tcx messages. It is for mainatt no longer necessary to
 * use argv[1]. It now sends out messages. (swa)
 *
 * Revision 1.9  1997/06/24 22:48:13  thrun
 * checking of the command line arguments.
 *
 * Revision 1.8  1997/06/24 17:05:43  thrun
 * Fixed some debug flags (swa)
 *
 * Revision 1.7  1997/06/23 23:48:43  thrun
 * shared memory and other stuff (swa)
 *
 * Revision 1.6  1997/06/23 02:36:00  thrun
 * Stefan's new camera server with shared memory
 *
 * Revision 1.5  1997/06/21 22:36:22  thrun
 * Improved interface, cleaner
 *
 * Revision 1.4  1997/06/20 01:09:29  thrun
 * Renamed function names to be more consistent. Currently the cameraServer
 * can track various object based on its color information. The color info
 * is supplied in a seperate textfile and is loaded into the server upon the
 * first request. (swa)
 *
 * Revision 1.3  1997/06/19 21:06:57  thrun
 * Added various stuff for faceInfo (callbacks, etc)
 *
 * Revision 1.2  1997/06/18 15:58:14  thrun
 * now with auto-update
 *
 * Revision 1.1.1.1  1997/06/16 22:32:20  thrun
 * New Camera Server, much faster, more reliable, works already with
 * commander.
 *
 */
