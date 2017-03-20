/* ========================================================================= */
/* */
/* = FILENAME */
/*    testServer.cpp */
/* */
/* = DESCRPITION */
/*    test server module used to learn about interfacing to mobility */
/*    based on gMoveServer.cpp */
/* */
/* = AUTHOR */
/*     Robert Todd Pack & Tyson D. Sawyer & Jamie Schulte */
/* */
/* ========================================================================= */

#include <stdio.h>
#include <unistd.h>	
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <asm/io.h>
/*#include <sys/io.h>*/

#include "devUtils.h"
#include "robot_arg.h"
#include "xr4000.h"
#include "bee_interface.h"
#include "baseMessages.h"
#include "baseServer.h"
#include "tcx.h"


#define LP1_BASEPORT 0x378 /* lp1 is the main parallel port */

#define LIFE_TIME 1800.0 /* in seconds, set to negative value if
			  * not wanted */

struct timeval watchdog_timestamp = {0, 0};


int global_watchdog = 1;

int global_set_tvel = 0;
int global_tvel_sign = 0;
int global_set_tvel_new = 0;

int global_set_rvel = 0;
int global_rvel_sign = 0;
int global_set_rvel_new = 0;

int global_relative_command = 0;
float global_relative_origin_x = 0.0;
float global_relative_origin_y = 0.0;
float global_relative_dist = 0.0;


void
baseClose(char *moduleName, TCX_MODULE_PTR module);
void
handleBaseVariable(TCX_REF_PTR message, RAI_FixedMsgType *msg_ptr);
void
handleBaseFixed(TCX_REF_PTR message, RAI_FixedMsgType *msg_ptr);
int
add_auto_update_module(TCX_MODULE_PTR module, int status_frequency);
int
remove_auto_update_module(TCX_MODULE_PTR module);

void tcxInitializeInternal(char *module, char *server);


int bee_tcx_initialized = 0;
int bee_busy = 0;

/* ********************************************************************** *\
 * ********************************************************************** *
\* ********************************************************************** */


void
bee_init( char* robotName){
  char tcxServer[80];

  TCX_REG_MSG_TYPE messageArray[] = {
    BASE_MESSAGES
  };

  TCX_REG_HND_TYPE handlerArray[] = 
  {
    {BASE_FIXED_MESSAGE, "handleBaseFixed", 
     (void *) handleBaseFixed, TCX_RECV_ALL, NULL},
    {BASE_VARIABLE_MESSAGE, "handleBaseVariable",
     (void *) handleBaseVariable,  TCX_RECV_ALL,  NULL}
  };


  if (bee_tcx_initialized)
    return;

  if ( robotName != NULL)
    tcxSetModuleNameExtension( robotName);
  
  strcpy(tcxServer, getenv("TCXHOST"));

  fprintf(stderr, "Initializing TCX (%d): name=%s server=%s.\n",
	  bee_tcx_initialized, "baseTCXServer", tcxServer);
  tcxInitialize("baseTCXServer", tcxServer); 

  tcxRegisterMessages(messageArray, sizeof(messageArray)
		      / sizeof(TCX_REG_MSG_TYPE));
  tcxRegisterHandlers(handlerArray,
		      sizeof(handlerArray) / sizeof(TCX_REG_HND_TYPE));
  tcxRegisterCloseHnd(baseClose);

  bee_tcx_initialized = 1;

    fprintf(stderr, "\n\t### ATTENTION: Using default calibration. ###\n\n");
}

/* ********************************************************************** *\
 * ********************************************************************** *
\* ********************************************************************** */





static int flag_new_trans_motion_command;
static int flag_new_rot_motion_command;
static float target_trans;
static float target_rot;


void
bee_check_trans_motion_terminator(float pos_x, float pos_y, float pos_o)
{
  static float initial_x, initial_y, initial_o;
  static int relative_motion = 0;
  float dist;

  if (flag_new_trans_motion_command){
    initial_x = pos_x;
    initial_y = pos_y;
    initial_o = pos_o;
    relative_motion = 1;
    flag_new_trans_motion_command = 0;
    /* fprintf(stderr, "XXX: Relative motion, initial position = %g %g.\n", 
       pos_x, pos_y);  */
  }

  else if (relative_motion){
    dist = 10.0 * sqrt(((initial_x - pos_x) * (initial_x - pos_x)) +
		       ((initial_y - pos_y) * (initial_y - pos_y)));
    /* fprintf(stderr,  
	     "XXX: Relative motion, position = %g %g, dist = %g (%g).\n", 
	     pos_x, pos_y, dist, target_trans);  */
    /* fprintf(stderr, "XXX Velocity: %d %d   %d %d\n", 
       global_set_tvel, global_tvel_sign, global_set_rvel, global_rvel_sign);*/
    
    if (dist >= target_trans){
      global_tvel_sign    = 0;
      global_rvel_sign    = 0;
      global_set_tvel_new = 1;
      global_set_rvel_new = 1;
      /* fprintf(stderr, "XXX: Relative motion terminated.\n"); */
      relative_motion = 0;      
    }
  }
}

void
bee_set_trans_motion_terminator(float dist)
{
  flag_new_trans_motion_command = 1;
  target_trans = dist;
  /* fprintf(stderr, "XXX: Relative motion, distance = %g.\n", dist); */
}





void
bee_check_rot_motion_terminator(float pos_x, float pos_y, float pos_o)
{
  static int relative_motion = 0;
  static float prev_o, total_o;
  float delta_o;

  if (flag_new_rot_motion_command){
    prev_o  = pos_o;
    total_o = 0.0;
    relative_motion = 1;
    flag_new_rot_motion_command = 0;
    /* fprintf(stderr, "XXX: Relative motion, initial orientation = %g.\n", 
       pos_o); */
  }

  else if (relative_motion){
    delta_o = prev_o - pos_o;
    for (; delta_o >   180.0; ) delta_o -= 360.0;
    for (; delta_o <= -180.0; ) delta_o += 360.0;
    total_o += fabs(delta_o);
    prev_o   = pos_o;
    
    /* fprintf(stderr,  
       "XXX: Relative motion, orientation = %g delta = %g total = %g (%g).\n", 
       pos_o, delta_o, total_o, target_rot);  
       fprintf(stderr, "XXX Velocity: %d %d   %d %d\n", 
       global_set_tvel, global_tvel_sign, global_set_rvel, global_rvel_sign);*/
    
    if (total_o >= target_rot){
      global_tvel_sign    = 0;
      global_rvel_sign    = 0;
      global_set_tvel_new = 1;
      global_set_rvel_new = 1;
      /* fprintf(stderr, "XXX: Relative motion terminated.\n"); */
      relative_motion = 0;      
    }
  }
}

void
bee_set_rot_motion_terminator(float angle)
{
  flag_new_rot_motion_command = 1;
  target_rot = fabs(angle);

  if (target_rot > 30.0)	/* hack: to compensate for deceleration */
    target_rot -= 8.0;
  else
    target_rot *= 0.733;

  /* fprintf(stderr, "XXX: Relative motion, angle = %g.\n", angle);  */
}


/* ********************************************************************** *\
 * ********************************************************************** *
\* ********************************************************************** */



void
handleBaseVariable(TCX_REF_PTR message, RAI_FixedMsgType *msg_ptr)
{
  fprintf(stderr, "handleBaseVariable\n");
}


void
handleBaseFixed(TCX_REF_PTR message, RAI_FixedMsgType *msg_ptr)
{
  int param;
  int v = 0;
  
  /* fprintf(stderr, "handleBaseFixed\n"); */

  param = (int) msg_ptr->parameter;

  switch(msg_ptr->operation){
  case (BASE_assumeWatchdog):
    if (v) fprintf(stderr, "TCX: Received a BASE_assumeWatchdog: %d.\n",
	    param);
    break;
  case (BASE_baseKill):
    if (v) fprintf(stderr, "TCX: Received a BASE_baseKill: %d.\n",
	    param);
    global_set_tvel     = 0;
    global_set_rvel     = 0;
    global_tvel_sign    = 0;
    global_rvel_sign    = 0;
    global_set_tvel_new = 1;
    global_set_rvel_new = 1;
    break;
  case (BASE_batteryVoltage):
    if (v) fprintf(stderr, "TCX: Received a BASE_batteryVoltage: %d.\n",
	    param);
    break;
  case (BASE_findRotIndex):
    global_set_rvel     = 0;
    global_rvel_sign    = 0;	/* this is a hack - to avoid that the robot*/
    global_set_rvel_new = 1;	/* turns immediately*/
    if (v) fprintf(stderr, "TCX: Received a BASE_findRotIndex: %d.\n",
	    param);
    break;
  case (BASE_rotateHalt):
    global_rvel_sign    = 0;
    global_set_rvel_new = 1;
    if (v) fprintf(stderr, "TCX: Received a BASE_rotateHalt: %d.\n",
	    param);
    break;
  case (BASE_rotateRelativeNeg):
    if (v) fprintf(stderr, "TCX: Received a BASE_rotateRelativeNeg: %d.\n",
	    param);
    global_tvel_sign    = 0;
    global_rvel_sign    = -1;
    global_set_tvel_new = 1;
    global_set_rvel_new = 1;
    bee_set_rot_motion_terminator(((float) param) * 360.0 / 1024.0);
    break;
  case (BASE_rotateRelativePos):
    if (v) fprintf(stderr, "TCX: Received a BASE_rotateRelativePos: %d.\n",
	    param);
    global_tvel_sign    = 0;
    global_rvel_sign    = 1;
    global_set_tvel_new = 1;
    global_set_rvel_new = 1;
    bee_set_rot_motion_terminator(((float) param) * 360.0 / 1024.0);
    break;
  case (BASE_rotateToPosition):
    if (v) fprintf(stderr, "TCX: Received a BASE_rotateToPosition: %d.\n",
	    param);
    break;
  case (BASE_rotateVelocityNeg):
    if (v) fprintf(stderr, "TCX: Received a BASE_rotateVelocityNeg: %d.\n",
	    param);
    global_rvel_sign    = -1;
    global_set_rvel_new = 1;
    break;
  case (BASE_rotateVelocityPos):
    if (v) fprintf(stderr, "TCX: Received a BASE_rotateVelocityPos: %d.\n",
	    param);
    global_rvel_sign    = 1;
    global_set_rvel_new = 1;
    break;
  case (BASE_rotateWhere):
    if (v) fprintf(stderr, "TCX: Received a BASE_rotateWhere: %d.\n",
	    param);
    break;
  case (BASE_setRotateAcceleration):
    if (v) fprintf(stderr, "TCX: Received a BASE_setRotateAcceleration: %d.\n",
	    param);
    break;
  case (BASE_setRotateVelocity):
    if (v) fprintf(stderr, "TCX: Received a BASE_setRotateVelocity: %d.\n",
	    param);
    global_set_rvel     = param;
    global_set_rvel_new = 1;
    break;
  case (BASE_setTranslateAcceleration):
    if (v) fprintf(stderr, "TCX: Received a BASE_setTranslateAcceleration: %d.\n",
	    param);
    break;
  case (BASE_setTranslateTorque):
    if (v) fprintf(stderr, "TCX: Received a BASE_setTranslateTorque: %d.\n",
	    param);
    break;
  case (BASE_setTranslateVelocity):
    if (v) fprintf(stderr, "TCX: Received a BASE_setTranslateVelocity: %d.\n",
	    param);
    global_set_tvel     = param;
    global_set_tvel_new = 1;
    break;
  case (BASE_statusReportData):
    if (v) fprintf(stderr, "TCX: Received a BASE_statusReportData: %d.\n",
	    param);
    break;
  case (BASE_statusReportPeriod):
    if (v) fprintf(stderr, "TCX: Received a BASE_statusReportPeriod: %d.\n",
	    param);
    break;
  case (BASE_translateHalt):
    if (v) fprintf(stderr, "TCX: Received a BASE_translateHalt: %d.\n",
	    param);
    global_tvel_sign    = 0;
    global_set_tvel_new = 1;
    break;
  case (BASE_translateRelativeNeg):
    if (v) fprintf(stderr, "TCX: Received a BASE_translateRelativeNeg: %d.\n",
	    param);
    global_tvel_sign    = -1;
    global_rvel_sign    = 0;
    global_set_tvel_new = 1;
    global_set_rvel_new = 1;
    bee_set_trans_motion_terminator((float) param);
    break;
  case (BASE_translateRelativePos):
    if (v) fprintf(stderr, "TCX: Received a BASE_translateRelativePos: %d.\n",
	    param);
    global_tvel_sign    = 1;
    global_rvel_sign    = 0;
    global_set_tvel_new = 1;
    global_set_rvel_new = 1;
    bee_set_trans_motion_terminator((float) param);
    break;
  case (BASE_translateVelocityNeg):
    if (v) fprintf(stderr, "TCX: Received a BASE_translateVelocityNeg: %d.\n",
	    param);
    global_tvel_sign    = -1;
    global_set_tvel_new = 1;
    break;
  case (BASE_translateVelocityPos):
    if (v) fprintf(stderr, "TCX: Received a BASE_translateVelocityPos: %d.\n",
	    param);
    global_tvel_sign    = 1;
    global_set_tvel_new = 1;
    break;
  case (BASE_watchdogTimer):
    gettimeofday(&watchdog_timestamp,NULL);
    if (0) fprintf(stderr, "TCX: Received a BASE_watchdogTimer: %d.\n",
		   param); 
    break;
  case (BASE_subscribe):
    if (v) fprintf(stderr, "TCX: Received a BASE_subscribe: %d.\n",
	    param);
    add_auto_update_module(message->module, (int) param);
    gettimeofday(&watchdog_timestamp,NULL);
    break;
  case (911):
    fprintf(stderr, "TCX: Received a BASE_brakeoff -> ???.\n");
    break;
  }
}


/* ********************************************************************** *\
 * ********************************************************************** *
\* ********************************************************************** */

void
baseClose(char *moduleName, TCX_MODULE_PTR module)
{
  fprintf(stderr, "TCX close connection detected: %s\n",
	  moduleName);
  
  if (!strcmp(moduleName, "TCX Server")){ /* TCX shut down */
    fprintf(stderr, "TCX Server died, and so will I.\n");
    shutdown_robot();
    exit(0);
  }



  remove_auto_update_module(module);
}

/* ********************************************************************** *\
 * ********************************************************************** *
\* ********************************************************************** */



/******* auto-reply stuff, data definitions ***************/


#define MAX_N_AUTO_UPDATE_MODULES 100


int n_auto_update_modules = 0; /* number of processes to whom
				* position should be forwarded
				* automatically upon change */



typedef struct{
  TCX_MODULE_PTR module;	/* pointer to TCX module */
  int status_frequency;
  int status_count;
} auto_update_type;


auto_update_type auto_update_modules[MAX_N_AUTO_UPDATE_MODULES];
                                /* collection of TCX-module ptrs */



/************************************************************************
 *
 *   NAME:         add_auto_update_module()
 *                 
 *   FUNCTION:     Adds a module to the list of modules that
 *                 get automatical map updates
 *                 
 *   PARAMETERS:   TCX_MODULE_PTR module       module pointer
 *                 
 *                 
 *   RETURN-VALUE: 1, if successful, 0 if not
 *                 
 ************************************************************************/



int
add_auto_update_module(TCX_MODULE_PTR module, int status_frequency)
{
  int i;

  if (n_auto_update_modules >= MAX_N_AUTO_UPDATE_MODULES){
    fprintf(stderr, "ERROR: auto-module memory overflow. Request rejected.\n");
    return 0;
  }
  else
    for (i = 0; i < n_auto_update_modules; i++)
      if (auto_update_modules[i].module == module){
	fprintf(stderr, 
		"Module %s already known.\n", tcxModuleName(module));
	return 1;
      }
  fprintf(stderr, "Add %s to auto-reply list.\n", tcxModuleName(module));
  auto_update_modules[n_auto_update_modules].module    = module; /* pointer*/
  if (status_frequency < 1)
    auto_update_modules[n_auto_update_modules].status_frequency = 1;
  else
    auto_update_modules[n_auto_update_modules].status_frequency = 
      status_frequency;
  auto_update_modules[n_auto_update_modules].status_count = -1;
  n_auto_update_modules++;
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


int
remove_auto_update_module(TCX_MODULE_PTR module)
{     
  int i, j, found = 0;;
  for (i = 0; i < n_auto_update_modules; i++) /* search for module */
    if (auto_update_modules[i].module == module){ /* if module found */
      fprintf(stderr, "Remove %s from auto-reply list.\n",
	      tcxModuleName(module));
      found++;
      n_auto_update_modules--;	/* remove that entry, one less now */
      for (j = i; j < n_auto_update_modules; j++)
	auto_update_modules[j].module = 
	  auto_update_modules[j+1].module; /* shift back */
    }
  if (!found)
    fprintf(stderr, "(%s: no auto-replies removed)\n",
	    tcxModuleName(module));
  return found;
}
  




/************************************************************************
 *
 *   NAME:         bee_send_status_update
 *                 
 *   FUNCTION:     sends to all modules on the list an automatic
 *                 update.
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/


void 
bee_send_status_update(float pos_x, float pos_y, float pos_o,
		       float tvel, float rvel)
{
  int i;
  statusReportType nstatus;
  RAI_VariableMsgType command;
  unsigned long x, y, o, t, r, tstat, rstat;
  static long count = 0;
  float pos_o_90;
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference; 
  float *test_x;
  static unsigned long count_x = 0;

  /*
  if (count_x++ % 20 == 0){
    static float *first = NULL;
    test_x = (float *) malloc(sizeof(float) * 10000);
    if (!first)
      first = test_x;
    fprintf(stderr, " %p ", test_x - first);
    free(test_x);
  }
  */


  if (bee_busy){
    fprintf(stderr, "\n\n\t??? TCX called twice (b) ???\n\n");
    return;
  }
  bee_busy = 1;

  if (0){
    gettimeofday(&this_time, NULL);
    if (last_time.tv_sec > 0){
      time_difference = 
	((float) (this_time.tv_sec - last_time.tv_sec))
	+ (((float) (this_time.tv_usec - last_time.tv_usec))
	   /  1000000.0);
      fprintf(stderr, " %g ", time_difference);
    }
    last_time.tv_sec = this_time.tv_sec;
    last_time.tv_usec = this_time.tv_usec;
  }

  pos_o_90 = 90.0 - pos_o;
  for (; pos_o_90 >= 360.0; ) pos_o_90 -= 360.0;
  for (; pos_o_90 <    0.0; ) pos_o_90 += 360.0;

  x = (unsigned long) (10.0 * (pos_x + (320.0*1024.0)));
  y = (unsigned long) (10.0 * (pos_y + (320.0*1024.0)));
  o = (unsigned long) (pos_o_90 * 1024.0 / 360.0);
  t = (unsigned long) fabs(tvel * 10.0);
  r = (unsigned long) fabs(rvel * 2.84);
  if (tvel >= 0.0)
    tstat = 0x000;
  else
    tstat = 0x100;
  if (rvel < 0.0)
    rstat = 0x000;
  else
    rstat = 0x100;

  nstatus.Request              = htonl(0);
  nstatus.Clock                = htonl(count);
  nstatus.GeneralStatus        = htonl(0);
  nstatus.Xpos                 = htonl(x);
  nstatus.Ypos                 = htonl(y);
  nstatus.Heading              = htonl(o);
  nstatus.BaseRelativeHeading  = htonl(0);
  nstatus.TranslateError       = htonl(0);
  nstatus.TranslateVelocity    = htonl(t);
  nstatus.TranslateStatus      = htonl(tstat);
  nstatus.RotateError          = htonl(0);
  nstatus.RotateVelocity       = htonl(r);
  nstatus.RotateStatus         = htonl(rstat);

  /*  fprintf(stderr, "pos= %g %g %g\n", pos_x, pos_y, pos_o); */

  /*  fprintf(stderr, "[%g %g %g]\n", (float) ntohl(nstatus.Xpos), */
  /* (float) ntohl(nstatus.Ypos), (float) ntohl(nstatus.Heading)); */

  /* *** STATUS DATA *** */

  command.operation = BASE_statusReport;
  command.bufsize   = sizeof(statusReportType);
  command.buffer    = (unsigned char*) (&nstatus);

  for (i = 0; i < n_auto_update_modules; i++){

    auto_update_modules[i].status_count += 1;

    if (auto_update_modules[i].status_frequency >= 1 &&
	auto_update_modules[i].status_count >=
	auto_update_modules[i].status_frequency){
      auto_update_modules[i].status_count = 0;      
      
      /* fprintf(stderr, "Send status update to %s.\n", */
      /* tcxModuleName(auto_update_modules[i].module)); */
      
      tcxSendMsg(auto_update_modules[i].module,
		 BASE_VARIABLE_MESSAGE, &command);
    }
    /* else */
    /* fprintf(stderr, "Skipping %s: %d (%d).\n", */
    /* tcxModuleName(auto_update_modules[i].module), */
    /* auto_update_modules[i].status_count, */
    /* auto_update_modules[i].status_frequency); */

  }
  
  count += 25;
  
  bee_busy = 0;
}






void 
bee_send_sonar_update(float *values)
{
  RAI_VariableMsgType command;
  int index, sonar_index, value_index, i;
  long msg_values[2048]; 
  extern int num_sonars;

  if (bee_busy){
    fprintf(stderr, "\n\n\t??? TCX called twice (c) ???\n\n");
    return;
  }
  bee_busy = 1;


  /* *** SONAR DATA *** */

  command.operation = SONAR_newValues;
  command.buffer    = (unsigned char*) (&msg_values);

  value_index       = 0;

  for(index = 0; index < num_sonars; index++) {
    msg_values[value_index++] = htonl(index);
    
    if (index < num_sonars)
      msg_values[value_index++] = htonl((unsigned long)
					(values[index] * 10.0)); /* in mm */
    else
      msg_values[value_index++] = htonl(10000); /* max range */
  }
	
  msg_values[value_index++] = htonl(-1);
  command.bufsize = sizeof(long) * value_index;


  for (i = 0; i < n_auto_update_modules; i++){

    /* fprintf(stderr, "Send status update to %s.\n", */
    /* tcxModuleName(auto_update_modules[i].module)); */

    tcxSendMsg(auto_update_modules[i].module,
	       BASE_VARIABLE_MESSAGE, &command);

  }

  bee_busy = 0;
}





